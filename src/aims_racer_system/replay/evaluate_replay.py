#!/usr/bin/env python3
"""Evaluate collected replay evidence; full tracking is distinct from safe loss handling."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import numpy as np


def quantiles(values):
    return dict(count=len(values),p50=float(np.percentile(values,50)),
                p95=float(np.percentile(values,95)),max=float(max(values))) if values else dict(count=0)


def evaluate(path):
    summary=json.loads((path/'summary.json').read_text())
    rows=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
    anchors=[r for r in rows if r['kind']=='anchor']
    commits=[r for r in anchors if r['values'].get('anchor_committed')=='true']
    health=[r for r in rows if r['kind']=='health']
    checks=dict(single_odom_tf_owner=summary['tf_authorities'].get('odom/base_link')==1,
                local_state_present=summary['counts'].get('ekf',0)>0 and summary['counts'].get('body_cloud',0)>0)
    details={}
    tf_rows=[json.loads(line) for line in (path/'tf-authorities.jsonl').read_text().splitlines()]
    details['tf_receive_gap_ms']={}
    for parent,child in [('map','odom'),('odom','base_link')]:
        receives=sorted(r['receive_steady_ns'] for r in tf_rows
                        if r.get('frame_id')==parent and r.get('child_frame_id')==child)
        details['tf_receive_gap_ms'][parent+'/'+child]=quantiles([
            (b-a)*1e-6 for a,b in zip(receives,receives[1:])])
    if summary['map_sha256']:
        initial_prefix=([r for r in anchors if r['values'].get('epoch')==commits[0]['values']['epoch']
                         and int(r['values']['event_sequence'])<int(commits[0]['values']['event_sequence'])] if commits else [])
        initial_confirmations=initial_prefix[-2:]
        initialized=bool(commits) and len(initial_confirmations)==2 and all(
            r['values'].get('reason')=='initializing' and r['values'].get('anchor_committed')=='false'
            for r in initial_confirmations) and int(commits[0]['values']['anchor_sequence'])==1
        if initialized:
            stamps=[r['source_ns'] for r in initial_confirmations]+[commits[0]['source_ns']]
            initialized=stamps[0]<stamps[1]<stamps[2]
        checks.update(single_map_tf_owner=summary['tf_authorities'].get('map/odom')==1,
                      trusted_initialization=initialized,
                      committed_sources_fresh=bool(commits) and all(0<=r['ros_now_ns']-r['source_ns']<=500000000 for r in commits),
                      independent_diagnostic_present=summary.get('independent_quality_samples',0)>0)
        if commits:
            checks['first_map_tf_after_confirmation']=all(json.loads(line)['stamp_ns']>=commits[0]['source_ns']
                for line in (path/'tf-authorities.jsonl').read_text().splitlines()
                if json.loads(line).get('frame_id')=='map')
    if summary['injection']:
        injection=summary['injection'];first=injection[0]
        lost=[r for r in health if r['received']>=first['received'] and r['values'].get('ready')=='false']
        checks['injected_outage_revokes_ready']=bool(lost)
        if lost:
            details['outage_to_loss_sec']=lost[0]['received']-first['received']
            details['loss_reason']=lost[0]['values'].get('reason')
        resume=next((r for r in injection if r['kind']=='ndt_resume'),None)
        if resume:
            restored=next((r for r in health if r['received']>=resume['received'] and r['values'].get('ready')=='true'),None)
            recovery=[]
            if restored and lost and restored['values']['epoch']==lost[0]['values']['epoch']:
                recovery=sorted((r for r in anchors if r['values'].get('epoch')==restored['values']['epoch']
                    and int(lost[0]['values']['event_sequence'])<int(r['values']['event_sequence'])<=int(restored['values']['event_sequence'])),
                    key=lambda r:int(r['values']['event_sequence']))
            tail=[]
            for r in reversed(recovery):
                if r['values'].get('anchor_committed')!='true':
                    break
                tail.insert(0,r)
            sequences=[int(r['values']['anchor_sequence']) for r in tail]
            checks['three_commit_recovery']=len(tail)>=3 and sequences==list(range(sequences[0],sequences[-1]+1)) and (
                sequences[-1]==int(restored['values']['anchor_sequence']))
            details['consecutive_commits_before_ready_recovery']=len(tail)
        if first['kind']=='lifecycle_deactivate':
            invalidated=next((r for r in anchors if r['received']>=first['received'] and r['values'].get('reason')=='lifecycle_inactive'),None)
            checks['lifecycle_deactivated']=summary['lifecycle_deactivated']
            checks['deactivate_revokes_native_trust']=invalidated is not None
            checks['no_commits_after_deactivate']=invalidated is not None and not any(r['received']>invalidated['received'] for r in commits)
            # Allow queued deliveries from before the synchronous deactivation service response.
            checks['no_tf_after_deactivate']=not any(json.loads(line).get('frame_id')=='map' and
                json.loads(line)['receive_steady_ns']>int((first['received']+.2)*1e9)
                for line in (path/'tf-authorities.jsonl').read_text().splitlines())
    elif commits:
        first_receive=commits[0]['received']
        # Exclude bag completion and the subsequent receive-watchdog expiry.
        last_source=max(r['ros_now_ns'] for r in rows)-200000000
        during=[r for r in health if r['received']>=first_receive and r['source_ns']<last_source]
        checks['continuous_tracking']=bool(during) and all(r['values'].get('ready')=='true' for r in during)
        details['unready_health_during_playback']=sum(r['values'].get('ready')!='true' for r in during)
    if summary.get('initializer_cli_exit') is not None:
        checks['installed_initialization_cli']=summary['initializer_cli_exit']==0
    trace=list(csv.DictReader((path/'fastlio-trace.csv').open()))
    details['fastlio_core_ms']=quantiles([float(r['core_ms']) for r in trace if r['event']=='core'])
    input_stamps={r['stamp']:int(r['steady_ns']) for r in trace if r['event']=='lidar_received'}
    received={r['id']:input_stamps[r['stamp']] for r in trace if r['event']=='convert' and r['stamp'] in input_stamps}
    details['fastlio_scan_to_output_ms']=quantiles([(int(r['steady_ns'])-received[r['id']])*1e-6
        for r in trace if r['event']=='output' and r['id'] in received])
    details['fastlio_max_pending_scans']=max((int(r['pending']) for r in trace),default=0)
    details['fastlio_dropped_trace_records']=sum(int(r['id']) for r in trace if r['event']=='trace_dropped')
    details['fastlio_trace_events']=dict(Counter(r['event'] for r in trace))
    details['intentionally_partial']=summary.get('intentionally_partial',False)
    if 'expected_raw_counts' in summary and not details['intentionally_partial']:
        checks['full_bag_playback_completed']=summary.get('full_bag_playback') is True
        checks['raw_sensor_delivery_complete']=all(details['fastlio_trace_events'].get(event,0)==summary['expected_raw_counts'].get(topic,0)
            for topic,event in [('/livox/lidar','lidar_received'),('/livox/imu','imu_received')])
    quality={r['values']['quality_stamp_ns']:float(r['values']['inlier_fraction']) for r in health if 'inlier_fraction' in r['values']}
    details['inlier_fraction']=quantiles(list(quality.values()))
    result=dict(checks=checks,all_checks_pass=all(checks.values()),details=details)
    (path/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    result=evaluate(args.output)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['all_checks_pass'] else 1)


if __name__=='__main__':main()
