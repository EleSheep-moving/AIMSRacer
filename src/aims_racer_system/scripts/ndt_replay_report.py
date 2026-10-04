#!/usr/bin/env python3
"""Build an auditable desktop replay report; comparisons are not ground truth."""
import argparse
import csv
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from ndt_replay_metrics import compose_planar
from ndt_replay_suite import BAGS


def historical_global(bag, initial_seed):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    db=next(bag.glob('*.db3'))
    with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:
        topics={name:(ident,typ) for ident,name,typ in con.execute('select id,name,type from topics')}
        odom_name='/rear_axle/lio_odom'
        if odom_name not in topics:return np.empty((0,4))
        ident,typ=topics[odom_name]
        odometry=[]
        for ts,data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ident,)):
            m=deserialize_message(data,get_message(typ));p=m.pose.pose
            q=p.orientation
            angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            odometry.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.position.x,p.position.y,angle))
        tf_name='/localizer/raw_tf' if '/localizer/raw_tf' in topics else '/tf'
        transforms={}
        if tf_name in topics:
            ident,typ=topics[tf_name]
            for ts,data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ident,)):
                for t in deserialize_message(data,get_message(typ)).transforms:
                    if t.header.frame_id!='map' or t.child_frame_id!='odom':continue
                    q=t.transform.rotation;p=t.transform.translation
                    stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
                    transforms[stamp]=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    odometry=np.array(odometry)
    if not len(odometry):return np.empty((0,4))
    if not transforms:
        # B: same one-time seed, old LIO propagation thereafter; no recorded global constraint.
        target=initial_seed['target_source_sec']
        old=odometry[np.argmin(abs(odometry[:,0]-target))]
        map_yaw=Rotation.from_quat(initial_seed['orientation']).as_euler('xyz')[2]
        angle=map_yaw-old[3]
        x=initial_seed['position'][0]-math.cos(angle)*old[1]+math.sin(angle)*old[2]
        y=initial_seed['position'][1]-math.sin(angle)*old[1]-math.cos(angle)*old[2]
        transforms[odometry[0,0]]=(x,y,angle)
    stamps=sorted(transforms)
    result=[]
    for stamp,x,y,angle in odometry:
        index=np.searchsorted(stamps,stamp,side='right')-1
        if index>=0:result.append((stamp,*compose_planar(transforms[stamps[index]],(x,y,angle))))
    return np.array(result)


def plot_case(folder,out,bag_root,case):
    provenance=json.loads((folder/'provenance.json').read_text())
    start=provenance['bag']['start_sec']
    rows=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines() if line]
    ekf=[r for r in rows if r['stream']=='ekf']
    health=[r for r in rows if r['stream']=='health']
    ndt=[r for r in rows if r['stream']=='ndt']
    tf_rows=sorted((r for r in rows if r['stream']=='map_tf'),key=lambda r:r['now'])
    global_pose=[];index=0;current=None
    # Causal latest correction received by observer; no TF interpolation across corrections.
    for row in sorted(ekf,key=lambda r:r['now']):
        while index<len(tf_rows) and tf_rows[index]['now']<=row['now']:
            current=tf_rows[index];index+=1
        if current:
            global_pose.append((row['stamp'],*compose_planar((*current['position'][:2],current['yaw']),(*row['position'][:2],row['yaw']))))
    global_pose=np.array(global_pose)
    old=historical_global(bag_root/BAGS[case[0]],provenance['bag']['seed'])
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    ax=axes[0,0]
    if len(global_pose):ax.plot(global_pose[:,1],global_pose[:,2],label='New wheel/IMU + NDT',lw=1.5)
    if len(old):ax.plot(old[:,1],old[:,2],label='Recorded old LIO (comparison)',lw=.8,alpha=.7)
    ax.set(xlabel='map x [m]',ylabel='map y [m]',title='Map trajectories; old output is not truth');ax.axis('equal');ax.legend(fontsize=8)
    ax=axes[0,1]
    quality=[r for r in health if r['values'].get('inlier_fraction') not in (None,'nan')]
    ax.plot([r['now']-start for r in quality],[float(r['values']['inlier_fraction']) for r in quality],label='Independent 0.25m inlier fraction')
    ax.set(xlabel='bag time [s]',ylabel='fraction',ylim=(0,1.02),title='Diagnostic consistency');ax.legend(fontsize=8)
    ax=axes[1,0]
    # Use the evaluator's causal receipt-time grid, rather than a diagnostic's
    # publish-time age sampled later by the observer.
    summary=json.loads((folder/'summary.json').read_text())
    begin=summary['initialization_time']+2.
    end=max(r['now'] for r in rows)
    grid=begin+np.arange(max(0,int((end-begin)/.05)))*.05
    anchors=sorted((r for r in rows if r['stream']=='anchor' and r.get('accepted')),key=lambda r:r['now'])
    ages=[];index=0;last=None
    for now in grid:
        while index<len(anchors) and anchors[index]['now']<=now:
            stamp=anchors[index]['stamp']
            if stamp<=now and (last is None or stamp>last):last=stamp
            index+=1
        ages.append(now-last if last is not None else now-begin+2.)
    ax.plot(grid-start,ages,label='Causally available trusted scan age')
    ax.axhline(.3,color='orange',ls='--',label='0.3s freshness gate');ax.axhline(2,color='red',ls=':',label='2s lost')
    ax.set(xlabel='bag time [s]',ylabel='age [s]',title='Timer TF cannot refresh this metric');ax.legend(fontsize=8)
    ax=axes[1,1]
    ax.plot([r['now']-start for r in ndt],[r.get('processing_ms',0.) for r in ndt],lw=.7)
    ax.axhline(100,color='orange',ls='--')
    ax.set(xlabel='bag time [s]',ylabel='registration [ms]',title='Native NDT wall time; 10Hz scan period=100ms')
    fig.suptitle(f'{case}: 1x replay, no independent ground truth')
    fig.savefig(out/(case+'.png'),dpi=150);plt.close(fig)
    # Source-time nearest-old comparison, always labelled disagreement rather than accuracy.
    differences=[]
    if len(old):
        for stamp,x,y,angle in global_pose[::20]:
            i=np.searchsorted(old[:,0],stamp)
            if i>=len(old):continue
            if abs(old[i,0]-stamp)>.15:continue
            differences.append((stamp-start,math.hypot(x-old[i,1],y-old[i,2]),math.atan2(math.sin(angle-old[i,3]),math.cos(angle-old[i,3]))))
    with (out/(case+'-old-output-disagreement.csv')).open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['bag_sec','translation_disagreement_m','yaw_disagreement_rad']);writer.writerows(differences)
    return dict(samples=len(differences),translation_p95_m=float(np.percentile(np.array(differences)[:,1],95)) if differences else None,
                limitation='Recorded LIO/global ICP output comparison; not independent truth. B uses one-time map seed and old LIO propagation only.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--bags',type=Path,default=Path('/bags'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((args.suite/'suite.json').read_text())
    if not manifest.get('finished_wall_sec'):raise RuntimeError('Suite is incomplete; preserve partial evidence instead of finalizing acceptance.')
    if set(manifest.get('planned_cases',[]))!=set(manifest['cases']):
        raise RuntimeError('Completed cases must match the planned acceptance scope.')
    if any(case.startswith('C-') for case in manifest['cases']):
        raise RuntimeError('C is outside the user-approved acceptance scope.')
    rows=[]
    for case,execution in manifest['cases'].items():
        if execution['returncode']:raise RuntimeError(f'Harness failed: {case}')
        summary=json.loads((args.suite/case/'summary.json').read_text())
        rows.append(dict(case=case,timing_pass=summary['timing_pass'],ekf_hz=summary['ekf']['rate_hz'],
            ekf_wall_interval_p99_ms=summary['ekf']['wall_interval_s']['p99']*1000,
            input_age_p99_ms=summary['ekf']['input_age_s']['p99']*1000 if summary['ekf']['input_age_s']['p99'] is not None else None,
            deskew_fraction=summary['deskew_metrics']['output_fraction'],
            deskew_pending_max=summary['deskew_metrics']['pending']['max'],
            deskew_source_age_head_p95_ms=(summary['deskew_metrics']['source_age_head_p95'] or 0)*1000,
            deskew_source_age_tail_p95_ms=(summary['deskew_metrics']['source_age_tail_p95'] or 0)*1000,
            accepted_age_p95_ms=(summary['ndt']['accepted_age_s']['p95'] or 0)*1000 if 'ndt' in case or 'pause' in case or 'drop' in case else None,
            longest_lost_sec=summary['ndt']['longest_lost_interval_s'] if 'map_freshness' in summary['gates'] else None,
            registration_p95_ms=summary['ndt']['registration_ms']['p95'],
            inlier_median=summary['independent_inlier_fraction']['p50'],
            inlier_first_half_mean=summary['quality_first_half_mean'],inlier_second_half_mean=summary['quality_second_half_mean'],
            accepted_recovery_s=summary.get('fault',{}).get('accepted_recovery_s'),gates=json.dumps(summary['gates'],sort_keys=True)))
    with (args.output/'cases.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    comparison={}
    for bag in ('A','B'):
        case=bag+'-ndt-r1'
        if case not in manifest['cases']:continue
        comparison[case]=plot_case(args.suite/case,args.output,args.bags,case)
    scope='Desktop Humble 1x replay, pinned NDT, supplied A/B regimes. C testing cancelled by user; partial C logs are not acceptance. No vehicle or Orin validation.'
    scope+=' D synthetic stationary wheel. No independent GT or controller validation.'
    result=dict(excluded_cases=manifest.get('excluded_cases',{}),cases=rows,old_output_comparisons=comparison,all_timing_gates_pass=all(r['timing_pass'] for r in rows),
                normal_map_gates_pass=all(r['timing_pass'] for r in rows if '-ndt-r' in r['case']),
                scope=scope)
    (args.output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    lines=['# Wheel/IMU + NDT replay results','',result['scope'],'',
           '| Case | EKF Hz | EKF P99 ms | Anchor P95 ms | Inlier median | Gate |','| --- | ---: | ---: | ---: | ---: | --- |']
    for row in rows:
        def fmt(value):return '-' if value is None else f'{value:.3f}'
        lines.append(f"| {row['case']} | {fmt(row['ekf_hz'])} | {fmt(row['ekf_wall_interval_p99_ms'])} | {fmt(row['accepted_age_p95_ms'])} | {fmt(row['inlier_median'])} | {'PASS' if row['timing_pass'] else 'FAIL'} |")
    lines+=['','Each case has raw events, stack/player logs, source/parameter hashes and initialization provenance.',
            'Controlled fault cases may fail normal freshness while passing their separate recovery gate; inspect cases.csv.',
            'Failing timing gates keep this prototype opt-in. Good inlier fractions do not establish absolute accuracy.']
    (args.output/'README.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(cases=len(rows),all_timing_gates_pass=result['all_timing_gates_pass'],output=str(args.output))))


if __name__=='__main__':main()
