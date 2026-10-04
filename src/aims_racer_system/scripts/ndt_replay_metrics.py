#!/usr/bin/env python3
"""Source-time replay acceptance. Historical odometry is comparison data, never truth."""
import argparse
import json
import math
from pathlib import Path
import statistics


def percentile(values, percentage):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    index = (len(values) - 1) * percentage / 100
    low = int(index)
    return values[low] + (values[min(low+1, len(values)-1)] - values[low]) * (index-low)


def distribution(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    return dict(count=len(values), p50=percentile(values, 50), p95=percentile(values, 95),
                p99=percentile(values, 99), max=max(values) if values else None)


def compose_planar(map_odom, odom_base):
    x, y, yaw = map_odom
    bx, by, byaw = odom_base
    return (x + math.cos(yaw)*bx - math.sin(yaw)*by,
            y + math.sin(yaw)*bx + math.cos(yaw)*by,
            math.atan2(math.sin(yaw+byaw), math.cos(yaw+byaw)))


def input_freshness(now, last_input):
    """A future or missing individual sensor cannot be hidden by aggregation."""
    if not all(key in last_input for key in ("imu","wheel")):
        return None
    ages=[now-last_input[key] for key in ("imu","wheel")]
    if not all(math.isfinite(age) and age>=-.005 for age in ages):
        return None
    return max(ages)


def summarize(rows, ndt_enabled=True, trim_seconds=2., initialization_time=None,
              injection=None):
    ekf_all = sorted((r for r in rows if r['stream'] == 'ekf'), key=lambda r:r['stamp'])
    start = ekf_all[0]['now'] if ekf_all else 0.
    end = max((r['now'] for r in rows), default=start)
    ekf = [r for r in ekf_all if r['now'] >= start+trim_seconds and r['now'] <= end-.05]
    # Both stamp interval and monotonic receipt interval: stamped prediction alone is insufficient.
    intervals = [b['stamp']-a['stamp'] for a,b in zip(ekf, ekf[1:]) if b['stamp']>a['stamp']]
    wall_intervals = [b['mono']-a['mono'] for a,b in zip(ekf,ekf[1:]) if b['mono']>a['mono']]
    rate = (len(ekf)-1)/(ekf[-1]['stamp']-ekf[0]['stamp']) if len(ekf)>1 else 0.
    state_age = [r['now']-r['stamp'] for r in ekf]
    # Missing input produces None, which is explicitly a failed gate, not dropped from the metric.
    input_age = [r.get('input_age') for r in ekf]
    ekf_stats = dict(samples=len(ekf), rate_hz=rate, interval_s=distribution(intervals),
                     wall_interval_s=distribution(wall_intervals), state_age_s=distribution(state_age),
                     input_age_s=distribution(input_age), missing_input_samples=input_age.count(None))
    ekf_gate = (190. <= rate <= 210. and (percentile(intervals,99) or math.inf)<=.015
                and (percentile(wall_intervals,99) or math.inf)<=.015
                and (percentile(state_age,99) if state_age else math.inf)<=.05
                and (min(state_age) if state_age else -math.inf)>=-.005
                and not input_age.count(None)
                and (min(input_age) if input_age and None not in input_age else -math.inf)>=-.005
                and (percentile(input_age,99) if input_age else math.inf)<=.05)
    result = dict(duration_s=end-start, ekf=ekf_stats, gates=dict(ekf_timing=ekf_gate))
    deskew = [r for r in rows if r['stream']=='deskew_status']
    result['deskew'] = deskew[-1].get('values',{}) if deskew else {}
    def numbers(key):
        result_values=[]
        for row in deskew:
            try:
                value=float(row.get('values',{}).get(key,'nan'))
                if math.isfinite(value):result_values.append(value)
            except (TypeError,ValueError):pass
        return result_values
    received=numbers('received');published=numbers('published')
    ratio=published[-1]/received[-1] if received and received[-1]>0 and published else 0.
    queue=numbers('pending');oldest=numbers('oldest_pending_age_ms');active=numbers('active_stage_age_ms')
    age=numbers('source_age_sec')
    result['deskew_metrics']=dict(output_fraction=ratio, pending=distribution(queue),
        queue_age_ms=distribution(numbers('queue_age_ms')), oldest_pending_age_ms=distribution(oldest),
        active_stage_age_ms=distribution(active),source_age_s=distribution([a for a in age if a>=0]),
        source_age_head_p95=percentile(age[:max(1,len(age)//4)],95),
        source_age_tail_p95=percentile(age[-max(1,len(age)//4):],95))
    result['gates']['deskew_coverage']=ratio>=.99 and any(r['stream']=='cloud' for r in rows)
    result['gates']['deskew_bounded']=bool(queue and oldest and active) and max(queue)<=2 and max(oldest)<=110. and max(active)<=110.
    accepted = sorted(set(r['stamp'] for r in rows if r['stream']=='anchor' and r.get('accepted')))
    ndt_rows = [r for r in rows if r['stream']=='ndt']
    anchor_rows = [r for r in rows if r['stream']=='anchor' and r.get('accepted')]
    ndt_begin = (initialization_time if initialization_time is not None else start)+trim_seconds
    # Sample every 50ms independent of status/TF timers. A paused timer cannot hide stale matches.
    grid = [ndt_begin+i*.05 for i in range(max(0,int((end-ndt_begin)/.05)))]
    ages = []
    longest_lost = lost = 0.
    arrivals = sorted(anchor_rows, key=lambda r:r['now'])
    arrival_index = 0
    last_available = None
    for now in grid:
        while arrival_index < len(arrivals) and arrivals[arrival_index]['now'] <= now:
            source = arrivals[arrival_index]['stamp']
            if source <= now and (last_available is None or source > last_available):
                last_available = source
            arrival_index += 1
        age = now-last_available if last_available is not None else now-ndt_begin+2.
        ages.append(age)
        lost = lost+.05 if age>2. else 0.
        longest_lost = max(longest_lost,lost)
    result['ndt'] = dict(scans=len(ndt_rows), accepted=len(accepted), raw_registration_ok=sum(bool(r.get('accepted')) for r in ndt_rows),
                         accepted_age_s=distribution(ages),
                         max_accepted_age_s=max(ages) if ages else None,
                         longest_lost_interval_s=longest_lost,
                         accepted_receipt_age_s=distribution([r['now']-r['stamp'] for r in anchor_rows]),
                         fitness=distribution([r.get('fitness') for r in ndt_rows]),
                         registration_ms=distribution([r.get('processing_ms') for r in ndt_rows]))
    if ndt_enabled:
        result['gates']['map_freshness'] = (bool(accepted) and bool(ages)
                                           and percentile(ages,95)<=.3 and longest_lost<=2.)
    health = [r for r in rows if r['stream']=='health']
    quality = [float(r['values']['inlier_fraction']) for r in health
               if r.get('values',{}).get('inlier_fraction') not in (None,'nan','')]
    result['independent_inlier_fraction'] = distribution(quality)
    # Report time trend; no ground-truth claim or universal inlier threshold.
    middle = len(quality)//2
    result['quality_first_half_mean'] = statistics.mean(quality[:middle]) if middle else None
    result['quality_second_half_mean'] = statistics.mean(quality[middle:]) if quality[middle:] else None
    if injection:
        resumed = injection['end_source_sec']
        after = next((r for r in sorted(anchor_rows,key=lambda r:r['now'])
                      if r['stamp'] >= resumed and r['now'] >= resumed), None)
        recovery = after['now']-resumed if after is not None else None
        result['fault'] = dict(**injection, accepted_recovery_s=recovery,
                              recovered_anchor_source_age_s=after['now']-after['stamp'] if after else None)
        result['gates']['fault_recovery'] = recovery is not None and recovery<=2.
    result['timing_pass'] = all(result['gates'].values())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', type=Path)
    parser.add_argument('--local-only', action='store_true')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.events.read_text().splitlines() if line]
    result = summarize(rows, ndt_enabled=not args.local_only)
    output = json.dumps(result, indent=2, allow_nan=False)+'\n'
    if args.out:
        args.out.write_text(output)
    else:
        print(output, end='')


if __name__ == '__main__':
    main()
