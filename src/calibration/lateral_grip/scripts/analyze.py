#!/usr/bin/env python3
"""Offline stage statistics; paired AX/AY are from one acceleration message."""
import argparse
import csv
import json
import math
from bisect import bisect_left
from collections import defaultdict
from pathlib import Path

def summarize(directory):
    directory=Path(directory)
    groups=defaultdict(list)
    channels=defaultdict(list)
    with (directory/'evidence.csv').open() as f:
        for r in csv.DictReader(f):
            if r['state']=='RUN' and r['quality']=='ok':
                channels[(r['channel'],r['direction'],r['stage'])].append(r)
            if r['state']=='RUN' and r.get('phase','HOLD')=='HOLD' and r['channel'].endswith('/acceleration') and r['quality']=='ok':
                ax,ay=float(r['x']),float(r['y'])
                if math.isfinite(ax) and math.isfinite(ay):
                    groups[(r['channel'],int(r['direction']),int(r['stage']))].append((ax,ay))
    session=json.loads((directory/"summary.json").read_text()) if (directory/"summary.json").exists() else {}
    completed={tuple(s) for s in session.get("completed_stages",[])}
    stages=[]
    for (channel,direction,stage),pairs in sorted(groups.items()):
        n=len(pairs)
        stages.append(dict(channel=channel,direction=direction,stage=stage,samples=n,completed=(direction,stage) in completed if session else None,
            ax_mean=sum(p[0] for p in pairs)/n,ay_mean=sum(p[1] for p in pairs)/n,
            ax_min=min(p[0] for p in pairs),ax_max=max(p[0] for p in pairs),
            ay_min=min(p[1] for p in pairs),ay_max=max(p[1] for p in pairs),
            max_resultant_mps2=max(math.hypot(*p) for p in pairs)))
    diagnostic_pairs=[]
    # Explicit nearest-source-time diagnostic, bounded to 25 ms, preserving all
    # original stamps. No latest-value snapshot and no physical-delay claim.
    for rows in channels.values():rows.sort(key=lambda r:float(r['source_time']))
    def nearest(rows, source):
        stamps=[float(r['source_time']) for r in rows]
        i=bisect_left(stamps,source)
        candidates=rows[max(0,i-1):i+1]
        if not candidates:return None
        match=min(candidates,key=lambda r:abs(float(r['source_time'])-source))
        return match if abs(float(match['source_time'])-source)<=.025 else None
    for (channel,direction,stage),rows in sorted(channels.items()):
        if not channel.endswith('/velocity'):continue
        frame=channel.rsplit('/',1)[0]
        yaw_rows=channels.get((frame+'/yaw_rate',direction,stage),[])
        accel_rows=channels.get((frame+'/acceleration',direction,stage),[])
        for v in rows:
            t=float(v['source_time']);yaw=nearest(yaw_rows,t);acc=nearest(accel_rows,t)
            if yaw is None:continue
            vx=float(v['x']);r=float(yaw['x'])
            if not math.isfinite(vx) or not math.isfinite(r) or abs(r)<1e-3:continue
            diagnostic_pairs.append(dict(frame=frame,direction=int(direction),stage=int(stage),
                velocity_source_time=t,yaw_rate_source_time=float(yaw['source_time']),
                acceleration_source_time=float(acc['source_time']) if acc else None,
                source_time_difference_s=float(yaw['source_time'])-t,
                radius_vx_over_yaw_rate_m=vx/r,vx_times_yaw_rate_mps2=vx*r,
                measured_ay_mps2=float(acc['y']) if acc else None))
    result={'source_alignment' :'AX/AY from same acceleration message; other channels unaligned',
            'interpretation':'Nominal fixed-steering trials; no independent tire-force truth or mu_safe update',
            'session_terminal_state':session.get('state'),'session_terminal_reason':session.get('reason'),'incomplete_stage':session.get('incomplete_stage'),'stages':stages,'diagnostic_pair_max_source_difference_s':.025,'diagnostic_pairs':diagnostic_pairs}
    (directory/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    fields=['channel','direction','stage','completed','samples','ax_mean','ay_mean','ax_min','ax_max','ay_min','ay_max','max_resultant_mps2']
    with (directory/'stage_stats.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(stages)
    with (directory/'radius_vxr_diagnostics.csv').open('w') as f:
        fields=['frame','direction','stage','velocity_source_time','yaw_rate_source_time','acceleration_source_time','source_time_difference_s','radius_vx_over_yaw_rate_m','vx_times_yaw_rate_mps2','measured_ay_mps2']
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(diagnostic_pairs)
    return result

def plot(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory);groups=defaultdict(list)
    with (directory/'evidence.csv').open() as f:
        for r in csv.DictReader(f):
            if r['state']=='RUN' and r.get('phase','HOLD')=='HOLD' and r['channel'].endswith('/acceleration') and r['quality']=='ok':
                a,b=float(r['x']),float(r['y'])
                if math.isfinite(a) and math.isfinite(b):groups[(r['channel'],r['direction'])].append((a,b))
    fig,ax=plt.subplots(figsize=(7,6))
    for (channel,direction),values in sorted(groups.items()):
        ax.scatter([v[1]/9.80665 for v in values],[v[0]/9.80665 for v in values],s=6,alpha=.5,label=f'{channel} dir {direction}')
    ax.set(xlabel='AY / g (same message)',ylabel='AX / g (same message)',title='Fixed nominal-steering calibration evidence');ax.axis('equal');ax.grid(True)
    if groups:ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(directory/'gg.png',dpi=180);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');p.add_argument('--no-plot',action='store_true');a=p.parse_args()
    summarize(a.directory)
    if not a.no_plot:plot(a.directory)
