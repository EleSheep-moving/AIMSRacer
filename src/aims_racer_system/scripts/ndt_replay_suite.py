#!/usr/bin/env python3
"""Sequential 1x replay suite. Never runs timing cases concurrently."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

BAGS=dict(A='venue-20261003-161459-bag',B='mpcc-test-20261003-013910',
          D='lio_mp_benchmark_20261003-raw_input_complete')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags',type=Path,default=Path('/bags'))
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--b-seed',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    script=Path(__file__).with_name('ndt_replay.py')
    cases=[('D-local-r1','D',['--local-only','--zero-wheel'])]
    motion_bags=('A','B')
    for repeat in (1,2,3):
        for bag in motion_bags:cases.append((f'{bag}-ndt-r{repeat}',bag,[]))
    for bag in motion_bags:cases.append((f'{bag}-local-r1',bag,['--local-only']))
    for kind in ('lidar_drop','ndt_pause'):
        for duration in (.2,.5,1.):
            cases.append((f'A-{kind}-{duration:g}','A',['--fault',kind,'--fault-duration',str(duration)]))
    state_file=args.output/'suite.json'
    state=json.loads(state_file.read_text()) if state_file.exists() else dict(cases={},started_wall_sec=time.time())
    planned=[name for name,bag,extra in cases]
    if state.get('planned_cases',planned)!=planned or any(name.startswith('C-') for name in state['cases']):
        raise RuntimeError('Acceptance scope changed; choose a fresh output suite to preserve earlier evidence.')
    state['planned_cases']=planned
    state['c_scope']='excluded_by_user'
    excluded=state.setdefault('excluded_cases',{})
    earlier=state.get('deferred_cases',{}).pop('C',{})
    excluded.setdefault('C',{}).update(earlier)
    excluded['C']['reason']='User cancelled C testing; no later C replay is planned. Partial logs are not acceptance.'
    if not state.get('deferred_cases'):state.pop('deferred_cases',None)
    state.pop('finished_wall_sec',None)
    for name,bag,extra in cases:
        folder=args.output/name
        if name in state['cases'] and state['cases'][name].get('returncode')==0:
            print(f'SKIP completed {name}',flush=True)
            continue
        if folder.exists() and any(folder.iterdir()):
            raise RuntimeError(f'Incomplete case evidence preserved at {folder}; choose a fresh output suite for retry.')
        command=[sys.executable,str(script),'--bag',str(args.bags/BAGS[bag]),'--map',str(args.map),
                 '--output',str(folder),*extra]
        if bag=='B':command+=['--initial-pose',str(args.b_seed)]
        state['active_case']=name;state_file.write_text(json.dumps(state,indent=2)+'\n')
        print(f'START {name}',flush=True)
        started=time.time()
        with (args.output/(name+'.log')).open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        state['cases'][name]=dict(returncode=result.returncode,wall_seconds=time.time()-started)
        if (folder/'summary.json').exists():
            summary=json.loads((folder/'summary.json').read_text())
            state['cases'][name].update(gates=summary['gates'],timing_pass=summary['timing_pass'])
        state['active_case']=None;state_file.write_text(json.dumps(state,indent=2)+'\n')
        print(f'END {name} {state["cases"][name]}',flush=True)
        if result.returncode:
            raise RuntimeError(f'Replay harness failed for {name}; inspect its log before continuing.')
    state['finished_wall_sec']=time.time();state_file.write_text(json.dumps(state,indent=2)+'\n')
    print(f'Completed {len(cases)} cases at 1x; failures in timing gates remain recorded.',flush=True)


if __name__=='__main__':main()
