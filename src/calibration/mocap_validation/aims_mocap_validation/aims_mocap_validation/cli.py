"""Unified passive acquisition and offline validation CLI."""
import argparse
import json
from pathlib import Path
import shutil
from .configuration import load


def main(legacy=False):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    demo=sub.add_parser('demo');demo.add_argument('--output',type=Path,required=True)
    for name in ['inspect','record','extract','analyze']:
        command=sub.add_parser(name);command.add_argument('--config',type=Path,required=True)
        if name!='inspect':command.add_argument('--output',type=Path,required=True)
        if name in ('inspect','record'):command.add_argument('--seconds',type=float)
        if name=='extract':
            command.add_argument('--bag',type=Path,required=True);command.add_argument('--vicon-csv',type=Path)
            command.add_argument('--timing',type=Path,help='native timing.csv; auto-discovered beside bag')
        if name=='analyze':
            command.add_argument('--input',type=Path,required=True);command.add_argument('--task',choices=['state','fastlio','actuator','all'],default='state' if legacy else 'all')
    args=parser.parse_args()
    if args.action=='demo':
        if legacy:
            from .example import create
            result=create(args.output)
        else:
            from .demo import create
            result=create(args.output)
        print(json.dumps(dict(synthetic=True,report=str(args.output/'report/report.zh-CN.md')),indent=2));return
    config=json.loads(args.config.read_text()) if legacy else load(args.config)
    if args.action=='inspect':
        from .session import inspect
        result=inspect(config)
    elif args.action=='record':
        from .session import record
        result=record(config,args.output,args.seconds)
    elif args.action=='extract':
        from .bag import extract
        from .analyze import sha
        result=extract(args.bag,config,args.output,args.vicon_csv)
        timing=args.timing or args.bag.parent/'timing.csv'
        if timing.exists():
            shutil.copy2(timing,args.output/'timing.csv');result['data_sha256']['timing.csv']=sha(args.output/'timing.csv')
            health=timing.with_suffix('.health.json')
            if health.exists():shutil.copy2(health,args.output/'timing.health.json');result['data_sha256']['timing.health.json']=sha(args.output/'timing.health.json')
        capture=args.bag.parent/'capture.json'
        if capture.exists():result['acquisition_manifest']=json.loads(capture.read_text())
        (args.output/'metadata.json').write_text(json.dumps(result,indent=2)+'\n')
    else:
        if legacy and args.task=='state':
            from .analyze import analyze
            result=analyze(args.input,config,args.output)
            result={key:result[key] for key in ['complete_required_outputs','reference_qualification','synthetic']}
        else:
            from .pipeline import analyze
            measured=analyze(args.input,config,args.output,args.task)
            result=dict(all_selected_tasks_computed=measured['all_selected_tasks_computed'],tasks=measured['tasks'],synthetic=measured['synthetic'])
    print(json.dumps(result,indent=2,ensure_ascii=False))
    if args.action=='inspect' and (result['capture_configuration_errors'] or result.get('type_mismatches') or result.get('missing_topics')):raise SystemExit(2)
    if args.action=='analyze' and result.get('all_selected_tasks_computed') is False:raise SystemExit(2)


if __name__=='__main__':main()
