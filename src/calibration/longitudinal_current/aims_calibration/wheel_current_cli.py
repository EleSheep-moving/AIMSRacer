"""Offline calibration commands; query mode is standard-library only."""
import argparse
import json
import sys
from pathlib import Path
from .wheel_current_model import WheelCurrentModel


def query_main(args=None):
    parser=argparse.ArgumentParser(description='Query an offline mean-wheel-acceleration calibration')
    parser.add_argument('--model',required=True,type=Path)
    parser.add_argument('--speed',required=True,type=float,help='Wheel speed in m/s')
    direction=parser.add_mutually_exclusive_group(required=True)
    direction.add_argument('--iq',type=float,help='Measured q-axis current in A')
    direction.add_argument('--acceleration',type=float,help='Desired mean wheel acceleration in m/s^2')
    options=parser.parse_args(args)
    try:
        model=WheelCurrentModel.load(options.model)
    except (OSError,ValueError)as exc:
        print(json.dumps(dict(valid=False,value=None,reason='invalid_model',error=str(exc))))
        return 2
    result=(model.predict_acceleration(options.iq,options.speed)if options.iq is not None
            else model.current_for_acceleration(options.acceleration,options.speed))
    payload=result.to_dict();payload['value_unit']='m/s^2'if options.iq is not None else'A'
    print(json.dumps(payload,allow_nan=False));return 0 if result.valid else 2


def fit_main(args=None):
    parser=argparse.ArgumentParser(description='Fit offline wheel-only current calibration; never publishes vehicle commands')
    parser.add_argument('--source-root',required=True,type=Path,help='Retained stage-bc-review directory')
    parser.add_argument('--output',required=True,type=Path,help='New or empty derived-output directory')
    options=parser.parse_args(args)
    try:
        from .wheel_current_fit import run_fit
        result=run_fit(options.source_root,options.output)
        print(json.dumps(result,indent=2,allow_nan=False));return 0
    except (OSError,ValueError,KeyError)as exc:
        print(json.dumps(dict(valid=False,error=str(exc)),allow_nan=False));return 2


def main(args=None):
    args=list(sys.argv[1:]if args is None else args)
    parser=argparse.ArgumentParser(description='Wheel-current offline model tools')
    parser.add_argument('command',choices=('fit','query'))
    options=parser.parse_args(args[:1])
    return fit_main(args[1:])if options.command=='fit'else query_main(args[1:])


if __name__=='__main__':raise SystemExit(main())
