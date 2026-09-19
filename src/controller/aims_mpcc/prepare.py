"""Offline CLI: raw odometry CSV to a verified-geometry rear-axle reference."""
import argparse
from .io import load_config
from .path import prepare_recording


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording');parser.add_argument('output')
    parser.add_argument('--vehicle-config',required=True)
    parser.add_argument('--left-width',type=float,required=True)
    parser.add_argument('--right-width',type=float,required=True)
    parser.add_argument('--start-time',type=float);parser.add_argument('--end-time',type=float)
    args=parser.parse_args()
    path=prepare_recording(args.recording,args.output,load_config(args.vehicle_config),
                           args.left_width,args.right_width,args.start_time,args.end_time)
    print(f'Prepared {path.length:.3f} m closed rear-axle path in {path.frame_id}: {args.output}')
