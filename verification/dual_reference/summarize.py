#!/usr/bin/env python3
"""Summarize recorded outputs; distributions are not truth errors."""
import argparse
import csv
import hashlib
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

ORIGIN = 1792002365474876930
WINDOWS = dict(static=(13.065745826,41.425777542), running=(41.926416916,220.431558514))


def stats(values):
    assert values and all(math.isfinite(value) for value in values)
    ordered = sorted(abs(value) for value in values)
    rank = (len(ordered)-1)*.95; index = int(rank)
    p95 = ordered[index]+(rank-index)*(ordered[min(index+1,len(ordered)-1)]-ordered[index])
    return dict(n=len(values),mean=statistics.mean(values),std_ddof0=statistics.pstdev(values),
                abs_p95=p95,abs_max=max(ordered))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    assert not args.output.exists()
    groups=defaultdict(list)
    with (args.replay/'outputs.csv').open() as stream:
        for row in csv.DictReader(stream):
            relative=(int(row['original_source_ns'])-ORIGIN)*1e-9
            names={'velocity':('vx','vy'),'acceleration':('ax','ay'),'yaw_rate':('yaw_rate',)}[row['kind']]
            for label,bounds in WINDOWS.items():
                if bounds[0] <= relative <= bounds[1]:
                    for name,column in zip(names,['x','y']):
                        groups[row['reference'],label,name].append(float(row[column]))
    result=dict(reference_points=['rear_axle','center_of_mass'],windows_source_relative_s=WINDOWS,
        independent_truth=False,interpretation='observed output distribution; not dynamic accuracy',
        input_output_sha256=hashlib.sha256((args.replay/'outputs.csv').read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),statistics={})
    for (reference,label,name),values in groups.items():
        result['statistics'].setdefault(reference,{}).setdefault(label,{})[name]=stats(values)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
