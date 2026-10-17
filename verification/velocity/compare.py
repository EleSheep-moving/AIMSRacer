#!/usr/bin/env python3
"""Strict trace comparison: all discrete states exact; continuous tolerance explicit."""
import argparse
import json
from pathlib import Path
import numpy as np

def compare(a,b):
    x=np.genfromtxt(a,delimiter=',',names=True,dtype=None,encoding='utf8');y=np.genfromtxt(b,delimiter=',',names=True,dtype=None,encoding='utf8')
    assert len(x)==len(y),(len(x),len(y))
    discrete=['input_source_ns','accepted','stamp_ns','valid','status','anchor_stamp_ns','rejected_lio','history_size','lio_reinitializations']
    for key in discrete:assert np.array_equal(x[key],y[key]),(key,np.flatnonzero(x[key]!=y[key])[:10])
    continuous=['vx','variance','px','py','pz','vwx','vwy','vwz','bix','biy','biz']
    errors={key:float(np.max(abs(x[key]-y[key]))) for key in continuous}
    assert max(errors.values())<=1e-8,errors
    return dict(rows=len(x),max_absolute_difference=errors,discrete_exact=True,tolerance=1e-8)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('first',type=Path);p.add_argument('second',type=Path);a=p.parse_args();print(json.dumps(compare(a.first,a.second),indent=2))
