import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

from test_reference import configuration, core, motion


def analyzer():
    path = Path(__file__).with_name('analyze.py')
    assert path.exists(), 'Vicon comparison/report program is missing'
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location('vicon_analyze', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def example_data(folder):
    folder.mkdir(); c, poses, t, q, v, a, omega = motion()
    c['body_orientation_measured'] = True
    with (folder/'poses.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source_ns','receive_ns','px','py','pz','qx','qy','qz','qw','valid'])
        for i in range(len(t)):
            writer.writerow([int(poses['source_ns'][i]),int(poses['receive_ns'][i]),
                             *poses['position'][i],*poses['quaternion'][i],1])
    values = np.column_stack([q.inv().apply(v)[:, :2],q.inv().apply(a)[:, :2],np.full(len(t), omega[2])])
    with (folder/'estimates.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source_ns','receive_ns','stream','reference','quantity','value','frame_id'])
        for i, stamp in enumerate(core().map_time(poses['source_ns'],c['clock'])):
            for ref in ['rear_axle','center_of_mass']:
                y = values[i].copy()
                if ref == 'center_of_mass':
                    d = np.array([.21,-.03,0])
                    y[:2] += np.cross(omega,d)[:2];y[2:4] += np.cross(omega,np.cross(omega,d))[:2]
                y[0] += .1
                for name,value in zip(core().QUANTITIES,y):
                    writer.writerow([int(stamp),int(stamp)+4_000_000,'own_'+ref,ref,name,value,
                                     'base_link' if ref=='rear_axle' else 'center_of_mass'])
    (folder/'metadata.json').write_text(json.dumps({'source':'synthetic analytic test','synthetic':True}))
    return c


def test_full_report_known_bias_and_common_window_support(tmp_path):
    m = analyzer(); folder = tmp_path/'data'; c = example_data(folder)
    out = tmp_path/'report'; result = m.analyze(folder,c,out)
    assert result['complete_required_outputs']
    vx = result['comparisons']['whole/own_rear_axle/rear_axle/vx']
    assert vx['windows']['0.12']['rmse'] == pytest.approx(.1,abs=2e-5)
    assert len({v['n'] for v in vx['windows'].values()}) == 1
    assert vx['receive_minus_source_ms']['p95'] == pytest.approx(4.)
    assert result['reference_qualification']['center_of_mass']['qualified'] is False
    assert (out/'report.zh-CN.md').exists() and (out/'rear_axle.png').exists()
    assert (out/'aligned.csv').exists() and (out/'reference.npz').exists()


def test_waveform_offset_sign_and_static_unidentifiability():
    m = core(); c = configuration(); origin = c['clock']['ros_anchor_ns']
    t = np.arange(1601)*.005; ns = origin+np.rint(t*1e9).astype(np.int64)
    values = np.column_stack([np.sin(2*np.pi*.7*t)]*5)
    segments = [dict(ns=ns,values=values,valid=np.ones(len(t),bool))]
    estimate = np.sin(2*np.pi*.7*(t-.06))
    result = m.waveform_offset(ns,estimate,segments,0,c['comparison'],c['clock']['uncertainty_ns'])
    assert result['identifiable'] and result['best_waveform_offset_ms'] == pytest.approx(60.)
    slow_segments=[dict(ns=ns,values=np.column_stack([np.sin(2*np.pi*.11*t)]*5),valid=np.ones(len(t),bool))]
    biased = m.waveform_offset(ns,np.sin(2*np.pi*.11*(t-.06))+.4,slow_segments,0,c['comparison'],c['clock']['uncertainty_ns'])
    assert biased['best_waveform_offset_ms'] == pytest.approx(60.)
    assert biased['best_rmse'] > .39  # Offset diagnosis must not hide the actual bias error.
    static = [dict(ns=ns,values=np.ones((len(t),5)),valid=np.ones(len(t),bool))]
    assert not m.waveform_offset(ns,np.ones(len(t)),static,0,c['comparison'],1e6)['identifiable']


def test_unknown_clock_is_not_silently_treated_as_synchronized(tmp_path):
    m = analyzer(); folder=tmp_path/'data';c=example_data(folder)
    c['clock']['source_anchor_ns'] = None
    with pytest.raises(ValueError,match='clock'): m.analyze(folder,c,tmp_path/'report')
