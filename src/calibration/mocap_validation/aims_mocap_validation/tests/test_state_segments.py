"""Reference-only descriptive regimes; estimator errors never create labels."""
import csv
import importlib

import numpy as np
import pytest

from test_fastlio import dataset, write_csv


def analyzer():
    return importlib.import_module('aims_mocap_validation.analyze')


def classify(segments,options=None):
    module=analyzer()
    assert hasattr(module,'classify_regimes'), 'reference-only regime classifier is missing'
    return module.classify_regimes(segments,options)


def segment(t, speed, yaw,valid=None):
    t=np.asarray(t);speed=np.broadcast_to(speed,t.shape);yaw=np.broadcast_to(yaw,t.shape)
    values=np.column_stack([speed,t*0,t*0,speed*yaw,yaw])
    return dict(ns=10**18+np.rint(t*1e9).astype(np.int64),values=values,
                valid=np.ones(len(t),bool) if valid is None else valid)


def test_analytic_static_straight_turn_and_opposite_sustained_s_lobes():
    segments=[segment(np.arange(0,1,.01),0,0),segment(np.arange(2,3,.01),1,0),
              segment(np.arange(4,5,.01),1,.3)]
    t=np.arange(6,8,.01)
    segments.append(segment(t,1,.3*np.sin(np.pi*(t-6))))
    result=classify(segments)
    assert set(result['support'])=={'static','straight','turn','s_turn'}
    assert all(s['status']=='available' for s in result['support'].values())
    assert all(s['samples']>50 for s in result['support'].values())
    assert result['source']=='primary_rear_axle_vicon_reference_only'
    assert result['thresholds_are_acceptance_limits'] is False
    assert 'estimated' not in str(result)


def test_s_turn_cannot_bridge_tracking_gap_or_short_noise_lobe():
    t=np.arange(0,2,.01);yaw=np.where(t<1,.3,-.3)
    valid=np.ones(len(t),bool);valid[95:105]=False
    result=classify([segment(t,1,yaw,valid)])
    assert result['support']['s_turn']['status']=='missing'
    assert result['support']['turn']['samples']>100
    yaw[:]=.3;yaw[100:105]=-.3
    result=classify([segment(t,1,yaw)])
    assert result['support']['s_turn']['status']=='missing'


def test_missing_insufficient_and_unclassified_support_are_explicit():
    result=classify([segment(np.arange(0,.05,.01),0,0)])
    assert result['support']['static']['status']=='insufficient_support'
    assert result['support']['straight']['status']=='missing'
    result=classify([segment(np.arange(0,1,.01),.1,.07)])
    assert result['unclassified_samples']==100
    assert all(s['status']=='missing' for s in result['support'].values())
    with pytest.raises(ValueError,match='segmentation|regime'):
        classify([segment(np.arange(0,1,.01),0,0)],dict(moving_speed_min_mps=-1))


def state_data(tmp_path):
    from aims_mocap_validation.reference import build_reference,sample_reference
    folder,cfg=dataset(tmp_path,moving=False)
    poses=list(csv.DictReader((folder/'poses.csv').open()))
    for row in poses:
        t=(int(row['source_ns'])-10**18)*1e-9
        row['px']=max(min(t-1,1),0)+max(t-3,0)
    write_csv(folder/'poses.csv',poses)
    module=analyzer();loaded=module.load_poses(folder/'poses.csv')
    refs=build_reference(loaded,cfg,.12)['rear_axle'];ns=loaded['source_ns'];truth=sample_reference(refs,ns)
    estimates=[]
    for i,n in enumerate(ns):
        if not np.isfinite(truth[i,0]):continue
        for name,bias,stride in [('own_rear_velocity',.1,1),('baseline',-.2,2)]:
            if i%stride:continue
            estimates.append(dict(source_ns=int(n),receive_ns=int(n)+1000000,stream=name,reference='rear_axle',quantity='vx',value=truth[i,0]+bias,frame_id='base_link'))
    estimates.sort(key=lambda row:row['source_ns'])
    write_csv(folder/'estimates.csv',estimates)
    cfg['comparison']['estimate_max_gap_s']=.04
    cfg['evaluation_windows']=[dict(name='whole',start_s=None,end_s=None),dict(name='manual',start_s=1.2,end_s=1.8)]
    return folder,cfg


def test_native_common_grid_and_manual_metrics_share_reference_masks(tmp_path):
    folder,cfg=state_data(tmp_path)
    result=analyzer().analyze(folder,cfg,tmp_path/'report')
    assert 'regime_classification' in result, 'state report is missing Vicon regime stratification'
    classification=result['regime_classification']
    assert classification['support']['static']['status']=='available'
    assert classification['support']['s_turn']['status']=='missing'
    assert 'whole/own_rear_velocity/rear_axle/vx' in result['comparisons']
    assert 'manual/own_rear_velocity/rear_axle/vx' in result['comparisons']
    entry=result['comparisons']['regime_static/own_rear_velocity/rear_axle/vx']
    assert entry['source_gaps_above_threshold']==0
    assert entry['windows']['0.12']['rmse']==pytest.approx(.1,abs=1e-10)
    common=result['common_grid_comparisons']['regime_static/rear_axle/vx']
    assert common['streams']['own_rear_velocity']['rmse']==pytest.approx(.1,abs=1e-10)
    assert result['comparisons']['regime_s_turn/own_rear_velocity/rear_axle/vx']['matched_samples']==0
    assert (tmp_path/'report'/'regime_labels.csv').exists()
    report=(tmp_path/'report'/'report.zh-CN.md').read_text()
    assert '工况' in report and '不是验收限值' in report
    # Changing every estimate does not change one reference-derived label.
    rows=list(csv.DictReader((folder/'estimates.csv').open()))
    for row in rows:row['value']='12345'
    write_csv(folder/'estimates.csv',rows)
    changed=analyzer().analyze(folder,cfg,tmp_path/'changed')
    assert changed['regime_classification']==classification


def test_configurable_support_and_regime_waveform_offset_not_computed(tmp_path):
    folder,cfg=state_data(tmp_path)
    cfg['state_segmentation']=dict(max_gap_s=.04,min_samples=1000)
    result=analyzer().analyze(folder,cfg,tmp_path/'report')
    assert result['regime_classification']['settings']['max_gap_s']==.04
    assert result['regime_classification']['support']['static']['status']=='insufficient_support'
    entry=result['comparisons']['regime_static/own_rear_velocity/rear_axle/vx']
    assert not entry['waveform_offset']['identifiable']
    assert entry['waveform_offset']['reason']=='not_computed_for_categorical_regime_mask'
    assert result['common_grid_comparisons']['regime_static/rear_axle/vx']['regime_support_status']=='insufficient_support'
