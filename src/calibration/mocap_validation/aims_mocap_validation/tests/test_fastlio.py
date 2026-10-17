"""Analytical rigid-transform, lever-arm, missing-data and provenance checks."""
import csv
import importlib
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def write_csv(path, rows):
    with path.open('w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def dataset(tmp_path, moving=True, gyro=True):
    folder = tmp_path/'input'; folder.mkdir()
    t = np.arange(0, 4.001, .01); ns = 10**18+np.rint(t*1e9).astype(np.int64)
    yaw = .3*t if moving else np.zeros(len(t))
    body = Rotation.from_euler('z', yaw)
    p = np.column_stack([t if moving else t*0, t*0, t*0])
    mount = Rotation.from_euler('xyz', [.2,-.1,.4]); lever=np.array([.3,.02,.03])
    align = Rotation.from_euler('z', .7); trans=np.array([2.,-3.,.5])
    pi = p+body.apply(lever); sensor=align.inv()*(body*mount)
    pi=align.inv().apply(pi-trans)
    omega = np.tile([0,0,.3 if moving else 0],(len(t),1))
    vi=align.inv().apply(np.tile([1. if moving else 0,0,0],(len(t),1))+body.apply(np.cross(omega,lever)))
    poses=[]; odom=[]; gyros=[]
    for i,n in enumerate(ns):
        poses.append(dict(source_ns=int(n),receive_ns=int(n)+1000000,**dict(zip(('px','py','pz'),p[i])),**dict(zip(('qx','qy','qz','qw'),body.as_quat()[i])),valid=1))
        odom.append(dict(source_ns=int(n),receive_ns=int(n)+2000000,stream='raw',frame_id='lio',child_frame_id='imu',**dict(zip(('px','py','pz'),pi[i])),**dict(zip(('qx','qy','qz','qw'),sensor.as_quat()[i])),**dict(zip(('vx','vy','vz'),vi[i])),wx=0,wy=0,wz=0))
        gyros.append(dict(source_ns=int(n),receive_ns=int(n)+1000000,**dict(zip(('wx','wy','wz'),mount.inv().apply(omega[i])))))
    write_csv(folder/'poses.csv',poses);write_csv(folder/'odometry.csv',odom)
    if gyro: write_csv(folder/'gyro.csv',gyros)
    (folder/'metadata.json').write_text(json.dumps({'synthetic':True}))
    config=dict(clock=dict(source_anchor_ns=10**18,ros_anchor_ns=10**18,scale=1,uncertainty_ns=500000,verified=True),vicon=dict(position_unit='m',tracking_semantics='csv_valid_flag',bridge_timestamp_semantics='camera',prediction_or_retiming_enabled=False),body_orientation_in_marker_xyzw=[0,0,0,1],body_orientation_measured=True,references={k:dict(origin_in_marker_m=[0,0,0],measured=True) for k in ['rear_axle','center_of_mass']},differentiation=dict(primary_window_s=.12,windows_s=[.12],polyorder=3,max_gap_s=.04,max_rotation_span_rad=2.8),comparison=dict(lag_limit_s=.2,lag_step_s=.01,min_dynamic_std=dict(vx=.01,vy=.01,ax=.1,ay=.1,yaw_rate=.01)),fastlio=dict(streams=[dict(name='raw',pose_reference='imu',velocity_frame='world',angular_velocity_valid=False)],world_alignment=dict(rotation_xyzw=align.as_quat().tolist(),translation_m=trans.tolist(),static_window_s=None),rear_to_imu_body_m=lever.tolist(),imu_mount_body_xyzw=mount.as_quat().tolist(),max_gap_s=.25))
    return folder,config


def module():
    assert importlib.util.find_spec('aims_mocap_validation.fastlio') is not None, 'FastLIO analysis is missing'
    return importlib.import_module('aims_mocap_validation.fastlio')


def test_raw_pose_and_velocity_mount_lever_and_fixed_world_transform(tmp_path):
    folder,cfg=dataset(tmp_path)
    result=module().analyze(folder,cfg,tmp_path/'report')
    stream=result['streams']['raw']
    assert stream['position_error_m']['rmse'] < 1e-10
    assert stream['orientation_error_rad']['rmse'] < 1e-10
    assert stream['velocity']['vx']['rmse'] < 1e-5
    assert stream['velocity']['vy']['rmse'] < 1e-5
    assert result['source_time_errors_without_curve_shift'] is True
    assert result['physical_acceptance'] is False
    assert (tmp_path/'report'/'report.zh-CN.md').exists()
    assert (tmp_path/'report'/'fastlio.png').exists()


def test_missing_gyro_never_assumes_zero_lever_velocity(tmp_path):
    folder,cfg=dataset(tmp_path,gyro=False)
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert result['streams']['raw']['position_error_m']['n'] > 0
    assert result['streams']['raw']['velocity']['available'] is False
    assert 'gyro' in result['streams']['raw']['velocity']['reason']


def test_no_trajectory_fit_and_only_static_window_allowed(tmp_path):
    folder,cfg=dataset(tmp_path)
    cfg['fastlio']['world_alignment']=dict(rotation_xyzw=None,translation_m=None,static_window_s=None)
    with pytest.raises(ValueError,match='static|alignment'):
        module().analyze(folder,cfg,tmp_path/'report')
    cfg['fastlio']['world_alignment']['static_window_s']=[0.,1.]
    with pytest.raises(ValueError,match='static|motion'):
        module().analyze(folder,cfg,tmp_path/'report')


def test_static_alignment_once_and_hash_verification(tmp_path):
    folder,cfg=dataset(tmp_path,moving=False)
    cfg['fastlio']['world_alignment']=dict(rotation_xyzw=None,translation_m=None,static_window_s=[0.,1.])
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert result['streams']['raw']['position_error_m']['rmse'] < 1e-10
    assert result['alignment']['method']=='designated_static_window'
    (folder/'metadata.json').write_text(json.dumps({'data_sha256':{'poses.csv':'bad'}}))
    with pytest.raises(ValueError,match='hash'):
        module().analyze(folder,cfg,tmp_path/'report2')


def test_invalid_tracking_gap_is_not_interpolated(tmp_path):
    folder,cfg=dataset(tmp_path)
    rows=list(csv.DictReader((folder/'poses.csv').open()))
    for row in rows[100:201]: row['valid']='0'
    write_csv(folder/'poses.csv',rows)
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert result['streams']['raw']['position_error_m']['n']==300


def test_static_transform_is_determined_once_and_reused(tmp_path):
    folder,cfg=dataset(tmp_path,moving=False)
    rows=list(csv.DictReader((folder/'odometry.csv').open()))
    for i,row in enumerate(rows[::3]):
        copy=dict(row);copy['stream']='raw_second';copy['px']=float(copy['px'])+.001*(i%2);rows.append(copy)
    write_csv(folder/'odometry.csv',rows)
    cfg['fastlio']['streams'].append(dict(name='raw_second',pose_reference='imu',velocity_frame='world',angular_velocity_valid=False))
    cfg['fastlio']['world_alignment']=dict(rotation_xyzw=None,translation_m=None,static_window_s=[0,1])
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert result['alignment']['determined_from_stream']=='raw'
    assert 0<result['streams']['raw_second']['position_error_m']['rmse']<.002


def test_known_waveform_offset_preserves_primary_unshifted_error_and_drift(tmp_path):
    folder,cfg=dataset(tmp_path,moving=False)
    poses=list(csv.DictReader((folder/'poses.csv').open()));rows=[]
    for row in poses:
        t=(int(row['source_ns'])-10**18)*1e-9
        x=(1-np.cos(1.7*t))/1.7+.3*(1-np.cos(4.3*t))/4.3
        row['px']=x
        delayed=t-.08
        rows.append(dict(source_ns=row['source_ns'],receive_ns=row['receive_ns'],stream='rear',frame_id='lio',child_frame_id='base_link',px=x+.01*t,py=0,pz=0,qx=0,qy=0,qz=0,qw=1,vx=np.sin(1.7*delayed)+.3*np.sin(4.3*delayed),vy=0,vz=0,wx=0,wy=0,wz=.1))
    write_csv(folder/'poses.csv',poses);write_csv(folder/'odometry.csv',rows)
    cfg['fastlio']['streams']=[dict(name='rear',pose_reference='rear_axle',velocity_frame='body',angular_velocity_valid=True)]
    cfg['fastlio']['world_alignment']=dict(rotation_xyzw=[0,0,0,1],translation_m=[0,0,0])
    result=module().analyze(folder,cfg,tmp_path/'report')
    entry=result['streams']['rear']; vx=entry['velocity']['vx']
    assert vx['rmse']>.05
    assert vx['waveform_offset']['identifiable']
    assert vx['waveform_offset']['best_waveform_offset_ms']==pytest.approx(80,abs=10)
    assert entry['drift_growth']['linear_growth_m_per_s']==pytest.approx(.01,abs=1e-8)
    assert entry['yaw_rate']['rmse']==pytest.approx(.1)


def test_gyro_gap_removes_only_velocity_support(tmp_path):
    folder,cfg=dataset(tmp_path)
    rows=list(csv.DictReader((folder/'gyro.csv').open()))
    write_csv(folder/'gyro.csv',rows[:100]+rows[201:])
    result=module().analyze(folder,cfg,tmp_path/'report')
    stream=result['streams']['raw']
    assert stream['position_error_m']['n']==401
    assert 0<stream['velocity']['vx']['n']<300


def test_nominal_imu_geometry_is_not_qualified_as_measured(tmp_path):
    folder,cfg=dataset(tmp_path)
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert 'imu_geometry_not_measured' in result['reference_qualification']['warnings']
    cfg['fastlio']['geometry_measured']=True
    result=module().analyze(folder,cfg,tmp_path/'report2')
    assert 'imu_geometry_not_measured' not in result['reference_qualification']['warnings']


def test_bad_raw_orientation_cannot_support_world_velocity_transform(tmp_path):
    folder,cfg=dataset(tmp_path)
    rows=list(csv.DictReader((folder/'odometry.csv').open()))
    for row in rows[100:201]:row['qx']='nan'
    write_csv(folder/'odometry.csv',rows)
    result=module().analyze(folder,cfg,tmp_path/'report')
    stream=result['streams']['raw']
    assert stream['position_error_m']['n']==300
    assert stream['velocity']['vx']['n']<300


def test_invalid_quaternion_cannot_support_angular_velocity(tmp_path):
    folder,cfg=dataset(tmp_path)
    cfg['fastlio']['streams'][0]['angular_velocity_valid']=True
    cfg['fastlio']['streams'][0]['angular_velocity_frame']='world'
    rows=list(csv.DictReader((folder/'odometry.csv').open()))
    for row in rows[100:201]:row['qx']='nan'
    write_csv(folder/'odometry.csv',rows)
    stream=module().analyze(folder,cfg,tmp_path/'report')['streams']['raw']
    assert stream['position_error_m']['n']==300
    assert stream['yaw_rate']['n']==stream['velocity']['vx']['n']
    assert stream['yaw_rate']['n']<300


@pytest.mark.parametrize('fault',[
    'aliased_translation','aliased_rotation','lio_gap','vicon_gap','vicon_occlusion',
    'lio_incomplete_window','vicon_incomplete_window',
])
def test_static_alignment_requires_full_unoccluded_raw_window(tmp_path,fault):
    folder,cfg=dataset(tmp_path,moving=False)
    cfg['fastlio']['world_alignment']=dict(rotation_xyzw=None,translation_m=None,static_window_s=[0.,1.])
    # LIO sampling at .1s sees zeros of 10Hz sinusoidal motion, whereas raw
    # Vicon sampling at .01s resolves that motion. It must never qualify static.
    odom=list(csv.DictReader((folder/'odometry.csv').open()))[::10]
    poses=list(csv.DictReader((folder/'poses.csv').open()))
    if fault.startswith('aliased'):
        for row in poses[:101]:
            t=(int(row['source_ns'])-10**18)*1e-9
            motion=.2*np.sin(20*np.pi*t)
            if fault=='aliased_translation':row['px']=motion
            else:row.update(dict(zip(('qx','qy','qz','qw'),Rotation.from_euler('z',motion).as_quat())))
    elif fault=='lio_gap':odom=odom[:11:5]+odom[11:]
    elif fault=='vicon_gap':poses=poses[:11]+poses[19:]
    elif fault=='vicon_occlusion':
        for row in poses[12:19]:row['valid']='0'
    elif fault=='lio_incomplete_window':odom=odom[1:]
    elif fault=='vicon_incomplete_window':
        # Keep the original mapped origin while ending Vicon coverage before
        # the configured static window's end; later poses support derivatives.
        cfg['fastlio']['world_alignment']['static_window_s']=[3.5,4.1]
    write_csv(folder/'poses.csv',poses);write_csv(folder/'odometry.csv',odom)
    with pytest.raises(ValueError,match='static|motion'):
        module().analyze(folder,cfg,tmp_path/'report')
