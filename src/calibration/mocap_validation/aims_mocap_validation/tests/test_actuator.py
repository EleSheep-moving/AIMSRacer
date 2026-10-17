"""Known causal response, support intervals, gaps and mode attribution."""
import csv
import importlib
import numpy as np
import pytest
from test_fastlio import dataset, write_csv


def module():
    assert importlib.util.find_spec('aims_mocap_validation.actuator') is not None, 'Actuator analysis is missing'
    return importlib.import_module('aims_mocap_validation.actuator')


def actuator_data(tmp_path, channel='speed', spike=False, motion=True):
    folder,cfg=dataset(tmp_path,moving=False)
    t=np.arange(0,4.001,.01);commands=[];telemetry=[]
    fields=dict(speed='target_speed_mps',current='target_current',duty='target_duty',brake='target_brake_current',steering='steering_angle')
    amplitudes=dict(speed=1,current=20,duty=.2,brake=20,steering=.2)
    for x in t:
        n=10**18+int(round(x*1e9));row=dict(source_ns=n,receive_ns=n+999000000,mode=channel if channel!='steering' else 'speed',steering_angle=0,target_speed_mps=0,target_current=0,target_duty=0,target_brake_current=0)
        if x>=1:row[fields[channel]]=amplitudes[channel]
        commands.append(row)
        erpm=2000 if x>=1.12 or (spike and abs(x-1.05)<1e-6) else 0
        telemetry.append(dict(source_ns=n,receive_ns=n+999000000,erpm=erpm,current_motor=0,avg_iq=0))
    write_csv(folder/'commands.csv',commands);write_csv(folder/'telemetry.csv',telemetry)
    poses=list(csv.DictReader((folder/'poses.csv').open()))
    if motion:
        for x,row in zip(t,poses):row['px']=.5*max(x-1.2,0)**2
    write_csv(folder/'poses.csv',poses)
    cfg['actuator']=dict(baseline_s=.4,response_s=1.,min_hold_s=.1,confirmation_s=.02,min_confirm_samples=3,noise_multiplier=4,max_gap_s=.04)
    return folder,cfg


def test_source_latency_sustained_crossing_and_threshold_sensitivity(tmp_path):
    folder,cfg=actuator_data(tmp_path,spike=True)
    result=module().analyze(folder,cfg,tmp_path/'report')
    event=result['events'][0];response=event['responses']['erpm']
    assert event['channel']=='speed' and event['valid']
    assert response['detected']
    assert response['response_interval_ms']==pytest.approx([110.,120.])
    assert response['confirmation_latency_ms']==pytest.approx(140.)
    assert set(response['threshold_sensitivity'])=={'0.5','1.0','2.0'}
    assert result['pure_servo_delay_identified'] is False
    assert (tmp_path/'report'/'actuator.png').exists()


def test_unfiltered_acceleration_support_contains_true_onset_no_future_leak(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    result=module().analyze(folder,cfg,tmp_path/'report')
    response=result['events'][0]['responses']['ax']
    assert response['detected']
    low,high=response['response_interval_ms']
    assert low<=200<=high and high>=200
    assert response['derivative_uses_future_samples'] is False
    rows=list(csv.DictReader((tmp_path/'report'/'response_signals.csv').open()))
    ax=[r for r in rows if r['quantity']=='ax']
    assert all(int(r['support_end_ns'])==int(r['source_ns']) for r in ax)
    assert all(float(r['value'])==pytest.approx(0,abs=1e-8) for r in ax if int(r['support_end_ns'])<10**18+1200000000)


@pytest.mark.parametrize('channel',['current','brake','duty','steering'])
def test_all_supported_command_channels(tmp_path,channel):
    folder,cfg=actuator_data(tmp_path,channel=channel)
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert result['events'][0]['channel']==channel
    assert result['events'][0]['valid']


def test_overlapping_commands_and_mode_transition_are_invalid(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'commands.csv').open()))
    for row in rows[100:]:row['steering_angle']='.2'
    write_csv(folder/'commands.csv',rows)
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert not result['events'][0]['valid']
    assert 'overlapping_commands' in result['events'][0]['invalid_reasons']
    for row in rows[100:]:row['steering_angle']='0';row['mode']='current'
    write_csv(folder/'commands.csv',rows)
    result=module().analyze(folder,cfg,tmp_path/'report2')
    assert 'mode_change' in result['events'][0]['invalid_reasons']


def test_tracking_occlusion_and_insufficient_baseline_flagged(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'poses.csv').open()))
    for row in rows[110:125]:row['valid']='0'
    write_csv(folder/'poses.csv',rows)
    result=module().analyze(folder,cfg,tmp_path/'report')
    assert 'vicon_occlusion_or_gap' in result['events'][0]['invalid_reasons']
    cfg['actuator']['baseline_s']=2.
    result=module().analyze(folder,cfg,tmp_path/'report2')
    assert 'insufficient_command_baseline' in result['events'][0]['invalid_reasons']


def test_manual_windows_and_no_excitation_are_explicit(tmp_path):
    folder,cfg=actuator_data(tmp_path,motion=False)
    cfg['actuator']['event_windows']=[dict(start_s=1.,end_s=1.9,channel='speed')]
    result=module().analyze(folder,cfg,tmp_path/'report')
    event=result['events'][0]
    assert event['selection']=='manual'
    assert event['responses']['ax']['detected'] is False
    assert event['responses']['ax']['reason']=='insufficient_response_excitation'


def test_acceleration_includes_rotating_body_term_and_measured_origin(tmp_path):
    from scipy.spatial.transform import Rotation
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'poses.csv').open()));lever=np.array([.3,.1,0]);omega=.5
    cfg['references']['rear_axle']['origin_in_marker_m']=lever.tolist()
    for row in rows:
        t=(int(row['source_ns'])-10**18)*1e-9;r=Rotation.from_euler('z',omega*t)
        rear=np.array([np.sin(omega*t)/omega,(1-np.cos(omega*t))/omega,0])
        marker=rear-r.apply(lever)
        row.update(dict(zip(('px','py','pz'),marker)));row.update(dict(zip(('qx','qy','qz','qw'),r.as_quat())))
    write_csv(folder/'poses.csv',rows)
    signals,_,_=module()._signals(module().load_poses(folder/'poses.csv'),cfg,.04)
    assert np.median([s[1] for s in signals['ay']])==pytest.approx(omega,abs=1e-4)
    assert np.median([s[1] for s in signals['vx']])==pytest.approx(1,abs=1e-4)


def test_gaps_cannot_confirm_response_across_missing_samples(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'telemetry.csv').open()))
    write_csv(folder/'telemetry.csv',rows[:113]+rows[125:])
    result=module().analyze(folder,cfg,tmp_path/'report')
    response=result['events'][0]['responses']['erpm']
    assert not response['detected']
    assert response['reason']=='signal_gap'


def test_native_erpm_clock_bound_does_not_include_vicon_mapping_uncertainty(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    cfg['clock']['uncertainty_ns']=10000000
    result=module().analyze(folder,cfg,tmp_path/'report')
    response=result['events'][0]['responses']['erpm']
    assert response['clock_expanded_response_interval_ms']==response['response_interval_ms']


def test_nonfinite_telemetry_preserves_previous_finite_crossing_bound(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'telemetry.csv').open()))
    for row in rows[110:112]:row['erpm']='nan'
    write_csv(folder/'telemetry.csv',rows)
    result=module().analyze(folder,cfg,tmp_path/'report')
    response=result['events'][0]['responses']['erpm']
    assert response['response_interval_ms']==pytest.approx([90.,120.])


def test_tampered_extracted_data_and_existing_output_rejected(tmp_path):
    import hashlib
    import json
    folder,cfg=actuator_data(tmp_path)
    digest=hashlib.sha256((folder/'commands.csv').read_bytes()).hexdigest()
    (folder/'metadata.json').write_text(json.dumps({'data_sha256':{'commands.csv':digest}}))
    module().analyze(folder,cfg,tmp_path/'report')
    with pytest.raises(ValueError,match='already exist'):
        module().analyze(folder,cfg,tmp_path/'report')
    with (folder/'commands.csv').open('a') as f:f.write('\n')
    with pytest.raises(ValueError,match='hash'):
        module().analyze(folder,cfg,tmp_path/'tampered')


def test_manual_window_retains_simultaneous_unselected_channel(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'commands.csv').open()))
    for row in rows[100:]:row['steering_angle']='.2'
    write_csv(folder/'commands.csv',rows)
    cfg['actuator']['event_windows']=[dict(start_s=1.,end_s=2.,channel='speed')]
    event=module().analyze(folder,cfg,tmp_path/'report')['events'][0]
    assert not event['valid']
    assert event['channels']==['speed','steering']
    assert 'overlapping_commands' in event['invalid_reasons']


@pytest.mark.parametrize('channel,field,increment,reason',[
    ('steering','steering_angle',.005,'overlapping_commands'),
    ('speed','target_speed_mps',.005,'unstable_command_plateau'),
])
def test_cumulative_subthreshold_ramp_invalidates_step_window(tmp_path,channel,field,increment,reason):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'commands.csv').open()))
    for i,row in enumerate(rows[115:]):row[field]=str(float(row[field])+increment*(i+1))
    write_csv(folder/'commands.csv',rows)
    event=module().analyze(folder,cfg,tmp_path/'report')['events'][0]
    assert event['channel']=='speed'
    assert not event['valid']
    assert reason in event['invalid_reasons']


def test_nonselected_stability_uses_combined_baseline_and_response_window(tmp_path):
    folder,cfg=actuator_data(tmp_path)
    rows=list(csv.DictReader((folder/'commands.csv').open()))
    # Each side varies less than .03; combined drift .04 is coupled motion.
    for i,row in enumerate(rows[60:201]):row['steering_angle']=str(.04*i/140)
    for row in rows[201:]:row['steering_angle']='.04'
    write_csv(folder/'commands.csv',rows)
    event=module().analyze(folder,cfg,tmp_path/'report')['events'][0]
    assert not event['valid']
    assert 'overlapping_commands' in event['invalid_reasons']
