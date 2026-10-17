import hashlib
import json
import numpy as np
import pytest


def fitter():
    import importlib.util
    assert importlib.util.find_spec('aims_calibration.wheel_current_fit') is not None, 'wheel-only fitting module is missing'
    from aims_calibration import wheel_current_fit
    return wheel_current_fit


def pulse(target=80., exit_time=.60):
    t=np.arange(100)*.01
    iq=np.full(len(t),12.);iq[10:61]=target
    wheel=np.column_stack([1.2+3.0*t])
    current=np.zeros((len(t),8));current[:,0]=iq;current[:,4]=16.
    mode=np.zeros((len(t),4));mode[9:60,0]=1;mode[9:60,2]=target
    series={'wheel':(t,wheel),'vesc':(t,current),'final':(t,mode),'request':(t,mode)}
    event=dict(run='synthetic',trial_id=1,plan_index=0,current=target,stage_letter='B' if target>12 else 'C',
               start_target_speed=1.,end_target_speed=1.5,success=True,reason='completed',
               first_current_request_stamp=.08,first_current_ack_stamp=.09,exit_final_stamp=exit_time)
    return event,series


@pytest.mark.parametrize('target',[80.,-80.,4.,0.])
def test_reconstruct_signed_current_and_coast(target):
    event,series=pulse(target)
    row=fitter().reconstruct_trial(event,series)
    assert row['status']=='reconstructed'
    assert row['start_stamp']==pytest.approx(.10)
    assert row['end_stamp']==pytest.approx(.60)
    assert row['acceleration_mps2']==pytest.approx(3.)
    assert row['iq_mean_a']==pytest.approx(target)
    assert row['wheel_samples']==51


def test_reconstruction_does_not_need_imu_lio_or_ekf():
    event,series=pulse()
    expected=fitter().reconstruct_trial(event,series)
    series.update(imu=('invalid',None),lio=('invalid',None),ekf=('invalid',None))
    assert fitter().reconstruct_trial(event,series)==expected


def test_late_extreme_does_not_truncate_valid_plateau():
    event,series=pulse();series['vesc'][1][55,0]=81.
    row=fitter().reconstruct_trial(event,series)
    assert row['start_stamp']==pytest.approx(.10)
    assert row['literal_extreme_stamp']==pytest.approx(.55)


def test_current_noise_does_not_end_plateau_early():
    event,series=pulse();series['vesc'][1][58,0]=20.
    row=fitter().reconstruct_trial(event,series)
    assert row['end_stamp']==pytest.approx(.60)


def test_no_measured_release_is_explicitly_censored():
    event,series=pulse();series['vesc'][1][61:,0]=80.
    assert fitter().reconstruct_trial(event,series)['status']=='right_censored_no_observed_departure'


def test_training_intervals_do_not_cross_reverse_motion():
    event,series=pulse(-80.)
    t=series['wheel'][0]
    # Down through zero, then back up: post-zero samples cannot be recycled.
    series['wheel'][1][:,0]=np.where(t<.35,3.-10*t,10*t-3.5)
    row=fitter().reconstruct_trial(event,series)
    records=fitter().training_records(row,series,dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))
    assert records and all(r['end_stamp']<.35 for r in records)
    assert sum(r['weight'] for r in records)==pytest.approx(1.)


def synthetic_records():
    records=[]
    for speed in (1.,2.,3.,4.,5.):
        for j,iq in enumerate((-60.,-30.,0.,15.,30.,45.,55.,70.,85.,100.)):
            records.append(dict(trial_key=f'{speed}:{j}',stage_letter='B' if iq>10 else 'C',
                iq_mean_a=iq,speed_mps=speed,acceleration_mps2=.1*(iq-10),weight=1.,
                start_target_speed=speed,current=iq,run='synthetic',trial_id=j,
                speed_min_mps=speed,speed_max_mps=speed))
    return records


def test_fitted_model_is_monotone_and_bidirectional():
    module=fitter()
    payload=module.fit_model(synthetic_records(),dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))
    from aims_calibration.wheel_current_model import WheelCurrentModel
    model=WheelCurrentModel.from_dict(payload)
    for speed in [1.,1.5,3.,4.5,5.]:
        output=[]
        for iq in [-50.,-20.,0.,10.,25.,40.,49.99,50.,50.01,65.,90.]:
            r=model.predict_acceleration(iq,speed)
            assert r.valid,(iq,speed,r)
            output.append(r.value)
            inverse=model.current_for_acceleration(r.value,speed)
            assert inverse.valid and model.predict_acceleration(inverse.value,speed).value==pytest.approx(r.value,abs=1e-7)
        assert np.all(np.diff(output)>=-1e-9)
        assert model.predict_acceleration(50.-1e-7,speed).value==pytest.approx(model.predict_acceleration(50.,speed).value,abs=1e-6)
        assert model.current_for_acceleration(0.,speed).value==pytest.approx(10.)


def test_overlapping_records_have_one_weight_per_trial():
    event,series=pulse();row=fitter().reconstruct_trial(event,series)
    rows=fitter().training_records(row,series,dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))
    assert sum(r['weight']for r in rows)==pytest.approx(1.)


def test_declining_high_current_curve_is_projected_monotone():
    module=fitter();records=synthetic_records()
    for r in records:
        if r['iq_mean_a']==85:r['acceleration_mps2']=3.
    p=module.fit_model(records,dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))
    assert np.all(np.diff(p['high']['acceleration_mps2'])>=0)


def test_archive_reader_uses_only_selected_channels_and_checks_digest(tmp_path):
    mod=fitter();event,series=pulse();arrays={}
    columns={}
    for name,(t,x)in series.items():
        arrays[name]=x;arrays[name+'_ts']=np.column_stack([np.rint(t*1e9),np.rint(t*1e9)]).astype(np.int64)
        columns[name]=['vx'] if name=='wheel' else (['iq','id','motor_current','input_current','voltage','duty','erpm','fault']if name=='vesc'else['mode','speed','current','steering'])
    archive=tmp_path/'signals.npz';np.savez_compressed(archive,**arrays)
    meta=dict(npz_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),columns=columns,errors={})
    archive.with_suffix('.json').write_text(json.dumps(meta))
    loaded,quality=mod.read_archive(archive)
    assert set(loaded)==set(series) and len(quality)==4
    meta['npz_sha256']='0'*64;archive.with_suffix('.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError,match='hash'):mod.read_archive(archive)


def test_cross_validation_keeps_each_trial_in_one_fold():
    records=synthetic_records()
    # A second, overlapping window from one trial must stay with that trial.
    extra=dict(records[0],weight=.5);records[0]['weight']=.5;records.append(extra)
    predictions,summary=fitter().cross_validate(records,dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))
    assert set(summary)=={'start_speed','current_level'}
    assert len(predictions)==2*len(records)
    for scheme in summary:
        grouped={}
        for p in predictions:
            if p['scheme']==scheme:grouped.setdefault(p['trial_key'],set()).add(p['heldout_group'])
        assert all(len(groups)==1 for groups in grouped.values())
        assert summary[scheme]['overall']['attempted_records']==len(records)


def test_independent_native_window_verification():
    event,series=pulse();row=fitter().reconstruct_trial(event,series)
    report=fitter().verify_windows([row],{'synthetic':series})
    assert report['independent_ols_windows']==1
    assert report['max_acceleration_difference_mps2']<1e-9


def test_shared_threshold_cannot_be_clamped_from_unobserved_lower_currents():
    records=[r for r in synthetic_records()if r['iq_mean_a']>=70.]
    with pytest.raises(ValueError,match='bracket'):
        fitter().fit_model(records,dict(coefficients_ascending=[10.],speed_domain_mps=[1.,5.]))
