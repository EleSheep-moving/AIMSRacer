from dataclasses import replace
import math
import pytest
from aims_calibration.engine import Config, Engine, Observation, Request, RCProfile, Regions


def config(stage='speed_hold', **kw):
    values = dict(stage=stage, steering_source='rc', speeds=(1.,), stable_seconds=.1,
                  measurement_seconds=.2, currents=(5.,) if stage=='accel_interval' else (-5.,),
                  start_speed=1., end_speed=2. if stage=='accel_interval' else .5)
    values.update(kw)
    return Config(**values)


def observation(t, v=1., current=7., **kw):
    values = dict(wheel_stamp=t, telemetry_stamp=t, velocity=v, current_motor=current,
                  voltage_input=16., erpm=100., rc_stamp=t, rc_valid=True, straight=True,
                  authority=True, authority_stamp=t)
    values.update(kw)
    return Observation(**values)


def step(engine, t, v=1., **kw):
    obs = observation(t,v,**kw)
    # The final driver echoes a recently sent request, never telemetry current.
    if engine.history:
        obs = replace(obs, final_request=replace(engine.history[-1],stamp=t), final_received=t)
    req = engine.tick(t,obs)
    engine.sent(req)
    return req


def prepared(stage='speed_hold', **kw):
    e = Engine(config(stage,**kw)); e.enable(True)
    step(e,1.); step(e,1.01); step(e,1.05); step(e,1.16)
    return e


def test_disabled_requests_fresh_zero_speed_and_current_lists_explicit():
    e=Engine(config()); assert e.tick(1.,observation(1.)).mode==0
    assert e.tick(2.,observation(2.)).speed==0
    for stage in ('accel_interval','decel_current'):
        with pytest.raises(ValueError): Config(stage=stage)
    with pytest.raises(ValueError): config('accel_interval',currents=(-1.,))
    with pytest.raises(ValueError): config('decel_current',currents=(1.,))
    with pytest.raises(ValueError): config('accel_interval',currents=(101.,))
    with pytest.raises(ValueError): config(speeds=(4.1,))


def test_hold_uses_actual_motor_current_unique_source_samples():
    e=prepared(); assert e.state=='MEASURE'
    step(e,1.17,current=6.); step(e,1.18,current=8.)
    n=len(e.samples)
    step(e,1.19,wheel_stamp=1.18,telemetry_stamp=1.18,current=99.)
    assert len(e.samples)==n
    step(e,1.39,current=10.)
    assert e.trials[0]['success'] and e.trials[0]['mean_current_motor']==8.
    assert e.state=='COMPLETE' and step(e,1.4).speed==0


def test_current_response_uses_wheel_source_time_and_signed_mode():
    e=prepared('accel_interval')
    req=step(e,1.17); assert req.mode==1 and req.current==5. and req.speed==0
    step(e,1.27,v=1.3); step(e,1.47,v=2.1)
    trial=e.trials[0]
    assert trial['success'] and trial['duration_s']==pytest.approx(.30)
    assert trial['start_speed']==1. and trial['end_speed']==2.1
    assert trial['mean_acceleration']==pytest.approx(1.1/.30)
    e=prepared('decel_current'); req=step(e,1.17)
    assert req.mode==1 and req.current==-5.
    req=step(e,1.27,v=.49)
    assert req.mode==0 and req.speed==0
    assert e.trials[0]['success'] and e.trials[0]['end_speed']==.49


def test_echo_accepts_recent_request_but_rejects_foreign_and_authority_loss():
    e=prepared(); old=e.history[-1]; req=step(e,1.17)
    obs=replace(observation(1.18),final_request=old,final_received=1.18)
    assert e.tick(1.18,obs).speed==1.
    obs=replace(observation(1.2),final_request=Request(1.2,1,current=30.),final_received=1.2)
    assert e.tick(1.2,obs).current==0 and e.state=='FAULT'
    assert not e.trials[-1]['success']
    step(e,1.21); assert e.state=='FAULT'
    e.enable(True,now=1.21); assert e.state=='PREPARE'
    step(e,1.22); step(e,1.225); step(e,1.23,authority=False)
    assert e.state=='FAULT'


@pytest.mark.parametrize('changes,reason', [({'telemetry_stamp':.5},'telemetry'),
    ({'wheel_stamp':.5},'wheel'),({'rc_valid':False},'rc'),
    ({'authority_stamp':.5},'authority'),({'current_motor':math.nan},'telemetry')])
def test_stale_or_invalid_data_latches_fault(changes,reason):
    e=prepared(); step(e,1.17,**changes)
    assert e.state=='FAULT' and reason in e.reason
    step(e,1.18); assert e.state=='FAULT'


def test_clock_backwards_invalidates_no_auto_resume():
    e=prepared(); step(e,.9)
    assert e.state=='FAULT' and e.reason=='clock_backwards'
    step(e,1.3); assert e.state=='FAULT'


def test_initial_selection_wait_does_not_fault_and_disable_aborts():
    e=Engine(config()); e.enable(True)
    assert step(e,1.,authority=False).speed==0 and e.state=='PREPARE'
    assert step(e,2.,authority=False).speed==0 and not e.trials
    step(e,2.01); step(e,2.13); step(e,2.24); e.enable(False)
    assert e.state=='STOPPING' and not e.trials[-1]['success']
    req=step(e,2.14); assert req.mode==1 and req.current==0


def test_turn_exit_invalidates_trial_and_does_not_combine_windows():
    e=prepared(); step(e,1.17,current=8.)
    step(e,1.18,straight=False)
    assert not e.trials[-1]['success'] and e.reason=='straight_region_exit'
    assert e.state=='RECOVER' and len(e.active_samples)==0
    step(e,1.2,straight=True); step(e,1.31)
    assert e.state=='MEASURE' and not e.active_samples
    step(e,1.32); assert len(e.active_samples)==1


def test_timeout_unsuccessful_response_no_acceleration():
    e=prepared('accel_interval',trial_timeout=.2)
    step(e,1.17); step(e,1.3,v=1.2); step(e,1.4,v=1.3)
    assert not e.trials[-1]['success'] and e.trials[-1]['reason']=='trial_timeout'
    assert e.trials[-1]['mean_acceleration'] is None


def test_regions_wrap_and_distinct_region_boundary():
    r=Regions(10.,[{'id':'a','start_s':8.,'end_s':2.},{'id':'b','start_s':3.,'end_s':4.}])
    assert r.region(9.)=='a' and r.region(.5)=='a'
    assert r.region(2.) is None and r.region(3.5)=='b'
    with pytest.raises(ValueError): Regions(10.,[{'id':'a','start_s':0.,'end_s':0.}])


def test_rc_profiles_match_selector_one_based_and_turning_deadzone():
    p=RCProfile(); ch=[992]*16; ch[4]=1810; ch[7]=1810
    assert p.evaluate(ch)==(True,0.,True)
    ch[0]=1810; valid,steering,straight=p.evaluate(ch)
    assert valid and steering==pytest.approx(-.4751) and not straight
    ch[4]=172; assert not p.evaluate(ch)[0]
    ch[4]=1810; ch[7]=172; ch[8]=1810; assert not p.evaluate(ch)[0]
    with pytest.raises(ValueError): RCProfile(name='unknown')


def test_speed_hold_unstable_window_restarts_and_is_unsuccessful():
    e=prepared(); step(e,1.17); step(e,1.18,v=1.4)
    assert not e.trials[-1]['success'] and e.trials[-1]['reason']=='speed_unstable'
    assert e.state=='PREPARE' and not e.active_samples


def test_negative_current_stops_on_end_speed_even_without_new_telemetry():
    e=prepared('decel_current'); step(e,1.17)
    req=step(e,1.18,v=.4,telemetry_stamp=1.17)
    assert req.mode==0 and req.current==0
    assert not e.trials[-1]['success'] and e.trials[-1]['reason']=='boundary_without_new_telemetry'


def test_region_file_bound_to_exact_reference_hash(tmp_path):
    import json, hashlib
    reference=b'{"points": []}\n'; (tmp_path/'reference.json').write_bytes(reference)
    payload=dict(schema_version=1,reference_sha256=hashlib.sha256(reference).hexdigest(),
                 regions=[dict(id='a',start_s=1.,end_s=2.,kind='straight')])
    (tmp_path/'calibration_regions.json').write_text(json.dumps(payload))
    assert Regions.load(tmp_path,10.).region(1.5)=='a'
    (tmp_path/'reference.json').write_bytes(reference+b' ')
    with pytest.raises(ValueError,match='hash mismatch'): Regions.load(tmp_path,10.)


def test_preparing_requires_final_speed_mode_at_requested_target():
    e=Engine(config('accel_interval')); e.enable(True)
    e.sent(Request(1.,1,current=5.))
    obs=replace(observation(1.05),final_request=e.history[-1],final_received=1.05)
    req=e.tick(1.05,obs); e.sent(req)
    obs=replace(observation(1.16),final_request=Request(1.15,1,current=5.),final_received=1.16)
    e.tick(1.16,obs)
    assert e.state=='PREPARE' and e.stable_since is None


def test_rc_steering_inside_deadzone_still_matches_selector_normalization():
    channels=[992]*16; channels[4]=channels[7]=1810; channels[0]=1017
    valid,steering,straight=RCProfile().evaluate(channels)
    assert valid and straight and steering==pytest.approx(-.4751*25/818)


def test_reverse_motion_crossing_is_unsuccessful_and_requests_zero_current():
    e=prepared('decel_current'); step(e,1.17)
    req=step(e,1.18,v=-.1)
    assert e.state=='FAULT' and req.mode==1 and req.current==0
    assert not e.trials[-1]['success'] and e.reason=='reverse_motion'


@pytest.mark.parametrize('stage,velocity',[('accel_interval',2.1),('decel_current',.4)])
def test_interval_crossing_on_first_selected_current_sample_is_unsuccessful(stage,velocity):
    e=prepared(stage); req=step(e,1.17,v=velocity)
    assert req.mode==0 and req.current==0
    assert not e.trials[0]['success'] and e.trials[0]['mean_acceleration'] is None
    assert e.trials[0]['reason']=='interval_without_positive_duration'


@pytest.mark.parametrize('stage,crossed',[('accel_interval',2.1),('decel_current',.4)])
def test_fresh_wheel_boundary_stops_when_telemetry_stamp_repeats(stage,crossed):
    currents=(100.,) if stage=='accel_interval' else (-100.,)
    e=prepared(stage,currents=currents); step(e,1.17)
    req=step(e,1.18,v=crossed,telemetry_stamp=1.17)
    assert req.mode==0 and req.speed==req.current==0
    assert not e.trials[-1]['success'] and e.trials[-1]['mean_acceleration'] is None
    assert e.trials[-1]['start_stamp']==1.17 and e.trials[-1]['end_stamp']==1.18
    assert e.trials[-1]['end_speed']==crossed
    assert step(e,1.19,v=crossed,telemetry_stamp=1.17).current==0


@pytest.mark.parametrize('stage,crossed',[('accel_interval',2.1),('decel_current',.4)])
def test_fresh_wheel_boundary_stops_before_current_phase_echo(stage,crossed):
    e=prepared(stage)
    previous=next(r for r in reversed(e.history) if r.mode==0 and r.speed==1.)
    obs=replace(observation(1.17,v=crossed),final_request=replace(previous,stamp=1.17),final_received=1.17)
    req=e.tick(1.17,obs)
    assert req.mode==0 and req.speed==req.current==0
    assert not e.trials[-1]['success'] and e.trials[-1]['reason']=='boundary_before_current_echo'
    assert e.trials[-1]['start_stamp'] is None and e.trials[-1]['end_stamp']==1.17


def test_explicit_reenable_after_clock_rollback_uses_fresh_new_domain_handshake():
    e=Engine(config()); e.tick(10.,observation(10.)); e.tick(1.,observation(1.))
    assert e.state=='FAULT'
    e.enable(True)
    # Old domain observations and final echo cannot authorize the fresh run.
    req=e.tick(1.1,observation(10.)); assert e.state=='PREPARE' and req.speed==0
    e.sent(req)
    old=replace(observation(1.11),final_request=Request(1.05),final_received=1.11)
    req=e.tick(1.11,old); assert e.state=='PREPARE' and req.speed==0 and not e.selected
    e.sent(req)
    obs=replace(observation(1.12),final_request=Request(1.12),final_received=1.12)
    req=e.tick(1.12,obs)
    assert e.state=='PREPARE' and e.selected and req.speed==1.
    e.sent(req)
    # Runtime rollback must still latch a fault, even in the fresh clock domain.
    e.tick(1.0,observation(1.0)); assert e.state=='FAULT'


def test_phase_ack_is_first_selected_stamp_and_excludes_earlier_physical_sources():
    e=prepared('accel_interval')
    step(e,1.17,wheel_stamp=1.165,telemetry_stamp=1.165)
    assert e.phase_ack_stamp==1.17 and e.started is None and not e.active_samples
    # Later echo carries a later publication stamp; delayed physical sources after
    # the first acknowledgement remain eligible, without a moving source bound.
    step(e,1.19,wheel_stamp=1.18,telemetry_stamp=1.18)
    assert e.started==1.18 and e.phase_ack_stamp==1.17
    step(e,1.21,v=2.1,wheel_stamp=1.20,telemetry_stamp=1.20)
    assert e.trials[-1]['success'] and e.trials[-1]['first_current_ack_stamp']==1.17


@pytest.mark.parametrize('stage,end_speed,velocity,current',[
    ('accel_interval',1.1,1.15,100.),('decel_current',.9,.85,-100.)])
def test_preparing_already_past_end_never_emits_first_current(stage,end_speed,velocity,current):
    e=Engine(config(stage,end_speed=end_speed,currents=(current,)))
    e.enable(True)
    requests=[step(e,t,v=velocity) for t in (1.,1.01,1.05,1.16)]
    assert requests[-1].mode==0 and requests[-1].speed==requests[-1].current==0
    assert all(r.current==0 for r in requests)
    assert e.state=='COMPLETE'
    result=e.trials[-1]
    assert not result['success'] and result['reason']=='boundary_before_current_request'
    assert result['start_stamp'] is None and result['start_speed'] is None
    assert result['end_stamp']==1.16 and result['end_speed']==velocity
    assert result['first_current_ack_stamp'] is None and result['mean_acceleration'] is None
