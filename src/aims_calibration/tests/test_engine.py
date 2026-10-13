from dataclasses import replace
import math
import pytest
from aims_calibration.engine import Config, Engine, Observation, Request, RCProfile, Regions


def config(stage='speed_hold', **kw):
    values = dict(stage=stage, steering_source='rc', speeds=(1.,), stable_seconds=.1,
                  measurement_seconds=.2, hold_sampling='continuous', currents=(5.,) if stage=='accel_interval' else (-5.,),
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


def test_cumulative_hold_pauses_turn_and_restabilizes_in_another_region():
    e=prepared(hold_sampling='cumulative')
    e.region='a'
    step(e,1.17,current=6.,region='a'); step(e,1.27,current=8.,region='a')
    assert e.hold_elapsed==pytest.approx(.1) and not e.trials
    step(e,1.28,straight=False,current=99.)
    assert e.state=='RECOVER' and len(e.active_samples)==2 and e.hold_elapsed==pytest.approx(.1)
    for t in (1.4,1.55,1.7,1.85,2.): step(e,t,straight=False,current=99.)
    step(e,2.02,region='b'); step(e,2.1,region='b')
    assert e.state=='PREPARE' and len(e.active_samples)==2
    step(e,2.13,region='b'); step(e,2.14,current=10.,region='b')
    assert e.hold_elapsed==pytest.approx(.1)
    req=step(e,2.24,current=12.,region='b')
    result=e.trials[-1]
    assert e.state=='COMPLETE' and req.speed==0 and result['success']
    assert result['samples']==4 and result['mean_current_motor']==9.
    assert result['duration_s']==pytest.approx(.2) and result['wall_duration_s']==pytest.approx(1.07)
    assert result['segment_count']==2 and [s['region'] for s in result['segments']]==['a','b']
    assert [s['duration_s'] for s in result['segments']]==pytest.approx([.1,.1])
    assert [s['mean_current_motor'] for s in result['segments']]==[7.,11.]
    assert [r['segment_id'] for r in e.samples]==[1,1,2,2]


def test_cumulative_unstable_speed_keeps_only_prior_admitted_samples():
    e=prepared(hold_sampling='cumulative')
    step(e,1.17,current=6.); step(e,1.22,current=8.)
    step(e,1.23,v=1.4,current=99.)
    assert e.state=='PREPARE' and not e.trials and e.hold_elapsed==pytest.approx(.05)
    step(e,1.25); step(e,1.36); step(e,1.37,current=10.)
    step(e,1.47,current=12.); step(e,1.52,current=14.)
    result=e.trials[-1]
    assert result['success'] and result['samples']==5 and result['mean_current_motor']==10.
    assert result['duration_s']==pytest.approx(.2) and result['segment_count']==2
    assert result['segments'][0]['end_reason']=='speed_unstable'


def test_cumulative_duplicate_sources_and_uncovered_gaps_do_not_add_time():
    e=prepared(hold_sampling='cumulative',measurement_seconds=.15)
    step(e,1.17,current=6.)
    step(e,1.19,wheel_stamp=1.17,telemetry_stamp=1.17,current=99.)
    step(e,1.2,telemetry_stamp=1.17,current=99.)
    assert e.hold_elapsed==0 and len(e.active_samples)==1
    step(e,1.22,current=8.)
    assert e.hold_elapsed==pytest.approx(.05)
    step(e,1.35,wheel_stamp=1.22,current=99.)
    step(e,1.4,wheel_stamp=1.22,current=99.)
    step(e,1.43,current=99.)
    assert e.state=='PREPARE' and e.reason=='sample_source_gap'
    assert e.hold_elapsed==pytest.approx(.05) and len(e.active_samples)==2
    step(e,1.44); step(e,1.55); step(e,1.56,current=10.); step(e,1.66,current=12.)
    assert e.trials[-1]['success'] and e.trials[-1]['duration_s']==pytest.approx(.15)
    assert e.trials[-1]['mean_current_motor']==9.


def test_cumulative_adjacent_region_change_cannot_bridge_sources():
    e=prepared(hold_sampling='cumulative')
    e.region='a'
    step(e,1.17,region='a'); step(e,1.22,region='a')
    step(e,1.23,region='b')
    assert e.state=='RECOVER' and e.hold_elapsed==pytest.approx(.05)
    step(e,1.24,region='b'); step(e,1.35,region='b'); step(e,1.36,region='b')
    assert e.hold_elapsed==pytest.approx(.05) and len(e.hold_segments)==2


@pytest.mark.parametrize('abort',['disable','fault','rollback'])
def test_cumulative_abort_while_paused_invalidates_entire_stage_and_reenable_resets(abort):
    e=prepared(hold_sampling='cumulative')
    step(e,1.17); step(e,1.22); step(e,1.23,straight=False)
    if abort=='disable': e.enable(False)
    elif abort=='fault': step(e,1.24,straight=False,authority=False)
    else: step(e,1.24,straight=False,wheel_stamp=1.21)
    assert not e.trials[-1]['success'] and e.trials[-1]['duration_s']==pytest.approx(.05)
    assert e.trials[-1]['segment_count']==1
    e.enable(True,now=1.25)
    assert e.index==0 and e.hold_elapsed==0 and not e.active_samples and not e.hold_segments


def test_nine_speed_stages_collect_five_source_seconds_each_then_stop():
    speeds=tuple(1.+.5*i for i in range(9))
    e=Engine(config(speeds=speeds,max_speed=5.,hold_sampling='cumulative',measurement_seconds=5.))
    e.enable(True,now=.9)
    t=1.
    next_commands=[]
    for _ in range(2200):
        target=e.plan[e.index] if e.index<len(e.plan) else 0.
        # Each straight supplies less than 5 s; every stage must span turns.
        straight=int(round(t*20))%80<60
        before=e.index
        request=step(e,t,v=target if straight else 1.,straight=straight,
                     region='a' if int(t/4)%2==0 else 'b',current=target+5. if straight else 99.)
        if e.index>before: next_commands.append(request.speed)
        t=round(t+.05,8)
        if e.state=='COMPLETE': break
    assert e.state=='COMPLETE' and len(e.trials)==9
    assert [r['requested_target'] for r in e.trials]==list(speeds)
    assert all(r['success'] and r['duration_s']==pytest.approx(5.) for r in e.trials)
    assert all(r['segment_count']>=2 and r['mean_current_motor']==r['requested_target']+5. for r in e.trials)
    assert next_commands==list(speeds[1:])+[0.]
    assert step(e,t,v=0.).speed==0 and len(e.trials)==9


def test_five_mps_requires_explicit_hold_limit_and_current_stages_support_same_bound():
    assert config(speeds=(5.,),max_speed=5.).max_speed==5.
    with pytest.raises(ValueError): config(speeds=(5.,))
    with pytest.raises(ValueError): config(speeds=(5.1,),max_speed=5.1)
    assert config('accel_interval',max_speed=5.).max_speed==5.
    with pytest.raises(ValueError): config(hold_sampling='unknown')


def test_hold_speed_overshoot_waits_for_settling_without_stopping():
    e=Engine(config(speeds=(5.,),max_speed=5.,speed_overshoot_tolerance=.2,hold_sampling='cumulative'))
    e.enable(True)
    for t in (1.,1.01,1.05,1.16,1.17): request=step(e,t,v=5.1)
    assert e.state=='MEASURE' and request.speed==5.
    samples=len(e.samples)
    request=step(e,1.18,v=5.23)
    assert e.state=='PREPARE' and request.speed==5. and e.enabled
    assert len(e.samples)==samples and not e.trials
    for t in (1.2,1.25,1.31,1.32,1.42,1.52,1.53): step(e,t,v=5.)
    assert e.state=='COMPLETE' and e.trials[0]['success']
    with pytest.raises(ValueError): config(speed_overshoot_tolerance=.21)
    with pytest.raises(ValueError): config('accel_interval',speed_overshoot_tolerance=.1)


def planned_engine(stage='accel_interval', items=None, **kw):
    from aims_calibration.planner import TrialPlan
    if items is None:
        items=[TrialPlan(stage,1.,2.,5.,kind='acceleration',baseline_current=1.)] if stage=='accel_interval' else [TrialPlan(stage,2.,1.,-5.,kind='braking')]
    c=config(stage,start_speed=items[0].start_speed,end_speed=items[0].end_speed,max_speed=5.,**kw)
    return Engine(c,trial_plan=items)


def prepare_item(e,t=1.):
    target=e.trial_plan[e.index].start_speed
    if not e.enabled: e.enable(True)
    for offset in (0,.01,.05,.16): step(e,round(t+offset,8),v=target)
    assert e.state=='APPLY'
    return t+.16


def tracking_ticks(e, start, current, v=None, count=17, dt=.05):
    v=e.trial_plan[e.index].start_speed if v is None else v
    for i in range(count):
        t=round(start+.01+i*dt,8)
        step(e,t,v=v,current=current)
        if e.state not in ('APPLY','MEASURE'): break
    return t


def test_current_stages_support_five_mps_without_changing_A_config_fields():
    assert config('accel_interval',max_speed=5.,end_speed=5.).max_speed==5.
    assert config('decel_current',max_speed=5.,start_speed=5.).max_speed==5.
    from dataclasses import asdict
    assert 'tracking_config' not in asdict(config()) and 'trial_plan' not in asdict(config())


def test_structured_plans_are_validated_against_stage_and_explicit_config_limits():
    from aims_calibration.planner import TrialPlan
    item=TrialPlan('decel_current',2.,1.5,2.,kind='positive_deceleration',baseline_current=7.)
    with pytest.raises(ValueError): Engine(config('accel_interval'),trial_plan=[item])
    with pytest.raises(ValueError): Engine(config('decel_current',max_current=1.),trial_plan=[item])
    with pytest.raises(ValueError): Engine(config(),trial_plan=[item])
    with pytest.raises(ValueError): Engine(config('decel_current'),trial_plan=[])


def test_positive_and_coast_C_use_per_item_speed_recovery_and_current_mode():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('decel_current',2.,1.5,2.,kind='positive_deceleration',baseline_current=7.),
           TrialPlan('decel_current',2.5,1.,0.,kind='coast',baseline_current=8.)]
    e=planned_engine('decel_current',items)
    prepare_item(e); req=step(e,1.17,v=2.,current=2.)
    assert req.mode==1 and req.current==2.
    step(e,1.27,v=1.5,current=2.)
    assert e.index==1 and e.trials[-1]['success']
    assert step(e,1.3,v=1.5).speed==2.5
    for t in (1.31,1.42): step(e,t,v=2.5)
    assert step(e,1.43,v=2.5,current=9.).mode==1
    step(e,1.5,v=1.,current=9.)
    assert e.state=='COMPLETE' and e.trials[-1]['tracking_status']=='not_applicable'
    assert e.trials[-1]['mean_acceleration']<0


def test_short_success_retains_unverified_tracking_and_all_response_timestamps():
    e=planned_engine(); prepare_item(e)
    step(e,1.17,current=5.); step(e,1.27,v=2.,current=5.)
    row=e.trials[-1]
    assert row['success'] and row['tracking_status']=='unverified'
    assert row['first_current_request_stamp']==pytest.approx(1.16)
    assert row['first_current_ack_stamp']==1.17 and row['starting_wheel_observation']['wheel_stamp']==1.17
    assert row['response_duration_s']==pytest.approx(.11)
    assert row['duration_s']==pytest.approx(.1) and row['mean_acceleration']==pytest.approx(10.)
    assert row['attempt_id']==1 and row['repeat']==1 and row['kind']=='acceleration'
    assert e.serializable_plan[0]['plan_index']==0


def test_tracking_waits_for_settle_window_and_unique_telemetry_then_skips_B_interval():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('accel_interval',1.,1.5,i,baseline_current=1.) for i in (5.,10.,15.)]
    items+=[TrialPlan('accel_interval',1.5,2.,5.,baseline_current=1.)]
    e=planned_engine(items=items); prepare_item(e)
    tracking_ticks(e,1.16,0.,count=15)
    assert e.index==0 and not e.trials
    step(e,1.88,v=1.,current=0.,telemetry_stamp=1.87)
    assert e.index==0
    step(e,1.92,v=1.,current=0.)
    assert e.trials[-1]['reason']=='current_tracking_unreachable'
    assert e.trials[-1]['tracking_evidence']['sample_count']>=10
    assert len(e.skipped)==2 and e.index==3
    assert [s['plan_index'] for s in e.skipped]==[1,2]
    assert all(s['triggering_attempt_id']==1 for s in e.skipped)
    assert step(e,1.93,v=1.).speed==1.5


def test_positive_C_tracking_error_skips_positive_but_preserves_coast_and_negative():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('decel_current',2.,1.5,i,kind='positive_deceleration',baseline_current=10.) for i in (5.,.104)]
    items += [TrialPlan('decel_current',2.,1.,0.,kind='coast'),TrialPlan('decel_current',2.,1.,-5.,kind='braking')]
    e=planned_engine('decel_current',items); prepare_item(e)
    tracking_ticks(e,1.16,9.)
    assert e.index==2 and len(e.skipped)==1 and e.skipped[0]['kind']=='positive_deceleration'
    assert e.trials[-1]['tracking_evidence']['error_side']=='above'


def test_negative_C_under_tracking_skips_stronger_brakes_same_speed_only():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('decel_current',2.,1.,-i,kind='braking') for i in (5.,10.,15.)]
    items += [TrialPlan('decel_current',2.5,1.,-5.,kind='braking')]
    e=planned_engine('decel_current',items); prepare_item(e)
    tracking_ticks(e,1.16,-1.)
    assert e.index==3 and len(e.skipped)==2
    assert e.trials[-1]['tracking_status']=='unreachable'


def test_matching_current_tracking_and_pure_speed_timeout_continue_next_current():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('accel_interval',1.,1.5,i,baseline_current=1.) for i in (5.,10.)]
    e=planned_engine(items=items,trial_timeout=1.)
    prepare_item(e); t=tracking_ticks(e,1.16,5.)
    assert not e.trials and e.tracking_status=='verified'
    for i in range(6): t=round(t+.05,8); step(e,t,v=1.,current=5.)
    assert e.index==1 and not e.skipped
    assert e.trials[-1]['reason']=='trial_timeout' and e.trials[-1]['tracking_status']=='verified'


def test_straight_exit_retries_same_item_three_attempts_then_advances_uncovered():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('accel_interval',1.,1.5,i,baseline_current=1.) for i in (5.,10.)]
    e=planned_engine(items=items)
    t=1.
    for attempt in range(1,4):
        prepare_item(e,t); step(e,t+.17,v=1.,current=5.)
        req=step(e,t+.18,v=1.,straight=False,current=5.)
        assert e.trials[-1]['attempt_id']==attempt and e.trials[-1]['attempt_number']==attempt
        assert req.mode==0
        assert e.index==(0 if attempt<3 else 1)
        t=round(t+.3,8)
    assert e.trials[-1]['uncovered'] is True and not e.skipped
    assert [r['plan_index'] for r in e.trials]==[0,0,0]


def test_vesc_fault_code_immediately_faults_and_zeroes_current():
    e=prepared('accel_interval'); step(e,1.17)
    req=step(e,1.18,fault_code=3)
    assert e.state=='FAULT' and e.reason=='vesc_fault_3' and req.mode==1 and req.current==0
    assert not e.trials[-1]['success']


def test_tracking_settings_are_independent_validated_and_serializable():
    from aims_calibration.engine import TrackingConfig
    from dataclasses import asdict
    settings=TrackingConfig()
    assert asdict(settings)==dict(settle_seconds=.25,window_seconds=.5,min_samples=10,
        abs_tolerance=1.,rel_tolerance=.1,failure_fraction=.9,max_attempts=3)
    e=Engine(config('accel_interval'),tracking_config=settings)
    assert asdict(e.tracking_config)==asdict(settings)
    with pytest.raises(ValueError): TrackingConfig(min_samples=0)
    with pytest.raises(ValueError): TrackingConfig(failure_fraction=1.1)


def test_A_ignores_new_BC_fault_code_and_keeps_legacy_sample_fields():
    e=prepared(); step(e,1.17,fault_code=3)
    assert e.state=='MEASURE' and e.enabled
    assert 'fault_code' not in e.samples[-1]


def test_disabling_before_first_current_echo_retains_attempt_evidence():
    e=planned_engine(); prepare_item(e)
    e.enable(False)
    assert len(e.trials)==1 and e.trials[0]['attempt_id']==1
    assert e.trials[0]['first_current_request_stamp']==pytest.approx(1.16)
    assert e.trials[0]['first_current_ack_stamp'] is None
    assert e.trials[0]['reason']=='operator_disabled' and not e.trials[0]['success']


def test_apply_without_current_ack_times_out_from_first_producer_request():
    e=planned_engine(trial_timeout=.2); prepare_item(e)
    speed_request=next(r for r in reversed(e.history) if r.mode==0 and r.speed==1.)
    for t in (1.17,1.27,1.37):
        repeated=replace(speed_request,stamp=t)
        e.sent(repeated)
        req=e.tick(t,replace(observation(t),final_request=repeated,final_received=t))
        e.sent(req)
    assert e.state=='COMPLETE'
    assert not e.trials[-1]['success'] and e.trials[-1]['reason']=='trial_timeout'
    assert e.trials[-1]['first_current_ack_stamp'] is None
    assert e.trials[-1]['response_duration_s']==pytest.approx(.21)


def test_unreachable_skip_cannot_skip_a_weaker_manual_current():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('accel_interval',1.,1.5,i) for i in (10.,5.)]
    e=planned_engine(items=items); prepare_item(e)
    tracking_ticks(e,1.16,0.)
    assert e.index==1 and not e.skipped


def test_current_tracking_counts_unique_telemetry_even_if_wheel_source_repeats():
    e=planned_engine(); prepare_item(e)
    for i in range(17):
        t=round(1.17+i*.05,8)
        # Alternate a repeat wheel observation with a new telemetry observation.
        wheel=round(1.17+(i-i%2)*.05,8)
        step(e,t,current=0.,wheel_stamp=wheel)
        if e.state=='COMPLETE': break
    assert e.trials[-1]['tracking_status']=='unreachable'
    assert len(set(r['telemetry_stamp'] for r in e.trials[-1]['tracking_observations']))==len(e.trials[-1]['tracking_observations'])


def test_tracking_ignores_duplicate_telemetry_and_small_dense_windows():
    e=planned_engine(); prepare_item(e)
    for i in range(81):
        t=round(1.17+i*.01,8)
        # Telemetry source advances at 10 Hz, while wheel source advances at100Hz.
        telemetry=round(1.17+(i//10)*.1,8)
        step(e,t,current=0.,telemetry_stamp=telemetry)
    assert not e.trials and e.tracking_status=='unverified'
    assert e.tracking_evidence['sample_count']<10


def test_positive_C_mixed_error_sides_do_not_skip_and_B_over_tracking_is_not_under_magnitude():
    from aims_calibration.planner import TrialPlan
    item=TrialPlan('decel_current',2.,1.5,5.,kind='positive_deceleration',baseline_current=10.)
    e=planned_engine('decel_current',[item]); prepare_item(e)
    for i in range(17): step(e,round(1.17+i*.05,8),v=2.,current=1. if i%2 else 9.)
    assert e.tracking_status=='verified' and not e.trials and not e.skipped
    b=planned_engine(); prepare_item(b); tracking_ticks(b,1.16,20.)
    assert b.tracking_status=='verified' and not b.trials


def test_endpoint_crossing_wins_over_new_under_tracking_window_on_same_tick():
    e=planned_engine(); prepare_item(e)
    tracking_ticks(e,1.16,0.,count=15)
    step(e,1.92,v=2.,current=0.)
    assert e.trials[-1]['success'] and e.trials[-1]['reason']=='completed'
    assert not e.skipped and e.trials[-1]['tracking_status']=='unverified'


def test_PP_turn_recovery_stays_at_low_speed_then_prepares_item_start_on_straight():
    from aims_calibration.planner import TrialPlan
    item=TrialPlan('accel_interval',4.,4.5,10.,baseline_current=5.)
    e=planned_engine(items=[item],steering_source='pp')
    prepare_item(e); step(e,1.17,v=4.,current=10.)
    req=step(e,1.18,v=4.,straight=False,current=10.)
    assert req.mode==0 and req.speed==1. and e.index==0
    assert step(e,1.2,v=1.,straight=False).speed==1.
    assert step(e,1.3,v=1.,straight=True).speed==4.


def test_nonfinal_completion_recovers_immediately_to_next_item_speed():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('accel_interval',1.,1.5,5.),TrialPlan('accel_interval',1.5,2.,5.)]
    e=planned_engine(items=items); prepare_item(e); step(e,1.17,current=5.)
    request=step(e,1.27,v=1.5,current=5.)
    assert e.index==1 and request.mode==0 and request.speed==1.5 and request.current==0


def test_nonfinal_unreachable_skip_recovers_immediately_to_next_interval_speed():
    from aims_calibration.planner import TrialPlan
    items=[TrialPlan('accel_interval',1.,1.5,i) for i in (5.,10.)]
    items += [TrialPlan('accel_interval',1.5,2.,5.)]
    e=planned_engine(items=items); prepare_item(e)
    tracking_ticks(e,1.16,0.)
    assert e.index==2 and e.history[-1].mode==0 and e.history[-1].speed==1.5


def test_tracking_window_evaluates_irregular_source_timing_without_exact_half_second_span():
    e=planned_engine(); prepare_item(e)
    for i in range(20):
        step(e,round(1.17+i*.047,8),current=0.)
        if e.state=='COMPLETE': break
    assert e.state=='COMPLETE' and e.trials[-1]['tracking_status']=='unreachable'
    evidence=e.trials[-1]['tracking_evidence']
    assert evidence['window_duration_s']==pytest.approx(.5)
    assert evidence['sample_count']>=10
    rows=e.trials[-1]['tracking_observations']
    window=[r for r in rows if evidence['window_start_stamp']-1e-9<=r['telemetry_stamp']<=evidence['window_end_stamp']]
    assert len(window)==evidence['sample_count']
    assert window[-1]['telemetry_stamp']-window[0]['telemetry_stamp']<.5


def test_tracking_window_accepts_configured_duration_at_ros_epoch_precision():
    from aims_calibration.engine import TrackingConfig,Observation
    e=planned_engine()
    e.tracking_config=TrackingConfig(settle_seconds=0.,window_seconds=.12,min_samples=5)
    epoch=1791906320.4279172;e.phase_ack_stamp=epoch
    for i in range(5):
        stamp=epoch+i*.03
        e._track_current(Observation(telemetry_stamp=stamp,wheel_stamp=stamp,current_motor=5.))
    assert e.tracking_status=='verified'
    assert e.tracking_evidence['sample_count']==5
