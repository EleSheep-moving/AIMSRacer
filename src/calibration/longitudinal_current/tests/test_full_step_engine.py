from dataclasses import replace
import math
import pytest

from aims_calibration.engine import Config, Engine, Regions
from aims_calibration.planner import TrialPlan
from aims_calibration.protection import PulseProtection, ProtectionSettings
from test_engine import step


def prepared_full(stage='accel_interval', **overrides):
    start, end = (1., 5.) if stage == 'accel_interval' else (5., 1.)
    current = 20. if stage == 'accel_interval' else -5.
    values = dict(stage=stage, sweep_protocol='full_range_step', max_speed=5.,
                  speeds=(1., 5.), currents=(current,), start_speed=start, end_speed=end,
                  stable_seconds=.1, recovery_speed=2., predictive_exit=False, measured_current_handoff=True)
    values.update(overrides)
    item = TrialPlan(stage, start, end, current, baseline_current=9.,
                     kind='acceleration' if current > 0 else 'braking')
    protection = PulseProtection(lambda v: 8.+v, lambda item: 2., ProtectionSettings())
    engine = Engine(Config(**values), trial_plan=[item, item], protection=protection)
    engine.enable(True)
    for i in range(9):
        step(engine, 1.+i*.02, v=start, straight_remaining_m=30.)
        if engine.state=='APPLY':break
    assert engine.state == 'APPLY'
    return engine


def hold_to_recovery(e, t, target):
    for i in range(1, 22):
        t += .02
        step(e, t, v=target, current=e.handoff['target_current'] if e.handoff else 0.,
             straight_remaining_m=20.)
        if e.handoff is None:
            return t
    raise AssertionError('handoff did not finish')


def test_full_step_requires_protection_and_current_stage():
    with pytest.raises(ValueError):
        Config(sweep_protocol='full_range_step')
    with pytest.raises(ValueError):
        Config(sweep_protocol='other')


def test_measured_endpoint_keeps_current_until_wheel_reaches_five():
    e = prepared_full()
    step(e, 1.18, v=1.01, current=20., straight_remaining_m=30.)
    request = step(e, 1.20, v=4.8, current=20., straight_remaining_m=30.)
    assert request.current==20. and e.state=='MEASURE' and not e.trials
    request = step(e, 1.21, v=5., current=20., straight_remaining_m=30.)
    assert e.state == 'UNLOAD' and request.mode == 1 and request.current == 13.
    row = e.trials[-1]
    assert row['stop_reason'] == 'speed_boundary'
    assert row['data_status'] == 'complete' and row['endpoint_reached']
    assert e.index == 0
    for t in (1.22, 1.24):
        req = step(e, t, v=4.9, current=20., straight_remaining_m=30.)
        assert req.mode == 1 and e.state == 'UNLOAD'
    step(e, 1.26, v=4.95, current=13., straight_remaining_m=30.)
    req = step(e, 1.28, v=4.95, current=13., straight_remaining_m=30.)
    assert e.state == 'HOLD' and req.mode == 0 and req.speed == 5.
    assert row['handoff']['actual_iq_confirm_stamp'] == 1.28
    assert row['wheel_speed_max'] == 5.


def test_natural_steady_is_success_even_without_reaching_boundary():
    e = prepared_full()
    for i in range(1, 160):
        t = 1.16+i*.02
        step(e, t, v=1., current=20., straight_remaining_m=30.)
        if e.trials:
            break
    row = e.trials[-1]
    assert row['stop_reason'] == 'steady_state' and row['success']
    assert row['steady_evidence']['steady'] and not row['endpoint_reached']
    assert e.handoff['target_speed'] == 1. and e.handoff['target_current'] == 9.


def test_flat_wheel_with_wrong_iq_is_not_steady_and_exits_neutral():
    e = prepared_full()
    for i in range(1, 55):
        step(e, 1.16+i*.02, v=1., current=0., straight_remaining_m=30.)
        if e.trials:
            break
    assert e.trials[-1]['stop_reason'] == 'current_tracking_not_achieved'
    assert not e.trials[-1]['success'] and e.handoff['target_current'] == 0.


def test_timeout_preserves_partial_and_waits_for_next_straight():
    e = prepared_full(trial_timeout=.2)
    for i in range(1, 13):
        step(e, 1.16+i*.02, v=1., current=20., straight_remaining_m=30.)
        if e.trials:
            break
    row = e.trials[-1]
    assert row['stop_reason'] == 'trial_timeout' and row['data_status'] == 'partial'
    assert not row['attempts_exhausted'] and e.index == 0
    t = hold_to_recovery(e, 1.40, 2.)
    for i in range(1, 12):
        req = step(e, t+i*.02, v=1., current=0., straight_remaining_m=20.)
        assert req.mode == 0 and req.speed == 2.
    assert e.index == 0 and e.state == 'RECOVER'
    step(e, t+.26, v=2., straight=False, region='', straight_remaining_m=math.nan)
    step(e, t+.28, v=1., region='next', straight_remaining_m=20.)
    for i in range(1, 17):
        step(e, t+.28+i*.02, v=1., region='next', straight_remaining_m=20.)
        if e.state=='APPLY':break
    assert e.state == 'APPLY' and e.attempt_number == 2


def test_region_reserve_releases_before_straight_boundary():
    e = prepared_full()
    step(e, 1.18, v=1., current=20., straight_remaining_m=20.)
    req = step(e, 1.20, v=1., current=20., straight_remaining_m=.9)
    assert e.state == 'UNLOAD' and req.current == 0.
    assert e.trials[-1]['stop_reason'] == 'straight_headroom'
    assert e.trials[-1]['data_status'] == 'partial'


def test_C_keeps_current_until_measured_one_then_records_tail():
    e = prepared_full('decel_current')
    step(e, 1.18, v=4.99, current=-5., straight_remaining_m=30.)
    req = step(e, 1.20, v=1.2, current=-5., straight_remaining_m=30.)
    assert req.current == -5. and e.state == 'MEASURE' and not e.trials
    req = step(e, 1.21, v=1., current=-5., straight_remaining_m=30.)
    assert req.current==9. and e.trials[-1]['endpoint_reached']
    req = step(e, 1.22, v=.9, current=-5., straight_remaining_m=30.)
    assert req.current == 9. and e.trials[-1]['wheel_speed_min'] == .9


def test_missing_straight_distance_prevents_current_request():
    e = prepared_full()
    e._reset_trial(); e.state = 'PREPARE'
    for i in range(1, 15):
        req = step(e, 1.16+i*.02, v=1., straight_remaining_m=math.nan)
        assert req.mode != 1 or req.current != 20.
    assert e.trials[-1]['data_status'] == 'uncollected'
    assert e.trials[-1]['stop_reason'] == 'straight_distance_unavailable'


def test_wrapped_region_remaining_distance():
    r = Regions(10., [dict(id='a', start_s=8., end_s=2.)])
    assert r.remaining(9.) == 3.
    assert r.remaining(.5) == 1.5
    assert r.remaining(2.) is None


def test_source_gap_resets_start_speed_confirmation():
    e = prepared_full()
    e._reset_trial(); e.state='PREPARE'
    step(e, 1.18, v=1., straight_remaining_m=20.)
    step(e, 1.30, v=1., straight_remaining_m=20.)
    assert e.state == 'PREPARE'


def test_C_can_prepare_from_standstill_without_near_reverse_fault():
    e = prepared_full('decel_current')
    e._reset_trial();e.state='PREPARE'
    req = step(e, 1.18, v=0., straight_remaining_m=30.)
    assert e.state=='PREPARE' and req.mode==0 and req.speed==5.


@pytest.mark.parametrize('velocity',[5.,5.03])
@pytest.mark.parametrize('current',[9.1393125,0.,-15.])
def test_C_switches_on_first_target_wheel_without_stable_or_acceleration_wait(velocity,current):
    e=prepared_full('decel_current',stable_seconds=10.)
    e._reset_trial();e.state='PREPARE'
    kind='positive_deceleration' if current>0 else ('coast' if current==0 else 'braking')
    e.trial_plan[0]=replace(e.trial_plan[0],current=current,kind=kind,baseline_current=14.1393125);e.plan[0]=current
    step(e,1.18,v=4.7,straight_remaining_m=20.)
    req=step(e,1.20,v=4.99,straight_remaining_m=19.9)
    assert e.state=='PREPARE' and req.mode==0
    req=step(e,1.22,v=velocity,straight_remaining_m=19.8)
    assert abs(e.protection.acceleration)>e.protection.settings.quiet_acceleration
    assert e.state=='APPLY' and req.mode==1 and req.current==current
    assert req.stamp==e.first_current_request_stamp==1.22
    assert e.start_trigger_observation==dict(wheel_stamp=1.22,telemetry_stamp=1.22,
        velocity=velocity,request_stamp=1.22)
    step(e,1.24,v=5.04,current=current,straight_remaining_m=19.7)
    step(e,1.26,v=1.,current=current,straight_remaining_m=19.6)
    assert e.trials[-1]['start_trigger_observation']['velocity']==velocity
    assert e.trials[-1]['starting_wheel_observation']['velocity']==5.04


def test_C_target_trigger_requires_a_new_wheel_but_not_a_new_telemetry_sample():
    e=prepared_full('decel_current');e._reset_trial();e.state='PREPARE'
    step(e,1.18,v=4.7,straight_remaining_m=20.)
    step(e,1.20,v=4.99,straight_remaining_m=20.)
    req=step(e,1.21,v=5.03,wheel_stamp=1.20,straight_remaining_m=20.)
    assert e.state=='PREPARE' and req.mode==0
    req=step(e,1.22,v=5.03,telemetry_stamp=1.21,straight_remaining_m=20.)
    assert e.state=='APPLY' and req.mode==1 and req.current==-5.


def test_C_target_trigger_waits_for_matching_start_speed_echo():
    e=prepared_full('decel_current');e._reset_trial();e.state='PREPARE'
    # The old final command is still current mode when the wheel reaches 5.
    req=step(e,1.18,v=5.03,straight_remaining_m=20.)
    assert req.mode==0 and req.speed==5. and e.state=='PREPARE'
    req=step(e,1.20,v=5.04,straight_remaining_m=20.)
    assert req.mode==1 and req.current==-5. and e.state=='APPLY'


@pytest.mark.parametrize('override',[
    dict(straight=False,region='',straight_remaining_m=math.nan),
    dict(straight_remaining_m=.9),dict(straight_remaining_m=math.nan),
    dict(authority=False),dict(rc_valid=False),dict(telemetry_stamp=.9),
    dict(wheel_stamp=.9),dict(tracking_reason='Forward reference heading mismatch')])
def test_C_immediate_start_retains_region_authority_freshness_and_tracking_gates(override):
    e=prepared_full('decel_current');e._reset_trial();e.state='PREPARE'
    step(e,1.18,v=4.7,straight_remaining_m=20.)
    options=dict(straight_remaining_m=20.);options.update(override)
    req=step(e,1.20,v=5.03,**options)
    assert req.mode!=1 or req.current!=-5.
    assert e.start_trigger_observation is None


def test_boundary_on_old_speed_echo_is_partial_not_complete():
    e=prepared_full()
    step(e,1.18,v=1.,current=20.,straight_remaining_m=30.)
    step(e,1.20,v=1.001,current=20.,straight_remaining_m=30.)
    from test_engine import observation
    old=next(r for r in reversed(e.history) if r.mode==0 and r.speed==1.)
    obs=replace(observation(1.22,v=5.,current=20.,straight_remaining_m=30.),
        final_request=replace(old,stamp=1.22),final_received=1.22)
    e.tick(1.22,obs)
    row=e.trials[-1]
    assert row['data_status']=='partial' and not row['success']
    assert not row['endpoint_reached'] and not row['pulse_valid']


def test_unload_counts_two_strictly_post_ack_iq_samples():
    e=prepared_full();step(e,1.18,v=1.,current=20.,straight_remaining_m=30.)
    step(e,1.20,v=5.,current=20.,straight_remaining_m=30.)
    step(e,1.22,v=4.9,current=13.,straight_remaining_m=30.)
    assert e.state=='UNLOAD' and e.handoff['iq_samples']==0
    step(e,1.24,v=4.9,current=13.,straight_remaining_m=30.)
    assert e.state=='UNLOAD' and e.handoff['iq_samples']==1
    step(e,1.26,v=4.9,current=13.,straight_remaining_m=30.)
    assert e.state=='HOLD'


def test_late_iq_samples_cannot_rescue_expired_unload():
    e=prepared_full();step(e,1.18,v=1.,current=20.,straight_remaining_m=30.)
    step(e,1.20,v=5.,current=20.,straight_remaining_m=30.)
    # Keep all other sources fresh while current refuses to unload.
    for i in range(1,30):step(e,1.20+.02*i,v=4.9,current=20.,straight_remaining_m=30.)
    step(e,1.80,v=4.9,current=13.,straight_remaining_m=30.)
    req=step(e,1.82,v=4.9,current=13.,straight_remaining_m=30.)
    assert e.state=='FAULT' and req.current==0.


def test_three_truncated_attempts_are_separate_then_advance_current():
    e=prepared_full(trial_timeout=.2);t=1.16
    for attempt in range(1,4):
        count=len(e.trials)
        for _ in range(20):
            t+=.02
            step(e,t,v=1.,current=20.,region=f'straight-{attempt}',straight_remaining_m=30.)
            if len(e.trials)>count:break
        assert len(e.trials)==attempt
        assert e.trials[-1]['attempt_number']==attempt
        assert e.trials[-1]['attempts_exhausted']==(attempt==3)
        t=hold_to_recovery(e,t,2.)
        if attempt<3:
            t+=.02;step(e,t,v=2.,straight=False,straight_remaining_m=math.nan)
            for _ in range(30):
                t+=.02;step(e,t,v=1.,region=f'straight-{attempt+1}',straight_remaining_m=30.)
                if e.state=='APPLY':break
            assert e.state=='APPLY' and e.index==0
    assert e.index==1 and all(row['data_status']=='partial' for row in e.trials)
    assert all(a['end_stamp']<b['start_stamp'] for a,b in zip(e.trials,e.trials[1:]))


def test_historical_endpoint_bound_is_not_consulted_or_used_to_refuse_current():
    source=prepared_full()
    def forbidden(*args):raise AssertionError('endpoint prediction must not run')
    guard=PulseProtection(lambda v:8.+v,forbidden,ProtectionSettings())
    e=Engine(source.config,trial_plan=source.trial_plan,protection=guard);e.enable(True)
    requests=[step(e,1.+i*.02,v=1.,current=20.,straight_remaining_m=100.) for i in range(35)]
    assert e.state=='MEASURE' and not e.trials
    assert any(r.mode==1 and r.current==20. for r in requests)
    assert e.exit_prediction is None


def test_B_tail_above_old_five_point_two_threshold_is_logged_without_extra_cap():
    e=prepared_full()
    step(e,1.18,v=1.,current=20.,straight_remaining_m=30.)
    step(e,1.20,v=5.,current=20.,straight_remaining_m=30.)
    req=step(e,1.22,v=5.6,current=20.,straight_remaining_m=30.)
    assert e.state=='UNLOAD' and req.current==13.
    assert e.trials[-1]['wheel_speed_max']==5.6


def test_invalid_holding_current_fault_has_one_attempt_record():
    e=prepared_full(max_current=20.)
    e.protection.holding_current=lambda v:40.
    step(e,1.18,v=1.,current=20.,straight_remaining_m=30.)
    req=step(e,1.20,v=5.,current=20.,straight_remaining_m=30.)
    assert e.state=='FAULT' and req.current==0.
    assert len(e.trials)==1 and e.trials[0]['reason']=='invalid_holding_current'


def test_C_tail_valley_is_recorded_even_when_reversal_stop_trips():
    e=prepared_full('decel_current')
    step(e,1.18,v=4.9,current=-5.,straight_remaining_m=30.)
    step(e,1.20,v=1.,current=-5.,straight_remaining_m=30.)
    req=step(e,1.22,v=-.1,current=-5.,straight_remaining_m=30.)
    assert e.state=='FAULT' and req.mode==1 and req.current==0.
    assert e.trials[-1]['wheel_speed_min']==-.1
    assert e.trials[-1]['handoff']['failure']=='near_reverse'


@pytest.mark.parametrize('velocity',[.139,0.,-.1])
def test_C_disabled_reverse_guard_retains_iq_handoff_through_low_or_negative_wheel(velocity):
    e=prepared_full('decel_current')
    e.protection.settings=replace(e.protection.settings,reverse_guard_enabled=False)
    step(e,1.18,v=4.9,current=-5.,straight_remaining_m=30.)
    step(e,1.20,v=1.,current=-5.,straight_remaining_m=30.)
    req=step(e,1.22,v=velocity,current=-5.,straight_remaining_m=30.)
    assert e.state=='UNLOAD' and e.enabled and req.mode==1 and req.current==9.
    assert e.trials[-1]['wheel_speed_min']==velocity
    step(e,1.24,v=velocity,current=9.,straight_remaining_m=30.)
    req=step(e,1.26,v=velocity,current=9.,straight_remaining_m=30.)
    assert e.state=='HOLD' and req.mode==0 and req.speed==1.
    assert 'failure' not in e.handoff


@pytest.mark.parametrize('stage,velocity',[('accel_interval',.9),('decel_current',5.1)])
def test_steady_outside_baseline_domain_unloads_neutral_instead_of_raising(stage,velocity):
    e=prepared_full(stage)
    def bounded_holding(speed):
        if not 1.<=speed<=5.:raise ValueError('holding speed not covered by Stage A')
        return 8.+speed
    e.protection.holding_current=bounded_holding
    for i in range(1,170):
        req=step(e,1.16+.02*i,v=velocity,current=e._item().current,straight_remaining_m=30.)
        if e.trials:break
    assert e.state=='UNLOAD' and req.mode==1 and req.current==0.
    assert e.trials[-1]['stop_reason']=='steady_state'
    assert e.trials[-1]['handoff']['target_speed']==2.
    assert e.trials[-1]['handoff']['holding_current_unavailable']
