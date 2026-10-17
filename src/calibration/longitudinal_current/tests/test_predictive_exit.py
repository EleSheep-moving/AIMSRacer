from dataclasses import replace
import pytest
from aims_calibration.engine import Config, Engine, Request
from aims_calibration.planner import TrialPlan
try:
    from aims_calibration.protection import PulseProtection, ProtectionSettings
except ImportError:
    PulseProtection=ProtectionSettings=None
from test_engine import step, observation


def protected(stage='accel_interval', bound=2., end=None):
    assert PulseProtection is not None, 'predictive pulse protection is missing'
    start=1. if stage=='accel_interval' else 3.
    end=(1.5 if stage=='accel_interval' else 1.) if end is None else end
    item=TrialPlan(stage,start,end,20. if stage=='accel_interval' else -20.,baseline_current=10.,kind='acceleration' if stage=='accel_interval' else 'braking')
    cfg=Config(stage=stage,steering_source='pp',speeds=(1.,),currents=(20.,) if stage=='accel_interval' else (-20.,),
        start_speed=start,end_speed=end,max_speed=5.,stable_seconds=.1,recovery_speed=2.,predictive_exit=True)
    protection=PulseProtection(lambda v: 8.+v,lambda trial:bound,ProtectionSettings())
    engine=Engine(cfg,trial_plan=[item,item],protection=protection);engine.enable(True)
    for t in (1.,1.02,1.04,1.06,1.08,1.10,1.12,1.14,1.16):step(engine,t,v=start)
    return engine


def test_B_exits_before_wheel_boundary_and_commands_basic_current():
    e=protected();assert e.state=='APPLY'
    step(e,1.18,v=1.02);request=step(e,1.20,v=1.25)
    assert request.mode==1 and request.current==pytest.approx(9.5)
    assert e.state=='UNLOAD' and e.index==0
    row=e.trials[-1];assert row['reason']=='predictive_exit' and not row['success']
    assert row['end_speed']<1.5 and row['mean_acceleration'] is None
    assert row['pulse_mean_acceleration']>0 and row['exit_prediction']['predicted_velocity']>=1.5


def test_C_exits_before_target_and_cannot_reapply_negative_current():
    e=protected('decel_current',bound=5.)
    step(e,1.18,v=2.95);request=step(e,1.20,v=1.5)
    assert request.mode==1 and request.current==9. and e.state=='UNLOAD'
    assert e.index==0 and e.trials[-1]['end_speed']>1.
    for t in (1.22,1.24):assert step(e,t,v=1.2,current=-20.).current==9.
    assert e.state=='UNLOAD'


def test_unload_needs_actual_IQ_and_new_post_ack_telemetry():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    step(e,1.22,v=1.5,current=9.5,telemetry_stamp=1.20)
    assert e.state=='UNLOAD'
    step(e,1.24,v=1.5,current=9.5);assert e.state=='UNLOAD'
    request=step(e,1.26,v=1.5,current=9.5)
    assert e.state=='HOLD' and request.mode==0 and request.speed==1.5 and e.index==0
    for t in (1.28,1.30,1.32,1.34,1.36,1.38,1.40,1.42):step(e,t,v=1.5,current=9.5)
    assert e.index==1 and e.state=='PREPARE'
    assert e.trials[-1]['handoff']['hold_confirmed']


def test_unload_timeout_neutral_fault_has_no_next_pulse():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    for i in range(1,33):req=step(e,1.20+.02*i,v=1.5,current=20.)
    assert e.state=='FAULT' and req.mode==1 and req.current==0 and e.index==0
    assert e.reason=='iq_unload_timeout'


def test_current_with_insufficient_headroom_is_never_sent():
    e=protected(bound=20.)
    assert all(r.mode!=1 or r.current!=20. for r in e.history)
    assert e.trials[-1]['reason']=='insufficient_exit_headroom'
    assert e.trials[-1]['first_current_request_stamp'] is None


@pytest.mark.parametrize('stage,v,reason', [('accel_interval',5.3,'wheel_overspeed'),('decel_current',.2,'near_reverse')])
def test_emergency_gate_bypasses_holding_current(stage,v,reason):
    e=protected(stage);request=step(e,1.18,v=v)
    assert request.mode==1 and request.current==0 and e.state=='FAULT' and e.reason==reason


def test_turn_exit_unloads_before_requesting_2mps_speed():
    e=protected();request=step(e,1.18,straight=False)
    assert e.state=='UNLOAD' and request.mode==1 and request.current==0
    step(e,1.20,straight=False,current=0.);request=step(e,1.22,straight=False,current=0.)
    assert request.mode==0 and request.speed==2. and e.state=='HOLD'


def test_protection_requires_baseline_and_stage_BC():
    with pytest.raises(ValueError,match='protection'):Engine(Config(predictive_exit=True))


def test_causal_wheel_derivative_ignores_duplicates_and_resets_gap():
    p=PulseProtection(lambda v:9.,lambda trial:2.,ProtectionSettings())
    p.observe(1.,1.);p.observe(1.02,1.04);assert p.acceleration==pytest.approx(2.)
    p.observe(1.02,99.);assert p.acceleration==pytest.approx(2.)
    p.observe(1.5,1.);assert p.acceleration is None


def test_holding_current_must_be_finite_and_bounded():
    e=protected();e.protection.holding_current=lambda v:float('nan')
    request=step(e,1.18,v=1.4)
    assert e.state=='FAULT' and request.current==0


def test_BC_profiles_use_two_mps_and_enable_protection():
    import yaml
    from pathlib import Path
    for name in ('accel_sweep_1to5','decel_sweep_1to5'):
        p=yaml.safe_load((Path(__file__).parents[1]/'config'/f'{name}.yaml').read_text())['longitudinal_calibration']['ros__parameters']
        assert p['recovery_speed']==2. and p['predictive_exit'] is True


def test_configuration_binds_holding_current_and_response_limits_to_A(tmp_path):
    import json
    from dataclasses import asdict
    from aims_calibration.configuration import prepare_engine
    from aims_calibration.planner import export_baseline
    from test_cli import stage_a_payload
    baseline=export_baseline(stage_a_payload());path=tmp_path/'baseline.json';path.write_text(json.dumps(baseline))
    limits=tmp_path/'limits.json';limits.write_text(json.dumps(dict(schema_version=1,current_signal='avg_iq',
        baseline_sha256=baseline['baseline_sha256'],limits=[dict(stage='accel_interval',start_speed=1.,current=13.9,acceleration_bound=2.)])))
    values=asdict(Config());values.update(stage='accel_interval',speeds=(1.,1.5),predictive_exit=True)
    engine,metadata,manifest=prepare_engine(values,plan_mode='sweep',base_current_file=str(path),response_limits_file=str(limits))
    assert engine.protection.holding_current(1.5)==pytest.approx(9.6)
    assert engine.protection.response_limit(engine.trial_plan[0])==2.
    assert metadata['response_limits_sha256'] and manifest['protection_configuration']['exit_horizon']==.14
    with pytest.raises(ValueError):prepare_engine(values,plan_mode='sweep',base_current_file=str(path))
    with pytest.raises(ValueError):engine.protection.holding_current(.5)
    wrong=json.loads(limits.read_text());wrong['baseline_sha256']='bad';limits.write_text(json.dumps(wrong))
    with pytest.raises(ValueError):prepare_engine(values,plan_mode='sweep',base_current_file=str(path),response_limits_file=str(limits))


def test_handoff_invalid_IQ_cannot_be_hidden_by_matching_motor_current():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    for t in (1.22,1.24,1.26):step(e,t,v=1.5,current=9.5,avg_iq=20.)
    assert e.state=='UNLOAD'


def test_matching_old_pulse_echo_never_delays_predictive_exit():
    e=protected();step(e,1.18,v=1.02)
    obs=observation(1.20,v=1.3)
    old_speed=Request(1.19,0,speed=1.)
    # This speed command was actually emitted during preparation and is recent.
    e.history.append(old_speed)
    req=e.tick(1.20,replace(obs,final_request=old_speed,final_received=1.20))
    assert req.mode==1 and req.current==9.5 and e.state=='UNLOAD'


def test_first_prediction_uses_empirical_bound_before_derivative_is_available():
    p=PulseProtection(lambda v:9.,lambda trial:2.,ProtectionSettings())
    item=TrialPlan('accel_interval',1.,1.5,20.)
    assert p.prediction(item,1.,observation(1.))['predicted_velocity']==pytest.approx(1.28)


def test_group_recovery_waits_for_real_IQ_zero_even_when_wheel_forward():
    e=protected('decel_current');step(e,1.18,v=2.95)
    step(e,1.20,v=-.1,current=-20.)
    assert e.state=='GROUP_RECOVER'
    for i in range(1,15):step(e,1.20+i*.02,v=.1,current=-20.)
    assert e.state=='GROUP_RECOVER'


def test_post_pulse_fault_records_handoff_failure_without_duplicate_attempt():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    request=step(e,1.22,v=1.5,current=20.,authority=False)
    assert len(e.trials)==1 and request.current==0
    assert e.trials[0]['handoff']['failure']=='authority_lost_or_stale'


def test_pulse_validity_does_not_claim_full_endpoint_coverage():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    assert e.trials[0]['pulse_valid'] and not e.trials[0]['interval_completed']
    assert not e.trials[0]['success']


def test_IQ_confirmation_restarts_after_bad_selected_echo_and_actual_IQ():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    step(e,1.22,v=1.5,current=9.5)
    stale_pulse=Request(1.18,1,current=20.)
    obs=observation(1.24,v=1.5,current=20.,final_request=stale_pulse,final_received=1.24)
    req=e.tick(1.24,obs);e.sent(req)
    step(e,1.26,v=1.5,current=9.5)
    assert e.state=='UNLOAD'


def test_hold_clock_restarts_on_mismatching_echo_even_without_new_wheel():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    for t in (1.22,1.24,1.26,1.28,1.30,1.32):step(e,t,v=1.5,current=9.5)
    assert e.state=='HOLD' and e.handoff['quiet_since'] is not None
    old=Request(1.24,1,current=9.5)
    obs=observation(1.34,v=1.5,current=9.5,wheel_stamp=1.32,final_request=old,final_received=1.34)
    req=e.tick(1.34,obs);e.sent(req)
    assert e.handoff['quiet_since'] is None
    for t in (1.36,1.38,1.40,1.42):step(e,t,v=1.5,current=9.5)
    assert e.state=='HOLD' and e.index==0


def test_result_audit_rejects_protection_provenance_mutation(tmp_path):
    import json
    from dataclasses import asdict
    from aims_calibration.configuration import prepare_engine
    from aims_calibration.planner import export_baseline
    from aims_calibration.operator import summarize_results,canonical_hash
    from test_cli import stage_a_payload
    baseline=export_baseline(stage_a_payload());path=tmp_path/'A.json';path.write_text(json.dumps(baseline))
    limits=dict(schema_version=1,current_signal='avg_iq',baseline_sha256=baseline['baseline_sha256'],
        limits=[dict(stage='accel_interval',start_speed=1.,current=13.9,acceleration_bound=2.)])
    limitfile=tmp_path/'limits.json';limitfile.write_text(json.dumps(limits))
    values=asdict(Config());values.update(stage='accel_interval',speeds=(1.,1.5),predictive_exit=True,steering_source='rc',stable_seconds=.1)
    e,metadata,manifest=prepare_engine(values,plan_mode='sweep',base_current_file=str(path),response_limits_file=str(limitfile))
    cfg=asdict(e.config);metadata.update(evidence_valid=True,configuration_sha256=canonical_hash(cfg))
    data=dict(schema_version=2,configuration=cfg,metadata=metadata,trials=[],skipped=[],**manifest)
    assert summarize_results(data)['evidence_valid']
    e.enable(True)
    for t in (1.,1.02,1.04,1.06,1.08,1.10,1.12,1.14,1.16):step(e,t)
    step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    for i in range(1,13):step(e,1.20+.02*i,v=1.5,current=9.6)
    data['trials']=e.trials
    summary=summarize_results(data)
    assert summary['evidence_valid'],summary
    assert summary['measured_pulses']==1 and summary['settled_pulses']==1 and not summary['coverage_complete']
    handoff=data['trials'][0]['handoff'];original=handoff['started'];handoff['started']=None
    assert not summarize_results(data)['evidence_valid']
    handoff['started']=original
    data['protection_configuration']['exit_horizon']=.02
    assert not summarize_results(data)['evidence_valid']


def test_reversing_during_handoff_revokes_pulse_acceptance():
    e=protected('decel_current',bound=5.)
    step(e,1.18,v=2.95);step(e,1.20,v=1.5)
    assert e.trials[-1]['pulse_valid']
    step(e,1.22,v=-.1,current=-20.)
    assert e.state=='GROUP_RECOVER' and not e.trials[-1]['pulse_valid']


def test_speed_PID_resuming_excessive_deceleration_faults_before_near_reverse():
    e=protected('decel_current',bound=5.)
    step(e,1.18,v=2.95);step(e,1.20,v=1.5)
    for t in (1.22,1.24,1.26,1.28,1.30):step(e,t,v=1.2,current=9.)
    assert e.state=='HOLD'
    request=step(e,1.32,v=1.05,current=-30.)
    assert request.mode==1 and request.current==0 and e.state=='FAULT'
    assert e.reason=='post_pulse_predicted_excursion'


def test_overspeed_guard_also_applies_during_C_group_recovery():
    e=protected('decel_current');step(e,1.18,v=2.95);step(e,1.20,v=-.1)
    request=step(e,1.22,v=5.3,current=0.)
    assert e.state=='FAULT' and request.mode==1 and request.current==0 and e.reason=='wheel_overspeed'


def test_waiting_handoff_does_not_checkpoint_all_trials_at_50Hz():
    e=protected();step(e,1.18,v=1.02);step(e,1.20,v=1.25)
    revision=e.results_revision
    for t in (1.22,1.24,1.26):step(e,t,v=1.5,current=20.)
    assert e.results_revision==revision
    step(e,1.28,v=1.5,current=9.5);step(e,1.30,v=1.5,current=9.5)
    assert e.state=='HOLD' and e.results_revision==revision+1
