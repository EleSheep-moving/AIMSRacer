"""Version 3 evidence keeps execution, coverage, and handoff separate."""
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import pytest
from aims_calibration.engine import Config, TrackingConfig
from aims_calibration.io import OutputLog
from aims_calibration.operator import canonical_hash, summarize_results
from aims_calibration import planner
from aims_calibration.protection import ProtectionSettings
from aims_calibration.steady import SteadySettings
from test_planner import a_payload


@dataclass
class FullLogConfig:
    stage: str = 'accel_interval'
    sweep_protocol: str = 'full_range_step'


def payload():
    baseline=planner.export_baseline(a_payload())
    config=asdict(Config(stage='accel_interval',currents=(5.,),max_speed=5.,max_current=10.,start_speed=1.,end_speed=5.))
    config.update(sweep_protocol='full_range_step',predictive_exit=False,measured_current_handoff=True)
    plan=[dict(plan_index=i,**asdict(p)) for i,p in enumerate(planner.build_full_step_plan('accel_interval',baseline,max_current=10.))]
    tracking=asdict(TrackingConfig());steady=asdict(SteadySettings());protection=asdict(ProtectionSettings())
    metadata=dict(evidence_valid=True,current_signal='avg_iq',configuration_sha256=canonical_hash(config),
        reference_sha256='a'*64,plan_mode='sweep',sweep_protocol='full_range_step',current_step=5.,baseline_sha256=baseline['baseline_sha256'],
        tracking_configuration_sha256=canonical_hash(tracking),steady_configuration_sha256=canonical_hash(steady),
        protection_configuration_sha256=canonical_hash(protection),response_limits_role='not_used',
        endpoint_exit_policy='measured_wheel_speed',operating_wheel_speed_limit_mps=None,current_handoff_enabled=True)
    return dict(schema_version=3,sweep_protocol='full_range_step',current_signal='avg_iq',configuration=config,
        metadata=metadata,plan=plan,plan_sha256=canonical_hash(plan),baseline=baseline,tracking_configuration=tracking,
        steady_configuration=steady,protection_configuration=protection,response_limits=None,trials=[],skipped=[],
        endpoint_exit_policy='measured_wheel_speed',operating_wheel_speed_limit_mps=None,current_handoff_enabled=True)


def row(item,attempt=1,complete=True):
    return dict(plan_index=item['plan_index'],attempt_id=attempt,stage=item['stage'],kind=item['kind'],repeat=item['repeat'],
        current_signal='avg_iq',requested_target=item['current'],current=item['current'],baseline_current=item['baseline_current'],
        start_target_speed=item['start_speed'],end_target_speed=item['end_speed'],evidence_valid=True,
        success=complete,endpoint_reached=False,data_status='complete' if complete else 'partial',
        reason='steady_state' if complete else 'straight_region_exit',stop_reason='steady_state' if complete else 'straight_region_exit',
        attempts_exhausted=not complete,attempt_number=1 if complete else 3,duration_s=3.,samples=151,mean_iq=item['current'],std_iq=.1,
        start_speed=1.,end_speed=2.,wheel_speed_min=1.,wheel_speed_max=2.,
        first_current_request_stamp=10.,first_current_ack_stamp=10.02,start_stamp=10.02,end_stamp=13.02,
        response_duration_s=3.02,mean_acceleration=1./3.,pulse_valid=True,
        steady_evidence=dict(steady=complete,slope=.01,speed_span=.02,sample_count=51,current_sample_count=26,
            current_good_fraction=1.,wheel_stamp=13.02,telemetry_stamp=13.02,window_start_stamp=12.02,
            current_window_start_stamp=12.52,confirmation_start_stamp=12.02,confirmed_seconds=1.),
        handoff=dict(started=13.02,ack_stamp=13.04,last_iq_stamp=13.08,actual_iq_confirm_stamp=13.08,
            iq_samples=2,hold_started=13.1,quiet_since=13.2,hold_confirmed_stamp=14.3,hold_confirmed=True,failure=''))


def test_full_log_uses_v3_without_flattening_nested_tail_evidence(tmp_path):
    log=OutputLog(tmp_path,FullLogConfig(),manifest=dict(sweep_protocol='full_range_step',plan=[]))
    trials=[dict(success=True,endpoint_reached=False,steady_evidence={'steady':True},handoff={'hold_confirmed':True})]
    log.trials(trials);log.close()
    result=json.loads((log.directory/'trials.json').read_text())
    assert result['schema_version']==3
    assert result['trials']==trials


def test_full_summary_complete_steady_response_does_not_require_endpoint():
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    result=summarize_results(data)
    assert result['valid'],result
    assert result['complete_responses']==len(data['plan'])
    assert result['execution_complete'] and result['coverage_complete']
    assert result['endpoint_reached']==0


def test_partial_exhaustion_resolves_execution_with_coverage_gap():
    data=payload();data['trials']=[row(p,i+1,complete=i!=0) for i,p in enumerate(data['plan'])]
    result=summarize_results(data)
    assert result['execution_complete'] and not result['coverage_complete']
    assert result['partial_responses']==1 and result['coverage_gaps']==[0]
    assert result['unexecuted']==[]
    assert not result['valid']


def test_final_handoff_is_required_even_if_all_responses_complete():
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    data['trials'][-1]['handoff']['hold_confirmed']=False
    result=summarize_results(data)
    assert not result['execution_complete'] and not result['valid']
    assert not result['final_handoff_committed']


@pytest.mark.parametrize('change', ['protocol','plan','baseline','steady','forged_steady','iq_confirmation'])
def test_full_summary_rejects_tampered_provenance_and_evidence(change):
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    if change=='protocol':data['sweep_protocol']='interval'
    elif change=='plan':
        data['plan'][0]['end_speed']=4.;data['plan_sha256']=canonical_hash(data['plan'])
    elif change=='baseline':data['baseline']['points'][0]['mean_iq']=999.
    elif change=='steady':data['steady_configuration']['max_speed_span']=99.
    elif change=='forged_steady':data['trials'][0]['steady_evidence']['sample_count']=1
    else:data['trials'][-1]['handoff']['actual_iq_confirm_stamp']=0.
    result=summarize_results(data)
    assert not result['valid']


def test_full_summary_suffix_rebuild_and_uncollected_preflight_remain_visible():
    data=payload();full=deepcopy(data['plan']);data['plan']=data['plan'][1:]
    for i,p in enumerate(data['plan']):p['plan_index']=i
    data['plan_sha256']=canonical_hash(data['plan'])
    data['metadata'].update(sweep_start_index=1,sweep_full_plan_trials=len(full),sweep_full_plan_sha256=canonical_hash(full))
    trial=row(data['plan'][0]);trial.update(success=False,data_status='uncollected',stop_reason='operator_disabled',reason='operator_disabled',pulse_valid=False,first_current_request_stamp=None,
        first_current_ack_stamp=None,start_stamp=None,duration_s=None,samples=0,handoff={})
    data['trials']=[trial]
    result=summarize_results(data)
    assert result['evidence_valid'],result
    assert result['uncollected_responses']==1 and not result['coverage_complete']
    assert not result['execution_complete']
    assert result['trials'][0]['stop_reason']=='operator_disabled'


def test_unconfirmed_earlier_handoff_invalidates_execution_even_with_final_commit():
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    data['trials'][0]['handoff']['hold_confirmed']=False
    result=summarize_results(data)
    assert not result['execution_complete']


def test_full_plan_does_not_infer_legacy_C_speed_group_skips():
    data=payload();data['trials']=[row(data['plan'][0])]
    data['skipped']=[dict(plan_index=1,triggering_attempt_id=1,reason='skipped_speed_group')]
    result=summarize_results(data)
    assert not result['valid'] and 'invalid_full_step_skip' in result['errors']


def test_exhausted_flag_without_three_truncation_attempts_cannot_finish_execution():
    data=payload();data['trials']=[row(p,i+1,complete=i!=0) for i,p in enumerate(data['plan'])]
    data['trials'][0]['attempt_number']=1
    result=summarize_results(data)
    assert not result['execution_complete']
    assert 0 in result['missing']


def test_one_sample_truncation_remains_valid_partial_evidence():
    data=payload();trial=row(data['plan'][0],complete=False)
    trial.update(samples=1,duration_s=0.,end_stamp=10.02,response_duration_s=.02,end_speed=1.,wheel_speed_max=1.)
    data['trials']=[trial]
    result=summarize_results(data)
    assert result['evidence_valid'],result
    assert result['partial_responses']==1
    assert not result['coverage_complete']


def test_global_fault_cannot_be_relabelled_complete_using_old_steady_evidence():
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    data['trials'][0].update(reason='near_reverse',stop_reason='near_reverse')
    assert not summarize_results(data)['valid']


@pytest.mark.parametrize('finish',['steady','endpoint'])
def test_actual_frozen_engine_rows_round_trip_through_v3_writer_and_acceptance(tmp_path,finish):
    from test_full_step_configuration import prepare_full
    from test_engine import step
    engine,metadata,manifest=prepare_full(tmp_path)
    metadata.update(configuration_sha256=canonical_hash(asdict(engine.config)))
    metadata.update({k:v for k,v in manifest['baseline']['source']['context'].items() if k in
        ('vehicle_sha256','map_sha256','reference_sha256','calibration_regions_sha256')})
    engine.enable(True)
    for i in range(450):
        now=1.+.02*i
        target=engine.handoff['target_speed'] if engine.handoff else (5.01 if finish=='endpoint' and len(engine.active_samples)>=2 else 1.)
        current=engine.handoff['target_current'] if engine.handoff else (engine._item().current if engine.state in ('APPLY','MEASURE') else 0.)
        step(engine,now,v=target,current=current,straight_remaining_m=100.)
        if any(r.get('handoff',{}).get('hold_confirmed') for r in engine.trials):break
    assert any(r['data_status']=='complete' for r in engine.trials)
    log=OutputLog(tmp_path/'output',engine.config,metadata,manifest)
    log.trials(engine.trials,skipped=engine.skipped);log.close()
    result=summarize_results(json.loads((log.directory/'trials.json').read_text()))
    assert result['schema_version']==3 and result['evidence_valid'],result
    assert result['complete_responses']==1 and result['uncollected_responses']==0
    assert result['final_handoff_committed']
    assert not result['execution_complete'] and not result['coverage_complete']


@pytest.mark.parametrize('damage',['old_window','source_skew','early_confirmation','early_handoff'])
def test_response_evidence_is_causal_and_bound_to_its_own_end(damage):
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    trial=data['trials'][0]
    if damage=='old_window':
        trial.update(first_current_request_stamp=1.,first_current_ack_stamp=1.02,response_duration_s=12.02)
        for key in ('wheel_stamp','telemetry_stamp','window_start_stamp','current_window_start_stamp','confirmation_start_stamp'):
            trial['steady_evidence'][key]-=5.
    elif damage=='source_skew':trial['steady_evidence']['telemetry_stamp']+=.2
    elif damage=='early_confirmation':trial['steady_evidence']['confirmation_start_stamp']=10.03;trial['steady_evidence']['confirmed_seconds']=2.99
    else:
        for key in ('started','ack_stamp','last_iq_stamp','actual_iq_confirm_stamp','hold_started','quiet_since','hold_confirmed_stamp'):
            trial['handoff'][key]-=5.
    result=summarize_results(data)
    assert not result['valid'],result


def test_nonempty_v3_skipped_cannot_bypass_retry_exhaustion():
    data=payload();data['trials']=[row(p,i+1,complete=i!=0) for i,p in enumerate(data['plan'])]
    data['trials'][0]['attempt_number']=1
    data['skipped']=[dict(plan_index=0,triggering_attempt_id=1,reason='attempts_exhausted')]
    result=summarize_results(data)
    assert not result['execution_complete']
    assert 'invalid_full_step_skip' in result['errors']


def test_hold_timeout_starts_after_actual_unload_confirmation():
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    data['trials'][0]['handoff'].update(actual_iq_confirm_stamp=13.5,last_iq_stamp=13.5,
        hold_started=13.5,quiet_since=18.,hold_confirmed_stamp=19.1)
    assert summarize_results(data)['valid']



@pytest.mark.parametrize('reason',['predictive_exit','response_limit_unavailable','insufficient_exit_headroom'])
def test_measured_endpoint_policy_rejects_removed_prediction_stops(reason):
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    data['trials'][0].update(reason=reason,stop_reason=reason)
    assert not summarize_results(data)['valid']


@pytest.mark.parametrize('change',['predictive_flag','handoff_flag','endpoint_policy','invented_limit','response_role'])
def test_measured_acquisition_policy_is_explicit_and_frozen(change):
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    if change=='predictive_flag':data['configuration']['predictive_exit']=True
    elif change=='handoff_flag':data['configuration']['measured_current_handoff']=False
    elif change=='endpoint_policy':data['endpoint_exit_policy']='predicted_speed'
    elif change=='invented_limit':data['operating_wheel_speed_limit_mps']=5.2
    else:data['metadata']['response_limits_role']='safety_gate'
    data['metadata']['configuration_sha256']=canonical_hash(data['configuration'])
    assert not summarize_results(data)['valid']



@pytest.mark.parametrize('damage',['iq_at_ack','unload_timeout','hold_timeout'])
def test_handoff_requires_strictly_postack_iq_and_configured_deadlines(damage):
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    handoff=data['trials'][0]['handoff']
    if damage=='iq_at_ack':
        handoff.update(actual_iq_confirm_stamp=handoff['ack_stamp'],last_iq_stamp=handoff['ack_stamp'])
    elif damage=='unload_timeout':
        handoff.update(actual_iq_confirm_stamp=13.8,last_iq_stamp=13.8,hold_started=13.8,quiet_since=13.9,hold_confirmed_stamp=15.)
    else:handoff['hold_confirmed_stamp']=20.
    assert not summarize_results(data)['valid']


def test_measured_endpoint_policy_accepts_acquisition_without_response_limits():
    data=payload();data['trials']=[row(p,i+1) for i,p in enumerate(data['plan'])]
    data['configuration'].update(predictive_exit=False,measured_current_handoff=True)
    data['metadata']['configuration_sha256']=canonical_hash(data['configuration'])
    for part in (data,data['metadata']):
        part.update(endpoint_exit_policy='measured_wheel_speed',operating_wheel_speed_limit_mps=None,current_handoff_enabled=True)
    data['metadata']['response_limits_role']='not_used'
    data['response_limits']=None
    data['metadata'].pop('response_limits_content_sha256',None)
    result=summarize_results(data)
    assert result['valid'],result
