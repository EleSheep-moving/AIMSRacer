from copy import deepcopy
import hashlib
import json
import pytest

from aims_calibration.operator import check_start, summarize_results


def ready():
    return dict(state='IDLE', enabled=False, enable_count=0, logging_error='', logging_dropped=0,
        freshness=.2, ages={name:0.01 for name in ('wheel_source','telemetry_source','final_source',
            'rc_source','authority_source','wheel_receipt','telemetry_receipt','final_receipt','rc_receipt','authority_receipt')},
        current_signal='avg_iq',wheel_speed=0., telemetry_valid=True, authority=True, rc_valid=True, final_zero=True,
        publishers=dict(calibration=1, final=1, drive=0), driver_startup_disabled=True,
        driver_parameter_age=.1, pp=True, localization_reason='', cross_track=.1, heading_error=.2,
        require_acceleration=True, acceleration_valid=True, artifact_error='')


def payload():
    config=dict(stage='speed_hold',speeds=[1.,1.5],repeats=1,measurement_seconds=5.)
    digest=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return dict(configuration=config,metadata=dict(evidence_valid=True,configuration_sha256=digest),
        trials=[dict(plan_index=i,stage='speed_hold',requested_target=v,success=True,
                     evidence_valid=True,duration_s=5.) for i,v in enumerate(config['speeds'])])


def test_ready_start():
    assert check_start(ready())==[]


@pytest.mark.parametrize('key,value,reason',[
    ('authority',False,'calibration_authority_required'),
    ('driver_startup_disabled',False,'driver_startup_must_be_disabled'),
    ('driver_parameter_age',3.,'driver_parameter_unverified'),
    ('wheel_speed',.11,'vehicle_must_be_stationary'),
    ('cross_track',.26,'reference_distance_exceeded'),
    ('heading_error',.36,'reference_heading_exceeded'),
    ('enable_count',1,'acquisition_already_started'),
    ('enabled',True,'acquisition_already_started'),
    ('acceleration_valid',False,'acceleration_unavailable'),
    ('artifact_error','reference changed','artifact_changed'),
])
def test_refuses_invalid_preflight(key,value,reason):
    data=ready();data[key]=value
    assert reason in check_start(data)


def test_stale_source_even_when_receipt_is_fresh():
    data=ready();data['ages']['wheel_source']=.3
    assert 'wheel_source_stale_or_future' in check_start(data)
    data['ages']['wheel_source']=-.01
    assert 'wheel_source_stale_or_future' in check_start(data)


def test_missing_data_fails_closed_and_acc_is_optional():
    assert check_start({})
    data=ready();data['require_acceleration']=False;data['acceleration_valid']=False
    assert check_start(data)==[]


def test_rc_mode_does_not_require_map_or_reference():
    data=ready();data.update(pp=False,cross_track=float('nan'),heading_error=float('nan'),localization_reason='no map')
    assert check_start(data)==[]


def test_complete_results_and_configuration_identity():
    data=payload();summary=summarize_results(data)
    assert summary['valid'] and summary['accepted']==summary['planned']==2
    data['configuration']['measurement_seconds']=4.
    assert 'configuration_hash_mismatch' in summarize_results(data)['errors']


def test_partial_rejected_duplicate_and_wrong_target_do_not_pass():
    data=payload();data['trials'].pop()
    assert summarize_results(data)['missing']==[1]
    data=payload();data['trials'][0]['evidence_valid']=False
    assert not summarize_results(data)['valid']
    data=payload();data['trials'].append(deepcopy(data['trials'][0]))
    assert not summarize_results(data)['valid']
    data=payload();data['trials'][1]['requested_target']=2.
    assert not summarize_results(data)['valid']
    data=payload();data['metadata']['evidence_valid']=False
    assert not summarize_results(data)['valid']


def test_sample_duration_and_metadata_must_be_valid():
    data=payload();data['trials'][0]['duration_s']=4.99
    assert not summarize_results(data)['valid']
    assert not summarize_results({})['valid']


def test_invalid_metadata_never_counts_a_trial_as_accepted():
    data=payload();data['metadata']['evidence_valid']=False
    assert summarize_results(data)['accepted']==0


def test_malformed_container_is_reported_not_raised():
    assert not summarize_results([])['valid']
    assert not summarize_results({'configuration':None,'metadata':None})['valid']


def test_explicit_unknown_schema_is_rejected_and_legacy_is_retained():
    assert summarize_results(payload())['valid']
    data=payload();data['schema_version']=1
    assert summarize_results(data)['valid']
    data['schema_version']=3
    assert not summarize_results(data)['valid']


def current_payload():
    from dataclasses import asdict
    from aims_calibration.engine import Config,TrackingConfig
    from aims_calibration.planner import TrialPlan
    config=asdict(Config(stage='accel_interval',currents=(14.,19.),start_speed=1.,end_speed=1.5))
    settings=asdict(TrackingConfig())
    plan=[dict(plan_index=i,**asdict(TrialPlan('accel_interval',1.,1.5,target,baseline_current=9.)))
          for i,target in enumerate((14.,19.))]
    def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    obs=[dict(telemetry_stamp=1.25+.05*i,avg_iq=0.,current_motor=99.) for i in range(11)]
    evidence=dict(current_signal='avg_iq',target_current=14.,tolerance=1.4,sample_count=11,
        window_start_stamp=1.25,window_end_stamp=1.75,window_duration_s=.5,
        settle_seconds=.25,window_seconds=.5,min_samples=10,failure_fraction=.9,
        failure_count=11,failure_ratio=1.,under_magnitude_count=11,mean_iq=0.,mean_current_motor=99.)
    trigger=dict(current_signal='avg_iq',trial_id=1,attempt_id=1,attempt_number=1,plan_index=0,stage='accel_interval',
        requested_target=14.,current=14.,repeat=1,kind='acceleration',baseline_current=9.,
        start_target_speed=1.,end_target_speed=1.5,success=False,evidence_valid=True,
        reason='current_tracking_unreachable',first_current_request_stamp=.9,first_current_ack_stamp=1.,
        start_stamp=1.1,end_stamp=1.75,start_speed=1.,end_speed=1.,duration_s=.65,
        response_duration_s=.85,samples=20,mean_iq=0.,std_iq=0.,mean_current_motor=99.,std_current_motor=0.,mean_acceleration=None,
        tracking_status='unreachable',tracking_evidence=evidence,tracking_observations=obs,uncovered=False)
    skip=dict(**plan[1],success=False,reason='skipped_unreachable_current',triggering_attempt_id=1,evidence=deepcopy(evidence))
    return dict(schema_version=2,current_signal='avg_iq',configuration=config,
        metadata=dict(current_signal='avg_iq',evidence_valid=True,configuration_sha256=digest(config),tracking_configuration_sha256=digest(settings)),
        plan=plan,plan_sha256=digest(plan),tracking_configuration=settings,trials=[trigger],skipped=[skip])


def test_valid_current_limit_skip_is_resolved_without_becoming_a_measurement():
    summary=summarize_results(current_payload())
    assert summary['valid'] and summary['execution_complete'] and summary['coverage_complete']
    assert summary['accepted']==0 and summary['unreachable']==1 and summary['skipped']==1


def test_current_skip_needs_real_feedback_and_matching_scope():
    data=current_payload();data['trials'][0]['tracking_observations']=[dict(telemetry_stamp=1.25+.05*i,avg_iq=14.,current_motor=0.) for i in range(11)]
    assert not summarize_results(data)['valid']
    data=current_payload();data['skipped'][0]['triggering_attempt_id']=[]
    assert not summarize_results(data)['valid']
    data=current_payload();data['skipped'][0]['triggering_attempt_id']=99
    assert not summarize_results(data)['valid']
    data=current_payload();data['plan'][1]['start_speed']=1.5
    data['plan'][1]['end_speed']=2.
    data['plan_sha256']=hashlib.sha256(json.dumps(data['plan'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
    data['skipped'][0].update(start_speed=1.5,end_speed=2.)
    assert not summarize_results(data)['valid']


def test_successful_retry_resolves_plan_and_timeout_reports_gap():
    data=current_payload();data['skipped']=[];data['plan']=data['plan'][:1]
    data['plan_sha256']=hashlib.sha256(json.dumps(data['plan'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
    first=data['trials'][0];first.update(reason='straight_region_exit',tracking_status='unverified')
    second=deepcopy(first);second.update(trial_id=2,attempt_id=2,attempt_number=2,success=True,reason='completed',
        end_speed=1.5,mean_acceleration=.5/.65,tracking_status='unverified')
    data['trials'].append(second)
    summary=summarize_results(data)
    assert summary['valid'] and summary['accepted']==1 and summary['records']==2
    data['trials']=[first];first['reason']='trial_timeout'
    summary=summarize_results(data)
    assert summary['execution_complete'] and not summary['coverage_complete'] and not summary['valid']


def test_sweep_result_revalidates_baseline_and_generated_plan(tmp_path):
    from dataclasses import asdict
    from aims_calibration.configuration import prepare_engine
    from aims_calibration.engine import Config
    from aims_calibration.planner import export_baseline
    from aims_calibration.operator import canonical_hash
    from test_cli import stage_a_payload
    path=tmp_path/'baseline.json';path.write_text(json.dumps(export_baseline(stage_a_payload())))
    values=asdict(Config());values.update(stage='accel_interval',max_speed=5.,max_current=13.9,speeds=(1.,1.5))
    engine,metadata,manifest=prepare_engine(values,plan_mode='sweep',base_current_file=str(path))
    data=current_payload();data['plan']=manifest['plan'];data['plan_sha256']=manifest['plan_sha256']
    data['baseline']=manifest['baseline'];data['configuration']=asdict(engine.config)
    data['metadata'].update(metadata,configuration_sha256=canonical_hash(data['configuration']))
    data['metadata'].update(map_sha256='map',reference_sha256='reference')
    assert 'baseline_or_sweep_plan_invalid' not in summarize_results(data)['errors']
    data['baseline']['points'][0]['mean_iq']=99.
    assert 'baseline_or_sweep_plan_invalid' in summarize_results(data)['errors']
    value=data['baseline'];value['baseline_sha256']=canonical_hash({k:v for k,v in value.items() if k!='baseline_sha256'})
    data['metadata']['baseline_sha256']=value['baseline_sha256']
    assert 'baseline_or_sweep_plan_invalid' in summarize_results(data)['errors']


@pytest.mark.parametrize('part',['result','metadata','plan','trial','tracking','motor_only'])
def test_current_reader_refuses_unsupported_q_axis_identity(part):
    data=current_payload()
    if part=='result':data.pop('current_signal')
    elif part=='metadata':data['metadata'].pop('current_signal')
    elif part=='plan':
        for item in data['plan']:item.pop('current_signal')
        from aims_calibration.operator import canonical_hash
        data['plan_sha256']=canonical_hash(data['plan'])
    elif part=='trial':data['trials'][0].pop('current_signal')
    elif part=='tracking':data['trials'][0]['tracking_evidence'].pop('current_signal')
    else:
        for point in data['trials'][0]['tracking_observations']:point.pop('avg_iq')
    assert not summarize_results(data)['valid']


def test_preflight_refuses_telemetry_without_q_axis_identity():
    data=ready();data.pop('current_signal')
    assert 'q_axis_current_required' in check_start(data)


def test_historical_A_summary_explicitly_identifies_legacy_current():
    data=payload();summary=summarize_results(data)
    assert summary['valid'] and summary['legacy_current_signal'] is True
    assert summary['current_signal']=='legacy_current_motor'


def test_current_reader_rejects_floating_result_schema_even_with_q_identity():
    data=current_payload();data['schema_version']=2.0
    assert not summarize_results(data)['valid']


def test_completed_current_result_requires_actual_q_axis_stats():
    data=current_payload();data['skipped']=[];data['plan']=data['plan'][:1]
    from aims_calibration.operator import canonical_hash
    data['plan_sha256']=canonical_hash(data['plan'])
    row=data['trials'][0];row.update(success=True,reason='completed',end_speed=1.5,mean_acceleration=.5/.65)
    assert summarize_results(data)['valid']
    row.pop('mean_iq')
    assert not summarize_results(data)['valid']


def test_current_reader_trial_baseline_cannot_differ_from_frozen_q_axis_plan():
    data=current_payload();data['trials'][0]['baseline_current']=90.
    assert not summarize_results(data)['valid']
