from copy import deepcopy
import hashlib
import json
import pytest

from aims_calibration.operator import check_start, summarize_results


def ready():
    return dict(state='IDLE', enabled=False, enable_count=0, logging_error='', logging_dropped=0,
        freshness=.2, ages={name:0.01 for name in ('wheel_source','telemetry_source','final_source',
            'rc_source','authority_source','wheel_receipt','telemetry_receipt','final_receipt','rc_receipt','authority_receipt')},
        wheel_speed=0., telemetry_valid=True, authority=True, rc_valid=True, final_zero=True,
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
