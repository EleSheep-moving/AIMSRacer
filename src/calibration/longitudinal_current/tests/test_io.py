import csv
import json
from types import SimpleNamespace as NS
import pytest
from aims_calibration.engine import Config, Engine, Request, Observation
from aims_calibration.io import motor_telemetry, apply_request, OutputLog


def test_motor_telemetry_never_uses_battery_current_or_iq():
    state=NS(current_motor=7.,current_input=99.,avg_iq=123.,voltage_input=16.,speed=200.)
    assert motor_telemetry(NS(state=state))==(7.,16.,200.)
    del state.current_motor
    with pytest.raises(ValueError): motor_telemetry(NS(state=state))


def test_current_result_manifest_and_skip_checkpoint_preserve_a_schema(tmp_path):
    manifest=dict(plan=[dict(plan_index=0,current=15.)],plan_sha256='digest',tracking_configuration={})
    log=OutputLog(tmp_path,Config(stage='accel_interval',currents=(15.,),start_speed=1.,end_speed=1.5),manifest=manifest)
    skip=dict(plan_index=0,reason='skipped_unreachable_current',triggering_attempt_id=1)
    log.trials([],skipped=[skip]);log.close()
    result=json.loads((log.directory/'trials.json').read_text())
    assert result['schema_version']==2 and result['plan']==manifest['plan']
    assert result['skipped']==[skip]
    a=OutputLog(tmp_path,Config());a.close()
    assert 'schema_version' not in json.loads((a.directory/'trials.json').read_text())


def test_native_command_only_active_field_no_direct_driver_output():
    for request in (Request(2.,0,speed=1.),Request(2.,1,current=9.),Request(2.,1,current=-9.)):
        message=NS(header=NS(stamp=None,frame_id=''),control_mode=9,steering_angle=9.,
                   target_speed_mps=9.,target_current=9.,target_duty=9.,
                   target_brake_current=9.,target_acceleration_mps2=9.)
        apply_request(message,request,'ROS_STAMP')
        assert message.header.stamp=='ROS_STAMP' and message.header.frame_id=='base_link'
        assert message.control_mode==request.mode
        assert message.target_speed_mps==request.speed and message.target_current==request.current
        assert message.target_duty==message.target_brake_current==message.target_acceleration_mps2==0


def test_output_unique_raw_observations_and_atomic_trial_results(tmp_path):
    e=Engine(Config(steering_source='rc')); log=OutputLog(tmp_path,e.config)
    obs=Observation(wheel_stamp=1.,telemetry_stamp=1.,velocity=1.,current_motor=7.,voltage_input=16.,erpm=100.)
    log.observation(obs,Request(1.,0,speed=1.),'PREPARE')
    log.observation(obs,Request(1.01,0,speed=1.),'PREPARE')
    trial=dict(trial_id=1,success=False,reason='operator_disabled',mean_acceleration=None)
    log.trials([trial]); log.close()
    rows=list(csv.DictReader((log.directory/'observations.csv').open()))
    assert len(rows)==1 and rows[0]['current_motor']=='7.0'
    result=json.loads((log.directory/'trials.json').read_text())
    assert result['trials']==[trial] and result['configuration']['stage']=='speed_hold'
    assert list(csv.DictReader((log.directory/'trials.csv').open()))[0]['success']=='False'


def test_async_log_flushes_evidence_and_runtime_on_shutdown(tmp_path):
    from aims_calibration.io import AsyncOutputLog
    log=AsyncOutputLog(tmp_path,Config(steering_source='rc'),capacity=4)
    log.observation(Observation(wheel_stamp=1.,telemetry_stamp=1.),Request(1.),'IDLE')
    log.runtime(dict(ros_stamp=1.,callback_ms=.4,pending=1,dropped=0))
    log.trials([dict(success=False,reason='test')]); log.close()
    assert len(list(csv.DictReader((log.directory/'observations.csv').open())))==1
    assert len(list(csv.DictReader((log.directory/'runtime.csv').open())))==1
    assert json.loads((log.directory/'trials.json').read_text())['trials'][0]['reason']=='test'


def test_segment_evidence_round_trips_json_and_csv(tmp_path):
    log=OutputLog(tmp_path,Config(steering_source='rc'))
    segments=[dict(segment_id=1,region='a',duration_s=2.,samples=100),
              dict(segment_id=2,region='b',duration_s=3.,samples=150)]
    trial=dict(trial_id=1,success=True,hold_sampling='cumulative',duration_s=5.,
               wall_duration_s=12.,segment_count=2,segments=segments)
    log.trials([trial]); log.close()
    assert json.loads((log.directory/'trials.json').read_text())['trials']==[trial]
    csv_row=list(csv.DictReader((log.directory/'trials.csv').open()))[0]
    assert json.loads(csv_row['segments'])==segments


def test_async_log_overflow_is_visible_and_never_silently_drops(tmp_path):
    from aims_calibration.io import AsyncOutputLog
    from threading import Event
    log=AsyncOutputLog(tmp_path,Config(steering_source='rc'),capacity=1)
    entered,release=Event(),Event()
    real=log.writer.observation
    def blocked(*args):
        entered.set(); release.wait(2.); real(*args)
    log.writer.observation=blocked
    obs=Observation(wheel_stamp=1.,telemetry_stamp=1.)
    try:
        log.observation(obs,Request(1.),'IDLE'); assert entered.wait(1.)
        log.runtime(dict(ros_stamp=1.,callback_ms=.4,pending=1,dropped=0))
        with pytest.raises(RuntimeError,match='queue'): log.trials([])
        assert log.dropped_count==1 and log.pending_count==1
    finally:
        release.set(); log.close()


def test_overflow_durably_invalidates_previously_queued_success(tmp_path):
    from dataclasses import asdict
    from threading import Event
    from aims_calibration.io import AsyncOutputLog
    from aims_calibration.operator import canonical_hash,summarize_results
    config=Config(steering_source='rc',speeds=(1.,),measurement_seconds=.2)
    metadata=dict(evidence_valid=True,configuration_sha256=canonical_hash(asdict(config)))
    log=AsyncOutputLog(tmp_path,config,metadata,capacity=1)
    entered,release=Event(),Event();original=log.writer.observation
    def blocked(*args):entered.set();release.wait(2.);original(*args)
    log.writer.observation=blocked
    try:
        log.observation(Observation(wheel_stamp=1.,telemetry_stamp=1.),Request(1.),'MEASURE')
        assert entered.wait(1.)
        log.trials([dict(trial_id=1,plan_index=0,stage='speed_hold',requested_target=1.,
                        success=True,evidence_valid=True,duration_s=.25,reason='completed')])
        with pytest.raises(RuntimeError,match='overflow'):log.runtime(dict(ros_stamp=1.))
    finally:
        release.set();log.close()
    result=json.loads((log.directory/'trials.json').read_text())
    assert log.error is not None and result['metadata']['evidence_valid'] is False
    assert not summarize_results(result)['valid']


def test_failed_writer_accepts_latest_invalid_snapshot_without_queue_item(tmp_path):
    from aims_calibration.io import AsyncOutputLog
    log=AsyncOutputLog(tmp_path,Config())
    try:
        log._latch_error(RuntimeError('injected overflow'))
        with pytest.raises(RuntimeError):log.trials([dict(trial_id=1,success=False,reason='fault')])
        log.flush()
        result=json.loads((log.directory/'trials.json').read_text())
        assert len(result['trials'])==1 and result['metadata']['evidence_valid'] is False
    finally:log.close()


def test_overflow_backlog_cannot_replace_latest_failure_snapshot(tmp_path):
    from aims_calibration.io import AsyncOutputLog
    from threading import Event
    log=AsyncOutputLog(tmp_path,Config(),capacity=2)
    entered,release=Event(),Event();original=log.writer.runtime
    def blocked(row):entered.set();release.wait(2.);original(row)
    log.writer.runtime=blocked
    rows=[dict(trial_id=i,success=False,reason='fault') for i in range(1,4)]
    try:
        log.runtime(dict(stamp=1.));assert entered.wait(1.)
        log.trials(rows[:1]);log.trials(rows[:2])
        with pytest.raises(RuntimeError):log.trials(rows)
        release.set();log.flush()
        payload=json.loads((log.directory/'trials.json').read_text())
        assert len(payload['trials'])==3 and payload['metadata']['evidence_valid'] is False
    finally:release.set();log.close()


@pytest.mark.parametrize('damage',['name','duplicate_record','duplicate_key','stale','future'])
def test_localization_health_requires_fresh_unique_named_envelope(damage):
    from aims_calibration.io import health_values
    stamp=NS(sec=10,nanosec=0)
    record=NS(name='aims_racer_system/localization',values=[NS(key='protocol_version',value='1')])
    message=NS(header=NS(stamp=stamp),status=[record])
    assert health_values(message,10_000_000_000)=={'protocol_version':'1'}
    if damage=='name': record.name='arbitrary_record'
    elif damage=='duplicate_record': message.status.append(record)
    elif damage=='duplicate_key': record.values.append(NS(key='protocol_version',value='2'))
    elif damage=='stale': stamp.sec=9
    elif damage=='future': stamp.sec=11
    assert health_values(message,10_000_000_000)=={}


def test_csv_checkpoint_failure_never_leaves_success_json_without_later_queue(tmp_path,monkeypatch):
    from pathlib import Path
    from aims_calibration.io import AsyncOutputLog
    log=AsyncOutputLog(tmp_path,Config(steering_source='rc'))
    real_open=Path.open
    def fail_csv(path,*args,**kwargs):
        if path.name=='trials.csv.tmp': raise OSError('injected CSV checkpoint failure')
        return real_open(path,*args,**kwargs)
    monkeypatch.setattr(Path,'open',fail_csv)
    try:
        log.trials([dict(trial_id=1,success=True,reason='completed',mean_acceleration=2.)])
        log.queue.join()  # No subsequent queued result exists to repair this checkpoint.
        payload=json.loads((log.directory/'trials.json').read_text())
        assert log.error is not None and log.failed_after_trial_count==0
        assert payload['metadata']['evidence_valid'] is False
        assert payload['trials'] and not payload['trials'][0]['success']
        assert not payload['trials'][0]['evidence_valid'] and payload['trials'][0]['mean_acceleration'] is None
        assert payload['trials'][0]['response_completed'] and 'CSV checkpoint' in payload['trials'][0]['evidence_failure']
    finally:
        log.close()


@pytest.mark.parametrize('state',[NS(current_motor=12.),NS(current_motor=12.,avg_iq=float('nan')),NS(current_motor=12.,avg_iq=float('inf'))])
def test_q_axis_telemetry_requires_finite_iq_without_motor_fallback(state):
    from aims_calibration.io import q_axis_telemetry
    with pytest.raises(ValueError): q_axis_telemetry(NS(state=state))


def test_q_axis_telemetry_preserves_actual_q_value_and_output_identity(tmp_path):
    from aims_calibration.io import q_axis_telemetry
    assert q_axis_telemetry(NS(state=NS(current_motor=99.,avg_iq=1.23)))==1.23
    log=OutputLog(tmp_path,Config());log.close()
    assert json.loads((log.directory/'trials.json').read_text())['metadata']['current_signal']=='avg_iq'
