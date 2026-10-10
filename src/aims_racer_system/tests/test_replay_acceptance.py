"""A transport failure must not become a successful full replay audit."""
import importlib.util
import json
from pathlib import Path


spec=importlib.util.spec_from_file_location('evaluate_replay',Path(__file__).resolve().parents[3]/'verification/localization/evaluate_replay.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def write_fixture(path,partial):
    summary=dict(tf_authorities={'odom/base_link':1},counts={'ekf':1,'body_cloud':1},
                 map_sha256=None,injection=[],full_bag_playback=False,
                 intentionally_partial=partial,
                 expected_raw_counts={'/livox/lidar':1,'/livox/imu':1})
    (path/'summary.json').write_text(json.dumps(summary))
    (path/'events.jsonl').write_text('')
    (path/'tf-authorities.jsonl').write_text('')
    (path/'fastlio-trace.csv').write_text('event,id,stamp,steady_ns,pending\n'
                                       'imu_received,0,1.0,1,0\nlidar_received,0,1.0,1,0\n')


def test_failed_player_cannot_pass_full_audit_even_when_all_inputs_arrived(tmp_path):
    write_fixture(tmp_path,False)
    result=module.evaluate(tmp_path)
    assert result['checks']['raw_sensor_delivery_complete']
    assert not result['checks']['full_bag_playback_completed']
    assert not result['all_checks_pass']


def test_explicit_partial_run_is_labelled_as_partial(tmp_path):
    write_fixture(tmp_path,True)
    result=module.evaluate(tmp_path)
    assert result['details']['intentionally_partial']
    assert 'full_bag_playback_completed' not in result['checks']
    assert 'raw_sensor_delivery_complete' not in result['checks']


def write_tracking_fixture(path,loss_after_ready):
    write_fixture(path,False)
    summary=json.loads((path/'summary.json').read_text())
    summary.update(map_sha256='fixture',full_bag_playback=True,independent_quality_samples=1,
                   tf_authorities={'odom/base_link':1,'map/odom':1})
    (path/'summary.json').write_text(json.dumps(summary))
    rows=[]
    for i in (1,2,3):
        rows.append(dict(kind='anchor',source_ns=i*10**9,ros_now_ns=i*10**9,received=float(i),
                         values=dict(epoch='e',event_sequence=str(i),anchor_sequence='1' if i==3 else '0',
                                     anchor_committed='true' if i==3 else 'false',reason='committed' if i==3 else 'initializing')))
    for received,event,sequence,ready in [(3.001,2,0,False),(3.002,3,1,True),(3.1,4,2,not loss_after_ready),(4.,5,3,True)]:
        rows.append(dict(kind='health',source_ns=int(received*10**9),ros_now_ns=int(received*10**9),received=received,
                         values=dict(epoch='e',event_sequence=str(event),anchor_sequence=str(sequence),
                                     ready=str(ready).lower())))
    (path/'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    (path/'tf-authorities.jsonl').write_text(json.dumps(dict(frame_id='map',child_frame_id='odom',
                                                         stamp_ns=3*10**9,receive_steady_ns=3*10**9))+'\n')


def test_initial_heartbeat_before_monitor_confirmation_is_not_tracking_loss(tmp_path):
    write_tracking_fixture(tmp_path,False)
    result=module.evaluate(tmp_path)
    assert result['all_checks_pass']
    assert result['details']['unready_health_during_playback']==0


def test_loss_after_monitor_confirmation_still_fails_tracking(tmp_path):
    write_tracking_fixture(tmp_path,True)
    result=module.evaluate(tmp_path)
    assert not result['checks']['continuous_tracking']
    assert not result['all_checks_pass']
