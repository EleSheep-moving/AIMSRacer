"""A transport failure must not become a successful full replay audit."""
import importlib.util
import json
from pathlib import Path


spec=importlib.util.spec_from_file_location('evaluate_replay',Path(__file__).resolve().parents[1]/'replay/evaluate_replay.py')
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
