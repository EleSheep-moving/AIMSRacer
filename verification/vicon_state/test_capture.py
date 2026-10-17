import importlib.util
from pathlib import Path
import sys

from test_bag import bag_config


def capture():
    path=Path(__file__).with_name('capture.py')
    assert path.exists(), 'field inspect/record helper is missing'
    sys.path.insert(0,str(path.parent))
    spec=importlib.util.spec_from_file_location('vicon_capture',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_preflight_reports_missing_and_mismatched_topics():
    m=capture();c=bag_config()
    result=m.inspect_configuration(c,{'/mocap/car':['geometry_msgs/msg/PoseStamped']})
    assert result['type_mismatches'] and result['missing_topics']
    assert result['analysis_configuration_errors']==[]
    c['vicon']['topic']=None;c['clock']['source_anchor_ns']=None
    result=m.inspect_configuration(c,{})
    assert result['analysis_configuration_errors'] and result['capture_configuration_errors']


def test_record_topic_set_contains_raw_inputs_and_all_requested_streams():
    m=capture();c=bag_config();c['extra_topics']=['/vicon/frame_number','/vicon/occluded']
    topics=m.record_topics(c)
    assert {'/mocap/car','/own/velocity','/own/status','/livox/imu_bias_corrected',
            '/fastlio2/lio_odom','/vicon/frame_number','/vicon/occluded'} <= set(topics)
