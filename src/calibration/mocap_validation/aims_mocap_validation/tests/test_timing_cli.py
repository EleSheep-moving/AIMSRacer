from pathlib import Path
import csv
import importlib
import json
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def test_clock_domains_and_unavailable_dds_are_distinct(tmp_path):
    assert (ROOT/'aims_mocap_validation/timing.py').exists(),'timing analysis is missing'
    timing=importlib.import_module('aims_mocap_validation.timing')
    origin=1_800_000_000_000_000_000
    fields=['sequence','topic','type','source_ns','dds_pub_ns','dds_receive_ns','receive_ros_ns','receive_system_ns','receive_steady_ns','cdr_sha256']
    with (tmp_path/'timing.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(fields)
        writer.writerow([1,'/own','sensor_msgs/msg/Imu',origin,origin+2_000_000,origin+3_000_000,origin+7_000_000,origin+7_000_000,100,'a'])
        writer.writerow([2,'/no_stamp','std_msgs/msg/Float64','','','',origin,origin,101,'b'])
    result=timing.analyze(tmp_path,{'vicon':{'topic':'/vicon'},'clock':{}},tmp_path/'out')
    assert result['topics']['/own']['source_to_dds_ms']['p50']==pytest.approx(2.)
    assert result['topics']['/own']['dds_to_callback_ms']['p50']==pytest.approx(5.)
    assert result['topics']['/own']['source_to_callback_ros_ms']['p50']==pytest.approx(7.)
    assert result['topics']['/no_stamp']['missing_source']==1
    assert 'source_to_dds_ms' not in result['topics']['/no_stamp']


def test_inspect_unknown_site_config_remains_actionable():
    assert (ROOT/'aims_mocap_validation/cli.py').exists(),'new CLI is missing'
    from aims_mocap_validation.configuration import load
    from aims_mocap_validation.session import readiness
    result=readiness(load(None))
    assert 'vrpn.server' in result['capture_missing']
    assert 'clock' in result['analysis_missing']
    assert result['can_record'] is False
