"""Native executable contract tests. Run with NATIVE_CALIBRATION_EXECUTABLE set.

ROS_DOMAIN_ID must be isolated; these peers never launch or touch a driver.
"""
import json
import os
from pathlib import Path
import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from rosgraph_msgs.msg import Clock
from crsf_receiver_msg.msg import CRSFChannels16
from vesc_msgs.msg import VescCommand, VescStateStamped

EXE = os.environ.get('NATIVE_CALIBRATION_EXECUTABLE', '/tmp/calib-native-install/aims_calibration/lib/aims_calibration/longitudinal_calibration')


def vehicle_path():
    return Path(__file__).resolve().parents[3] / 'controller/config/vehicle.yaml'


def command(tmp_path, *extra):
    return [EXE, '--ros-args', '-p', 'steering_source:=rc', '-p', f'vehicle_config:={vehicle_path()}',
            '-p', f'log_directory:={tmp_path}', '-p', 'freshness:=0.1', '-p', 'speeds:=[1.0]', *extra]


def test_native_executable_exists_and_is_elf():
    assert Path(EXE).is_file(), 'sustained native executable is missing'
    assert Path(EXE).read_bytes()[:4] == b'\x7fELF'


def test_native_refuses_auto_start(tmp_path):
    completed = subprocess.run(command(tmp_path, '-p', 'auto_start:=true'), capture_output=True, text=True, timeout=15)
    assert completed.returncode != 0
    assert 'auto_start must remain false' in completed.stdout + completed.stderr


class Peer(Node):
    def __init__(self, frozen=False):
        super().__init__('native_calibration_test_peer')
        self.frozen = frozen
        self.clock_value = 5.
        self.clock_advances = False
        self.last_clock_feed = time.monotonic()
        self.pp = None
        self.sequence = 0
        self.values = {}
        self.requests = []
        self.echo = VescCommand()
        self.speed = 0.
        self.duplicate = None
        self.wheel = self.create_publisher(Odometry, '/rear_axle/wheel_odom', 10)
        self.telemetry = self.create_publisher(VescStateStamped, '/sensors/core', 10)
        self.rc = self.create_publisher(CRSFChannels16, '/rc/channels', 1)
        self.authority = self.create_publisher(Bool, '/control/calibration_enabled', 1)
        self.final = self.create_publisher(VescCommand, '/vesc/command', 1)
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.create_subscription(DiagnosticArray, '/calib/status', self.status, 10)
        self.create_subscription(VescCommand, '/calib/vesc_command', self.request, 1)
        self.enable = self.create_client(SetBool, '/calib/enable')
        self.create_service(GetParameters, '/vesc_driver_node/get_parameters', self.parameters)

    def request(self, message):
        self.echo = message
        self.requests.append(message)

    def parameters(self, request, response):
        response.values = [ParameterValue(type=1, bool_value=False)]
        return response

    def status(self, message):
        if message.status:
            self.values = {v.key: v.value for v in message.status[0].values}

    def feed(self, stale_source=False, receipt=True):
        stamp = self.get_clock().now().to_msg()
        if self.frozen:
            steady = time.monotonic()
            if self.clock_advances: self.clock_value += steady - self.last_clock_feed
            self.last_clock_feed = steady
            stamp.sec = int(self.clock_value)
            stamp.nanosec = int((self.clock_value - stamp.sec) * 1e9)
            clock = Clock(); clock.clock = stamp; self.clock.publish(clock)
            # DDS orders individual topics, not /clock versus wheel/telemetry.
            # A small source lag keeps these measurements causal whichever
            # subscription executes first, and remains inside freshness=.1.
            from builtin_interfaces.msg import Time
            source = self.clock_value - .05
            stamp = Time(sec=int(source), nanosec=int((source-int(source))*1e9))
        if stale_source:
            stamp.sec -= 1
        if not receipt:
            return
        wheel = Odometry(); wheel.header.stamp = stamp; wheel.child_frame_id = 'base_link'; wheel.twist.twist.linear.x = self.speed
        self.wheel.publish(wheel)
        telemetry = VescStateStamped(); telemetry.header.stamp = stamp
        telemetry.state.avg_iq = 1.23; telemetry.state.current_motor = 7.; telemetry.state.voltage_input = 16.; telemetry.state.speed = 100.
        self.telemetry.publish(telemetry)
        rc = CRSFChannels16()
        for i in range(1, 17): setattr(rc, f'ch{i}', 992)
        rc.ch8 = 1810
        self.rc.publish(rc)
        self.authority.publish(Bool(data=True))
        echo = self.echo
        # A synthetic final echo must retain the native request's source stamp
        # when the sensor measurements intentionally lag simulated /clock.
        if not self.frozen or not self.requests:echo.header.stamp = stamp
        echo.header.frame_id = 'base_link'
        self.final.publish(echo)
        if self.pp:
            import math
            from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
            from std_msgs.msg import String
            self.sequence += 1
            health = self.pp['fixture'].health_values(self.sequence, stamp.sec*10**9+stamp.nanosec)
            record = DiagnosticStatus(name='aims_racer_system/localization', values=[KeyValue(key=k, value=v) for k, v in health.items()])
            message = DiagnosticArray(status=[record]); message.header.stamp = stamp
            self.pp['health'].publish(message)
            self.pp['map'].publish(String(data='a'*64)); self.pp['valid'].publish(Bool(data=True))
            odometry = Odometry(); odometry.header.stamp = stamp; odometry.header.frame_id = 'odom'; odometry.child_frame_id = 'base_link'
            odometry.pose.pose.position.x = 2.; odometry.pose.pose.orientation.z = math.sin(math.pi/4); odometry.pose.pose.orientation.w = math.cos(math.pi/4)
            odometry.twist.twist.linear.x = self.speed
            self.pp['odometry'].publish(odometry)

    def spin(self, duration=.05, **feed):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.feed(**feed)
            rclpy.spin_once(self, timeout_sec=.005)

    def call(self, value):
        future = self.enable.call_async(SetBool.Request(data=value))
        end = time.monotonic() + 3.
        while not future.done() and time.monotonic() < end: self.spin(.01)
        assert future.done()
        return future.result()


@pytest.fixture
def runtime(tmp_path, request):
    rclpy.init()
    peer = Peer()
    log = (tmp_path / 'process.log').open('w')
    process = subprocess.Popen(command(tmp_path, *getattr(request, "param", [])), stdout=log, stderr=log)
    try:
        end = time.monotonic() + 12.
        while not peer.values and process.poll() is None and time.monotonic() < end: peer.spin()
        assert process.poll() is None, (tmp_path / 'process.log').read_text()
        assert peer.values
        yield peer, process, tmp_path
    finally:
        process.send_signal(__import__('signal').SIGINT)
        process.wait(timeout=10)
        log.close(); peer.destroy_node(); rclpy.shutdown()


def test_start_disabled_services_preflight_and_native_process(runtime):
    peer, process, root = runtime
    assert peer.values['state'] == 'IDLE' and peer.values['enabled'] == 'false'
    assert peer.values['startup_helper_exited'] == 'true'
    assert peer.values['runtime_implementation'] == 'native_cpp17'
    assert all(m.control_mode == 0 and m.target_speed_mps == 0 for m in peer.requests)
    assert process.poll() is None
    # Jetson kernels can omit task/children. PPid is available independently
    # of CONFIG_CHECKPOINT_RESTORE and still checks for sustained subprocesses.
    Path(f'/proc/{process.pid}/status').read_text()
    children = []
    for candidate in Path('/proc').iterdir():
        if not candidate.name.isdigit():continue
        try:
            fields = dict(line.split(':',1) for line in (candidate/'status').read_text().splitlines() if ':' in line)
            if int(fields['PPid']) == process.pid:children.append(int(candidate.name))
        except (FileNotFoundError,PermissionError,ProcessLookupError):
            continue
    assert not children, 'one-time Python helper must exit before native runtime'
    peer.duplicate = peer.create_publisher(VescCommand, '/vesc/command', 1)
    peer.spin(.8)
    refused = peer.call(True)
    assert not refused.success and 'exclusive_command_producers_required' in refused.message
    peer.destroy_publisher(peer.duplicate); peer.duplicate = None
    peer.spin(.8)
    assert peer.values['start_ready'] == 'true', peer.values['start_refusals']
    assert peer.call(True).success
    assert not peer.call(True).success
    assert peer.call(False).success
    peer.spin(.25)
    assert peer.requests[-1].control_mode == 1 and peer.requests[-1].target_current == 0
    directory = Path(peer.values['output_directory'])
    assert (directory / 'enable-attempt.json').is_file()
    assert (directory / 'runtime.csv').read_text().count('\n') > 3
    payload = json.loads((directory / 'trials.json').read_text())
    assert payload['metadata']['current_signal'] == 'avg_iq'


@pytest.mark.parametrize('failure', ['source', 'receipt', 'duplicate', 'writer'])
def test_native_latches_zero_for_freshness_publishers_and_writer(runtime, failure):
    peer, process, root = runtime
    peer.spin(.8)
    assert peer.call(True).success
    peer.spin(.2)
    if failure == 'source': peer.spin(.4, stale_source=True)
    elif failure == 'receipt': peer.spin(.4, receipt=False)
    elif failure == 'duplicate':
        peer.duplicate = peer.create_publisher(VescCommand, '/calib/vesc_command', 1);peer.spin(.6)
    else:
        directory = Path(peer.values['output_directory'])
        (directory / 'trials.json.tmp').symlink_to('/dev/full')
        assert peer.call(False).success
        peer.spin(.6)
    assert peer.values['state'] == 'FAULT', peer.values
    assert peer.requests[-1].control_mode == 1 and peer.requests[-1].target_current == 0
    if failure == 'writer': assert 'logging' in peer.values['reason']


def test_native_steady_timer_stops_when_ros_clock_freezes(tmp_path):
    rclpy.init(); peer = Peer(frozen=True)
    log = (tmp_path / 'process.log').open('w')
    process = subprocess.Popen(command(tmp_path, '-p', 'use_sim_time:=true'), stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 12.
        while peer.values.get('start_ready') != 'true' and process.poll() is None and time.monotonic() < deadline: peer.spin()
        assert peer.values.get('start_ready') == 'true', (tmp_path / 'process.log').read_text() + str(peer.values)
        assert peer.call(True).success
        peer.clock_advances = True
        peer.spin(.3)
        assert peer.values['state'] != 'FAULT', (peer.values['state'],peer.values['reason'])
        assert any(m.control_mode == 0 and m.target_speed_mps == 1. for m in peer.requests)
        peer.clock_advances = False
        peer.spin(.7)
        assert peer.values['state'] == 'FAULT'
        assert 'ros_clock_stalled' in peer.values['reason']
        assert peer.requests[-1].control_mode == 1 and peer.requests[-1].target_current == 0
    finally:
        process.send_signal(__import__('signal').SIGINT); process.wait(timeout=10)
        log.close(); peer.destroy_node(); rclpy.shutdown()


def test_native_runtime_parameters_are_frozen(runtime):
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter as ParameterMessage
    peer, process, root = runtime
    client = peer.create_client(SetParameters, '/longitudinal_calibration/set_parameters')
    assert client.wait_for_service(timeout_sec=2.)
    request = SetParameters.Request(parameters=[ParameterMessage(name='max_speed', value=ParameterValue(type=3, double_value=2.5))])
    future = client.call_async(request)
    deadline = time.monotonic() + 3.
    while not future.done() and time.monotonic() < deadline: peer.spin(.02)
    assert future.done()
    assert not future.result().results[0].successful, 'prepared calibration parameters must remain frozen'


@pytest.mark.parametrize('stage', ['accel_interval', 'decel_current'])
def test_native_full_step_startup_preserves_frozen_v3_and_measured_endpoint_policy(tmp_path, stage):
    import hashlib
    import importlib.util
    from aims_calibration.planner import export_baseline
    from aims_calibration.operator import summarize_results
    from test_cli import stage_a_payload
    source_root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('native_full_step_reference_fixture', source_root / 'aims_pp/test/conftest.py')
    fixture = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)
    reference = tmp_path / 'reference'; reference.mkdir()
    reference_file = reference / 'reference.json'; reference_file.write_text(json.dumps(fixture.circle_payload.__wrapped__()))
    reference_hash = hashlib.sha256(reference_file.read_bytes()).hexdigest()
    regions = reference / 'calibration_regions.json'
    regions.write_text(json.dumps(dict(schema_version=1, reference_sha256=reference_hash,
                                      regions=[dict(id='native-test-straight', start_s=0., end_s=6., kind='straight')])))
    source = stage_a_payload()
    source['metadata'].update(map_sha256='a'*64, reference_sha256=reference_hash,
                              calibration_regions_sha256=hashlib.sha256(regions.read_bytes()).hexdigest())
    baseline = tmp_path / 'A.json'; baseline.write_text(json.dumps(export_baseline(source)))
    profile = 'accel_full_step_1to5.yaml' if stage == 'accel_interval' else 'decel_full_step_5to1.yaml'
    config_root = source_root / 'calibration/longitudinal_current/config'
    args = [EXE, '--ros-args', '--params-file', str(config_root / profile),
            '-p', f'vehicle_config:={config_root / "speed_hold_vehicle.yaml"}',
            '-p', f'pp_config:={config_root / "speed_hold_pp.yaml"}', '-p', f'reference_directory:={reference}',
            '-p', f'base_current_file:={baseline}', '-p', f'log_directory:={tmp_path / "evidence"}',
            '-p', f'response_limits_file:={tmp_path / "absent-limits.json"}']
    rclpy.init(); peer = Peer(); log = (tmp_path / 'process.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 12.
        while not peer.values and process.poll() is None and time.monotonic() < deadline: peer.spin()
        assert process.poll() is None, (tmp_path / 'process.log').read_text()
        assert peer.values['state'] == 'IDLE' and peer.values['stage'] == stage
        assert peer.values['sweep_protocol'] == 'full_range_step'
        assert peer.values['predictive_exit'] == 'false' and peer.values['measured_current_handoff'] == 'true'
        assert all(m.control_mode == 0 and m.target_speed_mps == 0 for m in peer.requests)
        directory = Path(peer.values['output_directory'])
        payload = json.loads((directory / 'trials.json').read_text())
        assert payload['schema_version'] == 3 and payload['plan']
        assert payload['endpoint_exit_policy'] == 'measured_wheel_speed'
        assert payload['operating_wheel_speed_limit_mps'] is None
        assert payload['metadata']['response_limits_role'] == 'not_used'
        assert payload['response_limits'] is None
        summary = summarize_results(payload)
        assert summary['evidence_valid'], summary
        assert not summary['execution_complete'] and not summary['coverage_complete']
        assert not peer.call(True).success
        assert peer.values['logging_error'] == ''
        if stage == 'decel_current':
            from rclpy.qos import DurabilityPolicy
            from std_msgs.msg import String
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            peer.pp = dict(fixture=fixture,
                           health=peer.create_publisher(DiagnosticArray, '/localization/status', 10),
                           map=peer.create_publisher(String, '/localization/map_sha256', durable),
                           valid=peer.create_publisher(Bool, '/localization/map_valid', durable),
                           odometry=peer.create_publisher(Odometry, '/odometry/filtered', 10))
            deadline = time.monotonic() + 3.
            while peer.values.get('start_ready') != 'true' and time.monotonic() < deadline: peer.spin()
            assert peer.values.get('start_ready') == 'true', peer.values
            assert peer.call(True).success
            peer.spin(.25)
            assert peer.values['state'] == 'PREPARE', peer.values
            assert peer.requests[-1].control_mode == 0 and peer.requests[-1].target_speed_mps == 5.
            peer.requests.clear()
            peer.speed = 5.01
            peer.spin(.15)
            assert any(m.control_mode == 1 and m.target_current == payload['plan'][0]['current'] for m in peer.requests), peer.values
            # First wheel at/above 5 m/s admits C without waiting for the one-second steady window.
            assert peer.call(False).success
            peer.spin(.2)
            stopped = json.loads((directory / 'trials.json').read_text())
            row = stopped['trials'][0]
            assert row['start_trigger_observation']['velocity'] >= 5.
            assert row['starting_wheel_observation']['velocity'] >= 5.
            assert row['start_trigger_observation']['wheel_stamp'] <= row['first_current_request_stamp']
            assert not stopped['protection_configuration']['reverse_guard_enabled']
            assert peer.requests[-1].control_mode == 1 and peer.requests[-1].target_current == 0
    finally:
        process.send_signal(__import__('signal').SIGINT); process.wait(timeout=10)
        log.close(); peer.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('runtime', [[
    '-p', 'stable_seconds:=0.02', '-p', 'measurement_seconds:=0.08', '-p', 'hold_sampling:=continuous']], indirect=True)
def test_native_completed_stage_a_artifacts_pass_existing_acceptance(runtime):
    import csv
    from aims_calibration.operator import summarize_results
    peer, process, root = runtime
    peer.spin(.8)
    assert peer.call(True).success
    peer.speed = 1.
    deadline = time.monotonic() + 3.
    while peer.values.get('state') != 'COMPLETE' and time.monotonic() < deadline: peer.spin()
    assert peer.values.get('state') == 'COMPLETE', peer.values
    directory = Path(peer.values['output_directory'])
    payload = json.loads((directory / 'trials.json').read_text())
    result = summarize_results(payload)
    assert result['valid'] and result['accepted'] == 1, result
    assert payload['trials'][0]['mean_iq'] == pytest.approx(1.23)
    assert payload['trials'][0]['mean_current_motor'] == pytest.approx(7.)
    assert (directory / 'trials.csv').is_file()
    with (directory / 'observations.csv').open() as stream:
        observations = list(csv.DictReader(stream))
    assert any(row['state'] == 'MEASURE' for row in observations)
    assert observations[-1]['avg_iq'] == '1.23' and observations[-1]['current_motor'] == '7.0'
    with (directory / 'runtime.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows and all(float(row['callback_ms']) >= 0 for row in rows)
