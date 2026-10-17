"""Real DDS contracts for the independent IMU/LIO body acceleration observer."""
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def test_native_executable_exists():
    executable = Path(os.environ.get('BODY_ACCELERATION_EXECUTABLE', '/missing/body_acceleration'))
    assert executable.is_file(), 'body_acceleration native executable must exist'


@pytest.fixture
def graph(tmp_path, request):
    executable = Path(os.environ.get('BODY_ACCELERATION_EXECUTABLE', '/missing/body_acceleration'))
    assert executable.is_file(), 'body_acceleration native executable must exist'
    rclpy.init()
    node = rclpy.create_node('body_acceleration_runtime_test')
    outputs, statuses, debug = [], [], []
    subscriptions = [node.create_subscription(AccelWithCovarianceStamped,
        '/state_estimation/body_acceleration', outputs.append, 100),
        node.create_subscription(DiagnosticArray, '/state_estimation/body_acceleration/status', statuses.append, 100),
        node.create_subscription(DiagnosticArray, '/state_estimation/body_acceleration/debug', debug.append, 100)]
    clock = node.create_publisher(Clock, '/clock', 10)
    imu = node.create_publisher(Imu, '/livox/imu_bias_corrected', qos_profile_sensor_data)
    lio = node.create_publisher(Odometry, '/rear_axle/lio_odom', qos_profile_sensor_data)
    logfile = (tmp_path / 'node.log').open('w')
    parameters = [item for key, value in getattr(request, 'param', {}).items()
                  for item in ('-p', f'{key}:={str(value).lower()}')]
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'use_sim_time:=true', *parameters],
        stdout=logfile, stderr=subprocess.STDOUT)
    class Graph:
        now = 1_800_000_000_000_000_003
        def spin(self, duration=.03):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                assert process.poll() is None, (tmp_path / 'node.log').read_text()
                rclpy.spin_once(node, timeout_sec=.005)
        def tick(self, ns=None):
            if ns is not None:
                self.now = ns
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(self.now, 10**9)
            clock.publish(msg)
            self.spin()
        def imu(self, ns=None, force=(0., 0., 1.), omega=(0., 0., 0.), frame='livox_frame', available=True, bad_covariance=False):
            msg = Imu()
            msg.header.frame_id = frame
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(self.now if ns is None else ns, 10**9)
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = force
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = omega
            msg.linear_acceleration_covariance[0] = 0. if available else -1.
            msg.angular_velocity_covariance[0] = 0. if available else -1.
            if bad_covariance:
                msg.angular_velocity_covariance[8] = float('nan')
            imu.publish(msg)
            self.spin()
        def lio(self, ns=None, pitch=0., frame='odom', child='base_link'):
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = frame, child
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(self.now if ns is None else ns, 10**9)
            msg.pose.pose.orientation.y = math.sin(pitch / 2)
            msg.pose.pose.orientation.w = math.cos(pitch / 2)
            lio.publish(msg)
            self.spin()
        def fields(self):
            self.spin(.15)
            assert statuses
            return {kv.key: kv.value for kv in statuses[-1].status[0].values}
    g = Graph()
    g.node, g.outputs, g.statuses, g.debug = node, outputs, statuses, debug
    try:
        deadline = time.monotonic() + 10.
        while min(clock.get_subscription_count(), imu.get_subscription_count(), lio.get_subscription_count()) < 1:
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            g.spin()
        g.spin(.2)
        g.tick()
        yield g
    finally:
        process.terminate()
        process.wait(timeout=5)
        logfile.close()
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


def _initialize(g, pitch=0., force=(0., 0., 1.)):
    first = g.now
    for index in range(31):
        g.tick(first + index * 5_000_000)
        g.imu(force=force)
    assert not g.outputs, 'IMU alone cannot determine body tilt'
    g.lio(first + 50_000_000, pitch=pitch)
    assert not g.outputs, 'delayed LIO callback must not publish old acceleration'
    g.tick(g.now + 5_000_000)
    g.imu(force=force)
    assert g.outputs, g.fields()


def test_delayed_initial_lio_source_stamp_and_independent_graph(graph):
    g = graph
    # Pose pitch=.2, f_B=R_WB^T*[0,0,g] => gravity corrected translation is zero.
    pitch = .2
    _initialize(g, pitch, (-math.sin(pitch), 0., math.cos(pitch)))
    out = g.outputs[-1]
    assert out.header.stamp.sec * 10**9 + out.header.stamp.nanosec == g.now
    assert out.header.frame_id == 'base_link'
    assert abs(out.accel.accel.linear.x) < .03
    assert 0 < out.accel.covariance[0] < 1e6
    assert [out.accel.covariance[i * 7] for i in range(1, 6)] == [1e6] * 5
    fields = g.fields()
    assert fields['x_valid'] == 'true'
    assert all(fields[key] == 'false' for key in ('y_valid', 'z_valid', 'angular_valid', 'extrinsics_calibrated', 'accelerometer_calibrated'))
    assert fields['covariance'] == 'model_uncalibrated'
    assert g.debug
    debug = {kv.key: kv.value for kv in g.debug[-1].status[0].values}
    assert abs(float(debug['reconstructed_ax'])) < .03
    for report in (fields, debug):
        assert 0. < float(report['alpha_window_span_s']) <= .025 + 1e-12
        assert 0. < float(report['alpha_sample_center_age_s']) <= .025
        assert float(report['alpha_sample_center_age_s']) <= float(report['alpha_window_span_s'])
        assert float(report['estimate_lio_anchor_age_s']) == pytest.approx(.105)
        assert float(report['latest_received_lio_source_age_s']) == pytest.approx(.105)
    subscriptions = {name for name, _ in g.node.get_subscriber_names_and_types_by_node('body_acceleration', '/')}
    assert '/livox/imu_bias_corrected' in subscriptions and '/rear_axle/lio_odom' in subscriptions
    assert subscriptions <= {'/livox/imu_bias_corrected', '/rear_axle/lio_odom', '/clock', '/parameter_events'}
    publishers = {name for name, _ in g.node.get_publisher_names_and_types_by_node('body_acceleration', '/')}
    assert not publishers.intersection({'/tf', '/tf_static', '/drive', '/vesc/command', '/odometry/filtered'})


def test_forward_translation_with_zero_tilt(graph):
    g = graph
    _initialize(g, force=(2. / 9.80665, 0., 1.))
    assert abs(g.outputs[-1].accel.accel.linear.x - 2.) < .03
    # A late attitude insertion must not republish the current IMU stamp.
    before = len(g.outputs)
    g.lio(g.now - 10_000_000)
    assert len(g.outputs) == before
    g.imu()  # Duplicate source time also must not produce another output.
    assert len(g.outputs) == before
    fields = g.fields()
    assert int(fields['observer.replays']) >= 2
    assert int(fields['observer.duplicate']) >= 1
    # A newer received anchor is distinct from the anchor applied at the last IMU.
    g.tick(g.now + 5_000_000)
    g.lio()
    assert len(g.outputs) == before
    fields = g.fields()
    assert float(fields['latest_received_lio_source_age_s']) == pytest.approx(0.)
    assert float(fields['lio_age_s']) == pytest.approx(0.)
    assert float(fields['estimate_lio_anchor_age_s']) == pytest.approx(.01)


def test_nonfinite_covariance_and_finite_overflow_cannot_publish(graph):
    g = graph
    _initialize(g)
    before = len(g.outputs)
    g.tick(g.now + 5_000_000)
    g.imu(bad_covariance=True)
    g.imu(omega=(float('nan'), 0., 0.))
    assert len(g.outputs) == before
    assert int(g.fields()['input.imu_nonfinite']) >= 2
    g.imu(force=(1e308, 0., 1.))
    assert len(g.outputs) == before
    fields = g.fields()
    assert fields['x_valid'] == 'false'
    assert fields['reason'] == 'numerical_error'
    assert int(fields['input.numerical_output_invalid']) >= 1
    g.tick(g.now + 5_000_000)
    g.imu()
    assert len(g.outputs) == before + 1
    assert g.fields()['x_valid'] == 'true'


def test_rejects_bad_inputs_reports_outage_and_resets_on_clock_reversal(graph):
    g = graph
    _initialize(g)
    before = len(g.outputs)
    g.imu(frame='base_link')
    g.imu(available=False)
    g.imu(force=(float('nan'), 0., 1.))
    g.imu(ns=g.now + 1)
    g.imu(ns=-1)
    g.lio(frame='map')
    assert len(g.outputs) == before
    fields = g.fields()
    for key in ('input.imu_wrong_frame', 'input.imu_unavailable', 'input.imu_nonfinite',
                'input.future_stamp', 'input.invalid_stamp', 'input.lio_wrong_frame'):
        assert int(fields[key]) >= 1, fields
    # Continue IMU beyond the LIO deadline: no fabricated zero or stale estimate.
    start = g.now
    for index in range(1, 251):
        g.tick(start + index * 5_000_000)
        g.imu()
    before = len(g.outputs)
    g.tick(g.now + 5_000_000)
    g.imu()
    assert len(g.outputs) == before
    fields = g.fields()
    assert fields['x_valid'] == 'false'
    assert fields['reason']
    assert float(fields['lio_age_s']) > .5
    # Fresh first-LIO/IMU pair is required after a global simulated clock rewind.
    g.tick(start - 2_000_000_000)
    g.imu()
    assert len(g.outputs) == before
    fields = g.fields()
    assert int(fields['input.clock_resets']) >= 1
    assert fields['x_valid'] == 'false'


def test_launch_uses_shared_geometry_and_field_flags_default_off(monkeypatch, tmp_path):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    def load(name):
        spec = importlib.util.spec_from_file_location('body_contract_' + name.replace('.', '_'), root / 'launch' / name)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(root))
        return module
    for name in ('vehicle.launch.py', 'race.launch.py'):
        context = LaunchContext()
        defaults = {a.name: ''.join(v.perform(context) for v in a.default_value)
                    for a in load(name).generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument) and a.default_value is not None}
        assert defaults['body_acceleration'] == 'false'
    module = load('body_acceleration.launch.py')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(geometry_config=str(root / 'params/rear_axle_geometry.yaml'),
        config=str(root / 'params/body_acceleration.yaml'), use_sim_time='false')
    nodes = module.build_node(context)
    assert len(nodes) == 1
    assert nodes[0]['package'] == 'aims_racer_system'
    assert nodes[0]['executable'] == 'body_acceleration'
    assert nodes[0]['parameters'][1] == dict(livox_translation=[.30, 0., .03],
        livox_quaternion=[0., 0., 0., 1.], use_sim_time=False)
    vehicle = load('vehicle.launch.py')
    monkeypatch.setattr(vehicle, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(vehicle, 'include', lambda package, filename, arguments=None: dict(include=filename))
    context.launch_configurations.update(mapping='false', vesc_config='/vesc', lio_config='/lio', mid360_config='/mid')
    assert not any(a.get('include') == 'body_acceleration.launch.py' for a in vehicle.build_vehicle(context))
    context.launch_configurations['body_acceleration'] = 'true'
    assert sum(a.get('include') == 'body_acceleration.launch.py' for a in vehicle.build_vehicle(context)) == 1
    race = load('race.launch.py')
    seen = []
    monkeypatch.setattr(race, 'include', lambda package, filename, arguments=None: seen.append((filename, arguments)) or object())
    map_file = tmp_path / 'map.pcd'
    map_file.write_text('map')
    bundle = tmp_path / 'bundle'
    bundle.mkdir()
    context.launch_configurations.update(map_file=str(map_file), artifact_directory=str(bundle),
        record='false', auto_start='false', repeat_laps='false', initial_pose='')
    race.build_race(context)
    assert next(args for name, args in seen if name == 'vehicle.launch.py')['body_acceleration'] == 'true'
    assert next(args for name, args in seen if name == 'mpcc.launch.py')['auto_start'] == 'false'
    from aims_racer_system.launch_support import recording_actions
    from launch.utilities import perform_substitutions
    context.launch_configurations.update(record='true', session_directory=str(tmp_path / 'session'))
    record = recording_actions(context, root, mode='race')[0]
    topics = {perform_substitutions(context, part) for part in record.cmd}
    assert {'/state_estimation/body_acceleration', '/state_estimation/body_acceleration/status',
        '/state_estimation/body_acceleration/debug', '/livox/imu_bias_corrected', '/rear_axle/lio_odom'} <= topics
