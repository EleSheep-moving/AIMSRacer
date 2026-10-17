"""Real DDS contracts for dual reference-point estimates, not vehicle accuracy."""
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def test_native_executable_exists():
    assert Path(os.environ['STATE_OBSERVER_EXECUTABLE']).is_file(), 'dual-reference C++ node must be built'


@pytest.fixture
def graph(tmp_path, request):
    executable = Path(os.environ['STATE_OBSERVER_EXECUTABLE'])
    assert executable.is_file()
    rclpy.init()
    node = rclpy.create_node('dual_reference_contract_test')
    outputs = {ref: {kind: [] for kind in ('velocity', 'acceleration', 'yaw_rate', 'status')}
               for ref in ('rear_axle', 'center_of_mass')}
    types = dict(velocity=TwistWithCovarianceStamped, acceleration=AccelWithCovarianceStamped,
                 yaw_rate=Imu, status=DiagnosticArray)
    subscriptions = [node.create_subscription(types[kind], f'/state_estimation/{ref}/{kind}',
                        outputs[ref][kind].append, 1000)
                     for ref in outputs for kind in types]
    imu = node.create_publisher(Imu, '/livox/imu_bias_corrected', 256)
    lio = node.create_publisher(Odometry, '/fastlio2/lio_odom', 256)
    clock = node.create_publisher(Clock, '/clock', 10)
    log = (tmp_path/'state-node.log').open('w')
    mount_yaw = getattr(request, 'param', 0.)
    mounting = [0., 0., math.sin(mount_yaw/2), math.cos(mount_yaw/2)]
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'use_sim_time:=true',
                               '-p', 'livox_quaternion:='+str(mounting)],
                               stdout=log, stderr=subprocess.STDOUT)

    class Graph:
        epoch = now = 10_000_000_000

        def spin(self, duration=.012):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                assert process.poll() is None, (tmp_path/'state-node.log').read_text()
                rclpy.spin_once(node, timeout_sec=.002)

        def tick(self, stamp):
            self.now = stamp
            message = Clock(); message.clock.sec, message.clock.nanosec = divmod(stamp, 10**9)
            clock.publish(message); self.spin()

        def send_imu(self, stamp=None, frame='livox_frame', rate=.4, force=(0., 0., 1.), available=True):
            message = Imu(); message.header.frame_id = frame
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(self.now if stamp is None else stamp, 10**9)
            message.angular_velocity.z = float(rate)
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, force)
            if not available:
                message.linear_acceleration_covariance[0] = -1.
            imu.publish(message); self.spin()

        def send_lio(self, rate=.4, velocity=(2., .3), stamp=None, frame='odom'):
            stamp = self.now if stamp is None else stamp
            t = (stamp-self.epoch)*1e-9
            message = Odometry(); message.header.frame_id = frame; message.child_frame_id = 'livox_frame'
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            message.pose.pose.orientation.w = math.cos((rate*t+mount_yaw)/2)
            message.pose.pose.orientation.z = math.sin((rate*t+mount_yaw)/2)
            message.pose.pose.position.x = velocity[0]*t; message.pose.pose.position.y = velocity[1]*t
            lio.publish(message); self.spin()

        def motion(self, count=101, rate=.4, with_lio=True):
            start = self.now
            for i in range(count):
                self.tick(start+i*5_000_000)
                if with_lio and i % 20 == 0:
                    self.send_lio(rate=rate)
                self.send_imu(rate=rate)

        def fields(self, ref='rear_axle'):
            self.spin(.15)
            assert outputs[ref]['status']
            return {entry.key: entry.value for entry in outputs[ref]['status'][-1].status[0].values}

    g = Graph(); g.outputs = outputs; g.node = node
    try:
        deadline = time.monotonic()+10
        while not all(pub.get_subscription_count() for pub in (imu, lio, clock)):
            assert time.monotonic() < deadline, 'state node did not subscribe'
            g.spin(.05)
        g.tick(g.now)
        yield g
    finally:
        process.terminate(); process.wait(timeout=5); log.close()
        node.destroy_node(); rclpy.shutdown()


def source_stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


@pytest.mark.parametrize('graph', [0., math.pi/2], indirect=True)
def test_nonzero_vy_two_reference_points_and_independent_graph(graph):
    g = graph; g.motion()
    rear = g.outputs['rear_axle']; cg = g.outputs['center_of_mass']
    assert all(rear[kind] and cg[kind] for kind in ('velocity', 'acceleration', 'yaw_rate'))
    rv, cv = rear['velocity'][-1], cg['velocity'][-1]
    # At t=.5 s yaw=.2, world velocity=(2,.3). Analytic lever
    # corrections: rear vy -=.4*.30; CG vx +=.4*.03, vy -=.4*.09.
    vx_imu = 2*math.cos(.2)+.3*math.sin(.2)
    vy_imu = -2*math.sin(.2)+.3*math.cos(.2)
    assert rv.twist.twist.linear.x == pytest.approx(vx_imu, abs=1e-7)
    assert rv.twist.twist.linear.y == pytest.approx(vy_imu-.12, abs=1e-7)
    assert cv.twist.twist.linear.x == pytest.approx(vx_imu+.012, abs=1e-7)
    assert cv.twist.twist.linear.y == pytest.approx(vy_imu-.036, abs=1e-7)
    assert cv.twist.covariance[1] == pytest.approx(-2.7e-7, abs=1e-12)
    assert cv.twist.covariance[6] == cv.twist.covariance[1]
    assert rv.header.frame_id == 'base_link' and cv.header.frame_id == 'center_of_mass'
    ra, ca = rear['acceleration'][-1], cg['acceleration'][-1]
    assert ra.accel.accel.linear.x == pytest.approx(.048, abs=1e-7)
    assert ra.accel.accel.linear.y == pytest.approx(0., abs=1e-7)
    assert ca.accel.accel.linear.x == pytest.approx(.0144, abs=1e-7)
    assert ca.accel.accel.linear.y == pytest.approx(.0048, abs=1e-7)
    for ref, frame in [('rear_axle', 'base_link'), ('center_of_mass', 'center_of_mass')]:
        for kind in ('velocity', 'acceleration', 'yaw_rate'):
            m = g.outputs[ref][kind][-1]
            assert m.header.frame_id == frame and source_stamp(m) == g.now
            times = [source_stamp(m) for m in g.outputs[ref][kind]]
            assert all(b > a for a, b in zip(times, times[1:]))
        v = g.outputs[ref]['velocity'][-1]
        assert v.twist.covariance[0] > 0 and v.twist.covariance[7] > 0
        assert all(v.twist.covariance[k] >= 1e6 for k in [14, 21, 28, 35])
        a = g.outputs[ref]['acceleration'][-1]
        assert a.accel.covariance[0] > 0 and a.accel.covariance[7] > 0
        assert all(a.accel.covariance[k] >= 1e6 for k in [14, 21, 28, 35])
        r = g.outputs[ref]['yaw_rate'][-1]
        assert r.orientation_covariance[0] == -1 and r.linear_acceleration_covariance[0] == -1
        assert r.angular_velocity_covariance[0] >= 1e6 and r.angular_velocity_covariance[4] >= 1e6
        f = g.fields(ref)
        assert all(f[key] == 'true' for key in ['vx_valid','vy_valid','ax_valid','ay_valid','yaw_rate_valid'])
        assert f['cg_height_estimated'] == 'true' and f['geometry_calibrated'] == 'false'
        assert f['dynamic_accuracy_validated'] == 'false'
    assert rear['yaw_rate'][-1].angular_velocity.z == cg['yaw_rate'][-1].angular_velocity.z
    assert rear['yaw_rate'][-1].angular_velocity.z == pytest.approx(.4, abs=1e-8)
    subscriptions = {name for name, _ in g.node.get_subscriber_names_and_types_by_node('state_observer', '/')}
    assert subscriptions <= {'/livox/imu_bias_corrected','/fastlio2/lio_odom','/clock','/parameter_events'}
    publications = {name for name, _ in g.node.get_publisher_names_and_types_by_node('state_observer', '/')}
    assert not publications.intersection({'/tf','/tf_static','/drive','/vesc/command'})
    counts = {kind: len(rear[kind]) for kind in ('velocity','acceleration','yaw_rate')}
    g.send_lio(stamp=g.now-25_000_000)
    assert all(len(rear[kind]) == count for kind, count in counts.items())


def test_lio_dropout_continues_imu_prediction_with_warning_and_no_fake_zero(graph):
    g = graph; g.motion()
    g.tick(g.now+5_000_000); g.motion(count=115, with_lio=False)
    rear = g.outputs['rear_axle']; f = g.fields()
    assert f['vx_valid'] == f['vy_valid'] == f['ax_valid'] == f['ay_valid'] == 'true'
    assert f['velocity_reason'] == f['acceleration_reason'] == 'lio_stale'
    assert g.outputs['rear_axle']['status'][-1].status[0].level in (1, bytes([1]))
    assert f['yaw_rate_valid'] == 'true'
    assert source_stamp(rear['yaw_rate'][-1]) == g.now
    count = len(rear['velocity']); acc_count = len(rear['acceleration'])
    g.tick(g.now+5_000_000); g.send_imu()
    assert len(rear['velocity']) == count+1 and len(rear['acceleration']) == acc_count+1
    assert source_stamp(rear['velocity'][-1]) == source_stamp(rear['acceleration'][-1]) == g.now
    assert abs(rear['velocity'][-1].twist.twist.linear.x) > .1
    # A normal resumed LIO must not cause the old timeout-driven three-pose reset.
    g.tick(g.now+5_000_000); g.send_lio(); g.send_imu()
    assert g.fields()['velocity_reason'] == 'valid'
    assert g.fields()['velocity_reacquisitions'] == '0'
    # After IMU dropout, status invalidates every quantity without zero messages.
    yaw_count = len(rear['yaw_rate']); g.tick(g.now+60_000_000)
    assert g.fields()['yaw_rate_valid'] == 'false'
    assert len(rear['yaw_rate']) == yaw_count


def test_late_arriving_continuous_imu_is_published_with_source_age_warning(graph):
    g = graph; g.motion(); rear = g.outputs['rear_axle']
    previous = g.now
    counts = {kind: len(rear[kind]) for kind in ('velocity','acceleration','yaw_rate')}
    # Transport delay differs from a missing source interval: preserve every 5 ms sample.
    g.tick(previous+80_000_000)
    for i in range(1,5): g.send_imu(stamp=previous+i*5_000_000)
    for kind in counts:
        assert len(rear[kind]) == counts[kind]+4
        assert source_stamp(rear[kind][-1]) == previous+20_000_000
    f = g.fields()
    assert f['velocity_reason'] == f['acceleration_reason'] == f['yaw_rate_reason'] == 'imu_stale'
    assert g.outputs['rear_axle']['status'][-1].status[0].level in (1, bytes([1]))


def test_delayed_lio_within_history_is_not_rejected_by_age(graph):
    g = graph; g.motion(); old_anchor = int(g.fields()['velocity_lio_anchor_stamp_ns'])
    g.tick(g.now+5_000_000); g.motion(count=115, with_lio=False)
    delayed = g.now-550_000_000
    assert delayed > old_anchor
    g.send_lio(stamp=delayed)
    g.tick(g.now+5_000_000); g.send_imu()
    f = g.fields()
    assert int(f['velocity_lio_anchor_stamp_ns']) == delayed
    assert int(f['acceleration_lio_anchor_stamp_ns']) == delayed
    assert f['input.lio_age_warnings'] == '1'
    assert f['velocity_reacquisitions'] == '0'
    assert f['vx_valid'] == f['vy_valid'] == f['ax_valid'] == f['ay_valid'] == 'true'


def test_faults_force_independence_gap_and_epoch(graph):
    g = graph; g.motion(); rear = g.outputs['rear_axle']
    count = {kind: len(rear[kind]) for kind in ('velocity','acceleration','yaw_rate')}
    g.send_imu(); g.send_imu(stamp=g.now+1); g.send_imu(frame='bad')
    # Fresh source stamps are essential: otherwise duplicate admission would
    # hide whether NaN/overflow validation was exercised.
    g.tick(g.now+5_000_000); g.send_imu(rate=float('nan'))
    assert all(g.fields()[key] == 'false' for key in ['vx_valid','vy_valid','ax_valid','ay_valid','yaw_rate_valid'])
    g.tick(g.now+5_000_000); g.send_imu(rate=1e308)
    assert all(len(rear[kind]) == n for kind, n in count.items())
    g.tick(g.now+5_000_000); g.send_imu()
    assert all(g.fields()[key] == 'true' for key in ['vx_valid','vy_valid','ax_valid','ay_valid','yaw_rate_valid'])
    count = {kind: len(rear[kind]) for kind in ('velocity','acceleration','yaw_rate')}
    for i in range(1,15):
        g.tick(g.now+5_000_000); g.send_imu(available=False)
    assert len(rear['velocity']) == count['velocity'] and len(rear['acceleration']) == count['acceleration']
    assert len(rear['yaw_rate']) > count['yaw_rate']
    f = g.fields(); assert f['yaw_rate_valid'] == 'true' and f['vx_valid'] == f['ay_valid'] == 'false'
    counts = {kind: len(rear[kind]) for kind in ('velocity','acceleration','yaw_rate')}
    g.tick(g.now+40_000_000); g.send_lio(); g.send_imu()
    assert len(rear['velocity']) == counts['velocity']
    assert len(rear['yaw_rate']) == counts['yaw_rate']
    g.tick(g.now+5_000_000); g.motion(count=81)
    assert len(rear['velocity']) > counts['velocity']
    # A new ROS clock epoch may have lower absolute source stamps.
    count = len(rear['velocity']); g.epoch = 2_000_000_000
    g.tick(g.epoch); g.motion(count=81)
    assert len(rear['velocity']) > count and source_stamp(rear['velocity'][-1]) == g.now
    assert all(g.fields()[key] == 'true' for key in ['vx_valid','vy_valid','ax_valid','ay_valid','yaw_rate_valid'])


def test_dense_time_support_is_invalid_then_recovers(graph):
    g = graph; g.motion(); rear = g.outputs['rear_axle']; count = len(rear['acceleration'])
    g.tick(g.now+40_000_000); g.send_lio(); g.send_imu()
    for _ in range(3):
        g.tick(g.now+1_000); g.send_imu()
    f = g.fields()
    assert f['ax_valid'] == f['ay_valid'] == 'false'
    assert f['acceleration_reason'] == 'gyro_time_support'
    assert len(rear['acceleration']) == count
    for _ in range(5):
        g.tick(g.now+5_000_000); g.send_imu()
    assert len(rear['acceleration']) > count and g.fields()['ay_valid'] == 'true'


def test_launch_geometry_recording_and_opt_in(monkeypatch, tmp_path):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    root = Path(__file__).resolve().parents[1]; context = LaunchContext()

    def load(name):
        spec = importlib.util.spec_from_file_location('dual_'+name.replace('.','_'), root/'launch'/name)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(root))
        return module

    for name in ['vehicle.launch.py', 'race.launch.py']:
        module = load(name)
        defaults = {action.name: perform_substitutions(context, action.default_value)
                    for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument) and action.default_value is not None}
        assert defaults['state_observer'] == 'false'
    module = load('state_observer.launch.py'); monkeypatch.setattr(module, 'Node', lambda **kw: kw)
    context.launch_configurations.update(config=str(root/'params/state_observer.yaml'),
        geometry_config=str(root/'params/rear_axle_geometry.yaml'),
        cg_geometry_config=str(root/'params/vehicle_geometry_measurements.yaml'), use_sim_time='false')
    built = module.build_node(context)[0]
    assert built['executable'] == 'state_observer'
    assert built['parameters'][1]['imu_to_cg'] == [-.09,-.03,-.03]
    assert built['parameters'][1]['cg_height_estimated'] is True
    from aims_racer_system.launch_support import recording_actions
    context.launch_configurations.update(record='true', session_directory=str(tmp_path/'session'))
    topics = {perform_substitutions(context, item) for item in recording_actions(context,root)[0].cmd}
    assert {f'/state_estimation/{ref}/{kind}' for ref in ['rear_axle','center_of_mass']
            for kind in ['velocity','acceleration','yaw_rate','status']} <= topics
    assert (tmp_path/'session/params/state_observer.yaml').exists()
    vehicle = load('vehicle.launch.py')
    monkeypatch.setattr(vehicle, 'Node', lambda **kw: kw)
    monkeypatch.setattr(vehicle, 'include', lambda pkg,name,arguments=None: {'include':name})
    context.launch_configurations.update(mapping='false', vesc_config='/v', lio_config='/l', mid360_config='/m')
    assert not any(n.get('include')=='state_observer.launch.py' for n in vehicle.build_vehicle(context))
    context.launch_configurations['state_observer'] = 'true'
    assert sum(n.get('include')=='state_observer.launch.py' for n in vehicle.build_vehicle(context)) == 1
    race = load('race.launch.py'); seen=[]
    monkeypatch.setattr(race,'include',lambda pkg,name,arguments=None:seen.append((name,arguments)) or object())
    map_file=tmp_path/'map';map_file.write_text('map'); bundle=tmp_path/'bundle';bundle.mkdir()
    context.launch_configurations.update(map_file=str(map_file),artifact_directory=str(bundle),record='false',
        auto_start='false',repeat_laps='false',initial_pose='')
    race.build_race(context)
    assert next(args for name,args in seen if name=='vehicle.launch.py')['state_observer']=='true'


def test_selected_state_and_geometry_files_are_recorded(tmp_path):
    from launch import LaunchContext
    from aims_racer_system.launch_support import recording_actions
    root = Path(__file__).resolve().parents[1]; context = LaunchContext()
    selected = {}
    for key in ['state_observer_config','state_geometry_config','cg_geometry_config']:
        path = tmp_path/(key+'.yaml'); path.write_text('selected: '+key+'\n'); selected[key]=str(path)
    context.launch_configurations.update(record='true',session_directory=str(tmp_path/'selected-session'),**selected)
    recording_actions(context,root)
    for key,label in [('state_observer_config','state_observer'),('state_geometry_config','state_geometry'),('cg_geometry_config','cg_geometry')]:
        assert (tmp_path/'selected-session/params'/(label+'_selected.yaml')).read_text() == Path(selected[key]).read_text()
