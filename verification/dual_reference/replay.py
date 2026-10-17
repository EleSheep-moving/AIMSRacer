#!/usr/bin/env python3
"""1x recorded-receipt ROS replay of dual-reference outputs; no vehicle graph.

Consumes the existing velocity extraction (corrected IMU + raw LIO pose).
Preserves source/receipt age with a common epoch shift. Verifies transport,
reference-point identities and finite outputs, not independent dynamic accuracy.
"""
import argparse
import csv
import hashlib
import json
import os
import signal
import shutil
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def observer_process(launch_pid, executable):
    """Resolve the actual child ELF, rather than merely trusting package lookup."""
    parents = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = dict(line.split(':', 1) for line in (path/'status').read_text().splitlines() if ':' in line)
            parents[int(path.name)] = int(fields['PPid'])
        except (OSError, ValueError, KeyError):
            continue
    children = {launch_pid}
    for _ in range(len(parents)):
        added = {pid for pid, parent in parents.items() if parent in children}-children
        if not added:
            break
        children |= added
    matches = []
    for pid in children-{launch_pid}:
        try:
            if Path(os.readlink(f'/proc/{pid}/exe')).resolve() == executable.resolve():
                matches.append(pid)
        except OSError:
            continue
    assert len(matches) == 1, ('expected one launched observer ELF', matches)
    return matches[0]


def require_alive(process, pid):
    try:
        status = Path(f'/proc/{pid}/status').read_text()
    except OSError:
        raise RuntimeError('observer stopped during replay') from None
    if process.poll() is not None or any(line.startswith('State:') and 'Z' in line for line in status.splitlines()):
        raise RuntimeError('observer stopped during replay')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path, help='folder containing input.npz and metadata.json')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--start-source-ns', type=int, help='optional source-time lower bound; all event ages retained')
    parser.add_argument('--duration', type=float, help='receipt-time seconds; omit for full selected recording')
    args = parser.parse_args()
    script_sha256 = digest(Path(__file__))
    args.output.mkdir(parents=True, exist_ok=False)
    metadata = json.loads((args.input/'metadata.json').read_text())
    assert digest(args.input/'input.npz') == metadata['npz_sha256']
    with np.load(args.input/'input.npz') as archive:
        data = {key: archive[key].copy() for key in archive.files}
    events = [tuple(map(int, row)) for row in data['receipt_order'] if int(row[2]) in (1, 2)]
    if args.start_source_ns is not None:
        events = [event for event in events if int(data['imu_ts' if event[2] == 1 else 'lio_ts'][event[3], 1]) >= args.start_source_ns]
    if args.duration is not None:
        events = [event for event in events if event[0] <= events[0][0]+int(args.duration*1e9)]
    assert events
    prefix = Path(get_package_prefix('aims_racer_system'))
    share = Path(get_package_share_directory('aims_racer_system'))
    executable = prefix/'lib/aims_racer_system/state_observer'
    installed_files = [executable, share/'launch/state_observer.launch.py',
        share/'params/state_observer.yaml', share/'params/rear_axle_geometry.yaml',
        share/'params/vehicle_geometry_measurements.yaml']
    implementation = dict(install_prefix=str(prefix), package_share=str(share),
        file_sha256={str(path): digest(path) for path in installed_files})
    # Preload before the clock starts. NPZ decompression in the timed path would
    # change source ages and invalidate the transport test.
    input_stamps = set(map(int, data['imu_ts'][:, 1]))
    gyro_by_stamp = {int(stamp): row[3:6] for stamp, row in zip(data['imu_ts'][:, 1], data['imu'])}
    outputs = defaultdict(list)
    status = defaultdict(list)
    rclpy.init(); node = rclpy.create_node('dual_reference_receipt_replay')
    imu_pub = node.create_publisher(Imu, '/livox/imu_bias_corrected', 256)
    lio_pub = node.create_publisher(Odometry, '/fastlio2/lio_odom', 256)
    offset = 0
    stream = (args.output/'outputs.csv').open('w')
    writer = csv.writer(stream)
    writer.writerow(['reference', 'kind', 'shifted_source_ns', 'original_source_ns', 'x', 'y', 'variance_x', 'variance_y'])

    def consume(reference, kind, message):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        original = stamp-offset
        assert original in input_stamps
        assert message.header.frame_id == ('base_link' if reference == 'rear_axle' else 'center_of_mass')
        if kind == 'velocity':
            x, y = message.twist.twist.linear.x, message.twist.twist.linear.y
            cx, cy = message.twist.covariance[0], message.twist.covariance[7]
            assert all(message.twist.covariance[k] >= 1e6 for k in [14,21,28,35])
        elif kind == 'acceleration':
            x, y = message.accel.accel.linear.x, message.accel.accel.linear.y
            cx, cy = message.accel.covariance[0], message.accel.covariance[7]
            assert all(message.accel.covariance[k] >= 1e6 for k in [14,21,28,35])
        else:
            x, y = message.angular_velocity.z, 0.
            cx, cy = message.angular_velocity_covariance[8], 1e6
            assert message.orientation_covariance[0] == message.linear_acceleration_covariance[0] == -1
        assert np.isfinite([x,y,cx,cy]).all() and cx > 0 and cy > 0
        key = (reference, kind)
        assert not outputs[key] or stamp > outputs[key][-1][0]
        outputs[key].append((stamp, x, y))
        writer.writerow([reference,kind,stamp,original,x,y,cx,cy])

    subscriptions = []
    for reference in ['rear_axle','center_of_mass']:
        for kind, msgtype in [('velocity',TwistWithCovarianceStamped),('acceleration',AccelWithCovarianceStamped),('yaw_rate',Imu)]:
            subscriptions.append(node.create_subscription(msgtype, f'/state_estimation/{reference}/{kind}',
                lambda msg, ref=reference, key=kind: consume(ref,key,msg), 8192))
        subscriptions.append(node.create_subscription(DiagnosticArray,f'/state_estimation/{reference}/status',
            lambda msg, ref=reference: status[ref].append({item.key:item.value for item in msg.status[0].values}),256))
    log = (args.output/'node.log').open('w')
    process = subprocess.Popen(['ros2','launch','aims_racer_system','state_observer.launch.py'],
                               stdout=log,stderr=subprocess.STDOUT)
    sent = defaultdict(int); lateness = []
    try:
        deadline = time.monotonic()+15
        while not (imu_pub.get_subscription_count() and lio_pub.get_subscription_count()):
            assert process.poll() is None
            assert time.monotonic() < deadline, 'observer graph did not become ready'
            rclpy.spin_once(node,timeout_sec=.02)
        # Ensure numeric subscribers have discovered publishers before sending.
        while not all(node.count_publishers(f'/state_estimation/{ref}/{kind}') for ref in ['rear_axle','center_of_mass']
                      for kind in ['velocity','acceleration','yaw_rate']):
            assert time.monotonic() < deadline
            rclpy.spin_once(node,timeout_sec=.02)
        pid = observer_process(process.pid, executable)
        require_alive(process, pid)
        implementation['observer_pid'] = pid
        command = Path(f'/proc/{pid}/cmdline').read_bytes().decode().rstrip('\0').split('\0')
        implementation['actual_command'] = command
        implementation['actual_executable_sha256'] = digest(Path(f'/proc/{pid}/exe'))
        assert implementation['actual_executable_sha256'] == implementation['file_sha256'][str(executable)]
        parameters = args.output/'resolved-parameters'; parameters.mkdir()
        implementation['resolved_parameter_files'] = []
        for index, item in enumerate(command[:-1]):
            if item == '--params-file':
                path = Path(command[index+1]); copy = parameters/f'{index}.yaml'
                shutil.copy2(path, copy)
                implementation['resolved_parameter_files'].append(dict(original=str(path), archived=str(copy),sha256=digest(copy)))
        dumped = subprocess.run(['ros2','param','dump','/state_observer'],capture_output=True,text=True,timeout=10,check=True)
        (parameters/'effective.yaml').write_text(dumped.stdout)
        implementation['effective_parameters_sha256'] = digest(parameters/'effective.yaml')
        implementation['effective_parameters_file'] = str(parameters/'effective.yaml')
        offset = node.get_clock().now().nanoseconds+300_000_000-events[0][0]
        steady_start = time.monotonic_ns()+300_000_000
        for receipt, _, kind, index in events:
            require_alive(process, pid)
            due = steady_start+receipt-events[0][0]
            while time.monotonic_ns() < due:
                require_alive(process, pid)
                rclpy.spin_once(node,timeout_sec=max(0.,min(.001,(due-time.monotonic_ns())*1e-9)))
            lateness.append(max(0.,(time.monotonic_ns()-due)*1e-6))
            name = 'imu' if kind == 1 else 'lio'
            values = data[name][index]; stamp = int(data[name+'_ts'][index,1])+offset
            if kind == 1:
                message = Imu(); message.header.frame_id = 'livox_frame'
                message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z = map(float,values[:3])
                message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z = map(float,values[3:6])
            else:
                message = Odometry(); message.header.frame_id = 'odom'; message.child_frame_id = 'livox_frame'
                message.pose.pose.position.x,message.pose.pose.position.y,message.pose.pose.position.z = map(float,values[:3])
                message.pose.pose.orientation.x,message.pose.pose.orientation.y,message.pose.pose.orientation.z,message.pose.pose.orientation.w = map(float,values[3:7])
            message.header.stamp.sec,message.header.stamp.nanosec = divmod(stamp,10**9)
            (imu_pub if kind == 1 else lio_pub).publish(message)
            sent[name] += 1
            rclpy.spin_once(node,timeout_sec=0.)
        until = time.monotonic()+1.
        while time.monotonic() < until:
            require_alive(process, pid)
            rclpy.spin_once(node,timeout_sec=.005)
        require_alive(process, pid)
        assert 'state_observer' in node.get_node_names(), 'observer graph disappeared'
        graph = dict(subscriptions=[name for name,_ in node.get_subscriber_names_and_types_by_node('state_observer','/')],
                     publishers=[name for name,_ in node.get_publisher_names_and_types_by_node('state_observer','/')])
        assert {'/livox/imu_bias_corrected','/fastlio2/lio_odom'} <= set(graph['subscriptions'])
        assert set(graph['subscriptions']) <= {'/livox/imu_bias_corrected','/fastlio2/lio_odom','/parameter_events'}
        assert {f'/state_estimation/{ref}/{kind}' for ref in ['rear_axle','center_of_mass']
                for kind in ['velocity','acceleration','yaw_rate','status']} <= set(graph['publishers'])
        assert not set(graph['publishers']).intersection({'/tf','/tf_static','/drive','/vesc/command'})
        checks = {}
        for kind in ['velocity','acceleration','yaw_rate']:
            rear, cg = outputs['rear_axle',kind], outputs['center_of_mass',kind]
            assert rear and len(rear) == len(cg), (kind,len(rear),len(cg))
            assert [row[0] for row in rear] == [row[0] for row in cg], kind
            if kind == 'velocity':
                # Difference of the same velocity state at the two points. This
                # identity does not establish the underlying VY is accurate.
                expected = np.array([np.cross(gyro_by_stamp[row[0]-offset], [.21,-.03,0.])[:2] for row in rear])
                difference = np.array([[c[1]-r[1],c[2]-r[2]] for r,c in zip(rear,cg)])
                maximum = float(abs(difference-expected).max()); assert maximum < 1e-10
                checks['velocity_reference_identity_max_abs_mps'] = maximum
            if kind == 'yaw_rate':
                maximum = max(abs(r[1]-c[1]) for r,c in zip(rear,cg)); assert maximum == 0.
                checks['yaw_reference_identity_max_abs_radps'] = maximum
        stream.flush()
        result = dict(input_sha256=metadata['npz_sha256'],input_source=metadata['source'],
            full_input_events=len(data['receipt_order']),selected_input_events=len(events),sent=dict(sent),
            receipt_duration_s=(events[-1][0]-events[0][0])*1e-9,common_epoch_shift_ns=offset,
            output_counts={f'{ref}/{kind}':len(values) for (ref,kind),values in outputs.items()},
            identity_checks=checks,graph=graph,final_status={ref: values[-1] for ref,values in status.items()},
            scheduling_lateness_ms=dict(p95=float(np.percentile(lateness,95)),maximum=max(lateness)),
            output_sha256=digest(args.output/'outputs.csv'),script_sha256=script_sha256,
            implementation=implementation,
            independent_truth=False,scope='selected 1x corrected IMU/raw LIO playback only; no vehicle/load/dynamic accuracy acceptance')
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ['graph','final_status']},indent=2))
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate();process.wait(timeout=5)
        stream.close();log.close();node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
