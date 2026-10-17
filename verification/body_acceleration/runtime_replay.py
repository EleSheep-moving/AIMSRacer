#!/usr/bin/env python3
"""One-timespeed recorded receipt replay into an isolated standalone ROS observer.
Software receipt/scheduling/publication boundary only; no hardware latency claim.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time
import numpy as np
os.environ.update(ROS_DOMAIN_ID='193', ROS_LOCALHOST_ONLY='1')
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'output', 'executable'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--start-source-offset', type=float, default=41.926416916)
    p.add_argument('--origin-ns', type=int, default=1792002365474876930)
    p.add_argument('--duration', type=float, default=20.)
    p.add_argument('--config', type=Path)
    for name, default in [('imu-frame', 'livox_frame'), ('world-frame', 'odom'), ('body-frame', 'base_link')]:
        p.add_argument('--' + name, default=default)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    with np.load(a.input, allow_pickle=False) as archive:
        z = {k: archive[k] for k in ('corrected', 'corrected_ts', 'corrected_cov', 'lio', 'lio_ts', 'lio_cov')}
    begin = a.origin_ns + round(a.start_source_offset * 1e9); end = begin + round(a.duration * 1e9)
    events = sorted((int(r), int(s), kind, i) for kind in ('corrected', 'lio')
                    for i, (r, s) in enumerate(z[kind + '_ts']) if begin <= r < end)
    if not events or len({e[0] for e in events}) != len(events):
        raise ValueError('empty window or ambiguous receipt ties')
    rclpy.init(); node = rclpy.create_node('body_acceleration_receipt_replay')
    outputs, statuses, sent, scheduling_ms = [], [], [], []
    def receive_output(m):
        outputs.append([ns(m.header.stamp), node.get_clock().now().nanoseconds,
                        m.accel.accel.linear.x, m.accel.covariance[0]])
    def receive_status(m):
        statuses.append(dict(receipt_ros_ns=node.get_clock().now().nanoseconds,
            receipt_steady_ns=time.monotonic_ns(), header_stamp_ns=ns(m.header.stamp),
            status=[dict(name=s.name, level=s.level[0] if isinstance(s.level, bytes) else int(s.level), message=s.message,
                         values={v.key: v.value for v in s.values}) for s in m.status]))
    subscriptions = [node.create_subscription(AccelWithCovarianceStamped,
        '/state_estimation/body_acceleration', receive_output, 1000),
        node.create_subscription(DiagnosticArray, '/state_estimation/body_acceleration/status', receive_status, 100)]
    imu = node.create_publisher(Imu, '/livox/imu_bias_corrected', qos_profile_sensor_data)
    lio = node.create_publisher(Odometry, '/rear_axle/lio_odom', qos_profile_sensor_data)
    command = [str(a.executable), '--ros-args']
    if a.config: command += ['--params-file', str(a.config)]
    for key, value in [('use_sim_time', 'false'), ('imu_frame', a.imu_frame),
                       ('world_frame', a.world_frame), ('body_frame', a.body_frame)]:
        command += ['-p', f'{key}:={value}']
    log = (a.output / 'node.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    def pump():
        if process.poll() is not None: raise RuntimeError('observer exited; see node.log')
        rclpy.spin_once(node, timeout_sec=0)
    def until(deadline):
        while time.monotonic_ns() < deadline:
            pump(); time.sleep(min(.0005, max(0., (deadline - time.monotonic_ns()) * 1e-9)))
    try:
        deadline = time.monotonic() + 10.
        while min(imu.get_subscription_count(), lio.get_subscription_count()) < 1:
            if time.monotonic() > deadline: raise RuntimeError('DDS discovery timed out')
            pump(); time.sleep(.005)
        until(time.monotonic_ns() + 1_000_000_000)
        base_ros = node.get_clock().now().nanoseconds; base_steady = time.monotonic_ns(); first = events[0][0]
        for receipt, source, kind, index in events:
            due = base_steady + receipt - first; until(due)
            stamp = base_ros + source - first; values = z[kind][index]; covariance = z[kind + '_cov'][index]
            if kind == 'corrected':
                m = Imu(); m.header.frame_id = a.imu_frame
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, values[:3])
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, values[3:6])
                m.linear_acceleration_covariance = covariance[:9].tolist()
                m.angular_velocity_covariance = covariance[9:18].tolist()
                m.orientation_covariance = covariance[18:27].tolist()
            else:
                m = Odometry(); m.header.frame_id = a.world_frame; m.child_frame_id = a.body_frame
                m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = map(float, values[:3])
                m.pose.pose.position.x, m.pose.pose.position.y = map(float, values[3:5])
                q = m.pose.pose.orientation; q.x, q.y, q.z, q.w = map(float, values[5:9])
                m.pose.covariance = covariance[:36].tolist(); m.twist.covariance = covariance[36:72].tolist()
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
            scheduling_ms.append((time.monotonic_ns() - due) * 1e-6)
            (imu if kind == 'corrected' else lio).publish(m); sent.append([receipt, source, stamp, kind]); pump()
        until(time.monotonic_ns() + 250_000_000)
        subs = {t for t, _ in node.get_subscriber_names_and_types_by_node('body_acceleration', '/')}
        pubs = {t for t, _ in node.get_publisher_names_and_types_by_node('body_acceleration', '/')}
        for snapshot in statuses:
            snapshot['phase'] = ('warmup' if snapshot['receipt_steady_ns'] < base_steady else
                'replay' if snapshot['receipt_steady_ns'] <= base_steady + events[-1][0] - first else 'drain')
        fields = [s['status'][0]['values'] for s in statuses if s['status']]
        replay_fields = [s['status'][0]['values'] for s in statuses if s['status'] and s['phase'] == 'replay']
        source_stamps = [int(o[0]) for o in outputs]; imu_stamps = {s[2] for s in sent if s[3] == 'corrected'}
        checks = dict(enough_outputs=len(outputs) > a.duration * 100,
            monotonic_sources=all(y > x for x, y in zip(source_stamps, source_stamps[1:])),
            exact_source_subset=set(source_stamps) <= imu_stamps,
            finite_ax_variance=all(np.isfinite(o[2:]).all() and o[3] >= 0 for o in outputs),
            independent_graph=subs <= {'/livox/imu_bias_corrected', '/rear_axle/lio_odom', '/clock', '/parameter_events'}
                and not pubs.intersection({'/tf', '/tf_static', '/drive', '/vesc/command', '/odometry/filtered'}))
        paths = [a.input, a.executable, Path(__file__)] + ([a.config] if a.config else [])
        report = dict(boundary=__doc__, args={k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()},
            hashes={str(f): sha(f) for f in paths}, platform=platform.platform(), command=command,
            base_ros_ns=base_ros, base_steady_ns=base_steady, first_receipt_ns=first, ros_domain_id=193, checks=checks,
            counts=dict(inputs=dict(Counter(s[3] for s in sent)), outputs=len(outputs), status=len(statuses)),
            status_valid_counts=dict(Counter(f.get('x_valid') for f in fields)),
            status_reason_counts=dict(Counter(f.get('reason') for f in fields)),
            replay_status_valid_counts=dict(Counter(f.get('x_valid') for f in replay_fields)),
            replay_status_reason_counts=dict(Counter(f.get('reason') for f in replay_fields)), subscriptions=sorted(subs), publishers=sorted(pubs),
            scheduling_ms=dict(p50=float(np.percentile(scheduling_ms, 50)), p99=float(np.percentile(scheduling_ms, 99)), maximum=max(scheduling_ms)),
            callback_p99_ms={k: dict(maximum=max(float(f[k]) for f in fields), final=float(fields[-1][k]))
                for k in ('callback.imu.p99_ms', 'callback.lio.p99_ms')},
            callback_boundary='max/final reported p99 over rolling <=512 callbacks; not whole-run p99 or physical delay', statuses=statuses)
        for kind in ('corrected', 'lio'):
            stamps = [s[1] for s in sent if s[3] == kind]
            gaps = np.diff(stamps) * 1e-9
            report[kind + '_source_gaps'] = dict(maximum_s=float(max(gaps)),
                over_deadline=int(sum(gaps > (.03 if kind == 'corrected' else .5))))
        np.savez_compressed(a.output / 'outputs.npz', source_ns=np.array(source_stamps, dtype=np.int64),
            receipt_ros_ns=np.array([o[1] for o in outputs], dtype=np.int64), ax=[o[2] for o in outputs], variance=[o[3] for o in outputs])
        (a.output / 'metrics.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(checks=checks, counts=report['counts'], callback_p99_ms=report['callback_p99_ms'])))
        return 0 if all(checks.values()) else 2
    finally:
        process.terminate(); process.wait(timeout=5); log.close()
        for sub in subscriptions: node.destroy_subscription(sub)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
