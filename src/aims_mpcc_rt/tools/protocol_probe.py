#!/usr/bin/env python3
"""Functional ROS protocol evidence using synthetic inputs in domain 225 only.

The selector echo and lagged bicycle plant are synthetic. No RC selector, VESC,
NDT estimator, physical map, or hardware is exercised by this probe.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped as Drive
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from aims_mpcc.integration import DRIVING_TOPICS, topic_remap_arguments
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath
from ament_index_python.packages import get_package_prefix


PREFIX = '/mpcc_protocol'


def map_identity(path):
    if path.frame_id!='map':return 'b'*64
    value=path.metadata.get('map_sha256','')
    if len(value)!=64 or any(c not in '0123456789abcdefABCDEF' for c in value):
        raise ValueError('map reference requires a valid map_sha256')
    return value


def atomic_health_record(*,epoch,health_sequence,anchor_sequence,anchor_ns,map_hash,kind):
    values=dict(protocol_version='1',epoch=epoch,health_sequence=str(health_sequence),
        anchor_sequence=str(anchor_sequence),last_anchor_stamp_ns=str(anchor_ns),
        ready='false' if kind=='not_ready' else 'true',state='synthetic',map_sha256=map_hash,
        alignment_valid='true',alignment_epoch=epoch,alignment_anchor_sequence=str(anchor_sequence),
        alignment_stamp_ns=str(anchor_ns),map_odom_x='0',map_odom_y='0',map_odom_z='0',
        map_odom_qx='0',map_odom_qy='0',map_odom_qz='0',map_odom_qw='1')
    if kind=='malformed':values.pop('last_anchor_stamp_ns')
    return values


def base_link_position(x,y,yaw,rear_offset):
    return x+rear_offset*math.cos(yaw),y+rear_offset*math.sin(yaw)


def source_timeout_s(steady_now,ros_now_ns,last_good_ns):
    return steady_now+(last_good_ns-ros_now_ns)*1e-9+.1


def reason_matches(reason,expected):
    options=(expected,) if isinstance(expected,str) else expected
    return any(value.lower() in reason.lower() for value in options)


class Probe(Node):
    def __init__(self, bundle):
        super().__init__('mpcc_protocol_probe')
        self.config = load_config(bundle / 'input_config.yaml')
        path = ReferencePath.load(bundle / 'input_reference')
        self.reference_frame=path.frame_id
        point = path.at(0.)
        self.x, self.y, self.yaw = point['x'], point['y'], point['yaw']
        self.speed = self.steer = 0.
        self.target = Drive()
        self.autonomy = True
        self.odom_fault = None
        self.health_kind = 'ready'
        self.epoch = 'synthetic_epoch_1'
        self.health_seq = self.anchor_seq = 0
        self.anchor_ns = 0
        self.hold_anchor = False
        self.expected_map_hash=map_identity(path)
        self.map_hash=self.expected_map_hash
        self.status = {}
        self.commands = []
        self.events = []
        self.last = time.monotonic()
        self.last_odom_ns = self.last_good_odom_ns = 0
        self.last_odom = None
        self.fresh_input_updates = 0
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom = self.create_publisher(Odometry, PREFIX + '/odometry/filtered', 10)
        self.mode = self.create_publisher(Bool, PREFIX + '/control/autonomy_speed_enabled', 10)
        self.ack = self.create_publisher(Drive, PREFIX + '/ackermann_cmd', 10)
        self.identity = self.create_publisher(String, PREFIX + '/localization/map_sha256', latch)
        self.health = self.create_publisher(DiagnosticArray, PREFIX + '/localization/status', 10)
        self.tf = self.create_publisher(TFMessage, PREFIX + '/tf_static', latch)
        self.create_subscription(Drive, PREFIX + '/drive', self.observe_drive, 10)
        self.create_subscription(DiagnosticArray, PREFIX + '/mpcc/status', self.observe_status, 10)
        self.client = self.create_client(SetBool, PREFIX + '/mpcc/enable')
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.
        self.tf.publish(TFMessage(transforms=[transform]))
        self.timer = self.create_timer(.02, self.tick)

    def observe_drive(self, message):
        self.target = message
        self.commands.append((time.monotonic(), message.drive.speed,
                              message.drive.acceleration, message.drive.steering_angle))

    def observe_status(self, message):
        for status in message.status:
            if status.name == 'aims_mpcc':
                values = {kv.key: json.loads(kv.value) for kv in status.values}
                self.status = values
                self.events.append(dict(time=time.monotonic(), status=values))

    def tick(self):
        now = time.monotonic()
        dt = min(.05, now - self.last)
        self.last = now
        target_speed = self.target.drive.speed if self.autonomy else 0.
        self.speed += (target_speed - self.speed) * (1 - math.exp(-dt / .2))
        self.steer += (self.target.drive.steering_angle - self.steer) * (1 - math.exp(-dt / .15))
        rate = self.speed * math.tan(self.steer) / self.config.wheelbase
        self.x += self.speed * math.cos(self.yaw + .5 * rate * dt) * dt
        self.y += self.speed * math.sin(self.yaw + .5 * rate * dt) * dt
        self.yaw += rate * dt
        ns = self.get_clock().now().nanoseconds
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        stamp = ns
        if self.odom_fault == 'stale': stamp -= 200_000_000
        if self.odom_fault == 'future': stamp += 200_000_000
        if self.odom_fault == 'backwards': stamp = self.last_odom_ns - 1_000_000
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 10**9)
        self.last_odom_ns = stamp
        if self.odom_fault is None:self.last_good_odom_ns=stamp
        odom.pose.pose.position.x,odom.pose.pose.position.y=base_link_position(
            self.x,self.y,self.yaw,self.config.rear_offset)
        if self.odom_fault != 'quaternion':
            scale = 2. if self.odom_fault == 'nonunit_quaternion' else 1.
            odom.pose.pose.orientation.z = scale * math.sin(self.yaw / 2)
            odom.pose.pose.orientation.w = scale * math.cos(self.yaw / 2)
        else:
            odom.pose.pose.orientation.w = 0.
        odom.twist.twist.linear.x = -.2 if self.odom_fault == 'reverse' else self.speed
        self.odom.publish(odom)
        self.last_odom = odom
        self.mode.publish(Bool(data=self.autonomy))
        forwarded = self.target if self.autonomy else Drive()
        self.ack.publish(forwarded)
        self.identity.publish(String(data=self.map_hash))
        self.health_seq += 1
        if not self.hold_anchor and (not self.anchor_ns or ns - self.anchor_ns > 200_000_000):
            self.anchor_seq += 1
            self.anchor_ns = ns
        values=atomic_health_record(epoch=self.epoch,health_sequence=self.health_seq,
            anchor_sequence=self.anchor_seq,anchor_ns=self.anchor_ns,map_hash=self.map_hash,kind=self.health_kind)
        status = DiagnosticStatus(name='aims_racer_system/localization',
                                  values=[KeyValue(key=k, value=v) for k, v in values.items()])
        self.health.publish(DiagnosticArray(status=[status]))

    def refresh_inputs(self):
        """Stress receive/output concurrency with unchanged valid authority.

        Keep the plant and localization protocol at their normal rates. Only
        republish its latest valid pose with its original source timestamp and
        duplicate the same authority; neither input withdraws authorization.
        """
        if self.last_odom is None:
            return
        self.odom.publish(self.last_odom)
        for _ in range(8):
            self.mode.publish(Bool(data=self.autonomy))
        self.fresh_input_updates += 1

    def spin_until(self, predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.005)
            if predicate(): return True
        return False

    def settle(self, duration=.15):
        end = time.monotonic() + duration
        self.spin_until(lambda: time.monotonic() >= end, duration + .1)

    def enable(self, flag=True):
        assert self.spin_until(self.client.service_is_ready, 10.), 'enable service unavailable'
        future = self.client.call_async(SetBool.Request(data=flag))
        assert self.spin_until(future.done, 3.), 'enable service timed out'
        response = future.result()
        return dict(success=response.success, message=response.message)


def lifecycle_pass(commands,statuses,baseline,ended,duration):
    """Positive publication and repeated accepted activations must persist."""
    if not commands or not statuses:return False
    gaps=[after[0]-before[0] for before,after in zip(commands,commands[1:])]
    return bool(commands[-1][0]-commands[0][0]>=duration-.05
        and ended-commands[-1][0]<=.1 and len(commands)>=.95*50*duration
        and all(row[1]>0. for row in commands) and max(gaps,default=math.inf)<=.1
        and all(status.get('status')=='RUNNING' for status in statuses)
        and statuses[-1].get('activated',0)-baseline.get('activated',0)>=2
        and all(status.get(key,0)==baseline.get(key,0) for status in statuses for key in ('failed','rejected')))


def run(args):
    if os.environ.get('ROS_DOMAIN_ID') != '225' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('probe requires ROS_DOMAIN_ID=225 and ROS_LOCALHOST_ONLY=1')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    bundle = Path(args.bundle).resolve()
    if args.scenario=='health' and ReferencePath.load(bundle/'input_reference').frame_id!='map':
        raise ValueError('health scenario requires a map reference; odom reference cannot test localization authorization')
    remaps = topic_remap_arguments(PREFIX)
    command = ['ros2', 'run', 'aims_mpcc_rt', 'mpcc_rt_node', '--ros-args',
               '-p', 'artifact_directory:=' + str(bundle), '-p', 'simulation:=true',
               '-p', 'repeat_laps:=true','-p',f'solve_frequency:={args.frequency}',
               '-p',f'handover_delay:={args.handover_delay}','-p',f'solver_timeout:={args.budget}',
               '-p', 'log_directory:=' + str(out / 'controller'), *remaps]
    command=[sys.executable,str(Path(__file__).with_name('runtime_supervisor.py')),
             '--status-topic',PREFIX+'/mpcc/status','--report',str(out/'supervisor.json'),
             '--restart-limit','1' if args.scenario=='watchdog' else '0','--',*command]
    rclpy.init(args=[])
    probe = Probe(bundle)
    binary = Path(get_package_prefix('aims_mpcc_rt')) / 'lib/aims_mpcc_rt/mpcc_rt_node'
    report = dict(scope='synthetic map identity, TF, health, selector echo and lagged bicycle plant; no hardware or NDT',
                  scenario=args.scenario, domain=225, prefix=PREFIX, command=command, checks=[],
                  profile=dict(frequency=args.frequency,handover_delay=args.handover_delay,budget=args.budget,seconds=args.seconds),
                  controller_binary=str(binary), controller_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  bundle_fingerprint=json.loads((bundle / 'manifest.json').read_text())['fingerprint'])
    log = (out / 'controller.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    extra = None

    def check(name, condition, **evidence):
        report['checks'].append(dict(name=name, passed=bool(condition), **evidence))
        assert condition, name + ': ' + json.dumps(evidence)

    def ready():
        check('worker_and_service_ready', probe.spin_until(lambda: probe.status.get('worker_ready') is True
              and probe.client.service_is_ready(), 15.), status=probe.status)
        probe.settle(.2)

    def activate():
        check('stationary_before_enable', probe.spin_until(lambda: probe.speed < .03, 4.), speed=probe.speed)
        probe.settle(.15)
        reply = probe.enable()
        check('enable_with_valid_inputs', reply['success'], reply=reply)
        check('running_status_after_enable', probe.spin_until(lambda: probe.status.get('status') == 'RUNNING', .3),
              status=probe.status)
        check('positive_command_after_enable', probe.spin_until(lambda: len(probe.commands) >= 2
              and all(row[1] > 0. for row in probe.commands[-2:])
              and probe.status.get('status') == 'RUNNING', 6.),
              command_speed=probe.target.drive.speed, status=probe.status)

    def fault(name, inject, expected, maximum=.1,actionable_s=None):
        begin = time.monotonic()
        actionable_s=begin if actionable_s is None else actionable_s
        check(name + '_positive_before_injection', bool(probe.commands)
              and probe.commands[-1][1] > 0. and begin - probe.commands[-1][0] <= .1
              and probe.status.get('status') == 'RUNNING',
              command_speed=probe.target.drive.speed, status=probe.status)
        inject()
        observed = probe.spin_until(lambda: probe.status.get('status') == 'FAULT'
                                   and probe.commands[-1][0] >= begin and probe.commands[-1][1] == 0., max(0.,actionable_s-begin)+maximum+.2)
        fault_times = [event['status'].get('fault_steady_s',event['time']) for event in probe.events if event['time'] >= begin
                       and event['status'].get('status') == 'FAULT']
        fault_latency = min(fault_times) - actionable_s if fault_times else None
        zero_times = [t for t, speed, _, _ in probe.commands if t >= begin and speed == 0.]
        latency = min(zero_times) - actionable_s if zero_times else None
        check(name + '_fault_detection_within_bound', observed and fault_latency is not None
              and 0. <= fault_latency <= maximum and reason_matches(probe.status.get('reason',''),expected),
              status=probe.status, fault_detection_latency_s=fault_latency, maximum_s=maximum,
              injected_s=begin,actionability_s=actionable_s)
        check(name + '_positive_to_zero_within_bound', latency is not None and 0. <= latency <= maximum,
              zero_command_latency_s=latency, maximum_s=maximum)
        probe.settle(.3)
        stopped_commands = [row for row in probe.commands if row[0] >= min(zero_times)]
        check(name + '_sustained_zero', len(stopped_commands) >= 10
              and all(row[1] == 0. for row in stopped_commands)
              and time.monotonic() - stopped_commands[-1][0] <= .1
              and stopped_commands[-1][0] - stopped_commands[0][0] >= .25
              and probe.status.get('status') == 'FAULT',
              zero_samples=len(stopped_commands), sustained_zero_s=stopped_commands[-1][0] - stopped_commands[0][0],
              status=probe.status)

    try:
        ready()
        original = {topic: probe.count_publishers(topic) for topic in DRIVING_TOPICS}
        report['unprefixed_driving_publishers'] = original
        check('unprefixed_driving_publishers_zero', not any(original.values()), counts=original)
        if args.scenario == 'health':
            probe.map_hash=('0' if probe.expected_map_hash[0]!='0' else '1')+probe.expected_map_hash[1:]; probe.settle()
            reply = probe.enable(); check('wrong_map_hash_blocks_enable', not reply['success'], reply=reply)
            probe.map_hash=probe.expected_map_hash; probe.health_kind = 'not_ready'; probe.settle()
            reply = probe.enable(); check('not_ready_blocks_enable', not reply['success'], reply=reply)
            probe.health_kind = 'ready'; probe.anchor_ns = 0; probe.settle(); activate()
            fault('malformed_health_faults', lambda: setattr(probe, 'health_kind', 'malformed'), 'localization')
            probe.health_kind = 'ready'; probe.anchor_ns = 0; probe.settle(); activate()
            fault('epoch_change_faults', lambda: setattr(probe, 'epoch', 'synthetic_epoch_2'), 'epoch')
            probe.anchor_ns = 0; probe.settle(); activate()
            probe.hold_anchor = True
            fault('trusted_anchor_loss_faults', lambda: setattr(probe, 'health_kind', 'not_ready'), 'localization')
            probe.health_kind = 'ready'; probe.settle()
            reply = probe.enable(); check('same_anchor_cannot_recover', not reply['success'], reply=reply)
            probe.hold_anchor = False; probe.anchor_ns = 0; probe.settle(); activate()
            fault('active_map_identity_mismatch_faults', lambda: setattr(probe,'map_hash',('0' if probe.expected_map_hash[0]!='0' else '1')+probe.expected_map_hash[1:]), 'map identity')
        elif args.scenario == 'odometry':
            activate()
            before=len(probe.commands);event_start=len(probe.events)
            delayed=copy.deepcopy(probe.last_odom)
            stamp=delayed.header.stamp.sec*10**9+delayed.header.stamp.nanosec-40_000_000
            delayed.header.stamp.sec,delayed.header.stamp.nanosec=divmod(stamp,10**9)
            # A pose jump in this old packet would fault if it were adopted.
            delayed.pose.pose.position.x+=100.
            probe.odom.publish(delayed);probe.settle(.3)
            check('single_reordered_source_packet_ignored',probe.commands[before:]
                  and all(row[1]>0. for row in probe.commands[before:])
                  and all(e['status'].get('status')=='RUNNING' for e in probe.events[event_start:]),
                  source_reorder_ns=40_000_000,discarded_packet_position_offset_m=100.,status=probe.status)
            reply=probe.enable(False)
            check('stop_between_reorder_and_fault_cases',reply['success'] and probe.spin_until(
                  lambda:probe.status.get('status')=='READY' and probe.speed<.03,6.),status=probe.status)
            for defect, reason in [('stale', 'odometry'), ('future', 'odometry'),
                                   ('backwards', ('stale or future odometry','Input freshness expired')), ('quaternion', 'quaternion'),
                                   ('reverse', 'reverse'), ('nonunit_quaternion', 'quaternion')]:
                activate()
                actionable_s=source_timeout_s(time.monotonic(),probe.get_clock().now().nanoseconds,
                    probe.last_good_odom_ns) if defect=='backwards' else None
                fault(defect + '_odometry_faults', lambda d=defect: setattr(probe, 'odom_fault', d), reason,actionable_s=actionable_s)
                probe.odom_fault = None; probe.settle(.25)
        elif args.scenario == 'ownership':
            activate()
            check('positive_command_observed', probe.spin_until(lambda: probe.target.drive.speed > .2, 6.),
                  command_speed=probe.target.drive.speed)
            def appear():
                nonlocal extra
                extra = probe.create_publisher(Drive, PREFIX + '/drive', 10)
            fault('late_command_owner', appear, 'publisher', maximum=.2)
            probe.destroy_publisher(extra); extra = None; probe.settle(.5); activate()
            fault('manual_authority_withdrawal_faults', lambda: setattr(probe, 'autonomy', False), 'withdrawn')
            probe.autonomy = True; probe.settle(.3)
            check('authority_return_requires_reenable', probe.status.get('status') == 'FAULT'
                  and probe.target.drive.speed == 0., status=probe.status)
            activate()
            check('moving_before_operator_stop', probe.spin_until(lambda: probe.speed > .3, 6.), speed=probe.speed)
            begin = time.monotonic(); reply = probe.enable(False)
            check('operator_disable_accepted', reply['success'], reply=reply)
            check('operator_stop_completes', probe.spin_until(lambda: probe.status.get('status') == 'READY'
                  and probe.target.drive.speed == 0. and probe.speed < .05, 6.), status=probe.status)
            commands = [(t, s, a) for t, s, a, _ in probe.commands if t >= begin]
            check('disable_deceleration_bound', commands and -.1 > min(a for _, _, a in commands) >= -probe.config.brake_limit - .01,
                  minimum_acceleration=min(a for _, _, a in commands), brake_limit=probe.config.brake_limit)
        elif args.scenario == 'isolation':
            activate(); probe.settle(.3)
            check('remapped_service_status_and_output_isolated', probe.status.get('backend') == 'acados_cpp'
                  and probe.count_publishers(PREFIX + '/drive') == 1
                  and probe.count_publishers('/mpcc_rt_shadow/drive') == 0,
                  status=probe.status, prefixed_drive_publishers=probe.count_publishers(PREFIX + '/drive'))
        elif args.scenario == 'watchdog':
            # Freeze the entire disposable ROS child group while disabled. This
            # measures a process stall, not blocking inside a native solver call.
            supervisor_file=out/'supervisor.json'
            check('disabled_before_process_stall', probe.status.get('status') in ('WAITING','READY')
                  and all(row[1]==0. for row in probe.commands),status=probe.status)
            initial=json.loads(supervisor_file.read_text())
            child_pid=next(event['pid'] for event in initial['events'] if event['event']=='started')
            begin=time.monotonic();os.killpg(child_pid,signal.SIGSTOP)
            def restarted():
                report=json.loads(supervisor_file.read_text())
                return report['restarts']==1 and sum(e['event']=='started' for e in report['events'])==2
            check('whole_process_stall_bounded_restart',probe.spin_until(restarted,2.))
            lifecycle=json.loads(supervisor_file.read_text())
            stopped=next(e for e in lifecycle['events'] if e['event']=='stopped')
            check('stopped_group_killed_within_bound', stopped['reason']=='status_stalled'
                  and stopped['signals']==['SIGTERM','SIGKILL'] and stopped['time']-begin<1.2,
                  lifecycle=stopped,scope='whole process SIGSTOP while disabled; no native-call stall injected')
            restart_time=max(e['time'] for e in lifecycle['events'] if e['event']=='started')
            check('restart_returns_disabled',probe.spin_until(lambda: any(e['time']>restart_time
                  and e['status'].get('worker_ready') is True and e['status'].get('status') in ('WAITING','READY')
                  for e in probe.events),10.))
            probe.settle(.3)
            check('restart_never_auto_enables',all(row[1]==0. for row in probe.commands)
                  and probe.status.get('status') in ('WAITING','READY'),status=probe.status)
            report['process_stall_scope']='whole process SIGSTOP, disabled synthetic domain; native-call wedging not exercised'
        elif args.scenario == 'lifecycle':
            activate()
            baseline=dict(probe.status)
            command_start,event_start=len(probe.commands),len(probe.events)
            begin=time.monotonic();probe.settle(args.seconds);ended=time.monotonic()
            commands=probe.commands[command_start:]
            statuses=[event['status'] for event in probe.events[event_start:]]
            report['lifecycle']=dict(duration_s=ended-begin,baseline=baseline,
                command_samples=len(commands),status_samples=len(statuses),
                accepted_activations_delta=statuses[-1].get('activated',0)-baseline.get('activated',0) if statuses else 0)
            check('repeated_positive_accepted_activations',lifecycle_pass(commands,statuses,baseline,ended,args.seconds),
                  evidence=report['lifecycle'],final_status=probe.status)
        elif args.scenario == 'freshness':
            activate()
            begin = time.monotonic()
            baseline = dict(probe.status)
            command_start, event_start = len(probe.commands), len(probe.events)
            stress_timer = probe.create_timer(.001, probe.refresh_inputs)
            try:
                probe.settle(6.)
            finally:
                probe.destroy_timer(stress_timer)
            duration = time.monotonic() - begin
            commands = probe.commands[command_start:]
            events = probe.events[event_start:]
            statuses = [event['status'] for event in events]
            positive_duration = (commands[-1][0] - commands[0][0]) if commands else 0.
            gaps = [after[0] - before[0] for before, after in zip(commands, commands[1:])]
            faults = [event for event in events if event['status'].get('status') != 'RUNNING']
            report['freshness_stress'] = dict(
                duration_s=duration, input_updates=probe.fresh_input_updates,
                input_update_hz=probe.fresh_input_updates / duration,
                authority_publish_hz=8 * probe.fresh_input_updates / duration,
                command_samples=len(commands), status_samples=len(events),
                positive_command_duration_s=positive_duration,
                maximum_command_gap_s=max(gaps, default=0.),
                baseline_status=baseline, nonrunning_events=faults)
            check('high_rate_valid_receive_updates', probe.fresh_input_updates / duration >= 250.,
                  duration_s=duration, input_updates=probe.fresh_input_updates,
                  input_update_hz=probe.fresh_input_updates / duration)
            check('sustained_positive_running_for_five_seconds', positive_duration >= 5.
                  and len(commands) >= 200 and all(row[1] > 0. for row in commands)
                  and max(gaps, default=math.inf) <= .1 and len(statuses) >= 25
                  and not faults and probe.status.get('status') == 'RUNNING',
                  positive_command_duration_s=positive_duration,
                  command_samples=len(commands), status_samples=len(statuses),
                  maximum_command_gap_s=max(gaps, default=0.),
                  nonrunning_events=faults[:1])
            counters = ('failed', 'rejected')
            degraded = [event for event in events if event['status'].get('native_status') != 0
                        or any(event['status'].get(key) != baseline.get(key) for key in counters)]
            check('no_source_or_protocol_degradation', not degraded,
                  counters=list(counters), degraded_events=degraded[:1])
        final_counts = {topic: probe.count_publishers(topic) for topic in DRIVING_TOPICS}
        check('final_unprefixed_driving_publishers_zero', not any(final_counts.values()), counts=final_counts)
        report['overall_pass'] = True
    except Exception as exc:
        report['overall_pass'] = False
        report['error'] = str(exc)
    finally:
        if extra is not None: probe.destroy_publisher(extra)
        if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close()
        report['controller_exit_code'] = process.returncode
        report['final_status'] = probe.status
        (out / 'status.json').write_text(json.dumps(probe.events, indent=2) + '\n')
        (out / 'commands.json').write_text(json.dumps(probe.commands, indent=2) + '\n')
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        probe.destroy_node(); rclpy.shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['overall_pass'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--scenario', choices=['health', 'odometry', 'ownership', 'isolation', 'freshness', 'watchdog', 'lifecycle'], required=True)
    parser.add_argument('--frequency',type=float,default=20.)
    parser.add_argument('--handover-delay',type=float,default=.02)
    parser.add_argument('--budget',type=float,default=.05)
    parser.add_argument('--seconds',type=float,default=6.)
    options=parser.parse_args()
    if any(not math.isfinite(v) for v in (options.frequency,options.handover_delay,options.budget,options.seconds)) or not 0<options.frequency<=50 or not 0<=options.handover_delay<=.1 or options.budget<=0 or options.seconds<1:
        parser.error('frequency in (0,50], lead in [0,.1], positive budget and seconds>=1 required')
    raise SystemExit(run(options))
