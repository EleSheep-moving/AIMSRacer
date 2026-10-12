#!/usr/bin/env python3
"""Real-time DDS input replay against the ROS wrapper, without vehicle drivers.

An instrumented node records actual callback order and compares each publication
to a canonical observer snapshot. This verifies routing/timestamps, not physical
acceleration truth or full vehicle CPU load. Requires an isolated ROS domain.
"""
from argparse import ArgumentParser
from dataclasses import asdict
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from aims_racer_system.longitudinal_acceleration import LongitudinalAccelerationNode
from aims_racer_system.longitudinal_observer import LongitudinalObserver, ObserverConfig
from replay import load_runs


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


def main():
    parser=ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--forces',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--run',default='193524_v35_n15_accel1_cap4_lap02')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('This input replay requires ROS_DOMAIN_ID=191 and ROS_LOCALHOST_ONLY=1')
    args.output.mkdir(parents=True,exist_ok=True)
    run=next(r for r in load_runs(args.inputs,args.forces,args.output) if r.name==args.run)
    events=[e for e in run.events if e[2] in ('speed','force')]
    config=ObserverConfig(use_imu=True,jerk_psd=30.,disturbance_psd=.1,
                          speed_variance=.01,force_variance=16.)
    oracle=LongitudinalObserver(config)
    rows,seen,statuses=[],{},[]
    costs={'wheel':[],'imu':[]}

    class InstrumentedNode(LongitudinalAccelerationNode):
        latest_imu_source_ns=None

        def _on_wheel(self,msg):
            oracle.receive('speed',stamp_ns(msg.header.stamp),msg.twist.twist.linear.x)
            super()._on_wheel(msg)
            costs['wheel'].append(self.timing_ms['wheel'][-1])

        def _on_imu(self,msg):
            self.latest_imu_source_ns=stamp_ns(msg.header.stamp)
            if msg.linear_acceleration_covariance[0]>=0:
                oracle.receive('force',self.latest_imu_source_ns,msg.linear_acceleration.x)
            super()._on_imu(msg)
            costs['imu'].append(self.timing_ms['imu'][-1])

    rclpy.init(args=[])
    node=None;probe=None;executor=None
    try:
        node=InstrumentedNode()
        assert node.config==config
        probe=rclpy.create_node('longitudinal_acceleration_input_replay')
        wheel_pub=probe.create_publisher(Odometry,'/rear_axle/wheel_odom',50)
        imu_pub=probe.create_publisher(Imu,'/rear_axle/imu',50)
        accel_sub=probe.create_subscription(AccelWithCovarianceStamped,
            '/state_estimation/longitudinal_acceleration',
            lambda m:seen.setdefault(stamp_ns(m.header.stamp),m),qos_profile_sensor_data)
        status_sub=probe.create_subscription(DiagnosticArray,
            '/state_estimation/longitudinal_acceleration/status',
            lambda m:statuses.append({v.key:v.value for s in m.status for v in s.values}),10)
        original_publish=node.acceleration_publisher.publish

        def checked_publish(msg):
            now=stamp_ns(msg.header.stamp)
            expected=oracle.snapshot(now)
            assert node.initialized and node.latest_imu_source_ns is not None
            assert msg.header.frame_id=='base_link'
            assert all(msg.accel.covariance[i]==1e6 for i in [7,14,21,28,35])
            error=abs(expected.acceleration-msg.accel.accel.linear.x)
            variance_error=abs(expected.acceleration_variance-msg.accel.covariance[0])
            assert error<1e-9 and variance_error<1e-9
            rows.append(dict(stamp_ns=now,imu_source_ns=node.latest_imu_source_ns,
                             acceleration=msg.accel.accel.linear.x,
                             acceleration_variance=msg.accel.covariance[0],
                             oracle_acceleration_error=error,oracle_variance_error=variance_error))
            original_publish(msg)
        node.acceleration_publisher.publish=checked_publish
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(probe)
        deadline=time.monotonic()+12.
        while wheel_pub.get_subscription_count()<1 or imu_pub.get_subscription_count()<1 or probe.count_publishers('/state_estimation/longitudinal_acceleration')<1:
            assert time.monotonic()<deadline,'DDS discovery timed out'
            executor.spin_once(timeout_sec=.005)
        for _ in range(50):executor.spin_once(timeout_sec=.005)
        ros_origin=probe.get_clock().now().nanoseconds+500_000_000
        index=0
        while index<len(events):
            now=probe.get_clock().now().nanoseconds
            receipt,source,kind,value,valid=events[index]
            if ros_origin+receipt-run.origin_ns>now:
                executor.spin_once(timeout_sec=.001)
                continue
            mapped=ros_origin+source-run.origin_ns
            msg=Odometry() if kind=='speed' else Imu()
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(mapped,1_000_000_000)
            if kind=='speed':
                msg.header.frame_id='odom';msg.child_frame_id='base_link'
                msg.twist.twist.linear.x=value;wheel_pub.publish(msg)
            else:
                msg.header.frame_id='base_link';msg.linear_acceleration.x=value if valid else 0.
                msg.linear_acceleration_covariance[0]=.05 if valid else -1.
                imu_pub.publish(msg)
            index+=1
            executor.spin_once(timeout_sec=0.)
        deadline=time.monotonic()+1.5
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.001)
        node._publish_status()
        deadline=time.monotonic()+.2
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.001)
        assert rows and len(seen)==len(rows),'Acceleration DDS delivery lost output'
        assert probe.count_publishers('/drive')==0 and probe.count_publishers('/tf')==0
        assert oracle.counters['numerical_failure']==0 and oracle.counters['too_old']==0
        assert node.observer.counters['numerical_failure']==0
        assert node.input_counters['future_stamp']==node.input_counters['snapshot_failure']==0
        for row in rows:
            assert seen[row['stamp_ns']].accel.accel.linear.x==row['acceleration']
        stamps=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
        imu_stamps=np.array([r['imu_source_ns'] for r in rows],dtype=np.int64)
        duration=(stamps[-1]-stamps[0])*1e-9
        import inspect
        metrics=dict(run=run.name,config=asdict(config),
                     runtime_module=inspect.getfile(LongitudinalAccelerationNode),
                     input_events_sent=len(events),
                     input_events_received=sum(node.input_counters[k] for k in ['wheel_callbacks','imu_callbacks']),
                     imu_callbacks=node.input_counters['imu_callbacks'],outputs_published=len(rows),outputs_delivered=len(seen),
                     output_rate_hz=(len(rows)-1)/duration,
                     output_interval_ms_p50_p95_max=np.percentile(np.diff(stamps)*1e-6,[50,95,100]).tolist(),
                     latest_imu_source_to_estimate_ms_p50_p95_max=np.percentile((stamps-imu_stamps)*1e-6,[50,95,100]).tolist(),
                     oracle_max_acceleration_error=max(r['oracle_acceleration_error'] for r in rows),
                     oracle_max_variance_error=max(r['oracle_variance_error'] for r in rows),
                     instrumented_callback_ms={k:np.percentile(v,[50,95,100]).tolist() for k,v in costs.items()},
                     observer_counters=dict(node.observer.counters),input_counters=dict(node.input_counters),
                     validation='DDS wrapper equivalence, timestamps and delivery only; no vehicle joint load or physical truth')
        assert metrics['input_events_received']==len(events),'Input DDS delivery lost observations'
        (args.output/'runtime-replay.json').write_text(json.dumps(metrics,indent=2)+'\n')
        with (args.output/'outputs.jsonl').open('w') as file:
            for row in rows:file.write(json.dumps(row)+'\n')
        (args.output/'status.json').write_text(json.dumps(statuses,indent=2)+'\n')
        print(json.dumps(metrics,indent=2),flush=True)
    finally:
        if executor:executor.shutdown()
        if node:node.destroy_node()
        if probe:probe.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
