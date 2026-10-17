"""Inspect/record only; never launches vehicle, motion commands or estimators."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from .bag import topic_specifications
from .reference import map_time, vector

RAW_TOPICS = ['/livox/lidar','/livox/imu','/livox/imu_bias_corrected','/fastlio2/lio_odom',
              '/rear_axle/imu','/rear_axle/lio_odom','/rear_axle/wheel_odom',
              '/odometry/filtered','/tf_static','/clock','/imu/gyro_bias/status',
              '/vesc/command','/drive','/mpcc/status','/sensors/core','/sensors/servo_position_command','/control/autonomy_speed_enabled','/calib/status','/tf',
              '/localization/status','/localization/anchor_status','/localization/map_sha256']


def record_topics(config):
    return list(dict.fromkeys([*topic_specifications(config),*RAW_TOPICS,*config.get('extra_topics',[])]))


def inspect_configuration(config, graph):
    capture_errors,analysis_errors=[],[]
    try: specs=topic_specifications(config)
    except (ValueError,KeyError) as error: capture_errors.append(str(error));specs={}
    try:
        clock=config['clock'];start=clock.get('source_anchor_ns')
        map_time([start if start is not None else 0],clock)
    except (ValueError,KeyError,TypeError) as error:analysis_errors.append(str(error))
    for value,n,label in [(config.get('body_orientation_in_marker_xyzw'),4,'body orientation')]+[
            (point.get('origin_in_marker_m'),3,name+' origin') for name,point in config['references'].items()]:
        try:vector(value,n,label)
        except (ValueError,TypeError) as error:analysis_errors.append(str(error))
    missing=[name for name in specs if name not in graph]
    mismatches={name:dict(expected=item['message_type'],actual=graph[name]) for name,item in specs.items()
                if name in graph and graph[name]!=[item['message_type']]}
    return dict(capture_configuration_errors=capture_errors,analysis_configuration_errors=analysis_errors,
        missing_topics=missing,type_mismatches=mismatches,graph=graph,
        note='missing publishers may start later; capture can wait, mismatched existing types require correction')


def inspect_live(config, seconds=3.):
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from .bag import source_ns
    rclpy.init();node=rclpy.create_node('vicon_state_inspect');subscriptions=[];samples={}
    try:
        deadline=time.monotonic()+min(1.,seconds/2)
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        graph=dict(node.get_topic_names_and_types());result=inspect_configuration(config,graph)
        result['nodes']=[namespace.rstrip('/')+'/'+name for name,namespace in node.get_node_names_and_namespaces()]
        if not result['capture_configuration_errors']:
            for topic,item in topic_specifications(config).items():
                if graph.get(topic)!=[item['message_type']]:continue
                samples[topic]=dict(messages=0)
                def receive(message,topic=topic):
                    entry=samples[topic];entry['messages']+=1
                    entry['last_source_ns']=source_ns(message);entry['last_frame_id']=message.header.frame_id
                    entry['last_local_ros_receive_ns']=node.get_clock().now().nanoseconds
                subscriptions.append(node.create_subscription(get_message(item['message_type']),topic,receive,
                    QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)))
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        result['samples']=samples;result['sample_duration_s']=seconds
        result['sample_note']='Vicon source clock may differ; no age/latency inferred until clock mapping is known'
        return result
    finally:node.destroy_node();rclpy.shutdown()


def command_snapshot(command,path):
    try:
        result=subprocess.run(command,text=True,capture_output=True,timeout=10)
        path.write_text(result.stdout+result.stderr)
        return dict(command=command,returncode=result.returncode)
    except (OSError,subprocess.TimeoutExpired) as error:
        path.write_text(str(error)+'\n');return dict(command=command,error=str(error))


def record(config,output,seconds=None):
    """Compatibility API: use the same native timing recorder as the CLI."""
    from .session import record as native_record
    return native_record(config,output,seconds)
