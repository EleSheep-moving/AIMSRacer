"""Exercise the installed native executable against identical ROS input gates."""
import json
import math
import os
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from vesc_msgs.msg import VescCommand

from conftest import health_values


@pytest.fixture
def ros_nodes(tmp_path, circle_payload, request):
    from types import SimpleNamespace
    exe=os.environ.get("AIMS_PP_NATIVE_NODE")
    if not exe:pytest.skip("set AIMS_PP_NATIVE_NODE to installed pp_node")
    (tmp_path/'reference.json').write_text(json.dumps(circle_payload))
    vehicle = tmp_path/'vehicle.yaml'
    vehicle.write_text('wheelbase: 0.36\nsteer_limit: 0.45\nrear_offset: 0.0\nhalf_length: null\nfront_extent: 0.52\nrear_extent: 0.1\nhalf_width: 0.16\nmax_speed: 4.0\ngeometry_verified: true\n')
    rclpy.init()
    sim = getattr(request,'param',False)
    process=subprocess.Popen([exe,'--ros-args','-p',f'reference_directory:={tmp_path}','-p',f'vehicle_config:={vehicle}','-p',f'log_directory:={tmp_path}/logs','-p',f'use_sim_time:={str(sim).lower()}'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    pp=SimpleNamespace(runtime=SimpleNamespace(enabled=False,phase='DISABLED'))
    driver = rclpy.create_node('pp_test_inputs',parameter_overrides=[Parameter('use_sim_time',value=sim)])
    clock_pub = driver.create_publisher(Clock,'/clock',10) if sim else None
    latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = dict(odom=driver.create_publisher(Odometry,'/odometry/filtered',1),health=driver.create_publisher(DiagnosticArray,'/localization/status',10),
                identity=driver.create_publisher(String,'/localization/map_sha256',latched),valid=driver.create_publisher(Bool,'/localization/map_valid',latched),
                authority=driver.create_publisher(Bool,'/control/autonomy_speed_enabled',10),command=driver.create_publisher(VescCommand,'/vesc/command',1))
    commands = []
    def receive_status(message):
        for record in message.status:
            values={v.key:v.value for v in record.values}
            pp.runtime.enabled=values.get('enabled')=='true';pp.runtime.phase=values.get('phase','')
    driver.create_subscription(DiagnosticArray,'/pp/status',receive_status,10)
    driver.create_subscription(AckermannDriveStamped,'/drive',commands.append,1)
    sequence = 0
    sim_ns = 10_000_000_000
    def pump(duration=.3, authority=True, mode=0, freeze=False):
        nonlocal sequence
        nonlocal sim_ns
        until = time.monotonic()+duration
        while time.monotonic()<until:
            if sim and not freeze:
                sim_ns += 20_000_000
                clock = Clock()
                clock.clock.sec,clock.clock.nanosec = divmod(sim_ns,1_000_000_000)
                clock_pub.publish(clock)
                for _ in range(2):
                    pass
                    rclpy.spin_once(driver,timeout_sec=.002)
            ns = driver.get_clock().now().nanoseconds
            stamp = driver.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp,odom.header.frame_id,odom.child_frame_id = stamp,'odom','base_link'
            odom.pose.pose.position.x = 2.
            odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = math.sin(math.pi/4),math.cos(math.pi/4)
            odom.twist.twist.linear.x = 1.
            sequence += 1
            status = DiagnosticStatus(name='aims_racer_system/localization',values=[KeyValue(key=k,value=v) for k,v in health_values(sequence,ns).items()])
            health = DiagnosticArray(status=[status])
            health.header.stamp = stamp
            command = VescCommand()
            command.header.stamp,command.header.frame_id,command.control_mode = stamp,'base_link',mode
            if mode == 0: command.target_speed_mps = 1.
            else: command.target_current = 1.
            pubs['identity'].publish(String(data='a'*64))
            pubs['valid'].publish(Bool(data=True))
            pubs['health'].publish(health)
            pubs['authority'].publish(Bool(data=authority))
            pubs['command'].publish(command)
            pubs['odom'].publish(odom)
            for _ in range(8):
                pass
                rclpy.spin_once(driver,timeout_sec=.002)
        return commands
    def service(enable):
        client = driver.create_client(SetBool,'/pp/enable')
        assert client.wait_for_service(timeout_sec=2.)
        future = client.call_async(SetBool.Request(data=enable))
        until = time.monotonic()+2.
        while not future.done() and time.monotonic()<until:
            pass
            rclpy.spin_once(driver,timeout_sec=.002)
        assert future.done()
        result = future.result()
        driver.destroy_client(client)
        return result
    try:
        pump(1.)
        yield pp,driver,pump,service,commands,tmp_path
    finally:
        process.send_signal(signal.SIGINT)
        output=process.communicate(timeout=5.)[0]
        assert process.returncode==0,output
        driver.destroy_node()
        rclpy.shutdown()


def test_ros_setbool_default_disabled_and_authority_loss(ros_nodes):
    pp,_,pump,service,commands,path = ros_nodes
    assert commands and all(c.drive.speed == 0 for c in commands)
    assert service(True).success
    pump()
    assert any(c.drive.speed == 1. for c in commands)
    assert all(c.header.frame_id=='base_link' and c.header.stamp.sec>0 for c in commands)
    pump(authority=False)
    assert not pp.runtime.enabled and commands[-1].drive.speed==0
    pump()
    assert commands[-1].drive.speed==0
    assert service(True).success
    pump()
    assert service(False).success
    pump(.1)
    assert commands[-1].drive.speed==0
    assert (path/'logs'/'runtime.csv').exists()


def test_ros_duplicate_publisher_and_current_mode_stop(ros_nodes):
    pp,driver,pump,service,commands,_ = ros_nodes
    assert service(True).success
    pump(.1)
    duplicate = driver.create_publisher(AckermannDriveStamped,'/drive',1)
    pump(.5)
    assert pp.runtime.phase=='FAULT' and commands[-1].drive.speed==0
    assert not service(True).success
    driver.destroy_publisher(duplicate)
    pump(.5)
    assert service(True).success
    pump(mode=1)
    assert pp.runtime.phase=='FAULT' and commands[-1].drive.speed==0


@pytest.mark.parametrize('ros_nodes',[True],indirect=True)
def test_sim_clock_freeze_stops_on_monotonic_source_age(ros_nodes):
    pp,_,pump,service,commands,_ = ros_nodes
    assert service(True).success
    pump(.1)
    assert pp.runtime.enabled
    pump(.25,freeze=True)
    assert pp.runtime.phase=='FAULT' and commands[-1].drive.speed==0
    assert not service(True).success


def test_cli_repeated_sigint_cleans_up_without_traceback(tmp_path,circle_payload):
    (tmp_path/'reference.json').write_text(json.dumps(circle_payload))
    vehicle=tmp_path/'vehicle.yaml'
    vehicle.write_text('wheelbase: 0.36\nsteer_limit: 0.45\nrear_offset: 0.0\nhalf_length: null\nfront_extent: 0.52\nrear_extent: 0.1\nhalf_width: 0.16\nmax_speed: 4.0\ngeometry_verified: true\n')
    exe=os.environ.get('AIMS_PP_NATIVE_NODE')
    if not exe:pytest.skip('native node unavailable')
    command=[exe,'--ros-args','-p',f'reference_directory:={tmp_path}',
             '-p',f'vehicle_config:={vehicle}','-p',f'log_directory:={tmp_path}/logs']
    process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=dict(os.environ,ROS_DOMAIN_ID='95'))
    try:
        time.sleep(.8)
        process.send_signal(signal.SIGINT)
        time.sleep(.005)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        output=process.communicate(timeout=3.)[0]
        assert process.returncode==0,output
        assert 'Traceback' not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()

def test_native_publication_observed_near_50hz(ros_nodes):
    _,_,pump,_,commands,_=ros_nodes
    start=len(commands);pump(1.)
    times=[c.header.stamp.sec+c.header.stamp.nanosec*1e-9 for c in commands[start:]]
    assert len(times)>=35
    rate=(len(times)-1)/(times[-1]-times[0])
    assert 40.<=rate<=60.,rate
