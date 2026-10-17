"""Synthetic isolated ROS graph only. Never run on domain 0."""
import os
import subprocess
import time
import tempfile
import shutil
import copy
from pathlib import Path
import pytest

@pytest.mark.parametrize("fault", ["authority", "wheel_stale", "wheel_frozen", "final_mismatch", "publisher_conflict", "operator_stop", "driver_true", "driver_missing", "final_frozen", "final_delayed", "rc_stale", "telemetry_fault"])
def test_explicit_start_and_authority_abort(fault):
    assert int(os.environ.get('ROS_DOMAIN_ID','0')) == 227
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from vesc_msgs.msg import VescCommand, VescStateStamped
    from crsf_receiver_msg.msg import CRSFChannels16
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from diagnostic_msgs.msg import DiagnosticArray
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterValue, ParameterType
    rclpy.init()
    fake=Node('lateral_fake_selector')
    pubs={
        'wheel':fake.create_publisher(Odometry,'/rear_axle/wheel_odom',10),
        'telemetry':fake.create_publisher(VescStateStamped,'/sensors/core',10),
        'rc':fake.create_publisher(CRSFChannels16,'/rc/channels',10),
        'authority':fake.create_publisher(Bool,'/control/calibration_enabled',10),
        'final':fake.create_publisher(VescCommand,'/vesc/command',10),
    }
    requests=[];statuses=[]
    fake.create_subscription(VescCommand,'/calib/vesc_command',lambda m:requests.append(m),10)
    fake.create_subscription(DiagnosticArray,'/calib/lateral/status',lambda m:statuses.append(m),10)
    service=fake.create_client(SetBool,'/calib/lateral/enable')
    if fault!='driver_missing':
        def driver_parameters(request,response):
            response.values=[ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=fault=='driver_true')]
            return response
        driver_service=fake.create_service(GetParameters,'/vesc_driver_node/get_parameters',driver_parameters)

    temp=tempfile.TemporaryDirectory()
    proc=subprocess.Popen([os.environ['LATERAL_NODE_EXECUTABLE'],'--ros-args','-p','physical_configuration_selected:=true','-p','ramp_rate:=1.0','-p','wheelbase:=0.3','-p','nominal_radius:=3.0','-p','speed_steps:=[0.4, 0.6]','-p','speed_limit:=1.0','-p','steering_limit:=0.2','-p','max_motor_current:=10.0','-p','min_voltage:=10.0','-p','max_temperature:=60.0','-p','stop_seconds:=0.15','-p','output_root:='+temp.name])
    last=None
    frozen_stamp=None
    frozen_final=None
    def pump(seconds,authority=True,inject=False):
        nonlocal last, frozen_stamp, frozen_final
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            stamp=fake.get_clock().now().to_msg()
            w=Odometry();w.header.stamp=stamp;w.child_frame_id='base_link'
            if inject and fault=='wheel_frozen':
                if frozen_stamp is None:frozen_stamp=stamp
                w.header.stamp=frozen_stamp
            if not (inject and fault=='wheel_stale'):pubs['wheel'].publish(w)
            v=VescStateStamped();v.header.stamp=stamp;v.state.voltage_input=20.
            if inject and fault=="telemetry_fault":v.state.current_motor=100.
            pubs['telemetry'].publish(v)
            rc=CRSFChannels16()
            for k in range(1,17):setattr(rc,'ch'+str(k),992)
            rc.ch8=1800
            if not (inject and fault=='rc_stale'):pubs['rc'].publish(rc)
            pubs['authority'].publish(Bool(data=authority))
            if requests:last=copy.deepcopy(requests[-1])
            if last:
                last.header.stamp=stamp
                if inject and fault=='final_frozen':
                    if frozen_final is None:frozen_final=stamp
                    last.header.stamp=frozen_final
                if inject and fault=='final_delayed':last.header.stamp.sec-=1
                if inject and fault=='final_mismatch':last.target_speed_mps=8.0
                pubs['final'].publish(last)
            rclpy.spin_once(fake,timeout_sec=.02)
    try:
        pump(1.)
        assert service.wait_for_service(timeout_sec=3)
        assert requests and all(m.target_speed_mps==0 for m in requests)
        req=SetBool.Request();req.data=True
        operator=subprocess.Popen([str(Path(os.environ["LATERAL_OPERATOR_DIR"])/"start")],stdout=subprocess.PIPE,text=True)
        pump(.6)
        start_code=operator.wait(timeout=5)
        start_output=operator.communicate()[0]
        if fault.startswith('driver_'):
            assert start_code==1 and 'start refused' in start_output
            assert all(m.target_speed_mps==0 for m in requests)
            assert statuses[-1].status[0].message=='IDLE: not_started'
            diagnostics={v.key:v.value for v in statuses[-1].status[0].values}
            assert diagnostics['driver_current_startup_disabled_verified']=='false'
            assert diagnostics['authority']=='true'
            return
        assert start_code==0
        assert "session started" in start_output
        assert any(m.target_speed_mps>0 and m.steering_angle>0 for m in requests)
        assert statuses[-1].status[0].message=='RUN: running'
        preconditions={v.key:v.value for v in statuses[-1].status[0].values}
        for key in ('authority','exclusive_publishers','driver_current_startup_disabled_verified'):
            assert preconditions[key]=='true'
        assert all(preconditions[f'control_{k}_valid']=='true' for k in range(5))
        fault_status_start=len(statuses)
        if fault=="operator_stop":
            operator=subprocess.Popen([str(Path(os.environ["LATERAL_OPERATOR_DIR"])/"stop")],stdout=subprocess.PIPE,text=True)
            pump(.2)
            assert operator.wait(timeout=5)==0
            assert "coast requested" in operator.communicate()[0]
        if fault=="publisher_conflict":pubs["conflict"]=fake.create_publisher(VescCommand,"/calib/vesc_command",10)
        authority_healthy=fault!="authority"
        pump(.5,authority=authority_healthy,inject=True)
        expected_reason={
            "authority":"invalid_or_nonmonotonic_control_sample",
            "wheel_stale":"control_input_stale_or_invalid",
            "wheel_frozen":"invalid_or_nonmonotonic_control_sample",
            "final_mismatch":"invalid_or_nonmonotonic_control_sample",
            "publisher_conflict":"publisher_conflict",
            "operator_stop":"operator_stop",
            "final_frozen":"invalid_or_nonmonotonic_control_sample",
            "final_delayed":"invalid_or_nonmonotonic_control_sample",
            "rc_stale":"control_input_stale_or_invalid",
            "telemetry_fault":"invalid_or_nonmonotonic_control_sample",
        }[fault]
        # Establish fault-specific abort before inspect or any cleanup action.
        assert statuses and statuses[-1].status[0].name=='lateral_grip'
        assert statuses[-1].status[0].message=='ABORTED: '+expected_reason
        assert requests[-1].control_mode==VescCommand.MODE_CURRENT
        assert requests[-1].target_current==0 and requests[-1].target_speed_mps==0
        diagnostics={v.key:v.value for v in statuses[-1].status[0].values}
        assert diagnostics['authority']==('false' if fault=='authority' else 'true')
        invalid_channel={"wheel_frozen":0,"telemetry_fault":1,"authority":3,
                         "final_mismatch":4,"final_frozen":4,"final_delayed":4}.get(fault)
        if invalid_channel is not None:
            assert diagnostics[f'control_{invalid_channel}_valid']=='false'
        if fault in ('wheel_frozen','final_frozen'):
            channel=0 if fault=='wheel_frozen' else 4
            first_abort=next(s.status[0] for s in statuses[fault_status_start:] if s.status[0].message.startswith('ABORTED'))
            first_diagnostics={v.key:v.value for v in first_abort.values}
            assert float(first_diagnostics[f'control_{channel}_source_age'])<.25
            assert float(first_diagnostics[f'control_{channel}_receive_age'])<.1
        if fault in ('wheel_stale','rc_stale'):
            channel=0 if fault=='wheel_stale' else 2
            assert float(diagnostics[f'control_{channel}_receive_age'])>.25
        if fault=='publisher_conflict':assert diagnostics['exclusive_publishers']=='false'
        operator=subprocess.Popen([str(Path(os.environ['LATERAL_OPERATOR_DIR'])/'inspect')],stdout=subprocess.PIPE,text=True)
        # Keep the same isolated fault active: wrong-final cases never receive a
        # correct echo, and non-authority cases never withdraw authority.
        pump(.2,authority=authority_healthy,inject=True)
        assert operator.wait(timeout=5)==0
        assert 'ABORTED: '+expected_reason in operator.communicate()[0]
        req.data=True;future=service.call_async(req)
        pump(.2,authority=authority_healthy,inject=True)
        assert future.done() and not future.result().success
    finally:
        proc.send_signal(2)
        proc.wait(timeout=10)
        assert list(Path(temp.name).glob('*/summary.json'))
        if os.environ.get('LATERAL_TEST_ARTIFACT_ROOT'):
            destination=Path(os.environ['LATERAL_TEST_ARTIFACT_ROOT'])/fault
            shutil.copytree(temp.name,destination,dirs_exist_ok=True)
            destination.chmod(0o755)
            for path in destination.rglob("*"):path.chmod(0o755 if path.is_dir() else 0o644)
        fake.destroy_node();rclpy.shutdown();temp.cleanup()
