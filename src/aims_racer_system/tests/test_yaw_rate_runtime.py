"""Real DDS and launch contracts: yaw-only gyro observer; no vehicle hardware."""
import importlib.util
import os
from pathlib import Path
import subprocess
import time
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import AccelWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock

@pytest.fixture
def graph(tmp_path):
    exe=Path(os.environ.get('YAW_RATE_EXECUTABLE','/missing/yaw_rate'))
    assert exe.is_file()
    rclpy.init();node=rclpy.create_node('yaw_rate_contract_test')
    output=[];accel=[];status=[]
    subs=[node.create_subscription(Imu,'/state_estimation/yaw_rate',output.append,100),
          node.create_subscription(AccelWithCovarianceStamped,'/state_estimation/yaw_acceleration',accel.append,100),
          node.create_subscription(DiagnosticArray,'/state_estimation/yaw_rate/status',status.append,100)]
    pub=node.create_publisher(Imu,'/livox/imu_bias_corrected',qos_profile_sensor_data)
    clock=node.create_publisher(Clock,'/clock',10)
    log=(tmp_path/'node.log').open('w')
    proc=subprocess.Popen([str(exe),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT)
    class G:
        now=10_000_000_000
        def spin(self,seconds=.08):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        def tick(self,stamp=None):
            if stamp is not None:self.now=stamp
            m=Clock();m.clock.sec,m.clock.nanosec=divmod(self.now,10**9);clock.publish(m);self.spin()
        def send(self,rate=.5,stamp=None,frame='livox_frame',available=True,bad_cov=False):
            m=Imu();m.header.frame_id=frame
            m.header.stamp.sec,m.header.stamp.nanosec=divmod(self.now if stamp is None else stamp,10**9)
            m.angular_velocity.z=rate;m.angular_velocity_covariance[0]=0. if available else -1.
            if bad_cov:m.angular_velocity_covariance[8]=float('nan')
            pub.publish(m);self.spin()
        def ready(self):
            for _ in range(6):self.tick(self.now+5_000_000);self.send()
        def fields(self):
            self.spin(.15);assert status
            return {x.key:x.value for x in status[-1].status[0].values}
    g=G();g.node=node;g.output=output;g.accel=accel
    try:
        deadline=time.monotonic()+10
        while not pub.get_subscription_count() or not clock.get_subscription_count():
            assert time.monotonic()<deadline;g.spin()
        g.tick();yield g
    finally:
        proc.terminate();proc.wait(timeout=5);log.close()
        for s in subs:node.destroy_subscription(s)
        node.destroy_node();rclpy.shutdown()

def test_source_time_covariance_and_graph(graph):
    g=graph;g.ready();assert g.output and len(g.output)==len(g.accel)
    m=g.output[-1];a=g.accel[-1]
    assert m.header.frame_id=='base_link' and m.header.stamp.sec*10**9+m.header.stamp.nanosec==g.now
    assert m.angular_velocity.z==pytest.approx(.5) and abs(a.accel.accel.angular.z)<1e-8
    assert m.orientation_covariance[0]==-1 and m.linear_acceleration_covariance[0]==-1
    assert m.angular_velocity_covariance[0]>=1e6 and m.angular_velocity_covariance[8]>0
    assert all(a.accel.covariance[k]>=1e6 for k in [0,7,14,21,28])
    assert a.accel.covariance[35]>0
    subs={n for n,_ in g.node.get_subscriber_names_and_types_by_node('yaw_rate','/')}
    assert '/livox/imu_bias_corrected' in subs
    assert subs <= {'/livox/imu_bias_corrected','/clock','/parameter_events'}
    pubs={n for n,_ in g.node.get_publisher_names_and_types_by_node('yaw_rate','/')}
    assert not pubs.intersection({'/tf','/tf_static','/drive','/vesc/command','/odometry/filtered','/state_estimation/body_acceleration'})

def test_input_faults_gap_and_clock_reset(graph):
    g=graph;g.ready();count=len(g.output)
    g.send();g.send(stamp=g.now-1);g.send(stamp=g.now+1)
    g.send(frame='bad');g.send(available=False);g.send(rate=float('nan'));g.send(bad_cov=True)
    assert len(g.output)==count
    f=g.fields()
    for k in ['duplicate','out_of_order','future_stamp','wrong_frame','unavailable','nonfinite']:
        assert int(f[k])>=1,(k,f)
    g.tick(g.now+100_000_000);assert g.fields()['valid']=='false'
    g.send();assert len(g.output)==count
    g.tick(g.now+5_000_000);g.send();assert len(g.output)==count
    g.tick(g.now+5_000_000);g.send();assert len(g.output)==count+1
    assert int(g.fields()['gyro_gaps'])==1
    g.tick(g.now-2_000_000_000);g.send();assert len(g.output)==count+1
    assert int(g.fields()['clock_resets'])==1 and g.fields()['valid']=='false'
    for _ in range(2):g.tick(g.now+5_000_000);g.send()
    assert len(g.output)==count+2
    count=len(g.output);g.tick(g.now+100_000_000);g.send(stamp=g.now-60_000_000)
    assert len(g.output)==count and int(g.fields()['stale_stamp'])>=1

def test_launch_flags_and_recording(monkeypatch,tmp_path):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    root=Path(__file__).resolve().parents[1]
    def load(name):
        spec=importlib.util.spec_from_file_location('yaw_contract_'+name.replace('.','_'),root/'launch'/name)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        monkeypatch.setattr(module,'get_package_share_directory',lambda _:str(root));return module
    c=LaunchContext()
    for name in ['vehicle.launch.py','race.launch.py']:
        defaults={a.name:perform_substitutions(c,a.default_value) for a in load(name).generate_launch_description().entities if isinstance(a,DeclareLaunchArgument) and a.default_value is not None}
        assert defaults['yaw_rate']=='false'
    mod=load('yaw_rate.launch.py');monkeypatch.setattr(mod,'Node',lambda **kw:kw)
    c.launch_configurations.update(geometry_config=str(root/'params/rear_axle_geometry.yaml'),config=str(root/'params/yaw_rate.yaml'),use_sim_time='false')
    n=mod.build_node(c)[0];assert n['executable']=='yaw_rate'
    assert n['parameters'][1]=={'livox_quaternion':[0.,0.,0.,1.],'use_sim_time':False}
    vehicle=load('vehicle.launch.py');monkeypatch.setattr(vehicle,'Node',lambda **kw:kw)
    monkeypatch.setattr(vehicle,'include',lambda pkg,name,arguments=None:{'include':name})
    c.launch_configurations.update(mapping='false',vesc_config='/v',lio_config='/l',mid360_config='/m',yaw_rate='false')
    assert not any(x.get('include')=='yaw_rate.launch.py' for x in vehicle.build_vehicle(c))
    c.launch_configurations['yaw_rate']='true'
    assert sum(x.get('include')=='yaw_rate.launch.py' for x in vehicle.build_vehicle(c))==1
    race=load('race.launch.py');seen=[]
    monkeypatch.setattr(race,'include',lambda pkg,name,arguments=None:seen.append((name,arguments)) or object())
    mapfile=tmp_path/'map.pcd';mapfile.write_text('map');bundle=tmp_path/'bundle';bundle.mkdir()
    c.launch_configurations.update(map_file=str(mapfile),artifact_directory=str(bundle),record='false',auto_start='false',repeat_laps='false',initial_pose='')
    race.build_race(c);assert next(args for name,args in seen if name=='vehicle.launch.py')['yaw_rate']=='true'
    from aims_racer_system.launch_support import recording_actions
    c.launch_configurations.update(record='true',session_directory=str(tmp_path/'session'))
    rec=recording_actions(c,root,mode='race')[0]
    topics={perform_substitutions(c,part) for part in rec.cmd}
    assert {'/state_estimation/yaw_rate','/state_estimation/yaw_acceleration','/state_estimation/yaw_rate/status'} <= topics
    assert (tmp_path/'session/params/yaw_rate.yaml').is_file()
