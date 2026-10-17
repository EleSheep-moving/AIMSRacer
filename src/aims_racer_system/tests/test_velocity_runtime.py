"""Real ROS transport, input-age, epoch and independent graph contracts."""
import importlib.util
import os
from pathlib import Path
import subprocess
import time
import pytest
import rclpy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock

@pytest.fixture
def graph(tmp_path):
    rclpy.init();node=rclpy.create_node('vx_contract_test');output=[];status=[]
    subs=[node.create_subscription(TwistWithCovarianceStamped,'/state_estimation/vx',output.append,1000),node.create_subscription(DiagnosticArray,'/state_estimation/vx/status',status.append,100)]
    pub=node.create_publisher(Imu,'/livox/imu_bias_corrected',256);lio=node.create_publisher(Odometry,'/fastlio2/lio_odom',256);clock=node.create_publisher(Clock,'/clock',10)
    log=(tmp_path/'node.log').open('w');proc=subprocess.Popen([os.environ['VELOCITY_EXECUTABLE'],'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT)
    class G:
        now=10_000_000_000
        def spin(self,d=.018):
            end=time.monotonic()+d
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.002)
        def tick(self,t):
            self.now=t;m=Clock();m.clock.sec,m.clock.nanosec=divmod(t,10**9);clock.publish(m);self.spin()
        def imu(self,t=None,frame='livox_frame',ax=0):
            m=Imu();m.header.frame_id=frame;m.header.stamp.sec,m.header.stamp.nanosec=divmod(self.now if t is None else t,10**9);m.linear_acceleration.z=1.;m.linear_acceleration.x=float(ax);pub.publish(m);self.spin()
        def lio(self,t=None,p=None,frame='odom'):
            t=self.now if t is None else t;m=Odometry();m.header.frame_id=frame;m.child_frame_id='livox_frame';m.header.stamp.sec,m.header.stamp.nanosec=divmod(t,10**9);m.pose.pose.orientation.w=1.;m.pose.pose.position.x=2*(t-10_000_000_000)*1e-9 if p is None else float(p);lio.publish(m);self.spin()
        def ready(self):
            base=self.now
            for i in range(61):
                self.tick(base+i*5_000_000)
                if i%20==0:self.lio()
                self.imu()
        def fields(self):
            self.spin(.12);assert status;return {x.key:x.value for x in status[-1].status[0].values}
    g=G();g.node=node;g.output=output
    try:
        deadline=time.monotonic()+10
        while not all(p.get_subscription_count() for p in [pub,lio,clock]):
            assert time.monotonic()<deadline;g.spin(.05)
        g.tick(g.now);yield g
    finally:
        proc.terminate();proc.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()

def test_output_and_independent_graph(graph):
    g=graph;g.ready();assert g.output;m=g.output[-1]
    assert m.header.frame_id=='base_link' and m.twist.twist.linear.x==pytest.approx(2,abs=1e-8)
    assert m.header.stamp.sec*10**9+m.header.stamp.nanosec==g.now
    assert m.twist.covariance[0]>0 and all(m.twist.covariance[k]>=1e6 for k in [7,14,21,28,35])
    subs={n for n,_ in g.node.get_subscriber_names_and_types_by_node('vx_observer','/')}
    assert subs<= {'/livox/imu_bias_corrected','/fastlio2/lio_odom','/clock','/parameter_events'}
    pubs={n for n,_ in g.node.get_publisher_names_and_types_by_node('vx_observer','/')}
    assert not pubs.intersection({'/tf','/tf_static','/drive','/vesc/command'})
    count=len(g.output);g.lio(t=g.now-50_000_000);assert len(g.output)==count
    stamps=[m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in g.output];assert all(b>a for a,b in zip(stamps,stamps[1:]))

def test_faults_outage_gap_and_clock_reset(graph):
    g=graph;g.ready();count=len(g.output)
    g.imu();g.imu(t=g.now+1);g.imu(frame='bad');g.imu(ax=float('nan'));g.imu(t=g.now-60_000_000);g.lio(p=100)
    assert len(g.output)==count
    f=g.fields()
    for k in ['rejected_imu','future_stamp','wrong_frame','stale_imu','rejected_lio_input']:assert int(f[k])>=1
    # Keep IMU continuous while LIO ages: valid -> degraded -> suppressed.
    base=g.now
    for i in range(1,103):g.tick(base+i*5_000_000);g.imu()
    assert g.fields()['valid']=='false';assert len(g.output)>count
    last=g.output[-1].header.stamp;assert last.sec*10**9+last.nanosec<=base+500_000_000
    count=len(g.output);g.tick(g.now+40_000_000);g.imu();assert len(g.output)==count
    assert g.fields()['valid']=='false';g.tick(g.now-2_000_000_000);g.imu()
    assert int(g.fields()['clock_resets'])==1;assert len(g.output)==count
    g.ready();assert len(g.output)>count

def test_launch_and_recording(monkeypatch,tmp_path):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    root=Path(__file__).resolve().parents[1];c=LaunchContext()
    def load(name):
        spec=importlib.util.spec_from_file_location('vx_'+name.replace('.','_'),root/'launch'/name);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);monkeypatch.setattr(m,'get_package_share_directory',lambda _:str(root));return m
    for name in ['race.launch.py','vehicle.launch.py']:
        defaults={a.name:perform_substitutions(c,a.default_value) for a in load(name).generate_launch_description().entities if isinstance(a,DeclareLaunchArgument) and a.default_value is not None}
        assert defaults['vx_observer']=='false'
    m=load('velocity.launch.py');monkeypatch.setattr(m,'Node',lambda **kw:kw)
    c.launch_configurations.update(geometry_config=str(root/'params/rear_axle_geometry.yaml'),config=str(root/'params/velocity.yaml'),use_sim_time='false')
    n=m.build_node(c)[0];assert n['executable']=='vx_observer' and n['parameters'][1]['livox_translation']==[.3,0,.03]
    from aims_racer_system.launch_support import recording_actions
    c.launch_configurations.update(record='true',session_directory=str(tmp_path/'session'))
    rec=recording_actions(c,root)[0];topics={perform_substitutions(c,p) for p in rec.cmd}
    assert {'/state_estimation/vx','/state_estimation/vx/status','/state_estimation/vx/debug'}<=topics
