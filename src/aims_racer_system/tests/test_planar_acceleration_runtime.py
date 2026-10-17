"""Opt-in XY/G outputs on the existing delayed-attitude physical reconstruction."""
import importlib.util
from pathlib import Path
import pytest
from geometry_msgs.msg import Vector3Stamped
from test_body_acceleration_runtime import graph, _initialize

@pytest.mark.parametrize('graph',[{'lateral_enabled':True}],indirect=True)
def test_planar_g_covariance_and_fault_behavior(graph):
    g=graph;gs=[]
    sub=g.node.create_subscription(Vector3Stamped,'/state_estimation/body_acceleration/g',gs.append,100)
    try:
        g.spin(.2);_initialize(g,force=(1/9.80665,2/9.80665,1.))
        g.spin(.1);m=g.outputs[-1];assert gs
        assert m.accel.accel.linear.x==pytest.approx(1.)
        assert m.accel.accel.linear.y==pytest.approx(2.)
        assert gs[-1].header==m.header
        assert gs[-1].vector.x==pytest.approx(1/9.80665)
        assert gs[-1].vector.y==pytest.approx(2/9.80665)
        assert m.accel.covariance[7]<1e6 and m.accel.covariance[7]==m.accel.covariance[0]
        assert all(m.accel.covariance[k]==1e6 for k in [14,21,28,35])
        f=g.fields();assert f['x_valid']=='true' and f['y_valid']=='true' and f['z_valid']=='false'
        count=len(gs);g.imu();assert len(gs)==count # duplicate
        g.tick(g.now+5_000_000);g.imu(force=(0.,float('nan'),1.));assert len(gs)==count
        g.tick(g.now+600_000_000);g.imu();assert len(gs)==count
        assert g.fields()['y_valid']=='false'
        pubs={n for n,_ in g.node.get_publisher_names_and_types_by_node('body_acceleration','/')}
        assert not pubs.intersection({'/tf','/drive','/vesc/command','/odometry/filtered'})
    finally:g.node.destroy_subscription(sub)

def test_planar_launch_is_opt_in_and_records_g(monkeypatch,tmp_path):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    root=Path(__file__).resolve().parents[1]
    def load(name):
        spec=importlib.util.spec_from_file_location('planar_'+name.replace('.','_'),root/'launch'/name)
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        monkeypatch.setattr(m,'get_package_share_directory',lambda _:str(root));return m
    c=LaunchContext()
    for name in ['race.launch.py','vehicle.launch.py']:
        d={a.name:perform_substitutions(c,a.default_value) for a in load(name).generate_launch_description().entities if isinstance(a,DeclareLaunchArgument) and a.default_value is not None}
        assert d['planar_acceleration']=='false'
    m=load('planar_acceleration.launch.py');monkeypatch.setattr(m,'Node',lambda **kw:kw)
    c.launch_configurations.update(config=str(root/'params/planar_acceleration.yaml'),geometry_config=str(root/'params/rear_axle_geometry.yaml'),use_sim_time='false')
    n=m.build_node(c)[0];assert n['executable']=='body_acceleration' and n['name']=='planar_acceleration'
    from aims_racer_system.launch_support import recording_actions
    c.launch_configurations.update(record='true',session_directory=str(tmp_path/'session'))
    topics={perform_substitutions(c,x) for x in recording_actions(c,root,mode='race')[0].cmd}
    assert {'/state_estimation/planar_acceleration','/state_estimation/planar_acceleration/g','/state_estimation/planar_acceleration/status','/state_estimation/planar_acceleration/debug'} <= topics
    assert (tmp_path/'session/params/planar_acceleration.yaml').is_file()
    v=load('vehicle.launch.py');monkeypatch.setattr(v,'Node',lambda **kw:kw)
    monkeypatch.setattr(v,'include',lambda pkg,name,arguments=None:{'include':name})
    c.launch_configurations.update(mapping='false',vesc_config='/v',lio_config='/l',mid360_config='/m')
    assert not any(x.get('include')=='planar_acceleration.launch.py' for x in v.build_vehicle(c))
    c.launch_configurations['planar_acceleration']='true'
    assert sum(x.get('include')=='planar_acceleration.launch.py' for x in v.build_vehicle(c))==1
    r=load('race.launch.py');seen=[];monkeypatch.setattr(r,'include',lambda pkg,name,arguments=None:seen.append((name,arguments)) or object())
    f=tmp_path/'map';f.write_text('m');b=tmp_path/'bundle';b.mkdir()
    c.launch_configurations.update(map_file=str(f),artifact_directory=str(b),record='false',auto_start='false',repeat_laps='false',initial_pose='')
    r.build_race(c);assert next(args for name,args in seen if name=='vehicle.launch.py')['planar_acceleration']=='true'
