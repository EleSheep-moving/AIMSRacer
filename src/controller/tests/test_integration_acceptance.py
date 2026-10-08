"""Acceptance follows control authority and bounded worker recovery contracts."""
import pytest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from aims_mpcc import integration
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath
from aims_mpcc.config import VehicleConfig


def metrics(status='COMPLETE'):
    return dict(status_at_end=status, maximum_command_speed_mps=.5,
                maximum_command_steering_rad=.2, minimum_footprint_margin_m=.3,
                cross_track_rms_m=.02, cross_track_max_m=.04,
                finish_error_m=.03, final_speed_mps=.001,
                lap_progress_m=12.5, reference_length_m=12.56)


def test_annular_metric_accepts_field_asymmetric_geometry():
    config_file=Path(integration.__file__).parents[1]/'config/experimental_native_rti2.yaml'
    config=load_config(config_file)
    assert config.half_length is None
    offsets=config.longitudinal_offsets()
    assert offsets==(.52,-.10)
    actual=integration.annular_footprint_margins(2.,0.,0.,offsets,config.half_width)
    expected=[.9-abs(np.hypot(2.+along,across)-2.)
              for along in (-.10,.52) for across in (-.16,.16)]
    assert actual==pytest.approx(expected,abs=1e-15)


@pytest.mark.parametrize('seed',range(20))
def test_symmetric_annular_metric_matches_original_corner_arithmetic_exactly(seed):
    import math
    rng=np.random.default_rng(seed)
    config=VehicleConfig(profile='synthetic',rear_offset=.18,half_length=.5,
                         half_width=.3,geometry_verified=True)
    offsets=config.longitudinal_offsets()
    for _ in range(5):
        x,y=rng.uniform(-3.,3.,2);yaw=float(rng.uniform(-np.pi,np.pi))
        c,sn=math.cos(yaw),math.sin(yaw);expected=[]
        for sx in (-1,1):
            for sy in (-1,1):
                along=config.rear_offset+sx*config.half_length;across=sy*config.half_width
                corner_x=x+along*c-across*sn;corner_y=y+along*sn+across*c
                expected.append(.9-abs(math.hypot(corner_x,corner_y)-2.))
        actual=integration.annular_footprint_margins(x,y,yaw,offsets,config.half_width)
        assert actual==expected


def test_nominal_recovery_stop_reports_cause_without_waiting_for_timeout():
    with pytest.raises(RuntimeError, match='Recovery stopped; re-enable required'):
        integration.check_nominal_status('READY', 'Recovery stopped; re-enable required')


@pytest.mark.parametrize('scenario', ['manual', 'rc_loss'])
def test_authority_withdrawal_requires_latched_fault_and_zero_output(scenario):
    result=metrics('FAULT')
    result['fault_detection_s']=.1
    integration.check_scenario_acceptance(result,scenario,1.,.4,0.,0.)
    result['status_at_end']='RUNNING'
    with pytest.raises(AssertionError):
        integration.check_scenario_acceptance(result,scenario,1.,.4,0.,0.)


@pytest.mark.parametrize('scenario', ['solver_stall', 'solver_crash'])
def test_worker_recovery_accepts_moving_resume_or_latched_stop(scenario):
    result=metrics('RUNNING')
    result.update(worker_restarts=1,worker_ready=True,recovery_observed=True,
                  post_restart_activations=2,final_speed_mps=.2)
    integration.check_scenario_acceptance(result,scenario,1.,.4,.2,.2)
    result['post_restart_activations']=1
    with pytest.raises(AssertionError):
        integration.check_scenario_acceptance(result,scenario,1.,.4,.2,.2)
    result.update(status_at_end='READY',final_speed_mps=.001,
                  reason_at_end='Recovery stopped; re-enable required',
                  stopped_latch_duration_s=.6)
    integration.check_scenario_acceptance(result,scenario,1.,.4,0.,0.)


def test_nominal_geometry_and_finish_limits_remain_strict():
    result=metrics()
    integration.check_scenario_acceptance(result,'nominal',1.,.4,0.,0.)
    result['minimum_footprint_margin_m']=-.001
    with pytest.raises(AssertionError):
        integration.check_scenario_acceptance(result,'nominal',1.,.4,0.,0.)


def test_speed_attainment_includes_slow_recovery_samples_in_central_lap():
    fractions=np.arange(10)/10
    angles=2*np.pi*fractions
    speed=np.array([0.,.4,.5,.6,.1,.2,.3,.7,1.,0.])
    samples=np.c_[fractions,2*np.cos(angles),2*np.sin(angles),speed,
                  np.zeros((10,3))]
    report=integration.speed_attainment(samples,4*np.pi,1.,(2.,0.))
    assert report['central_sample_count']==7
    assert report['central_speed_mean_mps']==pytest.approx(.4)
    assert report['central_speed_p50_mps']==pytest.approx(.4)
    assert report['central_speed_p50_ratio']==pytest.approx(.4)
    assert not report['speed_tracking_pass']


def test_speed_attainment_uses_requested_speed_and_rejects_missing_central_motion():
    fractions=np.linspace(0.,.9,100)
    angles=2*np.pi*fractions
    samples=np.c_[fractions,2*np.cos(angles),2*np.sin(angles),
                  np.full(100,.45),np.zeros((100,3))]
    report=integration.speed_attainment(samples,4*np.pi,.5,(2.,0.))
    assert report['central_speed_p50_ratio']==pytest.approx(.9)
    assert report['speed_tracking_pass']
    report=integration.speed_attainment(samples[:5],4*np.pi,.5,(2.,0.))
    assert report['central_sample_count']==0
    assert report['central_speed_p50_mps'] is None
    assert not report['speed_tracking_pass']


@pytest.mark.parametrize('horizon', [10,15,20])
@pytest.mark.parametrize('prefix', ['', '/mpcc_shadow'])
def test_same_horizon_reaches_offline_cache_and_online_ros_node(horizon,prefix,tmp_path,monkeypatch):
    from aims_mpcc import prepare_solver
    config_file=Path(integration.__file__).parents[1]/'config/synthetic.yaml'
    config=load_config(config_file)
    angle=np.arange(16)*2*np.pi/16
    geometry={key:getattr(config,key) for key in ('wheelbase','rear_offset','half_width',
                                                'half_length','front_extent','rear_extent')}
    path=ReferencePath(np.c_[2*np.cos(angle),2*np.sin(angle)],.9,.9,
                       metadata=dict(closed_lap=True,vehicle_geometry=geometry))
    path.save(tmp_path/'reference')
    args=SimpleNamespace(output=str(tmp_path),vehicle_config=str(config_file),
                         reuse_prepared_reference=True,prepare_only=True,
                         backend='qp',solve_frequency=20.,horizon=horizon,topic_prefix=prefix)
    prepared=[]
    monkeypatch.setattr(prepare_solver,'main',prepared.append)
    integration.run(args)
    assert prepared[0][prepared[0].index('--horizon')+1]==str(horizon)
    ros_arguments=[]
    def capture_init(*,args):
        ros_arguments.extend(args)
        raise RuntimeError('ROS arguments captured')
    monkeypatch.setattr(integration.rclpy,'init',capture_init)
    args.prepare_only=False
    with pytest.raises(RuntimeError,match='ROS arguments captured'):
        integration.run(args)
    assert f'horizon:={horizon}' in ros_arguments
    if prefix:
        assert ros_arguments[-len(integration.topic_remap_arguments(prefix)):]==integration.topic_remap_arguments(prefix)
    else:
        assert '-r' not in ros_arguments


def test_shadow_remaps_cover_all_command_routes_and_auxiliary_entities():
    remaps=integration.topic_remap_arguments('/mpcc_shadow')
    mapping=dict(rule.split(':=') for rule in remaps[1::2])
    topics={'/drive','/ackermann_cmd','/commands/motor/speed',
            '/commands/motor/current','/commands/motor/duty_cycle',
            '/commands/servo/position','/rc/channels','/control/autonomy_speed_enabled',
            '/odometry/filtered','/calib/ackermann_cmd','/sensors/core',
            '/mpcc/status','/mpcc/reference','/mpcc/prediction','/mpcc/enable',
            '/localization/map_sha256','/localization/status','/tf','/tf_static'}
    assert topics<=mapping.keys()
    assert all(mapping[topic]=='/mpcc_shadow'+topic for topic in topics)
    assert remaps[::2]==['-r']*len(mapping)
    for command in integration.child_commands('vesc.yaml',remaps):
        assert command[1][-len(remaps):]==remaps
        assert command[1].count('--ros-args')==1


def test_empty_prefix_preserves_original_child_arguments():
    assert integration.topic_remap_arguments('')==[]
    assert integration.child_commands('vesc.yaml',[])==[
        ('rc',['ros2','run','ackermann_mux','joystick_control_v2','--ros-args',
               '-p','channel_profile:=steering_ch1_throttle_ch3_aux_ch5_to_ch10']),
        ('converter',['ros2','run','vesc_ackermann','ackermann_to_vesc_node',
                      '--ros-args','--params-file','vesc.yaml'])]


@pytest.mark.parametrize('prefix',['/','shadow','/shadow/','/shadow//a',
                                   '/shadow-invalid','/3shadow','/shadow:=/drive'])
def test_invalid_prefix_is_rejected_before_ros_start(prefix):
    with pytest.raises(ValueError,match='namespace'):
        integration.topic_remap_arguments(prefix)


def test_isolation_checks_raw_graph_and_requires_complete_shadow_route():
    class Graph:
        def __init__(self):self.counts={};self.queries=[]
        def count_publishers(self,topic):
            self.queries.append(topic)
            return self.counts.get(topic,0)
    graph=Graph()
    report=integration.shadow_graph_report(graph,'/mpcc_shadow')
    assert not report['shadow_route_ready']
    assert report['unprefixed_driving_publishers_zero']
    assert all(count==0 for count in report['unprefixed_driving_publisher_counts'].values())
    for topic in report['shadow_driving_publisher_counts']:
        graph.counts[topic]=1
    assert integration.shadow_graph_report(graph,'/mpcc_shadow')['shadow_route_ready']
    graph.counts['/commands/motor/current']=1
    assert not integration.shadow_graph_report(graph,'/mpcc_shadow')['unprefixed_driving_publishers_zero']


def test_harness_authority_graph_query_applies_ros_topic_resolution(monkeypatch):
    queries=[]
    monkeypatch.setattr(integration.Node,'count_publishers',lambda self,topic:queries.append(topic) or 1)
    node=object.__new__(integration.ShadowMPCCNode)
    monkeypatch.setattr(integration.ShadowMPCCNode,'resolve_topic_name',
                        lambda self,topic:'/mpcc_shadow'+topic)
    assert node.count_publishers('/drive')==1
    assert queries==['/mpcc_shadow/drive']


@pytest.mark.parametrize('appears_after_first_audit',[False,True])
def test_unprefixed_publisher_blocks_enable_and_records_failed_graph(tmp_path,monkeypatch,appears_after_first_audit):
    """Even a ready worker and enable service cannot bypass the raw graph gate."""
    import json
    enable_requests=[]
    client=SimpleNamespace(service_is_ready=lambda:True,
                           call_async=lambda request:enable_requests.append(request))
    controller=SimpleNamespace(worker=SimpleNamespace(ready=True),
                               supervisor=SimpleNamespace(fresh=lambda now:True,status='READY'),
                               destroy_node=lambda:None)
    queries=[]
    def count_publishers(topic):
        queries.append(topic)
        if topic.startswith('/mpcc_shadow/'):return 1
        return int(topic=='/commands/motor/current' and
                   (not appears_after_first_audit or len(queries)>12))
    plant=SimpleNamespace(samples=[],destroy_node=lambda:None,
                          create_client=lambda *args:client,
                          count_publishers=count_publishers)
    executor=SimpleNamespace(add_node=lambda node:None,spin_once=lambda **kwargs:None)
    monkeypatch.setattr(integration,'fixture',lambda *args:object())
    monkeypatch.setattr(integration,'child_commands',lambda *args:[])
    monkeypatch.setattr(integration,'ShadowMPCCNode',lambda:controller)
    monkeypatch.setattr(integration,'Plant',lambda *args:plant)
    monkeypatch.setattr(integration,'SingleThreadedExecutor',lambda:executor)
    monkeypatch.setattr(integration.rclpy,'init',lambda **kwargs:None)
    monkeypatch.setattr(integration.rclpy,'ok',lambda:False)
    args=SimpleNamespace(output=str(tmp_path),topic_prefix='/mpcc_shadow',
        vehicle_config=str(Path(integration.__file__).parents[1]/'config/synthetic.yaml'),
        reuse_prepared_reference=False,prepare_only=False,clockwise=False,backend='qp',
        horizon=10,solve_frequency=20.,scenario='nominal',vesc_config='unused',
        speed_tau=.2,steer_tau=.15,odom_delay=0.,timeout=1.)
    with pytest.raises(RuntimeError,match='Unprefixed driving publisher'):
        integration.run(args)
    assert not enable_requests
    result=json.loads((tmp_path/'result.json').read_text())
    assert result['status']=='FAIL'
    assert result['unprefixed_driving_publisher_counts']['/commands/motor/current']==1
    assert not result['unprefixed_driving_publishers_zero']
