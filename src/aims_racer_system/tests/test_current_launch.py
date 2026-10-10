"""Public launch contracts for the sole field vehicle/control stack."""
import importlib.util
from pathlib import Path
import sys
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.utilities import perform_substitutions
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

@pytest.fixture(autouse=True)
def package_lookup(monkeypatch):
    import aims_racer_system.launch_support as support
    monkeypatch.setattr(support,'get_package_share_directory',lambda _:str(ROOT))

def load(name):
 p=ROOT/'launch'/name
 spec=importlib.util.spec_from_file_location(name.replace('.','_'),p)
 m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 m.get_package_share_directory=lambda _:str(ROOT)
 return m

def defaults(name):
 c=LaunchContext();d=load(name).generate_launch_description()
 return {a.name:''.join(s.perform(c) for s in a.default_value) for a in d.entities if isinstance(a,DeclareLaunchArgument) and a.default_value is not None}

def test_race_defaults_do_not_record_or_enable_or_choose_a_backend():
 d=defaults('race.launch.py')
 assert d['record']=='false' and d['auto_start']=='false'
 assert d['repeat_laps']=='false'
 assert not {'implementation','backend','shadow','cpp_solve_frequency'} & d.keys()

def test_recording_retains_reliable_imu_and_all_replay_inputs(tmp_path):
 from aims_racer_system.launch_support import recording_actions
 c=LaunchContext();c.launch_configurations.update(record='true',session_directory=str(tmp_path/'new-session'))
 actions=recording_actions(c,ROOT,mode='race')
 record=next(a for a in actions if isinstance(a,ExecuteProcess))
 cmd=[perform_substitutions(c,item) for item in record.cmd]
 assert cmd[:3]==['ros2','bag','record']
 assert '--qos-profile-overrides-path' in cmd
 assert {'/livox/lidar','/livox/imu','/odometry/filtered','/mpcc/status','/localization/anchor_status','/tf_static'} <= set(cmd)
 assert not any('localizer/raw_tf' in item for item in cmd)
 assert (tmp_path/'new-session/session.json').is_file()
 with pytest.raises(FileExistsError):recording_actions(c,ROOT,mode='race')

def test_no_recording_does_not_create_a_session(tmp_path):
 from aims_racer_system.launch_support import recording_actions
 c=LaunchContext();c.launch_configurations.update(record='false',session_directory=str(tmp_path/'unused'))
 assert recording_actions(c,ROOT,mode='race')==[]
 assert not (tmp_path/'unused').exists()

def test_vehicle_preserves_single_ekf_tf_owner_and_no_old_localizer():
 m=load('vehicle.launch.py');c=LaunchContext()
 c.launch_configurations.update(mapping='false',vesc_config='/vesc',lio_config='/lio',mid360_config='/livox')
 nodes=m.build_vehicle(c)
 packages=[n.node_package for n in nodes if isinstance(n,Node)]
 assert packages.count('fastlio2')==1 and packages.count('robot_localization')==1
 assert 'localizer' not in packages
 c.launch_configurations['mapping']='true'
 mapping=m.build_vehicle(c)
 packages=[n.node_package for n in mapping if isinstance(n,Node)]
 assert 'robot_localization' not in packages and packages.count('fastlio2')==1

def test_mapping_only_adds_pgo_and_never_mpcc():
 names=defaults('mapping.launch.py')
 assert names['record']=='false'
 c=LaunchContext();c.launch_configurations.update(record='false',session_directory='',pgo_config='/pgo')
 actions=load('mapping.launch.py').build_mapping(c)
 packages=[n.node_package for n in actions if isinstance(n,Node)]
 assert packages==['pgo']
 assert any(isinstance(a,IncludeLaunchDescription) for a in actions)

def test_bad_initial_pose_fails_before_creating_recording(tmp_path):
 map_file=tmp_path/'map.pcd';map_file.write_text('map')
 bundle=tmp_path/'bundle';bundle.mkdir()
 c=LaunchContext();c.launch_configurations.update(map_file=str(map_file),artifact_directory=str(bundle),
     auto_start='false',repeat_laps='false',record='true',session_directory=str(tmp_path/'invalid-session'),
     initial_pose='0 0 nan 0 0 0',log_directory='')
 with pytest.raises(ValueError,match='six finite'):load('race.launch.py').build_race(c)
 assert not (tmp_path/'invalid-session').exists()

def test_native_launch_defaults_match_last_field_request_timing(tmp_path):
 source=ROOT.parents[0]/'aims_mpcc_rt/launch/mpcc.launch.py'
 spec=importlib.util.spec_from_file_location('native_launch',source)
 m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 c=LaunchContext();d=m.generate_launch_description()
 args={a.name:''.join(s.perform(c) for s in a.default_value) for a in d.entities if isinstance(a,DeclareLaunchArgument) and a.default_value is not None}
 assert args['solve_frequency']=='20.0' and args['solver_timeout']=='0.05'
 assert args['handover_delay']=='0.02' and args['odom_topic']=='/odometry/filtered'
 assert args['auto_start']=='false' and args['simulation']=='false'
 args['artifact_directory']=str(tmp_path);c.launch_configurations.update(args)
 nodes=m.build_controller(c)
 assert len(nodes)==1 and isinstance(nodes[0],Node)
 assert nodes[0].node_package=='aims_mpcc_rt'

def test_fastlio_namespace_and_rc_parameters_match_field_bringup():
 m=load('vehicle.launch.py');m.Node=lambda **kw:kw
 c=LaunchContext();c.launch_configurations.update(mapping='false',vesc_config='/vesc',lio_config='/lio',mid360_config='/livox')
 nodes=[a for a in m.build_vehicle(c) if isinstance(a,dict)]
 lio=next(n for n in nodes if n['package']=='fastlio2')
 assert lio.get('namespace')=='fastlio2'
 selector=next(n for n in nodes if n['executable']=='joystick_control_v2')['parameters'][0]
 assert selector['current_limit_max_current']==100.0 and selector['channel_deadzone']==50
