"""Regression checks for unified Livox mounting and rear-axle compensation."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml
from launch import LaunchContext
from nav_msgs.msg import Odometry

from aims_racer_system.rear_axle_odometry import convert_odometry

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('mount_rpy', [[0., 0., 0.], [.2, -.1, .4]])
def test_turning_sensor_state_recovers_rear_axle(mount_rpy):
    mount = Rotation.from_euler('xyz', mount_rpy)
    attitude = Rotation.from_euler('xyz', [.1, .2, .8])
    lever = np.array([.30, .04, .03])
    position = np.array([2., 3., .4])
    velocity = np.array([1.2, .1, 0.])
    omega = np.array([.1, -.2, .7])
    sensor_position = position + attitude.apply(lever)
    sensor_velocity = mount.inv().apply(velocity + np.cross(omega, lever))
    msg = Odometry()
    msg.header.frame_id = 'odom'; msg.child_frame_id = 'livox_frame'
    msg.header.stamp.sec = 123
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = sensor_position.tolist()
    q = (attitude * mount).as_quat().tolist()
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = sensor_velocity.tolist()
    out = convert_odometry(msg, mount.inv().apply(omega), lever, mount.as_quat())
    p, v, w, q = out.pose.pose.position, out.twist.twist.linear, out.twist.twist.angular, out.pose.pose.orientation
    np.testing.assert_allclose([p.x,p.y,p.z], position, atol=1e-12)
    np.testing.assert_allclose([v.x,v.y,v.z], velocity, atol=1e-12)
    np.testing.assert_allclose([w.x,w.y,w.z], omega, atol=1e-12)
    np.testing.assert_allclose(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(), attitude.as_matrix(), atol=1e-12)
    assert out.child_frame_id == 'base_link' and out.header == msg.header
    assert msg.child_frame_id == 'livox_frame'
    assert np.linalg.eigvalsh(np.array(out.twist.covariance).reshape(6,6)).min() > 0
    msg.child_frame_id = 'livox_imu'
    with pytest.raises(ValueError, match='livox_frame'):
        convert_odometry(msg, omega, lever, mount.as_quat())


@pytest.mark.parametrize('mapping', [False, True])
def test_shared_launch_uses_mount_directly_and_one_livox_frame(mapping):
    spec = importlib.util.spec_from_file_location('rear_frames_launch', PACKAGE/'launch/rear_axle_frames.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.Node = lambda **kwargs: kwargs
    context = LaunchContext()
    context.launch_configurations.update(geometry_config=str(PACKAGE/'params/rear_axle_geometry.yaml'),
        lio_config=str(PACKAGE/'params/fastlio_rear.yaml'), gyro_bias_config=str(PACKAGE/'params/gyro_bias.yaml'), publish_odom_tf=str(mapping).lower())
    nodes = module.build_nodes(context)
    static = [n for n in nodes if n['executable']=='static_transform_publisher']
    assert [n['arguments'][-1] for n in static] == ['livox_frame', 'base_footprint']
    assert [float(static[0]['arguments'][i]) for i in (1,3,5)] == [.3,0.,.03]
    adapters = [n for n in nodes if n['executable'] in ['lio_to_rear_axle.py', 'imu_to_rear_axle']]
    for node in adapters:
        assert node['parameters'][0]['livox_translation'] == [.3,0.,.03]
    assert adapters[0]['parameters'][0]['publish_tf'] is mapping
    lio = yaml.safe_load((PACKAGE/'params/fastlio_rear.yaml').read_text())
    assert lio['t_il'] == [-.011,-.02329,.04412]  # Preserved internally, not added to mounting.
    assert lio['r_il'] == [1.,0.,0.,0.,1.,0.,0.,0.,1.]


def test_gyro_correction_is_shared_by_both_rear_adapters():
    spec = importlib.util.spec_from_file_location('rear_bias_launch', PACKAGE/'launch/rear_axle_frames.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.Node = lambda **kwargs: kwargs
    context = LaunchContext()
    context.launch_configurations.update(geometry_config=str(PACKAGE/'params/rear_axle_geometry.yaml'),
        lio_config=str(PACKAGE/'params/fastlio_rear.yaml'), gyro_bias_config=str(PACKAGE/'params/gyro_bias.yaml'),
        publish_odom_tf='false')
    nodes=module.build_nodes(context)
    correctors=[n for n in nodes if n['executable']=='livox_gyro_bias.py']
    assert len(correctors)==1
    adapters=[n for n in nodes if n['executable'] in ['lio_to_rear_axle.py','imu_to_rear_axle']]
    assert len(adapters)==2
    for n in adapters:
        assert ('/livox/imu','/livox/imu_bias_corrected') in n['remappings']
