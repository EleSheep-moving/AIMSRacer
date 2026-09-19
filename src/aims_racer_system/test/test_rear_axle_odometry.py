import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from aims_racer_system.rear_axle_odometry import convert_odometry, sensor_extrinsic
from nav_msgs.msg import Odometry


def message(yaw=0.):
    m = Odometry()
    m.header.frame_id = 'odom'
    m.child_frame_id = 'livox_imu'
    m.header.stamp.sec = 10
    q = Rotation.from_euler('z', yaw).as_quat()
    m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
    return m


def test_turning_sensor_velocity_is_removed():
    m = message()
    m.pose.pose.position.x = .3
    m.twist.twist.linear.x = 1.
    m.twist.twist.linear.y = .6  # rear speed 1, yaw rate 2, lever arm .3
    out = convert_odometry(m, [0., 0., 2.], [.3, 0., 0.], [0., 0., 0., 1.])
    assert out.pose.pose.position.x == pytest.approx(0.)
    assert out.twist.twist.linear.x == pytest.approx(1.)
    assert out.twist.twist.linear.y == pytest.approx(0.)
    assert out.twist.twist.angular.z == pytest.approx(2.)
    assert out.header == m.header and out.child_frame_id == 'base_link'
    assert m.twist.twist.linear.y == .6
    assert np.linalg.eigvalsh(np.array(out.twist.covariance).reshape(6, 6)).min() > 0


def test_lever_arm_rotates_with_world_heading():
    m = message(np.pi/2)
    m.pose.pose.position.y = .3
    out = convert_odometry(m, [0., 0., 0.], [.3, 0., 0.], [0., 0., 0., 1.])
    assert out.pose.pose.position.y == pytest.approx(0.)
    assert out.pose.pose.position.x == pytest.approx(0., abs=1e-12)


def test_upside_down_imu_and_lidar_extrinsic():
    q = Rotation.from_euler('x', np.pi).as_quat()
    t, qi = sensor_extrinsic([.3, 0., .03], q, np.eye(3), [-.011, -.02329, .04412])
    assert t == pytest.approx([.311, -.02329, .07412])
    m = message()
    m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
    out = convert_odometry(m, [0., 0., -1.], t, qi)
    assert out.pose.pose.orientation.w == pytest.approx(1.)
    assert out.twist.twist.angular.z == pytest.approx(1.)


@pytest.mark.parametrize('frame', ['base_link', 'laser', ''])
def test_wrong_input_frame_rejected(frame):
    m = message(); m.child_frame_id = frame
    with pytest.raises(ValueError):
        convert_odometry(m, [0., 0., 0.], [.3, 0., 0.], [0., 0., 0., 1.])


def test_nan_and_invalid_quaternion_rejected():
    m = message(); m.twist.twist.linear.x = float('nan')
    with pytest.raises(ValueError):
        convert_odometry(m, [0., 0., 0.], [.3, 0., 0.], [0., 0., 0., 1.])


def test_ekf_virtual_frame_undoes_helper_rotation_once():
    raw = np.array([.1, .2, 1.])
    helper = Rotation.from_euler('x', np.pi).as_matrix()
    base_raw = Rotation.from_euler('xyz', [.2, -.1, .3]).as_matrix()
    base_corrected = base_raw @ helper.T
    assert base_corrected @ (helper @ raw) == pytest.approx(base_raw @ raw)


def test_bad_extrinsic_rotation_rejected():
    with pytest.raises(ValueError):
        sensor_extrinsic([.3, 0., .03], [0., 0., 0., 1.], np.zeros((3, 3)), [0., 0., 0.])
