"""Gyro calibration must use external stillness and retain real rotation."""
import numpy as np
import pytest
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from aims_racer_system.gyro_bias import GyroBiasCalibrator, correct_imu

BIAS = np.array([.001, -.002, .02])

def feed(c, start=0., end=10.1, speed=0., turning=0., bias=BIAS):
    for t in np.arange(start, end, .005):
        c.add_wheel(t, speed)
        q=Rotation.from_euler('z', turning*t).as_quat()
        c.add_lio(t, [speed*t,0.,0.], q, [speed,0.,0.])
        c.process(t, bias+[0.,0.,turning], [.3,.4,np.sqrt(.75)])
    return c

def test_inclined_stationary_sensor_calibrates_all_axes_and_freezes():
    c=feed(GyroBiasCalibrator())
    assert c.ready
    np.testing.assert_allclose(c.bias, BIAS, atol=1e-12)
    np.testing.assert_allclose(c.process(10.2, BIAS+[.1,.2,.5], [.3,.4,np.sqrt(.75)]), [.1,.2,.5], atol=1e-12)
    np.testing.assert_allclose(c.bias, BIAS, atol=1e-12)

def test_missing_or_future_observations_cannot_calibrate():
    c=GyroBiasCalibrator()
    for t in np.arange(0.,11.,.005):c.process(t,BIAS,[0.,0.,1.])
    assert not c.ready
    c.add_wheel(100.,0.);c.add_lio(100.,[0.,0.,0.],[0.,0.,0.,1.],[0.,0.,0.])
    for t in np.arange(12.,23.,.005):c.process(t,BIAS,[0.,0.,1.])
    assert not c.ready

def test_wheel_movement_and_slow_rotation_prevent_learning_motion_as_bias():
    assert not feed(GyroBiasCalibrator(),speed=.1).ready
    assert not feed(GyroBiasCalibrator(),turning=.003).ready

def test_motion_resets_the_contiguous_static_window():
    c=feed(GyroBiasCalibrator(),end=6.)
    feed(c,start=6.,end=7.,speed=.1)
    feed(c,start=7.,end=16.)
    assert not c.ready
    feed(c,start=16.,end=17.2)
    assert c.ready

def test_clock_rewind_invalidates_bias_and_requires_new_observations():
    c=feed(GyroBiasCalibrator())
    assert c.process(1.,BIAS,[0.,0.,1.]) is None
    assert not c.ready
    assert c.process(1.1,BIAS,[0.,0.,1.]) is None

def test_nonfinite_inputs_and_invalid_parameters_are_rejected():
    c=GyroBiasCalibrator()
    assert c.process(0.,[0.,0.,np.nan],[0.,0.,1.]) is None
    assert not c.ready
    with pytest.raises(ValueError):GyroBiasCalibrator(duration=0.)
    with pytest.raises(ValueError):GyroBiasCalibrator(min_samples=0)

def test_correction_preserves_header_accel_and_unknown_covariance_and_raw_input():
    m=Imu();m.header.frame_id='livox_frame';m.header.stamp.sec=123
    m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=BIAS.tolist()
    m.linear_acceleration.x=.3;m.linear_acceleration.y=.4;m.linear_acceleration.z=.866
    out=correct_imu(m,BIAS)
    assert out.header==m.header and out.linear_acceleration==m.linear_acceleration
    assert list(out.angular_velocity_covariance)==[0.]*9
    assert m.angular_velocity.z==.02 and out.angular_velocity.z==0.
    m.angular_velocity_covariance=[.01,0.,0.,0.,.01,0.,0.,0.,.01]
    assert correct_imu(m,BIAS).angular_velocity_covariance[8] >= .01
    m.angular_velocity_covariance[0]=-1.
    with pytest.raises(ValueError):correct_imu(m,BIAS)


def test_sensor_axis_bias_is_removed_before_rotated_lever_compensation():
    from nav_msgs.msg import Odometry
    from aims_racer_system.rear_axle_odometry import convert_odometry
    mount=Rotation.from_euler('xyz',[.2,-.1,.4])
    omega=np.array([.1,-.2,.7]);velocity=np.array([1.2,.1,0.]);lever=np.array([.3,.04,.03])
    imu=Imu();sensor_omega=mount.inv().apply(omega)+BIAS
    imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=sensor_omega.tolist()
    corrected=correct_imu(imu,BIAS);w=corrected.angular_velocity
    odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='livox_frame'
    q=mount.as_quat().tolist();odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=q
    v=mount.inv().apply(velocity+np.cross(omega,lever))
    odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=v.tolist()
    out=convert_odometry(odom,[w.x,w.y,w.z],lever,q);v=out.twist.twist.linear
    np.testing.assert_allclose([v.x,v.y,v.z],velocity,atol=1e-12)
