"""Rigid-body unified Livox-origin odometry conversion to rear-axle base_link."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _covariance(values, floors):
    c = np.asarray(values, dtype=float).reshape(6, 6)
    if not np.isfinite(c).all():
        raise ValueError('Nonfinite odometry covariance')
    if np.linalg.eigvalsh((c+c.T)/2).min() < -1e-9:
        raise ValueError('Invalid odometry covariance')
    # Upstream FAST-LIO currently supplies zeros: do not interpret them as certainty.
    return (c+c.T)/2 + np.diag(np.maximum(0., np.asarray(floors)-np.diag(c)))


def convert_odometry(msg, angular_velocity, translation, quaternion,
                     pose_variance=(.01, .01, .01, .01, .01, .01),
                     twist_variance=(.04, .04, .04, .01, .01, .01)):
    """Angular velocity is in unified Livox axes; extrinsic is T_base_livox.

    Pose covariance uses world-fixed small rotation perturbations. Twist is
    body-expressed. Gyro covariance floors account for unobserved gyro bias;
    these are conservative assumptions, not calibrated uncertainty estimates.
    """
    if msg.header.frame_id != 'odom' or msg.child_frame_id != 'livox_frame':
        raise ValueError('Expected odom / livox_frame odometry')
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v = msg.twist.twist.linear
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z,
              *angular_velocity, *translation, *quaternion]
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite state or extrinsic')
    q_wi = np.array([q.x, q.y, q.z, q.w])
    if abs(np.linalg.norm(q_wi)-1.) > .01 or abs(np.linalg.norm(quaternion)-1.) > .01:
        raise ValueError('Invalid orientation quaternion')
    r_wi = Rotation.from_quat(q_wi).as_matrix()
    r_bi = Rotation.from_quat(quaternion).as_matrix()
    t_bi = np.asarray(translation)
    r_wb = r_wi @ r_bi.T
    lever_world = r_wb @ t_bi
    position = np.array([p.x, p.y, p.z]) - lever_world
    omega = r_bi @ np.asarray(angular_velocity)
    velocity = r_bi @ np.array([v.x, v.y, v.z]) - np.cross(omega, t_bi)
    out = copy.deepcopy(msg)
    out.child_frame_id = 'base_link'
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position.tolist()
    quat = Rotation.from_matrix(r_wb).as_quat().tolist()
    out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = quat
    out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity.tolist()
    out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = omega.tolist()
    jp = np.eye(6); jp[:3, 3:] = skew(lever_world)
    jt = np.zeros((6, 6)); jt[:3, :3] = r_bi; jt[3:, 3:] = r_bi
    jt[:3, 3:] = skew(t_bi) @ r_bi
    out.pose.covariance = (jp @ _covariance(msg.pose.covariance, pose_variance) @ jp.T).ravel().tolist()
    out.twist.covariance = (jt @ _covariance(msg.twist.covariance, twist_variance) @ jt.T).ravel().tolist()
    return out
