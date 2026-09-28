"""Planar map/odom alignment for a continuous odom-frame controller."""

import math

from scipy.spatial.transform import Rotation


def planar_alignment(transform):
    """Return map<-odom (x, y, yaw), rejecting a nonplanar localization TF."""
    tf = transform.transform
    q = tf.rotation
    rotation = Rotation.from_quat((q.x, q.y, q.z, q.w))
    matrix = rotation.as_matrix()
    tilt = math.acos(max(-1., min(1., float(matrix[2, 2]))))
    alignment = (float(tf.translation.x), float(tf.translation.y),
                 math.atan2(matrix[1, 0], matrix[0, 0]))
    if tilt > .05 or not all(math.isfinite(value) for value in alignment):
        raise ValueError('Map-to-odom TF is nonplanar or nonfinite')
    return alignment


def apply_alignment(x, y, yaw, alignment):
    """Use one map<-odom snapshot for reference association, not vehicle dynamics."""
    tx, ty, angle = alignment
    c, s = math.cos(angle), math.sin(angle)
    return tx + c * x - s * y, ty + s * x + c * y, yaw + angle
