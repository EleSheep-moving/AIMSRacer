"""Project map/odom alignment into the continuous planar controller frame."""

import math

from scipy.spatial.transform import Rotation


def planar_alignment(transform):
    """Return map<-odom (x, y, yaw) without a roll/pitch rejection gate."""
    tf = transform.transform
    q = tf.rotation
    rotation = Rotation.from_quat((q.x, q.y, q.z, q.w))
    matrix = rotation.as_matrix()
    alignment = (float(tf.translation.x), float(tf.translation.y),
                 math.atan2(matrix[1, 0], matrix[0, 0]))
    if not all(math.isfinite(value) for value in alignment):
        raise ValueError('Map-to-odom TF is nonfinite')
    return alignment


def apply_alignment(x, y, yaw, alignment):
    """Use one map<-odom snapshot for reference association, not vehicle dynamics."""
    tx, ty, angle = alignment
    c, s = math.cos(angle), math.sin(angle)
    return tx + c * x - s * y, ty + s * x + c * y, yaw + angle
