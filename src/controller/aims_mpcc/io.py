"""Configuration and ROS-independent pose conversion."""
import math
from pathlib import Path
import yaml
from .config import VehicleConfig


def load_config(filename):
    payload=yaml.safe_load(Path(filename).read_text())
    if not isinstance(payload,dict): raise ValueError('vehicle configuration must be a mapping')
    return VehicleConfig(**payload).validate()


def yaw_from_quaternion(q):
    values=[q.x,q.y,q.z,q.w]
    norm=math.sqrt(sum(v*v for v in values))
    if not all(math.isfinite(v) for v in values) or abs(norm-1.)>.01:
        raise ValueError('Invalid orientation quaternion')
    x,y,z,w=[v/norm for v in values]
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
