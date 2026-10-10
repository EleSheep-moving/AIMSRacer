import csv
import hashlib
import math

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

from aims_mpcc.config import VehicleConfig
from aims_mpcc.frames import apply_alignment, planar_alignment
from aims_mpcc.path import ReferencePath, prepare_recording


def test_map_alignment_applies_translation_and_rotation_to_rear_axle():
    transform = TransformStamped()
    transform.transform.translation.x = 10.
    transform.transform.translation.y = -3.
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    alignment = planar_alignment(transform)
    x, y, yaw = apply_alignment(1., 2., math.pi / 6, alignment)
    assert (x, y, yaw) == pytest.approx((8., -2., math.radians(120)))
    transform.transform.rotation.x = math.sin(.1)
    transform.transform.rotation.z = 0.
    transform.transform.rotation.w = math.cos(.1)
    assert planar_alignment(transform) == pytest.approx((10., -3., 0.))
    transform.transform.translation.x = math.nan
    with pytest.raises(ValueError, match='nonfinite'):
        planar_alignment(transform)


def test_map_reference_stores_exact_map_identity(tmp_path):
    pcd = tmp_path / 'map.pcd'
    pcd.write_bytes(b'one specific PGO map')
    csv_path = tmp_path / 'lap.csv'
    with csv_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('timestamp', 'x', 'y', 'yaw', 'speed', 'frame_id', 'child_frame_id'))
        for i in range(41):
            angle = 2 * math.pi * i / 40
            writer.writerow((i * .3, 2 * math.cos(angle), 2 * math.sin(angle),
                             angle + math.pi / 2, 1., 'map', 'base_link'))
    config = VehicleConfig(rear_offset=0., front_extent=.52, rear_extent=.10,
                           half_width=.16,
                           geometry_verified=True, max_speed=1.5,
                           steer_limit=.45)
    with pytest.raises(ValueError, match='requires --map-file'):
        prepare_recording(csv_path, tmp_path / 'missing', config, .5, .5)
    path = prepare_recording(csv_path, tmp_path / 'prepared', config, .5, .5,
                             map_file=pcd)
    assert path.frame_id == 'map'
    assert path.metadata['map_sha256'] == hashlib.sha256(pcd.read_bytes()).hexdigest()
    loaded = ReferencePath.load(tmp_path / 'prepared')
    loaded.validate_config(config, require_recording=True)
    assert loaded.frame_id == 'map'
    assert loaded.left_width == loaded.right_width == .5
    assert loaded.metadata['vehicle_geometry']['front_extent'] == .52
    assert loaded.metadata['vehicle_geometry']['rear_extent'] == .10
    with pytest.raises(ValueError, match='front_extent'):
        loaded.validate_config(VehicleConfig(rear_offset=0., front_extent=.51,
                                             rear_extent=.10, half_width=.16,
                                             geometry_verified=True))
