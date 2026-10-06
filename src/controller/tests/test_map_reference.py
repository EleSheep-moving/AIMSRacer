import csv
import hashlib
import math
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

from aims_mpcc.config import VehicleConfig
from aims_mpcc.frames import apply_alignment, planar_alignment
from aims_mpcc.node import MPCCNode
from aims_mpcc.path import ReferencePath, prepare_recording
from aims_mpcc.solver import MPCCSolver


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


def test_map_solver_keeps_odom_dynamics_under_alignment_change():
    angles = np.arange(32) * 2 * math.pi / 32
    path = ReferencePath(np.c_[2 * np.cos(angles), 2 * np.sin(angles)],
                         1., 1., frame_id='map', metadata={'map_sha256': '0' * 64})
    config = VehicleConfig(rear_offset=0., front_extent=.52, rear_extent=.10,
                           half_width=.16,
                           geometry_verified=True, max_speed=1.5,
                           steer_limit=.45, steer_rate=2.)
    solver = MPCCSolver(path, config, horizon=2, dt=.1)
    assert solver.corner_offsets == [(.52, .16), (.52, -.16), (-.10, .16), (-.10, -.16)]
    previous = dict(acceleration=0., steering=0., steering_rate=0.)
    state_map = dict(x=2., y=0., yaw=math.pi / 2, speed=.3, steering=.1)
    first = solver.solve(state_map, previous, [.5] * 3,
                         map_alignment=(0., 0., 0.))
    assert first['success'], first['status']
    solver.reset()
    alignment = (5., -2., .3)
    c, s = math.cos(alignment[2]), math.sin(alignment[2])
    state_odom = dict(x=c * (2. - alignment[0]) + s * (0. - alignment[1]),
                      y=-s * (2. - alignment[0]) + c * (0. - alignment[1]),
                      yaw=state_map['yaw'] - alignment[2], speed=.3, steering=.1)
    second = solver.solve(state_odom, previous, [.5] * 3,
                          map_alignment=alignment)
    assert second['success'], second['status']
    np.testing.assert_allclose(second['controls'], first['controls'], atol=1e-3)
    second_in_map = np.asarray([apply_alignment(row[0], row[1], row[2], alignment)
                                for row in second['states']])
    np.testing.assert_allclose(second_in_map, np.asarray(first['states'])[:, :3], atol=1e-3)


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


def test_reference_identity_without_localization_health_inputs():
    fake = SimpleNamespace(map_sha256='right',
                           path=SimpleNamespace(metadata={'map_sha256': 'right'}))
    assert MPCCNode.map_matches_reference(fake)
    fake.map_sha256 = 'wrong'
    assert not MPCCNode.map_matches_reference(fake)
    fake.map_sha256 = 'right'
    assert MPCCNode.map_matches_reference(fake)
    fake.map_sha256 = None
    assert not MPCCNode.map_matches_reference(fake)


def test_front_overhang_blocks_a_turn_in_the_one_metre_corridor():
    config = VehicleConfig(rear_offset=0., front_extent=.52, rear_extent=.10,
                           half_width=.16, geometry_verified=True)
    fake = SimpleNamespace(config=config, path=SimpleNamespace(left_width=.5,
                                                               right_width=.5))
    reference = dict(x=0., y=0., yaw=0.)
    assert MPCCNode.footprint_inside(fake, SimpleNamespace(x=0., y=0., yaw=0.),
                                     reference)
    assert not MPCCNode.footprint_inside(fake,
                                         SimpleNamespace(x=0., y=0., yaw=math.pi / 3),
                                         reference)
