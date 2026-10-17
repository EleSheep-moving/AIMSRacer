import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def core():
    p = Path(__file__).with_name('reference.py')
    assert p.exists(), 'Vicon five-state reference implementation is missing'
    spec = importlib.util.spec_from_file_location('vicon_reference', p)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def configuration():
    return dict(clock=dict(source_anchor_ns=1_800_000_000_000_000_000,
                           ros_anchor_ns=1_800_000_000_000_000_123,
                           scale=1., uncertainty_ns=1_000_000, verified=True),
                vicon=dict(position_unit='m', tracking_semantics='valid field supplied'),
                body_orientation_in_marker_xyzw=[0., 0., 0., 1.],
                references=dict(rear_axle=dict(origin_in_marker_m=[.3, -.1, .07], measured=True),
                                center_of_mass=dict(origin_in_marker_m=[.51, -.13, .07], measured=False)),
                differentiation=dict(windows_s=[.08, .12, .18], primary_window_s=.12,
                                     polyorder=3, max_gap_s=.025, max_rotation_span_rad=2.8),
                comparison=dict(estimate_max_gap_s=.04, lag_limit_s=.4, lag_step_s=.005,
                                min_dynamic_std=dict(vx=.05, vy=.03, ax=.2, ay=.2, yaw_rate=.05)),
                evaluation_windows=[])


def motion():
    c = configuration(); t = np.arange(601)*.01
    omega = np.array([.3, -.2, .7])
    q = Rotation.from_euler('xyz', [.4, -.3, .2])*Rotation.from_rotvec(t[:, None]*omega)
    p = np.column_stack([.1*t**3, .2*t**2+.3*t, .1*t])
    v = np.column_stack([.3*t**2, .4*t+.3, np.full(len(t), .1)])
    a = np.column_stack([.6*t, np.full(len(t), .4), np.zeros(len(t))])
    lever = np.asarray(c['references']['rear_axle']['origin_in_marker_m'])
    poses = dict(source_ns=c['clock']['source_anchor_ns']+np.rint(t*1e9).astype(np.int64),
                 receive_ns=c['clock']['ros_anchor_ns']+np.rint(t*1e9).astype(np.int64)+2_000_000,
                 position=p-q.apply(lever), quaternion=q.as_quat(), valid=np.ones(len(t), bool))
    return c, poses, t, q, v, a, omega


def test_three_dimensional_reference_point_before_differentiation():
    m = core(); c, poses, t, q, v, a, omega = motion()
    result = m.build_reference(poses, c, .12)
    ns = m.map_time(poses['source_ns'], c['clock'])
    rear = m.sample_reference(result['rear_axle'], ns)
    cg = m.sample_reference(result['center_of_mass'], ns)
    valid = np.isfinite(rear).all(axis=1)
    expected_v, expected_a = q.inv().apply(v), q.inv().apply(a)
    np.testing.assert_allclose(rear[valid, :2], expected_v[valid, :2], atol=2e-6)
    np.testing.assert_allclose(rear[valid, 2:4], expected_a[valid, :2], atol=3e-5)
    np.testing.assert_allclose(rear[valid, 4], omega[2], atol=1e-10)
    d = np.asarray(c['references']['center_of_mass']['origin_in_marker_m'])-np.asarray(c['references']['rear_axle']['origin_in_marker_m'])
    np.testing.assert_allclose(cg[valid, :2]-rear[valid, :2], np.broadcast_to(np.cross(omega, d)[:2], rear[valid, :2].shape), atol=3e-6)
    np.testing.assert_allclose(cg[valid, 2:4]-rear[valid, 2:4], np.broadcast_to(np.cross(omega, np.cross(omega, d))[:2], rear[valid, 2:4].shape), atol=4e-5)
    assert not np.isfinite(rear[:5]).any() and not np.isfinite(rear[-5:]).any()


def test_mount_rotation_and_quaternion_sign_are_handled():
    m = core(); c, poses, t, q, v, a, omega = motion()
    mount = Rotation.from_euler('xyz', [.2, .1, -.4])
    # New marker axes, same physical body and target origins.
    poses['quaternion'] = (q*mount.inv()).as_quat()
    poses['quaternion'][::2] *= -1
    c['body_orientation_in_marker_xyzw'] = mount.as_quat().tolist()
    for point in c['references'].values():
        point['origin_in_marker_m'] = mount.apply(point['origin_in_marker_m']).tolist()
    result = m.build_reference(poses, c, .12)
    rear = m.sample_reference(result['rear_axle'], m.map_time(poses['source_ns'], c['clock']))
    good = np.isfinite(rear).all(axis=1)
    np.testing.assert_allclose(rear[good, :2], q.inv().apply(v)[good, :2], atol=2e-6)
    np.testing.assert_allclose(rear[good, 4], omega[2], atol=1e-10)


def test_invalid_tracking_gaps_and_bad_quaternions_are_not_bridged():
    m = core(); c, poses, *_ = motion()
    poses['valid'][200:220] = False
    poses['quaternion'][400] = 0
    result = m.build_reference(poses, c, .12)
    ns = m.map_time(poses['source_ns'], c['clock'])
    values = m.sample_reference(result['rear_axle'], ns)
    assert np.isnan(values[195:225]).all() and np.isnan(values[395:405]).all()
    assert np.isfinite(values[100]).all() and np.isfinite(values[300]).all()


def test_clock_mapping_preserves_epoch_precision_and_requires_metadata():
    m = core(); c = configuration(); origin = c['clock']['source_anchor_ns']
    ns = np.array([origin, origin+1, origin+200_000_000], np.int64)
    mapped = m.map_time(ns, c['clock'])
    assert list(mapped-ns) == [123, 123, 123]
    with pytest.raises(ValueError, match='clock'): m.map_time(ns, dict(scale=1.))
    c['references']['rear_axle']['origin_in_marker_m'] = None
    with pytest.raises(ValueError, match='origin'): m.build_reference(motion()[1], c, .12)


def test_mm_requires_explicit_units_and_invalid_clock_is_rejected():
    m = core(); c, poses, *_ = motion()
    base = m.build_reference(poses, c, .12)
    poses['position'] *= 1000; c['vicon']['position_unit'] = 'mm'
    converted = m.build_reference(poses, c, .12)
    np.testing.assert_allclose(base['rear_axle'][0]['values'], converted['rear_axle'][0]['values'], atol=1e-9, equal_nan=True)
    c['clock']['scale'] = float('nan')
    with pytest.raises(ValueError, match='clock'): m.map_time(poses['source_ns'], c['clock'])
