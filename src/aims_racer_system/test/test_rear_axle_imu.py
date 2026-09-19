"""Independent rigid-body fixtures for rear-point specific force."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from aims_racer_system.rear_axle_imu import compensate_force, AngularHistory


@pytest.mark.parametrize('omega,alpha', [([0,0,2],[0,0,0]), ([0,0,0],[0,0,3]), ([.3,-.4,2],[1,2,-3])])
def test_removes_tangential_and_centripetal_terms(omega, alpha):
    r = np.array([.311,.02329,-.01412])
    desired = np.array([1.,2.,9.80665])
    measured = desired + np.cross(alpha,r) + np.cross(omega,np.cross(omega,r))
    corrected, covariance = compensate_force(measured, omega, alpha, r, np.eye(3)*.05, np.eye(3)*.01, np.eye(3))
    assert corrected == pytest.approx(desired)
    assert np.linalg.eigvalsh(covariance).min() > 0


def test_gravity_is_preserved_for_ekf():
    rotation = Rotation.from_euler('xyz', [.2,-.3,.5])
    gravity = rotation.inv().apply([0,0,9.80665])
    corrected, _ = compensate_force(gravity, np.zeros(3), np.zeros(3), [.3,0,0], np.eye(3), np.eye(3), np.eye(3))
    assert corrected == pytest.approx(gravity)


def test_covariance_increases_with_angular_uncertainty():
    args = ([0,0,9.8], [0,0,2], [0,0,1], [.3,0,0], np.eye(3)*.05)
    _, small = compensate_force(*args, np.eye(3)*.001, np.eye(3)*.001)
    _, large = compensate_force(*args, np.eye(3)*.1, np.eye(3)*.1)
    assert np.trace(large) > np.trace(small)


def test_linear_gyro_ramp_and_attitude_propagation():
    h = AngularHistory(window=.025, max_gap=.03)
    for i in range(9):
        t=10+i*.005
        h.add(t, [0,0,2*(t-10)], np.eye(3)*.01)
    alpha, cov = h.derivative()
    assert alpha == pytest.approx([0,0,2])
    q = h.orientation(10., [0,0,0,1], 10.04)
    assert Rotation.from_quat(q).as_rotvec()[2] == pytest.approx(.0016, abs=1e-8)
    assert np.linalg.eigvalsh(cov).min() > 0


def test_warmup_gap_and_time_reset_invalidate_derivative():
    h = AngularHistory(window=.025,max_gap=.03)
    h.add(1.,[0,0,0],np.eye(3))
    assert h.derivative() is None
    h.add(1.005,[0,0,0],np.eye(3)); h.add(1.010,[0,0,0],np.eye(3))
    assert h.derivative() is not None
    h.add(1.2,[0,0,0],np.eye(3))
    assert h.derivative() is None
    h.add(.5,[0,0,0],np.eye(3))
    assert h.derivative() is None
    assert h.orientation(1.,[0,0,0,1],.5) is None


def test_future_or_uncovered_attitude_not_used():
    h = AngularHistory()
    h.add(1.,[0,0,0],np.eye(3))
    assert h.orientation(1.1,[0,0,0,1],1.) is None
    assert h.orientation(.5,[0,0,0,1],1.) is None
