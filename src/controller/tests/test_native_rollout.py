import numpy as np
from aims_mpcc.config import VehicleConfig


def test_compiled_midpoint_rollout_matches_independent_python(tmp_path):
    from aims_mpcc.rollout_native import prepare_kernel, native_rollout
    from aims_mpcc.envelope import python_rollout
    library=prepare_kernel(tmp_path)
    c=VehicleConfig(steering_tau=.08)
    rng=np.random.default_rng(7)
    for _ in range(5):
        initial=np.array([1.,-2.,.3,.5,4.,.1])
        applied=np.array([0.,.1,0.])
        controls=rng.uniform([-.2,-.3,0.],[.2,.3,1.],(10,3))
        actual=native_rollout(library,initial,applied,controls,c,.1,.02)
        expected=python_rollout(initial,applied,controls,c,.1,.02)
        np.testing.assert_allclose(actual,expected,atol=2e-12,rtol=2e-12)


def test_compiled_rollout_has_correct_straight_acceleration(tmp_path):
    from aims_mpcc.rollout_native import prepare_kernel,native_rollout
    library=prepare_kernel(tmp_path)
    samples=native_rollout(library,np.array([0.,0.,0.,.5,0.,0.]),np.zeros(3),
                           np.tile([.1,0.,.5],(10,1)),VehicleConfig(),.1,0.)
    np.testing.assert_allclose(samples[-1],[.55,0.,0.,.6,.5,0.],atol=1e-12)
