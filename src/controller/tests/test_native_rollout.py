import numpy as np
import pytest
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


@pytest.fixture(params=['truncated', 'missing_symbol'])
def bad_cached_kernel(request, tmp_path, monkeypatch):
    from aims_mpcc import rollout_native
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR', str(tmp_path))
    target = rollout_native.kernel_path()
    if request.param == 'truncated':
        target.write_bytes(b'incomplete cached binary')
    else:
        # Build the deliberately incompatible fixture offline, before entering
        # the runtime path whose compilation must remain forbidden.
        rollout_native.subprocess.run(
            ['gcc', '-shared', '-fPIC', '-x', 'c', '-o', str(target), '-'],
            input='int another_symbol(void) { return 0; }\n', text=True, check=True)
    original = target.read_bytes()

    def forbidden_compile(*args, **kwargs):
        pytest.fail('An invalid runtime cache must not trigger online compilation')

    monkeypatch.setattr(rollout_native.subprocess, 'run', forbidden_compile)
    yield target
    assert target.read_bytes() == original


def test_bad_cached_kernel_raises_validation_error(bad_cached_kernel):
    from aims_mpcc.rollout_native import native_rollout
    with pytest.raises(ValueError, match='Cannot load independent rollout cache'):
        native_rollout(bad_cached_kernel, np.zeros(6), np.zeros(3),
                       np.zeros((2, 3)), VehicleConfig())


def test_validation_rejects_bad_cached_kernel_without_online_compile(bad_cached_kernel):
    from aims_mpcc.validation import validate_candidate
    plan = dict(states=np.zeros((3, 6)).tolist(), controls=np.zeros((2, 3)).tolist(),
                validation_applied=[0., 0., 0.])
    result = validate_candidate(plan, VehicleConfig(enforce_corridor=False))
    assert not result['accepted']
    assert 'Cannot load independent rollout cache' in result['reason']
