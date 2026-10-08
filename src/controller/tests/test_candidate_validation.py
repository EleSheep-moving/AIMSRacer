import math
import numpy as np

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout


def candidate():
    c = VehicleConfig(geometry_verified=True, rear_offset=0., front_extent=.52,
                      rear_extent=.1, half_width=.16, enforce_corridor=False)
    initial = [0., 0., 0., .2, 0., 0.]
    previous = [0., 0., 0.]
    controls = np.tile([0., 0., .2], (10, 1))
    states = independent_rollout(initial, previous, controls, c)[::5]
    return c, dict(states=states.tolist(), controls=controls.tolist(), previous_steering=0.,
                   validation_applied=previous, dt=.1)


def test_independent_validation_checks_rollout_and_hard_controls():
    from aims_mpcc.validation import validate_candidate
    c, plan = candidate()
    assert validate_candidate(plan, c)['accepted']
    plan['states'][5][0] += .1
    assert not validate_candidate(plan, c)['accepted']
    c, plan = candidate()
    plan['controls'][0][0] = c.accel_limit+.1
    assert not validate_candidate(plan, c)['accepted']


def test_nonfinite_and_missing_applied_epoch_cannot_be_validated():
    from aims_mpcc.validation import validate_candidate
    c, plan = candidate()
    plan['controls'][0][1] = math.nan
    assert not validate_candidate(plan, c)['accepted']


def test_physical_steering_bound_is_checked_even_at_zero_speed():
    from aims_mpcc.validation import validate_candidate
    c,plan=candidate()
    initial=[0.,0.,0.,0.,0.,c.steer_limit+.1]
    controls=np.zeros((10,3))
    plan['states']=independent_rollout(initial,[0.,0.,0.],controls,c)[::5].tolist()
    plan['controls']=controls.tolist()
    assert not validate_candidate(plan,c)['accepted']


def test_corridor_validation_evaluates_geometry_in_batch():
    from aims_mpcc.validation import validate_candidate
    from aims_mpcc.path import ReferencePath
    from dataclasses import replace
    angles=np.arange(64)*2*np.pi/64
    path=ReferencePath(np.c_[3*np.cos(angles),3*np.sin(angles)],1.,1.)
    c,plan=candidate()
    c=replace(c,enforce_corridor=True)
    initial=[3.,0.,np.pi/2,.2,0.,0.]
    plan['states']=independent_rollout(initial,[0.,0.,0.],plan['controls'],c)[::5].tolist()
    calls=[]
    original=path.at
    def counted(progress):
        calls.append(progress)
        return original(progress)
    path.at=counted
    result=validate_candidate(plan,c,path)
    assert result['accepted']
    assert len(calls)<=2, 'Scalar geometry in every sample overwhelms the 50 Hz callback budget'
    c, plan = candidate()
    del plan['validation_applied']
    assert not validate_candidate(plan, c)['accepted']
