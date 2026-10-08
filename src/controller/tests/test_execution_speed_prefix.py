"""Replanning must preserve forwarded speed targets despite physical motor lag."""
import math
from dataclasses import replace

import numpy as np

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout
from aims_mpcc.runtime import Command, State, Supervisor, clip


def candidate(supervisor, now, physical_speed, applied):
    initial=[supervisor.state.x,0.,0.,physical_speed,supervisor.progress,0.]
    desired=clip(.8*(1.-physical_speed),-.3,.3)
    acceleration=applied['acceleration']
    controls=[]
    for _ in range(10):
        acceleration=clip(desired,acceleration-.1,acceleration+.1)
        controls.append([acceleration,0.,physical_speed])
    states=independent_rollout(initial,[applied['acceleration'],0.,0.],controls,supervisor.config)[::5].tolist()
    return dict(success=True,generation=supervisor.generation,states=states,controls=controls,
                validation_applied=[applied['acceleration'],0.,0.],previous_steering=0.,
                constraint_violation=0.,source_stamp=now-.02,submitted_at=now-.02,stamp=now)


def run_lagged_plant(period):
    config=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    supervisor=Supervisor(config,100.,handover_delay=.02,solve_period=period)
    supervisor.observe(State(0.,0.,0.,0.,0.,100.),10.,0.,0.)
    supervisor.set_mode(True,10.)
    supervisor.start(10.)
    speed=0.;position=0.;next_plan=10.02;previous_acceleration=0.
    for tick in range(1,401):
        now=10.+tick*.02
        supervisor.state=State(position,0.,0.,speed,0.,100.+tick*.02)
        supervisor.state_received=supervisor.mode_received=now
        applied=dict(speed=supervisor.last_command.speed,steering=0.,
                     acceleration=supervisor.last_acceleration,steering_rate=0.)
        if now>=next_plan-1e-9:
            result=candidate(supervisor,now,speed,applied)
            assert supervisor.accept(result,now)
            assert supervisor.activate(now,supervisor.state,applied)
            next_plan+=period
        command=supervisor.command(now)
        assert supervisor.status=='RUNNING'
        assert 0.<=command.speed<=config.max_speed
        assert -config.brake_limit<=supervisor.last_acceleration<=config.accel_limit
        assert abs(supervisor.last_acceleration-previous_acceleration)<=config.jerk_limit*.02+1e-9
        previous_acceleration=supervisor.last_acceleration
        speed+=(command.speed-speed)*(1.-math.exp(-.02/.2))
        position+=speed*.02
    return speed,supervisor.last_command.speed


def test_lagged_speed_attainment_is_consistent_at_five_and_twenty_hz():
    slow=run_lagged_plant(.2)
    fast=run_lagged_plant(.05)
    assert slow[0]>.9 and fast[0]>.9,(slow,fast)
    assert abs(slow[0]-fast[0])<.04,(slow,fast)


def test_real_wire_speed_is_separate_from_validated_physical_state():
    config=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s=Supervisor(config,100.,handover_delay=.02,solve_period=.05)
    s.observe(State(0.,0.,0.,0.,0.,100.),10.,0.,0.)
    s.set_mode(True,10.)
    s.start(10.)
    s.state=replace(s.state,speed=.4)
    s.last_command=Command(.6,0.)
    applied=dict(speed=.6,steering=0.,acceleration=0.,steering_rate=0.)
    result=candidate(s,10.02,.4,applied)
    assert s.accept(result,10.02)
    assert s.activate(10.02,s.state,applied)
    assert s.plan['states'][0][3]==.4
    assert s.plan['execution_speed_targets'][0]==.6
    np.testing.assert_allclose(np.diff(s.plan['execution_speed_targets']),
                               np.asarray(s.plan['controls'])[:,0]*.1,atol=1e-12)
    assert s.plan['source_stamp']==10.
    assert s.command(10.02).speed>=.6


def test_execution_targets_are_bounded_without_changing_physical_controls():
    config=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    for physical_speed,wire_speed,sign in ((.7,1.45,1.),(.8,.05,-1.)):
        s=Supervisor(config,100.,handover_delay=.02,solve_period=.05)
        s.observe(State(0.,0.,0.,0.,0.,100.),10.,0.,0.)
        s.set_mode(True,10.)
        s.start(10.)
        s.state=replace(s.state,speed=physical_speed)
        applied=dict(speed=wire_speed,steering=0.,acceleration=0.,steering_rate=0.)
        controls=[[sign*min((i+1)*.1,.3),0.,physical_speed] for i in range(10)]
        result=candidate(s,10.02,physical_speed,applied)
        result['controls']=controls
        result['states']=independent_rollout([0.,0.,0.,physical_speed,0.,0.],
            [0.,0.,0.],controls,config)[::5].tolist()
        assert s.accept(result,10.02)
        assert s.activate(10.02,s.state,applied)
        assert s.plan['controls']==controls
        assert all(0.<=v<=config.max_speed for v in s.plan['execution_speed_targets'])
        assert s.plan['execution_speed_targets'][-1] in (0.,config.max_speed)
        assert s.plan['states'][-1][3]!=s.plan['execution_speed_targets'][-1]
