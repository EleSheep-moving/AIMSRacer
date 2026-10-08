"""Asynchronous selector prefixes must not invalidate a usable optimizer plan."""
import copy

import numpy as np

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout
from aims_mpcc.runtime import Command, State, Supervisor
from aims_mpcc.validation import validate_candidate


def prepared(initial, previous, controls):
    config=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    supervisor=Supervisor(config,100.,handover_delay=.02,solve_period=.05)
    supervisor.observe(State(0.,0.,0.,0.,0.,100.),10.,0.,0.)
    supervisor.set_mode(True,10.)
    supervisor.start(10.)
    supervisor.state=State(initial[0],initial[1],initial[2],initial[3],initial[5],100.02)
    states=independent_rollout(initial,previous,controls,config)[::5].tolist()
    result=dict(success=True,generation=supervisor.generation,states=states,
                controls=np.asarray(controls).tolist(),validation_applied=list(previous),
                constraint_violation=0.,source_stamp=10.,submitted_at=10.,stamp=10.02,
                previous_steering=previous[1])
    result['validation']=validate_candidate(result,config)
    assert result['validation']['accepted']
    return supervisor,result


def test_trace_acceleration_prefix_is_reprojected_without_renewing_or_skipping():
    previous=[.07474900302617726,.018494155257940292,.13125126272394314]
    controls=[[previous[0]+.1,.04486494937414733,.2],
              [previous[0]+.2,.05986494937414733,.2]]+[[.3,.06386494937414733,.2]]*8
    s,result=prepared([0.,0.,0.,.1,0.,.01345072196797285],previous,controls)
    snapshot=copy.deepcopy(result)
    applied=dict(speed=.1,steering=.02195948362350464,
                 acceleration=.034749730002609176,steering_rate=.08204024390415686)
    # Model a consistent live internal smoother and actual selector prefix.
    # The optimizer prefix remains different and still needs reprojection.
    s.last_command=Command(applied['speed'],applied['steering'])
    s.last_acceleration=applied['acceleration']
    s.last_steering_rate=applied['steering_rate']
    assert s.accept(result,10.02)
    assert s.activate(10.02,s.state,applied)
    assert s.plan['validation']['accepted']
    assert len(s.plan['controls'])==len(controls)
    assert s.plan['source_stamp']==10.
    assert s.plan['controls'][0][0]<=applied['acceleration']+.1+1e-12
    assert s.plan['controls'][2:]==controls[2:]
    assert result==snapshot
    assert s.plan['handover_reprojection']['max_acceleration_correction']>.039


def test_actual_steering_rate_prefix_preserves_all_hard_limits():
    controls=[[0.,angle,.1] for angle in (.15,.2,.25,.28,.29,.29,.29,.29,.29,.29)]
    s,result=prepared([0.,0.,0.,.1,0.,.1],[0.,.1,.3],controls)
    assert s.accept(result,10.02)
    s.last_command=Command(.1,.1)
    assert s.activate(10.02,s.state,dict(speed=.1,steering=.1,acceleration=0.,steering_rate=0.))
    assert s.plan['validation']['envelope']['hard_control_violation']<=1e-12
    assert s.plan['controls'][0][1]<=.12+1e-12
    assert len(s.plan['controls'])==len(controls)


def test_reprojection_cannot_authorize_an_invalid_original_candidate():
    controls=[[0.,0.,.1]]*10
    s,result=prepared([0.,0.,0.,.1,0.,0.],[0.,0.,0.],controls)
    result['controls'][0][0]=s.config.accel_limit+.1
    old=dict(source_stamp=9.99)
    s.plan=old
    assert s.accept(result,10.02)
    assert not s.activate(10.02,s.state,dict(speed=.1,steering=0.,acceleration=0.,steering_rate=0.))
    assert s.plan is old
