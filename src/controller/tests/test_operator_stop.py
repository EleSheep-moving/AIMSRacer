"""Operator braking survives optimizer expiry without renewing a plan."""
from collections import deque
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time

from aims_mpcc.config import VehicleConfig
from aims_mpcc.node import MPCCNode
from aims_mpcc.runtime import Command,State,Supervisor


def moving():
    config=VehicleConfig(cruise_speed=.5,max_speed=1.,enforce_corridor=False)
    s=Supervisor(config,100.,plan_ttl=.8,handover_delay=.02,solve_period=.05)
    s.observe(State(0.,0.,0.,0.,0.,100.),7.,0.,0.)
    s.set_mode(True,7.);s.start(7.)
    # Frozen failed disable prefix: braking lasts longer than the old TTL.
    s.state=State(0.,0.,0.,.45581881179438183,.1772737438661454,103.)
    s.last_command=Command(.48728169530781507,.17774741775501707)
    s.last_acceleration=.2718328679357388;s.last_steering_rate=.0034530525624121284
    s.last_tick=9.98;s.state_received=s.mode_received=10.
    s.plan=dict(source_stamp=9.90,stamp=9.92,previous_steering=.1777,
                states=[[0.,0.,0.,.5,0.,.1773]]*11,
                controls=[[0.,.18,.5]]*10)
    return s


@pytest.mark.parametrize('solve_period',[.05,None])
def test_operator_stop_brakes_beyond_plan_ttl_and_then_latches_ready(solve_period):
    s=moving();s.solve_period=solve_period;s.stop()
    speeds=[];accelerations=[s.last_acceleration];rates=[s.last_steering_rate]
    for i in range(150):
        now=10.+i*.02;s.state_received=s.mode_received=now
        s.state=replace(s.state,speed=s.last_command.speed)
        command=s.command(now)
        assert s.status!='FAULT'
        speeds.append(command.speed);accelerations.append(s.last_acceleration);rates.append(s.last_steering_rate)
        if now>10.72:assert s.plan is None
    assert s.status=='READY' and s.reason=='Stopped'
    assert speeds[-1]==0.
    assert np.max(speeds)<=s.config.max_speed and np.min(speeds)>=0.
    assert np.max(np.abs(np.diff(accelerations)))<=s.config.jerk_limit*.02+1e-12
    assert np.max(np.abs(np.diff(rates)))<=s.config.steer_acceleration*.02+1e-12
    assert np.max(np.abs(rates))<=s.config.steer_rate+1e-12


def test_operator_stop_cancels_pending_and_ignores_late_results_without_plan_renewal():
    s=moving();old=s.plan;last_update=s.last_usable_update
    candidate=dict(old,generation=s.generation,success=True,constraint_violation=0.,
                   source_stamp=10.,submitted_at=10.,stamp=10.02)
    s.pending_plan=candidate;s.stop()
    assert s.pending_plan is None
    assert s.plan is old and s.last_usable_update==last_update
    assert not s.accept(candidate,10.02)
    assert s.pending_plan is None
    # Defend activation even if an already queued result reaches this method.
    s.pending_plan=candidate
    assert not s.activate(10.02,s.state)
    assert s.pending_plan is None and s.plan is old
    assert s.last_usable_update==last_update and s.status=='STOPPING'


@pytest.mark.parametrize('enforce_plan_age',[True,False])
def test_operator_stop_brakes_without_consuming_an_exhausted_horizon(enforce_plan_age):
    s=moving();s.plan.update(source_stamp=9.68,stamp=9.70,
                            states=s.plan['states'][:3],controls=s.plan['controls'][:2])
    s.stop();before=s.last_command
    s.command(10.,enforce_plan_age=enforce_plan_age)
    assert s.plan is None and s.status=='STOPPING'
    assert s.last_acceleration==pytest.approx(.2518328679357388,abs=1e-12)
    assert s.last_command.speed==pytest.approx(before.speed+s.last_acceleration*.02,abs=1e-12)


@pytest.mark.parametrize('guard',['negative_age','future_phase'])
def test_operator_stop_preserves_invalid_plan_clock_faults(guard):
    s=moving();s.stop()
    if guard=='negative_age':s.plan['source_stamp']=10.01
    else:s.plan['stamp']=10.01
    s.command(10.)
    assert s.status=='FAULT' and s.last_command.speed==0.


def test_running_without_initial_plan_still_faults_at_initial_deadline():
    s=moving();s.plan=None;s.last_usable_update=10.
    s.command(10.)
    assert s.status=='FAULT' and s.reason=='Initial plan deadline expired'


def test_stopping_keeps_plan_steering_until_expiry_then_smoothly_holds_angle():
    s=moving();s.PLAN_TTL=.3;s.plan['source_stamp']=9.765;s.plan['stamp']=9.8
    s.plan['controls']=[[0.,.2,.5]]*10
    s.last_tick=10.;s.last_command=Command(.5,.1);s.last_acceleration=0.;s.last_steering_rate=0.
    s.stop();angles=[];rates=[]
    for i in range(1,7):
        now=10.+i*.02;s.state_received=s.mode_received=now
        s.command(now);angles.append(s.last_command.steering);rates.append(s.last_steering_rate)
    assert rates==pytest.approx([.04,.08,.12,.08,.04,0.],abs=1e-12)
    assert angles==pytest.approx([.1008,.1024,.1048,.1064,.1072,.1072],abs=1e-12)
    assert s.plan is None and s.status=='STOPPING'


@pytest.mark.parametrize('guard',['odometry','selector','backwards','late','authority'])
def test_operator_stop_retains_freshness_authority_and_timing_faults(guard):
    s=moving();s.stop();now=10.02
    if guard=='odometry':s.state_received=now-.11
    elif guard=='selector':s.mode_received=now-.11
    elif guard=='backwards':s.last_tick=now+.01
    elif guard=='late':s.last_tick=now-.11
    else:s.set_mode(False,now)
    s.command(now)
    assert s.status=='FAULT' and s.last_command.speed==0.


class TickHarness(SimpleNamespace):
    def __getattr__(self,name):
        return None


def test_stopping_tick_publishes_braking_without_submitting_optimization(monkeypatch):
    import aims_mpcc.node as adapter
    monkeypatch.setattr(adapter.time,'monotonic',lambda:10.02)
    s=moving();s.stop();requests=[];commands=[]
    node=TickHarness(supervisor=s,config=s.config,path=SimpleNamespace(frame_id='odom'),
                     worker=SimpleNamespace(poll=lambda now:None,ready=True,pending=None,restart_count=0,
                                            submit=lambda request:True),
                     next_solve=10.,solve_period=.05,solve_times=[],horizon=10,plan_ttl=.8,
                     solver_timeout=.25,handover_delay=.02,deadline_misses=0,recent_proposals=deque(),
                     prepare_request=lambda now:requests.append(now) or {},count_publishers=lambda topic:1,
                     command_pub=SimpleNamespace(publish=commands.append),
                     status_pub=SimpleNamespace(publish=lambda message:None),
                     get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=100.)))
    MPCCNode.tick(node)
    assert requests==[]
    assert len(commands)==1 and commands[0].drive.speed>0.
    assert s.status=='STOPPING'
