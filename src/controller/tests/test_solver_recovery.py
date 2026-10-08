"""Recover transient optimizer failures without extending an old plan."""
from dataclasses import replace
from types import SimpleNamespace

from aims_mpcc.config import VehicleConfig
from aims_mpcc.runtime import Command, State, Supervisor


def running():
    s = Supervisor(VehicleConfig(cruise_speed=1., max_speed=1.5, enforce_corridor=False), 100.,
                   plan_ttl=.8, handover_delay=.02, solve_period=.2)
    s.observe(State(0., 0., 0., 0., 0., 100.), 10., 0., 0.)
    s.set_mode(True, 10.)
    s.start(10.)
    s.state = replace(s.state, speed=.5)
    s.last_command = Command(.5, 0.)
    s.plan = dict(source_stamp=9.99, stamp=10.,
                  states=[[i*.05, 0., 0., .5, i*.05, 0.] for i in range(11)],
                  controls=[[0., 0., .5] for _ in range(10)])
    return s


def test_failed_candidate_preserves_live_plan_without_latching_fault():
    s = running()
    old = s.plan
    assert not s.accept(dict(generation=s.generation, success=False), 10.02)
    assert s.status == 'RUNNING'
    assert s.plan is old
    assert s.plan['source_stamp'] == 9.99


def test_two_periods_without_usable_update_decelerate_and_continue_solving():
    s = running()
    for i in range(1, 31):
        now = 10. + i*.02
        s.state_received = s.mode_received = now
        s.state = replace(s.state, speed=s.last_command.speed)
        command = s.command(now)
    assert s.status == 'RECOVERING'
    assert s.active
    assert 0. < command.speed < .5
    assert s.refs(10, .1) == [0.]*11


def test_expired_plan_is_not_consumed_during_recovery():
    s = running()
    for i in range(1, 51):
        now = 10. + i*.02
        s.state_received = s.mode_received = now
        s.state = replace(s.state, speed=s.last_command.speed)
        s.command(now)
    assert s.plan is None
    assert s.status in ('RECOVERING', 'READY')
    assert s.last_command.speed < .5


def test_stopped_recovery_requires_manual_reenable():
    s = running()
    for i in range(1, 151):
        now = 10. + i*.02
        s.state_received = s.mode_received = now
        s.state = replace(s.state, speed=s.last_command.speed)
        s.command(now)
    assert s.status == 'READY'
    assert not s.active
    assert s.reason == 'Recovery stopped; re-enable required'


def test_late_reply_replays_first_new_control_at_actual_epoch():
    from aims_mpcc.envelope import independent_rollout
    s = running()
    controls=[[0., 0., .5] for _ in range(10)]
    initial=[.01,0.,0.,.5,0.,0.]
    states=independent_rollout(initial,[0.,0.,0.],controls,s.config)[::5].tolist()
    result=dict(success=True, generation=s.generation, states=states, controls=controls,
                constraint_violation=0.,source_stamp=10.,submitted_at=10.,stamp=10.02,
                previous_steering=0.,validation_applied=[0.,0.,0.])
    assert s.accept(result,10.10)
    actual=State(.055,0.,0.,.5,0.,100.10)
    expected=State(.05,0.,0.,.5,0.,100.10)
    applied=dict(speed=.5,steering=0.,acceleration=0.,steering_rate=0.)
    assert s.activate(10.10,actual,applied,expected_at_activation=expected)
    assert s.plan['states'][0][0] == actual.x
    assert s.plan['stamp'] == 10.10
    assert s.plan['source_stamp'] == 10.
    assert s.plan['controls'][0] == controls[0]
    assert s.plan['validation']['accepted']


def test_two_independently_valid_handover_candidates_restore_moving_recovery():
    s=running()
    s.status='RECOVERING'
    for now in (10.02,10.04):
        initial=[s.state.x,s.state.y,s.state.yaw,.5,0.,0.]
        from aims_mpcc.envelope import independent_rollout
        controls=[[0.,0.,.5] for _ in range(10)]
        states=independent_rollout(initial,[0.,0.,0.],controls,s.config)[::5].tolist()
        result=dict(success=True,generation=s.generation,states=states,controls=controls,
                    constraint_violation=0.,source_stamp=now-.02,submitted_at=now-.02,stamp=now,
                    previous_steering=0.,validation_applied=[0.,0.,0.])
        assert s.accept(result,now)
        assert s.activate(now,s.state,dict(speed=.5,steering=0.,acceleration=0.,steering_rate=0.))
        if now==10.02: assert s.status=='RECOVERING'
    assert s.status=='RUNNING'


def test_operator_withdrawal_latches_until_explicit_reenable():
    s=running()
    s.status='RECOVERING'
    s.set_mode(False,10.02)
    assert not s.active
    generation=s.generation
    s.set_mode(True,10.04)
    assert not s.active
    assert s.generation==generation


def test_corrupt_solver_poses_are_checked_before_rebase():
    s=running()
    from aims_mpcc.envelope import independent_rollout
    controls=[[0.,0.,.5] for _ in range(10)]
    states=independent_rollout([0.,0.,0.,.5,0.,0.],[0.,0.,0.],controls,s.config)[::5].tolist()
    states[5][0]+=10.
    result=dict(success=True,generation=s.generation,states=states,controls=controls,
                constraint_violation=0.,source_stamp=10.,submitted_at=10.,stamp=10.02,
                validation_applied=[0.,0.,0.],previous_steering=0.)
    assert s.accept(result,10.02)
    assert not s.activate(10.02,s.state,dict(speed=.5,steering=0.,acceleration=0.,steering_rate=0.))


def test_current_map_correction_is_used_for_future_corridor():
    from aims_mpcc.envelope import independent_rollout
    s=running()
    s.config=replace(s.config,enforce_corridor=True,geometry_verified=True,
                     rear_offset=0.,front_extent=.52,rear_extent=.1,half_width=.15)
    s.state=replace(s.state,steering=.2)
    controls=[[0.,.2,.5] for _ in range(3)]
    states=independent_rollout([0.,0.,0.,.5,0.,.2],[0.,.2,0.],controls,s.config)[::5].tolist()
    path=SimpleNamespace(frame_id='map',left_width=.3,right_width=.3,
                         at=lambda progress: dict(x=progress,y=0.,yaw=0.))
    result=dict(success=True,generation=s.generation,states=states,controls=controls,
                constraint_violation=0.,source_stamp=10.,submitted_at=10.,stamp=10.02,
                validation_applied=[0.,.2,0.],previous_steering=.2,map_alignment=(0.,0.,0.))
    assert s.accept(result,10.02)
    assert not s.activate(10.02,s.state,dict(speed=.5,steering=.2,acceleration=0.,steering_rate=0.),
                          path=path,map_alignment=(0.,.12,0.))


def test_pending_candidate_expiry_preserves_unexpired_old_plan():
    s=running()
    s.PLAN_TTL=.08
    old=s.plan
    old['source_stamp']=9.97
    from aims_mpcc.envelope import independent_rollout
    controls=[[0.,0.,.5] for _ in range(10)]
    states=independent_rollout([0.,0.,0.,.5,0.,0.],[0.,0.,0.],controls,s.config)[::5].tolist()
    result=dict(success=True,generation=s.generation,states=states,controls=controls,
                constraint_violation=0.,source_stamp=9.93,submitted_at=10.,stamp=10.02,
                validation_applied=[0.,0.,0.],previous_steering=0.)
    assert s.accept(result,10.)
    assert not s.activate(10.02,s.state,dict(speed=.5,steering=0.,acceleration=0.,steering_rate=0.))
    assert s.plan is old
    assert s.status=='RUNNING'
