"""Contract tests for execution independent of ROS and the optimizer."""
import math
from types import SimpleNamespace
import pytest
from aims_mpcc.runtime import Supervisor, State, speed_envelope


def supervisor():
    config = SimpleNamespace(max_speed=1., cruise_speed=.5, accel_limit=.5,
                             brake_limit=.5, jerk_limit=1., steer_limit=.4,
                             steer_rate=.5, steer_acceleration=2., steering_tau=.115)
    return Supervisor(config, length=10.)


def ready(s, now=1.):
    s.observe(State(0., 0., 0., 0., 0., now), now, 0., 0.)
    s.set_mode(True, now)
    s.start(now)


def plan(stamp=1., generation=1):
    return dict(success=True, generation=generation, stamp=stamp,
                states=[[0, 0, 0, i*.05, i*.02, 0] for i in range(16)],
                controls=[[.5, .1, .5] for _ in range(15)],
                constraint_violation=0., solve_time_s=.01)


def test_stale_plan_cannot_be_refreshed_by_publishing():
    s=supervisor(); ready(s)
    assert s.accept(plan(), 1.01)
    assert s.command(1.02).speed > 0
    s.observe(State(0,0,0,0,0,1.26),1.26,0,0); s.set_mode(True,1.26)
    assert s.command(1.26).speed == 0
    assert s.status == 'FAULT'


def test_manual_override_latches_until_explicit_restart():
    s=supervisor(); ready(s); s.accept(plan(),1.01)
    s.set_mode(False,1.02); assert s.command(1.02).speed == 0
    s.set_mode(True,1.03); assert s.command(1.03).speed == 0
    assert s.status == 'FAULT'
    assert not s.accept(plan(),1.03)


@pytest.mark.parametrize('kind', ['state', 'mode'])
def test_input_expiry_stops(kind):
    s=supervisor(); ready(s); s.accept(plan(),1.01)
    if kind=='state': s.set_mode(True,1.12)
    else: s.observe(State(0,0,0,0,0,1.12),1.12,0,0)
    assert s.command(1.12).speed == 0
    assert s.status=='FAULT'


def test_reject_old_generation_and_solver_failure():
    s=supervisor(); ready(s)
    assert not s.accept(plan(generation=0),1.01)
    assert not s.accept(dict(plan(),success=False),1.02)
    assert s.status=='FAULT'


def test_clock_reset_and_pose_jump():
    for state in [State(0,0,0,0,0,.5), State(5,0,0,0,0,1.02)]:
        s=supervisor(); ready(s)
        s.observe(state,1.02,0,0)
        assert s.status=='FAULT'


def test_stop_envelope_and_explicit_stop():
    s=supervisor(); ready(s); s.accept(plan(),1.01)
    assert speed_envelope(0,.5,1,.25)==0
    assert speed_envelope(.1,.5,1,.25)<speed_envelope(1,.5,1,.25)
    s.stop(); assert s.command(1.02).speed==0


def test_start_requires_near_start_and_fresh_mode():
    s=supervisor()
    with pytest.raises(ValueError): s.start(1.)
    s.observe(State(0,0,0,.4,0,1),1,0,0); s.set_mode(True,1)
    with pytest.raises(ValueError): s.start(1.)


def test_one_lap_cannot_complete_at_start():
    s=supervisor(); ready(s)
    for i in range(35):
        t=1+i*.02
        s.observe(State(0,0,0,0,0,t),t,0,0); s.set_mode(True,t)
        s.accept(plan(stamp=t),t); s.command(t)
    assert s.status!='COMPLETE'


def test_nonfinite_plan_rejected_and_late_reply_rejected():
    for p,t in [(dict(plan(),constraint_violation=math.nan),1.01),(plan(),1.3)]:
        s=supervisor(); ready(s)
        assert not s.accept(p,t)
        assert s.status=='FAULT'


def test_steering_acceleration_bounded_between_plans():
    s=supervisor();ready(s)
    previous=0.
    for i in range(1,10):
        t=1+i*.02
        s.observe(State(0,0,0,0,0,t),t,0,0);s.set_mode(True,t)
        p=plan(stamp=t-.02)
        p['controls']=[[.5, .4 if i<5 else -.4,.5] for _ in range(15)]
        s.accept(p,t)
        before=s.last_command.steering
        command=s.command(t)
        rate=(command.steering-before)/.02
        assert abs(rate-previous)<=2*.02+1e-8
        previous=rate


def test_measurement_source_age_is_not_refreshed_at_receipt():
    s=supervisor();ready(s);s.accept(plan(),1.01)
    # Message received at 1.09 was captured at 1.01; adapter passes capture epoch.
    s.observe(State(0,0,0,0,0,1.01),1.01,0,0);s.set_mode(True,1.09)
    s.command(1.08)  # maintain normal scheduling
    assert s.command(1.12).speed==0
    assert s.status=='FAULT'


def test_infeasible_steering_rate_at_bound_faults():
    from aims_mpcc.runtime import Command
    s=supervisor();ready(s);s.last_command=Command(0.,.39);s.last_steering_rate=.5
    p=plan();p['previous_steering']=.39;p['controls']=[[0.,.4,0.] for _ in range(15)]
    s.accept(p,1.);s.command(1.02)
    assert s.status=='FAULT'
    assert 'steering' in s.reason.lower()


@pytest.mark.parametrize('direction', [1,-1])
def test_steering_brakes_before_bound_and_reports_achieved_rate(direction):
    s=supervisor();ready(s);previous_rate=0.
    for i in range(1,151):
        t=1+i*.02;s.observe(State(0,0,0,0,0,t),t,0,0);s.set_mode(True,t)
        p=plan(stamp=t-.1);p['previous_steering']=direction*.4;p['controls']=[[0,direction*.4,0] for _ in range(15)]
        s.accept(p,t)
        before=s.last_command.steering;command=s.command(t)
        assert s.status=='RUNNING',s.reason
        rate=(command.steering-before)/.02
        assert abs(rate)<=.5+1e-8
        assert abs(rate-previous_rate)<=2*.02+1e-8
        assert abs(command.steering)<=.4+1e-8
        assert s.last_steering_rate==pytest.approx(rate,abs=1e-8)
        previous_rate=rate
    assert direction*s.last_command.steering>.399


def test_lap_finish_requires_stationary_hold_then_complete():
    s=supervisor();ready(s)
    # Feed a continuous increasing progress sequence; route end is crossed once.
    for i in range(1,1001):
        t=1+i*.02;progress=i*.01
        s.observe(State(progress,0,0,.5,0,t),t,progress%10,0);s.set_mode(True,t)
        s.accept(plan(stamp=t),t);s.command(t)
    assert s.status=='RUNNING'
    for i in range(1,28):
        t=21+i*.02
        s.observe(State(10,0,0,0,0,t),t,0,0);s.set_mode(True,t)
        s.accept(plan(stamp=t),t);s.command(t)
        if i<26: assert s.status=='RUNNING'
    assert s.status=='COMPLETE'
    assert s.command(22.).speed==0.


def test_infeasible_braking_near_zero_faults_instead_of_inventing_acceleration():
    from aims_mpcc.runtime import Command
    s=supervisor();ready(s);s.last_command=Command(.005,0.);s.last_acceleration=-.5
    p=plan();p['states']=[[0,0,0,0,0,0] for _ in range(16)];p['controls']=[[0,0,0] for _ in range(15)]
    s.accept(p,1.);s.stop();s.command(1.02)
    assert s.status=='FAULT'
    assert 'acceleration' in s.reason.lower()


def test_speed_brakes_acceleration_before_zero_and_reports_achieved():
    from aims_mpcc.runtime import Command
    s=supervisor();ready(s);s.last_command=Command(.2,0.);previous_accel=0.
    s.stop()
    for i in range(1,101):
        t=1+i*.02;s.observe(State(0,0,0,.2,0,t),t,0,0);s.set_mode(True,t)
        p=plan(stamp=t);p['states']=[[0,0,0,0,0,0] for _ in range(16)];p['controls']=[[0,0,0] for _ in range(15)]
        s.accept(p,t);before=s.last_command.speed;command=s.command(t)
        assert s.status=='STOPPING',s.reason
        achieved=(command.speed-before)/.02
        assert abs(achieved-previous_accel)<=.02+1e-8
        assert achieved==pytest.approx(s.last_acceleration,abs=1e-8)
        assert command.speed>=0
        previous_accel=achieved
    assert s.last_command.speed<1e-5


def test_finish_envelope_caps_valid_high_speed_plan_before_smooth_limiter():
    from aims_mpcc.runtime import Command
    s=supervisor();ready(s);s.progress=9.9;s.last_command=Command(.3,0.)
    p=plan();p['states']=[[0,0,0,.5,9.9,0] for _ in range(16)];p['controls']=[[.5,0,.5] for _ in range(15)]
    s.accept(p,1.);command=s.command(1.02)
    assert s.status=='RUNNING'
    assert command.speed<.3
    assert command.speed==pytest.approx(.3-.02*.02)
    assert s.last_acceleration==pytest.approx(-.02)


def test_start_rejects_speed_below_reverse_tolerance():
    s=supervisor()
    s.observe(State(0.,0.,0.,-.051,0.,1.),1.,0.,0.)
    s.set_mode(True,1.)
    with pytest.raises(ValueError,match='reverse motion'):
        s.start(1.)
    assert not s.active
