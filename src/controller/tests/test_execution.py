"""Validate the nominal emitted schedule separately from optimizer intervals."""
import copy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import evaluate_envelope,independent_rollout,utilization
from aims_mpcc.runtime import Command,State,Supervisor
from aims_mpcc.validation import validate_candidate


def prepared(config,initial,applied,controls,now=10.02):
    s=Supervisor(config,100.,handover_delay=.02,solve_period=.2)
    s.observe(State(0.,0.,0.,0.,initial[5],100.),10.,0.,0.)
    s.set_mode(True,10.);s.start(10.)
    s.state=State(initial[0],initial[1],initial[2],initial[3],initial[5],100.02)
    s.state_received=s.mode_received=now
    s.last_command=Command(initial[3],applied[1])
    s.last_acceleration=applied[0];s.last_steering_rate=applied[2]
    candidate=dict(success=True,generation=s.generation,
                   states=independent_rollout(initial,applied,controls,config)[::5].tolist(),
                   controls=np.asarray(controls).tolist(),validation_applied=list(applied),
                   constraint_violation=0.,source_stamp=now-.02,submitted_at=now-.02,stamp=now,
                   previous_steering=applied[1])
    candidate['validation']=validate_candidate(candidate,config)
    assert candidate['validation']['accepted']
    actual=dict(speed=initial[3],steering=applied[1],acceleration=applied[0],steering_rate=applied[2])
    return s,candidate,actual


def recovery_case(deadline=.24):
    cfg=VehicleConfig(rear_offset=0.,front_extent=.52,rear_extent=.10,half_width=.16,
                      geometry_verified=True,cruise_speed=1.,max_speed=1.5,
                      steer_limit=.45,steer_rate=2.,steering_tau=.08,enforce_corridor=False,
                      envelope_soft_enabled=True,recovery_jerk_enabled=True,
                      envelope_recovery_time=deadline)
    initial=np.array([0.,0.,0.,.9185,0.,.4373]);applied=np.array([0.,.4373,0.])
    rates=np.array([-.2,-.4,-.6,-.8,-.6,-.4,-.2,0.,0.,0.])
    controls=np.c_[np.zeros(10),.4373+np.cumsum(rates)*.1,np.full(10,.5)]
    controls[0,0]=-.12
    controls=np.asarray(evaluate_envelope(initial,applied,controls,cfg)['reference_controls'])
    return cfg,initial,applied,controls


def test_strict_gate_records_actual_slew_endpoint_without_changing_output():
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s,candidate,applied=prepared(cfg,[0.,0.,0.,.5,0.,0.],[0.,0.,0.],[[.1,0.,.5]]*10)
    assert s.accept(candidate,10.02);assert s.activate(10.02,s.state,applied)
    validation=s.plan['execution_validation']
    assert validation['accepted']
    assert validation['states'][5][3]==pytest.approx(.506,abs=1e-12)
    assert s.plan['states'][1][3]==pytest.approx(.510,abs=1e-12)
    observed=[]
    for i in range(5):
        now=10.02+i*.02;s.state_received=s.mode_received=now
        s.command(now);observed.append(s.last_acceleration)
    assert observed==pytest.approx([.02,.04,.06,.08,.10],abs=1e-12)
    assert np.asarray(validation['controls'])[:5,0]==pytest.approx(observed,abs=1e-12)


def test_macro_valid_soft_recovery_cannot_activate_deadline_violating_execution():
    cfg,initial,previous,controls=recovery_case()
    s,candidate,applied=prepared(cfg,initial,previous,controls)
    assert candidate['validation']['envelope']['future_violation_duration_s']==pytest.approx(.2)
    old=dict(source_stamp=9.99,stamp=10.)
    s.plan=old;snapshot=copy.deepcopy(candidate)
    assert s.accept(candidate,10.02)
    assert not s.activate(10.02,s.state,applied)
    assert s.plan is old
    assert candidate==snapshot
    validation=s.execution_validation
    assert not validation['envelope']['recovery_deadline_satisfied']
    samples=np.asarray(validation['states']);realized=np.asarray(validation['controls'])
    # Direct acceleration proposals change the old catch-up trace (E=1.115235)
    # but still cannot meet the unchanged 0.24 s physical recovery deadline.
    assert utilization(samples[12],realized[11,0],cfg)>1.05


def test_soft_comparator_keeps_macro_prefix_steering_and_strict_executor_jerk():
    cfg,initial,previous,controls=recovery_case(.6)
    s,candidate,applied=prepared(cfg,initial,previous,controls)
    assert s.accept(candidate,10.02);s.activate(10.02,s.state,applied)
    validation=s.execution_validation
    executed=np.asarray(validation['controls']);reference=np.asarray(validation['reference_controls'])
    assert np.array_equal(executed[:5],reference[:5])
    assert np.array_equal(executed[:,1],reference[:,1])
    assert executed[:5,0]==pytest.approx([-.02,-.04,-.06,-.08,-.10],abs=1e-12)
    assert np.max(np.abs(np.diff(np.r_[previous[0],executed[:,0]])))<=cfg.jerk_limit*.02+1e-12
    assert validation['envelope']['reference_initial_diagnostic_dt']==.1
    assert validation['envelope']['reference_braking_begins_s']==.1
    assert validation['envelope']['braking_comparison_satisfied']


def test_soft_mode_still_accepts_a_safe_executed_schedule():
    cfg,*_=recovery_case(.6)
    s,candidate,applied=prepared(cfg,[0.,0.,0.,.5,0.,0.],[0.,0.,0.],[[0.,0.,.5]]*10)
    assert s.accept(candidate,10.02);assert s.activate(10.02,s.state,applied)
    assert s.execution_validation['accepted']


def test_late_activation_preserves_original_recovery_epoch():
    cfg,initial,previous,controls=recovery_case(.6)
    s,candidate,applied=prepared(cfg,initial,previous,controls)
    now=10.42;s.last_tick=now-.02;s.state_received=s.mode_received=now
    assert s.accept(candidate,now)
    assert not s.activate(now,s.state,applied)
    diagnostic=s.execution_validation['envelope']
    assert diagnostic['recovery_epoch']==10.02
    assert diagnostic['recovery_time_elapsed_at_activation']==pytest.approx(.4)
    assert not diagnostic['recovery_deadline_satisfied']


def test_schedule_reuses_live_first_tick_and_does_not_mutate_supervisor():
    from aims_mpcc.execution import execution_schedule,prospective_status
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,minimum_drive_speed=.2,enforce_corridor=False)
    initial=[0.,0.,0.,.1,0.,.05]
    controls=[[.1,.06,.2],[.2,.05,.2]]+[[.2,.05,.2]]*8
    s,candidate,applied=prepared(cfg,initial,[0.,.05,0.],controls)
    s.last_command=Command(.01,.05);s.last_tick=9.98
    targets=[applied['speed']]
    for u in controls:targets.append(targets[-1]+u[0]*.1)
    plan=dict(candidate,execution_speed_targets=targets)
    snapshot=copy.deepcopy(s.__dict__)
    schedule=execution_schedule(s,plan,initial,applied,10.02)
    assert s.__dict__==snapshot
    actual=copy.copy(s);actual.plan=dict(plan,stamp=10.02)
    actual.status=prospective_status(s,initial[3]);actual.last_usable_update=10.02
    physical=np.asarray(initial,float)
    progress=s.progress
    for i in range(50):
        now=10.02+i*.02
        actual.state=replace(actual.state,speed=physical[3],steering=physical[5])
        actual.state_received=actual.mode_received=now;actual.progress=progress
        command=actual.command(now,enforce_plan_age=False);wire=actual.actuator_command(command)
        assert schedule['internal'][i]==pytest.approx([command.speed,command.steering,
             actual.last_acceleration if actual.active else 0.,actual.last_steering_rate if actual.active else 0.])
        assert schedule['wire'][i]==pytest.approx([wire.speed,wire.steering])
        physical=schedule['states'][i+1];progress=schedule['physical_progress'][i+1]
    assert schedule['elapsed'][0]==pytest.approx(.04)
    assert schedule['internal'][0,0]<cfg.minimum_drive_speed
    assert schedule['wire'][0,0]==cfg.minimum_drive_speed
    assert schedule['states'][0,3]==.1  # never initialize physical speed from wire/internal target


@pytest.mark.parametrize('status,good,expected',[('STOPPING',0,'STOPPING'),
                                               ('RECOVERING',0,'RECOVERING'),
                                               ('RECOVERING',1,'RUNNING')])
def test_gate_uses_prospective_status_and_stopping_context(status,good,expected):
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s,candidate,applied=prepared(cfg,[0.,0.,0.,.5,0.,0.],[0.,0.,0.],[[.1,0.,.5]]*10)
    s.status=status;s.recovery_good_candidates=good
    assert s.accept(candidate,10.02);assert s.activate(10.02,s.state,applied)
    validation=s.plan['execution_validation']
    assert validation['context']['status']==expected
    assert s.status==expected
    acceleration=validation['controls'][0][0]
    assert acceleration==pytest.approx(.02 if expected=='RUNNING' else -.02,abs=1e-12)


@pytest.mark.parametrize('field',['targets','progress','lap_goal','last_tick','acceleration','status'])
def test_execution_fingerprint_changes_with_output_context(field):
    from aims_mpcc.execution import execution_fingerprint
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    initial=[0.,0.,0.,.5,0.,0.]
    s,candidate,applied=prepared(cfg,initial,[0.,0.,0.],[[.1,0.,.5]]*10)
    plan=dict(candidate,execution_speed_targets=[.5+i*.01 for i in range(11)])
    before=execution_fingerprint(s,plan,initial,applied,10.02)
    if field=='targets':plan['execution_speed_targets'][1]+=.001
    elif field=='progress':s.progress+=.01
    elif field=='lap_goal':s.lap_goal+=.01
    elif field=='last_tick':s.last_tick+=.001
    elif field=='acceleration':s.last_acceleration+=.001
    else:s.status='STOPPING'
    assert execution_fingerprint(s,plan,initial,applied,10.02)!=before


def test_inconsistent_internal_and_selector_steering_prefix_cannot_activate():
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s,candidate,applied=prepared(cfg,[0.,0.,0.,.5,0.,.1],[0.,.1,0.],[[0.,.1,.5]]*10)
    s.last_command=Command(.5,0.)
    old=dict(source_stamp=9.99,stamp=10.);s.plan=old
    assert s.accept(candidate,10.02)
    assert not s.activate(10.02,s.state,applied)
    assert s.plan is old
    assert s.execution_validation['hard_control_violation']>.08


def test_actual_first_elapsed_interval_is_preserved_by_gate():
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s,candidate,applied=prepared(cfg,[0.,0.,0.,.5,0.,0.],[0.,0.,0.],[[.1,0.,.5]]*10)
    s.last_tick=9.98
    assert s.accept(candidate,10.02);assert s.activate(10.02,s.state,applied)
    assert s.execution_validation['output_intervals_s'][0]==pytest.approx(.04)
    assert s.execution_validation['controls'][0][0]==pytest.approx(.04)


def test_near_finish_uses_projected_physical_motion_not_optimizer_progress_speed():
    from aims_mpcc.execution import execution_schedule,validate_execution
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    initial=[0.,0.,0.,.5,0.,0.]
    controls=[[0.,0.,1.5]]*10  # virtual progress is three times physical speed
    s,candidate,applied=prepared(cfg,initial,[0.,0.,0.],controls)
    s.lap_goal=.65
    path=SimpleNamespace(frame_id='odom',length=100.,left_width=1.,right_width=1.,
                         project=lambda xy:(float(xy[0])%100.,float(xy[1])))
    plan=dict(candidate,execution_speed_targets=[.5]*11)
    validation=validate_execution(s,plan,initial,applied,10.02,path)
    assert validation['accepted'],validation['reason']
    assert validation['context']['progress_model']=='projected_nominal_physical_position'
    assert validation['controls'][0][0]<0.  # physical finish cap is applied
    np.testing.assert_allclose(validation['physical_progress'],np.asarray(validation['states'])[:,0],atol=1e-12)
    assert validation['physical_progress'][-1]<s.lap_goal
    assert validation['states'][-1][4]>s.lap_goal
    # Independently drive the live numerical executor from those modeled
    # physical observations and projections; optimizer theta never feeds it.
    actual=copy.copy(s);actual.plan=dict(plan,stamp=10.02);actual.last_usable_update=10.02
    for i,state in enumerate(validation['states'][:-1]):
        now=10.02+i*.02
        actual.state=replace(actual.state,x=state[0],y=state[1],yaw=state[2],
                             speed=state[3],steering=state[5])
        actual.progress=path.project(state[:2])[0]
        actual.state_received=actual.mode_received=now
        command=actual.command(now,enforce_plan_age=False)
        assert [command.speed,command.steering]==pytest.approx(validation['internal_commands'][i][:2])
    snapshot=copy.deepcopy(s.__dict__)
    execution_schedule(s,plan,initial,applied,10.02,path)
    assert s.__dict__==snapshot


def test_map_alignment_is_used_for_physical_progress_projection():
    from aims_mpcc.execution import execution_schedule
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    initial=[0.,0.,0.,.5,0.,0.]
    s,candidate,applied=prepared(cfg,initial,[0.,0.,0.],[[0.,0.,1.5]]*10)
    path=SimpleNamespace(frame_id='map',length=100.,left_width=1.,right_width=1.,
                         project=lambda xy:((float(xy[1])+2.)%100.,float(xy[0])-4.))
    plan=dict(candidate,execution_speed_targets=[.5]*11,map_alignment=[4.,-2.,np.pi/2])
    schedule=execution_schedule(s,plan,initial,applied,10.02,path)
    np.testing.assert_allclose(schedule['physical_progress'],schedule['states'][:,0],atol=1e-12)


def test_startup_acceleration_proposal_does_not_catch_up_past_raw_envelope_bound():
    # Recorded QP startup witness: internal and wire speeds BOTH start at zero;
    # no minimum-drive mapping is involved. Speed-target catch-up previously
    # requested a=.5 although the accepted raw tail was a=.4879345.
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    angles=[.018811171532362166,.048500264023987355,.0830530116554766,.11804330638595645,
            .15045975119839408,.17861679007908948,.20205181293919747,.2213103773220904,
            .2376105181814578,.2524562267031009]
    controls=[[a,d,.5] for a,d in zip([.1,.2,.3,.4]+[.4879345]*6,angles)]
    s,candidate,applied=prepared(cfg,[0.,0.,0.,0.,0.,0.],[0.,0.,0.],controls)
    assert candidate['validation']['envelope']['terminal_utilization']<.97
    assert s.accept(candidate,10.02)
    assert s.activate(10.02,s.state,applied)
    executed=np.asarray(s.execution_validation['controls'])
    assert np.max(executed[:,0])<=.4879345+1e-12
    assert s.execution_validation['envelope']['future_slack_max']<=1e-4
    assert np.max(np.abs(np.diff(np.r_[0.,executed[:,0]])))<=cfg.jerk_limit*.02+1e-12
