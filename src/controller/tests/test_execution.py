"""Validate the nominal emitted schedule separately from optimizer intervals."""
import copy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import evaluate_envelope,independent_rollout,utilization
from aims_mpcc.runtime import Command,State,Supervisor
from aims_mpcc.path import ReferencePath
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
    if status=='STOPPING':
        from aims_mpcc.execution import validate_execution
        # The helper can diagnose stopping, but operator stop accepts no new
        # plan and cannot renew an old plan's validity through a late result.
        assert not s.accept(candidate,10.02)
        validation=validate_execution(s,candidate,candidate['states'][0],applied,10.02)
        assert s.plan is None
    else:
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


def test_recorded_turn_reversal_uses_rate_proposal_without_angle_catch_up():
    # Frozen seq18 from the 20 Hz QP ROS lap: raw/rebased E<1 but the
    # previous angle catch-up smoother overshot the endpoints, E=1.00238829.
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    initial=[1.9997559271901302, 0.05921209287385421, 1.5809914758518657, 0.21619726072657378, 12.624264524554068, 0.09566828730884487]
    controls=[
        [0.48793451625015394, 0.14968746946174596, 0.24149355651882456],
        [0.48793451625015394, 0.1933651935578563, 0.29028195274563434],
        [0.48793451625015394, 0.2277150618404539, 0.3390703489718037],
        [0.48793451625015394, 0.24469071265235384, 0.38785874519622177],
        [0.48793451625015394, 0.24290293610453947, 0.4366471414237709],
        [0.48793451625015394, 0.22557466845468535, 0.4854355376487038],
        [0.48793451625015394, 0.1987559704364171, 0.534223933875227],
        [0.48613245396277427, 0.16950181874726572, 0.5829222363213816],
        [0.4856348097942709, 0.1428820011981779, 0.6315055653608879],
        [0.4856348097942709, 0.11895978304652849, 0.6800640147651503]]
    previous=[0.48793451625015505, 0.11363961547613144, 0.16047850432368535]
    now=392542.330401044
    s,candidate,actual=prepared(cfg,initial,previous,controls,now)
    s.last_command=Command(0.3054128769332148,0.11363961902937746)
    s.last_tick=392542.31039285
    actual['speed']=0.3054128885269165
    assert candidate['validation']['accepted']
    assert s.accept(candidate,now)
    assert s.activate(now,s.state,actual)
    executed=s.execution_validation
    assert executed['envelope']['future_slack_max']==0.
    assert executed['envelope']['terminal_utilization']<.994
    # Preserve live angle/rate: the first output advances .02 from the actual
    # prior rate through the unchanged 2 rad/s^2 smoother, never resets to pose.
    elapsed=now-s.last_tick
    rate=min((controls[0][1]-actual['steering'])/.1,previous[2]+cfg.steer_acceleration*elapsed)
    assert executed['wire_commands'][0][1]==pytest.approx(s.last_command.steering+rate*elapsed,abs=1e-12)
    trace=np.asarray(executed['internal_commands'])
    intervals=np.asarray(executed['output_intervals_s'])
    assert np.all(np.abs(np.diff(np.r_[previous[2],trace[:,3]]))<=cfg.steer_acceleration*intervals+1e-12)
    assert np.max(np.abs(trace[:,3]))<=cfg.steer_rate+1e-12
    assert np.max(np.abs(trace[:,1]))<=cfg.steer_limit+1e-12


@pytest.mark.parametrize('status,solve_period,remaining,expected_rate',[
    ('RUNNING',.2,100.,.14),('RUNNING',.2,.4,.14),
    ('STOPPING',.2,100.,.06),('RECOVERING',.2,100.,.06),
    ('RUNNING',None,100.,.06)])
def test_rate_proposals_preserve_status_and_finish_braking_context(status,solve_period,remaining,expected_rate):
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s,candidate,_=prepared(cfg,[0.,0.,0.,.3,0.,.02],[0.,.05,.1],[[0.,.07,.3]]*10)
    s.status=status;s.solve_period=solve_period;s.plan=candidate
    s.plan['stamp']=10.;s.last_tick=10.02
    s.last_command=Command(.3,.06);s.last_steering_rate=.1
    s.lap_goal=s.progress+remaining;s.state_received=s.mode_received=10.04
    s.command(10.04)
    assert s.last_steering_rate==pytest.approx(expected_rate,abs=1e-12)
    assert s.last_command.steering==pytest.approx(.06+expected_rate*.02,abs=1e-12)
    if remaining<1. or status in ('STOPPING','RECOVERING'):
        assert s.last_acceleration<0.


def test_repeated_handover_continues_live_steering_angle_and_rate():
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    initial=[0.,0.,0.,.2,0.,.02]
    s,candidate,applied=prepared(cfg,initial,[0.,.1,.04],[[0.,.12,.2]]*10)
    assert s.accept(candidate,10.02);assert s.activate(10.02,s.state,applied)
    s.command(10.02)
    first=s.last_command.steering;rate=s.last_steering_rate
    assert first==pytest.approx(.1016,abs=1e-12)
    assert rate==pytest.approx(.08,abs=1e-12)
    # A reversed second plan starts from the actual emitted target/rate while
    # physical steering remains different. Neither internal state is reset.
    initial[5]=.025
    _,second,actual=prepared(cfg,initial,[0.,first,rate],[[0.,first-.01,.2]]*10,10.04)
    s.state=State(0.,0.,0.,.2,.025,100.04)
    s.state_received=s.mode_received=10.04
    assert s.accept(second,10.04);assert s.activate(10.04,s.state,actual)
    certificate=s.execution_validation
    assert s.last_command.steering==first and s.last_steering_rate==rate
    s.command(10.04)
    assert s.last_steering_rate==pytest.approx(.04,abs=1e-12)
    assert s.last_command.steering==pytest.approx(first+.04*.02,abs=1e-12)
    assert certificate['wire_commands'][0][1]==pytest.approx(s.last_command.steering,abs=1e-12)


def projected_case(seed=0,remaining=None,branches=False):
    rng=np.random.default_rng(seed);angle=np.arange(64)*2*np.pi/64
    if branches:points=np.c_[8*np.cos(angle),.3*np.sin(angle)]
    else:
        radius=3.+.6*np.cos(3*angle)+.15*np.sin(5*angle)
        points=np.c_[radius*np.cos(angle),radius*np.sin(angle)]
    frame='map' if seed%2 else 'odom'
    path=ReferencePath(points,1.2,1.,frame,{'map_sha256':'a'*64} if frame=='map' else None)
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    theta=float(rng.uniform(0.,path.length));ref=path.at(theta)
    alignment=np.r_[rng.normal(size=2),rng.uniform(-.5,.5)] if frame=='map' else None
    if branches:
        xy=np.array([0.,.02]);yaw=-np.pi/2;theta=path.project(xy)[0]
    else:
        xy=np.array([ref['x'],ref['y']])+rng.normal(size=2)*.025;yaw=ref['yaw']+rng.uniform(-.15,.15)
    if alignment is not None:
        c,sn=np.cos(alignment[2]),np.sin(alignment[2])
        xy=np.array([[c,sn],[-sn,c]])@(xy-alignment[:2]);yaw-=alignment[2]
    speed=.3 if branches else rng.uniform(.25,.8);steering=rng.uniform(-.1,.1)
    initial=[*xy,yaw,speed,theta,steering]
    controls=[];acceleration=0.;endpoint=steering;rate=0.
    for _ in range(10):
        acceleration=np.clip(acceleration+rng.uniform(-.08,.08),-.2,.2)
        rate=np.clip(rate+rng.uniform(-.2,.2),-.2,.2)
        endpoint=np.clip(endpoint+rate*.1,-.15,.15)
        controls.append([float(acceleration),float(endpoint),float(rng.uniform(0.,1.5))])
    s,candidate,applied=prepared(cfg,initial,[0.,steering,0.],controls)
    s.length=path.length;s.progress=theta+3*path.length
    s.lap_goal=s.progress+(path.length if remaining is None else remaining)
    targets=[applied['speed']]
    for control in controls:targets.append(np.clip(targets[-1]+control[0]*.1,0.,cfg.max_speed))
    return s,dict(candidate,execution_speed_targets=targets,map_alignment=alignment),initial,applied,path


def test_far_finish_schedule_avoids_scalar_projection_refinements(monkeypatch):
    from aims_mpcc.execution import execution_schedule
    import aims_mpcc.path as path_module
    s,plan,initial,applied,path=projected_case();calls=[];original=path_module.minimize_scalar
    def recorded(*args,**kwargs):
        calls.append(True)
        return original(*args,**kwargs)
    monkeypatch.setattr(path_module,'minimize_scalar',recorded)
    schedule=execution_schedule(s,plan,initial,applied,10.02,path)
    assert calls==[]
    assert schedule['context']['finish_independent_certificate']['proven']


@pytest.mark.parametrize('seed',range(40))
def test_finish_independent_schedule_matches_forced_physical_projection(seed):
    from aims_mpcc.execution import execution_schedule
    s,plan,initial,applied,path=projected_case(seed)
    snapshot=copy.deepcopy(s.__dict__)
    fast=execution_schedule(s,plan,initial,applied,10.02,path)
    slow=execution_schedule(s,plan,initial,applied,10.02,path,force_slow=True)
    assert fast['context']['finish_independent_certificate']['proven']
    assert s.__dict__==snapshot
    for key in ('controls','internal','wire','elapsed'):
        assert np.array_equal(fast[key],slow[key]),(seed,key)
    assert fast['statuses']==slow['statuses']
    np.testing.assert_allclose(fast['states'],slow['states'],rtol=0.,atol=1e-12)
    bounds=np.asarray(fast['physical_progress_bounds_m'])
    assert np.all(np.asarray(slow['physical_progress'])>=bounds[:,0]-1e-12)
    assert np.all(np.asarray(slow['physical_progress'])<=bounds[:,1]+1e-12)
    assert fast['physical_progress'] is None  # bounds never claim exact projection


@pytest.mark.parametrize('remaining',[.15,.65,2.3,3.8])
def test_near_finish_schedule_falls_back_to_physical_projection(remaining):
    from aims_mpcc.execution import execution_schedule
    s,plan,initial,applied,path=projected_case(4,remaining)
    if remaining==.15:
        # A stopped snapshot exercises finish completion instead of asking a
        # moving vehicle to stop inside an infeasible remaining distance.
        initial[3]=0.;applied['speed']=0.;s.state=replace(s.state,speed=0.)
        s.last_command=Command(0.,initial[5]);s.config.cruise_speed=0.
        plan['controls']=[[0.,initial[5],0.]]*10;plan['execution_speed_targets']=[0.]*11
    actual=execution_schedule(s,plan,initial,applied,10.02,path)
    slow=execution_schedule(s,plan,initial,applied,10.02,path,force_slow=True)
    if remaining<3.8:
        assert not actual['context']['finish_independent_certificate']['proven']
    for key in ('controls','internal','wire','elapsed','states'):
        assert np.array_equal(actual[key],slow[key])
    if remaining==.15:assert actual['statuses'][-1]=='COMPLETE'


def test_near_branch_switch_does_not_certify_an_ambiguous_unwrap():
    from aims_mpcc.execution import execution_schedule
    s,plan,initial,applied,path=projected_case(0,branches=True)
    actual=execution_schedule(s,plan,initial,applied,10.02,path)
    slow=execution_schedule(s,plan,initial,applied,10.02,path,force_slow=True)
    assert not actual['context']['finish_independent_certificate']['proven']
    for key in ('controls','internal','wire','elapsed','states'):
        assert np.array_equal(actual[key],slow[key])


def test_instance_projection_override_uses_actual_projector_and_slow_fallback():
    from aims_mpcc.execution import execution_schedule
    s,plan,initial,applied,path=projected_case()
    original=path.project
    path.project=lambda xy:original(xy)
    actual=execution_schedule(s,plan,initial,applied,10.02,path)
    slow=execution_schedule(s,plan,initial,applied,10.02,path,force_slow=True)
    assert not actual['context']['finish_independent_certificate']['proven']
    for key in ('controls','internal','wire','elapsed','states'):
        assert np.array_equal(actual[key],slow[key])
