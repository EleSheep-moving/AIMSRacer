"""Frozen RUNNING finish-cap witness and measured-steering forecast regressions."""
import copy
import numpy as np
import pytest
from aims_mpcc import validation
from aims_mpcc.runtime import Supervisor,State,Command
from aims_mpcc.path import ReferencePath
from aims_mpcc.execution import execution_schedule
from test_recovery_braking_capacity import POINTS,native_nominal_rollout,recovering

# ros-shadow-qp-shared-912d3d1-01 seq201 snapshot SHA256: 412a9da6b754268de7476e3c6bbb67abffa1b6a3532b3d0594fd4dab348433fe
FROZEN_RUNNING = {'supervisor': {'config': {'profile': 'synthetic',
                           'wheelbase': 0.36,
                           'rear_offset': 0.18,
                           'half_length': 0.5,
                           'front_extent': None,
                           'rear_extent': None,
                           'half_width': 0.3,
                           'geometry_verified': True,
                           'enforce_corridor': True,
                           'cruise_speed': 1.0,
                           'max_speed': 1.5,
                           'minimum_drive_speed': 0.0,
                           'steer_limit': 0.4,
                           'steer_rate': 0.5,
                           'steer_acceleration': 2.0,
                           'accel_limit': 0.5,
                           'brake_limit': 0.5,
                           'jerk_limit': 1.0,
                           'steering_tau': 0.115,
                           'understeer_coefficient': 0.0,
                           'lateral_accel_limit': 1.0,
                           'longitudinal_envelope_accel': None,
                           'longitudinal_envelope_brake': None,
                           'envelope_soft_enabled': False,
                           'envelope_slack_limit': 0.5,
                           'envelope_slack_weight': 10000.0,
                           'envelope_recovery_time': 0.6,
                           'acados_envelope_margin': 0.01,
                           'acados_rti_steps': 1,
                           'optimization_envelope_margin': 0.01,
                           'recovery_jerk_enabled': False,
                           'recovery_jerk_limit': 2.0,
                           'contour_scale': 0.05,
                           'lag_scale': 0.2,
                           'heading_scale': 0.05,
                           'speed_scale': 0.6,
                           'steering_scale': 0.314159,
                           'acceleration_scale': 2.0,
                           'contour_weight': 1.0,
                           'heading_weight': 4.0,
                           'speed_weight': 133.33333333333334,
                           'steering_weight': 0.6666666666666666,
                           'steering_rate_weight': 0.3,
                           'steering_acceleration_weight': 0.6,
                           'terminal_weight': 3.0,
                           'solver_max_iterations': 35},
                'length': 12.565068635988721,
                'PLAN_TTL': 0.8,
                'handover_delay': 0.02,
                'solve_period': 0.05,
                'last_usable_update': 21247.126704422,
                'consecutive_failures': 0,
                'recovery_good_candidates': 0,
                'status': 'RUNNING',
                'reason': '',
                'state': {'x': 0.2756892562629288,
                          'y': -1.9742662137392721,
                          'yaw': 0.13893590823475652,
                          'speed': 0.9999999999999944,
                          'steering': 0.17864658921075546,
                          'timestamp': 1791498604.4442341},
                'state_received': 21247.20335902839,
                'mode_received': 21247.205674143,
                'mode': True,
                'generation': 1,
                'rejected_plans': 0,
                'started': 21236.008679147,
                'last_tick': 21247.186856854,
                'progress': 22.266329779949302,
                'wrapped_progress': 9.701261143960597,
                'start_progress': 12.565068635988721,
                'lap_goal': 25.130137271977443,
                'cross_track': 0.006577905024596067,
                'heading_error': 0.00019169965911336462,
                'last_command': {'speed': 1.0, 'steering': 0.17864982606975816},
                'last_acceleration': 0.0,
                'last_steering_rate': 1.6822658450151332e-05,
                'stationary_since': None},
 'candidate': {'success': True,
               'status': 'solved',
               'iterations': 175,
               'constraint_violation': 2.2632720350634905e-16,
               'states': [[0.21945445578206507,
                           -1.9813165125770429,
                           6.393693060250298,
                           1.0,
                           22.209471867088684,
                           0.1786452637042468],
                          [0.3185262880632943,
                           -1.9678007231274834,
                           6.443851924185981,
                           1.0000000094969366,
                           22.309461506748843,
                           0.17864832642601447],
                          [0.4167958112315678,
                           -1.9493343017503795,
                           6.494023729252812,
                           1.000000441085879,
                           22.40945116846105,
                           0.17875795407608364],
                          [0.5140159760045697,
                           -1.925960228191948,
                           6.544252790340129,
                           1.0000204764973422,
                           22.509441853417147,
                           0.17904720363997825],
                          [0.6099837587094884,
                           -1.8977151689759968,
                           6.594610296429071,
                           1.0009505784627106,
                           22.609480040319976,
                           0.17947474330774502],
                          [0.7045194741939423,
                           -1.8646261966197892,
                           6.645160357052858,
                           1.0024482556905459,
                           22.709639603605705,
                           0.17994687804354126],
                          [0.7969390670431357,
                           -1.826936989621354,
                           6.695664861228221,
                           0.9939459329183822,
                           22.809448970901006,
                           0.18038111663618703],
                          [0.8861273946331218,
                           -1.7852276807526701,
                           6.745598003401956,
                           0.9754436101462196,
                           22.907908245814237,
                           0.1807350295805891],
                          [0.9710715399122057,
                           -1.740265844194058,
                           6.794425093899271,
                           0.9469412873740526,
                           23.004017531953405,
                           0.1810043351235278],
                          [1.051076191110406,
                           -1.692862874169679,
                           6.841731191487327,
                           0.913101512775554,
                           23.097010036181434,
                           0.18120634640330846],
                          [1.126035923417558,
                           -1.6436036526930808,
                           6.887405649708573,
                           0.8809810528183128,
                           23.18670487038246,
                           0.18136429607157353]],
               'controls': [[9.496934776472552e-08, 0.17865135928664913, 0.9998963966011352],
                            [4.315889371293127e-06, 0.17892908306520489, 0.9998966171212619],
                            [0.00020035411467884837, 0.1794178910720585, 0.9999068495612315],
                            [0.009301019653706387, 0.17996431351156064, 1.0003818690287027],
                            [0.014976772278355166, 0.18044776442778776, 1.0015956328565057],
                            [-0.08502322772164485, 0.18081672023246023, 0.99809367295358],
                            [-0.1850232277216449, 0.18107649210347096, 0.9845927491320626],
                            [-0.2850232277216449, 0.18125921375140694, 0.9610928613918173],
                            [-0.33839774598501055, 0.18139815962565467, 0.9299250422805834],
                            [-0.3212045995724217, 0.18151797483601123, 0.8969483420108374]],
               'corridor_enforced': True,
               'envelope_soft_enabled': False,
               'execution_authorized': False,
               'solve_time_s': 0.022856455001601717,
               'kind': 'result',
               'backend': 'qp',
               'generation': 1,
               'solve_sequence': 201,
               'stamp': 21247.146704422,
               'previous_steering': 0.17864887619071335,
               'handover_command': {'speed': 1.0,
                                    'steering': 0.17864887619071335,
                                    'acceleration': 0.0,
                                    'steering_rate': 1.6822658451749335e-05},
               'submitted_at': 21247.126704422,
               'source_stamp': 21247.1234686986,
               'worker_started_at': 21247.179574419,
               'worker_finished_at': 21247.20490916,
               'validation_applied': [0.0, 0.17864887619071335, 1.6822658451749335e-05],
               'map_alignment': None,
               'dt': 0.1,
               'validation': {'accepted': True,
                              'candidate_fingerprint': '6970ffc917cecd11bd3b1cd1135726169ff0766218f5254c7365220193754f31',
                              'dynamics_max_residual': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              'actual_steering_bound_violation': 0.0,
                              'hard_ok': True,
                              'envelope_ok': True,
                              'minimum_margin_m': 0.5887702878870306,
                              'envelope': {'initial_acceleration_interval': [-0.1, 0.1],
                                           'initial_actuator_jerk_feasible': True,
                                           'minimum_initial_utilization': 0.2515866137548597,
                                           'initial_unavoidable_violation': 0.0,
                                           'initial_lateral_utilization': 0.2515866137548597,
                                           'initial_lateral_violation': 0.0,
                                           'initial_candidate_utilization': 0.25158661375489577,
                                           'future_slack_max': 0.0,
                                           'future_violation_duration_s': 0.0,
                                           'terminal_utilization': 0.5689907163149382,
                                           'candidate_excess_integral': 0.0,
                                           'candidate_lateral_excess_integral': 0.0,
                                           'hard_control_violation': 4.163336342344337e-17,
                                           'speed_bound_violation': 0.0,
                                           'actual_steering_bound_violation': 0.0,
                                           'recovery_needed': False,
                                           'recovery_bounds_satisfied': True,
                                           'recovery_deadline_satisfied': True,
                                           'initial_envelope_satisfied': True,
                                           'active_envelope_slack_limit': 0.0,
                                           'execution_authorized': False,
                                           'active_jerk_limits': [1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0,
                                                                  1.0],
                                           'diagnostic_scope': 'independent_strict_bounds',
                                           'braking_comparison_performed': False},
                              'reason': ''},
               'worker_validation_time_s': 0.002390090998233063,
               'discarded': False,
               'skip_notified': False,
               'request_age_s': 0.07986235700082034},
 'args': [21247.206566779,
          {'x': 0.2788657379855445,
           'y': -1.9738194188986373,
           'yaw': 0.14054487755977635,
           'speed': 1.0,
           'steering': 0.1786466782990817,
           'timestamp': 1791498604.447442},
          {'speed': 1.0,
           'steering': 0.17864982783794403,
           'acceleration': 0.0,
           'steering_rate': 1.6822658450151332e-05}],
 'kwargs': {'expected_at_activation': {'x': 0.2788436207672805,
                                       'y': -1.9738225386642385,
                                       'yaw': 6.4237191929894415,
                                       'speed': 1.0,
                                       'steering': 0.17864670958622633,
                                       'timestamp': 1791498604.5040975},
            'progress': 22.269547758227414,
            'map_alignment': None}}


def running_frozen():
    from aims_mpcc.config import VehicleConfig
    b=copy.deepcopy(FROZEN_RUNNING['supervisor']);cfg=VehicleConfig(**b['config'])
    s=Supervisor(cfg,b['length'],b['PLAN_TTL'],b['handover_delay'],b['solve_period'])
    for k,v in b.items():
        if k=='config':v=cfg
        elif k=='state':v=State(**v)
        elif k=='last_command':v=Command(**v)
        setattr(s,k,v)
    s.pending_plan=copy.deepcopy(FROZEN_RUNNING['candidate'])
    now,actual,applied=copy.deepcopy(FROZEN_RUNNING['args'])
    kwargs=copy.deepcopy(FROZEN_RUNNING['kwargs']);kwargs['expected_at_activation']=State(**kwargs['expected_at_activation'])
    kwargs['path']=ReferencePath(POINTS,.9,.9)
    return s,now,State(**actual),applied,kwargs


def test_frozen_running_finish_cap_passes_unchanged_all_activation_gates(monkeypatch):
    s,now,actual,applied,kwargs=running_frozen();prefix=(s.last_command,s.last_acceleration,s.last_steering_rate)
    assert s.status=='RUNNING' and s.consecutive_failures==0
    raw=validation.validate_candidate(s.pending_plan,s.config,kwargs['path']);assert raw['accepted']
    gates=[];original=validation.validate_candidate
    def recorded(*args,**kw):
        v=original(*args,**kw);gates.append(v);return v
    monkeypatch.setattr(validation,'validate_candidate',recorded)
    assert s.activate(now,actual,applied,**kwargs)
    assert gates[0]['accepted']
    e=s.execution_validation
    assert e['accepted'] and e['context']['status']=='RUNNING'
    assert e['envelope']['future_slack_max']==0.
    assert e['envelope']['terminal_utilization']==pytest.approx(.9931317998945491,abs=1e-12)
    assert (s.last_command,s.last_acceleration,s.last_steering_rate)==prefix
    assert s.status=='RUNNING' and s.PLAN_TTL==.8 and s.config.jerk_limit==1.
    trace=np.asarray(e['internal_commands']);intervals=np.asarray(e['output_intervals_s'])
    assert np.max(np.abs(np.diff(np.r_[prefix[1],trace[:,2]]))-s.config.jerk_limit*intervals)<1e-12


def running_override(speed,physical_steering,acceleration,target,internal):
    s=recovering(speed,physical_steering,acceleration,0.,internal);s.status='RUNNING'
    s.started=9.;s.last_usable_update=10.
    s.plan=dict(stamp=10.,source_stamp=10.,dt=.1,
                states=[[0.,0.,0.,speed,0.,physical_steering]]*11,
                controls=[[0.,physical_steering,speed]]*10,
                execution_speed_targets=[target]*11,previous_steering=physical_steering)
    return s


@pytest.mark.parametrize('target,internal,a,sign',[(1.1,.4,.45,1.),(1.1,1.2,-.45,-1.)])
def test_running_override_caps_both_signed_axes_without_reset(target,internal,a,sign):
    s=running_override(.8,.25,a,target,internal)
    measured=s.state;s.command(10.);budget=s.braking_budget
    assert budget['scope']=='running_speed_override'
    assert budget['feasible'] and budget['achieved_within_capacity']
    assert s.state is measured and s.last_command.speed!=s.state.speed
    assert abs(s.last_acceleration-a)<=.020001
    assert -budget['brake_capacity']-1e-12<=s.last_acceleration<=budget['accel_capacity']+1e-12
    assert sign*s.last_acceleration>0.


def test_running_override_infeasible_budget_keeps_bounded_emergency_continuation():
    s=running_override(1.,.18,-.5,1.1,1.2);s.command(10.)
    d=s.braking_budget
    assert not d['feasible'] and not d['achieved_within_capacity']
    assert d['reason']=='jerk and nominal capacity intervals do not intersect'
    assert s.last_acceleration==pytest.approx(-.5,abs=1e-12)
    assert d['brake_capacity']<.48


def test_ordinary_running_does_not_use_override_budget():
    s=running_override(.4,.1,.1,.5,.4);s.plan['controls']=[[.1,.1,.4]]*10
    s.command(10.)
    assert s.braking_budget is None and s.last_acceleration==pytest.approx(.1,abs=1e-12)


@pytest.mark.parametrize('cap',['cruise','max'])
def test_far_finish_override_restarts_full_physical_steering_schedule(cap):
    # Far from finish, an override still depends on the evolving measured
    # angle. The scalar-v shortcut must discard its clone and replay exactly.
    angle=np.arange(64)*2*np.pi/64
    path=ReferencePath(np.c_[8*np.cos(angle),8*np.sin(angle)],2.,2.)
    ref=path.at(0.)
    s=running_override(1.,.25,-.4,1.1,1.2)
    if cap=='max':s.config.cruise_speed=1.;s.config.max_speed=1.;s.last_command=Command(.9,.02)
    else:s.last_command=Command(1.2,.02)
    s.state=State(ref['x'],ref['y'],ref['yaw'],1.,.25,100.)
    s.length=path.length;s.progress=0.;s.lap_goal=100.
    s.plan['controls']=[[0.,.02,1.]]*10
    s.plan['previous_steering']=.02
    s.plan['execution_speed_targets']=[1.1 if cap=='cruise' else 1.2]*11
    initial=[ref['x'],ref['y'],ref['yaw'],1.,0.,.25]
    applied=dict(speed=s.last_command.speed,steering=.02,acceleration=-.4,steering_rate=0.)
    before=copy.deepcopy(s.__dict__)
    fast=execution_schedule(s,s.plan,initial,applied,10.,path)
    slow=execution_schedule(s,s.plan,initial,applied,10.,path,force_slow=True)
    assert s.__dict__==before
    for k in ('controls','states','internal','wire','elapsed'):
        assert np.array_equal(fast[k],slow[k]),k
    assert fast['physical_progress']==slow['physical_progress']
    assert fast['statuses']==slow['statuses']
    assert not fast['context']['finish_independent_certificate']['proven']


@pytest.mark.parametrize('scalar',[np.float32,np.float64])
def test_numpy_running_override_budget_keeps_native_json_types(scalar):
    import json
    from dataclasses import replace
    s=running_override(.8,.25,.45,1.1,.4)
    s.state=replace(s.state,speed=scalar(s.state.speed),steering=scalar(s.state.steering))
    s.last_command=Command(scalar(s.last_command.speed),scalar(s.last_command.steering))
    s.last_acceleration=scalar(s.last_acceleration);s.command(10.)
    d=s.braking_budget
    assert d['scope']=='running_speed_override'
    json.dumps(d,allow_nan=False)
    assert all(v is None or type(v) in (str,bool,float) for v in d.values())
