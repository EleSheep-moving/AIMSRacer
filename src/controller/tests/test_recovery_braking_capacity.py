"""Nominal recovery braking respects lateral capacity without changing gates.

Frozen NX ffbfbab case03 seq22: raw/rebased pass; old RECOVERING override
reached -.5 with nonzero lateral load, Emax=1.007844599815307. Snapshot
SHA256 is recorded beside the embedded fixture for reproducible provenance.
"""
import copy
from dataclasses import replace
import math

import numpy as np
import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.envelope import independent_rollout, utilization
from aims_mpcc.path import ReferencePath
from aims_mpcc.runtime import Command, State, Supervisor
from aims_mpcc import validation

# activation-first-rejection.json: d08f956d15f20aa3734688244bad6dd170b6ff6d2b3e127a0194f4a237f8af06
FROZEN = {'supervisor': {'config': {'profile': 'synthetic',
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
                'last_usable_update': 19609.24364423,
                'consecutive_failures': 2,
                'recovery_good_candidates': 0,
                'status': 'RECOVERING',
                'reason': 'Invalid, failed, or expired solver result',
                'state': {'x': 1.9917780620246521,
                          'y': 0.21027422025546055,
                          'yaw': 1.6627046626787856,
                          'speed': 0.4374913635360486,
                          'steering': 0.20870749036747493,
                          'timestamp': 1791496966.7108395},
                'state_received': 19609.469966501998,
                'mode_received': 19609.481525331,
                'mode': True,
                'generation': 1,
                'rejected_plans': 0,
                'started': 19608.044477178,
                'last_tick': 19609.462332314,
                'progress': 12.775409852525279,
                'wrapped_progress': 0.21034121653655297,
                'start_progress': 12.565068635988721,
                'lap_goal': 25.130137271977443,
                'cross_track': -0.002846748023345106,
                'heading_error': -0.013273170087402697,
                'last_command': {'speed': 0.5322181960101158, 'steering': 0.21571200306642282},
                'last_acceleration': 0.3676439202484272,
                'last_steering_rate': 0.0,
                'stationary_since': None},
 'candidate': {'success': True,
               'constraint_violation': 3.3306690738754696e-16,
               'states': [[1.992845743847124,
                           0.19821876606745512,
                           1.655943415091584,
                           0.42501045630354534,
                           12.763326297653556,
                           0.20677105663348547],
                          [1.9885320239995876,
                           0.24204719506946812,
                           1.6819599120990256,
                           0.45581843212839895,
                           12.807363179022826,
                           0.21139147361879418],
                          [1.9827155637251195,
                           0.28830367208142915,
                           1.709817024816819,
                           0.4766264079532537,
                           12.853980590582694,
                           0.21186972337374863],
                          [1.9753512745154669,
                           0.335939174580642,
                           1.7384898873138932,
                           0.4874343837781095,
                           12.902178635941787,
                           0.20973291058727483],
                          [1.966520622455118,
                           0.3839154246712907,
                           1.7670528422312086,
                           0.48824235960296364,
                           12.950957418707844,
                           0.20547467316435306],
                          [1.9564334312191645,
                           0.4312148791930624,
                           1.7946139985307394,
                           0.4790503354278189,
                           12.999317042489313,
                           0.19886610975112662],
                          [1.9454225890811725,
                           0.4768494494572024,
                           1.8202761247948773,
                           0.45985831125267246,
                           13.04625761089402,
                           0.18926749706532078],
                          [1.9339301105433147,
                           0.5198659747841007,
                           1.8431708487709917,
                           0.43066628707752713,
                           13.090779227530415,
                           0.17621605152642586],
                          [1.9224815380559555,
                           0.5593459031988701,
                           1.862593847915459,
                           0.3914742629023829,
                           13.131881996006502,
                           0.1602051032604338],
                          [1.9116105236316874,
                           0.5945176872155945,
                           1.8782385089606137,
                           0.34480312957472475,
                           13.16869205141349,
                           0.1431038737467669],
                          [1.9016928437586862,
                           0.6250961255644056,
                           1.890369500727109,
                           0.2981319962470666,
                           13.200835477038854,
                           0.1268463865449425]],
               'controls': [[0.3080797582485484, 0.2142369238858467, 0.44036881369295644],
                            [0.20807975824854835, 0.21121286629476266, 0.4661741155991192],
                            [0.10807975824854832, 0.20669353121678313, 0.48198045359091235],
                            [0.008079758248548303, 0.20027523852581766, 0.4877878276603031],
                            [-0.09192024175145172, 0.19103593082381215, 0.4835962378147234],
                            [-0.1919202417514517, 0.17803242811405906, 0.46940568404735117],
                            [-0.29192024175145176, 0.16123094284596337, 0.4452161663641994],
                            [-0.39192024175145174, 0.14241783369903538, 0.4110276847606447],
                            [-0.46671133327658637, 0.1249882383032304, 0.36810055407030634],
                            [-0.46671133327658637, 0.11022211108241457, 0.3214342562534036]],
               'generation': 1,
               'solve_sequence': 22,
               'stamp': 19609.441896476,
               'previous_steering': 0.21571200306642282,
               'handover_command': {'speed': 0.5169435083441376,
                                    'steering': 0.21571200306642282,
                                    'acceleration': 0.4080797582485484,
                                    'steering_rate': 0.0},
               'submitted_at': 19609.421896476,
               'source_stamp': 19609.40988997677,
               'validation_applied': [0.4080797582485484, 0.21571200306642282, 0.0],
               'map_alignment': None,
               'dt': 0.1,
               'validation': {'accepted': True,
                              'candidate_fingerprint': 'ef59431c07961e0bd3e47a17858b99a4979b065709baad6415e7990957da065e',
                              'dynamics_max_residual': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              'actual_steering_bound_violation': 0.0,
                              'hard_ok': True,
                              'envelope_ok': True,
                              'minimum_margin_m': 0.5878356921381519,
                              'envelope': {'initial_acceleration_interval': [0.3080797582485484, 0.5],
                                           'initial_actuator_jerk_feasible': True,
                                           'minimum_initial_utilization': 0.39073093785184,
                                           'initial_unavoidable_violation': 0.0,
                                           'initial_lateral_utilization': 0.011078388081903916,
                                           'initial_lateral_violation': 0.0,
                                           'initial_candidate_utilization': 0.39073093785184,
                                           'future_slack_max': 0.0,
                                           'future_violation_duration_s': 0.0,
                                           'terminal_utilization': 0.8722693035167118,
                                           'candidate_excess_integral': 0.0,
                                           'candidate_lateral_excess_integral': 0.0,
                                           'hard_control_violation': 5.551115123125783e-17,
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
                              'reason': ''}},
 'args': [19609.482283587,
          {'x': 1.991271383997115,
           'y': 0.2156770577794429,
           'yaw': 1.6659028699079723,
           'speed': 0.4436499060370466,
           'steering': 0.20941892921252928,
           'timestamp': 1791496966.7231567},
          {'speed': 0.5322182178497314,
           'steering': 0.21571199595928192,
           'acceleration': 0.3676439202484272,
           'steering_rate': 0.0}],
 'kwargs': {'expected_at_activation': {'x': 1.9912612366457618,
                                       'y': 0.21571982988304883,
                                       'yaw': 1.666254859578873,
                                       'speed': 0.44520401180319064,
                                       'steering': 0.2094189300110545,
                                       'timestamp': 1791496966.7512271},
            'progress': 12.780827567657404,
            'map_alignment': None}}
POINTS = [[2.0, 2.2717643490231154e-19],
 [1.9975138424378445, 0.09969177132139369],
 [1.9900615507308028, 0.199135693191632],
 [1.9776616524502575, 0.298084532352347],
 [1.960344975697088, 0.39629228639879266],
 [1.938154572458156, 0.4935147953805842],
 [1.9111456115722822, 0.589510348821805],
 [1.879385241571818, 0.6840402866513333],
 [1.8429524237408166, 0.7768695925493847],
 [1.8019377358048403, 0.8677674782351111],
 [1.75644314674046, 0.9565079572426308],
 [1.7065817632643152, 1.0428704067589905],
 [1.6524775486319931, 1.1266401161272381],
 [1.594265014445849, 1.2076088206509485],
 [1.5320888862379611, 1.2855752193730716],
 [1.466103743659658, 1.3603454755418314],
 [1.3964736361721526, 1.43173369851943],
 [1.3233716751937263, 1.4995624059354609],
 [1.2469796037174754, 1.5636629649360518],
 [1.1674873444695892, 1.6238760114317061],
 [1.0850925277315293, 1.6800518463015357],
 [1.0000000000000115, 1.7320508075688699],
 [0.9124213147063379, 1.7797436176229307],
 [0.8225742062612365, 1.8230117046233398],
 [0.7306820487328043, 1.8617474972884027],
 [0.6369733005033843, 1.8958546923342587],
 [0.5416809362860261, 1.9252484939000196],
 [0.4450418679126452, 1.9498558243636435],
 [0.34729635533387726, 1.969615506024413],
 [0.24868740929498756, 1.984478413200342],
 [0.14946018717286594, 1.9944075943623587],
 [0.04986138347616333, 1.9993783640016323],
 [-0.04986138347612848, 1.9993783640016332],
 [-0.1494601871728316, 1.9944075943623616],
 [-0.2486874092949529, 1.9844784132003461],
 [-0.34729635533384356, 1.9696155060244196],
 [-0.44504186791261113, 1.949855824363651],
 [-0.5416809362859926, 1.9252484939000292],
 [-0.6369733005033505, 1.8958546923342696],
 [-0.7306820487327714, 1.8617474972884147],
 [-0.8225742062612046, 1.823011704623354],
 [-0.9124213147063063, 1.7797436176229473],
 [-0.9999999999999803, 1.7320508075688883],
 [-1.0850925277314987, 1.6800518463015555],
 [-1.1674873444695586, 1.6238760114317268],
 [-1.2469796037174452, 1.563662964936076],
 [-1.3233716751936968, 1.4995624059354873],
 [-1.396473636172124, 1.4317336985194569],
 [-1.4661037436596314, 1.3603454755418611],
 [-1.5320888862379345, 1.2855752193731034],
 [-1.5942650144458235, 1.207608820650982],
 [-1.6524775486319692, 1.1266401161272739],
 [-1.7065817632642912, 1.0428704067590282],
 [-1.7564431467404382, 0.9565079572426707],
 [-1.8019377358048199, 0.8677674782351535],
 [-1.8429524237407984, 0.776869592549429],
 [-1.8793852415718015, 0.684040286651379],
 [-1.911145611572268, 0.5895103488218522],
 [-1.938154572458144, 0.4935147953806338],
 [-1.9603449756970777, 0.39629228639884373],
 [-1.9776616524502495, 0.29808453235239935],
 [-1.9900615507307975, 0.1991356931916855],
 [-1.9975138424378416, 0.09969177132144902],
 [-2.0, 5.620492316435386e-14],
 [-1.9975138424378476, -0.09969177132133675],
 [-1.9900615507308093, -0.19913569319157454],
 [-1.977661652450266, -0.29808453235228904],
 [-1.9603449756970992, -0.39629228639873354],
 [-1.9381545724581712, -0.49351479538052495],
 [-1.9111456115723007, -0.5895103488217454],
 [-1.879385241571839, -0.6840402866512741],
 [-1.8429524237408421, -0.7768695925493252],
 [-1.801937735804869, -0.8677674782350523],
 [-1.7564431467404913, -0.9565079572425726],
 [-1.7065817632643498, -1.0428704067589332],
 [-1.6524775486320324, -1.126640116127181],
 [-1.5942650144458914, -1.207608820650892],
 [-1.5320888862380067, -1.2855752193730179],
 [-1.4661037436597064, -1.3603454755417803],
 [-1.396473636172203, -1.4317336985193798],
 [-1.3233716751937794, -1.4995624059354142],
 [-1.2469796037175294, -1.5636629649360094],
 [-1.167487344469644, -1.6238760114316662],
 [-1.0850925277315837, -1.6800518463015006],
 [-1.0000000000000653, -1.7320508075688386],
 [-0.912421314706391, -1.7797436176229033],
 [-0.8225742062612881, -1.823011704623317],
 [-0.7306820487328544, -1.8617474972883827],
 [-0.6369733005034325, -1.895854692334242],
 [-0.5416809362860727, -1.9252484939000065],
 [-0.445041867912692, -1.9498558243636326],
 [-0.34729635533392245, -1.9696155060244047],
 [-0.2486874092950306, -1.9844784132003368],
 [-0.1494601871729069, -1.9944075943623563],
 [-0.049861383476202205, -1.9993783640016314],
 [0.049861383476091384, -1.9993783640016338],
 [0.14946018717279635, -1.9944075943623643],
 [0.24868740929492045, -1.9844784132003501],
 [0.3472963553338133, -1.9696155060244245],
 [0.44504186791258205, -1.9498558243636577],
 [0.5416809362859663, -1.9252484939000365],
 [0.6369733005033275, -1.8958546923342774],
 [0.7306820487327511, -1.8617474972884234],
 [0.822574206261187, -1.8230117046233625],
 [0.9124213147062908, -1.7797436176229544],
 [0.9999999999999675, -1.7320508075688952],
 [1.0850925277314891, -1.6800518463015617],
 [1.1674873444695508, -1.6238760114317328],
 [1.246979603717441, -1.5636629649360794],
 [1.3233716751936955, -1.4995624059354884],
 [1.3964736361721246, -1.4317336985194564],
 [1.4661037436596345, -1.360345475541858],
 [1.53208888623794, -1.2855752193730972],
 [1.5942650144458312, -1.207608820650972],
 [1.652477548631978, -1.1266401161272608],
 [1.7065817632643017, -1.0428704067590115],
 [1.756443146740448, -0.9565079572426519],
 [1.801937735804831, -0.8677674782351302],
 [1.8429524237408097, -0.7768695925494019],
 [1.8793852415718126, -0.6840402866513484],
 [1.911145611572278, -0.5895103488218175],
 [1.9381545724581537, -0.49351479538059434],
 [1.9603449756970865, -0.3962922863988003],
 [1.9776616524502564, -0.298084532352352],
 [1.9900615507308028, -0.19913569319163615],
 [1.9975138424378442, -0.09969177132139513]]


def frozen_case():
    b=copy.deepcopy(FROZEN['supervisor']);cfg=VehicleConfig(**b.pop('config'))
    s=Supervisor(cfg,b['length'],b['PLAN_TTL'],b['handover_delay'],b['solve_period'])
    for k,v in b.items():
        if k=='state':v=State(**v)
        elif k=='last_command':v=Command(**v)
        setattr(s,k,v)
    s.pending_plan=copy.deepcopy(FROZEN['candidate'])
    now,actual,applied=copy.deepcopy(FROZEN['args'])
    kwargs=copy.deepcopy(FROZEN['kwargs']);kwargs['expected_at_activation']=State(**kwargs['expected_at_activation'])
    kwargs['path']=ReferencePath(POINTS,.9,.9)
    return s,now,State(**actual),applied,kwargs


def test_frozen_nx_recovery_candidate_passes_unchanged_execution_gate(monkeypatch):
    s,now,actual,applied,kwargs=frozen_case();prefix=(s.last_command,s.last_acceleration,s.last_steering_rate)
    gates=[];original=validation.validate_candidate
    def record(candidate,*args,**kw):
        result=original(candidate,*args,**kw);gates.append(result);return result
    monkeypatch.setattr(validation,'validate_candidate',record)
    assert s.pending_plan['validation']['accepted']
    assert s.activate(now,actual,applied,**kwargs)
    assert gates[0]['accepted']  # unchanged rebased macro validator
    e=s.execution_validation
    assert e['accepted'] and e['envelope']['future_slack_max']<=1e-4
    assert e['envelope']['terminal_utilization']<=1.0001
    assert e['context']['initial'][3]==actual.speed
    assert e['context']['last_command']['speed']==prefix[0].speed
    assert (s.last_command,s.last_acceleration,s.last_steering_rate)==prefix
    assert s.status=='RECOVERING' and s.recovery_good_candidates==1
    assert s.PLAN_TTL==.8 and s.config.jerk_limit==1.
    a=np.asarray(e['controls'])[:,0]
    assert np.max(np.abs(np.diff(np.r_[prefix[1],a])))<=.020001
    assert a[0]==pytest.approx(.3476926472488199,abs=1e-12)


def test_frozen_witness_still_rejects_original_full_brake_proposal(monkeypatch):
    s,now,actual,applied,kwargs=frozen_case()
    # Remove only the new proposal cap; retain every original numerical gate
    # and the actual frozen prefix/status. This reproduces NX before the fix.
    monkeypatch.setattr(Supervisor,'recovery_braking_budget',lambda *args:
                        dict(available=False,feasible=False))
    assert not s.activate(now,actual,applied,**kwargs)
    e=s.execution_validation
    assert e['envelope']['future_slack_max']==pytest.approx(.00784459981530694,abs=1e-12)
    assert e['envelope']['terminal_utilization']==pytest.approx(1.003486583041222,abs=1e-12)
    assert s.recovery_good_candidates==0


@pytest.fixture(scope='module',autouse=True)
def native_nominal_rollout(tmp_path_factory):
    """Use a fresh kernel from this source, not an image's earlier cache."""
    import os
    from aims_mpcc.rollout_native import prepare_kernel
    directory=tmp_path_factory.mktemp('recovery-rollout')
    old=os.environ.get('AIMS_MPCC_ROLLOUT_DIR')
    os.environ['AIMS_MPCC_ROLLOUT_DIR']=str(directory)
    prepare_kernel(directory)
    yield
    if old is None:os.environ.pop('AIMS_MPCC_ROLLOUT_DIR',None)
    else:os.environ['AIMS_MPCC_ROLLOUT_DIR']=old


def recovering(speed=.8,steering=.25,acceleration=0.,rate=0.,internal=.9):
    cfg=VehicleConfig(cruise_speed=1.,max_speed=1.5,enforce_corridor=False)
    s=Supervisor(cfg,100.,handover_delay=.02,solve_period=.05)
    s.status='RECOVERING';s.mode=True;s.last_tick=9.98
    s.last_command=Command(internal,steering);s.last_acceleration=acceleration;s.last_steering_rate=rate
    s.state=State(0.,0.,0.,speed,steering,100.)
    s.state_received=s.mode_received=10.;s.lap_goal=100.
    return s


def rollout_recovery(s,count=50):
    initial=[0.,0.,0.,s.state.speed,0.,s.state.steering];physical=np.array(initial)
    previous=[s.last_acceleration,s.last_command.steering,s.last_steering_rate]
    rows=[];a=[];r=[];diagnostics=[]
    for i in range(count):
        now=10.+i*.02;s.state_received=s.mode_received=now
        s.state=replace(s.state,speed=physical[3],steering=physical[5])
        s.command(now);u=[s.last_acceleration,s.last_command.steering,0.]
        segment=independent_rollout(physical,previous,[u],s.config,.02)
        rows.extend([(x.copy(),u[0]) for x in segment]);physical=segment[-1]
        a.append(u[0]);r.append(s.last_steering_rate)
        diagnostics.append(copy.deepcopy(s.recovery_braking))
        previous=[u[0],u[1],s.last_steering_rate]
    return rows,np.array(a),np.array(r),diagnostics


@pytest.mark.parametrize('steering,rate',[(.25,0.),(-.25,0.),(.02,-.3),(-.02,.3)])
def test_recovery_retains_strict_ellipse_with_positive_a_and_steering_response(steering,rate):
    s=recovering(.55,steering,.3,rate,1.1)
    rows,a,r,d=rollout_recovery(s,50)
    assert max(utilization(x,u,s.config) for x,u in rows)<=1.0001
    assert np.max(np.abs(np.diff(np.r_[.3,a])))<=.020001
    assert np.max(np.abs(np.diff(np.r_[rate,r])))<=.040001
    assert all(x['available'] and x['feasible'] and x['achieved_within_capacity'] for x in d)
    assert s.last_command.speed>=0.  # forward-only target, no physical-v reset


def test_recovery_empty_capacity_interval_keeps_jerk_and_reports_no_certificate():
    s=recovering(1.5,.4,0.,0.,1.)
    s.command(10.)
    assert s.last_acceleration==pytest.approx(-.02,abs=1e-12)
    assert not s.recovery_braking['feasible']
    assert not s.recovery_braking['achieved_within_capacity']
    assert s.recovery_braking['lateral_utilization_bound']>1.
    assert s.status=='RECOVERING' and s.last_command.speed>0.


def test_recovery_empty_jerk_intersection_does_not_snap_positive_acceleration():
    s=recovering(1.2,.20,.5,0.,1.3)
    s.command(10.)
    assert s.last_acceleration==pytest.approx(.48,abs=1e-12)
    assert not s.recovery_braking['feasible']
    assert not s.recovery_braking['achieved_within_capacity']
    assert s.recovery_braking['reason']=='jerk and nominal capacity intervals do not intersect'


@pytest.mark.parametrize('speed',[-.8,.8])
def test_capacity_bound_handles_signed_physical_speed_without_target_reset(speed):
    s=recovering(speed,.25,-.4,0.,1.)
    physical_before=s.state;s.command(10.)
    assert s.state is physical_before and s.state.speed==speed
    d=s.recovery_braking
    assert d['available'] and d['feasible'] and d['achieved_within_capacity']
    segment=independent_rollout([0.,0.,0.,speed,0.,.25],[-.4,.25,0.],[[s.last_acceleration,s.last_command.steering,0.]],s.config,.02)
    assert max(utilization(x,s.last_acceleration,s.config) for x in segment)<=1.0001


@pytest.mark.parametrize('tau',[.0009])
def test_unavailable_response_bound_preserves_bounded_emergency_braking(tau):
    s=recovering(.8,.25,-.4);s.config.steering_tau=tau;s.command(10.)
    assert s.last_acceleration==pytest.approx(-.42,abs=1e-12)
    assert not s.recovery_braking['available']
    assert not s.recovery_braking['achieved_within_capacity']


def test_operator_stop_policy_remains_distinct_from_nominal_recovery_capacity():
    s=recovering(.8,.25,-.48);s.status='STOPPING';s.command(10.)
    assert s.last_acceleration==pytest.approx(-.5,abs=1e-12)
    assert s.recovery_braking is None


def test_straight_recovery_is_bit_exact_old_longitudinal_smoother():
    from aims_mpcc.runtime import clip
    s=recovering(.8,0.,.3,0.,1.1)
    speed=s.last_command.speed;accel=s.last_acceleration;last_tick=s.last_tick
    for i in range(100):
        now=10.+i*.02;dt=clip(now-last_tick,0.,.05);last_tick=now
        desired=clip(-speed/dt,-s.config.brake_limit,s.config.accel_limit)
        def safe(distance):
            return math.sqrt((s.config.jerk_limit*dt)**2+2*s.config.jerk_limit*max(0.,distance))-s.config.jerk_limit*dt
        lo=max(-s.config.brake_limit,accel-s.config.jerk_limit*dt,-safe(speed))
        hi=min(s.config.accel_limit,accel+s.config.jerk_limit*dt,safe(s.config.max_speed-speed))
        a=clip(desired,lo,hi);new=clip(speed+a*dt,0.,s.config.max_speed);accel=(new-speed)/dt;speed=new
        s.state_received=s.mode_received=now;s.command(now)
        assert s.last_command==Command(speed,0.) and s.last_acceleration==accel


@pytest.mark.parametrize('dt',[.005,.02,.05])
@pytest.mark.parametrize('physical,target,rate',[(.1,-.3,.4),(-.1,.3,-.4),(.3,-.1,-.4),(-.3,.1,.4)])
def test_budget_uses_physical_angle_and_new_coasting_target_across_tick_jitter(dt,physical,target,rate):
    s=recovering(.8,physical,-.4,rate,1.)
    s.last_command=Command(1.,target);s.last_tick=10.-dt
    measured=s.state;s.command(10.);budget=s.recovery_braking
    assert s.state is measured and s.state.speed==.8
    assert budget['physical_steering_abs_bound']==max(abs(physical),abs(s.last_command.steering))
    assert budget['feasible'] and budget['achieved_within_capacity']
    assert abs(s.last_acceleration+.4)<=s.config.jerk_limit*dt+1e-12
    segment=independent_rollout([0.,0.,0.,.8,0.,physical],[-.4,target,rate],
                              [[s.last_acceleration,s.last_command.steering,0.]],s.config,.02)
    assert max(utilization(x,s.last_acceleration,s.config) for x in segment)<=1.0001


@pytest.mark.parametrize('understeer',[0.,.8,3.])
def test_capacity_bound_uses_configured_signed_halfaxes_and_understeer(understeer):
    s=recovering(.75,.3,-.25,0.,1.)
    s.config.understeer_coefficient=understeer
    s.config.longitudinal_envelope_accel=.35;s.config.longitudinal_envelope_brake=.3
    s.config.lateral_accel_limit=.8;s.command(10.)
    d=s.recovery_braking
    assert d['feasible'] and d['achieved_within_capacity']
    assert d['accel_capacity']/d['brake_capacity']==pytest.approx(.35/.3)
    segment=independent_rollout([0.,0.,0.,.75,0.,.3],[-.25,.3,0.],
                              [[s.last_acceleration,s.last_command.steering,0.]],s.config,.02)
    assert max(utilization(x,s.last_acceleration,s.config) for x in segment)<=1.0001


@pytest.mark.parametrize('positive_axis,feasible',[(.25,False),(.35,True)])
def test_positive_live_acceleration_requires_positive_halfaxis_budget(positive_axis,feasible):
    s=recovering(.5,.2,.3,0.,1.)
    s.config.longitudinal_envelope_accel=positive_axis
    s.command(10.);d=s.recovery_braking
    assert s.last_acceleration==pytest.approx(.28,abs=1e-12)
    assert d['brake_capacity']>.45
    assert d['feasible'] is feasible and d['achieved_within_capacity'] is feasible
    if not feasible:assert d['accel_capacity']<s.last_acceleration


@pytest.mark.parametrize('outside',[False,True])
@pytest.mark.parametrize('scalar',[np.float32,np.float64])
def test_numpy_live_prefix_recovery_budget_is_native_json(outside,scalar):
    import json
    s=recovering(1.5 if outside else .55,.35 if outside else .25,.3,0. if outside else .1,1.1)
    s.state=replace(s.state,speed=scalar(s.state.speed),steering=scalar(s.state.steering))
    s.last_command=Command(scalar(s.last_command.speed),scalar(s.last_command.steering))
    s.last_acceleration=scalar(s.last_acceleration);s.last_steering_rate=scalar(s.last_steering_rate)
    s.command(10.)
    budget=s.recovery_braking
    json.dumps(budget,allow_nan=False)
    assert type(budget['feasible']) is bool
    for value in budget.values():
        assert value is None or type(value) in (str,bool,float)


def test_numpy_unavailable_response_budget_remains_native_json():
    import json
    s=recovering();s.config.steering_tau=.0009
    s.state=replace(s.state,speed=np.float64(s.state.speed),steering=np.float64(s.state.steering))
    s.command(10.)
    budget=s.recovery_braking
    assert budget['available'] is False and budget['feasible'] is False
    json.dumps(budget,allow_nan=False)
    for value in budget.values():
        assert value is None or type(value) in (str,bool,float)
