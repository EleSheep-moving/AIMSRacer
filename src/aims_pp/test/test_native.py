"""Python equations and field gates are the independent native-port oracle."""
import dataclasses, json, math, os, subprocess
import pytest
from conftest import health_values
from aims_pp.core import PPConfig,EnhancedPP
from aims_pp.reference import NumericReference

@pytest.fixture
def native(tmp_path,circle_payload,request):
    exe=os.environ.get('AIMS_PP_NATIVE_RUNNER')
    if not exe: pytest.skip('set AIMS_PP_NATIVE_RUNNER to built native_runner')
    (tmp_path/'reference.json').write_text(json.dumps(circle_payload))
    p=subprocess.Popen([exe,str(tmp_path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    options=getattr(request,'param',{})
    p.stdin.write(json.dumps(dict(config=dataclasses.asdict(PPConfig(max_speed=5.)),**options))+'\n');p.stdin.flush()
    def call(op,**kw):
        p.stdin.write(json.dumps(dict(op=op,**kw))+'\n');p.stdin.flush()
        return json.loads(p.stdout.readline())
    yield call
    p.stdin.close(); p.wait(timeout=3)

def test_numeric_reference_and_enhanced_pp_parity(native,circle_payload):
    ref=NumericReference(circle_payload);pp=EnhancedPP(PPConfig(max_speed=5.))
    for s in [-.2,0,.1,1.5,3.2,7.,ref.length,ref.length+.2]:
        got=native('at',s=s)['result']; assert {k:got[k] for k in ['x','y','yaw','curvature']}==pytest.approx(ref.at(s),abs=1e-11)
        assert got['speed']==pytest.approx(ref.speed_at(s),abs=1e-12)
        at=ref.at(s);xy=[at['x']+.04,at['y']-.07]
        projected=native('project',pose=xy)['result'];expected=ref.project(xy)
        assert ((projected['progress']-expected[0]+ref.length/2)%ref.length-ref.length/2)==pytest.approx(0,abs=1e-9)
        assert projected['lateral_error']==pytest.approx(expected[1],abs=1e-10)
        for speed in [0.,1.,4.,5.,6.]:
            args=[*xy,at['yaw']+.2,at['x'],at['y'],at['yaw'],at['curvature'],speed]
            steering,debug=pp.compute_steering(xy,args[2],args[3:5],args[5],args[6],speed)
            got=native('steering',values=args)['result'];assert got['steering']==pytest.approx(steering,abs=1e-12)
            assert got['debug']==pytest.approx(debug,abs=1e-12)

def refresh(native,now=10000000000,steady=10.,sequence=1,epoch='one'):
    native('map',value='a'*64);native('valid',value=True)
    native('health',values=health_values(sequence,now,epoch),now=now,steady=steady)
    native('odom',values=dict(x=2.,y=0.,yaw=math.pi/2,velocity=1.,stamp_ns=now),now=now,steady=steady)
    native('authority',value=True,steady=steady)
    native('command',values=dict(control_mode=0,steering=0.,speed=1.,stamp_ns=now),now=now,steady=steady)

def test_native_field_gates_clock_reenable_and_duplicate_source(native):
    refresh(native)
    assert native('tick')['result']['speed']==0
    assert native('enable')['result']['success']
    assert native('tick')['result']['speed']==1
    native('valid',value=False);native('valid',value=True)
    assert native('tick')['status']['phase']=='FAULT'
    assert native('enable')['result']['success']
    native('clock',now=9000000000)
    status=native('snapshot',now=9000000000,steady=10.01)
    assert status['result']['map_sha256']=='a'*64
    assert status['result']['reason']=='Fresh source odometry required'
    assert native('tick',now=9000000000,steady=10.01)['status']['phase']=='FAULT'
    refresh(native,9000000000,10.02,1,'after-reset')
    assert native('tick',now=9000000000,steady=10.02)['result']['speed']==0
    assert native('enable',now=9000000000,steady=10.02)['result']['success']
    native('odom',values=dict(x=2.,y=0.,yaw=math.pi/2,velocity=1.,stamp_ns=9000000000),now=9090000000,steady=10.11)
    assert native('snapshot',now=9000000000,steady=10.13)['result']['reason']=='Fresh source odometry required'

@pytest.mark.parametrize('distance,accepted',[(.1974932886,True),(.314,True),(.316,False)])
def test_native_freshest_field_jump_gate(native,distance,accepted):
    assert native('odom',values=dict(x=0.,y=0.,yaw=0.,velocity=1.,stamp_ns=10000000000))['ok']
    assert native('odom',values=dict(x=distance,y=0.,yaw=0.,velocity=1.,stamp_ns=10003000000),now=10003000000,steady=10.003)['ok']==accepted

def test_native_anchor_mutation_loss_and_epoch_retirement(native):
    refresh(native)
    vals=health_values(health_sequence=2);vals['ready']='false';native('health',values=vals)
    native('health',values=health_values(health_sequence=3))
    assert native('snapshot')['result']['reason']=='Fresh trusted qualified localization required'
    native('health',values=health_values(sequence=2,health_sequence=4))
    assert native('snapshot')['result']['reason']==''
    vals=health_values(sequence=2,health_sequence=5);vals['map_odom_x']='1';native('health',values=vals)
    assert native('snapshot')['result']['reason']
    native('health',values=health_values(epoch='two'))
    native('health',values=health_values(epoch='one',health_sequence=20))
    assert native('snapshot')['result']['anchor_epoch']=='two'

@pytest.mark.parametrize('damage',['hash','geometry','frame','seam','knots','speed','closed','nan'])
def test_native_rejects_invalid_reference(tmp_path,circle_payload,damage):
    exe=os.environ.get('AIMS_PP_NATIVE_RUNNER')
    if not exe:pytest.skip('native runner unavailable')
    import copy
    d=copy.deepcopy(circle_payload)
    if damage=='hash':d['metadata']['map_sha256']='bad'
    if damage=='geometry':d['metadata']['vehicle_geometry']['wheelbase']=.4
    if damage=='frame':d['frame_id']='odom'
    if damage=='seam':d['coefficients'][-1][0][0]+=.1
    if damage=='knots':d['knots'][1]=d['knots'][0]
    if damage=='speed':d['speed_profile']['positions']=[0,3,3,9]
    if damage=='closed':d['metadata']['closed_lap']=False
    if damage=='nan':d['coefficients'][0][0][0]=float('nan')
    (tmp_path/'reference.json').write_text(json.dumps(d))
    p=subprocess.run([exe,str(tmp_path)],input=json.dumps({'config':dataclasses.asdict(PPConfig())}),text=True,capture_output=True)
    assert p.returncode!=0

def test_native_one_lap_completion_dense_forward(native,circle_payload):
    ref=NumericReference(circle_payload);refresh(native)
    assert native('enable')['result']['success']
    for i in range(1,201):
        n=10000000000+i*70000000;t=10.+i*.07;s=i*ref.length/200;at=ref.at(s)
        native('health',values=health_values(i+1,n),now=n,steady=t)
        native('odom',values=dict(x=at['x'],y=at['y'],yaw=at['yaw'],velocity=1.,stamp_ns=n),now=n,steady=t)
        native('authority',value=True,now=n,steady=t)
        native('command',values=dict(control_mode=0,steering=0.,speed=1.,stamp_ns=n),now=n,steady=t)
        out=native('tick',now=n,steady=t)
        if i<200:assert out['status']['phase']=='RUNNING'
    assert out['status']['phase']=='COMPLETE' and out['result']['speed']==0

def test_native_heading_quaternion_timer_and_future_stamp_faults(native):
    refresh(native)
    assert not native('odom',values=dict(x=2.,y=0.,quaternion=[0,0,0,0],velocity=1.,stamp_ns=10001000000),now=10001000000,steady=10.001)['ok']
    refresh(native,10002000000,10.002,2)
    assert native('enable',now=10002000000,steady=10.002)['result']['success']
    assert native('tick',now=10002000000,steady=10.103)['status']['phase']=='FAULT'
    assert not native('odom',values=dict(x=2.,y=0.,yaw=math.pi/2,velocity=1.,stamp_ns=11000000000))['ok']

def test_native_malformed_map_health_revokes_previous_alignment(native):
    refresh(native)
    v=health_values(health_sequence=2);v['map_sha256']=123
    native('health',values=v)
    assert native('snapshot')['result']['reason']=='Fresh trusted qualified localization required'

@pytest.mark.parametrize('native',[{'allow_reverse_velocity':True,'allow_speed_overshoot':True}],indirect=True)
def test_native_calibration_reverse_and_overshoot_keep_discontinuity_gates(native):
    assert native('odom',values=dict(x=0.,y=0.,yaw=0.,velocity=-.075,stamp_ns=10000000000))['ok']
    assert native('odom',values=dict(x=1.6,y=0.,yaw=0.,velocity=8.,stamp_ns=10200000000),now=10200000000,steady=10.2)['ok']
    assert not native('odom',values=dict(x=4.,y=0.,yaw=0.,velocity=8.,stamp_ns=10210000000),now=10210000000,steady=10.21)['ok']


def test_native_vehicle_yaml_ignores_tuning_and_extra_keys(native,tmp_path):
    import yaml
    from aims_pp.core import PPConfig
    vehicle=tmp_path/'vehicle.yaml'
    values={k:v for k,v in dataclasses.asdict(PPConfig()).items() if k in {'wheelbase','steer_limit','rear_offset','half_length','front_extent','rear_extent','half_width','max_speed','geometry_verified','enforce_corridor'}}
    values.update(lookahead_gain='ignored non numeric vehicle value',lookahead_speed_gain='also ignored',vehicle_notes='other vehicle configuration')
    vehicle.write_text(yaml.safe_dump(values))
    expected=PPConfig.from_files(vehicle)
    got=native('load_config',vehicle=str(vehicle))
    assert got['ok']
    assert got['result']==dataclasses.asdict(expected)
