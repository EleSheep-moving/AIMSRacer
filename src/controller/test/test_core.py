import csv
import dataclasses
import numpy as np
import pytest
import casadi as ca
from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath, prepare_recording
from aims_mpcc.solver import MPCCSolver


def config():
    return VehicleConfig(rear_offset=.18, half_length=.5, half_width=.3, geometry_verified=True)


def circle(direction=1):
    a = direction * np.linspace(0, 2*np.pi, 100, endpoint=False)
    return ReferencePath(np.c_[3*np.cos(a), 3*np.sin(a)], .8, .9)


def recording(path, direction=1):
    a = direction*np.linspace(0,2*np.pi,201)
    yaw = a+direction*np.pi/2
    rear = np.c_[3*np.cos(a),3*np.sin(a)]
    xy = rear + .18*np.c_[np.cos(yaw),np.sin(yaw)]
    with open(path,'w') as f:
        w=csv.writer(f); w.writerow(['timestamp','x','y','yaw','speed','frame_id','child_frame_id'])
        w.writerows((i*.1,*p,h,.5,'odom','base_link') for i,(p,h) in enumerate(zip(xy,yaw)))


def test_config_gate():
    with pytest.raises(ValueError): VehicleConfig().validate(require_verified=True)
    config().validate(require_verified=True)
    for field,value in [('max_speed',np.nan),('half_width',-1),('steer_limit',2),('cruise_speed',2)]:
        with pytest.raises(ValueError): dataclasses.replace(config(), **{field:value}).validate()


@pytest.mark.parametrize('direction', [1,-1])
def test_path_roundtrip_projection_and_spline(tmp_path,direction):
    path=circle(direction); path.save(tmp_path); loaded=ReferencePath.load(tmp_path)
    assert loaded.frame_id=='odom'
    assert np.allclose(loaded.points,path.points)
    s,error=path.project([3,0]); assert error<1e-7
    q=ca.MX.sym('q'); expr=path.symbolic(q); fun=ca.Function('reference',[q],[expr['xy'],expr['curvature']])
    for s in [-.1,0,1,path.length+.2]:
        xy,k=fun(s); r=path.at(s)
        assert np.allclose(np.array(xy).ravel(),[r['x'],r['y']],atol=1e-10)
        assert float(k)==pytest.approx(r['curvature'],abs=1e-10)
        assert r['curvature']*direction>0


@pytest.mark.parametrize('direction',[1,-1])
def test_prepare_transforms_rear_and_preserves_raw(tmp_path,direction):
    src=tmp_path/'raw.csv'; recording(src,direction)
    path=prepare_recording(src,tmp_path/'prepared',config(),.8,.9)
    assert np.allclose(path.points[0],[3,0],atol=1e-8)
    assert (tmp_path/'prepared'/'raw.csv').read_bytes()==src.read_bytes()
    assert path.metadata['closed_lap'] is True


@pytest.mark.parametrize('fault',['open','nonfinite','frames','time','reverse','narrow'])
def test_prepare_rejects_bad_recordings(tmp_path,fault):
    src=tmp_path/'raw.csv'; recording(src)
    rows=list(csv.DictReader(src.open()))
    if fault=='open': rows=rows[:100]
    if fault=='nonfinite': rows[30]['x']='nan'
    if fault=='frames': rows[30]['frame_id']='map'
    if fault=='time': rows[30]['timestamp']=rows[29]['timestamp']
    if fault=='reverse': rows[30]['speed']='-.1'
    with src.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    with pytest.raises(ValueError): prepare_recording(src,tmp_path/'out',config(),.2 if fault=='narrow' else .8,.9)


def test_solver_real_feasible_result():
    path=circle(); cfg=config(); solver=MPCCSolver(path,cfg,horizon=5)
    delta=np.arctan(cfg.wheelbase/3)
    result=solver.solve(dict(x=3,y=0,yaw=np.pi/2,speed=.5,steering=delta),dict(acceleration=0,steering=delta,steering_rate=0))
    assert result['success'],result
    assert result['constraint_violation']<1e-4
    assert result['minimum_predicted_margin_m']>0
    states=np.array(result['states']); controls=np.array(result['controls'])
    assert states.shape==(6,6) and controls.shape==(5,3)
    assert np.max(abs(controls[:,1]))<=cfg.steer_limit+1e-5
    solver.reset(); assert solver.previous is None


def test_path_rejects_self_intersections_and_duplicate_knots():
    with pytest.raises(ValueError): ReferencePath([[0,0],[2,2],[0,2],[2,0]],1,1)
    with pytest.raises(ValueError): ReferencePath([[0,0],[1,0],[1,0],[0,1]],1,1)


def test_recording_rejects_curvature(tmp_path):
    src=tmp_path/'raw.csv'; recording(src)
    rows=list(csv.DictReader(src.open()))
    for r in rows:
        h=float(r['yaw']);r['x']=str((float(r['x'])-.18*np.cos(h))*.1+.18*np.cos(h));r['y']=str((float(r['y'])-.18*np.sin(h))*.1+.18*np.sin(h))
    with src.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    with pytest.raises(ValueError,match='curvature'): prepare_recording(src,tmp_path/'out',config(),.8,.9)


def test_solver_limits_and_rollout_independently():
    path=circle(-1);cfg=config();solver=MPCCSolver(path,cfg,horizon=6)
    delta=-np.arctan(cfg.wheelbase/3)
    result=solver.solve(dict(x=3,y=0,yaw=-np.pi/2,speed=.5,steering=delta),dict(acceleration=0,steering=delta,steering_rate=0),speed_refs=np.ones(7)*.5)
    assert result['success'],result
    x=np.asarray(result['states']);u=np.asarray(result['controls'])
    assert np.max(np.abs(np.diff(np.r_[0,u[:,0]]))/.1)<=cfg.jerk_limit+1e-4
    rates=np.diff(np.r_[delta,u[:,1]])/.1
    assert np.max(abs(rates))<=cfg.steer_rate+1e-4
    assert np.max(abs(np.diff(np.r_[0,rates]))/.1)<=cfg.steer_acceleration+1e-4
    assert np.all((x[:,3]>=-1e-5)&(x[:,3]<=cfg.max_speed+1e-5))
    assert np.max(abs(np.diff(x[:,3])-.1*u[:,0]))<1e-6
    assert x[-1,1]<0
    with pytest.raises(ValueError): solver.solve(dict(x=3,y=0,yaw=0,speed=.5,steering=0),dict(acceleration=0,steering=0,steering_rate=0),speed_refs=[np.nan]*7)


def test_projection_signed_left_positive():
    path=circle()
    assert path.project([2.9,0])[1]==pytest.approx(.1,abs=1e-6)
    assert path.project([3.1,0])[1]==pytest.approx(-.1,abs=1e-6)


def test_saved_path_schema_and_geometry_binding(tmp_path):
    import json
    src=tmp_path/'raw.csv';recording(src)
    path=prepare_recording(src,tmp_path/'out',config(),.8,.9)
    assert path.metadata['vehicle_geometry']['half_width']==.3
    path.validate_config(config())
    with pytest.raises(ValueError):path.validate_config(dataclasses.replace(config(),rear_offset=.2))
    meta_file=tmp_path/'out'/'metadata.json';meta=json.loads(meta_file.read_text());meta['schema_version']=999;meta_file.write_text(json.dumps(meta))
    with pytest.raises(ValueError):ReferencePath.load(tmp_path/'out')


def test_config_profile_provenance_gate():
    assert config().profile == 'measured'
    synthetic = dataclasses.replace(config(), profile='synthetic')
    synthetic.validate(require_verified=True)
    synthetic.validate(require_verified=True, allow_synthetic=True)
    with pytest.raises(ValueError, match='synthetic'):
        synthetic.validate(require_verified=True, allow_synthetic=False)
    config().validate(require_verified=True, allow_synthetic=False)
    with pytest.raises(ValueError, match='profile'):
        dataclasses.replace(config(), profile='unknown').validate()


@pytest.mark.parametrize('value', ['false', 'true', 1, 0, None])
def test_geometry_verified_requires_boolean(value):
    with pytest.raises(ValueError, match='geometry_verified'):
        dataclasses.replace(config(), geometry_verified=value).validate()


@pytest.mark.parametrize('field', ['wheelbase', 'rear_offset', 'half_width', 'cruise_speed'])
def test_numeric_config_rejects_boolean(field):
    with pytest.raises(ValueError, match=field):
        dataclasses.replace(config(), **{field: True}).validate()


def test_prepare_preserves_existing_bundle(tmp_path):
    src=tmp_path/'raw.csv'; recording(src)
    output=tmp_path/'out'
    prepare_recording(src,output,config(),.8,.9)
    before={p.name:p.read_bytes() for p in output.iterdir()}
    with pytest.raises(ValueError, match='exist'):
        prepare_recording(src,output,config(),.9,.9)
    assert before=={p.name:p.read_bytes() for p in output.iterdir()}


def test_prepare_refuses_raw_recording_directory(tmp_path):
    src=tmp_path/'raw.csv';recording(src)
    before=src.read_bytes()
    with pytest.raises(ValueError, match='exist'):
        prepare_recording(src,tmp_path,config(),.8,.9)
    assert src.read_bytes()==before
    assert not (tmp_path/'path.csv').exists()


def test_cold_warm_start_respects_command_rates_and_physical_progress():
    solver=MPCCSolver(circle(),config())
    initial=np.array([3.,0.,np.pi/2,0.,0.,0.])
    applied=np.zeros(3)
    states,controls=solver._warm_start(initial,applied,np.full(16,.5),.1)
    states=np.asarray(states);controls=np.asarray(controls)
    assert np.max(abs(np.diff(np.r_[0.,controls[:,0]])))<=.1+1e-9
    rates=np.diff(np.r_[0.,controls[:,1]])/.1
    assert np.max(abs(rates))<=.5+1e-9
    assert np.max(abs(np.diff(np.r_[0.,rates])))<=.2+1e-9
    assert controls[0,2]<.01
    assert controls[0,0]==pytest.approx(.1)
    assert np.max(abs(controls[:,2]-states[:-1,3]))<.04
    assert states[-1,3]>.4


def test_drive_requires_recording_provenance(tmp_path):
    fixture=circle()
    with pytest.raises(ValueError):fixture.validate_config(config(),require_recording=True)
    src=tmp_path/'raw.csv';recording(src)
    prepared=prepare_recording(src,tmp_path/'out',config(),.8,.9)
    prepared.validate_config(config(),require_recording=True)
    for bad in [False,'true',1,None]:
        prepared.metadata['closed_lap']=bad
        with pytest.raises(ValueError):prepared.validate_config(config(),require_recording=True)
    prepared.metadata['closed_lap']=True
    for bad in [None,{},dict(wheelbase=.36,rear_offset=.18,half_length=.5,half_width=float('nan'))]:
        prepared.metadata['vehicle_geometry']=bad
        with pytest.raises(ValueError):prepared.validate_config(config(),require_recording=True)


def test_path_schema_boolean_is_not_version_one(tmp_path):
    import json
    circle().save(tmp_path)
    filename=tmp_path/'metadata.json';meta=json.loads(filename.read_text());meta['schema_version']=True;filename.write_text(json.dumps(meta))
    with pytest.raises(ValueError):ReferencePath.load(tmp_path)


@pytest.mark.parametrize('speed', [-.001, -.05])
def test_solver_tolerates_stationary_negative_speed_noise(speed):
    solver=MPCCSolver(circle(),config(),horizon=5)
    result=solver.solve(dict(x=3.,y=0.,yaw=np.pi/2,speed=speed,steering=0.),
                        dict(acceleration=0.,steering=0.,steering_rate=0.))
    assert result['success'],result
    assert result['states'][0][3] == pytest.approx(0.,abs=1e-6)


def test_solver_still_rejects_reverse_motion():
    solver=MPCCSolver(circle(),config(),horizon=5)
    with pytest.raises(ValueError,match='reverse motion'):
        solver.solve(dict(x=3.,y=0.,yaw=np.pi/2,speed=-.051,steering=0.),
                     dict(acceleration=0.,steering=0.,steering_rate=0.))


@pytest.mark.parametrize('speed', [-.001, -.05])
def test_prepare_accepts_small_negative_speed_noise_and_keeps_raw(tmp_path, speed):
    src=tmp_path/'raw.csv';recording(src)
    rows=list(csv.DictReader(src.open()));rows[0]['speed']=str(speed)
    with src.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    prepare_recording(src,tmp_path/'out',config(),.8,.9)
    assert (tmp_path/'out/raw.csv').read_bytes()==src.read_bytes()


def test_prepare_checks_reverse_only_inside_selected_lap(tmp_path):
    src=tmp_path/'raw.csv';recording(src)
    rows=list(csv.DictReader(src.open()))
    before=dict(rows[0],timestamp='-1',speed='-.2')
    with src.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows([before]+rows)
    prepare_recording(src,tmp_path/'out',config(),.8,.9,start_time=0.)
    with pytest.raises(ValueError,match='reverse motion'):
        prepare_recording(src,tmp_path/'rejected',config(),.8,.9)
