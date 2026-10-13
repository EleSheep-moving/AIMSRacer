import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath

ROOT=Path(__file__).resolve().parents[4]


def circle(radius=4):
    a=np.linspace(0,2*np.pi,12,endpoint=False)
    return (radius*np.c_[np.cos(a),np.sin(a)]).tolist()


def project():
    return dict(schema_version=1,waypoints=circle(),left_width=.5,right_width=.5,
                map_sha256='a'*64,calibration_regions=[dict(id='trial-1',name='straight trial',start_s=0.,end_s=1.,kind='straight')])


def test_preview_export_reopen_exact_geometry(tmp_path):
    from tools.reference.editor.backend import EditorService
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path)
    from pypcd4 import PointCloud
    pcd=tmp_path/'input.pcd';PointCloud.from_xyz_points(np.zeros((1,3),dtype=np.float32)).save(pcd)
    data=project();data['map_sha256']=svc.import_map(pcd)['sha256']
    preview=svc.preview(data)
    result=svc.export(data,'lap')
    directory=tmp_path/result['directory']
    frozen=json.loads((directory/'reference.json').read_text())
    loaded=ReferencePath.load(directory)
    assert frozen['coefficients']==preview['reference']['coefficients']==loaded.curve.coefficients.tolist()
    assert json.loads((directory/'metadata.json').read_text())['source']=='gui'
    assert not {'start_time','end_time'}&json.loads((directory/'metadata.json').read_text()).keys()
    assert svc.load_reference(directory)['waypoints']==project()['waypoints']
    regions=json.loads((directory/'calibration_regions.json').read_text())
    assert regions['reference_sha256']==hashlib.sha256((directory/'reference.json').read_bytes()).hexdigest()
    with pytest.raises(ValueError,match='exists'): svc.export(data,'lap')


@pytest.mark.parametrize('update,match',[
    ({'waypoints':circle()[:5]},'six'),
    ({'waypoints':circle(.3)},'steering'),
    ({'map_sha256':'oops'},'SHA'),
    ({'waypoints':[[0,0],[2,2],[0,2],[2,0],[3,0],[3,3]]},'intersect'),
    ({'calibration_regions':[{'id':'bad','name':'bad','start_s':0,'end_s':9999,'kind':'straight'}]},'progress'),
])
def test_invalid_editor_projects_rejected(tmp_path,update,match):
    from tools.reference.editor.backend import EditorService
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path)
    data=project();data.update(update)
    with pytest.raises(ValueError,match=match): svc.preview(data)


def test_pcd_original_hash_and_display_filter(tmp_path):
    from tools.reference.editor.backend import load_map
    from pypcd4 import PointCloud
    points=np.array([[0,0,0],[.01,0,0],[1,2,3],[np.nan,0,0]],dtype=np.float32)
    pcd=tmp_path/'map.pcd';PointCloud.from_xyz_points(points).save(pcd)
    result=load_map(pcd)
    assert result['sha256']==hashlib.sha256(pcd.read_bytes()).hexdigest()
    assert result['original_count']==4
    assert len(result['points'])==2
    assert result['voxel_size_m']==.05


def test_server_api_roundtrip_and_validation(tmp_path):
    from tools.reference.editor.server import create_app
    from fastapi.testclient import TestClient
    client=TestClient(create_app(ROOT/'src/controller/config/vehicle.yaml',tmp_path))
    response=client.post('/api/preview',json=project())
    assert response.status_code==200,response.text
    assert response.json()['max_steering_rad']<.45
    from pypcd4 import PointCloud
    pcd=tmp_path/'api.pcd';PointCloud.from_xyz_points(np.zeros((1,3),dtype=np.float32)).save(pcd)
    with pcd.open('rb') as f: imported=client.post('/api/map',files={'file':('api.pcd',f,'application/octet-stream')})
    data=project();data['map_sha256']=imported.json()['sha256']
    assert client.post('/api/export',json={'project':data,'name':'api-lap'}).status_code==200
    assert client.post('/api/export',json={'project':project(),'name':'../escape'}).status_code==422
    assert client.get('/').status_code==200
    assert client.get('/static/vendor/three.module.min.js').status_code==200


def test_export_requires_original_loaded_map(tmp_path):
    from tools.reference.editor.backend import EditorService
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path)
    with pytest.raises(ValueError,match='original map'): svc.export(project(),'unknown-map')


def test_export_has_immutable_map_provenance(tmp_path):
    from tools.reference.editor.backend import EditorService
    from pypcd4 import PointCloud
    map_file=tmp_path/'input.pcd'
    PointCloud.from_xyz_points(np.array([[0,0,0],[1,0,0]],dtype=np.float32)).save(map_file)
    original=map_file.read_bytes()
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path/'outputs')
    data=project();data['map_sha256']=svc.import_map(map_file)['sha256']
    svc.export(data,'provenance')
    assert (svc.map_directory/(data['map_sha256']+'.pcd')).read_bytes()==original
    map_file.write_bytes(b'changed external map')
    assert svc.preview(data)['reference']['metadata']['map_sha256']==data['map_sha256']


def test_evaluated_curve_crossings_are_checked_after_waypoint_topology(tmp_path):
    from tools.reference.editor.backend import EditorService
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path)
    data=project()
    # Radial ordering forms a simple control polygon, but the quintic overshoots.
    data['waypoints']=[[2.991647366,0],[.553740103,.553740103],[0,4.197091056],[-1.557214309,1.557214309],[-.197248562,0],[-4.739424981,-4.739424981],[0,-2.842144316],[2.220391720,-2.220391720]]
    with pytest.raises(ValueError,match='evaluated curve'): svc.preview(data)


def test_load_reference_rejects_project_that_disagrees_with_frozen_geometry(tmp_path):
    from tools.reference.editor.backend import EditorService
    from pypcd4 import PointCloud
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path/'outputs')
    pcd=tmp_path/'input.pcd';PointCloud.from_xyz_points(np.zeros((1,3),dtype=np.float32)).save(pcd)
    data=project();data['map_sha256']=svc.import_map(pcd)['sha256']
    svc.export(data,'lap')
    directory=svc.output_directory/'lap'
    altered=dict(data,waypoints=circle(5))
    (directory/'project.json').write_text(json.dumps(altered))
    with pytest.raises(ValueError,match='frozen'): svc.load_reference(directory)


def test_speed_dependent_steering_demand_respects_vehicle_understeer(tmp_path):
    from tools.reference.editor.backend import EditorService
    import yaml
    config=yaml.safe_load((ROOT/'src/controller/config/vehicle.yaml').read_text())
    config['understeer_coefficient']=1.
    config_file=tmp_path/'understeer.yaml';config_file.write_text(yaml.safe_dump(config))
    svc=EditorService(config_file,tmp_path/'output')
    data=project();data['waypoints']=circle(1.5)
    with pytest.raises(ValueError,match='steering'): svc.preview(data)


def test_preview_speed_uses_the_frozen_periodic_interpolation(tmp_path):
    from tools.reference.editor.backend import EditorService
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path)
    data=project();data['waypoints']=(np.array(circle())*[1,1.3]).tolist()
    preview=svc.preview(data);profile=preview['reference']['speed_profile']
    for sample in preview['samples'][-10:]:
        expected=np.interp(sample['s'],profile['positions']+[preview['length']],profile['speeds']+[profile['speeds'][0]])
        assert sample['speed']==pytest.approx(expected,abs=1e-12)


def test_api_rejects_missing_project_and_corrupt_reference(tmp_path):
    from tools.reference.editor.server import create_app
    from fastapi.testclient import TestClient
    client=TestClient(create_app(ROOT/'src/controller/config/vehicle.yaml',tmp_path))
    assert client.post('/api/export',json={'name':'bad'}).status_code==422
    assert client.post('/api/reference',files={'file':('bad.zip',b'bad zip','application/zip')}).status_code==422


def test_trial_regions_reject_overlap_including_wrap_and_allow_adjacent(tmp_path):
    from tools.reference.editor.backend import EditorService
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path)
    data=project();data['calibration_regions'].append(dict(id='trial-2',name='Second',kind='straight',start_s=.5,end_s=2.))
    with pytest.raises(ValueError,match='overlap'):svc.preview(data)
    data['calibration_regions'][1].update(start_s=24.,end_s=.5)
    with pytest.raises(ValueError,match='overlap'):svc.preview(data)
    data['calibration_regions'][1].update(start_s=1.,end_s=2.)
    assert len(svc.preview(data)['regions'])==2


def test_frozen_speed_profile_mismatch_requires_explicit_replan(tmp_path):
    from tools.reference.editor.backend import EditorService
    from pypcd4 import PointCloud
    import yaml
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path/'output')
    pcd=tmp_path/'map.pcd';PointCloud.from_xyz_points(np.zeros((1,3),dtype=np.float32)).save(pcd)
    data=project();data['map_sha256']=svc.import_map(pcd)['sha256'];svc.export(data,'lap')
    directory=svc.output_directory/'lap'
    config=yaml.safe_load(svc.config_file.read_text());config['cruise_speed']=1.
    config_file=tmp_path/'slower.yaml';config_file.write_text(yaml.safe_dump(config))
    other=EditorService(config_file,svc.output_directory)
    with pytest.raises(ValueError,match='Replan'): other.load_reference(directory)
    replanned=other.load_reference(directory,replan_speed=True)
    assert replanned['speed_replan']['prior_reference_sha256']==hashlib.sha256((directory/'reference.json').read_bytes()).hexdigest()
    assert replanned['speed_replan']['selected_config_sha256']==hashlib.sha256(config_file.read_bytes()).hexdigest()
    result=other.export(replanned,'replanned')
    frozen=json.loads((other.output_directory/result['directory']/'reference.json').read_text())
    assert max(frozen['speed_profile']['speeds'])<=1.
    assert frozen['metadata']['speed_replan']==replanned['speed_replan']


def test_export_rejects_replan_evidence_from_another_config(tmp_path):
    from tools.reference.editor.backend import EditorService
    from pypcd4 import PointCloud
    svc=EditorService(ROOT/'src/controller/config/vehicle.yaml',tmp_path/'output')
    pcd=tmp_path/'map.pcd';PointCloud.from_xyz_points(np.zeros((1,3),dtype=np.float32)).save(pcd)
    data=project();data['map_sha256']=svc.import_map(pcd)['sha256']
    data['speed_replan']=dict(prior_reference_sha256='a'*64,prior_speed_profile_sha256='b'*64,selected_config_sha256='c'*64)
    with pytest.raises(ValueError,match='configuration'):svc.export(data,'invalid-evidence')
