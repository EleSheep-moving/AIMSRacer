"""Local file preparation for the offline reference editor."""
import hashlib
import json
import re
import shutil
from pathlib import Path
import numpy as np
from pypcd4 import PointCloud
from aims_mpcc.gui_reference import prepare_gui_reference
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath
from aims_mpcc.reference_export import serialize_reference


def json_bytes(value):
    return (json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()


def load_map(filename):
    filename=Path(filename)
    original=filename.read_bytes()
    cloud=PointCloud.from_path(filename)
    if not {'x','y','z'}.issubset(cloud.fields): raise ValueError('PCD must have x, y and z fields')
    points=cloud.numpy(('x','y','z'))
    original_count=len(points)
    points=points[np.isfinite(points).all(axis=1)]
    if not len(points): raise ValueError('PCD has no finite XYZ points')
    cells=np.floor(points/.05).astype(np.int64)
    _,indices=np.unique(cells,axis=0,return_index=True)
    points=points[np.sort(indices)]
    voxel_count=len(points)
    if len(points)>200000: points=points[np.linspace(0,len(points)-1,200000,dtype=int)]
    return dict(sha256=hashlib.sha256(original).hexdigest(),original_count=original_count,
                display_count=len(points),voxel_count=voxel_count,voxel_size_m=.05,
                points=points.tolist(),bounds=[points.min(axis=0).tolist(),points.max(axis=0).tolist()])


class EditorService:
    def __init__(self,config_file,output_directory):
        self.config_file=Path(config_file).resolve()
        self.config_sha256=hashlib.sha256(self.config_file.read_bytes()).hexdigest()
        self.config=load_config(self.config_file)
        self.config.validate(require_verified=True)
        self.output_directory=Path(output_directory).resolve()
        self.output_directory.mkdir(parents=True,exist_ok=True)
        self.map_directory=self.output_directory/'.maps'
        self.map_directory.mkdir(exist_ok=True)
        self.active_map=None

    def import_map(self,filename):
        result=load_map(filename)
        destination=self.map_directory/(result['sha256']+'.pcd')
        if not destination.exists():
            with destination.open('xb') as stream: stream.write(Path(filename).read_bytes())
            destination.chmod(0o444)
        if hashlib.sha256(destination.read_bytes()).hexdigest()!=result['sha256']:
            raise ValueError('immutable original map hash mismatch')
        self.active_map=result
        return result

    def _prepare(self,project):
        path,checks=prepare_gui_reference(project,self.config)
        evidence=path.metadata.get('speed_replan')
        if evidence and evidence['selected_config_sha256']!=self.config_sha256:
            raise ValueError('project uses another speed replan configuration; load the reference ZIP and explicitly Replan speed')
        return path,checks

    def preview(self,project):
        path,checks=self._prepare(project)
        return self._preview(path,checks)

    def _preview(self,path,checks):
        reference=serialize_reference(path,self.config)
        profile=reference['speed_profile']
        speeds=np.interp(checks['positions'],profile['positions']+[path.length],profile['speeds']+[profile['speeds'][0]])
        steering=np.arctan(self.config.wheelbase*(1+self.config.understeer_coefficient*speeds**2)*checks['curvature'])
        if np.max(np.abs(steering))>self.config.steer_limit+1e-9:
            raise ValueError('planned-speed steering demand exceeds steering capability')
        return dict(reference=reference,samples=[dict(s=float(s),x=float(p[0]),y=float(p[1]),curvature=float(k),
                       steering_rad=float(d),speed=float(v)) for s,p,k,d,v in zip(checks['positions'],checks['xy'],checks['curvature'],steering,speeds)],
                    length=path.length,max_curvature=float(np.max(np.abs(checks['curvature']))),
                    max_steering_rad=float(np.max(np.abs(steering))),steer_limit_rad=self.config.steer_limit,
                    vehicle_geometry=reference['metadata']['vehicle_geometry'],regions=checks['regions'],
                    validation='sampled at <=0.02 m of progress; no obstacle or footprint-clearance guarantee')

    @staticmethod
    def check_name(name):
        if not isinstance(name,str) or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}',name) is None:
            raise ValueError('bundle name must use 1-80 letters, digits, underscores or dashes')
        return name

    def export(self,project,name):
        name=self.check_name(name)
        if not isinstance(project,dict): raise ValueError('editable project mapping required')
        map_hash=project.get('map_sha256','')
        if not isinstance(map_hash,str) or re.fullmatch(r'[0-9a-f]{64}',map_hash) is None:
            raise ValueError('original map SHA-256 required')
        original=self.map_directory/(map_hash+'.pcd')
        if not original.is_file() or hashlib.sha256(original.read_bytes()).hexdigest()!=map_hash:
            raise ValueError('load the original map PCD before exporting this project')
        path,checks=self._prepare(project)
        preview=self._preview(path,checks)
        target=self.output_directory/name
        try: target.mkdir(exist_ok=False)
        except FileExistsError as exc: raise ValueError('output directory already exists; choose a new bundle name') from exc
        try:
            path.save(target)
            (target/'reference.json').write_bytes(json_bytes(preview['reference']))
            reference_hash=hashlib.sha256((target/'reference.json').read_bytes()).hexdigest()
            saved=dict(project,schema_version=1,frame_id='map',coordinate_reference='rear_axle',calibration_regions=checks['regions'])
            (target/'project.json').write_bytes(json_bytes(saved))
            (target/'calibration_regions.json').write_bytes(json_bytes(dict(schema_version=1,reference_sha256=reference_hash,regions=checks['regions'])))
        except Exception:
            shutil.rmtree(target)
            raise
        return dict(directory=name,reference_sha256=reference_hash,map_sha256=map_hash)

    def load_reference(self,directory,replan_speed=False):
        directory=Path(directory)
        path=ReferencePath.load(directory)
        if path.frame_id!='map': raise ValueError('GUI edits map-frame references only')
        path.validate_config(self.config,require_recording=True)
        regions=[]
        if (directory/'calibration_regions.json').is_file():
            payload=json.loads((directory/'calibration_regions.json').read_text())
            expected=hashlib.sha256((directory/'reference.json').read_bytes()).hexdigest()
            if payload.get('reference_sha256')!=expected: raise ValueError('calibration regions reference hash mismatch')
            regions=payload['regions']
        if (directory/'project.json').is_file():
            project=json.loads((directory/'project.json').read_text())
        else:
            project=dict(schema_version=1,waypoints=path.points.tolist(),left_width=path.left_width,right_width=path.right_width,
                         map_sha256=path.metadata.get('map_sha256'),frame_id='map',coordinate_reference='rear_axle',calibration_regions=regions)
        supplied=None
        if (directory/'reference.json').is_file():
            supplied=json.loads((directory/'reference.json').read_text())
            if replan_speed:
                project['speed_replan']=dict(prior_reference_sha256=hashlib.sha256((directory/'reference.json').read_bytes()).hexdigest(),
                    prior_speed_profile_sha256=hashlib.sha256(json_bytes(supplied.get('speed_profile'))).hexdigest(),
                    selected_config_sha256=self.config_sha256)
        preview=self.preview(project)
        frozen=serialize_reference(path,self.config)
        for key in ('knots','coefficients','length','left_width','right_width','frame_id'):
            if preview['reference'][key]!=frozen[key]:
                raise ValueError('editable project disagrees with frozen reference geometry')
        if project.get('map_sha256')!=path.metadata.get('map_sha256'):
            raise ValueError('editable project disagrees with frozen map identity')
        if supplied is not None:
            if supplied.get('speed_profile')!=frozen['speed_profile'] and not replan_speed:
                raise ValueError('Frozen speed profile differs from the selected vehicle config. Enable Replan speed to explicitly recompute it.')
            for key in ('knots','coefficients','length','left_width','right_width','frame_id'):
                if supplied.get(key)!=frozen[key]:
                    raise ValueError('path.csv disagrees with frozen reference geometry')
        return project
