"""Rear-axle periodic references and conservative recorded-lap preparation."""
import csv
import json
import shutil
from pathlib import Path
import numpy as np
import casadi as ca
from scipy.optimize import minimize_scalar
from .vendor.track import PeriodicCubic
from .config import REVERSE_SPEED_TOLERANCE


def _wrap(a): return (a + np.pi) % (2*np.pi) - np.pi


def _check_intersections(points):
    # Reject touching nonadjacent edges as well as proper crossings.
    def cross(a,b): return a[0]*b[1]-a[1]*b[0]
    count=len(points)
    for i in range(count):
        a,b=points[i],points[(i+1)%count]
        for j in range(i+2,count):
            if i==0 and j==count-1: continue
            c,d=points[j],points[(j+1)%count]
            if np.any(np.maximum(a,b)<np.minimum(c,d)-1e-10) or np.any(np.maximum(c,d)<np.minimum(a,b)-1e-10): continue
            if cross(b-a,c-a)*cross(b-a,d-a)<=1e-12 and cross(d-c,a-c)*cross(d-c,b-c)<=1e-12:
                raise ValueError('self-intersecting or touching closed lap')


class ReferencePath:
    def __init__(self, points, left_width, right_width, frame_id='odom', metadata=None):
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)<4 or not np.isfinite(points).all():
            raise ValueError('at least four finite Nx2 points required')
        if np.linalg.norm(points[-1]-points[0])<1e-9: points=points[:-1]
        self.points=points.copy()
        self.left_width=float(left_width);self.right_width=float(right_width)
        if not np.isfinite([self.left_width,self.right_width]).all() or min(self.left_width,self.right_width)<=0:
            raise ValueError('positive finite corridor widths required')
        if frame_id!='odom': raise ValueError('reference frame must be odom')
        self.frame_id=frame_id; self.metadata=dict(metadata or {})
        closed=np.vstack([points,points[0]])
        segments=np.linalg.norm(np.diff(closed,axis=0),axis=1)
        if len(points)<4 or np.any(segments<1e-8): raise ValueError('zero-length path segment')
        _check_intersections(points)
        self.s=np.r_[0,np.cumsum(segments)]; self.length=float(self.s[-1])
        self.curve=PeriodicCubic(self.s,closed,self.length,'aims_reference')
        probes=np.linspace(0,self.length,max(100,len(points)*5),endpoint=False)
        if np.min(np.linalg.norm(self.curve.numpy(probes,1),axis=1))<1e-6: raise ValueError('zero tangent')
        self.reference=self

    def symbolic(self,s):
        xy=self.curve.symbolic(s); d=self.curve.symbolic(s,1);dd=self.curve.symbolic(s,2)
        return dict(xy=xy,dxy=d,curvature=(d[0]*dd[1]-d[1]*dd[0])/ca.power(ca.dot(d,d),1.5))

    def at(self,s):
        xy=self.curve.numpy(s);d=self.curve.numpy(s,1);dd=self.curve.numpy(s,2)
        return dict(x=float(xy[0]),y=float(xy[1]),yaw=float(np.arctan2(d[1],d[0])),
                    curvature=float((d[0]*dd[1]-d[1]*dd[0])/np.linalg.norm(d)**3))

    numpy=at

    def project(self,xy):
        xy=np.asarray(xy,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all(): raise ValueError('finite xy required')
        grid=np.linspace(0,self.length,max(100,int(self.length/.05)),endpoint=False)
        guess=grid[np.argmin(np.sum((self.curve.numpy(grid)-xy)**2,axis=1))]
        step=self.length/len(grid)
        result=minimize_scalar(lambda s:float(np.sum((self.curve.numpy(s)-xy)**2)),bounds=(guess-step,guess+step),method='bounded',options={'xatol':1e-12})
        s=float(result.x%self.length)
        ref=self.at(s);normal=np.array([-np.sin(ref['yaw']),np.cos(ref['yaw'])])
        return s,float(np.dot(xy-np.array([ref['x'],ref['y']]),normal))

    def validate_config(self,config,require_recording=False):
        config.validate(require_verified=True)
        if min(self.left_width,self.right_width)<=config.half_width:
            raise ValueError('corridor narrower than body')
        geometry=self.metadata.get('vehicle_geometry')
        if require_recording and (self.metadata.get('closed_lap') is not True or not isinstance(geometry,dict)):
            raise ValueError('drive requires closed recording with complete vehicle geometry')
        if geometry is not None:
            if not isinstance(geometry,dict):
                raise ValueError('vehicle_geometry must be a mapping')
            for key in ['wheelbase','rear_offset','half_length','half_width']:
                value=geometry.get(key)
                if (isinstance(value,bool) or not isinstance(value,(int,float)) or not np.isfinite(value)
                        or not np.isclose(value,getattr(config,key),rtol=0,atol=1e-9)):
                    raise ValueError(f'prepared path geometry mismatch: {key}')
        return self

    def save(self,directory):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        np.savetxt(directory/'path.csv',self.points,delimiter=',',header='x,y',comments='')
        payload=dict(self.metadata,frame_id=self.frame_id,left_width=self.left_width,right_width=self.right_width,
                     coordinate_reference='rear_axle',schema_version=1)
        (directory/'metadata.json').write_text(json.dumps(payload,indent=2,allow_nan=False)+'\n')

    @classmethod
    def load(cls,directory):
        directory=Path(directory);meta=json.loads((directory/'metadata.json').read_text())
        if type(meta.get('schema_version')) is not int or meta['schema_version']!=1: raise ValueError('unsupported path schema_version')
        if meta.get('coordinate_reference')!='rear_axle': raise ValueError('rear-axle path required')
        return cls(np.loadtxt(directory/'path.csv',delimiter=',',skiprows=1),meta['left_width'],meta['right_width'],meta['frame_id'],meta)


def prepare_recording(csv_path, output_directory, config, left_width, right_width, start_time=None, end_time=None):
    output=Path(output_directory)
    if output.exists():
        raise ValueError('output directory already exists; choose a new recording bundle')
    config.validate(require_verified=True)
    if min(left_width,right_width)<=config.half_width: raise ValueError('corridor narrower than vehicle')
    src=Path(csv_path)
    with src.open() as f: rows=list(csv.DictReader(f))
    required={'timestamp','x','y','yaw','speed','frame_id','child_frame_id'}
    if not rows or not required.issubset(rows[0]): raise ValueError('recording columns missing')
    if any(r['frame_id']!='odom' or r['child_frame_id']!='base_link' for r in rows): raise ValueError('expected consistent odom/base_link frames')
    values=np.array([[float(r[k]) for k in ['timestamp','x','y','yaw','speed']] for r in rows])
    if not np.isfinite(values).all() or np.any(np.diff(values[:,0])<=0): raise ValueError('finite increasing timestamps and states required')
    selected=np.ones(len(values),bool)
    if start_time is not None: selected &= values[:,0]>=start_time
    if end_time is not None: selected &= values[:,0]<=end_time
    values=values[selected]
    if np.any(values[:,4] < -REVERSE_SPEED_TOLERANCE): raise ValueError('reverse motion unsupported')
    if len(values)<10: raise ValueError('too few recorded samples')
    yaw=values[:,3]; points=values[:,1:3]-config.rear_offset*np.c_[np.cos(yaw),np.sin(yaw)]
    intervals=np.diff(values[:,0])
    speed_bound=np.maximum(config.max_speed,np.maximum(values[:-1,4],values[1:,4]))
    if np.any(np.linalg.norm(np.diff(points,axis=0),axis=1)>1.5*speed_bound*intervals+.15):
        raise ValueError('recorded localization discontinuity exceeds speed/time bound')
    if np.linalg.norm(points[-1]-points[0])>.3 or abs(_wrap(yaw[-1]-yaw[0]))>np.deg2rad(20):
        raise ValueError('recording is not an explicitly closed lap')
    keep=[0]
    for i in range(1,len(points)):
        if np.linalg.norm(points[i]-points[keep[-1]])>=.02: keep.append(i)
    points=points[keep];yaw=yaw[keep]
    if np.linalg.norm(points[-1]-points[0])<.02: points=points[:-1];yaw=yaw[:-1]
    delta=np.roll(points,-1,axis=0)-points
    if np.any(np.sum(delta*np.c_[np.cos(yaw),np.sin(yaw)],axis=1)<=0): raise ValueError('heading inconsistent with forward path')
    path=ReferencePath(points,left_width,right_width)
    dense_s=np.linspace(0,path.length,max(100,int(np.ceil(path.length/.02))),endpoint=False)
    dense=path.curve.numpy(dense_s)
    # Compare each interpolated point to its original chord, preserving local topology.
    idx=np.minimum(np.searchsorted(path.s,dense_s,side='right')-1,len(points)-1)
    a=points[idx]; b=points[(idx+1)%len(points)]; chord=b-a
    t=np.clip(np.sum((dense-a)*chord,axis=1)/np.sum(chord*chord,axis=1),0,1)
    deviation=float(np.max(np.linalg.norm(dense-a-t[:,None]*chord,axis=1)))
    if deviation>.05: raise ValueError('spline displaces recording by more than 0.05 m')
    _check_intersections(dense)
    if max(abs(path.at(s)['curvature']) for s in dense_s)>np.tan(config.steer_limit)/config.wheelbase:
        raise ValueError('path curvature exceeds steering capability')
    # Check footprint at recorded headings against the same local corridor as the OCP.
    for p,h,s in zip(points,yaw,path.s[:-1]):
        ref=path.at(s);normal=np.array([-np.sin(ref['yaw']),np.cos(ref['yaw'])])
        forward=np.array([np.cos(h),np.sin(h)]);left=np.array([-np.sin(h),np.cos(h)])
        for sx in [-1,1]:
            for sy in [-1,1]:
                lateral=np.dot((config.rear_offset+sx*config.half_length)*forward+sy*config.half_width*left,normal)
                if not -right_width<lateral<left_width: raise ValueError('recorded body heading violates corridor')
    samples=np.linspace(0,path.length,int(np.ceil(path.length/.1)),endpoint=False)
    result=ReferencePath(path.curve.numpy(samples),left_width,right_width,metadata=dict(closed_lap=True,
        source_frame='base_link',vehicle_geometry={k:getattr(config,k) for k in ['wheelbase','rear_offset','half_length','half_width']},sample_spacing_m=.1,max_spline_displacement_m=deviation,
        rear_offset_m=config.rear_offset,start_time=float(values[0,0]),end_time=float(values[-1,0])))
    if max(abs(result.at(s)['curvature']) for s in np.linspace(0,result.length,len(result.points)*5,endpoint=False))>np.tan(config.steer_limit)/config.wheelbase:
        raise ValueError('resampled path curvature exceeds steering capability')
    # Exclusive creation also prevents overwrites if a bundle appeared during validation.
    try:
        output.mkdir(parents=True,exist_ok=False)
    except FileExistsError as exc:
        raise ValueError('output directory already exists; choose a new recording bundle') from exc
    result.save(output)
    if src.resolve()!=(output/'raw.csv').resolve(): shutil.copyfile(src,output/'raw.csv')
    return result
