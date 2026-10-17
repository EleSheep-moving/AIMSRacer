"""Offline map/rear-axle reference preparation; no ROS imports or publishers."""
import re
import numpy as np
from .path import ReferencePath

GEOMETRY_KEYS = ('wheelbase', 'rear_offset', 'half_width', 'half_length', 'front_extent', 'rear_extent')


def check_curve_intersections(points):
    """Reject touching/crossing nonadjacent evaluated chords using exact segments."""
    points=np.asarray(points,float)
    ends=np.roll(points,-1,axis=0)
    low=np.minimum(points,ends);high=np.maximum(points,ends)
    count=len(points)
    def cross(a,b): return a[...,0]*b[...,1]-a[...,1]*b[...,0]
    for i in range(count-2):
        js=np.arange(i+2,count-(1 if i==0 else 0))
        candidate=np.all(high[i]>=low[js]-1e-10,axis=1)&np.all(high[js]>=low[i]-1e-10,axis=1)
        js=js[candidate]
        if not len(js): continue
        a,b=points[i],ends[i];c,d=points[js],ends[js]
        if np.any((cross(b-a,c-a)*cross(b-a,d-a)<=1e-12)&
                  (cross(d-c,a-c)*cross(d-c,b-c)<=1e-12)):
            raise ValueError('self-intersecting or touching evaluated curve')


def validate_regions(regions,length):
    result=[];ids=set()
    if not isinstance(regions,list): raise ValueError('calibration regions must be a list')
    for region in regions:
        if not isinstance(region,dict): raise ValueError('invalid calibration region')
        start=region.get('start_s');end=region.get('end_s')
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not np.isfinite(v)
               or not 0<=v<length for v in (start,end)) or start==end:
            raise ValueError('calibration region progress must be distinct and within [0,length)')
        id_=region.get('id');name=region.get('name',id_)
        if not isinstance(id_,str) or not id_ or id_ in ids or not isinstance(name,str) or not name:
            raise ValueError('unique calibration region id and name required')
        if region.get('kind')!='straight': raise ValueError('calibration region kind must be straight')
        ids.add(id_);result.append(dict(id=id_,name=name,start_s=float(start),end_s=float(end),kind='straight'))
    boundaries=sorted({0.,length,*[r[k] for r in result for k in ('start_s','end_s')]})
    for a,b in zip(boundaries,boundaries[1:]):
        midpoint=(a+b)/2
        def inside(r):
            start,end=r['start_s'],r['end_s']
            return start<=midpoint<end if start<end else midpoint>=start or midpoint<end
        if sum(inside(r) for r in result)>1: raise ValueError('overlapping calibration progress regions')
    return result


def prepare_gui_reference(project, config):
    """Fit once from the exact editor points; dense checks are sampled validation."""
    if not isinstance(project,dict): raise ValueError('editable project mapping required')
    if type(project.get('schema_version')) is not int or project['schema_version']!=1:
        raise ValueError('unsupported editor project schema_version')
    if project.get('frame_id','map')!='map' or project.get('coordinate_reference','rear_axle')!='rear_axle':
        raise ValueError('GUI supports map-frame rear-axle references only')
    map_hash=project.get('map_sha256')
    if not isinstance(map_hash,str) or re.fullmatch(r'[0-9a-f]{64}',map_hash) is None:
        raise ValueError('original map SHA-256 required')
    points=np.asarray(project.get('waypoints'),float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<6 or not np.isfinite(points).all():
        raise ValueError('at least six unique finite XY waypoints required')
    if len(points)>1000: raise ValueError('at most 1000 editable waypoints supported')
    if len(np.unique(points,axis=0))!=len(points): raise ValueError('at least six unique finite XY waypoints required')
    metadata=dict(schema_version=1,source='gui',source_frame='base_link',closed_lap=True,
                  coordinate_reference='rear_axle',map_sha256=map_hash,
                  vehicle_geometry={k:getattr(config,k) for k in GEOMETRY_KEYS},
                  validation_sample_spacing_m=.02)
    if project.get('speed_replan') is not None:
        evidence=project['speed_replan']
        keys=('prior_reference_sha256','prior_speed_profile_sha256','selected_config_sha256')
        if not isinstance(evidence,dict) or any(not isinstance(evidence.get(k),str) or re.fullmatch(r'[0-9a-f]{64}',evidence[k]) is None for k in keys):
            raise ValueError('complete SHA-256 speed replan evidence required')
        metadata['speed_replan']={key:evidence[key] for key in keys}
    path=ReferencePath(points,project.get('left_width',.5),project.get('right_width',.5),
                       frame_id='map',metadata=metadata)
    path.validate_config(config,require_recording=True)
    count=max(200,int(np.ceil(path.length/.02)),len(points)*20)
    if count>20000: raise ValueError('reference exceeds supported 20000 validation samples')
    positions=np.linspace(0,path.length,count,endpoint=False)
    xy=path.curve.numpy(positions);d=path.curve.numpy(positions,1);dd=path.curve.numpy(positions,2)
    norms=np.linalg.norm(d,axis=1)
    if not np.isfinite(xy).all() or np.min(norms)<1e-6: raise ValueError('zero or nonfinite tangent')
    for derivative in range(3):
        if not np.allclose(path.curve._spline(0,nu=derivative),path.curve._spline(path.length,nu=derivative),rtol=0,atol=1e-8):
            raise ValueError('closed seam must match position, tangent and curvature')
    check_curve_intersections(xy)
    curvature=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/norms**3
    steering=np.arctan(config.wheelbase*curvature)
    if np.max(np.abs(steering))>config.steer_limit+1e-9:
        raise ValueError('path curvature exceeds steering capability')
    regions=validate_regions(project.get('calibration_regions',[]),path.length)
    return path,dict(positions=positions,xy=xy,curvature=curvature,steering=steering,regions=regions)
