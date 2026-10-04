#!/usr/bin/env python3
"""Offline first-scan map initialization for bags without recorded map alignment.

Runs the pinned upstream BBS + NDT candidate search once, exports hypotheses and
an explicit map/base_link seed. This is a bootstrap hypothesis, not ground truth.
It is never used for ongoing state propagation or automatic recovery.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message
from livox_ros_driver2.msg import CustomMsg
from localization_map_io import map_points


def json_safe(value):
    """Preserve failed hypotheses without nonstandard JSON Infinity/NaN."""
    if isinstance(value,float) and not math.isfinite(value):
        return None
    if isinstance(value,dict):
        return {key:json_safe(item) for key,item in value.items()}
    if isinstance(value,(tuple,list)):
        return [json_safe(item) for item in value]
    return value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Output must be new to preserve seed evidence.')
    args.output.mkdir(parents=True)
    prefix=Path(get_package_prefix('lidar_localization_ros2'))
    sys.path.insert(0,str(prefix/'lib/lidar_localization_ros2'))
    from global_localization_query import GlobalLocalizationEngine,GlobalLocalizationConfig
    from PIL import Image
    points=map_points(args.map.read_bytes())
    walls=points[(points[:,2]>.5)&(points[:,2]<3.)]
    resolution=.1
    origin=np.floor(walls[:,:2].min(axis=0)/resolution)*resolution-.5
    size=np.ceil((walls[:,:2].max(axis=0)-origin)/resolution).astype(int)+5
    if np.prod(size)>10000000:raise ValueError('Occupancy extent is too large for the first-scan bootstrap.')
    occupied=np.zeros((size[1],size[0]),dtype=np.uint8)
    indices=np.floor((walls[:,:2]-origin)/resolution).astype(int)
    occupied[indices[:,1],indices[:,0]]=1
    Image.fromarray(np.flipud(np.where(occupied,0,254).astype(np.uint8))).save(args.output/'occupancy.png')
    occupancy=dict(image='occupancy.png',resolution=resolution,origin=[float(origin[0]),float(origin[1]),0.],negate=0,occupied_thresh=.65,free_thresh=.25)
    (args.output/'occupancy.yaml').write_text(yaml.safe_dump(occupancy))
    db=sorted(args.bag.glob('*.db3'))
    if len(db)!=1:raise ValueError('Single database required.')
    with sqlite3.connect(f'file:{db[0]}?mode=ro',uri=True) as con:
        start=con.execute('select min(timestamp) from messages').fetchone()[0]
        ident=con.execute("select id from topics where name='/livox/lidar'").fetchone()[0]
        ts,data=con.execute('select timestamp,data from messages where topic_id=? order by abs(timestamp-?) limit 1',(ident,start+int(1e9))).fetchone()
        cloud=deserialize_message(data,CustomMsg)
    geometry=yaml.safe_load(args.geometry.read_text())
    xyz=np.array([[p.x,p.y,p.z] for p in cloud.points],dtype=np.float32)
    xyz=Rotation.from_euler('xyz',geometry['livox_rpy']).apply(xyz)+geometry['livox_translation']
    ranges=np.linalg.norm(xyz,axis=1)
    xyz=xyz[np.isfinite(xyz).all(axis=1)&(ranges>=.5)&(ranges<=30.)]
    config=GlobalLocalizationConfig(z_min_m=.5,z_max_m=3.,max_scan_points=512,angular_resolution_rad=math.radians(5.),
        pyramid_depth=5,max_candidates=16,nms_radius_m=1.,dilate_cells=1,use_cpp_backend=True,
        enable_registration_scoring=True,registration_refine_candidates=True,map_path=str(args.map),registration_score_gate=1.5,
        ndt_resolution=1.,ndt_num_threads=2,ndt_max_iterations=35,ndt_scan_voxel_leaf_size=.2,ndt_local_map_radius=30.)
    engine=GlobalLocalizationEngine(config,occupancy_yaml=args.output/'occupancy.yaml')
    if engine.backend!='cpp' or engine.registration_scorer is None:
        raise RuntimeError(f'Pinned native bootstrap backends unavailable: {engine.backend_error} {engine.registration_scoring_error}')
    started=time.monotonic()
    result=engine.query(xyz)
    tree=cKDTree(points)
    sample=xyz[::max(1,math.ceil(len(xyz)/2000))]
    candidates=[]
    for candidate in result.candidates:
        row=asdict(candidate)
        transform=Rotation.from_euler('z',candidate.yaw_rad)
        moved=transform.apply(sample)+[candidate.x_m,candidate.y_m,candidate.z_m]
        distances,_=tree.query(moved,workers=1)
        row['independent_inlier_fraction']=float(np.mean(distances<=.25))
        candidates.append(row)
    evidence=dict(config=asdict(config),result=asdict(result),candidates=candidates,wall_seconds=time.monotonic()-started,
                  map_sha256=hashlib.sha256(args.map.read_bytes()).hexdigest(),source='offline_first_raw_scan_bbs_ndt',
                  scan_record_sec=ts*1e-9,scan_start_sec=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9,
                  warning='B lacks recorded map identity/alignment; this assumes the supplied map describes this scene. No ground truth. Raw stationary bootstrap scan, no ongoing historical input.')
    (args.output/'candidates.json').write_text(json.dumps(json_safe(evidence),indent=2,allow_nan=False)+'\n')
    good=[r for r in candidates if r['registration_converged'] and r['registration_fitness'] is not None and r['registration_fitness']<.05 and r['independent_inlier_fraction']>=.9]
    if not good:raise RuntimeError('No bootstrap candidate passes fitness<.05 and independent inlier>=.9. Inspect candidates; no seed exported.')
    best=good[0]
    distant=[r for r in good[1:] if math.hypot(r['x_m']-best['x_m'],r['y_m']-best['y_m'])>1. or abs(math.atan2(math.sin(r['yaw_rad']-best['yaw_rad']),math.cos(r['yaw_rad']-best['yaw_rad'])))>.2]
    if any(r['registration_fitness']<=best['registration_fitness']*1.2 for r in distant):
        raise RuntimeError('Ambiguous distant map hypotheses; inspect candidates and supply a manual seed.')
    seed=dict(position=[best['x_m'],best['y_m'],best['z_m']],orientation=Rotation.from_euler('z',best['yaw_rad']).as_quat().tolist(),
              provenance=dict(**{k:evidence[k] for k in ('source','map_sha256','scan_record_sec','scan_start_sec','warning')},candidate=best,evidence_file=str(args.output/'candidates.json')))
    (args.output/'initial_pose.json').write_text(json.dumps(seed,indent=2,allow_nan=False)+'\n')
    print(json.dumps(seed))


if __name__=='__main__':
    main()
