"""Build a map-frame rear-axle lap from saved PGO key poses and a matched CSV.

The CSV supplies the selected circuit, time and speed. Optimized PGO key poses
supply map-frame geometry. The output remains a candidate until the footprint,
track corridor and live relocalization have been checked on the vehicle.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def recover(poses_file, geometry_file, source_csv, output, first, last):
    poses_file, geometry_file, source_csv, output = map(
        Path, (poses_file, geometry_file, source_csv, output))
    map_file = poses_file.parent / 'map.pcd'
    if not map_file.is_file():
        raise ValueError('saved PGO map.pcd missing beside poses.txt')
    with geometry_file.open() as stream:
        geometry = yaml.safe_load(stream)
    mount_xyz = np.asarray(geometry['livox_translation'], dtype=float)
    mount_rpy = np.asarray(geometry['livox_rpy'], dtype=float)
    if mount_xyz.shape != (3,) or mount_rpy.shape != (3,) or not np.isfinite(
            np.r_[mount_xyz, mount_rpy]).all():
        raise ValueError('invalid rear-to-Livox geometry')
    mount_rotation = Rotation.from_euler('xyz', mount_rpy)
    lines = poses_file.read_text().splitlines()
    if first < 0 or last >= len(lines) or last - first < 9:
        raise ValueError('invalid inclusive PGO key-pose indices')
    pose_rows = [line.split() for line in lines[first:last + 1]]
    for index, row in enumerate(pose_rows, first):
        if len(row) != 8 or row[0] != f'{index}.pcd':
            raise ValueError(f'unexpected PGO pose row {index}')
    pose_values = np.asarray([[float(v) for v in row[1:]] for row in pose_rows])
    if not np.isfinite(pose_values).all():
        raise ValueError('nonfinite PGO pose')
    raw_rotation = Rotation.from_quat(pose_values[:, [4, 5, 6, 3]])
    base_rotation = raw_rotation * mount_rotation.inv()
    rear_xyz = pose_values[:, :3] - base_rotation.apply(mount_xyz)
    map_xy = rear_xyz[:, :2]
    yaw = np.unwrap(np.arctan2(base_rotation.as_matrix()[:, 1, 0],
                                base_rotation.as_matrix()[:, 0, 0]))
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(map_xy, axis=0), axis=1))]
    if np.any(np.diff(distance) <= 0) or distance[-1] < 2.:
        raise ValueError('invalid or too short PGO lap')
    closure = map_xy[-1] - map_xy[0]
    yaw_drift = wrap(yaw[-1] - yaw[0])
    if np.linalg.norm(closure) > .3 or abs(yaw_drift) > math.radians(20):
        raise ValueError('PGO segment is not one nearly closed lap')
    fraction = distance / distance[-1]
    closed_xy = map_xy - fraction[:, None] * closure
    corrected_yaw = yaw - fraction * yaw_drift

    with source_csv.open(newline='') as stream:
        source = list(csv.DictReader(stream))
    if len(source) < 10 or any((r['frame_id'], r['child_frame_id']) !=
                               ('odom', 'base_link') for r in source):
        raise ValueError('source must be the recovered odom/base_link lap')
    values = np.asarray([[float(r[k]) for k in ('timestamp', 'x', 'y', 'speed')]
                         for r in source])
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError('invalid source CSV')
    source_xy = values[:, 1:3]
    source_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(source_xy, axis=0), axis=1))]
    source_fraction = source_s / source_s[-1]
    # A rigid spatial fit checks that the selected PGO segment is the same lap.
    matched = np.c_[np.interp(source_fraction, fraction, closed_xy[:, 0]),
                    np.interp(source_fraction, fraction, closed_xy[:, 1])]
    x, y = source_xy[:-1], matched[:-1]
    cx, cy = x.mean(axis=0), y.mean(axis=0)
    left, _, right = np.linalg.svd((x - cx).T @ (y - cy))
    rotation = left @ np.diag([1., np.linalg.det(left @ right)]) @ right
    translation = cy - cx @ rotation
    errors = np.linalg.norm(x @ rotation + translation - y, axis=1)
    rms = float(np.sqrt(np.mean(errors ** 2)))
    if rms > .15 or float(np.max(errors)) > .3:
        raise ValueError('PGO key poses do not match the selected CSV circuit')

    times = np.interp(fraction, source_fraction, values[:, 0])
    speeds = np.interp(fraction, source_fraction, values[:, 3])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('timestamp', 'x', 'y', 'yaw', 'speed', 'frame_id', 'child_frame_id'))
        for timestamp, point, angle, speed in zip(times, closed_xy, corrected_yaw, speeds):
            writer.writerow((timestamp, point[0], point[1], wrap(angle),
                             speed, 'map', 'base_link'))
    report = dict(source_csv=str(source_csv.resolve()),
                  source_sha256=hashlib.sha256(source_csv.read_bytes()).hexdigest(),
                  poses_file=str(poses_file.resolve()), first_pose=first, last_pose=last,
                  geometry_file=str(geometry_file.resolve()), map_file=str(map_file.resolve()),
                  map_sha256=hashlib.sha256(map_file.read_bytes()).hexdigest(),
                  samples=len(times), length_m=float(distance[-1]),
                  closure_distance_m=float(np.linalg.norm(closure)),
                  yaw_drift_deg=math.degrees(yaw_drift),
                  matched_csv_rms_m=rms, matched_csv_max_m=float(np.max(errors)),
                  rigid_match_yaw_deg=math.degrees(math.atan2(rotation[0, 1], rotation[0, 0])),
                  output_frame='map', source_frame='base_link')
    output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses_file')
    parser.add_argument('geometry_file')
    parser.add_argument('source_csv')
    parser.add_argument('output')
    parser.add_argument('--first-pose', type=int, required=True)
    parser.add_argument('--last-pose', type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(recover(args.poses_file, args.geometry_file, args.source_csv,
                             args.output, args.first_pose, args.last_pose), indent=2))


if __name__ == '__main__':
    main()
