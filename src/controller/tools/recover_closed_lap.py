"""Recover a single odom-frame lap from a longer recorder CSV.

This only removes small accumulated closure drift. It does not register the
path to a PGO map or verify track boundaries and vehicle clearance.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def recover(source, output, first, last, sigma=0.75):
    source, output = Path(source), Path(output)
    with source.open(newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = list(reader)
    required = {'timestamp', 'x', 'y', 'yaw', 'speed', 'frame_id', 'child_frame_id'}
    if fields is None or not required.issubset(fields):
        raise ValueError('missing recorder CSV columns')
    if first < 0 or last >= len(rows) or last - first < 9:
        raise ValueError('invalid inclusive row indices')
    rows = rows[first:last + 1]
    if any((r['frame_id'], r['child_frame_id']) != ('odom', 'base_link') for r in rows):
        raise ValueError('expected odom/base_link input')
    values = np.asarray([[float(r[k]) for k in ('timestamp', 'x', 'y', 'yaw', 'speed')]
                         for r in rows])
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError('nonfinite or non-increasing recording')
    if np.min(values[:, 4]) < -0.05:
        raise ValueError('selected lap includes reverse motion')
    xy = values[:, 1:3]
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    if distance[-1] < 2. or np.any(np.diff(distance) <= 0):
        raise ValueError('selected lap is too short or has stationary duplicates')
    closure = xy[-1] - xy[0]
    heading = np.unwrap(values[:, 3])
    heading_drift = wrap(heading[-1] - heading[0])
    if np.linalg.norm(closure) > 0.3 or abs(heading_drift) > math.radians(20):
        raise ValueError('selected lap does not close within 0.3 m and 20 deg')
    if not 0 <= sigma <= 2:
        raise ValueError('sigma must be in [0, 2] samples')

    fraction = distance / distance[-1]
    closed = xy - fraction[:, None] * closure
    smoothed = closed[:-1] if sigma == 0 else gaussian_filter1d(
        closed[:-1], sigma=sigma, axis=0, mode='wrap')
    displacement = np.linalg.norm(smoothed - closed[:-1], axis=1)
    if np.max(displacement) > 0.05:
        raise ValueError('smoothing moves a point by more than 5 cm')
    corrected_xy = np.vstack([smoothed, smoothed[0]])
    corrected_yaw = heading - fraction * heading_drift
    corrected_yaw[-1] = corrected_yaw[0] + 2 * math.pi * round(
        (corrected_yaw[-1] - corrected_yaw[0]) / (2 * math.pi))

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row, point, yaw in zip(rows, corrected_xy, corrected_yaw):
            writer.writerow(dict(row, x=repr(float(point[0])), y=repr(float(point[1])),
                                 yaw=repr(wrap(float(yaw)))))
    report = dict(source=str(source.resolve()), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  first_row=first, last_row=last, samples=len(rows),
                  start_time=float(values[0, 0]), end_time=float(values[-1, 0]),
                  lap_length_m=float(distance[-1]), closure_vector_m=closure.tolist(),
                  closure_distance_m=float(np.linalg.norm(closure)),
                  yaw_drift_deg=math.degrees(heading_drift), gaussian_sigma_samples=sigma,
                  max_smoothing_displacement_m=float(np.max(displacement)),
                  output_frame='odom', map_aligned=False)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('output')
    parser.add_argument('--first-row', type=int, required=True)
    parser.add_argument('--last-row', type=int, required=True)
    parser.add_argument('--sigma', type=float, default=0.75)
    args = parser.parse_args()
    print(json.dumps(recover(args.source, args.output, args.first_row,
                             args.last_row, args.sigma), indent=2))


if __name__ == '__main__':
    main()
