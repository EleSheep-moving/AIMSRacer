#!/usr/bin/env python3
"""Source-time waveform comparison; no shared sensor is independent truth."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interpolate(t, values, grid, gap):
    assert np.all(np.diff(t) > 0) and np.isfinite(values).all()
    indices = np.clip(np.searchsorted(t, grid, side='right')-1, 0, len(t)-2)
    good = (grid >= t[0]) & (grid <= t[-1]) & (t[indices+1]-t[indices] <= gap)
    return np.interp(grid, t, values), good


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    metadata = json.loads((args.raw/'raw-meta.json').read_text())
    assert digest(args.raw/'raw.npz') == metadata['npz_sha256']
    origin = metadata['origin_ns']
    with np.load(args.raw/'raw.npz') as archive:
        raw = {key: archive[key].copy() for key in archive.files}
    replay = json.loads((args.replay/'summary.json').read_text())
    assert replay['input_source'] == metadata['source']
    assert digest(args.replay/'outputs.csv') == replay['output_sha256']
    own = {key: [] for key in ['velocity', 'yaw_rate']}
    with (args.replay/'outputs.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['reference'] == 'rear_axle' and row['kind'] in own:
                own[row['kind']].append([(int(row['original_source_ns'])-origin)*1e-9,
                                         float(row['x']), float(row['y'])])
    own = {key: np.asarray(values) for key, values in own.items()}
    grid = np.arange(8800, 43800)*.005  # source-relative [44,219) s, no endpoint drift
    lags = np.arange(-160, 161)*.005
    ot = (raw['odometry__filtered_ts'][:, 1]-origin)*1e-9
    result = dict(method='estimate(t) versus reference(t-lag); positive lag = estimate trails reference',
        source_window_s=[44., 219.], step_s=.005, lag_search_ms=[-800, 800],
        independent_truth=False, fitted_gain_or_bias=False,
        interpretation='waveform alignment and disagreement only; not physical delay or accuracy',
        same_mask_for_estimators_and_all_candidate_lags=True,
        provenance=dict(raw_sha256=digest(args.raw/'raw.npz'), replay_csv_sha256=replay['output_sha256'],
                        actual_executable_sha256=replay['implementation']['actual_executable_sha256'],
                        script_sha256=digest(Path(__file__))), comparisons={})
    for name, topic, column, own_kind, own_column, gap, unit in [
        ('vx', 'rear_axle__wheel_odom', 0, 'velocity', 1, .04, 'm/s'),
        ('vy', 'rear_axle__lio_odom', 1, 'velocity', 2, .25, 'm/s'),
        ('yaw_rate', 'rear_axle__imu', 5, 'yaw_rate', 1, .03, 'rad/s'),
    ]:
        rt = (raw[topic+'_ts'][:, 1]-origin)*1e-9
        values, support = {}, []
        for label, t, v, max_gap in [
            ('robot_localization', ot, raw['odometry__filtered'][:, {'vx': 0, 'vy': 1, 'yaw_rate': 2}[name]], .04),
            ('own_observer', own[own_kind][:, 0], own[own_kind][:, own_column], .03),
        ]:
            values[label], good = interpolate(t, v, grid, max_gap)
            support.append(good)
        targets = []
        for lag in lags:
            v, good = interpolate(rt, raw[topic][:, column], grid-lag, gap)
            targets.append(v); support.append(good)
        common = np.logical_and.reduce(support)
        assert common.sum() > .9*len(grid), (name, common.sum())
        targets = np.asarray(targets)
        metric = dict(reference_topic='/'+topic.replace('__', '/'), unit=unit,
                      n=int(common.sum()), estimators={})
        for label, estimate in values.items():
            scores = np.sqrt(np.mean((estimate[None, common]-targets[:, common])**2, axis=1))
            best = int(np.argmin(scores))
            metric['estimators'][label] = dict(zero_shift_rmse=float(scores[160]),
                best_lag_ms=float(lags[best]*1000), best_rmse=float(scores[best]),
                boundary_optimum=best in (0, len(lags)-1))
        result['comparisons'][name] = metric
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
