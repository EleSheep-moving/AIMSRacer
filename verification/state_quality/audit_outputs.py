#!/usr/bin/env python3
"""Read-only audit of archived observer outputs; does not run ROS or tune filters.

Requires NumPy, SciPy and Matplotlib. Paths identify the accepted 2026-10-15
MPCC recording; --data-root relocates its external evidence. Writes only a new
output directory. Missing independent vy/CG outputs remain missing.
"""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ORIGIN = 1792002365474876930
STATIC = (13.065745826, 41.425777542)
RUN = (41.926416916, 220.431558514)
G = 9.80665


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'n': 0}
    assert np.isfinite(a).all(), 'Nonfinite data must be reported, not dropped'
    return dict(n=len(a), mean=float(a.mean()), std_ddof0=float(a.std()),
                median=float(np.median(a)), abs_p95=float(np.percentile(abs(a), 95)),
                abs_max=float(abs(a).max()), minimum=float(a.min()), maximum=float(a.max()))


def read_trace(path):
    # dtype inference retains integer nanosecond timestamps, never round-trips
    # them through a float matrix.
    a = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')
    assert np.issubdtype(a['stamp_ns'].dtype, np.integer)
    return a


def window(a, bounds):
    t = (a['stamp_ns'] - ORIGIN) * 1e-9
    return a[(t >= bounds[0]) & (t <= bounds[1])]


def select(a):
    mask = (a['accepted'] == 1) & (a['valid'] == 1)
    if 'event' in a.dtype.names:
        mask &= np.char.lower(a['event']) == 'imu'
    a = a[mask]
    assert np.all(np.diff(a['stamp_ns']) > 0), 'Outputs must be unique and increasing'
    return a


def yaw_recalculate(a):
    """Matrix implementation, independent of archived scalar C++ arithmetic."""
    x = None
    prev = None
    P = None
    outputs = []
    for row in a:
        stamp, z = int(row['stamp_ns']), float(row['measured_rate'])
        dt = None if prev is None else (stamp - prev) * 1e-9
        if dt is None or dt > .03 + 1e-12:
            x, P = np.array([z, 0.]), np.diag([1e-4, 100.])
        else:
            F = np.array([[1., dt], [0., 1.]])
            Q = 4 * np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]])
            x, P = F @ x, F @ P @ F.T + Q
            K = P[:, 0] / (P[0, 0] + 1e-4)
            x += K * (z - x[0])
            A = np.eye(2); A[:, 0] -= K
            P = A @ P @ A.T + 1e-4 * np.outer(K, K)
        prev = stamp
        outputs.append(x.copy())
    out = np.asarray(outputs)
    return dict(rate_max_abs_difference=float(abs(out[:, 0] - a['rate']).max()),
                alpha_max_abs_difference=float(abs(out[:, 1] - a['alpha']).max()),
                n=len(a), scope='same measurements and model; arithmetic check, not physical truth')


def relative_velocity(raw, origin):
    def stream(key):
        ts = raw[key + '_ts'][:, 1]
        assert np.all(np.diff(ts) > 0)
        assert np.isfinite(raw[key]).all()
        return (ts - origin) * 1e-9, raw[key]
    ot, ekf = stream('odometry__filtered')
    lt, lio = stream('rear_axle__lio_odom')
    wt, wheel = stream('rear_axle__wheel_odom')
    grid = np.arange(8800, 43800) * .005
    lags = np.arange(-160, 161) * .005
    results = {}
    for name, rt, ref, estimate, max_gap in [
        ('vx_ekf_relative_wheel', wt, wheel[:, 0], ekf[:, 0], .04),
        ('vy_ekf_relative_lio', lt, lio[:, 1], ekf[:, 1], .25),
    ]:
        y = np.interp(grid, ot, estimate, left=np.nan, right=np.nan)
        ei = np.clip(np.searchsorted(ot, grid, side='right') - 1, 0, len(ot)-2)
        mask = np.isfinite(y) & (ot[ei+1]-ot[ei] <= .04)
        target = []
        for lag in lags:
            g = grid - lag
            idx = np.clip(np.searchsorted(rt, g, side='right')-1, 0, len(rt)-2)
            mask &= (g >= rt[0]) & (g <= rt[-1]) & (rt[idx+1]-rt[idx] <= max_gap)
            target.append(np.interp(g, rt, ref, left=np.nan, right=np.nan))
        target = np.asarray(target)
        scores = np.sqrt(np.mean((y[None, mask]-target[:, mask])**2, axis=1))
        best = int(np.argmin(scores))
        shifts = []
        for start in np.arange(44., 214., 5.):
            m = mask & (grid >= start) & (grid < start+5)
            score = np.sqrt(np.mean((y[None, m]-target[:, m])**2, axis=1))
            shifts.append(float(lags[int(np.argmin(score))]*1000))
        results[name] = dict(n=int(mask.sum()), zero_shift_rmse_mps=float(scores[lags == 0][0]),
                             best_lag_ms=float(lags[best]*1000), best_rmse_mps=float(scores[best]),
                             window_lag_ms_min=min(shifts), window_lag_ms_max=max(shifts),
                             boundary_optimum=best in (0, len(lags)-1))
    return results, (ot, ekf, lt, lio, wt, wheel)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    asset = args.repo / 'docs/reports/assets'
    experiment = args.data_root / 'experiments'
    provenance = {}

    def checked(path, expected=None):
        digest = sha(path)
        provenance[str(path)] = dict(sha256=digest, bytes=path.stat().st_size,
                                     expected_sha256=expected, matches_expected=None if expected is None else digest == expected)
        assert expected is None or digest == expected, f'Hash mismatch: {path}'
        return path

    def js(path):
        return json.loads(checked(path).read_text())

    vp = experiment / 'vx-observer-20261015'
    psummary = js(asset/'2026-10-15-planar-acceleration/summary.json')
    ysummary = js(asset/'2026-10-15-yaw-rate/quality-summary.json')
    vsummary = js(vp/'accepted/offline/mpcc/summary.json')
    vpath = checked(vp/'accepted/offline/mpcc/trace.csv', vsummary['trace_sha256'])
    ppath = checked(experiment/'planar-acceleration-20261015/quality-02/mpcc-trace.csv', psummary['runs'][0]['trace_sha256'])
    ypath = checked(experiment/'yaw-rate-20261015/quality-02/mpcc-trace.csv', ysummary['runs'][0]['trace_sha256'])
    va, pa, ya = [read_trace(p) for p in [vpath, ppath, ypath]]
    v, p, y = [select(a) for a in [va, pa, ya]]
    assert (p['lateral_valid'] == 1).all(), 'AY requires lateral validity'
    out = dict(audited_at_utc=datetime.now(timezone.utc).isoformat(),
               runtime=dict(python=platform.python_version(), numpy=np.__version__,
                            scipy=importlib.metadata.version('scipy'),
                            matplotlib=importlib.metadata.version('matplotlib')),
               evidence_recording_date='2026-10-15', origin_ns=ORIGIN,
               static_window_s=STATIC, running_window_s=RUN, independent_truth=False,
               reference_point='base_link rear axle; no CG or independent vy output exists',
               metric_definition='std ddof=0; abs_p95=percentile(abs(x),95); no mean removal or time shift',
               geometry_user_estimate_20261017=dict(imu_to_cg_body_m=[-.09, -.03, -.03],
                    z_is_estimate=True, uncertainty_m=None, applied_to_outputs=False),
               counts={}, static={}, running={}, algebra_checks={}, archived_evidence={})
    for name, full, selected in [('vx', va, v), ('planar', pa, p), ('yaw', ya, y)]:
        imu = full if 'event' not in full.dtype.names else full[np.char.lower(full['event']) == 'imu']
        out['counts'][name] = dict(events=len(full), imu_rows=len(imu), valid_imu_rows=len(selected),
                                  running_valid_imu_rows=len(window(selected, RUN)),
                                  accepted_imu_rows=int((imu['accepted'] == 1).sum()))
        if 'status' in full.dtype.names:
            out['counts'][name]['imu_status_counts'] = dict(Counter(imu['status']))
    for name, a, field in [('vx', v, 'vx'), ('ax', p, 'ax'), ('ay', p, 'ay'), ('yaw_rate', y, 'rate')]:
        out['static'][name] = stats(window(a, STATIC)[field])
        out['running'][name] = stats(window(a, RUN)[field])
    for key in ['raw_ax', 'raw_ay', 'alpha_window_span_s', 'alpha_sample_center_age_s']:
        out['static'][key] = stats(window(p, STATIC)[key])
        out['running'][key] = stats(window(p, RUN)[key])
    out['static']['corrected_gyro_z'] = stats(window(y, STATIC)['measured_rate'])
    omega = np.column_stack([p['omega_'+a] for a in 'xyz'])
    alpha = np.column_stack([p['alpha_'+a] for a in 'xyz'])
    force = np.column_stack([p['force_'+a] for a in 'xyz'])
    gravity = np.column_stack([p['gravity_'+a] for a in 'xyz'])
    lever = np.array([.30, 0., .03])
    reconstructed = force + gravity - np.cross(alpha, lever) - np.cross(omega, np.cross(omega, lever))
    out['algebra_checks']['unfiltered_acceleration_xyz_max_abs_difference'] = {
        a: float(abs(reconstructed[:, j]-p['raw_a'+a]).max()) for j, a in enumerate('xyz')}
    # Hypothetical +1 cm height perturbation, not a measured error bound. These
    # are changes to the unfiltered rigid-body conversion, not new CG outputs.
    dz = np.array([0., 0., .01])
    delta_v = np.cross(omega, dz)
    delta_a = np.cross(alpha, dz) + np.cross(omega, np.cross(omega, dz))
    time = (p['stamp_ns']-ORIGIN)*1e-9
    motion = (time >= RUN[0]) & (time <= RUN[1])
    out['cg_height_sensitivity_per_plus_1cm'] = {
        name: stats(values[motion, axis])
        for name, values, axis in [('dvx', delta_v, 0), ('dvy', delta_v, 1),
                                   ('dax_unfiltered', delta_a, 0), ('day_unfiltered', delta_a, 1)]}
    out['algebra_checks']['yaw_matrix_recalculation'] = yaw_recalculate(ya)
    assert max(out['algebra_checks']['unfiltered_acceleration_xyz_max_abs_difference'].values()) < 1e-10
    assert out['algebra_checks']['yaw_matrix_recalculation']['rate_max_abs_difference'] < 1e-10
    assert out['algebra_checks']['yaw_matrix_recalculation']['alpha_max_abs_difference'] < 1e-9
    common = np.intersect1d(v['stamp_ns'], np.intersect1d(p['stamp_ns'], y['stamp_ns']))
    ct = (common-ORIGIN)*1e-9
    out['common_four_output_stamps'] = dict(all_valid=len(common),
         running=int(((ct >= RUN[0]) & (ct <= RUN[1])).sum()), matching='exact int64 IMU source stamp, no interpolation')
    rawroot = args.data_root/'sessions/2026-10-15/022604-mpcc-review/critical-review'
    metadata = js(rawroot/'raw-meta.json')
    with np.load(checked(rawroot/'raw.npz', metadata['npz_sha256'])) as raw:
        relative, streams = relative_velocity(raw, metadata['origin_ns'])
        out['relative_velocity_comparison'] = relative
        ot, ekf, lt, lio, wt, wheel = streams
        for name, t, a in [('ekf', ot, ekf), ('lio', lt, lio)]:
            for label, bounds in [('static', STATIC), ('running', RUN)]:
                m = (t >= bounds[0]) & (t <= bounds[1])
                out[label]['vy_'+name+'_comparison_only'] = stats(a[m, 1])
    previous = js(args.data_root/'sessions/2026-10-15/022604-mpcc-review/model-consistency-20261015/velocity-input-review.json')
    out['algebra_checks']['historical_relative_metrics_max_abs_difference'] = max(
        abs(out['relative_velocity_comparison'][name][key]-previous['comparisons'][name][key])
        for name in relative for key in ['n', 'zero_shift_rmse_mps', 'best_lag_ms', 'best_rmse_mps'])
    assert out['algebra_checks']['historical_relative_metrics_max_abs_difference'] < 1e-10
    source_groups = [
        ('velocity', js(vp/'manifest.json')['source_sha256'], ['velocity.cpp', 'velocity.hpp', 'velocity_node.cpp', 'velocity.yaml']),
        ('planar', js(asset/'2026-10-15-planar-acceleration/source-hashes.json')['sha256'], ['body_acceleration.cpp', 'body_acceleration.hpp', 'body_acceleration_node.cpp', 'planar_acceleration.yaml']),
        ('yaw', js(asset/'2026-10-15-yaw-rate/validation-summary.json')['source_files'], ['yaw_rate.cpp', 'yaw_rate.hpp', 'yaw_rate_node.cpp', 'yaw_rate.yaml'])]
    out['current_source_vs_archived_snapshot'] = {}
    for group, mapping, names in source_groups:
        out['current_source_vs_archived_snapshot'][group] = {
            key: dict(matches=sha(args.repo/key) == expected, current=sha(args.repo/key), archived=expected)
            for key, expected in mapping.items() if Path(key).name in names and (args.repo/key).exists()}
    out['archived_evidence']['vx_nx'] = js(vp/'accepted/nx/acceptance-summary.json')
    out['archived_evidence']['yaw_synthetic_selected'] = next(r for r in ysummary['synthetic']['records'] if r['q'] == 4)
    out['archived_evidence']['planar_synthetic'] = psummary['synthetic']
    out['archived_evidence']['vx_selected_synthetic'] = js(vp/'accepted/synthetic/synthetic.json')['selected']
    out['source_files'] = provenance
    out['analysis_script_sha256'] = sha(Path(__file__))
    (args.output/'summary.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n')
    with (args.output/'metrics.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['window', 'signal', 'n', 'mean', 'std_ddof0', 'abs_p95', 'abs_max'])
        for label in ['static', 'running']:
            for name, m in out[label].items():
                writer.writerow([label, name, *[m[k] for k in ['n', 'mean', 'std_ddof0', 'abs_p95', 'abs_max']]])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5, 2, figsize=(14, 13), constrained_layout=True)
    rows = [
        ('Vx (m/s)', [((v['stamp_ns']-ORIGIN)*1e-9, v['vx'], 'Independent Vx', '#2464a6', '-'), (wt, wheel[:, 0], 'Wheel comparison', '#b97518', '--')]),
        ('Vy (m/s)', [(ot, ekf[:, 1], 'EKF comparison only', '#2464a6', '--'), (lt, lio[:, 1], 'LIO comparison only', '#b97518', ':')]),
        ('Ax (m/s²)', [((p['stamp_ns']-ORIGIN)*1e-9, p['raw_ax'], 'Reconstructed, unfiltered', '#999999', '-'), ((p['stamp_ns']-ORIGIN)*1e-9, p['ax'], 'Observer output', '#2464a6', '-')]),
        ('Ay (m/s²)', [((p['stamp_ns']-ORIGIN)*1e-9, p['raw_ay'], 'Reconstructed, unfiltered', '#999999', '-'), ((p['stamp_ns']-ORIGIN)*1e-9, p['ay'], 'Observer output', '#2464a6', '-')]),
        ('r (rad/s)', [((y['stamp_ns']-ORIGIN)*1e-9, y['measured_rate'], 'Corrected gyro', '#999999', '-'), ((y['stamp_ns']-ORIGIN)*1e-9, y['rate'], 'Yaw observer', '#2464a6', '-')])]
    for j, (label, series) in enumerate(rows):
        full = va if j == 0 else (pa if j in (2, 3) else (ya if j == 4 else None))
        invalid_t = np.array([]) if full is None else (full['stamp_ns'][full['valid'] == 0]-ORIGIN)*1e-9
        for col, bounds in enumerate([RUN, (RUN[0], RUN[0]+10)]):
            ax = axes[j, col]
            for t, values, name, color, style in series:
                m = (t >= bounds[0]) & (t <= bounds[1])
                indices = np.flatnonzero(m)
                tx, yy = t[indices].copy(), values[indices].copy()
                # Do not connect invalid/gap intervals; each series retains its
                # own source support. LIO is ~10 Hz, others ~100/200 Hz.
                gap = .25 if 'LIO' in name else .03
                if len(tx) > 1:
                    yy[np.r_[False, np.diff(tx) > gap]] = np.nan
                    if name not in ('Wheel comparison',) and len(invalid_t):
                        bad = np.searchsorted(tx, invalid_t)
                        bad = bad[(bad > 0) & (bad < len(tx))]
                        yy[bad] = np.nan
                ax.plot(tx, yy, color=color, ls=style, lw=.65, label=name)
            ax.set_xlim(*bounds); ax.set_ylabel(label); ax.grid(alpha=.25); ax.legend(loc='upper right', fontsize=8)
            if j == 1:
                ax.set_title('Independent Vy: NOT IMPLEMENTED; shown signals share inputs', fontsize=10)
    axes[0, 0].set_title('Complete RUNNING window'); axes[0, 1].set_title('First 10 s of RUNNING (fixed selection)')
    for ax in axes[-1]:
        ax.set_xlabel('IMU/source time relative to bag origin (s)')
    fig.suptitle('Archived rear-axle outputs: signal comparison, no independent truth; no CG conversion', fontsize=13)
    fig.savefig(args.output/'five-output-overview.png', dpi=150)
    plt.close(fig)
    from scipy.signal import welch
    fig, axes = plt.subplots(5, 1, figsize=(11, 14), constrained_layout=True)
    psds = {}
    grid = np.arange(int(np.ceil(RUN[0]*200)), int(np.floor(RUN[1]*200))+1)/200
    for j, (ax, (label, series)) in enumerate(zip(axes, rows)):
        full = va if j == 0 else (pa if j in (2, 3) else (ya if j == 4 else None))
        invalid_t = np.array([]) if full is None else (full['stamp_ns'][full['valid'] == 0]-ORIGIN)*1e-9
        mask = np.ones(len(grid), dtype=bool)
        interpolated = []
        for t, values, name, color, style in series:
            gap = .25 if 'LIO' in name else .03
            idx = np.clip(np.searchsorted(t, grid, side='right')-1, 0, len(t)-2)
            mask &= (grid >= t[0]) & (grid <= t[-1]) & (t[idx+1]-t[idx] <= gap)
            if name not in ('Wheel comparison',) and len(invalid_t):
                bad = np.searchsorted(t, invalid_t, side='right')-1
                bad = bad[(bad >= 0) & (bad < len(t)-1)]
                mask &= ~np.isin(idx, bad)
            interpolated.append(np.interp(grid, t, values))
        indices = np.flatnonzero(mask)
        segments = np.split(indices, np.flatnonzero(np.diff(indices)>1)+1)
        parts = [s for s in segments if len(s) >= 1024]
        assert parts, label
        n_windows = [1+(len(s)-1024)//512 for s in parts]
        psds[label] = dict(grid_hz=200, common_supported_samples=int(mask.sum()),
                           samples_in_eligible_segments=sum(len(s) for s in parts),
                           eligible_segments=len(parts), welch_windows=sum(n_windows),
                           nperseg=1024, overlap=512, detrend='constant',
                           method='linear interpolation on common support; gaps and known invalid brackets split; window-count-weighted PSD',
                           series={})
        for values, (_, _, name, color, style) in zip(interpolated, series):
            spectra = []
            for s in parts:
                freq, power = welch(values[s], fs=200, window='hann', nperseg=1024,
                                    noverlap=512, detrend='constant', scaling='density')
                spectra.append(power)
            power = np.average(spectra, axis=0, weights=n_windows)
            limit = 5 if label.startswith('Vy') else (50 if 'Wheel' in name else 100)
            m = (freq > 0) & (freq <= limit)
            ax.semilogy(freq[m], power[m], color=color, ls=style, lw=1.1, label=name)
            psds[label]['series'][name] = dict(frequency_hz=freq.tolist(), power_density=power.tolist(),
                                              display_max_hz=limit)
        ax.set_ylabel(label+' PSD'); ax.set_xlabel('Hz'); ax.grid(alpha=.25); ax.legend(fontsize=9)
        ax.set_xlim(0, 5 if label.startswith('Vy') else 100)
        if label.startswith('Vy'):
            ax.set_title('Vy: existing EKF/LIO comparisons; interpolation does not add measurement bandwidth')
    fig.suptitle('Output PSD on common RUNNING support: motion + vibration + noise, not error PSD', fontsize=13)
    fig.savefig(args.output/'output-psd.png', dpi=150)
    plt.close(fig)
    (args.output/'output-psd.json').write_text(json.dumps(psds, indent=2)+'\n')
    out['output_psd_support'] = {k: {a: b for a, b in v.items() if a != 'series'} for k, v in psds.items()}
    out['output_artifacts_sha256'] = {p.name: sha(p) for p in args.output.iterdir() if p.name != 'summary.json'}
    (args.output/'summary.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output), static=out['static'], counts=out['counts'],
                          algebra_checks=out['algebra_checks'], relative=relative), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
