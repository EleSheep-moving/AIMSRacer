#!/usr/bin/env python3
"""MPCC ay versus our ax-integral times our yaw rate; wheel speed is comparison only."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

G0 = 9.80665


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def integrate(t, acceleration):
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('integration needs increasing source times')
    return np.r_[0., np.cumsum(np.diff(t)*(acceleration[:-1]+acceleration[1:])/2)]


def stats(x):
    return dict(n=len(x), mean=float(np.mean(x)), rms=float(np.sqrt(np.mean(x*x))),
                abs_p95=float(np.percentile(abs(x), 95)), abs_max=float(max(abs(x))))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planar-quality', type=Path, required=True)
    p.add_argument('--yaw-quality', type=Path, required=True)
    p.add_argument('--comparison-input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    bm = json.loads((a.planar_quality/'summary.json').read_text())
    bm = next(r for r in bm['runs'] if r['name'] == 'mpcc')
    ym = json.loads((a.yaw_quality/'summary.json').read_text())
    ym = next(r for r in ym['runs'] if r['label'] == 'mpcc')
    cm = json.loads((a.comparison_input/'metadata.json').read_text())
    body_path, yaw_path = a.planar_quality/'mpcc-trace.csv', a.yaw_quality/'mpcc-trace.csv'
    wheel_path = a.comparison_input/'yaw-input.npz'
    assert sha(body_path) == bm['trace_sha256']
    assert sha(yaw_path) == ym['trace_sha256']
    assert sha(wheel_path) == cm['npz_sha256']
    assert bm['input_sha256'] == ym['input_sha256']
    assert bm['original_bag'] == cm['source'] == ym['source']
    origin = bm['origin_ns']
    body = np.genfromtxt(body_path, names=True, delimiter=',', dtype=None, encoding='utf8')
    body = body[(body['event'] == 'IMU') & (body['accepted'] == 1) & (body['valid'] == 1)]
    time = (body['stamp_ns']-origin)*1e-9
    rest_lo, rest_hi = bm['windows']['static']['window_s']
    rest = (time >= rest_lo) & (time <= rest_hi)
    ax_offset = float(np.mean(body['ax'][rest]))
    # Initial rest is selected from the recorded pre-motion window, not inferred
    # from a later speed endpoint. No wheel reset, endpoint fit or gain fit.
    first = int(np.flatnonzero(time <= rest_hi)[-1])
    body, time = body[first:], time[first:]
    assert np.max(np.diff(time)) <= .030
    v_integral = integrate(time, body['ax'])
    v_static_corrected = integrate(time, body['ax']-ax_offset)
    yaw = np.genfromtxt(yaw_path, names=True, delimiter=',', dtype=None, encoding='utf8')
    indices = np.searchsorted(yaw['stamp_ns'], body['stamp_ns'])
    assert np.array_equal(yaw['stamp_ns'][indices], body['stamp_ns'])
    rate = yaw['rate'][indices]
    yaw_valid = (yaw['accepted'][indices] == 1) & (yaw['valid'][indices] == 1)
    with np.load(wheel_path) as z:
        wheel_source = z['wheel_ts'][:, 1]
        wheel_x = z['wheel'][:, 0]
    assert np.all(np.diff(wheel_source) > 0)
    wt = (wheel_source-origin)*1e-9
    wheel = np.interp(time, wt, wheel_x)
    right_unclipped = np.searchsorted(wheel_source, body['stamp_ns'], side='left')
    right = np.minimum(right_unclipped, len(wheel_source)-1)
    left = np.maximum(right-1, 0)
    exact = wheel_source[right] == body['stamp_ns']
    wheel_ok = ((body['stamp_ns'] >= wheel_source[0]) & (body['stamp_ns'] <= wheel_source[-1]) &
                (((wheel_source[right]-wheel_source[left]) <= 30_000_000) | exact))
    wheel[~wheel_ok] = np.nan
    lo, hi = bm['windows']['running']['window_s']
    running = (time >= lo) & (time <= hi) & yaw_valid
    common = running & wheel_ok
    assert np.any(common)
    signals = dict(imu_ay=body['ay'], ax_integral_times_rate=v_integral*rate,
                   static_corrected_integral_times_rate=v_static_corrected*rate,
                   wheel_times_rate=wheel*rate)
    comparison = {}
    for name, values in signals.items():
        error = values[common]-body['ay'][common]
        comparison[name] = dict(si=stats(values[common]), g=stats(values[common]/G0),
                                difference_from_imu_rmse_si=float(np.sqrt(np.mean(error*error))),
                                correlation_with_imu=float(np.corrcoef(values[common], body['ay'][common])[0, 1]),
                                fraction_abs_above_imu=float(np.mean(abs(values[common]) > abs(body['ay'][common]))))
    last = int(np.flatnonzero(common)[-1])
    time_windows = {}
    for name, (start, end) in [('first_10s', (lo, lo+10)), ('last_10s', (hi-10, hi))]:
        mask = common & (time >= start) & (time <= end)
        time_windows[name] = dict(window_s=[start, end], signals={k: stats(x[mask]/G0) for k, x in signals.items()})
    summary = dict(original_bag=bm['original_bag'], origin_ns=origin,
                   hashes={str(f): sha(f) for f in [body_path, yaw_path, wheel_path, Path(__file__)]},
                   initial_condition=dict(source_time_s=float(time[0]), vx_mps=0.,
                                          basis='last valid sample within recorded pre-motion stationary window'),
                   stationary_ax_mean_si=ax_offset, running_window_s=[lo, hi],
                   running_valid_n=int(sum(running)), common_n=int(sum(common)),
                   excluded_wheel_gap_n=int(sum(running & ~wheel_ok)),
                   integration_max_valid_source_interval_ms=float(np.max(np.diff(time))*1e3),
                   wheel_source_max_interval_ms=float(np.max(np.diff(wt))*1e3),
                   final_common_sample=dict(source_time_s=float(time[last]),
                                            integrated_vx_mps=float(v_integral[last]),
                                            static_corrected_integrated_vx_mps=float(v_static_corrected[last]),
                                            wheel_vx_mps=float(wheel[last])),
                   speed_common={k: stats(x[common]) for k, x in [('ax_integral', v_integral),
                                  ('static_corrected_ax_integral', v_static_corrected), ('wheel', wheel)]},
                   comparison=comparison, time_windows_g=time_windows,
                   assumptions=['planar a_x = dot(v_x) - r*v_y; integral neglects r*v_y',
                                'initial vx=0 at rest; no later reset or wheel correction',
                                'valid source gaps <=30ms integrated by endpoint trapezoids, including one omitted warmup sample',
                                'source stamp matching for new yaw observer; wheel interpolation <=30ms only, no phase fit',
                                'static-mean correction is a sensitivity case, not a changed estimator',
                                'same IMU drives ax, ay and yaw; their errors are correlated',
                                'wheel speed is not independent ground truth; wheel product is not ay truth'],
                   independent_truth=False)
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    np.savez_compressed(a.output/'aligned.npz', source_ns=body['stamp_ns'], source_relative_s=time,
                        ax=body['ax'], ay=body['ay'], new_rate=rate, integral_vx=v_integral,
                        static_corrected_integral_vx=v_static_corrected, wheel_vx=wheel,
                        running_valid=running, common=common)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'font.size': 11, 'axes.unicode_minus': False,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    fig.subplots_adjust(left=.085, right=.975, top=.87, bottom=.13, hspace=.33)
    display = (time >= lo) & (time <= hi)
    axes[0].plot(time[display], v_integral[display], color='#C87519', lw=1.2, label='ax 积分，静止 vx=0')
    axes[0].plot(time[display], v_static_corrected[display], color='#968140', lw=1, ls='--', label='扣静态 ax 均值后积分（敏感性）')
    axes[0].plot(time[display], wheel[display], color='#245F87', lw=1.2, label='轮速 vx（仅对照）')
    axes[0].set_ylabel('纵向速度 / (m/s)'); axes[0].legend(loc='upper left', ncol=3, fontsize=10)
    for axis, names in [(axes[1], ['imu_ay', 'ax_integral_times_rate']),
                        (axes[2], ['imu_ay', 'wheel_times_rate'])]:
        labels = {'imu_ay': 'IMU 重建 ay', 'ax_integral_times_rate': 'ax 积分速度 × 自研 r',
                  'wheel_times_rate': '轮速 vx × 自研 r（运动学对照）'}
        for name in names:
            values = np.where(running, signals[name]/G0, np.nan)
            axis.plot(time[display], values[display], color='#245F87' if name == 'imu_ay' else '#C87519',
                      lw=.75, alpha=.85, label=labels[name])
        axis.set_ylabel('横向 / g0'); axis.legend(loc='upper left', ncol=2, fontsize=10)
    for axis in axes:
        axis.axhline(0, color='#666', lw=.65, ls='--'); axis.grid(color='#E4E7EB', lw=.6); axis.set_xlim(lo, hi)
    axes[2].set_xlabel('IMU 源时间，相对录包起点 / s')
    fig.suptitle('MPCC：自研 ax／yaw rate 能否解释横向加速度？', x=.085, y=.97, ha='left', fontsize=18, weight='bold')
    fig.text(.085, .92, '同一后轴参考点、源时间对齐；积分使用全部既有有效 ax，不用轮速重置，不拟合曲线增益或时间偏移。', color='#444', fontsize=10.5)
    fig.text(.085, .055, '中、下图纵轴范围不同。速度积分忽略 r·vy，且会累计偏置与补偿残差；所有曲线均非独立动态真值。', color='#444', fontsize=10.5)
    fig.savefig(a.output/'ax-rate-ay-comparison.png', dpi=180, facecolor='white'); plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
