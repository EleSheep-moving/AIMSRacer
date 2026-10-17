#!/usr/bin/env python3
"""Inspect accepted MPCC native traces; no truth, tuning, or estimator changes."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

G0 = 9.80665


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def measure(rows):
    if len(rows) == 0:
        return {"n": 0}
    return dict(n=len(rows), ay_abs_p95_g=float(np.percentile(abs(rows['ay']) / G0, 95)),
                ay_abs_max_g=float(max(abs(rows['ay'])) / G0),
                tangent_abs_p95_si=float(np.percentile(abs(rows['tangential_y']), 95)),
                alpha_support_min_ms=float(min(rows['alpha_window_span_s']) * 1e3))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--quality', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    original = json.loads((a.quality / 'summary.json').read_text())
    meta = next(r for r in original['runs'] if r['name'] == 'mpcc')
    trace = a.quality / 'mpcc-trace.csv'
    assert sha(trace) == meta['trace_sha256']
    all_rows = np.genfromtxt(trace, names=True, delimiter=',', dtype=None, encoding='utf8')
    imu = all_rows[(all_rows['event'] == 'IMU') & (all_rows['accepted'] == 1)]
    time = (imu['stamp_ns'] - meta['origin_ns']) * 1e-9
    lo, hi = meta['windows']['running']['window_s']
    eligible = (time >= lo) & (time <= hi)
    temporal = imu[eligible]
    rows = imu[eligible & (imu['valid'] == 1) & (imu['lateral_valid'] == 1)]
    t = (rows['stamp_ns'] - meta['origin_ns']) * 1e-9
    assert len(rows) == meta['windows']['running']['n']
    assert np.all(np.diff(temporal['stamp_ns']) > 0)
    assert np.all(np.isfinite(rows['ay']))

    r = abs(rows['omega_z'])
    alpha = abs(rows['alpha_z'])
    masks = {'near_straight': (r < .15) & (alpha < 2),
             'steady_turn': (r >= .5) & (alpha < 2),
             'rapid_yaw_change': alpha >= 8}
    masks['other'] = ~np.logical_or.reduce(list(masks.values()))
    assert np.all(sum(m.astype(int) for m in masks.values()) == 1)
    peak = int(np.argmax(abs(rows['ay'])))
    # Pick the strongest alpha with a well-supported existing regression, without
    # excluding shorter-support samples from any overview or distribution.
    supported = np.flatnonzero(rows['alpha_window_span_s'] >= .020)
    assert len(supported)
    rapid = int(supported[np.argmax(alpha[supported])])
    selected = {}
    for name, index in [('strong_yaw_change', rapid), ('peak_ay', peak)]:
        mask = abs(t - t[index]) <= .5
        fields = ['stamp_ns', 'ay', 'raw_ay', 'force_y', 'gravity_y', 'tangential_y',
                  'centripetal_y', 'omega_z', 'alpha_x', 'alpha_z', 'alpha_window_span_s']
        selected[name] = dict(source_time_s=float(t[index]), window_s=[float(t[index]-.5), float(t[index]+.5)],
                              sample={k: rows[k][index].item() for k in fields}, **measure(rows[mask]))
    gaps = np.diff(temporal['stamp_ns']) * 1e-9
    invalid, counts = np.unique(temporal['status'][temporal['valid'] != 1], return_counts=True)
    summary = dict(original_bag=meta['original_bag'], trace_sha256=sha(trace),
                   input_sha256=meta['input_sha256'], script_sha256=sha(Path(__file__)),
                   origin_ns=meta['origin_ns'], running_window_s=[lo, hi],
                   accepted_imu_n=len(temporal), valid_imu_n=len(rows),
                   invalid_status_counts={str(k): int(v) for k, v in zip(invalid, counts)},
                   source_intervals=dict(min_ms=float(min(gaps)*1e3), max_ms=float(max(gaps)*1e3),
                                         below_1ms=int(sum(gaps < .001)), above_30ms=int(sum(gaps > .030))),
                   group_rules={'near_straight': '|r| < 0.15 rad/s and |alpha_z| < 2 rad/s^2',
                                'steady_turn': '|r| >= 0.5 rad/s and |alpha_z| < 2 rad/s^2',
                                'rapid_yaw_change': '|alpha_z| >= 8 rad/s^2',
                                'other': 'complement; groups partition valid samples, not sustained events'},
                   groups={name: measure(rows[m]) for name, m in masks.items()},
                   all_valid=measure(rows), selected_windows=selected,
                   scope='MPCC primary; B/C auxiliary only; group labels from existing gyro/alpha are heuristic, not truth',
                   independent_truth=False)
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family': font.get_name(), 'font.size': 11,
                         'axes.unicode_minus': False, 'axes.spines.top': False, 'axes.spines.right': False})
    colors = {'force_y': '#9AA7B2', 'raw_ay': '#C87519', 'ay': '#245F87'}
    labels = {'force_y': 'IMU 比力 y', 'raw_ay': '补偿后，未滤波', 'ay': '最终 ay'}
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.subplots_adjust(left=.085, right=.975, top=.88, bottom=.14, hspace=.30)
    for key in ['force_y', 'raw_ay', 'ay']:
        axes[0].plot(t, rows[key]/G0, color=colors[key], lw=.65 if key != 'ay' else .85,
                     alpha=.55 if key != 'ay' else 1, label=labels[key])
    axes[0].set_ylabel('横向 / g0'); axes[0].legend(ncol=3, loc='upper right', fontsize=10)
    axes[1].plot(t, rows['omega_z'], color='#245F87', lw=.8)
    axes[1].set_ylabel('车体 r / rad/s')
    axes[2].plot(t, rows['alpha_window_span_s']*1e3, color='#245F87', lw=.7, label='gyro 回归支持跨度')
    axes[2].plot(t, rows['alpha_sample_center_age_s']*1e3, color='#C87519', lw=.7, label='gyro 样本中心年龄')
    axes[2].set(ylabel='时间支持 / ms', xlabel='IMU 源时间，相对录包起点 / s')
    axes[2].legend(ncol=2, loc='lower right', fontsize=10)
    for ax in axes:
        ax.set_xlim(lo, hi); ax.grid(color='#E4E7EB', linewidth=.6)
    fig.suptitle('最新 MPCC：全运动段横向加速度与输入时间质量', x=.085, y=.97, ha='left', fontsize=18, weight='bold')
    fig.text(.085, .925, f'2026-10-15 022604 · {len(rows):,} 个有效 IMU 样本 · 后轴 base_link · 源时间输出', color='#444')
    fig.text(.085, .055, '显示全部有效样本，不人工移位或去均值；1 个计算无效样本单独计数。比力与惯性 ay 的物理定义不同。', color='#444', fontsize=10)
    fig.savefig(a.output / 'mpcc-ay-overview.png', dpi=180, facecolor='white'); plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(14, 8.4), sharex='col', sharey='row')
    fig.subplots_adjust(left=.085, right=.975, top=.83, bottom=.19, hspace=.35, wspace=.23)
    for column, (name, title) in enumerate([('strong_yaw_change', '受支持回归中，最大 yaw 变化率局部'), ('peak_ay', '全运动段 |ay| 峰值局部')]):
        center = selected[name]['source_time_s']; m = abs(t-center) <= .5
        for key in ['force_y', 'raw_ay', 'ay']:
            axes[0, column].plot(t[m], rows[key][m]/G0, color=colors[key], lw=1.1, label=labels[key])
        axes[0, column].set_title(f'{title}\n中心 {center:.3f} s', loc='left', fontsize=11, pad=12)
        for key, label, color, sign in [('gravity_y', '+ 重力 y', '#968140', 1),
                                        ('tangential_y', '− 切向 y', '#C87519', -1),
                                        ('centripetal_y', '− 向心 y', '#86718D', -1)]:
            axes[1, column].plot(t[m], sign*rows[key][m]/G0, color=color, lw=1.1, label=label)
        axes[1, column].set_xlabel('IMU 源时间，相对录包起点 / s')
        for ax in axes[:, column]:
            ax.set_xlim(center-.5, center+.5); ax.axhline(0, color='#777', ls='--', lw=.6)
            ax.grid(color='#E4E7EB', linewidth=.6)
    axes[0, 0].set_ylabel('横向 / g0'); axes[1, 0].set_ylabel('补偿贡献 / g0')
    axes[0, 0].legend(ncol=3, fontsize=9, loc='lower left')
    axes[1, 0].legend(ncol=3, fontsize=9, loc='upper right')
    fig.suptitle('MPCC 实包局部：快速 yaw 变化与横向峰值', x=.085, y=.97, ha='left', fontsize=18, weight='bold')
    fig.text(.085, .91, '两列相同行使用相同纵轴；未将峰值认定为真实车辆 G，也未把曲线差值作为精度误差。', color='#444', fontsize=10.5)
    fig.text(.085, .075, '窗口按明确数值规则自动选择；所有窗口外样本仍计入全运动段统计。没有用于拟合或更改观测器。', color='#444', fontsize=10)
    fig.savefig(a.output / 'mpcc-ay-details.png', dpi=180, facecolor='white'); plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
