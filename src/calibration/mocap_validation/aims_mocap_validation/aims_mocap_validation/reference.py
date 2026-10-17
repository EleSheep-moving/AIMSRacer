"""Offline reference only: centered derivatives use future motion-capture samples."""
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

QUANTITIES = ('vx', 'vy', 'ax', 'ay', 'yaw_rate')


def map_time(source_ns, clock):
    required = ('source_anchor_ns', 'ros_anchor_ns', 'scale', 'uncertainty_ns')
    if any(clock.get(k) is None for k in required):
        raise ValueError('clock mapping needs explicit source/ROS anchors, scale and uncertainty')
    scale, uncertainty = float(clock['scale']), float(clock['uncertainty_ns'])
    if not np.isfinite(scale) or scale <= 0 or not np.isfinite(uncertainty) or uncertainty < 0:
        raise ValueError('invalid clock scale or uncertainty')
    ns = np.asarray(source_ns, dtype=np.int64)
    relative = (ns-int(clock['source_anchor_ns']))*scale
    mapped = int(clock['ros_anchor_ns'])+np.rint(relative).astype(np.int64)
    if len(mapped) > 1 and np.any(np.diff(mapped) <= 0):
        raise ValueError('clock source is duplicate/backward: split epochs or repair the clock mapping explicitly')
    return mapped


def vector(value, n, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError(label+' needs explicit finite values')
    if n == 4 and abs(np.linalg.norm(a)-1) > 1e-3:
        raise ValueError(label+' must be a unit xyzw quaternion')
    return a


def build_reference(poses, config, window_s):
    ns = map_time(poses['source_ns'], config['clock'])
    if len(ns) < 7:
        raise ValueError('too few Vicon poses')
    unit = config['vicon'].get('position_unit')
    if unit not in ('m', 'mm'):
        raise ValueError('Vicon position_unit must explicitly be m or mm')
    p = np.asarray(poses['position'], float)*(1. if unit == 'm' else .001)
    quat = np.asarray(poses['quaternion'], float)
    if p.shape != (len(ns), 3) or quat.shape != (len(ns), 4):
        raise ValueError('invalid pose array shape')
    good = np.asarray(poses['valid'], bool)&np.isfinite(p).all(axis=1)&np.isfinite(quat).all(axis=1)
    good &= abs(np.linalg.norm(quat, axis=1)-1) <= 1e-3
    mount = Rotation.from_quat(vector(config.get('body_orientation_in_marker_xyzw'), 4, 'body orientation'))
    points = {name: vector(item.get('origin_in_marker_m'), 3, name+' origin')
              for name, item in config['references'].items()}
    if set(points) != {'rear_axle', 'center_of_mass'}:
        raise ValueError('both rear_axle and center_of_mass origins are required')
    d = config['differentiation']; order = int(d['polyorder'])
    gap = float(d['max_gap_s']); rotation_limit = float(d['max_rotation_span_rad'])
    if order < 2 or not np.isfinite(window_s) or window_s <= 0 or not np.isfinite(gap) or gap <= 0 or not 0 < rotation_limit < np.pi:
        raise ValueError('invalid differentiation parameters')
    breaks = np.r_[True, np.diff(ns)*1e-9 > gap]
    segments, current = [], []
    for i, ok in enumerate(good):
        if not ok or breaks[i]:
            if current: segments.append(np.asarray(current)); current = []
        if ok: current.append(i)
    if current: segments.append(np.asarray(current))
    out = {name: [] for name in points}
    for ids in segments:
        if len(ids) < order+3: continue
        t = (ns[ids]-ns[ids[0]])*1e-9
        dt = float(np.median(np.diff(t)))
        n = max(order+2, int(round(window_s/dt))); n += 1-n % 2
        if n > len(ids): continue
        u = np.arange(int(np.floor(t[-1]/dt+1e-6))+1)*dt
        u = u[u <= t[-1]+1e-12]; u = np.minimum(u, t[-1])
        if len(u) < n: continue
        marker = Rotation.from_quat(quat[ids])
        body = Slerp(t, marker*mount)(u)
        valid = np.ones(len(u), bool); half = n//2
        valid[:half] = False; valid[-half:] = False
        local = (np.arange(n)-half)*dt
        weights = np.linalg.pinv(np.vander(local, order+1, increasing=True))[1]
        angular = np.full((len(u), 3), np.nan)
        for i in range(half, len(u)-half):
            # Log(q_center^-1 * q_neighbor) is in the center body basis.
            rotation_vectors = (body[i].inv()*body[i-half:i+half+1]).as_rotvec()
            if np.max(np.linalg.norm(rotation_vectors, axis=1)) >= rotation_limit:
                valid[i] = False
            else: angular[i] = weights@rotation_vectors
        for name, lever in points.items():
            # Move the point in original pose space BEFORE differentiating.
            target = p[ids]+marker.apply(lever)
            position = np.column_stack([np.interp(u, t, target[:, j]) for j in range(3)])
            velocity = body.inv().apply(savgol_filter(position, n, order, deriv=1, delta=dt, axis=0))
            acceleration = body.inv().apply(savgol_filter(position, n, order, deriv=2, delta=dt, axis=0))
            values = np.column_stack([velocity[:, :2], acceleration[:, :2], angular[:, 2]])
            values[~valid] = np.nan
            out[name].append(dict(ns=ns[ids[0]]+np.rint(u*1e9).astype(np.int64),
                values=values, valid=valid.copy(), window_samples=n, dt_s=dt,
                effective_window_s=n*dt, sample_span_s=(n-1)*dt,
                segment_start_source_ns=int(ns[ids[0]])))
    if not any(out.values()):
        raise ValueError('no contiguous Vicon segment supports the derivative window')
    return out


def sample_reference(segments, ns):
    ns = np.asarray(ns, np.int64)
    out = np.full((len(ns), 5), np.nan)
    for segment in segments:
        # Split invalid interior windows as well as outside segment boundaries.
        ids = np.flatnonzero(segment['valid'])
        for part in np.split(ids, np.flatnonzero(np.diff(ids) != 1)+1):
            if len(part) < 2: continue
            source = segment['ns'][part]
            choose = (ns >= source[0])&(ns <= source[-1])
            x = (ns[choose]-source[0])*1e-9
            for j in range(5):
                out[choose, j] = np.interp(x, (source-source[0])*1e-9, segment['values'][part, j])
    return out


def error_statistics(error):
    error = np.asarray(error, float)
    if len(error) == 0: return dict(n=0)
    return dict(n=len(error), bias=float(np.mean(error)), rmse=float(np.sqrt(np.mean(error**2))),
                mae=float(np.mean(abs(error))), abs_p95=float(np.percentile(abs(error), 95)),
                abs_max=float(np.max(abs(error))))


def waveform_offset(ns, values, segments, column, comparison, clock_uncertainty_ns):
    step = float(comparison['lag_step_s']); limit = float(comparison['lag_limit_s'])
    if not np.isfinite(step) or not np.isfinite(limit) or step <= 0 or limit < step or limit > 2:
        raise ValueError('invalid waveform offset search')
    indices = np.arange(-int(round(limit/step)), int(round(limit/step))+1)
    lags_ns = np.rint(indices*step*1e9).astype(np.int64)
    targets = np.asarray([sample_reference(segments, ns-shift)[:, column] for shift in lags_ns])
    good = np.isfinite(targets).all(axis=0)&np.isfinite(values)
    threshold = float(comparison['min_dynamic_std'][QUANTITIES[column]])
    if not np.isfinite(threshold) or threshold < 0:raise ValueError('invalid minimum dynamic excitation')
    if good.sum() < 20 or min(np.std(values[good]), np.std(targets[len(indices)//2, good])) < threshold:
        return dict(identifiable=False, reason='insufficient_common_dynamic_support', n=int(good.sum()))
    scores = np.sqrt(np.mean((values[None, good]-targets[:, good])**2, axis=1))
    # Offset is a separate shape diagnostic: a DC bias can masquerade as a
    # large delay when minimizing raw RMSE over an incomplete maneuver.
    y=values[good]-np.mean(values[good])
    x=targets[:,good]-np.mean(targets[:,good],axis=1,keepdims=True)
    norm=np.linalg.norm(x,axis=1)*np.linalg.norm(y)
    correlation=np.divide(x@y,norm,out=np.full(len(x),-np.inf),where=norm>0)
    best = int(np.argmax(correlation)); shift = int(lags_ns[best])
    peaks=[i for i in range(1,len(indices)-1) if correlation[i]>correlation[i-1] and correlation[i]>=correlation[i+1]]
    ambiguous=any(abs(i-best)>2 and correlation[i]>=correlation[best]-1e-4 for i in peaks)
    identifiable=best not in (0,len(indices)-1) and correlation[best]>=.6 and not ambiguous
    return dict(identifiable=bool(identifiable), n=int(good.sum()),
                best_waveform_offset_ms=shift*1e-6, best_rmse=float(scores[best]),
                zero_offset_rmse_same_support=float(scores[len(indices)//2]),
                method='maximum demeaned normalized correlation; demeaning used for offset only',
                peak_correlation=float(correlation[best]),ambiguous_periodic_peak=bool(ambiguous),
                raw_rmse_minimum_offset_ms=float(lags_ns[int(np.argmin(scores))]*1e-6),
                boundary_optimum=best in (0, len(indices)-1),
                larger_than_clock_uncertainty=abs(shift) > float(clock_uncertainty_ns),
                interpretation='waveform offset, not isolated physical/filter latency')
