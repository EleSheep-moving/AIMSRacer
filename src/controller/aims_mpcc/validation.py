"""Independent candidate validation, shared by solver backends and handover."""
import math
import hashlib
import json
from dataclasses import asdict
import numpy as np

from .envelope import evaluate_envelope
from .frames import apply_alignment


def candidate_fingerprint(plan,config):
    payload={key:plan.get(key) for key in
             ('states','controls','validation_applied','map_alignment')}
    payload['dt']=plan.get('dt',.1)
    payload['config']=asdict(config)
    return hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_candidate(plan, config, path=None, tolerance=1e-4):
    try:
        states = np.asarray(plan['states'], float)
        controls = np.asarray(plan['controls'], float)
        applied = np.asarray(plan['validation_applied'], float)
        dt = float(plan.get('dt', .1))
        if (states.ndim != 2 or states.shape[1] != 6 or len(states) < 2 or
                controls.shape != (len(states)-1, 3) or applied.shape != (3,) or
                not all(np.isfinite(v).all() for v in (states, controls, applied))):
            raise ValueError('Invalid candidate shape or nonfinite values')
        diagnostic = evaluate_envelope(states[0], applied, controls, config, dt, tolerance=tolerance)
        samples = np.asarray(diagnostic.pop('candidate_samples'))
        diagnostic.pop('reference_samples')
        diagnostic.pop('reference_controls')
        independent = samples[::round(dt/.02)]
        residual = np.max(np.abs(states-independent), axis=0)
        dynamics_ok = bool(np.all(residual <= np.array([.005, .005, .001, tolerance, .005, .001])))
        steering_violation=float(np.maximum(np.abs(samples[:,5])-config.steer_limit,0.).max())
        hard_ok = (diagnostic['hard_control_violation'] <= tolerance and
                   diagnostic['speed_bound_violation'] <= tolerance and steering_violation<=tolerance)
        if config.envelope_soft_enabled:
            envelope_ok = (diagnostic['recovery_bounds_satisfied'] and
                           (not diagnostic['recovery_needed'] or diagnostic['recovery_acceptable']))
        else:
            envelope_ok = (diagnostic['initial_candidate_utilization'] <= 1.+tolerance and
                           diagnostic['future_slack_max'] <= tolerance)
        margin = math.inf
        if config.enforce_corridor:
            if path is None:
                raise ValueError('Corridor validation requires reference geometry')
            alignment = plan.get('map_alignment')
            if path.frame_id == 'map' and alignment is None:
                raise ValueError('Map corridor validation requires alignment')
            for row in samples:
                x,y,yaw = row[:3]
                if path.frame_id == 'map':
                    x,y,yaw = apply_alignment(x,y,yaw,alignment)
                reference = path.at(row[4])
                normal = np.array([-math.sin(reference['yaw']), math.cos(reference['yaw'])])
                for along in config.longitudinal_offsets():
                    for across in (-config.half_width, config.half_width):
                        corner = np.array([x + along*math.cos(yaw)-across*math.sin(yaw),
                                           y + along*math.sin(yaw)+across*math.cos(yaw)])
                        lateral = float(normal @ (corner-np.array([reference['x'], reference['y']])))
                        margin = min(margin,path.left_width-lateral,path.right_width+lateral)
        accepted = bool(dynamics_ok and hard_ok and envelope_ok and margin >= -tolerance)
        return dict(accepted=accepted, candidate_fingerprint=candidate_fingerprint(plan,config),
                    dynamics_max_residual=residual.tolist(),
                    actual_steering_bound_violation=steering_violation,
                    hard_ok=bool(hard_ok), envelope_ok=bool(envelope_ok),
                    minimum_margin_m=margin if math.isfinite(margin) else None,
                    envelope=diagnostic,
                    reason='' if accepted else 'Independent rollout, actuator, envelope or corridor check failed')
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return dict(accepted=False, reason=str(exc))
