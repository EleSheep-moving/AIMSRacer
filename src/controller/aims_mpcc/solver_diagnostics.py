"""JSON-safe IPOPT residuals and named constraint diagnostics."""
import math

import numpy as np


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (bool, str)) or value is None:
        return value
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return str(value)


def convergence(stats):
    """IPOPT's primal/dual infeasibilities retain their native scaling."""
    history = stats.get('iterations', {})
    result = {}
    for source, target in (
        ('inf_pr', 'primal_inf'), ('inf_du', 'dual_inf'), ('mu', 'barrier_parameter'),
        ('obj', 'objective'), ('d_norm', 'step_norm'),
        ('alpha_pr', 'primal_step_size'), ('alpha_du', 'dual_step_size'),
        ('regularization_size', 'regularization_size'),
    ):
        values = history.get(source, [])
        result[target] = json_safe(values[-1]) if len(values) else None
    return result


def constraint_summary(values, lower, upper, blocks):
    values, lower, upper = (np.asarray(item, dtype=float).reshape(-1)
                            for item in (values, lower, upper))
    if values.shape != lower.shape or values.shape != upper.shape or not np.isfinite(values).all():
        raise ValueError('finite constraint values and matching bounds required')
    if np.isnan(lower).any() or np.isnan(upper).any() or np.any(lower > upper):
        raise ValueError('invalid constraint bounds')
    violations = np.maximum(np.maximum(lower-values, values-upper), 0.)
    groups, worst = {}, []
    for block in blocks:
        first, end = block['first'], block['end']
        local = violations[first:end]
        if not len(local):
            continue
        row = first+int(np.argmax(local))
        maximum = float(violations[row])
        groups[block['group']] = max(groups.get(block['group'], 0.), maximum)
        if maximum > 0.:
            worst.append(dict(block, row=row, component=row-first, violation=maximum,
                              value=float(values[row]), lower=float(lower[row]), upper=float(upper[row])))
    worst.sort(key=lambda item: item['violation'], reverse=True)
    return json_safe(dict(max_constraint_violation=float(violations.max(initial=0.)),
                          constraint_violations=groups, worst_constraints=worst[:5]))
