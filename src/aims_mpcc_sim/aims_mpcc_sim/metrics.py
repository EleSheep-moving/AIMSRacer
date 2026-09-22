"""Small, deterministic summaries for numerical closed-loop reports."""

import math


def summarize_tracking(samples, finish):
    errors = [row[1] for row in samples]
    return {
        'cross_track_rms_m': math.sqrt(sum(error * error for error in errors) / len(errors)),
        'cross_track_max_m': max(abs(error) for error in errors),
        'finish_error_m': math.hypot(*finish),
    }


def meets_acceptance_thresholds(summary, controller):
    return (
        summary['cross_track_rms_m'] <= 0.10
        and summary['cross_track_max_m'] <= 0.25
        and summary['finish_error_m'] <= 0.20
        and controller.get('deadline_misses') == 0
    )
