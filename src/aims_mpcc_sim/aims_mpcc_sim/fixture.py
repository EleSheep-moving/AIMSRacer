"""Deterministic rear-axle reference recordings for numerical validation."""

import argparse
import csv
import math
from pathlib import Path


def circle_samples(radius: float, speed: float, samples: int):
    for index in range(samples + 1):
        angle = 2.0 * math.pi * index / samples
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        if abs(x) < 1e-12:
            x = 0.0
        if abs(y) < 1e-12:
            y = 0.0
        yield (
            index * 0.05,
            x,
            y,
            angle + math.pi / 2.0,
            speed,
            'odom',
            'base_link',
        )


def figure_eight_samples(radius: float, waist_ratio: float, speed: float, samples: int):
    """Return a simple closed double-lobe path with a continuous narrow waist.

    A literal self-crossing figure eight is ambiguous for online progress
    projection. This polar curve keeps the same two-lobe shape without a
    crossing, so its progress remains single-valued.
    """
    previous_yaw = None
    for index in range(samples + 1):
        angle = 2.0 * math.pi * index / samples
        radial = radius * (1.0 + waist_ratio * math.cos(2.0 * angle))
        radial_rate = -2.0 * radius * waist_ratio * math.sin(2.0 * angle)
        x = radial * math.cos(angle)
        y = radial * math.sin(angle)
        dx = radial_rate * math.cos(angle) - radial * math.sin(angle)
        dy = radial_rate * math.sin(angle) + radial * math.cos(angle)
        raw_yaw = math.atan2(dy, dx)
        yaw = raw_yaw if previous_yaw is None else previous_yaw + (raw_yaw - previous_yaw + math.pi) % (2.0 * math.pi) - math.pi
        previous_yaw = yaw
        yield (
            index * 0.05,
            x,
            y,
            yaw,
            speed,
            'odom',
            'base_link',
        )


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--vehicle-config', required=True)
    parser.add_argument('--radius', type=float, default=2.0)
    parser.add_argument('--shape', choices=('circle', 'figure_eight'), default='circle')
    parser.add_argument('--waist-ratio', type=float, default=0.5)
    parser.add_argument('--speed', type=float, default=1.2)
    parser.add_argument('--samples', type=int, default=500)
    parser.add_argument('--left-width', type=float, default=0.9)
    parser.add_argument('--right-width', type=float, default=0.9)
    parsed = parser.parse_args(args)
    if parsed.radius <= 0.0 or parsed.samples < 4:
        raise ValueError('radius must be positive and samples must be at least 4')
    if not 0.0 <= parsed.waist_ratio < 1.0:
        raise ValueError('waist_ratio must be in [0, 1)')
    directory = Path(parsed.output).resolve()
    recording = directory / f'{parsed.shape}.csv'
    directory.mkdir(parents=True, exist_ok=False)
    generator = circle_samples if parsed.shape == 'circle' else figure_eight_samples
    arguments = (parsed.radius, parsed.speed, parsed.samples) if parsed.shape == 'circle' else (
        parsed.radius, parsed.waist_ratio, parsed.speed, parsed.samples,
    )
    with recording.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['timestamp', 'x', 'y', 'yaw', 'speed', 'frame_id', 'child_frame_id'])
        writer.writerows(generator(*arguments))
    from aims_mpcc.io import load_config
    from aims_mpcc.path import prepare_recording
    prepared = prepare_recording(
        recording, directory / 'reference', load_config(parsed.vehicle_config),
        parsed.left_width, parsed.right_width,
    )
    print(f'Prepared {prepared.length:.3f} m numerical {parsed.shape} in {directory / "reference"}')
