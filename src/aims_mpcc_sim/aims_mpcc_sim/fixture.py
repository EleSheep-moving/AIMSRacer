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


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--vehicle-config', required=True)
    parser.add_argument('--radius', type=float, default=2.0)
    parser.add_argument('--speed', type=float, default=1.2)
    parser.add_argument('--samples', type=int, default=500)
    parser.add_argument('--left-width', type=float, default=0.9)
    parser.add_argument('--right-width', type=float, default=0.9)
    parsed = parser.parse_args(args)
    if parsed.radius <= 0.0 or parsed.samples < 4:
        raise ValueError('radius must be positive and samples must be at least 4')
    directory = Path(parsed.output).resolve()
    recording = directory / 'circle.csv'
    directory.mkdir(parents=True, exist_ok=False)
    with recording.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['timestamp', 'x', 'y', 'yaw', 'speed', 'frame_id', 'child_frame_id'])
        writer.writerows(circle_samples(parsed.radius, parsed.speed, parsed.samples))
    from aims_mpcc.io import load_config
    from aims_mpcc.path import prepare_recording
    prepared = prepare_recording(
        recording, directory / 'reference', load_config(parsed.vehicle_config),
        parsed.left_width, parsed.right_width,
    )
    print(f'Prepared {prepared.length:.3f} m numerical circle in {directory / "reference"}')
