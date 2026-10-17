"""Deterministic native trace parity against the retained Python reference."""
import os
import subprocess

import numpy as np
import pytest

from aims_racer_system.longitudinal_observer import LongitudinalObserver, ObserverConfig


def _compare_trace(commands, config):
    executable = os.environ.get('NATIVE_OBSERVER_TRACE_EXECUTABLE')
    if not executable:
        pytest.skip('NATIVE_OBSERVER_TRACE_EXECUTABLE was not provided')
    observer = LongitudinalObserver(config)
    expected = []
    for command in commands:
        fields = command.split()
        if fields[0] == 'M':
            variance = float(fields[4]) if len(fields) > 4 else None
            admitted = observer.receive(fields[1], int(fields[2]), float(fields[3]), variance)
            expected.append(('M', admitted, observer.history_size))
        elif fields[0] == 'S':
            try:
                estimate = observer.snapshot(int(fields[1]))
            except (ValueError, RuntimeError, FloatingPointError, OverflowError):
                expected.append(('E',))
            else:
                expected.append(('S', estimate.stamp_ns, np.array([
                    estimate.velocity, estimate.acceleration, estimate.disturbance,
                    estimate.acceleration_variance])))
        elif fields[0] == 'C':
            expected.append(('C', observer.counters.copy()))
    process = subprocess.run([executable, str(config.history_seconds),
        str(config.max_history_measurements), str(config.jerk_psd),
        str(config.disturbance_psd), str(config.speed_variance), str(config.force_variance)],
        input='\n'.join(commands) + '\n', capture_output=True, text=True, check=True)
    lines = process.stdout.splitlines()
    assert len(lines) == len(expected), process.stdout
    maximum_error = 0.
    for index, (line, result) in enumerate(zip(lines, expected)):
        values = line.split()
        assert values[0] == result[0], (index, commands[index], line, result)
        if result[0] == 'M':
            assert (bool(int(values[1])), int(values[2])) == result[1:]
        elif result[0] == 'S':
            assert int(values[1]) == result[1]
            native = np.array([float(value) for value in values[2:]])
            np.testing.assert_allclose(native, result[2], rtol=3e-11, atol=3e-11,
                err_msg=f'command {index}: {commands[index]}')
            maximum_error = max(maximum_error, float(np.max(np.abs(native-result[2]))))
        elif result[0] == 'C':
            assert dict((key, int(value)) for key, value in
                (item.split('=') for item in values[1:])) == result[1]
    return maximum_error


@pytest.mark.parametrize('history,cap', [(2., 4096), (.03, 7), (100., 2)])
def test_randomized_sorted_delayed_equal_and_bounded_parity(history, cap):
    config = ObserverConfig(use_imu=True, history_seconds=history,
        max_history_measurements=cap, jerk_psd=30., disturbance_psd=.1,
        speed_variance=.01, force_variance=16.)
    rng = np.random.default_rng(20261015)
    epoch = 1_800_000_000_000_000_003
    newest = epoch
    commands = ['S 0', 'C']
    for index in range(600):
        newest += int(rng.integers(0, 10_000_000))
        # Equal stamps retain callback receipt order; old samples may reject.
        stamp = newest - int(rng.choice([0, 0, 1, 3, 50])) * 1_000_000
        value = float(rng.normal())
        kind = 'force' if index % 3 else 'speed'
        commands.append(f'M {kind} {stamp} {value:.17g}')
        commands.append(f'S {newest + 17}')
        if index % 19 == 0:
            commands.append('C')
    commands.extend([f'S {newest + 1_000_000_000_000}', 'C'])
    _compare_trace(commands, config)


def test_invalid_numeric_failure_and_replay_anchor_parity():
    config = ObserverConfig(use_imu=True, max_history_measurements=3)
    commands = [
        'M speed -1 0', 'M unknown 0 0', 'M gravity 0 1',
        'M force 0 nan', 'M speed 0 1 0', 'M speed 0 1 inf',
        'M speed 100 -1e308', 'S 100', 'M speed 150 1e308 1e-300',
        'S 100', 'C',
    ]
    _compare_trace(commands, config)
    _compare_trace(['M speed 100 1e308', 'M speed 50 -1e308', 'S 100', 'C'], config)
    epoch = 1_800_000_000_000_000_003
    commands = [f'M speed {epoch} 1', f'M force {epoch+30} .7',
        f'M speed {epoch+10} 1.1', f'M force {epoch+30} .9',
        f'M speed {epoch+20} 1.2', f'M speed {epoch-1} 5',
        f'M force {epoch+10} 1.3', f'S {epoch+40}', 'C',
        f'S {epoch+30_000_000_000}', f'S {epoch+40}', 'C']
    _compare_trace(commands, config)
