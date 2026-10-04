"""One in-flight solver; late results are discarded without killing the worker."""
import multiprocessing as mp
import time
import traceback
from pathlib import Path


def _run(connection, path_directory, config_dict, horizon):
    path_directory=str(Path(path_directory).resolve())
    from .native import build_context
    try:
        with build_context():
            _serve(connection,path_directory,config_dict,horizon)
    except BaseException:
        try:
            connection.send(dict(kind='error', error=traceback.format_exc()))
        finally:
            connection.close()


def _serve(connection, path_directory, config_dict, horizon):
    try:
        from .config import VehicleConfig
        from .path import ReferencePath
        from .solver import MPCCSolver
        from .native import solver_options
        path = ReferencePath.load(path_directory)
        config = VehicleConfig(**config_dict)
        solver = MPCCSolver(path, config, horizon=horizon, jit_enabled=True, native_options=solver_options())
        p = path.at(0.)
        # Compile/load native solver before exposing READY to the operator.
        state = dict(x=p['x'], y=p['y'], yaw=p['yaw'], speed=0., steering=0.)
        try:
            warm = solver.solve(state, dict(acceleration=0., steering=0., steering_rate=0.),
                                [0.]*(solver.n+1), map_alignment=(0., 0., 0.)
                                if path.frame_id == 'map' else None)
        except RuntimeError as exc:
            raise RuntimeError('MPCC native cache unavailable. Run ros2 run aims_mpcc '
                               'prepare_solver <path_directory> --vehicle-config <vehicle.yaml> '
                               f'--horizon {horizon} '
                               'before starting the controller. Details: ' + str(exc)) from exc
        if not warm['success']:
            raise RuntimeError('Initial stationary solve failed: '+str(warm.get('status')))
        solver.reset()
        connection.send(dict(kind='ready'))
        generation = None
        while True:
            request = connection.recv()
            if request is None:
                return
            if generation != request['generation']:
                solver.reset(); generation = request['generation']
            worker_started_at=time.monotonic()
            result = solver.solve(request['state'], request['previous'], request['speed_refs'],
                                  request['elapsed'], request.get('map_alignment'))
            result.update(kind='result', generation=generation, stamp=request['stamp'],
                          previous_steering=request['previous']['steering'],
                          handover_command=request['handover_command'],
                          submitted_at=request['submitted_at'],source_stamp=request['source_stamp'],
                          worker_started_at=worker_started_at,worker_finished_at=time.monotonic())
            connection.send(result)
    except EOFError:
        pass
    except BaseException:
        try:
            connection.send(dict(kind='error', error=traceback.format_exc()))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class AsyncSolver:
    def __init__(self, directory, config, horizon=10, deadline=.25):
        from dataclasses import asdict
        if isinstance(horizon, bool) or not isinstance(horizon, int) or horizon < 1:
            raise ValueError('positive integer horizon required')
        import math
        if not math.isfinite(deadline) or deadline<=0:
            raise ValueError('Positive finite solver result budget required')
        self.deadline=deadline
        context = mp.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_run, args=(child, str(directory), asdict(config), horizon), daemon=True)
        self.process.start(); child.close()
        self.ready = False
        self.pending = None
        self.pending_skipped = False
        self.started = time.monotonic()
        self._pending_error = None

    def submit(self, request):
        if not self.ready or self.pending is not None:
            return False
        try:
            self.connection.send(request)
        except (BrokenPipeError, EOFError, OSError):
            self.close()
            self._pending_error = dict(kind='error', error='Solver connection failed during submission')
            return False
        self.pending = request.get('submitted_at', request['stamp'])
        self.pending_skipped = False
        return True

    def _terminal_error(self, message):
        self.close()
        return dict(kind='error', error=message)

    def poll(self, now):
        pending_error = getattr(self, '_pending_error', None)
        if pending_error is not None:
            self._pending_error = None
            return pending_error
        # Closed workers are terminal; actual process/pipe failure still faults.
        if self.connection.closed:
            return None
        if not self.ready and now-self.started > 180.:
            return self._terminal_error('Solver initialization deadline expired')
        try:
            if self.connection.poll():
                result = self.connection.recv()
                if result.get('kind') == 'error':
                    return self._terminal_error(result.get('error', 'Solver process failed'))
                if result.get('kind') == 'ready':
                    self.ready = True
                if result.get('kind') == 'result':
                    # Includes queueing and parent delivery, not just IPOPT CPU
                    # time. A late reply cannot activate, even if it converged.
                    result['discarded'] = (self.pending_skipped or
                        now > result['submitted_at'] + self.deadline)
                    result['skip_notified'] = self.pending_skipped
                    result['request_age_s'] = now-result['submitted_at']
                    self.pending = None
                    self.pending_skipped = False
                return result
        except (EOFError, OSError):
            return self._terminal_error('Solver process exited')
        if not self.process.is_alive():
            return self._terminal_error('Solver process exited')
        if (self.pending is not None and not self.pending_skipped
                and now > self.pending+self.deadline):
            # Notify once, but leave the solve in flight until its reply drains.
            # Serial solving prevents an unbounded queue of stale requests and
            # avoids expensive worker reinitialization on every overrun.
            self.pending_skipped = True
            return dict(kind='skipped',submitted_at=self.pending,
                        request_age_s=now-self.pending)
        return None

    def close(self):
        self.ready=False; self.pending=None
        self.pending_skipped=False
        if self.process.is_alive():
            self.process.kill()
        self.process.join(timeout=0.)
        if not self.connection.closed:
            self.connection.close()
