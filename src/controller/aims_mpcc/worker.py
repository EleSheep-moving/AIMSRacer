"""One in-flight solver process with a parent-enforced wall-clock deadline."""
import multiprocessing as mp
import time
import traceback
from pathlib import Path


def _run(connection, path_directory, config_dict):
    path_directory=str(Path(path_directory).resolve())
    from .native import build_context
    try:
        with build_context():
            _serve(connection,path_directory,config_dict)
    except BaseException:
        try:
            connection.send(dict(kind='error', error=traceback.format_exc()))
        finally:
            connection.close()


def _serve(connection, path_directory, config_dict):
    try:
        from .config import VehicleConfig
        from .path import ReferencePath
        from .solver import MPCCSolver
        from .native import solver_options
        path = ReferencePath.load(path_directory)
        config = VehicleConfig(**config_dict)
        solver = MPCCSolver(path, config, jit_enabled=True, native_options=solver_options())
        p = path.at(0.)
        # Compile/load native solver before exposing READY to the operator.
        state = dict(x=p['x'], y=p['y'], yaw=p['yaw'], speed=0., steering=0.)
        try:
            warm = solver.solve(state, dict(acceleration=0., steering=0., steering_rate=0.), [0.]*(solver.n+1))
        except RuntimeError as exc:
            raise RuntimeError('MPCC native cache unavailable. Run ros2 run aims_mpcc '
                               'prepare_solver <path_directory> --vehicle-config <vehicle.yaml> '
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
            result = solver.solve(request['state'], request['previous'], request['speed_refs'], request['elapsed'])
            result.update(kind='result', generation=generation, stamp=request['stamp'],
                          previous_steering=request['previous']['steering'])
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
    def __init__(self, directory, config):
        from dataclasses import asdict
        context = mp.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_run, args=(child, str(directory), asdict(config)), daemon=True)
        self.process.start(); child.close()
        self.ready = False
        self.pending = None
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
        return True

    def _terminal_error(self, message):
        self.close()
        return dict(kind='error', error=message)

    def poll(self, now):
        pending_error = getattr(self, '_pending_error', None)
        if pending_error is not None:
            self._pending_error = None
            return pending_error
        # Closed workers are terminal: do not repeatedly emit timeout errors.
        if self.connection.closed:
            return None
        # Enforce deadline even if an overdue result has already reached the pipe.
        if self.pending is not None and now-self.pending > .15:
            return self._terminal_error('Solver exceeded 150 ms deadline')
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
                    self.pending = None
                return result
        except (EOFError, OSError):
            return self._terminal_error('Solver process exited')
        if not self.process.is_alive():
            return self._terminal_error('Solver process exited')
        return None

    def close(self):
        self.ready=False; self.pending=None
        if self.process.is_alive():
            self.process.kill()
        self.process.join(timeout=0.)
        if not self.connection.closed:
            self.connection.close()
