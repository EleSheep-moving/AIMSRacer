"""A stalled child loses its old pipe and cannot publish an obsolete reply."""
import multiprocessing as mp
import time

from aims_mpcc.config import VehicleConfig
from aims_mpcc.worker import AsyncSolver


def sleep_child():
    time.sleep(30.)


def test_stalled_process_is_replaced_after_bounded_grace(tmp_path):
    context=mp.get_context('spawn')
    parent, child=context.Pipe()
    process=context.Process(target=sleep_child)
    process.start()
    worker=AsyncSolver.__new__(AsyncSolver)
    worker.process,worker.connection=process,parent
    worker.ready=True
    worker.pending=time.monotonic()-1.1
    worker.pending_skipped=True
    worker.deadline=.1
    worker.started=time.monotonic()
    worker._pending_error=None
    worker.restart_count=0
    worker._spawn_args=(str(tmp_path),VehicleConfig().__dict__,10,None)
    try:
        reply=worker.poll(time.monotonic())
        assert reply['kind']=='restarting'
        assert worker.process.pid != process.pid
        assert parent.closed
        assert worker.pending is None
        assert not worker.ready
        process.join(timeout=2.)
        assert not process.is_alive()
    finally:
        worker.close()
        child.close()
