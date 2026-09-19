"""Real pipe and process regression tests for worker terminal states."""
import multiprocessing as mp
import time
import pytest
from aims_mpcc.worker import AsyncSolver


def idle():
    time.sleep(60)


@pytest.fixture
def worker():
    context=mp.get_context('spawn')
    worker=AsyncSolver.__new__(AsyncSolver)
    worker.connection,peer=context.Pipe()
    worker.process=context.Process(target=idle)
    worker.process.start()
    worker.ready=True;worker.pending=None;worker.started=0.
    yield worker,peer
    worker.close();worker.process.join(timeout=2);peer.close()


def test_closed_worker_does_not_repeat_timeout(worker):
    w,_=worker;w.close()
    assert w.poll(100.) is None


def test_eof_is_terminal_and_emitted_once(worker):
    w,peer=worker;peer.close()
    assert w.poll(.1)['kind']=='error'
    assert not w.ready
    assert w.connection.closed
    assert w.poll(100.) is None
    assert not w.submit({'stamp':100.})


def test_dead_worker_without_eof_is_terminal(worker):
    w,_=worker;w.process.kill();w.process.join(timeout=2)
    assert w.poll(.1)['kind']=='error'
    assert not w.ready
    assert w.poll(100.) is None


def test_submit_broken_pipe_is_reported_by_poll(worker):
    w,peer=worker;peer.close()
    assert not w.submit({'stamp':.1})
    assert not w.ready
    assert w.poll(.1)['kind']=='error'
    assert w.poll(100.) is None


def test_child_error_closes_worker(worker):
    w,peer=worker;peer.send({'kind':'error','error':'native solver failed'})
    assert w.poll(.1)['kind']=='error'
    assert not w.ready
    assert w.poll(100.) is None
