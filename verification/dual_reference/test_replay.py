"""Real negative test: a dead observer must never yield a passing replay receipt."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def descendants(pid):
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = dict(line.split(':',1) for line in (entry/'status').read_text().splitlines() if ':' in line)
            if int(fields['PPid']) == pid:
                child = int(entry.name); found.append(child); found.extend(descendants(child))
        except (OSError, KeyError, ValueError):
            continue
    return found


def test_replay_fails_when_observer_dies_after_valid_outputs(tmp_path):
    output = tmp_path/'replay'
    env = dict(os.environ, ROS_DOMAIN_ID='203', ROS_LOCALHOST_ONLY='1')
    log = (tmp_path/'replay.log').open('w')
    process = subprocess.Popen([sys.executable,str(Path(__file__).with_name('replay.py')),
        '--input',os.environ['DUAL_REPLAY_INPUT'],'--output',str(output),
        '--start-source-ns','1792002407401293846','--duration','3'],env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+15
        while True:
            assert process.poll() is None, (tmp_path/'replay.log').read_text()
            path = output/'outputs.csv'
            text = path.read_text() if path.exists() else ''
            if ',velocity,' in text and ',acceleration,' in text:
                break
            assert time.monotonic() < deadline
            time.sleep(.02)
        observers = []
        for pid in descendants(process.pid):
            try:
                executable = os.readlink(f'/proc/{pid}/exe')
            except OSError:
                continue
            if executable.endswith('/state_observer'):
                observers.append(pid)
        assert len(observers) == 1
        os.kill(observers[0],signal.SIGTERM)
        process.wait(timeout=10)
        assert process.returncode != 0, 'replay wrongly passed after the observer died'
        assert 'observer stopped during replay' in (tmp_path/'replay.log').read_text(), 'observer death must be detected during playback, not only by the final graph lookup'
        assert not (output/'summary.json').exists(), 'a failed replay must not create a passing receipt'
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT);process.wait(timeout=10)
        log.close()
