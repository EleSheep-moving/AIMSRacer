import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def test_ros_launch_repeated_interrupt_flushes_and_exits_cleanly(tmp_path):
    src=Path(__file__).resolve().parents[3]
    launcher=tmp_path/'shutdown_entry.py'
    # Inject a slow cleanup to deterministically deliver the second SIGINT during drain.
    launcher.write_text("""import time
from aims_calibration.node import CalibrationNode, main
original=CalibrationNode.destroy_node
def drain(self):
    print('DRAIN',flush=True)
    time.sleep(.15)
    return original(self)
CalibrationNode.destroy_node=drain
main()
""")
    command=[sys.executable,str(launcher),'--ros-args','-p','steering_source:=rc',
             '-p',f'vehicle_config:={src}/controller/config/vehicle.yaml','-p',f'log_directory:={tmp_path}/evidence']
    child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                           env=dict(os.environ,ROS_DOMAIN_ID='194',ROS_LOCALHOST_ONLY='1'))
    try:
        deadline=time.monotonic()+5.
        output=[]
        while time.monotonic()<deadline:
            line=child.stdout.readline(); output.append(line)
            if 'Calibration disabled by default' in line: break
            assert child.poll() is None, ''.join(output)
        time.sleep(.05)
        child.send_signal(signal.SIGINT)
        while True:
            line=child.stdout.readline(); output.append(line)
            if 'DRAIN' in line: break
            assert line, ''.join(output)
        child.send_signal(signal.SIGINT)
        rest,_=child.communicate(timeout=5.); output.append(rest)
        assert child.returncode==0, ''.join(output)
        assert 'KeyboardInterrupt' not in ''.join(output)
        assert list((tmp_path/'evidence').glob('*/trials.json'))
    finally:
        if child.poll() is None: child.kill(); child.wait()
