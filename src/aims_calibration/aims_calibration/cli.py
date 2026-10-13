"""Four operator entry points; no vehicle launch or direct motor publisher."""
import argparse
import json
from pathlib import Path
import time

from .operator import check_start, summarize_results


def parser(name,description):
    value=argparse.ArgumentParser(prog=name,description=description)
    value.add_argument('--timeout',type=float,default=5.,help='Discovery/service timeout in seconds')
    return value


class Monitor:
    def __init__(self,ros_args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from std_srvs.srv import SetBool
        self.rclpy=rclpy;self.SetBool=SetBool
        # Keep DDS alive while Python handles Ctrl+C and sends the disable request.
        rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('calibration_operator')
        self.values={};self.status_received=0.;self.wheel=None;self.wheel_received=0.;self.wheel_stamp=None
        self.subscription=self.node.create_subscription(DiagnosticArray,'/calib/status',self._status,10)
        self.wheel_subscription=self.node.create_subscription(Odometry,'/rear_axle/wheel_odom',self._wheel,qos_profile_sensor_data)
        self.client=self.node.create_client(SetBool,'/calib/enable')

    def _status(self,message):
        for status in message.status:
            if status.name=='longitudinal_calibration':
                self.values={v.key:v.value for v in status.values};self.status_received=time.monotonic()

    def _wheel(self,message):
        if message.child_frame_id=='base_link':
            self.wheel=message.twist.twist.linear.x
            self.wheel_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.wheel_received=time.monotonic()

    def spin(self): self.rclpy.spin_once(self.node,timeout_sec=.05)

    def wait_status(self,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.spin()
            if self.values and time.monotonic()-self.status_received<=.4: return self.values
        raise RuntimeError('No fresh /calib/status; launch calibration first')

    def request(self,enabled,timeout):
        if not self.client.wait_for_service(timeout_sec=timeout): raise RuntimeError('/calib/enable unavailable')
        request=self.SetBool.Request();request.data=enabled
        future=self.client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node,future,timeout_sec=timeout)
        if not future.done(): raise RuntimeError('Enable response timed out; outcome is uncertain')
        response=future.result()
        if response is None or not response.success: raise RuntimeError(response.message if response else 'No service response')
        print(response.message,flush=True)

    def stationary(self):
        now=self.node.get_clock().now().nanoseconds*1e-9
        return self.wheel_stamp is not None and time.monotonic()-self.wheel_received<=.2 and 0<=now-self.wheel_stamp<=.2 and self.wheel is not None and abs(self.wheel)<=.05

    def __enter__(self): return self
    def __exit__(self,*args):
        self.node.destroy_node()
        if self.rclpy.ok(): self.rclpy.shutdown()


def inspect_main(args=None):
    options,ros_args=parser('inspect','Inspect the running calibration; never enable it').parse_known_args(args)
    try:
        with Monitor(ros_args) as monitor:
            values=monitor.wait_status(options.timeout)
            snapshot=json.loads(values.get('preflight_json','{}'))
            print(json.dumps(dict(status=values,preflight=snapshot,start_refusals=check_start(snapshot)),indent=2))
            return 0
    except (RuntimeError,ValueError,OSError) as error:
        print(str(error));return 2


def stop_main(args=None):
    options,ros_args=parser('stop','Disable calibration; keep vehicle launch and recorder running').parse_known_args(args)
    try:
        with Monitor(ros_args) as monitor:
            monitor.request(False,options.timeout)
            print('Stop requested. Confirm physical standstill; zero current alone does not guarantee it.')
        return 0
    except RuntimeError as error:
        print(str(error));return 3


def start_main(args=None):
    value=parser('start','Enable one acquisition in the existing launch and monitor it')
    value.add_argument('--max-seconds',type=float,default=900.,help='Monitor timeout; requests disable on expiry')
    options,ros_args=value.parse_known_args(args)
    if options.timeout<=0 or options.max_seconds<=0: value.error('timeouts must be positive')
    with Monitor(ros_args) as monitor:
        attempted=False
        try:
            values=monitor.wait_status(options.timeout)
            snapshot=json.loads(values.get('preflight_json','{}'))
            reasons=check_start(snapshot)
            if reasons:
                print(json.dumps(dict(started=False,reasons=reasons),indent=2));return 2
            # No retry: if the response is lost, request disable rather than a second enable.
            attempted=True;monitor.request(True,options.timeout)
            began=next_report=time.monotonic();terminal=None
            while time.monotonic()-began<options.max_seconds:
                monitor.spin();now=time.monotonic();values=monitor.values
                if now-monitor.status_received>1.: raise RuntimeError('Calibration monitor input lost')
                if now>=next_report:
                    keys=('state','target_speed','trials','planned_trials','sampled_seconds',
                          'measurement_seconds','segment_count','velocity','reason','output_directory')
                    print(json.dumps({key:values.get(key,'') for key in keys}),flush=True);next_report=now+2.
                if int(values.get('enable_count','0'))>=1 and values.get('state') in ('COMPLETE','FAULT','STOPPING'):
                    terminal=values['state'];break
            if terminal is None: raise RuntimeError('Acquisition monitor timeout')
            if terminal!='COMPLETE': monitor.request(False,options.timeout)
            deadline=time.monotonic()+20.;stationary_since=None
            while time.monotonic()<deadline:
                monitor.spin()
                if monitor.stationary():
                    if stationary_since is None: stationary_since=time.monotonic()
                    if time.monotonic()-stationary_since>=3.: break
                else: stationary_since=None
            stopped=stationary_since is not None and time.monotonic()-stationary_since>=3.
            print(json.dumps(dict(terminal=terminal,stationary_verified=stopped,
                output_directory=monitor.values.get('output_directory')),indent=2))
            if not stopped:
                print('Physical standstill unverified: operator stop required. Keep the vehicle launch running.')
                return 3
            if terminal!='COMPLETE': return 3
            path=Path(monitor.values['output_directory'])/'trials.json'
            # COMPLETE can precede the asynchronous disk checkpoint.
            deadline=time.monotonic()+options.timeout
            while time.monotonic()<deadline:
                result=summarize_results(json.loads(path.read_text()))
                if result['valid']: print(json.dumps(result,indent=2));return 0
                monitor.spin()
            print(json.dumps(result,indent=2));return 3
        except (RuntimeError,ValueError,OSError,KeyboardInterrupt) as error:
            print(str(error) or 'Operator interrupted',flush=True)
            if attempted:
                try: monitor.request(False,options.timeout)
                except Exception as stop_error: print('Stop request failed: '+str(stop_error)+'; lock the vehicle with RC',flush=True)
            return 3


def results_main(args=None):
    value=argparse.ArgumentParser(prog='results',description='Verify recorded trials; no ROS connection required')
    value.add_argument('path',type=Path,help='trials.json or its containing result directory')
    options=value.parse_args(args)
    path=options.path/'trials.json' if options.path.is_dir() else options.path
    try:
        result=summarize_results(json.loads(path.read_text()))
        print(json.dumps(result,indent=2,allow_nan=False));return 0 if result['valid'] else 3
    except (OSError,ValueError,TypeError) as error:
        print('Invalid result: '+str(error));return 3
