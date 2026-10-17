"""Operator entry points; no vehicle launch or direct motor publisher."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import io
import json
import math
from pathlib import Path
import signal
import time

from .operator import check_start, summarize_results


def parser(name,description):
    value=argparse.ArgumentParser(prog=name,description=description)
    value.add_argument('--timeout',type=float,default=5.,help='Discovery/service timeout in seconds')
    return value


def acquisition_timeout(stage,override):
    """Sweep coverage needs more time than the unchanged nine-speed hold."""
    value=override if override is not None else (900. if stage=='speed_hold' else 7200.)
    if not math.isfinite(value) or value<=0: raise ValueError('monitor timeout must be positive and finite')
    return value


def check_completion(monitor):
    values=monitor.values
    if time.monotonic()-monitor.status_received>1.:raise RuntimeError('Completion status lost')
    if values.get('state')!='COMPLETE':raise RuntimeError('Acquisition changed after completion: '+values.get('state','missing'))
    if values.get('logging_error') or int(values.get('logging_dropped','0')):
        raise RuntimeError('Logging failed after completion: '+values.get('logging_error','dropped evidence'))
    return (int(values.get('logging_committed_trials','-1'))==int(values.get('trials','-2')) and
            int(values.get('logging_committed_skips','0'))==int(values.get('skipped_trials','0')))


class Monitor:
    def __init__(self,ros_args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from std_srvs.srv import SetBool
        self.rclpy=rclpy;self.SetBool=SetBool
        self.interrupted=False
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

    def _interrupt(self,*args):self.interrupted=True

    def _check_interrupt(self):
        if self.interrupted:
            self.interrupted=False
            raise KeyboardInterrupt

    def spin(self):
        self._check_interrupt()
        self.rclpy.spin_once(self.node,timeout_sec=.05)
        self._check_interrupt()

    def wait_status(self,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.spin()
            if self.values and time.monotonic()-self.status_received<=.4: return self.values
        raise RuntimeError('No fresh /calib/status; launch calibration first')

    def request(self,enabled,timeout):
        if enabled:self._check_interrupt()
        if not self.client.wait_for_service(timeout_sec=timeout): raise RuntimeError('/calib/enable unavailable')
        if enabled:self._check_interrupt()
        request=self.SetBool.Request();request.data=enabled
        future=self.client.call_async(request)
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            self.rclpy.spin_once(self.node,timeout_sec=.05)
            if enabled:self._check_interrupt()
        if not future.done(): raise RuntimeError('Enable response timed out; outcome is uncertain')
        response=future.result()
        if response is None or not response.success: raise RuntimeError(response.message if response else 'No service response')
        print(response.message,flush=True)

    def stationary(self):
        now=self.node.get_clock().now().nanoseconds*1e-9
        return self.wheel_stamp is not None and time.monotonic()-self.wheel_received<=.2 and 0<=now-self.wheel_stamp<=.2 and self.wheel is not None and abs(self.wheel)<=.05

    def __enter__(self):
        # Raising inside pybind/rclpy can corrupt callback dispatch. Defer the
        # Python exception until native spinning returns; keep disable usable.
        self.previous_sigint=signal.signal(signal.SIGINT,self._interrupt)
        return self
    def __exit__(self,*args):
        try:
            self.node.destroy_node()
            if self.rclpy.ok(): self.rclpy.shutdown()
        finally:signal.signal(signal.SIGINT,self.previous_sigint)


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
    value.add_argument('--max-seconds',type=float,default=None,help='Monitor timeout: 900 s for A, 7200 s for B/C; requests disable on expiry')
    options,ros_args=value.parse_known_args(args)
    if not math.isfinite(options.timeout) or options.timeout<=0 or (options.max_seconds is not None and
            (not math.isfinite(options.max_seconds) or options.max_seconds<=0)):
        value.error('timeouts must be positive and finite')
    with Monitor(ros_args) as monitor:
        attempted=False
        try:
            values=monitor.wait_status(options.timeout)
            max_seconds=acquisition_timeout(values.get('stage','speed_hold'),options.max_seconds)
            snapshot=json.loads(values.get('preflight_json','{}'))
            reasons=check_start(snapshot)
            if reasons:
                print(json.dumps(dict(started=False,reasons=reasons),indent=2));return 2
            # No retry: if the response is lost, request disable rather than a second enable.
            attempted=True;monitor.request(True,options.timeout)
            began=next_report=time.monotonic();terminal=None
            while time.monotonic()-began<max_seconds:
                monitor.spin();now=time.monotonic();values=monitor.values
                if now-monitor.status_received>1.: raise RuntimeError('Calibration monitor input lost')
                if now>=next_report:
                    keys=('state','stage','trial_kind','target_speed','target_end_speed','target_current',
                          'trials','planned_trials','skipped_trials','sampled_seconds',
                          'measurement_seconds','segment_count','velocity','reason','output_directory')
                    print(json.dumps({key:values.get(key,'') for key in keys}),flush=True);next_report=now+2.
                if int(values.get('enable_count','0'))>=1 and values.get('state') in ('COMPLETE','FAULT','STOPPING'):
                    terminal=values['state'];break
            if terminal is None: raise RuntimeError('Acquisition monitor timeout')
            if terminal!='COMPLETE': monitor.request(False,options.timeout)
            deadline=time.monotonic()+20.;stationary_since=None
            while time.monotonic()<deadline:
                monitor.spin()
                if terminal=='COMPLETE':check_completion(monitor)
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
                monitor.spin()
                committed=check_completion(monitor)
                result=summarize_results(json.loads(path.read_text()))
                if result['valid'] and committed:
                    check_completion(monitor)
                    print(json.dumps(result,indent=2));return 0
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


def baseline_voltage(baseline,payload,source):
    """Attach voltage from exactly the accepted A segments, leaving A intact."""
    path=source.parent/'observations.csv'
    if not path.is_file():return baseline
    raw=path.read_bytes()
    observations=list(csv.DictReader(io.StringIO(raw.decode())))
    trials={row['plan_index']:row for row in payload['trials']}
    for point in baseline['points']:
        trial=trials[point['source_plan_index']]
        segments=trial.get('segments',[])
        if not segments:continue  # Older v1 results may not have segment bounds.
        admitted=[]
        for segment in segments:
            selected=[row for row in observations if
                segment['start_stamp']<=float(row['wheel_stamp'])<=segment['end_stamp'] and
                segment['start_telemetry_stamp']<=float(row['telemetry_stamp'])<=segment['end_telemetry_stamp']]
            pairs={(row['wheel_stamp'],row['telemetry_stamp']) for row in selected}
            if len(selected)!=segment['samples'] or len(pairs)!=len(selected):
                raise ValueError('A voltage evidence does not cover admitted segment')
            admitted.extend(float(row['voltage_input']) for row in selected)
        if len(admitted)!=trial['samples'] or not all(math.isfinite(v) and v>0 for v in admitted):
            raise ValueError('A admitted voltage samples invalid')
        point.update(voltage_input=sum(admitted)/len(admitted),voltage_samples=len(admitted),
            voltage_min=min(admitted),voltage_max=max(admitted))
    baseline['source'].update(observations_path=str(path),observations_sha256=hashlib.sha256(raw).hexdigest(),
        voltage_method='mean of paired samples within admitted A segment bounds')
    from .operator import canonical_hash
    baseline['baseline_sha256']=canonical_hash({k:v for k,v in baseline.items() if k!='baseline_sha256'})
    return baseline


def baseline_main(args=None):
    value=argparse.ArgumentParser(prog='baseline',description='Export a validated A baseline; offline, never changes recorded source')
    value.add_argument('path',type=Path,help='Stage A trials.json or its result directory')
    value.add_argument('--output',required=True,type=Path,help='Derived baseline_current.json path')
    value.add_argument('--iq-polynomial',nargs=4,type=float,metavar=('C0','C1','C2','C3'),
        help='Optional planning Iq(v)=C0+C1*v+C2*v^2+C3*v^3, valid for 1-5 m/s; retains measured A points')
    options=value.parse_args(args)
    source=(options.path/'trials.json' if options.path.is_dir() else options.path).expanduser().resolve()
    output=options.output.expanduser().resolve()
    try:
        if output==source: raise ValueError('baseline output must not overwrite source trials.json')
        from .planner import export_baseline
        payload=json.loads(source.read_text())
        baseline=baseline_voltage(export_baseline(payload,source_path=str(source)),payload,source)
        if options.iq_polynomial is not None:
            from .planner import with_polynomial_baseline
            baseline=with_polynomial_baseline(baseline,options.iq_polynomial)
        output.parent.mkdir(parents=True,exist_ok=True)
        pending=output.with_name(output.name+'.tmp')
        pending.write_text(json.dumps(baseline,indent=2,allow_nan=False)+'\n')
        pending.replace(output)
        print(json.dumps(dict(output=str(output),points=len(baseline['points']),
                             baseline_sha256=baseline['baseline_sha256']),indent=2))
        return 0
    except (OSError,ValueError,TypeError,KeyError) as error:
        print('Baseline refused: '+str(error));return 2


def plan_main(args=None):
    value=argparse.ArgumentParser(prog='plan',description='Preview the B/C sweep; offline, never enables acquisition')
    value.add_argument('--stage',required=True,choices=('accel_interval','decel_current'))
    value.add_argument('--base-current-file',required=True,type=Path)
    value.add_argument('--speeds',nargs='+',type=float,help='Interval B boundaries or C start speeds')
    value.add_argument('--sweep-protocol',choices=('interval','full_range_step'),default='interval')
    value.add_argument('--start-speed',type=float,help='Full-range start: B 1 m/s, C 5 m/s')
    value.add_argument('--end-speed',type=float,help='Full-range end: B 5 m/s, C 1 m/s')
    value.add_argument('--response-limits-file',type=Path,help='Deprecated for full_range_step; accepted and ignored')
    value.add_argument('--current-step',type=float,default=5.)
    value.add_argument('--max-current',type=float,default=100.)
    value.add_argument('--repeats',type=int,default=1)
    options=value.parse_args(args)
    try:
        from .planner import load_baseline,build_sweep_plan,build_full_step_plan
        baseline=load_baseline(options.base_current_file)
        speeds=options.speeds or [1.+.5*i for i in range(0 if options.stage=='accel_interval' else 1,9)]
        if options.speeds is None and options.stage=='decel_current': speeds.reverse()
        if options.sweep_protocol=='full_range_step':
            if options.speeds is not None:raise ValueError('--speeds is for interval sweeps; use --start-speed/--end-speed')
            plan=build_full_step_plan(options.stage,baseline,start_speed=options.start_speed,end_speed=options.end_speed,
                current_step=options.current_step,max_current=options.max_current,repeats=options.repeats)
        else:
            if options.start_speed is not None or options.end_speed is not None:
                raise ValueError('--start-speed/--end-speed require full_range_step')
            plan=build_sweep_plan(options.stage,speeds,baseline,current_step=options.current_step,
                max_current=options.max_current,repeats=options.repeats)
        report=dict(current_signal='avg_iq',stage=options.stage,sweep_protocol=options.sweep_protocol,planned=len(plan),
            baseline_sha256=baseline['baseline_sha256'],plan=[dict(plan_index=i,**asdict(trial)) for i,trial in enumerate(plan)])
        if options.sweep_protocol=='full_range_step':
            report['acquisition_policy']=full_step_acquisition_policy(options.response_limits_file)
        elif options.response_limits_file is not None:
            raise ValueError('--response-limits-file preview requires full_range_step')
        print(json.dumps(report,indent=2,allow_nan=False))
        return 0
    except (OSError,ValueError,TypeError,KeyError) as error:
        print('Plan refused: '+str(error));return 2



def full_step_acquisition_policy(response_limits_file=None):
    """Describe measured endpoints without inferring a wheel-speed operating cap."""
    result=dict(endpoint_exit_policy='measured_wheel_speed',current_handoff_enabled=True,
        operating_wheel_speed_limit_mps=None,response_limits_used=False,predictive_exit=False)
    if response_limits_file is not None:
        result['ignored_response_limits_file']=str(response_limits_file)
        result['diagnostic']='response_limits_file is ignored by full_range_step; no file was read'
    return result
