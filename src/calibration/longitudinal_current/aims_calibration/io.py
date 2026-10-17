"""Native command serialization and inspectable calibration evidence."""
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import time
from pathlib import Path
import uuid
from queue import Queue, Full, Empty
from threading import Thread, Event


def motor_telemetry(message):
    try:
        values=tuple(float(getattr(message.state,name)) for name in ('current_motor','voltage_input','speed'))
    except (AttributeError,TypeError,ValueError) as exc:
        raise ValueError('required VESC motor telemetry unavailable') from exc
    if not all(math.isfinite(v) for v in values): raise ValueError('nonfinite VESC motor telemetry')
    return values


def q_axis_telemetry(message):
    """Read actual VESC q-axis feedback, never a motor-current approximation."""
    try: value=float(message.state.avg_iq)
    except (AttributeError,TypeError,ValueError) as exc:
        raise ValueError('required VESC avg_iq telemetry unavailable') from exc
    if not math.isfinite(value): raise ValueError('nonfinite VESC avg_iq telemetry')
    return value


def health_values(message,ros_now_ns):
    """Same DiagnosticArray envelope accepted by the standalone PP adapter."""
    stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
    records=[r for r in message.status if r.name=='aims_racer_system/localization']
    if stamp<=0 or len(records)!=1 or not 0<=(ros_now_ns-stamp)*1e-9<=.3: return {}
    values={}
    for item in records[0].values:
        if item.key in values: return {}
        values[item.key]=item.value
    return values


def apply_request(message,request,stamp):
    message.header.stamp=stamp; message.header.frame_id='base_link'
    message.control_mode=request.mode; message.steering_angle=float(request.steering)
    for name in ('target_speed_mps','target_current','target_duty','target_brake_current','target_acceleration_mps2'):
        setattr(message,name,0.)
    if request.mode==0: message.target_speed_mps=float(request.speed)
    elif request.mode==1: message.target_current=float(request.current)
    else: raise ValueError('calibration emits only speed/current modes')
    return message


def _json_safe(value):
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {k:_json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [_json_safe(v) for v in value]
    return value


class OutputLog:
    def __init__(self,root,config,metadata=None,manifest=None):
        name=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
        self.directory=Path(root).expanduser()/name; self.directory.mkdir(parents=True,exist_ok=False)
        self.config=asdict(config); self.metadata=dict(metadata or {},current_signal='avg_iq'); self.last_pair=None
        self.manifest=manifest;self.skipped=[];self.skipped_count=0
        self.file=(self.directory/'observations.csv').open('w',newline='')
        self.writer=None; self.trial_count=0
        self.runtime_file=(self.directory/'runtime.csv').open('w',newline='')
        self.runtime_writer=None
        self.trials([])

    def observation(self,obs,request,state):
        pair=(obs.wheel_stamp,obs.telemetry_stamp)
        if min(pair)<=0 or (self.last_pair is not None and (pair[0]<=self.last_pair[0] or pair[1]<=self.last_pair[1])):
            return
        self.last_pair=pair
        row=asdict(obs); echo=row.pop('final_request')
        if self.config['stage']=='speed_hold': row.pop('fault_code',None)
        row.update(state=state,request_stamp=request.stamp,requested_mode=request.mode,
                   requested_speed=request.speed,requested_current=request.current,
                   requested_steering=request.steering,final_mode=echo['mode'] if echo else '',
                   final_speed=echo['speed'] if echo else '',final_current=echo['current'] if echo else '',
                   final_steering=echo['steering'] if echo else '',final_stamp=echo['stamp'] if echo else '')
        if self.writer is None:
            self.writer=csv.DictWriter(self.file,fieldnames=list(row)); self.writer.writeheader()
        self.writer.writerow(row); self.file.flush()

    def _publish_json(self,trials,evidence_error=''):
        self.metadata.setdefault('evidence_valid',True)
        if evidence_error:
            self.metadata['evidence_valid']=False
            self.metadata['evidence_errors']=list(dict.fromkeys([*self.metadata.get('evidence_errors',[]),evidence_error]))
        payload=dict(configuration=self.config,metadata=self.metadata,trials=trials)
        if self.manifest is not None:
            payload.update(**self.manifest,skipped=self.skipped)
            payload['schema_version']=3 if self.config.get('sweep_protocol')=='full_range_step' else 2
            payload['current_signal']='avg_iq'
        pending=self.directory/'trials.json.tmp'
        pending.write_text(json.dumps(_json_safe(payload),indent=2,allow_nan=False)+'\n')
        pending.replace(self.directory/'trials.json')

    def trials(self,trials,evidence_error='',skipped=None):
        # JSON is the final acceptance checkpoint. Prepare and publish the CSV
        # before making a successful response visible in the authoritative JSON.
        if skipped is not None: self.skipped=list(skipped)
        if trials:
            pending=self.directory/'trials.csv.tmp'
            with pending.open('w',newline='') as output:
                names=list(dict.fromkeys(k for row in trials for k in row))
                writer=csv.DictWriter(output,fieldnames=names); writer.writeheader()
                writer.writerows({k:json.dumps(_json_safe(v),allow_nan=False) if isinstance(v,(dict,list)) else v
                                  for k,v in row.items()} for row in trials)
            pending.replace(self.directory/'trials.csv')
        self._publish_json(trials,evidence_error)
        self.trial_count=len(trials)
        self.skipped_count=len(self.skipped)

    def failure(self,trials,error,first_trial):
        # A failed CSV/observation checkpoint may leave no further queued trials.
        # Publish its rejection independently of the broken CSV output path.
        invalid=[{**row,'success':False,'reason':'logging_writer_failure',
            'response_completed':row.get('response_completed',row['success']),
            'evidence_valid':False,'evidence_failure':str(error),'mean_acceleration':None}
            if i>=first_trial else row for i,row in enumerate(trials)]
        self._publish_json(invalid,'logging_writer_failure: '+str(error))

    def runtime(self,row):
        if self.runtime_writer is None:
            self.runtime_writer=csv.DictWriter(self.runtime_file,fieldnames=list(row))
            self.runtime_writer.writeheader()
        self.runtime_writer.writerow(row); self.runtime_file.flush()

    def reset_observations(self): self.last_pair=None

    def close(self): self.file.close(); self.runtime_file.close()


class AsyncOutputLog:
    """Bounded background writer; overflow/error explicitly invalidates acquisition."""
    def __init__(self,root,config,metadata=None,capacity=128,manifest=None):
        self.writer=OutputLog(root,config,metadata,manifest=manifest); self.directory=self.writer.directory
        self.queue=Queue(maxsize=capacity); self.stop=Event()
        self.error=None; self.failed_after_trial_count=0; self.dropped_count=0; self.trial_count=0
        self.latest_trials=[];self.latest_skipped=[];self.skipped_count=0
        self.snapshot_version=0;self.failure_snapshot_version=-1
        self.thread=Thread(target=self._run,name='calibration-evidence-writer',daemon=True)
        self.thread.start()

    @property
    def pending_count(self): return self.queue.qsize()

    def _enqueue(self,kind,args):
        if self.error is not None: raise RuntimeError('logging writer failure: '+str(self.error))
        if self.stop.is_set(): raise RuntimeError('logging writer closed')
        try: self.queue.put_nowait((kind,args))
        except Full:
            self.dropped_count+=1
            error=RuntimeError('logging queue overflow; acquisition invalidated')
            self._latch_error(error)
            raise error

    def _latch_error(self,error):
        if self.error is None:
            self.failed_after_trial_count=self.writer.trial_count
            self.error=error

    def _publish_failure_checkpoint(self):
        # This runs on the writer thread, bypassing its saturated input queue.
        if self.error is None or self.failure_snapshot_version==self.snapshot_version:return
        version=self.snapshot_version
        try:
            self.writer.skipped=list(self.latest_skipped)
            self.writer.failure(self.latest_trials,self.error,self.failed_after_trial_count)
            self.failure_snapshot_version=version
        except Exception as error:
            if 'failure checkpoint unavailable' not in str(self.error):
                self.error=RuntimeError(str(self.error)+'; failure checkpoint unavailable: '+str(error))

    def observation(self,obs,request,state): self._enqueue('observation',(obs,request,state))
    def runtime(self,row): self._enqueue('runtime',(dict(row),))
    def reset_observations(self): self._enqueue('reset_observations',())
    def trials(self,trials,evidence_error='',skipped=None):
        changed=list(trials)!=self.latest_trials or (skipped is not None and list(skipped)!=self.latest_skipped)
        self.latest_trials=list(trials)
        if skipped is not None: self.latest_skipped=list(skipped)
        if changed:self.snapshot_version+=1
        self._enqueue('trials',(self.latest_trials,evidence_error,self.latest_skipped)); self.trial_count=len(trials)
        self.skipped_count=len(self.latest_skipped)

    def _run(self):
        while not self.stop.is_set() or not self.queue.empty():
            self._publish_failure_checkpoint()
            try: kind,args=self.queue.get(timeout=.05)
            except Empty: continue
            try:
                if kind=='trials' and self.error is not None:
                    # The independent failure checkpoint owns the newest
                    # snapshot. Older queued checkpoints must not replace it.
                    continue
                getattr(self.writer,kind)(*args)
            except Exception as exc:
                self._latch_error(exc)
            finally:
                self._publish_failure_checkpoint()
                self.queue.task_done()
        self._publish_failure_checkpoint()
        self.writer.close()

    def flush(self,timeout=5.):
        """Wait for durable checkpoints outside the real-time callback."""
        deadline=time.monotonic()+timeout
        while self.queue.unfinished_tasks or (
                self.error is not None and self.failure_snapshot_version<self.snapshot_version):
            if not self.thread.is_alive(): raise RuntimeError('logging writer stopped before flush')
            if time.monotonic()>=deadline: raise RuntimeError('logging writer flush timed out')
            time.sleep(.005)

    def close(self):
        self.stop.set(); self.thread.join(timeout=5.)
        if self.thread.is_alive(): raise RuntimeError('logging writer shutdown timed out')
