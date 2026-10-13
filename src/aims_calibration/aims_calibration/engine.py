"""ROS-independent, source-time longitudinal calibration state machine."""
from collections import deque
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, pstdev

from .planner import TrialPlan


@dataclass(frozen=True)
class Config:
    stage: str = 'speed_hold'
    steering_source: str = 'pp'
    speeds: tuple = (1., 2., 3.)
    currents: tuple = ()
    start_speed: float = 1.
    end_speed: float = .5
    max_speed: float = 4.
    speed_overshoot_tolerance: float = 0.  # Legacy profile field; no measured-speed stop gate.
    max_current: float = 100.
    stable_tolerance: float = .2
    stable_seconds: float = 1.
    measurement_seconds: float = 10.
    hold_sampling: str = 'cumulative'
    trial_timeout: float = 8.
    freshness: float = .2
    repeats: int = 1
    recovery_speed: float = 1.

    def __post_init__(self):
        if self.stage not in ('speed_hold','accel_interval','decel_current'):
            raise ValueError('unknown calibration stage')
        if self.steering_source not in ('pp','rc'): raise ValueError('unknown steering source')
        numbers=(self.start_speed,self.end_speed,self.max_speed,self.speed_overshoot_tolerance,self.max_current,
                 self.stable_tolerance,self.stable_seconds,self.measurement_seconds,
                 self.trial_timeout,self.freshness,self.recovery_speed,*self.speeds,*self.currents)
        if not all(math.isfinite(v) for v in numbers): raise ValueError('nonfinite configuration')
        if self.hold_sampling not in ('cumulative','continuous'):
            raise ValueError('hold_sampling must be cumulative or continuous')
        speed_bound = 5.
        if not 0 < self.max_speed <= speed_bound or not 0 < self.max_current <= 100.:
            raise ValueError('vehicle speed/current bounds exceeded')
        if not 0<=self.speed_overshoot_tolerance<=.2 or (
                self.stage!='speed_hold' and self.speed_overshoot_tolerance!=0.):
            raise ValueError('speed overshoot tolerance is limited to 0.2 m/s in speed_hold')
        if self.repeats < 1 or int(self.repeats)!=self.repeats: raise ValueError('invalid repeats')
        if any(v<=0 for v in (self.stable_tolerance,self.stable_seconds,
                             self.measurement_seconds,self.trial_timeout,self.freshness)):
            raise ValueError('timing and tolerance must be positive')
        if self.freshness>.2: raise ValueError('freshness may not exceed 0.2 seconds')
        if not self.speeds or any(not 0 < v <= self.max_speed for v in self.speeds):
            raise ValueError('explicit valid hold speeds required')
        if not 0 < self.recovery_speed <= self.max_speed: raise ValueError('invalid recovery speed')
        if self.stage != 'speed_hold':
            if not self.currents: raise ValueError('explicit current list required')
            sign=1 if self.stage=='accel_interval' else -1
            if any(not 0 < sign*v <= self.max_current for v in self.currents):
                raise ValueError('current list must have stage sign and satisfy limit')
            if not 0 < self.start_speed <= self.max_speed or not 0 < self.end_speed <= self.max_speed:
                raise ValueError('current trial requires positive bounded speeds')
            if sign*(self.end_speed-self.start_speed)<=0: raise ValueError('invalid speed interval')


@dataclass(frozen=True)
class TrackingConfig:
    settle_seconds: float = .25
    window_seconds: float = .5
    min_samples: int = 10
    abs_tolerance: float = 1.
    rel_tolerance: float = .1
    failure_fraction: float = .9
    max_attempts: int = 3

    def __post_init__(self):
        values=(self.settle_seconds,self.window_seconds,self.abs_tolerance,
                self.rel_tolerance,self.failure_fraction)
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values):
            raise ValueError('nonfinite tracking settings')
        if self.settle_seconds<0 or self.window_seconds<=0 or self.abs_tolerance<=0 or self.rel_tolerance<0 or not 0<self.failure_fraction<=1:
            raise ValueError('invalid tracking settings')
        if type(self.min_samples) is not int or self.min_samples<2 or type(self.max_attempts) is not int or self.max_attempts<1:
            raise ValueError('invalid tracking sample/attempt count')


@dataclass(frozen=True)
class Request:
    stamp: float
    mode: int = 0
    steering: float = 0.
    speed: float = 0.
    current: float = 0.

    def corresponds(self, other):
        return self.mode==other.mode and all(math.isclose(a,b,abs_tol=1e-5) for a,b in
            ((self.steering,other.steering),(self.speed,other.speed),(self.current,other.current)))


@dataclass(frozen=True)
class Observation:
    wheel_stamp: float = 0.
    telemetry_stamp: float = 0.
    velocity: float = math.nan
    current_motor: float = math.nan
    voltage_input: float = math.nan
    erpm: float = math.nan
    authority: bool = False
    authority_stamp: float = 0.
    final_request: Request = None
    final_received: float = 0.
    rc_stamp: float = 0.
    rc_valid: bool = False
    straight: bool = False
    region: str = ''
    steering: float = 0.
    tracking_reason: str = ''
    acceleration: float = math.nan
    acceleration_stamp: float = 0.
    acceleration_quality: str = 'unavailable'
    fault_code: int = 0


@dataclass(frozen=True)
class RCProfile:
    name: str = 'steering_ch1_throttle_ch3_aux_ch5_to_ch10'
    steering_limit: float = .4751
    deadzone: int = 50

    def __post_init__(self):
        if self.name not in ('steering_ch1_throttle_ch3_aux_ch5_to_ch10','sequential_ch1_ch2'):
            raise ValueError('unknown RC profile')

    def evaluate(self, channels):
        steering,lock,calib,used=(1,5,8,(1,3,5,6,7,8,10)) if self.name.startswith('steering_') else (2,3,7,(1,2,3,4,5,6,7))
        if len(channels)!=16 or any(not 0 < channels[i-1] <= 2047 for i in used):
            return False,0.,False
        raw=channels[steering-1]; delta=raw-992
        # Match selector normalization exactly; deadzone determines sampling eligibility.
        angle=-max(-1.,min(1.,delta/(1810-992 if delta>0 else 992-172)))*self.steering_limit
        return channels[lock-1]>=992 and channels[calib-1]>992,angle,abs(delta)<=self.deadzone


class Regions:
    def __init__(self,length,regions):
        if not math.isfinite(length) or length<=0: raise ValueError('invalid reference length')
        self.length=length; self.regions=list(regions)
        ids=set()
        for r in self.regions:
            a,b=r['start_s'],r['end_s']
            if r.get('kind','straight')!='straight' or not r['id'] or r['id'] in ids:
                raise ValueError('invalid or duplicate straight region')
            if not all(math.isfinite(v) and 0<=v<length for v in (a,b)) or a==b:
                raise ValueError('invalid region interval')
            ids.add(r['id'])
        if not self.regions: raise ValueError('explicit straight regions required')
        # Overlap is ambiguous and can conceal an exit from a previous trial window.
        boundaries=sorted({0.,length,*[r[k] for r in self.regions for k in ('start_s','end_s')]})
        for a,b in zip(boundaries,boundaries[1:]):
            if sum(self._inside(r,(a+b)/2) for r in self.regions)>1:
                raise ValueError('overlapping straight regions')

    def _inside(self,r,s):
        a,b=r['start_s'],r['end_s']
        return a<=s<b if a<b else s>=a or s<b

    def region(self,s):
        if not math.isfinite(s): return None
        s%=self.length
        return next((r['id'] for r in self.regions if self._inside(r,s)),None)

    @classmethod
    def load(cls,directory,length):
        directory=Path(directory)
        value=json.loads((directory/'calibration_regions.json').read_text())
        digest=hashlib.sha256((directory/'reference.json').read_bytes()).hexdigest()
        if value.get('schema_version')!=1 or value.get('reference_sha256')!=digest:
            raise ValueError('straight regions reference hash mismatch')
        return cls(length,value['regions'])


class Engine:
    def __init__(self,config,trial_plan=None,tracking_config=None):
        self.config=config; self.state='IDLE'; self.reason='disabled'; self.enabled=False
        self.tracking_config=tracking_config if tracking_config is not None else TrackingConfig()
        if not isinstance(self.tracking_config,TrackingConfig): raise ValueError('invalid tracking configuration')
        self.history=deque(maxlen=24); self.samples=[]; self.trials=[]; self.skipped=[]
        self.index=0; self.last_now=None; self.armed_at=None; self.selected=False
        self.attempt_counts={}; self.attempt_id=None; self.attempt_number=0
        if trial_plan is not None:
            if config.stage=='speed_hold' or not trial_plan: raise ValueError('invalid structured plan')
            self.trial_plan=list(trial_plan)
            for item in self.trial_plan:
                if not isinstance(item,TrialPlan) or item.stage!=config.stage:
                    raise ValueError('structured plan stage mismatch')
                if max(item.start_speed,item.end_speed)>config.max_speed or abs(item.current)>config.max_current:
                    raise ValueError('structured trial exceeds configured limits')
                if item.repeat>config.repeats: raise ValueError('structured repeat exceeds configured repeats')
        elif config.stage!='speed_hold':
            kind='acceleration' if config.stage=='accel_interval' else 'braking'
            self.trial_plan=[TrialPlan(config.stage,config.start_speed,config.end_speed,current,repeat,kind)
                for current in config.currents for repeat in range(1,config.repeats+1)]
        else: self.trial_plan=[]
        self.plan=[item.current for item in self.trial_plan] if self.trial_plan else [v for v in config.speeds for _ in range(config.repeats)]
        self._reset_trial()

    @property
    def serializable_plan(self):
        if self.config.stage=='speed_hold': return []
        return [dict(plan_index=i,**asdict(item)) for i,item in enumerate(self.trial_plan)]

    def _item(self):
        return self.trial_plan[self.index]

    def _reset_trial(self):
        self.stable_since=None; self.started=None; self.start_velocity=None
        self.active_samples=[]; self.last_pair=None; self.region=None
        self.last_wheel=None; self.last_telemetry=None; self.phase_started=None; self.phase_ack_stamp=None
        self.hold_elapsed=0.; self.hold_segments=[]; self.hold_previous=None; self.hold_segment=None
        self.first_current_request_stamp=None; self.starting_wheel_observation=None
        self.tracking_window=deque(); self.tracking_observations=[]; self.last_tracking_stamp=None
        self.tracking_started_stamp=None
        self.tracking_status='unverified'; self.tracking_evidence={}

    def _pause_hold(self,reason):
        """Keep admitted samples, but break time integration and require settling."""
        if self.hold_segment is not None:
            self.hold_segment['end_reason']=reason
        self.hold_previous=None; self.hold_segment=None
        self.stable_since=None; self.phase_started=None; self.region=None
        self.reason=reason

    def _sample_hold(self,obs,row):
        if self.hold_segment is None:
            self.hold_segment=dict(segment_id=len(self.hold_segments)+1,region=obs.region,
                start_stamp=obs.wheel_stamp,end_stamp=obs.wheel_stamp,
                start_telemetry_stamp=obs.telemetry_stamp,end_telemetry_stamp=obs.telemetry_stamp,
                duration_s=0.,sample_offset=len(self.active_samples),samples=0,end_reason='sampling')
            self.hold_segments.append(self.hold_segment)
        if self.hold_previous is not None:
            duration=obs.wheel_stamp-self.hold_previous[0]
            self.hold_elapsed+=duration
            self.hold_segment['duration_s']+=duration
        self.hold_previous=(obs.wheel_stamp,obs.telemetry_stamp)
        self.hold_segment.update(end_stamp=obs.wheel_stamp,end_telemetry_stamp=obs.telemetry_stamp)
        self.hold_segment['samples']+=1
        row['segment_id']=self.hold_segment['segment_id']

    def enable(self,value,now=None):
        if self.started is not None or self.active_samples or (self.config.stage!='speed_hold' and self.state=='APPLY'):
            self._finish(False,'operator_disabled' if not value else 'operator_reenabled')
        self.enabled=value; self.selected=False; self._reset_trial()
        self.history.clear()
        self.state='PREPARE' if value else 'STOPPING'
        self.reason='waiting_for_selection' if value else 'operator_disabled'
        if value:
            self.index=0; self.last_now=None; self.armed_at=now
            self.attempt_counts={}

    def sent(self,request):
        self.history.append(request)
        if self.config.stage!='speed_hold' and self.state in ('APPLY','MEASURE') and request.mode==1 and math.isclose(request.current,self._item().current,abs_tol=1e-5) and self.first_current_request_stamp is None:
            self.first_current_request_stamp=request.stamp

    def _begin_attempt(self):
        self.attempt_counts[self.index]=self.attempt_counts.get(self.index,0)+1
        self.attempt_number=self.attempt_counts[self.index]
        self.attempt_id=len(self.trials)+1
        self.tracking_status='not_applicable' if self._item().kind=='coast' else 'unverified'

    def _advance_current(self,reason,skip_unreachable=False):
        item=self._item(); trigger=self.attempt_id; evidence=dict(self.tracking_evidence)
        self.index+=1
        if skip_unreachable:
            while self.index<len(self.plan):
                next_item=self._item()
                same_group=next_item.start_speed==item.start_speed and next_item.kind==item.kind
                if item.stage=='accel_interval': same_group= same_group and next_item.end_speed==item.end_speed
                if item.kind=='positive_deceleration': same_group= same_group and next_item.current<=item.current
                else: same_group= same_group and abs(next_item.current)>=abs(item.current)
                if not same_group: break
                self.skipped.append(dict(plan_index=self.index,**asdict(next_item),success=False,
                    reason='skipped_unreachable_current',triggering_attempt_id=trigger,evidence=evidence))
                self.index+=1
        self._reset_trial()
        self.state='COMPLETE' if self.index==len(self.plan) else 'PREPARE'
        self.reason=reason

    def _straight_exit(self,obs):
        active=self.state in ('MEASURE','APPLY')
        if active:
            self._finish(False,'straight_region_exit',dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
            if self.attempt_number>=self.tracking_config.max_attempts:
                self.trials[-1]['uncovered']=True
                self._advance_current('straight_region_exit')
            else:
                self._reset_trial(); self.state='RECOVER'; self.reason='straight_region_exit'
        else:
            self._reset_trial(); self.state='RECOVER'; self.reason='straight_region_exit'

    def _track_current(self,obs):
        settings=self.tracking_config; item=self._item()
        if item.kind=='coast': return False
        stamp=obs.telemetry_stamp
        # Epoch-valued ROS stamps have coarser float resolution than 1 ns.
        # Allow only their representation error, not a shorter physical window.
        time_tolerance=max(1e-9,2*math.ulp(stamp))
        if self.phase_ack_stamp is None or stamp+time_tolerance<self.phase_ack_stamp+settings.settle_seconds or (self.last_tracking_stamp is not None and stamp<=self.last_tracking_stamp):
            return False
        self.last_tracking_stamp=stamp
        if self.tracking_started_stamp is None: self.tracking_started_stamp=stamp
        row=dict(telemetry_stamp=stamp,current_motor=obs.current_motor,wheel_stamp=obs.wheel_stamp,
                 velocity=obs.velocity,region=obs.region)
        self.tracking_window.append(row); self.tracking_observations.append(row)
        while self.tracking_window and self.tracking_window[0]['telemetry_stamp']<stamp-settings.window_seconds-time_tolerance:
            self.tracking_window.popleft()
        count=len(self.tracking_window)
        window_start=max(self.tracking_started_stamp,stamp-settings.window_seconds)
        span=stamp-window_start
        tolerance=max(settings.abs_tolerance,settings.rel_tolerance*abs(item.current))
        evidence=dict(target_current=item.current,tolerance=tolerance,sample_count=count,
            window_start_stamp=window_start,window_end_stamp=stamp,
            window_duration_s=span,settle_seconds=settings.settle_seconds,
            window_seconds=settings.window_seconds,min_samples=settings.min_samples,
            failure_fraction=settings.failure_fraction)
        if count<settings.min_samples or span+time_tolerance<settings.window_seconds:
            if self.tracking_status=='unverified': self.tracking_evidence=evidence
            return False
        self.tracking_evidence=evidence
        values=[r['current_motor'] for r in self.tracking_window]
        if item.kind=='positive_deceleration':
            above=sum(v-item.current>tolerance for v in values)
            below=sum(item.current-v>tolerance for v in values)
            count_bad=max(above,below)
            evidence.update(error_side='above' if above>=below else 'below',above_count=above,below_count=below)
        else:
            sign=1 if item.current>0 else -1
            count_bad=sum(sign*(item.current-v)>tolerance for v in values)
            evidence['under_magnitude_count']=count_bad
        evidence.update(failure_count=count_bad,failure_ratio=count_bad/count,
                        mean_current_motor=mean(values))
        unreachable=count_bad/count+1e-12>=settings.failure_fraction
        self.tracking_status='unreachable' if unreachable else 'verified'
        return unreachable

    def fault(self,reason):
        if self.state!='FAULT': self._finish(False,reason)
        self.state='FAULT'; self.reason=reason; self.enabled=False; self._reset_trial()

    def invalidate_evidence(self,first_trial,reason):
        """Revoke acceptance if acquisition failed before its evidence checkpoint."""
        for i in range(first_trial,len(self.trials)):
            previous=self.trials[i]
            self.trials[i]={**previous,'success':False,'reason':str(reason),
                'response_completed':previous.get('response_completed',previous['success']),
                'evidence_valid':False,'evidence_failure':str(reason),'mean_acceleration':None}

    def _finish(self,success,reason,end=None):
        c=self.config
        if self.started is None and not self.active_samples and self.state not in ('MEASURE','APPLY'):
            return
        last=end or (self.active_samples[-1] if self.active_samples else {})
        ended=last.get('wheel_stamp'); velocity=last.get('velocity')
        duration=ended-self.started if ended is not None and self.started is not None else None
        currents=[s['current_motor'] for s in self.active_samples]
        cumulative=c.stage=='speed_hold' and c.hold_sampling=='cumulative'
        segments=[]
        if cumulative:
            for segment in self.hold_segments:
                segment=dict(segment)
                offset=segment.pop('sample_offset')
                values=currents[offset:offset+segment['samples']]
                segment.update(mean_current_motor=mean(values),std_current_motor=pstdev(values))
                if segment['end_reason']=='sampling': segment['end_reason']=reason
                segments.append(segment)
        result=dict(trial_id=len(self.trials)+1,plan_index=self.index,stage=c.stage,
            requested_target=self.plan[self.index],region=self.region,success=success,reason=reason,
            first_current_ack_stamp=self.phase_ack_stamp,evidence_valid=True,
            start_stamp=self.started,end_stamp=ended,start_speed=self.start_velocity,end_speed=velocity,
            duration_s=self.hold_elapsed if cumulative else duration,samples=len(currents),mean_current_motor=mean(currents) if currents else None,
            std_current_motor=pstdev(currents) if currents else None,
            mean_acceleration=(velocity-self.start_velocity)/duration if success and c.stage!='speed_hold'
                and duration is not None and duration>0 else None)
        if c.stage=='speed_hold':
            result.update(hold_sampling=c.hold_sampling,wall_duration_s=duration,
                segment_count=len(segments),segments=segments)
        else:
            item=self._item()
            result.update(attempt_id=self.attempt_id,attempt_number=self.attempt_number,repeat=item.repeat,
                kind=item.kind,current=item.current,start_target_speed=item.start_speed,end_target_speed=item.end_speed,
                baseline_current=item.baseline_current,first_current_request_stamp=self.first_current_request_stamp,
                starting_wheel_observation=self.starting_wheel_observation,
                response_duration_s=ended-self.first_current_request_stamp if ended is not None and self.first_current_request_stamp is not None else None,
                tracking_status=self.tracking_status,tracking_evidence=dict(self.tracking_evidence),
                tracking_observations=list(self.tracking_observations),uncovered=False)
        self.trials.append(result)

    def _next_current_request(self,now,obs):
        if self.state=='COMPLETE': return self._request(now,obs)
        speed=self.config.recovery_speed if not obs.straight and self.config.steering_source=='pp' else self._item().start_speed
        return self._request(now,obs,speed=speed)

    def _request(self,now,obs,mode=0,speed=0.,current=0.):
        return Request(now,mode,obs.steering if self.state not in ('IDLE','STOPPING','FAULT','COMPLETE') else 0.,speed,current)

    def _fresh(self,now,stamp): return math.isfinite(stamp) and stamp>0 and 0<=now-stamp<=self.config.freshness

    def tick(self,now,obs):
        c=self.config
        if not math.isfinite(now) or now<=0 or (self.last_now is not None and now<self.last_now):
            self.fault('clock_backwards'); return self._request(now,obs,1)
        self.last_now=now
        if self.state in ('FAULT','STOPPING'): return self._request(now,obs,1)
        if not self.enabled or self.state in ('IDLE','COMPLETE'): return self._request(now,obs)
        if c.stage!='speed_hold' and obs.fault_code!=0:
            self.fault('vesc_fault_'+str(obs.fault_code)); return self._request(now,obs,1)
        if self.armed_at is None: self.armed_at=now
        authority=obs.authority and self._fresh(now,obs.authority_stamp) and obs.authority_stamp>self.armed_at
        echo=obs.final_request
        echo_ok=echo is not None and self._fresh(now,obs.final_received) and self._fresh(now,echo.stamp) and any(
            self._fresh(now,r.stamp) and r.stamp<=echo.stamp and r.corresponds(echo) for r in self.history)
        if not self.selected:
            source_stamps=(obs.rc_stamp,obs.wheel_stamp,obs.telemetry_stamp)
            ready=authority and obs.rc_valid and all(
                self._fresh(now,t) and t>self.armed_at for t in source_stamps) and all(
                math.isfinite(v) for v in (obs.velocity,obs.current_motor,obs.voltage_input,obs.erpm,obs.steering))
            if not ready or not echo_ok or echo.stamp<=self.armed_at:
                return self._request(now,obs)
            self.selected=True
        if not authority:
            self.fault('authority_lost_or_stale'); return self._request(now,obs,1)
        reason=''
        if not self._fresh(now,obs.rc_stamp) or not obs.rc_valid: reason='rc_invalid_or_stale'
        elif not self._fresh(now,obs.wheel_stamp) or not math.isfinite(obs.velocity): reason='wheel_invalid_or_stale'
        elif not self._fresh(now,obs.telemetry_stamp) or not all(math.isfinite(v) for v in
            (obs.current_motor,obs.voltage_input,obs.erpm)): reason='telemetry_invalid_or_stale'
        elif c.steering_source=='pp' and obs.tracking_reason: reason=obs.tracking_reason
        elif not math.isfinite(obs.steering): reason='steering_invalid'
        elif obs.velocity<0.: reason='reverse_motion'
        if reason:
            self.fault(reason); return self._request(now,obs,1)
        if not echo_ok:
            # First selected tick establishes an initial echo, while still requesting zero.
            if not self.history: return self._request(now,obs)
            self.fault('final_command_mismatch_or_stale'); return self._request(now,obs,1)
        pair=(obs.wheel_stamp,obs.telemetry_stamp)
        if (self.last_wheel is not None and obs.wheel_stamp<self.last_wheel) or (
            self.last_telemetry is not None and obs.telemetry_stamp<self.last_telemetry):
            self.fault('source_clock_backwards'); return self._request(now,obs,1)
        self.last_wheel=obs.wheel_stamp; self.last_telemetry=obs.telemetry_stamp
        unique=self.last_pair is None or (pair[0]>self.last_pair[0] and pair[1]>self.last_pair[1])
        target=self.plan[self.index] if c.stage=='speed_hold' else self._item().start_speed
        end_speed=c.end_speed if c.stage=='speed_hold' else self._item().end_speed
        requested=self._request(now,obs,speed=target) if c.stage=='speed_hold' else self._request(now,obs,1,current=self.plan[self.index])
        phase_echo=echo.mode==requested.mode and math.isclose(echo.speed,requested.speed,abs_tol=1e-5) and math.isclose(echo.current,requested.current,abs_tol=1e-5)
        current_phase=c.stage!='speed_hold' and self.state in ('APPLY','MEASURE')
        if current_phase and phase_echo and self.phase_ack_stamp is None:
            self.phase_ack_stamp=echo.stamp
        acknowledged_sources=self.phase_ack_stamp is not None and min(pair)>=self.phase_ack_stamp
        boundary=current_phase and (obs.velocity>=end_speed if c.stage=='accel_interval' else obs.velocity<=end_speed)
        same_region=obs.straight and (self.region is None or self.region==obs.region)
        if current_phase and not same_region:
            self._straight_exit(obs)
            if self.state=='COMPLETE': return self._request(now,obs)
            return self._request(now,obs,speed=c.recovery_speed if c.steering_source=='pp' else self._item().start_speed)
        if boundary and not (unique and phase_echo and acknowledged_sources and same_region):
            if not phase_echo: reason='boundary_before_current_echo'
            elif not unique: reason='boundary_without_new_telemetry' if pair[1]<=self.last_pair[1] else 'boundary_without_new_wheel'
            elif not acknowledged_sources: reason='boundary_before_selected_current_observation'
            else: reason='straight_region_exit'
            self._finish(False,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
            self.index+=1; self._reset_trial()
            self.state='COMPLETE' if self.index==len(self.plan) else 'PREPARE'
            self.reason=reason
            return self._next_current_request(now,obs)
        if not obs.straight or (self.region is not None and self.region!=obs.region):
            if c.stage=='speed_hold' and c.hold_sampling=='cumulative':
                self._pause_hold('straight_region_exit')
                self.state='RECOVER'
                return self._request(now,obs,speed=c.recovery_speed if c.steering_source=='pp' else target)
            if self.state in ('MEASURE','APPLY'):
                self._finish(False,'straight_region_exit'); self.reason='straight_region_exit'
            self._reset_trial(); self.state='RECOVER'
            return self._request(now,obs,speed=c.recovery_speed if c.steering_source=='pp' else target)
        if self.state=='RECOVER': self.state='PREPARE'
        self.region=obs.region
        if self.state=='PREPARE':
            if unique:
                self.last_pair=pair
                if abs(obs.velocity-target)<=c.stable_tolerance and echo.mode==0 and math.isclose(echo.speed,target,abs_tol=1e-5):
                    if self.stable_since is None: self.stable_since=obs.wheel_stamp
                    if obs.wheel_stamp-self.stable_since>=c.stable_seconds:
                        self.state='MEASURE' if c.stage=='speed_hold' else 'APPLY'
                        self.phase_started=obs.wheel_stamp
                        if c.stage!='speed_hold': self._begin_attempt()
                        if c.stage!='speed_hold' and (obs.velocity>=end_speed if c.stage=='accel_interval' else obs.velocity<=end_speed):
                            # Stable preparation tolerance can overlap the end of a
                            # narrow interval. Never emit its first current request.
                            reason='boundary_before_current_request'
                            self._finish(False,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
                            self.index+=1; self._reset_trial()
                            self.state='COMPLETE' if self.index==len(self.plan) else 'PREPARE'
                            self.reason=reason
                            return self._next_current_request(now,obs)
                        if c.stage=='speed_hold' and c.hold_sampling=='continuous':
                            self.started=obs.wheel_stamp; self.start_velocity=obs.velocity
                else: self.stable_since=None
            return self._request(now,obs,speed=target) if c.stage=='speed_hold' or self.state=='PREPARE' else self._request(now,obs,1,current=self.plan[self.index])
        if c.stage=='speed_hold' and abs(obs.velocity-target)>c.stable_tolerance:
            if c.hold_sampling=='cumulative':
                self._pause_hold('speed_unstable'); self.state='PREPARE'
                return self._request(now,obs,speed=target)
            self._finish(False,'speed_unstable'); self._reset_trial()
            self.state='PREPARE'; self.reason='speed_unstable'
            return self._request(now,obs,speed=target)
        if c.stage=='speed_hold' and c.hold_sampling=='cumulative':
            if not phase_echo:
                self._pause_hold('waiting_for_target_speed_echo'); self.state='PREPARE'
                return requested
            if unique and self.hold_previous is not None and any(
                    stamp-previous>c.freshness for stamp,previous in zip(pair,self.hold_previous)):
                self._pause_hold('sample_source_gap'); self.state='PREPARE'
                return requested
        if current_phase and not boundary and phase_echo and acknowledged_sources:
            if self._track_current(obs):
                self._finish(False,'current_tracking_unreachable',dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
                self._advance_current('current_tracking_unreachable',skip_unreachable=True)
                return self._next_current_request(now,obs)
        if unique and phase_echo and (c.stage=='speed_hold' or acknowledged_sources):
            self.last_pair=pair
            row=asdict(obs); row.pop('final_request')
            if c.stage=='speed_hold': row.pop('fault_code')
            row.update(trial_id=len(self.trials)+1,plan_index=self.index,stage=c.stage,
                       requested_mode=requested.mode,requested_speed=requested.speed,requested_current=requested.current)
            if c.stage=='speed_hold' and c.hold_sampling=='cumulative': self._sample_hold(obs,row)
            if c.stage!='speed_hold':
                row.update(attempt_id=self.attempt_id,attempt_number=self.attempt_number,repeat=self._item().repeat,kind=self._item().kind)
                if self.starting_wheel_observation is None:
                    self.starting_wheel_observation=dict(wheel_stamp=obs.wheel_stamp,telemetry_stamp=obs.telemetry_stamp,velocity=obs.velocity)
            self.samples.append(row); self.active_samples.append(row)
            if self.started is None: self.started=obs.wheel_stamp; self.start_velocity=obs.velocity
            self.state='MEASURE'
            elapsed=obs.wheel_stamp-self.started
            hold_duration=self.hold_elapsed if c.hold_sampling=='cumulative' else elapsed
            reached=hold_duration+1e-9>=c.measurement_seconds if c.stage=='speed_hold' else (
                obs.velocity>=end_speed if c.stage=='accel_interval' else obs.velocity<=end_speed)
            timeout=c.stage!='speed_hold' and obs.wheel_stamp-self.phase_started>=c.trial_timeout
            if reached or timeout:
                valid_response=reached and (c.stage=='speed_hold' or elapsed>0)
                reason='completed' if valid_response else ('interval_without_positive_duration' if reached else 'trial_timeout')
                self._finish(valid_response,reason,row)
                self.index+=1; self._reset_trial()
                self.state='COMPLETE' if self.index==len(self.plan) else 'PREPARE'
                self.reason=reason
                if c.stage=='speed_hold':
                    return self._request(now,obs,speed=self.plan[self.index]) if self.state=='PREPARE' else self._request(now,obs)
                return self._next_current_request(now,obs)
        if current_phase and not boundary and self.first_current_request_stamp is not None and now-self.first_current_request_stamp>=c.trial_timeout:
            self._finish(False,'trial_timeout',dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
            self._advance_current('trial_timeout')
            return self._next_current_request(now,obs)
        return requested
