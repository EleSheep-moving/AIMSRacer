"""ROS-independent, source-time longitudinal calibration state machine."""
from collections import deque
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, pstdev

from .planner import TrialPlan
from .full_step import FullStepFlow, steady_settings
from .steady import SteadyDetector


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
    predictive_exit: bool = False
    measured_current_handoff: bool = False
    sweep_protocol: str = 'interval'
    steady_window_seconds: float = 1.
    steady_confirm_seconds: float = 1.
    steady_max_abs_acceleration: float = .1
    steady_max_speed_span: float = .15
    steady_min_samples: int = 40
    straight_exit_margin_m: float = 1.

    def __post_init__(self):
        if self.stage not in ('speed_hold','accel_interval','decel_current'):
            raise ValueError('unknown calibration stage')
        if self.steering_source not in ('pp','rc'): raise ValueError('unknown steering source')
        if self.sweep_protocol not in ('interval','full_range_step'):
            raise ValueError('unknown sweep protocol')
        if type(self.measured_current_handoff) is not bool:
            raise ValueError('measured_current_handoff must be a boolean')
        if self.sweep_protocol=='full_range_step' and (self.stage=='speed_hold' or self.predictive_exit or
                not self.measured_current_handoff or self.steering_source!='pp'):
            raise ValueError('full-range steps require B/C, PP regions, measured current handoff and no endpoint prediction')
        if self.measured_current_handoff and self.sweep_protocol!='full_range_step':
            raise ValueError('measured current handoff requires the full-range step protocol')
        for name in ('steady_window_seconds','steady_confirm_seconds','steady_max_abs_acceleration','steady_max_speed_span','straight_exit_margin_m'):
            value=getattr(self,name)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid '+name)
        if type(self.steady_min_samples) is not int or self.steady_min_samples<2:
            raise ValueError('invalid steady_min_samples')
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
    avg_iq: float = math.nan
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
    straight_remaining_m: float = math.nan


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

    def remaining(self,s):
        """Forward distance to this region's end, including a lap wrap."""
        if not math.isfinite(s): return None
        s%=self.length
        region=next((r for r in self.regions if self._inside(r,s)),None)
        return (region['end_s']-s)%self.length if region is not None else None

    @classmethod
    def load(cls,directory,length):
        directory=Path(directory)
        value=json.loads((directory/'calibration_regions.json').read_text())
        digest=hashlib.sha256((directory/'reference.json').read_bytes()).hexdigest()
        if value.get('schema_version')!=1 or value.get('reference_sha256')!=digest:
            raise ValueError('straight regions reference hash mismatch')
        return cls(length,value['regions'])


class Engine(FullStepFlow):
    def __init__(self,config,trial_plan=None,tracking_config=None,protection=None):
        needs_handoff=config.predictive_exit or config.measured_current_handoff
        if type(config.predictive_exit) is not bool or (needs_handoff and (config.stage=='speed_hold' or protection is None)):
            raise ValueError('current handoff/exit protection requires a B/C baseline and protection object')
        self.protection=protection if needs_handoff else None
        self.handoff=None;self.exit_prediction=None
        self.config=config; self.state='IDLE'; self.reason='disabled'; self.enabled=False
        self.tracking_config=tracking_config if tracking_config is not None else TrackingConfig()
        if not isinstance(self.tracking_config,TrackingConfig): raise ValueError('invalid tracking configuration')
        self.full_step=config.sweep_protocol=='full_range_step'
        self.steady_settings=steady_settings(config,self.tracking_config)
        self.steady=SteadyDetector(self.steady_settings) if self.full_step else None
        self.retry_region=None;self.retry_left=False;self.last_prepare_wheel=None
        self.history=deque(maxlen=24); self.samples=[]; self.trials=[]; self.skipped=[]
        self.index=0; self.last_now=None; self.armed_at=None; self.selected=False
        self.attempt_counts={}; self.attempt_id=None; self.attempt_number=0
        self.decel_owner_index=None
        self.group_recovery_started=None; self.group_recovery_request_stamp=None; self.group_recovery_ack_stamp=None
        self.group_forward_since=None; self.group_iq_samples=0;self.group_iq_stamp=None; self.results_revision=0
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
        if self.steady:self.steady.reset()
        self.last_prepare_wheel=None
        self.steady_evidence={}
        self.exit_prediction=None
        self.stable_since=None; self.started=None; self.start_velocity=None
        self.active_samples=[]; self.last_pair=None; self.region=None
        self.last_wheel=None; self.last_telemetry=None; self.phase_started=None; self.phase_ack_stamp=None
        self.hold_elapsed=0.; self.hold_segments=[]; self.hold_previous=None; self.hold_segment=None
        self.first_current_request_stamp=None; self.starting_wheel_observation=None
        self.start_trigger_observation=None
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
        if self.handoff is not None and self.state in ('UNLOAD','HOLD'):
            self.handoff.update(failure='operator_disabled' if not value else 'operator_reenabled',hold_confirmed=False)
            self.trials[-1]['handoff']=dict(self.handoff);self.results_revision+=1
        if self.started is not None or self.active_samples or (self.config.stage!='speed_hold' and self.state=='APPLY'):
            self._finish(False,'operator_disabled' if not value else 'operator_reenabled')
        self.enabled=value; self.selected=False; self._reset_trial()
        self.history.clear()
        self.handoff=None;self.exit_prediction=None
        self.retry_region=None;self.retry_left=False
        if self.protection:self.protection.reset()
        self.decel_owner_index=None
        self.group_recovery_started=None; self.group_recovery_request_stamp=None; self.group_recovery_ack_stamp=None
        self.group_forward_since=None
        self.state='PREPARE' if value else 'STOPPING'
        self.reason='waiting_for_selection' if value else 'operator_disabled'
        if value:
            self.index=0; self.last_now=None; self.armed_at=now
            self.attempt_counts={}

    def sent(self,request):
        self.history.append(request)
        if self.full_step and self.handoff is not None:
            field='withdraw_request_stamp' if self.state=='UNLOAD' else 'recovery_request_stamp'
            if field not in self.handoff:
                self.handoff[field]=request.stamp
                self.trials[-1]['handoff']=dict(self.handoff);self.results_revision+=1
        if self.state=='GROUP_RECOVER' and request.mode==1 and request.current==0 and self.group_recovery_request_stamp is None:
            self.group_recovery_request_stamp=request.stamp
        if self.config.stage!='speed_hold' and self.state in ('APPLY','MEASURE') and request.mode==1 and math.isclose(request.current,self._item().current,abs_tol=1e-5) and self.first_current_request_stamp is None:
            self.first_current_request_stamp=request.stamp
            if self.config.stage=='decel_current': self.decel_owner_index=self.index

    def _begin_attempt(self):
        self.attempt_counts[self.index]=self.attempt_counts.get(self.index,0)+1
        self.attempt_number=self.attempt_counts[self.index]
        self.attempt_id=len(self.trials)+1
        self.tracking_status='not_applicable' if self._item().kind=='coast' else 'unverified'

    def _advance_current(self,reason):
        self.retry_region=None;self.retry_left=False
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
        row=dict(telemetry_stamp=stamp,avg_iq=obs.avg_iq,current_motor=obs.current_motor,wheel_stamp=obs.wheel_stamp,
                 velocity=obs.velocity,region=obs.region)
        self.tracking_window.append(row); self.tracking_observations.append(row)
        while self.tracking_window and self.tracking_window[0]['telemetry_stamp']<stamp-settings.window_seconds-time_tolerance:
            self.tracking_window.popleft()
        count=len(self.tracking_window)
        window_start=max(self.tracking_started_stamp,stamp-settings.window_seconds)
        span=stamp-window_start
        tolerance=max(settings.abs_tolerance,settings.rel_tolerance*abs(item.current))
        evidence=dict(current_signal="avg_iq",target_current=item.current,tolerance=tolerance,sample_count=count,
            window_start_stamp=window_start,window_end_stamp=stamp,
            window_duration_s=span,settle_seconds=settings.settle_seconds,
            window_seconds=settings.window_seconds,min_samples=settings.min_samples,
            failure_fraction=settings.failure_fraction)
        if count<settings.min_samples or span+time_tolerance<settings.window_seconds:
            if self.tracking_status=='unverified': self.tracking_evidence=evidence
            return False
        self.tracking_evidence=evidence
        values=[r['avg_iq'] for r in self.tracking_window]
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
                        mean_iq=mean(values),mean_current_motor=mean([r['current_motor'] for r in self.tracking_window]))
        unreachable=count_bad/count+1e-12>=settings.failure_fraction
        self.tracking_status='outside_tolerance' if unreachable else 'verified'
        return unreachable

    def _skip_decel_group(self,now,obs,reason):
        """Resolve one real C attempt and skip its remaining speed group."""
        origin=self.decel_owner_index
        if self.state in ('APPLY','MEASURE'):
            self._finish(False,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
        trigger=next((r for r in reversed(self.trials) if r['plan_index']==origin),None)
        if trigger is None:
            self.fault('decel_group_trigger_unavailable'); return self._request(now,obs,1)
        if reason=='reverse_motion':
            trigger.update(response_completed=trigger['success'],success=False,reason=reason,mean_acceleration=None,
                **(dict(pulse_valid=False,interval_completed=False) if self.protection else {}),
                decision_evidence=dict(observed_at=now,observation=asdict(obs)))
            evidence=trigger['decision_evidence']
        else: evidence=trigger['tracking_evidence']
        speed=self.trial_plan[origin].start_speed
        end=origin+1
        while end<len(self.plan) and self.trial_plan[end].start_speed==speed: end+=1
        for index in range(max(self.index,origin+1),end):
            self.skipped.append(dict(plan_index=index,**asdict(self.trial_plan[index]),success=False,
                reason='skipped_speed_group',trigger_reason=reason,triggering_attempt_id=trigger['attempt_id'],evidence=evidence))
        self.index=max(self.index,end); self._reset_trial()
        self.results_revision+=1
        self.state='GROUP_RECOVER'; self.reason=reason
        self.group_recovery_started=now; self.group_recovery_request_stamp=None; self.group_recovery_ack_stamp=None
        self.group_forward_since=None;self.group_iq_samples=0;self.group_iq_stamp=None
        self.handoff=None
        return self._request(now,obs,1)

    def fault(self,reason):
        if self.handoff is not None and self.state in ('UNLOAD','HOLD'):
            self.handoff.update(failure=reason,hold_confirmed=False)
            self.trials[-1]['handoff']=dict(self.handoff);self.results_revision+=1
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
        iq_values=[s['avg_iq'] for s in self.active_samples]
        cumulative=c.stage=='speed_hold' and c.hold_sampling=='cumulative'
        segments=[]
        if cumulative:
            for segment in self.hold_segments:
                segment=dict(segment)
                offset=segment.pop('sample_offset')
                values=currents[offset:offset+segment['samples']]
                q_values=iq_values[offset:offset+segment['samples']]
                segment.update(current_signal="avg_iq",mean_current_motor=mean(values),std_current_motor=pstdev(values),
                    mean_iq=mean(q_values),std_iq=pstdev(q_values))
                if segment['end_reason']=='sampling': segment['end_reason']=reason
                segments.append(segment)
        result=dict(current_signal="avg_iq",trial_id=len(self.trials)+1,plan_index=self.index,stage=c.stage,
            requested_target=self.plan[self.index],region=self.region,success=success,reason=reason,
            first_current_ack_stamp=self.phase_ack_stamp,evidence_valid=True,
            start_stamp=self.started,end_stamp=ended,start_speed=self.start_velocity,end_speed=velocity,
            duration_s=self.hold_elapsed if cumulative else duration,samples=len(currents),mean_current_motor=mean(currents) if currents else None,
            std_current_motor=pstdev(currents) if currents else None,
            mean_iq=mean(iq_values) if iq_values else None,std_iq=pstdev(iq_values) if iq_values else None,
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
        if self.protection and c.stage!='speed_hold':
            result.update(exit_prediction=self.exit_prediction,
                pulse_mean_acceleration=(velocity-self.start_velocity)/duration if duration is not None and duration>0 else None,
                interval_completed=success,protective_flow=True,
                pulse_valid=reason in ('completed','predictive_exit') and duration is not None and duration>0 and len(currents)>=2)
        self.trials.append(result)
        if self.full_step:self._full_step_record(result,success,reason,last)

    def _begin_handoff(self,now,obs,reason,advance=True,turn=False):
        target=self.config.recovery_speed if turn else self._item().end_speed
        current=0. if turn else self.protection.holding_current(target)
        if not isinstance(current,(int,float)) or not math.isfinite(current) or abs(current)>self.config.max_current:
            self.fault('invalid_holding_current');return self._request(now,obs,1)
        self.handoff=dict(started=now,target_speed=target,target_current=current,advance=advance,
            turn=turn,ack_stamp=None,last_iq_stamp=None,iq_samples=0,quiet_since=None,
            last_hold_wheel=None,hold_confirmed=False,reason=reason)
        self.trials[-1]['handoff']=dict(self.handoff)
        self.started=None;self.active_samples=[]
        self.state='UNLOAD';self.reason=reason
        return self._request(now,obs,1,current=current)

    def _handoff_tick(self,now,obs,echo,pair):
        h=self.handoff;s=self.protection.settings
        # A straight exit during normal unloading must remove holding current too.
        if not obs.straight and not h['turn']:
            h.update(turn=True,target_speed=self.config.recovery_speed,target_current=0.,
                started=now,ack_stamp=None,last_iq_stamp=None,iq_samples=0,quiet_since=None)
            self.state='UNLOAD';self.results_revision+=1
        if self.state=='UNLOAD':
            if self.full_step and now-h['started']>s.unload_timeout+2*math.ulp(now):
                self.fault('iq_unload_timeout');return self._request(now,obs,1)
            matching=echo.mode==1 and math.isclose(echo.current,h['target_current'],abs_tol=1e-5) and echo.speed==0 and echo.stamp>=h['started']
            if matching and h['ack_stamp'] is None:h['ack_stamp']=echo.stamp
            if not matching:h['iq_samples']=0
            after_ack=h['ack_stamp'] is not None and (obs.telemetry_stamp>h['ack_stamp'] if self.full_step else obs.telemetry_stamp>=h['ack_stamp'])
            if after_ack and (h['last_iq_stamp'] is None or obs.telemetry_stamp>h['last_iq_stamp']):
                if h['last_iq_stamp'] is not None and obs.telemetry_stamp-h['last_iq_stamp']>s.source_gap:h['iq_samples']=0
                h['last_iq_stamp']=obs.telemetry_stamp
                h['iq_samples']=h['iq_samples']+1 if matching and abs(obs.avg_iq-h['target_current'])<=s.iq_tolerance else 0
                if h['iq_samples']>=s.iq_confirm_samples:
                    h['actual_iq_confirm_stamp']=obs.telemetry_stamp
                    self.state='HOLD';h['hold_started']=now;h['quiet_since']=None
                    self.results_revision+=1
            if self.state=='UNLOAD' and now-h['started']>=s.unload_timeout:
                self.fault('iq_unload_timeout');return self._request(now,obs,1)
            self.trials[-1]['handoff']=dict(h)
            if self.state=='UNLOAD':return self._request(now,obs,1,current=h['target_current'])
        if self.full_step and now-h['hold_started']>s.hold_timeout+2*math.ulp(now):
            self.fault('post_pulse_hold_timeout');return self._request(now,obs,1)
        matching=echo.mode==0 and math.isclose(echo.speed,h['target_speed'],abs_tol=1e-5) and echo.stamp>=h['hold_started']
        if matching and not self.full_step:
            filtered=self.protection.acceleration;instant=self.protection.instant_acceleration
            # Require directional agreement; an old filtered derivative alone
            # must not trip after wheel speed has already settled.
            if filtered is not None and instant is not None and filtered*instant>0 and abs(instant)>s.quiet_acceleration:
                predicted=obs.velocity+filtered*(s.exit_horizon+max(0.,now-obs.wheel_stamp))
                if (filtered>0 and predicted>h['target_speed']+self.config.stable_tolerance or
                        filtered<0 and predicted<h['target_speed']-self.config.stable_tolerance):
                    h['hold_prediction']=dict(predicted_velocity=predicted,wheel_acceleration=filtered,wheel_stamp=obs.wheel_stamp)
                    self.fault('post_pulse_predicted_excursion');return self._request(now,obs,1)
        if not matching:h['quiet_since']=None
        if h['last_hold_wheel'] is not None and obs.wheel_stamp-h['last_hold_wheel']>s.source_gap:h['quiet_since']=None
        fresh_hold=(h['last_hold_wheel'] is None or obs.wheel_stamp>h['last_hold_wheel']) and obs.wheel_stamp>=h['hold_started']
        if fresh_hold:
            h['last_hold_wheel']=obs.wheel_stamp
            quiet=matching and abs(obs.velocity-h['target_speed'])<=self.config.stable_tolerance and self.protection.acceleration is not None and abs(self.protection.acceleration)<=s.quiet_acceleration
            if quiet:
                if h['quiet_since'] is None:h['quiet_since']=obs.wheel_stamp
                if obs.wheel_stamp-h['quiet_since']>=self.config.stable_seconds:
                    h.update(hold_confirmed=True,hold_confirmed_stamp=obs.wheel_stamp)
                    self.trials[-1]['handoff']=dict(h);self.results_revision+=1
                    self.decel_owner_index=None
                    if h['advance']:self._advance_current(h['reason'])
                    else:self._reset_trial();self.state='RECOVER';self.reason=h['reason']
                    self.handoff=None
                    return self._next_current_request(now,obs)
            else:h['quiet_since']=None
        if now-h['hold_started']>=s.hold_timeout:
            self.fault('post_pulse_hold_timeout');return self._request(now,obs,1)
        self.trials[-1]['handoff']=dict(h)
        return self._request(now,obs,speed=h['target_speed'])

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
                math.isfinite(v) for v in (obs.velocity,obs.avg_iq,obs.current_motor,obs.voltage_input,obs.erpm,obs.steering))
            if not ready or not echo_ok or echo.stamp<=self.armed_at:
                return self._request(now,obs)
            self.selected=True
        if not authority:
            self.fault('authority_lost_or_stale'); return self._request(now,obs,1)
        reason=''
        if not self._fresh(now,obs.rc_stamp) or not obs.rc_valid: reason='rc_invalid_or_stale'
        elif not self._fresh(now,obs.wheel_stamp) or not math.isfinite(obs.velocity): reason='wheel_invalid_or_stale'
        elif not self._fresh(now,obs.telemetry_stamp) or not all(math.isfinite(v) for v in
            (obs.avg_iq,obs.current_motor,obs.voltage_input,obs.erpm)): reason='telemetry_invalid_or_stale'
        elif c.steering_source=='pp' and obs.tracking_reason: reason=obs.tracking_reason
        elif not math.isfinite(obs.steering): reason='steering_invalid'
        elif obs.velocity<0. and (c.stage!='decel_current' or self.decel_owner_index is None and self.state!='GROUP_RECOVER'): reason='reverse_motion'
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
        if self.protection:
            self.protection.observe(obs.wheel_stamp,obs.velocity)
            if not self.full_step and obs.velocity>c.max_speed+self.protection.settings.overspeed_margin:
                self.fault('wheel_overspeed');return self._request(now,obs,1)
        if self.full_step:
            return self._full_step_tick(now,obs,echo,pair,unique)
        if self.state=='GROUP_RECOVER':
            # A single negative sample must not cascade into skipped lower speeds.
            # Wait for selected neutral current and fresh forward/stationary sources.
            neutral=echo.mode==1 and echo.current==0 and echo.speed==0
            if neutral and self.group_recovery_request_stamp is not None and echo.stamp>=self.group_recovery_request_stamp and self.group_recovery_ack_stamp is None:
                self.group_recovery_ack_stamp=echo.stamp
            iq_ready=True
            if self.protection:
                s=self.protection.settings
                if not neutral:self.group_iq_samples=0
                if neutral and self.group_recovery_ack_stamp is not None and obs.telemetry_stamp>=self.group_recovery_ack_stamp and (self.group_iq_stamp is None or obs.telemetry_stamp>self.group_iq_stamp):
                    self.group_iq_stamp=obs.telemetry_stamp
                    self.group_iq_samples=self.group_iq_samples+1 if abs(obs.avg_iq)<=s.iq_tolerance else 0
                iq_ready=neutral and self.group_iq_samples>=s.iq_confirm_samples
                if not iq_ready and now-self.group_recovery_started>=s.unload_timeout:
                    self.fault('iq_unload_timeout');return self._request(now,obs,1)
            if iq_ready and obs.velocity>=0 and self.group_recovery_ack_stamp is not None and min(pair)>=self.group_recovery_ack_stamp:
                if self.group_forward_since is None: self.group_forward_since=obs.wheel_stamp
                if obs.wheel_stamp-self.group_forward_since>=c.stable_seconds:
                    self.decel_owner_index=None
                    self.state='COMPLETE' if self.index==len(self.plan) else 'PREPARE'
                    return self._next_current_request(now,obs)
            else: self.group_forward_since=None
            if now-self.group_recovery_started>=c.trial_timeout:
                self.fault('decel_group_recovery_timeout')
            return self._request(now,obs,1)
        if c.stage=='decel_current' and obs.velocity<0:
            return self._skip_decel_group(now,obs,'reverse_motion')
        if self.protection:
            settings=self.protection.settings
            if c.stage=='decel_current' and settings.reverse_guard_enabled and obs.velocity<=settings.reverse_guard and self.state in ('APPLY','MEASURE','UNLOAD','HOLD'):
                self.fault('near_reverse');return self._request(now,obs,1)
            if self.state in ('UNLOAD','HOLD'):return self._handoff_tick(now,obs,echo,pair)
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
            if self.protection:
                self._finish(False,'straight_region_exit',dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
                return self._begin_handoff(now,obs,'straight_region_exit',advance=self.attempt_number>=self.tracking_config.max_attempts,turn=True)
            self._straight_exit(obs)
            if self.state=='COMPLETE': return self._request(now,obs)
            return self._request(now,obs,speed=c.recovery_speed if c.steering_source=='pp' else self._item().start_speed)
        if boundary and not (unique and phase_echo and acknowledged_sources and same_region):
            if not phase_echo: reason='boundary_before_current_echo'
            elif not unique: reason='boundary_without_new_telemetry' if pair[1]<=self.last_pair[1] else 'boundary_without_new_wheel'
            elif not acknowledged_sources: reason='boundary_before_selected_current_observation'
            else: reason='straight_region_exit'
            self._finish(False,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
            if self.protection:return self._begin_handoff(now,obs,reason)
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
                if abs(obs.velocity-target)<=c.stable_tolerance and (not self.protection or self.protection.acceleration is not None and abs(self.protection.acceleration)<=self.protection.settings.quiet_acceleration) and echo.mode==0 and math.isclose(echo.speed,target,abs_tol=1e-5):
                    if self.stable_since is None: self.stable_since=obs.wheel_stamp
                    if obs.wheel_stamp-self.stable_since>=c.stable_seconds:
                        self.state='MEASURE' if c.stage=='speed_hold' else 'APPLY'
                        self.phase_started=obs.wheel_stamp
                        if c.stage!='speed_hold': self._begin_attempt()
                        if self.protection:
                            try:
                                prediction=self.protection.prediction(self._item(),now,obs)
                                reason='insufficient_exit_headroom' if prediction['exits'] else ''
                                self.exit_prediction=prediction
                            except ValueError as exc:reason=str(exc)
                            if reason:
                                self._finish(False,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
                                self._advance_current(reason)
                                return self._next_current_request(now,obs)
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
        predictive=False
        if self.protection and current_phase:
            try:self.exit_prediction=self.protection.prediction(self._item(),now,obs)
            except ValueError as exc:
                self.fault(str(exc));return self._request(now,obs,1)
            predictive=self.exit_prediction['exits']
        if current_phase and not boundary and phase_echo and acknowledged_sources:
            # B feedback remains diagnostic; C abandons the current speed group
            # only after a complete supported current-feedback failure window.
            if self._track_current(obs) and c.stage=='decel_current':
                return self._skip_decel_group(now,obs,'current_tracking_not_achieved')
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
            if reached or timeout or predictive:
                valid_response=reached and (c.stage=='speed_hold' or elapsed>0)
                reason='completed' if valid_response else ('interval_without_positive_duration' if reached else ('predictive_exit' if predictive else 'trial_timeout'))
                self._finish(valid_response,reason,row)
                if self.protection:return self._begin_handoff(now,obs,reason)
                self.index+=1; self._reset_trial()
                self.state='COMPLETE' if self.index==len(self.plan) else 'PREPARE'
                self.reason=reason
                if c.stage=='speed_hold':
                    return self._request(now,obs,speed=self.plan[self.index]) if self.state=='PREPARE' else self._request(now,obs)
                return self._next_current_request(now,obs)
        if self.protection and current_phase and predictive:
            self._finish(False,'predictive_exit_unverified',dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
            return self._begin_handoff(now,obs,'predictive_exit_unverified')
        if current_phase and not boundary and self.first_current_request_stamp is not None and now-self.first_current_request_stamp>=c.trial_timeout:
            self._finish(False,'trial_timeout',dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
            if self.protection:return self._begin_handoff(now,obs,'trial_timeout')
            self._advance_current('trial_timeout')
            return self._next_current_request(now,obs)
        return requested
