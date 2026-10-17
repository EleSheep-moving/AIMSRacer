"""Evidence-bound longitudinal sweep planning, independent of ROS."""
from dataclasses import dataclass
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path


BASELINE_SPEEDS = tuple(1. + .5*i for i in range(9))


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _sha(value):
    try:
        raw=json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    except (TypeError, ValueError) as exc:
        raise ValueError('noncanonical evidence') from exc
    return hashlib.sha256(raw).hexdigest()


@dataclass(frozen=True)
class TrialPlan:
    stage: str
    start_speed: float
    end_speed: float
    current: float
    repeat: int = 1
    kind: str = 'acceleration'
    baseline_current: float = None
    current_signal: str = 'avg_iq'

    def __post_init__(self):
        if self.current_signal!='avg_iq': raise ValueError('current plan requires avg_iq')
        if self.stage not in ('accel_interval', 'decel_current'):
            raise ValueError('structured trial requires a current stage')
        if not all(_number(v) for v in (self.start_speed,self.end_speed,self.current)):
            raise ValueError('nonfinite trial plan')
        if not (0<self.start_speed<=5 and 0<self.end_speed<=5 and abs(self.current)<=100):
            raise ValueError('trial bounds exceeded')
        if type(self.repeat) is not int or self.repeat<1:
            raise ValueError('invalid trial repeat')
        baseline=self.baseline_current
        if baseline is not None and (not _number(baseline) or not 0<baseline<=100):
            raise ValueError('invalid baseline current')
        if self.stage=='accel_interval':
            if self.kind!='acceleration' or self.end_speed<=self.start_speed or self.current<=0:
                raise ValueError('invalid acceleration trial')
            if baseline is not None and self.current<=baseline:
                raise ValueError('acceleration current must exceed baseline')
        else:
            if self.end_speed>=self.start_speed:
                raise ValueError('invalid deceleration interval')
            if self.current>0:
                if self.kind!='positive_deceleration' or baseline is None or self.current>=baseline:
                    raise ValueError('positive deceleration requires known higher baseline')
            elif self.current==0:
                if self.kind!='coast': raise ValueError('zero current requires coast trial')
            elif self.kind!='braking': raise ValueError('negative current requires braking trial')


def _validate_baseline(value):
    if not isinstance(value,dict) or type(value.get('schema_version')) is not int or value.get('schema_version') not in (2,3) or value.get('current_signal')!='avg_iq' or value.get('kind')!='aims_current_baseline' or value.get('stage')!='speed_hold':
        raise ValueError('invalid baseline schema')
    content={k:v for k,v in value.items() if k!='baseline_sha256'}
    if value.get('baseline_sha256')!=_sha(content):
        raise ValueError('baseline hash mismatch')
    source=value.get('source')
    if not isinstance(source,dict) or type(source.get('schema_version')) is not int or source.get('schema_version')!=1:
        raise ValueError('invalid source provenance')
    for name in ('sha256','configuration_sha256','metadata_sha256'):
        digest=source.get(name)
        if not isinstance(digest,str) or len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError('missing source '+name)
    context=source.get('context')
    if not isinstance(context,dict) or context.get('evidence_valid') is not True or context.get('current_signal')!='avg_iq' or _sha(context)!=source['metadata_sha256'] or context.get('configuration_sha256')!=source['configuration_sha256']:
        raise ValueError('invalid source metadata')
    points=value.get('points')
    if not isinstance(points,list) or len(points)!=len(BASELINE_SPEEDS):
        raise ValueError('incomplete baseline speed coverage')
    seen=set()
    for point in points:
        if not isinstance(point,dict): raise ValueError('invalid baseline point')
        speed=point.get('speed'); current=point.get('mean_iq'); std=point.get('std_iq')
        if not _number(speed) or speed not in BASELINE_SPEEDS or speed in seen:
            raise ValueError('invalid or duplicate baseline speed')
        if not _number(current) or not 0<current<=100 or not _number(std) or std<0:
            raise ValueError('invalid baseline current statistics')
        if type(point.get('samples')) is not int or point['samples']<1 or not _number(point.get('duration_s')) or point['duration_s']<=0:
            raise ValueError('invalid baseline sample coverage')
        voltage=point.get('voltage_input')
        if voltage is not None and (not _number(voltage) or voltage<=0):
            raise ValueError('invalid baseline voltage')
        seen.add(speed)
    if value['schema_version']==3:
        _validate_polynomial(value.get('planning_model'))
    elif 'planning_model' in value:
        raise ValueError('polynomial baseline requires schema v3')
    return value


def _polynomial_current(model,speed):
    low,high=model['speed_domain_mps']
    if not _number(speed) or not low<=speed<=high:
        raise ValueError('speed outside polynomial baseline domain')
    current=0.
    for coefficient in reversed(model['coefficients']):
        current=current*speed+coefficient
    return current


def _validate_polynomial(model):
    if (not isinstance(model,dict) or model.get('kind')!='cubic_polynomial' or
            model.get('current_signal')!='avg_iq' or model.get('coefficient_order')!='ascending' or
            model.get('speed_unit')!='m/s' or model.get('current_unit')!='A' or
            model.get('speed_domain_mps')!=[1.,5.]):
        raise ValueError('invalid polynomial baseline model')
    coefficients=model.get('coefficients')
    if not isinstance(coefficients,list) or len(coefficients)!=4 or not all(_number(c) for c in coefficients):
        raise ValueError('four finite cubic coefficients required')
    # Check extrema too: valid grid values alone cannot rule out an invalid
    # current between grid points inside the stated domain.
    _,c1,c2,c3=coefficients
    scale=max(abs(c1),abs(c2),abs(c3),1.)
    c1,c2,c3=c1/scale,c2/scale,c3/scale
    critical=[]
    if c3:
        discriminant=(2*c2)**2-12*c3*c1
        if discriminant>=0:
            critical=[(-2*c2+sign*math.sqrt(discriminant))/(6*c3) for sign in (-1,1)]
    elif c2:
        critical=[-c1/(2*c2)]
    for speed in [*BASELINE_SPEEDS,*[v for v in critical if 1.<=v<=5.]]:
        current=_polynomial_current(model,speed)
        if not _number(current) or not 0<current<=100:
            raise ValueError('polynomial baseline current outside bounds')


def with_polynomial_baseline(baseline,coefficients):
    """Select an explicit planning model without relabeling measured A points."""
    _validate_baseline(baseline)
    value=deepcopy(baseline)
    value.update(schema_version=3,planning_model=dict(kind='cubic_polynomial',
        current_signal='avg_iq',coefficient_order='ascending',coefficients=list(coefficients),
        speed_domain_mps=[1.,5.],speed_unit='m/s',current_unit='A'))
    _validate_polynomial(value['planning_model'])
    value['baseline_sha256']=_sha({k:v for k,v in value.items() if k!='baseline_sha256'})
    return _validate_baseline(value)


def export_baseline(payload, source_path=None):
    """Export all nine accepted A speed points with immutable source provenance."""
    if not isinstance(payload,dict) or type(payload.get('schema_version',1)) is not int or payload.get('schema_version',1)!=1:
        raise ValueError('A source must use result schema v1')
    config=payload.get('configuration'); metadata=payload.get('metadata')
    if not isinstance(config,dict) or not isinstance(metadata,dict):
        raise ValueError('missing A configuration or metadata')
    speeds=config.get('speeds'); repeats=config.get('repeats')
    if config.get('stage')!='speed_hold' or type(repeats) is not int or repeats!=1 or not isinstance(speeds,(list,tuple)) or len(speeds)!=9 or not all(_number(s) for s in speeds) or sorted(speeds)!=list(BASELINE_SPEEDS):
        raise ValueError('A source must cover nine speeds once')
    minimum=config.get('measurement_seconds')
    if not _number(minimum) or minimum<=0 or metadata.get('evidence_valid') is not True or metadata.get('current_signal')!='avg_iq' or metadata.get('configuration_sha256')!=_sha(config):
        raise ValueError('invalid A evidence metadata')
    trials=payload.get('trials')
    if not isinstance(trials,list) or len(trials)!=9:
        raise ValueError('missing or duplicate A trials')
    points=[]; seen=set()
    for row in trials:
        if not isinstance(row,dict): raise ValueError('invalid A trial')
        if row.get('current_signal','avg_iq')!='avg_iq':
            raise ValueError('conflicting A trial current signal')
        segments=row.get('segments',[])
        if not isinstance(segments,list) or any(not isinstance(segment,dict) or
                segment.get('current_signal','avg_iq')!='avg_iq' for segment in segments):
            raise ValueError('conflicting A segment current signal')
        index=row.get('plan_index'); speed=row.get('requested_target')
        if type(index) is not int or not 0<=index<9 or index in seen or not _number(speed) or speed!=config['speeds'][index]:
            raise ValueError('invalid A plan coverage')
        if row.get('stage')!='speed_hold' or row.get('success') is not True or row.get('evidence_valid') is not True or not _number(row.get('duration_s')) or row['duration_s']+1e-9<minimum:
            raise ValueError('A trial is not accepted')
        points.append(dict(speed=speed,mean_iq=row.get('mean_iq'),
            std_iq=row.get('std_iq'),samples=row.get('samples'),
            duration_s=row['duration_s'],source_plan_index=index,source_trial_id=row.get('trial_id'),
            voltage_input=row.get('mean_voltage_input',row.get('voltage_input'))))
        seen.add(index)
    points.sort(key=lambda p:p['speed'])
    raw_digest=_sha(payload)
    path=None
    if source_path is not None:
        path=str(Path(source_path))
        raw=Path(source_path).read_bytes()
        try: source_payload=json.loads(raw)
        except (ValueError,UnicodeError) as exc: raise ValueError('invalid source file') from exc
        if _sha(source_payload)!=_sha(payload): raise ValueError('source file does not match payload')
        raw_digest=hashlib.sha256(raw).hexdigest()
    value=dict(schema_version=2,current_signal='avg_iq',kind='aims_current_baseline',stage='speed_hold',
        source=dict(path=path,sha256=raw_digest,configuration_sha256=_sha(config),
            metadata_sha256=_sha(metadata),schema_version=1,context=metadata.copy()),points=points)
    value['baseline_sha256']=_sha(value)
    return _validate_baseline(value)


def load_baseline(path,raw=None):
    try: value=json.loads(Path(path).read_bytes() if raw is None else raw)
    except (ValueError,UnicodeError) as exc: raise ValueError('invalid baseline JSON') from exc
    return _validate_baseline(value)


def build_sweep_plan(stage, speeds, baseline, current_step=5., max_current=100., repeats=1):
    """Freeze B/C currents from measured A points or an explicit planning model."""
    _validate_baseline(baseline)
    if stage not in ('accel_interval','decel_current'):
        raise ValueError('sweep requires B or C stage')
    if not _number(current_step) or current_step<=0 or not _number(max_current) or not 0<max_current<=100:
        raise ValueError('invalid current grid')
    if type(repeats) is not int or repeats<1: raise ValueError('invalid repeats')
    speeds=list(speeds) if speeds is not None else list(BASELINE_SPEEDS if stage=='accel_interval' else reversed(BASELINE_SPEEDS[1:]))
    ascending=speeds==sorted(set(speeds))
    descending=speeds==sorted(set(speeds),reverse=True)
    if not speeds or any(not _number(s) or s not in BASELINE_SPEEDS for s in speeds) or not (ascending or stage=='decel_current' and descending):
        raise ValueError('invalid or uncovered sweep speeds')
    if stage=='accel_interval':
        if len(speeds)<2 or any(not math.isclose(b-a,.5,abs_tol=1e-9) for a,b in zip(speeds,speeds[1:])):
            raise ValueError('B requires half metre per second boundaries')
        intervals=list(zip(speeds,speeds[1:]))
    else:
        if any(s<1.5 for s in speeds): raise ValueError('C start speeds must exceed one metre per second')
        intervals=[(s,s-.5) for s in speeds]
    model=baseline.get('planning_model')
    lookup={p['speed']:(_polynomial_current(model,p['speed']) if model else p['mean_iq'])
            for p in baseline['points']}
    result=[]
    def add(start,end,current,kind,base):
        for repeat in range(1,repeats+1):
            result.append(TrialPlan(stage,start,end,current,repeat,kind,base))
    for start,end in intervals:
        base=lookup[start]
        if stage=='accel_interval':
            if base+current_step>max_current: raise ValueError('baseline leaves no first acceleration increment')
            k=1
            while base+k*current_step<=max_current:
                add(start,end,base+k*current_step,'acceleration',base); k+=1
            if not result or result[-1].start_speed!=start or result[-1].current!=max_current:
                add(start,end,max_current,'acceleration',base)
        else:
            k=1
            while base-k*current_step>0:
                current=base-k*current_step
                if current>max_current: raise ValueError('positive C grid exceeds limit')
                add(start,end,current,'positive_deceleration',base); k+=1
            add(start,1.,0.,'coast',base)
            k=1
            while k*current_step<=max_current:
                add(start,1.,-k*current_step,'braking',base); k+=1
    return result


def build_full_step_plan(stage, baseline, start_speed=None, end_speed=None,
                         current_step=5., max_current=100., repeats=1):
    """Freeze independent constant-current B/C trials over a full speed range.

    Measured schema-2 baselines cover their recorded speed points; an explicit
    schema-3 polynomial permits any endpoint within its validated domain.
    """
    _validate_baseline(baseline)
    if stage not in ('accel_interval', 'decel_current'):
        raise ValueError('full step requires B or C stage')
    if (not _number(current_step) or current_step <= 0 or
            not _number(max_current) or not 0 < max_current <= 100):
        raise ValueError('invalid current grid')
    if type(repeats) is not int or repeats < 1:
        raise ValueError('invalid repeats')
    accelerating = stage == 'accel_interval'
    start = (1. if accelerating else 5.) if start_speed is None else start_speed
    end = (5. if accelerating else 1.) if end_speed is None else end_speed
    if not all(_number(v) and 1. <= v <= 5. for v in (start, end)):
        raise ValueError('invalid or uncovered full-step speeds')
    if (end <= start if accelerating else end >= start):
        raise ValueError('invalid full-step speed direction')
    model = baseline.get('planning_model')
    lookup = {p['speed']: p['mean_iq'] for p in baseline['points']}
    if model:
        base = _polynomial_current(model, start)
        _polynomial_current(model, end)
    else:
        if start not in lookup or end not in lookup:
            raise ValueError('uncovered full-step speed')
        base = lookup[start]
    currents = []
    if accelerating:
        if base + current_step > max_current:
            raise ValueError('baseline leaves no first acceleration increment')
        k = 1
        while base + k * current_step < max_current:
            currents.append((base + k * current_step, 'acceleration'))
            k += 1
        currents.append((max_current, 'acceleration'))
    else:
        k = 1
        while base - k * current_step > 0:
            current = base - k * current_step
            if current > max_current:
                raise ValueError('positive C grid exceeds limit')
            currents.append((current, 'positive_deceleration'))
            k += 1
        currents.append((0., 'coast'))
        k = 1
        while k * current_step < max_current:
            currents.append((-k * current_step, 'braking'))
            k += 1
        currents.append((-max_current, 'braking'))
    return [TrialPlan(stage, start, end, current, repeat, kind, base)
            for current, kind in currents for repeat in range(1, repeats + 1)]
