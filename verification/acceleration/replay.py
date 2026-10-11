#!/usr/bin/env python3
"""Causal, receipt-ordered numerical comparison. No ROS or actuator interfaces."""
from __future__ import annotations
import argparse
from dataclasses import asdict, dataclass, replace
import csv
import hashlib
import json
from pathlib import Path
import time
from concurrent.futures import ProcessPoolExecutor
import numpy as np

G = 9.80665
HELD_OUT = ('192955_v35_n15_accel1_cap4_lap01', '193524_v35_n15_accel1_cap4_lap02')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass
class Run:
    name: str
    origin_ns: int
    start_s: float
    end_s: float
    events: list
    imu_receipt_ns: np.ndarray
    imu_source_ns: np.ndarray
    force: np.ndarray
    force_valid: np.ndarray
    gyro: np.ndarray
    wheel_source_ns: np.ndarray
    wheel_speed: np.ndarray
    lio_source_ns: np.ndarray
    lio_receipt_ns: np.ndarray
    gravity: np.ndarray
    lio_speed: np.ndarray


def load_runs(inputs: Path, forces: Path, output: Path):
    metrics = {x['run']: x for x in csv.DictReader((inputs/'field-run-metrics.csv').open())}
    windows = json.loads((inputs/'windows.json').read_text())
    force_manifest = {x['run']: x for x in json.loads((forces/'replay.json').read_text())['provenance']}
    runs, provenance = [], []
    for path in sorted((inputs/'extracts').glob('*.npz')):
        name = path.stem
        if metrics[name]['final_phase'] != 'COMPLETE':
            continue
        meta = json.loads(path.with_suffix('.json').read_text())
        assert not meta['decode_errors'] and sha(path) == meta['npz_sha256']
        force_path = forces/(name+'-force.txt')
        assert sha(force_path) == force_manifest[name]['output_sha256']
        assert force_manifest[name]['original_extract_sha256'] == meta['npz_sha256']
        original = np.load(path)
        source, raw_i, rear_i = np.intersect1d(
            original['raw_ts'][:, 1], original['rear_ts'][:, 1], return_indices=True)
        corrected = np.loadtxt(force_path, dtype=[('stamp', 'i8'), ('valid', 'i8'),
                                                 ('fx', 'f8'), ('fy', 'f8'), ('fz', 'f8')])
        assert np.array_equal(source, corrected['stamp'])
        first, last = windows[name]
        # Ten seconds of preceding data initializes all candidates using only past input.
        lower, upper = first['bag_ns']-10_000_000_000, last['bag_ns']+3_000_000_000
        mask = (original['rear_ts'][rear_i, 0] >= lower) & (original['rear_ts'][rear_i, 0] <= upper)
        receipts = original['rear_ts'][rear_i, 0][mask]
        source = source[mask]
        gyro = original['rear'][rear_i, :3][mask]
        force = corrected['fx'][mask]
        valid = corrected['valid'][mask].astype(bool)
        wh = original['wheel_ts']
        wm = (wh[:,0]>=lower)&(wh[:,0]<=upper)
        wheel_receipt, wheel_source = wh[wm,0], wh[wm,1]
        wheel_speed = original['wheel'][wm,7]
        lt = original['lio_ts']
        lm = (lt[:,0]>=lower)&(lt[:,0]<=upper)
        lio_receipt, lio_source = lt[lm,0], lt[lm,1]
        q = original['lio'][lm,3:7]
        qgood = np.isfinite(q).all(axis=1)&(abs(np.linalg.norm(q,axis=1)-1)<1e-5)
        assert np.all(qgood), 'Invalid LIO orientation in selected input'
        gravity = G*2*(q[:,0]*q[:,2]-q[:,3]*q[:,1])
        lio_speed = np.linalg.norm(original['lio'][lm,7:10],axis=1)
        assert all(np.all(np.diff(s)>0) for s in [source,wheel_source,lio_source])
        assert all(np.all(r>=s) for r,s in [(receipts,source),(wheel_receipt,wheel_source),
                                           (lio_receipt,lio_source)])
        origin = int(min(receipts[0],wheel_receipt[0],lio_receipt[0]))
        events = [(int(r),int(s),'speed',float(v),True) for r,s,v in
                  zip(wheel_receipt,wheel_source,wheel_speed)]
        events += [(int(r),int(s),'force',float(f),bool(ok)) for r,s,f,ok in
                   zip(receipts,source,force,valid)]
        events += [(int(r),int(s),'gravity',float(g),True) for r,s,g in
                   zip(lio_receipt,lio_source,gravity)]
        events.sort(key=lambda e:e[0])  # Source-time ordering is performed only after receive.
        run = Run(name,origin,(first['bag_ns']-origin)*1e-9,(last['bag_ns']-origin)*1e-9,
                  events,receipts,source,force,valid,gyro,wheel_source,wheel_speed,
                  lio_source,lio_receipt,gravity,lio_speed)
        runs.append(run)
        provenance.append({'run':name,'role':'held_out' if name in HELD_OUT else 'training',
                           'extract_sha256':meta['npz_sha256'],'force_sha256':sha(force_path),
                           'inputs':meta['inputs'],'force_rows':len(source),
                           'unavailable_force_rows':int((~valid).sum()),
                           'source_monotonic_per_stream':True,
                           'cross_stream_arrival_source_monotonic':all(
                               events[i][1]<=events[i+1][1] for i in range(len(events)-1)),
                           'clock_origin_ns':origin,
                           'lio_age_ms':np.percentile((lio_receipt-lio_source)*1e-6,[50,95,100]).tolist()})
    assert len(runs)==8 and set(HELD_OUT)<=set(r.name for r in runs)
    (output/'inputs.json').write_text(json.dumps(provenance,indent=2)+'\n')
    (output/'input-metadata-sha256.json').write_text(json.dumps({
        str(p):sha(p) for p in [inputs/'field-run-metrics.csv',inputs/'windows.json',
                               inputs/'windows-provenance.json',forces/'replay.json']},indent=2)+'\n')
    return runs


def replay(run: Run, config, measure_time=False):
    from observer import LongitudinalObserver
    observer = LongitudinalObserver(config)
    result, costs = [], []
    initialized = False
    for receipt, stamp, kind, value, valid in run.events:
        started = time.perf_counter_ns() if measure_time else 0
        admitted=False
        before_delayed=observer.counters['delayed']
        if valid:
            admitted=observer.receive(kind, stamp, value)
            initialized = admitted or initialized
        if kind == 'force' and initialized:
            estimate = observer.snapshot(receipt)
            result.append(((receipt-run.origin_ns)*1e-9, estimate.velocity, estimate.acceleration,
                           estimate.acceleration_variance,
                           estimate.disturbance if estimate.disturbance is not None else np.nan))
        if measure_time:
            costs.append((kind,admitted,observer.counters['delayed']>before_delayed,
                          (time.perf_counter_ns()-started)*1e-6))
    return np.asarray(result,dtype=float), dict(observer.counters), costs


def score(metrics):
    # Convert estimated a jitter to velocity-equivalent penalties at fixed short horizons.
    return metrics['forecast_100ms_rmse_mps']**2 + metrics['forecast_200ms_rmse_mps']**2 + \
        (.05*metrics['stationary_sigma_mps2'])**2 + (.015*metrics['active_hf_rms_mps2'])**2


def evaluate_trial(item):
    from evaluate import evaluate_run
    run, config = item
    estimates, counters, _ = replay(run, config)
    return evaluate_run(run, estimates), counters


def main():
    from observer import ObserverConfig
    from evaluate import evaluate_run, save_final
    parser=argparse.ArgumentParser()
    parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--forces',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--selected-configs',type=Path,
                        help='Reuse frozen settings without tuning (e.g. NX reproduction).')
    parser.add_argument('--jobs',type=int,default=1,help='Independent offline training workers only.')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    runs=load_runs(args.inputs,args.forces,args.output)
    training=[r for r in runs if r.name not in HELD_OUT]
    assert args.jobs>=1
    trials=[]
    pool=ProcessPoolExecutor(max_workers=args.jobs) if args.jobs>1 and not args.selected_configs else None
    def select(label, configs):
        choices=[]
        for index,config in enumerate(configs):
            rows=[]
            items=[(run,config) for run in training]
            results=pool.map(evaluate_trial,items) if pool else map(evaluate_trial,items)
            for run,(m,counters) in zip(training,results):
                rows.append(score(m))
                trials.append({'candidate':label,'index':index,'run':run.name,
                               'config':asdict(config),'score':rows[-1],'metrics':m,
                               'counters':counters})
            mean=float(np.mean(rows))
            assert np.isfinite(mean), 'Training score lacks a valid comparison window'
            choices.append((mean,index,config))
            print(label,index,'score',round(mean,8),flush=True)
        return min(choices,key=lambda x:x[:2])[2]
    if args.selected_configs:
        selected={name:ObserverConfig(**values) for name,values in
                  json.loads(args.selected_configs.read_text())['configurations'].items()}
    else:
        baseline=select('wheel',[ObserverConfig(jerk_psd=q,speed_variance=s*s)
                                for q in [.1,1.,10.,100.,1000.] for s in [.005,.02,.05,.1]])
        fused=select('wheel_imu',[replace(baseline,use_imu=True,jerk_psd=baseline.jerk_psd*f,
                                         force_variance=s*s,disturbance_psd=d)
                                 for f in [.3,1.,3.] for s in [.5,1.,2.,4.] for d in [.01,.1,1.]])
        gravity=select('wheel_imu_lio',[replace(fused,use_gravity=True,gravity_variance=s*s,
                                               disturbance_psd=d)
                                       for s in [.05,.15,.4,1.] for d in [.01,.1,1.]])
        selected={'wheel':baseline,'wheel_imu':fused,'wheel_imu_lio':gravity}
    if pool:pool.shutdown()
    (args.output/'selected-configs.json').write_text(json.dumps({
        'training':[r.name for r in training],'held_out':list(HELD_OUT),
        'selection_score':'mean(RMSE_v@100ms^2+RMSE_v@200ms^2+(0.05*sigma_stationary)^2+(0.015*RMS_hf)^2)',
        'configurations':{k:asdict(v) for k,v in selected.items()}},indent=2)+'\n')
    if trials:(args.output/'training-trials.json').write_text(json.dumps(trials,indent=2)+'\n')
    save_final(runs,selected,args.output,replay)


if __name__=='__main__':
    main()
