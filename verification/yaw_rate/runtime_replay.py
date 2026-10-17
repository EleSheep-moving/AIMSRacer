#!/usr/bin/env python3
"""1x receipt-scheduled replay into standalone yaw ROS node, not hardware latency."""
import argparse,json,os,subprocess,time,hashlib
from pathlib import Path
import numpy as np
os.environ.update(ROS_DOMAIN_ID='195',ROS_LOCALHOST_ONLY='1')
import rclpy
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data

def stamp(h):return h.sec*10**9+h.nanosec
def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--executable',type=Path,required=True)
 p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=20.)
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 # NpzFile lazily decompresses on each __getitem__; materialize before timing.
 with np.load(a.input,allow_pickle=False) as archive:
  z={k:archive[k] for k in ('corrected','corrected_ts','corrected_cov')}
 ts=z['corrected_ts'];origin=1792002365474876930
 begin=origin+round(41.926416916*1e9);ids=np.flatnonzero((ts[:,0]>=begin)&(ts[:,0]<begin+round(a.duration*1e9)))
 if len(ids)<100:raise ValueError('insufficient replay data')
 rclpy.init();node=rclpy.create_node('yaw_receipt_replay');rates=[];alphas=[];statuses=[];sent=[];schedule=[]
 def rate(m):rates.append([stamp(m.header.stamp),m.angular_velocity.z,m.angular_velocity_covariance[8]])
 def alpha(m):alphas.append([stamp(m.header.stamp),m.accel.accel.angular.z,m.accel.covariance[35]])
 def status(m):statuses.append({'receipt_steady_ns':time.monotonic_ns(),'values':{v.key:v.value for v in m.status[0].values}})
 subs=[node.create_subscription(Imu,'/state_estimation/yaw_rate',rate,1000),node.create_subscription(AccelWithCovarianceStamped,'/state_estimation/yaw_acceleration',alpha,1000),node.create_subscription(DiagnosticArray,'/state_estimation/yaw_rate/status',status,100)]
 pub=node.create_publisher(Imu,'/livox/imu_bias_corrected',qos_profile_sensor_data)
 log=(a.output/'node.log').open('w');proc=subprocess.Popen([str(a.executable)],stdout=log,stderr=subprocess.STDOUT)
 def pump():
  if proc.poll() is not None:raise RuntimeError('node exited')
  rclpy.spin_once(node,timeout_sec=0)
 def until(t):
  while time.monotonic_ns()<t:pump();time.sleep(min(.0005,max(0.,(t-time.monotonic_ns())*1e-9)))
 try:
  end=time.monotonic()+10
  while not pub.get_subscription_count():
   if time.monotonic()>end:raise RuntimeError('DDS discovery timeout')
   pump();time.sleep(.005)
  until(time.monotonic_ns()+1_000_000_000)
  base=time.monotonic_ns();ros=node.get_clock().now().nanoseconds;first=int(ts[ids[0],0])
  expected=[];previous=None;consecutive=0
  for i in ids:
   receipt,source=map(int,ts[i]);due=base+receipt-first;until(due)
   if previous is not None and source-previous>30_000_000:consecutive=0
   previous=source;consecutive+=1
   m=Imu();m.header.frame_id='livox_frame';ns=ros+source-first;m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9)
   m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,z['corrected'][i,3:6])
   m.angular_velocity_covariance=z['corrected_cov'][i,9:18].tolist()
   if consecutive>=3:expected.append(ns)
   pub.publish(m);sent.append(ns);schedule.append((time.monotonic_ns()-due)*1e-6);pump()
  replay_end=base+int(ts[ids[-1],0])-first;until(time.monotonic_ns()+300_000_000)
  subscriptions={n for n,_ in node.get_subscriber_names_and_types_by_node('yaw_rate','/')}
  pubs={n for n,_ in node.get_publisher_names_and_types_by_node('yaw_rate','/')}
  stamps=[int(x[0]) for x in rates];astamps=[int(x[0]) for x in alphas]
  checks={'complete_expected_source_sequence':stamps==expected,'monotonic_source':all(y>x for x,y in zip(stamps,stamps[1:])),
   'exact_source_subset':set(stamps)<=set(sent),'alpha_same_sources':stamps==astamps,
   'finite_rate_alpha_covariance':bool(rates and alphas and np.isfinite(np.asarray(rates)[:,1:]).all() and np.isfinite(np.asarray(alphas)[:,1:]).all() and min(x[2] for x in rates)>=0 and min(x[2] for x in alphas)>=0),
   'independent_graph':subscriptions <= {'/livox/imu_bias_corrected','/clock','/parameter_events'} and not pubs.intersection({'/tf','/tf_static','/drive','/vesc/command','/odometry/filtered','/state_estimation/body_acceleration'})}
  fields=[s['values'] for s in statuses if base<=s['receipt_steady_ns']<=replay_end]
  report={'checks':checks,'sent':len(sent),'expected_outputs':len(expected),'rate_outputs':len(rates),'alpha_outputs':len(alphas),
   'max_reported_callback_window_p99_ms':max((float(x.get('callback_window_p99_ms',0)) for x in fields),default=None),
   'scheduling_p99_ms':float(np.percentile(schedule,99)),'boundary':__doc__,
   'timing_definition':'maximum rolling <=512 callback window P99 during replay; includes publication; not full-run P99',
   'input_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'executable_sha256':hashlib.sha256(a.executable.read_bytes()).hexdigest(),
   'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'subscriptions':sorted(subscriptions),'publishers':sorted(pubs),'statuses':statuses}
  (a.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
  # Keep epoch nanoseconds out of mixed floating-point arrays.
  np.savez_compressed(a.output/'outputs.npz',rate_stamps_ns=np.asarray(stamps,dtype=np.int64),
      alpha_stamps_ns=np.asarray(astamps,dtype=np.int64),
      rate=np.asarray([x[1:] for x in rates]),alpha=np.asarray([x[1:] for x in alphas]),
      sent=np.asarray(sent,dtype=np.int64))
  assert all(checks.values()),checks
  print(json.dumps({k:v for k,v in report.items() if k!='statuses'},indent=2))
 finally:
  proc.terminate();proc.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
