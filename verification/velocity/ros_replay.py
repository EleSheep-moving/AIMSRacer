#!/usr/bin/env python3
"""Full 1x original receipt-order replay of ONLY selected sensors, isolated domain.

A common source/receipt shift maps recorded reception time onto wall time. Source
intervals and original LIO arrival lag stay unchanged. No TF/control processes.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import rclpy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray

def main(a):
    a.output.mkdir(parents=True,exist_ok=False)
    with np.load(a.input) as z:data={k:z[k] for k in z.files}
    native=np.genfromtxt(a.expected,delimiter=',',names=True,dtype=None,encoding='utf8')
    mask=(native['event']=='IMU')&(native['accepted']==1)&(native['valid']==1)
    expected=native[mask];sequence=expected['stamp_ns'];assert np.all(np.diff(sequence)>0)
    rclpy.init();node=rclpy.create_node('vx_full_receipt_replay');received=[];statuses=[];rss=[];sent=[]
    shift=0
    def result(m):received.append((m.header.stamp.sec*10**9+m.header.stamp.nanosec-shift,m.twist.twist.linear.x,m.twist.covariance[0]))
    node.create_subscription(TwistWithCovarianceStamped,'/state_estimation/vx',result,8192)
    node.create_subscription(DiagnosticArray,'/state_estimation/vx/status',lambda m:statuses.append({x.key:x.value for x in m.status[0].values}),4096)
    imu=node.create_publisher(Imu,'/livox/imu_bias_corrected',256);lio=node.create_publisher(Odometry,'/fastlio2/lio_odom',256)
    log=(a.output/'node.log').open('w');proc=subprocess.Popen([str(a.executable)],stdout=log,stderr=subprocess.STDOUT)
    def spin():rclpy.spin_once(node,timeout_sec=0)
    def memory():
        try:
            fields=Path(f'/proc/{proc.pid}/status').read_text().splitlines();return int(next(x for x in fields if x.startswith('VmRSS:')).split()[1])
        except (OSError,StopIteration):return -1
    try:
        limit=time.monotonic()+15
        while not imu.get_subscription_count() or not lio.get_subscription_count():
            assert proc.poll() is None and time.monotonic()<limit; rclpy.spin_once(node,timeout_sec=.02)
        order=data['receipt_order'];order=order[order[:,2]!=0]
        first=int(order[0,0]);start_wall=time.time_ns()+1_000_000_000;start_mono=time.monotonic_ns()+1_000_000_000;shift=start_wall-first
        last_sample=-1
        for receipt,row,kind,index in order:
            target=start_mono+int(receipt)-first
            while True:
                remaining=target-time.monotonic_ns()
                if remaining<=0:break
                spin()
                if remaining>500_000:time.sleep(min((remaining-200_000)*1e-9,.001))
            name='imu' if kind==1 else 'lio';stamp=int(data[name+'_ts'][index,1]);v=data[name][index]
            if kind==1:
                m=Imu();m.header.frame_id='livox_frame'
                m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,v[:3]);m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,v[3:6]);publisher=imu
            else:
                m=Odometry();m.header.frame_id='odom';m.child_frame_id='livox_frame';m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,v[:3]);m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,v[3:7]);publisher=lio
            m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp+shift,10**9);publisher.publish(m)
            sent.append((name,int(receipt),stamp,(time.monotonic_ns()-target)*1e-6));spin()
            sec=(int(receipt)-first)//10**9
            if sec!=last_sample:rss.append((int(sec),memory()));last_sample=sec
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        proc.terminate();proc.wait(timeout=5);log.close();node.destroy_node();rclpy.shutdown()
    for filename,header,rows in [('output.csv',['source_ns','vx','variance'],received),('sent.csv',['event','receipt_ns','source_ns','lateness_ms'],sent),('rss.csv',['second','rss_kib'],rss)]:
        with (a.output/filename).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
    (a.output/'status.json').write_text(json.dumps(statuses,indent=2)+'\n')
    actual=np.array([v[0] for v in received],dtype=np.int64)
    exact=np.array_equal(actual,sequence);summary=dict(expected_n=len(sequence),actual_n=len(actual),complete_expected_sequence=exact,
        duration_s=(int(order[-1,0])-first)*1e-9,common_time_shift_ns=shift,
        sent_n=len(sent),max_publish_lateness_ms=max(r[3] for r in sent),rss_first_kib=rss[0][1],rss_final_kib=rss[-1][1],rss_max_kib=max(v for _,v in rss),
        status_final=statuses[-1] if statuses else {},isolated_domain=os.environ.get('ROS_DOMAIN_ID'),
        scope='1x selected original sensors over full MPCC recording; wheel/TF/controller absent')
    if exact:summary['max_vx_difference_vs_planned_receipt_order']=float(max(abs(np.array([v[1] for v in received])-expected['vx'])))
    else:
        summary['missing_source_ns']=np.setdiff1d(sequence,actual).tolist();summary['unexpected_source_ns']=np.setdiff1d(actual,sequence).tolist()
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
    assert exact,summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--expected',type=Path,required=True);p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
