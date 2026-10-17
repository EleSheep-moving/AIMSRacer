#!/usr/bin/env python3
"""Read-only selective ROS2 sqlite extraction. Run in sourced ROS Humble Python."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

TOPICS={'/livox/imu':('raw',Imu),'/livox/imu_bias_corrected':('corrected',Imu),
        '/rear_axle/imu':('rear',Imu),'/rear_axle/lio_odom':('lio',Odometry)}

def stamp(s):return int(s.sec)*1_000_000_000+int(s.nanosec)
def vec(v):return [v.x,v.y,v.z]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);before=args.bag.stat();db=sqlite3.connect(args.bag.resolve().as_uri()+'?mode=ro',uri=True)
    topics={row[0]:row[1] for row in db.execute('SELECT id,name FROM topics')};ids=[i for i,n in topics.items() if n in TOPICS]
    data={name:[] for name,_ in TOPICS.values()};ts={k:[] for k in data};cov={k:[] for k in data};frames={k:set() for k in data};digest=hashlib.sha256()
    query='SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in ids)+') ORDER BY timestamp,id'
    count=0
    for tid,receipt,cdr in db.execute(query,ids):
        name,cls=TOPICS[topics[tid]];m=deserialize_message(cdr,cls);digest.update(int(tid).to_bytes(8,'little'));digest.update(int(receipt).to_bytes(8,'little'));digest.update(cdr)
        ts[name].append([receipt,stamp(m.header.stamp)]);frames[name].add(m.header.frame_id)
        if cls is Imu:
            data[name].append(vec(m.linear_acceleration)+vec(m.angular_velocity));cov[name].append(list(m.linear_acceleration_covariance)+list(m.angular_velocity_covariance)+list(m.orientation_covariance))
        else:
            q=m.pose.pose.orientation;v=m.twist.twist
            data[name].append([v.linear.x,v.linear.y,v.angular.z,m.pose.pose.position.x,m.pose.pose.position.y,q.x,q.y,q.z,q.w]);cov[name].append(list(m.pose.covariance)+list(m.twist.covariance))
        count+=1
    db.close();after=args.bag.stat()
    if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):raise RuntimeError('source changed during read')
    arrays={}
    for k in data:arrays[k]=np.asarray(data[k]);arrays[k+'_ts']=np.asarray(ts[k],dtype=np.int64);arrays[k+'_cov']=np.asarray(cov[k])
    dst=args.output/'imu-quality-input.npz';np.savez_compressed(dst,**arrays)
    meta={'source':str(args.bag),'source_before':{'bytes':before.st_size,'mtime_ns':before.st_mtime_ns},'source_after':{'bytes':after.st_size,'mtime_ns':after.st_mtime_ns},'selected_cdr_sha256':digest.hexdigest(),'selected_hash_protocol':'receipt order; topic_id uint64 little + receipt uint64 little + CDR','npz_sha256':sha(dst),'extractor_sha256':sha(Path(__file__)),'frames':{k:sorted(v) for k,v in frames.items()},'counts':{k:len(v) for k,v in data.items()},'columns':{'imu':'ax ay az wx wy wz; raw/corrected accel in g, rear SI','lio':'vx vy wz px py qx qy qz qw','imu_cov':'accel[9], gyro[9], orientation[9]','lio_cov':'pose[36], twist[36]'},'covariance_summary':{k:{'all_zero_rows':int(np.sum(np.all(np.asarray(v)==0,axis=1))),'first_diagonals':np.asarray(v)[0].tolist() if v else []} for k,v in cov.items()}}
    (args.output/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta))
if __name__=='__main__':main()
