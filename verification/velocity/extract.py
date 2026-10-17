#!/usr/bin/env python3
"""Read-only 3D IMU/raw-LIO extraction. Wheel/EKF/commands are excluded."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import numpy as np


def main():
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    before = a.bag.stat()
    db = sqlite3.connect(a.bag.resolve().as_uri()+'?mode=ro', uri=True)
    names = {'/livox/imu': ('raw', Imu), '/livox/imu_bias_corrected': ('imu', Imu),
             '/fastlio2/lio_odom': ('lio', Odometry)}
    topics = {i: name for i, name in db.execute('SELECT id,name FROM topics') if name in names}
    if set(topics.values()) != set(names):
        raise ValueError('missing required original topic')
    data, times, cov, frames, children = ({k: [] for k in ['raw','imu','lio']} for _ in range(5))
    digest = hashlib.sha256(); order = []
    query = 'SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ('+','.join('?'*len(topics))+') ORDER BY timestamp,id'
    def xyz(v): return [v.x,v.y,v.z]
    for row_id, tid, receipt, blob in db.execute(query, list(topics)):
        key, cls = names[topics[tid]]; msg = deserialize_message(blob, cls)
        source = int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
        digest.update(int(row_id).to_bytes(8,'little')); digest.update(int(tid).to_bytes(8,'little'))
        digest.update(int(receipt).to_bytes(8,'little')); digest.update(blob)
        index = len(data[key]); times[key].append([receipt,source,row_id]); frames[key].append(msg.header.frame_id)
        order.append([receipt,row_id,{'raw':0,'imu':1,'lio':2}[key],index])
        if key == 'lio':
            q = msg.pose.pose.orientation; v = msg.twist.twist
            data[key].append(xyz(msg.pose.pose.position)+[q.x,q.y,q.z,q.w]+xyz(v.linear)+xyz(v.angular))
            cov[key].append(list(msg.pose.covariance)+list(msg.twist.covariance));children[key].append(msg.child_frame_id)
        else:
            data[key].append(xyz(msg.linear_acceleration)+xyz(msg.angular_velocity))
            cov[key].append(list(msg.linear_acceleration_covariance)+list(msg.angular_velocity_covariance)+list(msg.orientation_covariance))
    db.close(); after = a.bag.stat()
    if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):
        raise RuntimeError('bag changed while reading')
    arrays = {'receipt_order':np.asarray(order,dtype=np.int64)}
    for key in data:
        arrays[key] = np.asarray(data[key]); arrays[key+'_ts'] = np.asarray(times[key],dtype=np.int64)
        arrays[key+'_cov'] = np.asarray(cov[key])
    np.savez_compressed(a.output/'input.npz', **arrays)
    h=lambda f: hashlib.sha256(f.read_bytes()).hexdigest()
    meta=dict(source=str(a.bag),bytes=before.st_size,mtime_ns=before.st_mtime_ns,
              npz_sha256=h(a.output/'input.npz'),extractor_sha256=h(Path(__file__)),
              selected_cdr_sha256=digest.hexdigest(),
              hash_protocol='receipt,id order; row_id,topic_id,receipt uint64 little then CDR',
              columns={'imu':'fx_g fy_g fz_g wx_rad_s wy_rad_s wz_rad_s',
                       'lio':'px py pz qx qy qz qw vx vy vz wx wy wz',
                       'timestamps':'receipt_ns source_ns sqlite_row_id',
                       'receipt_order':'receipt_ns sqlite_row_id kind(raw=0,imu=1,lio=2) row_index'},
              frames={k:sorted(set(v)) for k,v in frames.items()},
              child_frames={k:sorted(set(v)) for k,v in children.items()},counts={k:len(v) for k,v in data.items()})
    (a.output/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta))


if __name__ == '__main__': main()
