import csv
import math
import pytest
import rclpy
from nav_msgs.msg import Odometry
from aims_mpcc.recorder import PathRecorder
from aims_mpcc.path import prepare_recording
from test_core import recording,config


def test_recording_preserves_valid_rows_and_does_not_overwrite(tmp_path):
    target=tmp_path/'raw.csv'
    rclpy.init(args=['--ros-args','-p',f'output:={target}'])
    node=PathRecorder()
    try:
        for i in range(3):
            msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link'
            msg.header.stamp.sec=i+1;msg.pose.pose.orientation.w=1.
            msg.pose.pose.position.x=.2*i;msg.twist.twist.linear.x=.2
            node.record(msg)
        rows=list(csv.DictReader(target.open()))
        assert len(rows)==3 and float(rows[2]['x'])==.4
        assert node.count_publishers('/drive')==0
        with pytest.raises(FileExistsError):PathRecorder()
    finally:
        node.destroy_node();rclpy.shutdown()


def test_temporal_pose_jump_is_not_smoothed_into_a_valid_lap(tmp_path):
    src=tmp_path/'raw.csv';recording(src)
    rows=list(csv.DictReader(src.open()))
    # Remove an arc but compress time: geometrically smooth enough, physically a teleport.
    rows=rows[:20]+rows[26:]
    for i,row in enumerate(rows):row['timestamp']=str(i*.1)
    with src.open('w') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    with pytest.raises(ValueError,match='discontinuity'):
        prepare_recording(src,tmp_path/'out',config(),.8,.9)
