"""Record original base_link poses; conversion is a separate offline step."""
import csv
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from .io import yaw_from_quaternion


class PathRecorder(Node):
    def __init__(self):
        super().__init__('record_path')
        self.declare_parameter('output','lap.csv')
        self.declare_parameter('odom_topic','/odometry/filtered')
        self.file=Path(self.get_parameter('output').value).open('x',newline='')
        self.writer=csv.writer(self.file)
        self.writer.writerow(['timestamp','x','y','yaw','speed','frame_id','child_frame_id'])
        self.last_stamp=-math.inf
        self.path=PathMsg();self.path.header.frame_id='odom'
        self.publisher=self.create_publisher(PathMsg,'/mpcc/recorded_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,self.get_parameter('odom_topic').value,self.record,qos_profile_sensor_data)
        self.timer=self.create_timer(.5,lambda:self.publisher.publish(self.path))

    def record(self,msg):
        try:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            yaw=yaw_from_quaternion(msg.pose.pose.orientation)
            row=[stamp,msg.pose.pose.position.x,msg.pose.pose.position.y,yaw,msg.twist.twist.linear.x]
            if not all(math.isfinite(x) for x in row):raise ValueError('nonfinite odometry')
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':raise ValueError('expected odom/base_link')
            if stamp<=self.last_stamp:raise ValueError('non-increasing timestamp')
            self.writer.writerow(row+['odom','base_link']);self.file.flush();self.last_stamp=stamp
            pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose
            if not self.path.poses or math.hypot(pose.pose.position.x-self.path.poses[-1].pose.position.x,
                                               pose.pose.position.y-self.path.poses[-1].pose.position.y)>.02:
                self.path.poses.append(pose)
            self.path.header.stamp=msg.header.stamp
        except ValueError as exc:
            self.get_logger().error('Recording rejected: '+str(exc))

    def destroy_node(self):
        self.file.close();super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=PathRecorder();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
