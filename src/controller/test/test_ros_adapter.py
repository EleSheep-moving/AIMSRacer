"""Exercise ROS message contracts with the actual installed Humble types."""
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from aims_mpcc.node import MPCCNode
from aims_mpcc.runtime import Supervisor
from aims_mpcc.config import VehicleConfig


class Capture:
    def __init__(self): self.values=[]
    def publish(self,msg): self.values.append(msg)


def test_idle_tick_publishes_valid_diagnostic_and_zero_command():
    rclpy.init()
    node=object.__new__(MPCCNode);Node.__init__(node,'adapter_contract')
    node.supervisor=Supervisor(VehicleConfig(),10)
    node.worker=SimpleNamespace(poll=lambda now:None,ready=False)
    node.mode='shadow';node.command_pub=Capture();node.status_pub=Capture()
    node.solve_times=[];node.deadline_misses=0;node.log=None
    try:
        node.tick()
        assert node.command_pub.values[-1].drive.speed==0
        assert node.status_pub.values[-1].status[0].level==b'\x00'
    finally:
        Node.destroy_node(node);rclpy.shutdown()
