"""Exercise calibration against the real PP numeric reference/health fixture."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from aims_calibration.engine import Observation
from aims_calibration.node import CalibrationNode


@pytest.fixture
def pp_node(tmp_path):
    src=Path(__file__).resolve().parents[3]
    spec=importlib.util.spec_from_file_location('calibration_pp_fixture',src/'aims_pp/test/conftest.py')
    helper=importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    payload=helper.circle_payload.__wrapped__()
    raw=json.dumps(payload).encode(); (tmp_path/'reference.json').write_bytes(raw)
    (tmp_path/'calibration_regions.json').write_text(json.dumps(dict(schema_version=1,
        reference_sha256=hashlib.sha256(raw).hexdigest(),regions=[dict(id='seam',kind='straight',
            start_s=payload['length']-1.,end_s=1.)])))
    rclpy.init(); node=None
    try:
        node=CalibrationNode(parameter_overrides=[Parameter('vehicle_config',value=str(src/'controller/config/vehicle.yaml')),
            Parameter('reference_directory',value=str(tmp_path)),Parameter('log_directory',value=str(tmp_path/'logs'))])
        yield node,helper
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()


def test_pp_uses_shared_health_association_enhanced_steering_and_regions(pp_node):
    node,helper=pp_node
    helper.refresh(node.tracking,10.,10.)
    # Calibration has its own authority and current echo; localization gate uses neither nav input.
    node.tracking.observe_authority(False,10.); node.tracking.command_valid=False
    out=node._tracking_observation(Observation(),10.,10.)
    assert out.straight and out.region=='seam' and not out.tracking_reason
    pose=node.tracking.map_pose(); target=node.reference.at(node.pp.compute_lookahead(pose[3]))
    expected,_=node.pp.compute_steering(pose[:2],pose[2],(target['x'],target['y']),target['yaw'],target['curvature'],pose[3])
    assert out.steering==pytest.approx(expected)
    for i in range(1,9):
        at=node.reference.at(.375*i); now=10.+.1*i
        helper.refresh(node.tracking,now,now,sequence=i+1,pose=(at['x'],at['y'],at['yaw'],1.))
        out=node._tracking_observation(Observation(),now,now)
    assert not out.straight and out.region=='' and not out.tracking_reason
    node.tracking.observe_map_valid(False)
    assert node._tracking_observation(Observation(),10.8,10.8).tracking_reason


@pytest.mark.parametrize('angle', [.4, -.4])
def test_small_pose_change_with_large_projection_change_keeps_tracking(pp_node,angle):
    node,helper=pp_node
    # Near the centre of a curve, a centimetre-scale pose change can move
    # the nearest reference point much farther in either direction.
    first=(.03,0.,math.pi/2,1.)
    second=(.03*math.cos(angle),.03*math.sin(angle),math.pi/2+angle,1.)
    helper.refresh(node.tracking,10.,10.,pose=first)
    node.engine.enable(True,now=9.9);node.engine.selected=True
    node.tracking_generation=node.tracking.fault_generation
    initial=node._tracking_observation(Observation(),10.,10.)
    initial_s=node.association_runtime.association()[1]
    assert initial.tracking_reason==''
    helper.refresh(node.tracking,10.02,10.02,sequence=2,pose=second)
    current=node._tracking_observation(Observation(),10.02,10.02)
    current_s=node.association_runtime.association()[1]
    delta=(current_s-initial_s+node.reference.length/2)%node.reference.length-node.reference.length/2
    assert math.dist(first[:2],second[:2])<.02
    assert abs(delta)>.5
    assert current.tracking_reason=='' and current.region=='seam'
    assert math.isfinite(current.steering)
    node.tracking.observe_map_valid(False)
    assert node._tracking_observation(Observation(),10.02,10.02).tracking_reason
