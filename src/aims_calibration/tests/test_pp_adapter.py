"""Exercise calibration against the real PP numeric reference/health fixture."""
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from aims_calibration.engine import Observation
from aims_calibration.node import CalibrationNode


def test_pp_uses_shared_health_association_enhanced_steering_and_regions(tmp_path):
    src=Path(__file__).resolve().parents[2]
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
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
