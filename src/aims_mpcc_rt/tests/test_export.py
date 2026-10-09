"""Offline artifact contracts; run with the existing controller on PYTHONPATH."""
from pathlib import Path
import unittest
from dataclasses import replace
import json
import numpy as np


class ExportContract(unittest.TestCase):
    def test_profile_changes_source_artifact_fingerprint(self):
        from aims_mpcc.config import VehicleConfig
        from aims_mpcc.acados_backend import artifact_fingerprint
        from aims_mpcc.path import ReferencePath
        angles=np.linspace(0.,2*np.pi,40,endpoint=False)
        path=ReferencePath(3*np.c_[np.cos(angles),np.sin(angles)],1.,1.,'odom')
        legacy=VehicleConfig()
        selected=replace(legacy,command_profile='rate_bounded_v2',steering_acceleration_scale=legacy.steer_acceleration)
        first,old=artifact_fingerprint(path,legacy,10,.1)
        second,new=artifact_fingerprint(path,selected,10,.1)
        self.assertNotEqual(first,second)
        self.assertEqual(old['config']['command_profile'],'legacy_bounded_v1')
        self.assertEqual(new['config']['command_profile'],'rate_bounded_v2')

    def test_offline_export_entrypoint_exists(self):
        script = Path(__file__).resolve().parents[1] / 'scripts' / 'export_bundle.py'
        self.assertTrue(script.is_file(), 'offline C++ acados export is missing')


if __name__ == '__main__':
    unittest.main()
