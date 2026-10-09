"""Offline artifact contracts; run with the existing controller on PYTHONPATH."""
from pathlib import Path
import unittest


class ExportContract(unittest.TestCase):
    def test_offline_export_entrypoint_exists(self):
        script = Path(__file__).resolve().parents[1] / 'scripts' / 'export_bundle.py'
        self.assertTrue(script.is_file(), 'offline C++ acados export is missing')


if __name__ == '__main__':
    unittest.main()
