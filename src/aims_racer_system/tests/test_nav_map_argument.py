import ast
from pathlib import Path
import unittest


LAUNCH_FILE = Path(__file__).resolve().parents[3] / 'src/aims_racer_system/launch/nav.launch.py'


class NavLaunchMapArgumentTest(unittest.TestCase):
    def test_nav_launch_exposes_map_path_as_a_launch_argument(self):
        tree = ast.parse(LAUNCH_FILE.read_text())
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]

        self.assertTrue(any(
            isinstance(call.func, ast.Name)
            and call.func.id == 'DeclareLaunchArgument'
            and call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == 'map'
            for call in calls
        ))
        self.assertTrue(any(
            isinstance(call.func, ast.Name)
            and call.func.id == 'LaunchConfiguration'
            and call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == 'map'
            for call in calls
        ))


if __name__ == '__main__':
    unittest.main()
