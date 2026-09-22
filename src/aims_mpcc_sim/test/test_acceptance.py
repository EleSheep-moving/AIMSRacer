import math

from aims_mpcc_sim.acceptance import launch_initial_pose_arguments


class ReferenceAtOrigin:
    def at(self, progress):
        assert progress == 0.0
        return {'x': 9.0, 'y': -1.5, 'yaw': math.pi / 3.0}


def test_launch_initial_pose_arguments_match_reference_origin():
    assert launch_initial_pose_arguments(ReferenceAtOrigin()) == [
        'initial_x:=9.0',
        'initial_y:=-1.5',
        f'initial_yaw:={math.pi / 3.0}',
    ]
