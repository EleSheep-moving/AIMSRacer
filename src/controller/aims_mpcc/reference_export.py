"""One solver-independent frozen reference format for native controllers."""
from .speed_planner import SpeedPlanner


def serialize_reference(path, config):
    """Freeze the canonical quintic and existing speed planner, without fitting."""
    path.validate_config(config)
    planner = SpeedPlanner(path, config)
    return dict(schema_version=1, knots=path.curve.s.tolist(),
                coefficients=path.curve.coefficients.tolist(), length=path.length,
                left_width=path.left_width, right_width=path.right_width,
                frame_id=path.frame_id, metadata=dict(path.metadata,frame_id=path.frame_id,
                    left_width=path.left_width,right_width=path.right_width,
                    coordinate_reference='rear_axle',schema_version=1),
                speed_profile=dict(schema_version=1, positions=planner.positions.tolist(),
                                   speeds=planner.speeds.tolist()))
