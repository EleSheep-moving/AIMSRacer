"""One-shot startup compatibility preparation; exits before the C++ control process starts.

This deliberately reuses the tested Python parameter/configuration, frozen plan,
provenance and reference loaders. It never spins or runs a control callback.
"""
from dataclasses import asdict
import json
from pathlib import Path
import sys
import rclpy
from .io import _json_safe
from .node import CalibrationNode


def main():
    destination = Path(sys.argv[1])
    rclpy.init(args=sys.argv[2:])
    node = None
    try:
        node = CalibrationNode()
        parameters = {name: parameter.value for name, parameter in node.get_parameters_by_prefix('').items()}
        engine = node.engine
        manifest = node.output.writer.manifest
        prepared = dict(config=asdict(engine.config), plan=engine.serializable_plan,
                        tracking=asdict(engine.tracking_config),
                        protection=asdict(engine.protection.settings) if engine.protection else None,
                        baseline=manifest.get('baseline') if manifest else None,
                        response_limits=manifest.get('response_limits') if manifest else None)
        descriptor = dict(prepared=prepared, runtime_plan=engine.plan, parameters=parameters,
                          observation=asdict(node.obs), metadata=node.output.writer.metadata,
                          manifest=manifest, output_directory=str(node.output.directory),
                          pp_configuration=asdict(node.pp_config) if node.tracking else None,
                          regions=node.regions.regions if node.tracking else [],
                          reference_length=node.reference.length if node.tracking else None,
                          startup_helper_pid=__import__('os').getpid())
        node.output.flush()
        destination.write_text(json.dumps(_json_safe(descriptor), allow_nan=False))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
