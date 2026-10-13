<a id="校准资料与当前流程"></a>
# Calibration materials and current workflow

**English** · [简体中文](calibration.zh-CN.md)

Current longitudinal identification uses the dedicated `aims_calibration` package: speed hold, positive-current acceleration and negative-current deceleration, with enhanced PP or RC steering. Draw a map-aligned reference in the browser and mark straight trial intervals; follow [editor, PP and calibration](../../../docs/operations/pp-calibration.md). The acceleration-target execution chain is still separate. For current vehicle operation, use [main deployment](../../../docs/deployment/README.md), [vehicle bringup](../../../docs/operations/bringup.md), [known-map MPCC](../../../docs/operations/known-map-mpcc.md), and [recording](../../../docs/operations/recording.md).

The original tutorials for automatic collection, Pure Pursuit, and longitudinal/lateral calibration are retained in the [2026-10-10 Chinese archive](../../../docs/reports/2026-10-10-calibration-guide-archive.md) and [English archive](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md). Enhanced PP and longitudinal calibration have been extracted into the current packages; old commands and direct-driver remaps in the archive are not current run instructions. Horizontal grip identification is not restored in this version.

Existing response evidence is in the [2026-09-28 speed-mode analysis](../../../docs/reports/2026-09-28-speed-mode-calibration.md), with later actual driving in the [2026-10-10 field review](../../../docs/reports/2026-10-10-mpcc-field-review.md). Interpret the model's steering lag and command-to-yaw response conditions according to the corresponding records; no actual steering angle sensor measurement was provided.

Calibration now uses `/calib/vesc_command` (`vesc_msgs/msg/VescCommand`). Publishers
must use current ROS time and `base_link`, fill only the selected mode's target,
and send a new command after RC unlock/arming. Mode 4 is reserved; nonzero braking
is disabled by default. The historical Ackermann calibration schema cannot be
connected with a topic remap. See the [native command contract](../../../docs/vesc-command.md).
