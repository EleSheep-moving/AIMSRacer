# Calibration history and current operation

Current instructions are maintained in [校准资料与当前流程](calibration.md), [main deployment](../../../docs/deployment/README.md), and [vehicle bringup](../../../docs/operations/bringup.md). Full longitudinal identification remains planned for the future acceleration-control interface; existing bags can still support response analysis.

The previous English collection guide is preserved in the [2026-10-10 archive](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md). Its scripts, launch files and external workspaces describe historical source layouts; some entry points have been removed. Use the current deployment, known-map and recording guides for vehicle operation.

Calibration now uses `/calib/vesc_command` (`vesc_msgs/msg/VescCommand`). Publishers
must use current ROS time and `base_link`, fill only the selected mode's target,
and send a new command after RC unlock/arming. Mode 4 is reserved; nonzero braking
is disabled by default. The historical Ackermann calibration schema cannot be
connected with a topic remap. See the [native command contract](../../../docs/vesc-command.md).
