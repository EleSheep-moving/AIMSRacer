# Calibration history and current operation

Current instructions are maintained in [current and lateral-acceleration calibration](../../../docs/operations/calibration.md), [main deployment](../../../docs/deployment/README.md), and [vehicle bringup](../../../docs/operations/bringup.md). A/B/C collection and offline models are available; the acceleration-control interface remains separate.

The previous English collection guide is preserved in the [2026-10-10 archive](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md). Its scripts, launch files and external workspaces describe historical source layouts; some entry points have been removed. Use the current deployment, known-map and recording guides for vehicle operation.

Calibration now uses `/calib/vesc_command` (`vesc_msgs/msg/VescCommand`). Publishers
must use current ROS time and `base_link`, fill only the selected mode's target,
and send a new command after RC unlock/arming. Mode 4 is reserved; nonzero braking
is disabled by default. The historical Ackermann calibration schema cannot be
connected with a topic remap. See the [native command contract](../../../docs/vesc-command.md).

[Lateral AY validation and grip status](../../../docs/operations/lateral-acceleration-calibration.md) · [Canonical package notes](calibration.md)
