<a id="校准资料与当前流程"></a>
# Calibration materials and current workflow

**English** · [简体中文](calibration.zh-CN.md)

A/B/C current collection and offline models are available; independent body-acceleration accuracy and a closed-loop acceleration-control interface require separate acceptance. For current vehicle operation, use [main deployment](../../../docs/deployment/README.md), [vehicle bringup](../../../docs/operations/bringup.md), [known-map MPCC](../../../docs/operations/known-map-mpcc.md), and [recording](../../../docs/operations/recording.md).

The original tutorials for automatic collection, Pure Pursuit, and longitudinal/lateral calibration are retained in the [2026-10-10 Chinese archive](../../../docs/reports/2026-10-10-calibration-guide-archive.md) and [English archive](../../../docs/reports/2026-10-10-calibration-guide-en-archive.md). Their scripts and launch files belong to the old source layout, and some have been removed; the tutorial is not a source of current run commands.

Existing response evidence is in the [2026-09-28 speed-mode analysis](../../../docs/reports/2026-09-28-speed-mode-calibration.md), with later actual driving in the [2026-10-10 field review](../../../docs/reports/2026-10-10-mpcc-field-review.md). Interpret the model's steering lag and command-to-yaw response conditions according to the corresponding records; no actual steering angle sensor measurement was provided.

Calibration now uses `/calib/vesc_command` (`vesc_msgs/msg/VescCommand`). Publishers
must use current ROS time and `base_link`, fill only the selected mode's target,
and send a new command after RC unlock/arming. Mode 4 is reserved; nonzero braking
is disabled by default. The historical Ackermann calibration schema cannot be
connected with a topic remap. See the [native command contract](../../../docs/vesc-command.md).

## Current calibration entry points

[Current calibration A/B/C](../../../docs/operations/calibration.md#current-calibration) · [Lateral-acceleration calibration / AY validation](../../../docs/operations/lateral-acceleration-calibration.md) · [Motion-capture tools (Chinese)](../../../docs/operations/mocap-validation.zh-CN.md). These pages distinguish deployed acquisition, offline results and historical grip scripts.
