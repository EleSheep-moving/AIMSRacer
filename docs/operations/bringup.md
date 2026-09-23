# Vehicle bringup

Complete [installation](../installation.md) and configure the Livox network and
serial devices first. Run commands from the workspace root. These launch files
start hardware, including RC and VESC; keep the RC locked during startup checks.

## Environment

In bash:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Keep this bringup shell free of global `OPENBLAS_NUM_THREADS` and `OMP_NUM_THREADS` exports; scope them to the MPCC command instead.

Use the corresponding `setup.zsh` files in zsh. The Livox underlay path may differ
on another machine. Use system Python as described in the installation guide.

The vehicle's DDS control graph should use loopback so Wi-Fi changes do not alter
local discovery. Set `CYCLONEDDS_URI` to your verified local CycloneDDS XML. With
an explicit `lo` interface in that XML, use `ROS_LOCALHOST_ONLY=0` to avoid duplicate
loopback selection. That setting alone does not restrict DDS to loopback: the XML
must do so. Participant capacity must cover the full launch; the historical test
used `MaxAutoParticipantIndex=63`. Livox Ethernet UDP remains separate from DDS.
See [network configuration](../installation.md#113-cyclonedds).

## Choose one bringup

| Mode | Launch file | Purpose |
| --- | --- | --- |
| V2 | `base_orin_livox_bringup_v2.launch.py` | LiDAR, rear-axle EKF, RC/VESC |
| V3 | `base_orin_livox_bringup_v3.launch.py` | V2 plus ZED 2i perception |
| V1 | `base_orin_livox_bringup.launch.py` | Legacy mux-based integration |

For the current LiDAR/MPCC workflow:

```bash
ros2 launch aims_racer_system base_orin_livox_bringup_v2.launch.py
```

Alternatively, for ZED perception, stop V2 and launch V3:

```bash
ros2 launch aims_racer_system base_orin_livox_bringup_v3.launch.py
```

V3 includes V2; do not run both independently. ZED uses `NEURAL_LIGHT` at 30 Hz.
Camera positional tracking and its vehicle dynamic TF are disabled by default.
Enable its static vehicle mounting only after measuring the transform, using
`zed_publish_base_tf` and the `zed_tf_*` launch arguments.

## Verify and proceed

Check the topics and single-parent TF tree against [architecture](../architecture.md).
V2/V3 do not by themselves establish `map -> odom`. Their MPCC workflow uses the
same live `odom` session for recording and driving. Upstream FAST-LIO must be
launched with the documented TF isolation; keep submodules unmodified.

- For data collection, follow [recording](recording.md).
- For autonomous control, complete the [vehicle checklist](vehicle-checklist.md)
  and [MPCC usage](../../src/controller/docs/usage.md).
- For calibration, use the [package guide](../../src/aims_racer_system/docs/calibration.en.md).

Keep Nav2 and other `/drive` producers stopped during MPCC operation. Mapping has
a separate launch (`base_orin_livox_mapping_v2.launch.py`) and TF ownership; it is
not an additional node to start alongside the V2 driving pipeline.

## Chassis reverse transition

Engaging reverse on this chassis has produced approximately 2 g of vertical shock,
which can disrupt FAST-LIO's IMU assumptions. Avoid relying on localization through
that transition; check or reinitialize it before resuming autonomous operation.
