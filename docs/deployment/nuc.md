# NUC deployment

The NUC can run Nav2 and the optional CP2102 FDI IMU. It is not the current
vehicle sensor/actuator computer. `nav.launch.py` is also supported on the Orin;
its navigation output can reach `/drive`, so do not run either Nav2 instance
while MPCC is active.

## Native Nav2 workspace

Use Ubuntu 22.04 x86-64 and ROS 2 Humble. Clone the repository with its VESC and
CRSF submodules, install Nav2 and the package build dependencies, then build the
small navigation workspace:

```bash
git clone --branch feat/aims-mpcc --recurse-submodules \
  git@github.com:EleSheep-moving/AIMSRacer.git
cd AIMSRacer
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-yaml \
  ros-humble-ackermann-msgs ros-humble-diagnostic-updater \
  ros-humble-nav2-bringup ros-humble-robot-localization
rosdep install --from-paths \
  src/ackermann_mux src/ros2_crsf_receiver/crsf_receiver_msg \
  src/vesc/vesc_msgs src/aims_racer_system --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-select \
  crsf_receiver_msg vesc_msgs ackermann_mux aims_racer_system
source install/setup.bash
```

Install the NUC rule only when the CP2102 FDI IMU is physically attached:

```bash
sudo install -m 0644 rules/rulesForNUC/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The rule creates `/dev/ttyIMU`; this repository does not include an FDI IMU
serial-driver node. Before configuring that driver, check whether `brltty` or
`ModemManager` has claimed the CP2102 endpoint and apply the conditional remedy
in [serial service ownership](../../rules/README.md#serial-service-ownership).

Launch Nav2 with an explicit map path rather than relying on the historical
`/home/nuc/maps/...` location:

```bash
source /opt/ros/humble/setup.bash
source ~/AIMSRacer/install/setup.bash
ros2 launch aims_racer_system nav.launch.py \
  map:=/absolute/path/to/map.yaml
```

Use the same ROS domain as the rest of the vehicle graph; see the [deployment overview](README.md#ros-domain).

Check `ros2 topic info /drive` before enabling Nav2. Stop MPCC and every other
autonomous command producer first.
