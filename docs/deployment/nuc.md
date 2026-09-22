# NUC deployment

The NUC role is Nav2 and the optional CP2102 FDI IMU. It is not the current
vehicle sensor/actuator computer. Its navigation output can reach `/drive`, so
do not run its Nav2 launch while MPCC is active.

## Native Nav2 workspace

Use Ubuntu 22.04 x86-64 and ROS 2 Humble. Clone the repository with its VESC and
CRSF submodules, install Nav2 and the package build dependencies, then build the
small navigation workspace:

```bash
git clone --recurse-submodules git@github.com:EleSheep-moving/AIMSRacer.git
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
serial-driver node. Install and configure that driver separately before using the
device.

Launch Nav2 with an explicit map path rather than relying on the historical
`/home/nuc/maps/...` location:

```bash
source /opt/ros/humble/setup.bash
source ~/AIMSRacer/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch aims_racer_system nav.launch.py \
  map:=/absolute/path/to/map.yaml
```

Check `ros2 topic info /drive` before enabling Nav2. Stop MPCC and every other
autonomous command producer first.
