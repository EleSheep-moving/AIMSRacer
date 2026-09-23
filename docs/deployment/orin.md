# Jetson Orin deployment

The Orin NX is the vehicle computer. It directly owns the Livox, ELRS receiver,
VESC and optional ZED 2i. V2/V3 vehicle bringup is supported only by the native
JetPack installation below.

## Native vehicle stack

Use the complete [Orin native installation guide](../installation.md). It pins
Ubuntu 22.04, JetPack/L4T, ROS 2 Humble, the CUDA-enabled ZED SDK, Livox underlay
and AIMSRacer build. Do not substitute the x86-64 ZED SDK or desktop CUDA on the
Jetson.

After that guide completes, install the Orin device rules and start the vehicle
stack:

```bash
cd ~/AIMSRacer
sudo install -m 0644 rules/rulesForOrin/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch aims_racer_system base_orin_livox_bringup_v2.launch.py
```

Keep the vehicle bringup shell free of global `OPENBLAS_NUM_THREADS` and
`OMP_NUM_THREADS` exports. The MPCC guide scopes those limits to the MPCC
process so other vehicle nodes retain their own threading policy.

Use V3 only after stopping V2. Follow the [bringup guide](../operations/bringup.md)
and the [MPCC execution guide](../../src/controller/docs/usage.md) before enabling
motion.

## Shared Nav2 map launch

The Orin installation includes the Nav2 dependencies. If the Orin is assigned
to navigation, launch the same map server and Nav2 graph used by the NUC with an
explicit map path:

```bash
source /opt/ros/humble/setup.bash
source ~/AIMSRacer/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch aims_racer_system nav.launch.py \
  map:=/absolute/path/to/map.yaml
```

Do not run this Nav2 launch while MPCC is active: both can produce autonomous
drive commands.
