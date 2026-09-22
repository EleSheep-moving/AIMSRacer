# Device rules by platform

Install only the directory that matches the computer physically connected to the
device. Rules create stable `/dev` names; they do not install or start a ROS
driver.

| Computer | Directory | Devices | Stable names |
| --- | --- | --- | --- |
| Jetson Orin vehicle computer | `rulesForOrin/` | ELRS receiver and VESC | `/dev/ttyELRS`, `/dev/ttyVESC` |
| NUC navigation computer | `rulesForNUC/` | CP2102 FDI IMU | `/dev/ttyIMU` |

Inspect before installing:

```bash
cd ~/AIMSRacer
sed -n '1,200p' rules/rulesForOrin/*.rules
sed -n '1,200p' rules/rulesForNUC/*.rules
```

Install the Orin rules:

```bash
sudo install -m 0644 rules/rulesForOrin/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

Install the NUC rule:

```bash
sudo install -m 0644 rules/rulesForNUC/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing `dialout` membership. Confirm the expected
device with `ls -l /dev/ttyELRS /dev/ttyVESC /dev/ttyIMU` after reconnecting it.
The CP2102 rule preserves its existing `0777` mode; tighten it only after the
owning FDI IMU driver and user/group requirements are verified.
