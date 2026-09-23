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

## Serial service ownership

A udev rule creates a stable name but cannot stop another service from opening
the underlying serial device. The NUC CP2102 bridge (`10c4:ea60`) is especially
susceptible on Ubuntu images whose `brltty` udev rule still recognizes that
generic USB ID. `ModemManager` can also probe serial ports. Check the actual
connected device before changing either service:

```bash
lsusb -d 10c4:ea60
ls -l /dev/ttyIMU 2>/dev/null || true
sudo fuser -v /dev/ttyIMU 2>/dev/null || true
systemctl is-active brltty.service brltty-udev.service ModemManager.service || true
journalctl -b -u brltty-udev.service -u ModemManager.service --no-pager
```

If `lsusb` lists the connected CP2102 but `/dev/ttyIMU` is missing, or the
command output shows `brltty` claiming it, and the NUC does not require Braille
support, prevent both its boot and udev-activated forms from reopening the port:

```bash
sudo systemctl mask --now brltty.service brltty-udev.service
```

If `fuser` reports `ModemManager` and the NUC has no cellular modem that needs
it, stop it from probing the port:

```bash
sudo systemctl disable --now ModemManager.service
```

Unplug and reconnect the bridge, then rerun the diagnostics. Do not disable
either service merely because it is installed. The Orin's ELRS endpoint is its
`ttyTHS1` hardware UART and its VESC is USB CDC; neither uses the NUC CP2102
bridge.
