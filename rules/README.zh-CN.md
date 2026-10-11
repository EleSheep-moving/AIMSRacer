<a id="device-rules-by-platform"></a>
# 按平台选择设备规则

**简体中文** · [English](README.md)

仅安装与实际连接设备的电脑相匹配的目录。
规则创建稳定的 `/dev` 名称，不会安装或启动 ROS 驱动。

| 电脑 | 目录 | 设备 | 稳定名称 |
| --- | --- | --- | --- |
| Jetson Orin 车辆电脑 | `rulesForOrin/` | ELRS 接收机与 VESC | `/dev/ttyELRS`, `/dev/ttyVESC` |
| NUC 导航电脑 | `rulesForNUC/` | CP2102 FDI IMU | `/dev/ttyIMU` |

安装前先检查：

```bash
cd ~/AIMSRacer
sed -n '1,200p' rules/rulesForOrin/*.rules
sed -n '1,200p' rules/rulesForNUC/*.rules
```

安装 Orin 规则：

```bash
sudo install -m 0644 rules/rulesForOrin/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

安装 NUC 规则：

```bash
sudo install -m 0644 rules/rulesForNUC/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

修改 `dialout` 组成员关系后，退出并重新登录。重新连接设备后，
使用 `ls -l /dev/ttyELRS /dev/ttyVESC /dev/ttyIMU` 确认预期设备。
CP2102 规则保留已有的 `0777` 模式；只有核实负责该设备的 FDI IMU 驱动及用户/组要求后，
才收紧权限。

<a id="serial-service-ownership"></a>
## 串口服务占用

udev 规则创建稳定名称，但无法阻止其他服务打开底层串口设备。
在 `brltty` udev 规则仍识别通用 USB ID 的 Ubuntu 镜像上，
NUC CP2102 桥接设备（`10c4:ea60`）尤其容易被占用。
`ModemManager` 也可能探测串口。修改任一服务前，检查实际连接的设备：

```bash
lsusb -d 10c4:ea60
ls -l /dev/ttyIMU 2>/dev/null || true
sudo fuser -v /dev/ttyIMU 2>/dev/null || true
systemctl is-active brltty.service brltty-udev.service ModemManager.service || true
journalctl -b -u brltty-udev.service -u ModemManager.service --no-pager
```

如果 `lsusb` 列出了已连接的 CP2102，但 `/dev/ttyIMU` 缺失，或者
命令输出显示 `brltty` 正在占用该设备，并且 NUC 不需要盲文支持，
则阻止其开机启动和 udev 激活两种形式重新打开串口：

```bash
sudo systemctl mask --now brltty.service brltty-udev.service
```

如果 `fuser` 显示 `ModemManager`，且 NUC 没有需要该服务的蜂窝调制解调器，
则停止其串口探测：

```bash
sudo systemctl disable --now ModemManager.service
```

拔下并重新连接桥接设备，再运行上述诊断。不要仅因服务已安装就禁用它。
Orin 的 ELRS 端点是其 `ttyTHS1` 硬件 UART，VESC 使用 USB CDC；
二者都不使用 NUC CP2102 桥接设备。
