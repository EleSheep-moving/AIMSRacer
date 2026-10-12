<a id="车辆运行检查"></a>
# Vehicle operation checklist

**English** · [简体中文](vehicle-checklist.zh-CN.md)

This checklist covers the current native vehicle workflow. Software regression and experimental verification are in `verification/`; consult [bringup](bringup.md) and [known maps](known-map-mpcc.md) for field operation.

<a id="启动前"></a>
## Before launch

- Confirm RC/physical emergency stop, power, VESC, `/dev/ttyELRS`, and `/dev/ttyVESC` operate correctly; check Livox network configuration.
- Verify the identities of the map PCD, reference, and target-platform bundle; launch from the same workspace's install environment.
- Confirm that `base_link` rear axle location, sensor mounting, and vehicle footprint match the actual vehicle. The default footprint extends 0.52 m ahead of the rear axle, 0.10 m behind, and 0.16 m to each side.
- Confirm physical clearance in the intended driving area. Currently `enforce_corridor=false`; the 0.5 m widths on either side in the file do not provide real boundary protection.
- Launch only one of vehicle, mapping, or race; NDT in known-map race and PGO during mapping must not share ownership of map TF.

<a id="启用前"></a>
## Before enable

- Keep RC ready for immediate takeover; check steering signs and speed-mode units.
- Initialize with the actual **map/base_link** pose and observe whether the map, point cloud, and vehicle pose agree.
- Check ready state, map identity, epoch, and health in `/localization/status`; check solver, state freshness, and auto start reasons in `/mpcc/status`.
- Ensure only one `/drive` publisher exists and the RC selector forwards actual commands to `/vesc/command`.
- Keep the vehicle stationary with its heading within 30° of the reference tangent. `auto_start=true` enables autonomy once when conditions are met; the operator should complete field checks before granting permission.
- When data must be retained, use `record=true` and confirm that the new session directory is writable with available storage space.

<a id="行驶停止与记录"></a>
## Driving, stopping, and recording

- Observe lateral error, speed tracking, localization health, candidate/takeover rejections, and operator interventions. Existing bag responses can be analyzed further; complete longitudinal identification is planned with the future acceleration interface.
- Explicit `/mpcc/enable {data:false}` stops the vehicle and cancels pending auto start; use RC/physical emergency stop in an emergency.
- The default single lap stops automatically; repeated laps with `repeat_laps=true` are stopped by the operator. Record COMPLETE waiting and motion time separately.
- After the vehicle is stationary and permission has been revoked, close launch and check bag metadata, `runtime.csv`, and configuration snapshots.

Default configuration uses the 2026-10-10 v35 field-run parameters: cruise 3.5, hard cap 4, acceleration/braking 1, planning ay=1 (units m/s or m/s²), N15/dt=.1/RTI2. Planned peak was approximately 2.99 and measured peak approximately 2.89 m/s. Field evidence applies to this vehicle, map, and recorded conditions; evaluate later changes using repeated data on the same track. The [review](../reports/2026-10-10-mpcc-field-review.md) gives the complete limitations.
