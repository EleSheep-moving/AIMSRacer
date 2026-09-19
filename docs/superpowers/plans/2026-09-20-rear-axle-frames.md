# Rear-axle frame migration

Approved design: base_link is the rear-axle midpoint. FAST-LIO retains its physical IMU state; an explicit adapter converts pose, twist and covariance before EKF. EKF alone publishes odom -> base_link during driving. The mapping adapter publishes it when EKF is absent. Legacy bringup keeps its old configs. Retain the old LiDAR mounting orientation as an explicit unverified assumption; represent the existing IMU helper rotation with its own matching virtual-frame TF.

- [x] Add regression tests for turning lever-arm velocity, rotated pose, covariance and rejected frames.
- [x] Implement the odometry adapter and timestamped raw-IMU angular-rate input; reject missing/stale IMU data.
- [x] Add rear-frame configs and shared sensor TF launch; redirect FAST-LIO TF onto an unused private topic, keeping upstream source untouched and avoiding competing TF authority.
- [x] Wire V2 driving/mapping and inherited V3; represent IMU helper output axes with matching TF; MPCC rear_offset=0, geometry unverified.
- [x] Shift existing Nav2 footprint coordinates by +0.17 m; retain conservative MPCC rectangle until physical extents measured.
- [x] Docker regression and ROS smoke tests; document frame migration and remaining physical measurements.

Adapter reads raw gyro near the LIO source timestamp (maximum 50 ms difference), not the IMU helper's pre-rotated gyro. Raw gyro bias remains an approximation; no new identified dynamics are claimed. Zero upstream covariances receive explicit configurable nonzero uncertainty floors. LIO raw topics retain the IMU frame for map/cloud consumers. Static sensor TFs derive from one rear-frame config and the same LIO extrinsics used by FAST-LIO.
