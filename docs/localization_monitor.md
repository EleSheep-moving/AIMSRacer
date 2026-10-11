<a id="定位健康监控"></a>
# Localization health monitor

**English** · [简体中文](localization_monitor.zh-CN.md)

`known_map_localization.launch.py` launches the C++ executable
`aims_racer_system/localization_monitor` by default. Node names, topics, and parameter files retain the existing interfaces.
The Python version remains as a reference implementation; the default launch workflow no longer uses it.

<a id="修改原因"></a>
## Reason for the change

During the 2026-10-08 A1 weight experiment, localization health checks reported
`ekf_age_sec=0.110562809` and `ekf_receive_age_sec=0.003381803`, exceeding
the 100 ms source-time freshness threshold. EKF messages in the concurrent recording remained continuous and fresh in source time,
so the evidence points to a message processing backlog at the monitor. The available data cannot further distinguish the effects of Python execution,
DDS delivery, and operating system scheduling.

The old monitor's EKF subscription depth was 500; at 200 Hz this could accumulate approximately 2.5 seconds of data.
The C++ implementation uses a reliable subscription with depth 2. The point cloud subscription uses sensor QoS with depth 1.
EKF history for point cloud time alignment is retained independently for 5 seconds and does not depend on the subscription queue.
Matching quality computation uses one background task; the main executor does not wait for its result.
The background task only reads point clouds, transform snapshots, and the map; it does not modify health state.

<a id="保留的保护语义"></a>
## Preserved protection semantics

- EKF source-time age and monotonic-clock receive age must both remain within 100 ms.
- Both point cloud ages must remain within 500 ms; both trusted anchor ages must remain within 1 second.
- Repeated source times cannot refresh the receive watchdog; future or reversed input times are invalid.
- Anchors must satisfy the epoch, event_sequence, anchor_sequence, and commit-time protocol.
- Time reversal requires a new epoch; recovery after invalidation requires three valid commits.
- The health sequence resets with the epoch; static TF does not refresh the trusted anchor.
- Point cloud matching quality is diagnostic only and does not determine ready state.
- MPCC fault latching logic is unchanged.

Point cloud diagnostics still use EKF interpolation at source time and the most recently committed map→odom correction,
without interpolating corrections or extrapolating EKF. PCL nearest-neighbor queries use single-precision coordinates, so diagnostic values may differ slightly from
the Python double-precision implementation; the inlier distance threshold remains 0.25 m.

<a id="新增诊断字段"></a>
## Added diagnostic fields

In `/localization/status`:

| Field | Meaning |
|---|---|
| `implementation` | `cpp` indicates that the native implementation is in use |
| `ekf_callback_age_sec` | Source-time age when processing began for the most recent valid EKF callback |
| `ekf_max_callback_age_sec` | Maximum of that age since process startup |
| `ekf_callback_count` | Number of valid, nonduplicate EKF callbacks |
| `ekf_stale_callback_count` | Number of those callbacks whose source-time age exceeded the EKF threshold |
| `quality_worker_busy` | Whether a point cloud quality task has a result still pending collection |

`ekf_age_sec` still represents the source-time age of the latest processed EKF when health status is published;
`ekf_receive_age_sec` still represents monotonic-clock time elapsed since processing that message.
The new counters and maximum accumulate over the process lifetime and do not reset with epochs.
The replay tool records the native executable's SHA256 to identify the version actually run.

Launch commands and protection thresholds in `params/localization_monitor.yaml` do not need adjustment.
Successful compilation only establishes that the build completed; the amount of delay improvement and vehicle effects require separate verification.
