# FAST-LIO integration patch

`fastlio2-publish-tf.patch` adds a `publish_tf` YAML option, defaulting to true,
and guards the upstream dynamic TF broadcast. V2/V3 set it to false so EKF
owns `odom -> base_link`; the rear-axle adapter owns that TF during mapping.

The parent repository retains upstream commit
`f516daac08bc46e50e814a2e7d6c8352ed8141bb` as its submodule gitlink. From the
AIMSRacer root, prepare a fresh checkout before building:

```bash
git submodule update --init src/FASTLIO2_ROS2
bash scripts/apply_fastlio_patch.sh
```

The script checks the exact base revision, accepts an already-applied patch,
and checks for conflicts before changing files. The submodule will show a
modified working tree; this is expected. Do not discard that change before
building. Reapply after a submodule reset/update. There is no unpublished
submodule commit required for this repository to work.

The patch and its application are checked independently. The Docker estimator
tests do not compile/run FAST-LIO scan matching; build `fastlio2` on the vehicle
and check TF ownership before driving. See the
[real-car checklist](../src/controller/docs/REAL_CAR_CHECKLIST.md).
