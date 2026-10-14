# Validation record — version 0.2

Validated on 2026-10-14 in Windows 11 / Ubuntu 24.04 WSL2 / ROS 2 Humble Ubuntu 22.04 Docker, x86_64. No physical robot, Jetson or ZED was used.

| Check | Observed result |
| --- | --- |
| Portable + NumPy tests | 28 tests passed, including 20 seeded projection equivalence cases |
| ROS package | colcon symlink build passed |
| Live depth dropout | Moving robot's gate output became zero in 0.4986 s |
| Stationary hold after cancellation | Recorded odometry drift 0.0 m |
| Recovery route | Both waypoints completed, one confirmed-cancel pause, no extra readiness loss in final run |
| Operator cancellation | Confirmed CANCELED and no automatic resubmission |
| Live 64x24 perception pipeline | p50 2.031 ms, p95 2.565 ms including clustering in the depth callback; excludes transport |
| 320x180 synthetic projection benchmark | Same stride 2 / voxel 0.08: portable median 50.374 ms, NumPy 13.494 ms, 574 output points |
| Existing live navigation smoke check | Depth cloud, AMCL transform, cancellation and subsequent nearby goal passed before final NumPy optimization |

Raw measured results and journal are in `docs/evidence`. Timings are samples from these runs, not safety or device-performance guarantees. Benchmark timings exclude ROS transport and clustering; callback timings use the much smaller live synthetic image. They must not be compared as identical workloads.

Two earlier fault-test attempts reached the destination but observed a second readiness loss. Instrumentation showed scan/depth/odometry all stale together. A host scheduling/transport stall is a plausible explanation, not a proven root cause. The harness now records additional losses and checks every retry follows the previous final result, within the existing pause budget. The final run had no additional loss. The system has not been presented as immune to scheduling stalls.

`docs/media/ros_run.json`, trajectory and replay are historical version-0.1 full-route evidence from 2026-10-13: destination (6.85,4.85), result SUCCEEDED, recorded 7.707 m including the cancellation check. Current version-0.2 fault recovery is evidenced separately.

GitHub Actions runs the tests and fresh live scenarios for each push. Consult the workflow for the exact commit; this local record alone does not assert remote CI success.
