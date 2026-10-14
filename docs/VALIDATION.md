# Validation record

Validated on 2026-10-13 (America/New_York). These are development checks, not a hardware or production benchmark.

| Check | Result |
| --- | --- |
| Python standard-library unit suite on Windows | 12 tests passed |
| Offline scenarios | Normal and blocked succeeded; blocked replanned once; canceled and unreachable returned their expected terminal states |
| ROS package build | `colcon build --symlink-install` succeeded inside ROS 2 Humble / Ubuntu 22.04 Docker |
| Live sensor integration | Synthetic depth projected into nonempty PointCloud2 |
| Live localization | AMCL provided the `map -> base_link` transform chain |
| Live mission cancellation | Confirmed action terminal result `CANCELED` |
| Full Nav2 route after cancellation | Destination `(6.85, 4.85)` m reached, action returned `SUCCEEDED` |
| Recorded full-run trajectory | 7.707 m, including the initial cancellation check; one synthetic run, not a performance claim |

Development environment: Windows host, Ubuntu 24.04 WSL2, Docker 29.1.2, x86_64 ROS 2 Humble container. No Jetson, ZED or physical robot was used.

Representative evidence is in `docs/media/ros_run.json`, `docs/media/trajectory.csv` and `docs/media/ros_replay.html`. Fresh runs export untracked files under `artifacts/ros`.

A startup race found during validation was corrected: service/action discovery alone was insufficient; the mission client now waits for the navigator lifecycle state to be active. Tests also distinguish a cancel request from the eventual action result.

GitHub Actions configuration is included; local validation does not imply that a future remote workflow has already passed.
