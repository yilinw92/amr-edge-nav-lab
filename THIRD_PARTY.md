# References and third-party boundaries

- `config/nav2.yaml` is adapted from [Nav2 Humble nav2_params.yaml](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml). It retains Apache-2.0 licensing. Changes: initial pose, `base_link`, robot radius, inflation distance, stopped-velocity threshold, and depth-cloud observation source. See `third_party/nav2_LICENSE.txt` and `third_party/Apache-2.0.txt`.
- ROS 2 packages are installed by the Dockerfile; their upstream licenses remain applicable. They are not vendored or rebranded as original project code.
- [ROS 2 documentation](https://github.com/ros2/ros2_documentation), [Nav2 documentation](https://docs.nav2.org/), and [ZED ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper) are implementation references.
- The map, example scenarios, simulator, portable baseline, adapters, tests and tutorials are newly authored for this portfolio with AI assistance. No employer datasets or assets are included.
