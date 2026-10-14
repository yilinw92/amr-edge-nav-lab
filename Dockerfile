FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    python3-colcon-common-extensions python3-numpy && rm -rf /var/lib/apt/lists/*
WORKDIR /ws/src/amr-edge-nav-lab
COPY . .
WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
COPY tools/entrypoint.sh /portfolio-entrypoint.sh
RUN chmod +x /portfolio-entrypoint.sh
ENTRYPOINT ["/portfolio-entrypoint.sh"]
CMD ["ros2", "launch", "amr_edge_lab", "demo.launch.py"]
