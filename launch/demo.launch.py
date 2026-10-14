"""Nav2 Humble + AMCL + a synthetic differential-drive world."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("amr_edge_lab"))
    nav_share = Path(get_package_share_directory("nav2_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("goal_x", default_value="6.85"),
        DeclareLaunchArgument("goal_y", default_value="4.85"),
        Node(package="amr_edge_lab", executable="sim_world", output="screen"),
        Node(package="amr_edge_lab", executable="depth_projector", output="screen"),
        Node(package="amr_edge_lab", executable="velocity_gate", output="screen"),
        Node(package="amr_edge_lab", executable="mission_manager", output="screen", parameters=[{
            "goal_x": LaunchConfiguration("goal_x"), "goal_y": LaunchConfiguration("goal_y"),
        }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share / "launch/bringup_launch.py")), launch_arguments={
            "map": str(share / "config/warehouse.yaml"),
            "params_file": str(share / "config/nav2.yaml"),
            "use_sim_time": "False", "autostart": "True", "use_composition": "False",
        }.items()),
    ])
