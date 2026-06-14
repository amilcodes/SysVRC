"""Controller + kinematic plant backend — no Gazebo. Fast, deterministic, CI-friendly."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("visbot_control"), "config", "controller.yaml")
    mission = LaunchConfiguration("mission")
    sched = LaunchConfiguration("sched")
    cpu = LaunchConfiguration("cpu")
    poll_idle = LaunchConfiguration("poll_idle")
    spin_us = LaunchConfiguration("spin_us")
    dash = LaunchConfiguration("dash")
    record = LaunchConfiguration("record")
    snapshot_dir = LaunchConfiguration("snapshot_dir")
    return LaunchDescription([
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("snapshot_dir", default_value="/tmp"),
        DeclareLaunchArgument("mission", default_value="skills"),
        DeclareLaunchArgument("sched", default_value="fifo"),
        DeclareLaunchArgument("cpu", default_value="-1"),
        DeclareLaunchArgument("poll_idle", default_value="false"),
        DeclareLaunchArgument("spin_us", default_value="0.0"),
        DeclareLaunchArgument("dash", default_value="true"),
        Node(package="visbot_control", executable="plant_node", name="visbot_plant",
             parameters=[cfg], output="screen"),
        Node(package="visbot_control", executable="controller_node", name="visbot_controller",
             parameters=[cfg, {"mission": mission, "sched_policy": sched, "poll_idle": ParameterValue(poll_idle, value_type=bool), "spin_before_deadline_us": ParameterValue(spin_us, value_type=float), "cpu": ParameterValue(cpu, value_type=int)}], output="screen"),
        Node(package="visbot_dash", executable="dash_node", name="visbot_dash", output="screen",
             parameters=[{"record": record, "snapshot_dir": snapshot_dir}], condition=IfCondition(dash)),
    ])
