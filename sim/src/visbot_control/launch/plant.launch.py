"""Controller + kinematic plant backend — no Gazebo. Fast, deterministic, CI-friendly.

    ros2 launch visbot_control plant.launch.py mission:=state_solo_awp.blue robot:=robots/ours
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


REPO = os.environ.get("SYSVRC_ROOT", "/ws/src/sysvrc")


def robot_json(arg: str) -> str:
    if not arg:
        return ""
    p = arg if os.path.isabs(arg) else os.path.join(REPO, arg)
    return os.path.join(p, "robot.json") if os.path.isdir(p) else p


def nodes(context):
    cfg = os.path.join(get_package_share_directory("visbot_control"), "config", "controller.yaml")
    robot = robot_json(LaunchConfiguration("robot").perform(context))
    lc = LaunchConfiguration
    return [
        Node(package="visbot_control", executable="plant_node", name="visbot_plant",
             parameters=[cfg, {"mission": lc("mission"), "robot": robot}], output="screen"),
        Node(package="visbot_control", executable="controller_node", name="visbot_controller",
             parameters=[cfg, {"mission": lc("mission"), "sched_policy": lc("sched"), "robot": robot,
                               "poll_idle": ParameterValue(lc("poll_idle"), value_type=bool),
                               "spin_before_deadline_us": ParameterValue(lc("spin_us"), value_type=float),
                               "cpu": ParameterValue(lc("cpu"), value_type=int)}], output="screen"),
        Node(package="visbot_dash", executable="dash_node", name="visbot_dash", output="screen",
             parameters=[{"record": lc("record"), "snapshot_dir": lc("snapshot_dir"), "robot": robot}],
             condition=IfCondition(lc("dash"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("snapshot_dir", default_value="/tmp"),
        DeclareLaunchArgument("mission", default_value="skills"),
        DeclareLaunchArgument("sched", default_value="fifo"),
        DeclareLaunchArgument("cpu", default_value="-1"),
        DeclareLaunchArgument("poll_idle", default_value="false"),
        DeclareLaunchArgument("spin_us", default_value="0.0"),
        DeclareLaunchArgument("dash", default_value="true"),
        DeclareLaunchArgument("robot", default_value="", description="robots/<name> from tools/robot_studio"),
        OpaqueFunction(function=nodes),
    ])
