"""Full-fidelity backend: Gazebo Harmonic + ros_gz bridge + RT controller + dashboard.

    ros2 launch visbot_gazebo sim.launch.py                 # headless server
    ros2 launch visbot_gazebo sim.launch.py gui:=true       # with the Gazebo GUI
    ros2 launch visbot_gazebo sim.launch.py mission:=mogo_rush sched:=other
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gz_share = get_package_share_directory("visbot_gazebo")
    desc_share = get_package_share_directory("visbot_description")
    ctrl_share = get_package_share_directory("visbot_control")
    world = os.path.join(gz_share, "worlds", "vrc_field.sdf")
    ctrl_cfg = os.path.join(ctrl_share, "config", "controller.yaml")

    gui = LaunchConfiguration("gui")
    mission = LaunchConfiguration("mission")
    sched = LaunchConfiguration("sched")
    cpu = LaunchConfiguration("cpu")
    poll_idle = LaunchConfiguration("poll_idle")
    spin_us = LaunchConfiguration("spin_us")
    dash = LaunchConfiguration("dash")
    record = LaunchConfiguration("record")
    snapshot_dir = LaunchConfiguration("snapshot_dir")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", os.path.join(desc_share, "urdf", "visbot.urdf.xacro")]),
        value_type=str)

    gz_args_headless = ["-r -s -v 1 ", world]
    gz_args_gui = ["-r -v 1 ", world]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("mission", default_value="skills"),
        DeclareLaunchArgument("sched", default_value="fifo"),
        DeclareLaunchArgument("cpu", default_value="-1"),
        DeclareLaunchArgument("poll_idle", default_value="false"),
        DeclareLaunchArgument("spin_us", default_value="0.0"),
        DeclareLaunchArgument("dash", default_value="true"),
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("snapshot_dir", default_value="/tmp"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args_headless}.items(),
            condition=UnlessCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args_gui}.items(),
            condition=IfCondition(gui)),

        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),

        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "visbot", "-x", "0", "-y", "0", "-z", "0.08"]),

        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(gz_share, "config", "bridge.yaml"), "use_sim_time": True}]),

        # Give Gazebo a moment to spawn before the controller starts arming.
        TimerAction(period=3.0, actions=[
            Node(package="visbot_control", executable="controller_node", name="visbot_controller", output="screen",
                 parameters=[ctrl_cfg, {"mission": mission, "sched_policy": sched, "poll_idle": ParameterValue(poll_idle, value_type=bool), "spin_before_deadline_us": ParameterValue(spin_us, value_type=float), "cpu": ParameterValue(cpu, value_type=int), "start_delay_s": 3.0}]),
        ]),
        Node(package="visbot_dash", executable="dash_node", name="visbot_dash", output="screen",
             parameters=[{"record": record, "snapshot_dir": snapshot_dir}], condition=IfCondition(dash)),
    ])
