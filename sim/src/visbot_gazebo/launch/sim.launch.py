"""Full-fidelity backend: Gazebo Harmonic + ros_gz bridge + RT controller + dashboard.

    ros2 launch visbot_gazebo sim.launch.py                 # headless server
    ros2 launch visbot_gazebo sim.launch.py gui:=true       # with the Gazebo GUI
    ros2 launch visbot_gazebo sim.launch.py mission:=worlds_mogo_rush.blue sched:=other
    ros2 launch visbot_gazebo sim.launch.py robot:=robots/ours     # a robot from tools/robot_studio
"""
import json
import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


IN = 0.0254
AUTONS_DIR = os.environ.get("SYSVRC_AUTONS", "/ws/src/sysvrc/autons")
REPO = os.environ.get("SYSVRC_ROOT", "/ws/src/sysvrc")


def robot_json(arg: str):
    """robots/<name>, robots/<name>/robot.json, or an absolute path -> the file."""
    if not arg:
        return ""
    p = arg if os.path.isabs(arg) else os.path.join(REPO, arg)
    return os.path.join(p, "robot.json") if os.path.isdir(p) else p


def xacro_args(path: str) -> dict:
    """robot.json -> the robot xacro's arguments, Gazebo's drive limits included."""
    if not path:
        return {}
    with open(path, encoding="utf-8") as f:
        spec = json.load(f)
    d, fp = spec.get("drive", {}), spec.get("footprint", {})
    dia = d.get("wheel_diameter", 3.25)
    rpm = d.get("wheel_rpm") or d.get("cartridge_rpm", 600) * d.get("ratio", 0.75)
    track = d.get("track_width", 12.5)
    v = rpm / 60.0 * math.pi * dia * IN
    mass_lb = spec.get("mass_lb", 16.5)
    # same estimate as visbot::parseRobotSpec: tau ~ mass * speed^2 / motor power
    motors = 2 * d.get("motors_per_side", 3)
    watts = 5.5 if d.get("motor") == "5.5W" else 11.0
    vref = 450 / 60.0 * math.pi * 3.25 * IN
    tau = d.get("tau_s") or min(0.5, max(0.05, 0.12 * (mass_lb / 15) * (v / vref) ** 2 * 66 / (motors * watts)))
    args = {"wheel_d_in": dia, "track_in": track, "length_in": fp.get("length", 15), "width_in": fp.get("width", 15),
            "mass_kg": round(mass_lb * 0.45359237, 3), "max_v": round(v, 3),
            "max_w": round(2 * v / (track * IN), 3), "max_a": round(v / tau, 2)}
    mesh = os.path.join(os.path.dirname(path), "model.glb")
    if os.path.exists(mesh):
        args["mesh"] = "file://" + os.path.abspath(mesh)
    return args


def physical_start(mission: str):
    """Where the routine physically starts, (x in, y in, compass heading deg).

    Mirrors visbot::resolveMission + physicalStart in auton_file.hpp: the
    file's `# @field_start` if it has one, else its first odom_xyt_set, else
    the origin. Built-in routines start at the origin.
    """
    if mission in ("skills", "square", "chain"):
        return 0.0, 0.0, 0.0
    path = mission if ("/" in mission or mission.endswith(".auton")) else os.path.join(AUTONS_DIR, mission + ".auton")
    try:
        text = open(path, encoding="utf-8").read()
    except OSError:
        return 0.0, 0.0, 0.0
    m = re.search(r"^#\s*@field_start\s+(\S+)\s+(\S+)\s+(\S+)", text, re.M)
    if m:
        return tuple(float(v) for v in m.groups())
    m = re.search(r"^odom_xyt_set\(\s*([^,]+),\s*([^,]+),\s*([^)]+)\)", text, re.M)
    return tuple(float(v) for v in m.groups()) if m else (0.0, 0.0, 0.0)


def robot_nodes(context):
    """Everything that depends on which robot: its description and the
    controller that drives it."""
    desc_share = get_package_share_directory("visbot_description")
    ctrl_cfg = os.path.join(get_package_share_directory("visbot_control"), "config", "controller.yaml")
    robot = robot_json(LaunchConfiguration("robot").perform(context))
    cmd = [FindExecutable(name="xacro"), " ", os.path.join(desc_share, "urdf", "visbot.urdf.xacro")]
    for k, v in xacro_args(robot).items():
        cmd += [" ", f"{k}:={v}"]
    lc = lambda n: LaunchConfiguration(n)   # noqa: E731
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": ParameterValue(Command(cmd), value_type=str), "use_sim_time": True}]),
        # Give Gazebo a moment to spawn before the controller starts arming.
        TimerAction(period=3.0, actions=[
            Node(package="visbot_control", executable="controller_node", name="visbot_controller", output="screen",
                 parameters=[ctrl_cfg, {"mission": lc("mission"), "sched_policy": lc("sched"), "robot": robot,
                                        "poll_idle": ParameterValue(lc("poll_idle"), value_type=bool),
                                        "spin_before_deadline_us": ParameterValue(lc("spin_us"), value_type=float),
                                        "cpu": ParameterValue(lc("cpu"), value_type=int), "start_delay_s": 3.0}]),
        ]),
        Node(package="visbot_dash", executable="dash_node", name="visbot_dash", output="screen",
             parameters=[{"record": lc("record"), "snapshot_dir": lc("snapshot_dir"), "robot": robot}],
             condition=IfCondition(lc("dash"))),
    ]


def spawn(context):
    x, y, heading = physical_start(LaunchConfiguration("mission").perform(context))
    yaw = math.radians(90.0 - heading)  # compass (CW from +y) -> ROS yaw (CCW from +x)
    return [Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "visbot",
                            "-x", f"{x * IN:.4f}", "-y", f"{y * IN:.4f}", "-z", "0.08", "-Y", f"{yaw:.4f}"])]


def generate_launch_description():
    gz_share = get_package_share_directory("visbot_gazebo")
    world = os.path.join(gz_share, "worlds", "vrc_field.sdf")

    gui = LaunchConfiguration("gui")

    gz_args_headless = ["-r -s -v 1 ", world]
    gz_args_gui = ["-r -v 1 ", world]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("mission", default_value="skills",
                              description="built-in name, a name in autons/, or a path to an .auton file"),
        DeclareLaunchArgument("sched", default_value="fifo"),
        DeclareLaunchArgument("cpu", default_value="-1"),
        DeclareLaunchArgument("poll_idle", default_value="false"),
        DeclareLaunchArgument("spin_us", default_value="0.0"),
        DeclareLaunchArgument("dash", default_value="true"),
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("snapshot_dir", default_value="/tmp"),
        DeclareLaunchArgument("robot", default_value="",
                              description="robots/<name> from tools/robot_studio (default: the built-in drivebase)"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args_headless}.items(),
            condition=UnlessCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args_gui}.items(),
            condition=IfCondition(gui)),

        OpaqueFunction(function=robot_nodes),

        # Spawn where the routine physically starts, so the controller's belief
        # (odom_xyt_set) and Gazebo's ground truth describe the same robot.
        OpaqueFunction(function=spawn),

        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(gz_share, "config", "bridge.yaml"), "use_sim_time": True}]),

    ])
