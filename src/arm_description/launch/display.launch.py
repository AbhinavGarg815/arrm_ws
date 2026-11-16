"""Visualise the description on its own — no controllers, no MoveIt.

This is the file to reach for when you have changed the URDF and want to see
what it looks like, or drag a joint to its limit and check nothing intersects.

    ros2 launch arm_description display.launch.py

The joint_state_publisher_gui sliders are bounded by the URDF limits, so this
is also the quickest way to sanity-check a limit you have just edited in
config/joint_parameters.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("arm_description")

    # ParameterValue(..., value_type=str) is not optional. Without it, launch
    # tries to parse the xacro output as YAML, and a URDF is not YAML — you get
    # "Unable to parse the value of parameter robot_description as yaml", which
    # does not obviously point at the missing type hint.
    robot_description = {
        "robot_description": ParameterValue(Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "arm.urdf.xacro"]), " ",
            "prefix:=", LaunchConfiguration("prefix"), " ",
            # Always mock here: this launch file has no controller_manager, so
            # the hardware plugin would never be loaded anyway.
            "use_mock_hardware:=true",
        ]), value_type=str),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "prefix", default_value="",
            description="Link/joint name prefix passed to xacro.",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start joint_state_publisher_gui with a slider per joint.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])],
        ),
    ])
