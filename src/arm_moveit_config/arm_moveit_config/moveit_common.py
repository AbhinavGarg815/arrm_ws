"""Shared MoveIt configuration builder.

Exists because MoveItConfigsBuilder resolves its file paths and xacro mappings
when the launch file is *parsed*, not when it *runs*. A LaunchConfiguration
passed into `mappings` therefore arrives as an unevaluated substitution object
and xacro receives literal garbage. The fix is to build the config inside an
OpaqueFunction, where every argument has a real string value — which is what
every launch file here does, via `build()`.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


# Stable device paths, not /dev/ttyACM0 and /dev/ttyUSB0: those are assigned in
# enumeration order and move on a replug, which leaves a hardware component that
# cannot configure while RViz plans and "executes" against nothing.
#
# Both boards are matched by device identity. The Mega has its own serial
# number. The CP2102 on the ESP32 board reports the generic serial "0001", so
# its name is unique only while it is the ONLY CP2102 attached - if a second
# appears, write a udev rule keyed on the physical port for a stable
# /dev/arm-wrist rather than using a socket-dependent path.
MEGA_PORT = ("/dev/serial/by-id/"
             "usb-Arduino__www.arduino.cc__0042_85339313233351213111-if00")
ESP32_PORT = ("/dev/serial/by-id/"
              "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")


def build(context, use_mock_hardware="true", mega_port=MEGA_PORT,
          esp32_port=ESP32_PORT, use_fake_gripper="true", prefix=""):
    """Return a MoveItConfigs with the hardware switch already resolved."""
    description_share = FindPackageShare("arm_description").perform(context)

    return (
        MoveItConfigsBuilder("arm", package_name="arm_moveit_config")
        .robot_description(
            file_path=f"{description_share}/urdf/arm.urdf.xacro",
            mappings={
                "prefix": prefix,
                "use_mock_hardware": use_mock_hardware,
                "use_fake_gripper": use_fake_gripper,
                "mega_port": mega_port,
                "esp32_port": esp32_port,
            },
        )
        .robot_description_semantic(file_path="config/arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            # False on purpose: robot_state_publisher already publishes
            # /robot_description, and a second publisher means subscribers take
            # whichever copy arrives first. One topic, one source.
            publish_robot_description=False,
            # The SRDF has no other publisher, so move_group keeps this one.
            publish_robot_description_semantic=True,
            # RViz's MotionPlanning display draws the scene from this topic.
            # The "No 3D sensor plugin(s) defined for octomap updates" error at
            # startup is expected: there is no depth camera on this arm yet.
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )


def declare_hardware_arguments():
    """The four arguments every launch file in this package accepts."""
    from launch.actions import DeclareLaunchArgument
    return [
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description=(
                "true: mock_components/GenericSystem loopback - no serial port, "
                "no firmware, nothing can be damaged. "
                "false: drive the real Mega and ESP32."
            ),
        ),
        DeclareLaunchArgument(
            "mega_port", default_value=MEGA_PORT,
            description=(
                "Serial device for the 3 DS51150 joints (Arduino Mega 2560). "
                "Defaults to the by-id path, which does not move when devices "
                "re-enumerate - unlike /dev/ttyACM0."
            ),
        ),
        DeclareLaunchArgument(
            "esp32_port", default_value=ESP32_PORT,
            description=(
                "Serial device for the 3 ST3215 wrist joints (Waveshare ESP32). "
                "Defaults to the by-path name: the board's USB serial number is "
                "not unique, so it is identified by which socket it is in. Pass "
                "this explicitly if you move it."
            ),
        ),
        DeclareLaunchArgument(
            "use_fake_gripper", default_value="true",
            description=(
                "Keep the gripper joint on loopback even with real hardware. "
                "The N20 + lead screw gripper is not wired yet."
            ),
        ),
    ]
