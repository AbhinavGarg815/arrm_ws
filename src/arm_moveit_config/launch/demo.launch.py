"""Bring up the whole stack: description, controllers, MoveIt and RViz.

    # planner work, nothing connected, nothing can break:
    ros2 launch arm_moveit_config demo.launch.py

    # drive the real arm (see docs/PROJECT.md for what must be true first):
    ros2 launch arm_moveit_config demo.launch.py use_mock_hardware:=false

What starts, and why each piece is needed:

    robot_state_publisher   turns /joint_states into the TF tree everything
                            else reads
    ros2_control_node       loads the hardware component named in the URDF and
                            runs the controllers at 100 Hz
    spawner x3              activates joint_state_broadcaster (publishes
                            /joint_states), then arm_controller, then
                            gripper_controller (both accept
                            FollowJointTrajectory). Chained, because MoveIt
                            needs joint states before it will accept a plan.
    move_group              the planner
    rviz2                   the MotionPlanning panel: drag the marker, hit Plan
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from arm_moveit_config.moveit_common import build, declare_hardware_arguments


def launch_setup(context, *args, **kwargs):
    cfg = lambda name: LaunchConfiguration(name).perform(context)

    moveit_config = build(
        context,
        use_mock_hardware=cfg("use_mock_hardware"),
        mega_port=cfg("mega_port"),
        esp32_port=cfg("esp32_port"),
        use_fake_gripper=cfg("use_fake_gripper"),
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("arm_description"), "config", "ros2_controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # The controller manager takes the URDF from robot_state_publisher's
    # /robot_description topic, not as a parameter. Passing it as a parameter
    # too is the pre-Jazzy pattern: the manager then loads it twice and logs
    # "ResourceManager has already loaded a urdf ... Ignoring attempt to reload".
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
        remappings=[("~/robot_description", "/robot_description")],
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", *extra],
            output="screen",
        )

    # The hardware starts unconfigured, so both transitions happen here as
    # explicit commands: configure opens the serial ports and checks both boards
    # answer, activate energises the arm. Kept separate because they fail for
    # different reasons - a missing cable or wrong port versus a board refusing
    # to enable - and either way it is a failed command with a reason rather
    # than an aborted controller manager.
    #
    # This must precede the controllers: a controller cannot claim interfaces
    # from a component that is not active.
    configure_hardware = ExecuteProcess(
        cmd=["ros2", "control", "set_hardware_component_state", "arm_system", "inactive"],
        output="screen",
    )
    activate_hardware = ExecuteProcess(
        cmd=["ros2", "control", "set_hardware_component_state", "arm_system", "active"],
        output="screen",
    )

    # joint_state_broadcaster first: nothing downstream works without
    # /joint_states, and a controller that activates before it has state has
    # nothing to hold position against.
    jsb_spawner = spawner("joint_state_broadcaster")
    arm_spawner = spawner("arm_controller")
    # The gripper has no hardware, so its controller only ever moved a number
    # in a loopback - off by default, so a goal sent to it cannot look like it
    # did something. The joint still exists in the model and reports its URDF
    # initial_value, so nothing downstream changes shape. Turn it back on with
    # gripper_controller:=true when the N20 + DRV8871 is wired.
    gripper_spawner = spawner("gripper_controller")

    chain_activate = RegisterEventHandler(
        OnProcessExit(target_action=configure_hardware, on_exit=[activate_hardware])
    )
    chain_jsb = RegisterEventHandler(
        OnProcessExit(target_action=activate_hardware, on_exit=[jsb_spawner])
    )
    chain_arm = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )
    chain_gripper = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner, on_exit=[gripper_spawner]),
        condition=IfCondition(LaunchConfiguration("gripper_controller")),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("arm_moveit_config"), "rviz", "moveit.rviz"])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return [robot_state_publisher, control_node,
            TimerAction(period=4.0, actions=[configure_hardware]),
            chain_activate, chain_jsb, chain_arm, chain_gripper, move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        *declare_hardware_arguments(),
        DeclareLaunchArgument(
            "gripper_controller", default_value="false",
            description=(
                "Spawn the gripper controller. Off by default: the gripper is not "
                "built, so it only drives a loopback. Set true once it is wired."
            ),
        ),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start RViz with the MotionPlanning panel."),
        OpaqueFunction(function=launch_setup),
    ])
