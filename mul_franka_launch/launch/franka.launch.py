# Franka launch file adapted from https://github.com/frankaemika/franka_ros2/blob/v1.0.0/franka_bringup/launch/franka.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_parameter_name = "robot_ip"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    franka_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("mul_franka_description"),
            "robots",
            "panda_arm.urdf.xacro",
        ]
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " hand:=true",
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
        ]
    )

    franka_controllers = IfElseSubstitution(
        condition=NotSubstitution(use_fake_hardware),
        if_value=PathJoinSubstitution(
            [
                FindPackageShare("franka_moveit_config"),
                "config",
                "panda_ros_controllers.yaml",
            ]
        ),
        else_value=PathJoinSubstitution(
            [
                FindPackageShare("moveit_resources_panda_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ]
        ),
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        description="Hostname or IP address of the robot.",
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value="false",
        description="Use fake hardware",
    )

    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value="false",
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name
        ),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "franka/joint_states",
                    "panda_gripper/joint_states",
                ],
                "rate": 30,
            }
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            franka_controllers,
        ],
        remappings=[("joint_states", "franka/joint_states")],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    franka_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "panda_arm_controller"],
        output="screen",
    )

    franka_robot_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_robot_state_broadcaster"],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_gripper"),
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            use_fake_hardware_parameter_name: use_fake_hardware,
        }.items(),
    )

    return LaunchDescription(
        [
            # args
            robot_arg,
            use_fake_hardware_arg,
            fake_sensor_commands_arg,
            # state
            robot_state_publisher,
            joint_state_publisher,
            # controller
            ros2_control_node,
            franka_controllers_spawner,
            franka_robot_state_broadcaster_spawner,
            # launch files
            gripper_launch_file,
        ]
    )
