# include franka moveit and realsens launch files

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    franka_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mul_franka_launch"), "launch", "franka.launch.py"])
        ]),
        launch_arguments={
            "robot_ip": "192.168.13.1",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
        }.items(),
    )

    moveit_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mul_franka_launch"), "launch", "moveit.launch.py"])
        ]),
    )

    realsense_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mul_franka_launch"), "launch", "realsense.launch.py"])
        ]),
        launch_arguments={
            "mode": "640x480x30",
        }.items(),
    )

    return LaunchDescription([
        franka_launch_file,
        moveit_launch_file,
        realsense_launch_file,
    ])
