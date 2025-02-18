import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    bag_file_path_param_name = "file"
    bag_file_path = LaunchConfiguration(bag_file_path_param_name)
    bag_file_path_arg = DeclareLaunchArgument(
        bag_file_path_param_name,
        description="path to bag file",
    )

    player = ComposableNode(
        package="rosbag2_transport",
        name="player",
        namespace="",
        plugin="rosbag2_transport::Player",
        extra_arguments=[
            {'use_intra_process_comms': True},
        ],
        parameters=[
            {"storage.uri": bag_file_path},
        ],
    )

    rgbd = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzrgbNode",
        name="rgbd",
        namespace="",
        extra_arguments=[
            {'use_intra_process_comms': True},
        ],
        remappings=[
            # input
            ("depth_registered/image_rect", "/camera/aligned_depth_to_color/image_raw"),
            ("rgb/image_rect_color", "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            # output
            ("points", "/camera/points"),
        ],
    )

    container = ComposableNodeContainer(
        name="image_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            player,
            rgbd,
        ],
        emulate_tty=True,
    )

    return launch.LaunchDescription([
        container,
        bag_file_path_arg,
    ])
