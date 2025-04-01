import launch
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnShutdown
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import FindExecutable, Command


def generate_launch_description():
    bag_file_path_param_name = "file"
    bag_file_path = LaunchConfiguration(bag_file_path_param_name)
    bag_file_path_arg = DeclareLaunchArgument(
        bag_file_path_param_name,
        description="path to bag file",
    )

    container_name = "/camera_container"
    node_name = "recorder"

    recorder = ComposableNode(
        package="rosbag2_transport",
        name="recorder",
        namespace="",
        plugin="rosbag2_transport::Recorder",
        extra_arguments=[
            {'use_intra_process_comms': True},
        ],
        parameters=[
            {"storage.uri": bag_file_path},
            {"topics": [
                "/camera/color/image_raw/compressed",
                "/camera/color/camera_info",
                "/camera/depth/image_raw/compressed",
                "/camera/depth/camera_info",
                "/camera/imu",
                "/tf",
                "/tf_static",
            ]},
        ],
    )

    load_composable_recorder = LoadComposableNodes(
        target_container=container_name,
         composable_node_descriptions=[
            recorder,
        ],
    )

    # get_component_id = Command([
    #     FindExecutable(name="ros2"),
    #     "component",
    #     "list",
    #     container_name,
    #     "| grep " + node_name + " | awk '{print $1}'"
    # ])

    # get_component_id = ([
    #     # FindExecutable(name="bash"),
    #     f"ros2 component list {container_name} | grep {node_name} | awk '{{print $1}}'"
    # ])

    # unload_recorder = ExecuteProcess(
    #     cmd=[
    #         FindExecutable(name='ros2'),
    #         'component',
    #         'unload',
    #         container_name,
    #         get_component_id,
    #     ],
    #     output='screen'
    # )

    # unload_recorder = ExecuteProcess(cmd=[
    #         # FindExecutable(name='ros2'),
    #         f"ros2 component unload {container_name} $(ros2 component list {container_name} | grep {node_name} | awk '{{print $1}}')"
    #         # f"ros2 component list {container_name} | grep {node_name} | awk '{{print $1}}'"
    #     ],
    #     output='screen',
    # )

    unload_recorder = ExecuteProcess(cmd=[
            # "bash", "-c",
            f"iii=$(ros2 component list {container_name} | grep {node_name} | awk '{{print $1}}') && echo $iii && ros2 component unload {container_name} $iii"
            # f"ros2 component list {container_name} | grep {node_name} | awk '{{print $1}}'"
        ],
        shell=True,
        output='screen',
        on_exit=[LogInfo(msg="exit?")],
    )

    # on_shutdown_handler = RegisterEventHandler(
    #     OnShutdown(on_shutdown=[unload_recorder])
    # )

    # Keeps the launch file running until Ctrl+C is pressed
    # keep_alive = TimerAction(
    #     period=1.0,  # Keep looping every second
    #     actions=[LogInfo(msg="Recorder running. Press Ctrl+C to stop...")]
    # )

    keep_alive = ExecuteProcess(
        # cmd=["bash", "-c", "'sleep infinity'"],
        cmd=['sleep infinity'],
        output="screen",
        shell=True,
        # name="keep_alive_process",
        on_exit=[unload_recorder],
    )

    return launch.LaunchDescription([
        load_composable_recorder,
        # on_shutdown_handler,
        keep_alive,
        bag_file_path_arg,
    ])
