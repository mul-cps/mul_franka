import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# Parameter name: rgb_camera.color_profile
#   Type: string
#   Description: Available options are:
# 1280x720x15
# 1280x720x30
# 1280x720x6
# 1920x1080x15
# 1920x1080x30
# 1920x1080x6
# 320x180x30
# 320x180x6
# 320x180x60
# 320x240x30
# 320x240x6
# 320x240x60
# 424x240x15
# 424x240x30
# 424x240x6
# 424x240x60
# 640x360x15
# 640x360x30
# 640x360x6
# 640x360x60
# 640x480x15
# 640x480x30
# 640x480x6
# 640x480x60
# 848x480x15
# 848x480x30
# 848x480x6
# 848x480x60
# 960x540x15
# 960x540x30
# 960x540x6
# 960x540x60

# Parameter name: depth_module.depth_profile
#   Type: string
#   Description: Available options are:
# 1280x720x15
# 1280x720x30
# 1280x720x6
# 256x144x300
# 256x144x90
# 424x240x15
# 424x240x30
# 424x240x6
# 424x240x60
# 424x240x90
# 480x270x15
# 480x270x30
# 480x270x6
# 480x270x60
# 480x270x90
# 640x360x15
# 640x360x30
# 640x360x6
# 640x360x60
# 640x360x90
# 640x480x15
# 640x480x30
# 640x480x6
# 640x480x60
# 640x480x90
# 848x100x100
# 848x100x300
# 848x480x15
# 848x480x30
# 848x480x6
# 848x480x60
# 848x480x90

plugins = ["image_transport/raw", "image_transport/compressed"]

# JPEG quality [0, 100]: lower values reduce the data size but increase artifacts
compr_jpeg = {
    "format": "jpeg",
    "jpeg_quality": 95,
}

# PNG compression level [0, 9]: higher values reduce the data size but increase CPU usage
compr_png = {
    "format": "png",
    "png_level": 1,
}


def generate_launch_description():
    mode_parameter_name = "mode"

    mode_arg = DeclareLaunchArgument(
        mode_parameter_name,
        description="colour and depth mode",
        default_value="640x480x60",
    )

    mode = LaunchConfiguration(mode_parameter_name)

    realsense = ComposableNode(
        package="realsense2_camera",
        name="camera",
        namespace="",
        plugin="realsense2_camera::RealSenseNodeFactory",
        remappings=[
            ("/camera/aligned_depth_to_color/camera_info", "/camera/depth/camera_info"),
            ("/camera/aligned_depth_to_color/image_raw", "/camera/depth/image_raw"),
            ("/camera/aligned_depth_to_color/image_raw/compressed", "/camera/depth/image_raw/compressed"),
        ],
        extra_arguments=[
            # The RealSense node does not support compressed image transport
            # when intra-process communication is enabled.
            {'use_intra_process_comms': False},
        ],
        parameters=[
            {"camera_name": "camera"},
            # compression settings
            {
                "camera": {
                    "color.image_raw": {
                        "enable_pub_plugins": plugins,
                        "compressed": compr_jpeg,
                    },
                    "depth.image_rect_raw": {
                        "enable_pub_plugins": plugins,
                        "compressed": compr_png,
                    },
                    "aligned_depth_to_color.image_raw": {
                        "enable_pub_plugins": plugins,
                        "compressed": compr_png,
                    },
                }
            },
            # colour
            {"enable_color": True},
            {"rgb_camera.color_profile": mode},
            {"rgb_camera.enable_auto_exposure": True},
            # depth
            {"enable_depth": True},
            {"depth_module.depth_profile": mode},
            {"depth_module.enable_auto_exposure": True},
            {"align_depth.enable": True},
            # IR
            {"enable_infra1": False},
            {"enable_infra2": False},
            # IMU
            {"enable_sync": True},
            {"enable_gyro": True},
            {"enable_accel": True},
            {"unite_imu_method": 2},  # linear_interpolation
            # point cloud
            {"pointcloud.enable": False},
            {"pointcloud.stream_filter": 2}, # RS2_STREAM_COLOR, https://github.com/IntelRealSense/librealsense/blob/v2.56.3/include/librealsense2/h/rs_sensor.h#L43-L57
            {"pointcloud.ordered_pc": True}, # ordered/structured point cloud (W x H x [x,y,z,r,g,b])
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
            ("depth_registered/image_rect", "/camera/depth/image_raw"),
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
            realsense,
            rgbd,
        ],
        emulate_tty=True,
    )

    return launch.LaunchDescription([mode_arg, container])
