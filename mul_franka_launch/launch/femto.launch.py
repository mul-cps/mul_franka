import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


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
    # parameters:
    # https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html
    # modes:
    # https://www.orbbec.com/documentation/femto-bolt-hardware-specifications/

    color_exposure_parameter_name = "color_exposure"
    color_exposure = LaunchConfiguration(color_exposure_parameter_name)
    color_exposure_arg = DeclareLaunchArgument(
        color_exposure_parameter_name,
        default_value=str(0), # auto-exposure by default
        description="colour camera exposure time [1, 300]",
    )

    femto = ComposableNode(
        package="orbbec_camera",
        name="camera",
        namespace="camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        remappings=[
            ("/camera/gyro_accel/sample", "/camera/imu"),
        ],
        extra_arguments=[
            {'use_intra_process_comms': False},
        ],
        parameters=[
            {'camera_name': 'camera'},
            {'use_hardware_time': True},
            {'enable_heartbeat': True},
            {'enable_publish_extrinsic': True},

            # compression settings
            {"color.image_raw.enable_pub_plugins": plugins},
            {"depth.image_raw.enable_pub_plugins": plugins},
            {"ir.image_raw.enable_pub_plugins": plugins},
            {".color.image_raw.compressed": compr_jpeg},
            {".depth.image_raw.compressed": compr_png},
            {".ir.image_raw.compressed": compr_png},

            # colour
            {'enable_color': True},
            {'color_width': 1280},
            {'color_height': 720},
            {'color_fps': 30},
            {'color_format': 'MJPG'},
            {'enable_color_auto_exposure': PythonExpression(['bool(int(', color_exposure, ') == 0)'])},
            {'enable_color_auto_white_balance': True},
            {'color_exposure': color_exposure},

            # depth, NFOV unbinned
            {'enable_depth': True},
            {'depth_width': 640},
            {'depth_height': 576},
            {'depth_fps': 30},
            {'depth_format': 'Y16'},
            {'depth_registration': True},
            {'align_mode': 'SW'},
            {'enable_frame_sync': True},
            {'enable_depth_scale': True},

            # IR
            {'enable_ir': True},
            {'ir_width': 640},
            {'ir_height': 576},
            {'ir_fps': 30},
            {'ir_format': 'Y16'},
            {'enable_ir_auto_exposure': True},

            # IMU
            {'enable_gyro': True},
            {'accel_rate': '200hz'},
            {'accel_range': '4g'},
            {'enable_accel': True},
            {'gyro_rate': '200hz'},
            {'gyro_range': '1000dps'},
            {'enable_sync_output_accel_gyro': True},

            # point cloud
            {'enable_colored_point_cloud': False},
            {'ordered_pc': True},
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
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            femto,
            rgbd,
        ],
        emulate_tty=True,
    )

    return launch.LaunchDescription([
        color_exposure_arg,
        container,
    ])
