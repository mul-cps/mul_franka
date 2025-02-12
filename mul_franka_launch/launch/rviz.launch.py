# MoveIt launch file adapted from https://github.com/frankaemika/franka_ros2/blob/v1.0.0/franka_fr3_moveit_config/launch/moveit.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_moveit_config"),
        "srdf",
        "panda_arm.srdf.xacro",
    )
    robot_description_semantic_config = Command(
        [FindExecutable(name="xacro"), " ", franka_semantic_xacro_file, " hand:=true"]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml("franka_moveit_config", "config/kinematics.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("franka_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("franka_moveit_config"), "rviz"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description_semantic,
            ompl_planning_pipeline_config,
            {"robot_description_kinematics": kinematics_yaml},
        ],
    )

    env_lc = SetEnvironmentVariable(name="LC_NUMERIC", value="C")

    return LaunchDescription(
        [
            env_lc,
            rviz_node,
        ]
    )
