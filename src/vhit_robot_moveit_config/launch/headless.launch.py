from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "vhit_robotics_rev_1",
            package_name="vhit_robot_moveit_config"
        )
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory(
        "vhit_robot_moveit_config"
    )

    controllers_yaml = os.path.join(
        pkg_share,
        "config",
        "ros2_controllers.yaml"
    )

    # Robot state publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # Static world -> base TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "world",
            "base"
        ],
    )

    # ros2_control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controllers_yaml,
        ],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Main trajectory controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "vhit_arm_rev1_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # Move group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        static_tf,
        rsp_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        move_group_node,
    ])
