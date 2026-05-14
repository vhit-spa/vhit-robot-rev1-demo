from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    pkg_share = FindPackageShare("vhit_robot_demo").find(
        "vhit_robot_demo"
    )

    xacro_file = os.path.join(
        pkg_share,
        "urdf",
        "vhit_robotics_rev_1.urdf.xacro"
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file
        ])
    }

    rviz_config = os.path.join(
        pkg_share,
        "rviz",
        "view.rviz"
    )

    return LaunchDescription([

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    ])