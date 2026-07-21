import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(package_path, relative_path):
    with open(os.path.join(package_path, relative_path), "r") as file:
        return yaml.safe_load(file)


def load_file(package_path, relative_path):
    with open(os.path.join(package_path, relative_path), "r") as file:
        return file.read()


def generate_launch_description():
    moveit_config_path = get_package_share_directory("vhit_robot_moveit_config")

    robot_description_xml = xacro.process_file(
        os.path.join(
            moveit_config_path,
            "config",
            "vhit_robotics_rev_1.urdf.xacro",
        ),
        mappings={"hardware": "mock"},
    ).toxml()

    robot_description = {"robot_description": robot_description_xml}
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            moveit_config_path,
            "config/vhit_robotics_rev_1.srdf",
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            moveit_config_path,
            "config/kinematics.yaml",
        )
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(moveit_config_path, "config", "moveit.rviz"),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([rviz_node])
