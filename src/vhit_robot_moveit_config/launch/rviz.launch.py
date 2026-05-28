from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro

import yaml

def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    package_name = "vhit_robot_moveit_config"

    # URDF
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "vhit_robotics_rev_1.urdf.xacro"
    )

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            "hardware": "mock"
        }
    )

    robot_description = {
        "robot_description": robot_description_config.toxml()
    }

    # SRDF
    srdf_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "vhit_robotics_rev_1.srdf"
    )

    with open(srdf_file, "r") as f:
        robot_description_semantic_config = f.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "moveit.rviz"
    )

    # Kinematics
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            package_name,
            "config/kinematics.yaml"
        )
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        rviz_node
    ])