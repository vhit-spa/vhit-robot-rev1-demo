from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import yaml
import xacro


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r") as file:
        return file.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    package_name = "vhit_robot_moveit_config"
    description_package_name = "vhit_robot_demo"

    package_share = get_package_share_directory(package_name)

    # Robot description
    xacro_file = os.path.join(
        package_share,
        "config",
        "vhit_robotics_rev_1.urdf.xacro"
    )

    # Joint mapping to ethercat
    elac_mapping_file = os.path.join(
        package_share,
        "config",
        "elac_mapping.yaml"
    )
    
    robot_description_config = xacro.process_file(
        xacro_file,
        # Optional: Pass arguments to the xacro file if needed
        mappings={
            "hardware": "real",
            "elac_mapping_file": elac_mapping_file,
            # "initial_positions_file": "initial_positions.yaml"
        }
    )
    
    robot_description = {
        "robot_description": robot_description_config.toxml()
    }

    # Semantic description
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            package_name,
            "config/vhit_robotics_rev_1.srdf"
        )
    }

    # Kinematics
    robot_description_kinematics = load_yaml(
        package_name,
        "config/kinematics.yaml"
    )

    # Joint limits
    joint_limits = {
        "robot_description_planning": load_yaml(
            package_name,
            "config/joint_limits.yaml"
        )
    }

    # OMPL pipeline
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }

    # Controllers
    controllers_yaml = load_yaml(
        package_name,
        "config/moveit_controllers.yaml"
    )

    moveit_controllers = controllers_yaml

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control
    ros2_controllers_path = os.path.join(
        package_share,
        "config",
        "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
            # Realtime execution parameters
            # {
            #     "lock_memory": True,
            #     "cpu_affinity": [1],
            #     "thread_priority": 50,
            # }
        ],
        output="screen",
    )

    # Controller spawners
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

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "vhit_arm_rev1_controller",
            "--controller-manager",
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
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base"],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
    ])