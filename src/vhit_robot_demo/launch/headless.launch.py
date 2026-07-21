import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(package_path, relative_path):
    with open(os.path.join(package_path, relative_path), "r") as file:
        return file.read()


def load_yaml(package_path, relative_path):
    with open(os.path.join(package_path, relative_path), "r") as file:
        return yaml.safe_load(file)


def launch_setup(context, *args, **kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)
    moveit_config_path = get_package_share_directory("vhit_robot_moveit_config")

    xacro_mappings = {"hardware": hardware}
    if hardware == "real":
        xacro_mappings["elac_mapping_file"] = os.path.join(
            moveit_config_path,
            "config",
            "elac_mapping.yaml",
        )

    robot_description_xml = xacro.process_file(
        os.path.join(
            moveit_config_path,
            "config",
            "vhit_robotics_rev_1.urdf.xacro",
        ),
        mappings=xacro_mappings,
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
    joint_limits = {
        "robot_description_planning": load_yaml(
            moveit_config_path,
            "config/joint_limits.yaml",
        )
    }

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }

    moveit_controllers = load_yaml(
        moveit_config_path,
        "config/moveit_controllers.yaml",
    )
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

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base"],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            os.path.join(
                moveit_config_path,
                "config",
                "ros2_controllers.yaml",
            ),
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "vhit_arm_rev1_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    return [
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware",
                default_value="mock",
                choices=["mock", "gazebo", "real"],
                description="ros2_control hardware implementation to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
