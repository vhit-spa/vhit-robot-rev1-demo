import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)
    moveit_package = "vhit_robot_moveit_config"
    moveit_package_share = get_package_share_directory(moveit_package)

    xacro_mappings = {"hardware": hardware}
    if hardware == "real":
        xacro_mappings["elac_mapping_file"] = os.path.join(
            moveit_package_share,
            "config",
            "elac_mapping.yaml",
        )

    moveit_config = (
        MoveItConfigsBuilder(
            "vhit_robotics_rev_1",
            package_name=moveit_package,
        )
        .robot_description(
            file_path="config/vhit_robotics_rev_1.urdf.xacro",
            mappings=xacro_mappings,
        )
        .to_moveit_configs()
    )

    ros2_controllers = os.path.join(
        moveit_package_share,
        "config",
        "ros2_controllers.yaml",
    )

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
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description, ros2_controllers],
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

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
        ],
    )

    return [
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group,
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
