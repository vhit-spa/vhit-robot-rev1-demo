import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("vhit_elac_tester")
    use_mock_hardware = as_bool(
        LaunchConfiguration("use_mock_hardware").perform(context)
    )
    xacro_name = (
        "elac_tester_mock.urdf.xacro"
        if use_mock_hardware
        else "elac_tester.urdf.xacro"
    )
    xacro_file = os.path.join(package_share, "config", xacro_name)
    controllers_file = os.path.join(package_share, "config", "ros2_controllers.yaml")

    xacro_mappings = {
        "initial_position": LaunchConfiguration("initial_position").perform(context),
    }
    if not use_mock_hardware:
        xacro_mappings["hardware"] = "real"
        xacro_mappings["dl_node"] = LaunchConfiguration("dl_node").perform(context)

    robot_description_config = xacro.process_file(xacro_file, mappings=xacro_mappings)
    robot_description = {"robot_description": robot_description_config.toxml()}

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
        parameters=[robot_description, controllers_file],
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

    elac_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "vhit_elac_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    tester_node = Node(
        package="vhit_elac_tester",
        executable="elac_tester_node",
        name="elac_tester_node",
        output="screen",
        parameters=[
            {
                "controller_name": "vhit_elac_controller",
                "joints": ["elac_node"],
                "amplitude": ParameterValue(
                    LaunchConfiguration("amplitude"),
                    value_type=float,
                ),
                "period_s": ParameterValue(
                    LaunchConfiguration("period_s"),
                    value_type=float,
                ),
                "point_duration_s": ParameterValue(
                    LaunchConfiguration("point_duration_s"),
                    value_type=float,
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"),
                    value_type=float,
                ),
                "use_current_position_as_center": True,
            }
        ],
    )

    start_tester_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=elac_controller_spawner,
            on_exit=[tester_node],
        )
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        elac_controller_spawner,
        start_tester_after_controller,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dl_node",
                default_value="ELAC_node_LAN9253",
                description="ctrlX Data Layer ELAC node name for real hardware.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use mock_components/GenericSystem instead of the real VHIT hardware driver.",
            ),
            DeclareLaunchArgument(
                "initial_position",
                default_value="0.0",
                description="Initial joint position used by ros2_control.",
            ),
            DeclareLaunchArgument("amplitude", default_value="0.02"),
            DeclareLaunchArgument("period_s", default_value="6.0"),
            DeclareLaunchArgument("point_duration_s", default_value="2.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="0.5"),
            OpaqueFunction(function=launch_setup),
        ]
    )
