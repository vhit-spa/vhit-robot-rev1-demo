from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vhit_elac_tester",
                executable="elac_tester_node",
                name="elac_tester_node",
                output="screen",
                parameters=[
                    {
                        "controller_name": "vhit_elac_controller",
                        "joints": [
                            "elac_node",
                        ],
                        "amplitude": 0.02,
                        "period_s": 6.0,
                        "point_duration_s": 2.0,
                        "publish_rate_hz": 0.5,
                        "use_current_position_as_center": True,
                    }
                ],
            )
        ]
    )