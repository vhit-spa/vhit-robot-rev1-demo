#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class ElacTesterNode(Node):
    """
    Publishes simple cyclic JointTrajectory commands to vhit_elac_controller.

    Intended use:
      - bring up a simple elac system
      - make sure vhit_elac_controller is active
      - run this node to send small position trajectories
    """

    def __init__(self) -> None:
        super().__init__("elac_tester_node")

        self.declare_parameter(
            "controller_name",
            "vhit_elac_controller",
        )
        self.declare_parameter(
            "joints",
            ["elac_node",],
        )
        self.declare_parameter("amplitude", 0.05)
        self.declare_parameter("period_s", 4.0)
        self.declare_parameter("point_duration_s", 2.0)
        self.declare_parameter("publish_rate_hz", 0.5)
        self.declare_parameter("use_current_position_as_center", True)

        self.controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )
        self.joints: List[str] = [
            str(x)
            for x in self.get_parameter("joints").get_parameter_value().string_array_value
        ]
        self.amplitude = (
            self.get_parameter("amplitude").get_parameter_value().double_value
        )
        self.period_s = (
            self.get_parameter("period_s").get_parameter_value().double_value
        )
        self.point_duration_s = (
            self.get_parameter("point_duration_s").get_parameter_value().double_value
        )
        self.publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self.use_current_position_as_center = (
            self.get_parameter("use_current_position_as_center")
            .get_parameter_value()
            .bool_value
        )

        topic = f"/{self.controller_name}/joint_trajectory"
        self.publisher = self.create_publisher(JointTrajectory, topic, 10)

        self.current_positions = {}
        self.center_positions = {joint: 0.0 for joint in self.joints}
        self.center_initialized = not self.use_current_position_as_center

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            10,
        )

        timer_period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.publish_trajectory)

        self.start_time = self.get_clock().now()

        self.get_logger().info(f"Publishing test trajectories to {topic}")
        self.get_logger().info(f"Joints: {self.joints}")
        self.get_logger().info(
            f"Amplitude: {self.amplitude}, point duration: {self.point_duration_s}s"
        )

    def on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

        if self.center_initialized:
            return

        missing = [joint for joint in self.joints if joint not in self.current_positions]
        if missing:
            return

        self.center_positions = {
            joint: self.current_positions[joint] for joint in self.joints
        }
        self.center_initialized = True
        self.get_logger().info(
            f"Initialized center positions from /joint_states: {self.center_positions}"
        )

    def publish_trajectory(self) -> None:
        if not self.center_initialized:
            self.get_logger().warn(
                "Waiting for /joint_states before sending first trajectory..."
            )
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        phase = math.sin(2.0 * math.pi * elapsed / self.period_s)

        positions = [
            self.center_positions[joint] + self.amplitude * phase
            for joint in self.joints
        ]

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=self.point_duration_s).to_msg()

        msg.points.append(point)

        self.publisher.publish(msg)

        self.get_logger().info(
            "Published target: "
            + ", ".join(
                f"{joint}={position:.4f}"
                for joint, position in zip(self.joints, positions)
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElacTesterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()