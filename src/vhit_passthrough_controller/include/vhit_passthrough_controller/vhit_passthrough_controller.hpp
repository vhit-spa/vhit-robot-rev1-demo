#ifndef V_HIT_PASSTHROUGH_CONTROLLER_HPP_
#define V_HIT_PASSTHROUGH_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>


#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace vhit_passthrough_controller
{

enum class ExecutionState
{
  IDLE,
  RECEIVING_GOAL,
  TRANSFERRING,
  EXECUTING,
  SUCCEEDED,
  ABORTED,
  ERROR
};

class VhitPassthroughController : public controller_interface::ControllerInterface
{
public:
  VhitPassthroughController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interfaces_;
  std::vector<std::string> state_interfaces_;
  std::atomic<size_t> joints_number_;

  std::vector<std::vector<double>> trajectory_positions_;
  std::vector<std::vector<double>> trajectory_velocities_;
  std::vector<std::vector<double>> trajectory_accelerations_;

  std::vector<double> trajectory_times_;

  std::vector<control_msgs::msg::JointTolerance> path_tolerances_;
  std::vector<control_msgs::msg::JointTolerance> goal_tolerances_;

  rclcpp::Duration goal_time_tolerance_{0, 0};

  size_t current_point_index_ = 0;

  bool transfer_complete_ = false;

  ExecutionState execution_state_ = ExecutionState::IDLE;

  // start the action server to receive trajectories from the ROS2 control action interface and send them to the hardware interface
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
  active_goal_;
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  void start_action_server();
  bool check_goal_tolerance();
  bool check_goal_positions(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  bool check_goal_velocities(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  bool check_goal_accelerations(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

};

} // namespace vhit_passthrough_controller
#endif // V_HIT_PASSTHROUGH_CONTROLLER_HPP_
