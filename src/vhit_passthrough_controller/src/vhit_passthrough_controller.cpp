#include "vhit_passthrough_controller/vhit_passthrough_controller.hpp"

#include "rclcpp/logging.hpp"

namespace vhit_passthrough_controller
{

VhitPassthroughController::VhitPassthroughController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn VhitPassthroughController::on_init()
{
  // Initialization code here
  joint_names_.clear();
  command_interfaces_.clear();
  state_interfaces_.clear();

  auto param_listener = get_node();
  joint_names_ = param_listener->get_parameter("joint_names").as_string_array();
  command_interfaces_ = param_listener->get_parameter("command_interfaces").as_string_array();
  state_interfaces_ = param_listener->get_parameter("state_interfaces").as_string_array();
  joints_number_ = joint_names_.size();

  RCLCPP_INFO(
    rclcpp::get_logger("VhitPassthroughController"),
    "Initializing passthrough controller...");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration VhitPassthroughController::
command_interface_configuration() const
{
  // Define command interface configuration here
  controller_interface::InterfaceConfiguration config;
  // For example, if the controller needs position command interfaces for joints:
  // config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // config.names = {"joint1/position", "joint2/position", "joint3/position", "joint4/position", "joint5/position"};
  return config;
}

controller_interface::InterfaceConfiguration VhitPassthroughController::
state_interface_configuration() const
{
  // Define state interface configuration here
  controller_interface::InterfaceConfiguration config;
  // For example, if the controller needs position state interfaces for joints:
  // config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // config.names = {"joint1/position", "joint2/position", "joint3/position", "joint4/position", "joint5/position"};
  return config;
}

controller_interface::CallbackReturn VhitPassthroughController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Configuration code here
  start_action_server();

  RCLCPP_INFO(
    rclcpp::get_logger("VhitPassthroughController"),
    "Configuring passthrough controller from state '%s'", previous_state.label().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

void VhitPassthroughController::start_action_server()
{
  action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
    get_node(), std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(
      &VhitPassthroughController::goal_received_callback, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&VhitPassthroughController::goal_cancelled_callback, this, std::placeholders::_1),
    std::bind(&VhitPassthroughController::goal_accepted_callback, this, std::placeholders::_1));
  return;
}

controller_interface::CallbackReturn VhitPassthroughController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Activation code here
  RCLCPP_INFO(
    rclcpp::get_logger("VhitPassthroughController"),
    "Activating passthrough controller from state '%s'", previous_state.label().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VhitPassthroughController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Deactivation code here
  RCLCPP_INFO(
    rclcpp::get_logger("VhitPassthroughController"),
    "Deactivating passthrough controller from state '%s'", previous_state.label().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VhitPassthroughController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // Update code here
  // This function will be called periodically when the controller is active
  // The main control logic should be implemented here, e.g. reading state interfaces, computing commands, writing command interfaces

  return controller_interface::return_type::OK;
}

} // namespace vhit_passthrough_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  vhit_passthrough_controller::VhitPassthroughController,
  controller_interface::ControllerInterface)
