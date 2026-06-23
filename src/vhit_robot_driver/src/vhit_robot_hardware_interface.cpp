#include "vhit_robot_driver/vhit_robot_hardware_interface.hpp"
#include "ctrlx_datalayer_helper.h"
#include "shared_memory_helper.hpp"

#include "rclcpp/logging.hpp"

namespace vhit_robot_driver
{
// ============================================================
// Lifecycle Methods
// ============================================================

/**
     * Called once when the hardware plugin is created.
     *
     * Responsibilities:
     * - Parse URDF hardware parameters
     * - Validate joint/interface configuration
     * - Allocate internal vectors/buffers
     * - Initialize communication objects (without connecting yet)
     */
hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{

  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Additional initialization here
  // Parse joints
  num_joints_ = info_.joints.size();

  // Allocate vectors
  command_joint_positions_.resize(num_joints_, 0.0);
  current_joint_positions_.resize(num_joints_, 0.0);

  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "Joint '%s' added to hardware interface",
      joint.name.c_str());

    if (joint.parameters.find("DL_node") == joint.parameters.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "VhitRobotHardwareInterface"), "Missing parameter %s for joint %s",
        "DL_node", joint.name.c_str());
      return hardware_interface::CallbackReturn::FAILURE;
    }

    // SharedMemoryArea indexes variables by memory-map relative names
    JointDatalayerMapping jointMapping;
    jointMapping.joint_name = joint.name;
    jointMapping.ethercat_node = joint.parameters.at("DL_node");
    jointMapping.actual_position_variable = joint.parameters.at("DL_node") + "/" +
      g_positionActualValuePDO_;
    jointMapping.target_position_variable = joint.parameters.at("DL_node") + "/" +
      g_positionTargetValuePDO_;

    joint_dl_mappings_.emplace_back(jointMapping);

    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "creating a position CommandInterface for type <INT32> that maps from [%s, %s, %s]",
      joint.name.c_str(), jointMapping.actual_position_variable.c_str(),
      jointMapping.target_position_variable);

  }

  // Parse connection parameters for ctrlX CORE from hardware info if provided, otherwise use default values
  auto ip_param_it = info_.hardware_parameters.find("ip_address");
  if (ip_param_it != info_.hardware_parameters.end()) {
    ip_address_ = ip_param_it->second;
  } else {
    ip_address_ = "10.0.2.2"; // Default to ctrlX CORE virtual with port forwarding
  }
  auto username_param_it = info_.hardware_parameters.find("username");
  if (username_param_it != info_.hardware_parameters.end()) {
    username_ = username_param_it->second;
  } else {
    username_ = "boschrexroth";
  }
  auto password_param_it = info_.hardware_parameters.find("password");
  if (password_param_it != info_.hardware_parameters.end()) {
    password_ = password_param_it->second;
  } else {
    password_ = "boschrexroth";
  }
  auto ssl_port_param_it = info_.hardware_parameters.find("ssl_port");
  if (ssl_port_param_it != info_.hardware_parameters.end()) {
    ssl_port_ = std::stoi(ssl_port_param_it->second);
  } else {
    ssl_port_ = 8443;
  }

  datalayer_ = std::make_unique<comm::datalayer::DatalayerSystem>();

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
     * Called when transitioning from UNCONFIGURED -> INACTIVE.
     *
     * Responsibilities:
     * - Establish communication with ctrlX
     * - Connect to Data Layer
     * - Verify robot/controller availability
     * - Initialize robot state
     * - Prepare resources
     */
hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Configuration code here
  // datalayer connection initialization
  if (!datalayer_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Datalayer system is not initialized");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  datalayer_->start(false);

  // If running in snap environment, use IPC connection, otherwise use TCP connection with provided parameters
  if (isSnap()) {
    RCLCPP_INFO(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Running in snap environment, using IPC connection to ctrlX CORE");
    client_ = getClient(*datalayer_, "", "", "", 0);

  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Running in non-snap environment, attempting to connect to ctrlX CORE at %s:%d",
      ip_address_.c_str(),
      ssl_port_);
    client_ = getClient(*datalayer_, ip_address_, username_, password_, ssl_port_);
  }

  if (client_ == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Failed to create datalayer client instance");
    return hardware_interface::CallbackReturn::FAILURE;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Successfully created datalayer client");
  }

  if (!client_->isConnected()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Failed to connect to ctrlX CORE");
    return hardware_interface::CallbackReturn::FAILURE;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Successfully connected to ctrlX CORE");
  }

  // Test datalayer connection first
  comm::datalayer::Variant variant;
  auto result = client_->readSync("framework/metrics/system/cpu-utilisation-percent", &variant);
  if (STATUS_SUCCEEDED(result)) {
    RCLCPP_INFO(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Successfully read from ctrlX CORE: CPU Utilization = %f", double(variant));
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Failed to read from ctrlX CORE: %s", result.toString());
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // If datalayer works, initialize the shared memory maps
  readMemoryArea_ = std::make_unique<SharedMemoryArea>(
    datalayer_.get(), client_, g_ethercatReadingArea_);
  writeMemoryArea_ = std::make_unique<SharedMemoryArea>(
    datalayer_.get(), client_, g_ethercatWritingArea_);

  // Get memory mappings
  std::string what;
  result = readMemoryArea_->refresh_map(what);
  if (STATUS_FAILED(result)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"), what.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"), result.toString());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("VhitRobotHardwareInterface"), what.c_str());

  result = writeMemoryArea_->refresh_map(what);
  if (STATUS_FAILED(result)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"), what.c_str());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("VhitRobotHardwareInterface"), what.c_str());

  // Interface states - Datalayer mapping
  // Check correspondence between state_interfaces_to_dl_states_ and SharedMemoryVariable
  auto readMemoryAreaVars = readMemoryArea_->getVariables();
  for (int i = 0; i < num_joints_; i++) {
    if (readMemoryAreaVars.find(joint_dl_mappings_[i].actual_position_variable) ==
      readMemoryAreaVars.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("VhitRobotHardwareInterface"),
        "Mapping not found: [%s, %s]", joint_names_[i].c_str(),
        joint_dl_mappings_[i].actual_position_variable.c_str());
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  // Interface commands - Datalayer mapping
  // Check correspondence between command_interfaces_to_dl_commands_ and SharedMemoryVariable
  auto writeMemoryAreaVars = writeMemoryArea_->getVariables();
  for (int i = 0; i < num_joints_; i++) {
    if (writeMemoryAreaVars.find(joint_dl_mappings_[i].target_position_variable) ==
      writeMemoryAreaVars.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("VhitRobotHardwareInterface"),
        "Mapping not found: [%s, %s]", joint_names_[i].c_str(),
        joint_dl_mappings_[i].target_position_variable.c_str());
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
     * Called before shutdown/destruction.
     *
     * Responsibilities:
     * - Disconnect from ctrlX
     * - Release resources
     * - Close sockets/connections
     */
hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  // Cleanup code here
  if (client_ != nullptr) {
    delete client_;
    client_ = nullptr;
  }
  if (datalayer_) {
    datalayer_->stop();
    datalayer_.reset();
    datalayer_ = nullptr;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
     * Called when transitioning from INACTIVE -> ACTIVE.
     *
     * Responsibilities:
     * - Enable command streaming
     * - Enable trajectory execution
     * - Reset command buffers
     * - Ensure robot is ready for motion
     */
hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Activation code here
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
     * Called when transitioning from ACTIVE -> INACTIVE.
     *
     * Responsibilities:
     * - Stop motion safely
     * - Disable command execution
     * - Flush pending trajectories
     */
hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Deactivation code here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Shutdown code here
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
     * Called on fatal hardware/system error.
     *
     * Responsibilities:
     * - Put robot in safe state
     * - Report errors
     * - Attempt controlled recovery if desired
     */
hardware_interface::CallbackReturn VhitRobotHardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  // Error handling code here
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VhitRobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(
    rclcpp::get_logger("VhitRobotHardwareInterface"),
    "Exporting state interfaces:");

  // Export state interfaces here
  for (size_t i = 0; i < num_joints_; i++) {

    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "creating a position StateInteface for %s",
      joint_names_[i].c_str());

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], "position", &current_joint_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VhitRobotHardwareInterface::
export_command_interfaces()
{

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  RCLCPP_INFO(
    rclcpp::get_logger("VhitRobotHardwareInterface"),
    "Exporting command interfaces:");

  // Export command interfaces here
  for (size_t i = 0; i < num_joints_; i++) {

    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "creating a position CommandInterfaces for %s",
      joint_names_[i].c_str());

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], "position", &command_joint_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type VhitRobotHardwareInterface::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // Read from hardware here
  // Read the current state of the hardware frome the datalayer
  // Update the state interfaces with the read values
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VhitRobotHardwareInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // Write to hardware here
  return hardware_interface::return_type::OK;
}

} // namespace vhit_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vhit_robot_driver::VhitRobotHardwareInterface,
  hardware_interface::SystemInterface)
