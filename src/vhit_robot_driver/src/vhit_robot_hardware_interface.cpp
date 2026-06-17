#include "vhit_robot_driver/vhit_robot_hardware_interface.hpp"
#include "ctrlx_datalayer_helper.h"

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
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "Joint '%s' added to hardware interface",
      joint.name.c_str());
  }

  auto num_joints = joint_names_.size();
  if (num_joints != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "Expected %zu joints but got %zu", NUM_JOINTS, num_joints);
    return hardware_interface::CallbackReturn::ERROR;
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
  // Interface states - Datalayer mapping
  for (auto & [name, dl_type] : state_interfaces_to_dl_states_) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "Mapping state interfaces <%s>.",
      name.c_str());
    comm::datalayer::Variant readVariant;
    auto res = client_->readSync(dl_type.address(), &readVariant);
    if (STATUS_FAILED(res)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "VhitRobotHardwareInterface"), "Failed to get initial value for StateInterface<%s>.",
        name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "DataLayerHardwareInterface_NRT"), "Get initial values from StateInteface<%s> which is:{%f}",
      name.c_str(), dl_type.value);
  }

  // Command interfaces - Datalayer mapping
  for (auto & [name, dl_type] : command_interfaces_to_dl_commands_) {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "Mapping command interfaces <%s>.",
      name.c_str());
    comm::datalayer::Variant readVariant;
    auto res = client_->readSync(dl_type.address(), &readVariant);
    if (STATUS_FAILED(res)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "VhitRobotHardwareInterface"), "Failed to get initial value for StateInterface<%s>.",
        name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(
        "DataLayerHardwareInterface_NRT"), "Get initial values from StateInteface<%s> which is:{%f}",
      name.c_str(), dl_type.value);
  }

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
  comm::datalayer::Variant variant;
  variant.setType(comm::datalayer::VariantType::ARRAY_OF_FLOAT32);
  variant.setValue(std::vector<std::array<double, NUM_JOINTS>>{});
  auto result = client_->createSync(
    g_plcControllerApplication + "/" + g_plcPassthroughControllerAddress + "/" + g_trajectoryNode + "/positions",
    &variant);
  if (STATUS_FAILED(result)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("VhitRobotHardwareInterface"),
      "Failed to create trajectory node for passthrough trajectory controller. Error: %s",
      result.toString());
    return hardware_interface::CallbackReturn::FAILURE;
  }
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
  std::string elac_mapping_node;
  std::string full_variable_address;

  // Export state interfaces here
  for (size_t i = 0; i < NUM_JOINTS; i++) {

    elac_mapping_node = info_.joints[i].parameters["DL_node"];

    full_variable_address = g_ethercatReadingBaseAddress + "/" + elac_mapping_node + "/" +
      g_positionActualValuePDO;

    state_interfaces_to_dl_states_.emplace(
      std::make_pair(
        full_variable_address,
        DatalayerType(
          std::numeric_limits<double>::quiet_NaN(), comm::datalayer::VariantType::INT32,
          full_variable_address))
    );

    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "creating a position StateInteface for type <INT32> that maps from [%s, %s]",
      full_variable_address.c_str(), state_interfaces_to_dl_states_.at(
        full_variable_address).address().c_str());

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], "position", &current_joint_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], "velocity", &current_joint_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], "acceleration", &current_joint_accelerations_[i]));
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
  std::string elac_mapping_node;
  std::string full_variable_address;

  // Export command interfaces here
  for (size_t i = 0; i < NUM_JOINTS; i++) {

    elac_mapping_node = info_.joints[i].parameters["DL_node"];

    full_variable_address = g_ethercatWritingBaseAddress + "/" + elac_mapping_node + "/" +
      g_positionTargetValuePDO;

    command_interfaces_to_dl_commands_.emplace(
      std::make_pair(
        full_variable_address,
        DatalayerType(
          std::numeric_limits<double>::quiet_NaN(), comm::datalayer::VariantType::INT32,
          full_variable_address))
    );

    RCLCPP_INFO(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"), "creating a position CommandInterface for type <INT32> that maps from [%s, %s]",
      full_variable_address.c_str(), command_interfaces_to_dl_commands_.at(
        full_variable_address).address().c_str());

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], "position", &command_joint_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], "velocity", &command_joint_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], "acceleration", &command_joint_accelerations_[i]));
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
  // Write the command values from the command interfaces to the datalayer
  // In this case the command will be the full trajectory to execute
  // The datalayer will take care of the interpolation and execution of the trajectory
  // if (passthrough_trajectory_sent_) {
  //   // Only send the trajectory once
  //   return hardware_interface::return_type::OK;
  // }
  // if (!sendPassthroughTrajectory()) {
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("VhitRobotHardwareInterface"),
  //     "Failed to send trajectory to passthrough trajectory controller");
  //   return hardware_interface::return_type::ERROR;
  // } else {
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("VhitRobotHardwareInterface"),
  //     "Successfully sent trajectory to passthrough trajectory controller");
  //   passthrough_trajectory_sent_ = true;
  // }

  return hardware_interface::return_type::OK;
}

bool VhitRobotHardwareInterface::sendPassthroughTrajectory()
{
  // This function should take the trajectory from the command interfaces and send it to the passthrough trajectory controller via the datalayer
  // The trajectory should be sent in a format that the passthrough trajectory controller expects, which is currently not defined
  // For now, this function will just return true to indicate success
  if (client_ == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"),
      "Cannot send trajectory to passthrough trajectory controller because datalayer client is not connected");
    return false;
  }
  // Browse for the passthrough trajectory controller application
  std::string controller_address = g_plcControllerApplication + "/" +
    g_plcPassthroughControllerAddress;
  comm::datalayer::Variant read_variant;
  comm::datalayer::DlResult result = client_->browseSync(controller_address, &read_variant);
  if (STATUS_FAILED(result)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"),
      "Failed to send trajectory to passthrough trajectory controller. Failed to browse for controller application with error %s",
      result.toString());
    return false;
  }

  std::vector<std::string> apps;
  // Get the list of applications in the plc folder to find the controller application
  if (read_variant.getType() != comm::datalayer::VariantType::ARRAY_OF_STRING) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"),
      "Failed to send trajectory to passthrough trajectory controller. Failed to browse for controller application with error %s",
      result.toString());
    return false;
  }

  const char ** str = read_variant;
  for (uint32_t i = 0; i < read_variant.getCount(); i++) {
    std::cout << str[i] << std::endl;
    apps.push_back(str[i]);
  }

  if (apps.size() <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "VhitRobotHardwareInterface"),
      "Failed to send trajectory to passthrough trajectory controller. Controller application has no sub elements at address %s",
      controller_address.c_str());
  }

  // recursively browse the controller application to find the passthrough trajectory variables
  // for (const auto& app : apps) {
  //   std::string current_address = controller_address + "/" + app;
  //   comm::datalayer::Variant variant;
  //   result = client_->browseSync(passthrough_controller_address, &passthrough_variant);
  //   if (STATUS_FAILED(result)) {
  //     RCLCPP_ERROR(
  //       rclcpp::get_logger("VhitRobotHardwareInterface"),
  //       "Failed to send trajectory to passthrough trajectory controller. Failed to browse for passthrough controller variables with error %s",
  //       result.toString());
  //     return false;
  //   }
  //   if (passthrough_variant.getType() == comm::datalayer::VariantType::ARRAY_OF_STRING) {
  //     // Found the passthrough trajectory controller variables, now we can write the trajectory to the datalayer
  //     // The format of the trajectory is currently not defined, so this part is left as a TODO
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("VhitRobotHardwareInterface"),
  //       "Successfully found passthrough trajectory controller variables at address %s",
  //       passthrough_controller_address.c_str());
  //     }
  //   }
  return true;
}

} // namespace vhit_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vhit_robot_driver::VhitRobotHardwareInterface,
  hardware_interface::SystemInterface)
