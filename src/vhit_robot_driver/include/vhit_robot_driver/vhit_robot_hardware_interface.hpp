#ifndef VHIT_ROBOT_HARDWARE_INTERFACE_HPP_
#define VHIT_ROBOT_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "ctrlx_datalayer_helper.h"
#include "datalayer_type.hpp"

namespace vhit_robot_driver
{
class VhitRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  static const size_t NUM_JOINTS = 5;
  // SystemInterface
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state)
  override;
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  bool sendPassthroughTrajectory();

  std::vector<std::string> joint_names_;

  // /* Vectors used to store the trajectory received from the passthrough trajectory controller. The whole trajectory is
  // * received before it is sent to the robot. */
  std::vector<std::array<double, NUM_JOINTS>> trajectory_joint_positions_buffer_;
  std::vector<std::array<double, NUM_JOINTS>> trajectory_joint_velocities_buffer_;
  std::vector<std::array<double, NUM_JOINTS>> trajectory_joint_accelerations_buffer_;
  std::vector<double> trajectory_times_buffer_;

  std::array<double, NUM_JOINTS> command_joint_positions_;
  std::array<double, NUM_JOINTS> command_joint_velocities_;
  std::array<double, NUM_JOINTS> command_joint_accelerations_;

  std::array<double, NUM_JOINTS> current_joint_positions_;
  std::array<double, NUM_JOINTS> current_joint_velocities_;
  std::array<double, NUM_JOINTS> current_joint_accelerations_;

  // Passthrough trajectory controller interface values
  double passthrough_trajectory_transfer_state_;
  double passthrough_trajectory_abort_;
  double passthrough_trajectory_size_;
  bool passthrough_trajectory_controller_running_;

  // Connection parameters for ctrlX CORE
  std::string ip_address_;
  std::string username_;
  std::string password_;
  int ssl_port_;

  std::unique_ptr<comm::datalayer::DatalayerSystem> datalayer_;
  comm::datalayer::IClient * client_;

  // Datalayer addresses for passthrough controller
  const std::string g_plcControllerApplication = "plc/vhit_robot_controller";
  const std::string g_plcPassthroughControllerAddress = "PassthroughTrajectoryController";
  const std::string g_trajectoryNode = "Trajectory";

  // Datalayer addresses for RT comm
  // fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input/map
  const std::string g_ethercatReadingMap =
    "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input/map";
  const std::string g_ethercatWritingMap =
    "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output/map";

  const std::string g_positionActualValuePDO = "PdoTx1_MappingParameters.Position_Actual_Value";
  const std::string g_positionTargetValuePDO = "PdoRx1_MappingParameters.Target_Position";

  std::unordered_map<std::string, DatalayerType> state_interfaces_to_dl_states_;
  std::unordered_map<std::string, DatalayerType> command_interfaces_to_dl_commands_;

  // std::unordered_map<std::string, std::string> state_interfaces_to_dl_nodes_;
  // std::unordered_map<std::string, std::string> command_interfaces_to_dl_nodes_;
};

}  // namespace vhit_robot_driver
#endif  // VHIT_ROBOT_HARDWARE_INTERFACE_HPP_
