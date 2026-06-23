#ifndef VHIT_ROBOT_HARDWARE_INTERFACE_HPP_
#define VHIT_ROBOT_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "ctrlx_datalayer_helper.h"
#include "shared_memory_helper.hpp"

namespace vhit_robot_driver
{
class VhitRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
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
  std::vector<std::string> joint_names_;
  std::uint8_t num_joints_;

  std::vector<double> command_joint_positions_;
  std::vector<double> current_joint_positions_;

  // Connection parameters for ctrlX CORE
  std::string ip_address_;
  std::string username_;
  std::string password_;
  int ssl_port_;

  std::unique_ptr<comm::datalayer::DatalayerSystem> datalayer_;
  comm::datalayer::IClient * client_;
  std::unique_ptr<SharedMemoryArea> readMemoryArea_;
  std::unique_ptr<SharedMemoryArea> writeMemoryArea_;

  // Datalayer addresses for RT comm
  const std::string g_ethercatReadingArea_ =
    "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input";
  const std::string g_ethercatWritingArea_ =
    "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output";

  const std::string g_positionActualValuePDO_ = "PdoTx1_MappingParameters.Position_Actual_Value";
  const std::string g_positionTargetValuePDO_ = "PdoRx1_MappingParameters.Target_Position";

  std::unordered_map<std::string, std::string> state_interfaces_to_dl_states_;
  std::unordered_map<std::string, std::string> command_interfaces_to_dl_commands_;
};

}  // namespace vhit_robot_driver
#endif  // VHIT_ROBOT_HARDWARE_INTERFACE_HPP_
