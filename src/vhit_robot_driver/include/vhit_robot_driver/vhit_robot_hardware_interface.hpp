#ifndef VHIT_ROBOT_HARDWARE_INTERFACE_HPP_
#define VHIT_ROBOT_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "ctrlx_datalayer_helper.h"
#include "shared_memory_helper.hpp"

namespace vhit_robot_driver
{

struct JointDatalayerMapping
{
  std::string joint_name;
  std::string ethercat_node;
  double gear_ratio;
  double feed_constant;
  std::string actual_position_variable; // ELAC.../PdoTx...
  std::string target_position_variable; // ELAC.../PdoRx...
  /// Radians to actuator units conversion
  double rad_to_units(double rad) {return (rad * gear_ratio * feed_constant) / (2.0 * M_PI);}
  /// Actuator units to radians conversion
  double units_to_rad(double units) {return (units * 2.0 * M_PI) / (gear_ratio * feed_constant);}
};

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
  std::size_t num_joints_;

  std::vector<double> command_joint_positions_;
  std::vector<double> current_joint_positions_;

  // Connection parameters for ctrlX CORE
  std::string ip_address_;
  std::string username_;
  std::string password_;
  std::string connection_string_;
  int ssl_port_;

  std::unique_ptr<comm::datalayer::DatalayerSystem> datalayer_;
  comm::datalayer::IClient * client_ = nullptr;
  std::unique_ptr<SharedMemoryArea> readMemoryArea_;
  std::unique_ptr<SharedMemoryArea> writeMemoryArea_;

  // Datalayer addresses for RT comm
  const std::string g_ethercatReadingArea_ =
    "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input";
  const std::string g_ethercatWritingArea_ =
    "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output";

  const std::string g_positionActualValuePDO_ = "PdoTx1_MappingParameters.Position_Actual_Value";
  const std::string g_positionTargetValuePDO_ = "PdoRx1_MappingParameters.Target_Position";

  std::vector<JointDatalayerMapping> joint_dl_mappings_;

  // Realtime stuff
  std::vector<std::uint32_t> readVariableByteIndexes_;
  std::vector<std::uint32_t> writeVariableByteIndexes_;
  std::vector<std::int32_t> writeVariableValues_;

  uint8_t blocking_reads_count_ = 0;
  uint8_t blocking_writes_count_ = 0;
  // Error codes
  int32_t errorCode_ = 0;
};

}  // namespace vhit_robot_driver
#endif  // VHIT_ROBOT_HARDWARE_INTERFACE_HPP_
