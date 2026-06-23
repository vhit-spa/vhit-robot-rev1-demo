// Copyright (c) 2021 PickNik, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Denis Stogl

#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.1);  // 0.1 seconds for easier math
const auto COMPARE_DELTA = 0.0001;
}  // namespace

class TestGenericSystem : public ::testing::Test
{
public:
  void test_generic_system_with_mimic_joint(const std::string & urdf);
  void test_generic_system_with_mock_sensor_commands(const std::string & urdf);
  void test_generic_system_with_mock_gpio_commands(const std::string & urdf);

protected:
  void SetUp() override
  {
    // REMOVE THIS MEMBER ONCE FAKE COMPONENTS ARE REMOVED
    hardware_fake_system_ =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>vhit_robot_driver/VhitRobotHardwareInterface</plugin>
      <param name="ip_address">192.168.1.1</param>
      <param name="ssl_port">443</param>
      <param name="username">boschrexroth</param>
      <param name="password">B0schrexroth</param>
    </hardware>
    <joint name="joint1">
      <param name="DL_node">ELAC_node_LAN9253</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
    </joint>
    <joint name="joint2">
      <param name="DL_node">ELAC_node_LAN9253_1</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
    <joint name="joint3">
      <param name="DL_node">ELAC_node_LAN9253</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
    <joint name="joint4">
      <param name="DL_node">ELAC_node_LAN9253_1</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
    <joint name="joint5">
      <param name="DL_node">ELAC_node_LAN9253</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
  </ros2_control>
)";

    hardware_system_ =
      R"(
  <ros2_control name="GenericSystem" type="system">
    <hardware>
      <plugin>vhit_robot_driver/VhitRobotHardwareInterface</plugin>
      <param name="ip_address">192.168.1.1</param>
      <param name="ssl_port">443</param>
      <param name="username">boschrexroth</param>
      <param name="password">B0schrexroth</param>
    </hardware>
    <joint name="joint1">
      <param name="DL_node">ELAC_node_LAN9253</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
    </joint>
    <joint name="joint2">
      <param name="DL_node">ELAC_node_LAN9253_1</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
    <joint name="joint3">
      <param name="DL_node">ELAC_node_LAN9253</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
    <joint name="joint4">
      <param name="DL_node">ELAC_node_LAN9253_1</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
    <joint name="joint5">
      <param name="DL_node">ELAC_node_LAN9253</param>
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
  </ros2_control>
)";
  }

  std::string hardware_system_;
  std::string hardware_fake_system_;
};

// Forward declaration
namespace hardware_interface
{
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend TestGenericSystem;

  TestableResourceManager()
  : hardware_interface::ResourceManager() {}

  TestableResourceManager(
    const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
  : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  {
  }
};

void set_components_state(
  TestableResourceManager & rm, const std::vector<std::string> & components, const uint8_t state_id,
  const std::string & state_name)
{
  for (const auto & component : components) {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components = [](
  TestableResourceManager & rm,
  const std::vector<std::string> & components = {"GenericSystem"})
  {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

auto activate_components = [](
  TestableResourceManager & rm,
  const std::vector<std::string> & components = {"GenericSystem"})
  {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  };

auto deactivate_components = [](
  TestableResourceManager & rm,
  const std::vector<std::string> & components = {"GenericSystem"})
  {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

TEST_F(TestGenericSystem, load_generic_system_)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_ +
    ros2_control_test_assets::urdf_tail;
  try {
    TestableResourceManager rm(urdf);
  } catch (const std::exception & e) {
    FAIL() << "Exception: " << e.what();
  }
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}

TEST_F(TestGenericSystem, activate_generic_system_)
{
  auto urdf = ros2_control_test_assets::urdf_head +
    hardware_system_ +
    ros2_control_test_assets::urdf_tail;

  TestableResourceManager rm(urdf);

  ASSERT_NO_THROW(configure_components(rm));

  ASSERT_NO_THROW(activate_components(rm));
}
