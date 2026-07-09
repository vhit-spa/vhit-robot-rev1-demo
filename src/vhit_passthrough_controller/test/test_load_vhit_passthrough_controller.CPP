// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Daniel Azanov, Dr. Denis
//

#include <gmock/gmock.h>

#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadVhitPassthroughController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  const auto test_node = std::make_shared<rclcpp::Node>("test_resource_manager_node");

  {
    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf,
      test_node->get_node_clock_interface(),
      test_node->get_node_logging_interface(),
      true);

    controller_manager::ControllerManager cm(
      std::move(resource_manager), executor, "test_controller_manager");

    const auto controller = cm.load_controller(
      "test_vhit_passthrough_controller",
      "vhit_passthrough_controller/VhitPassthroughController");

    EXPECT_NE(controller, nullptr);
  }

  rclcpp::shutdown();
}
