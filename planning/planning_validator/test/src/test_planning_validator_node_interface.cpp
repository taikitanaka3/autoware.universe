// Copyright 2023 TIER IV, Inc.
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"
#include "planning_validator/planning_validator.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto planning_validator_dir =
    ament_index_cpp::get_package_share_directory("planning_validator");
  node_options.arguments(
    {"--ros-args", "--params-file",
     planning_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     planning_validator_dir + "/config/planning_validator.param.yaml"});
  auto test_target_node = std::make_shared<planning_validator::PlanningValidator>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "planning_validator/input/kinematics");

  // set subscriber with topic name: planning_validator → test_node_
  test_manager->setTrajectorySubscriber("planning_validator/output/trajectory");

  // set planning_validator's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("planning_validator/input/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(test_target_node));

  rclcpp::shutdown();
}
