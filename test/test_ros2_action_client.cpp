// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fibonacci_client");

  auto action_client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

  if (!action_client->wait_for_action_server(std::chrono::seconds(30))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 5;

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

  bool result_received = false;
  bool success = false;

  send_goal_options.result_callback =
    [&result_received, &success, node](const GoalHandleFibonacci::WrappedResult & result) {
      result_received = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (result.result->sequence.size() == 6) {
          success = true;
          RCLCPP_INFO(node->get_logger(), "Action succeeded");
        } else {
          RCLCPP_ERROR(
            node->get_logger(), "Expected 6 values, got %zu",
            result.result->sequence.size());
        }
      } else {
        RCLCPP_ERROR(node->get_logger(), "Action failed");
      }
    };

  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
    rclcpp::shutdown();
    return 1;
  }

  // Wait for result
  auto start = std::chrono::steady_clock::now();
  while (!result_received && rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(30)) {
      RCLCPP_ERROR(node->get_logger(), "Action did not finish before timeout");
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return success ? 0 : 1;
}
