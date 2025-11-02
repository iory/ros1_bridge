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
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Fibonacci::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  std::thread{[goal_handle]() {
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();

    feedback->sequence.push_back(0);
    feedback->sequence.push_back(1);

    for (int i = 1; i < goal_handle->get_goal()->order; ++i) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }
      feedback->sequence.push_back(
        feedback->sequence[i] + feedback->sequence[i - 1]);
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    result->sequence = feedback->sequence;
    goal_handle->succeed(result);
  }}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fibonacci_server");

  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, "fibonacci",
    handle_goal,
    handle_cancel,
    handle_accepted);

  rclcpp::sleep_for(std::chrono::seconds(5));  // Wait for bridge and client
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
