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

// Manual adapter for incompatible SwitchController service messages
// ROS 1 and ROS 2 have different field names and timeout types

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include "ros1_bridge/factory.hpp"

namespace ros1_bridge
{

// Service Request: ROS 1 to ROS 2
template<>
void
ServiceFactory<
  controller_manager_msgs::SwitchController,
  controller_manager_msgs::srv::SwitchController
>::translate_1_to_2(
  const controller_manager_msgs::SwitchController::Request & ros1_req,
  controller_manager_msgs::srv::SwitchController::Request & ros2_req)
{
  // Map field names: start_controllers -> activate_controllers
  ros2_req.activate_controllers = ros1_req.start_controllers;

  // Map field names: stop_controllers -> deactivate_controllers
  ros2_req.deactivate_controllers = ros1_req.stop_controllers;

  // Direct copy: strictness
  ros2_req.strictness = ros1_req.strictness;

  // Map field names: start_asap -> activate_asap
  ros2_req.activate_asap = ros1_req.start_asap;

  // Convert timeout: float64 -> builtin_interfaces::Duration
  // ROS 1: timeout is in seconds (float64)
  // ROS 2: timeout is {sec: int32, nanosec: uint32}
  ros2_req.timeout.sec = static_cast<int32_t>(ros1_req.timeout);
  ros2_req.timeout.nanosec = static_cast<uint32_t>(
    (ros1_req.timeout - static_cast<double>(ros2_req.timeout.sec)) * 1e9);
}

// Service Request: ROS 2 to ROS 1
template<>
void
ServiceFactory<
  controller_manager_msgs::SwitchController,
  controller_manager_msgs::srv::SwitchController
>::translate_2_to_1(
  const controller_manager_msgs::srv::SwitchController::Request & ros2_req,
  controller_manager_msgs::SwitchController::Request & ros1_req)
{
  // Map field names: activate_controllers -> start_controllers
  ros1_req.start_controllers = ros2_req.activate_controllers;

  // Map field names: deactivate_controllers -> stop_controllers
  ros1_req.stop_controllers = ros2_req.deactivate_controllers;

  // Direct copy: strictness
  ros1_req.strictness = ros2_req.strictness;

  // Map field names: activate_asap -> start_asap
  ros1_req.start_asap = ros2_req.activate_asap;

  // Convert timeout: builtin_interfaces::Duration -> float64
  // ROS 2: timeout is {sec: int32, nanosec: uint32}
  // ROS 1: timeout is in seconds (float64)
  ros1_req.timeout = static_cast<double>(ros2_req.timeout.sec) +
                     static_cast<double>(ros2_req.timeout.nanosec) / 1e9;
}

// Service Response: ROS 1 to ROS 2
template<>
void
ServiceFactory<
  controller_manager_msgs::SwitchController,
  controller_manager_msgs::srv::SwitchController
>::translate_1_to_2(
  const controller_manager_msgs::SwitchController::Response & ros1_res,
  controller_manager_msgs::srv::SwitchController::Response & ros2_res)
{
  // Direct copy: ok
  ros2_res.ok = ros1_res.ok;

  // ROS 1 does not have a 'message' field, so set it to empty string
  ros2_res.message = "";
}

// Service Response: ROS 2 to ROS 1
template<>
void
ServiceFactory<
  controller_manager_msgs::SwitchController,
  controller_manager_msgs::srv::SwitchController
>::translate_2_to_1(
  const controller_manager_msgs::srv::SwitchController::Response & ros2_res,
  controller_manager_msgs::SwitchController::Response & ros1_res)
{
  // Direct copy: ok
  ros1_res.ok = ros2_res.ok;

  // Note: ROS 1 does not have a 'message' field, so ros2_res.message is ignored
}

}  // namespace ros1_bridge
