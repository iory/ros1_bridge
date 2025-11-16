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

// Manual adapter for incompatible ControllerState messages
// ROS1 (Noetic) and ROS2 (Jazzy) have different ControllerState definitions

#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/msg/controller_state.hpp>
#include "ros1_bridge/factory.hpp"

namespace ros1_bridge
{

// Stub implementations for incompatible ControllerState messages
// Only converting the common fields (name, state, type)
template<>
void
Factory<
  controller_manager_msgs::ControllerState,
  controller_manager_msgs::msg::ControllerState
>::convert_1_to_2(
  const controller_manager_msgs::ControllerState & ros1_msg,
  controller_manager_msgs::msg::ControllerState & ros2_msg)
{
  ros2_msg.name = ros1_msg.name;

  // Map ROS 1 controller states to ROS 2 states
  // ROS 1: running, stopped
  // ROS 2: unconfigured, inactive, active, finalized
  if (ros1_msg.state == "running") {
    ros2_msg.state = "active";
  } else if (ros1_msg.state == "stopped") {
    ros2_msg.state = "inactive";
  } else {
    // Pass through unknown states
    ros2_msg.state = ros1_msg.state;
  }

  ros2_msg.type = ros1_msg.type;

  // Convert claimed_resources to required_state_interfaces
  // ROS 1 claimed_resources contains joint names we need
  for (const auto & resource_set : ros1_msg.claimed_resources) {
    // Extract interface type from hardware_interface string
    // e.g., "hardware_interface::PositionJointInterface" -> "position"
    std::string hw_iface = resource_set.hardware_interface;
    std::string iface_type = "position";  // default

    if (hw_iface.find("Position") != std::string::npos) {
      iface_type = "position";
    } else if (hw_iface.find("Velocity") != std::string::npos) {
      iface_type = "velocity";
    } else if (hw_iface.find("Effort") != std::string::npos) {
      iface_type = "effort";
    }

    // Create interface strings in ROS 2 format: "joint_name/interface_type"
    for (const auto & joint_name : resource_set.resources) {
      std::string state_interface = joint_name + "/" + iface_type;
      ros2_msg.required_state_interfaces.push_back(state_interface);
      // Also add to required_command_interfaces for controllers that need it
      ros2_msg.required_command_interfaces.push_back(state_interface);
    }
  }

  // Note: ROS2 has additional fields that don't exist in ROS1
  // (is_async, update_rate, etc.)
  // These will remain at their default values
}

template<>
void
Factory<
  controller_manager_msgs::ControllerState,
  controller_manager_msgs::msg::ControllerState
>::convert_2_to_1(
  const controller_manager_msgs::msg::ControllerState & ros2_msg,
  controller_manager_msgs::ControllerState & ros1_msg)
{
  ros1_msg.name = ros2_msg.name;

  // Map ROS 2 controller states to ROS 1 states
  // ROS 2: unconfigured, inactive, active, finalized
  // ROS 1: running, stopped
  if (ros2_msg.state == "active") {
    ros1_msg.state = "running";
  } else if (ros2_msg.state == "inactive" || ros2_msg.state == "unconfigured" || ros2_msg.state == "finalized") {
    ros1_msg.state = "stopped";
  } else {
    // Pass through unknown states
    ros1_msg.state = ros2_msg.state;
  }

  ros1_msg.type = ros2_msg.type;
  // Note: ROS1 has claimed_resources field that doesn't exist in ROS2
  // It will remain empty
}

}  // namespace ros1_bridge
