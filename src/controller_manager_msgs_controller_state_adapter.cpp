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
  ros2_msg.state = ros1_msg.state;
  ros2_msg.type = ros1_msg.type;
  // Note: ROS2 has additional fields that don't exist in ROS1
  // (is_async, update_rate, claimed_interfaces, etc.)
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
  ros1_msg.state = ros2_msg.state;
  ros1_msg.type = ros2_msg.type;
  // Note: ROS1 has claimed_resources field that doesn't exist in ROS2
  // It will remain empty
}

}  // namespace ros1_bridge
