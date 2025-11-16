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

// Manual adapter for incompatible JointTrajectoryControllerState messages
// ROS 1 and ROS 2 have different field names (actual/desired vs feedback/reference)

#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include "ros1_bridge/factory.hpp"

namespace ros1_bridge
{

// Convert JointTrajectoryControllerState from ROS 1 to ROS 2
template<>
void
Factory<
  control_msgs::JointTrajectoryControllerState,
  control_msgs::msg::JointTrajectoryControllerState
>::convert_1_to_2(
  const control_msgs::JointTrajectoryControllerState & ros1_msg,
  control_msgs::msg::JointTrajectoryControllerState & ros2_msg)
{
  // Convert header
  ros2_msg.header.stamp.sec = ros1_msg.header.stamp.sec;
  ros2_msg.header.stamp.nanosec = ros1_msg.header.stamp.nsec;
  ros2_msg.header.frame_id = ros1_msg.header.frame_id;

  // Copy joint names
  ros2_msg.joint_names = ros1_msg.joint_names;

  // Map field names: desired -> reference
  ros2_msg.reference.positions = ros1_msg.desired.positions;
  ros2_msg.reference.velocities = ros1_msg.desired.velocities;
  ros2_msg.reference.accelerations = ros1_msg.desired.accelerations;
  ros2_msg.reference.effort = ros1_msg.desired.effort;
  ros2_msg.reference.time_from_start.sec =
    static_cast<int32_t>(ros1_msg.desired.time_from_start.sec);
  ros2_msg.reference.time_from_start.nanosec =
    ros1_msg.desired.time_from_start.nsec;

  // Map field names: actual -> feedback
  ros2_msg.feedback.positions = ros1_msg.actual.positions;
  ros2_msg.feedback.velocities = ros1_msg.actual.velocities;
  ros2_msg.feedback.accelerations = ros1_msg.actual.accelerations;
  ros2_msg.feedback.effort = ros1_msg.actual.effort;
  ros2_msg.feedback.time_from_start.sec =
    static_cast<int32_t>(ros1_msg.actual.time_from_start.sec);
  ros2_msg.feedback.time_from_start.nanosec =
    ros1_msg.actual.time_from_start.nsec;

  // Copy error (field name is the same)
  ros2_msg.error.positions = ros1_msg.error.positions;
  ros2_msg.error.velocities = ros1_msg.error.velocities;
  ros2_msg.error.accelerations = ros1_msg.error.accelerations;
  ros2_msg.error.effort = ros1_msg.error.effort;
  ros2_msg.error.time_from_start.sec =
    static_cast<int32_t>(ros1_msg.error.time_from_start.sec);
  ros2_msg.error.time_from_start.nanosec =
    ros1_msg.error.time_from_start.nsec;

  // Note: ROS 2 has additional fields that don't exist in ROS 1:
  // - output (JointTrajectoryPoint)
  // - multi_dof_* fields
  // - speed_scaling_factor
  // These will remain at their default values
}

// Convert JointTrajectoryControllerState from ROS 2 to ROS 1
template<>
void
Factory<
  control_msgs::JointTrajectoryControllerState,
  control_msgs::msg::JointTrajectoryControllerState
>::convert_2_to_1(
  const control_msgs::msg::JointTrajectoryControllerState & ros2_msg,
  control_msgs::JointTrajectoryControllerState & ros1_msg)
{
  // Convert header
  ros1_msg.header.seq = 0;  // ROS 2 doesn't have seq
  ros1_msg.header.stamp.sec = ros2_msg.header.stamp.sec;
  ros1_msg.header.stamp.nsec = ros2_msg.header.stamp.nanosec;
  ros1_msg.header.frame_id = ros2_msg.header.frame_id;

  // Copy joint names
  ros1_msg.joint_names = ros2_msg.joint_names;

  // Map field names: reference -> desired
  ros1_msg.desired.positions = ros2_msg.reference.positions;
  ros1_msg.desired.velocities = ros2_msg.reference.velocities;
  ros1_msg.desired.accelerations = ros2_msg.reference.accelerations;
  ros1_msg.desired.effort = ros2_msg.reference.effort;
  ros1_msg.desired.time_from_start.sec = ros2_msg.reference.time_from_start.sec;
  ros1_msg.desired.time_from_start.nsec =
    static_cast<uint32_t>(ros2_msg.reference.time_from_start.nanosec);

  // Map field names: feedback -> actual
  ros1_msg.actual.positions = ros2_msg.feedback.positions;
  ros1_msg.actual.velocities = ros2_msg.feedback.velocities;
  ros1_msg.actual.accelerations = ros2_msg.feedback.accelerations;
  ros1_msg.actual.effort = ros2_msg.feedback.effort;
  ros1_msg.actual.time_from_start.sec = ros2_msg.feedback.time_from_start.sec;
  ros1_msg.actual.time_from_start.nsec =
    static_cast<uint32_t>(ros2_msg.feedback.time_from_start.nanosec);

  // Copy error (field name is the same)
  ros1_msg.error.positions = ros2_msg.error.positions;
  ros1_msg.error.velocities = ros2_msg.error.velocities;
  ros1_msg.error.accelerations = ros2_msg.error.accelerations;
  ros1_msg.error.effort = ros2_msg.error.effort;
  ros1_msg.error.time_from_start.sec = ros2_msg.error.time_from_start.sec;
  ros1_msg.error.time_from_start.nsec =
    static_cast<uint32_t>(ros2_msg.error.time_from_start.nanosec);

  // Note: ROS 2 fields output, multi_dof_*, and speed_scaling_factor
  // are not converted as they don't exist in ROS 1
}

}  // namespace ros1_bridge
