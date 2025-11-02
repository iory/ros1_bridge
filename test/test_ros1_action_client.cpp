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

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <actionlib_tutorials/FibonacciAction.h>

typedef actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> Client;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fibonacci_client");
  ros::NodeHandle nh;

  Client client("fibonacci", true);

  if (!client.waitForServer(ros::Duration(30.0))) {
    ROS_ERROR("Action server not available");
    return 1;
  }

  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 5;

  client.sendGoal(goal);

  if (!client.waitForResult(ros::Duration(30.0))) {
    ROS_ERROR("Action did not finish before timeout");
    return 1;
  }

  auto result = client.getResult();
  if (result->sequence.size() != 6) {
    ROS_ERROR("Expected 6 values, got %zu", result->sequence.size());
    return 1;
  }

  ROS_INFO("Action succeeded");
  return 0;
}
