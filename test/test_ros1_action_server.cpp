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

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <actionlib_tutorials/FibonacciAction.h>

typedef actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> Server;

void execute(const actionlib_tutorials::FibonacciGoalConstPtr & goal, Server * server)
{
  actionlib_tutorials::FibonacciFeedback feedback;
  actionlib_tutorials::FibonacciResult result;

  feedback.sequence.push_back(0);
  feedback.sequence.push_back(1);

  for (int i = 1; i < goal->order; ++i) {
    if (server->isPreemptRequested()) {
      server->setPreempted();
      return;
    }
    feedback.sequence.push_back(
      feedback.sequence[i] + feedback.sequence[i - 1]);
    server->publishFeedback(feedback);
    ros::Duration(0.1).sleep();
  }

  result.sequence = feedback.sequence;
  server->setSucceeded(result);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fibonacci_server");
  ros::NodeHandle nh;

  Server server(nh, "fibonacci", boost::bind(&execute, _1, &server), false);
  server.start();

  ros::Duration(5.0).sleep();  // Wait for bridge and client
  ros::spin();

  return 0;
}
