#!/bin/bash

echo "Starting Follow Joint Trajectory Bridge for Panda Robot"
echo "======================================================="
echo "ROS1 side: /follow_joint_trajectory"
echo "ROS2 side: /panda_arm_controller/follow_joint_trajectory"
echo ""

# Source both ROS environments
source /opt/ros/one/setup.bash
source /opt/ros/humble/setup.bash
source /home/iory/ros2/bridge/install/setup.bash

# Launch the bridge with panda configuration
ros2 launch ros1_bridge panda_action_bridge.launch.py