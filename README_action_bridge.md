# FollowJointTrajectory Action Bridge

This package provides bridges for `FollowJointTrajectory` actions between ROS1 and ROS2.

## Features

### 1. Automatic Discovery Bridge (`follow_joint_trajectory_auto_bridge`)
Automatically discovers all `FollowJointTrajectory` action servers in ROS2 and creates corresponding ROS1 servers.

**Usage:**
```bash
# Simple launch
ros2 launch ros1_bridge auto_action_bridge.launch.py

# Or run directly
ros2 run ros1_bridge follow_joint_trajectory_auto_bridge
```

**Naming Convention:**
- Main arm controllers (containing 'arm'): 
  - ROS2: `/xxx_arm_controller/follow_joint_trajectory` → ROS1: `/follow_joint_trajectory`
- Other controllers:
  - ROS2: `/xxx_controller/follow_joint_trajectory` → ROS1: `/xxx/follow_joint_trajectory`

**Examples:**
- `/panda_arm_controller/follow_joint_trajectory` → `/follow_joint_trajectory`
- `/panda_hand_controller/follow_joint_trajectory` → `/panda_hand/follow_joint_trajectory`

### 2. Manual Bridge (`follow_joint_trajectory_bridge`)
Manually specify the ROS1 and ROS2 action names.

**Usage:**
```bash
# Using launch file
ros2 launch ros1_bridge panda_action_bridge.launch.py

# Or with parameters
ros2 run ros1_bridge follow_joint_trajectory_bridge \
  --ros-args \
  -p ros1_action_name:=follow_joint_trajectory \
  -p ros2_action_name:=panda_arm_controller/follow_joint_trajectory
```

## Example Workflow

1. Start your ROS2 robot simulation (providing action servers):
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch your_robot_sim robot.launch.py
   ```

2. Start the auto bridge:
   ```bash
   source /opt/ros/one/setup.bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch ros1_bridge auto_action_bridge.launch.py
   ```

3. Use from ROS1 (no changes needed):
   ```bash
   source /opt/ros/one/setup.bash
   # Your existing ROS1 code works as-is
   ```

## Benefits

- **Zero Configuration**: Auto-discovery finds all available controllers
- **Backward Compatible**: Main arm controller keeps using `/follow_joint_trajectory`
- **Multi-Controller Support**: Automatically handles multiple controllers
- **No Code Changes**: Existing ROS1 code continues to work