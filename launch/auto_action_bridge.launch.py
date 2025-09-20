from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Automatically discovers and bridges all FollowJointTrajectory actions
    from ROS2 to ROS1.
    
    Naming convention:
    - Main arm controllers (containing 'arm' in name): 
      ROS2: /xxx_arm_controller/follow_joint_trajectory -> ROS1: /follow_joint_trajectory
    - Other controllers:
      ROS2: /xxx_controller/follow_joint_trajectory -> ROS1: /xxx/follow_joint_trajectory
    
    Examples:
    - /panda_arm_controller/follow_joint_trajectory -> /follow_joint_trajectory
    - /panda_hand_controller/follow_joint_trajectory -> /panda_hand/follow_joint_trajectory
    - /gripper_controller/follow_joint_trajectory -> /gripper/follow_joint_trajectory
    """
    
    auto_bridge_node = Node(
        package='ros1_bridge',
        executable='follow_joint_trajectory_auto_bridge',
        name='follow_joint_trajectory_auto_bridge',
        output='screen'
    )

    return LaunchDescription([auto_bridge_node])