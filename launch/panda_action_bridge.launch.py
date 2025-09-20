from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ros1_action_name = LaunchConfiguration('ros1_action_name', default='follow_joint_trajectory')
    ros2_action_name = LaunchConfiguration('ros2_action_name', default='panda_arm_controller/follow_joint_trajectory')
    
    declare_ros1_action_name_cmd = DeclareLaunchArgument(
        'ros1_action_name',
        default_value='follow_joint_trajectory',
        description='Name of the ROS1 action server')
        
    declare_ros2_action_name_cmd = DeclareLaunchArgument(
        'ros2_action_name',
        default_value='panda_arm_controller/follow_joint_trajectory',
        description='Name of the ROS2 action client')

    bridge_node = Node(
        package='ros1_bridge',
        executable='follow_joint_trajectory_bridge',
        name='follow_joint_trajectory_bridge',
        parameters=[{
            'ros1_action_name': ros1_action_name,
            'ros2_action_name': ros2_action_name,
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_ros1_action_name_cmd,
        declare_ros2_action_name_cmd,
        bridge_node
    ])