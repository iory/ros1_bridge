from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file to bridge ROS2 action servers to ROS1 with same action names."""
    return LaunchDescription([
        Node(
            package='ros1_bridge',
            executable='follow_joint_trajectory_bridge',
            name='head_controller_bridge',
            parameters=[{
                'ros1_node_name': 'head_controller_bridge',
                'ros1_action_name': 'head_controller/follow_joint_trajectory',
                'ros2_action_name': '/head_controller/follow_joint_trajectory',
            }],
            output='screen'
        ),
        Node(
            package='ros1_bridge',
            executable='follow_joint_trajectory_bridge',
            name='larm_controller_bridge',
            parameters=[{
                'ros1_node_name': 'larm_controller_bridge',
                'ros1_action_name': 'larm_controller/follow_joint_trajectory',
                'ros2_action_name': '/larm_controller/follow_joint_trajectory',
            }],
            output='screen'
        ),
        Node(
            package='ros1_bridge',
            executable='follow_joint_trajectory_bridge',
            name='rarm_controller_bridge',
            parameters=[{
                'ros1_node_name': 'rarm_controller_bridge',
                'ros1_action_name': 'rarm_controller/follow_joint_trajectory',
                'ros2_action_name': '/rarm_controller/follow_joint_trajectory',
            }],
            output='screen'
        ),
    ])
