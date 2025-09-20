#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from rosgraph_msgs.msg import Clock

def main():
    # ROS1ノードを初期化
    rospy.init_node('panda_trajectory_sender')
    
    # アクションクライアントを作成（ブリッジのROS1サーバーに接続）
    client = actionlib.SimpleActionClient('/follow_joint_trajectory', 
                                          FollowJointTrajectoryAction)
    
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server found!")
    
    # ゴールメッセージを作成
    goal = FollowJointTrajectoryGoal()
    
    # 1. 関節名の設定（ROS2側と同じ順番で）
    goal.trajectory.joint_names = [
        'panda_joint1',
        'panda_joint2',
        'panda_joint3',
        'panda_joint4',
        'panda_joint5',
        'panda_joint6',
        'panda_joint7',
    ]
    
    # 2. 軌道点の作成
    point = JointTrajectoryPoint()
    
    # 目標の関節角度を設定（ROS2の例と同じ値）
    point.positions = [0.0128, -0.5844, -0.0212, -2.4207, 0.0241, 2.617, 0.7316]
    
    # オプション：速度、加速度、努力も設定可能
    # point.velocities = [0.0] * 7
    # point.accelerations = [0.0] * 7
    # point.effort = [0.0] * 7
    
    # この目標姿勢に到達するまでの時間を設定（2秒）
    point.time_from_start = rospy.Duration(2.0)
    
    # 軌道にポイントを追加
    goal.trajectory.points.append(point)
    
    # オプション：複数の経由点を追加することも可能
    # point2 = JointTrajectoryPoint()
    # point2.positions = [0.1, -0.6, -0.1, -2.4, 0.1, 2.6, 0.8]
    # point2.time_from_start = rospy.Duration(4.0)
    # goal.trajectory.points.append(point2)
    
    # ゴールを送信
    rospy.loginfo("Sending trajectory goal...")
    client.send_goal(goal)
    
    # 結果を待つ（タイムアウト30秒）
    client.wait_for_result(rospy.Duration(30.0))
    
    # 結果を取得
    result = client.get_result()
    if result:
        rospy.loginfo("Result: %s", result)
    else:
        rospy.logwarn("No result received")
    
    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Trajectory execution succeeded!")
    else:
        rospy.logwarn("Trajectory execution failed with state: %d", state)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
