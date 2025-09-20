#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
import sys

class PandaTrajectoryClient:
    def __init__(self):
        rospy.init_node('panda_trajectory_client')
        
        # アクションクライアントを作成
        self.client = actionlib.SimpleActionClient(
            '/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for action server...")
        server_up = self.client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            sys.exit(1)
        rospy.loginfo("Connected to action server")
        
        self.joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
        ]
    
    def create_trajectory_goal(self, positions_list, time_list):
        """
        複数の経由点を含む軌道を作成
        
        Args:
            positions_list: 各経由点での関節角度のリスト [[pos1], [pos2], ...]
            time_list: 各経由点への到達時間のリスト [time1, time2, ...]
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        # ヘッダーのタイムスタンプを設定
        goal.trajectory.header.stamp = rospy.Time.now()
        
        # 各経由点を追加
        for positions, time_from_start in zip(positions_list, time_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(self.joint_names)
            point.time_from_start = rospy.Duration(time_from_start)
            goal.trajectory.points.append(point)
        
        # 許容誤差を設定（オプション）
        for joint_name in self.joint_names:
            tolerance = JointTolerance()
            tolerance.name = joint_name
            tolerance.position = 0.01  # ラジアン
            tolerance.velocity = 0.1
            tolerance.acceleration = 0.1
            goal.goal_tolerance.append(tolerance)
        
        # ゴール時間の許容誤差
        goal.goal_time_tolerance = rospy.Duration(0.5)
        
        return goal
    
    def send_trajectory(self, goal, timeout=30.0):
        """軌道を送信して結果を待つ"""
        rospy.loginfo("Sending trajectory...")
        
        # フィードバックコールバックを設定
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        
        # 結果を待つ
        finished = self.client.wait_for_result(rospy.Duration(timeout))
        
        if not finished:
            rospy.logwarn("Action did not finish before timeout")
            self.client.cancel_goal()
            return False
        
        # 結果を確認
        state = self.client.get_state()
        result = self.client.get_result()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Trajectory execution succeeded!")
            return True
        else:
            rospy.logwarn("Trajectory execution failed: %s", result.error_string)
            return False
    
    def feedback_callback(self, feedback):
        """フィードバックを受信したときの処理"""
        # 現在の関節位置を表示（最初の3関節のみ）
        if feedback.actual.positions:
            pos_str = ", ".join([f"{p:.3f}" for p in feedback.actual.positions[:3]])
            rospy.loginfo_throttle(1.0, f"Current position: [{pos_str}, ...]")
    
    def demo_single_point(self):
        """単一の目標点への移動デモ"""
        rospy.loginfo("=== Single Point Demo ===")
        
        # 目標位置（1点のみ）
        positions = [[0.0128, -0.5844, -0.0212, -2.4207, 0.0241, 2.617, 0.7316]]
        times = [3.0]  # 3秒で到達
        
        goal = self.create_trajectory_goal(positions, times)
        self.send_trajectory(goal)
    
    def demo_multi_points(self):
        """複数の経由点を通る軌道のデモ"""
        rospy.loginfo("=== Multi Points Demo ===")
        
        # 複数の経由点
        positions = [
            [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785],  # 初期位置
            [0.5, -0.5, 0.0, -2.0, 0.0, 2.0, 0.785],  # 経由点1
            [0.0128, -0.5844, -0.0212, -2.4207, 0.0241, 2.617, 0.7316],  # 最終位置
        ]
        times = [2.0, 4.0, 6.0]  # 各点への到達時間
        
        goal = self.create_trajectory_goal(positions, times)
        self.send_trajectory(goal)
    
    def demo_smooth_motion(self):
        """滑らかな動作のデモ（正弦波軌道）"""
        rospy.loginfo("=== Smooth Motion Demo ===")
        
        import numpy as np
        
        # 正弦波軌道を生成
        num_points = 20
        duration = 5.0
        
        positions = []
        times = []
        
        for i in range(num_points):
            t = i * duration / (num_points - 1)
            # Joint 1を正弦波で動かす
            pos = [
                0.5 * np.sin(2 * np.pi * t / duration),  # joint1
                -0.5,  # joint2 (固定)
                0.0,   # joint3 (固定)
                -2.0,  # joint4 (固定)
                0.0,   # joint5 (固定)
                2.0,   # joint6 (固定)
                0.785  # joint7 (固定)
            ]
            positions.append(pos)
            times.append(t)
        
        goal = self.create_trajectory_goal(positions, times)
        self.send_trajectory(goal, timeout=duration + 5.0)


def main():
    try:
        client = PandaTrajectoryClient()
        
        # デモを選択
        if len(sys.argv) > 1:
            demo_type = sys.argv[1]
            if demo_type == "single":
                client.demo_single_point()
            elif demo_type == "multi":
                client.demo_multi_points()
            elif demo_type == "smooth":
                client.demo_smooth_motion()
            else:
                rospy.logwarn("Unknown demo type. Use: single, multi, or smooth")
                client.demo_single_point()
        else:
            # デフォルトは単一点デモ
            client.demo_single_point()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error: %s", str(e))


if __name__ == '__main__':
    main()