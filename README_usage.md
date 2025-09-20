# ROS1からROS2へのFollowJointTrajectory使用方法

## セットアップ

### 1. ROS2シミュレーター起動（別ターミナル）
```bash
source /opt/ros/humble/setup.bash
# ROS2でロボットシミュレーターを起動
# /panda_arm_controller/follow_joint_trajectory サーバーが利用可能になる
```

### 2. ブリッジ起動（別ターミナル）
```bash
source /opt/ros/one/setup.bash
source /opt/ros/humble/setup.bash
source /home/iory/ros2/bridge/install/setup.bash
ros2 run ros1_bridge follow_joint_trajectory_bridge
```

### 3. ROS1から軌道を送信

## 基本的な使用例

### シンプルな例
```bash
source /opt/ros/one/setup.bash
python /home/iory/ros2/bridge/src/ros1_bridge/scripts/ros1_send_trajectory.py
```

### 高度な例
```bash
source /opt/ros/one/setup.bash

# 単一点への移動
python /home/iory/ros2/bridge/src/ros1_bridge/scripts/ros1_send_trajectory_advanced.py single

# 複数経由点を通る軌道
python /home/iory/ros2/bridge/src/ros1_bridge/scripts/ros1_send_trajectory_advanced.py multi

# 滑らかな正弦波軌道
python /home/iory/ros2/bridge/src/ros1_bridge/scripts/ros1_send_trajectory_advanced.py smooth
```

## Pythonコード例

### 最小限のコード
```python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('send_trajectory')
client = actionlib.SimpleActionClient('/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 
                                'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

point = JointTrajectoryPoint()
point.positions = [0.0128, -0.5844, -0.0212, -2.4207, 0.0241, 2.617, 0.7316]
point.time_from_start = rospy.Duration(2.0)

goal.trajectory.points.append(point)

client.send_goal(goal)
client.wait_for_result()
```

## C++コード例

```cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_client");
  
  Client client("/follow_joint_trajectory", true);
  client.waitForServer();
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3",
                                  "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = {0.0128, -0.5844, -0.0212, -2.4207, 0.0241, 2.617, 0.7316};
  point.time_from_start = ros::Duration(2.0);
  
  goal.trajectory.points.push_back(point);
  
  client.sendGoal(goal);
  client.waitForResult();
  
  return 0;
}
```

## MoveItからの使用

MoveItを使用している場合、controller設定で`/follow_joint_trajectory`を指定すれば、
自動的にブリッジ経由でROS2のロボットを制御できます。

```yaml
# moveit_config/config/controllers.yaml
controller_list:
  - name: panda_arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - panda_joint1
      - panda_joint2
      - panda_joint3
      - panda_joint4
      - panda_joint5
      - panda_joint6
      - panda_joint7
```

## トラブルシューティング

### "Action server not available"エラー
- ブリッジが起動しているか確認
- ROS2側のロボットシミュレーターが起動しているか確認

### 軌道が実行されない
- 関節名が正しいか確認
- 目標位置が関節の可動範囲内か確認
- `rostopic echo /follow_joint_trajectory/status` でステータスを確認

### タイムアウトエラー
- 軌道の実行時間を長めに設定
- ネットワーク遅延を考慮してタイムアウト時間を調整