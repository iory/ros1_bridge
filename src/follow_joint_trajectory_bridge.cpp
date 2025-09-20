#include <memory>
#include <string>
#include <chrono>

// ROS1 includes
#ifdef __clang__
#pragma clang diagnostic push
#endif
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using namespace std::chrono_literals;

// ROS1 types
using ROS1Action = control_msgs::FollowJointTrajectoryAction;
using ROS1Goal = control_msgs::FollowJointTrajectoryGoal;
using ROS1Result = control_msgs::FollowJointTrajectoryResult;
using ROS1Feedback = control_msgs::FollowJointTrajectoryFeedback;
using ROS1Client = actionlib::SimpleActionClient<ROS1Action>;
using ROS1Server = actionlib::ActionServer<ROS1Action>;

// ROS2 types
using ROS2Action = control_msgs::action::FollowJointTrajectory;
using ROS2Goal = ROS2Action::Goal;
using ROS2Result = ROS2Action::Result;
using ROS2Feedback = ROS2Action::Feedback;
using ROS2Client = rclcpp_action::Client<ROS2Action>;
using ROS2Server = rclcpp_action::Server<ROS2Action>;

class FollowJointTrajectoryBridge : public rclcpp::Node
{
public:
  FollowJointTrajectoryBridge() : Node("follow_joint_trajectory_bridge")
  {
    // Get parameters for action names
    this->declare_parameter("ros1_action_name", "follow_joint_trajectory");
    this->declare_parameter("ros2_action_name", "panda_arm_controller/follow_joint_trajectory");
    
    std::string ros1_action_name = this->get_parameter("ros1_action_name").as_string();
    std::string ros2_action_name = this->get_parameter("ros2_action_name").as_string();
    
    // Initialize ROS1 NodeHandle
    ros1_nh_ = std::make_shared<ros::NodeHandle>();
    
    // ROS1 action server (receives commands from ROS1 clients)
    ros1_server_ = std::make_unique<ROS1Server>(
      *ros1_nh_, ros1_action_name,
      boost::bind(&FollowJointTrajectoryBridge::handleROS1Goal, this, _1),
      boost::bind(&FollowJointTrajectoryBridge::handleROS1Cancel, this, _1),
      false);
    ros1_server_->start();
    
    // ROS2 action client (sends commands to ROS2 servers)
    ros2_client_ = rclcpp_action::create_client<ROS2Action>(this, ros2_action_name);
    
    RCLCPP_INFO(get_logger(), "FollowJointTrajectory bridge initialized");
    RCLCPP_INFO(get_logger(), "ROS1 server: /%s", ros1_action_name.c_str());
    RCLCPP_INFO(get_logger(), "ROS2 client: /%s", ros2_action_name.c_str());
  }

private:
  std::shared_ptr<ros::NodeHandle> ros1_nh_;
  std::unique_ptr<ROS1Server> ros1_server_;
  ROS2Client::SharedPtr ros2_client_;
  struct GoalHandlePair {
    ROS1Server::GoalHandle ros1_handle;
    ROS2Client::GoalHandle::SharedPtr ros2_handle;
  };
  std::map<std::string, GoalHandlePair> goal_handles_;  // key is goal ID string
  std::mutex goal_handles_mutex_;

  void handleROS1Goal(ROS1Server::GoalHandle ros1_goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received ROS1 goal, forwarding to ROS2");
    
    // Accept the goal
    ros1_goal_handle.setAccepted();
    
    // Wait for ROS2 action server
    if (!ros2_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "ROS2 action server not available");
      ros1_goal_handle.setAborted(ROS1Result(), "ROS2 action server not available");
      return;
    }
    
    // Convert ROS1 goal to ROS2 goal
    auto ros2_goal = std::make_shared<ROS2Goal>();
    convertGoal(*ros1_goal_handle.getGoal(), *ros2_goal);
    
    // Send goal to ROS2 server
    auto send_goal_options = ROS2Client::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      [this, ros1_goal_handle](ROS2Client::GoalHandle::SharedPtr goal_handle) mutable {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "ROS2 goal was rejected");
          ros1_goal_handle.setAborted(ROS1Result(), "ROS2 goal was rejected");
          return;
        }
        RCLCPP_INFO(get_logger(), "ROS2 goal accepted");
        // Store the mapping between ROS1 and ROS2 goal handles
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        std::string goal_id = ros1_goal_handle.getGoalID().id;
        goal_handles_[goal_id] = {ros1_goal_handle, goal_handle};
      };
    
    send_goal_options.feedback_callback = 
      [this, ros1_goal_handle](ROS2Client::GoalHandle::SharedPtr, const std::shared_ptr<const ROS2Feedback> ros2_feedback) mutable {
        // Convert ROS2 feedback to ROS1 feedback
        ROS1Feedback ros1_feedback;
        convertFeedback(*ros2_feedback, ros1_feedback);
        ros1_goal_handle.publishFeedback(ros1_feedback);
      };
    
    send_goal_options.result_callback = 
      [this, ros1_goal_handle](const ROS2Client::GoalHandle::WrappedResult & result) mutable {
        // Convert ROS2 result to ROS1 result
        ROS1Result ros1_result;
        convertResult(*result.result, ros1_result);
        
        // Remove the mapping
        {
          std::lock_guard<std::mutex> lock(goal_handles_mutex_);
          std::string goal_id = ros1_goal_handle.getGoalID().id;
          goal_handles_.erase(goal_id);
        }
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          ros1_goal_handle.setSucceeded(ros1_result, "Action succeeded");
          RCLCPP_INFO(get_logger(), "Action succeeded");
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          ros1_goal_handle.setCanceled(ros1_result, "Action was canceled");
          RCLCPP_INFO(get_logger(), "Action was canceled");
        } else {
          ros1_goal_handle.setAborted(ros1_result, "Action aborted");
          RCLCPP_ERROR(get_logger(), "Action aborted");
        }
      };
    
    ros2_client_->async_send_goal(*ros2_goal, send_goal_options);
  }
  
  void handleROS1Cancel(ROS1Server::GoalHandle ros1_goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received ROS1 cancel request, forwarding to ROS2");
    
    std::string goal_id = ros1_goal_handle.getGoalID().id;
    
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    auto it = goal_handles_.find(goal_id);
    if (it != goal_handles_.end() && it->second.ros2_handle) {
      // Send cancel request to ROS2 action
      auto cancel_future = ros2_client_->async_cancel_goal(it->second.ros2_handle);
      // Handle the cancel asynchronously
      std::thread([this, cancel_future, ros1_goal_handle]() mutable {
        auto cancel_response = cancel_future.get();
        if (cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
          RCLCPP_INFO(get_logger(), "Successfully sent cancel request to ROS2");
        } else {
          RCLCPP_WARN(get_logger(), "Failed to cancel ROS2 goal");
        }
      }).detach();
    } else {
      RCLCPP_WARN(get_logger(), "No corresponding ROS2 goal found for cancel request");
      ros1_goal_handle.setCanceled(ROS1Result(), "No corresponding ROS2 goal found");
    }
  }
  
  void convertGoal(const ROS1Goal& ros1_goal, ROS2Goal& ros2_goal)
  {
    // Convert trajectory
    ros2_goal.trajectory.header.stamp.sec = ros1_goal.trajectory.header.stamp.sec;
    ros2_goal.trajectory.header.stamp.nanosec = ros1_goal.trajectory.header.stamp.nsec;
    ros2_goal.trajectory.header.frame_id = ros1_goal.trajectory.header.frame_id;
    ros2_goal.trajectory.joint_names = ros1_goal.trajectory.joint_names;
    
    for (const auto& ros1_point : ros1_goal.trajectory.points) {
      trajectory_msgs::msg::JointTrajectoryPoint ros2_point;
      ros2_point.positions = ros1_point.positions;
      ros2_point.velocities = ros1_point.velocities;
      ros2_point.accelerations = ros1_point.accelerations;
      ros2_point.effort = ros1_point.effort;
      ros2_point.time_from_start.sec = ros1_point.time_from_start.sec;
      ros2_point.time_from_start.nanosec = ros1_point.time_from_start.nsec;
      ros2_goal.trajectory.points.push_back(ros2_point);
    }
    
    // Convert path tolerance
    for (const auto& ros1_tol : ros1_goal.path_tolerance) {
      control_msgs::msg::JointTolerance ros2_tol;
      ros2_tol.name = ros1_tol.name;
      ros2_tol.position = ros1_tol.position;
      ros2_tol.velocity = ros1_tol.velocity;
      ros2_tol.acceleration = ros1_tol.acceleration;
      ros2_goal.path_tolerance.push_back(ros2_tol);
    }
    
    // Convert goal tolerance
    for (const auto& ros1_tol : ros1_goal.goal_tolerance) {
      control_msgs::msg::JointTolerance ros2_tol;
      ros2_tol.name = ros1_tol.name;
      ros2_tol.position = ros1_tol.position;
      ros2_tol.velocity = ros1_tol.velocity;
      ros2_tol.acceleration = ros1_tol.acceleration;
      ros2_goal.goal_tolerance.push_back(ros2_tol);
    }
    
    // Convert goal time tolerance
    ros2_goal.goal_time_tolerance.sec = ros1_goal.goal_time_tolerance.sec;
    ros2_goal.goal_time_tolerance.nanosec = ros1_goal.goal_time_tolerance.nsec;
  }
  
  void convertFeedback(const ROS2Feedback& ros2_feedback, ROS1Feedback& ros1_feedback)
  {
    ros1_feedback.header.stamp.sec = ros2_feedback.header.stamp.sec;
    ros1_feedback.header.stamp.nsec = ros2_feedback.header.stamp.nanosec;
    ros1_feedback.header.frame_id = ros2_feedback.header.frame_id;
    ros1_feedback.joint_names = ros2_feedback.joint_names;
    
    ros1_feedback.desired.positions = ros2_feedback.desired.positions;
    ros1_feedback.desired.velocities = ros2_feedback.desired.velocities;
    ros1_feedback.desired.accelerations = ros2_feedback.desired.accelerations;
    ros1_feedback.desired.effort = ros2_feedback.desired.effort;
    ros1_feedback.desired.time_from_start.sec = ros2_feedback.desired.time_from_start.sec;
    ros1_feedback.desired.time_from_start.nsec = ros2_feedback.desired.time_from_start.nanosec;
    
    ros1_feedback.actual.positions = ros2_feedback.actual.positions;
    ros1_feedback.actual.velocities = ros2_feedback.actual.velocities;
    ros1_feedback.actual.accelerations = ros2_feedback.actual.accelerations;
    ros1_feedback.actual.effort = ros2_feedback.actual.effort;
    ros1_feedback.actual.time_from_start.sec = ros2_feedback.actual.time_from_start.sec;
    ros1_feedback.actual.time_from_start.nsec = ros2_feedback.actual.time_from_start.nanosec;
    
    ros1_feedback.error.positions = ros2_feedback.error.positions;
    ros1_feedback.error.velocities = ros2_feedback.error.velocities;
    ros1_feedback.error.accelerations = ros2_feedback.error.accelerations;
    ros1_feedback.error.effort = ros2_feedback.error.effort;
    ros1_feedback.error.time_from_start.sec = ros2_feedback.error.time_from_start.sec;
    ros1_feedback.error.time_from_start.nsec = ros2_feedback.error.time_from_start.nanosec;
  }
  
  void convertResult(const ROS2Result& ros2_result, ROS1Result& ros1_result)
  {
    ros1_result.error_code = ros2_result.error_code;
    ros1_result.error_string = ros2_result.error_string;
  }
};

int main(int argc, char** argv)
{
  // Separate arguments for ROS1 and ROS2
  std::vector<std::string> args;
  std::vector<std::string> ros2_args;
  bool is_ros2_arg = false;
  
  for (int i = 0; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--ros-args") {
      is_ros2_arg = true;
      ros2_args.push_back(argv[i]);
    } else if (is_ros2_arg) {
      ros2_args.push_back(argv[i]);
    } else {
      args.push_back(argv[i]);
    }
  }
  
  // Convert back to char* for ROS1
  std::vector<char*> ros1_argv;
  for (auto& arg : args) {
    ros1_argv.push_back(const_cast<char*>(arg.c_str()));
  }
  int ros1_argc = ros1_argv.size();
  
  // Convert back to char* for ROS2
  std::vector<char*> ros2_argv;
  if (ros2_args.empty()) {
    ros2_argv.push_back(const_cast<char*>(args[0].c_str()));
  } else {
    ros2_argv.push_back(const_cast<char*>(args[0].c_str()));
    for (auto& arg : ros2_args) {
      ros2_argv.push_back(const_cast<char*>(arg.c_str()));
    }
  }
  int ros2_argc = ros2_argv.size();
  
  // Initialize ROS1
  ros::init(ros1_argc, ros1_argv.data(), "follow_joint_trajectory_bridge");
  
  // Initialize ROS2
  rclcpp::init(ros2_argc, ros2_argv.data());
  
  auto bridge_node = std::make_shared<FollowJointTrajectoryBridge>();
  
  // Spin both ROS1 and ROS2
  ros::AsyncSpinner ros1_spinner(1);
  ros1_spinner.start();
  
  rclcpp::spin(bridge_node);
  
  ros1_spinner.stop();
  rclcpp::shutdown();
  ros::shutdown();
  
  return 0;
}