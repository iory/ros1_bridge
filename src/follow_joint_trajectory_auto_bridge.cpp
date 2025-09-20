#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>
#include <regex>

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
using ROS1Server = actionlib::ActionServer<ROS1Action>;

// ROS2 types
using ROS2Action = control_msgs::action::FollowJointTrajectory;
using ROS2Goal = ROS2Action::Goal;
using ROS2Result = ROS2Action::Result;
using ROS2Feedback = ROS2Action::Feedback;
using ROS2Client = rclcpp_action::Client<ROS2Action>;

class FollowJointTrajectoryAutoBridge : public rclcpp::Node
{
public:
  FollowJointTrajectoryAutoBridge() : Node("follow_joint_trajectory_auto_bridge")
  {
    // Discover available ROS2 action servers
    auto discovered_actions = discoverFollowJointTrajectoryActions();
    
    if (discovered_actions.empty()) {
      RCLCPP_ERROR(get_logger(), "No FollowJointTrajectory actions found in ROS2!");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Found %zu FollowJointTrajectory action(s) in ROS2:", discovered_actions.size());
    for (const auto& action_name : discovered_actions) {
      RCLCPP_INFO(get_logger(), "  - %s", action_name.c_str());
    }
    
    // Initialize ROS1 NodeHandle
    ros1_nh_ = std::make_shared<ros::NodeHandle>();
    
    // Create bridges for each discovered action
    for (const auto& ros2_action_name : discovered_actions) {
      createBridge(ros2_action_name);
    }
  }
  
  ~FollowJointTrajectoryAutoBridge()
  {
    // Clean up
    ros1_servers_.clear();
    ros2_clients_.clear();
  }

private:
  struct BridgeData {
    std::shared_ptr<ROS1Server> ros1_server;
    std::shared_ptr<ROS2Client> ros2_client;
    std::string ros1_name;
    std::string ros2_name;
  };
  
  std::shared_ptr<ros::NodeHandle> ros1_nh_;
  std::vector<BridgeData> bridges_;
  std::map<std::string, std::shared_ptr<ROS1Server>> ros1_servers_;
  std::map<std::string, std::shared_ptr<ROS2Client>> ros2_clients_;
  struct GoalHandlePair {
    ROS1Server::GoalHandle ros1_handle;
    ROS2Client::GoalHandle::SharedPtr ros2_handle;
    std::string action_name;
  };
  std::map<std::string, GoalHandlePair> goal_handles_;  // key is goal ID string
  std::mutex goal_handles_mutex_;
  
  std::vector<std::string> discoverFollowJointTrajectoryActions()
  {
    std::vector<std::string> discovered_actions;
    
    // Try using the action client to discover actions
    // This is more reliable than looking for specific topics
    auto node = std::make_shared<rclcpp::Node>("action_discovery_node");
    
    // Known common action names to check
    std::vector<std::string> potential_actions = {
      "/panda_arm_controller/follow_joint_trajectory",
      "/panda_hand_controller/follow_joint_trajectory",
      "/arm_controller/follow_joint_trajectory",
      "/joint_trajectory_controller/follow_joint_trajectory",
      "panda_arm_controller/follow_joint_trajectory",
      "arm_controller/follow_joint_trajectory"
    };
    
    for (const auto& action_name : potential_actions) {
      auto test_client = rclcpp_action::create_client<ROS2Action>(node, action_name);
      
      // Give it a short time to discover the action server
      if (test_client->wait_for_action_server(std::chrono::milliseconds(500))) {
        RCLCPP_INFO(get_logger(), "Found action server: %s", action_name.c_str());
        discovered_actions.push_back(action_name);
      }
    }
    
    // Also try the original topic-based discovery
    auto topics = this->get_topic_names_and_types();
    for (const auto& [topic_name, types] : topics) {
      if (topic_name.find("follow_joint_trajectory") != std::string::npos) {
        // Extract potential action name
        std::string potential_action = topic_name;
        size_t pos = potential_action.find("/_action");
        if (pos != std::string::npos) {
          potential_action = potential_action.substr(0, pos);
          
          // Check if it's actually a FollowJointTrajectory action
          for (const auto& type : types) {
            if (type.find("control_msgs") != std::string::npos && 
                type.find("FollowJointTrajectory") != std::string::npos) {
              discovered_actions.push_back(potential_action);
              break;
            }
          }
        }
      }
    }
    
    // Remove duplicates
    std::sort(discovered_actions.begin(), discovered_actions.end());
    discovered_actions.erase(std::unique(discovered_actions.begin(), discovered_actions.end()), discovered_actions.end());
    
    return discovered_actions;
  }
  
  std::string generateROS1Name(const std::string& ros2_name)
  {
    // Extract controller name from ROS2 action name
    // e.g., "/panda_arm_controller/follow_joint_trajectory" -> "panda_arm"
    // e.g., "/panda_hand_controller/follow_joint_trajectory" -> "panda_hand"
    
    std::regex controller_regex("^/?([^/]+)_controller/follow_joint_trajectory$");
    std::smatch matches;
    
    if (std::regex_match(ros2_name, matches, controller_regex)) {
      // Create ROS1 name based on controller name
      std::string controller_name = matches[1].str();
      
      // Check if it's the main arm controller
      if (controller_name.find("arm") != std::string::npos) {
        // Main arm controller uses the default name for backward compatibility
        return "follow_joint_trajectory";
      } else {
        // Other controllers get prefixed names
        return controller_name + "/follow_joint_trajectory";
      }
    }
    
    // Default fallback
    return "follow_joint_trajectory";
  }
  
  void createBridge(const std::string& ros2_action_name)
  {
    std::string ros1_action_name = generateROS1Name(ros2_action_name);
    
    // Create ROS2 client
    auto ros2_client = rclcpp_action::create_client<ROS2Action>(this, ros2_action_name);
    
    // Create ROS1 server
    auto ros1_server = std::make_shared<ROS1Server>(
      *ros1_nh_, ros1_action_name,
      [this, ros2_action_name](ROS1Server::GoalHandle goal_handle) {
        this->handleROS1Goal(goal_handle, ros2_action_name);
      },
      [this](ROS1Server::GoalHandle goal_handle) {
        this->handleROS1Cancel(goal_handle);
      },
      false);
    ros1_server->start();
    
    // Store references
    ros1_servers_[ros1_action_name] = ros1_server;
    ros2_clients_[ros2_action_name] = ros2_client;
    
    RCLCPP_INFO(get_logger(), "Created bridge: ROS1 [%s] <-> ROS2 [%s]", 
                ros1_action_name.c_str(), ros2_action_name.c_str());
  }
  
  void handleROS1Goal(ROS1Server::GoalHandle ros1_goal_handle, const std::string& ros2_action_name)
  {
    auto ros2_client = ros2_clients_[ros2_action_name];
    
    if (!ros2_client) {
      RCLCPP_ERROR(get_logger(), "Bridge components not found for %s", ros2_action_name.c_str());
      ros1_goal_handle.setRejected(ROS1Result(), "Bridge components not found");
      return;
    }
    
    // Accept the goal
    ros1_goal_handle.setAccepted();
    
    RCLCPP_INFO(get_logger(), "Received ROS1 goal, forwarding to ROS2 [%s]", ros2_action_name.c_str());
    
    // Wait for ROS2 action server
    if (!ros2_client->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "ROS2 action server [%s] not available", ros2_action_name.c_str());
      ros1_goal_handle.setAborted(ROS1Result(), "ROS2 action server not available");
      return;
    }
    
    // Convert ROS1 goal to ROS2 goal
    auto ros2_goal = std::make_shared<ROS2Goal>();
    convertGoal(*ros1_goal_handle.getGoal(), *ros2_goal);
    
    // Send goal to ROS2 server
    auto send_goal_options = ROS2Client::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      [this, ros1_goal_handle, ros2_action_name](ROS2Client::GoalHandle::SharedPtr goal_handle) mutable {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "ROS2 goal was rejected");
          ros1_goal_handle.setAborted(ROS1Result(), "ROS2 goal was rejected");
          return;
        }
        RCLCPP_INFO(get_logger(), "ROS2 goal accepted");
        // Store the mapping between ROS1 and ROS2 goal handles
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        std::string goal_id = ros1_goal_handle.getGoalID().id;
        goal_handles_[goal_id] = {ros1_goal_handle, goal_handle, ros2_action_name};
      };
    
    send_goal_options.feedback_callback = 
      [this, ros1_goal_handle](ROS2Client::GoalHandle::SharedPtr, 
                          const std::shared_ptr<const ROS2Feedback> ros2_feedback) mutable {
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
    
    ros2_client->async_send_goal(*ros2_goal, send_goal_options);
  }
  
  void handleROS1Cancel(ROS1Server::GoalHandle ros1_goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received ROS1 cancel request, forwarding to ROS2");
    
    std::string goal_id = ros1_goal_handle.getGoalID().id;
    
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    auto it = goal_handles_.find(goal_id);
    if (it != goal_handles_.end() && it->second.ros2_handle) {
      // Find the correct ROS2 client for this goal
      auto client_it = ros2_clients_.find(it->second.action_name);
      if (client_it != ros2_clients_.end() && client_it->second) {
        // Send cancel request to ROS2 action
        auto cancel_future = client_it->second->async_cancel_goal(it->second.ros2_handle);
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
        RCLCPP_ERROR(get_logger(), "No ROS2 client found for action %s", it->second.action_name.c_str());
      }
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
  ros::init(ros1_argc, ros1_argv.data(), "follow_joint_trajectory_auto_bridge");
  
  // Initialize ROS2
  rclcpp::init(ros2_argc, ros2_argv.data());
  
  auto bridge_node = std::make_shared<FollowJointTrajectoryAutoBridge>();
  
  // Spin both ROS1 and ROS2
  ros::AsyncSpinner ros1_spinner(1);
  ros1_spinner.start();
  
  rclcpp::spin(bridge_node);
  
  ros1_spinner.stop();
  rclcpp::shutdown();
  ros::shutdown();
  
  return 0;
}