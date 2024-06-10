#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "master_project_msgs/action/move_to_target.hpp"
#include "master_project_msgs/msg/task.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/robotiq_gripper.h>

using namespace ur_rtde;
using namespace std::chrono_literals;

class TaskActionServer : public rclcpp::Node {
public:
  using MoveToTarget = master_project_msgs::action::MoveToTarget;
  using GoalHandleMoveToTarget = rclcpp_action::ServerGoalHandle<MoveToTarget>;

  using Task = master_project_msgs::msg::Task;
  using Stage = master_project_msgs::msg::Stage;
  using Waypoint = master_project_msgs::msg::Waypoint;

  TaskActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("task_action_server", options), ip_("10.130.1.100") {
    rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
    gripper_ = std::make_unique<RobotiqGripper>(ip_);

    RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
    gripper_->connect();
    RCLCPP_INFO(this->get_logger(), "Gripper connected.");
    gripper_->activate();
    RCLCPP_INFO(this->get_logger(), "Gripper activated.");

    task_details_subscriber_ = this->create_subscription<master_project_msgs::msg::Task>(
        "task_details", 10, std::bind(&TaskActionServer::taskDetailsCallback, this, std::placeholders::_1));

    move_to_target_server_ = rclcpp_action::create_server<MoveToTarget>(
        this, "move_to_target",
        std::bind(&TaskActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&TaskActionServer::handleAccepted, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<master_project_msgs::msg::Task>::SharedPtr task_details_subscriber_;
  rclcpp_action::Server<MoveToTarget>::SharedPtr move_to_target_server_;

  std::string ip_;
  std::unique_ptr<RTDEControlInterface> rtde_control_;
  std::unique_ptr<RobotiqGripper> gripper_;
  std::unordered_map<std::string, Stage> stages_map_;

  void taskDetailsCallback(const master_project_msgs::msg::Task::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->name.c_str());
    for (const auto & stage : msg->stages) {
      stages_map_[stage.name] = stage;
    }
  }

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid, 
      std::shared_ptr<const MoveToTarget::Goal> goal) {
    (void)uuid;  // Suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received goal request for task: %s", goal->task_name.c_str());
    for (const auto &stage : goal->task.stages) {
      if (stages_map_.find(stage.name) == stages_map_.end()) {
        RCLCPP_WARN(this->get_logger(), "Stage %s not found", stage.name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleMoveToTarget> goal_handle) {
    (void)goal_handle;  // Suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleMoveToTarget> goal_handle) {
    std::thread{std::bind(&TaskActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveToTarget> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToTarget::Feedback>();
    auto result = std::make_shared<MoveToTarget::Result>();

    for (const auto &stage : goal->task.stages) {
      feedback->current_stage_name = stage.name;
      feedback->progress = 0.0;
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(), "Processing stage: %s", stage.name.c_str());

      // Process the stage
      processStage(stage);

      feedback->progress = 1.0;
      goal_handle->publish_feedback(feedback);
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

  void processStage(const Stage& stage) {
    RCLCPP_INFO(this->get_logger(), "Executing stage: %s", stage.name.c_str());

    if (stage.waypoints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Stage %s has no waypoints. Skipping...", stage.name.c_str());
      return;
    }

    std::vector<std::vector<double>> robot_path;
    double initial_gripper_position = -1;
    double final_gripper_position = -1;
    double finger_change = 0.1;

    for (size_t i = 0; i < stage.waypoints.size(); ++i) {
      const auto &waypoint = stage.waypoints[i];
      std::vector<double> joint_positions(6, 0.0);

      size_t joint_index = 0;
      for (const auto &joint : waypoint.joints) {
        if (joint_index < 6) {
          joint_positions[joint_index] = joint.position;
        } else if (joint_index == 6) {
          if (i == 0) {
            initial_gripper_position = map_finger_joint_to_gripper(joint.position);
          }
          if (i == stage.waypoints.size() - 1) {
            final_gripper_position = map_finger_joint_to_gripper(joint.position);
          }
        }
        joint_index++;
      }

      joint_positions.push_back(1.0);  // velocity
      joint_positions.push_back(1.2);  // acceleration
      joint_positions.push_back(0.005);  // blend (5mm tolerance)
      robot_path.push_back(joint_positions);
    }

    if (abs(abs(initial_gripper_position) - abs(final_gripper_position)) > finger_change) {
      RCLCPP_INFO(this->get_logger(), "Moving gripper to position: %f", final_gripper_position);
      gripper_->move(std::clamp(final_gripper_position, 0.0, 1.0));
      RCLCPP_INFO(this->get_logger(), "Gripper moved to position: %f", final_gripper_position);
    } else {
      rtde_control_->moveJ(robot_path, true);
    }
  }

  double map_finger_joint_to_gripper(double position) {
    // Implement your own mapping here
    return position;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
