#include <memory>
#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <master_project_msgs/action/master_action.hpp>
#include <master_project_msgs/msg/task.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>

using namespace ur_rtde;
using namespace std::chrono_literals;

class TaskActionServer : public rclcpp::Node {
public:
    using MasterAction = master_project_msgs::action::MasterAction;
    using GoalHandleMasterAction = rclcpp_action::ServerGoalHandle<MasterAction>;

    using Task = master_project_msgs::msg::Task;
    using Stage = master_project_msgs::msg::Stage;
    using Waypoint = master_project_msgs::msg::Waypoint;

    TaskActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("task_action_server", options), ip_("10.130.1.100") {
        rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
        rtde_receive_ = std::make_unique<RTDEReceiveInterface>(ip_);
        gripper_ = std::make_unique<RobotiqGripper>(ip_);

        RCLCPP_INFO(this->get_logger(), "Connecting to gripper...");
        gripper_->connect();
        RCLCPP_INFO(this->get_logger(), "Gripper connected.");
        gripper_->activate();
        RCLCPP_INFO(this->get_logger(), "Gripper activated.");

        task_details_subscriber_ = this->create_subscription<Task>(
            "task_details", 10, std::bind(&TaskActionServer::taskDetailsCallback, this, std::placeholders::_1));

        move_to_target_server_ = rclcpp_action::create_server<MasterAction>(
            this, "move_to_target",
            std::bind(&TaskActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TaskActionServer::handleCancel, this, std::placeholders::_1),
            std::bind(&TaskActionServer::handleAccepted, this, std::placeholders::_1));

        initializeMotionMap();
    }

private:
    rclcpp::Subscription<Task>::SharedPtr task_details_subscriber_;
    rclcpp_action::Server<MasterAction>::SharedPtr move_to_target_server_;

    std::string ip_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<RobotiqGripper> gripper_;
    std::unordered_map<std::string, Task> tasks_map_;
    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    bool task_received_;

    using MotionFunction = std::function<void()>;
    std::unordered_map<std::string, MotionFunction> motion_map_;

    void taskDetailsCallback(const Task::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->name.c_str());
        tasks_map_[msg->name] = *msg;
        task_received_ = true;
        task_cv_.notify_all();
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const MasterAction::Goal> goal) {
        (void)uuid;  // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "Received goal request for command: %s", goal->command_type.c_str());

        if (motion_map_.find(goal->command_type) != motion_map_.end()) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            RCLCPP_WARN(this->get_logger(), "Command %s not found", goal->command_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleMasterAction> goal_handle) {
        (void)goal_handle;  // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleMasterAction> goal_handle) {
        std::thread{std::bind(&TaskActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMasterAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MasterAction::Feedback>();
        auto result = std::make_shared<MasterAction::Result>();

        if (goal->command_type == "publish_pose") {
            if (publishPose()) {
                result->success = true;
                result->message = "Pose published and task details received.";
            } else {
                result->success = false;
                result->message = "Failed to receive task details within 10 seconds.";
            }
        } else if (motion_map_.find(goal->command_type) != motion_map_.end()) {
            motion_map_[goal->command_type]();
            result->success = true;
            result->message = "Motion command executed successfully.";
        } else {
            result->success = false;
            result->message = "Unknown command.";
        }

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal %s", result->success ? "succeeded" : "failed");
    }
    bool publishPose() {
        RCLCPP_INFO(this->get_logger(), "Waiting for task details...");

        std::unique_lock<std::mutex> lock(task_mutex_);
        if (task_cv_.wait_for(lock, 10s, [this]() { return task_received_; })) {
            RCLCPP_INFO(this->get_logger(), "Task details received.");
            task_received_ = false;  // Reset for the next command
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to receive task details within 10 seconds.");
            return false;
        }
    }

    void move() {
        if (tasks_map_.find("planned_task") != tasks_map_.end()) {
            auto task = tasks_map_["planned_task"];
            processTask(task);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Task 'planned_task' not found.");
        }
    }

    void gripperClose() {
        RCLCPP_INFO(this->get_logger(), "Closing gripper");
        gripper_->move(0.0); // Assuming 0.0 is the closed position
    }

    void gripperOpen() {
        RCLCPP_INFO(this->get_logger(), "Opening gripper");
        gripper_->move(1.0); // Assuming 1.0 is the open position
    }

    void initializeMotionMap() {
        motion_map_["move"] = std::bind(&TaskActionServer::move, this);
        motion_map_["publish_pose"] = std::bind(&TaskActionServer::publishPose, this);
        motion_map_["gripper_close"] = std::bind(&TaskActionServer::gripperClose, this);
        motion_map_["gripper_open"] = std::bind(&TaskActionServer::gripperOpen, this);
        motion_map_["check_status"] = std::bind(&TaskActionServer::checkRobotStatus, this);
    }

    void processTask(const Task &task) {
        for (const auto &stage : task.stages) {
            processStage(stage);
        }
    }

    void processStage(const Stage &stage) {
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

    bool isRobotMoving() {
        auto target_q = rtde_receive_->getTargetQ();
        auto actual_q = rtde_receive_->getActualQ();
        
        RCLCPP_INFO(this->get_logger(), "Target Q: ");
        for (const auto &q : target_q) {
            RCLCPP_INFO(this->get_logger(), "%f ", q);
        }
        RCLCPP_INFO(this->get_logger(), "Actual Q: ");
        for (const auto &q : actual_q) {
            RCLCPP_INFO(this->get_logger(), "%f ", q);
        }
        double tolerance = 1e-4; // Adjust tolerance
        for (size_t i = 0; i < target_q.size(); ++i) {
            if (std::fabs(target_q[i] - actual_q[i]) > tolerance) {
                RCLCPP_INFO(this->get_logger(), "Robot is moving");
                return true;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Robot is not moving");
        return false;
    }

    bool isGripperMoving() {
        int obj_status = gripper_->getVar("OBJ");
        RCLCPP_INFO(this->get_logger(), "Gripper OBJ status: %d", obj_status);
        return (obj_status == 0);  // 0 means the gripper is moving
    }

    void waitForRobotAndGripperToStop() {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot and gripper to stop moving...");

        while (isRobotMoving() || isGripperMoving()) {
            RCLCPP_INFO(this->get_logger(), "Robot or gripper is still moving...");
            std::this_thread::sleep_for(500ms);
        }

        RCLCPP_INFO(this->get_logger(), "Robot and gripper have stopped.");
    }

    void checkRobotStatus(){
        RCLCPP_INFO(this->get_logger(), "Checking robot status...");
        waitForRobotAndGripperToStop();
    }

    double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    double map_finger_joint_to_gripper(double finger_joint_position) {
        double close_finger_joint = 0.7; //0.69991196175
        double open_finger_joint = 0.0; //0.07397215645645
        return map_value(finger_joint_position, open_finger_joint, close_finger_joint, 1.0 , 0.0); //0.988235, 0.105882
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




/* #include <memory>
#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <master_project_msgs/action/project_action.hpp>
#include <master_project_msgs/msg/task.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/robotiq_gripper.h>

using namespace ur_rtde;
using namespace std::chrono_literals;

class TaskActionServer : public rclcpp::Node {
public:
    using ProjectAction = master_project_msgs::action::ProjectAction;
    using GoalHandleProjectAction = rclcpp_action::ServerGoalHandle<ProjectAction>;

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

        task_details_subscriber_ = this->create_subscription<Task>(
            "task_details", 10, std::bind(&TaskActionServer::taskDetailsCallback, this, std::placeholders::_1));

        move_to_target_server_ = rclcpp_action::create_server<ProjectAction>(
            this, "move_to_target",
            std::bind(&TaskActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TaskActionServer::handleCancel, this, std::placeholders::_1),
            std::bind(&TaskActionServer::handleAccepted, this, std::placeholders::_1));

        initializeMotionMap();
    }

private:
    rclcpp::Subscription<Task>::SharedPtr task_details_subscriber_;
    rclcpp_action::Server<ProjectAction>::SharedPtr move_to_target_server_;

    std::string ip_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RobotiqGripper> gripper_;
    Task current_task_;

    using MotionFunction = std::function<void()>;
    std::unordered_map<std::string, MotionFunction> motion_map_;

    void taskDetailsCallback(const Task::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received task details: %s", msg->name.c_str());
        current_task_ = *msg;
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const ProjectAction::Goal> goal) {
        (void)uuid;  // Suppress unused parameter warning
        const std::string &name = goal->task_name_or_stage_name;
        RCLCPP_INFO(this->get_logger(), "Received goal request for name: %s", name.c_str());

        if (motion_map_.find(name) != motion_map_.end()) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            RCLCPP_WARN(this->get_logger(), "Task or stage %s not found", name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleProjectAction> goal_handle) {
        (void)goal_handle;  // Suppress unused parameter warning
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleProjectAction> goal_handle) {
        std::thread{std::bind(&TaskActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleProjectAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto feedback = std::make_shared<ProjectAction::Feedback>();
        auto result = std::make_shared<ProjectAction::Result>();

        const std::string &name = goal_handle->get_goal()->task_name_or_stage_name;

        if (motion_map_.find(name) != motion_map_.end()) {
            feedback->current_stage_name = name;
            feedback->progress = 0.0;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Executing motion: %s", name.c_str());
            motion_map_[name]();

            feedback->progress = 1.0;
            goal_handle->publish_feedback(feedback);

            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Task '%s' not found.", name.c_str());
        }
    }

    void move() {
        if (!current_task_.stages.empty()) {
            for (const auto &stage : current_task_.stages) {
                processStage(stage);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "No task details available.");
        }
    }

    void gripperClose() {
        RCLCPP_INFO(this->get_logger(), "Closing gripper");
        gripper_->move(0.0); // Assuming 0.0 is the closed position
    }

    void gripperOpen() {
        RCLCPP_INFO(this->get_logger(), "Opening gripper");
        gripper_->move(1.0); // Assuming 1.0 is the open position
    }

    void initializeMotionMap() {
        motion_map_["move"] = std::bind(&TaskActionServer::move, this);
        motion_map_["gripper_close"] = std::bind(&TaskActionServer::gripperClose, this);
        motion_map_["gripper_open"] = std::bind(&TaskActionServer::gripperOpen, this);
    }

    void processStage(const Stage &stage) {
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

    double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    double map_finger_joint_to_gripper(double finger_joint_position) {
        double close_finger_joint = 0.7; //0.69991196175
        double open_finger_joint = 0.0; //0.07397215645645
        return map_value(finger_joint_position, open_finger_joint, close_finger_joint, 1.0 , 0.0); //0.988235, 0.105882
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 */