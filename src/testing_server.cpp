#include <memory>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <master_project_msgs/action/master_action.hpp>

using namespace std::chrono_literals;

using MasterAction = master_project_msgs::action::MasterAction;
using GoalHandleMasterAction = rclcpp_action::ClientGoalHandle<MasterAction>;

class MotionCommander : public rclcpp::Node {
public:
    MotionCommander() : Node("motion_commander") {
        this->client_ = rclcpp_action::create_client<MasterAction>(this, "move_to_target");
        this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

        // Start the sequence
        execute_sequence();
    }

private:
    rclcpp_action::Client<MasterAction>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    void execute_sequence() {
        //publish this create_pose(0.4, 0.4, 0.4, 0.0, 1.0, 0.0, 0.0)
        //then send_motion_command(publish_pose)
        if(send_motion_command("move")){
            if(send_motion_command("check_status")){
                if(send_motion_command("gripper_open")){

                }
            }
        }
        if (send_motion_command("move") &&
            send_motion_command("check_status") && 
            send_motion_command("gripper_open")) {

            if (send_motion_command("move") &&
                send_motion_command("check_status") && 
                send_motion_command("gripper_close")) {
                RCLCPP_INFO(this->get_logger(), "Sequence completed.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Sequence failed.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Sequence failed.");
        }

        rclcpp::shutdown();
    }

    geometry_msgs::msg::PoseStamped create_pose(double x, double y, double z, double ox, double oy, double oz, double ow) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = z;
        pose_msg.pose.orientation.x = ox;
        pose_msg.pose.orientation.y = oy;
        pose_msg.pose.orientation.z = oz;
        pose_msg.pose.orientation.w = ow;
        return pose_msg;
    }

    bool send_motion_command(const std::string &command, const std::optional<geometry_msgs::msg::PoseStamped> &pose = std::nullopt) {
        auto goal_msg = MasterAction::Goal();
        goal_msg.command_type = command;
        if (pose) {
            goal_msg.target_pose = *pose;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        if (!this->client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal: %s", command.c_str());
        auto send_goal_options = rclcpp_action::Client<MasterAction>::SendGoalOptions();

        auto future_goal_handle = this->client_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return false;
        }

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        auto future_result = this->client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Get result call failed");
            return false;
        }

        auto result = future_result.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Result: success = %d, message: %s", result.result->success, result.result->message.c_str());
        return result.result->success;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionCommander>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
