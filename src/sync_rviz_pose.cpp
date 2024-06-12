#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include <vector>
#include <thread>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/robotiq_gripper.h>

using namespace ur_rtde;
using namespace std::chrono_literals;

class URRTDENode : public rclcpp::Node {
public:
    URRTDENode() : Node("ur_rtde_node"), ip_("10.130.1.100"), success_(false) {
        this->declare_parameter<std::string>("robot_ip", ip_);
        this->get_parameter("robot_ip", ip_);

        try {
            rtde_receive_ = std::make_unique<RTDEReceiveInterface>(ip_);
            rtde_control_ = std::make_unique<RTDEControlInterface>(ip_);
            gripper_ = std::make_unique<RobotiqGripper>(ip_);

            gripper_->connect();
            gripper_->activate();
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Initialization failed: %s", e.what());
            rclcpp::shutdown();
        }
        
        // Defer the call to update_robot_position to ensure the object is fully constructed
        timer_ = this->create_wall_timer(
            100ms, [this]() {
                this->update_robot_position();
            });
    }
    
    ~URRTDENode() {
        if (gripper_ && gripper_->isActive()) {
            gripper_->disconnect();
        }
    }
    
private:
    void update_robot_position() {
        std::vector<double> current_q;
        try {
            current_q = rtde_receive_->getActualQ();
            RCLCPP_INFO(this->get_logger(), "Current Joint Positions:");
            for (const auto& position : current_q) {
                RCLCPP_INFO(this->get_logger(), "%f", position);
            }

            if (current_q.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get actual joint positions");
                return;
            }

            // Plan and visualize the trajectory for the robot joints
            plan_and_visualize_trajectory(current_q);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get actual joint positions: %s", e.what());
            return;
        }

        // Once the robot joints are updated, update the gripper position
        update_gripper_position();
    }

    void update_gripper_position() {
        double gripper_position;
        try {
            gripper_position = get_gripper_position();
            RCLCPP_INFO(this->get_logger(), "UR5e Gripper Position if 1 mean open: %f", gripper_position);

            // Plan and visualize the trajectory for the gripper
            plan_and_visualize_gripper_trajectory(gripper_position);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get gripper position: %s", e.what());
            return;
        }

        // Check if the process completed successfully
        if (success_) {
            rclcpp::shutdown();
        }
    }

    double get_gripper_position() {
        auto gripper_status = gripper_->getCurrentPosition();
        double position = gripper_status; // Normalize position between 0 and 1
        RCLCPP_INFO(this->get_logger(), "Real Gripper Position: %f", position);
        return position;
    }
    double map_value(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    double map_gripper_to_finger_joint(double gripper_position) {
        double close_finger_joint = 0.7; //0.69991196175
        double open_finger_joint = 0.0; //0.07397215645645
        double mapped_position = map_value(gripper_position, 1, 0, open_finger_joint, close_finger_joint); // 0.988235, 0.105882
        RCLCPP_INFO(this->get_logger(), "Mapped Finger Joint Position: %f", mapped_position);
        return mapped_position;
    }

    void plan_and_visualize_trajectory(const std::vector<double>& joint_positions) {
        // Load robot model
        auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        auto kinematic_model = robot_model_loader->getModel();
        auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
        kinematic_state->setToDefaultValues();

        // Set current state for the arm
        const moveit::core::JointModelGroup* arm_joint_model_group = kinematic_model->getJointModelGroup("ur5e_arm");
        kinematic_state->setJointGroupPositions(arm_joint_model_group, joint_positions);


        // Create MoveGroupInterface for the arm
        moveit::planning_interface::MoveGroupInterface arm_move_group_interface(shared_from_this(), "ur5e_arm");

        // Set the current state to the one just calculated
        arm_move_group_interface.setJointValueTarget(joint_positions);

        // Configure the planner and execution parameters
        arm_move_group_interface.setPlanningTime(10.0);
        arm_move_group_interface.setMaxVelocityScalingFactor(1.0);
        arm_move_group_interface.setMaxAccelerationScalingFactor(1.0);

        // Plan a trajectory for the arm
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_success = (arm_move_group_interface.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (arm_success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing the plan.");
            arm_move_group_interface.execute(arm_plan);
            success_ = true;  // Set the success flag if planning and execution succeed
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            success_ = false;  // Reset the success flag if planning fails
        }
    }

    void plan_and_visualize_gripper_trajectory(double gripper_position) {
        double finger_joint_position = map_gripper_to_finger_joint(gripper_position);
        // Load robot model
        auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        auto kinematic_model = robot_model_loader->getModel();
        auto kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
        kinematic_state->setToDefaultValues();

        // Set current state for the gripper
        const moveit::core::JointModelGroup* gripper_joint_model_group = kinematic_model->getJointModelGroup("gripper");
        std::vector<double> gripper_joint_positions(gripper_joint_model_group->getVariableCount(), finger_joint_position);
        RCLCPP_INFO(this->get_logger(), "RViz Gripper Position: %f", gripper_joint_positions[0]);
        kinematic_state->setJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);

        // Validate initial state
        if (!kinematic_state->satisfiesBounds()) {
            RCLCPP_ERROR(this->get_logger(), "Initial state is out of bounds");
            return;
        }

        // Create MoveGroupInterface for the gripper
        moveit::planning_interface::MoveGroupInterface gripper_move_group_interface(shared_from_this(), "gripper");

        // Set the current state to the one just calculated
        gripper_move_group_interface.setJointValueTarget(gripper_joint_positions);

        // Plan a trajectory for the gripper
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool gripper_success = (gripper_move_group_interface.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (gripper_success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing the plan.");
            gripper_move_group_interface.execute(gripper_plan);
            success_ = true;  // Set the success flag if planning and execution succeed
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            success_ = false;  // Reset the success flag if planning fails
        }
    }

    std::unique_ptr<RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<RTDEControlInterface> rtde_control_;
    std::unique_ptr<RobotiqGripper> gripper_;
    std::string ip_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool success_;

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<URRTDENode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}