#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <master_project_msgs/msg/task.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <std_msgs/msg/string.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
#include <cmath> // For M_PI

static const rclcpp::Logger LOGGER = rclcpp::get_logger("execution_task");

namespace mtc = moveit::task_constructor;

class MTCUtils
{
public:
    MTCUtils(const std::shared_ptr<rclcpp::Node> &node)
        : node_(node)
    {
        task_details_publisher_ = node_->create_publisher<master_project_msgs::msg::Task>("/task_details", 10);
    }

    master_project_msgs::msg::Task moveToTarget(const geometry_msgs::msg::PoseStamped &current_target_pose)
    {
        task_.reset();
        task_.stages()->clear();
        task_.stages()->setName("planned_task");
        task_.loadRobotModel(node_);

        const auto &arm_group_name = "ur5e_arm";
        const auto &hand_group_name = "gripper";
        const auto &hand_frame = "flange";

        task_.setProperty("group", arm_group_name);
        task_.setProperty("eef", hand_group_name);
        task_.setProperty("ik_frame", hand_frame);

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(0.005);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task_.add(std::move(stage_state_current));

        auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
        stage_close_hand->setGroup(hand_group_name);
        stage_close_hand->setGoal("close");
        task_.add(std::move(stage_close_hand));

        double distanceToSubtract = 0.1;
        geometry_msgs::msg::PoseStamped target_pose = subtractDistance(current_target_pose, distanceToSubtract);

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = hand_frame;
        ocm.header.frame_id = "world";
        ocm.orientation = target_pose.pose.orientation;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = 0.1;
        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.orientation_constraints.push_back(ocm);

        auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
        move_to_target->setGroup(arm_group_name);
        move_to_target->setGoal(target_pose);
        move_to_target->setPathConstraints(path_constraints);
        task_.add(std::move(move_to_target));

        try
        {
            task_.init();
        }
        catch (mtc::InitStageException &e)
        {
            RCLCPP_ERROR(node_->get_logger(), e.what());
            throw std::runtime_error("Task initialization failed");
        }

        if (!task_.plan(100))
        {
            RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
            throw std::runtime_error("Task planning failed");
        }

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
            throw std::runtime_error("Task execution failed");
        }

        RCLCPP_INFO(node_->get_logger(), "Task execution succeeded");
        publishTaskDetails(task_);
        return createTaskMessage(task_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    mtc::Task task_;
    rclcpp::Publisher<master_project_msgs::msg::Task>::SharedPtr task_details_publisher_;

    geometry_msgs::msg::PoseStamped subtractDistance(const geometry_msgs::msg::PoseStamped &original, double distance)
    {
        tf2::Quaternion q(
            original.pose.orientation.x,
            original.pose.orientation.y,
            original.pose.orientation.z,
            original.pose.orientation.w);

        tf2::Vector3 direction(1, 0, 0);
        tf2::Vector3 transformedDirection = tf2::quatRotate(q, direction);

        transformedDirection.normalize();
        transformedDirection *= distance;

        geometry_msgs::msg::PoseStamped newPose = original;
        newPose.pose.position.x -= transformedDirection.x();
        newPose.pose.position.y -= transformedDirection.y();
        newPose.pose.position.z -= transformedDirection.z();

        return newPose;
    }

    void publishTaskDetails(const moveit::task_constructor::Task& task)
    {
        master_project_msgs::msg::Task task_msg = createTaskMessage(task);
        RCLCPP_INFO(node_->get_logger(), "Publishing TaskDetails message: %s", task_msg.name.c_str());
        task_details_publisher_->publish(task_msg);
    }

    master_project_msgs::msg::Task createTaskMessage(const moveit::task_constructor::Task& task)
    {
        master_project_msgs::msg::Task task_msg;
        task_msg.name = task.name();

        const auto* container = task.stages();
        if (container)
        {
            container->traverseChildren([&](const moveit::task_constructor::Stage& stage, unsigned int /*depth*/) -> bool {
                master_project_msgs::msg::Stage stage_msg;
                stage_msg.name = stage.name();

                for (const auto& solution : stage.solutions())
                {
                    const auto* sub_trajectory = dynamic_cast<const moveit::task_constructor::SubTrajectory*>(solution.get());
                    if (sub_trajectory && sub_trajectory->trajectory())
                    {
                        const auto& trajectory = *sub_trajectory->trajectory();
                        for (size_t j = 0; j < trajectory.getWayPointCount(); ++j)
                        {
                            master_project_msgs::msg::Waypoint waypoint_msg;
                            const auto& waypoint = trajectory.getWayPoint(j);

                            for (const auto& joint : waypoint.getVariableNames())
                            {
                                master_project_msgs::msg::JointState joint_state_msg;
                                joint_state_msg.name = joint;
                                joint_state_msg.position = waypoint.getVariablePosition(joint);
                                waypoint_msg.joints.push_back(joint_state_msg);
                            }

                            stage_msg.waypoints.push_back(waypoint_msg);
                        }
                    }
                }

                task_msg.stages.push_back(stage_msg);
                return true; // Continue traversal
            });
        }

        task_msg.number_of_stages = task_msg.stages.size();
        return task_msg;
    }
};
