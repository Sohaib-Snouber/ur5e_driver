#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <master_project_msgs/msg/task.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
#include <cmath> // For M_PI
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("execution_task");
namespace mtc = moveit::task_constructor;

class TaskExecutionNode : public rclcpp::Node
{
public:
  TaskExecutionNode(const rclcpp::NodeOptions& options);
  void setup();

private:
  void taskPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void createAndPublishTask();
  void publishTaskDetails(const mtc::Task& task);
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;
  rclcpp::Publisher<master_project_msgs::msg::Task>::SharedPtr task_details_publisher_;
  geometry_msgs::msg::PoseStamped current_target_pose_;
  mtc::Task task_;
};

TaskExecutionNode::TaskExecutionNode(const rclcpp::NodeOptions& options) 
  : Node("task_execution_node", options)
{
  // Initialize the subscriber
  target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task_pose", 10, std::bind(&TaskExecutionNode::taskPoseCallback, this, std::placeholders::_1));
  
  // Initialize the publisher
  task_details_publisher_ = this->create_publisher<master_project_msgs::msg::Task>("/task_details", 10);
  RCLCPP_INFO(this->get_logger(), "TaskExecutionNode has been created.");

}
void TaskExecutionNode::setup()
{
  // Create a RobotModelLoader
  robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
  RCLCPP_INFO(this->get_logger(), "Robot model loaded.");

  // Create a PlanningSceneInterface to ensure it can access the planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Optionally wait for the planning scene to be initialized
  rclcpp::sleep_for(std::chrono::seconds(1));
}

void TaskExecutionNode::taskPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_target_pose_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received target pose.");
  createAndPublishTask();
}
// Function to subtract distance in the direction of the orientation
geometry_msgs::msg::PoseStamped subtractDistance(const geometry_msgs::msg::PoseStamped& original, double distance) {
    tf2::Quaternion q(
        original.pose.orientation.x,
        original.pose.orientation.y,
        original.pose.orientation.z,
        original.pose.orientation.w
    );

    // Create a vector in the direction of the orientation
    tf2::Vector3 direction(1, 0, 0); // Initial direction along the x-axis
    tf2::Vector3 transformedDirection = tf2::quatRotate(q, direction);

    // Normalize the direction vector to get only the direction
    transformedDirection.normalize();

    // Subtract distance along the direction
    transformedDirection *= distance;

    // Calculate the new position
    geometry_msgs::msg::PoseStamped newPose = original;
    newPose.pose.position.x -= transformedDirection.x();
    newPose.pose.position.y -= transformedDirection.y();
    newPose.pose.position.z -= transformedDirection.z();

    // Debug output for transformed direction
    std::cout << "Transformed Direction: (" << transformedDirection.x() << ", " << transformedDirection.y() << ", " << transformedDirection.z() << ")" << std::endl;

    return newPose;
}

void TaskExecutionNode::createAndPublishTask()
{
  task_.reset();
  task_.stages()->clear();

  //task_ = mtc::Task();
  task_.stages()->setName("planned_task");
  task_.loadRobotModel(shared_from_this());

  const auto& arm_group_name = "ur5e_arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "flange";

  // Set task properties
  task_.setProperty("group", arm_group_name);
  task_.setProperty("eef", hand_group_name);
  task_.setProperty("ik_frame", hand_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.005);

  // Current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task_.add(std::move(stage_state_current));

    // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task_.add(std::move(stage_close_hand));
  
  // Subtract 10 cm (0.1 meters)
  double distanceToSubtract = 0.1; // 10 cm
  geometry_msgs::msg::PoseStamped message_pose = current_target_pose_;
  geometry_msgs::msg::PoseStamped target_pose = subtractDistance(message_pose, distanceToSubtract);

  // Define orientation constraint
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = hand_frame;
  ocm.header.frame_id = "world";
  ocm.orientation = target_pose.pose.orientation; // Set the desired orientation
  ocm.absolute_x_axis_tolerance = 0.1; // Allowable deviation in x-axis (roll)
  ocm.absolute_y_axis_tolerance = 0.1; // Allowable deviation in y-axis (pitch)
  ocm.absolute_z_axis_tolerance = 0.1; // Allowable deviation in z-axis (yaw)
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(ocm);

/*   // not_full_open hand
  auto stage_not_full_open_hand = std::make_unique<mtc::stages::MoveTo>("not full open hand", sampling_planner);
  stage_not_full_open_hand->setGroup(hand_group_name);
  stage_not_full_open_hand->setGoal("not_full_open");
  task_.add(std::move(stage_not_full_open_hand)); */

  // Move to target pose
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move to target", sampling_planner);
  move_to_target->setGroup(arm_group_name);
  move_to_target->setGoal(target_pose);
  task_.add(std::move(move_to_target));

/*   // Define the slide pose
  geometry_msgs::msg::PoseStamped slide_pose = current_target_pose_;

  // Move to slide pose
  auto move_to_slide = std::make_unique<mtc::stages::MoveTo>("move to slide", cartesian_planner);
  move_to_slide->setGroup(arm_group_name);
  move_to_slide->setGoal(slide_pose);
  task_.add(std::move(move_to_slide));



  // Close hand
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task_.add(std::move(stage_close_hand));


  geometry_msgs::msg::PoseStamped slide2_pose = target_pose;

  // Move to slide pose
  auto move_to_slide2 = std::make_unique<mtc::stages::MoveTo>("move to slide2", cartesian_planner);
  move_to_slide2->setGroup(arm_group_name);
  move_to_slide2->setGoal(slide2_pose);
  task_.add(std::move(move_to_slide2));


  // Define the slide pose
  auto const position_pose = [message_pose]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";

	  msg.pose.orientation = message_pose.pose.orientation;
	  msg.pose.position.x = 0.40;
	  msg.pose.position.y = 0.0;
	  msg.pose.position.z = 0.30;
	  return msg;
	}();

  // Move to target pose
  auto move_to_position = std::make_unique<mtc::stages::MoveTo>("move to position", sampling_planner);
  move_to_position->setGroup(arm_group_name);
  move_to_position->setGoal(position_pose);
  task_.add(std::move(move_to_position));

  // open2 hand
  auto stage_open2_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open2_hand->setGroup(hand_group_name);
  stage_open2_hand->setGoal("not_full_open");
  task_.add(std::move(stage_open2_hand)); */

  // Initialize the task and publish task details
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR(this->get_logger(), e.what());
    return;
  }

  // Plan the task
  if (!task_.plan(100))
  {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }
  // Publish the planned task
  task_.introspection().publishSolution(*task_.solutions().front());
  publishTaskDetails(task_);

  // Execute the task
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Task execution failed");
    return;
  }
}
void TaskExecutionNode::publishTaskDetails(const moveit::task_constructor::Task& task)
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

  RCLCPP_INFO(this->get_logger(), "Publishing TaskDetails message: %s", task_msg.name.c_str());
  task_details_publisher_->publish(task_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<TaskExecutionNode>(options);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  });  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}