#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlanningSceneNode : public rclcpp::Node
{
public:
  PlanningSceneNode();

private:
  void setupPlanningScene();
  moveit_msgs::msg::CollisionObject createCollisionObject(
    const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose);
  moveit_msgs::msg::ObjectColor createObjectColor(
    const std::string& id, float r, float g, float b, float a);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;

};

PlanningSceneNode::PlanningSceneNode() : Node("planning_scene_node")
{
  planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  setupPlanningScene();
}

void PlanningSceneNode::setupPlanningScene()
{
  // Create collision objects
  shape_msgs::msg::SolidPrimitive primitive_wall1;
  primitive_wall1.type = primitive_wall1.BOX;
  primitive_wall1.dimensions = {0.1, 1.4, 0.5};
  geometry_msgs::msg::Pose wall1_pose;
  wall1_pose.orientation.w = 1.0;
  wall1_pose.position.x = 0.8;
  wall1_pose.position.y = 0.2;
  wall1_pose.position.z = 0.25;
  moveit_msgs::msg::CollisionObject collision_object_wall1 = createCollisionObject("wall1", primitive_wall1, wall1_pose);
  
  shape_msgs::msg::SolidPrimitive primitive_wall2;
  primitive_wall2.type = primitive_wall2.BOX;
  primitive_wall2.dimensions = {1.4, 0.1, 0.5};
  geometry_msgs::msg::Pose wall2_pose;
  wall2_pose.orientation.w = 1.0;
  wall2_pose.position.x = 0.2;
  wall2_pose.position.y = 0.8;
  wall2_pose.position.z = 0.25;
  moveit_msgs::msg::CollisionObject collision_object_wall2 = createCollisionObject("wall2", primitive_wall2, wall2_pose);

  shape_msgs::msg::SolidPrimitive primitive_wall3;
  primitive_wall3.type = primitive_wall3.BOX;
  primitive_wall3.dimensions = {0.1, 1.4, 0.5};
  geometry_msgs::msg::Pose wall3_pose;
  wall3_pose.orientation.w = 1.0;
  wall3_pose.position.x = -0.6;
  wall3_pose.position.y = 0.2;
  wall3_pose.position.z = 0.25;
  moveit_msgs::msg::CollisionObject collision_object_wall3 = createCollisionObject("wall3", primitive_wall3, wall3_pose);
  
  shape_msgs::msg::SolidPrimitive primitive_wall4;
  primitive_wall4.type = primitive_wall4.BOX;
  primitive_wall4.dimensions = {1.4, 0.1, 0.5};
  geometry_msgs::msg::Pose wall4_pose;
  wall4_pose.orientation.w = 1.0;
  wall4_pose.position.x = 0.2;
  wall4_pose.position.y = -0.6;
  wall4_pose.position.z = 0.25;
  moveit_msgs::msg::CollisionObject collision_object_wall4 = createCollisionObject("wall4", primitive_wall4, wall4_pose);

  /* shape_msgs::msg::SolidPrimitive primitive_table2;
  primitive_table2.type = primitive_table2.BOX;
  primitive_table2.dimensions = {0.6, 0.9, 0.2};
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = -0.60;
  table2_pose.position.y = 0.0;
  table2_pose.position.z = 0.1;
  moveit_msgs::msg::CollisionObject collision_object_table2 = createCollisionObject("table2", primitive_table2, table2_pose); */


  /* shape_msgs::msg::SolidPrimitive primitive_target1;
  primitive_target1.type = primitive_target1.CYLINDER;
  primitive_target1.dimensions = {0.1, 0.02}; // height, radius
  geometry_msgs::msg::Pose target1_pose;
  target1_pose.orientation.w = 1.0;
  target1_pose.position.x = 0.45;
  target1_pose.position.y = -0.35;
  target1_pose.position.z = 0.25;
  moveit_msgs::msg::CollisionObject collision_object_target1 = createCollisionObject("target1", primitive_target1, target1_pose);
 */
  shape_msgs::msg::SolidPrimitive primitive_surface;
  primitive_surface.type = primitive_surface.BOX;
  primitive_surface.dimensions = {0.75, 0.8, 0.1};
  geometry_msgs::msg::Pose surface_pose;
  surface_pose.orientation.w = 1.0;
  surface_pose.position.x = 0.16;
  surface_pose.position.y = 0.25;
  surface_pose.position.z = -0.05;
  moveit_msgs::msg::CollisionObject collision_object_surface = createCollisionObject("surface", primitive_surface, surface_pose);

  /* // Create object colors
  moveit_msgs::msg::ObjectColor table1_color = createObjectColor("table1", 1.0, 0.5, 0.5, 1.0);
  moveit_msgs::msg::ObjectColor table2_color = createObjectColor("table2", 1.0, 0.5, 0.5, 1.0);
  moveit_msgs::msg::ObjectColor target1_color = createObjectColor("target1", 0.5, 0.0, 1.0, 1.0); */
  moveit_msgs::msg::ObjectColor surface_color = createObjectColor("surface", 0.5, 0.5, 0.5, 1.0);
  moveit_msgs::msg::ObjectColor wall1_color = createObjectColor("wall1", 0.1, 0.2, 0.3, 1.0); // -y axis
  moveit_msgs::msg::ObjectColor wall2_color = createObjectColor("wall2", 1.0, 1.0, 1.0, 1.0); // +x axis
  moveit_msgs::msg::ObjectColor wall3_color = createObjectColor("wall3", 1.0, 1.0, 1.0, 1.0); // +y axis
  moveit_msgs::msg::ObjectColor wall4_color = createObjectColor("wall4", 1.0, 1.0, 1.0, 1.0); //- x axis


  // Create a PlanningScene message and add the collision objects and their colors
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects = {collision_object_surface, collision_object_wall1,collision_object_wall2,collision_object_wall3,collision_object_wall4};
  planning_scene_msg.object_colors = {surface_color, wall1_color, wall2_color, wall3_color, wall4_color};
  /* planning_scene_msg.world.collision_objects = {collision_object_table1, collision_object_table2, collision_object_target1, collision_object_surface};
  planning_scene_msg.object_colors = {table1_color, table2_color, target1_color, surface_color}; */

  // Apply the planning scene
  planning_scene_interface_.applyPlanningScene(planning_scene_msg);

  // Publish the planning scene
  planning_scene_publisher_->publish(planning_scene_msg);

  RCLCPP_INFO(this->get_logger(), "Planning scene setup complete");
}

moveit_msgs::msg::CollisionObject PlanningSceneNode::createCollisionObject(
  const std::string& id, const shape_msgs::msg::SolidPrimitive& primitive, const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = id;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;
  return collision_object;
}

moveit_msgs::msg::ObjectColor PlanningSceneNode::createObjectColor(
  const std::string& id, float r, float g, float b, float a)
{
  moveit_msgs::msg::ObjectColor color;
  color.id = id;
  color.color.r = r;
  color.color.g = g;
  color.color.b = b;
  color.color.a = a;
  return color;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}