#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class TargetPoseNode : public rclcpp::Node
{
public:
  TargetPoseNode();

private:
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
};

TargetPoseNode::TargetPoseNode() : Node("target_pose_node")
{
  target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10, std::bind(&TargetPoseNode::targetPoseCallback, this, std::placeholders::_1));
  target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/task_pose", 10);
}

void TargetPoseNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "New target pose received: (%.2f, %.2f, %.2f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  target_pose_publisher_->publish(*msg);
}
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}