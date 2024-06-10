#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) 
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_send_cartesian_goal_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "arm_group");

  geometry_msgs::msg::Pose current_pose =
    move_group_interface.getCurrentPose().pose;

  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);

  // Set a target Pose
  auto const target_pose = [current_pose] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = current_pose.orientation.w;
    msg.orientation.x = 1e-6;
    msg.orientation.y = 1e-6;
    msg.orientation.z = 1e-6;
    msg.position.x = current_pose.position.x + 0.1;
    msg.position.y = current_pose.position.y + 0.2;
    msg.position.z = current_pose.position.z - 0.4;
    return msg;
  }();
  // move_group_interface.setPoseTarget(target_pose);
  // move_group_interface.setGoalOrientationTolerance(0.5);
  // move_group_interface.setGoalPositionTolerance(0.5);
  // move_group_interface.setGoalTolerance(0.5);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("test_send_cartesian_goal_node"), "Planing failed!");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}