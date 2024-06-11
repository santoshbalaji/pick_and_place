#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "test_send_joint_space_goal_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("test_send_joint_space_goal_node");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_group");

  // pick
  std::vector<double> joint_group_positions_pick = {0.0, 0.802, 0.942, 1.361};
  move_group_interface.setJointValueTarget(joint_group_positions_pick);

  auto const [pick_success, pick_plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (pick_success)
  {
    move_group_interface.execute(pick_plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client =
    node->create_client<std_srvs::srv::SetBool>("switch");

  auto pick_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  pick_request->data = true;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto pick_result = client->async_send_request(pick_request);
  if (rclcpp::spin_until_future_complete(node, pick_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", pick_result.get()->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service switch vacuum");
  }

  std::vector<double> joint_group_positions_place = {1.431, 0.802, 0.942, 1.361};
  move_group_interface.setJointValueTarget(joint_group_positions_place);

  auto const [place_success, place_plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (place_success)
  {
    move_group_interface.execute(place_plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  auto place_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  place_request->data = false;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto place_result = client->async_send_request(place_request);
  if (rclcpp::spin_until_future_complete(node, place_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", place_result.get()->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service switch vacuum");
  }

  rclcpp::shutdown();
  return 0;
}