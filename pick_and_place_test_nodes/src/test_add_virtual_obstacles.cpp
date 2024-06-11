#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_add_virtual_obstacles_node");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("test_add_virtual_obstacles_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });
 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = 
  std::make_shared<tf2_ros::Buffer>(node->get_clock());
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ =
    std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node, "robot_description", tf_buffer_, "planning_scene_monitor");
  
  if (!planning_scene_monitor_->getPlanningScene())
  {
    RCLCPP_ERROR(LOGGER, "The planning scene was not retrieved!");
  }
  else
  {
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->providePlanningSceneService();
    planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
    planning_scene_monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene");
    planning_scene_monitor_->startSceneMonitor();

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "obstacle";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.5, 0.4, 0.2 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.4;
    box_pose.position.z = 0.2;
    box_pose.orientation.x = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // creating a planning scene for adding the collision obect
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    scene->processCollisionObjectMsg(collision_object);
  }
 
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
