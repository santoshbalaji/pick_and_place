cmake_minimum_required(VERSION 3.8)
project(pick_and_place_test_nodes)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(test_sending_trajectory src/test_sending_trajectory.cpp)

ament_target_dependencies(test_sending_trajectory
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(test_send_joint_space_goal src/test_send_joint_space_goal.cpp)

ament_target_dependencies(test_send_joint_space_goal
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

add_executable(test_send_cartesian_goal src/test_send_cartesian_goal.cpp)

ament_target_dependencies(test_send_cartesian_goal
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS
    test_sending_trajectory test_send_joint_space_goal test_send_cartesian_goal
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
