import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [FindPackageShare('pick_and_place_description'),
              "urdf", "robot_arm.xacro"]
          ),
          " ",
          "use_gazebo:=",
          "true",
          " ",
          "use_ignition:=",
          "false",
          " ",
          "use_fake_hardware:=",
          "false"
      ]
  )
  robot_description = {"robot_description": robot_description_content}

  bringup_dir = get_package_share_directory('pick_and_place_moveit_config')
  srdf_path = os.path.join(bringup_dir, 'srdf', 'robot_arm.srdf')
  srdf = open(srdf_path).read()

  robot_description_semantic = {"robot_description_semantic": srdf}

  robot_description_kinematics = PathJoinSubstitution(
      [FindPackageShare('pick_and_place_moveit_config'), "config", "kinematics.yaml"]
  )
  move_group_demo = Node(
      package="pick_and_place_test_nodes",
      executable="test_pick_and_place",
      output="screen",
      parameters=[
          robot_description,
          robot_description_semantic,
          robot_description_kinematics,
          {"use_sim_time": True}
      ],
  )

  return LaunchDescription([move_group_demo])
