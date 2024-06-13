import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix('pick_and_place_description'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

    use_gazebo = LaunchConfiguration("use_gazebo")
    use_ignition = LaunchConfiguration("use_ignition")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

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
            use_gazebo,
            " ",
            "use_ignition:=",
            use_ignition,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pick_and_place_description"),
         "rviz", "visualize_robot.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_position_controller'],
        output='screen'
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[rviz_node],
        )
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])],
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'robot_arm',
                   '-allow_renaming', 'true'],
    )

    delay_broadcaster_after_spawn_entity_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_controller_after_broadcaster_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    # static_transform_publisher_for_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ["0", "0", "1.2", "1.57", "3.14", "0.0", "base_link", "camera_link"],
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # delay_stfpublisher_after_gazebo_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity_node,
    #         on_exit=[static_transform_publisher_for_camera],
    #     )
    # )

    nodes_to_start = [
        robot_state_pub_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_broadcaster_after_spawn_entity_node,
        delay_controller_after_broadcaster_node,
        gazebo_node,
        spawn_entity_node,
        # delay_stfpublisher_after_gazebo_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Switch to enable gazebo control hardware plugin or not",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ignition",
            default_value="true",
            description="Switch to enable ignition control hardware plugin or not",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Switch to enable fake hardware hardware plugin or not",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
