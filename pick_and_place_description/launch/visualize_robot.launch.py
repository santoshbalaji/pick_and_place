from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessStart


def launch_setup(context, *args, **kwargs):

    # General arguments
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_ignition = LaunchConfiguration("use_ignition")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # rviz config file path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pick_and_place_description"), "rviz",
         "visualize_robot.rviz"]
    )

    # xacro command for model generation
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('pick_and_place_description'),
                 "urdf", "turtlebot_arm.xacro"]
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

    # robot state publisher for publishing robot model
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description]
    )

    # joint state publisher gui
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both"
    )

    # rviz node for visualizing robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # delay start of joint state publisher gui until start of robot state publisher
    delay_joint_state_publisher_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                LogInfo(
                    msg='Started robot state publisher. Starting joint state publisher gui'),
                joint_state_publisher_gui],
        )
    )

    # delay start of rviz until start of joint state publisher gui
    delay_rviz_after_joint_state_publisher_gui = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_publisher_gui,
            on_start=[
                LogInfo(msg='Started joint state publisher gui. Starting rviz node'),
                rviz_node],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        delay_joint_state_publisher_after_robot_state_publisher,
        delay_rviz_after_joint_state_publisher_gui,
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
            default_value="false",
            description="Switch to enable ignition control hardware plugin or not",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Switch to enable fake hardware hardware plugin or not",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
