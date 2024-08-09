from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "package",
            default_value="pi3hat_hardware_interface",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_file",
            default_value="test_single_motor.urdf.xacro",
        )
    )


    # Initialize Arguments
    package_name = LaunchConfiguration("package")
    urdf_file = LaunchConfiguration("urdf_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "urdf",
                    urdf_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ros2_control_demo_description"), "rrbot/rviz", "rrbot.rviz"]
    # )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher"
    )


    nodes = [
        robot_state_pub_node,
        joint_state_publisher, 
    ]

    return LaunchDescription(declared_arguments + nodes)