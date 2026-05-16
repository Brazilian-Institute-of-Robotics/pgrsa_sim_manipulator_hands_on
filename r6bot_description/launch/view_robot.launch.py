import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    r6bot_description_pkg = FindPackageShare("r6bot_description").find("r6bot_description")
    r6bot_model = os.path.join(r6bot_description_pkg, "urdf", "fixed_r6bot.urdf.xacro")

    # Get URDF via xacro
    robot_description = {"robot_description": Command(["xacro ", r6bot_model])}

    rviz_config_file = os.path.join(r6bot_description_pkg, "rviz", "r6bot.rviz")

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
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
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
