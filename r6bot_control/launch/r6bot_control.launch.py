import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    r6bot_control_pkg = FindPackageShare("r6bot_control").find("r6bot_control")
    robot_controllers = os.path.join(r6bot_control_pkg, "config", "r6bot_controllers.yaml")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--param-file", robot_controllers,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--param-file", robot_controllers,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
    ])