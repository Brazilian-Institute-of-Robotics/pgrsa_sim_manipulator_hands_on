import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    r6bot_control_pkg = FindPackageShare("r6bot_control").find("r6bot_control")

    robot_controllers = os.path.join(r6bot_control_pkg, "config", "r6bot_controllers.yaml")

    # O plugin do Gazebo já executa o control_node
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, robot_controllers],
    #     output="both",
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--param-file", robot_controllers],
    )

    return LaunchDescription([
        # control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
