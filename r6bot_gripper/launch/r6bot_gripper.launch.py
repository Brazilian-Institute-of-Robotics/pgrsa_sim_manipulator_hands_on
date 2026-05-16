"""
Launch: r6bot + gripper paralelo + bola no Gazebo
  ros2 launch r6bot_gripper r6bot_gripper.launch.py

Pré-requisitos:
  - ros2_control_demo_example_7 instalado (fornece o r6bot)
  - gz_ros2_control instalado
  - bola.sdf na pasta atual ou no GZ_SIM_RESOURCE_PATH
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Pacotes ────────────────────────────────────────────────────────────────
    pkg_gripper  = get_package_share_directory("r6bot_gripper")
    pkg_r6bot    = get_package_share_directory("r6bot_description")

    # ── Argumentos ────────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usa o tempo de simulação do Gazebo",
    )

    # ── URDF combinado: r6bot + gripper ───────────────────────────────────────
    #  O xacro do gripper assume que 'tool0' já existe (definido pelo r6bot).
    #  Fazemos um xacro wrapper que inclui os dois.
    robot_description_cmd = Command([
        "xacro ",
        os.path.join(pkg_gripper, "description", "urdf", "r6bot_with_gripper.urdf.xacro"),
    ])

    robot_description = {"robot_description": robot_description_cmd}

    # ── Robot State Publisher ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ── Gazebo (Ignition / GZ Sim) ────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "empty.sdf"],
        output="screen",
    )

    # Spawn do robô no Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "r6bot_with_gripper",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
        output="screen",
    )

    # Spawn da bola (bola.sdf deve estar acessível)
    spawn_ball = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", os.path.join(pkg_gripper, "description", "bola.sdf"),
            "-name", "bola",
            "-x", "0.5",
            "-y", "0.0",
            "-z", "0.80",   # posição acima da mesa
        ],
        output="screen",
    )

    # ── ros2_control node ─────────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg_gripper, "config", "gripper_controllers.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # ── Spawners dos controladores (com delay para o CM inicializar) ──────────
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    # ── RViz (opcional) ───────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_gripper, "config", "gripper.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        spawn_ball,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        rviz,
    ])
