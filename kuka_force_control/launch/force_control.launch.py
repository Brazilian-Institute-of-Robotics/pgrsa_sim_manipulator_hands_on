from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="admittance",
        description="Modo: admittance | hybrid | monitor_only",
    )

    ft_sensor = Node(
        package="kuka_force_control",
        executable="ft_sensor_sim",
        name="ft_sensor_sim",
        output="screen",
        parameters=[
            {
                "publish_rate": 50.0,
                "force_threshold": 2.0,
                "filter_alpha": 0.3,
            }
        ],
    )

    admittance = Node(
        package="kuka_force_control",
        executable="admittance_ctrl",
        name="admittance_controller",
        output="screen",
        parameters=[
            {
                "M_d": [2.0, 2.0, 2.0],
                "B_d": [20.0, 20.0, 20.0],
                "K_d": [0.0, 0.0, 0.0],
                "F_threshold": 1.0,
                "max_deviation": 0.05,
                "target": "pick",
            }
        ],
    )

    hybrid = Node(
        package="kuka_force_control",
        executable="hybrid_force_pos",
        name="hybrid_force_position",
        output="screen",
        parameters=[
            {
                "Kf": 50.0,
                "Kfi": 5.0,
                "F_des": 10.0,
                "Kp_pos": [200.0, 200.0],
                "Kd_pos": [20.0, 20.0],
                "p_des_x": 1.2,
                "p_des_y": 0.0,
                "mode": "surface_polish",
            }
        ],
    )

    contact_monitor = Node(
        package="kuka_force_control",
        executable="contact_monitor",
        name="contact_monitor",
        output="screen",
        parameters=[
            {
                "F_contact": 2.0,
                "F_grasp": 15.0,
                "F_overload": 80.0,
            }
        ],
    )

    return LaunchDescription(
        [
            mode_arg,
            ft_sensor,
            admittance,
            contact_monitor,
            # hybrid,
        ]
    )
