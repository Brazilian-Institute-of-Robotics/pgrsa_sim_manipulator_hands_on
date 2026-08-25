from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wn = LaunchConfiguration("wn")
    payload = LaunchConfiguration("payload")

    return LaunchDescription(
        [
            DeclareLaunchArgument("wn", default_value="6.0"),
            DeclareLaunchArgument("payload", default_value="25.0"),
            Node(
                package="kuka_control_lab",
                executable="strategy_compare",
                name="compare_nominal",
                output="screen",
                parameters=[
                    {
                        "wn": wn,
                        "zeta": 1.0,
                        "trajectory": "quintic",
                        "start": "home",
                        "goal": "pick",
                        "move_time": 3.0,
                        "duration": 6.0,
                        "payload": 0.0,
                        "label": "nominal",
                    }
                ],
            ),
            Node(
                package="kuka_control_lab",
                executable="strategy_compare",
                name="compare_exigente",
                output="screen",
                parameters=[
                    {
                        "wn": wn,
                        "zeta": 1.0,
                        "trajectory": "quintic",
                        "start": "home",
                        "goal": "pick",
                        "move_time": 1.2,
                        "duration": 4.0,
                        "payload": payload,
                        "label": "exigente",
                    }
                ],
            ),
        ]
    )
