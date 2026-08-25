from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tare = LaunchConfiguration("tare_duration")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tare_duration",
                default_value="5.0",
                description="segundos de calibracao do sensor " "virtual de forca",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="kuka_force_lab",
                        executable="spawn_panel",
                        name="panel_spawner",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=9.0,
                actions=[
                    Node(
                        package="kuka_force_lab",
                        executable="ft_estimator",
                        name="ft_estimator",
                        output="screen",
                        parameters=[{"tare_duration": tare, "auto_tare": True}],
                    ),
                ],
            ),
        ]
    )
