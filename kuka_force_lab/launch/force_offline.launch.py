from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    f_des = LaunchConfiguration("f_des")
    overshoot = LaunchConfiguration("overshoot")

    def compare(material, label):
        return Node(
            package="kuka_force_lab",
            executable="force_compare",
            name=f"force_compare_{label}",
            output="screen",
            parameters=[
                {
                    "material": material,
                    "f_des": f_des,
                    "overshoot": overshoot,
                    "label": label,
                }
            ],
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("f_des", default_value="20.0"),
            DeclareLaunchArgument("overshoot", default_value="0.02"),
            compare("espuma", "espuma"),
            compare("madeira", "madeira"),
            compare("aco", "aco"),
        ]
    )
