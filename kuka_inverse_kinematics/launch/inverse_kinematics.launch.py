from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_x',  default_value='1.2'),
        DeclareLaunchArgument('target_y',  default_value='0.5'),
        DeclareLaunchArgument('target_z',  default_value='0.8'),
        DeclareLaunchArgument('method',    default_value='lm'),
        DeclareLaunchArgument('execute',   default_value='true'),
        DeclareLaunchArgument('output_dir',
            default_value=os.path.expanduser('~/kuka_ik_plots')),

        Node(
            package='kuka_inverse_kinematics',
            executable='ik_node',
            name='ik_node',
            output='screen',
            parameters=[{
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_z': LaunchConfiguration('target_z'),
                'method':   LaunchConfiguration('method'),
                'execute':  LaunchConfiguration('execute'),
            }]
        ),
        Node(
            package='kuka_inverse_kinematics',
            executable='ik_plotter',
            name='ik_plotter',
            output='screen',
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'target_x':   LaunchConfiguration('target_x'),
                'target_y':   LaunchConfiguration('target_y'),
                'target_z':   LaunchConfiguration('target_z'),
            }]
        ),
    ])
