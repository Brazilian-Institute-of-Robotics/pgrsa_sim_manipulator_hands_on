"""
control_strategies.launch.py
Lança as 3 estratégias de controle simultaneamente para comparação.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    target_arg = DeclareLaunchArgument('target', default_value='pick')

    local_pd = Node(
        package='kuka_control_strategies',
        executable='local_pd_controller',
        name='local_pd_controller',
        output='screen',
        parameters=[{
            'kp':     [200.0, 200.0, 150.0, 100.0, 80.0, 50.0],
            'kd':     [20.0,  20.0,  15.0,  10.0,  8.0,  5.0],
            'target': LaunchConfiguration('target'),
        }]
    )

    computed_torque = Node(
        package='kuka_control_strategies',
        executable='computed_torque',
        name='computed_torque_controller',
        output='screen',
        parameters=[{
            'kp':     [150.0, 150.0, 120.0, 80.0, 60.0, 40.0],
            'kd':     [25.0,  25.0,  20.0,  15.0, 10.0, 8.0],
            'target': LaunchConfiguration('target'),
        }]
    )

    operational_space = Node(
        package='kuka_control_strategies',
        executable='operational_space',
        name='operational_space_controller',
        output='screen',
        parameters=[{
            'kp_pos': [300.0, 300.0, 300.0],
            'kd_pos': [30.0,  30.0,  30.0],
            'target': 'center',
        }]
    )

    benchmark = Node(
        package='kuka_control_strategies',
        executable='strategy_benchmark',
        name='strategy_benchmark',
        output='screen',
        parameters=[{'duration': 60.0, 'threshold': 0.01}]
    )

    return LaunchDescription([
        target_arg,
        local_pd,
        computed_torque,
        operational_space,
        benchmark,
    ])
