from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sweep = Node(
        package='kuka_gain_analysis',
        executable='gain_sweep',
        name='gain_sweep',
        output='screen',
        parameters=[{
            'kp_values':      [50.0, 100.0, 200.0, 400.0, 800.0],
            'kd_values':      [5.0,  10.0,  20.0,  40.0,  80.0],
            'test_duration':  8.0,
            'settle_threshold': 0.02,
            'output_csv':     '/tmp/gain_sweep_results.csv',
        }]
    )
    monitor = Node(
        package='kuka_gain_analysis',
        executable='stability_monitor',
        name='stability_monitor',
        output='screen',
    )
    report = Node(
        package='kuka_gain_analysis',
        executable='gain_report',
        name='gain_report',
        output='screen',
        parameters=[{'input_csv': '/tmp/gain_sweep_results.csv'}]
    )
    return LaunchDescription([sweep, monitor, report])
