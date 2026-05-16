import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    r6bot_gazebo_pkg = FindPackageShare("r6bot_gazebo").find("r6bot_gazebo")
    r6bot_description_pkg = FindPackageShare("r6bot_description").find("r6bot_description")
    r6bot_control_pkg = FindPackageShare("r6bot_control").find("r6bot_control")
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
  
    r6bot_model = os.path.join(r6bot_description_pkg, "urdf", "fixed_r6bot.urdf.xacro")
    robot_description = {"robot_description": Command(["xacro ", r6bot_model])}

    gazebo = IncludeLaunchDescription(
        os.path.join(ros_gz_sim_pkg, "launch", "gz_sim.launch.py"),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "fixed_r6bot",
            "-allow_renaming",
            "true",
        ],
    )

    bola = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'bola',
            '-file', 
            os.path.join(r6bot_gazebo_pkg, "models", "bola", "bola.sdf"),
            '-x', '1.0', '-y', '2.0', '-z', '0.5', '-R', '1.5708'
        ],
        output='screen'
    )


    mesa = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mesa',
            '-file', 
            os.path.join(r6bot_gazebo_pkg, "models", "mesa", "mesa.sdf"),
            '-x', '1.0', '-y', '-1.0', '-z', '0.0', '-R', '0.00'
        ],
        output='screen'
    )


    cadeira_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cadeira_1',
            '-file', 
            os.path.join(r6bot_gazebo_pkg, "models", "cadeira", "cadeira.sdf"),
            '-x', '1.5', '-y', '-1.0', '-z', '0.0', '-R', '0.00'
        ],
        output='screen'
    )

    cadeira_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cadeira_2',
            '-file', 
            os.path.join(r6bot_gazebo_pkg, "models", "cadeira", "cadeira.sdf"),
            '-x', '0.5', '-y', '-1.0', '-z', '0.0', '-R', '0.0', '-P','0.0', '-Y','3.1415'
        ],
        output='screen'
    )


    cadeira_3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cadeira_3',
            '-file', 
            os.path.join(r6bot_gazebo_pkg, "models", "cadeira", "cadeira.sdf"),
            '-x', '1.0', '-y', '-0.75', '-z', '0.0', '-R', '-0.00' ,'-P','0.00','-Y','1.54'
        ],
        output='screen'
    )

    cadeira_4 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cadeira_4',
            '-file', 
            os.path.join(r6bot_gazebo_pkg, "models", "cadeira", "cadeira.sdf"),
            '-x', '1.0', '-y', '-1.25', '-z', '0.0', '-R', '0.00','-P','0.00','-Y','-1.54'
        ],
        output='screen'
    )


    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    r6bot_control = IncludeLaunchDescription(
        os.path.join(r6bot_control_pkg, "launch", "r6bot_control.launch.py")
    )

    return LaunchDescription([
        gazebo,
        gz_spawn_entity,
        gazebo_bridge,
        robot_state_publisher,
        bola,
        mesa,
        cadeira_1,
        cadeira_2,
        cadeira_3,
        cadeira_4,
        r6bot_control,
    ])
