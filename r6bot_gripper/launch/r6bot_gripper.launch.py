import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_gripper    = FindPackageShare("r6bot_gripper").find("r6bot_gripper")
    pkg_control    = FindPackageShare("r6bot_control").find("r6bot_control")
    pkg_gazebo     = FindPackageShare("r6bot_gazebo").find("r6bot_gazebo")
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")

    xacro_wrapper = os.path.join(pkg_gripper, "urdf", "r6bot_with_gripper.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_wrapper]),
            value_type=str
        )
    }

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        os.path.join(ros_gz_sim_pkg, "launch", "gz_sim.launch.py"),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
    )

    # ── 2. Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ── 3. Bridge ─────────────────────────────────────────────────────────────
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

    # ── 4. Spawn robô (t=8s) ──────────────────────────────────────────────────
    gz_spawn_entity = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "/robot_description",
                    "-name",  "r6bot_with_gripper",
                    "-allow_renaming", "true",
                ],
            )
        ],
    )

    # ── 5. Spawn bola ─────────────────────────────────────────────────────────
    bola = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "bola",
                    "-file", os.path.join(pkg_gazebo, "models", "bola", "bola.sdf"),
                    "-x", "1.0", "-y", "2.0", "-z", "0.5", "-R", "1.5708",
                ],
                output="screen",
            )
        ],
    )

    # ── 6. Controladores do braço + gripper (t=15s) ───────────────────────────
    r6bot_control = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(pkg_control, "launch", "r6bot_control.launch.py")
            )
        ],
    )

    # ── 7. RViz ───────────────────────────────────────────────────────────────
    rviz_config = os.path.join(
        FindPackageShare("r6bot_description").find("r6bot_description"),
        "rviz", "r6bot.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    # ── 8. rqt_joint_trajectory_controller — controla braço E gripper ─────────
    # Após subir: selecione o controller no dropdown e use os sliders.
    # Para o braço: selecione joint_trajectory_controller
    # Para o gripper: selecione gripper_action_controller
    rqt_joint_traj = TimerAction(
        period=17.0,   # aguarda controllers estarem ativos
        actions=[
            Node(
                package="rqt_joint_trajectory_controller",
                executable="rqt_joint_trajectory_controller",
                name="rqt_joint_trajectory_controller",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # ── 9. rqt_gui com plugin Publisher — para tópicos do gripper ─────────────
    # Permite publicar em /gripper_action_controller/commands manualmente
    rqt = TimerAction(
        period=17.0,
        actions=[
            Node(
                package="rqt_gui",
                executable="rqt_gui",
                name="rqt_gui",
                output="screen",
                arguments=["--perspective-file",
                           os.path.join(pkg_gripper, "config", "gripper.perspective")]
                if os.path.exists(os.path.join(pkg_gripper, "config", "gripper.perspective"))
                else [],
            )
        ],
    )

    return LaunchDescription([
        gazebo,                  # t=0s
        robot_state_publisher,
        gazebo_bridge,
        rviz,
        gz_spawn_entity,         # t=8s
        bola,
        r6bot_control,           # t=15s
        rqt_joint_traj,          # t=17s — interface de juntas
        rqt,                     # t=17s — rqt geral
    ])