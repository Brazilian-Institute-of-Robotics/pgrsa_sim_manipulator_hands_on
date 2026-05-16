#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Declara parâmetros
        self.declare_parameter('joint_names', [f"joint_{i+1}" for i in range(6)])
        self.declare_parameter('goal_names', ["point_1"])
        self.declare_parameter('time_between_goals', 1.0)

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        goal_names = self.get_parameter('goal_names').get_parameter_value().string_array_value
        time_between_goals = self.get_parameter('time_between_goals').get_parameter_value().double_value

        traj = JointTrajectory()
        traj.joint_names = joint_names

        time_from_start = 0.0
        for goal_name in goal_names:
            self.declare_parameter(goal_name, [0.0 for _ in range(6)])
            positions_param = self.get_parameter(goal_name).get_parameter_value().double_array_value
            point = JointTrajectoryPoint()
            point.positions = positions_param
            time_from_start += time_between_goals
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            traj.points.append(point)

        self.publisher_.publish(traj)
        self.get_logger().info("Trajetória completa publicada!")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
