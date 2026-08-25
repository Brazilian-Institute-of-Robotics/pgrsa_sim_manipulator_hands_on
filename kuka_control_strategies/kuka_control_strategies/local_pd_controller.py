import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

DEMO_TARGETS = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pick": [0.0, -1.0, 1.2, 0.0, 0.8, 0.0],
    "place": [1.57, -1.0, 1.0, 0.0, 0.5, 0.0],
    "stretch": [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
}


class LocalPDController(Node):

    def __init__(self):
        super().__init__("local_pd_controller")

        self.declare_parameter("kp", [200.0, 200.0, 150.0, 100.0, 80.0, 50.0])
        self.declare_parameter("kd", [20.0, 20.0, 15.0, 10.0, 8.0, 5.0])
        self.declare_parameter("target", "pick")
        self.declare_parameter("custom_positions", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.kp = np.array(self.get_parameter("kp").value)
        self.kd = np.array(self.get_parameter("kd").value)
        self.target_name = self.get_parameter("target").value
        custom = self.get_parameter("custom_positions").value
        if self.target_name == "custom":
            self.q_des = np.array(custom)
        else:
            self.q_des = np.array(
                DEMO_TARGETS.get(self.target_name, DEMO_TARGETS["home"])
            )

        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}
        self.errors_history = []
        self.goal_sent = False

        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._cb_joint_states, 10
        )

        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/kuka_arm_controller/follow_joint_trajectory"
        )

        self.pub_error = self.create_publisher(
            Float64MultiArray, "/kuka_control_strategies/pd_error", 10
        )

        # Aguarda action server e envia goal
        self.create_timer(2.0, self._send_goal)

        # Timer para publicar erro
        self.create_timer(0.02, self._publish_error)

        self.get_logger().info(
            f"[LocalPD] Iniciado — alvo: {self.target_name} | "
            f"Kp={self.kp.tolist()} | Kd={self.kd.tolist()}"
        )

    def _cb_joint_states(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q[j] = msg.position[idx]
                self.qd[j] = msg.velocity[idx] if msg.velocity else 0.0

    def _send_goal(self):
        if self.goal_sent:
            return
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("[LocalPD] Action server não disponível ainda...")
            return

        self.goal_sent = True

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        # Waypoints: home → target com perfil de velocidade suave
        pt = JointTrajectoryPoint()
        pt.positions = self.q_des.tolist()
        pt.velocities = [0.0] * 6
        pt.accelerations = [0.0] * 6
        pt.time_from_start = Duration(sec=4)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f"[LocalPD] Enviando goal para: {self.q_des.tolist()}")
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[LocalPD] Goal rejeitado!")
            self.goal_sent = False
            return
        self.get_logger().info("[LocalPD] Goal aceito — robô em movimento.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        e = self.q_des - self.q
        rms = (
            np.sqrt(np.mean(np.array(self.errors_history) ** 2))
            if self.errors_history
            else 0.0
        )
        self.get_logger().info(
            f"[LocalPD] Movimento concluído | "
            f"erro final ||e||={np.linalg.norm(e):.4f} rad | RMS={rms:.4f}"
        )
        # Reseta para permitir novo goal
        self.goal_sent = False

    def _publish_error(self):
        e = self.q_des - self.q
        self.errors_history.append(np.linalg.norm(e))
        err_msg = Float64MultiArray()
        err_msg.data = e.tolist()
        self.pub_error.publish(err_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.errors_history:
            rms = np.sqrt(np.mean(np.array(node.errors_history) ** 2))
            node.get_logger().info(f"[LocalPD] RMS final: {rms:.6f} rad")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
