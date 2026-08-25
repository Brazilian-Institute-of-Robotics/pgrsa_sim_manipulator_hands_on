import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

# Importa a cinemática correta do kuka_forward_kinematicso)
from kuka_forward_kinematics.kinematics import (
    forward_kinematics,
    geometric_jacobian,
    gravity_vector as compute_gravity_vector,
    JOINT_NAMES,
    JOINT_LIMITS,
)


class AdmittanceController(Node):

    def __init__(self):
        super().__init__("admittance_controller")

        self.declare_parameter("M_d", [2.0, 2.0, 2.0])  # massa virtual (kg)
        self.declare_parameter("B_d", [20.0, 20.0, 20.0])  # amortecimento virtual
        self.declare_parameter(
            "K_d", [0.0, 0.0, 0.0]
        )  # rigidez virtual (0 = admitância pura)
        self.declare_parameter("F_threshold", 1.0)  # N — threshold de ativação
        self.declare_parameter("max_deviation", 0.05)  # m — desvio máximo permitido
        self.declare_parameter("target", "pick")

        M_d = self.get_parameter("M_d").value
        B_d = self.get_parameter("B_d").value
        K_d = self.get_parameter("K_d").value

        self.M_inv = np.diag([1.0 / m for m in M_d])
        self.B = np.diag(B_d)
        self.K = np.diag(K_d)
        self.F_thresh = self.get_parameter("F_threshold").value
        self.max_dev = self.get_parameter("max_deviation").value

        targets = {
            "pick": [0.0, -1.0, 1.2, 0.0, 0.8, 0.0],
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "stretch": [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
        }
        self.q_nom = np.array(
            targets.get(self.get_parameter("target").value, targets["pick"])
        )

        self.x_adm = np.zeros(3)  # desvio de posição
        self.xd_adm = np.zeros(3)  # velocidade de desvio
        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}
        self.F_ext = np.zeros(3)
        self.F_received = False

        self.dt = 0.02
        self.t_prev = None

        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._cb_js, 10
        )

        self.sub_ft = self.create_subscription(
            WrenchStamped, "/kuka_force_control/ft_wrist", self._cb_ft, 10
        )

        self.pub_traj = self.create_publisher(
            JointTrajectory, "/kuka_arm_controller/joint_trajectory", 10
        )

        self.pub_deviation = self.create_publisher(
            Float64MultiArray, "/kuka_force_control/admittance_deviation", 10
        )

        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f"[Admittance] Iniciado | M={M_d} | B={B_d} | K={K_d} | "
            f"F_thresh={self.F_thresh}N"
        )

    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q[j] = msg.position[idx]
                self.qd[j] = msg.velocity[idx] if msg.velocity else 0.0

    def _cb_ft(self, msg: WrenchStamped):
        self.F_ext = np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
            ]
        )
        self.F_received = True

    def _control_loop(self):
        F_eff = self.F_ext.copy()
        fn = np.linalg.norm(F_eff)
        if fn < self.F_thresh:
            F_eff = np.zeros(3)

        xdd = self.M_inv @ (F_eff - self.B @ self.xd_adm - self.K @ self.x_adm)
        self.xd_adm += xdd * self.dt
        self.x_adm += self.xd_adm * self.dt
        dev_norm = np.linalg.norm(self.x_adm)
        if dev_norm > self.max_dev:
            self.x_adm = self.x_adm / dev_norm * self.max_dev

        T_nom, transforms_nom = forward_kinematics(self.q_nom)
        p_nom = T_nom[:3, 3]

        p_des = p_nom + self.x_adm

        J = geometric_jacobian(self.q_nom, transforms_nom)
        J_pinv = np.linalg.pinv(J[:3, :])  # apenas parte linear

        delta_q = J_pinv @ self.x_adm
        q_des = self.q_nom + delta_q
        q_des = np.clip(q_des, [-np.pi] * 6, [np.pi] * 6)

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = q_des.tolist()
        pt.time_from_start = Duration(nanosec=int(self.dt * 1e9 * 2))
        traj.points = [pt]
        self.pub_traj.publish(traj)

        dev_msg = Float64MultiArray()
        dev_msg.data = [*self.x_adm.tolist(), float(fn)]
        self.pub_deviation.publish(dev_msg)

        if fn > self.F_thresh:
            self.get_logger().info(
                f"[Admittance] |F|={fn:.2f}N | desvio=({self.x_adm[0]:.4f}, "
                f"{self.x_adm[1]:.4f}, {self.x_adm[2]:.4f}) m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
