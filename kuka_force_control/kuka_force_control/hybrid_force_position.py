import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

from kuka_forward_kinematics.kinematics import (
    forward_kinematics,
    geometric_jacobian,
    gravity_vector as compute_gravity_vector,
    JOINT_NAMES,
    JOINT_LIMITS,
)


class HybridForcePosition(Node):
    """
    Controle híbrido força/posição com matriz de seleção S.
    Modo padrão: força em Z (polimento), posição em X e Y.
    """

    def __init__(self):
        super().__init__("hybrid_force_position")

        # Parâmetros de controle de força
        self.declare_parameter("Kf", 50.0)
        self.declare_parameter("Kfi", 5.0)
        self.declare_parameter("F_des", 10.0)

        # Parâmetros de controle de posição (X, Y)
        self.declare_parameter("Kp_pos", [200.0, 200.0])
        self.declare_parameter("Kd_pos", [20.0, 20.0])

        self.declare_parameter("p_des_x", 1.2)
        self.declare_parameter("p_des_y", 0.0)

        self.declare_parameter("mode", "surface_polish")

        self.Kf = self.get_parameter("Kf").value
        self.Kfi = self.get_parameter("Kfi").value
        self.F_des = self.get_parameter("F_des").value
        self.Kp_pos = np.array(self.get_parameter("Kp_pos").value)
        self.Kd_pos = np.array(self.get_parameter("Kd_pos").value)
        self.p_des_xy = np.array(
            [
                self.get_parameter("p_des_x").value,
                self.get_parameter("p_des_y").value,
            ]
        )
        self.mode = self.get_parameter("mode").value

        self.S_f, self.S_p = self._build_selection_matrices(self.mode)

        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}
        self.F_ext = np.zeros(6)
        self.F_int = 0.0
        self.dt = 0.02

        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._cb_js, 10
        )
        self.sub_ft = self.create_subscription(
            WrenchStamped, "/kuka_force_control/ft_wrist", self._cb_ft, 10
        )
        self.pub_traj = self.create_publisher(
            JointTrajectory, "/kuka_arm_controller/joint_trajectory", 10
        )
        self.pub_metrics = self.create_publisher(
            Float64MultiArray, "/kuka_force_control/hybrid_metrics", 10
        )

        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f"[HybridForcePos] Modo: {self.mode} | "
            f"F_des={self.F_des}N | Kf={self.Kf}"
        )

    def _build_selection_matrices(self, mode: str):
        """Constrói matrizes de seleção S_f e S_p = I - S_f."""
        S_f = np.zeros((6, 6))
        if mode == "surface_polish":
            S_f[2, 2] = 1.0
        elif mode == "wall_contact":
            S_f[0, 0] = 1.0
        elif mode == "free":
            pass
        S_p = np.eye(6) - S_f
        return S_f, S_p

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
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ]
        )

    def _control_loop(self):
        T_0n, transforms = forward_kinematics(self.q)
        p_cur = T_0n[:3, 3]
        J = geometric_jacobian(self.q, transforms)
        J_pinv = np.linalg.pinv(J)
        g = compute_gravity_vector(self.q)

        F_z_cur = self.F_ext[2]
        e_f = self.F_des - F_z_cur
        self.F_int += e_f * self.dt
        self.F_int = np.clip(self.F_int, -50.0, 50.0)
        u_f_z = self.Kf * e_f + self.Kfi * self.F_int  # força em Z

        F_cmd = self.S_f @ np.array([0.0, 0.0, u_f_z, 0.0, 0.0, 0.0])
        tau_f = J.T @ F_cmd

        ep_xy = self.p_des_xy - p_cur[:2]
        # ev_xy = -np.array(self.qd[:2]) * 0.0  # simplificado

        u_p = np.array(
            [
                self.Kp_pos[0] * ep_xy[0],
                self.Kp_pos[1] * ep_xy[1],
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )
        F_pos = self.S_p @ u_p
        tau_p = J.T @ F_pos

        tau_total = tau_f + tau_p + g

        q_des = self.q + J_pinv @ (
            np.array([ep_xy[0] * 0.005, ep_xy[1] * 0.005, u_f_z * 0.001, 0.0, 0.0, 0.0])
        )
        q_des = np.clip(q_des, [-np.pi] * 6, [np.pi] * 6)

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = q_des.tolist()
        pt.time_from_start = Duration(nanosec=int(self.dt * 1e9 * 2))
        traj.points = [pt]
        self.pub_traj.publish(traj)

        m = Float64MultiArray()
        m.data = [
            float(F_z_cur),
            float(self.F_des),
            float(e_f),
            float(np.linalg.norm(ep_xy)),
            float(np.linalg.norm(tau_total)),
        ]
        self.pub_metrics.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = HybridForcePosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
