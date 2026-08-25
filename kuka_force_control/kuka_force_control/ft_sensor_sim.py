import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray

from kuka_forward_kinematics.kinematics import (
    forward_kinematics,
    geometric_jacobian,
    gravity_vector as compute_gravity_vector,
    JOINT_NAMES,
    JOINT_LIMITS,
)

# Inércias dos elos
INERTIA_DIAG = np.array([50.0, 20.0, 8.0, 2.0, 0.5, 0.1])


class FTSensorSim(Node):

    def __init__(self):
        super().__init__("ft_sensor_sim")

        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("force_threshold", 2.0)
        self.declare_parameter("filter_alpha", 0.3)

        self.rate = self.get_parameter("publish_rate").value
        self.force_threshold = self.get_parameter("force_threshold").value
        self.alpha = self.get_parameter("filter_alpha").value

        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.qdd = np.zeros(6)
        self.tau = np.zeros(6)
        self.qd_prev = np.zeros(6)
        self.t_prev = None
        self.joint_order = {}

        self.wrench_filtered = np.zeros(6)

        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._cb_js, 10
        )

        self.pub_wrench = self.create_publisher(
            WrenchStamped, "/kuka_force_control/ft_wrist", 10
        )

        self.pub_contact = self.create_publisher(
            Float64MultiArray, "/kuka_force_control/contact_force_norm", 10
        )

        self.timer = self.create_timer(1.0 / self.rate, self._estimate_loop)

        self.get_logger().info("[FTSensorSim] Sensor F/T virtual iniciado no wrist.")

    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}

        t_now = self.get_clock().now()
        dt = 0.02
        if self.t_prev is not None:
            dt = max((t_now - self.t_prev).nanoseconds / 1e9, 1e-3)
        self.t_prev = t_now

        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                q_new = msg.position[idx]
                qd_new = msg.velocity[idx] if msg.velocity else 0.0
                self.qdd[j] = (qd_new - self.qd[j]) / dt
                self.q[j] = q_new
                self.qd[j] = qd_new

        if msg.effort and len(msg.effort) >= 6:
            for j, name in enumerate(JOINT_NAMES):
                if name in self.joint_order:
                    idx = self.joint_order[name]
                    self.tau[j] = msg.effort[idx]

    def _estimate_loop(self):

        T_0n, transforms = forward_kinematics(self.q)
        J = geometric_jacobian(self.q, transforms)

        g = compute_gravity_vector(self.q)

        tau_inertia = INERTIA_DIAG * self.qdd

        tau_ext = self.tau - g - tau_inertia

        JT = J.T
        lam2 = 0.05**2
        wrench_raw = np.linalg.solve(JT.T @ JT + lam2 * np.eye(6), JT.T @ tau_ext)

        F_MAX = 200.0
        TAU_MAX = 50.0
        raw_finite = np.all(np.isfinite(wrench_raw))
        if raw_finite:
            fmag = np.linalg.norm(wrench_raw[:3])
            if fmag > F_MAX:
                wrench_raw[:3] = wrench_raw[:3] * (F_MAX / fmag)
                self.get_logger().warn(
                    f"[FTSensor] Força bruta {fmag:.0f}N truncada para {F_MAX:.0f}N "
                    f"(provável singularidade cinemática nesta pose, não contato real).",
                    throttle_duration_sec=2.0,
                )
            wrench_raw[3:] = np.clip(wrench_raw[3:], -TAU_MAX, TAU_MAX)
        else:
            wrench_raw = None

        if wrench_raw is not None:
            self.wrench_filtered = (
                self.alpha * wrench_raw + (1 - self.alpha) * self.wrench_filtered
            )

        # WrenchStamped
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"
        msg.wrench.force.x = float(self.wrench_filtered[0])
        msg.wrench.force.y = float(self.wrench_filtered[1])
        msg.wrench.force.z = float(self.wrench_filtered[2])
        msg.wrench.torque.x = float(self.wrench_filtered[3])
        msg.wrench.torque.y = float(self.wrench_filtered[4])
        msg.wrench.torque.z = float(self.wrench_filtered[5])
        self.pub_wrench.publish(msg)

        # Norma
        fn = float(np.linalg.norm(self.wrench_filtered[:3]))
        fn_msg = Float64MultiArray()
        fn_msg.data = [fn]
        self.pub_contact.publish(fn_msg)

        if fn > self.force_threshold:
            self.get_logger().warn(
                f"[FTSensor] Contato detectado! |F|={fn:.2f}N | "
                f"F=({self.wrench_filtered[0]:.2f}, "
                f"{self.wrench_filtered[1]:.2f}, "
                f"{self.wrench_filtered[2]:.2f}) N"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FTSensorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
