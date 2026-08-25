import json
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from kuka_control_lab import dynamics as dyn
from kuka_forward_kinematics.kinematics import (
    forward_kinematics,
    geometric_jacobian,
    JOINT_NAMES,
)


class FTEstimator(Node):

    def __init__(self):
        super().__init__("ft_estimator")

        p = self.declare_parameter
        p("publish_rate", 50.0)
        p("filter_alpha", 0.25)
        p("vel_filter_alpha", 0.3)
        p("tare_duration", 4.0)
        p("auto_tare", True)
        p("payload_mass", 0.0)
        p("damping_lambda", 0.08)
        p("f_max", 500.0)
        p("tau_max", 100.0)
        p("frame_id", "base_link")

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = float(g("publish_rate"))
        self.alpha = float(g("filter_alpha"))
        self.alpha_v = float(g("vel_filter_alpha"))
        self.tare_duration = float(g("tare_duration"))
        self.auto_tare = bool(g("auto_tare"))
        self.lam = float(g("damping_lambda"))
        self.f_max = float(g("f_max"))
        self.tau_max = float(g("tau_max"))
        self.frame_id = g("frame_id")

        dyn.set_payload(float(g("payload_mass")))

        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.qdd = np.zeros(6)
        self.tau = np.zeros(6)
        self.t_prev = None
        self.have_effort = False
        self.warned_effort = False
        self.msg_count = 0

        self.wrench_filt = np.zeros(6)
        self.bias = np.zeros(6)
        self.tare_buf = deque(maxlen=int(self.rate * self.tare_duration) + 1)
        self.tare_done = not self.auto_tare
        self.t_start = None

        self.create_subscription(JointState, "/joint_states", self._cb_js, 20)
        self.pub_wrench = self.create_publisher(
            WrenchStamped, "/kuka_force_lab/ft_wrist", 10
        )
        self.pub_norm = self.create_publisher(
            Float64MultiArray, "/kuka_force_lab/force_norm", 10
        )
        self.pub_status = self.create_publisher(
            String, "/kuka_force_lab/sensor_status", 10
        )

        self.create_service_placeholder = None
        self.create_timer(1.0 / self.rate, self._loop)

        self.get_logger().info(
            f"[FT] sensor virtual iniciado a {self.rate:.0f} Hz. "
            + (
                f"CALIBRANDO por {self.tare_duration:.0f}s — deixe o robô "
                "parado e sem encostar em nada."
                if self.auto_tare
                else "tara automática desligada."
            )
        )

    # ── Leitura ──────────────────────────────────────────────────────────────
    def _cb_js(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return

        stamp = msg.header.stamp
        t_now = stamp.sec + stamp.nanosec * 1e-9
        dt = 0.02 if self.t_prev is None else max(t_now - self.t_prev, 1e-3)
        self.t_prev = t_now

        q_new = np.array([msg.position[idx[n]] for n in JOINT_NAMES])
        if msg.velocity and len(msg.velocity) >= len(msg.name):
            qd_new = np.array([msg.velocity[idx[n]] for n in JOINT_NAMES])
        else:
            qd_new = (q_new - self.q) / dt

        qdd_raw = (qd_new - self.qd) / dt
        self.qdd = self.alpha_v * qdd_raw + (1 - self.alpha_v) * self.qdd
        self.q = q_new
        self.qd = qd_new

        eff = list(msg.effort) if msg.effort else []
        if len(eff) >= len(msg.name):
            tau = np.array([eff[idx[n]] for n in JOINT_NAMES])
            if np.all(np.isfinite(tau)):
                self.tau = tau
                self.have_effort = True
            elif not self.warned_effort:
                self.warned_effort = True
                self._effort_warning("o campo `effort` chega preenchido com NaN")
        elif not self.warned_effort:
            self.warned_effort = True
            self._effort_warning("o campo `effort` chega vazio")

        self.msg_count += 1

    def _effort_warning(self, what):
        self.get_logger().error(
            f"[FT] {what} em /joint_states. Sem torque medido não há "
            "estimativa de força possível — o wrench publicado seria apenas "
            "o negativo da dinâmica compensada."
        )
        self.get_logger().error(
            "[FT] Causa: as juntas do braço não declaram "
            '<state_interface name="effort"/> nos arquivos '
            "*_ros2_control.xacro de kuka_description."
        )
        self.get_logger().error(
            "[FT] Correção: rode  "
            "ros2 run kuka_force_lab enable_effort_interface  "
            "e recompile + reinicie o Gazebo."
        )

    # Estimação de estados
    def _loop(self):
        if self.msg_count < 5:
            return
        now = self.get_clock().now()
        if self.t_start is None:
            self.t_start = now

        T, frames = forward_kinematics(self.q)
        J = geometric_jacobian(self.q, frames)

        tau_model = dyn.inverse_dynamics(self.q, self.qd, self.qdd)
        tau_ext = self.tau - tau_model

        JT = J.T
        wrench = np.linalg.solve(JT.T @ JT + self.lam**2 * np.eye(6), JT.T @ tau_ext)

        ok = np.all(np.isfinite(wrench))
        if ok:
            fmag = np.linalg.norm(wrench[:3])
            if fmag > self.f_max:
                wrench[:3] *= self.f_max / fmag
                self.get_logger().warn(
                    f"[FT] força bruta {fmag:.0f} N truncada em "
                    f"{self.f_max:.0f} N — provável singularidade, não contato.",
                    throttle_duration_sec=3.0,
                )
            wrench[3:] = np.clip(wrench[3:], -self.tau_max, self.tau_max)

        # ── Fase de tara ─────────────────────────────────────────────────────
        elapsed = (now - self.t_start).nanoseconds / 1e9
        if not self.tare_done:
            if ok:
                self.tare_buf.append(wrench.copy())
            if elapsed >= self.tare_duration and len(self.tare_buf) > 10:
                self.bias = np.mean(np.array(self.tare_buf), axis=0)
                self.tare_done = True
                self.get_logger().info(
                    "[FT] calibração concluída. Viés removido: "
                    f"F=({self.bias[0]:.2f}, {self.bias[1]:.2f}, "
                    f"{self.bias[2]:.2f}) N | "
                    f"‖F_viés‖={np.linalg.norm(self.bias[:3]):.2f} N"
                )
                self.get_logger().info(
                    "[FT] a partir de agora as leituras representam força "
                    "EXTERNA. Pode movimentar o robô."
                )
            self._publish_status(elapsed, calibrating=True)
            return

        wrench = wrench - self.bias

        # ── Filtro passa-baixa ───────────────────────────────────────────────
        # Só atualiza com leitura válida. Substituir uma leitura inválida pelo
        # último valor filtrado e então filtrar produziria um ponto fixo
        # (filtro(x,x) = x), travando a saída para sempre.
        if ok:
            self.wrench_filt = self.alpha * wrench + (1 - self.alpha) * self.wrench_filt

        self._publish(now)
        self._publish_status(elapsed, calibrating=False)

    def _publish(self, now):
        w = self.wrench_filt
        msg = WrenchStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = float(w[0])
        msg.wrench.force.y = float(w[1])
        msg.wrench.force.z = float(w[2])
        msg.wrench.torque.x = float(w[3])
        msg.wrench.torque.y = float(w[4])
        msg.wrench.torque.z = float(w[5])
        self.pub_wrench.publish(msg)
        self.pub_norm.publish(
            Float64MultiArray(
                data=[
                    float(np.linalg.norm(w[:3])),
                    float(w[0]),
                    float(w[1]),
                    float(w[2]),
                ]
            )
        )

    def _publish_status(self, elapsed, calibrating):
        self.pub_status.publish(
            String(
                data=json.dumps(
                    dict(
                        phase="CALIBRANDO" if calibrating else "ATIVO",
                        elapsed=round(elapsed, 2),
                        have_effort=self.have_effort,
                        bias_norm=float(np.linalg.norm(self.bias[:3])),
                        force_norm=float(np.linalg.norm(self.wrench_filt[:3])),
                    )
                )
            )
        )
        if not calibrating and int(elapsed * self.rate) % int(self.rate * 2) == 0:
            w = self.wrench_filt
            self.get_logger().info(
                f"[FT] F=({w[0]:7.2f}, {w[1]:7.2f}, {w[2]:7.2f}) N  "
                f"‖F‖={np.linalg.norm(w[:3]):6.2f} N"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FTEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
