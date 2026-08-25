import json

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kuka_control_lab import dynamics as dyn
from kuka_control_lab import trajectories as traj_mod
from kuka_forward_kinematics.kinematics import (
    forward_kinematics,
    geometric_jacobian,
    JOINT_LIMITS,
    JOINT_NAMES,
)


class AdmittanceCtrl(Node):

    def __init__(self):
        super().__init__("admittance_ctrl")

        p = self.declare_parameter
        p("mode", "hand_guiding")  # hand_guiding | force_control
        p("rate", 50.0)
        p("M_d", [50.0, 50.0, 50.0])
        p("B_d", [1000.0, 1000.0, 1000.0])
        p("K_d", [0.0, 0.0, 0.0])
        p("f_des", 20.0)
        p("force_axis", 2)  # 0=X, 1=Y, 2=Z
        p("deadband", 2.0)
        p("max_deviation", 0.12)
        p("max_joint_step", 0.02)  # rad por ciclo — trava de segurança
        p("damping_lambda", 0.08)
        p("nominal_posture", "pick")
        p("wrench_topic", "/kuka_force_lab/ft_wrist")
        p("command_topic", "/kuka_arm_controller/joint_trajectory")
        p("startup_delay", 3.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = g("mode")
        self.rate = float(g("rate"))
        self.dt = 1.0 / self.rate
        self.M_d = np.array(g("M_d"), float)
        self.B_d = np.array(g("B_d"), float)
        self.K_d = np.array(g("K_d"), float)
        self.deadband = float(g("deadband"))
        self.max_dev = float(g("max_deviation"))
        self.max_step = float(g("max_joint_step"))
        self.lam = float(g("damping_lambda"))
        self.startup_delay = float(g("startup_delay"))

        axis = int(g("force_axis"))
        self.S = np.zeros(3)
        self.F_des = np.zeros(3)
        if self.mode == "force_control":
            self.S[axis] = 1.0
            self.F_des[axis] = float(g("f_des"))

        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.have_state = False
        self.wrench = np.zeros(6)
        self.have_wrench = False

        self.x_a = np.zeros(3)
        self.xd_a = np.zeros(3)
        self.x_nom = None
        self.q_nom = traj_mod.get_posture(g("nominal_posture"))
        self.started = False
        self.t0 = None
        self.n_ticks = 0

        self.create_subscription(JointState, "/joint_states", self._cb_js, 20)
        self.create_subscription(WrenchStamped, g("wrench_topic"), self._cb_w, 20)
        self.pub_cmd = self.create_publisher(JointTrajectory, g("command_topic"), 10)
        self.pub_dev = self.create_publisher(
            Float64MultiArray, "/kuka_force_lab/admittance_deviation", 10
        )
        self.pub_tcp = self.create_publisher(
            PoseStamped, "/kuka_force_lab/tcp_pose", 10
        )
        self.pub_status = self.create_publisher(
            String, "/kuka_force_lab/admittance_status", 10
        )

        self.create_timer(self.dt, self._loop)

        self.get_logger().info(
            f"[Admitância] modo={self.mode} | M_d={self.M_d.tolist()} kg | "
            f"B_d={self.B_d.tolist()} N·s/m | K_d={self.K_d.tolist()} N/m"
        )
        if self.mode == "force_control":
            self.get_logger().info(
                f"[Admitância] regulando {self.F_des[axis]:.1f} N no eixo "
                f'{"XYZ"[axis]} — lembre que F_des é a LEITURA desejada no '
                "sensor (a reação do ambiente sobre o robô)."
            )
        else:
            self.get_logger().info(
                "[Admitância] guiagem manual: empurre o TCP (aplicando força "
                "externa na simulação) e o robô cede."
            )
        self.get_logger().info(
            "[Admitância] aguardando /joint_states e um wrench já CALIBRADO "
            "do ft_estimator..."
        )

    def _cb_js(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return
        self.q = np.array([msg.position[idx[n]] for n in JOINT_NAMES])
        if msg.velocity and len(msg.velocity) >= len(msg.name):
            self.qd = np.array([msg.velocity[idx[n]] for n in JOINT_NAMES])
        self.have_state = True

    def _cb_w(self, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])
        self.have_wrench = True

    # ── Laço de admitância ───────────────────────────────────────────────────
    def _loop(self):
        if not (self.have_state and self.have_wrench):
            return
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        elapsed = (now - self.t0).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            return

        if not self.started:
            # Congela a pose atual como referência nominal. A admitância opera
            # como DESVIO em relação a ela.
            T, _ = forward_kinematics(self.q)
            self.x_nom = T[:3, 3].copy()
            self.q_nom = self.q.copy()
            self.started = True
            self.get_logger().info(
                f"[Admitância] ativo. Pose nominal congelada em "
                f"({self.x_nom[0]:.3f}, {self.x_nom[1]:.3f}, "
                f"{self.x_nom[2]:.3f}) m."
            )

        F = self.wrench[:3].copy()
        for i in range(3):
            if self.S[i] < 0.5 and abs(F[i]) < self.deadband:
                F[i] = 0.0
        F_eff = F - self.F_des * self.S

        # Integração do modelo virtual (Euler semi-implícito).
        xdd = (F_eff - self.B_d * self.xd_a - self.K_d * self.x_a) / self.M_d
        self.xd_a = self.xd_a + xdd * self.dt
        self.x_a = self.x_a + self.xd_a * self.dt

        n = float(np.linalg.norm(self.x_a))
        saturated = n > self.max_dev
        if saturated:
            self.x_a *= self.max_dev / n
            self.xd_a *= 0.5

        # Conversão para juntas pelo Jacobiano AVALIADO NA POSE MEDIDA.
        T, frames = forward_kinematics(self.q)
        x = T[:3, 3]
        J = geometric_jacobian(self.q, frames)[:3, :]
        Jp = dyn.damped_pinv(J, self.lam)

        x_cmd = self.x_nom + self.x_a
        dq = Jp @ (x_cmd - x)

        # Trava de segurança: nenhum passo de junta maior que max_step.
        step = float(np.max(np.abs(dq)))
        if step > self.max_step:
            dq *= self.max_step / step
        q_cmd = np.clip(self.q + dq, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

        self._send(q_cmd)
        self._publish(now, x, x_cmd, F, saturated, step)
        self.n_ticks += 1

    def _send(self, q_cmd):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_cmd]
        pt.velocities = [0.0] * 6
        # Horizonte curto (2 ciclos): comando em streaming. Horizonte longo
        # faria o JTC interpolar devagar e o laço de admitância "brigaria" com
        # a interpolação.
        h = 2.0 * self.dt
        pt.time_from_start = Duration(sec=int(h), nanosec=int((h % 1.0) * 1e9))
        traj.points = [pt]
        self.pub_cmd.publish(traj)

    def _publish(self, now, x, x_cmd, F, saturated, step):
        self.pub_dev.publish(
            Float64MultiArray(
                data=[
                    float(self.x_a[0]),
                    float(self.x_a[1]),
                    float(self.x_a[2]),
                    float(np.linalg.norm(F)),
                    float(F[0]),
                    float(F[1]),
                    float(F[2]),
                ]
            )
        )

        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = "base_link"
        ps.pose.position.x = float(x[0])
        ps.pose.position.y = float(x[1])
        ps.pose.position.z = float(x[2])
        self.pub_tcp.publish(ps)

        self.pub_status.publish(
            String(
                data=json.dumps(
                    dict(
                        mode=self.mode,
                        deviation=[float(v) for v in self.x_a],
                        force=[float(v) for v in F],
                        f_des=[float(v) for v in self.F_des],
                        saturated=bool(saturated),
                        joint_step=float(step),
                    )
                )
            )
        )

        if self.n_ticks % int(self.rate) == 0:
            axis = int(np.argmax(self.S)) if self.S.any() else 2
            extra = ""
            if self.mode == "force_control":
                extra = f" | erro de força = " f"{F[axis] - self.F_des[axis]:+.2f} N"
            self.get_logger().info(
                f"[Admitância] desvio=({self.x_a[0] * 1000:+.1f}, "
                f"{self.x_a[1] * 1000:+.1f}, {self.x_a[2] * 1000:+.1f}) mm | "
                f"F=({F[0]:+.1f}, {F[1]:+.1f}, {F[2]:+.1f}) N{extra}"
                + ("  [DESVIO SATURADO]" if saturated else "")
            )


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Admitância] encerrado.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
