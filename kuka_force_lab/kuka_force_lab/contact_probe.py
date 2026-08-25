import json
import os

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kuka_control_lab import dynamics as dyn
from kuka_control_lab import plotting as base_plot
from kuka_forward_kinematics.kinematics import (
    forward_kinematics,
    geometric_jacobian,
    JOINT_LIMITS,
    JOINT_NAMES,
)

from . import plotting


class ContactProbe(Node):

    def __init__(self):
        super().__init__("contact_probe")

        p = self.declare_parameter
        p("panel_x", 1.50)
        p("panel_y", 0.0)
        p("z_surface", 0.95)
        p("approach_height", 0.10)
        p("overshoot", 0.02)
        p("descend_time", 6.0)
        p("hold_time", 8.0)
        p("n_waypoints", 30)
        p("start_posture", "pick")
        p("action_name", "/kuka_arm_controller/follow_joint_trajectory")
        p("publish_command", True)
        p("label", "contato")
        p("output_dir", base_plot.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "panel_x",
                "panel_y",
                "z_surface",
                "approach_height",
                "overshoot",
                "descend_time",
                "hold_time",
                "n_waypoints",
                "start_posture",
                "action_name",
                "publish_command",
                "label",
                "output_dir",
            )
        }

        self.q = np.zeros(6)
        self.have_state = False
        self.wrench = np.zeros(6)
        self.recording = False
        self.t0 = None
        self.log = {"t": [], "f": [], "x": [], "q": []}
        self.phase = "INIT"
        self.finished = False

        self.create_subscription(JointState, "/joint_states", self._cb_js, 20)
        self.create_subscription(
            WrenchStamped, "/kuka_force_lab/ft_wrist", self._cb_w, 20
        )
        self.client = ActionClient(self, FollowJointTrajectory, self.cfg["action_name"])
        self.create_timer(0.02, self._sample)
        self.create_timer(2.0, self._start)
        self._sent = False

    def _cb_js(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return
        self.q = np.array([msg.position[idx[n]] for n in JOINT_NAMES])
        self.have_state = True

    def _cb_w(self, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.wrench = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def _sample(self):
        if not (self.recording and self.have_state):
            return
        T, _ = forward_kinematics(self.q)
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.log["t"].append(t)
        self.log["f"].append(self.wrench.copy())
        self.log["x"].append(T[:3, 3].copy())
        self.log["q"].append(self.q.copy())

    #  Sequência  dos passos
    def _start(self):
        if self._sent or not self.have_state:
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                "[Probe] action server ausente — o Gazebo está rodando?"
            )
            return
        self._sent = True
        c = self.cfg

        x_app = np.array(
            [c["panel_x"], c["panel_y"], c["z_surface"] + c["approach_height"]]
        )
        x_tgt = np.array([c["panel_x"], c["panel_y"], c["z_surface"] - c["overshoot"]])

        self.q_app = self._ik(x_app, self.q)
        self.q_tgt = self._ik(x_tgt, self.q_app)
        self.x_app, self.x_tgt = x_app, x_tgt

        self.get_logger().info(
            f"[Probe] aproximação em ({x_app[0]:.3f}, {x_app[1]:.3f}, "
            f'{x_app[2]:.3f}) m, {c["approach_height"] * 100:.0f} cm acima do '
            "painel."
        )
        self.get_logger().info(
            f"[Probe] alvo em ({x_tgt[0]:.3f}, {x_tgt[1]:.3f}, "
            f'{x_tgt[2]:.3f}) m — {c["overshoot"] * 1000:.0f} mm ABAIXO da '
            "superfície. É a incerteza de posição do experimento."
        )

        self.phase = "APPROACH"
        self._send_point(self.q_app, 6.0, self._on_approached)

    def _on_approached(self, _f):
        self.get_logger().info(
            "[Probe] em posição de aproximação. Iniciando a descida — "
            "começando a registrar."
        )
        self.t0 = self.get_clock().now()
        self.recording = True
        self.phase = "DESCEND"
        self._send_path(
            self.q_app, self.q_tgt, float(self.cfg["descend_time"]), self._on_contact
        )

    def _on_contact(self, _f):
        self.phase = "HOLD"
        self.get_logger().info(
            f"[Probe] alvo alcançado. Mantendo por "
            f'{self.cfg["hold_time"]:.0f} s — é AQUI que se lê a força de '
            "regime."
        )
        self.create_timer(float(self.cfg["hold_time"]), self._on_held)

    def _on_held(self):
        if self.phase != "HOLD":
            return
        self.phase = "RETRACT"
        self.get_logger().info("[Probe] recolhendo.")
        self._send_point(self.q_app, 6.0, self._on_done)

    def _on_done(self, _f):
        if self.finished:
            return
        self.finished = True
        self.recording = False
        self._report()

    # ── Envio de trajetória ──────────────────────────────────────────────────
    def _send_point(self, q, secs, cb):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(secs))
        traj.points = [pt]
        self._dispatch(traj, cb)

    def _send_path(self, q_a, q_b, secs, cb):
        n = int(self.cfg["n_waypoints"])
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        # Subdivide a descida em waypoints CARTESIANOS: interpolar ângulos de
        # junta em linha reta não produz uma reta no espaço real, e um desvio
        # lateral durante a descida raspa a borda do painel.
        for i in range(1, n + 1):
            s = i / n
            p = 10 * s**3 - 15 * s**4 + 6 * s**5
            x_i = self.x_app + p * (self.x_tgt - self.x_app)
            q_seed = q_a if i == 1 else np.array(traj.points[-1].positions)
            q_i = self._ik(x_i, q_seed)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q_i]
            pt.velocities = [0.0] * 6
            t = secs * s
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(pt)
        del q_b
        self._dispatch(traj, cb)

    def _dispatch(self, traj, cb):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        def _accepted(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("[Probe] goal rejeitado.")
                return
            gh.get_result_async().add_done_callback(cb)

        self.client.send_goal_async(goal).add_done_callback(_accepted)

    def _ik(self, x_target, q_seed, iters=120, tol=1e-5):
        """IK de posição por mínimos quadrados amortecidos sobre a FK do
        workspace. Só posição (3 GdL); a orientação acomoda-se livremente, o
        que é aceitável para uma sonda pontual encostando num plano."""
        q = np.array(q_seed, float).copy()
        for _ in range(iters):
            T, frames = forward_kinematics(q)
            e = np.asarray(x_target, float) - T[:3, 3]
            if np.linalg.norm(e) < tol:
                break
            J = geometric_jacobian(q, frames)[:3, :]
            q = q + dyn.damped_pinv(J, 0.05) @ e
            q = np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        err = np.linalg.norm(
            np.asarray(x_target, float) - forward_kinematics(q)[0][:3, 3]
        )
        if err > 1e-3:
            self.get_logger().warn(
                f"[Probe] IK convergiu com {err * 1000:.1f} mm de resíduo — "
                "o ponto pode estar fora do alcance útil."
            )
        return q

    # ── Relatório ────────────────────────────────────────────────────────────
    def _report(self):
        if len(self.log["t"]) < 20:
            self.get_logger().error(
                "[Probe] poucos dados registrados — o ft_estimator está "
                "publicando em /kuka_force_lab/ft_wrist?"
            )
            return

        c = self.cfg
        t = np.array(self.log["t"])
        f = np.array(self.log["f"])
        x = np.array(self.log["x"])

        # A janela de regime é o trecho final da fase de manutenção, antes do
        # recolhimento.
        t_hold_end = float(c["descend_time"]) + float(c["hold_time"])
        mask = (t > t_hold_end - 0.5 * float(c["hold_time"])) & (t <= t_hold_end)
        if not np.any(mask):
            mask = t > 0.7 * t.max()
        fz = f[:, 2]
        steady = float(np.mean(fz[mask]))
        ripple = float(np.std(fz[mask]))
        pen = np.maximum(float(c["z_surface"]) - x[:, 2], 0.0)

        log = self.get_logger()
        log.info("─" * 66)
        log.info(f'  SONDA DE CONTATO — {c["label"]}')
        log.info(
            f'  comando: {c["overshoot"] * 1000:.0f} mm abaixo da '
            f'superfície (Z = {c["z_surface"]:.3f} m)'
        )
        log.info(f"  pico de |F_z|      : {np.max(np.abs(fz)):.2f} N")
        log.info(
            f"  F_z em regime      : {steady:.2f} N  " f"(ondulação {ripple:.3f} N)"
        )
        log.info(f"  penetração média   : {np.mean(pen[mask]) * 1000:.2f} mm")
        log.info(f"  Z final do TCP     : {x[mask, 2].mean():.4f} m")
        log.info(f"  amostras           : {len(t)}")
        log.info("─" * 66)

        outdir = base_plot.ensure_dir(c["output_dir"])
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axes[0].plot(
            t, fz, color=plotting.COLORS["admittance"], lw=1.5, label="F_z estimada"
        )
        axes[0].axhline(0, color="0.7", lw=0.8)
        axes[0].set_ylabel("força normal [N]")
        axes[0].grid(alpha=0.3)
        axes[0].legend(fontsize=8)

        axes[1].plot(
            t, x[:, 2], color=plotting.COLORS["admittance"], lw=1.5, label="Z do TCP"
        )
        axes[1].axhline(
            float(c["z_surface"]),
            color=plotting.COLORS["surface"],
            lw=2,
            label="superfície",
        )
        axes[1].axhline(
            float(c["z_surface"]) - float(c["overshoot"]),
            ls="--",
            color=plotting.COLORS["reference"],
            label="Z comandado",
        )
        axes[1].set_ylabel("altura [m]")
        axes[1].grid(alpha=0.3)
        axes[1].legend(fontsize=8)

        axes[2].plot(t, np.linalg.norm(f[:, :3], axis=1), color="0.3", lw=1.3)
        axes[2].set_ylabel("‖F‖ [N]")
        axes[2].set_xlabel("t [s]")
        axes[2].grid(alpha=0.3)

        for ax in axes:
            ax.axvspan(float(c["descend_time"]), t_hold_end, color="0.9", zorder=0)
        fig.suptitle(
            f'Contato no Gazebo — {c["label"]}\n'
            "(faixa cinza = fase de manutenção, onde se lê o regime)"
        )
        fig.tight_layout()
        path = os.path.join(outdir, f'probe_{c["label"]}.png')
        fig.savefig(path, dpi=120)
        plt.close(fig)

        with open(os.path.join(outdir, f'probe_{c["label"]}.json'), "w") as fh:
            json.dump(
                dict(
                    config=c,
                    peak_N=float(np.max(np.abs(fz))),
                    steady_N=steady,
                    ripple_N=ripple,
                    penetration_mm=float(np.mean(pen[mask]) * 1000.0),
                    n_samples=int(len(t)),
                ),
                fh,
                indent=2,
                default=float,
            )

        log.info(f"[Probe] gráfico salvo: {path}")
        log.info("[Probe] concluído — Ctrl+C para sair.")


def main(args=None):
    rclpy.init(args=args)
    node = ContactProbe()
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
