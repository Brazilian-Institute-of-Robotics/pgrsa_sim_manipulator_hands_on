import json
import os

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kuka_forward_kinematics.kinematics import forward_kinematics

from . import plotting
from . import trajectories as traj_mod

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class JtcProbe(Node):

    def __init__(self):
        super().__init__("jtc_probe")

        p = self.declare_parameter
        p("start", "home")
        p("goal", "pick")
        p("move_time", 3.0)
        p("n_waypoints", 40)
        p("settle_time", 2.0)
        p("action_name", "/kuka_arm_controller/follow_joint_trajectory")
        p("label", "jtc")
        p("output_dir", plotting.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "start",
                "goal",
                "move_time",
                "n_waypoints",
                "settle_time",
                "action_name",
                "label",
                "output_dir",
            )
        }

        self.q0 = traj_mod.get_posture(self.cfg["start"])
        self.qf = traj_mod.get_posture(self.cfg["goal"])
        self.ref = traj_mod.JointTrajectory(
            self.q0, self.qf, duration=float(self.cfg["move_time"]), kind="quintic"
        )

        self.t_log, self.q_log = [], []
        self.recording = False
        self.t0 = None
        self.finished = False

        self.create_subscription(JointState, "/joint_states", self._cb_js, 50)
        self.client = ActionClient(self, FollowJointTrajectory, self.cfg["action_name"])
        self.create_timer(1.0, self._start)
        self._sent = False

    # colocar o manipulador na posição inicial
    def _start(self):
        if self._sent:
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                "[JtcProbe] action server não encontrado ainda — "
                "o Gazebo está rodando? tentando de novo..."
            )
            return
        self._sent = True
        self.get_logger().info("[JtcProbe] indo para a postura inicial...")
        self._send(self.q0, 4.0, self._on_homed)

    def _on_homed(self, _future):
        self.get_logger().info(
            "[JtcProbe] postura inicial atingida — iniciando o movimento medido."
        )
        self.t_log.clear()
        self.q_log.clear()
        self.recording = True
        self.t0 = self.get_clock().now()
        self._send_profile()

    # ── Fase 2: enviar o MESMO perfil quíntico usado nas simulações ──────────
    def _send_profile(self):
        n = int(self.cfg["n_waypoints"])
        T = float(self.cfg["move_time"])
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        for i in range(1, n + 1):
            t = T * i / n
            q, qd, _ = self.ref(t)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [float(v) for v in qd]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("[JtcProbe] goal rejeitado pelo controlador.")
            return
        gh.get_result_async().add_done_callback(self._on_done)

    def _on_done(self, _future):
        self.get_logger().info(
            f"[JtcProbe] movimento concluído — coletando mais "
            f'{self.cfg["settle_time"]}s de regime permanente...'
        )
        self.create_timer(float(self.cfg["settle_time"]), self._finish)

    def _cb_js(self, msg: JointState):
        if not self.recording or self.t0 is None:
            return
        idx = {name: i for i, name in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return
        q = np.array([msg.position[idx[n]] for n in JOINT_NAMES])
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.t_log.append(t)
        self.q_log.append(q)

    def _finish(self):
        if self.finished:
            return
        self.finished = True
        self.recording = False

        if len(self.t_log) < 10:
            self.get_logger().error(
                "[JtcProbe] poucos dados de /joint_states — a simulação está "
                "publicando? Verifique com: ros2 topic hz /joint_states"
            )
            return

        t = np.array(self.t_log)
        q = np.array(self.q_log)
        q_des = np.array([self.ref(ti)[0] for ti in t])
        x = np.array([forward_kinematics(qi)[0][:3, 3] for qi in q])
        x_des = np.array([forward_kinematics(qi)[0][:3, 3] for qi in q_des])

        e_j = np.linalg.norm(q_des - q, axis=1)
        e_c = np.linalg.norm(x_des - x, axis=1) * 1000.0
        tail = max(len(t) // 10, 1)
        mets = {
            "rms_joint_rad": float(np.sqrt(np.mean(e_j**2))),
            "max_joint_rad": float(np.max(e_j)),
            "final_joint_rad": float(np.mean(e_j[-tail:])),
            "rms_cart_mm": float(np.sqrt(np.mean(e_c**2))),
            "max_cart_mm": float(np.max(e_c)),
            "final_cart_mm": float(np.mean(e_c[-tail:])),
            "per_joint_max_rad": [float(v) for v in np.max(np.abs(q_des - q), axis=0)],
            "n_samples": int(len(t)),
        }

        log = self.get_logger()
        log.info("─" * 62)
        log.info("  JTC DO GAZEBO — desempenho medido")
        log.info(f'  RMS erro juntas : {mets["rms_joint_rad"]:.5f} rad')
        log.info(f'  pico erro juntas: {mets["max_joint_rad"]:.5f} rad')
        log.info(f'  RMS erro TCP    : {mets["rms_cart_mm"]:.3f} mm')
        log.info(f'  pico erro TCP   : {mets["max_cart_mm"]:.3f} mm')
        log.info(f'  regime TCP      : {mets["final_cart_mm"]:.4f} mm')
        log.info(
            "  pico por junta [rad]: "
            + ", ".join(
                f"j{i+1}={v:.4f}" for i, v in enumerate(mets["per_joint_max_rad"])
            )
        )
        log.info("─" * 62)

        outdir = plotting.ensure_dir(self.cfg["output_dir"])
        label = self.cfg["label"]

        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
        for j, ax in enumerate(axes.ravel()):
            ax.plot(
                t,
                np.degrees(q_des[:, j]),
                "--",
                color=plotting.COLORS["reference"],
                label="desejado",
            )
            ax.plot(t, np.degrees(q[:, j]), color="#9467bd", label="Gazebo")
            ax.set_ylabel(f"joint_{j+1} [°]")
            ax.grid(alpha=0.3)
            if j == 0:
                ax.legend(fontsize=8)
        axes[-1, 0].set_xlabel("t [s]")
        axes[-1, 1].set_xlabel("t [s]")
        fig.suptitle(f"JTC do Gazebo — rastreamento por junta ({label})")
        fig.tight_layout()
        f1 = os.path.join(outdir, f"jtc_joints_{label}.png")
        fig.savefig(f1, dpi=120)
        plt.close(fig)

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        axes[0].plot(t, e_j, color="#9467bd")
        axes[0].set_ylabel("‖erro juntas‖ [rad]")
        axes[0].grid(alpha=0.3)
        axes[1].plot(t, e_c, color="#9467bd")
        axes[1].set_ylabel("‖erro TCP‖ [mm]")
        axes[1].set_xlabel("t [s]")
        axes[1].grid(alpha=0.3)
        fig.suptitle(f"JTC do Gazebo — erro de rastreamento ({label})")
        fig.tight_layout()
        f2 = os.path.join(outdir, f"jtc_error_{label}.png")
        fig.savefig(f2, dpi=120)
        plt.close(fig)

        with open(os.path.join(outdir, f"jtc_metrics_{label}.json"), "w") as fh:
            json.dump(
                {"config": self.cfg, "metrics": mets}, fh, indent=2, default=float
            )

        log.info(f"[JtcProbe] gráfico salvo: {f1}")
        log.info(f"[JtcProbe] gráfico salvo: {f2}")
        log.info("[JtcProbe] concluído — Ctrl+C para sair.")

    def _send(self, q, secs, cb):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(secs))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        def _accepted(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error("[JtcProbe] goal inicial rejeitado.")
                return
            gh.get_result_async().add_done_callback(cb)

        self.client.send_goal_async(goal).add_done_callback(_accepted)


def main(args=None):
    rclpy.init(args=args)
    node = JtcProbe()
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
