import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import controllers as ctrl_mod
from . import plotting
from . import simulator as sim_mod
from . import trajectories as traj_mod
from . import dynamics as dyn


class StrategySim(Node):

    def __init__(self):
        super().__init__("strategy_sim")

        p = self.declare_parameter
        p("strategy", "computed_torque")
        p("wn", 6.0)
        p("zeta", 1.0)
        p("kp_scale", 1.0)
        p("kd_scale", 1.0)
        p("gravity_comp", True)
        p("model_error", 1.0)
        p("null_kp", 4.0)
        p("trajectory", "quintic")  # quintic | step | sine | small_step
        p("start", "home")
        p("goal", "pick")
        p("move_time", 3.0)
        p("duration", 6.0)
        p("step_delta", 0.05)
        p("control_rate", 200.0)
        p("physics_rate", 1000.0)
        p("payload", 0.0)
        p("coulomb_friction", False)
        p("encoder_noise", 0.0)
        p("feedback_delay", 0.0)
        p("label", "run")
        p("output_dir", plotting.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "strategy",
                "wn",
                "zeta",
                "kp_scale",
                "kd_scale",
                "gravity_comp",
                "model_error",
                "null_kp",
                "trajectory",
                "start",
                "goal",
                "move_time",
                "duration",
                "step_delta",
                "control_rate",
                "physics_rate",
                "payload",
                "coulomb_friction",
                "encoder_noise",
                "feedback_delay",
                "label",
                "output_dir",
            )
        }

        self.pub = self.create_publisher(String, "/kuka_control_lab/run_report", 10)
        self.create_timer(0.5, self._run_once)
        self._done = False

    def _run_once(self):
        if self._done:
            return
        self._done = True
        c = self.cfg

        if c["trajectory"] == "small_step":
            tj, tc, q0, _ = traj_mod.make_step_around(
                c["start"], delta=float(c["step_delta"])
            )
        else:
            tj, tc, q0, _ = traj_mod.make_pair(
                c["start"],
                c["goal"],
                duration=float(c["move_time"]),
                kind=c["trajectory"],
            )

        controller = ctrl_mod.build(
            c["strategy"],
            wn=float(c["wn"]),
            zeta=float(c["zeta"]),
            kp_scale=float(c["kp_scale"]),
            kd_scale=float(c["kd_scale"]),
            gravity_comp=bool(c["gravity_comp"]),
            model_error=float(c["model_error"]),
            null_kp=float(c["null_kp"]),
        )

        self.get_logger().info(
            f'[StrategySim] estratégia={c["strategy"]} | '
            f"{controller.gains_description()}"
        )
        self.get_logger().info(
            f'[StrategySim] referência={c["trajectory"]} '
            f'{c["start"]}→{c["goal"]} | simulando {c["duration"]}s a '
            f'{c["control_rate"]}Hz de controle...'
        )

        coulomb = dyn.COULOMB_NOMINAL if c["coulomb_friction"] else None
        res = sim_mod.simulate(
            controller,
            tj,
            tc,
            q0=q0,
            duration=float(c["duration"]),
            control_rate=float(c["control_rate"]),
            physics_rate=float(c["physics_rate"]),
            coulomb=coulomb,
            payload=float(c["payload"]),
            encoder_noise=float(c["encoder_noise"]),
            feedback_delay=float(c["feedback_delay"]),
        )

        m = res.metrics()
        self._report(c, controller, m)

        files = plotting.plot_single_run(res, label=c["label"], outdir=c["output_dir"])
        for f in files:
            self.get_logger().info(f"[StrategySim] gráfico salvo: {f}")

        payload = dict(config=c, gains=controller.gains_description(), metrics=m)
        msg = String()
        msg.data = json.dumps(payload, default=float)
        self.pub.publish(msg)

        out = os.path.join(
            plotting.ensure_dir(c["output_dir"]), f'ctrl_metrics_{c["label"]}.json'
        )
        with open(out, "w") as fh:
            json.dump(payload, fh, indent=2, default=float)
        self.get_logger().info(f"[StrategySim] métricas salvas: {out}")
        self.get_logger().info("[StrategySim] concluído — Ctrl+C para sair.")

    def _report(self, c, controller, m):
        log = self.get_logger()
        log.info("─" * 62)
        log.info(f'  RESULTADO — {ctrl_mod.STRATEGY_LABELS[c["strategy"]]}')
        log.info(f"  ganhos           : {controller.gains_description()}")
        log.info(f'  RMS erro TCP     : {m["rms_cart_mm"]:.3f} mm')
        log.info(f'  pico erro TCP    : {m["max_cart_mm"]:.3f} mm')
        log.info(f'  regime erro TCP  : {m["final_cart_mm"]:.4f} mm')
        log.info(f'  RMS erro juntas  : {m["rms_joint_rad"]:.5f} rad')
        log.info(f'  regime juntas    : {m["final_joint_rad"]:.6f} rad')
        st = (
            f'{m["settling_time_cart_s"]:.2f} s'
            if m["settling_time_cart_s"] >= 0
            else "não assentou"
        )
        log.info(f"  assentamento TCP : {st}")
        log.info(f'  sobressinal      : {m["overshoot_pct"]:.1f} %')
        log.info(
            f'  torque RMS/pico  : {m["tau_rms_Nm"]:.1f} / '
            f'{m["tau_peak_Nm"]:.1f} N·m'
        )
        log.info(
            f'  ondulação final  : {m["ripple_rad"]:.6f} rad / '
            f'{m["ripple_mm"]:.4f} mm  (decaimento {m["decay_ratio"]:.2f})'
        )
        log.info(f'  ESTÁVEL          : {"NÃO" if m["unstable"] else "sim"}')
        log.info("─" * 62)


def main(args=None):
    rclpy.init(args=args)
    node = StrategySim()
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
