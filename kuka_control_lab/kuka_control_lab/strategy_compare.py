import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import controllers as ctrl_mod
from . import dynamics as dyn
from . import plotting
from . import simulator as sim_mod
from . import trajectories as traj_mod


class StrategyCompare(Node):

    def __init__(self):
        super().__init__("strategy_compare")

        p = self.declare_parameter
        p("wn", 6.0)
        p("zeta", 1.0)
        p("trajectory", "quintic")
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
        p("gravity_comp", True)
        p("model_error", 1.0)
        p("label", "cmp")
        p("output_dir", plotting.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "wn",
                "zeta",
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
                "gravity_comp",
                "model_error",
                "label",
                "output_dir",
            )
        }

        self.pub = self.create_publisher(String, "/kuka_control_lab/comparison", 10)
        self.create_timer(0.5, self._run)
        self._done = False

    def _run(self):
        if self._done:
            return
        self._done = True
        c = self.cfg
        log = self.get_logger()

        if c["trajectory"] == "small_step":
            tj, tc, q0, qf = traj_mod.make_step_around(
                c["start"], delta=float(c["step_delta"])
            )
        else:
            tj, tc, q0, qf = traj_mod.make_pair(
                c["start"],
                c["goal"],
                duration=float(c["move_time"]),
                kind=c["trajectory"],
            )

        log.info(
            f'[Compare] referência {c["trajectory"]} '
            f'{c["start"]}→{c["goal"]} | ωn={c["wn"]} ζ={c["zeta"]} | '
            f'carga={c["payload"]} kg'
        )
        log.info(f"[Compare] q_inicial={[round(v, 3) for v in q0]}")
        log.info(f"[Compare] q_alvo   ={[round(v, 3) for v in qf]}")

        coulomb = dyn.COULOMB_NOMINAL if c["coulomb_friction"] else None
        results = {}
        mets = {}

        for name in ("local_pd", "computed_torque", "operational_space"):
            controller = ctrl_mod.build(
                name,
                wn=float(c["wn"]),
                zeta=float(c["zeta"]),
                gravity_comp=bool(c["gravity_comp"]),
                model_error=float(c["model_error"]),
            )
            log.info(f"[Compare] rodando {name} ...")
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
                seed=0,
            )
            results[name] = res
            mets[name] = res.metrics()
            m = mets[name]
            log.info(
                f'[Compare]   RMS TCP={m["rms_cart_mm"]:.3f} mm | '
                f'regime={m["final_cart_mm"]:.4f} mm | '
                f'RMS junta={m["rms_joint_rad"]:.5f} rad | '
                f'estável={"NÃO" if m["unstable"] else "sim"}'
            )

        table = plotting.format_table(
            mets,
            title=f'Comparação de estratégias — {c["label"]} '
            f'(ωn={c["wn"]}, ζ={c["zeta"]}, carga={c["payload"]}kg)',
        )
        for line in table.split("\n"):
            log.info(line)

        outdir = plotting.ensure_dir(c["output_dir"])
        with open(os.path.join(outdir, f'cmp_table_{c["label"]}.txt'), "w") as fh:
            fh.write(table + "\n")

        files = plotting.plot_comparison(results, label=c["label"], outdir=outdir)
        for f in files:
            log.info(f"[Compare] gráfico salvo: {f}")

        payload = dict(config=c, metrics=mets)
        with open(os.path.join(outdir, f'cmp_metrics_{c["label"]}.json'), "w") as fh:
            json.dump(payload, fh, indent=2, default=float)

        msg = String()
        msg.data = json.dumps(payload, default=float)
        self.pub.publish(msg)
        log.info("[Compare] concluído — Ctrl+C para sair.")


def main(args=None):
    rclpy.init(args=args)
    node = StrategyCompare()
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
