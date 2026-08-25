import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_control_lab import simulator as sim_mod

from . import experiment as exp
from . import plotting


class ForceSim(Node):

    def __init__(self):
        super().__init__("force_sim")
        p = self.declare_parameter
        p("controller", "hybrid")
        p("material", "madeira")
        p("stiffness", 0.0)
        p("surface", "plane")
        p("z_surface", exp.Z_SURFACE_DEFAULT)
        p("overshoot", exp.OVERSHOOT_DEFAULT)
        p("approach_time", exp.APPROACH_TIME_DEFAULT)
        p("lateral", 0.0)
        p("lateral_time", 0.0)
        p("f_des", 20.0)
        p("duration", 8.0)
        p("control_rate", 200.0)
        p("physics_rate", 1000.0)
        p("force_noise", 0.0)
        p("M_d", 0.0)
        p("B_d", 0.0)
        p("K_d", 0.0)
        p("Kf", 0.0)
        p("Kfi", -1.0)  # -1 = padrão; 0.0 para desligar o integral de fato
        p("servo_wn", 0.0)
        p("wn", 0.0)
        p("label", "run")
        p("output_dir", plotting.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "controller",
                "material",
                "stiffness",
                "surface",
                "z_surface",
                "overshoot",
                "approach_time",
                "lateral",
                "lateral_time",
                "f_des",
                "duration",
                "control_rate",
                "physics_rate",
                "force_noise",
                "M_d",
                "B_d",
                "K_d",
                "Kf",
                "Kfi",
                "servo_wn",
                "wn",
                "label",
                "output_dir",
            )
        }

        self.pub = self.create_publisher(String, "/kuka_force_lab/run", 10)
        self.create_timer(0.5, self._run)
        self._done = False

    def _run(self):
        if self._done:
            return
        self._done = True
        c = self.cfg
        log = self.get_logger()

        ref_j, ref_c, q0, env, info = exp.build_scene(
            z_surface=float(c["z_surface"]),
            overshoot=float(c["overshoot"]),
            approach_time=float(c["approach_time"]),
            material=c["material"],
            surface=c["surface"],
            stiffness=float(c["stiffness"]) or None,
            lateral=float(c["lateral"]),
            lateral_time=float(c["lateral_time"]) or None,
        )

        kw = dict(control_rate=float(c["control_rate"]))
        if float(c["M_d"]) > 0:
            kw["M_d"] = (float(c["M_d"]),) * 3
        if float(c["B_d"]) > 0:
            kw["B_d"] = (float(c["B_d"]),) * 3
        if float(c["K_d"]) > 0:
            kw["K_d"] = (float(c["K_d"]),) * 3
        if float(c["Kf"]) > 0:
            kw["Kf"] = float(c["Kf"])
        if float(c["Kfi"]) >= 0:
            kw["Kfi"] = float(c["Kfi"])
        if float(c["servo_wn"]) > 0:
            kw["servo_wn"] = float(c["servo_wn"])
        if float(c["wn"]) > 0:
            kw["wn"] = float(c["wn"])

        ctrl = exp.build_controller(c["controller"], f_des=float(c["f_des"]), **kw)
        log.info(f'[ForceSim] {plotting.LABELS.get(c["controller"], c["controller"])}')
        log.info(f"[ForceSim] sintonia: {ctrl.gains_description()}")
        log.info(
            f'[ForceSim] superfície {info["material"]} '
            f'(K_env = {info["stiffness"]:.3g} N/m) em Z='
            f'{info["z_surface"]:.3f} m; comando '
            f'{info["overshoot_mm"]:.0f} mm abaixo dela.'
        )

        res = sim_mod.simulate(
            ctrl,
            ref_j,
            ref_c,
            q0=q0,
            duration=float(c["duration"]),
            control_rate=float(c["control_rate"]),
            physics_rate=float(c["physics_rate"]),
            environment=env,
            force_noise=float(c["force_noise"]),
        )

        m = exp.force_metrics(
            res, f_des=float(c["f_des"]), z_surface=float(c["z_surface"])
        )
        log.info("─" * 62)
        log.info(f'  pico de força      : {m["peak_N"]:.2f} N')
        log.info(f'  força em regime    : {m["steady_N"]:.2f} N')
        log.info(
            f'  erro de regime     : {m["error_N"]:+.2f} N '
            f'(alvo {c["f_des"]:.1f} N)'
        )
        log.info(f'  ondulação          : {m["ripple_N"]:.4f} N')
        log.info(f'  penetração média   : {m["penetration_mm"]:.3f} mm')
        log.info(f'  penetração máxima  : {m["max_penetration_mm"]:.3f} mm')
        log.info(f'  torque de pico     : {m["tau_peak_Nm"]:.0f} N·m')
        log.info(f'  tempo em contato   : {m["contact_fraction"] * 100:.0f} %')
        if m["ripple_N"] > 1.0:
            log.warn(
                "  ONDULAÇÃO ALTA — instabilidade de contato. Reduza o "
                "ganho de força, aumente B_d, ou use um material menos "
                "rígido."
            )
        log.info("─" * 62)

        outdir = plotting.ensure_dir(c["output_dir"])
        files = plotting.plot_force_run(
            res,
            f_des=float(c["f_des"]),
            z_surface=float(c["z_surface"]),
            label=c["label"],
            outdir=outdir,
        )
        for f in files:
            log.info(f"[ForceSim] gráfico salvo: {f}")

        payload = dict(config=c, scene=info, gains=ctrl.gains_description(), metrics=m)
        with open(os.path.join(outdir, f'force_metrics_{c["label"]}.json'), "w") as fh:
            json.dump(payload, fh, indent=2, default=float)
        self.pub.publish(String(data=json.dumps(payload, default=float)))
        log.info("[ForceSim] concluído — Ctrl+C para sair.")


def main(args=None):
    rclpy.init(args=args)
    node = ForceSim()
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
