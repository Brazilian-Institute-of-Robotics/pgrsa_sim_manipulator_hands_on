import csv
import json
import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_control_lab import simulator as sim_mod

from . import experiment as exp
from . import plotting

FIELDS = [
    "controller",
    "stiffness",
    "kf",
    "peak_N",
    "steady_N",
    "error_N",
    "ripple_N",
    "penetration_mm",
    "max_penetration_mm",
    "tau_peak_Nm",
    "contact_fraction",
    "unstable",
]


class StiffnessSweep(Node):

    def __init__(self):
        super().__init__("stiffness_sweep")
        p = self.declare_parameter
        p("controllers", ["position_only", "impedance", "admittance", "hybrid"])
        p("stiffness_values", [1.0e3, 5.0e3, 2.0e4, 1.0e5, 5.0e5, 2.0e6])
        p("sweep_kf", [0.0])  # 0.0 = usa só o Kf padrão
        p("f_des", 20.0)
        p("overshoot", exp.OVERSHOOT_DEFAULT)
        p("z_surface", exp.Z_SURFACE_DEFAULT)
        p("approach_time", exp.APPROACH_TIME_DEFAULT)
        p("duration", 7.0)
        p("control_rate", 200.0)
        p("physics_rate", 2000.0)
        p("ripple_tol_N", 1.0)
        p("output_csv", os.path.expanduser("~/kuka_control_plots/stiffness_sweep.csv"))
        p("label", "rigidez")
        p("output_dir", plotting.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "controllers",
                "stiffness_values",
                "sweep_kf",
                "f_des",
                "overshoot",
                "z_surface",
                "approach_time",
                "duration",
                "control_rate",
                "physics_rate",
                "ripple_tol_N",
                "output_csv",
                "label",
                "output_dir",
            )
        }

        self.pub = self.create_publisher(String, "/kuka_force_lab/stiffness_sweep", 10)
        self.rows = []
        self.create_timer(1.0, self._run)
        self._done = False

    def _run(self):
        if self._done:
            return
        self._done = True
        c = self.cfg
        log = self.get_logger()

        Ks = [float(v) for v in c["stiffness_values"]]
        kfs = [float(v) for v in c["sweep_kf"] if float(v) > 0.0] or [None]
        total = len(Ks) * len(kfs) * len(c["controllers"])
        log.info(
            f"[StiffnessSweep] {total} simulações "
            f'({len(c["controllers"])} leis × {len(Ks)} rigidezes'
            + (f" × {len(kfs)} ganhos de força" if kfs != [None] else "")
            + ")"
        )
        log.info(
            "[StiffnessSweep] a física é integrada a "
            f'{c["physics_rate"]:.0f} Hz — ambientes muito rígidos '
            "exigem passo pequeno, senão o contato vira artefato "
            "numérico em vez de fenômeno físico."
        )

        k = 0
        data = {}
        for kind in c["controllers"]:
            for K in Ks:
                for kf in kfs:
                    k += 1
                    row = self._one(kind, K, kf)
                    self.rows.append(row)
                    data.setdefault(kind, []).append(
                        (K, row["steady_N"], row["ripple_N"], row["peak_N"])
                    )
                    flag = "INSTÁVEL" if row["unstable"] else "estável "
                    log.info(
                        f"[{k:3d}/{total}] {kind:15s} K={K:9.3g} N/m  {flag}  "
                        f'regime={row["steady_N"]:8.2f} N  '
                        f'ondul={row["ripple_N"]:8.3f} N  '
                        f'pico={row["peak_N"]:9.1f} N'
                    )

        self._save(data)

    def _one(self, kind, K, kf):
        c = self.cfg
        ref_j, ref_c, q0, env, _ = exp.build_scene(
            z_surface=float(c["z_surface"]),
            overshoot=float(c["overshoot"]),
            approach_time=float(c["approach_time"]),
            stiffness=K,
        )
        kw = dict(control_rate=float(c["control_rate"]))
        if kf is not None:
            kw["Kf"] = kf
        ctrl = exp.build_controller(kind, f_des=float(c["f_des"]), **kw)
        res = sim_mod.simulate(
            ctrl,
            ref_j,
            ref_c,
            q0=q0,
            duration=float(c["duration"]),
            control_rate=float(c["control_rate"]),
            physics_rate=float(c["physics_rate"]),
            environment=env,
        )
        m = exp.force_metrics(
            res, f_des=float(c["f_des"]), z_surface=float(c["z_surface"])
        )
        row = dict(controller=kind, stiffness=K, kf=(kf if kf is not None else ""), **m)
        row["unstable"] = bool(
            row["ripple_N"] > float(c["ripple_tol_N"])
            or not np.isfinite(row["steady_N"])
        )
        return row

    def _save(self, data):
        c = self.cfg
        log = self.get_logger()
        path = os.path.expanduser(c["output_csv"])
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            for r in self.rows:
                w.writerow({k: r.get(k, "") for k in FIELDS})
        log.info(f"[StiffnessSweep] CSV salvo: {path}")

        files = plotting.plot_stiffness_sweep(
            data, label=c["label"], outdir=c["output_dir"]
        )
        for f in files:
            log.info(f"[StiffnessSweep] gráfico salvo: {f}")

        log.info("─" * 70)
        log.info("  LIMITE DE ESTABILIDADE POR LEI DE INTERAÇÃO")
        for kind in c["controllers"]:
            sub = [r for r in self.rows if r["controller"] == kind]
            stable = [r["stiffness"] for r in sub if not r["unstable"]]
            if stable:
                log.info(
                    f"  {plotting.LABELS.get(kind, kind):<40} "
                    f"estável até K_env = {max(stable):.3g} N/m"
                )
            else:
                log.info(
                    f"  {plotting.LABELS.get(kind, kind):<40} "
                    "instável em toda a faixa testada"
                )
        log.info(
            "  Acima desse limite, a saída são as duas alternativas "
            "clássicas: reduzir o ganho de força"
        )
        log.info(
            "  (perdendo banda de resposta) ou colocar complacência "
            "física — uma mola real entre a"
        )
        log.info("  ferramenta e a peça, que baixa o K_env visto pela malha.")
        log.info("─" * 70)

        self.pub.publish(
            String(data=json.dumps(dict(config=c, rows=self.rows), default=str))
        )
        log.info("[StiffnessSweep] concluído — Ctrl+C para sair.")


def main(args=None):
    rclpy.init(args=args)
    node = StiffnessSweep()
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
