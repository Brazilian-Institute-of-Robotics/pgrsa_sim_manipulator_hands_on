import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_control_lab import simulator as sim_mod

from . import experiment as exp
from . import plotting


class ForceCompare(Node):

    def __init__(self):
        super().__init__("force_compare")
        p = self.declare_parameter
        p("controllers", ["position_only", "impedance", "admittance", "hybrid"])
        p("material", "madeira")
        p("stiffness", 0.0)  # 0 = usa o preset do material
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
        p("label", "cmp")
        p("output_dir", plotting.DEFAULT_DIR)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {
            n: g(n)
            for n in (
                "controllers",
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
                "label",
                "output_dir",
            )
        }

        self.pub = self.create_publisher(String, "/kuka_force_lab/comparison", 10)
        self.create_timer(0.5, self._run)
        self._done = False

    def _run(self):
        if self._done:
            return
        self._done = True
        c = self.cfg
        log = self.get_logger()

        stiff = float(c["stiffness"]) or None
        ref_j, ref_c, q0, env, info = exp.build_scene(
            z_surface=float(c["z_surface"]),
            overshoot=float(c["overshoot"]),
            approach_time=float(c["approach_time"]),
            material=c["material"],
            surface=c["surface"],
            stiffness=stiff,
            lateral=float(c["lateral"]),
            lateral_time=float(c["lateral_time"]) or None,
        )

        log.info(
            f'[ForceCompare] superfície em Z={info["z_surface"]:.3f} m, '
            f'material {info["material"]} '
            f'(K_env = {info["stiffness"]:.3g} N/m)'
        )
        log.info(
            f"[ForceCompare] TCP parte de "
            f'{[round(v, 3) for v in info["x_start"]]} e é comandado até '
            f'{[round(v, 3) for v in info["x_goal"]]}'
        )
        log.info(
            f'[ForceCompare] ou seja, {info["overshoot_mm"]:.0f} mm '
            "ABAIXO da superfície — a incerteza de posição do experimento."
        )
        log.info(f'[ForceCompare] força desejada: {c["f_des"]:.1f} N')

        results, rows = {}, {}
        for kind in c["controllers"]:
            _, _, _, env_k, _ = exp.build_scene(
                z_surface=float(c["z_surface"]),
                overshoot=float(c["overshoot"]),
                approach_time=float(c["approach_time"]),
                material=c["material"],
                surface=c["surface"],
                stiffness=stiff,
                lateral=float(c["lateral"]),
                lateral_time=float(c["lateral_time"]) or None,
            )
            ctrl = exp.build_controller(
                kind, f_des=float(c["f_des"]), control_rate=float(c["control_rate"])
            )
            log.info(f"[ForceCompare] rodando {kind}: " f"{ctrl.gains_description()}")
            res = sim_mod.simulate(
                ctrl,
                ref_j,
                ref_c,
                q0=q0,
                duration=float(c["duration"]),
                control_rate=float(c["control_rate"]),
                physics_rate=float(c["physics_rate"]),
                environment=env_k,
                force_noise=float(c["force_noise"]),
            )
            results[kind] = res
            rows[kind] = exp.force_metrics(
                res, f_des=float(c["f_des"]), z_surface=float(c["z_surface"])
            )
            r = rows[kind]
            log.info(
                f'[ForceCompare]   pico={r["peak_N"]:.1f} N | '
                f'regime={r["steady_N"]:.2f} N | '
                f'erro={r["error_N"]:+.2f} N | '
                f'ondulação={r["ripple_N"]:.3f} N | '
                f'penetração={r["penetration_mm"]:.3f} mm'
            )

        table = plotting.format_force_table(
            rows,
            title=(
                f'Contato em {info["material"]} '
                f'(K_env={info["stiffness"]:.3g} N/m), comando '
                f'{info["overshoot_mm"]:.0f} mm abaixo da superfície, '
                f'F_des={c["f_des"]:.0f} N'
            ),
        )
        for line in table.split("\n"):
            log.info(line)

        outdir = plotting.ensure_dir(c["output_dir"])
        with open(os.path.join(outdir, f'fcmp_table_{c["label"]}.txt'), "w") as fh:
            fh.write(table + "\n")

        files = plotting.plot_force_comparison(
            results,
            f_des=float(c["f_des"]),
            z_surface=float(c["z_surface"]),
            label=c["label"],
            outdir=outdir,
        )
        for f in files:
            log.info(f"[ForceCompare] gráfico salvo: {f}")

        payload = dict(config=c, scene=info, metrics=rows)
        with open(os.path.join(outdir, f'fcmp_metrics_{c["label"]}.json'), "w") as fh:
            json.dump(payload, fh, indent=2, default=float)
        self.pub.publish(String(data=json.dumps(payload, default=float)))

        self._interpretation(rows, info, float(c["f_des"]))
        log.info("[ForceCompare] concluído — Ctrl+C para sair.")

    def _interpretation(self, rows, info, f_des):
        log = self.get_logger()
        log.info("─" * 70)
        log.info("  LEITURA DO RESULTADO")
        if "position_only" in rows:
            r = rows["position_only"]
            log.info(
                f'  Sem controle de força, {info["overshoot_mm"]:.0f} mm '
                f'de erro de posição produziram {r["steady_N"]:.0f} N de '
                "contato."
            )
            log.info(
                f"  Essa força é K_env × penetração — muda com o "
                "MATERIAL, não com o que se pediu."
            )
        for k in ("impedance", "admittance", "hybrid"):
            if k not in rows:
                continue
            r = rows[k]
            direct = "direta" if k == "hybrid" else "indireta"
            log.info(
                f'  {plotting.LABELS[k]}: regime {r["steady_N"]:.2f} N '
                f'({r["error_N"]:+.2f} N em relação aos {f_des:.0f} N '
                f"pedidos) — controle de força {direct}."
            )
        if "impedance" in rows and "hybrid" in rows:
            log.info(
                "  A impedância NÃO tem referência de força: o valor que "
                "ela produz é K_d × deslocamento, e por isso muda quando"
            )
            log.info(
                "  o material muda. O híbrido fecha malha sobre a força "
                "medida e por isso entrega o valor pedido em qualquer"
            )
            log.info(
                "  material — ao custo de precisar de um sensor de força " "confiável."
            )
        worst = max(rows.items(), key=lambda kv: kv[1]["ripple_N"])
        if worst[1]["ripple_N"] > 1.0:
            log.info(
                f"  ATENÇÃO: {plotting.LABELS[worst[0]]} apresentou "
                f'ondulação de {worst[1]["ripple_N"]:.1f} N — sinal de '
                "instabilidade de contato."
            )
            log.info(
                "  Ambiente rígido demais para o ganho usado. É o limite "
                "fundamental do controle de força."
            )
        log.info("─" * 70)


def main(args=None):
    rclpy.init(args=args)
    node = ForceCompare()
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
