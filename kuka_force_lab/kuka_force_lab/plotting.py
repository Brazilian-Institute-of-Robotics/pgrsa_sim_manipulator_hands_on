import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from kuka_control_lab.plotting import ensure_dir, DEFAULT_DIR  # noqa: E402,F401

COLORS = {
    "position_only": "#d62728",
    "impedance": "#1f77b4",
    "admittance": "#2ca02c",
    "reference": "#7f7f7f",
    "surface": "#8c564b",
}

LABELS = {
    "position_only": "Posição pura (sem controle de força)",
    "impedance": "Impedância (indireto, torque)",
    "admittance": "Admitância (indireto, posição)",
    "hybrid": "Híbrido força/posição (direto)",
}


def _save(fig, outdir, name):
    path = os.path.join(outdir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_force_run(res, f_des=None, z_surface=None, label="force", outdir=None):
    """Figuras de uma única execução de contato."""
    outdir = ensure_dir(outdir)
    key = res.meta.get("controller", "impedance")
    color = COLORS.get(key, "#1f77b4")
    files = []

    fn = np.linalg.norm(res.f_ext[:, :3], axis=1)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(res.t, res.f_ext[:, 2], color=color, lw=1.6, label="F_z medida")
    if f_des is not None:
        axes[0].axhline(
            f_des,
            ls="--",
            color=COLORS["reference"],
            label=f"F desejada = {f_des:.1f} N",
        )
    axes[0].set_ylabel("força normal [N]")
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)

    axes[1].plot(res.t, res.x[:, 2], color=color, lw=1.6, label="Z do TCP")
    axes[1].plot(
        res.t,
        res.x_des[:, 2],
        "--",
        color=COLORS["reference"],
        lw=1.2,
        label="Z comandado",
    )
    if z_surface is not None:
        axes[1].axhline(z_surface, color=COLORS["surface"], lw=2, label="superfície")
    axes[1].set_ylabel("altura [m]")
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)

    axes[2].plot(res.t, fn, color=color, lw=1.4)
    axes[2].set_ylabel("‖F_ext‖ [N]")
    axes[2].set_xlabel("t [s]")
    axes[2].grid(alpha=0.3)

    fig.suptitle(f"Contato — {LABELS.get(key, key)} ({label})")
    files.append(_save(fig, outdir, f"force_run_{label}.png"))

    fig, ax = plt.subplots(figsize=(10, 5))
    for j in range(6):
        ax.plot(res.t, res.tau[:, j], lw=1.0, label=f"joint_{j+1}")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("τ [N·m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=6)
    ax.set_title(f"Torque nas juntas durante o contato ({label})")
    files.append(_save(fig, outdir, f"force_torque_{label}.png"))

    return files


def plot_force_comparison(
    results, f_des=20.0, z_surface=None, label="cmp", outdir=None
):
    """Comparação entre as leis de interação, sob contato idêntico."""
    outdir = ensure_dir(outdir)
    files = []

    # Força de contato, o gráfico principal
    fig, ax = plt.subplots(figsize=(11, 5.5))
    for k, r in results.items():
        ax.plot(r.t, r.f_ext[:, 2], color=COLORS.get(k), lw=1.6, label=LABELS.get(k, k))
    ax.axhline(
        f_des,
        ls="--",
        color=COLORS["reference"],
        lw=1.5,
        label=f"força desejada = {f_des:.0f} N",
    )
    ax.set_xlabel("t [s]")
    ax.set_ylabel("força normal de contato [N]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title(
        "Força de contato — mesma superfície, mesmo comando de "
        f"posição, quatro leis diferentes ({label})"
    )
    files.append(_save(fig, outdir, f"fcmp_force_{label}.png"))

    # O mesmo em escala logarítmica (a de posição pura sai da escala linear)
    fig, ax = plt.subplots(figsize=(11, 5.5))
    for k, r in results.items():
        ax.semilogy(
            r.t,
            np.maximum(np.abs(r.f_ext[:, 2]), 1e-2),
            color=COLORS.get(k),
            lw=1.6,
            label=LABELS.get(k, k),
        )
    ax.axhline(f_des, ls="--", color=COLORS["reference"], lw=1.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|força normal| [N] (escala log)")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)
    ax.set_title(
        "Mesma comparação em escala logarítmica — necessária porque a "
        f"posição pura sai da escala ({label})"
    )
    files.append(_save(fig, outdir, f"fcmp_force_log_{label}.png"))

    # Altura do TCP versus a superfície
    fig, ax = plt.subplots(figsize=(11, 5))
    first = list(results.values())[0]
    ax.plot(
        first.t,
        first.x_des[:, 2],
        "--",
        color=COLORS["reference"],
        lw=1.4,
        label="Z comandado",
    )
    if z_surface is not None:
        ax.axhline(z_surface, color=COLORS["surface"], lw=2, label="superfície")
    for k, r in results.items():
        ax.plot(r.t, r.x[:, 2], color=COLORS.get(k), lw=1.5, label=LABELS.get(k, k))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Z do TCP [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title(
        "Onde cada lei deixa o TCP parar — a de posição pura insiste "
        f"em ir para o ponto comandado, dentro do material ({label})"
    )
    files.append(_save(fig, outdir, f"fcmp_height_{label}.png"))

    #  Barras de resumo
    keys = list(results.keys())
    peaks, regimes, ripples = [], [], []
    for k in keys:
        fz = results[k].f_ext[:, 2]
        tail = fz[int(0.7 * len(fz)) :]
        peaks.append(float(np.max(np.abs(fz))))
        regimes.append(float(np.mean(tail)))
        ripples.append(float(np.std(tail)))
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    for ax, vals, title, logy in (
        (axes[0], peaks, "pico de força [N]", True),
        (axes[1], regimes, "força em regime [N]", False),
        (axes[2], ripples, "ondulação da força [N]", True),
    ):
        ax.bar(
            range(len(keys)),
            np.maximum(vals, 1e-3),
            color=[COLORS.get(k) for k in keys],
        )
        if logy:
            ax.set_yscale("log")
        ax.set_xticks(range(len(keys)))
        ax.set_xticklabels(
            [LABELS.get(k, k).split(" (")[0] for k in keys],
            rotation=20,
            ha="right",
            fontsize=8,
        )
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis="y")
        for i, v in enumerate(vals):
            ax.text(i, max(v, 1e-3), f"{v:.3g}", ha="center", va="bottom", fontsize=8)
    if f_des:
        axes[1].axhline(f_des, ls="--", color=COLORS["reference"])
    fig.suptitle(f"Resumo do contato ({label})")
    files.append(_save(fig, outdir, f"fcmp_summary_{label}.png"))

    return files


def plot_stiffness_sweep(data, label="rigidez", outdir=None):
    """
    `data`: dict {controlador: [(K_env, f_regime, f_ripple, f_peak), ...]}.
    Mostra como cada lei se comporta conforme o ambiente endurece — e onde
    cada uma perde a estabilidade.
    """
    outdir = ensure_dir(outdir)
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for k, rows in data.items():
        rows = sorted(rows)
        K = [r[0] for r in rows]
        axes[0].loglog(
            K,
            [max(abs(r[1]), 1e-2) for r in rows],
            "o-",
            color=COLORS.get(k),
            label=LABELS.get(k, k),
        )
        axes[1].loglog(
            K,
            [max(r[2], 1e-3) for r in rows],
            "o-",
            color=COLORS.get(k),
            label=LABELS.get(k, k),
        )
        axes[2].loglog(
            K,
            [max(abs(r[3]), 1e-2) for r in rows],
            "o-",
            color=COLORS.get(k),
            label=LABELS.get(k, k),
        )
    for ax, t in zip(
        axes,
        (
            "força em regime [N]",
            "ondulação da força [N]\n(alta = instabilidade " "de contato)",
            "pico de força [N]",
        ),
    ):
        ax.set_xlabel("rigidez do ambiente K_env [N/m]")
        ax.set_title(t, fontsize=10)
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=7)
    fig.suptitle(
        "Efeito da rigidez do ambiente sobre cada lei de interação " f"({label})"
    )
    return [_save(fig, outdir, f"fsweep_stiffness_{label}.png")]


def format_force_table(rows, title="Comparação de leis de interação"):
    hdr = (
        f'{"Lei de interação":<38}{"Pico":>10}{"Regime":>10}'
        f'{"Erro":>10}{"Ondul.":>10}{"Penetr.":>11}{"τ pico":>10}'
    )
    lines = [
        "",
        title,
        "=" * len(hdr),
        hdr,
        f'{"":<38}{"[N]":>10}{"[N]":>10}{"[N]":>10}{"[N]":>10}'
        f'{"[mm]":>11}{"[N·m]":>10}',
        "-" * len(hdr),
    ]
    for k, r in rows.items():
        lines.append(
            f'{LABELS.get(k, k):<38}{r["peak_N"]:>10.1f}'
            f'{r["steady_N"]:>10.2f}{r["error_N"]:>10.2f}'
            f'{r["ripple_N"]:>10.3f}{r["penetration_mm"]:>11.3f}'
            f'{r["tau_peak_Nm"]:>10.0f}'
        )
    lines.append("=" * len(hdr))
    return "\n".join(lines)
