import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_DIR = os.path.expanduser("~/kuka_control_plots")

COLORS = {
    "local_pd": "#1f77b4",
    "computed_torque": "#ff7f0e",
    "operational_space": "#2ca02c",
    "reference": "#7f7f7f",
    "bad": "#d62728",
}

LABELS = {
    "local_pd": "Local (PD por junta)",
    "computed_torque": "Centralizado (torque computado)",
    "operational_space": "Espaço operacional",
}


def ensure_dir(path=None):
    path = path or DEFAULT_DIR
    os.makedirs(path, exist_ok=True)
    return path


def _save(fig, outdir, name):
    path = os.path.join(outdir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_single_run(res, label="run", outdir=None):
    """Quatro figuras descrevendo uma execução isolada."""
    outdir = ensure_dir(outdir)
    files = []
    key = res.meta.get("controller", "local_pd")
    color = COLORS.get(key, "#1f77b4")

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for j, ax in enumerate(axes.ravel()):
        ax.plot(
            res.t,
            np.degrees(res.q_des[:, j]),
            "--",
            color=COLORS["reference"],
            lw=1.4,
            label="desejado",
        )
        ax.plot(res.t, np.degrees(res.q[:, j]), color=color, lw=1.6, label="medido")
        ax.set_ylabel(f"joint_{j+1} [°]")
        ax.grid(alpha=0.3)
        if j == 0:
            ax.legend(fontsize=8)
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.suptitle(f"Rastreamento por junta — {LABELS.get(key, key)} ({label})")
    files.append(_save(fig, outdir, f"ctrl_joints_{label}.png"))

    e_j = np.linalg.norm(res.q_des - res.q, axis=1)
    e_c = np.linalg.norm(res.x_des - res.x, axis=1) * 1000.0
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].semilogy(res.t, np.maximum(e_j, 1e-8), color=color)
    axes[0].set_ylabel("‖erro juntas‖ [rad]")
    axes[0].grid(alpha=0.3, which="both")
    axes[1].semilogy(res.t, np.maximum(e_c, 1e-5), color=color)
    axes[1].set_ylabel("‖erro TCP‖ [mm]")
    axes[1].grid(alpha=0.3, which="both")
    for j in range(6):
        axes[2].plot(res.t, res.tau[:, j], lw=1.0, label=f"j{j+1}")
    axes[2].set_ylabel("torque [N·m]")
    axes[2].set_xlabel("t [s]")
    axes[2].grid(alpha=0.3)
    axes[2].legend(fontsize=7, ncol=6)
    fig.suptitle(f"Erro e esforço de controle — {LABELS.get(key, key)} ({label})")
    files.append(_save(fig, outdir, f"ctrl_error_torque_{label}.png"))

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        res.x_des[:, 0],
        res.x_des[:, 1],
        res.x_des[:, 2],
        "--",
        color=COLORS["reference"],
        label="desejado",
    )
    ax.plot(res.x[:, 0], res.x[:, 1], res.x[:, 2], color=color, label="real")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    ax.set_title(f"Trajetória do TCP — {LABELS.get(key, key)} ({label})")
    files.append(_save(fig, outdir, f"ctrl_tcp3d_{label}.png"))

    d = res.q_des[-1] - res.q[0]
    j = int(np.argmax(np.abs(d)))
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(
        np.degrees(res.q[:, j] - res.q_des[:, j]),
        np.degrees(res.qd[:, j]),
        color=color,
        lw=1.2,
    )
    ax.plot(0, 0, "k*", ms=12, label="alvo")
    ax.set_xlabel(f"erro joint_{j+1} [°]")
    ax.set_ylabel(f"velocidade joint_{j+1} [°/s]")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title(
        f"Plano de fase (joint_{j+1}) — {LABELS.get(key, key)} ({label})\n"
        "espiral para o centro = estável · órbita fechada = ciclo limite"
    )
    files.append(_save(fig, outdir, f"ctrl_phase_{label}.png"))

    return files


# Gráficos das estratégias
def plot_comparison(results, label="cmp", outdir=None):
    """
    `results`: dict {nome_estrategia: SimResult}. Gera as figuras que sustentam
    a comparação entre abordagens local, centralizada e de espaço operacional.
    """
    outdir = ensure_dir(outdir)
    files = []

    fig, ax = plt.subplots(figsize=(10, 5))
    for k, r in results.items():
        e = np.linalg.norm(r.x_des - r.x, axis=1) * 1000.0
        ax.semilogy(
            r.t,
            np.maximum(e, 1e-4),
            color=COLORS.get(k),
            lw=1.6,
            label=LABELS.get(k, k),
        )
    ax.set_xlabel("t [s]")
    ax.set_ylabel("‖erro do TCP‖ [mm]")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    ax.set_title(
        "Erro no espaço da TAREFA — única régua comparável entre as três\n" f"({label})"
    )
    files.append(_save(fig, outdir, f"cmp_cart_error_{label}.png"))

    fig, ax = plt.subplots(figsize=(10, 5))
    for k, r in results.items():
        e = np.linalg.norm(r.q_des - r.q, axis=1)
        ax.semilogy(
            r.t,
            np.maximum(e, 1e-8),
            color=COLORS.get(k),
            lw=1.6,
            label=LABELS.get(k, k),
        )
    ax.set_xlabel("t [s]")
    ax.set_ylabel("‖erro de juntas‖ [rad]")
    ax.grid(alpha=0.3, which="both")
    ax.legend()
    ax.set_title(
        "Erro no espaço das JUNTAS — o controlador de espaço operacional\n"
        "não regula esta grandeza (deixa a postura livre no espaço nulo)"
        f" ({label})"
    )
    files.append(_save(fig, outdir, f"cmp_joint_error_{label}.png"))

    # Esforço de controle
    fig, ax = plt.subplots(figsize=(10, 5))
    for k, r in results.items():
        ax.plot(
            r.t,
            np.linalg.norm(r.tau, axis=1),
            color=COLORS.get(k),
            lw=1.4,
            label=LABELS.get(k, k),
        )
    ax.set_xlabel("t [s]")
    ax.set_ylabel("‖τ‖ [N·m]")
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title(f"Esforço de controle — quanto custa cada estratégia ({label})")
    files.append(_save(fig, outdir, f"cmp_torque_{label}.png"))

    # Barras de métricas
    keys = list(results.keys())
    mets = {k: results[k].metrics() for k in keys}
    fig, axes = plt.subplots(1, 4, figsize=(15, 4.5))
    specs = [
        ("rms_cart_mm", "RMS TCP [mm]"),
        ("max_cart_mm", "pico TCP [mm]"),
        ("final_cart_mm", "regime TCP [mm]"),
        ("tau_rms_Nm", "RMS torque [N·m]"),
    ]
    for ax, (mk, title) in zip(axes, specs):
        vals = [mets[k][mk] for k in keys]
        ax.bar(range(len(keys)), vals, color=[COLORS.get(k) for k in keys])
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
            ax.text(i, v, f"{v:.3g}", ha="center", va="bottom", fontsize=8)
    fig.suptitle(f"Resumo comparativo das estratégias ({label})")
    files.append(_save(fig, outdir, f"cmp_summary_{label}.png"))

    # Trajetória 3D sobreposta
    fig = plt.figure(figsize=(7.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    first = results[keys[0]]
    ax.plot(
        first.x_des[:, 0],
        first.x_des[:, 1],
        first.x_des[:, 2],
        "--",
        color=COLORS["reference"],
        lw=2,
        label="desejado",
    )
    for k, r in results.items():
        ax.plot(
            r.x[:, 0],
            r.x[:, 1],
            r.x[:, 2],
            color=COLORS.get(k),
            lw=1.4,
            label=LABELS.get(k, k),
        )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(fontsize=8)
    ax.set_title(f"Caminho do TCP sob cada estratégia ({label})")
    files.append(_save(fig, outdir, f"cmp_tcp3d_{label}.png"))

    return files


def format_table(mets, title="Comparação de estratégias"):
    """Tabela de texto com as métricas — impressa no log e salva em .txt."""
    hdr = (
        f'{"Estratégia":<34}{"RMS TCP":>10}{"Pico TCP":>10}'
        f'{"Regime TCP":>12}{"RMS junta":>11}{"Assent.":>9}'
        f'{"Sobressin.":>11}{"RMS τ":>10}{"Estável":>9}'
    )
    lines = [
        f"\n{title}",
        "=" * len(hdr),
        hdr,
        f'{"":<34}{"[mm]":>10}{"[mm]":>10}{"[mm]":>12}{"[rad]":>11}'
        f'{"[s]":>9}{"[%]":>11}{"[N·m]":>10}{"":>9}',
        "-" * len(hdr),
    ]
    for k, m in mets.items():
        st = "s" if m["settling_time_cart_s"] >= 0 else -1
        settle = (
            f'{m["settling_time_cart_s"]:.2f}'
            if m["settling_time_cart_s"] >= 0
            else "N/A"
        )
        lines.append(
            f'{LABELS.get(k, k):<34}{m["rms_cart_mm"]:>10.3f}'
            f'{m["max_cart_mm"]:>10.3f}{m["final_cart_mm"]:>12.4f}'
            f'{m["rms_joint_rad"]:>11.5f}{settle:>9}'
            f'{m["overshoot_pct"]:>11.1f}{m["tau_rms_Nm"]:>10.1f}'
            f'{("NÃO" if m["unstable"] else "sim"):>9}'
        )
        del st
    lines.append("=" * len(hdr))
    return "\n".join(lines)
