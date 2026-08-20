"""
ik_plotter.py — Gráficos de cinemática inversa

Gera automaticamente:
  1. Evolução dos ângulos das juntas por iteração (por método)
  2. Convergência do erro de posição e orientação (comparação entre métodos)
  3. Trajetória no espaço de juntas — fase portrait
  4. Comparação dos 3 métodos num mesmo gráfico de convergência
  5. Visualização 3D do manipulador na solução encontrada
"""

import rclpy
from rclpy.node import Node
import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

from kuka_inverse_kinematics.ik_solver import (
    solve_ik, forward_kinematics, JOINT_NAMES, JOINT_LIMITS
)


OUTPUT_DIR = os.path.expanduser('~/kuka_ik_plots')


def plot_joint_evolution(result: dict, path: str):
    """Evolução dos ângulos das juntas por iteração."""
    q_hist = np.array(result['q'])  # (n_iter, 6)
    n_iter = q_hist.shape[0]
    iters = np.arange(n_iter)
    colors = ['#e74c3c','#e67e22','#f1c40f','#2ecc71','#3498db','#9b59b6']

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle(
        f'Evolução das Juntas — IK {result["method"]}\n'
        f'Convergiu: {result["converged"]} | Iterações: {result["iterations"]}',
        fontsize=13, fontweight='bold'
    )

    for i, (ax, name, col) in enumerate(zip(axes.flat, JOINT_NAMES, colors)):
        q_deg = np.degrees(q_hist[:, i])
        ax.plot(iters, q_deg, color=col, linewidth=2)
        ax.axhline(np.degrees(JOINT_LIMITS[i, 0]), color='k', linestyle='--',
                   alpha=0.4, linewidth=1, label='Limite')
        ax.axhline(np.degrees(JOINT_LIMITS[i, 1]), color='k', linestyle='--',
                   alpha=0.4, linewidth=1)
        ax.axhline(q_deg[-1], color='gray', linestyle=':', alpha=0.6)
        ax.fill_between(iters, q_deg, q_deg[-1], alpha=0.15, color=col)
        ax.set_xlabel('Iteração', fontsize=9)
        ax.set_ylabel('Ângulo (°)', fontsize=9)
        ax.set_title(f'{name}: {q_deg[-1]:.2f}°', fontsize=10, fontweight='bold', color=col)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'[IK Plotter] Salvo: {path}')


def plot_convergence(result: dict, path: str):
    """Gráfico de convergência do erro de posição e orientação."""
    ep = np.array(result['error_pos']) * 1000  # mm
    eo = np.degrees(np.array(result['error_ori']))
    iters = np.arange(len(ep))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(
        f'Convergência IK — {result["method"]}\n'
        f'Alvo: ({result["p_des"][0]:.3f},{result["p_des"][1]:.3f},{result["p_des"][2]:.3f}) m',
        fontsize=13, fontweight='bold'
    )

    # Erro de posição
    ax1.semilogy(iters, ep, 'royalblue', linewidth=2)
    ax1.axhline(0.1, color='r', linestyle='--', label='Tolerância (0.1mm)')
    ax1.set_ylabel('Erro de Posição (mm)', fontsize=11)
    ax1.set_title('Erro de Posição', fontsize=11)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.annotate(f'{ep[-1]:.4f}mm', (len(ep)-1, ep[-1]),
                 textcoords='offset points', xytext=(-50, 10), fontsize=9, color='royalblue')

    # Erro de orientação
    ax2.semilogy(iters, np.maximum(eo, 1e-6), 'tomato', linewidth=2)
    ax2.axhline(0.1, color='r', linestyle='--', label='Tolerância (0.1°)')
    ax2.set_xlabel('Iteração', fontsize=11)
    ax2.set_ylabel('Erro de Orientação (°)', fontsize=11)
    ax2.set_title('Erro de Orientação', fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'[IK Plotter] Salvo: {path}')


def plot_methods_comparison(results: list, path: str):
    """Compara os 3 métodos de IK no mesmo gráfico."""
    colors = {'Pseudo-Inverso': 'royalblue', 'Transposto': 'tomato', 'Levenberg-Marquardt': 'seagreen'}
    styles = {'Pseudo-Inverso': '-', 'Transposto': '--', 'Levenberg-Marquardt': '-.'}

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Comparação dos Métodos de Cinemática Inversa\nKUKA KR70 R2100',
                 fontsize=13, fontweight='bold')

    for result in results:
        name = result['method']
        col = colors.get(name, 'gray')
        sty = styles.get(name, '-')
        ep = np.array(result['error_pos']) * 1000
        eo = np.degrees(np.array(result['error_ori']))
        iters = np.arange(len(ep))
        label = f'{name} ({result["iterations"]} iter)'

        ax1.semilogy(iters, ep, color=col, linestyle=sty, linewidth=2, label=label)
        ax2.semilogy(iters, np.maximum(eo, 1e-6), color=col, linestyle=sty, linewidth=2, label=label)

    ax1.axhline(0.1, color='k', linestyle=':', alpha=0.5, label='Tol. 0.1mm')
    ax1.set_xlabel('Iteração', fontsize=11)
    ax1.set_ylabel('Erro de Posição (mm)', fontsize=11)
    ax1.set_title('Convergência — Posição', fontsize=11)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    ax2.axhline(0.1, color='k', linestyle=':', alpha=0.5, label='Tol. 0.1°')
    ax2.set_xlabel('Iteração', fontsize=11)
    ax2.set_ylabel('Erro de Orientação (°)', fontsize=11)
    ax2.set_title('Convergência — Orientação', fontsize=11)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'[IK Plotter] Salvo: {path}')


def plot_joint_space_trajectory(results: list, path: str):
    """Trajetória no espaço de juntas para cada método."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Trajetória no Espaço de Juntas — Comparação de Métodos IK\nKUKA KR70 R2100',
                 fontsize=13, fontweight='bold')
    colors_method = ['royalblue', 'tomato', 'seagreen']

    for result, col in zip(results, colors_method):
        q_hist = np.degrees(np.array(result['q']))
        for i, ax in enumerate(axes.flat):
            ax.plot(q_hist[:, i], alpha=0.8, linewidth=2, color=col, label=result['method'])
            ax.axhline(np.degrees(JOINT_LIMITS[i, 0]), color='k', linestyle='--', alpha=0.3)
            ax.axhline(np.degrees(JOINT_LIMITS[i, 1]), color='k', linestyle='--', alpha=0.3)
            ax.set_title(JOINT_NAMES[i], fontsize=10, fontweight='bold')
            ax.set_xlabel('Iteração', fontsize=8)
            ax.set_ylabel('Ângulo (°)', fontsize=8)
            ax.grid(True, alpha=0.3)

    axes.flat[0].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'[IK Plotter] Salvo: {path}')


def plot_solution_3d(result: dict, p_des: np.ndarray, path: str):
    """Visualização 3D do manipulador na solução IK."""
    q_sol = result['q_result']
    T, frames = forward_kinematics(q_sol)
    pts = np.array([f[:3, 3] for f in frames])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors_link = ['#e74c3c','#e67e22','#f1c40f','#2ecc71','#3498db','#9b59b6']
    labels_link = ['base→J1','J1→J2','J2→J3','J3→J4','J4→J5','J5→TCP']

    for i in range(len(pts)-1):
        ax.plot([pts[i,0], pts[i+1,0]],
                [pts[i,1], pts[i+1,1]],
                [pts[i,2], pts[i+1,2]],
                '-', color=colors_link[i], linewidth=4, label=labels_link[i])
        ax.scatter(*pts[i], color=colors_link[i], s=80, zorder=5)

    # TCP atual
    p_tcp = pts[-1]
    ax.scatter(*p_tcp, color='gold', s=200, marker='*', zorder=10,
               label=f'TCP ({p_tcp[0]:.3f},{p_tcp[1]:.3f},{p_tcp[2]:.3f})')

    # Alvo desejado
    ax.scatter(*p_des, color='red', s=200, marker='X', zorder=10,
               label=f'Alvo ({p_des[0]:.3f},{p_des[1]:.3f},{p_des[2]:.3f})')

    # Linha do erro
    ax.plot([p_tcp[0], p_des[0]], [p_tcp[1], p_des[1]], [p_tcp[2], p_des[2]],
            'r--', linewidth=1.5, alpha=0.7,
            label=f'Erro: {np.linalg.norm(p_des-p_tcp)*1000:.2f}mm')

    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.set_zlabel('Z (m)', fontsize=11)
    ax.set_title(
        f'Solução IK — {result["method"]}\n'
        f'{"Convergiu" if result["converged"] else "NÃO convergiu"} em {result["iterations"]} iterações',
        fontsize=12, fontweight='bold'
    )
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'[IK Plotter] Salvo: {path}')


class IKPlotter(Node):

    def __init__(self):
        super().__init__('ik_plotter')
        self.declare_parameter('output_dir', os.path.expanduser('~/kuka_ik_plots'))
        self.declare_parameter('target_x', 1.2)
        self.declare_parameter('target_y', 0.5)
        self.declare_parameter('target_z', 0.8)

        self.out_dir = self.get_parameter('output_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        p_des = np.array([
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
            self.get_parameter('target_z').value,
        ])

        self.get_logger().info(
            f'[IK Plotter] Gerando gráficos para alvo '
            f'({p_des[0]:.3f},{p_des[1]:.3f},{p_des[2]:.3f})...'
        )
        self._generate_all(p_des)
        self.get_logger().info(f'[IK Plotter] Todos os gráficos salvos em {self.out_dir}')

    def _generate_all(self, p_des: np.ndarray):
        q_init = np.zeros(6)

        # Resolve com os 3 métodos
        results = []
        for method in ['pinv', 'transpose', 'lm']:
            r = solve_ik(p_des, q_init=q_init, method=method, max_iter=300)
            results.append(r)
            self.get_logger().info(
                f'[IK Plotter] {r["method"]}: '
                f'converged={r["converged"]} | iter={r["iterations"]} | '
                f'err={r["final_error_pos"]*1000:.3f}mm'
            )

        # Gráficos individuais para o melhor método (LM)
        lm_result = results[2]
        plot_joint_evolution(lm_result,
            os.path.join(self.out_dir, 'ik_joint_evolution_lm.png'))
        plot_convergence(lm_result,
            os.path.join(self.out_dir, 'ik_convergence_lm.png'))

        # Comparação entre métodos
        plot_methods_comparison(results,
            os.path.join(self.out_dir, 'ik_methods_comparison.png'))

        # Trajetória no espaço de juntas
        plot_joint_space_trajectory(results,
            os.path.join(self.out_dir, 'ik_joint_space_trajectory.png'))

        # Visualização 3D da solução LM
        plot_solution_3d(lm_result, p_des,
            os.path.join(self.out_dir, 'ik_solution_3d.png'))

        # Múltiplos alvos — tabela de soluções
        self._plot_multiple_targets()

    def _plot_multiple_targets(self):
        """Gera gráfico com soluções IK para múltiplos alvos."""
        targets = [
            ([1.2,  0.5, 0.8], 'Alvo 1'),
            ([1.0, -0.5, 1.0], 'Alvo 2'),
            ([0.8,  0.0, 1.5], 'Alvo 3'),
            ([1.4,  0.3, 0.5], 'Alvo 4'),
        ]

        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        fig.suptitle('Cinemática Inversa — Múltiplos Alvos\nKUKA KR70 R2100 (Método LM)',
                     fontsize=13, fontweight='bold')

        colors = ['royalblue', 'tomato', 'seagreen', 'purple']

        for (p_des, label), col in zip(targets, colors):
            p = np.array(p_des)
            r = solve_ik(p, q_init=np.zeros(6), method='lm', max_iter=200)
            T, frames = forward_kinematics(r['q_result'])
            pts = np.array([f[:3, 3] for f in frames])

            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                    '-o', color=col, linewidth=2, markersize=5, alpha=0.7,
                    label=f'{label} → {"✓" if r["converged"] else "✗"} ({r["iterations"]}it)')
            ax.scatter(*p, color=col, s=150, marker='X', zorder=5)

        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_zlabel('Z (m)', fontsize=11)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        path = os.path.join(self.out_dir, 'ik_multiple_targets_3d.png')
        plt.savefig(path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f'[IK Plotter] Salvo: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = IKPlotter()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
