"""
ik_comparison.py — Comparação: Juntas Estimadas pela IK vs Sensores

Executa a IK para um alvo cartesiano, envia o movimento para o robô e
coleta em tempo real:
  - Ângulos das juntas estimados pela IK (solução)
  - Ângulos das juntas lidos pelos sensores virtuais (/joint_states)

Ao encerrar (Ctrl+C), salva gráficos comparativos:
  - Ângulos estimados vs sensores por joint ao longo do tempo
  - Erro entre estimativa IK e medição do sensor
  - Trajetória 3D do TCP durante o movimento

Uso:
  ros2 run kuka_inverse_kinematics ik_comparison --ros-args \
    -p target_x:=1.2 -p target_y:=0.5 -p target_z:=0.8 \
    -p method:=lm -p label:=alvo1 -p output_dir:=~/kuka_ik_plots
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import json

from kuka_inverse_kinematics.ik_solver import (
    solve_ik, forward_kinematics, JOINT_NAMES, JOINT_LIMITS
)


class IKComparison(Node):

    def __init__(self):
        super().__init__('ik_comparison')

        self.declare_parameter('target_x',   1.2)
        self.declare_parameter('target_y',   0.5)
        self.declare_parameter('target_z',   0.8)
        self.declare_parameter('method',     'lm')
        self.declare_parameter('label',      'alvo')
        self.declare_parameter('output_dir', os.path.expanduser('~/kuka_ik_plots'))
        self.declare_parameter('move_time',  5.0)

        self.p_des = np.array([
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
            self.get_parameter('target_z').value,
        ])
        self.method   = self.get_parameter('method').value
        self.label    = self.get_parameter('label').value
        self.out_dir  = self.get_parameter('output_dir').value
        self.move_time = self.get_parameter('move_time').value
        os.makedirs(self.out_dir, exist_ok=True)

        # Estado
        self.q_current = np.zeros(6)
        self.joint_order = {}
        self.q_ik_solution = None  # solução da IK (constante após resolução)

        # Buffers de coleta
        self.t_start = None
        self.sensor_times = []
        self.sensor_q     = []   # ângulos lidos pelos sensores
        self.sensor_xyz   = []   # TCP calculado pelos sensores
        self.ik_q_repeated = []  # solução IK repetida para comparação
        self.collecting   = False

        # ROS
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/kuka_arm_controller/follow_joint_trajectory'
        )

        self.pub_result = self.create_publisher(
            Float64MultiArray, '/ik/joint_solution', 10)

        self.pub_info = self.create_publisher(
            String, '/ik/convergence_info', 10)

        # Resolve e executa após 2s
        self.create_timer(2.0, self._solve_and_execute)

        self.get_logger().info(
            f'[IK Comparison] Iniciado | alvo=({self.p_des[0]:.3f},'
            f'{self.p_des[1]:.3f},{self.p_des[2]:.3f}) | método={self.method}'
        )

    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q_current[j] = msg.position[idx]

        if self.collecting:
            t = self.get_clock().now().nanoseconds / 1e9
            if self.t_start is None:
                self.t_start = t
            self.sensor_times.append(t - self.t_start)
            self.sensor_q.append(self.q_current.copy())
            T, _ = forward_kinematics(self.q_current)
            self.sensor_xyz.append(T[:3, 3].copy())
            if self.q_ik_solution is not None:
                self.ik_q_repeated.append(self.q_ik_solution.copy())

    def _solve_and_execute(self):
        """Resolve IK e envia trajetória para o robô."""
        self.get_logger().info(
            f'[IK Comparison] Resolvendo IK ({self.method}) para '
            f'({self.p_des[0]:.4f},{self.p_des[1]:.4f},{self.p_des[2]:.4f})...'
        )

        result = solve_ik(
            p_des=self.p_des,
            q_init=self.q_current.copy(),
            method=self.method,
            max_iter=300,
        )

        self.q_ik_solution = result['q_result']

        # Log
        T_check, _ = forward_kinematics(self.q_ik_solution)
        p_check = T_check[:3, 3]
        self.get_logger().info(
            f'[IK Comparison] {"CONVERGIU" if result["converged"] else "NÃO CONVERGIU"} | '
            f'iter={result["iterations"]} | '
            f'erro={result["final_error_pos"]*1000:.3f}mm'
        )
        self.get_logger().info(
            f'[IK Comparison] TCP verificado: ({p_check[0]:.4f},{p_check[1]:.4f},{p_check[2]:.4f})'
        )
        self.get_logger().info(
            f'[IK Comparison] Solução (°): ' +
            ' | '.join(f'{JOINT_NAMES[i]}={np.degrees(self.q_ik_solution[i]):.2f}°'
                      for i in range(6))
        )

        # Publica resultado
        sol_msg = Float64MultiArray()
        sol_msg.data = self.q_ik_solution.tolist()
        self.pub_result.publish(sol_msg)

        info = {
            'method': result['method'],
            'converged': result['converged'],
            'iterations': result['iterations'],
            'error_pos_mm': result['final_error_pos'] * 1000,
            'q_solution_deg': np.degrees(self.q_ik_solution).round(3).tolist(),
            'tcp_check': p_check.round(4).tolist(),
        }
        info_msg = String()
        info_msg.data = json.dumps(info, indent=2)
        self.pub_info.publish(info_msg)

        # Inicia coleta e envia para o robô
        self.collecting = True
        self._send_to_robot(self.q_ik_solution)

    def _send_to_robot(self, q_sol: np.ndarray):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('[IK Comparison] Action server não disponível.')
            return

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions     = q_sol.tolist()
        pt.velocities    = [0.0] * 6
        pt.accelerations = [0.0] * 6
        pt.time_from_start = Duration(sec=int(self.move_time))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(f'[IK Comparison] Enviando para o robô ({self.move_time}s)...')
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        gh = future.result()
        if gh.accepted:
            self.get_logger().info('[IK Comparison] Movimento iniciado — coletando dados...')
            gh.get_result_async().add_done_callback(self._result_cb)
        else:
            self.get_logger().error('[IK Comparison] Goal rejeitado.')

    def _result_cb(self, future):
        self.get_logger().info(
            '[IK Comparison] Movimento concluído. '
            'Pressione Ctrl+C para gerar os gráficos.'
        )

    def generate_plots(self):
        if len(self.sensor_q) < 5:
            self.get_logger().warn('[IK Comparison] Dados insuficientes.')
            return

        t_s   = np.array(self.sensor_times)
        q_s   = np.degrees(np.array(self.sensor_q))
        xyz_s = np.array(self.sensor_xyz)
        q_ik  = np.degrees(np.array(self.ik_q_repeated)) if self.ik_q_repeated else None

        colors_j = ['#e74c3c','#e67e22','#f1c40f','#2ecc71','#3498db','#9b59b6']

        # ── Gráfico 1: Juntas estimadas IK vs sensores ────────────────────
        fig, axes = plt.subplots(2, 3, figsize=(15, 9))
        fig.suptitle(
            f'Ângulos das Juntas — IK Estimada vs Sensores\n'
            f'KUKA KR70 R2100 | Alvo: {self.label} '
            f'({self.p_des[0]:.3f},{self.p_des[1]:.3f},{self.p_des[2]:.3f}) m',
            fontsize=13, fontweight='bold'
        )
        for i, (ax, col) in enumerate(zip(axes.flat, colors_j)):
            # Sensor
            ax.plot(t_s, q_s[:, i], color=col, linewidth=2,
                    label='Sensor (medido)', zorder=3)
            # IK (linha horizontal — valor constante da solução)
            if self.q_ik_solution is not None:
                q_ik_val = np.degrees(self.q_ik_solution[i])
                ax.axhline(q_ik_val, color='k', linewidth=2,
                           linestyle='--', label=f'IK solução: {q_ik_val:.2f}°', zorder=2)
            # Limites
            ax.axhline(np.degrees(JOINT_LIMITS[i, 0]), color='gray',
                       linestyle=':', alpha=0.5, linewidth=1)
            ax.axhline(np.degrees(JOINT_LIMITS[i, 1]), color='gray',
                       linestyle=':', alpha=0.5, linewidth=1)
            ax.set_title(JOINT_NAMES[i], fontsize=10, fontweight='bold', color=col)
            ax.set_xlabel('Tempo (s)', fontsize=9)
            ax.set_ylabel('Ângulo (°)', fontsize=9)
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
        plt.tight_layout()
        p1 = os.path.join(self.out_dir, f'ik_comparison_joints_{self.label}.png')
        plt.savefig(p1, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'[IK Comparison] Salvo: {p1}')

        # ── Gráfico 2: Erro por joint ──────────────────────────────────────
        if self.q_ik_solution is not None:
            fig, axes = plt.subplots(2, 3, figsize=(15, 8))
            fig.suptitle(
                f'Erro por Joint — |IK Estimada − Sensor|\n'
                f'Alvo: {self.label}',
                fontsize=13, fontweight='bold'
            )
            for i, (ax, col) in enumerate(zip(axes.flat, colors_j)):
                q_ik_val = np.degrees(self.q_ik_solution[i])
                error_i  = np.abs(q_s[:, i] - q_ik_val)
                ax.plot(t_s, error_i, color=col, linewidth=2)
                ax.fill_between(t_s, error_i, alpha=0.2, color=col)
                ax.set_title(
                    f'{JOINT_NAMES[i]} | RMS={np.sqrt(np.mean(error_i**2)):.3f}°',
                    fontsize=10, fontweight='bold', color=col
                )
                ax.set_xlabel('Tempo (s)', fontsize=9)
                ax.set_ylabel('Erro (°)', fontsize=9)
                ax.grid(True, alpha=0.3)
            plt.tight_layout()
            p2 = os.path.join(self.out_dir, f'ik_comparison_error_{self.label}.png')
            plt.savefig(p2, dpi=150, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f'[IK Comparison] Salvo: {p2}')

        # ── Gráfico 3: Trajetória 3D do TCP ───────────────────────────────
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(xyz_s[:, 0], xyz_s[:, 1], xyz_s[:, 2],
                color='royalblue', linewidth=2, label='Trajetória TCP (sensor)')
        ax.scatter(*xyz_s[0],  color='green', s=120, zorder=5, label='Início')
        ax.scatter(*xyz_s[-1], color='red',   s=120, zorder=5, label='Posição final')
        ax.scatter(*self.p_des, color='black', s=200, marker='X',
                   zorder=5, label=f'Alvo IK ({self.p_des[0]:.3f},{self.p_des[1]:.3f},{self.p_des[2]:.3f})')
        # Linha do erro final
        p_final = xyz_s[-1]
        ax.plot([p_final[0], self.p_des[0]],
                [p_final[1], self.p_des[1]],
                [p_final[2], self.p_des[2]],
                'r--', linewidth=1.5,
                label=f'Erro: {np.linalg.norm(p_final-self.p_des)*1000:.2f}mm')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.set_zlabel('Z (m)', fontsize=10)
        ax.set_title(
            f'Trajetória 3D — Cinemática Inversa\nAlvo: {self.label}',
            fontsize=12, fontweight='bold'
        )
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        p3 = os.path.join(self.out_dir, f'ik_comparison_3d_{self.label}.png')
        plt.savefig(p3, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'[IK Comparison] Salvo: {p3}')

        self.get_logger().info(
            f'[IK Comparison] ✓ 3 gráficos gerados em {self.out_dir}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = IKComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[IK Comparison] Encerrando — gerando gráficos...')
        node.generate_plots()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
