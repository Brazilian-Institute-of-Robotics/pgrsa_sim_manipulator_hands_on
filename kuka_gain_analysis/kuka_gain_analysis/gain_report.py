"""
gain_report.py — Gerador de relatório de análise de ganhos

Lê o CSV produzido pelo gain_sweep e gera relatório textual com:
- Tabela de desempenho por (Kp, Kd)
- Identificação da região estável
- Recomendação de ganhos ótimos
- Critério de Ziegler-Nichols estimado
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import csv
import json
import os


class GainReport(Node):

    def __init__(self):
        super().__init__('gain_report')

        self.declare_parameter('input_csv', '/tmp/gain_sweep_results.csv')
        self.declare_parameter('publish_interval', 10.0)

        self.input_csv = self.get_parameter('input_csv').value
        self.sweep_finished = False

        self.pub_report = self.create_publisher(
            String, '/kuka_gain_analysis/gain_report', 10)

        # Assina o status do sweep para detectar quando terminar
        self.sub_status = self.create_subscription(
            String, '/kuka_gain_analysis/sweep_status', self._cb_status, 10)

        self.timer = self.create_timer(
            self.get_parameter('publish_interval').value,
            self._try_generate
        )

        self.get_logger().info(
            f'[GainReport] Aguardando dados em {self.input_csv}'
        )

    def _cb_status(self, msg: String):
        data = json.loads(msg.data)
        # Quando o sweep terminar (test == total E estado DONE), gera relatório.
        # CORREÇÃO: antes era criado um create_timer(5.0, ...) a cada mensagem,
        # sem cancelamento — os timers se acumulavam. Agora apenas marca a flag;
        # o timer periódico já existente cuida da geração.
        if data.get('state') == 'DONE' or data.get('test') == data.get('total'):
            if not self.sweep_finished:
                self.sweep_finished = True
                self.get_logger().info(
                    '[GainReport] Sweep concluído — gerando relatório.'
                )

    def _try_generate(self):
        if not os.path.exists(self.input_csv):
            self.get_logger().info('[GainReport] CSV ainda não disponível...')
            return

        results = []
        with open(self.input_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                results.append({
                    'kp':           float(row['kp']),
                    'kd':           float(row['kd']),
                    'rms_error':    float(row['rms_error']),
                    'max_error':    float(row['max_error']),
                    'final_error':  float(row['final_error']),
                    'overshoot':    float(row['overshoot']),
                    'settling_time': float(row['settling_time']),
                    'unstable':     str(row['unstable']).strip().lower() == 'true',
                    # Coluna nova (gain_sweep com SetParameters). CSVs antigos
                    # não a possuem — assume None = "desconhecido".
                    'gains_applied': (
                        str(row['gains_applied']).strip().lower() == 'true'
                        if 'gains_applied' in row and row['gains_applied'] != ''
                        else None
                    ),
                })

        if not results:
            return

        report_lines = []
        report_lines.append('=' * 70)
        report_lines.append('  RELATÓRIO DE ANÁLISE DE GANHOS — KUKA KR70 R2100')
        report_lines.append('=' * 70)

        # Aviso se os ganhos não chegaram ao controlador
        n_applied = sum(1 for r in results if r['gains_applied'] is True)
        if n_applied == 0 and any(r['gains_applied'] is False for r in results):
            report_lines.append(
                '\n  ⚠ ATENÇÃO: nenhum teste teve os ganhos efetivamente aplicados\n'
                '    ao controlador. As diferenças entre combinações refletem apenas\n'
                '    ruído da simulação, não o efeito de Kp/Kd. Verifique se o\n'
                '    JointTrajectoryController usa command_interface de effort:\n'
                '        ros2 param list /kuka_arm_controller | grep gains'
            )

        # Tabela
        report_lines.append(
            f'\n{"Kp":>8} {"Kd":>8} {"RMS":>10} {"Max":>10} '
            f'{"Settle(s)":>10} {"Overshoot":>10} {"Estável":>8} {"Ganhos":>8}'
        )
        report_lines.append('-' * 80)
        for r in sorted(results, key=lambda x: (x['kp'], x['kd'])):
            estavel = 'SIM' if not r['unstable'] else 'NÃO'
            settle = f'{r["settling_time"]:.2f}' if r['settling_time'] > 0 else 'N/A'
            ganhos = {True: 'OK', False: 'NÃO', None: '?'}[r['gains_applied']]
            report_lines.append(
                f'{r["kp"]:>8.1f} {r["kd"]:>8.1f} {r["rms_error"]:>10.5f} '
                f'{r["max_error"]:>10.5f} {settle:>10} '
                f'{r["overshoot"]:>10.3f} {estavel:>8} {ganhos:>8}'
            )

        # Estatísticas
        stable = [r for r in results if not r['unstable']]
        unstable = [r for r in results if r['unstable']]

        report_lines.append('\n' + '=' * 70)
        report_lines.append('  RESUMO')
        report_lines.append('=' * 70)
        report_lines.append(f'  Total testado:    {len(results)}')
        report_lines.append(f'  Estáveis:         {len(stable)} ({100*len(stable)/len(results):.0f}%)')
        report_lines.append(f'  Instáveis:        {len(unstable)} ({100*len(unstable)/len(results):.0f}%)')
        report_lines.append(f'  Ganhos aplicados: {n_applied}/{len(results)}')

        if stable:
            best_rms = min(stable, key=lambda r: r['rms_error'])
            best_settle = min(
                [r for r in stable if r['settling_time'] > 0],
                key=lambda r: r['settling_time'],
                default=None
            )

            report_lines.append(f'\n  Melhor RMS: Kp={best_rms["kp"]} Kd={best_rms["kd"]} → RMS={best_rms["rms_error"]:.5f}')
            if best_settle:
                report_lines.append(
                    f'  Mais rápido: Kp={best_settle["kp"]} Kd={best_settle["kd"]} → '
                    f'settle={best_settle["settling_time"]:.2f}s'
                )

            # Kp crítico estimado (menor Kp instável)
            kp_values_unstable = sorted(set(r['kp'] for r in unstable))
            if kp_values_unstable:
                kp_crit = kp_values_unstable[0]
                # Ziegler-Nichols: Kp_opt = 0.6 * Kpu, Kd_opt = Kp * Tu / 8
                report_lines.append(f'\n  Kp crítico estimado: {kp_crit}')
                report_lines.append(f'  Z-N sugerido: Kp ≈ {0.6*kp_crit:.1f}')

        report_lines.append('\n' + '=' * 70)

        report_text = '\n'.join(report_lines)
        self.get_logger().info(report_text)

        msg = String()
        msg.data = report_text
        self.pub_report.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GainReport()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
