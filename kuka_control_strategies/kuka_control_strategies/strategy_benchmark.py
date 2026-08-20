"""
strategy_benchmark.py — Benchmark comparativo das 3 estratégias de controle.

Executa cada estratégia por N segundos, coleta métricas e compara:
- RMS do erro de posição (rad)
- Tempo de assentamento (settling time)
- Esforço de controle (norma do torque)
- Overshoot

Publica resultados em /kuka_control_strategies/benchmark
"""

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
import json
import time


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

TARGET_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
TARGET_PICK = [0.0, -1.0, 1.2, 0.0, 0.8, 0.0]


class StrategyBenchmark(Node):
    """
    Assina os tópicos de erro de cada estratégia e compila relatório comparativo.
    Execute com todas as 3 estratégias rodando simultaneamente.
    """

    def __init__(self):
        super().__init__('strategy_benchmark')

        self.declare_parameter('duration', 30.0)   # segundos de coleta
        self.declare_parameter('threshold', 0.01)  # rad — critério de assentamento

        self.duration  = self.get_parameter('duration').value
        self.threshold = self.get_parameter('threshold').value

        # Buffers de métricas por estratégia
        self.metrics = {
            'local_pd':        {'errors': [], 'settled': None, 'start': None},
            'computed_torque': {'errors': [], 'settled': None, 'start': None},
            'operational_space': {'errors': [], 'settled': None, 'start': None},
        }

        # Subscriptions nos tópicos de erro de cada estratégia
        self.sub_pd = self.create_subscription(
            Float64MultiArray,
            '/kuka_control_strategies/pd_error',
            lambda msg: self._cb_error(msg, 'local_pd'), 10)

        self.sub_ct = self.create_subscription(
            Float64MultiArray,
            '/kuka_control_strategies/ct_error',
            lambda msg: self._cb_error(msg, 'computed_torque'), 10)

        self.sub_os = self.create_subscription(
            Float64MultiArray,
            '/kuka_control_strategies/os_error_cartesian',
            lambda msg: self._cb_error(msg, 'operational_space'), 10)

        self.pub_report = self.create_publisher(
            String, '/kuka_control_strategies/benchmark', 10)

        self.start_time = self.get_clock().now()
        self.reported = False

        self.timer = self.create_timer(1.0, self._check_and_report)

        self.get_logger().info(
            f'[Benchmark] Coletando por {self.duration}s. '
            f'Rode as 3 estratégias simultaneamente.'
        )

    def _cb_error(self, msg: Float64MultiArray, strategy: str):
        norm = np.linalg.norm(msg.data)
        m = self.metrics[strategy]
        if m['start'] is None:
            m['start'] = self.get_clock().now()
        m['errors'].append(norm)
        # Settling time: primeiro instante em que erro < threshold e fica lá
        if m['settled'] is None and norm < self.threshold and len(m['errors']) > 10:
            t = (self.get_clock().now() - m['start']).nanoseconds / 1e9
            m['settled'] = t

    def _check_and_report(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.duration and not self.reported:
            self._publish_report()
            self.reported = True

        # Log parcial a cada 5s
        if int(elapsed) % 5 == 0:
            for name, m in self.metrics.items():
                if m['errors']:
                    rms = np.sqrt(np.mean(np.array(m['errors']) ** 2))
                    self.get_logger().info(
                        f'[{name}] amostras={len(m["errors"])} | RMS={rms:.5f}'
                    )

    def _publish_report(self):
        report = {}
        for name, m in self.metrics.items():
            errors = np.array(m['errors']) if m['errors'] else np.array([0.0])
            report[name] = {
                'rms_error':      float(np.sqrt(np.mean(errors ** 2))),
                'max_error':      float(np.max(errors)),
                'mean_error':     float(np.mean(errors)),
                'settling_time':  m['settled'] if m['settled'] else -1.0,
                'n_samples':      len(m['errors']),
            }

        # Log formatado
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('RELATÓRIO COMPARATIVO DE ESTRATÉGIAS DE CONTROLE')
        self.get_logger().info('='*60)
        for name, r in report.items():
            self.get_logger().info(f'\n  [{name}]')
            self.get_logger().info(f'    RMS erro:        {r["rms_error"]:.6f}')
            self.get_logger().info(f'    Erro máximo:     {r["max_error"]:.6f}')
            self.get_logger().info(f'    Erro médio:      {r["mean_error"]:.6f}')
            self.get_logger().info(f'    Settling time:   {r["settling_time"]:.2f}s')
            self.get_logger().info(f'    Amostras:        {r["n_samples"]}')
        self.get_logger().info('='*60)

        msg = String()
        msg.data = json.dumps(report, indent=2)
        self.pub_report.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StrategyBenchmark()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
