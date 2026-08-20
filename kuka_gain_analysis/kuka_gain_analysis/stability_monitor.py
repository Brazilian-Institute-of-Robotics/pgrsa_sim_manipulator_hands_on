"""
stability_monitor.py — Monitor de estabilidade em tempo real

Monitora sinais de instabilidade no sistema de controle:
- Divergência do erro de posição
- Oscilações de alta frequência (FFT do erro)
- Saturação de torque
- Cruzamentos de zero excessivos (chatter)

Publica alertas em /kuka_gain_analysis/stability_alert
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from collections import deque
import json


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class StabilityMonitor(Node):
    """
    Monitora estabilidade do controlador ativo em tempo real.
    Detecta: divergência, oscilação, saturação, chatter.
    """

    def __init__(self):
        super().__init__('stability_monitor')

        self.declare_parameter('window_size', 200)        # amostras para análise
        self.declare_parameter('diverge_factor', 2.0)     # erro > X * inicial = divergência
        self.declare_parameter('osc_threshold', 5)        # cruzamentos de zero por janela
        self.declare_parameter('rate_hz', 50.0)

        self.window = self.get_parameter('window_size').value
        self.diverge_factor = self.get_parameter('diverge_factor').value
        self.osc_threshold = self.get_parameter('osc_threshold').value
        self.rate_hz = self.get_parameter('rate_hz').value

        self.q  = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}

        # Janelas deslizantes de erro
        self.error_window = deque(maxlen=self.window)
        self.vel_window   = deque(maxlen=self.window)

        self.q_target = np.array([0.0, -1.0, 1.2, 0.0, 0.8, 0.0])
        self.initial_error = None

        # Contadores de alertas
        self.alert_counts = {
            'divergence':  0,
            'oscillation': 0,
            'chatter':     0,
        }

        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        self.pub_alert = self.create_publisher(
            String, '/kuka_gain_analysis/stability_alert', 10)

        self.pub_metrics = self.create_publisher(
            Float64MultiArray, '/kuka_gain_analysis/stability_metrics', 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._monitor_loop)

        self.get_logger().info('[StabilityMonitor] Iniciado.')

    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q[j]  = msg.position[idx]
                self.qd[j] = msg.velocity[idx] if msg.velocity else 0.0

    def _monitor_loop(self):
        e = self.q_target - self.q
        norm_e = float(np.linalg.norm(e))

        if self.initial_error is None:
            self.initial_error = max(norm_e, 1e-3)

        self.error_window.append(norm_e)
        self.vel_window.append(float(np.linalg.norm(self.qd)))

        alerts = []

        if len(self.error_window) >= self.window:
            errors = np.array(self.error_window)

            # 1. Divergência: erro atual > fator * erro inicial
            if norm_e > self.diverge_factor * self.initial_error and norm_e > 0.1:
                self.alert_counts['divergence'] += 1
                alerts.append({
                    'type': 'DIVERGENCE',
                    'severity': 'HIGH',
                    'message': f'Erro divergindo: {norm_e:.4f} > {self.diverge_factor}x inicial ({self.initial_error:.4f})',
                    'value': norm_e,
                })

            # 2. Oscilação: FFT do erro — energia em altas frequências
            fft = np.abs(np.fft.rfft(errors - errors.mean()))
            freqs = np.fft.rfftfreq(len(errors), d=1.0/self.rate_hz)
            high_freq_energy = float(np.sum(fft[freqs > 2.0]))
            total_energy = float(np.sum(fft)) + 1e-9
            if high_freq_energy / total_energy > 0.4:
                self.alert_counts['oscillation'] += 1
                alerts.append({
                    'type': 'OSCILLATION',
                    'severity': 'MEDIUM',
                    'message': f'Alta energia em freq > 2Hz: {high_freq_energy/total_energy:.2%}',
                    'value': high_freq_energy / total_energy,
                })

            # 3. Chatter: muitos cruzamentos de zero na derivada do erro
            d_errors = np.diff(errors)
            zero_crossings = int(np.sum(np.diff(np.sign(d_errors)) != 0))
            if zero_crossings > self.osc_threshold * 2:
                self.alert_counts['chatter'] += 1
                alerts.append({
                    'type': 'CHATTER',
                    'severity': 'MEDIUM',
                    'message': f'Chatter detectado: {zero_crossings} cruzamentos/janela',
                    'value': zero_crossings,
                })

            # Publica métricas numéricas
            m_msg = Float64MultiArray()
            m_msg.data = [
                norm_e,
                float(np.std(errors)),
                high_freq_energy / total_energy,
                float(zero_crossings),
                float(np.mean(np.array(self.vel_window))),
            ]
            self.pub_metrics.publish(m_msg)

        # Publica alertas
        if alerts:
            alert_msg = String()
            alert_msg.data = json.dumps({
                'alerts': alerts,
                'counts': self.alert_counts,
                'current_error': norm_e,
            })
            self.pub_alert.publish(alert_msg)
            for a in alerts:
                self.get_logger().warn(f'[{a["type"]}] {a["message"]}')

        # Log normal a cada 5s
        if len(self.error_window) % int(self.rate_hz * 5) == 0:
            self.get_logger().info(
                f'[StabilityMonitor] ||e||={norm_e:.4f} | '
                f'alertas: {self.alert_counts}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StabilityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
