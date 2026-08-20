"""
gain_sweep.py — Varrimento automático de ganhos Kp/Kd

Itera sobre combinações de (Kp, Kd), APLICA cada configuração ao controlador
ativo via rcl_interfaces/SetParameters, executa um degrau home → pick,
coleta métricas e salva CSV com resultados.

Fluxo por combinação:
  1. SET_GAINS — aplica (Kp, Kd) nos parâmetros do JointTrajectoryController
                 e aguarda a confirmação do serviço
  2. RUNNING   — envia o degrau e coleta erro por test_duration segundos
  3. HOMING    — retorna a home e aguarda home_settle segundos
  4. próxima combinação

O nó é uma máquina de estados dirigida por UM único timer. A versão anterior
criava um timer novo a cada teste (create_timer dentro de _finish_test) que
nunca era cancelado — o resultado era o reinício de testes já concluídos e a
reescrita repetida do CSV. Aqui não há criação dinâmica de timers.

IMPORTANTE — pré-requisito para que a varredura tenha efeito físico:
  O JointTrajectoryController só expõe (e só usa) os parâmetros de ganho
  'gains.<joint>.p' / '.d' quando está configurado com command_interface de
  effort ou velocity (malha fechada interna). Com command_interface de
  position, o hardware/Gazebo fecha a malha internamente, os parâmetros de
  ganho não existem e a varredura NÃO altera o comportamento do robô.
  Verifique com:
      ros2 param list /kuka_arm_controller | grep gains
  Se nada aparecer, ajuste o YAML do controlador para effort antes de rodar.
  O nó detecta essa situação, avisa e continua em modo somente-observação,
  marcando gains_applied=False em cada linha do CSV.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, String
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import json
import csv
import os


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

TARGET_HOME = np.array([0.0,   0.0,  0.0, 0.0,  0.0, 0.0])
TARGET_PICK = np.array([0.0,  -1.0,  1.2, 0.0,  0.8, 0.0])


class GainSweep(Node):
    """
    Varrimento de ganhos: aplica cada par (Kp, Kd) ao controlador e coleta:
    - RMS do erro de posição
    - Settling time
    - Overshoot máximo
    - Indicador de instabilidade (erro divergindo)
    """

    # ── Estados da máquina ────────────────────────────────────────────────
    ST_INIT      = 'INIT'        # espera inicial (Gazebo estabilizar)
    ST_SET_GAINS = 'SET_GAINS'   # aplicando ganhos via SetParameters
    ST_RUNNING   = 'RUNNING'     # degrau em execução, coletando dados
    ST_HOMING    = 'HOMING'      # retornando a home entre testes
    ST_DONE      = 'DONE'        # varredura concluída

    def __init__(self):
        super().__init__('gain_sweep')

        self.declare_parameter('kp_values', [50.0, 100.0, 200.0, 400.0, 800.0])
        self.declare_parameter('kd_values', [5.0,  10.0,  20.0,  40.0,  80.0])
        self.declare_parameter('test_duration', 8.0)      # segundos por combinação
        self.declare_parameter('settle_threshold', 0.02)  # rad
        self.declare_parameter('output_csv', '/tmp/gain_sweep_results.csv')

        # Reconfiguração dinâmica de ganhos
        self.declare_parameter('controller_name', 'kuka_arm_controller')
        self.declare_parameter('apply_gains', True)       # False = só observa
        self.declare_parameter('gain_service_timeout', 5.0)
        self.declare_parameter('startup_delay', 3.0)      # espera antes do 1º teste
        self.declare_parameter('home_settle', 3.0)        # espera após voltar a home

        self.kp_values        = self.get_parameter('kp_values').value
        self.kd_values        = self.get_parameter('kd_values').value
        self.test_duration    = self.get_parameter('test_duration').value
        self.settle_threshold = self.get_parameter('settle_threshold').value
        self.output_csv       = self.get_parameter('output_csv').value
        self.controller_name  = self.get_parameter('controller_name').value
        self.apply_gains      = self.get_parameter('apply_gains').value
        self.svc_timeout      = self.get_parameter('gain_service_timeout').value
        self.startup_delay    = self.get_parameter('startup_delay').value
        self.home_settle      = self.get_parameter('home_settle').value

        # Estado dos joints
        self.q  = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}

        # Estado do teste atual
        self.current_kp = None
        self.current_kd = None
        self.errors_buffer = []
        self.peak_error = 0.0
        self.initial_error = None
        self.settled_time = None
        self.unstable = False
        self.gains_ok = False          # ganhos confirmados para o teste atual

        # Máquina de estados
        self.state = self.ST_INIT
        self.state_since = self.get_clock().now()
        self.gain_future = None

        # Resultados acumulados
        self.results = []

        self.combinations = [
            (kp, kd)
            for kp in self.kp_values
            for kd in self.kd_values
        ]
        self.combo_idx = 0
        self.q_des = TARGET_PICK
        self.q_home = TARGET_HOME

        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        self.pub_traj = self.create_publisher(
            JointTrajectory, f'/{self.controller_name}/joint_trajectory', 10)

        self.pub_metrics = self.create_publisher(
            Float64MultiArray, '/kuka_gain_analysis/current_metrics', 10)

        self.pub_status = self.create_publisher(
            String, '/kuka_gain_analysis/sweep_status', 10)

        # ── Cliente do serviço de reconfiguração ──────────────────────────
        self.cli_set_params = self.create_client(
            SetParameters, f'/{self.controller_name}/set_parameters')
        self.cli_list_params = self.create_client(
            ListParameters, f'/{self.controller_name}/list_parameters')

        self.gain_params_available = False
        if self.apply_gains:
            self._probe_gain_parameters()
        else:
            self.get_logger().warn(
                '[GainSweep] apply_gains=False — modo somente-observação. '
                'Os ganhos varridos NÃO serão aplicados ao controlador.'
            )

        # ÚNICO timer da classe: 20 Hz, despacha conforme o estado
        self.timer = self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f'[GainSweep] {len(self.combinations)} combinações para testar. '
            f'Duração por teste: {self.test_duration}s | '
            f'aplicação de ganhos: '
            f'{"ATIVA" if self.gain_params_available else "INATIVA"}'
        )

    # ── Reconfiguração de ganhos ──────────────────────────────────────────
    def _probe_gain_parameters(self):
        """
        Verifica se o controlador expõe parâmetros 'gains.<joint>.p/.d'.
        Sem eles (command_interface=position), a varredura não tem efeito
        físico e o nó segue em modo somente-observação.
        """
        if not self.cli_set_params.wait_for_service(timeout_sec=self.svc_timeout):
            self.get_logger().error(
                f'[GainSweep] Serviço /{self.controller_name}/set_parameters '
                f'indisponível após {self.svc_timeout}s. '
                f'Seguindo em modo somente-observação.'
            )
            return

        if not self.cli_list_params.wait_for_service(timeout_sec=self.svc_timeout):
            self.get_logger().warn(
                '[GainSweep] list_parameters indisponível — assumindo que os '
                'parâmetros de ganho existem e tentando aplicá-los mesmo assim.'
            )
            self.gain_params_available = True
            return

        req = ListParameters.Request()
        req.prefixes = ['gains']
        req.depth = 0
        future = self.cli_list_params.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.svc_timeout)

        names = list(future.result().result.names) if future.result() else []
        expected = [f'gains.{j}.p' for j in JOINT_NAMES]
        found = [n for n in names if n in expected]

        if found:
            self.gain_params_available = True
            self.get_logger().info(
                f'[GainSweep] Parâmetros de ganho detectados em '
                f'/{self.controller_name} ({len(found)}/6 juntas). '
                f'Reconfiguração dinâmica ATIVA.'
            )
        else:
            self.get_logger().error(
                f'[GainSweep] /{self.controller_name} não expõe parâmetros '
                f'gains.<joint>.p — o controlador provavelmente usa '
                f'command_interface=position. A varredura NÃO alterará o '
                f'comportamento do robô. Ajuste o YAML do controlador para '
                f'effort/velocity para obter resultados significativos.'
            )

    def _build_gain_request(self, kp: float, kd: float) -> SetParameters.Request:
        """Monta o request com gains.<joint>.p e gains.<joint>.d para as 6 juntas."""
        req = SetParameters.Request()
        for joint in JOINT_NAMES:
            for suffix, value in (('p', kp), ('d', kd)):
                p = Parameter()
                p.name = f'gains.{joint}.{suffix}'
                p.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(value),
                )
                req.parameters.append(p)
        return req

    def _request_gains(self, kp: float, kd: float):
        """Dispara o SetParameters de forma assíncrona (não bloqueia o timer)."""
        if not self.gain_params_available:
            self.gains_ok = False
            return None
        req = self._build_gain_request(kp, kd)
        return self.cli_set_params.call_async(req)

    def _check_gain_future(self) -> bool:
        """
        Consulta o future do SetParameters. Retorna True quando resolvido
        (com sucesso ou não) e o teste pode prosseguir.
        """
        if self.gain_future is None:
            return True

        if not self.gain_future.done():
            # Timeout de segurança para não travar a varredura
            if self._elapsed_in_state() > self.svc_timeout:
                self.get_logger().warn(
                    f'[GainSweep] SetParameters não respondeu em '
                    f'{self.svc_timeout}s — prosseguindo sem confirmação.'
                )
                self.gains_ok = False
                self.gain_future = None
                return True
            return False

        response = self.gain_future.result()
        self.gain_future = None

        if response is None:
            self.get_logger().error('[GainSweep] SetParameters falhou (resposta nula).')
            self.gains_ok = False
            return True

        failures = [r for r in response.results if not r.successful]
        if failures:
            self.gains_ok = False
            self.get_logger().error(
                f'[GainSweep] {len(failures)}/{len(response.results)} parâmetros '
                f'rejeitados. Primeiro motivo: "{failures[0].reason}"'
            )
        else:
            self.gains_ok = True
            self.get_logger().info(
                f'[GainSweep] Ganhos aplicados: Kp={self.current_kp} '
                f'Kd={self.current_kd} '
                f'({len(response.results)} parâmetros confirmados)'
            )
        return True

    # ── Callbacks e utilitários ───────────────────────────────────────────
    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q[j]  = msg.position[idx]
                self.qd[j] = msg.velocity[idx] if msg.velocity else 0.0

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _set_state(self, state: str):
        self.state = state
        self.state_since = self.get_clock().now()

    # ── Máquina de estados (único timer) ──────────────────────────────────
    def _loop(self):
        if self.state == self.ST_INIT:
            if self._elapsed_in_state() >= self.startup_delay:
                self._begin_combination()

        elif self.state == self.ST_SET_GAINS:
            if self._check_gain_future():
                self._start_step()

        elif self.state == self.ST_RUNNING:
            self._collect()

        elif self.state == self.ST_HOMING:
            if self._elapsed_in_state() >= self.home_settle:
                self.combo_idx += 1
                self._begin_combination()

        # ST_DONE: nada a fazer

    def _begin_combination(self):
        """Inicia a próxima combinação: aplica os ganhos e vai para SET_GAINS."""
        if self.combo_idx >= len(self.combinations):
            self._save_results()
            self._set_state(self.ST_DONE)
            self.get_logger().info('[GainSweep] Varredura concluída.')
            return

        kp, kd = self.combinations[self.combo_idx]
        self.current_kp = kp
        self.current_kd = kd
        self.gains_ok = False

        self.get_logger().info(
            f'[GainSweep] Teste {self.combo_idx+1}/{len(self.combinations)}: '
            f'Kp={kp} Kd={kd} — aplicando ganhos...'
        )

        status = String()
        status.data = json.dumps({
            'test': self.combo_idx + 1,
            'total': len(self.combinations),
            'kp': kp, 'kd': kd,
            'state': self.ST_SET_GAINS,
        })
        self.pub_status.publish(status)

        self.gain_future = self._request_gains(kp, kd)
        self._set_state(self.ST_SET_GAINS)

    def _start_step(self):
        """Zera os buffers e envia o degrau home → pick."""
        self.errors_buffer = []
        self.peak_error = 0.0
        self.initial_error = None
        self.settled_time = None
        self.unstable = False

        self._send_trajectory(self.q_des, sec=1)
        self._set_state(self.ST_RUNNING)

    def _collect(self):
        """Coleta métricas do teste corrente (20 Hz)."""
        elapsed = self._elapsed_in_state()

        e = self.q_des - self.q
        norm_e = float(np.linalg.norm(e))

        if self.initial_error is None:
            self.initial_error = norm_e

        self.errors_buffer.append(norm_e)
        self.peak_error = max(self.peak_error, norm_e)

        if self.settled_time is None and norm_e < self.settle_threshold:
            self.settled_time = elapsed

        # Instabilidade: erro > 3x o inicial E crescendo
        if (len(self.errors_buffer) > 50 and
                norm_e > 3 * (self.initial_error or 1.0) and
                norm_e > self.errors_buffer[-50]):
            if not self.unstable:
                self.get_logger().warn(
                    f'[GainSweep] INSTÁVEL! Kp={self.current_kp} Kd={self.current_kd}'
                )
            self.unstable = True

        m_msg = Float64MultiArray()
        m_msg.data = [
            float(self.current_kp), float(self.current_kd),
            norm_e, float(elapsed),
            1.0 if self.unstable else 0.0,
        ]
        self.pub_metrics.publish(m_msg)

        if elapsed >= self.test_duration:
            self._finish_test()

    def _finish_test(self):
        """Consolida as métricas e manda o robô de volta a home."""
        errors = np.array(self.errors_buffer)
        rms = float(np.sqrt(np.mean(errors ** 2))) if len(errors) > 0 else -1.0

        final_error = (float(errors[-10:].mean()) if len(errors) >= 10
                       else float(errors[-1]))
        overshoot = float(
            (self.peak_error - final_error) / max(self.initial_error or 1.0, 1e-6)
        )

        result = {
            'kp':            self.current_kp,
            'kd':            self.current_kd,
            'rms_error':     rms,
            'max_error':     float(self.peak_error),
            'final_error':   final_error,
            'overshoot':     max(0.0, overshoot),
            'settling_time': self.settled_time if self.settled_time else -1.0,
            'unstable':      self.unstable,
            'gains_applied': self.gains_ok,
        }
        self.results.append(result)

        settle_txt = (f'{self.settled_time:.2f}s'
                      if self.settled_time is not None else 'N/A')
        self.get_logger().info(
            f'[GainSweep] Resultado: Kp={self.current_kp} Kd={self.current_kd} | '
            f'RMS={rms:.5f} | settled={settle_txt} | '
            f'unstable={self.unstable} | gains_applied={self.gains_ok}'
        )

        self._send_trajectory(self.q_home, sec=3)
        self._set_state(self.ST_HOMING)

    def _send_trajectory(self, positions: np.ndarray, sec: int):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions.tolist()
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=sec)
        traj.points = [pt]
        self.pub_traj.publish(traj)

    # ── Persistência e resumo ─────────────────────────────────────────────
    def _save_results(self):
        if not self.results:
            self.get_logger().warn('[GainSweep] Nenhum resultado para salvar.')
            return

        out_dir = os.path.dirname(self.output_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        fieldnames = list(self.results[0].keys())
        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.results)

        self.get_logger().info(f'[GainSweep] Resultados salvos em: {self.output_csv}')

        if not any(r['gains_applied'] for r in self.results):
            self.get_logger().warn(
                '[GainSweep] ATENÇÃO: nenhum teste teve os ganhos efetivamente '
                'aplicados. As diferenças entre combinações refletem apenas ruído '
                'da simulação, não o efeito de Kp/Kd.'
            )

        stable = [r for r in self.results if not r['unstable']]
        if stable:
            best = min(stable, key=lambda r: r['rms_error'])
            settle_txt = (f'{best["settling_time"]:.2f}s'
                          if best['settling_time'] > 0 else 'N/A')
            self.get_logger().info(
                f'[GainSweep] Melhor: Kp={best["kp"]} Kd={best["kd"]} '
                f'RMS={best["rms_error"]:.5f} settled={settle_txt}'
            )

        # Status final para o gain_report
        status = String()
        status.data = json.dumps({
            'test': len(self.combinations),
            'total': len(self.combinations),
            'kp': self.current_kp, 'kd': self.current_kd,
            'state': self.ST_DONE,
        })
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = GainSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[GainSweep] Interrompido — salvando parciais...')
        node._save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
