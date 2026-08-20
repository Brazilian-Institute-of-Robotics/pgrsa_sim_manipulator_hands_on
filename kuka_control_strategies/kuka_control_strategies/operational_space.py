"""
operational_space.py — Controle no Espaço Operacional (Task-Space)

Controla diretamente posição do end-effector via Jacobiano.
Usa action server do JTC para enviar trajetória.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

from kuka_control_strategies.computed_torque import dh_matrix, compute_gravity_vector, DH_PARAMS


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Alvos cartesianos calculados via FK com DH corretos do KR70 R2100
CARTESIAN_TARGETS = {
    'home':    np.array([1.522,  0.763,  0.5705]),  # q=[0,0,0,0,0,0]
    'pick':    np.array([1.213,  1.294,  0.5705]),  # q=[0,-1,1.2,0,0.8,0]
    'stretch': np.array([0.520,  1.468,  0.5705]),  # q=[0,-1.57,1.57,0,1.57,0]
    'center':  np.array([1.213,  1.294,  0.5705]),  # mesmo que pick
    'left':    np.array([1.213,  1.294,  0.5705]),
}

# Posição inicial correspondente para cada alvo cartesiano
JOINT_SEEDS = {
    'home':    [0.0,   0.0,   0.0,  0.0,  0.0,  0.0],
    'pick':    [0.0,  -1.0,   1.2,  0.0,  0.8,  0.0],
    'stretch': [0.0,  -1.57,  1.57, 0.0,  1.57, 0.0],
    'center':  [0.0,  -1.0,   1.2,  0.0,  0.8,  0.0],
    'left':    [0.0,  -1.0,   1.2,  0.0,  0.8,  0.0],
}


def forward_kinematics(q):
    T = np.eye(4)
    transforms = [np.eye(4)]
    for i in range(6):
        a, alpha, d, th0 = DH_PARAMS[i]
        T = T @ dh_matrix(a, alpha, d, q[i] + th0)
        transforms.append(T.copy())
    return T, transforms


def geometric_jacobian(q, transforms):
    J = np.zeros((6, 6))
    p_n = transforms[6][:3, 3]
    for i in range(6):
        z_i = transforms[i][:3, 2]
        p_i = transforms[i][:3, 3]
        J[:3, i] = np.cross(z_i, p_n - p_i)
        J[3:, i] = z_i
    return J


class OperationalSpaceController(Node):

    def __init__(self):
        super().__init__('operational_space_controller')

        self.declare_parameter('kp_pos', [300.0, 300.0, 300.0])
        self.declare_parameter('kd_pos', [30.0,  30.0,  30.0])
        self.declare_parameter('target', 'center')
        # CORREÇÃO: este parâmetro era lido sem ter sido declarado, o que
        # levantava ParameterNotDeclaredException e derrubava o nó no __init__.
        self.declare_parameter('custom_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.kp_pos = np.array(self.get_parameter('kp_pos').value)
        self.kd_pos = np.array(self.get_parameter('kd_pos').value)
        self.target_name = self.get_parameter('target').value
        custom = self.get_parameter('custom_positions').value
        if self.target_name == 'custom':
            if len(custom) != 6:
                self.get_logger().error(
                    f'[OpSpace] custom_positions precisa ter 6 elementos '
                    f'(recebeu {len(custom)}). Usando alvo "center".'
                )
                self.target_name = 'center'
                self.p_des = CARTESIAN_TARGETS['center']
                self.q_seed = np.array(JOINT_SEEDS['center'])
            else:
                self.q_seed = np.array(custom, dtype=float)
                T_tmp, _ = forward_kinematics(self.q_seed)
                self.p_des = T_tmp[:3, 3]
        else:
            self.p_des = CARTESIAN_TARGETS.get(self.target_name, CARTESIAN_TARGETS['center'])
            self.q_seed = np.array(JOINT_SEEDS.get(self.target_name, JOINT_SEEDS['center']))

        self.q  = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}
        self.pos_errors = []
        self.goal_sent = False

        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/kuka_arm_controller/follow_joint_trajectory'
        )

        self.pub_error = self.create_publisher(
            Float64MultiArray, '/kuka_control_strategies/os_error_cartesian', 10)

        self.pub_tcp = self.create_publisher(
            PoseStamped, '/kuka_control_strategies/tcp_pose', 10)

        self.pub_tau = self.create_publisher(
            Float64MultiArray, '/kuka_control_strategies/os_torque', 10)

        self.create_timer(2.0, self._send_goal)
        self.create_timer(0.02, self._publish_metrics)

        self.get_logger().info(
            f'[OpSpace] Iniciado — alvo: {self.target_name} p_des={self.p_des.tolist()}'
        )

    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q[j]  = msg.position[idx]
                self.qd[j] = msg.velocity[idx] if msg.velocity else 0.0

    def _send_goal(self):
        if self.goal_sent:
            return
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            return
        self.goal_sent = True

        # Usa seed de joints correspondente ao alvo cartesiano
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions     = self.q_seed.tolist()
        pt.velocities    = [0.0] * 6
        pt.accelerations = [0.0] * 6
        pt.time_from_start = Duration(sec=4)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        T, _ = forward_kinematics(self.q_seed)
        p_reach = T[:3, 3]
        self.get_logger().info(
            f'[OpSpace] Goal → joints={self.q_seed.tolist()} | '
            f'TCP estimado=({p_reach[0]:.3f},{p_reach[1]:.3f},{p_reach[2]:.3f})'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('[OpSpace] Goal rejeitado!')
            self.goal_sent = False
            return
        self.get_logger().info('[OpSpace] Goal aceito.')
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        T, _ = forward_kinematics(self.q)
        p_cur = T[:3, 3]
        ep = np.linalg.norm(self.p_des - p_cur)
        self.get_logger().info(
            f'[OpSpace] Concluído | TCP=({p_cur[0]:.3f},{p_cur[1]:.3f},{p_cur[2]:.3f}) | '
            f'||ep||={ep:.4f}m'
        )
        self.goal_sent = False

    def _publish_metrics(self):
        T, transforms = forward_kinematics(self.q)
        p_cur = T[:3, 3]
        J = geometric_jacobian(self.q, transforms)
        ep = self.p_des - p_cur
        self.pos_errors.append(float(np.linalg.norm(ep)))

        err_msg = Float64MultiArray()
        err_msg.data = ep.tolist()
        self.pub_error.publish(err_msg)

        tcp_msg = PoseStamped()
        tcp_msg.header.stamp = self.get_clock().now().to_msg()
        tcp_msg.header.frame_id = 'base_link'
        tcp_msg.pose.position.x = float(p_cur[0])
        tcp_msg.pose.position.y = float(p_cur[1])
        tcp_msg.pose.position.z = float(p_cur[2])
        self.pub_tcp.publish(tcp_msg)

        g = compute_gravity_vector(self.q)

        # CORREÇÃO: antes o ganho era fixo em 300.0 e kp_pos/kd_pos eram
        # declarados mas nunca usados. Agora a lei é PD no espaço operacional:
        #   F = Kp_pos * (p_des - p) - Kd_pos * v_tcp
        #   tau = J^T * F + g(q)
        v_tcp = J[:3, :] @ self.qd          # velocidade linear do TCP
        F_lin = self.kp_pos * ep - self.kd_pos * v_tcp
        F = np.array([*F_lin, 0., 0., 0.])
        tau = J.T @ F + g
        tau_msg = Float64MultiArray()
        tau_msg.data = tau.tolist()
        self.pub_tau.publish(tau_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OperationalSpaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.pos_errors:
            rms = np.sqrt(np.mean(np.array(node.pos_errors)**2))
            node.get_logger().info(f'[OpSpace] RMS final: {rms:.6f} m')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
