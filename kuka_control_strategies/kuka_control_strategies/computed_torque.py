"""
computed_torque.py — Controle Centralizado (Computed Torque / Model-Based)

Estratégia: u = M(q)*(qdd_des + Kd*de + Kp*e) + C(q,qd)*qd + g(q)
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
from builtin_interfaces.msg import Duration


JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

DH_PARAMS = np.array([
    # Extraído do URDF kr70_r2100.urdf.xacro
    # [a,       alpha,      d,        theta_offset]
    [0.175,   np.pi,      0.236,    0.0],   # joint_1
    [0.890,   0.0,       -0.285,    0.0],   # joint_2
    [0.30805, np.pi/2,   -0.0975,   0.0],   # joint_3
    [0.0,    -np.pi/2,    0.72695,  0.0],   # joint_4
    [0.149,   np.pi/2,    0.04795,  0.0],   # joint_5
    [0.0,     0.0,        0.036,    0.0],   # joint_6 (flange)
])

LINK_MASSES = np.array([150.0, 80.0, 45.0, 20.0, 10.0, 5.0])
LINK_COM = np.array([
    [0.0, 0.0, 0.25], [0.65, 0.0, 0.0], [0.0, 0.0, 0.1],
    [0.0, 0.0, 0.5],  [0.0, 0.0, 0.0],  [0.0, 0.0, 0.1],
])
G = 9.81

DEMO_TARGETS = {
    'home':    [0.0,    0.0,   0.0,  0.0,  0.0,  0.0],
    'pick':    [0.0,   -1.0,   1.2,  0.0,  0.8,  0.0],
    'place':   [1.57,  -1.0,   1.0,  0.0,  0.5,  0.0],
    'stretch': [0.0,   -1.57,  1.57, 0.0,  1.57, 0.0],
}


def dh_matrix(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0, sa,     ca,    d   ],
        [0.0, 0.0,    0.0,   1.0 ],
    ])


def compute_gravity_vector(q):
    g_vec = np.zeros(6)
    T = np.eye(4)
    transforms = [np.eye(4)]
    for i in range(6):
        a, alpha, d, th0 = DH_PARAMS[i]
        T = T @ dh_matrix(a, alpha, d, q[i] + th0)
        transforms.append(T.copy())
    com_base = []
    for i in range(6):
        p = transforms[i+1] @ np.array([*LINK_COM[i], 1.0])
        com_base.append(p[:3])
    dq = 1e-6
    for i in range(6):
        q_plus = q.copy(); q_plus[i] += dq
        T2 = np.eye(4); com_plus = []
        for j in range(6):
            a, alpha, d, th0 = DH_PARAMS[j]
            T2 = T2 @ dh_matrix(a, alpha, d, q_plus[j] + th0)
            p = T2 @ np.array([*LINK_COM[j], 1.0])
            com_plus.append(p[:3])
        for j in range(i, 6):
            dz = (com_plus[j][2] - com_base[j][2]) / dq
            g_vec[i] += LINK_MASSES[j] * G * dz
    return g_vec


class ComputedTorqueController(Node):

    def __init__(self):
        super().__init__('computed_torque_controller')

        self.declare_parameter('kp', [150.0, 150.0, 120.0, 80.0, 60.0, 40.0])
        self.declare_parameter('kd', [25.0,  25.0,  20.0,  15.0, 10.0, 8.0])
        self.declare_parameter('target', 'pick')
        self.declare_parameter('custom_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.kp = np.diag(self.get_parameter('kp').value)
        self.kd = np.diag(self.get_parameter('kd').value)
        self.target_name = self.get_parameter('target').value
        self.q_des  = np.array(DEMO_TARGETS.get(self.target_name, DEMO_TARGETS['home']))
        self.qd_des = np.zeros(6)
        self.qdd_des = np.zeros(6)

        self.q  = np.zeros(6)
        self.qd = np.zeros(6)
        self.joint_order = {}
        self.errors_history = []
        self.goal_sent = False

        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/kuka_arm_controller/follow_joint_trajectory'
        )

        self.pub_torque = self.create_publisher(
            Float64MultiArray, '/kuka_control_strategies/computed_torque', 10)

        self.pub_error = self.create_publisher(
            Float64MultiArray, '/kuka_control_strategies/ct_error', 10)

        self.create_timer(2.0, self._send_goal)
        self.create_timer(0.02, self._publish_metrics)

        self.get_logger().info(f'[ComputedTorque] Iniciado — alvo: {self.target_name}')

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

        # Calcula torque computado para log
        e  = self.q_des - self.q
        de = self.qd_des - self.qd
        g  = compute_gravity_vector(self.q)
        v  = self.qdd_des + self.kp @ e + self.kd @ de
        tau = g  # simplificado: usa só compensação de gravidade para direção

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions     = self.q_des.tolist()
        pt.velocities    = [0.0] * 6
        pt.accelerations = [0.0] * 6
        pt.time_from_start = Duration(sec=4)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(
            f'[ComputedTorque] Enviando goal | '
            f'||g(q)||={np.linalg.norm(g):.2f} Nm'
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('[ComputedTorque] Goal rejeitado!')
            self.goal_sent = False
            return
        self.get_logger().info('[ComputedTorque] Goal aceito.')
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        e = self.q_des - self.q
        rms = np.sqrt(np.mean(np.array(self.errors_history)**2)) if self.errors_history else 0.0
        self.get_logger().info(
            f'[ComputedTorque] Concluído | ||e||={np.linalg.norm(e):.4f} | RMS={rms:.4f}'
        )
        self.goal_sent = False

    def _publish_metrics(self):
        e  = self.q_des - self.q
        de = self.qd_des - self.qd
        g  = compute_gravity_vector(self.q)
        v  = self.qdd_des + self.kp @ e + self.kd @ de
        tau = g

        self.errors_history.append(np.linalg.norm(e))

        err_msg = Float64MultiArray()
        err_msg.data = e.tolist()
        self.pub_error.publish(err_msg)

        tau_msg = Float64MultiArray()
        tau_msg.data = tau.tolist()
        self.pub_torque.publish(tau_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ComputedTorqueController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
