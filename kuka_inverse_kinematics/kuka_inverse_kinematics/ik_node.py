"""
ik_node.py — Nó ROS 2 de Cinemática Inversa

Aceita alvos cartesianos via parâmetro ou tópico e:
  1. Resolve a IK com o método escolhido
  2. Envia o resultado ao manipulador via action server
  3. Publica métricas de convergência

Uso:
  ros2 run kuka_inverse_kinematics ik_node --ros-args \
    -p target_x:=1.2 -p target_y:=0.5 -p target_z:=0.8 \
    -p method:=lm -p execute:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import json

from kuka_inverse_kinematics.ik_solver import solve_ik, forward_kinematics, JOINT_NAMES


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        # Alvo cartesiano
        self.declare_parameter('target_x', 1.2)
        self.declare_parameter('target_y', 0.5)
        self.declare_parameter('target_z', 0.8)

        # Orientação desejada (Euler ZYX em graus)
        self.declare_parameter('target_yaw',   0.0)
        self.declare_parameter('target_pitch',  0.0)
        self.declare_parameter('target_roll',   0.0)

        # Controle
        self.declare_parameter('method', 'lm')       # pinv | transpose | lm
        self.declare_parameter('execute', True)       # envia para o robô
        self.declare_parameter('max_iter', 200)
        self.declare_parameter('tol_pos',  1e-4)
        self.declare_parameter('tol_ori',  1e-3)
        self.declare_parameter('move_time', 4.0)     # segundos para o movimento

        # Lê parâmetros
        self.p_des = np.array([
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
            self.get_parameter('target_z').value,
        ])
        yaw   = np.radians(self.get_parameter('target_yaw').value)
        pitch = np.radians(self.get_parameter('target_pitch').value)
        roll  = np.radians(self.get_parameter('target_roll').value)
        self.R_des = self._euler_zyx_to_rotation(yaw, pitch, roll)

        self.method    = self.get_parameter('method').value
        self.execute   = self.get_parameter('execute').value
        self.max_iter  = self.get_parameter('max_iter').value
        self.tol_pos   = self.get_parameter('tol_pos').value
        self.tol_ori   = self.get_parameter('tol_ori').value
        self.move_time = self.get_parameter('move_time').value

        # Estado do robô
        self.q_current = np.zeros(6)
        self.joint_order = {}

        # ROS interfaces
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)

        self.sub_target = self.create_subscription(
            PoseStamped, '/ik/target_pose', self._cb_target, 10)

        self.pub_result  = self.create_publisher(Float64MultiArray, '/ik/joint_solution', 10)
        self.pub_metrics = self.create_publisher(String,            '/ik/convergence_info', 10)
        self.pub_tcp_est = self.create_publisher(PoseStamped,       '/ik/estimated_tcp', 10)

        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/kuka_arm_controller/follow_joint_trajectory'
        )

        # Resolve após 2s (para ter joint_states disponível)
        self.create_timer(2.0, self._solve_and_execute)

        self.get_logger().info(
            f'[IK] Nó iniciado | alvo: ({self.p_des[0]:.3f},{self.p_des[1]:.3f},{self.p_des[2]:.3f}) | '
            f'método: {self.method}'
        )

    def _euler_zyx_to_rotation(self, yaw, pitch, roll) -> np.ndarray:
        """Constrói matriz de rotação a partir de Euler ZYX."""
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw),  np.cos(yaw), 0],
                       [0, 0, 1]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll),  np.cos(roll)]])
        return Rz @ Ry @ Rx

    def _cb_js(self, msg: JointState):
        if not self.joint_order:
            self.joint_order = {name: i for i, name in enumerate(msg.name)}
        for j, name in enumerate(JOINT_NAMES):
            if name in self.joint_order:
                idx = self.joint_order[name]
                self.q_current[j] = msg.position[idx]

    def _cb_target(self, msg: PoseStamped):
        """Aceita alvo via tópico /ik/target_pose."""
        self.p_des = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        # Quaternion → rotação
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.R_des = self._quat_to_rotation(qx, qy, qz, qw)
        self.get_logger().info(
            f'[IK] Novo alvo recebido via tópico: ({self.p_des[0]:.3f},{self.p_des[1]:.3f},{self.p_des[2]:.3f})'
        )
        self._solve_and_execute()

    def _quat_to_rotation(self, x, y, z, w) -> np.ndarray:
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
            [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
            [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])

    def _solve_and_execute(self):
        self.get_logger().info(
            f'[IK] Resolvendo IK ({self.method}) para '
            f'p=({self.p_des[0]:.4f},{self.p_des[1]:.4f},{self.p_des[2]:.4f})...'
        )

        result = solve_ik(
            p_des=self.p_des,
            R_des=self.R_des,
            q_init=self.q_current.copy(),
            method=self.method,
            max_iter=self.max_iter,
            tol_pos=self.tol_pos,
            tol_ori=self.tol_ori,
        )

        q_sol = result['q_result']

        # Verifica resultado via FK
        T_check, _ = forward_kinematics(q_sol)
        p_check = T_check[:3, 3]
        p_error = np.linalg.norm(self.p_des - p_check)

        # Log resultado
        self.get_logger().info(
            f'[IK] {"CONVERGIU" if result["converged"] else "NÃO CONVERGIU"} | '
            f'iterações={result["iterations"]} | '
            f'erro_pos={result["final_error_pos"]*1000:.3f}mm | '
            f'erro_ori={np.degrees(result["final_error_ori"]):.3f}°'
        )
        self.get_logger().info(
            f'[IK] Verificação FK: p_check=({p_check[0]:.4f},{p_check[1]:.4f},{p_check[2]:.4f}) | '
            f'|p_des-p_fk|={p_error*1000:.3f}mm'
        )
        self.get_logger().info(
            f'[IK] Solução (graus): ' +
            ' '.join(f'{np.degrees(q_sol[i]):.2f}°' for i in range(6))
        )

        # Publica resultado
        sol_msg = Float64MultiArray()
        sol_msg.data = q_sol.tolist()
        self.pub_result.publish(sol_msg)

        metrics = {
            'method':      result['method'],
            'converged':   result['converged'],
            'iterations':  result['iterations'],
            'error_pos_mm': result['final_error_pos'] * 1000,
            'error_ori_deg': np.degrees(result['final_error_ori']),
            'q_solution_deg': np.degrees(q_sol).round(3).tolist(),
            'tcp_check': p_check.round(4).tolist(),
        }
        info_msg = String()
        info_msg.data = json.dumps(metrics, indent=2)
        self.pub_metrics.publish(info_msg)

        # Publica TCP estimado
        tcp_msg = PoseStamped()
        tcp_msg.header.stamp = self.get_clock().now().to_msg()
        tcp_msg.header.frame_id = 'world'
        tcp_msg.pose.position.x = float(p_check[0])
        tcp_msg.pose.position.y = float(p_check[1])
        tcp_msg.pose.position.z = float(p_check[2])
        self.pub_tcp_est.publish(tcp_msg)

        # Executa no robô
        if self.execute and result['converged']:
            self._send_to_robot(q_sol)

    def _send_to_robot(self, q_sol: np.ndarray):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('[IK] Action server não disponível.')
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

        self.get_logger().info(f'[IK] Enviando para o robô em {self.move_time}s...')
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        gh = future.result()
        if gh.accepted:
            self.get_logger().info('[IK] Movimento iniciado.')
            gh.get_result_async().add_done_callback(
                lambda f: self.get_logger().info('[IK] Movimento concluído.')
            )
        else:
            self.get_logger().error('[IK] Goal rejeitado.')


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
