import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, Float64MultiArray
from collections import deque
import json


class ContactMonitor(Node):

    STATE_FREE = "FREE"
    STATE_CONTACT = "CONTACT"
    STATE_GRASPING = "GRASPING"
    STATE_OVERLOAD = "OVERLOAD"

    def __init__(self):
        super().__init__("contact_monitor")

        self.declare_parameter("F_contact", 2.0)  # N — limiar de contato
        self.declare_parameter("F_grasp", 15.0)  # N — limiar de grasping
        self.declare_parameter("F_overload", 80.0)  # N — limiar de sobrecarga
        self.declare_parameter("window", 20)  # amostras para filtro

        self.F_contact = self.get_parameter("F_contact").value
        self.F_grasp = self.get_parameter("F_grasp").value
        self.F_overload = self.get_parameter("F_overload").value
        self.window_size = self.get_parameter("window").value

        self.force_window = deque(maxlen=self.window_size)
        self.torque_window = deque(maxlen=self.window_size)

        self.state = self.STATE_FREE
        self.state_history = []
        self.contact_count = 0
        self.max_force_seen = 0.0
        self.contact_start_time = None

        self.stiffness_buffer = deque(maxlen=50)
        self.F_prev = 0.0

        self.sub_ft = self.create_subscription(
            WrenchStamped, "/kuka_force_control/ft_wrist", self._cb_ft, 10
        )

        self.pub_state = self.create_publisher(
            String, "/kuka_force_control/contact_state", 10
        )

        self.pub_metrics = self.create_publisher(
            Float64MultiArray, "/kuka_force_control/contact_metrics", 10
        )

        self.timer = self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            f"[ContactMonitor] Limiares: F_contact={self.F_contact}N | "
            f"F_grasp={self.F_grasp}N | F_overload={self.F_overload}N"
        )

    def _cb_ft(self, msg: WrenchStamped):
        F = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        T = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        fn = float(np.linalg.norm(F))
        tn = float(np.linalg.norm(T))

        self.force_window.append(fn)
        self.torque_window.append(tn)
        self.max_force_seen = max(self.max_force_seen, fn)

        prev_state = self.state
        fn_filtered = float(np.mean(self.force_window))

        if fn_filtered >= self.F_overload:
            new_state = self.STATE_OVERLOAD
        elif fn_filtered >= self.F_grasp:
            new_state = self.STATE_GRASPING
        elif fn_filtered >= self.F_contact:
            new_state = self.STATE_CONTACT
        else:
            new_state = self.STATE_FREE

        if new_state != prev_state:
            self.state = new_state
            self.state_history.append(
                {
                    "from": prev_state,
                    "to": new_state,
                    "force": fn_filtered,
                }
            )

            if new_state == self.STATE_CONTACT:
                self.contact_count += 1
                self.contact_start_time = self.get_clock().now()
                self.get_logger().info(
                    f"[ContactMonitor] CONTATO #{self.contact_count} | F={fn_filtered:.2f}N"
                )
            elif new_state == self.STATE_GRASPING:
                self.get_logger().info(
                    f"[ContactMonitor] GRASPING | F={fn_filtered:.2f}N"
                )
            elif new_state == self.STATE_OVERLOAD:
                self.get_logger().error(
                    f"[ContactMonitor] ⚠ SOBRECARGA! F={fn_filtered:.2f}N > {self.F_overload}N"
                )
            elif new_state == self.STATE_FREE:
                if self.contact_start_time is not None:
                    duration = (
                        self.get_clock().now() - self.contact_start_time
                    ).nanoseconds / 1e9
                    self.get_logger().info(
                        f"[ContactMonitor] Contato encerrado | duração={duration:.2f}s"
                    )

    def _publish_state(self):
        fn_filtered = float(np.mean(self.force_window)) if self.force_window else 0.0
        tn_filtered = float(np.mean(self.torque_window)) if self.torque_window else 0.0

        state_msg = String()
        state_msg.data = json.dumps(
            {
                "state": self.state,
                "force_norm": fn_filtered,
                "torque_norm": tn_filtered,
                "contact_count": self.contact_count,
                "max_force": self.max_force_seen,
                "thresholds": {
                    "contact": self.F_contact,
                    "grasp": self.F_grasp,
                    "overload": self.F_overload,
                },
            }
        )
        self.pub_state.publish(state_msg)

        m = Float64MultiArray()
        m.data = [
            fn_filtered,
            tn_filtered,
            float(self.contact_count),
            self.max_force_seen,
            {"FREE": 0.0, "CONTACT": 1.0, "GRASPING": 2.0, "OVERLOAD": 3.0}[self.state],
        ]
        self.pub_metrics.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
