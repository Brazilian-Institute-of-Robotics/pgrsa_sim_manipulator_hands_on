import os
import subprocess

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class PanelSpawner(Node):

    def __init__(self):
        super().__init__("panel_spawner")
        pkg = get_package_share_directory("kuka_force_lab")

        p = self.declare_parameter
        p("config_file", os.path.join(pkg, "config", "contact_scene.yaml"))
        p("model_name", "contact_panel")
        UNSET = -999.0
        p("x", UNSET)
        p("y", UNSET)
        p("z", UNSET)
        p("yaw", UNSET)

        g = lambda n: self.get_parameter(n).value
        cfg_path = g("config_file")
        with open(cfg_path) as fh:
            cfg = yaml.safe_load(fh)
        panel = cfg.get("contact_panel", {})

        def pick(name, default):
            v = float(g(name))
            return default if v <= UNSET else v

        x = pick("x", float(panel.get("x", 1.50)))
        y = pick("y", float(panel.get("y", 0.0)))
        z = pick("z", float(panel.get("z", 0.0)))
        yaw = pick("yaw", float(panel.get("yaw", 0.0)))

        sdf = os.path.join(pkg, "models", "contact_panel", "model.sdf")
        if not os.path.exists(sdf):
            self.get_logger().error(f"SDF nao encontrado: {sdf}")
            return

        name = g("model_name")
        self.get_logger().info(
            f'[Painel] criando "{name}" em ({x:.2f}, {y:.2f}, {z:.2f}), '
            f"yaw={yaw:.2f} rad"
        )
        self.get_logger().info(
            f"[Painel] topo da superficie em Z = "
            f'{z + cfg.get("surface", {}).get("z_surface", 0.95):.3f} m'
        )

        cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-file",
            sdf,
            "-name",
            name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-Y",
            str(yaw),
        ]
        try:
            subprocess.run(cmd, timeout=20, check=True, capture_output=True, text=True)
            self.get_logger().info(f'[Painel] "{name}" criado com sucesso.')
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(f"[Painel] falha ao criar: {exc.stderr}")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("[Painel] tempo esgotado ao criar o modelo.")
        except FileNotFoundError:
            self.get_logger().error(
                "[Painel] `ros2 run ros_gz_sim create` nao encontrado — "
                "instale ros-humble-ros-gz-sim."
            )


def main(args=None):
    rclpy.init(args=args)
    node = PanelSpawner()
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
