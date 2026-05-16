#ifndef R6BOT_GRIPPER__GRIPPER_CONTROLLER_HPP_
#define R6BOT_GRIPPER__GRIPPER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace r6bot_gripper
{

/**
 * @brief Controlador do Gripper para manipulação da bola (raio = 0.12 m).
 *
 * Interfaces ROS 2:
 *   Subscriber  ~/gripper_cmd   (std_msgs/Float64)
 *       Valor em [0.0, 1.0]:
 *           0.0 → fechado  (posição mínima = 0.01 m cada dedo)
 *           1.0 → aberto   (posição máxima = 0.14 m cada dedo)
 *
 *   Subscriber  ~/gripper_grasp (std_msgs/Bool)
 *       true  → fecha para pegar a bola (abertura = raio da bola + margem)
 *       false → abre totalmente
 *
 * Os dois dedos são sempre simétricos: finger_left = finger_right = setpoint.
 */
class GripperController : public controller_interface::ControllerInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  GripperController() = default;

  // ── ControllerInterface ────────────────────────────────────────────────────
  controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // ── Lifecycle ──────────────────────────────────────────────────────────────
  controller_interface::CallbackReturn on_init()     override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // Nomes das juntas (mesma ordem do URDF)
  std::vector<std::string> joint_names_{
    "gripper_finger_left_joint",
    "gripper_finger_right_joint"
  };

  // Limites (em metros) — devem coincidir com o URDF
  double min_pos_{0.01};
  double max_pos_{0.14};
  double ball_radius_{0.12};   // raio da bola.sdf

  // Posição alvo para cada dedo
  std::array<double, 2> target_position_{0.14, 0.14};

  // Buffer thread-safe para o comando normalizado [0,1]
  realtime_tools::RealtimeBuffer<double> cmd_buffer_;
  // Buffer para o comando de agarrar booleano
  realtime_tools::RealtimeBuffer<bool>   grasp_buffer_;

  // Publishers / Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    grasp_sub_;

  // Converte valor normalizado → posição em metros
  double normalized_to_position(double norm) const;

  // Posição de fechamento para pegar a bola (deixa 2 mm de folga)
  double grasp_position() const { return ball_radius_ + 0.002; }

  rclcpp::Logger logger_{rclcpp::get_logger("GripperController")};
};

}  // namespace r6bot_gripper

#endif  // R6BOT_GRIPPER__GRIPPER_CONTROLLER_HPP_
