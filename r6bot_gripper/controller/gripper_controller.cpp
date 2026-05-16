#include "gripper_controller.hpp"

#include <algorithm>
#include <cmath>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace r6bot_gripper
{

// ═══════════════════════════════════════════════════════════════════════════════
// on_init
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::CallbackReturn GripperController::on_init()
{
  try {
    // Declara parâmetros configuráveis
    auto_declare<double>("ball_radius", ball_radius_);
    auto_declare<double>("min_position", min_pos_);
    auto_declare<double>("max_position", max_pos_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "on_init falhou: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// on_configure
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::CallbackReturn
GripperController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Lê parâmetros
  ball_radius_ = get_node()->get_parameter("ball_radius").as_double();
  min_pos_     = get_node()->get_parameter("min_position").as_double();
  max_pos_     = get_node()->get_parameter("max_position").as_double();

  // Subscriber: comando normalizado [0,1]
  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/gripper_cmd", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      const double clamped = std::clamp(msg->data, 0.0, 1.0);
      cmd_buffer_.writeFromNonRT(clamped);
    });

  // Subscriber: agarrar bola (bool)
  grasp_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/gripper_grasp", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      grasp_buffer_.writeFromNonRT(msg->data);
    });

  // Valores iniciais dos buffers
  cmd_buffer_.writeFromNonRT(1.0);     // aberto
  grasp_buffer_.writeFromNonRT(false); // sem grasp

  RCLCPP_INFO(logger_,
    "GripperController configurado. ball_radius=%.3f  faixa=[%.3f, %.3f]",
    ball_radius_, min_pos_, max_pos_);

  return controller_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// command_interface_configuration
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    conf.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

// ═══════════════════════════════════════════════════════════════════════════════
// state_interface_configuration
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    conf.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    conf.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

// ═══════════════════════════════════════════════════════════════════════════════
// on_activate
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::CallbackReturn
GripperController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Lê posição atual e usa como setpoint inicial (evita saltos)
  if (!state_interfaces_.empty()) {
    target_position_[0] = state_interfaces_[0].get_value();
    target_position_[1] = state_interfaces_[3].get_value();
  }
  RCLCPP_INFO(logger_, "GripperController ativado.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// on_deactivate
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::CallbackReturn
GripperController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "GripperController desativado.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// update  — loop de controle em tempo real
// ═══════════════════════════════════════════════════════════════════════════════
controller_interface::return_type
GripperController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // --- Lê comandos dos buffers thread-safe ---
  const bool   do_grasp = *grasp_buffer_.readFromRT();
  const double norm_cmd = *cmd_buffer_.readFromRT();

  // --- Calcula posição alvo ---
  double pos;
  if (do_grasp) {
    // Fecha ao redor da bola
    pos = grasp_position();
  } else {
    pos = normalized_to_position(norm_cmd);
  }
  pos = std::clamp(pos, min_pos_, max_pos_);

  // --- Escreve nas command_interfaces (ambos os dedos iguais) ---
  //  command_interfaces_[0] → finger_left/position
  //  command_interfaces_[1] → finger_right/position
  command_interfaces_[0].set_value(pos);
  command_interfaces_[1].set_value(pos);

  target_position_[0] = pos;
  target_position_[1] = pos;

  RCLCPP_DEBUG(logger_,
    "Setpoint → %.4f m  (grasp=%s  cmd_norm=%.2f)",
    pos, do_grasp ? "true" : "false", norm_cmd);

  return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════════
double GripperController::normalized_to_position(double norm) const
{
  // 0 → min_pos_ (fechado), 1 → max_pos_ (aberto)
  return min_pos_ + norm * (max_pos_ - min_pos_);
}

}  // namespace r6bot_gripper

// ── Exportação do plugin ──────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  r6bot_gripper::GripperController,
  controller_interface::ControllerInterface)
