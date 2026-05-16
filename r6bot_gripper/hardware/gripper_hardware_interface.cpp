#include "gripper_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace r6bot_gripper
{

// ═══════════════════════════════════════════════════════════════════════════════
// on_init
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
GripperHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validação: exatamente 2 juntas
  if (info_.joints.size() != N_JOINTS) {
    RCLCPP_ERROR(logger_,
      "GripperHardwareInterface espera %zu juntas, recebeu %zu.",
      N_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Lê parâmetros opcionais do URDF
  if (info_.hardware_parameters.count("loop_rate")) {
    loop_rate_ = std::stod(info_.hardware_parameters.at("loop_rate"));
  }

  // Inicializa vetores
  hw_position_.fill(max_position_);   // gripper começa aberto
  hw_velocity_.fill(0.0);
  hw_effort_.fill(0.0);
  hw_command_.fill(max_position_);

  RCLCPP_INFO(logger_, "GripperHardwareInterface inicializado (loop_rate=%.1f Hz).", loop_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// on_configure
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
GripperHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Em hardware real: abrir porta serial, inicializar driver, etc.
  RCLCPP_INFO(logger_, "Gripper configurado — aguardando ativação.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// on_activate
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
GripperHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Garante que o comando inicial seja a posição atual (sem saltos)
  hw_command_ = hw_position_;
  RCLCPP_INFO(logger_, "Gripper ativado. Posição inicial: esq=%.4f  dir=%.4f",
    hw_position_[LEFT], hw_position_[RIGHT]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// on_deactivate
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
GripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Gripper desativado.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════════
// export_state_interfaces
// ═══════════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
GripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < N_JOINTS; ++i) {
    const auto & jname = info_.joints[i].name;
    interfaces.emplace_back(jname, hardware_interface::HW_IF_POSITION, &hw_position_[i]);
    interfaces.emplace_back(jname, hardware_interface::HW_IF_VELOCITY, &hw_velocity_[i]);
    interfaces.emplace_back(jname, hardware_interface::HW_IF_EFFORT,   &hw_effort_[i]);
  }
  return interfaces;
}

// ═══════════════════════════════════════════════════════════════════════════════
// export_command_interfaces
// ═══════════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::CommandInterface>
GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < N_JOINTS; ++i) {
    const auto & jname = info_.joints[i].name;
    interfaces.emplace_back(jname, hardware_interface::HW_IF_POSITION, &hw_command_[i]);
  }
  return interfaces;
}

// ═══════════════════════════════════════════════════════════════════════════════
// read  — atualiza state_interfaces com valores do hardware (aqui: simulação)
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::return_type
GripperHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  for (std::size_t i = 0; i < N_JOINTS; ++i) {
    simulate_step(i, dt);
  }
  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// write  — envia comandos ao hardware (aqui: apenas log em debug)
// ═══════════════════════════════════════════════════════════════════════════════
hardware_interface::return_type
GripperHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Em hardware real: envia hw_command_[LEFT/RIGHT] para o atuador.
  // Satura nos limites para segurança.
  for (std::size_t i = 0; i < N_JOINTS; ++i) {
    hw_command_[i] = std::clamp(hw_command_[i], min_position_, max_position_);
  }
  RCLCPP_DEBUG(logger_, "Comando → esq=%.4f  dir=%.4f",
    hw_command_[LEFT], hw_command_[RIGHT]);
  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// simulate_step  — modelo de 1ª ordem simples para o dedo
// ═══════════════════════════════════════════════════════════════════════════════
void GripperHardwareInterface::simulate_step(std::size_t idx, double dt)
{
  constexpr double tau    = 0.1;   // constante de tempo (s)
  constexpr double v_max  = 0.05;  // velocidade máxima (m/s)
  constexpr double f_nom  = 5.0;   // esforço nominal (N) — proporcional ao erro

  const double target = std::clamp(hw_command_[idx], min_position_, max_position_);
  const double error  = target - hw_position_[idx];

  // Velocidade limitada
  double vel = error / tau;
  vel = std::clamp(vel, -v_max, v_max);

  hw_position_[idx] += vel * dt;
  hw_position_[idx]  = std::clamp(hw_position_[idx], min_position_, max_position_);
  hw_velocity_[idx]  = vel;
  hw_effort_[idx]    = std::abs(error) * f_nom;
}

}  // namespace r6bot_gripper

// ── Exportação do plugin ──────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  r6bot_gripper::GripperHardwareInterface,
  hardware_interface::SystemInterface)
