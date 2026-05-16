#ifndef R6BOT_GRIPPER__GRIPPER_HARDWARE_INTERFACE_HPP_
#define R6BOT_GRIPPER__GRIPPER_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <array>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace r6bot_gripper
{

/**
 * @brief Hardware Interface do Gripper para o r6bot (Example 7).
 *
 * Implementa um gripper paralelo de 2 dedos simétricos.
 * As duas juntas prismáticas (esquerda/direita) são sempre comandadas
 * com a mesma posição — o controlador envia um único setpoint "abertura"
 * e este driver espelha o comando nos dois dedos.
 *
 * Em hardware real, substitua os métodos read()/write() pela comunicação
 * serial/CAN/Ethernet com o atuador do gripper.
 */
class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GripperHardwareInterface)

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Loop de controle ───────────────────────────────────────────────────────

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // ── Exportação de interfaces ───────────────────────────────────────────────

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // Índices: 0 = dedo esquerdo, 1 = dedo direito
  static constexpr std::size_t N_JOINTS = 2;
  static constexpr std::size_t LEFT  = 0;
  static constexpr std::size_t RIGHT = 1;

  // Limites físicos (m)
  double min_position_{0.01};
  double max_position_{0.14};
  double loop_rate_{50.0};

  // Estado atual (simulado)
  std::array<double, N_JOINTS> hw_position_{};
  std::array<double, N_JOINTS> hw_velocity_{};
  std::array<double, N_JOINTS> hw_effort_{};

  // Comandos recebidos do controlador
  std::array<double, N_JOINTS> hw_command_{};

  // Simulação simples: move o dedo em direção ao comando
  void simulate_step(std::size_t idx, double dt);

  rclcpp::Logger logger_{rclcpp::get_logger("GripperHardwareInterface")};
};

}  // namespace r6bot_gripper

#endif  // R6BOT_GRIPPER__GRIPPER_HARDWARE_INTERFACE_HPP_
