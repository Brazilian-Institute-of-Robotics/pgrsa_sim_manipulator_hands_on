# r6bot_gripper

Gripper paralelo de 2 dedos para o **r6bot** (Example 7 — ros2_control_demos).  
Projetado para manipular a **bola.sdf** (raio = 0.12 m) no Gazebo.

---

## Estrutura do pacote

```
r6bot_gripper/
├── description/
│   ├── urdf/
│   │   ├── r6bot_gripper.urdf.xacro        # gripper standalone
│   │   └── r6bot_with_gripper.urdf.xacro   # r6bot + gripper (wrapper)
│   └── bola.sdf                            # copie a bola.sdf aqui
│
├── hardware/
│   ├── gripper_hardware_interface.hpp
│   └── gripper_hardware_interface.cpp
│
├── controller/
│   ├── gripper_controller.hpp
│   └── gripper_controller.cpp
│
├── config/
│   └── gripper_controllers.yaml
│
├── launch/
│   └── r6bot_gripper.launch.py
│
├── r6bot_gripper_hardware.xml    # plugin export (hardware)
├── r6bot_gripper_controller.xml  # plugin export (controller)
├── CMakeLists.txt
└── package.xml
```

---

## Geometria do Gripper

```
         tool0 (flange r6bot)
            │
    ┌───────┴───────┐  ← gripper_base (80×100×40 mm)
    │               │
  [←]             [→]   ← finger_left / finger_right
  dedo             dedo   (cada um: 100×20×40 mm)
```

| Parâmetro             | Valor       |
|-----------------------|-------------|
| Abertura máxima total | 280 mm      |
| Abertura mínima total | 20 mm       |
| Diâmetro da bola      | 240 mm      |
| Posição de grasp      | 122 mm/lado |
| Fricção dos dedos     | μ = 1.2     |
| Massa total           | ~400 g      |

---

## Pré-requisitos

```bash
sudo apt install ros-rolling-ros2-control \
                 ros-rolling-ros2-controllers \
                 ros-rolling-gz-ros2-control \
                 ros-rolling-realtime-tools

# Clone o Example 7
git clone -b master https://github.com/ros-controls/ros2_control_demos.git
```

---

## Compilação

```bash
# Copie bola.sdf para description/
cp /caminho/para/bola.sdf description/

# Na raiz do seu workspace ROS 2:
colcon build --packages-select r6bot_gripper ros2_control_demo_example_7
source install/setup.bash
```

---

## Execução

```bash
# Inicia Gazebo + r6bot + gripper + bola
ros2 launch r6bot_gripper r6bot_gripper.launch.py
```

---

## Controle via linha de comando

### Abrir / fechar com valor normalizado (0.0 = fechado, 1.0 = aberto)

```bash
# Abrir completamente
ros2 topic pub --once /gripper_controller/gripper_cmd \
  std_msgs/msg/Float64 "{data: 1.0}"

# Fechar completamente
ros2 topic pub --once /gripper_controller/gripper_cmd \
  std_msgs/msg/Float64 "{data: 0.0}"

# Posição intermediária (50%)
ros2 topic pub --once /gripper_controller/gripper_cmd \
  std_msgs/msg/Float64 "{data: 0.5}"
```

### Agarrar a bola (fecha automaticamente ao redor do raio = 0.12 m)

```bash
# PEGAR bola
ros2 topic pub --once /gripper_controller/gripper_grasp \
  std_msgs/msg/Bool "{data: true}"

# SOLTAR bola
ros2 topic pub --once /gripper_controller/gripper_grasp \
  std_msgs/msg/Bool "{data: false}"
```

### Monitorar estado dos dedos

```bash
ros2 topic echo /joint_states
```

---

## Integração com MoveIt 2

Para usar o gripper com MoveIt 2, adicione o grupo `gripper` no SRDF:

```xml
<group name="gripper">
  <joint name="gripper_finger_left_joint"/>
  <joint name="gripper_finger_right_joint"/>
</group>

<end_effector name="gripper"
              parent_link="tool0"
              group="gripper"/>
```

---

## Adaptação para hardware real

No arquivo `hardware/gripper_hardware_interface.cpp`, substitua os métodos:

```cpp
return_type GripperHardwareInterface::read(...) {
    // Leia encoder/sensor do atuador real aqui
    // Exemplo: porta serial, CAN bus, Modbus, etc.
}

return_type GripperHardwareInterface::write(...) {
    // Envie hw_command_[LEFT] e hw_command_[RIGHT] ao atuador
}
```

---

## Tópicos ROS 2

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/gripper_controller/gripper_cmd` | `std_msgs/Float64` | Abertura normalizada [0,1] |
| `/gripper_controller/gripper_grasp` | `std_msgs/Bool` | Liga/desliga grasp da bola |
| `/joint_states` | `sensor_msgs/JointState` | Estado atual dos dedos |
| `/controller_manager/...` | serviços | Gerenciamento dos controladores |
