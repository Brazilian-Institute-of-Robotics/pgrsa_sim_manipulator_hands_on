cmake_minimum_required(VERSION 3.16)
project(r6bot_gripper)

# Padrão C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependências ───────────────────────────────────────────────────────────────
find_package(ament_cmake          REQUIRED)
find_package(hardware_interface   REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib             REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(rclcpp_lifecycle     REQUIRED)
find_package(realtime_tools       REQUIRED)
find_package(std_msgs             REQUIRED)

# ── Biblioteca: Hardware Interface ─────────────────────────────────────────────
add_library(r6bot_gripper_hardware SHARED
  hardware/gripper_hardware_interface.cpp
)
target_include_directories(r6bot_gripper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(r6bot_gripper_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(
  hardware_interface r6bot_gripper_hardware.xml)

# ── Biblioteca: Controller ─────────────────────────────────────────────────────
add_library(r6bot_gripper_controller SHARED
  controller/gripper_controller.cpp
)
target_include_directories(r6bot_gripper_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(r6bot_gripper_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)
pluginlib_export_plugin_description_file(
  controller_interface r6bot_gripper_controller.xml)

# ── Instalação ─────────────────────────────────────────────────────────────────
install(
  TARGETS
    r6bot_gripper_hardware
    r6bot_gripper_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    description/
    config/
    launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    r6bot_gripper_hardware.xml
    r6bot_gripper_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

# ── Testes ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
