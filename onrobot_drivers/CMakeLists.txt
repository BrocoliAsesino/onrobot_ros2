cmake_minimum_required(VERSION 3.5)
project(onrobot_drivers)

# Enable PIC for all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/include/onrobot_drivers
  ${rclcpp_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

# Enable serial communication for the Modbus library.
set(MODBUS_SERIAL_COMMUNICATION ON CACHE BOOL "Enable Modbus Serial Communication" FORCE)

# Add the Modbus library as a subdirectory.
add_subdirectory(include/onrobot_drivers/common/Modbus)

# Build the VG hardware interface as a shared library.
add_library(onrobot_vg_plugin
  SHARED
  src/vg/vg_hardware_interface.cpp
  src/vg/VG.cpp
  include/onrobot_drivers/common/TCPConnectionWrapper.hpp
  include/onrobot_drivers/common/SerialConnectionWrapper.hpp
  include/onrobot_drivers/common/IModbusConnection.hpp
)
target_link_libraries(onrobot_vg_plugin
  Modbus
)
target_include_directories(onrobot_vg_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  include
)
ament_target_dependencies(onrobot_vg_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface vg_hardware_interface.xml)

# Build the RG hardware interface as a shared library.
add_library(onrobot_rg_plugin
  SHARED
  src/rg/rg_hardware_interface.cpp
  src/rg/RG.cpp
  include/onrobot_drivers/common/TCPConnectionWrapper.hpp
  include/onrobot_drivers/common/SerialConnectionWrapper.hpp
  include/onrobot_drivers/common/IModbusConnection.hpp
)
target_link_libraries(onrobot_rg_plugin
  Modbus
)
target_include_directories(onrobot_rg_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  include
)
ament_target_dependencies(onrobot_rg_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface rg_hardware_interface.xml)

# INSTALL
install(TARGETS onrobot_vg_plugin onrobot_rg_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  onrobot_vg_plugin
  onrobot_rg_plugin
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()