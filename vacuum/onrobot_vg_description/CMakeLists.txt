cmake_minimum_required(VERSION 3.10)
project(onrobot_vg_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
