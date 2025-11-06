cmake_minimum_required(VERSION 3.8)
project(onrobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VGC10Status.srv"
  "action/VGC10GripperCommand.action"
  DEPENDENCIES 
)

install(
  DIRECTORY  srv action
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
