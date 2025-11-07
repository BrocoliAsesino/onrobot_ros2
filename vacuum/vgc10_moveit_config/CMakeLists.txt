cmake_minimum_required(VERSION 3.16)
project(vgc10_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config srdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
