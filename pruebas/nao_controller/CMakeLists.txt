cmake_minimum_required(VERSION 3.10.2)
project(nao_controller)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
