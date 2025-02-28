cmake_minimum_required(VERSION 3.5)
project(marrtinorobot2_dynamixel)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
