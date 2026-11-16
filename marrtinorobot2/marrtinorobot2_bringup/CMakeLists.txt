cmake_minimum_required(VERSION 3.5)
project(marrtinorobot2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS marrtinorobot2_bringup/laser_filter_180.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
