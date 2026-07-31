cmake_minimum_required(VERSION 3.8)
project(rsf_navigation_executor)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
