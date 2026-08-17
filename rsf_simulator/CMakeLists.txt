cmake_minimum_required(VERSION 3.8)
project(rsf_simulator)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(interlace_decimate STATIC src/interlace_decimate.cpp)
target_compile_features(interlace_decimate PUBLIC cxx_std_17)
target_include_directories(interlace_decimate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(interlace_decimate builtin_interfaces sensor_msgs)

add_executable(interlace_decimate_node src/interlace_decimate_node.cpp)
target_link_libraries(interlace_decimate_node interlace_decimate)
ament_target_dependencies(interlace_decimate_node rclcpp sensor_msgs)

install(TARGETS interlace_decimate_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
