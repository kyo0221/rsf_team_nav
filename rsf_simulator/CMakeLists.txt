cmake_minimum_required(VERSION 3.8)
project(rsf_simulator)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(interlace_deskew src/interlace_deskew.cpp)
target_compile_features(interlace_deskew PUBLIC cxx_std_17)
target_include_directories(interlace_deskew PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(interlace_deskew sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interlace_deskew test/test_interlace_deskew.cpp)
  target_link_libraries(test_interlace_deskew interlace_deskew)
endif()

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
