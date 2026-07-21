cmake_minimum_required(VERSION 3.8)
project(rsf_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(tf_odom_to_footprint
  src/tf_odom_to_footprint.cpp
)
target_compile_features(tf_odom_to_footprint PUBLIC cxx_std_17)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
