cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(ekf_localizer
  src/ekf_localizer_node.cpp
  src/ekf_localizer.cpp
)
ament_target_dependencies(ekf_localizer kalman_filter)

# if(BUILD_TESTING)
#   find_package(ament_cmake_ros REQUIRED)
#   ament_add_ros_isolated_gtest(ekf_localizer-test test/test_ekf_localizer.test
#     test/src/test_ekf_localizer.cpp
#     src/ekf_localizer.cpp
#     src/kalman_filter/kalman_filter.cpp
#     src/kalman_filter/time_delay_kalman_filter.cpp
#   )
#   target_include_directories(ekf_localizer-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(ekf_localizer-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
