cmake_minimum_required(VERSION 3.8)
project(line_follower_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(line_follower_interfaces REQUIRED)

add_library(control_core
  src/pure_pursuit.cpp
  src/rotate_pid.cpp
  src/safety_policy.cpp)
target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(motion_controller src/motion_controller_node.cpp)
target_link_libraries(motion_controller
  control_core rclcpp::rclcpp ${nav_msgs_TARGETS} ${line_follower_interfaces_TARGETS})

add_executable(safety_guard src/safety_guard_node.cpp)
target_link_libraries(safety_guard
  control_core rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${line_follower_interfaces_TARGETS})

install(TARGETS control_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS motion_controller safety_guard DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_policy test/test_safety_policy.cpp)
  target_link_libraries(test_safety_policy control_core)
endif()

ament_package()
