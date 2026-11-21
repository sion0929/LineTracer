cmake_minimum_required(VERSION 3.8)
project(line_follower_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(line_follower_interfaces REQUIRED)

add_library(mission_core
  src/mission_state_machine.cpp
  src/finish_judge.cpp)
target_include_directories(mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_core PUBLIC ${line_follower_interfaces_TARGETS})

add_executable(mission_manager src/mission_manager_node.cpp)
target_link_libraries(mission_manager
  mission_core rclcpp::rclcpp ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})

install(TARGETS mission_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS mission_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  target_link_libraries(test_mission_state_machine mission_core)
  ament_add_gtest(test_command_policy test/test_command_policy.cpp)
  target_link_libraries(test_command_policy mission_core)
  ament_add_gtest(test_snapshot test/test_snapshot.cpp)
  target_link_libraries(test_snapshot mission_core)
endif()

ament_package()
