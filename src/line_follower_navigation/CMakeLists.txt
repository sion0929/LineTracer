cmake_minimum_required(VERSION 3.8)
project(line_follower_navigation)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(line_follower_interfaces REQUIRED)

add_library(navigation_core
  src/pose_estimator.cpp
  src/local_path_mapper.cpp
  src/local_path_memory.cpp
  src/candidate_feature_builder.cpp
  src/path_evaluator.cpp
  src/trajectory_planner.cpp)
target_include_directories(navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(navigation_core PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${line_follower_interfaces_TARGETS})

set_target_properties(navigation_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(path_planner_component SHARED src/path_planner_node.cpp)
target_link_libraries(path_planner_component navigation_core rclcpp_components::component ${std_srvs_TARGETS})
rclcpp_components_register_node(path_planner_component
  PLUGIN "line_follower_navigation::PathPlannerNode" EXECUTABLE path_planner)

install(TARGETS navigation_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS path_planner_component LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_memory_policy test/test_memory_policy.cpp)
  target_link_libraries(test_memory_policy navigation_core)
  ament_add_gtest(test_mapping_memory test/test_mapping_memory.cpp)
  target_link_libraries(test_mapping_memory navigation_core)
  ament_add_gtest(test_trajectory_planner test/test_trajectory_planner.cpp)
  target_link_libraries(test_trajectory_planner navigation_core)
endif()

ament_package()
