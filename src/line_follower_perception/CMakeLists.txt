cmake_minimum_required(VERSION 3.8)
project(line_follower_perception)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(line_follower_interfaces REQUIRED)

add_library(line_detector_core src/line_detector_backend.cpp)
target_include_directories(line_detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(line_detector_core PUBLIC
  ${OpenCV_LIBS}
  ${line_follower_interfaces_TARGETS})

set_target_properties(line_detector_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(line_detector_component SHARED src/line_detector_node.cpp)
target_link_libraries(line_detector_component
  rclcpp_components::component
  line_detector_core
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

# Extension only (design §2.3): not launched by competition.launch.py.
add_library(obstacle_detector_component SHARED src/obstacle_detector_node.cpp)
target_link_libraries(obstacle_detector_component
  rclcpp_components::component
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${line_follower_interfaces_TARGETS})

install(TARGETS line_detector_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
rclcpp_components_register_node(line_detector_component
  PLUGIN "line_follower_perception::LineDetectorNode" EXECUTABLE line_detector)
rclcpp_components_register_node(obstacle_detector_component
  PLUGIN "line_follower_perception::ObstacleDetectorNode" EXECUTABLE obstacle_detector)
install(TARGETS line_detector_component obstacle_detector_component LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_line_detector_backend test/test_line_detector_backend.cpp)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
  target_link_libraries(test_line_detector_backend line_detector_core ${OpenCV_LIBS})
  target_compile_definitions(test_line_detector_backend PRIVATE
    TEST_IMAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()
