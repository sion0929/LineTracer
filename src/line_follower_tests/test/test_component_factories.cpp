#include <algorithm>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

TEST(ComponentFactories, OptionsAndStandaloneDefaults)
{
  rclcpp::init(0, nullptr);
  {
    auto manager = std::make_shared<rclcpp_components::ComponentManager>();
    for (const auto & entry : std::vector<std::pair<std::string, std::string>>{
      {"line_follower_perception", "line_follower_perception::LineDetectorNode"},
      {"line_follower_perception", "line_follower_perception::ObstacleDetectorNode"},
      {"line_follower_navigation", "line_follower_navigation::PathPlannerNode"}})
    {
      const auto resources = manager->get_component_resources(entry.first);
      auto resource = std::find_if(resources.begin(), resources.end(), [&](const auto & r) {
            return r.first == entry.second;
      });
      ASSERT_NE(resource, resources.end()) << entry.second;
      auto factory = manager->create_component_factory(*resource);
      for (bool intra : {false, true}) {
        auto options = rclcpp::NodeOptions().use_intra_process_comms(intra);
        options.parameter_overrides({rclcpp::Parameter("base_frame", "component_test_base")});
        auto instance = factory->create_node_instance(options);
        EXPECT_EQ(instance.get_node_base_interface()->get_use_intra_process_default(), intra);
      }
    }
  }
  rclcpp::shutdown();
}

#include <chrono>
#include <thread>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "line_follower_interfaces/msg/line_observation.hpp"

TEST(ComponentFactories, RepeatedPerceptionUnloadDuringInputs)
{
  rclcpp::init(0, nullptr);
  for (const auto & component : std::vector<std::pair<std::string, std::string>>{
    {"line_follower_perception", "line_follower_perception::LineDetectorNode"},
    {"line_follower_navigation", "line_follower_navigation::PathPlannerNode"}})
  {
    auto manager = std::make_shared<rclcpp_components::ComponentManager>();
    auto resources = manager->get_component_resources(component.first);
    auto resource = std::find_if(resources.begin(), resources.end(), [&](const auto & r) {
          return r.first == component.second;
    });
    ASSERT_NE(resource, resources.end());
    auto factory = manager->create_component_factory(*resource);
    auto source = std::make_shared<rclcpp::Node>("component_unload_source",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    auto publisher = source->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw",
      rclcpp::SensorDataQoS());
    auto line_pub =
      source->create_publisher<line_follower_interfaces::msg::LineObservation>("/perception/line",
      10);
    auto odom_pub = source->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    auto imu_pub = source->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(source);
    std::thread spin([&]() {executor.spin();});
    sensor_msgs::msg::Image image;
    image.width = 640; image.height = 480; image.step = 1920; image.encoding = "bgr8";
    image.data.assign(640 * 480 * 3, 205);
    for (int row = 280; row < 480; ++row) {
      std::fill(image.data.begin() + (row * 640 + 312) * 3,
        image.data.begin() + (row * 640 + 328) * 3, 0);
    }
    for (int cycle = 0; cycle < 80; ++cycle) {
      auto instance =
        factory->create_node_instance(rclcpp::NodeOptions().use_intra_process_comms(true));
      executor.add_node(instance.get_node_base_interface());
      for (int sample = 0; sample < 12; ++sample) {
        image.header.stamp = source->now();
        if (component.first == "line_follower_perception") {
          publisher->publish(image);
        } else {
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = image.header.stamp; odom.pose.pose.orientation.w = 1.;
          sensor_msgs::msg::Imu imu; imu.header.stamp = image.header.stamp; imu.orientation.w = 1.;
          odom_pub->publish(odom); imu_pub->publish(imu);
          line_follower_interfaces::msg::LineObservation obs;
          obs.header.stamp = image.header.stamp; obs.header.frame_id = "base_footprint";
          obs.line_found = true; obs.projection_valid = true; obs.quality = .9;
          obs.segments.resize(1); obs.segments[0].confidence = .9;
          for (int k = 0; k < 100; ++k) {
            line_follower_interfaces::msg::PathPoint p; p.x = .316 + k * .005; p.confidence = .9;
            obs.segments[0].points.push_back(p);
          }
          line_pub->publish(obs);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1500 + cycle % 7 * 100));
      }
      executor.remove_node(instance.get_node_base_interface());
    }
    executor.cancel(); spin.join();
    executor.remove_node(source);
  }
  rclcpp::shutdown();
}
