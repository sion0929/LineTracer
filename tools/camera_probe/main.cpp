#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <line_follower_interfaces/msg/perception_status.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("external_camera_probe");
  size_t images = 0, statuses = 0;
  std::vector<float> processing;
  auto begin = std::chrono::steady_clock::time_point{};
  const auto on_image = [&](sensor_msgs::msg::Image::ConstSharedPtr) {
      if (!images++) {begin = std::chrono::steady_clock::now();}
    };
  auto image = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", std::getenv("PROBE_RELIABLE") ?
    rclcpp::QoS(10) : rclcpp::QoS(rclcpp::SensorDataQoS()), on_image);
  const auto on_status = [&](line_follower_interfaces::msg::PerceptionStatus::ConstSharedPtr m) {
      if (images) {++statuses; processing.push_back(m->postprocess_ms);}
    };
  auto status = node->create_subscription<line_follower_interfaces::msg::PerceptionStatus>(
    "/perception/status", 1000, on_status);
  auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(40);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < limit) {
    rclcpp::spin_some(node);
    if (images && std::chrono::steady_clock::now() - begin >= std::chrono::seconds(20)) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  std::cout << "CPP_IMAGES=" << images << " WINDOW_SEC=" << elapsed << " HZ=" <<
    (images > 1 ? (images - 1) / elapsed : 0) << " DETECTOR_STATUSES=" << statuses << std::endl;
  if (!processing.empty()) {
    std::sort(processing.begin(), processing.end());
    std::cout << "PROCESS_P50_MS=" << processing[processing.size() / 2] <<
      " PROCESS_P95_MS=" << processing[std::min(processing.size() - 1,
      size_t(processing.size() * .95))] << std::endl;
  }
  rclcpp::shutdown();
  return images > 100 ? 0 : 1;
}
