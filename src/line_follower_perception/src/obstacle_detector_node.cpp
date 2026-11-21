#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "line_follower_interfaces/msg/obstacle_array.hpp"

namespace line_follower_perception
{

using line_follower_interfaces::msg::ObstacleArray;

// Obstacle region estimation (design §2.3, §4.2). EXTENSION ONLY: not part of the competition launch.
// DUMMY: always reports no obstacles. TODO(T-18): separate model from the line model, left/right
// avoidance candidates, safe return to the line.
class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_detector", options)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    pub_ = create_publisher<ObstacleArray>("/perception/obstacles", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        auto out = std::make_unique<ObstacleArray>();
        out->header.stamp = msg->header.stamp;  // capture stamp
        out->header.frame_id = base_frame_;
        out->healthy = true;
        pub_->publish(std::move(out));
      });
    RCLCPP_INFO(get_logger(), "obstacle_detector up (extension, dummy)");
  }

private:
  std::string base_frame_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace line_follower_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(line_follower_perception::ObstacleDetectorNode)
