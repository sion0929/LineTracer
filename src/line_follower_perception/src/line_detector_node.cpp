#include <chrono>
#include <algorithm>
#include <memory>
#include <mutex>
#include <cmath>
#include <deque>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include "line_follower_perception/terrain_trust.hpp"

#include "line_follower_interfaces/msg/line_observation.hpp"
#include "line_follower_interfaces/msg/perception_status.hpp"
#include "line_follower_perception/line_detector_backend.hpp"

namespace line_follower_perception
{

using line_follower_interfaces::msg::PerceptionStatus;

class LineDetectorNode : public rclcpp::Node
{
public:
  explicit LineDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("line_detector", options)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const auto requested = declare_parameter<std::string>("backend", "opencv_baseline");
    const auto model_path = declare_parameter<std::string>("fast_scnn_model_path", "");
    // Camera model and bounded flat-ground projection (no track coordinates).
    camera_.x_offset = declare_parameter("camera_x_offset", 0.076);
    camera_.height = declare_parameter("camera_height", 0.103);
    camera_.hfov_deg = declare_parameter("camera_hfov_deg", 62.2);
    camera_.width = declare_parameter("image_width", 640);
    camera_.height_px = declare_parameter("image_height", 480);

    camera_.range_max = declare_parameter("projection_range_max", 1.2);
    camera_.grid_resolution = declare_parameter("projection_grid_resolution", 0.004);
    camera_.slope_limit_rad = declare_parameter("projection_slope_limit_rad", 0.0349066);
    if (camera_.height <= 0 || camera_.width <= 0 || camera_.height_px <= 0 ||
      camera_.hfov_deg <= 1 || camera_.hfov_deg >= 179 || camera_.range_max <= .35 ||
      camera_.range_max > 3. || camera_.grid_resolution < .002 || camera_.grid_resolution > .01)
    {
      throw std::invalid_argument("invalid camera/projection parameters");
    }
    backend_ = make_backend(requested, model_path, fallback_active_, camera_);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS(),
        [this, lifetime = lifetime_](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(lifetime->mutex);
          if (lifetime->closing) {return;}
          const auto & q = msg->orientation;
          const double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
          if (msg->orientation_covariance[0] < 0 || !std::isfinite(norm) ||
          std::abs(norm - 1.) > .05) {return;}
          const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1., 1.));
          const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z),
          1 - 2 * (q.x * q.x + q.y * q.y));
          attitude_.push_back({rclcpp::Time(msg->header.stamp).seconds(), pitch, roll});
          while (attitude_.size() > 400) {attitude_.pop_front();}
          if(pending_image_) {
            auto image=std::move(pending_image_);pending_image_.reset();on_image(image);
          }
      });
    if (fallback_active_) {
      RCLCPP_WARN(
        get_logger(), "backend '%s' unavailable, falling back to '%s'",
        requested.c_str(), backend_->name().c_str());
    }

    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this,lifetime=lifetime_](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(lifetime->mutex);if(lifetime->closing) return;
        const double stamp=rclcpp::Time(m->header.stamp).seconds();
        if(terrain_stamp_ && stamp<*terrain_stamp_) {terrain_=TerrainTrust{};terrain_position_.reset();}
        const auto & p=m->pose.pose.position;
        const double travel=terrain_position_?std::hypot(p.x-terrain_position_->first,p.y-terrain_position_->second):0.;
        if(!attitude_.empty() && std::abs(stamp-attitude_.back().stamp)<.05 && travel<.1) {
          terrain_.update(attitude_.back().pitch,travel,std::hypot(m->twist.twist.linear.x,m->twist.twist.linear.y));
        }
        terrain_position_=std::make_pair(p.x,p.y);terrain_stamp_=stamp;
      });
    line_pub_ = create_publisher<LineObservation>("/perception/line", 10);
    status_pub_ = create_publisher<PerceptionStatus>("/perception/status", 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      [this, lifetime = lifetime_](sensor_msgs::msg::Image::ConstSharedPtr msg,
      const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> lock(lifetime->mutex);
        if (lifetime->closing) {return;}
        if (!reported_transport_) {
          RCLCPP_INFO(get_logger(), "camera callback intra_process=%s",
            info.get_rmw_message_info().from_intra_process ? "true" : "false");
          reported_transport_ = true;
        }
        on_image(msg);
      });

    RCLCPP_INFO(get_logger(), "line_detector up (backend=%s)", backend_->name().c_str());
  }

  ~LineDetectorNode() override
  {
    // Component unload can overlap an executor callback. A shared gate outlives
    // the node, drains the running callback and prevents queued access to this.
    std::lock_guard<std::mutex> lock(lifetime_->mutex);
    lifetime_->closing = true;
  }

private:
  struct CallbackLifetime {std::mutex mutex; bool closing{false};};
  std::shared_ptr<CallbackLifetime> lifetime_{std::make_shared<CallbackLifetime>()};

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const double image_stamp=rclcpp::Time(msg->header.stamp).seconds();
    // A camera and IMU carrying the same stamp may arrive in either order.
    // Bound storage to one frame and wait for its capture attitude rather than
    // accepting an old sample or repeatedly resetting projection recovery.
    if(attitude_.empty() || image_stamp-attitude_.back().stamp>.04) {
      pending_image_=msg;return;
    }
    const auto t0 = std::chrono::steady_clock::now();

    auto obs = std::make_unique<LineObservation>();
    obs->header.stamp = msg->header.stamp;  // capture stamp, not receive time
    obs->header.frame_id = base_frame_;

    bool ok = false;
    try {
      auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
      const double capture = rclcpp::Time(msg->header.stamp).seconds();
      bool matched = false;
      // Capture-time attitude, never callback-time orientation. Nearest sample is
      // bounded to 50 ms; IMU arrives at 200 Hz. No extrapolation across stale input.
      double distance = .05;
      Attitude best{};
      for (const auto & sample : attitude_) {
        if (std::abs(sample.stamp - capture) < distance) {
          distance = std::abs(sample.stamp - capture); best = sample; matched = true;
        }
      }
      const bool terrain_fresh=terrain_stamp_ && std::abs(capture-*terrain_stamp_)<.1;
      backend_->set_attitude(best.pitch,best.roll,matched && terrain_fresh && terrain_.trusted());
      ok = backend_->detect(cv_img->image, *obs);
      if(std::getenv("LF_PROJECTION_DEBUG")) {
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"projection=%d attitude=%d pitch=%.5f terrain=%.5f trusted=%d fresh=%d",obs->projection_valid,matched,best.pitch,terrain_.height(),terrain_.trusted(),terrain_fresh);
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
    }
    line_pub_->publish(std::move(obs));

    const auto t1 = std::chrono::steady_clock::now();
    auto status = std::make_unique<PerceptionStatus>();
    status->header.stamp = now();
    status->last_capture_stamp = msg->header.stamp;
    status->capture_to_publish_ms =
      static_cast<float>((now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0);
    status->inference_ms = 0.0f;
    status->postprocess_ms =
      std::chrono::duration<float, std::milli>(t1 - t0).count();
    status->backend = backend_->id();
    status->fallback_active = fallback_active_;
    status->healthy = ok;
    status->detail = ok ? "" : "detection failed";
    status_pub_->publish(std::move(status));
  }

  struct Attitude {double stamp, pitch, roll;};
  std::deque<Attitude> attitude_;
  TerrainTrust terrain_;
  std::optional<double> terrain_stamp_;
  std::optional<std::pair<double,double>> terrain_position_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::string base_frame_;
  CameraGeometry camera_{};
  bool fallback_active_{false};
  bool reported_transport_{false};
  std::unique_ptr<LineDetectorBackend> backend_;
  rclcpp::Publisher<LineObservation>::SharedPtr line_pub_;
  rclcpp::Publisher<PerceptionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace line_follower_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(line_follower_perception::LineDetectorNode)
