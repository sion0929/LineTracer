#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "line_follower_interfaces/msg/line_observation.hpp"

namespace line_follower_perception
{

using line_follower_interfaces::msg::LineObservation;

// Common output schema for every recognition backend (design §6.1).
// detect() fills segments/junctions/quality in the robot base frame; the caller owns header.
class LineDetectorBackend
{
public:
  virtual ~LineDetectorBackend() = default;
  virtual std::string name() const = 0;
  virtual uint8_t id() const = 0;
  virtual bool ready() const = 0;
  virtual bool detect(const cv::Mat & bgr, LineObservation & out) = 0;
  virtual void set_attitude(double, double, bool = true) {}
};

// C++ OpenCV baseline (always-available fallback).
struct CameraGeometry
{
  double x_offset{0.076}, height{0.103}, hfov_deg{62.2};
  int64_t width{640}, height_px{480};
  double range_max{1.2}, grid_resolution{0.004}, slope_limit_rad{0.0349066};
};
class OpenCvBaselineDetector : public LineDetectorBackend
{
public:
  explicit OpenCvBaselineDetector(const CameraGeometry & camera = {})
  : camera_(camera) {}
  cv::Point2d project_pixel(double u, double v) const;
  void set_attitude(double pitch, double roll, bool available = true) override
  {pitch_ = pitch; roll_ = roll; attitude_available_ = available;}
  std::string name() const override {return "opencv_baseline";}
  uint8_t id() const override;
  bool ready() const override {return true;}
  bool detect(const cv::Mat & bgr, LineObservation & out) override;

private:
  CameraGeometry camera_;
  double pitch_{0.}, roll_{0.};
  bool attitude_available_{true};
  bool projection_untrusted_{false};
  int recovery_frames_{0};
  cv::Mat map_x_, map_y_;
};

// Fast-SCNN segmentation via ONNX Runtime CPU.
// STUB: never ready, so the node falls back to the OpenCV baseline. TODO(D-15): load ONNX model.
class FastScnnDetector : public LineDetectorBackend
{
public:
  explicit FastScnnDetector(std::string model_path)
  : model_path_(std::move(model_path)) {}
  std::string name() const override {return "fast_scnn";}
  uint8_t id() const override;
  bool ready() const override {return false;}
  bool detect(const cv::Mat & bgr, LineObservation & out) override;

private:
  std::string model_path_;
};

// Returns the requested backend if ready, otherwise the OpenCV baseline.
// fallback_active is set when the requested backend could not be used.
std::unique_ptr<LineDetectorBackend> make_backend(
  const std::string & requested, const std::string & model_path, bool & fallback_active,
  const CameraGeometry & camera = {});

}  // namespace line_follower_perception
