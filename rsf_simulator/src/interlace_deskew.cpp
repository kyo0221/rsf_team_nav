#include "rsf_simulator/interlace_deskew.hpp"

#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rsf_simulator
{

namespace
{
constexpr double kScanPeriod = 0.05;
constexpr double kHorizontalPitch = 0.10471975511965977;
}

int interlace_frame_index(const builtin_interfaces::msg::Time & stamp, int interlace)
{
  const double t = stamp.sec + stamp.nanosec * 1e-9;
  return static_cast<int>(std::llround(t / kScanPeriod) % interlace);
}

double interlace_offset_angle(int frame_index, int interlace)
{
  return frame_index * kHorizontalPitch / interlace;
}

sensor_msgs::msg::PointCloud2 deskew_interlace(
  const sensor_msgs::msg::PointCloud2 & input, int interlace)
{
  sensor_msgs::msg::PointCloud2 output = input;
  if (interlace <= 1) {
    return output;
  }
  const double angle =
    interlace_offset_angle(interlace_frame_index(input.header.stamp, interlace), interlace);
  const float c = static_cast<float>(std::cos(angle));
  const float s = static_cast<float>(std::sin(angle));
  sensor_msgs::PointCloud2Iterator<float> x_it(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(output, "y");
  for (; x_it != x_it.end(); ++x_it, ++y_it) {
    const float x = *x_it;
    const float y = *y_it;
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    *x_it = c * x - s * y;
    *y_it = s * x + c * y;
  }
  return output;
}

}
