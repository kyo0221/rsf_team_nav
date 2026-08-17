#include "rsf_simulator/interlace_decimate.hpp"

#include <cstring>
#include <vector>

#include "rsf_simulator/interlace_constants.hpp"

namespace rsf_simulator
{

namespace
{
constexpr uint32_t kAzimuthSpan = static_cast<uint32_t>(kAzimuthCount - 1);
}

int oversample_factor(uint32_t width)
{
  if (width <= kAzimuthSpan || (width - 1) % kAzimuthSpan != 0) {
    return 0;
  }
  return static_cast<int>((width - 1) / kAzimuthSpan);
}

int interlace_frame_index(const builtin_interfaces::msg::Time & stamp, int interlace)
{
  if (interlace < 1) {
    return 0;
  }
  const long long ns = stamp.sec * 1000000000LL + stamp.nanosec;
  return static_cast<int>((ns / kScanPeriodNs) % interlace);
}

sensor_msgs::msg::PointCloud2 decimate_interlace(
  const sensor_msgs::msg::PointCloud2 & input, int interlace)
{
  const int oversample = oversample_factor(input.width);
  if (interlace < 1 || oversample < 1 || oversample % interlace != 0) {
    return input;
  }

  const uint32_t step = static_cast<uint32_t>(oversample);
  const uint32_t phase = static_cast<uint32_t>(
    interlace_frame_index(input.header.stamp, interlace) * (oversample / interlace));

  std::vector<uint32_t> columns;
  for (uint32_t c = phase; c < input.width; c += step) {
    columns.push_back(c);
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.height = input.height;
  output.width = static_cast<uint32_t>(columns.size());
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.point_step * output.width;
  output.is_dense = input.is_dense;
  output.data.resize(static_cast<size_t>(output.row_step) * output.height);

  for (uint32_t row = 0; row < input.height; ++row) {
    for (size_t i = 0; i < columns.size(); ++i) {
      std::memcpy(
        &output.data[(static_cast<size_t>(row) * output.width + i) * output.point_step],
        &input.data[(static_cast<size_t>(row) * input.width + columns[i]) * input.point_step],
        input.point_step);
    }
  }

  return output;
}

}
