#ifndef RSF_SIMULATOR__INTERLACE_DECIMATE_HPP_
#define RSF_SIMULATOR__INTERLACE_DECIMATE_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rsf_simulator
{

int oversample_factor(uint32_t width);

int interlace_frame_index(const builtin_interfaces::msg::Time & stamp, int interlace);

sensor_msgs::msg::PointCloud2 decimate_interlace(
  const sensor_msgs::msg::PointCloud2 & input, int interlace);

}

#endif
