#ifndef RSF_SIMULATOR__INTERLACE_DESKEW_HPP_
#define RSF_SIMULATOR__INTERLACE_DESKEW_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rsf_simulator
{

int interlace_frame_index(const builtin_interfaces::msg::Time & stamp, int interlace);
double interlace_offset_angle(int frame_index, int interlace);
sensor_msgs::msg::PointCloud2 deskew_interlace(
  const sensor_msgs::msg::PointCloud2 & input, int interlace);

}

#endif
