#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rsf_simulator/interlace_deskew.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<std::array<float, 3>> & points, int32_t sec, uint32_t nanosec)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "rsf_hokuyo3d";
  cloud.header.stamp.sec = sec;
  cloud.header.stamp.nanosec = nanosec;
  cloud.height = 1;
  cloud.width = points.size();
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> i_it(cloud, "intensity");
  for (const auto & p : points) {
    *x_it = p[0];
    *y_it = p[1];
    *z_it = p[2];
    *i_it = 1.0f;
    ++x_it;
    ++y_it;
    ++z_it;
    ++i_it;
  }
  return cloud;
}

}

TEST(InterlaceFrameIndex, ZeroStampIsZero) {
  builtin_interfaces::msg::Time stamp;
  EXPECT_EQ(rsf_simulator::interlace_frame_index(stamp, 4), 0);
}

TEST(InterlaceFrameIndex, SecondFrameIsOne) {
  builtin_interfaces::msg::Time stamp;
  stamp.nanosec = 50000000;
  EXPECT_EQ(rsf_simulator::interlace_frame_index(stamp, 4), 1);
}

TEST(InterlaceFrameIndex, WrapsAfterCycle) {
  builtin_interfaces::msg::Time stamp;
  stamp.nanosec = 200000000;
  EXPECT_EQ(rsf_simulator::interlace_frame_index(stamp, 4), 0);
}

TEST(InterlaceFrameIndex, RoundsToNearestGrid) {
  builtin_interfaces::msg::Time stamp;
  stamp.nanosec = 49900000;
  EXPECT_EQ(rsf_simulator::interlace_frame_index(stamp, 4), 1);
}

TEST(InterlaceOffsetAngle, QuarterPitch) {
  EXPECT_NEAR(rsf_simulator::interlace_offset_angle(1, 4), 0.026179938779914945, 1e-12);
}

TEST(DeskewInterlace, IdentityWhenInterlaceOne) {
  const auto input = make_cloud({{1.0f, 0.0f, 0.0f}}, 0, 50000000);
  const auto output = rsf_simulator::deskew_interlace(input, 1);
  EXPECT_EQ(output.data, input.data);
}

TEST(DeskewInterlace, IdentityAtPhaseZero) {
  const auto input = make_cloud({{1.0f, 2.0f, 3.0f}}, 0, 0);
  const auto output = rsf_simulator::deskew_interlace(input, 4);
  EXPECT_EQ(output.data, input.data);
}

TEST(DeskewInterlace, RotatesByOffset) {
  const auto input = make_cloud({{1.0f, 0.0f, 0.0f}}, 0, 50000000);
  const auto output = rsf_simulator::deskew_interlace(input, 4);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(output, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(output, "y");
  EXPECT_NEAR(*x_it, std::cos(0.026179938779914945), 1e-6);
  EXPECT_NEAR(*y_it, std::sin(0.026179938779914945), 1e-6);
}

TEST(DeskewInterlace, KeepsNonFinitePoints) {
  const float inf = std::numeric_limits<float>::infinity();
  const auto input = make_cloud({{inf, inf, 0.0f}}, 0, 50000000);
  const auto output = rsf_simulator::deskew_interlace(input, 4);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(output, "x");
  EXPECT_TRUE(std::isinf(*x_it));
}

TEST(DeskewInterlace, PreservesMetadata) {
  const auto input = make_cloud({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}}, 3, 50000000);
  const auto output = rsf_simulator::deskew_interlace(input, 4);
  EXPECT_EQ(output.header.frame_id, input.header.frame_id);
  EXPECT_EQ(output.width, input.width);
  EXPECT_EQ(output.height, input.height);
  EXPECT_EQ(output.fields.size(), input.fields.size());
}
