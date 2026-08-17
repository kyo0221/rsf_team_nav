#ifndef RSF_SIMULATOR__INTERLACE_DECIMATE_NODE_HPP_
#define RSF_SIMULATOR__INTERLACE_DECIMATE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rsf_simulator
{

class InterlaceDecimateNode : public rclcpp::Node
{
public:
  InterlaceDecimateNode();

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  int interlace_;
  bool layout_checked_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}

#endif
