#ifndef RSF_SIMULATOR__INTERLACE_DESKEW_NODE_HPP_
#define RSF_SIMULATOR__INTERLACE_DESKEW_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rsf_simulator
{

class InterlaceDeskewNode : public rclcpp::Node
{
public:
  InterlaceDeskewNode();

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  int interlace_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}

#endif
