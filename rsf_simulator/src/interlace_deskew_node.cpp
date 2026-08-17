#include "rsf_simulator/interlace_deskew_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>

#include "rsf_simulator/interlace_deskew.hpp"

namespace rsf_simulator
{

InterlaceDeskewNode::InterlaceDeskewNode()
: Node("interlace_deskew_node"),
  interlace_(static_cast<int>(declare_parameter<int>("interlace", 1)))
{
  if (interlace_ < 1 || interlace_ > 20) {
    RCLCPP_WARN(get_logger(), "interlace %d out of range [1, 20], clamped", interlace_);
    interlace_ = std::clamp(interlace_, 1, 20);
  }
  RCLCPP_INFO(get_logger(), "interlace=%d", interlace_);
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/rsf/hokuyo_cloud2", rclcpp::QoS(10));
  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rsf/hokuyo3d/points_raw", rclcpp::SensorDataQoS(),
    std::bind(&InterlaceDeskewNode::cloud_callback, this, std::placeholders::_1));
}

void InterlaceDeskewNode::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  publisher_->publish(deskew_interlace(*msg, interlace_));
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rsf_simulator::InterlaceDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
