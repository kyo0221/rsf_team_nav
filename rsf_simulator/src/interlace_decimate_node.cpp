#include "rsf_simulator/interlace_decimate_node.hpp"

#include <functional>
#include <memory>

#include "rsf_simulator/interlace_constants.hpp"
#include "rsf_simulator/interlace_decimate.hpp"

namespace rsf_simulator
{

InterlaceDecimateNode::InterlaceDecimateNode()
: Node("interlace_decimate_node"),
  interlace_(static_cast<int>(declare_parameter<int>("interlace", 1))),
  layout_checked_(false)
{
  if (interlace_ < 1) {
    RCLCPP_WARN(get_logger(), "interlace %d is invalid, using 1", interlace_);
    interlace_ = 1;
  }
  RCLCPP_INFO(get_logger(), "interlace=%d", interlace_);
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/rsf/hokuyo_cloud2", rclcpp::QoS(10));
  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rsf/hokuyo3d/points_raw", rclcpp::SensorDataQoS(),
    std::bind(&InterlaceDecimateNode::cloud_callback, this, std::placeholders::_1));
}

void InterlaceDecimateNode::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!layout_checked_) {
    layout_checked_ = true;
    const int oversample = oversample_factor(msg->width);
    if (oversample < 1) {
      RCLCPP_ERROR(
        get_logger(),
        "input width %u is not a %d-azimuth fine grid, passing through unchanged",
        msg->width, kAzimuthCount);
    } else if (oversample % interlace_ != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "interlace %d does not divide the SDF oversample factor %d, passing through unchanged",
        interlace_, oversample);
    } else {
      RCLCPP_INFO(get_logger(), "oversample=%d, azimuths per frame=%d", oversample, kAzimuthCount);
    }
  }
  publisher_->publish(decimate_interlace(*msg, interlace_));
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rsf_simulator::InterlaceDecimateNode>());
  rclcpp::shutdown();
  return 0;
}
