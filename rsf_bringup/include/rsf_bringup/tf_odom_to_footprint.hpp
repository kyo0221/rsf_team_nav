#ifndef RSF_BRINGUP__TF_ODOM_TO_FOOTPRINT_HPP_
#define RSF_BRINGUP__TF_ODOM_TO_FOOTPRINT_HPP_

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rsf_bringup
{

class TfOdomToFootprint : public rclcpp::Node
{
public:
  TfOdomToFootprint();

private:
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::string odom_frame_;
  std::string sensor_frame_;
  std::string base_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}

#endif
