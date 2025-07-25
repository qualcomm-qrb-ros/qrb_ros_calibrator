/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef DATA_COLLECTOR_HPP_
#define DATA_COLLECTOR_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "calibrator/calib_data.hpp"
#include <vector>
namespace qrb_ros
{
namespace laser_odom_collector
{
class DataCollector : public rclcpp::Node
{
public:
  void start_capture();
  std::vector<qrb::laser_odom_calibrator::Laser_Data> laser_data_set;
  std::vector<qrb::laser_odom_calibrator::Odom_Data> odom_data_set;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_{ nullptr };
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{ nullptr };
  bool current_capture_succeed();
  DataCollector(const std::string & laser_topic, const std::string & odom_topic);

private:
  void laser_sub_callback(sensor_msgs::msg::LaserScan::ConstPtr laser_msg);
  void odom_sub_callback(nav_msgs::msg::Odometry::ConstPtr odom_msg);
  void laser_msg2point_cloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
      pcl::PointCloud<pcl::PointXYZ> & src_cloud);
  bool laser_captured_ = true;
  bool odom_captured_ = true;
  pcl::PointCloud<pcl::PointXYZ> cumulative_point_cloud_;
  int32_t collected_frame_ = 0;
  double max_range_ = 30.0;
  int32_t last_laser_time_ = 0;
  int32_t last_odom_time_ = 0;
  int32_t min_frame_num_ = 4;
  int32_t max_time_delay_ = 3;
};
}  // namespace laser_odom_collector
}  // namespace qrb_ros

#endif
