/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef DATA_COLLECTOR_HPP_
#define DATA_COLLECTOR_HPP_
#include <opencv2/opencv.hpp>

#include "calibrator_lc/calib_data.hpp"
#include "rclcpp/version.h"
#if RCLCPP_VERSION_MAJOR >= 20
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define MAX_TIME_DELAY 3
#define MAX_RANGE 30.0
#define MAX_FRAME 3
namespace qrb_ros
{
namespace laser_cam_collector
{
class DataCollector : public rclcpp::Node
{
public:
  void start_capture();
  void capture_laser_plane_image();
  std::vector<qrb::laser_cam_calibrator::CameraData> camera_data_set;
  std::vector<qrb::laser_cam_calibrator::LaserData> laser_data_set;
  qrb::laser_cam_calibrator::LaserPlane laser_plane_image;
  bool current_capture_succeed();
  bool laser_plane_image_capture_succeed();
  DataCollector(const std::string & image_topic, const std::string & laser_topic);

private:
  bool image_captured_;
  bool laser_captured_;
  bool capture_laser_plane_image_;
  int32_t last_laser_time_;
  int32_t last_image_time_;
  int32_t collected_frame_;
  pcl::PointCloud<pcl::PointXYZ> cumulative_point_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  void laser_sub_callback(sensor_msgs::msg::LaserScan::ConstPtr laser_msg);
  void image_sub_callback(sensor_msgs::msg::Image::SharedPtr image_msg);
  void laser_msg2point_cloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
      pcl::PointCloud<pcl::PointXYZ> & src_cloud);
};
}  // namespace laser_cam_collector
}  // namespace qrb_ros
#endif
