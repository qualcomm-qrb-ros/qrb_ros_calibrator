/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "data_collector/data_collector.hpp"
namespace qrb_ros
{
namespace laser_cam_collector
{
DataCollector::DataCollector(const std::string & image_topic, const std::string & laser_topic)
  : Node("data_collector")
{
  image_captured_ = true;
  laser_captured_ = true;
  capture_laser_plane_image_ = false;
  laser_sub_ = nullptr;
  image_sub_ = nullptr;
  last_laser_time_ = 0;
  last_image_time_ = 0;
  collected_frame_ = 0;
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 30, std::bind(&DataCollector::image_sub_callback, this, std::placeholders::_1));
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic, 1, std::bind(&DataCollector::laser_sub_callback, this, std::placeholders::_1));
}
void DataCollector::laser_sub_callback(sensor_msgs::msg::LaserScan::ConstPtr laser_msg)
{
  if (laser_captured_) {
    return;
  }
  if (last_laser_time_ != 0) {
    if (abs(last_laser_time_ - laser_msg->header.stamp.sec) < MAX_TIME_DELAY) {
      return;
    }
  }

  if (collected_frame_ >= MAX_FRAME) {
    if (image_captured_) {
      laser_captured_ = true;
      if (last_laser_time_ == 0) {
        last_laser_time_ = laser_msg->header.stamp.sec;
      }
      last_laser_time_ = laser_msg->header.stamp.sec;
      qrb::laser_cam_calibrator::LaserData laser_data;
      laser_data.point_cloud = cumulative_point_cloud_;
      laser_data.can_be_used = true;
      laser_data_set.emplace_back(laser_data);
      std::cout << "current data capture done" << std::endl;
    } else {
      return;
    }
  } else {
    laser_msg2point_cloud(laser_msg, cumulative_point_cloud_);
    collected_frame_++;
  }
}
void DataCollector::image_sub_callback(sensor_msgs::msg::Image::SharedPtr image_msg)
{
  if (image_captured_) {
    return;
  }
  if (last_image_time_ != 0) {
    if (abs(last_image_time_ - image_msg->header.stamp.sec) < MAX_TIME_DELAY) {
      return;
    }
  }
  if (last_image_time_ == 0) {
    last_image_time_ = image_msg->header.stamp.sec;
  }
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;
  if (capture_laser_plane_image_) {
    laser_plane_image.laser_plane_image = image.clone();
    image_captured_ = true;
    capture_laser_plane_image_ = false;
  } else {
    image_captured_ = true;
    qrb::laser_cam_calibrator::CameraData camera_data;
    camera_data.image = image.clone();
    camera_data_set.emplace_back(camera_data);
  }
}

bool DataCollector::current_capture_succeed()
{
  return image_captured_ && laser_captured_;
}
bool DataCollector::laser_plane_image_capture_succeed()
{
  return image_captured_;
}

void DataCollector::start_capture()
{
  collected_frame_ = 0;
  image_captured_ = false;
  laser_captured_ = false;
  cumulative_point_cloud_.clear();
}
void DataCollector::capture_laser_plane_image()
{
  image_captured_ = false;
  capture_laser_plane_image_ = true;
}

void DataCollector::laser_msg2point_cloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    pcl::PointCloud<pcl::PointXYZ> & src_cloud)
{
  size_t scan_count = scan_msg->ranges.size();
  for (size_t i = 0; i < scan_count; ++i) {
    if (!std::isfinite(scan_msg->ranges[i]) || scan_msg->ranges[i] > MAX_RANGE) {
      continue;
    }
    double angle_each = scan_msg->angle_min + scan_msg->angle_increment * i;
    pcl::PointXYZ src_point;
    src_point.x = scan_msg->ranges[i] * cos(angle_each);
    src_point.y = scan_msg->ranges[i] * sin(angle_each);
    src_point.z = 0;
    src_cloud.push_back(src_point);
  }
}
}  // namespace laser_cam_collector
}  // namespace qrb_ros
