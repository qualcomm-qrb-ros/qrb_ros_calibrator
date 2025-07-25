/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "data_collector/data_collector.hpp"

namespace qrb_ros
{
namespace laser_odom_collector
{
DataCollector::DataCollector(const std::string & laser_topic, const std::string & odom_topic)
  : Node("data_collector")
{
  RCLCPP_INFO(this->get_logger(), "data_collector node start");
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic, 1, std::bind(&DataCollector::laser_sub_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 1, std::bind(&DataCollector::odom_sub_callback, this, std::placeholders::_1));
}

void DataCollector::laser_sub_callback(sensor_msgs::msg::LaserScan::ConstPtr laser_msg)
{
  if (laser_captured_) {
    return;
  }
  if (last_laser_time_ != 0) {
    if (abs(last_laser_time_ - laser_msg->header.stamp.sec) < max_time_delay_) {
      return;
    }
  }

  if (collected_frame_ >= min_frame_num_) {
    if (odom_captured_) {
      laser_captured_ = true;
      if (last_laser_time_ == 0) {
        last_laser_time_ = laser_msg->header.stamp.sec;
      }
      last_laser_time_ = laser_msg->header.stamp.sec;
      qrb::laser_odom_calibrator::Laser_Data laser_data;
      laser_data.point_cloud = cumulative_point_cloud_;
      laser_data.can_be_used = true;
      laser_data_set.push_back(laser_data);
    } else {
      return;
    }
  } else {
    laser_msg2point_cloud(laser_msg, cumulative_point_cloud_);
    collected_frame_++;
  }
}

void DataCollector::odom_sub_callback(nav_msgs::msg::Odometry::ConstPtr odom_msg)
{
  if (true == odom_captured_) {
    return;
  }
  if (last_odom_time_ != 0) {
    if (abs(last_odom_time_ - odom_msg->header.stamp.sec) < max_time_delay_) {
      return;
    }
    if (abs(odom_msg->header.stamp.sec - last_laser_time_) < max_time_delay_) {
      return;
    }
  }

  if (last_odom_time_ == 0) {
    last_odom_time_ = odom_msg->header.stamp.sec;
  }

  qrb::laser_odom_calibrator::Odom_Data odom_data;
  odom_data.odom_pose_xy[0] = odom_msg->pose.pose.position.x;
  odom_data.odom_pose_xy[1] = odom_msg->pose.pose.position.y;
  Eigen::Quaterniond yaw_q;
  yaw_q.w() = odom_msg->pose.pose.orientation.w;
  yaw_q.x() = odom_msg->pose.pose.orientation.x;
  yaw_q.y() = odom_msg->pose.pose.orientation.y;
  yaw_q.z() = odom_msg->pose.pose.orientation.z;
  Eigen::Matrix3d rotation_yaw = yaw_q.toRotationMatrix();
  odom_data.odom_pose_yaw_rotation = rotation_yaw.block(0, 0, 2, 2);
  odom_data.currrent_yaw = rotation_yaw.eulerAngles(2, 1, 0)[0];
  odom_data.can_be_used = true;
  odom_data_set.push_back(odom_data);
  last_odom_time_ = odom_msg->header.stamp.sec;
  odom_captured_ = true;
}

void DataCollector::laser_msg2point_cloud(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    pcl::PointCloud<pcl::PointXYZ> & src_cloud)
{
  int scan_count = scan_msg->ranges.size();
  for (int i = 0; i < scan_count; ++i) {
    if (!std::isfinite(scan_msg->ranges[i]) || scan_msg->ranges[i] > max_range_) {
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

void DataCollector::start_capture()
{
  collected_frame_ = 0;
  odom_captured_ = false;
  laser_captured_ = false;
  cumulative_point_cloud_.clear();
}

bool DataCollector::current_capture_succeed()
{
  return odom_captured_ && laser_captured_;
}

}  // namespace laser_odom_collector
}  // namespace qrb_ros