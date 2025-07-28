/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef CALIB_DATA_HPP_
#define CALIB_DATA_HPP_
#include <pcl/PCLHeader.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>

namespace qrb
{
namespace laser_odom_calibrator
{
struct LaserData
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  Eigen::Vector2d last2current_xy;
  double last_yaw_in_current;
  Eigen::Matrix2d last2current_rotation;
  Eigen::Vector2d intersaction_point;
  Eigen::Vector2d long_edge_direction;
  pcl::PointXYZ long_end_pt;
  Eigen::Vector2d short_edge_direction;
  pcl::PointXYZ short_end_pt;
  bool can_be_used;
};
struct OdomData
{
  Eigen::Vector2d odom_pose_xy;
  Eigen::Matrix2d odom_pose_yaw_rotation;
  double currrent_yaw;
  double currrent_in_last_yaw;
  Eigen::Vector2d last2current_xy;
  Eigen::Matrix2d last2current_rotation;
  bool can_be_used;
};
}  // namespace laser_odom_calibrator
}  // namespace qrb

#endif