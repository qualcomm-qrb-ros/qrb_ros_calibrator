/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef CALIB_DATA_HPP_
#define CALIB_DATA_HPP_
#include <pcl/PCLHeader.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
namespace qrb
{
namespace laser_cam_calibrator
{
struct CameraData
{
  cv::Mat image;
  Eigen::Matrix3d target_orientation;
  Eigen::Vector3d target_xyz;
  Eigen::Vector4d chessboard_plane;
  Eigen::Vector3d left_margin_line;
  Eigen::Vector3d right_margin_line;
  Eigen::Vector3d up_margin_line;
  Eigen::Vector3d down_margin_line;
  double distance;
};
struct LaserData
{
  bool can_be_used;
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::PointCloud<pcl::PointXYZ> selected_line_seg;
  Eigen::Vector3d line_parameter;
  std::vector<Eigen::Vector2d> pts_in_line;
  Eigen::Vector2d line_dir;
};
struct LaserPlane
{
  Eigen::Matrix3d chessboard_orientation;
  cv::Mat laser_plane_image;
  double dist_from_laser2chessboard_origin;
};
struct PointLinePair
{
  double laser_pt1[3];
  double image_line1[3];
  double laser_pt2[3];
  double image_line2[3];
};
struct PointPlanePair
{
  double laser_pt1[3];
  double plane[4];
  double laser_pt2[3];
};
struct Extrinsic
{
  Eigen::Matrix3d R_c2l;
  Eigen::Vector3d t_c2l;
};
}  // namespace laser_cam_calibrator
}  // namespace qrb

#endif