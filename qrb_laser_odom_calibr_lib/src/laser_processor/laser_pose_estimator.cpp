/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "laser_processor/laser_pose_estimator.hpp"

namespace qrb
{
namespace laser_odom_calibrator
{

LaserPoseEstimator::LaserPoseEstimator() {}

LaserPoseEstimator::~LaserPoseEstimator() {}

void LaserPoseEstimator::frame2frame_pose_estimation(std::vector<LaserData> & laser_datas)
{
  for (size_t i = 1; i < laser_datas.size(); ++i) {
    if (laser_datas[i].can_be_used == false) {
      continue;
    }
    if (laser_datas[i - 1].can_be_used == false) {
      continue;
    }

    Eigen::Vector2d last_long_dir = laser_datas[i - 1].long_edge_direction;
    Eigen::Vector2d last_short_dir = laser_datas[i - 1].short_edge_direction;
    Eigen::Vector2d current_long_dir = laser_datas[i].long_edge_direction;
    Eigen::Vector2d current_short_dir = laser_datas[i].short_edge_direction;
    double norm_inv1 = 1.0 / last_long_dir.dot(current_long_dir);
    double norm_inv2 = 1.0 / last_short_dir.dot(current_short_dir);
    // least-square solution
    double cos_theta = (norm_inv1 + norm_inv2) / (norm_inv1 * norm_inv1 + norm_inv2 * norm_inv2);
    double last_yaw_ref = atan2(last_long_dir[1], last_long_dir[0]);
    double current_yaw_ref = atan2(current_long_dir[1], current_long_dir[0]);
    double last_yaw_in_current_frame;
    last_yaw_in_current_frame = acos(fminl(fmaxl(cos_theta, -1.0), 1.0));
    if (last_yaw_ref - current_yaw_ref > 0) {
      last_yaw_in_current_frame *= -1.;
    } else {
      last_yaw_in_current_frame = last_yaw_in_current_frame;
    }
    laser_datas[i].last_yaw_in_current = last_yaw_in_current_frame;
    Eigen::Matrix2d rotation;
    rotation << cos(last_yaw_in_current_frame), -sin(last_yaw_in_current_frame),
        sin(last_yaw_in_current_frame), cos(last_yaw_in_current_frame);
    laser_datas[i].last2current_rotation = rotation;
    Eigen::Vector2d translation;
    Eigen::Vector2d current_pt = laser_datas[i].intersaction_point;
    Eigen::Vector2d last_pt = laser_datas[i - 1].intersaction_point;
    translation = current_pt - laser_datas[i].last2current_rotation * last_pt;
    laser_datas[i].last2current_xy = translation;
  }
}

}  // namespace laser_odom_calibrator
}  // namespace qrb