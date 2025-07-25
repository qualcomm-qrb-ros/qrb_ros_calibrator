/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "odom_processor/odom_data_processor.hpp"

namespace qrb
{
namespace laser_odom_calibrator
{
void OdomDataProcessor::frame2frame_pose_estimation(std::vector<Odom_Data> & odom_datas)
{
  for (size_t i = 1; i < odom_datas.size(); ++i) {
    Eigen::Matrix2d last_yaw_rotation = odom_datas[i - 1].odom_pose_yaw_rotation;
    Eigen::Vector2d last_xy = odom_datas[i - 1].odom_pose_xy;
    Eigen::Matrix2d current_yaw_rotation = odom_datas[i].odom_pose_yaw_rotation;
    Eigen::Vector2d current_xy = odom_datas[i].odom_pose_xy;
    Eigen::Matrix2d last2current_rot = current_yaw_rotation.transpose() * last_yaw_rotation;
    double yaw_rad = atan2(-last2current_rot(0, 1), last2current_rot(0, 0));
    Eigen::Vector2d last2current_xy = current_yaw_rotation.transpose() * (last_xy - current_xy);
    odom_datas[i].last2current_xy = last2current_xy;
    odom_datas[i].last2current_rotation = last2current_rot;
  }
}
}  // namespace laser_odom_calibrator
}  // namespace qrb
