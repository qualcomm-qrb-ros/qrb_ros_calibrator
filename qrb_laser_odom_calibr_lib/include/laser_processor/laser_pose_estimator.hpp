/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef LASER_POSE_ESTIMATOR_HPP_
#define LASER_POSE_ESTIMATOR_HPP_
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLHeader.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <queue>
#include "qrb_laser_odom_calibr_lib/calib_data.hpp"

namespace qrb
{
namespace laser_odom_calibrator
{
class LaserPoseEstimator
{
private:
public:
  LaserPoseEstimator();
  ~LaserPoseEstimator();
  /**
   * @desc Calculate the frame to frame pose in laser data set
   * @param laser_datas. Input laser data set
   * @return void
   */
  void frame2frame_pose_estimation(std::vector<Laser_Data> & laser_datas);
};

}  // namespace laser_odom_calibrator
}  // namespace qrb
#endif