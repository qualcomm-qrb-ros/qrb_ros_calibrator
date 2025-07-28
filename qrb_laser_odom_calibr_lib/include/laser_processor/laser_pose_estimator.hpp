/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef LASER_POSE_ESTIMATOR_HPP_
#define LASER_POSE_ESTIMATOR_HPP_
#include <pcl/PCLHeader.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>
#include <queue>
#include <vector>

#include "calibrator/calib_data.hpp"

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
  void frame2frame_pose_estimation(std::vector<LaserData> & laser_datas);
};

}  // namespace laser_odom_calibrator
}  // namespace qrb
#endif
