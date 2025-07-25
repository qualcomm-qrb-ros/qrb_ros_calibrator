/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef ODOM_DATA_PROCESSOR_HPP_
#define ODOM_DATA_PROCESSOR_HPP_
#include "qrb_laser_odom_calibr_lib/calib_data.hpp"
#include <vector>

namespace qrb
{
namespace laser_odom_calibrator
{
class OdomDataProcessor
{
private:
public:
  /**
   * @desc Calculate the frame to frame pose in odometry data set
   * @param laser_datas. Input odometry data set
   * @return N/A
   */
  void frame2frame_pose_estimation(std::vector<qrb::laser_odom_calibrator::Odom_Data> & odom_datas);
};
}  // namespace laser_odom_calibrator
}  // namespace qrb
#endif