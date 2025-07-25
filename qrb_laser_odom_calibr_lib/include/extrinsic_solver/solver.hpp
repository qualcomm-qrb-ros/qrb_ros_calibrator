/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef SLOVER_HPP_
#define SLOVER_HPP_
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include "calibrator/calib_data.hpp"
#include <vector>
#define MIN_OBSERVE_NUM 3

namespace qrb
{
namespace laser_odom_calibrator
{
class Slover
{
private:
  Eigen::Vector2d extrinsic_xy_;
  Eigen::Matrix2d extrinsic_yaw_;
  double extrinsic_yaw_angle_;
  double diff_tolerance_laser_odom_;
  int min_observe_num_;
  bool solve_linear_equation(std::vector<laser_odom_calibrator::Odom_Data> & odom_datas,
      std::vector<laser_odom_calibrator::Laser_Data> & laser_datas);
  void rotation2quaternion(const Eigen::Matrix2d & rotation, double * quaternion);
  void quaternion2rotation(const double * quaternion, Eigen::Matrix2d & rotation);

public:
  /**
   * @desc Execute the calibtration
   * @param odom_datas Input odometry data set
   * @param laser_datas. Input laser data set
   * @return void
   */
  bool calibrate(std::vector<laser_odom_calibrator::Odom_Data> & odom_datas,
      std::vector<laser_odom_calibrator::Laser_Data> & laser_datas);
  void set_tolerance(const double & diff_tolerance_laser_odom);
  void print_extrinsic_ladar2odom();
  void get_extrinsic(Eigen::Matrix2d & rotation, Eigen::Vector2d & translation);
  Slover();
  ~Slover();
};
}  // namespace laser_odom_calibrator
}  // namespace qrb
#endif