/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef PARAMETERS_IO_HPP_
#define PARAMETERS_IO_HPP_
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace qrb
{
namespace laser_odom_calibrator
{
class ParametersIO
{
private:
  double long_edge_length_;
  double short_edge_length_;
  std::string input_file_name_;
  std::string output_file_name_;

public:
  ParametersIO();
  bool get_env_parameters(double & long_edge_length, double & short_edge_length);
  bool get_laser_process_parameters(double & max_dist_seen_as_continuous,
      double & line_length_tolerance,
      double & ransac_fitline_dist_th,
      int & ransac_max_iterations,
      int & min_point_num_stop_ransac,
      double & min_proportion_stop_ransac);
  bool get_solver_parameters(double & diff_tolerance_laser_odom);
  void save_extrinsic_parameters(const Eigen::Matrix2d & rotation_l2o,
      const Eigen::Vector2d & translation_l2o);
};
}  // namespace laser_odom_calibrator
}  // namespace qrb

#endif