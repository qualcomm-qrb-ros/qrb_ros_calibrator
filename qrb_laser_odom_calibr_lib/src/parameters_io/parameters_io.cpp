/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "parameters_io/parameters_io.hpp"

namespace qrb
{
namespace laser_odom_calibrator
{
ParametersIO::ParametersIO()
{
  input_file_name_ = "parameters_input.yaml";
  output_file_name_ = "extrinsic.yaml";
}

bool ParametersIO::get_env_parameters(double & long_edge_length, double & short_edge_length)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  long_edge_length = f_reader_["long_edge_length"];
  short_edge_length = f_reader_["short_edge_length"];
  return true;
}

void ParametersIO::save_extrinsic_parameters(const Eigen::Matrix2d & rotation_l2o,
    const Eigen::Vector2d & translation_l2o)
{
  cv::FileStorage f_writer_(output_file_name_, cv::FileStorage::WRITE);
  cv::Mat rotation;
  cv::Mat translation;
  if (!f_writer_.isOpened()) {
    std::cout << "rotation_l2o: \n" << rotation << std::endl;
    std::cout << "Write file failed!" << std::endl;
    return;
  }
  cv::eigen2cv(rotation_l2o, rotation);
  cv::eigen2cv(translation_l2o, translation);
  f_writer_ << "rotation_l2o" << rotation;
  f_writer_ << "translation_l2o" << translation;
  f_writer_.release();
}

bool ParametersIO::get_laser_process_parameters(double & max_dist_seen_as_continuous,
    double & line_length_tolerance,
    double & ransac_fitline_dist_th,
    int & ransac_max_iterations,
    int & min_point_num_stop_ransac,
    double & min_proportion_stop_ransac)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  max_dist_seen_as_continuous = f_reader_["max_dist_seen_as_continuous"];
  line_length_tolerance = f_reader_["line_length_tolerance"];
  ransac_fitline_dist_th = f_reader_["ransac_fitline_dist_th"];
  ransac_max_iterations = f_reader_["ransac_max_iterations"];
  min_point_num_stop_ransac = f_reader_["min_point_num_stop_ransac"];
  min_proportion_stop_ransac = f_reader_["min_proportion_stop_ransac"];
  f_reader_.release();
  return true;
}

bool ParametersIO::get_solver_parameters(double & diff_tolerance_laser_odom)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  diff_tolerance_laser_odom = f_reader_["diff_tolerance_laser_odom"];
  f_reader_.release();
  return true;
}
}  // namespace laser_odom_calibrator
}  // namespace qrb