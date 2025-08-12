/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef PARAMETERS_READER_HPP_
#define PARAMETERS_READER_HPP_
#include <eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "parameters_io_lc/parameters_input.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
class ParametersIO
{
private:
  std::string input_file_name_;
  std::string output_file_name_;

public:
  ParametersIO();
  bool get_laser_process_parameters(qrb::laser_cam_calibrator::EnvParameters & env_info);
  bool get_cam_parameters(qrb::laser_cam_calibrator::CameraInfo & cam_info);
  bool get_relative_dist(double & dist);
  bool get_chessboard_parameters(qrb::laser_cam_calibrator::ChessboardInfo & chess_info);
  bool get_topic_names(std::string & laser_topic, std::string & image_topic);
  bool get_laser_axis(std::vector<std::string> & laser_xyz_axis);
  void save_extrinsic_parameters(const Eigen::Matrix3d & rotation_l2c,
      const Eigen::Vector3d & translation_l2c);
};
}  // namespace laser_cam_calibrator
}  // namespace qrb

#endif