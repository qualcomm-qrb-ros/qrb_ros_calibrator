/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef PARAMETERS_INPUT_HPP_
#define PARAMETERS_INPUT_HPP_
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
namespace qrb
{
namespace laser_cam_calibrator
{
struct CameraInfo
{
  cv::Mat intrinsic;
  cv::Mat distortion;
};
struct ChessboardInfo
{
  int rows;
  int cols;
  float square_height;
  float square_width;
  double left_margin_length;
  double right_margin_length;
  double up_margin_length;
  double down_margin_length;
};
struct EnvParameters
{
  double max_dist_seen_as_continuous;
  double ransac_fitline_dist_th;
  int ransac_max_iterations;
  int min_point_num;
  double min_proportion;
  double chessboard_length_in_laser_frame;
};
}  // namespace laser_cam_calibrator
}  // namespace qrb

#endif