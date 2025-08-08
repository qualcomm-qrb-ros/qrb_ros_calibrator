/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef IMAGE_PROCESSOR_HPP_
#define IMAGE_PROCESSOR_HPP_
#include <opencv2/opencv.hpp>
#include <vector>

#include "calibrator_lc/calib_data.hpp"
#include "parameters_io_lc/parameters_input.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
class ImageProcessor
{
private:
  CameraInfo camera_info_;
  ChessboardInfo chessboard_info_;
  std::vector<cv::Point3f> world_pts_;
  cv::Size pattern_size_;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  /**
   * @desc To solve the chessboard pose w.r.t camera frame
   * @param cornors Input detected cornor points in image frame
   * @param rot Output chessboard orientation w.r.t camera frame
   * @param t Output chessboard position w.r.t camera frame
   * @return void
   */
  void solve_pattern_pose(const std::vector<cv::Point2f> cornors, cv::Mat & rot, cv::Mat & t);

public:
  /**
   * @desc Initialize the image processor with parameters input
   * @param camera_info Input camera information
   * @param chessboard_info Input chessboard information
   * @return void
   */
  void initialize(const CameraInfo & camera_info, const ChessboardInfo & chessboard_info);
  bool find_chessboard(const cv::Mat & image);
  void get_target_pose(const cv::Mat & image, cv::Mat & rot, cv::Mat & t);
};
}  // namespace laser_cam_calibrator
}  // namespace qrb
#endif