/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <data_processor_lc/image_processor.hpp>
namespace qrb
{
namespace laser_cam_calibrator
{
void ImageProcessor::initialize(const CameraInfo & camera_info,
    const ChessboardInfo & chessboard_info)
{
  camera_info_ = camera_info;
  chessboard_info_ = chessboard_info;
  float height = chessboard_info_.square_height;
  float width = chessboard_info_.square_width;
  for (int i = 0; i < chessboard_info_.rows; ++i) {
    for (int j = 0; j < chessboard_info_.cols; ++j) {
      cv::Point3f pt;
      pt.x = i * height;
      pt.y = j * width;
      pt.z = 0.0;
      world_pts_.emplace_back(pt);
    }
  }
  pattern_size_.height = chessboard_info_.rows;
  pattern_size_.width = chessboard_info_.cols;
  intrinsic_ = camera_info.intrinsic;
  distortion_ = camera_info.distortion;
}
bool ImageProcessor::find_chessboard(const cv::Mat & image)
{
  std::vector<cv::Point2f> cornors;
  if (image.empty()) {
    std::cout << "no image input!" << std::endl;
    return false;
  }
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  bool detected = cv::findChessboardCorners(gray, pattern_size_, cornors);
  return detected;
}
void ImageProcessor::solve_pattern_pose(const std::vector<cv::Point2f> cornors,
    cv::Mat & rot,
    cv::Mat & t)
{
  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(world_pts_, cornors, intrinsic_, distortion_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
  cv::solvePnPRefineLM(world_pts_, cornors, intrinsic_, distortion_, rvec, tvec);
  cv::Rodrigues(rvec, rot);
  t = tvec / 1000.0;
}
void ImageProcessor::get_target_pose(const cv::Mat & image, cv::Mat & rot, cv::Mat & t)
{
  cv::Mat gray;
  std::vector<cv::Point2f> cornors;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  cv::findChessboardCorners(gray, pattern_size_, cornors);
  cv::find4QuadCornerSubpix(gray, cornors, cv::Size(3, 3));
  solve_pattern_pose(cornors, rot, t);
}
}  // namespace laser_cam_calibrator
}  // namespace qrb