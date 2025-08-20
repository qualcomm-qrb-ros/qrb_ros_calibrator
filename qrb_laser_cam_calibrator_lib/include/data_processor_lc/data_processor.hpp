/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef DATA_PROCESSOR_HPP_
#define DATA_PROCESSOR_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "data_processor_lc/image_processor.hpp"
#include "data_processor_lc/laser_processor.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
class DataProcessor
{
private:
  std::shared_ptr<ImageProcessor> image_processor_;
  std::shared_ptr<LaserProcessor> laser_processor_;
  CameraInfo cam_info_;
  ChessboardInfo chessboard_info_;
  std::vector<double> dists_;
  void solve_margin_line(CameraData & cam_data);
  void line_fit(const Eigen::Vector3d & pt1, const Eigen::Vector3d & pt2, Eigen::Vector3d & line);

public:
  DataProcessor();
  void set_cam_info(const CameraInfo & cam_info, const ChessboardInfo & chessboard_info);
  void set_env_paramaters(const EnvParameters & env_prameters);
  bool find_chessboard(const cv::Mat & image);
  /**
   * @desc To solve the laser plane parameters
   * @param laser_plane Input LaserPlane
   * @param plane Output laser plane ax + by + cz + d = 0
   * @return bool If chessboard detected, return true. If no chessboard detected, return false
   */
  void solve_laser_plane_parameters(LaserPlane & laser_plane, Eigen::Vector4d & plane);
  /**
   * @desc To solve the plane parameters from a frame of CameraData
   * @param cam_data_set Input captured camera data
   * @return void
   */
  void solve_chessboard_plane_parameters(CameraData & cam_data_set);
  void process_image_data(std::vector<CameraData> & cam_data_set);
  void process_laser_data(std::vector<LaserData> & laser_data_set);

  void update_parameters_detect(const double & max_dist_seen_as_continuous,
      const double & ransac_fitline_dist_th,
      LaserData & laser_data,
      const int & index);
};
}  // namespace laser_cam_calibrator
}  // namespace qrb
#endif