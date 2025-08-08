/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_
#include <eigen3/Eigen/Core>

#include "calibrator_lc/calib_data.hpp"
#include "data_processor_lc/data_processor.hpp"
#include "extrinsic_solver_lc/solver.hpp"
#include "parameters_io_lc/parameters_input.hpp"
#include "parameters_io_lc/parameters_io.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
class Calibrator
{
private:
  CameraInfo cam_info_;
  ChessboardInfo chessboard_info_;
  EnvParameters env_info_;
  LaserPlane laser_plane_;
  std::vector<std::string> laser_axis_;
  std::shared_ptr<DataProcessor> data_processor_;
  std::shared_ptr<Solver> solver_;
  std::shared_ptr<ParametersIO> parameters_io_;
  double relative_dist_;
  void draw_line(cv::Mat & image, const Eigen::Vector3d & image_line);

public:
  std::vector<CameraData> cam_data_set;
  std::vector<LaserData> laser_data_set;
  /**
   * @desc Initialize the calibrator
   * @param void
   * @return void
   */
  void initialize();
  /**
   * @desc Set the laser plane for initial rotation guess
   * @param laser_plane Captured laser plane
   * @return void
   */
  void set_laser_plane(const LaserPlane & laser_plane);
  /**
   * @desc Set the laser axis w.r.t chessboard coordinate frame for initial rotation guess
   * @param laser_axis The axises correspondence between laser axis and chessboard axis
   * @return void
   */
  void set_laser_axis(const std::vector<std::string> & laser_axis);
  /**
   * @desc To Check if there is a chessboard in the image
   * @param image Input image
   * @return bool If chessboard detected, return true. If no chessboard detected, return false
   */
  bool find_chessboard(const cv::Mat & image);
  /**
   * @desc Give the captured data to calibrator
   * @param cam_data_set_in Input camera data set
   * @param laser_data_set_in Input laser data set
   * @return void
   */
  void input_data(const std::vector<CameraData> & cam_data_set_in,
      const std::vector<LaserData> & laser_data_set_in);
  /**
   * @desc using specific parameters to detect the line in i-th laser data
   * @param id The index of data set
   * @param max_dist_seen_as_continuous The distance that two points can be seen as continuous
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @return void
   */
  void update_parameters_detect(const int & index,
      const double & max_dist_seen_as_continuous,
      const double & ransac_fitline_dist_th);
  /**
   * @desc Get the user input parameters
   * @param max_dist_seen_as_continuous The distance that two points can be seen as continuous
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @return void
   */
  void get_parameters_adjust(double & max_dist_seen_as_continuous, double & ransac_fitline_dist_th);
  /**
   * @desc Get the ROS topic name for data subscription
   * @param laser_topic Laser topic name
   * @param image_topic Image topic name
   * @return void
   */
  void get_topic_name(std::string & laser_topic, std::string & image_topic);
  /**
   * @desc Based on the calibrated result, the laser points can be projected into image.
   * @param project_result_img Input image set
   * @return void
   */
  void draw_projection_result(std::vector<cv::Mat> & project_result_img);
  /**
   * @desc Excute calibration
   * @param void
   * @return void
   */
  bool calibrate();
  /**
   * @desc Excute Save the calibration result in extrinsic.xml file
   * @param void
   * @return void
   */
  void save_result();
  Eigen::Matrix3d R_l2c;
  Eigen::Vector3d t_l2c;
};
}  // namespace laser_cam_calibrator
}  // namespace qrb

#endif