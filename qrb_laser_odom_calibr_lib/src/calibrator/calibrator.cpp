/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "calibrator/calibrator.hpp"

namespace qrb
{
namespace laser_odom_calibrator
{
Calibrator::Calibrator(std::vector<Laser_Data> & laser_data_set_in,
    std::vector<Odom_Data> & odom_data_set_in)
  : laser_data_set(laser_data_set_in), odom_data_set(odom_data_set_in)
{
  laser_pose_estimator_ = nullptr;
  laser_data_processor_ = nullptr;
  odom_data_processor_ = nullptr;
  parameters_io_ = nullptr;
  extrinsic_solver_ = nullptr;
  min_data_size_ = MIN_DATA_SIZE;
  initialize();
}

bool Calibrator::initialize()
{
  parameters_io_ = std::make_shared<ParametersIO>();
  double long_edge_length;
  double short_edge_length;
  double max_dist_seen_as_continuous;
  double line_length_tolerance;
  double ransac_fitline_dist_th;
  int ransac_max_iterations;
  int min_point_num_stop_ransac;
  double min_proportion_stop_ransac;
  double diff_tolerance_laser_odom;
  bool initialize_done = parameters_io_->get_env_parameters(long_edge_length, short_edge_length);
  if (long_edge_length < 0.0 || short_edge_length < 0.0) {
    std::cout << "invalid parametes input " << std::endl;
    return false;
  }
  if (long_edge_length < short_edge_length) {
    std::cout << "invalid parametes input " << std::endl;
    return false;
  }
  initialize_done =
      initialize_done && parameters_io_->get_laser_process_parameters(max_dist_seen_as_continuous,
                             line_length_tolerance, ransac_fitline_dist_th, ransac_max_iterations,
                             min_point_num_stop_ransac, min_proportion_stop_ransac);
  initialize_done =
      initialize_done && parameters_io_->get_solver_parameters(diff_tolerance_laser_odom);
  if (!initialize_done) {
    std::cout << "Can't read parameters_input.ymal file, please check and re-calibrate"
              << std::endl;
    return false;
  }
  max_dist_seen_as_continuous_ = max_dist_seen_as_continuous;
  line_length_tolerance_ = line_length_tolerance;
  ransac_fitline_dist_th_ = ransac_fitline_dist_th;
  laser_data_processor_ = std::make_shared<LaserDataProcessor>();
  laser_pose_estimator_ = std::make_shared<LaserPoseEstimator>();
  laser_data_processor_->set_laser_process_parameters(max_dist_seen_as_continuous,
      line_length_tolerance, ransac_fitline_dist_th, ransac_max_iterations,
      min_point_num_stop_ransac, min_proportion_stop_ransac);
  laser_data_processor_->set_line_length(long_edge_length, short_edge_length);
  odom_data_processor_ = std::make_shared<OdomDataProcessor>();
  extrinsic_solver_ = std::make_shared<Slover>();
  extrinsic_solver_->set_tolerance(diff_tolerance_laser_odom);
  return initialize_done;
}

void Calibrator::get_parameters_adjust(double & max_dist_seen_as_continuous,
    double & line_length_tolerance,
    double & ransac_fitline_dist_th)
{
  max_dist_seen_as_continuous = max_dist_seen_as_continuous_;
  line_length_tolerance = line_length_tolerance_;
  ransac_fitline_dist_th = ransac_fitline_dist_th_;
}

void Calibrator::detecte_features()
{
  for (auto i = 0; i < laser_data_set.size(); ++i) {
    laser_data_processor_->process_laser_data(laser_data_set[i]);
  }
}

void Calibrator::update_parameters_detect(const int & id,
    const double & max_dist_seen_as_continuous,
    const double & line_length_tolerance,
    const double & ransac_fitline_dist_th)
{
  laser_data_processor_->update_parameters(
      max_dist_seen_as_continuous, line_length_tolerance, ransac_fitline_dist_th);
  laser_data_processor_->process_laser_data(laser_data_set[id]);
}

bool Calibrator::calibrate()
{
  if (laser_data_set.size() < min_data_size_) {
    std::cout << "There is no enough data input, please re-capture data and re-calibrate \n"
              << "More than 10 sets of data input is recommended" << std::endl;
    return false;
  }
  odom_data_processor_->frame2frame_pose_estimation(odom_data_set);
  laser_pose_estimator_->frame2frame_pose_estimation(laser_data_set);
  bool calib_success = extrinsic_solver_->calibrate(odom_data_set, laser_data_set);
  if (!calib_success) {
    return false;
  }
  Eigen::Matrix2d rotation_l2o;
  Eigen::Vector2d translation_l2o;
  extrinsic_solver_->get_extrinsic(rotation_l2o, translation_l2o);
  parameters_io_->save_extrinsic_parameters(rotation_l2o, translation_l2o);
  return true;
}
}  // namespace laser_odom_calibrator
}  // namespace qrb