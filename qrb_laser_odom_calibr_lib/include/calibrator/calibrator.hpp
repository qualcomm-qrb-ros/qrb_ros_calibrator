/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef CALIBRATOR_HPP_
#define CALIBRATOR_HPP_
#include "calibrator/calib_data.hpp"
#include "extrinsic_solver/solver.hpp"
#include "laser_processor/laser_data_processor.hpp"
#include "laser_processor/laser_pose_estimator.hpp"
#include "odom_processor/odom_data_processor.hpp"
#include "parameters_io/parameters_io.hpp"
#define MIN_DATA_SIZE 3

namespace qrb
{
namespace laser_odom_calibrator
{
class Calibrator
{
private:
  std::shared_ptr<LaserPoseEstimator> laser_pose_estimator_;
  std::shared_ptr<LaserDataProcessor> laser_data_processor_;
  std::shared_ptr<OdomDataProcessor> odom_data_processor_;
  std::shared_ptr<ParametersIO> parameters_io_;
  std::shared_ptr<Solver> extrinsic_solver_;
  double max_dist_seen_as_continuous_;
  double line_length_tolerance_;
  double ransac_fitline_dist_th_;
  int min_data_size_;
  bool initialize();

public:
  Calibrator(std::vector<LaserData> & laser_data_set_in, std::vector<OdomData> & odom_data_set_in);
  std::vector<LaserData> laser_data_set;
  std::vector<OdomData> odom_data_set;
  /**
   * @Excute calibration
   * @param void
   * @return void
   */
  bool calibrate();
  /**
   * @Detect line features from all laser data
   * @param void
   * @return void
   */
  void detecte_features();
  /**
   * @desc using specific parameters to detect the line in i-th laser data
   * @param id The index of data set
   * @param max_dist_seen_as_continuous The distance that two points can be seen as continuous
   * @param line_length_tolerance. The tolerance between detected line length and real line length.
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @return void
   */
  void update_parameters_detect(const int & id,
      const double & max_dist_seen_as_continuous,
      const double & line_length_tolerance,
      const double & ransac_fitline_dist_th);
  /**
   * @desc Get the user input parameters
   * @param max_dist_seen_as_continuous The distance that two points can be seen as continuous
   * @param line_length_tolerance. The tolerance between detected line length and real line length.
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @return void
   */
  void get_parameters_adjust(double & max_dist_seen_as_continuous,
      double & line_length_tolerance,
      double & ransac_fitline_dist_th);
};
}  // namespace laser_odom_calibrator
}  // namespace qrb
#endif