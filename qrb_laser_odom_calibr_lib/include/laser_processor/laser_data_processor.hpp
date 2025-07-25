/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef LASER_DATA_PROCESSOR_HPP_
#define LASER_DATA_PROCESSOR_HPP_
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLHeader.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/icp_nl.h>
#include <eigen3/Eigen/Core>
#include "qrb_laser_odom_calibr_lib/calib_data.hpp"
#include <vector>
#include <queue>
#define MIN_DIST_TWO_LINE 0.4

namespace qrb
{
namespace laser_odom_calibrator
{
class LaserDataProcessor
{
private:
  double long_length_;
  double short_length_;
  double max_dist_seen_as_continuous_;
  double line_length_tolerance_;
  double ransac_fitline_dist_th_;
  int ransac_max_iterations_;
  int min_point_num_;
  double min_proportion_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_;
  pcl::ExtractIndices<pcl::PointXYZ> extractor_;
  double dist(const pcl::PointXYZ & a, const pcl::PointXYZ & b);
  /**
   * @desc Excute clustering in a single point cloud
   * @param point_cloud Input point cloud
   * @param clusters Output clusters which record the point index
   * @return void
   */
  void clustering(pcl::PointCloud<pcl::PointXYZ> & point_cloud,
      std::vector<std::vector<int>> & clusters);
  /**
   * @desc Segment all lines in a point cloud
   * @param source_points Input point cloud
   * @param coefficients_set Output line coefficients set
   * @param line_seg_set. Every line segment point set
   * @return void
   */
  void seg_all_lines(pcl::PointCloud<pcl::PointXYZ> & source_points,
      std::vector<pcl::ModelCoefficients> & coefficients_set,
      std::vector<pcl::PointCloud<pcl::PointXYZ>> & line_seg_set);
  /**
   * @desc Get clustered point cloud
   * @param point_cloud Input point cloud
   * @param cluster Input clusters
   * @param all_clustered_sets. Every clusterd set in the form of points
   * @return void
   */
  void clustered_point_clouds(const pcl::PointCloud<pcl::PointXYZ> & point_cloud,
      const std::vector<std::vector<int>> & cluster,
      std::vector<pcl::PointCloud<pcl::PointXYZ>> & all_clustered_sets);
  /**
   * @desc Get the end points in the line segment point set
   * @param line_seg_set Input line segment point sets
   * @param endpt_set Output end points sets
   * @return void
   */
  void search_end_pts(const std::vector<pcl::PointCloud<pcl::PointXYZ>> & line_seg_set,
      std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> & endpt_set);
  /**
   * @desc Find the line index using given length
   * @param endpts Input line segment point sets
   * @param long_edge_index The index of the long edge line
   * @param short_edge_index The index of the short edge line
   * @return void
   */
  void filter_using_length(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> & endpts,
      int & long_edge_index,
      int & short_edge_index);
  /**
   * @desc Calculate the line parameters from pcl line model
   * @param long_line_parameter PCL line model of the long line
   * @param short_line_parameter PCL line model of the short line
   * @param laser_data The defined line data
   * @return void
   */
  void calculate_parameters(const pcl::ModelCoefficients & long_line_parameter,
      const pcl::ModelCoefficients & short_line_parameter,
      Laser_Data & laser_data);
  /**
   * @desc Normalize the line parameters
   * @param long_edge_endpts Two end points of the detected long line
   * @param short_edge_endpts Two end points of the detected short line
   * @param laser_data The defined line data
   * @return void
   */
  void normalize_parameters(const std::pair<pcl::PointXYZ, pcl::PointXYZ> & long_edge_endpts,
      const std::pair<pcl::PointXYZ, pcl::PointXYZ> & short_edge_endpts,
      Laser_Data & laser_data);
  /**
   * @desc Calculate the intersection point of two lines
   * @param line1 Input first line
   * @param line2 Input second line
   * @param intersaction_point Output intersection point
   * @return void
   */
  void two_line_intersaction(const Eigen::Vector3d & line1,
      const Eigen::Vector3d & line2,
      Eigen::Vector2d & intersaction_point);

public:
  /**
   * @desc Set input parameters for laser process
   * @param max_dist_seen_as_continuous Input point cloud
   * @param line_length_tolerance. The tolerance between detected line length and real line length.
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @param ransac_max_iterations. Max iteration number of RANSAC
   * @param min_point_num_stop_ransac. Minimum point number for stop RANSAC iteration
   * @param min_proportion_stop_ransac. Minimum proportion for stop RANSAC iteration
   * @return void
   */
  void set_laser_process_parameters(const double & max_dist_seen_as_continuous,
      const double & line_length_tolerance,
      const double & ransac_fitline_dist_th,
      const int & ransac_max_iterations,
      const int & min_point_num_stop_ransac,
      const double & min_proportion_stop_ransac);
  /**
   * @desc Set the target line length for line detection
   * @param long_length The length of the long edge of the calibration target
   * @param short_length The length of the long edge of the calibration target
   * @return void
   */
  void set_line_length(const double & long_length, const double & short_length);
  /**
   * @desc Update the parameters for line detection
   * @param max_dist_seen_as_continuous Input point cloud
   * @param line_length_tolerance. The tolerance between detected line length and real line length.
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @return void
   */
  void update_parameters(const double & max_dist_seen_as_continuous,
      const double & line_length_tolerance,
      const double & ransac_fitline_dist_th);
  /**
   * @desc Detect lines for the input laser data
   * @param input_data Input laser data
   * @return void
   */
  void process_laser_data(Laser_Data & input_data);
  LaserDataProcessor();
};

}  // namespace laser_odom_calibrator
}  // namespace qrb

#endif