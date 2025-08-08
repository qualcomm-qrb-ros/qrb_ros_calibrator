/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef LASER_PROCESSOR_HPP_
#define LASER_PROCESSOR_HPP_
#include <pcl/PCLHeader.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <eigen3/Eigen/Core>

#include "calibrator_lc/calib_data.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
class LaserProcessor
{
private:
  double max_dist_seen_as_continuous_;
  double ransac_fitline_dist_th_;
  int ransac_max_iterations_;
  int min_point_num_;
  double min_proportion_;
  double chessboard_length_in_laser_frame_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_;
  pcl::ExtractIndices<pcl::PointXYZ> extractor_;
  /**
   * @desc Excute clustering in a single point cloud
   * @param point_cloud Input point cloud
   * @param clusters Output clusters which record the point index
   * @return void
   */
  void clustering(pcl::PointCloud<pcl::PointXYZ> & point_cloud,
      std::vector<std::vector<int>> & cluster);
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
   * @desc Find the line index using given length
   * @param endpts Input line segment point sets
   * @param long_edge_index The index of the long edge line
   * @param short_edge_index The index of the short edge line
   * @return void
   */
  void filter_using_length(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> & end_pt_set,
      int & id);
  /**
   * @desc Calculate the distance between two points
   * @param a Input first point
   * @param b Input second point
   * @return double The calculated distance
   */
  double dist(const pcl::PointXYZ & a, const pcl::PointXYZ & b);
  /**
   * @desc Calculate the mean distance of a point cloud
   * @param line_seg Input point cloud
   * @return double The calculated mean distance
   */
  double mean_dist(const pcl::PointCloud<pcl::PointXYZ> & line_seg);
  /**
   * @desc Get the end points in the line segment point set
   * @param line_seg_set Input line segment point sets
   * @param endpt_set Output end points sets
   * @return void
   */
  void search_end_pts(const pcl::PointCloud<pcl::PointXYZ> & line_seg,
      std::pair<pcl::PointXYZ, pcl::PointXYZ> & endpts);
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
   * @desc Calculate the line parameters from pcl line model
   * @param long_line_parameter PCL line model of the long line
   * @param short_line_parameter PCL line model of the short line
   * @param laser_data The defined line data
   * @return void
   */
  void calculate_parameter(const pcl::ModelCoefficients & fitted_line_parameter,
      LaserData & laser_data);

public:
  /**
   * @desc Set input parameters for laser process
   * @param max_dist_seen_as_continuous Input point cloud
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @param ransac_max_iterations. Max iteration number of RANSAC
   * @param min_point_num_stop_ransac. Minimum point number for stop RANSAC iteration
   * @param min_proportion_stop_ransac. Minimum proportion for stop RANSAC iteration
   * @param chessboard_length_in_laser_frame. The possible length that the chessboard can be scaned
   * by laser
   * @return void
   */
  void set_laser_process_parameters(const double & max_dist_seen_as_continuous,
      const double & ransac_fitline_dist_th,
      const int & ransac_max_iterations,
      const int & min_point_num_stop_ransac,
      const double & min_proportion_stop_ransac,
      const double & chessboard_length_in_laser_frame);
  /**
   * @desc Update the parameters for line detection
   * @param max_dist_seen_as_continuous Input point cloud
   * @param ransac_fitline_dist_th. The max distance that can be taken as inner point of RANSAC when
   * fitting line
   * @return void
   */
  void update_parameters(const double & max_dist_seen_as_continuous,
      const double & ransac_fitline_dist_th);
  /**
   * @desc Detect lines for the input laser data
   * @param input_data Input laser data
   * @param distance Distance from laser to chessboard
   * @return void
   */
  void process_laser_data(LaserData & laser_data, double & distance);
};
}  // namespace laser_cam_calibrator
}  // namespace qrb

#endif