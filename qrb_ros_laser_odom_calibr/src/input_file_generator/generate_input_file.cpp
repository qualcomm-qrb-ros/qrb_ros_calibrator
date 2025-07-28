/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <opencv2/opencv.hpp>
int main(int argc, char * argv[])
{
  std::string input_file_name = "parameters_input.yaml";
  cv::FileStorage f_writer(input_file_name, cv::FileStorage::WRITE);
  double long_edge_length = 2.0;
  double short_edge_length = 1.0;
  double max_dist_seen_as_continuous = 0.07;
  double line_length_tolerance = 0.25;
  double ransac_fitline_dist_th = 0.04;
  int ransac_max_iterations = 10000;
  int min_point_num_stop_ransac = 10;
  double min_proportion_stop_ransac = 0.01;
  double diff_tolerance_laser_odom = 0.05;
  std::string laser_topic_name = "/scan";
  std::string odom_topic_name = "/odom";
  f_writer << "laser_topic_name" << laser_topic_name;
  f_writer << "odom_topic_name" << odom_topic_name;
  f_writer << "long_edge_length" << long_edge_length;
  f_writer << "short_edge_length" << short_edge_length;
  f_writer << "max_dist_seen_as_continuous" << max_dist_seen_as_continuous;
  f_writer << "line_length_tolerance" << line_length_tolerance;
  f_writer << "ransac_max_iterations" << ransac_max_iterations;
  f_writer << "ransac_fitline_dist_th" << ransac_fitline_dist_th;
  f_writer << "min_point_num_stop_ransac" << min_point_num_stop_ransac;
  f_writer << "min_proportion_stop_ransac" << min_proportion_stop_ransac;
  f_writer << "diff_tolerance_laser_odom" << diff_tolerance_laser_odom;
  f_writer.release();
  std::cout << "parameters_input.yaml generated!" << std::endl;
  return 0;
}
