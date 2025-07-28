/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "laser_processor/laser_data_processor.hpp"

namespace qrb
{
namespace laser_odom_calibrator
{
LaserDataProcessor::LaserDataProcessor()
{
  long_length_ = 1.2;
  short_length_ = 0.5;
  max_dist_seen_as_continuous_ = 0.07;
  line_length_tolerance_ = 0.25;
  ransac_fitline_dist_th_ = 0.04;
  ransac_max_iterations_ = 10000;
  min_point_num_ = 10;
  min_proportion_ = 0.01;
}

void LaserDataProcessor::set_laser_process_parameters(const double & max_dist_seen_as_continuous,
    const double & line_length_tolerance,
    const double & ransac_fitline_dist_th,
    const int & ransac_max_iterations,
    const int & min_point_num_stop_ransac,
    const double & min_proportion_stop_ransac)
{
  max_dist_seen_as_continuous_ = max_dist_seen_as_continuous;
  line_length_tolerance_ = line_length_tolerance;
  ransac_fitline_dist_th_ = ransac_fitline_dist_th;
  ransac_max_iterations_ = ransac_max_iterations;
  min_point_num_ = min_point_num_stop_ransac;
  min_proportion_ = min_proportion_stop_ransac;
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_LINE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(ransac_max_iterations_);
  seg_.setDistanceThreshold(ransac_fitline_dist_th_);
}

void LaserDataProcessor::set_line_length(const double & long_length, const double & short_length)
{
  long_length_ = long_length;
  short_length_ = short_length;
}

void LaserDataProcessor::update_parameters(const double & max_dist_seen_as_continuous,
    const double & line_length_tolerance,
    const double & ransac_fitline_dist_th)
{
  max_dist_seen_as_continuous_ = max_dist_seen_as_continuous;
  line_length_tolerance_ = line_length_tolerance;
  ransac_fitline_dist_th_ = ransac_fitline_dist_th;
  seg_.setDistanceThreshold(ransac_fitline_dist_th_);
}

void LaserDataProcessor::process_laser_data(LaserData & input_data)
{
  input_data.can_be_used = true;
  std::vector<pcl::ModelCoefficients> coefficients_set;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> line_seg_set;
  std::vector<std::vector<int>> clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> all_clustered_sets;
  clustering(input_data.point_cloud, clusters);
  clustered_point_clouds(input_data.point_cloud, clusters, all_clustered_sets);
  for (size_t i = 0; i < all_clustered_sets.size(); ++i) {
    seg_all_lines(all_clustered_sets[i], coefficients_set, line_seg_set);
  }
  if (line_seg_set.empty()) {
    input_data.can_be_used = false;
    return;
  }
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> endpt_set;
  search_end_pts(line_seg_set, endpt_set);
  int long_edge_index;
  int short_edge_index;
  filter_using_length(endpt_set, long_edge_index, short_edge_index);
  if (long_edge_index == -1 || short_edge_index == -1) {
    input_data.can_be_used = false;
    return;
  }
  calculate_parameters(
      coefficients_set[long_edge_index], coefficients_set[short_edge_index], input_data);
  normalize_parameters(endpt_set[long_edge_index], endpt_set[short_edge_index], input_data);
}

void LaserDataProcessor::seg_all_lines(pcl::PointCloud<pcl::PointXYZ> & source_points,
    std::vector<pcl::ModelCoefficients> & coefficients_set,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> & line_seg_set)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = source_points.makeShared();
  int nr_points = (int)cloud->size();
  while (cloud->size() > nr_points * min_proportion_ && cloud->size() > min_point_num_) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ> cloud_line;
    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      break;
    }
    extractor_.setInputCloud(cloud);
    extractor_.setIndices(inliers);
    extractor_.setNegative(false);
    extractor_.filter(cloud_line);
    line_seg_set.push_back(cloud_line);
    coefficients_set.push_back(*coefficients);
    extractor_.setNegative(true);
    extractor_.filter(*cloud);
  }
}

double LaserDataProcessor::dist(const pcl::PointXYZ & a, const pcl::PointXYZ & b)
{
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

void LaserDataProcessor::search_end_pts(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>> & line_seg_set,
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> & endpt_set)
{
  for (size_t i = 0; i < line_seg_set.size(); ++i) {
    pcl::PointXYZ anchor_point = line_seg_set[i][0];
    double max_dist = 0.0;
    pcl::PointXYZ endpt1;
    for (size_t j = 0; j < line_seg_set[i].size(); ++j) {
      double current_dist = dist(anchor_point, line_seg_set[i][j]);
      if (current_dist > max_dist) {
        max_dist = current_dist;
        endpt1 = line_seg_set[i][j];
      }
    }
    anchor_point = endpt1;
    max_dist = 0.0;
    pcl::PointXYZ endpt2;
    for (size_t j = 0; j < line_seg_set[i].size(); ++j) {
      double current_dist = dist(anchor_point, line_seg_set[i][j]);
      if (current_dist > max_dist) {
        max_dist = current_dist;
        endpt2 = line_seg_set[i][j];
      }
    }
    endpt_set.push_back(std::make_pair(endpt1, endpt2));
  }
}

void LaserDataProcessor::clustering(pcl::PointCloud<pcl::PointXYZ> & point_cloud,
    std::vector<std::vector<int>> & clusters)
{
  // using dfs and kd-tree to clustring the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = point_cloud.makeShared();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  std::vector<bool> processed(cloud->size(), false);
  for (size_t i = 0; i < cloud->size(); ++i) {
    if (processed[i]) {
      continue;
    }
    std::vector<int> cluster;
    std::queue<int> queue;
    queue.push(i);
    while (!queue.empty()) {
      int idx = queue.front();
      queue.pop();
      if (processed[idx]) {
        continue;
      }
      processed[idx] = true;
      cluster.push_back(idx);
      std::vector<int> indices;
      std::vector<float> distances;
      if (kdtree.radiusSearch(
              cloud->points[idx], max_dist_seen_as_continuous_, indices, distances) > 0) {
        for (int id : indices) {
          if (!processed[id]) {
            queue.push(id);
          }
        }
      }
    }
    if (!cluster.empty()) {
      clusters.push_back(cluster);
    }
  }
}

void LaserDataProcessor::clustered_point_clouds(const pcl::PointCloud<pcl::PointXYZ> & point_cloud,
    const std::vector<std::vector<int>> & cluster,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> & all_clustered_sets)
{
  for (size_t i = 0; i < cluster.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ> clustered_set;
    for (size_t j = 0; j < cluster[i].size(); ++j) {
      clustered_set.push_back(point_cloud[cluster[i][j]]);
    }
    all_clustered_sets.push_back(clustered_set);
  }
}

void LaserDataProcessor::filter_using_length(
    const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> & endpts,
    int & long_edge_index,
    int & short_edge_index)
{
  std::vector<int> long_edge_indices;
  std::vector<int> short_edge_indices;
  for (size_t i = 0; i < endpts.size(); ++i) {
    double length = dist(endpts[i].first, endpts[i].second);
    if (fabs(length - short_length_) <= line_length_tolerance_) {
      short_edge_indices.push_back(i);
    } else if (fabs(length - long_length_) <= line_length_tolerance_) {
      long_edge_indices.push_back(i);
    }
  }
  long_edge_index = -1;
  short_edge_index = -1;
  double min_dist = MIN_DIST_TWO_LINE;
  for (size_t i = 0; i < long_edge_indices.size(); ++i) {
    for (size_t j = 0; j < short_edge_indices.size(); ++j) {
      pcl::PointXYZ long_edge_end_pt1 = endpts[long_edge_indices[i]].first;
      pcl::PointXYZ long_edge_end_pt2 = endpts[long_edge_indices[i]].second;
      pcl::PointXYZ short_edge_end_pt1 = endpts[short_edge_indices[j]].first;
      pcl::PointXYZ short_edge_end_pt2 = endpts[short_edge_indices[j]].second;
      double dist1 = dist(long_edge_end_pt1, short_edge_end_pt1);
      double dist2 = dist(long_edge_end_pt1, short_edge_end_pt2);
      double dist3 = dist(long_edge_end_pt2, short_edge_end_pt1);
      double dist4 = dist(long_edge_end_pt2, short_edge_end_pt2);
      std::vector<double> dists = { dist1, dist2, dist3, dist4 };
      double local_min_dist = *std::min_element(dists.begin(), dists.end());
      if (local_min_dist < min_dist) {
        min_dist = local_min_dist;
        long_edge_index = long_edge_indices[i];
        short_edge_index = short_edge_indices[j];
      }
    }
  }
}

void LaserDataProcessor::calculate_parameters(const pcl::ModelCoefficients & long_line_parameter,
    const pcl::ModelCoefficients & short_line_parameter,
    LaserData & laser_data)
{
  Eigen::Vector2d long_line_dir;
  Eigen::Vector2d short_line_dir;
  long_line_dir << long_line_parameter.values[3], long_line_parameter.values[4];
  short_line_dir << short_line_parameter.values[3], short_line_parameter.values[4];
  double b_short = short_line_parameter.values[1] -
                   (short_line_dir[1] / short_line_dir[0]) * short_line_parameter.values[0];
  double b_long = long_line_parameter.values[1] -
                  (long_line_dir[1] / long_line_dir[0]) * long_line_parameter.values[0];
  Eigen::Vector3d long_line;  // ax+by+c = 0
  Eigen::Vector3d short_line;
  long_line << long_line_dir[1], -long_line_dir[0], long_line_dir[0] * b_long;
  short_line << short_line_dir[1], -short_line_dir[0], short_line_dir[0] * b_short;
  Eigen::Vector2d intersection_point;
  two_line_intersaction(long_line, short_line, intersection_point);
  laser_data.long_edge_direction = long_line_dir / long_line_dir.norm();
  laser_data.short_edge_direction = short_line_dir / short_line_dir.norm();
  laser_data.intersaction_point = intersection_point;
}

void LaserDataProcessor::normalize_parameters(
    const std::pair<pcl::PointXYZ, pcl::PointXYZ> & long_edge_endpts,
    const std::pair<pcl::PointXYZ, pcl::PointXYZ> & short_edge_endpts,
    LaserData & laser_data)
{
  pcl::PointXYZ intersaction_point;
  intersaction_point.x = laser_data.intersaction_point[0];
  intersaction_point.y = laser_data.intersaction_point[1];
  intersaction_point.z = 0.0;
  pcl::PointXYZ long_edge_endpt1 = long_edge_endpts.first;
  pcl::PointXYZ long_edge_endpt2 = long_edge_endpts.second;
  pcl::PointXYZ vector1;

  if (dist(long_edge_endpt1, intersaction_point) > dist(long_edge_endpt2, intersaction_point)) {
    vector1.x = long_edge_endpt1.x - intersaction_point.x;
    vector1.y = long_edge_endpt1.y - intersaction_point.y;
    vector1.z = 0.0;
    laser_data.long_end_pt = long_edge_endpt1;
  } else {
    vector1.x = long_edge_endpt2.x - intersaction_point.x;
    vector1.y = long_edge_endpt2.y - intersaction_point.y;
    vector1.z = 0.0;
    laser_data.long_end_pt = long_edge_endpt2;
  }
  if (vector1.x * laser_data.long_edge_direction[0] +
          vector1.y * laser_data.long_edge_direction[1] <
      0) {
    laser_data.long_edge_direction *= -1.0;
  }
  pcl::PointXYZ short_edge_endpt1 = short_edge_endpts.first;
  pcl::PointXYZ short_edge_endpt2 = short_edge_endpts.second;
  pcl::PointXYZ vector2;

  if (dist(short_edge_endpt1, intersaction_point) > dist(short_edge_endpt2, intersaction_point)) {
    vector2.x = short_edge_endpt1.x - intersaction_point.x;
    vector2.y = short_edge_endpt1.y - intersaction_point.y;
    vector2.z = 0.0;
    laser_data.short_end_pt = short_edge_endpt1;
  } else {
    vector2.x = short_edge_endpt2.x - intersaction_point.x;
    vector2.y = short_edge_endpt2.y - intersaction_point.y;
    vector2.z = 0.0;
    laser_data.short_end_pt = short_edge_endpt2;
  }
  if (vector2.x * laser_data.short_edge_direction[0] +
          vector2.y * laser_data.short_edge_direction[1] <
      0) {
    laser_data.short_edge_direction *= -1.0;
  }
}

void LaserDataProcessor::two_line_intersaction(const Eigen::Vector3d & line1,
    const Eigen::Vector3d & line2,
    Eigen::Vector2d & intersaction_point)
{
  Eigen::Matrix2d A;
  Eigen::Vector2d b;
  A << line1[0], line1[1], line2[0], line2[1];
  b << -line1[2], -line2[2];
  intersaction_point = A.inverse() * b;
}

}  // namespace laser_odom_calibrator

}  // namespace qrb