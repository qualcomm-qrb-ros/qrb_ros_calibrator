/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "data_processor_lc/laser_processor.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
void LaserProcessor::clustering(pcl::PointCloud<pcl::PointXYZ> & point_cloud,
    std::vector<std::vector<int>> & clusters)
{
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

void LaserProcessor::seg_all_lines(pcl::PointCloud<pcl::PointXYZ> & source_points,
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
      std::cout << "no line detected !" << std::endl;
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

void LaserProcessor::set_laser_process_parameters(const double & max_dist_seen_as_continuous,
    const double & ransac_fitline_dist_th,
    const int & ransac_max_iterations,
    const int & min_point_num_stop_ransac,
    const double & min_proportion_stop_ransac,
    const double & chessboard_length_in_laser_frame)
{
  max_dist_seen_as_continuous_ = max_dist_seen_as_continuous;
  ransac_fitline_dist_th_ = ransac_fitline_dist_th;
  ransac_max_iterations_ = ransac_max_iterations;
  min_point_num_ = min_point_num_stop_ransac;
  min_proportion_ = min_proportion_stop_ransac;
  chessboard_length_in_laser_frame_ = chessboard_length_in_laser_frame;
  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_LINE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(ransac_max_iterations_);
  seg_.setDistanceThreshold(ransac_fitline_dist_th_);
}

void LaserProcessor::search_end_pts(const pcl::PointCloud<pcl::PointXYZ> & line_seg,
    std::pair<pcl::PointXYZ, pcl::PointXYZ> & endpts)
{
  if (line_seg.empty()) {
    return;
  }
  pcl::PointXYZ anchor_point = line_seg[0];
  double max_dist = 0.0;
  pcl::PointXYZ endpt1;
  for (size_t j = 0; j < line_seg.size(); ++j) {
    double current_dist = dist(anchor_point, line_seg[j]);
    if (current_dist > max_dist) {
      max_dist = current_dist;
      endpt1 = line_seg[j];
    }
  }
  anchor_point = endpt1;
  max_dist = 0.0;
  pcl::PointXYZ endpt2;
  for (size_t j = 0; j < line_seg.size(); ++j) {
    double current_dist = dist(anchor_point, line_seg[j]);
    if (current_dist > max_dist) {
      max_dist = current_dist;
      endpt2 = line_seg[j];
    }
  }
  endpts = std::make_pair(endpt1, endpt2);
}
void LaserProcessor::clustered_point_clouds(const pcl::PointCloud<pcl::PointXYZ> & point_cloud,
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
void LaserProcessor::update_parameters(const double & max_dist_seen_as_continuous,
    const double & ransac_fitline_dist_th)
{
  max_dist_seen_as_continuous_ = max_dist_seen_as_continuous;
  ransac_fitline_dist_th_ = ransac_fitline_dist_th;
  seg_.setDistanceThreshold(ransac_fitline_dist_th_);
}

void LaserProcessor::process_laser_data(LaserData & laser_data, double & distance)
{
  std::vector<pcl::ModelCoefficients> coefficients_set;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> line_seg_set;
  std::vector<std::vector<int>> clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> all_clustered_sets;
  clustering(laser_data.point_cloud, clusters);
  clustered_point_clouds(laser_data.point_cloud, clusters, all_clustered_sets);
  for (size_t i = 0; i < all_clustered_sets.size(); ++i) {
    seg_all_lines(all_clustered_sets[i], coefficients_set, line_seg_set);
  }
  if (line_seg_set.empty()) {
    laser_data.can_be_used = false;
    std::cout << "No line detected, discard this set of data" << std::endl;
    return;
  }

  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> endpt_set;
  for (size_t i = 0; i < line_seg_set.size(); ++i) {
    std::pair<pcl::PointXYZ, pcl::PointXYZ> endpts_temp;
    search_end_pts(line_seg_set[i], endpts_temp);
    endpt_set.push_back(endpts_temp);
  }
  int id = -1;
  filter_using_length(endpt_set, id);
  if (id == -1) {
    laser_data.can_be_used = false;
    std::cout << "No target line detected, discard this set of data" << std::endl;
    return;
  }
  laser_data.selected_line_seg = line_seg_set[id];
  std::pair<pcl::PointXYZ, pcl::PointXYZ> endpts;
  endpts = endpt_set[id];
  calculate_parameter(coefficients_set[id], laser_data);
  Eigen::Vector2d pt1;
  Eigen::Vector2d pt2;
  Eigen::Vector2d pt3;
  pt1(0) = endpts.first.x;
  pt1(1) = endpts.first.y;
  pt3(0) = endpts.second.x;
  pt3(1) = endpts.second.y;
  pt2 = 0.5 * (pt1 + pt3);
  laser_data.pts_in_line.clear();
  laser_data.pts_in_line.push_back(pt1);
  laser_data.pts_in_line.push_back(pt2);
  laser_data.pts_in_line.push_back(pt3);
  std::cout << "process done" << std::endl;
}
void LaserProcessor::filter_using_length(
    const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> & end_pt_set,
    int & id)
{
  id = -1;
  double min_diff = 100.0;
  for (size_t i = 0; i < end_pt_set.size(); ++i) {
    double line_length = dist(end_pt_set[i].first, end_pt_set[i].second);
    double diff = fabs(line_length - chessboard_length_in_laser_frame_);
    pcl::PointXYZ origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    if (diff < min_diff && dist(end_pt_set[i].first, origin) < 4.0 &&
        dist(end_pt_set[i].second, origin) < 4.0) {
      id = i;
      min_diff = diff;
    }
  }
}
void LaserProcessor::calculate_parameter(const pcl::ModelCoefficients & fitted_line_parameter,
    LaserData & laser_data)
{
  Eigen::Vector2d line_dir;
  line_dir << fitted_line_parameter.values[3], fitted_line_parameter.values[4];
  laser_data.line_dir = line_dir;
  double b = fitted_line_parameter.values[1] -
             (line_dir[1] / line_dir[0]) * fitted_line_parameter.values[0];
  Eigen::Vector3d line;
  line << line_dir[1], -line_dir[0], line_dir[0] * b;
  laser_data.line_parameter = line;
}

double LaserProcessor::dist(const pcl::PointXYZ & a, const pcl::PointXYZ & b)
{
  return sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

double LaserProcessor::mean_dist(const pcl::PointCloud<pcl::PointXYZ> & line_seg)
{
  double size = static_cast<double>(line_seg.size());
  double mean = 0.0;
  pcl::PointXYZ ori;
  ori.x = 0.0;
  ori.y = 0.0;
  ori.z = 0.0;
  for (size_t i = 0; i < line_seg.size(); ++i) {
    mean += dist(ori, line_seg[i]) / size;
  }
  return mean;
}
}  // namespace laser_cam_calibrator
}  // namespace qrb