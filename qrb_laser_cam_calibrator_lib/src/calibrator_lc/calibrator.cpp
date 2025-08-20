/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "calibrator_lc/calibrator.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
void Calibrator::initialize()
{
  data_processor_ = nullptr;
  solver_ = nullptr;
  parameters_io_ = nullptr;
  parameters_io_ = std::make_shared<ParametersIO>();
  qrb::laser_cam_calibrator::CameraInfo cam_info;
  qrb::laser_cam_calibrator::ChessboardInfo chessboard_info;
  qrb::laser_cam_calibrator::EnvParameters env_info;
  std::vector<std::string> laser_axis;
  bool succeed = parameters_io_->get_laser_axis(laser_axis);
  if (!succeed) {
    std::cout << "initialize get_laser_axis failed, check the input file" << std::endl;
  }
  succeed = succeed && parameters_io_->get_cam_parameters(cam_info);
  if (!succeed) {
    std::cout << "initialize cam_parameters failed, check the input file" << std::endl;
  }
  succeed = succeed && parameters_io_->get_chessboard_parameters(chessboard_info);
  if (!succeed) {
    std::cout << "initialize chessboard_parameters failed, check the input file" << std::endl;
  }
  succeed = succeed && parameters_io_->get_laser_process_parameters(env_info);
  if (!succeed) {
    std::cout << "initialize env_info failed, check the input file" << std::endl;
  }
  succeed = succeed && parameters_io_->get_relative_dist(relative_dist_);
  if (!succeed) {
    std::cout << "initialize relative_dist_ failed, check the input file" << std::endl;
  }
  if (!succeed) {
    std::cout << "initialize failed, check the input file" << std::endl;
  }
  if (!succeed) {
    throw std::runtime_error("Calibrator initalization failed! Check the input file");
  }
  set_laser_axis(laser_axis);
  cam_info_ = cam_info;
  chessboard_info_ = chessboard_info;
  env_info_ = env_info;
  data_processor_ = std::make_shared<DataProcessor>();
  data_processor_->set_cam_info(cam_info_, chessboard_info_);
  data_processor_->set_env_paramaters(env_info_);
}
void Calibrator::draw_line(cv::Mat & image, const Eigen::Vector3d & image_line)
{
  int width = image.cols;
  int height = image.rows;
  double a = image_line(0);
  double b = image_line(1);
  double c = image_line(2);
  cv::Point pt1, pt2;
  if (b != 0.0) {
    pt1 = cv::Point(0, -c / b);
    pt2 = cv::Point(width, -(a * width + c) / b);
  } else {
    pt1 = cv::Point(-c / a, 0);
    pt2 = cv::Point(-c / a, height);
  }
  cv::line(image, pt1, pt2, cv::Scalar(0, 255, 0), 2);
}
void Calibrator::draw_projection_result(std::vector<cv::Mat> & project_result_img)
{
  Eigen::Matrix3d intrinsic_matrix;
  cv::cv2eigen(cam_info_.intrinsic, intrinsic_matrix);
  t_l2c = t_l2c * 1000;
  for (size_t i = 0; i < cam_data_set.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ> point_cloud = laser_data_set[i].point_cloud;
    std::vector<cv::Point2f> pt_images;
    std::vector<cv::Point2f> pt_key;
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      Eigen::Vector3d pt;
      pt << point_cloud[j].x, point_cloud[j].y, point_cloud[j].z;
      pt = pt * 1000;
      Eigen::Vector3d pt_cam;
      // Transformation from laser frame to camera frame
      pt_cam = R_l2c * pt + t_l2c;
      // Filter out the points that depth < 0
      if (pt_cam(2) < 0) {
        continue;
      }
      pt_cam = pt_cam / pt_cam(2);
      // Calculate the image coordinate
      Eigen::Vector3d pt_img = intrinsic_matrix * pt_cam;
      cv::Point2f pt_image(pt_img(0), pt_img(1));
      pt_images.push_back(pt_image);
    }
    Eigen::Vector3d pt1;
    pt1 << laser_data_set[i].pts_in_line[0](0), laser_data_set[i].pts_in_line[0](1), 0.0;
    pt1 = 1000 * pt1;
    Eigen::Vector3d pt2;
    pt2 << laser_data_set[i].pts_in_line[1](0), laser_data_set[i].pts_in_line[1](1), 0.0;
    pt2 = 1000 * pt2;
    Eigen::Vector3d pt3;
    pt3 << laser_data_set[i].pts_in_line[2](0), laser_data_set[i].pts_in_line[2](1), 0.0;
    pt3 = 1000 * pt3;
    Eigen::Vector3d pt1_cam;
    Eigen::Vector3d pt2_cam;
    Eigen::Vector3d pt3_cam;
    pt1_cam = R_l2c * pt1 + t_l2c;
    pt1_cam = pt1_cam / pt1_cam(2);
    pt2_cam = R_l2c * pt2 + t_l2c;
    pt2_cam = pt2_cam / pt2_cam(2);
    pt3_cam = R_l2c * pt3 + t_l2c;
    pt3_cam = pt3_cam / pt3_cam(2);
    Eigen::Vector3d pt1_img = intrinsic_matrix * pt1_cam;
    cv::Point2f pt1_image(pt1_img(0), pt1_img(1));
    pt_key.push_back(pt1_image);
    Eigen::Vector3d pt2_img = intrinsic_matrix * pt2_cam;
    cv::Point2f pt2_image(pt2_img(0), pt2_img(1));
    pt_key.push_back(pt2_image);
    Eigen::Vector3d pt3_img = intrinsic_matrix * pt3_cam;
    cv::Point2f pt3_image(pt3_img(0), pt3_img(1));
    pt_key.push_back(pt3_image);

    cv::Mat image_temp = cam_data_set[i].image.clone();
    for (size_t j = 0; j < pt_images.size(); ++j) {
      cv::circle(image_temp, pt_images[j], 3, cv::Scalar(0, 0, 255), 2);
    }
    for (size_t j = 0; j < pt_key.size(); ++j) {
      cv::circle(image_temp, pt_key[j], 3, cv::Scalar(0, 255, 0), 2);
    }
    draw_line(image_temp, cam_data_set[i].left_margin_line);
    draw_line(image_temp, cam_data_set[i].right_margin_line);
    draw_line(image_temp, cam_data_set[i].up_margin_line);
    draw_line(image_temp, cam_data_set[i].down_margin_line);
    project_result_img.push_back(image_temp);
  }
}
void Calibrator::get_parameters_adjust(double & max_dist_seen_as_continuous,
    double & ransac_fitline_dist_th)
{
  max_dist_seen_as_continuous = env_info_.max_dist_seen_as_continuous;
  ransac_fitline_dist_th = env_info_.ransac_fitline_dist_th;
}
bool Calibrator::find_chessboard(const cv::Mat & image)
{
  return data_processor_->find_chessboard(image);
}
void Calibrator::set_laser_plane(const LaserPlane & laser_plane)
{
  laser_plane_ = laser_plane;
  laser_plane_.dist_from_laser2chessboard_origin = relative_dist_;
}
void Calibrator::input_data(const std::vector<CameraData> & cam_data_set_in,
    const std::vector<LaserData> & laser_data_set_in)
{
  cam_data_set = cam_data_set_in;
  laser_data_set = laser_data_set_in;
}
void Calibrator::set_laser_axis(const std::vector<std::string> & laser_axis)
{
  laser_axis_ = laser_axis;
}
void Calibrator::update_parameters_detect(const int & index,
    const double & max_dist_seen_as_continuous,
    const double & ransac_fitline_dist_th)
{
  data_processor_->update_parameters_detect(
      max_dist_seen_as_continuous, ransac_fitline_dist_th, laser_data_set[index], index);
}
void Calibrator::get_topic_name(std::string & laser_topic, std::string & image_topic)
{
  parameters_io_->get_topic_names(laser_topic, image_topic);
}
bool Calibrator::calibrate()
{
  Eigen::Vector4d laser_plane;
  data_processor_->solve_laser_plane_parameters(laser_plane_, laser_plane);
  data_processor_->process_image_data(cam_data_set);
  solver_ = std::make_shared<Solver>(laser_data_set, cam_data_set, laser_plane);
  solver_->set_camera_info(cam_info_);
  solver_->set_first_orientation(laser_plane_.chessboard_orientation);
  bool done = solver_->solve_rotation(laser_axis_);
  if (!done) {
    return false;
  }
  done = solver_->calibrate();
  if (!done) {
    return false;
  }
  cam_data_set = solver_->cam_data_set;
  laser_data_set = solver_->laser_data_set;
  R_l2c = solver_->R_l2c;
  t_l2c = solver_->t_l2c;
  return true;
}
void Calibrator::save_result()
{
  parameters_io_->save_extrinsic_parameters(R_l2c, t_l2c);
}

}  // namespace laser_cam_calibrator
}  // namespace qrb