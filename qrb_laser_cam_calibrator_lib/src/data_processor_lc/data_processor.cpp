/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "data_processor_lc/data_processor.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
DataProcessor::DataProcessor()
{
  image_processor_ = nullptr;
  laser_processor_ = nullptr;
}
void DataProcessor::set_cam_info(const CameraInfo & cam_info,
    const ChessboardInfo & chessboard_info)
{
  cam_info_ = cam_info;
  chessboard_info_ = chessboard_info;
  image_processor_ = std::make_shared<ImageProcessor>();
  image_processor_->initialize(cam_info, chessboard_info);
}
void DataProcessor::set_env_paramaters(const EnvParameters & env_prameters)
{
  laser_processor_ = std::make_shared<LaserProcessor>();
  laser_processor_->set_laser_process_parameters(env_prameters.max_dist_seen_as_continuous,
      env_prameters.ransac_fitline_dist_th, env_prameters.ransac_max_iterations,
      env_prameters.min_point_num, env_prameters.min_proportion,
      env_prameters.chessboard_length_in_laser_frame);
}
bool DataProcessor::find_chessboard(const cv::Mat & image)
{
  return image_processor_->find_chessboard(image);
}
void DataProcessor::process_image_data(std::vector<CameraData> & cam_data_set)
{
  for (size_t i = 0; i < cam_data_set.size(); ++i) {
    cv::Mat rot;
    cv::Mat t;
    image_processor_->get_target_pose(cam_data_set[i].image, rot, t);
    cv::cv2eigen(rot, cam_data_set[i].target_orientation);
    cv::cv2eigen(t, cam_data_set[i].target_xyz);
    cam_data_set[i].distance = cam_data_set[i].target_xyz.norm();
    solve_chessboard_plane_parameters(cam_data_set[i]);
    solve_margin_line(cam_data_set[i]);
    dists_.push_back(cam_data_set[i].distance);
  }
}
void DataProcessor::solve_chessboard_plane_parameters(CameraData & cam_data_set)
{
  double nc_x = cam_data_set.target_orientation(0, 2);
  double nc_y = cam_data_set.target_orientation(1, 2);
  double nc_z = cam_data_set.target_orientation(2, 2);
  double x = cam_data_set.target_xyz(0);
  double y = cam_data_set.target_xyz(1);
  double z = cam_data_set.target_xyz(2);
  double d = -nc_x * x - nc_y * y - nc_z * z;
  cam_data_set.chessboard_plane << nc_x, nc_y, nc_z, d;
}
void DataProcessor::process_laser_data(std::vector<LaserData> & laser_data_set)
{
  for (size_t i = 0; i < laser_data_set.size(); ++i) {
    laser_processor_->process_laser_data(laser_data_set[i], dists_[i]);
  }
}
void DataProcessor::update_parameters_detect(const double & max_dist_seen_as_continuous,
    const double & ransac_fitline_dist_th,
    LaserData & laser_data,
    const int & index)
{
  laser_processor_->update_parameters(max_dist_seen_as_continuous, ransac_fitline_dist_th);
  laser_processor_->process_laser_data(laser_data, dists_[index]);
}
void DataProcessor::solve_laser_plane_parameters(LaserPlane & laser_plane, Eigen::Vector4d & plane)
{
  cv::Mat rot;
  cv::Mat t;
  image_processor_->get_target_pose(laser_plane.laser_plane_image, rot, t);
  // A 3D plane can be noted as ax+by+cz+d = 0
  double a = rot.at<double>(0, 0);
  double b = rot.at<double>(1, 0);
  double c = rot.at<double>(2, 0);
  cv::cv2eigen(rot, laser_plane.chessboard_orientation);
  cv::Mat direction = (cv::Mat_<double>(3, 1) << a, b, c);
  t = laser_plane.dist_from_laser2chessboard_origin * direction + t;
  double x = t.at<double>(0, 0);
  double y = t.at<double>(1, 0);
  double z = t.at<double>(2, 0);
  double d = -a * x - b * y - c * z;
  plane << a, b, c, d;
}
void DataProcessor::line_fit(const Eigen::Vector3d & pt1,
    const Eigen::Vector3d & pt2,
    Eigen::Vector3d & line)
{
  // line: y = kx + b
  Eigen::Vector3d tmp = pt2 - pt1;
  double k = tmp(1) / tmp(0);
  double b = pt1(1) - k * pt1(0);
  line << k, -1.0, b;
}
void DataProcessor::solve_margin_line(CameraData & cam_data)
{
  Eigen::Matrix3d rot = cam_data.target_orientation;
  Eigen::Vector3d t = cam_data.target_xyz * 1000;
  double square_height = chessboard_info_.square_height;
  double square_width = chessboard_info_.square_width;
  double chess_row = chessboard_info_.rows;
  double chess_col = chessboard_info_.cols;
  double left_margin_length = chessboard_info_.left_margin_length;
  double up_margin_length = chessboard_info_.up_margin_length;
  double right_margin_length = chessboard_info_.right_margin_length;
  double down_margin_length = chessboard_info_.down_margin_length;
  Eigen::Matrix3d intrinsic;
  cv::cv2eigen(cam_info_.intrinsic, intrinsic);
  Eigen::Vector3d pt_left = { 0., -left_margin_length - square_width, 0. };
  Eigen::Vector3d pt_left_up = { -up_margin_length - square_height,
    -left_margin_length - square_width, 0. };
  Eigen::Vector3d pt_up = { -up_margin_length - square_height, 0., 0. };
  Eigen::Vector3d pt_right = { 0., square_width * chess_col + right_margin_length, 0. };
  Eigen::Vector3d pt_right_up = { -up_margin_length - square_height,
    square_width * chess_col + right_margin_length, 0. };
  Eigen::Vector3d pt_down = { square_height * chess_row + down_margin_length, 0., 0. };
  Eigen::Vector3d pt_left_down = { square_height * chess_row + down_margin_length,
    -left_margin_length - square_width, 0. };
  Eigen::Vector3d pt_right_down = { square_height * chess_row + down_margin_length,
    square_width * chess_col + right_margin_length, 0. };
  pt_left = rot * pt_left + t;
  pt_up = rot * pt_up + t;
  pt_right = rot * pt_right + t;
  pt_down = rot * pt_down + t;

  pt_left_up = rot * pt_left_up + t;
  pt_right_up = rot * pt_right_up + t;
  pt_left_down = rot * pt_left_down + t;
  pt_right_down = rot * pt_right_down + t;

  pt_left = pt_left / pt_left(2);
  pt_up = pt_up / pt_up(2);
  pt_right = pt_right / pt_right(2);
  pt_down = pt_down / pt_down(2);

  pt_left_up = pt_left_up / pt_left_up(2);
  pt_right_up = pt_right_up / pt_right_up(2);
  pt_left_down = pt_left_down / pt_left_down(2);
  pt_right_down = pt_right_down / pt_right_down(2);

  pt_left = intrinsic * pt_left;
  pt_up = intrinsic * pt_up;
  pt_right = intrinsic * pt_right;
  pt_down = intrinsic * pt_down;
  pt_left_up = intrinsic * pt_left_up;
  pt_right_up = intrinsic * pt_right_up;
  pt_left_down = intrinsic * pt_left_down;
  pt_right_down = intrinsic * pt_right_down;

  Eigen::Vector3d left_line_image;
  Eigen::Vector3d right_line_image;
  Eigen::Vector3d up_line_image;
  Eigen::Vector3d down_line_image;

  line_fit(pt_left_up, pt_left_down, left_line_image);
  line_fit(pt_right_up, pt_right_down, right_line_image);
  line_fit(pt_left_up, pt_right_up, up_line_image);
  line_fit(pt_left_down, pt_right_down, down_line_image);
  cam_data.left_margin_line = left_line_image;
  cam_data.right_margin_line = right_line_image;
  cam_data.up_margin_line = up_line_image;
  cam_data.down_margin_line = down_line_image;
}

}  // namespace laser_cam_calibrator
}  // namespace qrb
