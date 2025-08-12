/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "parameters_io_lc/parameters_io.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
ParametersIO::ParametersIO()
{
  input_file_name_ = "parameters_input.yaml";
  output_file_name_ = "extrinsic.yaml";
}
bool ParametersIO::get_topic_names(std::string & laser_topic, std::string & image_topic)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  f_reader_["laser_topic_name"] >> laser_topic;
  f_reader_["image_topic_name"] >> image_topic;
  f_reader_.release();
  return true;
}
bool ParametersIO::get_relative_dist(double & dist)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  dist = f_reader_["reletive_dist_from_laser2chessboard_origin"];
  f_reader_.release();
  return true;
}

bool ParametersIO::get_cam_parameters(qrb::laser_cam_calibrator::CameraInfo & cam_info)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    std::cout << "can't open " + input_file_name_ << std::endl;
    return false;
  }
  cv::Mat intrinsic;
  cv::Mat distortion;
  if (!f_reader_["intrinsic"].isNone()) {
    f_reader_["intrinsic"] >> intrinsic;
  } else {
    std::cout << "'intrinsic' not found" << std::endl;
  }

  if (!f_reader_["distortion"].isNone()) {
    f_reader_["distortion"] >> distortion;
  } else {
    std::cout << "'distortion' not found" << std::endl;
  }

  if (intrinsic.empty() || distortion.empty()) {
    std::cout << "open but can't read" << std::endl;
    f_reader_.release();
    return false;
  }
  cam_info.distortion = distortion.clone();
  cam_info.intrinsic = intrinsic.clone();
  f_reader_.release();
  return true;
}
bool ParametersIO::get_chessboard_parameters(qrb::laser_cam_calibrator::ChessboardInfo & chess_info)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  chess_info.rows = f_reader_["chessboard_rows"];
  chess_info.cols = f_reader_["chessboard_cols"];
  chess_info.square_height = f_reader_["chessboard_square_height"];
  chess_info.square_width = f_reader_["chessboard_square_width"];
  chess_info.left_margin_length = f_reader_["left_margin_length"];
  chess_info.right_margin_length = f_reader_["right_margin_length"];
  chess_info.up_margin_length = f_reader_["up_margin_length"];
  chess_info.down_margin_length = f_reader_["down_margin_length"];
  f_reader_.release();
  return true;
}
bool ParametersIO::get_laser_axis(std::vector<std::string> & laser_xyz_axis)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  std::string laser_x = f_reader_["laser_x_wrt_chessboard"];
  std::string laser_y = f_reader_["laser_y_wrt_chessboard"];
  std::string laser_z = f_reader_["laser_z_wrt_chessboard"];
  if (laser_x.empty() || laser_y.empty() || laser_z.empty()) {
    std::cout << "laser axix empty" << std::endl;
    return false;
  }
  laser_xyz_axis = { laser_x, laser_y, laser_z };
  f_reader_.release();
  return true;
}
void ParametersIO::save_extrinsic_parameters(const Eigen::Matrix3d & rotation_l2c,
    const Eigen::Vector3d & translation_l2c)
{
  cv::FileStorage f_writer_(output_file_name_, cv::FileStorage::WRITE);
  cv::Mat rotation;
  cv::Mat translation;
  if (!f_writer_.isOpened()) {
    std::cout << "Write file failed!" << std::endl;
    return;
  }
  cv::eigen2cv(rotation_l2c, rotation);
  cv::eigen2cv(translation_l2c, translation);
  f_writer_ << "rotation_l2c" << rotation;
  f_writer_ << "translation_l2c" << translation;
  f_writer_.release();
}
bool ParametersIO::get_laser_process_parameters(qrb::laser_cam_calibrator::EnvParameters & env_info)
{
  cv::FileStorage f_reader_(input_file_name_, cv::FileStorage::READ);
  if (!f_reader_.isOpened()) {
    return false;
  }
  env_info.max_dist_seen_as_continuous = f_reader_["max_dist_seen_as_continuous"];
  env_info.ransac_fitline_dist_th = f_reader_["ransac_fitline_dist_th"];
  env_info.ransac_max_iterations = f_reader_["ransac_max_iterations"];
  env_info.min_point_num = f_reader_["min_point_num_stop_ransac"];
  env_info.min_proportion = f_reader_["min_proportion_stop_ransac"];
  env_info.chessboard_length_in_laser_frame = f_reader_["chessboard_length_in_laser_frame"];
  f_reader_.release();
  return true;
}

}  // namespace laser_cam_calibrator
}  // namespace qrb