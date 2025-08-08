/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <opencv2/opencv.hpp>
int main(int argc, char * argv[])
{
  std::string input_file_name = "parameters_input.yaml";
  cv::FileStorage f_writer(input_file_name, cv::FileStorage::WRITE);
  double max_dist_seen_as_continuous = 0.5;
  double ransac_fitline_dist_th = 0.02;
  int32_t ransac_max_iterations = 10000;
  int32_t min_point_num_stop_ransac = 10;
  double min_proportion_stop_ransac = 0.01;
  cv::Mat intrinsic = (cv::Mat_<float>(3, 3) << 644.076904296875, 0., 641.3648681640625, 0.,
      643.2040405273438, 359.3841857910156, 0., 0., 1.);
  cv::Mat distortion = (cv::Mat_<float>(1, 4) << -0.05502111092209816, 0.06547785550355911,
      0.0002284314832650125, 0.0006337873637676239);
  int32_t chessboard_rows = 8;
  int32_t chessboard_cols = 11;
  double chessboard_square_height = 45.0;
  double chessboard_square_width = 45.0;
  double reletive_dist_from_laser2chessboard_origin = 0.193;
  double chessboard_length_in_laser_frame = 0.60;
  double left_margin_length = 29.0;
  double right_margin_length = 30.0;
  double up_margin_length = 22.0;
  double down_margin_length = 22.0;
  std::string laser_x_wrt_chessboard = "z";
  std::string laser_y_wrt_chessboard = "y";
  std::string laser_z_wrt_chessboard = "x-";
  std::string image_topic_name = "/camera/color/image_raw";
  std::string laser_topic_name = "/scan";
  f_writer << "image_topic_name" << image_topic_name;
  f_writer << "laser_topic_name" << laser_topic_name;
  f_writer << "reletive_dist_from_laser2chessboard_origin"
           << reletive_dist_from_laser2chessboard_origin;
  f_writer << "chessboard_length_in_laser_frame" << chessboard_length_in_laser_frame;
  f_writer << "laser_x_wrt_chessboard" << laser_x_wrt_chessboard;
  f_writer << "laser_y_wrt_chessboard" << laser_y_wrt_chessboard;
  f_writer << "laser_z_wrt_chessboard" << laser_z_wrt_chessboard;
  f_writer << "intrinsic" << intrinsic;
  f_writer << "distortion" << distortion;
  f_writer << "chessboard_rows" << chessboard_rows;
  f_writer << "chessboard_cols" << chessboard_cols;
  f_writer << "chessboard_square_height" << chessboard_square_height;
  f_writer << "chessboard_square_width" << chessboard_square_width;
  f_writer << "left_margin_length" << left_margin_length;
  f_writer << "right_margin_length" << right_margin_length;
  f_writer << "up_margin_length" << up_margin_length;
  f_writer << "down_margin_length" << down_margin_length;
  f_writer << "max_dist_seen_as_continuous" << max_dist_seen_as_continuous;
  f_writer << "ransac_max_iterations" << ransac_max_iterations;
  f_writer << "ransac_fitline_dist_th" << ransac_fitline_dist_th;
  f_writer << "min_point_num_stop_ransac" << min_point_num_stop_ransac;
  f_writer << "min_proportion_stop_ransac" << min_proportion_stop_ransac;
  f_writer.release();
  std::cout << "parameters_input.yaml generated!" << std::endl;
  return 0;
}
