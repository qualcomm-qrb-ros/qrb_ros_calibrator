/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "extrinsic_solver_lc/solver.hpp"
namespace qrb
{
namespace laser_cam_calibrator
{
Solver::Solver(std::vector<LaserData> & laser_data_set_in,
    std::vector<CameraData> & cam_data_set_in,
    Eigen::Vector4d & laser_plane_in)
  : laser_data_set(laser_data_set_in), cam_data_set(cam_data_set_in), laser_plane_(laser_plane_in)
{
  min_diff_th_pix_ = MIN_PIX_DIFF;
  proportion_ = PROPORTION;
}
void Solver::set_first_orientation(const Eigen::Matrix3d & orientation)
{
  orientation_ = orientation;
}
int Solver::parse_laser_axis(std::string axis)
{
  int ret;
  if (axis.size() == 1) {
    ret = axis[0] - 'w';
    return ret;
  } else {
    ret = axis[0] - 'w';
    return -ret;
  }
}
bool Solver::solve_rotation(const std::vector<std::string> & laser_axis)
{
  Eigen::Vector3d r1;
  Eigen::Vector3d r2;
  Eigen::Vector3d r3;
  int idx1 = parse_laser_axis(laser_axis[0]);
  int idx2 = parse_laser_axis(laser_axis[1]);
  int idx3 = parse_laser_axis(laser_axis[2]);
  if (idx1 < -3 || idx1 > 3) {
    std::cout << "invalid laser axis input \n" << std::endl;
    return false;
  }
  if (idx2 < -3 || idx2 > 3) {
    std::cout << "invalid laser axis input \n" << std::endl;
    return false;
  }
  if (idx3 < -3 || idx3 > 3) {
    std::cout << "invalid laser axis input \n" << std::endl;
    return false;
  }
  if (idx1 > 0) {
    r1 = orientation_.col(idx1 - 1);
  } else {
    r1 = -orientation_.col(abs(idx1) - 1);
  }
  if (idx2 > 0) {
    r2 = orientation_.col(idx2 - 1);
  } else {
    r2 = -orientation_.col(abs(idx2) - 1);
  }
  if (idx3 > 0) {
    r3 = orientation_.col(idx3 - 1);
  } else {
    r3 = -orientation_.col(abs(idx3) - 1);
  }
  R_l2c.block<3, 1>(0, 0) = r1;
  R_l2c.block<3, 1>(0, 1) = r2;
  R_l2c.block<3, 1>(0, 2) = r3;
  return true;
}
void Solver::set_camera_info(const CameraInfo & camera_info)
{
  camera_info_ = camera_info;
}
void Solver::find_correspondce(PointLinePair & constraint,
    const CameraData & cam_data,
    const LaserData & laser_data)
{
  Eigen::Matrix3d intrinsic;
  cv::cv2eigen(camera_info_.intrinsic, intrinsic);
  Eigen::Vector3d laser_pt1;
  laser_pt1 << laser_data.pts_in_line[0](0), laser_data.pts_in_line[0](1), 0.0;
  laser_pt1 = laser_pt1 * 1000;
  Eigen::Vector3d laser_pt2;
  laser_pt2 << laser_data.pts_in_line[2](0), laser_data.pts_in_line[2](1), 0.0;
  laser_pt2 = laser_pt2 * 1000;
  laser_pt1 = R_l2c * laser_pt1 + t_l2c * 1000;
  laser_pt2 = R_l2c * laser_pt2 + t_l2c * 1000;
  laser_pt1 = laser_pt1 / laser_pt1(2);
  laser_pt2 = laser_pt2 / laser_pt2(2);
  laser_pt1 = intrinsic * laser_pt1;
  laser_pt2 = intrinsic * laser_pt2;
  Eigen::Vector3d image_line1;
  Eigen::Vector3d image_line2;
  if (find_nearest_line(laser_pt1, cam_data, image_line1)) {
    double laser_pt[3];
    double image_line[3];
    constraint.laser_pt1[0] = laser_data.pts_in_line[0](0) * 1000;
    constraint.laser_pt1[1] = laser_data.pts_in_line[0](1) * 1000;
    constraint.laser_pt1[2] = 0.0;
    constraint.image_line1[0] = image_line1(0);
    constraint.image_line1[1] = image_line1(1);
    constraint.image_line1[2] = image_line1(2);
  } else {
    constraint.laser_pt1[0] = 0.0;
  }
  if (find_nearest_line(laser_pt2, cam_data, image_line2)) {
    double laser_pt[3];
    double image_line[3];
    constraint.laser_pt2[0] = laser_data.pts_in_line[2](0) * 1000;
    constraint.laser_pt2[1] = laser_data.pts_in_line[2](1) * 1000;
    constraint.laser_pt2[2] = 0.0;
    constraint.image_line2[0] = image_line2(0);
    constraint.image_line2[1] = image_line2(1);
    constraint.image_line2[2] = image_line2(2);
  } else {
    constraint.laser_pt2[0] = 0.0;
  }
}
void Solver::nonlinear_optimization()
{
  ceres::Problem problem;
  Eigen::Matrix3d intrinsic;
  std::cout << "start nonlinear optimization" << std::endl;
  cv::cv2eigen(camera_info_.intrinsic, intrinsic);
  double intrinsic_m[]{ intrinsic(0, 0), intrinsic(0, 1), intrinsic(0, 2), intrinsic(1, 0),
    intrinsic(1, 1), intrinsic(1, 2), intrinsic(2, 0), intrinsic(2, 1), intrinsic(2, 2) };
  Eigen::AngleAxisd angle_axisd(R_l2c);

  double angle_axis[]{ angle_axisd.angle() * angle_axisd.axis()(0),
    angle_axisd.angle() * angle_axisd.axis()(1), angle_axisd.angle() * angle_axisd.axis()(2) };
  double translation[]{ t_l2c(0), t_l2c(1), t_l2c(2) };
  double laser_plane[]{ laser_plane_(0), laser_plane_(1), laser_plane_(2), laser_plane_(3) };
  std::cout << "nonlinear_optimizator initialize done" << std::endl;
  std::vector<PointLinePair> constraints;
  std::vector<PointPlanePair> inplane_constraints;
  for (size_t i = 0; i < laser_data_set.size(); ++i) {
    if (!laser_data_set[i].can_be_used) {
      continue;
    }
    PointLinePair constraint;
    find_correspondce(constraint, cam_data_set[i], laser_data_set[i]);
    PointPlanePair inplane_constraint;
    inplane_constraint.laser_pt1[0] = constraint.laser_pt1[0] / 1000;
    inplane_constraint.laser_pt1[1] = constraint.laser_pt1[1] / 1000;
    inplane_constraint.laser_pt1[2] = constraint.laser_pt1[2] / 1000;
    inplane_constraint.laser_pt2[0] = constraint.laser_pt2[0] / 1000;
    inplane_constraint.laser_pt2[1] = constraint.laser_pt2[1] / 1000;
    inplane_constraint.laser_pt2[2] = constraint.laser_pt2[2] / 1000;
    inplane_constraint.plane[0] = cam_data_set[i].chessboard_plane(0);
    inplane_constraint.plane[1] = cam_data_set[i].chessboard_plane(1);
    inplane_constraint.plane[2] = cam_data_set[i].chessboard_plane(2);
    inplane_constraint.plane[3] = cam_data_set[i].chessboard_plane(3);
    inplane_constraints.push_back(inplane_constraint);
    constraints.push_back(constraint);
  }
  for (size_t i = 0; i < laser_data_set.size(); ++i) {
    if (constraints[i].laser_pt1[0] != 0.0) {
      auto cost_function1 =
          PointPlaneDist::create(inplane_constraints[i].laser_pt1, inplane_constraints[i].plane);
      problem.AddResidualBlock(cost_function1, NULL, angle_axis, translation);
      auto cost_function2 = PointPlaneDist::create(inplane_constraints[i].laser_pt1, laser_plane);
      problem.AddResidualBlock(cost_function2, NULL, angle_axis, translation);
    }
    if (constraints[i].laser_pt2[0] != 0.0) {
      auto cost_function1 =
          PointPlaneDist::create(inplane_constraints[i].laser_pt2, inplane_constraints[i].plane);
      problem.AddResidualBlock(cost_function1, NULL, angle_axis, translation);
      auto cost_function2 = PointPlaneDist::create(inplane_constraints[i].laser_pt2, laser_plane);
      problem.AddResidualBlock(cost_function2, NULL, angle_axis, translation);
    }
  }
  ceres::Solver::Options option;
  option.linear_solver_type = ceres::DENSE_QR;
  option.max_num_iterations = 500;
  ceres::Solver::Summary summary;
  ceres::Solve(option, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  double angle = sqrtf64(angle_axis[0] * angle_axis[0] + angle_axis[1] * angle_axis[1] +
                         angle_axis[2] * angle_axis[2]);
  Eigen::Vector3d axis(angle_axis[0] / angle, angle_axis[1] / angle, angle_axis[2] / angle);
  Eigen::AngleAxisd new_angle_axis(angle, axis);
  R_l2c = new_angle_axis.toRotationMatrix();
  t_l2c << translation[0], translation[1], translation[2];
  std::cout << "Extrinsic parameters after otpimization " << std::endl;
  std::cout << R_l2c << std::endl;
  std::cout << t_l2c << std::endl;
}
bool Solver::find_nearest_line(const Eigen::Vector3d & pt,
    const CameraData & cam_data,
    Eigen::Vector3d & line)
{
  double a = cam_data.left_margin_line(0);
  double b = cam_data.left_margin_line(1);
  double c = cam_data.left_margin_line(2);
  double dist = fabs((a * pt(0) + b * pt(1) + c) / sqrtf64(a * a + b * b));
  double min_diff = dist;
  line = cam_data.left_margin_line;

  a = cam_data.right_margin_line(0);
  b = cam_data.right_margin_line(1);
  c = cam_data.right_margin_line(2);
  dist = fabs((a * pt(0) + b * pt(1) + c) / sqrtf64(a * a + b * b));
  double second_min_diff;
  if (dist < min_diff) {
    second_min_diff = min_diff;
    min_diff = dist;
    line = cam_data.right_margin_line;
  } else {
    second_min_diff = dist;
  }

  a = cam_data.up_margin_line(0);
  b = cam_data.up_margin_line(1);
  c = cam_data.up_margin_line(2);
  dist = fabs((a * pt(0) + b * pt(1) + c) / sqrtf64(a * a + b * b));
  if (dist < min_diff) {
    second_min_diff = min_diff;
    min_diff = dist;
    line = cam_data.up_margin_line;
  }
  a = cam_data.down_margin_line(0);
  b = cam_data.down_margin_line(1);
  c = cam_data.down_margin_line(2);
  dist = fabs((a * pt(0) + b * pt(1) + c) / sqrtf64(a * a + b * b));
  if (dist < min_diff) {
    second_min_diff = min_diff;
    min_diff = dist;
    line = cam_data.down_margin_line;
  }
  if (min_diff > min_diff_th_pix_) {
    return false;
  }
  if (min_diff < proportion_ * second_min_diff) {
    return true;
  } else {
    return false;
  }
}

bool Solver::solve_closed_form()
{
  int observe_num = 0;
  for (size_t i = 0; i < laser_data_set.size(); ++i) {
    if (laser_data_set[i].can_be_used) {
      observe_num++;
    }
  }
  if (observe_num < MIN_CONDITION) {
    std::cout << "no enough data given" << std::endl;
    return false;
  }
  Eigen::MatrixXd A(observe_num, 3);
  Eigen::MatrixXd b(observe_num, 1);
  int current_observe = 0;
  for (size_t i = 0; i < laser_data_set.size(); ++i) {
    if (!laser_data_set[i].can_be_used) {
      continue;
    }
    Eigen::Matrix<double, 1, 3> A_temp;
    Eigen::Matrix<double, 1, 1> b_temp;
    std::cout << "chessboard_plane " << cam_data_set[i].chessboard_plane << std::endl;
    double n1 = cam_data_set[i].chessboard_plane(0);
    double n2 = cam_data_set[i].chessboard_plane(1);
    double n3 = cam_data_set[i].chessboard_plane(2);
    double d = cam_data_set[i].chessboard_plane(3);

    Eigen::Vector3d pt2_tmp;
    pt2_tmp << laser_data_set[i].pts_in_line[1](0), laser_data_set[i].pts_in_line[1](1), 0.0;

    pt2_tmp = R_l2c * pt2_tmp;
    double x2 = pt2_tmp(0);
    double y2 = pt2_tmp(1);
    double z2 = pt2_tmp(2);
    A_temp << n1, n2, n3;
    b_temp << -d - n1 * x2 - n2 * y2 - n3 * z2;
    A.block<1, 3>(1 * i, 0) = A_temp;
    b.block<1, 1>(1 * i, 0) = b_temp;
    current_observe++;
  }

  Eigen::VectorXd x(3, 1);
  x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
  t_l2c = x;
  std::cout << "-----------------closed form solution --------------------" << std::endl;
  std::cout << "R_l2c: \n" << R_l2c << std::endl;
  std::cout << "t_l2c: \n" << t_l2c << std::endl;
  return true;
}
bool Solver::calibrate()
{
  if (!solve_closed_form()) {
    return false;
  }
  nonlinear_optimization();
  return true;
}
}  // namespace laser_cam_calibrator
}  // namespace qrb