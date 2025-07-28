/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "extrinsic_solver/solver.hpp"
namespace qrb
{
namespace laser_odom_calibrator
{
Solver::Solver()
{
  diff_tolerance_laser_odom_ = 0.05;
  min_observe_num_ = MIN_OBSERVE_NUM;
}

Solver::~Solver() {}
void Solver::set_tolerance(const double & diff_tolerance_laser_odom)
{
  diff_tolerance_laser_odom_ = diff_tolerance_laser_odom;
}

bool Solver::solve_linear_equation(std::vector<OdomData> & odom_datas,
    std::vector<LaserData> & laser_datas)
{
  int observe_num = 0;
  for (size_t i = 0; i < laser_datas.size(); ++i) {
    if (laser_datas[i].can_be_used == false) {
      odom_datas[i].can_be_used = false;
    }
    if (std::isnan(laser_datas[i].last2current_xy.norm())) {
      laser_datas[i].can_be_used = false;
      odom_datas[i].can_be_used = false;
    }
  }
  for (size_t i = 1; i < odom_datas.size(); ++i) {
    if (odom_datas[i].can_be_used == false || odom_datas[i - 1].can_be_used == false) {
      continue;
    }
    double laser_odom_diff =
        fabs(odom_datas[i].last2current_xy.norm() - laser_datas[i].last2current_xy.norm());
    if (laser_odom_diff > diff_tolerance_laser_odom_) {
      continue;
    } else {
      observe_num++;
      odom_datas[i].can_be_used = true;
      laser_datas[i].can_be_used = true;
    }
  }
  if (observe_num < min_observe_num_) {
    return false;
  }
  Eigen::MatrixXd A(observe_num * 2, 4);
  Eigen::MatrixXd b(observe_num * 2, 1);
  int current_observe = 0;
  for (size_t i = 1; i < odom_datas.size(); ++i) {
    double laser_odom_diff =
        fabs(odom_datas[i].last2current_xy.norm() - laser_datas[i].last2current_xy.norm());
    if ((!odom_datas[i].can_be_used) || (!laser_datas[i].can_be_used)) {
      continue;
    }

    if (laser_odom_diff > diff_tolerance_laser_odom_) {
      continue;
    }
    Eigen::Matrix<double, 2, 4> A_temp;
    Eigen::Matrix<double, 2, 1> b_temp;
    Eigen::Matrix2d laser_rotation = laser_datas[i].last2current_rotation;
    Eigen::Vector2d laser_last2current_xy = laser_datas[i].last2current_xy;
    Eigen::Vector2d odom_last2current_xy = odom_datas[i].last2current_xy;
    A_temp << laser_rotation(0, 0) - 1.0, laser_rotation(0, 1), -laser_last2current_xy(0),
        laser_last2current_xy(1), laser_rotation(1, 0), laser_rotation(1, 1) - 1.0,
        -laser_last2current_xy(1), -laser_last2current_xy(0);
    b_temp << -odom_last2current_xy(0), -odom_last2current_xy(1);
    A.block<2, 4>(2 * current_observe, 0) = A_temp;
    b.block<2, 1>(2 * current_observe, 0) = b_temp;
    current_observe++;
  }
  Eigen::Vector4d x = (A.transpose() * A).ldlt().solve(A.transpose() * b);
  extrinsic_xy_ << x(0), x(1);
  double yaw_extrinsic = atan2(x(3), x(2));
  extrinsic_yaw_angle_ = yaw_extrinsic;
  extrinsic_yaw_ << cos(yaw_extrinsic), -sin(yaw_extrinsic), sin(yaw_extrinsic), cos(yaw_extrinsic);
  return true;
}

bool Solver::calibrate(std::vector<OdomData> & odom_datas, std::vector<LaserData> & laser_datas)
{
  bool calib = solve_linear_equation(odom_datas, laser_datas);
  if (calib == false) {
    return calib;
  } else {
    print_extrinsic_ladar2odom();
    return calib;
  }
}

void Solver::print_extrinsic_ladar2odom()
{
  std::cout << "extrinsic laser to odom is :" << std::endl;
  std::cout << "extrinsic_xy: " << "\n" << extrinsic_xy_ << std::endl;
  std::cout << "extrinsic yaw: " << extrinsic_yaw_angle_ / 3.14 * 180 << std::endl;
}

void Solver::get_extrinsic(Eigen::Matrix2d & rotation, Eigen::Vector2d & translation)
{
  rotation = extrinsic_yaw_;
  translation = extrinsic_xy_;
}

void Solver::rotation2quaternion(const Eigen::Matrix2d & rotation, double * quaternion)
{
  Eigen::Matrix3d rot3d;
  rot3d << rotation(0, 0), rotation(0, 1), 0.0, rotation(1, 0), rotation(1, 1), 0.0, 0.0, 0, 1.0;
  Eigen::Quaterniond q(rot3d);
  quaternion[0] = q.w();
  quaternion[1] = q.x();
  quaternion[2] = q.y();
  quaternion[3] = q.z();
}

void Solver::quaternion2rotation(const double * quaternion, Eigen::Matrix2d & rotation)
{
  Eigen::Quaterniond q;
  q.w() = quaternion[0];
  q.x() = quaternion[1];
  q.y() = quaternion[2];
  q.z() = quaternion[3];
  Eigen::Matrix3d rotation3d;
  rotation3d = q.toRotationMatrix();
  rotation = rotation3d.block<2, 2>(0, 0);
}
}  // namespace laser_odom_calibrator
}  // namespace qrb