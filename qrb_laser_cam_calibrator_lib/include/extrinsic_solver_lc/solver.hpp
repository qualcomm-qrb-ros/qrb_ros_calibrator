/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef SOLVER_HPP_
#define SOLVER_HPP_
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <eigen3/Eigen/Core>

#include "data_processor_lc/data_processor.hpp"
#define MIN_PIX_DIFF 30.0
#define PROPORTION 0.85
#define MIN_CONDITION 3
namespace qrb
{
namespace laser_cam_calibrator
{
class Solver
{
private:
  CameraInfo camera_info_;
  Eigen::Vector4d laser_plane_;
  Eigen::Matrix3d orientation_;
  bool solve_closed_form();
  int parse_laser_axis(std::string axis);
  bool find_nearest_line(const Eigen::Vector3d & pt,
      const CameraData & cam_data,
      Eigen::Vector3d & line);
  void find_correspondce(PointLinePair & constraint,
      const CameraData & cam_data,
      const LaserData & laser_data);
  void nonlinear_optimization();
  double min_diff_th_pix_;
  double proportion_;

public:
  std::vector<LaserData> laser_data_set;
  std::vector<CameraData> cam_data_set;
  Eigen::Matrix3d R_l2c;
  Eigen::Vector3d t_l2c;
  Solver(std::vector<LaserData> & laser_data_set_in,
      std::vector<CameraData> & cam_data_set_in,
      Eigen::Vector4d & laser_plane_in);
  /**
   * @desc Execute the calibtration
   * @return bool calibration succeed or not
   */
  bool calibrate();
  /**
   * @desc Set the camera parameters
   * @param camera_info Input camera parameters
   * @return void
   */
  void set_camera_info(const CameraInfo & camera_info);
  /**
   * @desc Execute the calibtration
   * @param odom_datas Input odometry data set
   * @param laser_datas. Input laser data set
   * @return void
   */
  bool solve_rotation(const std::vector<std::string> & laser_axis);
  /**
   * @desc Execute the calibtration
   * @param odom_datas Input odometry data set
   * @param laser_datas. Input laser data set
   * @return void
   */
  void set_first_orientation(const Eigen::Matrix3d & orientation);
};
struct PointLineDist
{
public:
  PointLineDist(double * laser_pt, double * image_line, double * intrinsic)
    : laser_pt_(laser_pt), image_line_(image_line), intrinsic_(intrinsic)
  {
  }

  template <typename T>
  bool operator()(const T * const angle_axis, const T * const translation, T * residuals) const
  {
    T pt_laser[3] = { T(laser_pt_[0]), T(laser_pt_[1]), T(laser_pt_[2]) };
    T pt_cam[3];
    T a = T(image_line_[0]);
    T b = T(image_line_[1]);
    T c = T(image_line_[2]);
    ceres::AngleAxisRotatePoint(angle_axis, pt_laser, pt_cam);
    pt_cam[0] += translation[0] * 1000.0;
    pt_cam[1] += translation[1] * 1000.0;
    pt_cam[2] += translation[2] * 1000.0;
    pt_cam[0] = pt_cam[0] / pt_cam[2];
    pt_cam[1] = pt_cam[1] / pt_cam[2];
    pt_cam[2] = pt_cam[2] / pt_cam[2];
    T fx = T(intrinsic_[0]);
    T fy = T(intrinsic_[4]);
    T u0 = T(intrinsic_[2]);
    T v0 = T(intrinsic_[5]);
    T pt_image[2];
    pt_image[0] = fx * pt_cam[0] + u0;
    pt_image[1] = fy * pt_cam[1] + v0;
    residuals[0] = (pt_image[0] * a + pt_image[1] * b + c) / sqrt(a * a + b * b);
    return true;
  }
  static ceres::CostFunction * create(double * laser_pt, double * image_line, double * intrinsic)
  {
    return (new ceres::AutoDiffCostFunction<PointLineDist, 1, 3, 3>(
        new PointLineDist(laser_pt, image_line, intrinsic)));
  }
  double * laser_pt_;
  double * image_line_;
  double * intrinsic_;
};
struct PointPlaneDist
{
public:
  PointPlaneDist(double * laser_pt, double * plane) : laser_pt_(laser_pt), plane_(plane) {}

  template <typename T>
  bool operator()(const T * const angle_axis, const T * const translation, T * residuals) const
  {
    T pt_laser[3] = { T(laser_pt_[0]), T(laser_pt_[1]), T(laser_pt_[2]) };
    T pt_cam[3];
    T a = T(plane_[0]);
    T b = T(plane_[1]);
    T c = T(plane_[2]);
    T d = T(plane_[3]);
    ceres::AngleAxisRotatePoint(angle_axis, pt_laser, pt_cam);
    pt_cam[0] += translation[0];
    pt_cam[1] += translation[1];
    pt_cam[2] += translation[2];

    residuals[0] =
        (pt_cam[0] * a + pt_cam[1] * b + pt_cam[2] * c + d) / sqrt(a * a + b * b + c * c);
    return true;
  }
  static ceres::CostFunction * create(double * laser_pt, double * plane)
  {
    return (new ceres::AutoDiffCostFunction<PointPlaneDist, 1, 3, 3>(
        new PointPlaneDist(laser_pt, plane)));
  }
  double * laser_pt_;
  double * plane_;
};

}  // namespace laser_cam_calibrator
}  // namespace qrb
#endif
