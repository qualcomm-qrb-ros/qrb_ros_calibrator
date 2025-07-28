/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POINTCLOUDVIEWER_HPP
#define POINTCLOUDVIEWER_HPP
#include <QVTKOpenGLNativeWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkLineSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <QMainWindow>

#include "calibrator/calib_data.hpp"
class PointCloudViewer : public QMainWindow
{
  Q_OBJECT
public:
  explicit PointCloudViewer(QWidget * parent = nullptr);
  void load_cloud(const qrb::laser_odom_calibrator::LaserData & laser_data);
  void draw_line(const pcl::PointXYZ & p1, const pcl::PointXYZ & p2);
  void updata_laser_data(const qrb::laser_odom_calibrator::LaserData & laser_data);

private:
  QVTKOpenGLNativeWidget * vtk_widget_;
  vtkSmartPointer<vtkRenderer> renderer_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
};
#endif
