/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "main_window/point_cloud_viewer.hpp"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

PointCloudViewer::PointCloudViewer(QWidget * parent) : QMainWindow(parent)
{
  vtk_widget_ = new QVTKOpenGLNativeWidget(this);
  renderer_ = vtkSmartPointer<vtkRenderer>::New();
  vtk_widget_->renderWindow()->AddRenderer(renderer_);
  setCentralWidget(vtk_widget_);
}

void PointCloudViewer::load_cloud(const qrb::laser_cam_calibrator::LaserData & laser_data)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = laser_data.point_cloud.makeShared();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

  for (const auto & p : *cloud) {
    vtkIdType id = points->InsertNextPoint(p.x, p.y, p.z);
    vertices->InsertNextCell(1, &id);
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetVerts(vertices);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(polydata);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(2);
  actor->GetProperty()->SetColor(0, 1, 0);

  renderer_->AddActor(actor);
  renderer_->ResetCamera();
  vtk_widget_->renderWindow()->Render();
}

void PointCloudViewer::draw_line(const pcl::PointXYZ & p1, const pcl::PointXYZ & p2)
{
  vtkSmartPointer<vtkLineSource> line_source = vtkSmartPointer<vtkLineSource>::New();
  line_source->SetPoint1(p1.x, p1.y, p1.z);
  line_source->SetPoint2(p2.x, p2.y, p2.z);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(line_source->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(3);
  actor->GetProperty()->SetColor(1, 0, 0);

  renderer_->AddActor(actor);
  vtk_widget_->renderWindow()->Render();
}

void PointCloudViewer::updata_laser_data(const qrb::laser_cam_calibrator::LaserData & laser_data)
{
  renderer_->RemoveAllViewProps();
  load_cloud(laser_data);
  pcl::PointXYZ pt1;
  pt1.x = laser_data.pts_in_line[0](0);
  pt1.y = laser_data.pts_in_line[0](1);
  pt1.z = 0.0;
  pcl::PointXYZ pt2;
  pt2.x = laser_data.pts_in_line[2](0);
  pt2.y = laser_data.pts_in_line[2](1);
  pt2.z = 0.0;
  if (laser_data.can_be_used != false) {
    draw_line(pt1, pt2);
  }
}
