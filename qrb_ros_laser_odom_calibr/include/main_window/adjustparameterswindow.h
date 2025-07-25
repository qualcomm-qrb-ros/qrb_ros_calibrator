/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef ADJUSTPARAMETERSWINDOW_H
#define ADJUSTPARAMETERSWINDOW_H
#include <QMainWindow>
#include <QString>
#include <QWindow>
#include <QProgressDialog>
#include "calibrator/calibrator.hpp"
#include "main_window/point_cloud_viewer.hpp"
namespace Ui
{
class AdjustParametersWindow;
}

class AdjustParametersWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit AdjustParametersWindow(QWidget * parent = nullptr,
      std::shared_ptr<qrb::laser_odom_calibrator::Calibrator> calibrator = nullptr);
  ~AdjustParametersWindow();

private slots:
  void on_lastButton_clicked();
  void on_nextButton_clicked();
  void on_p1Slider_valueChanged(int value);
  void on_p2Slider_valueChanged(int value);
  void on_p3Slider_valueChanged(int value);
  void on_p1Slider_sliderReleased();
  void on_p2Slider_sliderReleased();
  void on_p3Slider_sliderReleased();
  void on_p1Slider_sliderPressed();
  void on_p2Slider_sliderPressed();
  void on_p3Slider_sliderPressed();

signals:
  void cloud_update_requested(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

private:
  Ui::AdjustParametersWindow * ui_;
  double mid_max_dist_seen_as_continuous_;
  PointCloudViewer * viewer_;
  double mid_line_length_tolerance_;
  double mid_ransac_fitline_dist_th_;
  double max_dist_seen_as_continuous_;
  double line_length_tolerance_;
  double ransac_fitline_dist_th_;
  int current_frame_id_;
  int max_size_;
  int min_size_;
  std::shared_ptr<qrb::laser_odom_calibrator::Calibrator> calibrator_ = nullptr;
};

#endif  // ADJUSTPARAMETERSWINDOW_H
