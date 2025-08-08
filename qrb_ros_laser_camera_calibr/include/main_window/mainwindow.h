/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <pcl/io/pcd_io.h>

#include <QDialog>
#include <QDir>
#include <QFileDialog>
#include <QLabel>
#include <QMainWindow>
#include <QObject>
#include <QVBoxLayout>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "calibrator_lc/calibrator.hpp"
#include "data_collector/data_collector.hpp"
#include "main_window/adjustparameterswindow.h"
namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

private slots:

  void on_captureButton_clicked();

  void on_loadButton_clicked();

  void on_detectButton_clicked();

  void on_calibrateButton_clicked();

  void on_saveButton_clicked();

private:
  Ui::MainWindow * ui_;
  AdjustParametersWindow * adjust_parameters_window_;
  std::shared_ptr<qrb_ros::laser_cam_collector::DataCollector> data_collector_;
  std::vector<qrb::laser_cam_calibrator::LaserData> laser_data_set_;
  std::vector<qrb::laser_cam_calibrator::CameraData> camera_data_set_;
  cv::Mat laser_plane_image_;
  qrb::laser_cam_calibrator::LaserPlane laser_plane_;
  std::shared_ptr<qrb::laser_cam_calibrator::Calibrator> calibrator_;
  std::string input_file_name_;
  bool capture_laser_plane_image_;
  void save_data();
  bool load_data();
  void save_projection_results();
  QString select_folder_path(QWidget * parent);
};

#endif  // MAINWINDOW_H
