/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QDir>
#include <QFileDialog>
#include <QObject>
#include <pcl/io/pcd_io.h>
#include "adjustparameterswindow.h"
#include "calibrator/calibrator.hpp"
#include "data_collector/data_collector.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
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
  std::shared_ptr<qrb_ros::laser_odom_collector::DataCollector> data_collector_;
  std::vector<qrb::laser_odom_calibrator::Laser_Data> laser_data_set_;
  std::vector<qrb::laser_odom_calibrator::Odom_Data> odom_data_set_;
  std::shared_ptr<qrb::laser_odom_calibrator::Calibrator> calibrator_ = nullptr;
  std::string input_file_name_ = "parameters_input.yaml";
  void save_data();
  bool load_data();
  QString select_folder_path(QWidget * parent);
  void save_eigen2yaml(const Eigen::Vector2d & odom_pose_xy,
      const Eigen::Matrix2d & odom_pose_yaw_rotation,
      const std::string & filename);
  bool load_eigen_from_yaml(Eigen::Vector2d & odom_pose_xy,
      Eigen::Matrix2d & odom_pose_yaw_rotation,
      const std::string & filename);
};

#endif  // MAINWINDOW_H
