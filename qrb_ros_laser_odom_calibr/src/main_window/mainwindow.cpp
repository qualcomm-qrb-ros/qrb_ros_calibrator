/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "main_window/mainwindow.h"

#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui_(new Ui::MainWindow)
{
  ui_->setupUi(this);
  cv::FileStorage f_reader(input_file_name_, cv::FileStorage::READ);
  if (!f_reader.isOpened()) {
  }
  std::string laser_topic_name = f_reader["laser_topic_name"];
  std::string odom_topic_name = f_reader["odom_topic_name"];

  data_collector_ = std::make_shared<qrb_ros::laser_odom_collector::DataCollector>(
      laser_topic_name, odom_topic_name);
  ui_->calibrateButton->setEnabled(false);
  ui_->saveButton->setEnabled(false);
  ui_->detectButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
  delete ui_;
}

void MainWindow::on_captureButton_clicked()
{
  data_collector_->start_capture();
  while (!data_collector_->current_capture_succeed()) {
    rclcpp::spin_some(data_collector_);
  }
  ui_->textBrowser->append(QString::fromStdString("Capture current sueeced \n"));
  ui_->loadButton->setEnabled(false);
  ui_->detectButton->setEnabled(true);
}

void MainWindow::on_loadButton_clicked()
{
  bool succeed = load_data();
  if (succeed) {
    ui_->textBrowser->append(QString::fromStdString(std::to_string(laser_data_set_.size()) + " sets"
                                                                                             " "
                                                                                             "of "
                                                                                             "data "
                                                                                             "are "
                                                                                             "loade"
                                                                                             "d"));
    ui_->detectButton->setEnabled(true);
    ui_->captureButton->setEnabled(false);
    ui_->saveButton->setEnabled(false);
  } else {
    ui_->textBrowser->append(QString::fromStdString("Load data failed"));
  }
}

void MainWindow::on_detectButton_clicked()
{
  if (ui_->captureButton->isEnabled()) {
    laser_data_set_ = data_collector_->laser_data_set;
    odom_data_set_ = data_collector_->odom_data_set;
  }
  calibrator_ =
      std::make_shared<qrb::laser_odom_calibrator::Calibrator>(laser_data_set_, odom_data_set_);
  adjust_parameters_window_ = new AdjustParametersWindow(nullptr, calibrator_);
  adjust_parameters_window_->show();
  ui_->calibrateButton->setEnabled(true);
}

void MainWindow::on_calibrateButton_clicked()
{
  bool succeed = calibrator_->calibrate();
  adjust_parameters_window_->close();
  delete adjust_parameters_window_;
  if (!succeed) {
    ui_->textBrowser->append(
        QString::fromStdString("Calibration Failed, No enough valid data input."));
  } else {
    ui_->textBrowser->append(QString::fromStdString("Calibration Done"));
    ui_->textBrowser->append(QString::fromStdString("Calibration result saved in extrinsic.yaml"));
  }
}

void MainWindow::on_saveButton_clicked()
{
  save_data();
}
void MainWindow::save_data()
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream folder_ss;
  folder_ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
  std::string folder_name = folder_ss.str();
  if (!std::filesystem::exists(folder_name)) {
    std::filesystem::create_directory(folder_name);
  }
  laser_data_set_ = data_collector_->laser_data_set;
  odom_data_set_ = data_collector_->odom_data_set;
  for (auto i = 0; i < laser_data_set_.size(); ++i) {
    pcl::io::savePCDFileASCII(
        folder_name + "/laser_scan" + std::to_string(i) + ".pcd", laser_data_set_[i].point_cloud);
    save_eigen2yaml(odom_data_set_[i].odom_pose_xy, odom_data_set_[i].odom_pose_yaw_rotation,
        folder_name + "/odom_pose" + std::to_string(i) + ".yaml");
  }
  cv::FileStorage fs(folder_name + "/config.yaml", cv::FileStorage::WRITE);
  fs << "data_num" << int(laser_data_set_.size());
  fs.release();
  ui_->textBrowser->append(QString::fromStdString("Data saved in " + folder_name + "\n"));
}
void MainWindow::save_eigen2yaml(const Eigen::Vector2d & odom_pose_xy,
    const Eigen::Matrix2d & odom_pose_yaw_rotation,
    const std::string & filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "odom_pose_xy" << cv::Mat(cv::Vec2d(odom_pose_xy[0], odom_pose_xy[1]));
  cv::Mat rotation_mat(2, 2, CV_64F);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      rotation_mat.at<double>(i, j) = odom_pose_yaw_rotation(i, j);
    }
  }
  fs << "odom_pose_yaw_rotation" << rotation_mat;
  fs.release();
}
QString MainWindow::select_folder_path(QWidget * parent)
{
  QString defaultPath = QDir::homePath();
  QString folderPath = QFileDialog::getExistingDirectory(parent, QObject::tr("Slect Folder"),
      defaultPath, QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  return folderPath;
}
bool MainWindow::load_eigen_from_yaml(Eigen::Vector2d & odom_pose_xy,
    Eigen::Matrix2d & odom_pose_yaw_rotation,
    const std::string & filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ui_->textBrowser->append("can not open yaml");
    return false;
  }
  cv::Mat xy_mat;
  fs["odom_pose_xy"] >> xy_mat;
  odom_pose_xy << xy_mat.at<double>(0), xy_mat.at<double>(1);
  cv::Mat rotation_mat;
  fs["odom_pose_yaw_rotation"] >> rotation_mat;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      odom_pose_yaw_rotation(i, j) = rotation_mat.at<double>(i, j);
    }
  }
  fs.release();
  return true;
}
bool MainWindow::load_data()
{
  QString file_path = select_folder_path(nullptr);
  if (file_path.isEmpty()) {
    return false;
  }
  std::string folder_path = file_path.toStdString();
  int data_num = 0;
  ui_->textBrowser->append(QString::fromStdString("Load " + folder_path + "/config.yaml"));
  cv::FileStorage fs(folder_path + "/config.yaml", cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }
  fs["data_num"] >> data_num;
  fs.release();
  ui_->textBrowser->append(QString::fromStdString(std::to_string(data_num) + " sets of data will "
                                                                             "be "
                                                                             "loaded"));
  for (int i = 0; i < data_num; ++i) {
    qrb::laser_odom_calibrator::OdomData odom_data;
    qrb::laser_odom_calibrator::LaserData laser_data;
    ui_->textBrowser->append(
        QString::fromStdString("Load " + folder_path + "/laser_scan" + std::to_string(i) + ".pcd"));
    bool succeed1 = load_eigen_from_yaml(odom_data.odom_pose_xy, odom_data.odom_pose_yaw_rotation,
        folder_path + "/odom_pose" + std::to_string(i) + ".yaml");
    int succeed2 = pcl::io::loadPCDFile<pcl::PointXYZ>(
        folder_path + "/laser_scan" + std::to_string(i) + ".pcd", laser_data.point_cloud);
    if (!succeed1) {
      ui_->textBrowser->append(QString::fromStdString(
          "Can't Load " + folder_path + "/odom_pose" + std::to_string(i) + ".yaml"));
      return false;
    }
    if (succeed2 < 0) {
      ui_->textBrowser->append(QString::fromStdString(
          "Can't Load " + folder_path + "/laser_scan" + std::to_string(i) + ".pcd"));
      return false;
    }
    laser_data_set_.push_back(laser_data);
    odom_data_set_.push_back(odom_data);
  }
  return true;
}
