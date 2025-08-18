/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "main_window/mainwindow.h"

#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui_(new Ui::MainWindow)
{
  capture_laser_plane_image_ = true;
  input_file_name_ = "parameters_input.yaml";
  calibrator_ = nullptr;
  ui_->setupUi(this);
  calibrator_ = std::make_shared<qrb::laser_cam_calibrator::Calibrator>();
  std::string image_topic_name;
  std::string laser_topic_name;
  calibrator_->initialize();
  calibrator_->get_topic_name(laser_topic_name, image_topic_name);
  data_collector_ = std::make_shared<qrb_ros::laser_cam_collector::DataCollector>(
      image_topic_name, laser_topic_name);

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
  if (capture_laser_plane_image_) {
    QDialog dialog(this);
    dialog.setWindowTitle(tr("Reminder"));

    QString message =
        tr("Please ensure that:\n"
           "The three axes of the calibration board coordinate system "
           "in the first captured image frame "
           "are parallel to the 2D LiDAR coordinate system axes and "
           "specify the corressponding relationships in parameters_input.yaml");
    QLabel * label = new QLabel(message, &dialog);
    label->setWordWrap(true);
    QVBoxLayout * layout = new QVBoxLayout(&dialog);
    layout->addWidget(label);

    QPushButton * okButton = new QPushButton(tr("OK"), &dialog);
    connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    layout->addWidget(okButton, 0, Qt::AlignRight);

    dialog.exec();
    data_collector_->capture_laser_plane_image();
    while (!data_collector_->laser_plane_image_capture_succeed()) {
      rclcpp::spin_some(data_collector_);
    }
    if (calibrator_->find_chessboard(data_collector_->laser_plane_image.laser_plane_image)) {
      ui_->textBrowser->append("First image for initialization captured!");
      capture_laser_plane_image_ = true;
    } else {
      ui_->textBrowser->append(
          "First image for initialization captured failed! No cornor detected");
      capture_laser_plane_image_ = false;
    }
  } else {
    data_collector_->start_capture();
    while (!data_collector_->current_capture_succeed()) {
      rclcpp::spin_some(data_collector_);
    }
    if (calibrator_->find_chessboard(
            data_collector_->camera_data_set[data_collector_->camera_data_set.size() - 1].image)) {
      ui_->textBrowser->append(
          QString::fromStdString("Capture current laser data and image data sueeced \n"));
      ui_->detectButton->setEnabled(true);
      if (!ui_->saveButton->isEnabled()) {
        ui_->saveButton->setEnabled(true);
      }
      ui_->loadButton->setEnabled(false);
    } else {
      ui_->textBrowser->append(
          QString::fromStdString("No cornor detected, please check the chessboard or the "
                                 "input_prameters.yaml \n"));
      data_collector_->camera_data_set.pop_back();
      data_collector_->laser_data_set.pop_back();
    }
  }
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
    ui_->saveButton->setEnabled(true);
    ui_->loadButton->setEnabled(false);
  } else {
    ui_->textBrowser->append(QString::fromStdString("Load data failed"));
  }
}

void MainWindow::on_detectButton_clicked()
{
  if (ui_->captureButton->isEnabled()) {
    laser_data_set_ = data_collector_->laser_data_set;
    camera_data_set_ = data_collector_->camera_data_set;
    laser_plane_ = data_collector_->laser_plane_image;
  }
  calibrator_->input_data(camera_data_set_, laser_data_set_);
  calibrator_->set_laser_plane(laser_plane_);
  adjust_parameters_window_ = new AdjustParametersWindow(nullptr, calibrator_);
  adjust_parameters_window_->show();
  ui_->calibrateButton->setEnabled(true);
}

void MainWindow::on_calibrateButton_clicked()
{
  bool succeed = calibrator_->calibrate();
  adjust_parameters_window_->close();
  delete adjust_parameters_window_;
  calibrator_->save_result();
  if (!succeed) {
    ui_->textBrowser->append(
        QString::fromStdString("Calibration Failed, No enough valid data input."));
  } else {
    ui_->textBrowser->append(QString::fromStdString("Calibration Done"));
    ui_->textBrowser->append(QString::fromStdString("Calibration result saved in extrinsic.yaml"));
    save_projection_results();
    ui_->textBrowser->append(QString::fromStdString("Projection Results Saved"));
  }
}
void MainWindow::save_projection_results()
{
  std::vector<cv::Mat> projection_results;
  calibrator_->draw_projection_result(projection_results);
  for (auto i = 0; i < projection_results.size(); ++i) {
    cv::imwrite("projection_result" + std::to_string(i) + ".jpg", projection_results[i]);
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
  auto laser_data_set_temp = data_collector_->laser_data_set;
  auto camera_data_set_temp = data_collector_->camera_data_set;
  cv::imwrite("laser_image.jpg", data_collector_->laser_plane_image.laser_plane_image.clone());
  for (auto i = 0; i < laser_data_set_temp.size(); ++i) {
    pcl::io::savePCDFileASCII(folder_name + "/point_cloud" + std::to_string(i) + ".pcd",
        laser_data_set_temp[i].point_cloud);
    cv::imwrite(folder_name + "/image" + std::to_string(i) + ".jpg", camera_data_set_temp[i].image);
  }
  cv::FileStorage fs(folder_name + "/config.xml", cv::FileStorage::WRITE);
  fs << "data_num" << int(laser_data_set_temp.size());
  fs.release();
  ui_->textBrowser->append(QString::fromStdString("Data saved in " + folder_name + "\n"));
}

QString MainWindow::select_folder_path(QWidget * parent)
{
  QString defaultPath = QDir::homePath();
  QString folderPath = QFileDialog::getExistingDirectory(parent, QObject::tr("Slect Folder"),
      defaultPath, QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  return folderPath;
}
bool MainWindow::load_data()
{
  QString file_path = select_folder_path(nullptr);
  if (file_path.isEmpty()) {
    return false;
  }
  std::string folder_path = file_path.toStdString();
  int data_num = 0;
  ui_->textBrowser->append(QString::fromStdString("Load " + folder_path + "/config.xml"));
  cv::FileStorage fs(folder_path + "/config.xml", cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ui_->textBrowser->append("No config.xml file found. Data Load failed!");
    return false;
  }
  laser_plane_image_ = cv::imread(folder_path + "/laser_image" + ".jpg");
  if (laser_plane_image_.empty()) {
    ui_->textBrowser->append(QString::fromStdString(
        "Can't Load " + folder_path + "/laser_image" + ".jpg for initialization"));
    return false;
  }
  laser_plane_.laser_plane_image = laser_plane_image_;
  laser_data_set_.clear();
  camera_data_set_.clear();
  fs["data_num"] >> data_num;
  fs.release();
  ui_->textBrowser->append(QString::fromStdString(std::to_string(data_num) + " sets of data will "
                                                                             "be "
                                                                             "loaded"));
  for (int i = 0; i < data_num; ++i) {
    qrb::laser_cam_calibrator::CameraData camera_data;
    qrb::laser_cam_calibrator::LaserData laser_data;
    ui_->textBrowser->append(QString::fromStdString(
        "Load " + folder_path + "/point_cloud" + std::to_string(i) + ".pcd"));
    cv::Mat image_data = cv::imread(folder_path + "/image" + std::to_string(i) + ".jpg");
    bool load_image_failed = image_data.empty();
    int succeed = pcl::io::loadPCDFile<pcl::PointXYZ>(
        folder_path + "/point_cloud" + std::to_string(i) + ".pcd", laser_data.point_cloud);
    if (load_image_failed) {
      ui_->textBrowser->append(QString::fromStdString(
          "Can't Load " + folder_path + "/image" + std::to_string(i) + ".jpg"));
      return false;
    }
    camera_data.image = image_data.clone();
    if (succeed < 0) {
      ui_->textBrowser->append(QString::fromStdString(
          "Can't Load " + folder_path + "/point_cloud" + std::to_string(i) + ".pcd"));
      return false;
    }
    laser_data.can_be_used = true;
    laser_data_set_.emplace_back(laser_data);
    camera_data_set_.emplace_back(camera_data);
  }
  return true;
}
