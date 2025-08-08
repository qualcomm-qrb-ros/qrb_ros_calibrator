/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "main_window/adjustparameterswindow.h"

#include "ui_adjustparameterswindow.h"

AdjustParametersWindow::AdjustParametersWindow(QWidget * parent,
    std::shared_ptr<qrb::laser_cam_calibrator::Calibrator> calibrator)
  : QMainWindow(parent), calibrator_(calibrator), ui_(new Ui::AdjustParametersWindow)
{
  current_frame_id_ = 0;
  min_size_ = 0;
  ui_->setupUi(this);
  calibrator_->get_parameters_adjust(mid_max_dist_seen_as_continuous_, mid_ransac_fitline_dist_th_);

  QProgressDialog progress(
      "Processing...", "Cancel", 0, calibrator_->laser_data_set.size(), nullptr);
  progress.setWindowTitle("Progress");
  progress.setWindowModality(Qt::WindowModal);
  for (int i = 0; i < calibrator_->laser_data_set.size(); ++i) {
    calibrator_->update_parameters_detect(
        i, mid_max_dist_seen_as_continuous_, mid_ransac_fitline_dist_th_);
    progress.setValue(i);
  }
  ui_->p1Slider->setRange(0, 200);
  ui_->p1Slider->setValue(100);
  ui_->p1_value->setText(QString::fromStdString(std::to_string(mid_max_dist_seen_as_continuous_)));
  ui_->p3Slider->setRange(0, 100);
  ui_->p3Slider->setValue(50);
  ui_->p3_value->setText(QString::fromStdString(std::to_string(mid_ransac_fitline_dist_th_)));
  ui_->lastButton->setEnabled(false);

  max_size_ = calibrator_->laser_data_set.size();
  if (current_frame_id_ + 1 >= max_size_) {
    ui_->nextButton->setEnabled(false);
  }
  viewer_ = new PointCloudViewer();
  viewer_->setWindowTitle("Line detection results");
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
  viewer_->show();
}

AdjustParametersWindow::~AdjustParametersWindow()
{
  delete viewer_;
  delete ui_;
}

void AdjustParametersWindow::on_lastButton_clicked()
{
  current_frame_id_--;
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
  if (current_frame_id_ <= min_size_) {
    ui_->lastButton->setEnabled(false);
  }
  if (!ui_->nextButton->isEnabled()) {
    ui_->nextButton->setEnabled(true);
  }
}

void AdjustParametersWindow::on_nextButton_clicked()
{
  current_frame_id_++;
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
  if (current_frame_id_ >= max_size_ - 1) {
    ui_->nextButton->setEnabled(false);
  }
  if (!ui_->lastButton->isEnabled()) {
    ui_->lastButton->setEnabled(true);
  }
}

void AdjustParametersWindow::on_p1Slider_valueChanged(int value)
{
  max_dist_seen_as_continuous_ = (mid_max_dist_seen_as_continuous_ / 100.0) * double(value);
  ui_->p1_value->setText(QString::fromStdString(std::to_string(max_dist_seen_as_continuous_)));
}

void AdjustParametersWindow::on_p3Slider_valueChanged(int value)
{
  ransac_fitline_dist_th_ = (mid_ransac_fitline_dist_th_ / 50) * double(value);
  ui_->p3_value->setText(QString::fromStdString(std::to_string(ransac_fitline_dist_th_)));
}

void AdjustParametersWindow::on_p1Slider_sliderReleased()
{
  calibrator_->update_parameters_detect(
      current_frame_id_, max_dist_seen_as_continuous_, ransac_fitline_dist_th_);
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
}

void AdjustParametersWindow::on_p3Slider_sliderReleased()
{
  calibrator_->update_parameters_detect(
      current_frame_id_, max_dist_seen_as_continuous_, ransac_fitline_dist_th_);
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
}

void AdjustParametersWindow::on_p1Slider_sliderPressed()
{
  double value = double(ui_->p1Slider->value());
  max_dist_seen_as_continuous_ = (mid_max_dist_seen_as_continuous_ / 100.0) * value;

  calibrator_->update_parameters_detect(
      current_frame_id_, max_dist_seen_as_continuous_, ransac_fitline_dist_th_);
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
}

void AdjustParametersWindow::on_p3Slider_sliderPressed()
{
  double value = double(ui_->p3Slider->value());
  ransac_fitline_dist_th_ = (ransac_fitline_dist_th_ / 50.0) * value;
  calibrator_->update_parameters_detect(
      current_frame_id_, max_dist_seen_as_continuous_, ransac_fitline_dist_th_);
  viewer_->updata_laser_data(calibrator_->laser_data_set[current_frame_id_]);
}
