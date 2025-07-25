/*
 * Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "data_collector/data_collector.hpp"
#include "main_window/mainwindow.h"
#include <iostream>
#include <QApplication>
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication a(argc, argv);
  MainWindow main_window;
  main_window.show();
  return a.exec();
}
