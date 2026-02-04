cmake_minimum_required(VERSION 3.5)
project(qrb_ros_laser_camera_calibrator VERSION 1.0.1)
#set(CMAKE_BUILD_TYPE debug)
# Default to C99
#if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(i386)|(i686)")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)|(arm)")
  add_custom_target(skip_build ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Skipped building on ${CMAKE_SYSTEM_PROCESSOR}"
)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(i386)|(i686)")
  set(CMAKE_C_STANDARD 99)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
  # find dependencies
  find_package(Qt5 COMPONENTS Widgets Concurrent Svg Core Gui REQUIRED)
  find_package(VTK REQUIRED)
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(nav_2d_msgs REQUIRED)
  find_package(PCL REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge)

  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTOMOC ON)

  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  set(UI_FORMS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_window/*.ui
)
  set(UI_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/main_window)
  file(MAKE_DIRECTORY ${UI_OUTPUT_DIR})
  qt5_wrap_ui(
    UI_HEADERS ${UI_FORMS}
    OUTPUT_DIRECTORY ${UI_OUTPUT_DIR}
  )
  qt5_wrap_cpp(MOC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/main_window/point_cloud_viewer.hpp ${CMAKE_CURRENT_SOURCE_DIR}/include/main_window/adjustparameterswindow.h ${CMAKE_CURRENT_SOURCE_DIR}/include/main_window/mainwindow.h)
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  include_directories(/usr/local/include/)
  include_directories(${VTK_INCLUDE_DIRS})
  include_directories( ${PROJECT_SOURCE_DIR} )
  include_directories(include)

  add_definitions( ${PCL_DEFINITIONS} )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checklibs for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  set(CALIBRATOR_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_window/adjustparameterswindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_window/mainwindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main_window/point_cloud_viewer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_collector/data_collector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  )
  set(TEMPLATE_GENERATOR_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/input_file_generator/generate_input_file.cpp
  )  


  add_executable(qrb_ros_laser_camera_calibrator ${CALIBRATOR_SOURCES} ${MOC_FILES})
  add_executable(qrb_ros_inputfile_template_generator ${TEMPLATE_GENERATOR_SOURCE})



  target_link_libraries(qrb_ros_laser_camera_calibrator ${PCL_LIBRARIES})
  target_link_libraries(qrb_ros_laser_camera_calibrator ${OpenCV_LIBS})
  target_link_libraries(qrb_ros_laser_camera_calibrator Qt5::Widgets Qt5::Core Qt5::Gui )
  target_link_libraries(qrb_ros_laser_camera_calibrator qrb_laser_cam_calibrator_lib)

  ament_target_dependencies(
          qrb_ros_laser_camera_calibrator
          "rclcpp"
          "std_msgs"
          "sensor_msgs"
          "nav_msgs"
          "nav_2d_msgs"
          "OpenCV"
          "PCL"
          "Qt5"
          "cv_bridge"
          "Eigen3"
  )
  ament_target_dependencies(
          qrb_ros_inputfile_template_generator
          "OpenCV"
  )
  install(TARGETS
          qrb_ros_laser_camera_calibrator
          DESTINATION lib/${PROJECT_NAME})
  install(TARGETS
          qrb_ros_inputfile_template_generator
          DESTINATION lib/${PROJECT_NAME})
  ament_package()
endif()
