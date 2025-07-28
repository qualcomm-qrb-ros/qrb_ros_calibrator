cmake_minimum_required(VERSION 3.5)
project(qrb_laser_odom_calibr_lib VERSION 1.0.1)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(PCL REQUIRED )
find_package(Eigen3)
find_package(OpenCV REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
add_definitions( ${PCL_DEFINITIONS} )
set(LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calibrator/calibrator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/extrinsic_solver/solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/laser_processor/laser_data_processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/laser_processor/laser_pose_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odom_processor/odom_data_processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parameters_io/parameters_io.cpp
)
add_library(qrb_laser_odom_calibr_lib SHARED ${LIB_SOURCES})
target_link_libraries(qrb_laser_odom_calibr_lib ${PCL_LIBRARIES})
target_link_libraries(qrb_laser_odom_calibr_lib ${OpenCV_LIBS})
install(TARGETS
        qrb_laser_odom_calibr_lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY
        include/
        DESTINATION include)
