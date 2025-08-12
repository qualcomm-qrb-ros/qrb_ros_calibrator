cmake_minimum_required(VERSION 3.5)
project(qrb_laser_cam_calibrator_lib  VERSION 1.0.1)
#set(CMAKE_BUILD_TYPE debug)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PCL REQUIRED )
find_package(Eigen3)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include)
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(${CERES_INCLUDE_DIRS})
include_directories( ${PROJECT_SOURCE_DIR} )

add_definitions( ${PCL_DEFINITIONS} )

set(LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calibrator_lc/calibrator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/extrinsic_solver_lc/solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_processor_lc/laser_processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_processor_lc/image_processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_processor_lc/data_processor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parameters_io_lc/parameters_io.cpp
)


add_library(qrb_laser_cam_calibrator_lib SHARED ${LIB_SOURCES})



target_link_libraries(qrb_laser_cam_calibrator_lib ${PCL_LIBRARIES})
target_link_libraries(qrb_laser_cam_calibrator_lib ${OpenCV_LIBS})
target_link_libraries(qrb_laser_cam_calibrator_lib ${CERES_LIBRARIES})

install(DIRECTORY
        include/
        DESTINATION include)


install(TARGETS
        qrb_laser_cam_calibrator_lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

