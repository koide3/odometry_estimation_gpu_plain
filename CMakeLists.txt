cmake_minimum_required(VERSION 3.16)
project(odometry_estimation_gpu_plain VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glim REQUIRED)
find_package(gtsam_points REQUIRED)

### odometry_estimation_gpu_plain ###
ament_auto_add_library(odometry_estimation_gpu_plain SHARED
  src/glim/odometry/odometry_estimation_gpu_plain_params.cpp
  src/glim/odometry/odometry_estimation_gpu_plain.cpp
  src/glim/odometry/odometry_estimation_gpu_plain_create.cpp
)
target_include_directories(odometry_estimation_gpu_plain PUBLIC
  include
)
target_link_libraries(odometry_estimation_gpu_plain
  glim::glim
  gtsam_points::gtsam_points
  Iridescence::Iridescence
)

ament_auto_package()
