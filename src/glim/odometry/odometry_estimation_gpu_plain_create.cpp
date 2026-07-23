#include <glim/odometry/odometry_estimation_gpu_plain.hpp>

// @brief Loading the odometry estimation module as a shared library
extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationPlainGPUParams params;
  return new glim::OdometryEstimationGPUPlain(params);
}