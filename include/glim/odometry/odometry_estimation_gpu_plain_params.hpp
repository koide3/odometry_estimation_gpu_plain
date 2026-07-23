#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glim {

/**
 * @brief Parameters for OdometryEstimationGPUPlain
 */
struct OdometryEstimationPlainGPUParams {
public:
  OdometryEstimationPlainGPUParams();
  virtual ~OdometryEstimationPlainGPUParams();

public:
  // Sensor params;
  double imu_bias_noise;          // IMU bias noise (over time)
  Eigen::Isometry3d T_lidar_imu;  // LiDAR-IMU transformation

  // Registration params
  double voxel_resolution;         // Base voxel resolution
  int voxelmap_levels;             // Number of voxelmap levels
  double voxelmap_scaling_factor;  // Scaling factor for voxelmap levels

  int max_num_keyframes;            // Maximum number of keyframes
  int full_connection_window_size;  // Full connection window size for matching cost factors

  // Keyframe management params
  double keyframe_min_overlap;  // Minimum overlap ratio for keyframe deletion
  double keyframe_max_overlap;  // Maximum overlap ratio for keyframe insertion
  double keyframe_delta_trans;  // Minimum translation displacement for keyframe insertion
  double keyframe_delta_rot;    // Minimum rotation displacement for keyframe insertion

  // Optimization params
  double smoother_lag;              // Fixed-lag smoother lag time (optimization window size)
  bool use_isam2_dogleg;            // If true, use Dogleg optimizer (slow but stable)
  double isam2_relinearize_skip;    // Number of updates to skip before relinearization
  double isam2_relinearize_thresh;  // Threshold for relinearization

  int num_threads;  // Number of threads for parallel processing
};
}  // namespace glim