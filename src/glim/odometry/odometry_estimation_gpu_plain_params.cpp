#include <glim/odometry/odometry_estimation_gpu_plain_params.hpp>

#include <glim/util/config.hpp>

namespace glim {

/// @brief Constructor for OdometryEstimationPlainGPUParams
/// @details Just reads parameters from config files
OdometryEstimationPlainGPUParams::OdometryEstimationPlainGPUParams() {
  // sensor config
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
  T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
  imu_bias_noise_acc = sensor_config.param<double>("sensors", "imu_bias_noise_acc", 1e-3);
  imu_bias_noise_gyro = sensor_config.param<double>("sensors", "imu_bias_noise_gyro", 1e-3);

  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  // Registration params
  voxel_resolution = config.param<double>("odometry_estimation", "voxel_resolution", 0.25);
  voxelmap_levels = config.param<int>("odometry_estimation", "voxelmap_levels", 2);
  voxelmap_scaling_factor = config.param<double>("odometry_estimation", "voxelmap_scaling_factor", 2.0);

  max_num_keyframes = config.param<int>("odometry_estimation", "max_num_keyframes", 15);
  full_connection_window_size = config.param<int>("odometry_estimation", "full_connection_window_size", 3);

  // Keyframe management params
  keyframe_min_overlap = config.param<double>("odometry_estimation", "keyframe_min_overlap", 0.3);
  keyframe_max_overlap = config.param<double>("odometry_estimation", "keyframe_max_overlap", 0.7);
  keyframe_delta_trans = config.param<double>("odometry_estimation", "keyframe_delta_trans", 1.0);
  keyframe_delta_rot = config.param<double>("odometry_estimation", "keyframe_delta_rot", 0.25);

  smoother_lag = config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  use_isam2_dogleg = config.param<bool>("odometry_estimation", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1);

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
}

/// @brief Destructor
OdometryEstimationPlainGPUParams::~OdometryEstimationPlainGPUParams() {}

}  // namespace glim