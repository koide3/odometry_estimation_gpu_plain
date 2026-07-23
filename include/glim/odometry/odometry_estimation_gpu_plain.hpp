#pragma once

#include <map>
#include <memory>
#include <random>

#include <spdlog/spdlog.h>

#include <glim/odometry/odometry_estimation_base.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/util/indexed_sliding_window.hpp>

#include <glim/util/config.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/imu_validation.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/odometry/initial_state_estimation.hpp>
#include <glim/odometry/loose_initial_state_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

#include <glim/odometry/odometry_estimation_gpu_plain_params.hpp>

namespace glim {

/**
 * @brief Odometry estimation GPU with a plain implementation
 */
class OdometryEstimationGPUPlain : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationGPUPlain(const OdometryEstimationPlainGPUParams& params);
  virtual ~OdometryEstimationGPUPlain() override;

  /// @brief IMU data callback
  /// @param stamp        Timestamp
  /// @param linear_acc   Linear acceleration
  /// @param angular_vel  Angular velocity
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;

  /// @brief Point cloud data callback. This is where the main odometry estimation process happens.
  /// @param frame                       Preprocessed point cloud
  /// @param marginalized_frames         [out] Marginalized estimation frames
  /// @return EstimationFrame::ConstPtr  Estimation result for the latest frame
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

  /// @brief Image data callback (optional)
  /// @param stamp   Timestamp
  /// @param image   Image
  virtual void insert_image(const double stamp, const cv::Mat& image) override;

  /// @brief Pop out the remaining non-marginalized frames (called at the end of the sequence)
  /// @return std::vector<EstimationFrame::ConstPtr>  Remaining non-marginalized frames
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

protected:
  /// @brief Convert the point cloud of the given frame to GPU-based point cloud and create multi-resolution voxel maps
  /// @param frame  [in/out] Estimation frame
  void create_frame(EstimationFrame::Ptr& frame);

  /// @brief Create matching cost factors for the latest frame
  /// @param current      Index of the latest frame
  /// @return gtsam::NonlinearFactorGraph  New factors for the latest frame
  gtsam::NonlinearFactorGraph create_matching_cost_factors(const int current);

  /**
   * @brief Keyframe management based on an overlap metric
   * @ref   Koide et al., "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA2022
   */
  void update_keyframes_overlap(int current);

  /**
   * @brief Keyframe management based on displacement criteria
   * @ref   Engel et al., "Direct Sparse Odometry", IEEE Trans. PAMI, 2018
   */
  void update_keyframes_displacement(int current);

  /// @brief Update the information of estimation frames in the optimization window after the smoother update
  /// @param current      Index of the latest frame
  /// @param new_factors  New factors for the latest frame
  void update_frames(const int current);

protected:
  const OdometryEstimationPlainGPUParams params;

  // *** Sensor extrinsic params ***
  // T_lidar_imu brings a point in the IMU frame to the LiDAR frame. (pt_lidar = T_lidar_imu * pt_imu)
  // T_imu_lidar is the inverse of T_lidar_imu. (pt_imu = T_imu_lidar * pt_lidar)
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // *** Frames & keyframes ***
  // frames[0] ~ frames[marginalized_cursor] are marginalized frames (released and became nullptr)
  // frames[marginalized_cursor] ~ frames[frames.size()-1] are active frames in the optimization window
  int marginalized_cursor;                                          ///< Index of the last marginalized frame in the sliding window
  gtsam_points::IndexedSlidingWindow<EstimationFrame::Ptr> frames;  //< All frames (marginalized + active), marginalized ones are released and became nullptr
  std::vector<EstimationFrame::ConstPtr> keyframes;                 //< Keyframes. Some are active and some are marginalized.

  // *** Utility classes ***
  std::unique_ptr<InitialStateEstimation> init_estimation;           ///< Initial state estimation. It will be released once the initial estimation is done.
  std::unique_ptr<IMUIntegration> imu_integration;                   ///< IMU integration
  std::unique_ptr<IMUValidation> imu_validation;                     ///< IMU relative pose validation
  std::unique_ptr<CloudDeskewing> deskewing;                         ///< Point cloud deskewing
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;  ///< Point covariance estimation

  // *** Optimizer ***
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  // CUDA-related
  std::unique_ptr<gtsam_points::CUDAStream> stream;
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> stream_buffer_roundrobin;
};

}  // namespace glim
