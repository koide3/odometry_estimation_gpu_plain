#include <glim/odometry/odometry_estimation_gpu_plain.hpp>

#include <glim/util/convert_to_string.hpp>

namespace glim {

// Callbacks are used to publish the internal states of the odometry estimation module to other modules in GLIM.
// The published information are fed to visualization and extension modules.
using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

/// @brief Constructor for OdometryEstimationGPUPlain
/// @param params  Parameters for OdometryEstimationGPUPlain
OdometryEstimationGPUPlain::OdometryEstimationGPUPlain(const OdometryEstimationPlainGPUParams& params) : params(params) {
  logger->info("Hello OdometryEstimationGPUPlain!!");

  marginalized_cursor = 0;
  T_lidar_imu = params.T_lidar_imu;
  T_imu_lidar = T_lidar_imu.inverse();

  // initialize utility classes
  init_estimation = std::make_unique<RobustInitialStateEstimation>(params.T_lidar_imu);
  imu_integration.reset(new IMUIntegration);
  imu_validation.reset(new IMUValidation(logger, true));
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation(params.num_threads));

  // initialize the optimizer
  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.findUnusedFactorSlots = true;
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);
  smoother.reset(new FixedLagSmootherExt(params.smoother_lag, isam2_params));

  // initialize CUDA-related objects
  stream.reset(new gtsam_points::CUDAStream());
  stream_buffer_roundrobin.reset(new gtsam_points::StreamTempBufferRoundRobin());
}

/// @brief Destructor for OdometryEstimationGPUPlain
OdometryEstimationGPUPlain::~OdometryEstimationGPUPlain() {
  frames.clear();
  keyframes.clear();
  smoother.reset();
}

/// @brief IMU data callback
/// @param stamp        Timestamp
/// @param linear_acc   Linear acceleration
/// @param angular_vel  Angular velocity
void OdometryEstimationGPUPlain::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

/// @brief Point cloud data callback. This is where the main odometry estimation process happens.
/// @param frame                       Preprocessed point cloud
/// @param marginalized_frames         [out] Marginalized estimation frames
/// @return EstimationFrame::ConstPtr  Estimation result for the latest frame
EstimationFrame::ConstPtr OdometryEstimationGPUPlain::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  if (raw_frame->size()) {
    logger->trace("insert_frame points={} times={} ~ {}", raw_frame->size(), raw_frame->times.front(), raw_frame->times.back());
  } else {
    logger->warn("insert_frame points={}", raw_frame->size());
  }
  Callbacks::on_insert_frame(raw_frame);

  const int current = frames.size();  // Index of the current frame
  const int last = current - 1;       // Index of the last frame

  // Handling the very first frame
  if (frames.empty()) {
    // Perform initial state (pose & velocity & IMU bias) estimation and see if it is ready
    EstimationFrame::ConstPtr init_state;
    init_estimation->insert_frame(raw_frame);
    init_state = init_estimation->initial_pose();

    // If there are not enough observations to conduct initial state estimation, wait for more data to arrive
    if (init_state == nullptr) {
      logger->debug("waiting for initial IMU state estimation to be finished");
      return nullptr;
    }

    // Initial estimation is done, release the initial state estimation module
    init_estimation.reset();

    logger->info("initial IMU state estimation result");
    logger->info("T_world_imu={}", convert_to_string(init_state->T_world_imu));
    logger->info("v_world_imu={}", convert_to_string(init_state->v_world_imu));
    logger->info("imu_bias={}", convert_to_string(init_state->imu_bias));

    // Initialize the first estimation frame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    T_lidar_imu = init_state->T_lidar_imu;
    T_imu_lidar = T_lidar_imu.inverse();

    new_frame->T_lidar_imu = init_state->T_lidar_imu;
    new_frame->T_world_lidar = init_state->T_world_lidar;
    new_frame->T_world_imu = init_state->T_world_imu;

    new_frame->v_world_imu = init_state->v_world_imu;
    new_frame->imu_bias = init_state->imu_bias;
    new_frame->raw_frame = raw_frame;

    // Transform LiDAR points into the IMU frame
    std::vector<Eigen::Vector4d> points_imu(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points_imu[i] = T_imu_lidar * raw_frame->points[i];
    }

    // Estimate point normals and covariances
    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    covariance_estimation->estimate(points_imu, raw_frame->neighbors, normals, covs);

    // Create a point cloud frame
    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points_imu);
    if (raw_frame->intensities.size()) {
      frame->add_intensities(raw_frame->intensities);
    }
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::IMU;

    // Convert the point cloud to GPU-based one and create multi-level voxel maps
    create_frame(new_frame);

    // Publish the new frame and add it to the frame list
    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);

    // Initialize the estimator
    gtsam::Values new_values;                           // New values to be added to the optimizer
    gtsam::NonlinearFactorGraph new_factors;            // New factors to be added to the optimizer
    gtsam::FixedLagSmootherKeyTimestampMap new_stamps;  // Timestamps for the new values

    new_stamps[X(0)] = raw_frame->stamp;
    new_stamps[V(0)] = raw_frame->stamp;
    new_stamps[B(0)] = raw_frame->stamp;

    new_values.insert(X(0), gtsam::Pose3(new_frame->T_world_imu.matrix()));
    new_values.insert(V(0), new_frame->v_world_imu);
    new_values.insert(B(0), gtsam::imuBias::ConstantBias(new_frame->imu_bias));

    // Prior for initial IMU states
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, 1e6);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(B(0), 6, 1e6);
    new_factors.add(create_matching_cost_factors(current));

    // Add new values, factors, and timestamps to the optimizer and update internal states
    smoother->update(new_factors, new_values, new_stamps);
    update_frames(current);

    return frames.back();
  }

  /*** Process of non-first frames ***/

  // New values, factors, and timestamps to be added to the optimizer
  gtsam::Values new_values;                           // New values to be added to the optimizer
  gtsam::NonlinearFactorGraph new_factors;            // New factors to be added to the optimizer
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;  // Timestamps for the new values

  // The last frame's timestamp and states
  const double last_stamp = frames[last]->stamp;
  const auto last_T_world_imu_ = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_T_world_imu = gtsam::Pose3(last_T_world_imu_.rotation().normalized(), last_T_world_imu_.translation());
  const auto last_v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(last));
  const auto last_imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last));
  const gtsam::NavState last_nav_world_imu(last_T_world_imu, last_v_world_imu);

  // IMU integration from the last frame to the current frame for state prediction
  int num_imu_integrated = 0;
  const int imu_read_cursor = imu_integration->integrate_imu(last_stamp, raw_frame->stamp, last_imu_bias, &num_imu_integrated);
  imu_integration->erase_imu_data(imu_read_cursor);
  logger->trace("num_imu_integrated={}", num_imu_integrated);

  if (num_imu_integrated <= 2) {
    logger->warn("insufficient number of IMU data between LiDAR scans!! (odometry_estimation)");
    logger->warn("t_last={:.6f} t_current={:.6f} num_imu={}", last_stamp, raw_frame->stamp, num_imu_integrated);
    new_factors.add(gtsam::BetweenFactor<gtsam::Vector3>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
  }

  // IMU state prediction for the current frame
  gtsam::NavState predicted_nav_world_imu = imu_integration->integrated_measurements().predict(last_nav_world_imu, last_imu_bias);
  gtsam::Pose3 predicted_T_world_imu = predicted_nav_world_imu.pose();
  gtsam::Vector3 predicted_v_world_imu = predicted_nav_world_imu.velocity();

  // Create new IMU pose, velocity, and bias variables and their timestamps for the current frame
  // These are used as initial guesses and then updated by the optimizer
  new_stamps[X(current)] = raw_frame->stamp;
  new_stamps[V(current)] = raw_frame->stamp;
  new_stamps[B(current)] = raw_frame->stamp;
  new_values.insert(X(current), predicted_T_world_imu);
  new_values.insert(V(current), predicted_v_world_imu);
  new_values.insert(B(current), last_imu_bias);

  // Create an identity between factor for the IMU bias based on the constant IMU bias assumption
  // B(last) and B(current) are the IMU bias variables for the last and current frames, respectively
  const double sqrt_dt = std::sqrt(raw_frame->stamp - last_stamp);
  gtsam::Vector6 bias_noise;
  bias_noise << gtsam::Vector3::Constant(params.imu_bias_noise_acc * sqrt_dt), gtsam::Vector3::Constant(params.imu_bias_noise_gyro * sqrt_dt);
  const auto bias_noise_model = gtsam::noiseModel::Diagonal::Sigmas(bias_noise);
  new_factors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), bias_noise_model));

  // Create an IMU factor
  gtsam::ImuFactor::shared_ptr imu_factor = gtsam::make_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
  new_factors.add(imu_factor);

  // Motion prediction for deskewing (intra-scan)
  std::vector<double> pred_imu_times;
  std::vector<Eigen::Isometry3d> pred_imu_poses;
  imu_integration->integrate_imu(raw_frame->stamp, raw_frame->scan_end_time, predicted_nav_world_imu, last_imu_bias, pred_imu_times, pred_imu_poses);

  // Create EstimationFrame
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;

  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->T_world_imu = Eigen::Isometry3d(predicted_T_world_imu.matrix());
  new_frame->T_world_lidar = Eigen::Isometry3d(predicted_T_world_imu.matrix()) * T_imu_lidar;
  new_frame->v_world_imu = predicted_v_world_imu;
  new_frame->imu_bias = last_imu_bias.vector();
  new_frame->raw_frame = raw_frame;

  // Save the predicted IMU poses at the IMU frequency
  new_frame->imu_rate_trajectory.resize(8, pred_imu_times.size());
  for (int i = 0; i < pred_imu_times.size(); i++) {
    const Eigen::Vector3d trans = pred_imu_poses[i].translation();
    const Eigen::Quaterniond quat(pred_imu_poses[i].linear());
    new_frame->imu_rate_trajectory.col(i) << pred_imu_times[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
  }

  // Deskew and tranform LiDAR points into the IMU frame
  auto deskewed = deskewing->deskew(T_imu_lidar, pred_imu_times, pred_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  for (auto& pt : deskewed) {
    pt = T_imu_lidar * pt;
  }

  // Re-estimate point normals and covariances after deskewing
  std::vector<Eigen::Vector4d> deskewed_normals;
  std::vector<Eigen::Matrix4d> deskewed_covs;
  covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);

  // Create a point cloud frame and create multi-resolution voxel maps
  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
  if (raw_frame->intensities.size()) {
    frame->add_intensities(raw_frame->intensities);
  }
  frame->add_covs(deskewed_covs);
  frame->add_normals(deskewed_normals);
  new_frame->frame = frame;
  new_frame->frame_id = FrameID::IMU;
  create_frame(new_frame);  // Upload to GPU and create multi-resolution voxel maps

  // Publish the new frame and add it to the frame list
  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);

  // Create matching cost factors for the current frame
  new_factors.add(create_matching_cost_factors(current));

  // Update smoother
  Callbacks::on_smoother_update(*smoother, new_factors, new_values, new_stamps);
  smoother->update(new_factors, new_values, new_stamps);
  Callbacks::on_smoother_update_finish(*smoother);

  // Find out marginalized frames
  while (marginalized_cursor < current) {
    double span = frames[current]->stamp - frames[marginalized_cursor]->stamp;
    if (span < params.smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }
  logger->debug("|frames|={} |active|={} |marginalized|={}", frames.size(), frames.inner_size(), marginalized_frames.size());
  Callbacks::on_marginalized_frames(marginalized_frames);

  // Update the states of the frames in the optimization window based on the latest optimization results
  update_frames(current);

  // Check if IMU prediction is good or not
  imu_validation->validate(
    Eigen::Isometry3d(last_T_world_imu.matrix()),
    last_v_world_imu,
    Eigen::Isometry3d(predicted_T_world_imu.matrix()),
    predicted_v_world_imu,
    new_frame->T_world_imu,
    new_frame->v_world_imu,
    new_frame->stamp - last_stamp);
  imu_validation->validate(new_frame->imu_bias);

  // Publish the updated frames in the optimization window
  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  Callbacks::on_update_new_frame(active_frames.back());
  Callbacks::on_update_frames(active_frames);
  logger->trace("frames updated");

  // Check if the optimizer fallback (optimization corruption) happened
  if (smoother->fallbackHappened()) {
    logger->warn("odometry estimation smoother fallback happened (time={})", raw_frame->stamp);
  }

  return frames[current];
}

/// @brief Image data callback (optional)
/// @param stamp   Timestamp
/// @param image   Image
void OdometryEstimationGPUPlain::insert_image(const double stamp, const cv::Mat& image) {
  logger->debug("insert_image stamp={}", stamp);
  // This function is intentionally left empty. You can implement image processing if needed.
}

/// @brief Pop out the remaining non-marginalized frames (called at the end of the sequence)
/// @return std::vector<EstimationFrame::ConstPtr>  Remaining non-marginalized frames
std::vector<EstimationFrame::ConstPtr> OdometryEstimationGPUPlain::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> marginalized_frames;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    marginalized_frames.push_back(frames[i]);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);

  return marginalized_frames;
}

/// @brief Convert the point cloud of the given frame to GPU-based point cloud and create multi-resolution voxel maps
/// @param frame  [in/out] Estimation frame
void OdometryEstimationGPUPlain::create_frame(EstimationFrame::Ptr& new_frame) {
  // Convert to PointCloudGPU
  new_frame->frame = gtsam_points::PointCloudGPU::clone(*new_frame->frame);

  // Create multi-resolution voxel maps
  for (int i = 0; i < params.voxelmap_levels; i++) {
    if (!new_frame->frame->size()) {
      break;
    }

    const double resolution = params.voxel_resolution * std::pow(params.voxelmap_scaling_factor, i);
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, *stream);
    voxelmap->insert(*new_frame->frame);
    new_frame->voxelmaps.push_back(voxelmap);
  }
}

/// @brief Create matching cost factors for the latest frame
/// @param current      Index of the latest frame
/// @return gtsam::NonlinearFactorGraph  New factors for the latest frame
gtsam::NonlinearFactorGraph OdometryEstimationGPUPlain::create_matching_cost_factors(const int current) {
  // We don't need to create matching cost factors for the first frame or if the current frame has no points
  if (current == 0 || !frames[current]->frame->size()) {
    return gtsam::NonlinearFactorGraph();
  }

  /// @brief Function to create a binary matching cost factor between two frames. (Both frames can move)
  /// @param factors      Factor graph to which the new factor will be added
  /// @param target_key   Key of the target frame
  /// @param source_key   Key of the source frame
  /// @param target       Target frame
  /// @param source       Source frame
  const auto create_binary_factor = [this](
                                      gtsam::NonlinearFactorGraph& factors,
                                      gtsam::Key target_key,
                                      gtsam::Key source_key,
                                      const glim::EstimationFrame::ConstPtr& target,
                                      const glim::EstimationFrame::ConstPtr& source) {
    // Get a CUDA stream and a temporary buffer for GPU computations
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    // Create one matching cost factor for each voxel map in the target frame
    for (const auto& voxelmap : target->voxelmaps) {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(target_key, source_key, voxelmap, source->frame, stream, buffer);
      factor->set_enable_surface_validation(true);
      factors.add(factor);
    }
  };

  /// @brief Function to create a unary matching cost factor for a keyframe. (Target frame is fixed, only source frame can move)
  /// @param factors              Factor graph to which the new factor will be added
  /// @param fixed_target_pose    Pose of the fixed target frame in the world frame
  /// @param source_key           Key of the source frame
  /// @param target               Target frame
  /// @param source               Source frame
  const auto create_unary_factor = [this](
                                     gtsam::NonlinearFactorGraph& factors,
                                     const gtsam::Pose3& fixed_target_pose,
                                     gtsam::Key source_key,
                                     const glim::EstimationFrame::ConstPtr& target,
                                     const glim::EstimationFrame::ConstPtr& source) {
    // Get a CUDA stream and a temporary buffer for GPU computations
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    // Create one matching cost factor for each voxel map in the target frame
    for (const auto& voxelmap : target->voxelmaps) {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(fixed_target_pose, source_key, voxelmap, source->frame, stream, buffer);
      factor->set_enable_surface_validation(true);
      factors.add(factor);
    }
  };

  gtsam::NonlinearFactorGraph factors;

  // Full connection window
  // Create factors between the latest frame and the previous frames within the full connection window (e.g., 3 frames)
  for (int target = current - params.full_connection_window_size; target < current; target++) {
    if (target < 0 || !frames.has_index(target)) {
      continue;
    }

    create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
  }

  // Keyframe matching factors
  // Create factors between the latest frame and all keyframes
  for (const auto& keyframe : keyframes) {
    if (keyframe->id >= current - params.full_connection_window_size) {
      // This keyframe is in the full connection window, so we skip it to avoid duplicate factors
      continue;
    }

    // Get a CUDA stream and a temporary buffer for GPU computations
    auto stream_buffer = stream_buffer_roundrobin->get_stream_buffer();
    const auto& stream = stream_buffer.first;
    const auto& buffer = stream_buffer.second;

    // Check if the keyframe is within the optimization window
    double span = frames[current]->stamp - keyframe->stamp;
    if (span > params.smoother_lag - 0.1 || !frames[keyframe->id]) {
      // The keyframe is outside the optimization window and is already marginalized
      // Create a unary factor with the keyframe's pose fixed in the world frame
      const gtsam::Pose3 key_T_world_imu(keyframe->T_world_imu.matrix());
      create_unary_factor(factors, key_T_world_imu, X(current), keyframe, frames[current]);
    } else {
      // The keyframe is still in the optimization window
      // Create a binary factor between the keyframe and the current frame
      const int target = keyframe->id;
      create_binary_factor(factors, X(target), X(current), frames[target], frames[current]);
    }
  }

  return factors;
}

/**
 * @brief Keyframe management based on an overlap metric
 * @ref   Koide et al., "Globally Consistent and Tightly Coupled 3D LiDAR Inertial Mapping", ICRA2022
 */
void OdometryEstimationGPUPlain::update_keyframes_overlap(int current) {
  if (!frames[current]->frame->size()) {
    return;
  }

  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  // Check the overlap between the current frame and all existing keyframes.
  // If the overlap is too high, we don't add the current frame as a new keyframe.
  std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> keyframes_(keyframes.size());
  std::vector<Eigen::Isometry3d> delta_from_keyframes(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    keyframes_[i] = keyframes[i]->voxelmaps.back();
    delta_from_keyframes[i] = keyframes[i]->T_world_imu.inverse() * frames[current]->T_world_imu;
  }

  const double overlap = gtsam_points::overlap_gpu(keyframes_, frames[current]->frame, delta_from_keyframes, *stream);
  if (overlap > params.keyframe_max_overlap) {
    // The current frame has too much overlap with existing keyframes, so we don't add it as a new keyframe.
    return;
  }

  // Insert the current frame as a new keyframe
  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= params.max_num_keyframes) {
    // The number of keyframes is within the limit, so we don't need to remove any keyframes.
    return;
  }

  // We need to remove some keyframes to keep the number of keyframes within the limit.
  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;

  // Remove keyframes without overlap to the new keyframe
  for (int i = 0; i < keyframes.size(); i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = gtsam_points::overlap_gpu(keyframes[i]->voxelmaps.back(), new_keyframe->frame, delta, *stream);
    if (overlap < params.keyframe_min_overlap) {
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      i--;
    }
  }

  if (keyframes.size() <= params.max_num_keyframes) {
    // We have removed enough keyframes, so we don't need to remove any more.
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    return;
  }

  // Remove the keyframe with the minimum score
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = 0; i < keyframes.size() - 1; i++) {
    const auto& keyframe = keyframes[i];
    const double overlap_latest = gtsam_points::overlap_gpu(keyframe->voxelmaps.back(), new_keyframe->frame, keyframe->T_world_imu.inverse() * new_keyframe->T_world_imu, *stream);

    std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> other_keyframes;
    std::vector<Eigen::Isometry3d> delta_from_others;
    for (int j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const auto& other = keyframes[j];
      other_keyframes.push_back(other->voxelmaps.back());
      delta_from_others.push_back(other->T_world_imu.inverse() * keyframe->T_world_imu);
    }

    const double overlap_others = gtsam_points::overlap_gpu(other_keyframes, keyframe->frame, delta_from_others, *stream);
    scores[i] = overlap_latest * (1.0 - overlap_others);
  }

  double min_score = scores[0];
  int frame_to_eliminate = 0;
  for (int i = 1; i < scores.size(); i++) {
    if (scores[i] < min_score) {
      min_score = scores[i];
      frame_to_eliminate = i;
    }
  }

  marginalized_keyframes.push_back(keyframes[frame_to_eliminate]);
  keyframes.erase(keyframes.begin() + frame_to_eliminate);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/**
 * @brief Keyframe management based on displacement criteria
 * @ref   Engel et al., "Direct Sparse Odometry", IEEE Trans. PAMI, 2018
 */
void OdometryEstimationGPUPlain::update_keyframes_displacement(int current) {
  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }

  // Check the displacement between the current frame and the last keyframe.
  // If the displacement is too small, we don't add the current frame as a new keyframe.
  const Eigen::Isometry3d delta_from_last = keyframes.back()->T_world_imu.inverse() * frames[current]->T_world_imu;
  const double delta_trans = delta_from_last.translation().norm();
  const double delta_rot = Eigen::AngleAxisd(delta_from_last.linear()).angle();

  if (delta_trans < params.keyframe_delta_trans && delta_rot < params.keyframe_delta_rot) {
    return;
  }

  // Insert the current frame as a new keyframe
  const auto& new_keyframe = frames[current];
  keyframes.push_back(new_keyframe);

  if (keyframes.size() <= params.max_num_keyframes) {
    // The number of keyframes is within the limit, so we don't need to remove any keyframes.
    return;
  }

  // Cull keyframes that have no overlap with the new keyframe
  for (int i = 0; i < keyframes.size() - 1; i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    const double overlap = gtsam_points::overlap_gpu(keyframes[i]->voxelmaps.back(), new_keyframe->frame, delta, *stream);

    if (overlap < 0.01) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
      return;
    }
  }

  // Compute the scores for all keyframes and remove the one with the maximum score
  const int leave_window = 2;
  const double eps = 1e-3;
  std::vector<double> scores(keyframes.size() - 1, 0.0);
  for (int i = leave_window; i < keyframes.size() - 1; i++) {
    double sum_inv_dist = 0.0;
    for (int j = 0; j < keyframes.size() - 1; j++) {
      if (i == j) {
        continue;
      }

      const double dist = (keyframes[i]->T_world_imu.translation() - keyframes[j]->T_world_imu.translation()).norm();
      sum_inv_dist += 1.0 / (dist + eps);
    }

    const double d0 = (keyframes[i]->T_world_imu.translation() - new_keyframe->T_world_imu.translation()).norm();
    scores[i] = std::sqrt(d0) * sum_inv_dist;
  }

  const auto max_score_loc = std::max_element(scores.begin(), scores.end());
  const int max_score_index = std::distance(scores.begin(), max_score_loc);

  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
  marginalized_keyframes.push_back(keyframes[max_score_index]);
  keyframes.erase(keyframes.begin() + max_score_index);
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

/// @brief Update the information of estimation frames in the optimization window after the smoother update
/// @param current      Index of the latest frame
/// @param new_factors  New factors for the latest frame
void OdometryEstimationGPUPlain::update_frames(int current) {
  logger->trace("update frames current={} marginalized_cursor={}", current, marginalized_cursor);

  for (int i = marginalized_cursor; i < frames.size(); i++) {
    try {
      Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(smoother->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
      Eigen::Vector3d v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(i));
      Eigen::Matrix<double, 6, 1> imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(i)).vector();

      frames[i]->T_world_imu = T_world_imu;
      frames[i]->T_world_lidar = T_world_imu * T_imu_lidar;
      frames[i]->v_world_imu = v_world_imu;
      frames[i]->imu_bias = imu_bias;
    } catch (std::out_of_range& e) {
      logger->error("caught {}", e.what());
      logger->error("current={}", current);
      logger->error("marginalized_cursor={}", marginalized_cursor);
      Callbacks::on_smoother_corruption(frames[current]->stamp);
      break;
    }
  }

  // Choose either overlap-based or displacement-based keyframe management
  // Overlap-based is more adaptive to the environment, but a bit tricky to tune the parameters.
  // Displacement-based is simpler and more intuitive, but may not be optimal for some sensors and environments.
  update_keyframes_overlap(current);
  // update_keyframes_displacement(current);

  Callbacks::on_update_keyframes(keyframes);
}

}  // namespace glim
