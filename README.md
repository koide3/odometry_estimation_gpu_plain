# odometry_estimation_gpu_plain

## About

This package offers a plain implementation of the LiDAR-IMU odometry estimation in GLIM for educational purposes. Some of the unnecessary features for extension purposes in the original implementation are removed, and the code is simplified to make it easier to understand.

Paper: Koide et al., GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors, Robotics and Autonomous Systems, 2024


## How to use

### Installation

Assuming you have already installed GLIM.

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/koide3/odometry_estimation_gpu_plain

# Build the package
cd ..
colcon build

# Check the shared library is built
ls install/odometry_estimation_gpu_plain/lib/
# libodometry_estimation_gpu_plain.so
```

### Configuration

#### 1. Copy the template configuration files to your local config directory:
```bash
cp -R ~/ros2_ws/src/glim/config ~/config
cp ~/ros2_ws/src/odometry_estimation_gpu_plain/config/config_odometry_gpu_plain.json ~/config/
```

#### 2. Edit `config/config.json` to switch the odometry estimation configuration to `config_odometry_gpu_plain.json`:

```json
{
  "global": {
    // ...
    "config_odometry": "config_odometry_gpu_plain.json",  // Edit this line
  }
}
```

#### 3. Edit `config/config_ros.json` to modify the input topics

```json
{
  "glim_ros": {
    // ...
    "imu_topic": "/livox/imu",
    "points_topic": "/livox/lidar",
    // ...
  }
}
```

### Run

Test data:
- [mid360_01.tar.gz (154MB)](https://zenodo.org/record/7233945/files/mid360_01.tar.gz?download=1)
- [mid360_02.tar.gz (253MB)](https://zenodo.org/record/7233945/files/mid360_02.tar.gz?download=1)

Run with the configuration
```bash
ros2 run glim_ros glim_rosbag mid360_02 --ros-args -p config_path:=$(realpath ~/config)
```
