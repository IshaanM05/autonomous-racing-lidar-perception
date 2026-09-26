# Autonomous Racing LiDAR Perception

A ROS2 perception pipeline built for **IITB Racing Driverless**'s Perception subsystem (Trainee Winter Project) that turns raw LiDAR point clouds into classified track cones for an autonomous formula-style race car. Built collaboratively — LiDAR ground segmentation, clustering, and real-time performance tuning by [@IshaanM05](https://github.com/IshaanM05), cone-color classification (SVM/LightGBM) by teammate Atharav Sonawane.

## Pipeline

The car needs to tell blue cones from yellow cones from orange cones, in real time, from a raw `sensor_msgs/PointCloud` stream:

1. **Ground segmentation** — iterative RANSAC plane fitting (PCL `SACSegmentation`) removes ground points from each scan, isolating potential obstacles
2. **Clustering** — remaining non-ground points are grouped into per-object clusters (candidate cones)
3. **Feature extraction** (`feature_extractor.cpp`) — per cluster, computes point count, average/std-dev of intensity ("band") deviation, and per-band point-count histograms, written out as labelled CSV rows (`features_svm.csv`, `features_lgbm_custom_track_*.csv`)
4. **Classification** — an SVM and a LightGBM model (`lgbm_featuresdat.joblib`) trained on those features classify each cluster as blue / yellow / orange
5. **Tracking (experimental)** — a 2D Kalman filter (`experimental.txt`) prototypes frame-to-frame cone tracking to smooth out classification noise
6. **Visualization** — classified cones, ground/non-ground points, and clusters are all published as RViz `MarkerArray`s for live debugging (`rviz_config/config.rviz`)

## Performance work

The initial single-threaded implementation couldn't keep up with the LiDAR's scan rate. The optimized path (`fastworkiniter.txt`) reworks the node around a producer/consumer queue with a dedicated processing thread and OpenMP-parallelized clustering, with `time.time()` instrumentation added to profile each stage (RANSAC, clustering, feature extraction) independently. Iteration history for this optimization work is kept as checkpointed source snapshots (`workiniter.txt` → `fastworkiniter.txt` → `finalworkiniter.txt`) alongside a stable git branch (`stable_cst1_2`) pinned to the working LightGBM model.

## Stack

`ROS2` (rclcpp) · `PCL` (segmentation, filtering) · `Open3D` · `Eigen` · `OpenMP` · `scikit-learn` (SVM) · `LightGBM`

## Repo layout

```
src/perception_winter/          # ROS2 package (ament_cmake)
  src/process_lidar.cpp         # ground segmentation + clustering node
  src/feature_extractor.cpp     # per-cluster feature extraction node
  launch/main.launch.py
  rviz_config/config.rviz
csv_creater.cpp                 # standalone feature-CSV generation utility
lidar_band_statistics.csv       # per-band intensity stats from real scans
lidar_cluster_debug.csv         # cluster-level debug output
features_lgbm_custom_track_*.csv  # labelled training data for the classifier
lgbm_featuresdat.joblib         # trained LightGBM classifier
```

## Running it

```bash
# inside a ROS2 workspace
colcon build --packages-select perception_winter
source install/setup.bash
ros2 launch perception_winter main.launch.py
```

Expects a `sensor_msgs/msg/PointCloud` stream on `/carmaker/pointcloud` (e.g. from a CARMaker simulation rosbag, or the car's live LiDAR driver).
