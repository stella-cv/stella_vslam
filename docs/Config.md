# Stella VSLAM - Complete Configuration Guide

## Overview

This guide provides comprehensive information about all configuration parameters available in Stella VSLAM. The system uses YAML configuration files to control all aspects of the SLAM system, from camera setup to algorithm parameters.

## Configuration Structure

All configurations are defined in YAML files with the following structure:

```yaml
# Camera configuration
Camera:
  # Camera parameters...

# System configuration
System:
  # System parameters...

# Feature extraction
Feature:
  # Feature parameters...

# Preprocessing
Preprocessing:
  # Preprocessing parameters...

# Tracking
Tracking:
  # Tracking parameters...

# Mapping
Mapping:
  # Mapping parameters...

# Loop detection
LoopDetector:
  # Loop detection parameters...

# Global optimization
GlobalOptimizer:
  # Global optimization parameters...

# And more...
```

## Camera Configuration

### Basic Camera Information

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `name` | string | Camera identifier | - | `"RICOH THETA S 960"` |
| `setup` | string | Camera setup type | - | `"monocular"`, `"stereo"`, `"RGBD"` |
| `model` | string | Camera model type | - | `"perspective"`, `"fisheye"`, `"equirectangular"`, `"radial_division"` |

### Intrinsic Parameters

| Parameter | Type | Description | Units | Default | Example |
|-----------|------|-------------|-------|---------|---------|
| `fx` | double | Focal length in x-direction | pixels | - | `718.856` |
| `fy` | double | Focal length in y-direction | pixels | - | `718.856` |
| `cx` | double | Principal point x-coordinate | pixels | - | `607.1928` |
| `cy` | double | Principal point y-coordinate | pixels | - | `185.2157` |

### Distortion Parameters

#### Perspective Camera
| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `k1` | double | Radial distortion coefficient 1 | `0.0` | `0.262383` |
| `k2` | double | Radial distortion coefficient 2 | `0.0` | `-0.953104` |
| `p1` | double | Tangential distortion coefficient 1 | `0.0` | `-0.005358` |
| `p2` | double | Tangential distortion coefficient 2 | `0.0` | `0.002628` |
| `k3` | double | Radial distortion coefficient 3 | `0.0` | `1.163314` |

#### Fisheye Camera
| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `k1` | double | Fisheye distortion coefficient 1 | `0.0` | `0.1` |
| `k2` | double | Fisheye distortion coefficient 2 | `0.0` | `0.05` |
| `k3` | double | Fisheye distortion coefficient 3 | `0.0` | `0.0` |
| `k4` | double | Fisheye distortion coefficient 4 | `0.0` | `0.0` |

### Image Parameters

| Parameter | Type | Description | Units | Default | Example |
|-----------|------|-------------|-------|---------|---------|
| `fps` | double | Frame rate of input images | Hz | - | `30.0` |
| `cols` | unsigned int | Image width | pixels | - | `1241` |
| `rows` | unsigned int | Image height | pixels | - | `376` |
| `color_order` | string | Image colour format | - | - | `"Gray"`, `"RGB"`, `"RGBA"`, `"BGR"`, `"BGRA"` |

### Stereo/RGBD Parameters

| Parameter | Type | Description | Units | Default | Example |
|-----------|------|-------------|-------|---------|---------|
| `focal_x_baseline` | double | Baseline × focal length (stereo) or depth accuracy parameter (RGBD) | pixels × meters | `0.0` | `718.856` |
| `depth_threshold` | double | Depth threshold ratio for filtering | ratio | `40.0` | `40.0` |

## System Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `map_format` | string | Map storage format | `"msgpack"` | `"msgpack"`, `"sqlite3"` |
| `num_grid_cols` | unsigned int | Number of grid columns for spatial indexing | `64` | `96` |
| `num_grid_rows` | unsigned int | Number of grid rows for spatial indexing | `48` | `48` |

## Feature Extraction Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `name` | string | Feature extraction model name | `"default ORB feature extraction setting"` | `"custom ORB"` |
| `scale_factor` | float | Scale factor for image pyramid | `1.2` | `1.2` |
| `num_levels` | unsigned int | Number of pyramid levels | `8` | `8` |
| `ini_fast_threshold` | unsigned int | Initial FAST threshold | `20` | `20` |
| `min_fast_threshold` | unsigned int | Minimum FAST threshold | `7` | `7` |

## Preprocessing Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `min_size` | unsigned int | Minimum image size for processing | `800` | `800` |
| `depthmap_factor` | double | Depth map scaling factor | `1.0` | `5000.0` |
| `num_grid_cols` | unsigned int | Grid columns for preprocessing | `64` | `96` |
| `num_grid_rows` | unsigned int | Grid rows for preprocessing | `48` | `48` |
| `mask_rectangles` | array | Image mask rectangles | `[]` | `[[0.0, 1.0, 0.0, 0.1]]` |

## Tracking Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `backend` | string | Optimization backend | `"g2o"` | `"g2o"`, `"gtsam"` |
| `reloc_distance_threshold` | double | Relocalization distance threshold | `0.2` | `0.2` |
| `reloc_angle_threshold` | double | Relocalization angle threshold | `0.45` | `0.45` |
| `init_retry_threshold_time` | double | Initialization retry threshold time | `5.0` | `5.0` |
| `enable_auto_relocalization` | bool | Enable automatic relocalization | `true` | `true` |
| `enable_temporal_keyframe_only_tracking` | bool | Enable temporal keyframe only tracking | `false` | `false` |
| `use_robust_matcher_for_relocalization_request` | bool | Use robust matcher for relocalization | `false` | `false` |
| `max_num_local_keyfrms` | unsigned int | Maximum number of local keyframes | `60` | `60` |
| `margin_local_map_projection` | float | Local map projection margin | `5.0` | `10.0` |
| `margin_local_map_projection_unstable` | float | Unstable local map projection margin | `20.0` | `20.0` |
| `margin_last_frame_projection` | float | Last frame projection margin | `20.0` | `20.0` |

### G2O Backend Parameters
| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `num_trials_robust` | unsigned int | Number of robust trials | `2` | `2` |
| `num_trials` | unsigned int | Number of trials | `2` | `2` |
| `num_each_iter` | unsigned int | Number of iterations | `10` | `10` |

### GTSAM Backend Parameters
| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `num_iter` | unsigned int | Number of iterations | `5` | `5` |
| `relative_error_tol` | double | Relative error tolerance | `1e-2` | `1e-2` |
| `lambda_initial` | double | Initial lambda value | `1e-5` | `1e-5` |
| `lambda_upper_bound` | double | Lambda upper bound | `1e-2` | `1e-2` |
| `enable_outlier_elimination` | bool | Enable outlier elimination | `true` | `true` |
| `verbosity` | string | Verbosity level | `"SILENT"` | `"SILENT"` |

## Mapping Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `backend` | string | Optimization backend | `"g2o"` | `"g2o"`, `"gtsam"` |
| `baseline_dist_thr` | double | Baseline distance threshold | `1.0` | `0.07471049682` |
| `baseline_dist_thr_ratio` | double | Baseline distance threshold ratio | `0.02` | `0.02` |
| `redundant_obs_ratio_thr` | double | Redundant observations ratio threshold | `0.9` | `0.95` |
| `observed_ratio_thr` | double | Observed ratio threshold | `0.3` | `0.3` |
| `num_reliable_keyfrms` | unsigned int | Number of reliable keyframes | `2` | `2` |
| `num_covisibilities_for_landmark_generation` | unsigned int | Covisibilities for landmark generation | `10` | `20` |
| `num_covisibilities_for_landmark_fusion` | unsigned int | Covisibilities for landmark fusion | `10` | `20` |
| `residual_deg_thr` | float | Residual degree threshold | `0.2` | `0.4` |
| `enable_interruption_of_landmark_generation` | bool | Enable landmark generation interruption | `true` | `true` |
| `enable_interruption_before_local_BA` | bool | Enable interruption before local BA | `true` | `true` |
| `erase_temporal_keyframes` | bool | Erase temporal keyframes | `false` | `false` |
| `num_temporal_keyframes` | unsigned int | Number of temporal keyframes | `15` | `15` |
| `top_n_covisibilities_to_search` | unsigned int | Top N covisibilities to search | `30` | `30` |

### Additional G2O/GTSAM Parameters
| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `use_additional_keyframes_for_monocular` | bool | Use additional keyframes for monocular | `false` | `false` |

## Loop Detection Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `backend` | string | Optimization backend | `"g2o"` | `"g2o"`, `"gtsam"` |
| `enabled` | bool | Enable loop detection | `true` | `true` |
| `num_final_matches_threshold` | unsigned int | Final matches threshold | `40` | `40` |
| `min_continuity` | unsigned int | Minimum continuity | `3` | `3` |
| `reject_by_graph_distance` | bool | Reject by graph distance | `false` | `true` |
| `min_distance_on_graph` | unsigned int | Minimum distance on graph | `50` | `50` |
| `num_matches_thr` | unsigned int | Number of matches threshold | `20` | `20` |
| `num_matches_thr_robust_matcher` | unsigned int | Robust matcher matches threshold | `0` | `0` |
| `num_optimized_inliers_thr` | unsigned int | Optimized inliers threshold | `20` | `20` |
| `top_n_covisibilities_to_search` | unsigned int | Top N covisibilities to search | `0` | `0` |
| `use_fixed_seed` | bool | Use fixed seed | `false` | `false` |
| `num_common_words_thr_ratio` | float | Common words threshold ratio | `0.8` | `0.8` |

## Global Optimizer Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `thr_neighbor_keyframes` | unsigned int | Neighbor keyframes threshold | `15` | `100` |
| `num_iter` | unsigned int | Number of iterations | `10` | `10` |
| `use_huber_kernel` | bool | Use Huber kernel | `false` | `false` |

## Graph Optimizer Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `min_num_shared_lms` | unsigned int | Minimum number of shared landmarks | `100` | `200` |

## Initializer Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `num_ransac_iterations` | unsigned int | Number of RANSAC iterations | `100` | `100` |
| `min_num_valid_pts` | unsigned int | Minimum number of valid points | `50` | `50` |
| `min_num_triangulated_pts` | unsigned int | Minimum number of triangulated points | `50` | `100` |
| `parallax_deg_threshold` | float | Parallax degree threshold | `1.0` | `1.0` |
| `reprojection_error_threshold` | float | Reprojection error threshold | `4.0` | `4.0` |
| `num_ba_iterations` | unsigned int | Number of BA iterations | `20` | `20` |
| `scaling_factor` | float | Scaling factor | `1.0` | `1.0` |
| `use_fixed_seed` | bool | Use fixed seed | `false` | `false` |
| `required_keyframes_for_marker_initialization` | unsigned int | Required keyframes for marker initialization | `3` | `3` |

## Relocalizer Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `bow_match_lowe_ratio` | double | BoW match Lowe ratio | `0.75` | `0.75` |
| `proj_match_lowe_ratio` | double | Projection match Lowe ratio | `0.9` | `0.9` |
| `robust_match_lowe_ratio` | double | Robust match Lowe ratio | `0.8` | `0.8` |
| `min_num_bow_matches` | unsigned int | Minimum number of BoW matches | `20` | `20` |
| `min_num_valid_obs` | unsigned int | Minimum number of valid observations | `50` | `50` |
| `use_fixed_seed` | bool | Use fixed seed | `false` | `false` |
| `search_neighbor` | bool | Search neighbor | `true` | `true` |
| `top_n_covisibilities_to_search` | unsigned int | Top N covisibilities to search | `10` | `10` |
| `num_common_words_thr_ratio` | float | Common words threshold ratio | `0.8` | `0.8` |
| `max_num_ransac_iter` | unsigned int | Maximum number of RANSAC iterations | `30` | `30` |
| `max_num_local_keyfrms` | unsigned int | Maximum number of local keyframes | `60` | `60` |

## Keyframe Inserter Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `max_interval` | double | Maximum interval | `1.0` | `1.0` |
| `min_interval` | double | Minimum interval | `0.1` | `0.1` |
| `max_distance` | double | Maximum distance | `-1.0` | `-1.0` |
| `min_distance` | double | Minimum distance | `-1.0` | `-1.0` |
| `lms_ratio_thr_almost_all_lms_are_tracked` | double | Landmarks ratio threshold for tracking | `0.9` | `0.9` |
| `lms_ratio_thr_view_changed` | double | Landmarks ratio threshold for view change | `0.5` | `0.5` |
| `enough_lms_thr` | unsigned int | Enough landmarks threshold | `100` | `100` |
| `wait_for_local_bundle_adjustment` | bool | Wait for local bundle adjustment | `false` | `false` |
| `required_keyframes_for_marker_initialization` | unsigned int | Required keyframes for marker initialization | `3` | `3` |

## Stereo Rectifier Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `model` | string | Camera model type | `"perspective"` | `"perspective"`, `"fisheye"` |
| `K_left` | array | Left camera intrinsic matrix | - | `[458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0]` |
| `K_right` | array | Right camera intrinsic matrix | - | `[457.587, 0.0, 379.999, 0.0, 456.134, 255.238, 0.0, 0.0, 1]` |
| `D_left` | array | Left camera distortion parameters | - | `[-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0]` |
| `D_right` | array | Right camera distortion parameters | - | `[-0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0]` |
| `R_left` | array | Left camera rectification matrix | - | `[0.999966347530033, -0.001422739138722922, 0.008079580483432283, ...]` |
| `R_right` | array | Right camera rectification matrix | - | `[0.9999633526194376, -0.003625811871560086, 0.007755443660172947, ...]` |

## Marker Model Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `type` | string | Marker model type | - | `"aruco"`, `"aruconano"` |
| `width` | double | Physical size of marker | - | `0.1` |
| `marker_size` | double | Marker size | - | `4`, `5`, `6`, `7` |
| `max_markers` | double | Maximum number of markers | - | `50`, `100`, `250`, `1000` |
| `dict` | int | Dictionary ID (for aruconano) | `0` | `0` |

## Pangolin Viewer Configuration

| Parameter | Type | Description | Default | Example |
|-----------|------|-------------|---------|---------|
| `keyframe_size` | double | Keyframe size | - | `0.05` |
| `keyframe_line_width` | int | Keyframe line width | - | `1` |
| `graph_line_width` | int | Graph line width | - | `1` |
| `point_size` | int | Point size | - | `2` |
| `camera_size` | double | Camera size | - | `0.08` |
| `camera_line_width` | int | Camera line width | - | `3` |
| `viewpoint_x` | double | Viewpoint X coordinate | - | `0` |
| `viewpoint_y` | double | Viewpoint Y coordinate | - | `-0.9` |
| `viewpoint_z` | double | Viewpoint Z coordinate | - | `-1.9` |
| `viewpoint_f` | double | Viewpoint focal length | - | `400` |

## Configuration Examples

### KITTI Monocular
```yaml
Camera:
  name: "KITTI monocular"
  setup: "monocular"
  model: "perspective"
  fx: 718.856
  fy: 718.856
  cx: 607.1928
  cy: 185.2157
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
  fps: 10.0
  cols: 1241
  rows: 376
  color_order: "Gray"

Feature:
  name: "default ORB feature extraction setting"
  scale_factor: 1.2
  num_levels: 8
  ini_fast_threshold: 20
  min_fast_threshold: 7

Mapping:
  backend: "g2o"
  baseline_dist_thr_ratio: 0.02
  redundant_obs_ratio_thr: 0.9

Tracking:
  backend: "g2o"

System:
  map_format: "msgpack"
  num_grid_cols: 40
  num_grid_rows: 13
```

### EuRoC Stereo
```yaml
Camera:
  name: "EuRoC stereo"
  setup: "stereo"
  model: "perspective"
  fx: 435.2046959714599
  fy: 435.2046959714599
  cx: 367.4517211914062
  cy: 252.2008514404297
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
  fps: 20.0
  cols: 752
  rows: 480
  focal_x_baseline: 47.90639384423901
  depth_threshold: 40
  color_order: "Gray"

StereoRectifier:
  model: "perspective"
  K_left: [458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0]
  D_left: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0]
  K_right: [457.587, 0.0, 379.999, 0.0, 456.134, 255.238, 0.0, 0.0, 1]
  D_right: [-0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0]
  R_left: [0.999966347530033, -0.001422739138722922, 0.008079580483432283, ...]
  R_right: [0.9999633526194376, -0.003625811871560086, 0.007755443660172947, ...]

Mapping:
  backend: "g2o"
  baseline_dist_thr: 0.11007784219
  redundant_obs_ratio_thr: 0.9
  erase_temporal_keyframes: false
  num_temporal_keyframes: 15

Tracking:
  backend: "g2o"
  enable_temporal_keyframe_only_tracking: false
  margin_last_frame_projection: 10.0

System:
  map_format: "msgpack"
  num_grid_cols: 47
  num_grid_rows: 30
```

### TUM RGBD
```yaml
Camera:
  name: "TUM-RGBD RGBD"
  setup: "RGBD"
  model: "perspective"
  fx: 517.306408
  fy: 516.469215
  cx: 318.643040
  cy: 255.313989
  k1: 0.262383
  k2: -0.953104
  p1: -0.005358
  p2: 0.002628
  k3: 1.163314
  fps: 30.0
  cols: 640
  rows: 480
  focal_x_baseline: 40.0
  depth_threshold: 40.0
  color_order: "RGB"

Preprocessing:
  min_size: 800
  depthmap_factor: 5000.0

Mapping:
  baseline_dist_thr: 0.07471049682
  redundant_obs_ratio_thr: 0.9

Tracking:
  margin_local_map_projection: 10.0
```

### Equirectangular (360° Camera)
```yaml
Camera:
  name: "RICOH THETA S 960"
  setup: "monocular"
  model: "equirectangular"
  fps: 30.0
  cols: 1920
  rows: 960
  color_order: "RGB"

Preprocessing:
  min_size: 800
  mask_rectangles:
    - [0.0, 1.0, 0.0, 0.1]
    - [0.0, 1.0, 0.84, 1.0]
    - [0.0, 0.2, 0.7, 1.0]
    - [0.8, 1.0, 0.7, 1.0]

Mapping:
  backend: "g2o"
  baseline_dist_thr_ratio: 0.02
  redundant_obs_ratio_thr: 0.95
  num_covisibilities_for_landmark_generation: 20
  num_covisibilities_for_landmark_fusion: 20
  residual_deg_thr: 0.4

LoopDetector:
  backend: "g2o"
  enabled: true
  reject_by_graph_distance: true
  min_distance_on_graph: 50

System:
  map_format: "msgpack"
  num_grid_cols: 96
  num_grid_rows: 48
```

## Available Options

### Map Storage Formats
- **msgpack**: Binary format for efficient storage (default)
- **sqlite3**: Database format for complex queries

### Trajectory Export Formats
- **KITTI**: KITTI dataset format (4x4 transformation matrix)
- **TUM**: TUM RGB-D dataset format (timestamp, translation, quaternion)

### Optimization Backends
- **g2o**: Default optimization backend (always available)
- **gtsam**: Alternative optimization backend (requires GTSAM compilation)

### Camera Models
- **perspective**: Standard pinhole camera model
- **fisheye**: Wide-angle fisheye lens model
- **equirectangular**: 360° panoramic camera model
- **radial_division**: Custom radial distortion model

### Camera Setups
- **monocular**: Single camera
- **stereo**: Two-camera setup
- **RGBD**: RGB + depth camera

### Marker Models
- **aruco**: Standard ArUco markers
- **aruconano**: ArUco Nano markers (if compiled with support)

### BOW Frameworks
- **FBoW**: Fast Bag of Words (default)
- **DBoW2**: DBoW2 framework (alternative)

### Compilation Options
- **USE_GTSAM**: Enable GTSAM backend support
- **USE_ARUCO**: Enable ArUco marker detection
- **USE_ARUCO_NANO**: Enable ArUco Nano marker detection
- **USE_OPENMP**: Enable OpenMP parallelization
- **USE_SSE_ORB**: Enable SSE instructions for ORB extraction
- **USE_SSE_FP_MATH**: Enable SSE instructions for floating-point operations
- **USE_TRACY**: Enable Tracy profiler integration

## Parameter Tuning Guidelines

### Performance vs Accuracy
- **High Performance**: Reduce `num_levels`, increase `scale_factor`, lower thresholds
- **High Accuracy**: Increase `num_levels`, decrease `scale_factor`, higher thresholds

### Memory Usage
- **Low Memory**: Reduce `num_grid_cols/rows`, lower `num_temporal_keyframes`
- **High Memory**: Increase grid size, more temporal keyframes

### Robustness
- **High Robustness**: Increase `min_num_valid_pts`, lower `reprojection_error_threshold`
- **Fast Tracking**: Decrease thresholds, enable `enable_temporal_keyframe_only_tracking`

### Loop Detection
- **Aggressive**: Lower `min_distance_on_graph`, higher `num_matches_thr`
- **Conservative**: Higher `min_distance_on_graph`, lower `num_matches_thr`

## Advanced Configuration

### Compilation-Time Options
```bash
# Enable GTSAM backend
cmake -DUSE_GTSAM=ON ..

# Enable ArUco marker detection
cmake -DUSE_ARUCO=ON ..

# Enable ArUco Nano
cmake -DUSE_ARUCO_NANO=ON ..

# Enable OpenMP
cmake -DUSE_OPENMP=ON ..

# Enable SSE optimizations
cmake -DUSE_SSE_ORB=ON -DUSE_SSE_FP_MATH=ON ..

# Choose BOW framework
cmake -DBOW_FRAMEWORK=DBoW2 ..
```

### Runtime Configuration
```cpp
// Save map in different formats
slam.save_map_database("map.msg");  // MessagePack format
slam.save_map_database("map.db");   // SQLite format

// Save trajectories in different formats
slam.save_frame_trajectory("trajectory.txt", "TUM");
slam.save_keyframe_trajectory("keyframe_trajectory.txt", "KITTI");
``` 