# LD06 2D LiDAR – Logitech C930e calibration

ROS 2 Humble / Ubuntu 22.04 prototype for estimating `T_camera_lidar` using a checkerboard and point-to-plane optimization.

## Inputs and conventions

- `/scan`: `sensor_msgs/msg/LaserScan`, metres.
- `/camera/image_raw`: raw `sensor_msgs/msg/Image`, 1280×720.
- Checkerboard: 10×7 squares, **9×6 inner corners**, square size **0.025 m**.
- Transform: `p_camera_optical = R @ p_lidar + t`.
- Camera optical axes: right/down/forward. Use the actual LaserScan frame from the driver.

The supplied camera intrinsics are a device-specific reference for a C930e at focus 0 and zoom 100. Calibrate your own camera at the exact capture mode, focus and zoom before collecting extrinsic samples. No precomputed extrinsic is supplied.

## Install and build

Keep the existing camera and LiDAR drivers. Install missing dependencies:

```bash
sudo apt install python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-opencv python3-yaml ros-humble-rclpy ros-humble-cv-bridge \
  ros-humble-sensor-msgs ros-humble-std-srvs ros-humble-ament-index-python
```

From the repository root:

```bash
source /opt/ros/humble/setup.bash
# Useful when user-site NumPy conflicts with ROS binary packages:
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
colcon --log-base /tmp/lidar-calib/log build \
  --base-paths "$PWD" --build-base /tmp/lidar-calib/build \
  --install-base /tmp/lidar-calib/install --packages-select lidar_camera_calib
source /tmp/lidar-calib/install/setup.bash
```

Source ROS and the install setup in each terminal. Rebuild if `/tmp` is cleared. All nodes and drivers must use the same DDS domain. Do not start a second camera driver on the same device.

## Capture

Mount both sensors rigidly. Use a flat, rigid, matte board. The camera must see all inner corners and the laser plane must intersect the solid board. Keep the board stationary for each capture.

```bash
ros2 run lidar_camera_calib capture --ros-args \
  -p camera_yaml:="$PWD/config/camera.yaml" -p output_dir:="$PWD/samples"
```

The image window shows detected corners. The LiDAR top view shows gray scan points, orange ROI points, and the selected green segment. **READY does not prove the segment belongs to the board.** Move the board to identify its scan segment, isolate it with the angle/range gates, then verify it before saving.

Adjust these example parameters to the actual board location:

```bash
ros2 param set /lidar_camera_calib_capture angle_min_deg -12.0
ros2 param set /lidar_camera_calib_capture angle_max_deg 8.0
ros2 param set /lidar_camera_calib_capture min_range_m 0.35
ros2 param set /lidar_camera_calib_capture max_range_m 0.70
```

Angles are in degrees and ranges in metres. Press **C** to save one sample, **Q** to exit, or call:

```bash
ros2 service call /lidar_camera_calib_capture/capture std_srvs/srv/Trigger '{}'
```

Collect 20–30 distinct positions and tilts about multiple axes. Move only the board. Each NPZ contains selected and full valid scan points, ROI, timestamps, frames, board pose, corners and camera configuration; a PNG records detected corners. Do not mix sensor mountings or camera settings in one directory.

Defaults: at least 6 points, one contiguous candidate cluster, line RMSE ≤8 mm, image reprojection RMSE ≤1.5 px, timestamp difference ≤0.1 s. Use `show_preview:=false` without a desktop and verify ROI independently. This is buffered approximate timestamp pairing, not hardware synchronization or scan deskew.

## Solve

```bash
ros2 run lidar_camera_calib solve --data-dir samples --output extrinsic.yaml
```

The solver derives an initial transform from the data, projects rotation onto SO(3), then optimizes robust point-to-plane residuals with balanced pose weighting. It checks frame/config consistency, board-normal diversity, convergence, Jacobian rank/condition, residuals and positive camera depth. Failed checks do not overwrite the requested output; an older file may still exist.

Optional `--init RX RY RZ TX TY TZ` accepts a measured approximate Rodrigues vector in radians and translation in camera optical coordinates in metres. It is not roll/pitch/yaw. Do not invent the mounting transform or scale translation to force a chosen distance.

## Independent evaluation

Capture a new set using `-p output_dir:="$PWD/samples_check"`, keeping both sensors and camera settings unchanged. Do not optimize on this set:

```bash
ros2 run lidar_camera_calib evaluate \
  --data-dir samples_check --extrinsic extrinsic.yaml \
  --camera-yaml config/camera.yaml --training-dir samples \
  --output evaluation.yaml
```

Reports include point-weighted/equal-sample RMSE, signed bias, MAE, P95, maximum error, and per-sample CSV. The evaluator never changes the transform and rejects overlapping training/check timestamp pairs. Point-plane RMSE is not pixel error or 3D pen-tip accuracy.

## Visual validation

```bash
ros2 run lidar_camera_calib validate --ros-args \
  -p camera_yaml:="$PWD/config/camera.yaml" \
  -p extrinsic_yaml:="$PWD/extrinsic.yaml"
```

Green points are the selected ROI segment; gray points are the remaining scan. Projection uses K/D on the raw image, filters points behind the camera/outside the image, and displays live point-plane RMSE. It does not infer occlusions. ROI parameters on `/lidar_camera_calib_validate` are independent of capture; set the same gates explicitly. Invalid/stale input suppresses successful overlays or RMSE.

To compare two transforms on the exact same pair and ROI, pass `-p compare_extrinsic_yaml:=/absolute/path/other.yaml`. A is left, B is right. Neither file is changed.

## Independent range check

Place a stationary plane perpendicular to laser +X, covering the ±2° sector. Measure from the laser origin to the plane with a ruler:

```bash
ros2 run lidar_camera_calib check_range
# Only if the ruler actually reads 0.50 m:
ros2 run lidar_camera_calib check_range --distance-m 0.50
```

This summarizes X=range*cos(angle) over 50 scan medians without using the camera or extrinsic. Keep hands/background out of the sector. It applies no correction.

## Tests and practical limits

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m unittest discover -s tests -v
```

Tests cover synthetic transform recovery/noise, degeneracy, ROI ambiguity, invalid ranges, distortion projection, fixed-transform evaluation, overlap rejection, validation rendering/comparison and forward range geometry. Real hardware validation remains necessary. A recorded prototype run achieved 8.26 mm training point-plane RMSE and 12.30 mm on 14 held-out samples; this is device/session-specific and does not establish pen-tip accuracy.

A plane constraint extends infinitely: a small residual does not prove a projected point lies within the board. A small 2D laser target may have too few returns; the pen tip must intersect the scan plane and produce identifiable returns before image/scan association can recover a meaningful distance.

Raw images, sensor recordings, calibration sessions, local reports, build outputs and credentials are excluded from this source distribution.

## Source layout

- `capture.py`: subscriptions, pairing, board pose, manual ROI, previews, saving.
- `geometry.py`: geometry and projection shared by nodes.
- `solve.py`: offline optimization and output checks.
- `evaluate.py`: held-out fixed-transform metrics.
- `validate.py`: distortion-aware live overlay and comparison.
- `check_range.py`: independent forward range diagnostic.
- `tests/`: synthetic/unit tests; `config/`: camera and board configuration.

See [OpenCV pose conventions](https://docs.opencv.org/4.10.0/d5/d1f/calib3d_solvePnP.html).
