# LD06 LiDAR–camera calibration

Prototype ROS 2 Humble / Ubuntu 22.04 để tìm phép biến đổi từ LiDAR 2D LD06 sang camera Logitech C930e:

```text
p_camera_optical = R @ p_lidar + t
```

## Cấu hình

| Thành phần | Giá trị |
|---|---|
| LiDAR | `/scan`, `sensor_msgs/msg/LaserScan`, mét |
| Camera | `/camera/image_raw`, ảnh raw 1280×720 |
| Bảng | 10×7 ô, **9×6 góc trong**, mỗi ô **25 mm** |
| Camera optical | X sang phải, Y xuống, Z ra trước |

`config/camera.yaml` chứa intrinsic tham khảo của một camera cụ thể, focus 0, zoom 100. Camera khác hoặc mode/focus/zoom khác cần intrinsic phù hợp. Gắn cứng hai cảm biến; chỉ di chuyển bảng khi thu mẫu.

## Build

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1

colcon --log-base /tmp/lidar-calib/log build \
  --base-paths "$PWD" --build-base /tmp/lidar-calib/build \
  --install-base /tmp/lidar-calib/install \
  --packages-select lidar_camera_calib
source /tmp/lidar-calib/install/setup.bash
```

Các terminal cần cùng DDS domain với driver. Source lại ở terminal mới; build lại nếu `/tmp` bị xóa. Dependency và xử lý môi trường xem [hướng dẫn chi tiết](docs/USAGE.md).

## Quy trình

```bash
# 1. Thu 20–30 tư thế; C lưu một mẫu, Q thoát
ros2 run lidar_camera_calib capture --ros-args \
  -p camera_yaml:="$PWD/config/camera.yaml" \
  -p output_dir:="$PWD/samples"

# 2. Tính extrinsic
ros2 run lidar_camera_calib solve \
  --data-dir samples --output extrinsic.yaml

# 3. Thu tư thế mới, giữ nguyên cảm biến và camera settings
ros2 run lidar_camera_calib capture --ros-args \
  -p camera_yaml:="$PWD/config/camera.yaml" \
  -p output_dir:="$PWD/samples_check"

# 4. Đánh giá trên tập mới, không tối ưu lại
ros2 run lidar_camera_calib evaluate \
  --data-dir samples_check --extrinsic extrinsic.yaml \
  --camera-yaml config/camera.yaml --training-dir samples \
  --output evaluation.yaml

# 5. Overlay có distortion; xanh là đoạn ROI được chọn
ros2 run lidar_camera_calib validate --ros-args \
  -p camera_yaml:="$PWD/config/camera.yaml" \
  -p extrinsic_yaml:="$PWD/extrinsic.yaml"
```

ROI phải chỉ chứa điểm thuộc bảng. **READY không xác nhận đó là bảng.** Giữ bảng đứng yên, chỉnh góc/range gate và xác nhận segment trước khi lưu. Không chạy `solve` trên tập kiểm tra. Tham số ROI của capture và validate độc lập.

## Kiểm thử

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m unittest discover -s tests -v
```

Có 10 kiểm thử mô phỏng/unit cho geometry, solver, evaluator, projection và range. Phiên cảm biến gần nhất có RMSE point-to-plane 8,26 mm trên tập calib và 12,30 mm trên 14 mẫu kiểm tra; không bảo đảm sai số 3D đầu bút tương ứng. Cần xác nhận overlay, occlusion và độ chính xác mục tiêu thực. Đầu bút phải tạo được điểm đo trong mặt phẳng quét LD06.

## Source

- `capture.py`: nhận dữ liệu, tìm bảng, chọn ROI và lưu mẫu.
- `solve.py`: tối ưu point-to-plane, kiểm tra khả năng quan sát, ghi transform 4×4.
- `evaluate.py`: đánh giá transform cố định trên tập riêng.
- `validate.py`: overlay và so sánh hai transform.
- `check_range.py`: kiểm tra range LiDAR độc lập camera.
- `geometry.py`: hàm hình học dùng chung.

Xem [docs/USAGE.md](docs/USAGE.md) để cài dependency, chỉnh ROI, so sánh nghiệm và kiểm tra range. `.gitignore` loại dữ liệu thu mẫu, ảnh, build outputs và cấu hình cá nhân; các file đó vẫn được giữ trên máy.
