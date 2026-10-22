# LD06 LiDAR–camera calibration và overlay

ROS 2 Humble / Ubuntu 22.04. Tìm phép biến đổi từ LiDAR 2D LD06 sang camera Logitech C930e bằng bảng caro, rồi chiếu đường quét LiDAR lên ảnh camera (tô màu theo khoảng cách, có nhãn mét), chạy được trên laptop và Raspberry Pi 4.

```text
p_camera_optical = R @ p_lidar + t
```

Kết quả: một đường quét 2D bám đúng lên vật trong ảnh (mặt bàn, lưng ghế, tờ giấy, người), kèm khoảng cách. Sai số quan sát bằng mắt khoảng 1–2 cm, **chưa đo bằng thước** (xem [Kết quả và hạn chế](#kết-quả-và-hạn-chế)).

## Cấu hình

| Thành phần | Giá trị |
|---|---|
| LiDAR | LD06 qua bộ chuyển CP210x, `/scan`, `sensor_msgs/msg/LaserScan`, mét, 10 Hz, 230400 baud |
| Camera | Logitech C930e, `/camera/image_raw`, 1280×720, YUYV (~10 fps) |
| Bảng | 10×7 ô, **9×6 góc trong**, mỗi ô **25 mm** |
| Camera optical | X sang phải, Y xuống, Z ra trước |
| Lắp đặt | camera đặt **trên đầu LiDAR**: lệch 1,8 cm về phía trước, cao 3 cm so với mặt phẳng quét |

`config/camera.yaml` là intrinsic tham khảo của một chiếc C930e ở focus 0, zoom 100. Camera khác, hoặc mode/focus/zoom khác, cần intrinsic riêng. Gắn cứng hai cảm biến; chỉ di chuyển bảng khi thu mẫu.

## Cài đặt

### Driver

```bash
# Camera
sudo apt install ros-humble-v4l2-camera v4l-utils

# LiDAR LD06 (driver chính thức của LDRobot)
mkdir -p ~/lidar_ws/src && cd ~/lidar_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ~/lidar_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select ldlidar_stl_ros2
```

Nếu `/dev/ttyUSB0` hiện lên rồi biến mất: `sudo apt remove brltty`. Thêm user vào nhóm `dialout`, `video` rồi đăng nhập lại.

### Gói này

```bash
sudo apt install python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-opencv python3-yaml ros-humble-cv-bridge

source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
colcon --log-base /tmp/lidar-calib/log build \
  --base-paths "$PWD" --build-base /tmp/lidar-calib/build \
  --install-base /tmp/lidar-calib/install --packages-select lidar_camera_calib
source /tmp/lidar-calib/install/setup.bash
```

Source lại ở mỗi terminal; build lại nếu `/tmp` bị xóa. Mọi node cần cùng DDS domain. Chi tiết trong [docs/USAGE.md](docs/USAGE.md).

## Chạy driver

```bash
# Terminal 1: LiDAR
source ~/lidar_ws/install/setup.bash
ros2 launch ldlidar_stl_ros2 ld06.launch.py

# Terminal 2: camera. Số /dev/videoN lấy từ `v4l2-ctl --list-devices` (thiết bị đầu tiên của C930e)
v4l2-ctl -d /dev/videoN -c focus_automatic_continuous=0 && v4l2-ctl -d /dev/videoN -c focus_absolute=0
ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/camera \
  -p video_device:=/dev/videoN -p image_size:="[1280,720]" \
  -p pixel_format:=YUYV -p camera_frame_id:=camera_optical_frame
```

Mỗi lần node camera khởi động lại, autofocus tự bật lại, nên chạy lại hai lệnh `v4l2-ctl`. Kiểm tra: `ros2 topic hz /scan /camera/image_raw` (cả hai ≈ 10 Hz).

## Quy trình calib

### 1. Thu mẫu với `capture_guide`

`capture_guide` thay `capture`, báo trực tiếp khi nào nên bấm `C` và còn thiếu tư thế nào:

```bash
ros2 run lidar_camera_calib capture_guide --ros-args \
  -p camera_yaml:="$PWD/config/camera.yaml" -p output_dir:="$PWD/samples_new" \
  -p angle_min_deg:=-30.0 -p angle_max_deg:=30.0 -p max_range_m:=1.5
```

Thanh màu ở đầu cửa sổ: **xanh lá** = bấm `C` ngay (tư thế tốt và mới), **vàng** = hợp lệ nhưng giống mẫu đã lưu, **đỏ** = chưa sẵn sàng (kèm lý do; terminal cũng in `NOT READY: ...`). Dòng chữ nhỏ liệt kê tư thế còn thiếu (gần/giữa/xa, nghiêng trái/phải/trên/dưới).

Điều kiện để xanh lá: bảng được nhận ra, ≥ 8 điểm LiDAR, bảng cách camera 0,3–1,6 m, đoạn LiDAR dài 6–50 cm và khớp vị trí bảng trong ảnh, bảng đứng yên ~0,3 s, khác các mẫu đã lưu. Các ngưỡng nới sẵn cho trường hợp **cầm bảng bằng tay**; chỉnh bằng `-p`. Node chọn đúng cụm LiDAR của bảng bằng vị trí bảng từ camera nên không cần dọn sạch ROI khỏi tay và vật khác. Chỉ báo `no LiDAR segment matches the board` khi tay dính liền vào bảng ở độ cao mặt phẳng quét, lúc đó hãy cầm ở mép dưới hoặc sau bảng.

**Yêu cầu dữ liệu:** 20–30 mẫu, trong đó ≥ 10 mẫu bảng **nghiêng mạnh (30–45°)** theo cả bốn hướng, khoảng cách 0,5–1,0 m. Bảng gần như luôn hướng thẳng vào camera thì `solve` ra nghiệm "đẹp" nhưng sai vật lý (xem [Xử lý sự cố](#xử-lý-sự-cố)). Dùng bảng phẳng, cứng, dán lên tấm cứng.

### 2. Tính extrinsic

```bash
ros2 run lidar_camera_calib solve --data-dir samples_new --output extrinsic.yaml \
  --camera-pos 0.018 0 0.03
```

- `solve` tự loại mẫu ngoại lai (sai bảng, rung), từng mẫu một, và in tên file bị loại. Tắt bằng `--no-reject`; dừng loại khi còn `--min-poses` mẫu (mặc định 12).
- `--camera-pos FWD LEFT UP` (mét, theo hệ LiDAR) cố định vị trí camera theo số đo thật và chỉ giải phần xoay. **Khuyên dùng.** Calib bằng mặt phẳng quét xác định tịnh tiến rất yếu: nghiệm tự do từng đặt camera lệch 16 cm theo chiều cao dù RMSE vẫn "tốt" (11,7 mm).
- Từ chối ghi file nếu RMSE > `--max-rmse` (mặc định 0,02 m).

### 3. Overlay

```bash
ros2 run lidar_camera_calib overlay --ros-args \
  -p extrinsic_yaml:="$PWD/extrinsic.yaml" -p near_m:=0.2 -p far_m:=3.0
```

Đường quét tô màu theo khoảng cách (xanh = gần, đỏ = xa), các điểm liền kề được nối thành đường, có nhãn mét cạnh điểm và hai dòng thông số ở góc dưới trái:

```text
99/377 scan points in image | scan-image dt 1 ms | LiDAR 10.0 Hz
9.4 FPS | delay 49 ms | CPU 40% | 51 C | node 83% of 1 core, 218 MB
```

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `near_m`, `far_m` | 0.2, 4.0 | dải khoảng cách của thang màu |
| `stream_width` | 640 | thu ảnh về bề rộng này trước khi vẽ và publish (0 = giữ 1280) |
| `label_gap_px` | 80 | khoảng cách tối thiểu giữa hai nhãn mét (0 = tắt nhãn) |
| `height_m` | 0 | vẽ thêm sọc dọc; trục z LiDAR trong nghiệm này hướng **xuống** nên dùng số âm |
| `show_preview` | true | cửa sổ OpenCV; tắt trên Pi không có màn hình |
| `publish_image` | true | publish ảnh ở `/lidar_camera_calib/overlay` |

Node chỉ xử lý khi có khung ảnh mới (không vẽ lại cùng một khung), nên nhẹ hơn nhiều so với vẽ theo timer.

### 4. Đánh giá độc lập (khuyến nghị)

```bash
ros2 run lidar_camera_calib capture_guide --ros-args -p output_dir:="$PWD/samples_check" \
  -p angle_min_deg:=-30.0 -p angle_max_deg:=30.0
ros2 run lidar_camera_calib evaluate --data-dir samples_check --extrinsic extrinsic.yaml \
  --camera-yaml config/camera.yaml --training-dir samples_new --output evaluation.yaml
```

Không chạy `solve` trên tập kiểm tra. `validate` vẽ riêng đoạn ROI của bảng và so sánh hai nghiệm.

## Chạy trên Raspberry Pi

Script [scripts/deploy_to_pi.sh](scripts/deploy_to_pi.sh) đẩy gói và driver LD06 lên `~/lidar_cam/src` trên Pi (cùng cách với `CoVip/scripts/deploy_to_pi.sh`: một kết nối SSH dùng chung, `DRY_RUN=1` để xem trước). Không đẩy `samples*/`, `*.npz`.

```bash
./lidar_camera_calib/scripts/deploy_to_pi.sh                     # chỉ copy
INSTALL_DEPS=1 BUILD=1 ./lidar_camera_calib/scripts/deploy_to_pi.sh   # lần đầu: cài apt + build
```

Biến môi trường: `PI_HOST` (mặc định `piros2@192.168.50.1`), `LDLIDAR_SRC`, `ROS_DISTRO_PI`. `extrinsic.yaml` bị `.gitignore` nhưng vẫn được copy từ máy.

Trên Pi, 4 terminal (đều `source /opt/ros/humble/setup.bash && source ~/lidar_cam/install/setup.bash`):

```bash
ros2 launch ldlidar_stl_ros2 ld06.launch.py                       # T1
ros2 run v4l2_camera v4l2_camera_node --ros-args ...              # T2 (như phần "Chạy driver")
cd ~/lidar_cam/src/lidar_camera_calib                             # T3
ros2 run lidar_camera_calib overlay --ros-args -p extrinsic_yaml:="$PWD/extrinsic.yaml" -p show_preview:=false
ros2 run web_video_server web_video_server                        # T4 (bỏ qua nếu đã chạy sẵn)
```

Xem trên laptop: `http://<IP Pi>:8080/stream_viewer?topic=/lidar_camera_calib/overlay`. Ảnh phát 640×360 nên `web_video_server` nhẹ.

## Kết quả và hạn chế

**Calib (47 mẫu thu bằng `capture_guide`, bảng cầm tay):**

| | Giá trị |
|---|---|
| Mẫu dùng / loại tự động | 38 / 9 |
| RMSE point-to-plane | 18,2 mm (vị trí camera cố định theo số đo); nghiệm tự do 11,7 mm nhưng vô lý về vật lý |
| Nghiêng bảng | trung vị 20°, tối đa 62°, 12/47 mẫu > 30° |
| Sai số overlay quan sát | ~1–2 cm bằng mắt (chưa đo thước) |

**Tài nguyên (laptop Ryzen 7 5800H, không cửa sổ, 1280×720, 10 fps):** `overlay` ≈ 12–13% một lõi, 226 MB; driver camera ≈ 11%; driver LiDAR ≈ 2%.

**Trên Pi 4 (đọc từ 4 khung trích ra từ video quay màn hình stream, không phải đo liên tục):**

| | Giá trị |
|---|---|
| FPS xử lý | 8,7–10,0 |
| Delay (nhận ảnh → publish) | 37–59 ms |
| CPU cả máy (gồm camera, LiDAR, overlay, web_video_server) | 38–46% |
| `overlay` | 67–87% của một lõi, 215–218 MB |
| Nhiệt độ CPU | 50–52 °C |
| LiDAR | 10,0 Hz, ~97–103 / 377–381 điểm nằm trong ảnh |

**Hạn chế đã biết:**
- Một đường quét 2D chỉ cho một lát cắt ngang, không có độ cao. Camera phải nhìn thấy mặt phẳng quét; bảng/vật phải cắt mặt phẳng đó.
- Sai số 1–2 cm là quan sát bằng mắt, chưa đo bằng thước ở các khoảng cách cố định; `evaluate` trên tập mới cũng chưa chạy.
- Intrinsic trong `config/camera.yaml` là của một chiếc C930e khác; sai intrinsic làm lệch overlay ở xa tâm ảnh.
- FPS bị giới hạn ~10 do camera ở YUYV 1280×720 (chưa kiểm tra MJPEG).
- Nếu hạ/xoay camera sau khi thu mẫu thì `extrinsic.yaml` mất hiệu lực, phải thu lại.

## Xử lý sự cố

| Triệu chứng | Nguyên nhân / cách xử lý |
|---|---|
| `Need >=8 poses` | chưa đủ mẫu; kiểm tra đúng thư mục `--data-dir` |
| `Calibration rejected ... No output written` | RMSE > ngưỡng do mẫu sai bảng hoặc quá ít nghiêng; dùng `capture_guide`, thu thêm mẫu nghiêng 30–45° |
| RMSE tốt nhưng camera nằm cách LiDAR cả chục cm hoặc quay lệch hàng chục độ | nghiệm tịnh tiến yếu; dùng `--camera-pos` với số đo thật, thu thêm mẫu nghiêng |
| `ROI has N candidate segments` (với `capture` gốc) | có nhiều cụm trong ROI; dùng `capture_guide` (tự chọn cụm theo vị trí bảng) hoặc thu hẹp góc/range |
| `Failed opening device /dev/videoN` | số thiết bị đổi sau khi cắm lại USB, hoặc có hai node camera cùng chạy; `v4l2-ctl --list-devices`, `pkill -f v4l2_camera_node` |
| `get ldlidar data is time out` | LD06 không còn gửi dữ liệu: mất nguồn/dây lỏng, cắm lại cổng USB khác, tránh hub |
| `Timestamp mismatch` | camera hoặc LiDAR không chạy/quá chậm; kiểm tra `ros2 topic hz` |
| Ảnh mờ, overlay lệch dần | autofocus tự bật lại sau khi khởi động lại node camera; chạy lại lệnh `v4l2-ctl` |
| Sọc `height_m` hướng ngược | trục z LiDAR hướng xuống trong nghiệm này; dùng `height_m` âm |

## Kiểm thử

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m unittest discover -s tests -v
```

26 kiểm thử mô phỏng/unit cho geometry, solver, evaluator, projection, range, overlay (chiếu, nhãn, thu nhỏ ảnh) và `capture_guide` (chọn cụm, điều kiện sẵn sàng, độ phủ tư thế). Chưa có kiểm thử tích hợp với phần cứng thật.

## Source

- `capture.py`: nhận dữ liệu, tìm bảng, chọn ROI và lưu mẫu (thủ công).
- `capture_guide.py`: như `capture` nhưng báo khi nên chụp, chọn cụm LiDAR của bảng, gợi ý tư thế còn thiếu.
- `solve.py`: tối ưu point-to-plane, tự loại mẫu ngoại lai, `--camera-pos`, kiểm tra khả năng quan sát, ghi transform 4×4.
- `evaluate.py`: đánh giá transform cố định trên tập riêng.
- `validate.py`: overlay đoạn bảng và so sánh hai transform.
- `overlay.py`: chiếu toàn bộ scan 2D lên ảnh camera, tô màu theo range, nhãn mét, thông số FPS/CPU.
- `sysstats.py`: CPU/RAM/nhiệt độ đọc từ `/proc`.
- `check_range.py`: kiểm tra range LiDAR độc lập camera.
- `geometry.py`: hàm hình học dùng chung (góc quét chuẩn hoá về (-π, π] vì driver LD06 xuất 0..2π).
- `scripts/deploy_to_pi.sh`: đưa gói và driver lên Pi.

`.gitignore` loại dữ liệu thu mẫu (`samples*/`, `*.npz`), ảnh, build outputs và `extrinsic*.yaml` (riêng cho từng bộ lắp đặt); các file đó vẫn được giữ trên máy.
