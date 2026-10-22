#!/usr/bin/env bash
# deploy_to_pi.sh — Đưa gói lidar_camera_calib + driver LD06 lên Raspberry Pi qua ssh/scp.
#
# Cùng mẫu với CoVip/scripts/deploy_to_pi.sh. Trên Pi tạo workspace riêng:
#   ~/lidar_cam/src/lidar_camera_calib   (gói này + extrinsic.yaml)
#   ~/lidar_cam/src/ldlidar_stl_ros2     (driver LDRobot, lấy từ ~/lidar_ws/src trên máy dev)
#
# Chạy (từ bất kỳ đâu):
#   ./lidar_camera_calib/scripts/deploy_to_pi.sh
#
# Đổi Pi:            PI_HOST=user@10.0.0.5 ./lidar_camera_calib/scripts/deploy_to_pi.sh
# Chỉ in, không copy: DRY_RUN=1 ./lidar_camera_calib/scripts/deploy_to_pi.sh
# Cài apt trên Pi (hỏi sudo):  INSTALL_DEPS=1 ./lidar_camera_calib/scripts/deploy_to_pi.sh
# Build trên Pi sau khi copy:  BUILD=1 ./lidar_camera_calib/scripts/deploy_to_pi.sh
# Driver LD06 ở chỗ khác:      LDLIDAR_SRC=/path/to/ldlidar_stl_ros2 ...
# ROS khác Humble:             ROS_DISTRO_PI=jazzy ...
#
# Không copy: samples*/, *.npz, build/, install/ (dữ liệu thu mẫu nặng và không cần trên Pi).

set -euo pipefail

PI_HOST="${PI_HOST:-piros2@192.168.50.1}"
PI_HOME="${PI_HOME:-~}"
DRY_RUN="${DRY_RUN:-0}"
INSTALL_DEPS="${INSTALL_DEPS:-0}"
BUILD="${BUILD:-0}"
ROS_DISTRO_PI="${ROS_DISTRO_PI:-humble}"
PI_WS="${PI_HOME}/lidar_cam"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LDLIDAR_SRC="${LDLIDAR_SRC:-${HOME}/lidar_ws/src/ldlidar_stl_ros2}"

# Cặp (cùng chỉ số): file trong PKG_DIR -> đường dẫn tương đối trong ${PI_WS}/src/lidar_camera_calib
PKG_FILES=(
    "package.xml"
    "setup.py"
    "setup.cfg"
    "resource/lidar_camera_calib"
    "config/camera.yaml"
    "extrinsic.yaml"
    "README.md"
    "docs/USAGE.md"
    "lidar_camera_calib/__init__.py"
    "lidar_camera_calib/capture.py"
    "lidar_camera_calib/capture_guide.py"
    "lidar_camera_calib/check_range.py"
    "lidar_camera_calib/evaluate.py"
    "lidar_camera_calib/geometry.py"
    "lidar_camera_calib/overlay.py"
    "lidar_camera_calib/solve.py"
    "lidar_camera_calib/sysstats.py"
    "lidar_camera_calib/validate.py"
)

CTRL_SOCK="${TMPDIR:-/tmp}/deploy-lidar-pi-%C"
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=${CTRL_SOCK}" -o ControlPersist=300)
if [ "${DRY_RUN}" != "1" ]; then
    echo "Kết nối ${PI_HOST} (nhập mật khẩu 1 lần)..."
    ssh "${SSH_OPTS[@]}" -fN "${PI_HOST}"
    trap 'ssh "${SSH_OPTS[@]}" -O exit "${PI_HOST}" 2>/dev/null || true' EXIT
fi

run_remote() {  # run_remote [-t] "lệnh"
    local tty=()
    [ "$1" = "-t" ] && { tty=(-t); shift; }
    if [ "${DRY_RUN}" = "1" ]; then echo "  ssh ${PI_HOST} \"$1\""; else ssh "${tty[@]}" "${SSH_OPTS[@]}" "${PI_HOST}" "$1"; fi
}

echo "== Deploy lidar_camera_calib lên Pi =="
echo "Nguồn gói   : ${PKG_DIR}"
echo "Nguồn driver: ${LDLIDAR_SRC}"
echo "Đích        : ${PI_HOST}:${PI_WS}/src"
[ "${DRY_RUN}" = "1" ] && echo "(DRY RUN — chỉ in ra, không copy thật)"
echo

echo "-- Tạo thư mục đích --"
DIRS=("${PI_WS}/src/lidar_camera_calib" "${PI_WS}/src/ldlidar_stl_ros2")
for f in "${PKG_FILES[@]}"; do DIRS+=("${PI_WS}/src/lidar_camera_calib/$(dirname "$f")"); done
run_remote "mkdir -p $(printf '%s\n' "${DIRS[@]}" | sort -u | tr '\n' ' ')"

echo
echo "-- Copy gói lidar_camera_calib --"
MISSING=()
for f in "${PKG_FILES[@]}"; do
    src="${PKG_DIR}/${f}"
    if [ ! -f "${src}" ]; then
        echo "  [BỎ QUA] không thấy ${src}"; MISSING+=("${f}"); continue
    fi
    if [ "${DRY_RUN}" = "1" ]; then
        echo "  scp ${src} ${PI_HOST}:${PI_WS}/src/lidar_camera_calib/${f}"
    else
        echo "  ${f}"
        scp -q "${SSH_OPTS[@]}" "${src}" "${PI_HOST}:${PI_WS}/src/lidar_camera_calib/${f}"
    fi
done

echo
echo "-- Copy driver ldlidar_stl_ros2 (tar qua ssh, bỏ .git) --"
if [ ! -d "${LDLIDAR_SRC}" ]; then
    echo "  [BỎ QUA] không thấy ${LDLIDAR_SRC}"; MISSING+=("ldlidar_stl_ros2")
elif [ "${DRY_RUN}" = "1" ]; then
    echo "  tar -C ${LDLIDAR_SRC} --exclude=.git -cf - . | ssh ${PI_HOST} 'tar -C ${PI_WS}/src/ldlidar_stl_ros2 -xf -'"
else
    tar -C "${LDLIDAR_SRC}" --exclude=.git -cf - . | ssh "${SSH_OPTS[@]}" "${PI_HOST}" "tar -C ${PI_WS}/src/ldlidar_stl_ros2 -xf -"
fi

if [ "${INSTALL_DEPS}" = "1" ]; then
    echo
    echo "-- Cài gói apt trên Pi (hỏi mật khẩu sudo) --"
    run_remote -t "sudo apt update && sudo apt install -y ros-${ROS_DISTRO_PI}-v4l2-camera ros-${ROS_DISTRO_PI}-cv-bridge ros-${ROS_DISTRO_PI}-image-transport ros-${ROS_DISTRO_PI}-web-video-server python3-opencv python3-numpy python3-scipy python3-yaml v4l-utils python3-colcon-common-extensions && sudo usermod -aG dialout,video \$USER"
fi

if [ "${BUILD}" = "1" ]; then
    echo
    echo "-- Build trên Pi --"
    run_remote "cd ${PI_WS} && source /opt/ros/${ROS_DISTRO_PI}/setup.bash && colcon build --packages-select ldlidar_stl_ros2 lidar_camera_calib"
fi

echo
if [ "${#MISSING[@]}" -gt 0 ]; then
    echo "XONG NHƯNG THIẾU ${#MISSING[@]} mục (xem log ở trên): ${MISSING[*]}"
    exit 1
fi
echo "XONG — đã đưa gói + driver lên ${PI_HOST}:${PI_WS}/src"
echo "Chạy trên Pi:  source /opt/ros/${ROS_DISTRO_PI}/setup.bash && source ${PI_WS}/install/setup.bash"
