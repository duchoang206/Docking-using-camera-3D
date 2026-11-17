#!/usr/bin/env bash
# =============================================================================
# install_ros2_jazzy.sh
# Cài ROS 2 Jazzy + RViz2 + công cụ cho dự án "Docking bằng Camera 3D"
# Hệ điều hành: Ubuntu 24.04 (Noble) — amd64 (PC) hoặc arm64 (Raspberry Pi/Jetson)
#
# Cách chạy:
#   bash tools/install_ros2_jazzy.sh                       # máy DEV: đầy đủ GUI
#   bash tools/install_ros2_jazzy.sh --robot               # máy ROBOT: tối giản, không GUI
#   bash tools/install_ros2_jazzy.sh --camera realsense    # thêm driver camera
#        (--camera: realsense | orbbec | oakd | none)
#
# Chạy lại nhiều lần vẫn an toàn (idempotent). Không nâng cấp toàn bộ hệ thống.
# =============================================================================
set -euo pipefail

MODE="dev"
CAMERA="none"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)  MODE="robot"; shift ;;
    --camera) CAMERA="${2:-none}"; shift 2 ;;
    *) echo "Tham số không hợp lệ: $1"; exit 2 ;;
  esac
done

ROS_DISTRO="jazzy"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_DIR}/install_ros2_${ROS_DISTRO}_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG_FILE") 2>&1

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[!] %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m[x] %s\033[0m\n' "$*"; exit 1; }

# --- 0) Kiểm tra hệ điều hành ------------------------------------------------
. /etc/os-release
[[ "${VERSION_CODENAME:-}" == "noble" ]] \
  || die "Script dành cho Ubuntu 24.04 (noble). Máy này: ${PRETTY_NAME:-unknown}"
ARCH="$(dpkg --print-architecture)"
log "Ubuntu ${VERSION_ID} (${VERSION_CODENAME}) / ${ARCH} — chế độ: ${MODE}, camera: ${CAMERA}"
log "Log: ${LOG_FILE}"

# --- 1) Quyền sudo: hỏi mật khẩu 1 lần, giữ phiên sudo trong lúc cài ----------
log "Cần quyền sudo — hãy nhập MẬT KHẨU của bạn vào terminal này"
sudo -v
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
KEEPALIVE_PID=$!
trap 'kill "$KEEPALIVE_PID" 2>/dev/null || true' EXIT

# --- 2) Locale UTF-8 (ROS 2 yêu cầu) -----------------------------------------
if ! locale 2>/dev/null | grep -qi 'utf-8'; then
  warn "Locale chưa là UTF-8 → đặt en_US.UTF-8"
  sudo apt-get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi

# --- 3) Kho apt: universe + kho chính thức ROS 2 (gói ros2-apt-source) -------
log "Cấu hình kho apt"
sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates
if ! grep -qs "universe" /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list; then
  sudo add-apt-repository -y universe
fi
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [[ -n "${ROS_APT_SOURCE_VERSION}" ]] || die "Không lấy được phiên bản ros-apt-source (kiểm tra mạng)"
  TMP_DIR="$(mktemp -d)"
  curl -fL -o "${TMP_DIR}/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i "${TMP_DIR}/ros2-apt-source.deb"
  rm -rf "${TMP_DIR}"
fi
sudo apt-get update

# --- 4) Danh sách gói ----------------------------------------------------------
COMMON_PKGS=(
  ros-dev-tools                                  # colcon, rosdep, vcstool
  "ros-${ROS_DISTRO}-vision-msgs"
  "ros-${ROS_DISTRO}-depth-image-proc"           # depth image -> point cloud
  "ros-${ROS_DISTRO}-depthimage-to-laserscan"    # depth -> "laser scan" (kỹ thuật bài ICARCV'18)
  "ros-${ROS_DISTRO}-pointcloud-to-laserscan"
  "ros-${ROS_DISTRO}-image-transport-plugins"
  "ros-${ROS_DISTRO}-navigation2"
  "ros-${ROS_DISTRO}-opennav-docking"            # Nav2 Docking server
  "ros-${ROS_DISTRO}-apriltag-ros"               # ground-truth khi kiểm thử độ chính xác
  "ros-${ROS_DISTRO}-teleop-twist-keyboard"
  "ros-${ROS_DISTRO}-rosbag2-storage-mcap"
  python3-numpy python3-scipy python3-opencv python3-yaml python3-pytest
)
DEV_PKGS=(
  "ros-${ROS_DISTRO}-desktop"                    # ROS 2 + RViz2 + rqt
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-ros-gz"                     # Gazebo Harmonic + cầu nối ROS (mô phỏng)
  "ros-${ROS_DISTRO}-plotjuggler-ros"            # vẽ đồ thị để tinh chỉnh bộ điều khiển
)
ROBOT_PKGS=(
  "ros-${ROS_DISTRO}-ros-base"                   # không GUI — nhẹ cho máy nhúng
)
case "$CAMERA" in
  realsense) CAM_PKGS=("ros-${ROS_DISTRO}-realsense2-camera") ;;
  orbbec)    CAM_PKGS=("ros-${ROS_DISTRO}-orbbec-camera") ;;
  oakd)      CAM_PKGS=("ros-${ROS_DISTRO}-depthai-ros") ;;
  none)      CAM_PKGS=() ;;
  *) die "--camera không hợp lệ: ${CAMERA} (realsense | orbbec | oakd | none)" ;;
esac

if [[ "$MODE" == "dev" ]]; then
  PKGS=("${DEV_PKGS[@]}" "${COMMON_PKGS[@]}" "${CAM_PKGS[@]}")
else
  PKGS=("${ROBOT_PKGS[@]}" "${COMMON_PKGS[@]}" "${CAM_PKGS[@]}")
fi

log "Cài ${#PKGS[@]} gói (có thể mất 10–30 phút tuỳ tốc độ mạng)…"
printf '   %s\n' "${PKGS[@]}"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${PKGS[@]}"

# --- 5) CloudCompare: xem/cắt point cloud, tạo template (chỉ máy dev) --------
if [[ "$MODE" == "dev" ]] && command -v snap >/dev/null 2>&1; then
  if ! snap list cloudcompare >/dev/null 2>&1; then
    log "Cài CloudCompare (snap)"
    sudo snap install cloudcompare || warn "Không cài được CloudCompare — bỏ qua"
  fi
fi

# --- 6) rosdep -----------------------------------------------------------------
log "Khởi tạo rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro "$ROS_DISTRO" || warn "rosdep update lỗi — chạy lại sau: rosdep update"

# --- 7) Tự nạp môi trường ROS cho mọi terminal mới ----------------------------
LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -qxF "$LINE" "${HOME}/.bashrc"; then
  printf '\n# ROS 2 %s (thêm bởi install_ros2_jazzy.sh)\n%s\n' "$ROS_DISTRO" "$LINE" >> "${HOME}/.bashrc"
  log "Đã thêm '${LINE}' vào ~/.bashrc"
fi

# --- 8) Kiểm tra nhanh ----------------------------------------------------------
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
log "Kiểm tra cài đặt"
echo "ROS_DISTRO=${ROS_DISTRO}  (gói ROS đã cài: $(ros2 pkg list 2>/dev/null | wc -l))"
for b in ros2 colcon rosdep rviz2 rqt gz; do
  if command -v "$b" >/dev/null 2>&1; then echo "  OK  $b"; else echo "  --  $b"; fi
done
if ros2 pkg prefix opennav_docking >/dev/null 2>&1; then echo "  OK  opennav_docking"; else echo "  --  opennav_docking"; fi

log "XONG! Mở terminal MỚI (hoặc chạy: source ~/.bashrc) để dùng lệnh ros2."
