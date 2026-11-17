# Nạp môi trường cho dự án "Docking bằng Camera 3D".
# Dùng:  source env.sh      (từ bất kỳ thư mục nào)
_D3_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -f "${_D3_DIR}/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_D3_DIR}/ros2_ws/install/setup.bash"
fi
# Dùng numpy/scipy của apt, bỏ qua ~/.local (numpy 2.x xung đột với ROS 2 Jazzy)
export PYTHONNOUSERSITE=1
echo "[docking3d] ROS 2 ${ROS_DISTRO} + workspace đã nạp (PYTHONNOUSERSITE=1)"
