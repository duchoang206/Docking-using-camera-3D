#!/usr/bin/env bash
# =============================================================================
# pin_apps_to_dock.sh — Tạo launcher cho RViz2, rqt, Gazebo, PlotJuggler
# (+ CloudCompare, + "Docking Sim" nếu dùng --with-sim) và ghim lên thanh
# taskbar (Ubuntu Dock / GNOME favorites). Không cần sudo. Chạy lại an toàn.
#
#   bash tools/pin_apps_to_dock.sh             # các ứng dụng ROS 2 + CloudCompare
#   bash tools/pin_apps_to_dock.sh --with-sim  # thêm nút "Docking Sim"
# =============================================================================
set -euo pipefail

WITH_SIM=0
[[ "${1:-}" == "--with-sim" ]] && WITH_SIM=1

ROS_SETUP="/opt/ros/jazzy/setup.bash"
APP_DIR="${HOME}/.local/share/applications"
ICON_DIR="${HOME}/.local/share/icons/docking3d"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$APP_DIR" "$ICON_DIR"

[[ -f "$ROS_SETUP" ]] || { echo "Chưa cài ROS 2 Jazzy ($ROS_SETUP)"; exit 1; }

# --- Icon SVG tự vẽ (Gazebo/RViz không kèm icon ứng dụng) ---------------------
make_icon() {  # make_icon <file> <màu nền> <chữ>
  cat > "$1" <<EOF
<svg xmlns="http://www.w3.org/2000/svg" width="128" height="128" viewBox="0 0 128 128">
  <rect x="8" y="8" width="112" height="112" rx="24" fill="$2"/>
  <text x="64" y="80" font-family="DejaVu Sans, sans-serif" font-size="44"
        font-weight="bold" fill="#ffffff" text-anchor="middle">$3</text>
</svg>
EOF
}
make_icon "${ICON_DIR}/rviz2.svg"  "#2b5797" "RViz"
make_icon "${ICON_DIR}/gazebo.svg" "#f58113" "Gz"
cat > "${ICON_DIR}/docking-sim.svg" <<'EOF'
<svg xmlns="http://www.w3.org/2000/svg" width="128" height="128" viewBox="0 0 128 128">
  <rect x="8" y="8" width="112" height="112" rx="24" fill="#1e7a4f"/>
  <rect x="62" y="40" width="44" height="10" fill="#d9b382"/>
  <rect x="62" y="62" width="44" height="10" fill="#d9b382"/>
  <rect x="62" y="40" width="8" height="32" fill="#a77b4f"/>
  <rect x="98" y="40" width="8" height="32" fill="#a77b4f"/>
  <rect x="18" y="48" width="32" height="24" rx="5" fill="#ffffff"/>
  <circle cx="26" cy="76" r="5" fill="#ffffff"/><circle cx="42" cy="76" r="5" fill="#ffffff"/>
  <path d="M52 60 L60 56 L60 64 Z" fill="#ffffff"/>
  <text x="64" y="104" font-family="DejaVu Sans, sans-serif" font-size="18"
        font-weight="bold" fill="#ffffff" text-anchor="middle">DOCK</text>
</svg>
EOF

# --- Tạo file .desktop ----------------------------------------------------------
make_desktop() {  # make_desktop <id> <Tên> <Mô tả> <lệnh> <icon> [WMClass]
  local file="${APP_DIR}/$1.desktop"
  cat > "$file" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$2
Comment=$3
Exec=bash -c "source ${ROS_SETUP} && export PYTHONNOUSERSITE=1 && exec $4"
Icon=$5
Terminal=false
Categories=Development;
StartupNotify=true
EOF
  [[ -n "${6:-}" ]] && echo "StartupWMClass=$6" >> "$file"
  chmod +x "$file"
  desktop-file-validate "$file" || echo "[!] $file có cảnh báo (vẫn dùng được)"
  echo "  + $1.desktop"
}

echo "==> Tạo launcher trong ${APP_DIR}"
make_desktop ros2-rviz2       "RViz2"        "ROS 2 3D visualizer"            "rviz2"           "${ICON_DIR}/rviz2.svg"  "rviz2"
make_desktop ros2-rqt         "rqt"          "ROS 2 GUI tools"                "rqt"             "/opt/ros/jazzy/share/rqt_gui/resource/rqt.svg"
make_desktop ros2-gazebo      "Gazebo Sim"   "Gazebo Harmonic simulator"      "gz sim"          "${ICON_DIR}/gazebo.svg"
make_desktop ros2-plotjuggler "PlotJuggler"  "Plot ROS 2 topics"              "ros2 run plotjuggler plotjuggler" "/opt/ros/jazzy/share/icons/plotjuggler.svg" "PlotJuggler"

FAVS_NEW=(ros2-rviz2.desktop ros2-rqt.desktop ros2-gazebo.desktop ros2-plotjuggler.desktop)
if [[ -f /var/lib/snapd/desktop/applications/cloudcompare_cloudcompare.desktop ]]; then
  FAVS_NEW+=(cloudcompare_cloudcompare.desktop)
fi

if [[ "$WITH_SIM" == 1 ]]; then
  WS_SETUP="${PROJECT_DIR}/ros2_ws/install/setup.bash"
  file="${APP_DIR}/docking3d-sim.desktop"
  cat > "$file" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=Docking Sim
Comment=Mô phỏng docking bằng camera 3D (Gazebo + RViz)
Exec=gnome-terminal --title="Docking Sim" -- bash -c "source ${ROS_SETUP} && source '${WS_SETUP}' && export PYTHONNOUSERSITE=1 && ros2 launch camera3d_docking_sim sim.launch.py rviz:=true; exec bash"
Icon=${ICON_DIR}/docking-sim.svg
Terminal=false
Categories=Development;
EOF
  chmod +x "$file"
  desktop-file-validate "$file" || echo "[!] $file có cảnh báo (vẫn dùng được)"
  echo "  + docking3d-sim.desktop"
  FAVS_NEW+=(docking3d-sim.desktop)
fi

command -v update-desktop-database >/dev/null && update-desktop-database "$APP_DIR" 2>/dev/null || true

# --- Ghim vào favorites (giữ nguyên thứ tự cũ, bỏ trùng) -----------------------
echo "==> Ghim lên taskbar"
CURRENT="$(gsettings get org.gnome.shell favorite-apps)"
NEW_LIST="$(python3 - "$CURRENT" "${FAVS_NEW[@]}" <<'PY'
import ast, sys
cur = sys.argv[1].strip()
if cur.startswith("@as"):
    cur = cur[3:].strip()
favs = list(ast.literal_eval(cur)) if cur else []
for app in sys.argv[2:]:
    if app not in favs:
        favs.append(app)
print(str(favs))
PY
)"
gsettings set org.gnome.shell favorite-apps "$NEW_LIST"
echo "Favorites hiện tại: $(gsettings get org.gnome.shell favorite-apps)"
