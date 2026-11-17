#!/usr/bin/env bash
# Chạy test end-to-end docking trong Gazebo (headless). Log: <project>/logs/e2e_case*.log
#   bash ros2_ws/src/camera3d_docking_sim/scripts/run_e2e.sh
#   bash ros2_ws/src/camera3d_docking_sim/scripts/run_e2e.sh --starts "1.6,0,0" "1.5,0.25,15"
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "${HERE}/../../../.." && pwd)"
# shellcheck disable=SC1091
source "${PROJECT}/env.sh"
cd "${PROJECT}"
exec ros2 run camera3d_docking_sim e2e_docking_test --log-dir "${PROJECT}/logs" "$@"
