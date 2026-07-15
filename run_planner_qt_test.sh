#!/usr/bin/env bash
# 启动 r2_planner_qt_test
# 用法:
#   ./run_planner_qt_test.sh           # 直接启动，不编译
#   ./run_planner_qt_test.sh -b        # 先 colcon build 整个工作空间再启动
#   ./run_planner_qt_test.sh --build   # 同上
#   ./run_planner_qt_test.sh -p "r2_meilin_planner"  # 仅编译指定包再启动
#   ./run_planner_qt_test.sh -h        # 帮助
#
# 透传参数到 ros2 run:
#   ./run_planner_qt_test.sh -- --ros-args -p some_param:=value
#   ./run_planner_qt_test.sh -b -- --ros-args -p some_param:=value

set -e

usage() {
  cat <<EOF
Usage: $(basename "$0") [-b|--build] [-p|--packages "pkg1 pkg2"] [-h|--help] [-- <ros2 run args>]

  -b, --build       启动前先 colcon build (默认 build 整个工作空间)
  -p, --packages    仅 build 指定的包，例如: -p "r2_meilin_planner"
  -h, --help        显示帮助
  --                后续参数原样传给 ros2 run
EOF
}

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

DO_BUILD=0
PACKAGES=""
RUN_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -b|--build)
      DO_BUILD=1
      shift
      ;;
    -p|--packages)
      PACKAGES="$2"
      DO_BUILD=1
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      RUN_ARGS=("$@")
      break
      ;;
    *)
      echo "[WARN] 未知参数: $1, 透传给 ros2 run"
      RUN_ARGS+=("$1")
      shift
      ;;
  esac
done

# Source ROS2
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[ERROR] 未找到 /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi

# 编译
if [ "${DO_BUILD}" -eq 1 ]; then
  echo "==> colcon build (workspace: ${WORKSPACE_DIR})"
  cd "${WORKSPACE_DIR}"
  if [ -n "${PACKAGES}" ]; then
    echo "==> 仅编译: ${PACKAGES}"
    colcon build --packages-select ${PACKAGES}
  else
    colcon build
  fi
fi

# Source 工作空间
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  echo "[ERROR] 未找到 ${WORKSPACE_DIR}/install/setup.bash, 请先 colcon build"
  exit 1
fi

# 从 zone.conf 读红/蓝区，透传给 qt_test（否则监视窗口默认红区，会把蓝区真车的
# plan 画到红区方块外——路线飞出场地、格子着色左右镜像错位）。默认 blue，与 start_planner.sh 一致。
# 注入 --ros-args -p zone:=<zone>；用户在 -- 后自带的 -p zone:= 排在其后，可覆盖本值。
ZONE_FILE="${WORKSPACE_DIR}/autostart/zone.conf"
ZONE="blue"
if [ -f "${ZONE_FILE}" ]; then
  ZONE="$(grep -vE '^\s*#' "${ZONE_FILE}" | tr -d '[:space:]' | head -c 16)"
  ZONE="${ZONE:-blue}"
fi
echo "==> zone=${ZONE} (来自 ${ZONE_FILE})"

echo "==> ros2 run r2_meilin_planner r2_planner_qt_test --ros-args -p zone:=${ZONE} ${RUN_ARGS[*]}"
exec ros2 run r2_meilin_planner r2_planner_qt_test --ros-args -p "zone:=${ZONE}" "${RUN_ARGS[@]}"
