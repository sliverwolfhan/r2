#!/usr/bin/env bash
# 启动 plan_yaml_publisher_node
# 用法:
#   ./run_plan_publisher.sh           # 直接启动，不编译
#   ./run_plan_publisher.sh -b        # 先 colcon build 整个工作空间再启动
#   ./run_plan_publisher.sh --build   # 同上
#   ./run_plan_publisher.sh -p "r2_meilin_planner"  # 仅编译指定包再启动
#   ./run_plan_publisher.sh -h        # 帮助
#
# 透传参数到 ros2 run:
#   ./run_plan_publisher.sh -- --ros-args -p plan_file:=/path/to/plan.yaml
#   ./run_plan_publisher.sh -b -- --ros-args -p plan_file:=/path/to/plan.yaml

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

echo "==> ros2 run r2_meilin_planner plan_yaml_publisher_node ${RUN_ARGS[*]}"
exec ros2 run r2_meilin_planner plan_yaml_publisher_node "${RUN_ARGS[@]}"
