#!/usr/bin/env bash
# 启动 simple_bt_runner
# 用法:
#   ./run_bt_runner.sh                    # 直接启动，默认行为树
#   ./run_bt_runner.sh -b                 # 先 colcon build 整个工作空间再启动
#   ./run_bt_runner.sh --build            # 同上
#   ./run_bt_runner.sh -p "at_r2_bt"      # 仅编译指定包再启动
#   ./run_bt_runner.sh grasp_head.xml     # 指定行为树 xml
#   ./run_bt_runner.sh -b grasp_head.xml  # 编译后用指定行为树启动
#   ./run_bt_runner.sh -h                 # 帮助
#
# 透传参数到 ros2 run:
#   ./run_bt_runner.sh -- --ros-args -p tf_namespace:=AT_R2

set -e

usage() {
  cat <<EOF
Usage: $(basename "$0") [-b|--build] [-p|--packages "pkg1 pkg2"] [-h|--help] [bt_xml] [-- <ros2 run args>]

  -b, --build       启动前先 colcon build (默认 build 整个工作空间)
  -p, --packages    仅 build 指定的包，例如: -p "at_r2_bt"
  -h, --help        显示帮助
  bt_xml            行为树 xml 文件名 (位于 at_r2_bt/behavior_trees/)
  --                后续参数原样传给 ros2 run
EOF
}

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

DO_BUILD=0
PACKAGES=""
BT_XML=""
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
    *.xml)
      BT_XML="$1"
      shift
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

CMD=(ros2 run at_r2_bt simple_bt_runner)
if [ -n "${BT_XML}" ]; then
  CMD+=("${BT_XML}")
fi
if [ ${#RUN_ARGS[@]} -gt 0 ]; then
  CMD+=("${RUN_ARGS[@]}")
fi

echo "==> ${CMD[*]}"
exec "${CMD[@]}"
