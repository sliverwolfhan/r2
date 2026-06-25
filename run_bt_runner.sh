#!/usr/bin/env bash
# 启动 simple_bt_runner (通过 ros2 launch at_r2_bt r2_bt_launch.py)
# launch 内部已加载 weapon_grasp_params.yaml, 参数(grasp_count 等)自动生效。
# 用法:
#   ./run_bt_runner.sh                    # 直接启动，默认行为树
#   ./run_bt_runner.sh -b                 # 先 colcon build 整个工作空间再启动
#   ./run_bt_runner.sh --build            # 同上
#   ./run_bt_runner.sh -p "at_r2_bt"      # 仅编译指定包再启动
#   ./run_bt_runner.sh grasp_head_2.xml   # 指定行为树 xml
#   ./run_bt_runner.sh -b grasp_head_2.xml # 编译后用指定行为树启动
#   ./run_bt_runner.sh -h                 # 帮助
#
# 透传参数到 ros2 launch:
#   ./run_bt_runner.sh -- grasp_count:=2

set -e

usage() {
  cat <<EOF
Usage: $(basename "$0") [-b|--build] [-p|--packages "pkg1 pkg2"] [-h|--help] [bt_xml] [-- <ros2 launch args>]

  -b, --build       启动前先 colcon build (默认 build 整个工作空间)
  -p, --packages    仅 build 指定的包，例如: -p "at_r2_bt"
  -h, --help        显示帮助
  bt_xml            行为树 xml 文件名 (位于 at_r2_bt/behavior_trees/)
  --                后续参数原样传给 ros2 launch
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
      echo "[WARN] 未知参数: $1, 透传给 ros2 launch"
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

# 通过 launch 启动: r2_bt_launch.py 内部已加载 weapon_grasp_params.yaml,
# 所以 grasp_count / grasp_start / weapon_x.grasp_prep_* 等参数自动生效。
# 行为树 xml 通过 launch 参数 bt_xml 传入 (默认 grasp_head.xml)。
CMD=(ros2 launch at_r2_bt r2_bt_launch.py)
if [ -n "${BT_XML}" ]; then
  CMD+=("bt_xml:=${BT_XML}")
fi
if [ ${#RUN_ARGS[@]} -gt 0 ]; then
  CMD+=("${RUN_ARGS[@]}")
fi

echo "==> ${CMD[*]}"
exec "${CMD[@]}"
