#!/usr/bin/env bash
# 【红区】启动 r2_meilin_planner (通过 ros2 launch r2_meilin_planner r2_planner_bringup_launch_red.py)
# launch 内部 zone 默认 red, 加载 planner_params.yaml, 发布 /r2_planner/plan。
# 用法:
#   ./run_planner_red.sh                    # 直接启动 (zone=red)
#   ./run_planner_red.sh -b                 # 先 colcon build 整个工作空间再启动
#   ./run_planner_red.sh --build            # 同上
#   ./run_planner_red.sh -p "r2_meilin_planner"  # 仅编译指定包再启动
#   ./run_planner_red.sh -h                 # 帮助
#
# 透传参数到 ros2 launch (覆盖 ignore_height / can_climb_400 / params_file 等):
#   ./run_planner_red.sh -- can_climb_400:=false

set -e

LAUNCH_FILE="r2_planner_bringup_launch_red.py"

usage() {
  cat <<EOF
Usage: $(basename "$0") [-b|--build] [-p|--packages "pkg1 pkg2"] [-h|--help] [-- <ros2 launch args>]

  【红区】固定使用 ${LAUNCH_FILE} (zone=red)
  -b, --build       启动前先 colcon build (默认 build 整个工作空间)
  -p, --packages    仅 build 指定的包，例如: -p "r2_meilin_planner"
  -h, --help        显示帮助
  --                后续参数原样传给 ros2 launch
EOF
}

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# 本脚本通过 ros2 launch 启动, 无单一可执行文件。校验主要拦截 0 字节的坏产物。
RUN_PKG="r2_meilin_planner"

# 完整性校验: 扫描相关包的 build/install 里被截断成 0 字节的 .o/.so/可执行文件。
# 这类空文件通常由编译中途被强杀(Ctrl-C)产生, 会让增量编译误判为"已最新"而跳过重编,
# 最终链接失败或运行时报 "Exec format error"。发现即报错并给出修复命令, 绝不带病启动。
#   参数: 要检查的包名列表(空格分隔); 为空则检查整个 build/ 与 install/ 树。
verify_build_artifacts() {
  local pkgs="$1"
  local -a dirs=()
  if [ -n "${pkgs}" ]; then
    local p
    for p in ${pkgs}; do
      dirs+=("${WORKSPACE_DIR}/build/${p}" "${WORKSPACE_DIR}/install/${p}")
    done
  else
    dirs+=("${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install")
  fi

  local -a existing=()
  local d
  for d in "${dirs[@]}"; do
    [ -d "${d}" ] && existing+=("${d}")
  done
  [ ${#existing[@]} -eq 0 ] && return 0

  local bad
  bad="$(find "${existing[@]}" -type f -size 0 \
    \( -name '*.o' -o -name '*.so' -o -name '*.so.*' \) 2>/dev/null)"

  if [ -n "${bad}" ]; then
    echo "[ERROR] 检测到 0 字节的损坏编译产物(通常是上次编译被中途强杀留下的):" >&2
    echo "${bad}" | sed 's/^/  - /' >&2
    echo "" >&2
    echo "  这些空文件会让增量编译误判为最新而跳过重编, 导致启动失败。" >&2
    echo "  修复(全量重编受影响的包):" >&2
    if [ -n "${pkgs}" ]; then
      for p in ${pkgs}; do
        echo "    rm -rf '${WORKSPACE_DIR}/build/${p}' '${WORKSPACE_DIR}/install/${p}'" >&2
      done
      echo "    ./$(basename "$0") -p \"${pkgs}\"" >&2
    else
      echo "    rm -rf '${WORKSPACE_DIR}/build/${RUN_PKG}' '${WORKSPACE_DIR}/install/${RUN_PKG}'" >&2
      echo "    ./$(basename "$0") -b" >&2
    fi
    return 1
  fi
  return 0
}

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

  # 编译后立刻校验: 拦截本次编译产生/残留的截断文件, 不让它流到运行时
  if ! verify_build_artifacts "${PACKAGES}"; then
    exit 1
  fi
fi

# Source 工作空间
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  echo "[ERROR] 未找到 ${WORKSPACE_DIR}/install/setup.bash, 请先 colcon build"
  exit 1
fi

# 启动前再校验一次: 即使本次未加 -b, 也要拦截历史遗留的 0 字节坏产物,
# 避免独立终端窗口把 "Exec format error" 一闪而过地吞掉。
if ! verify_build_artifacts "r2_meilin_planner"; then
  exit 1
fi

CMD=(ros2 launch r2_meilin_planner "${LAUNCH_FILE}")
if [ ${#RUN_ARGS[@]} -gt 0 ]; then
  CMD+=("${RUN_ARGS[@]}")
fi

echo "==> [红区] ${CMD[*]}"
exec "${CMD[@]}"
