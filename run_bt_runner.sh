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

# 本脚本最终 ros2 run 的目标: 包名 + 可执行文件名。校验时会断言该 exe 存在且非空。
RUN_PKG="at_r2_bt"
RUN_EXE="simple_bt_runner"

# 完整性校验: 拦截"编译没完成/被强杀"留下的坏产物, 绝不带病启动。
# 1) 扫描相关包的 build/install 里被截断成 0 字节的 .o/.so/可执行文件——这类空文件会让
#    增量编译误判为"已最新"而跳过重编, 最终链接失败或运行时报 "Exec format error"。
# 2) 断言本脚本要 ros2 run 的那个可执行文件确实存在且非空 (RUN_PKG/RUN_EXE)。
# 发现问题即报错并给出修复命令。
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

  # 0 字节的目标文件 / 共享库 / 可执行文件都是损坏产物
  local bad=""
  if [ ${#existing[@]} -gt 0 ]; then
    bad="$(find "${existing[@]}" -type f -size 0 \
      \( -name '*.o' -o -name '*.so' -o -name '*.so.*' \
         -o -path "*/lib/${RUN_PKG}/${RUN_EXE}" -o -name "${RUN_EXE}" \) 2>/dev/null)"
  fi

  # 目标可执行文件必须存在且非空 (直接拦住 0 字节 / 未安装的情况)
  local exe="${WORKSPACE_DIR}/install/${RUN_PKG}/lib/${RUN_PKG}/${RUN_EXE}"
  local missing=0
  if [ ! -s "${exe}" ]; then
    missing=1
  fi

  if [ -n "${bad}" ] || [ "${missing}" -eq 1 ]; then
    echo "[ERROR] 编译产物校验未通过, 拒绝启动:" >&2
    if [ -n "${bad}" ]; then
      echo "  检测到 0 字节的损坏产物(通常是上次编译被中途强杀留下的):" >&2
      echo "${bad}" | sed 's/^/    - /' >&2
    fi
    if [ "${missing}" -eq 1 ]; then
      echo "  目标可执行文件缺失或为空: ${exe}" >&2
    fi
    echo "" >&2
    echo "  修复(全量重编 ${RUN_PKG}):" >&2
    echo "    rm -rf '${WORKSPACE_DIR}/build/${RUN_PKG}' '${WORKSPACE_DIR}/install/${RUN_PKG}'" >&2
    echo "    ./$(basename "$0") -p \"${RUN_PKG}\"" >&2
    return 1
  fi
  return 0
}

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
if ! verify_build_artifacts "at_r2_bt"; then
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
