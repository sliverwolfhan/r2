#!/usr/bin/env bash
# 一键中断开机自启动 (虚拟串口 + 导航 + 机械臂 + 规划器 + 行为树)。
# 用法:
#   ./stop_all.sh                    # 停掉全部 (本次停, 下次登录仍会自启)
#   ./stop_all.sh serial nav         # 只停指定程序 (可写多个, 空格分隔)
#   ./stop_all.sh --disable          # 停掉并禁用自启 (以后开机/登录不再启动)
#   ./stop_all.sh --disable arm      # 只停并禁用机械臂
#   ./stop_all.sh --status           # 停完后顺带打印状态
#   ./stop_all.sh --list             # 列出可选的程序名后退出
#
# 可选程序名 (别名):
#   serial   -> atrc-virtual-serial.service   虚拟串口
#   nav      -> atrc-navigation.service       导航
#   arm      -> atrc-arm.service              机械臂
#   planner  -> atrc-planner.service          规划器
#   qt       -> atrc-planner-qt-test.service  规划器Qt测试
#   bridge   -> atrc-serial-bridge-test.service 串口桥接test
#   bt       -> atrc-bt-runner.service        行为树
#   all      -> 全部 (等价于不带程序名)

set -u

# 别名 -> 服务单元 映射
declare -A ALIASES=(
  [serial]=atrc-virtual-serial.service
  [nav]=atrc-navigation.service
  [arm]=atrc-arm.service
  [planner]=atrc-planner.service
  [qt]=atrc-planner-qt-test.service
  [bridge]=atrc-serial-bridge-test.service
  [bt]=atrc-bt-runner.service
)
# 别名 -> 中文说明 (仅用于 --list 展示)
declare -A DESCS=(
  [serial]="虚拟串口"
  [nav]="导航"
  [arm]="机械臂"
  [planner]="规划器"
  [qt]="规划器Qt测试"
  [bridge]="串口桥接test"
  [bt]="行为树"
)

ALL_UNITS=(atrc-virtual-serial.service atrc-navigation.service atrc-arm.service atrc-planner.service atrc-planner-qt-test.service atrc-serial-bridge-test.service atrc-bt-runner.service)
DISABLE=0
SHOW_STATUS=0
SELECTED=()   # 用户指定的服务单元; 为空则表示全部

list_programs() {
  echo "可选程序名:"
  for a in serial nav arm planner qt bridge bt; do
    printf "  %-8s -> %-28s %s\n" "$a" "${ALIASES[$a]}" "${DESCS[$a]}"
  done
  echo "  all      -> 全部"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --disable) DISABLE=1; shift ;;
    --status)  SHOW_STATUS=1; shift ;;
    --list|-l) list_programs; exit 0 ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "[WARN] 未知参数: $1"; shift ;;
    all) SELECTED=("${ALL_UNITS[@]}"); shift ;;
    *)
      key="$1"
      if [[ -n "${ALIASES[$key]:-}" ]]; then
        SELECTED+=("${ALIASES[$key]}")
      elif [[ " ${ALL_UNITS[*]} " == *" $key "* ]]; then
        # 也允许直接写完整服务单元名
        SELECTED+=("$key")
      else
        echo "[WARN] 未知程序名: $key (用 --list 查看可选项)"
      fi
      shift ;;
  esac
done

if [ "${#SELECTED[@]}" -eq 0 ]; then
  # 未指定 -> 全部, 连带 atrc.target
  echo "==> 停止 atrc.target (连带四件套)"
  systemctl --user stop atrc.target "${ALL_UNITS[@]}" 2>/dev/null || true
  TARGETS=("${ALL_UNITS[@]}")
else
  echo "==> 停止指定程序: ${SELECTED[*]}"
  systemctl --user stop "${SELECTED[@]}" 2>/dev/null || true
  TARGETS=("${SELECTED[@]}")
fi

if [ "${DISABLE}" -eq 1 ]; then
  echo "==> 禁用自启 (以后开机/登录不再启动)"
  if [ "${#SELECTED[@]}" -eq 0 ]; then
    systemctl --user disable atrc.target "${ALL_UNITS[@]}" 2>/dev/null || true
  else
    systemctl --user disable "${SELECTED[@]}" 2>/dev/null || true
  fi
fi

echo "==> 已停止。"
if [ "${SHOW_STATUS}" -eq 1 ]; then
  echo "----- 当前状态 -----"
  systemctl --user --no-pager status 'atrc-*' 2>/dev/null || true
else
  echo "    查看状态: systemctl --user status 'atrc-*'"
  echo "    重新启动: systemctl --user start atrc.target"
fi
