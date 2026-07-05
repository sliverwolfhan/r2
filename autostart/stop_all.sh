#!/usr/bin/env bash
# 一键中断开机自启动四件套 (虚拟串口 + 导航 + 机械臂 + 规划器)。
# 用法:
#   ./stop_all.sh            # 停掉全部三个 (本次停, 下次登录仍会自启)
#   ./stop_all.sh --disable  # 停掉并禁用自启 (以后开机/登录不再启动)
#   ./stop_all.sh --status   # 停完后顺带打印状态

set -u

UNITS=(atrc-virtual-serial.service atrc-navigation.service atrc-arm.service atrc-planner.service)
DISABLE=0
SHOW_STATUS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --disable) DISABLE=1; shift ;;
    --status)  SHOW_STATUS=1; shift ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[WARN] 未知参数: $1"; shift ;;
  esac
done

echo "==> 停止 atrc.target (连带三件套)"
systemctl --user stop atrc.target "${UNITS[@]}" 2>/dev/null || true

if [ "${DISABLE}" -eq 1 ]; then
  echo "==> 禁用自启 (以后开机/登录不再启动)"
  systemctl --user disable atrc.target "${UNITS[@]}" 2>/dev/null || true
fi

echo "==> 已停止。"
if [ "${SHOW_STATUS}" -eq 1 ]; then
  echo "----- 当前状态 -----"
  systemctl --user --no-pager status 'atrc-*' 2>/dev/null || true
else
  echo "    查看状态: systemctl --user status 'atrc-*'"
  echo "    重新启动: systemctl --user start atrc.target"
fi
