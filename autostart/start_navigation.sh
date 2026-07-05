#!/usr/bin/env bash
# 根据 zone.conf 选择蓝区/红区导航并启动。
# 切区只需编辑 autostart/zone.conf 写 blue 或 red。

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ZONE_FILE="${SCRIPT_DIR}/zone.conf"

# 读取 zone (去除空白/注释)，默认 blue
ZONE="blue"
if [ -f "${ZONE_FILE}" ]; then
  ZONE="$(grep -vE '^\s*#' "${ZONE_FILE}" | tr -d '[:space:]' | head -c 16)"
  ZONE="${ZONE:-blue}"
fi

case "${ZONE}" in
  blue) RUN="${WORKSPACE_DIR}/run_navigation_blue.sh" ;;
  red)  RUN="${WORKSPACE_DIR}/run_navigation_red.sh" ;;
  *)
    echo "[ERROR] zone.conf 非法值: '${ZONE}' (只能是 blue 或 red)" >&2
    exit 1 ;;
esac

if [ ! -x "${RUN}" ] && [ ! -f "${RUN}" ]; then
  echo "[ERROR] 未找到导航脚本: ${RUN}" >&2
  exit 1
fi

echo "==> zone=${ZONE}, 启动 ${RUN}"
exec bash "${RUN}" "$@"
