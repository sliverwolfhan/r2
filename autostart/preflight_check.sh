#!/usr/bin/env bash
# 硬件预检：按需等待「某一路」硬件就绪。
# 每个业务服务只检查自己对应的那一路，谁的硬件先好谁先启动，互不牵连。
#
#   --usb <vid:pid>   等待指定 USB 设备插上 (可重复; 例 0483:5740=虚拟串口 0483:5741=机械臂)
#   --lidar           等待激光雷达 (Livox MID360) ping 通
#   (不带选择项)      检查全部: env 里的 USB_IDS 全部 + 雷达
#
#   -t, --timeout N   最长等待秒数 (默认取 env 的 PREFLIGHT_TIMEOUT)
#   --once            只查一次不等待 (手动排查用)
#   -h, --help        帮助
#
# 全部选中的硬件就绪 -> exit 0；超时仍有缺项 -> 打印摘要并 exit 1。
#
# 例:
#   ./preflight_check.sh --usb 0483:5740        # 只等虚拟串口 USB
#   ./preflight_check.sh --lidar                # 只等雷达
#   ./preflight_check.sh --once                 # 一次性检查全部

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 载入集中配置
if [ -f "${SCRIPT_DIR}/autostart.env" ]; then
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/autostart.env"
else
  echo "[ERROR] 未找到 ${SCRIPT_DIR}/autostart.env" >&2
  exit 2
fi

# 默认值兜底
: "${LIDAR_IP:=192.168.1.154}"
: "${LIDAR_NIC:=enp2s0}"
: "${LIDAR_HOST_IP:=192.168.1.50}"
: "${PREFLIGHT_TIMEOUT:=120}"

ONCE=0
CHECK_USB=()        # 本次要检查的 USB 列表
CHECK_LIDAR=0       # 本次是否检查雷达
SELECTED=0          # 用户是否显式选择了检查项

while [[ $# -gt 0 ]]; do
  case "$1" in
    --usb)     CHECK_USB+=("$2"); SELECTED=1; shift 2 ;;
    --lidar)   CHECK_LIDAR=1;     SELECTED=1; shift ;;
    -t|--timeout) PREFLIGHT_TIMEOUT="$2"; shift 2 ;;
    --once)    ONCE=1; shift ;;
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[WARN] 未知参数: $1" >&2; shift ;;
  esac
done

# 未显式选择 -> 检查全部 (env 里的 USB_IDS + 雷达)
if [ "${SELECTED}" -eq 0 ]; then
  if [ -n "${USB_IDS+x}" ]; then CHECK_USB=("${USB_IDS[@]}"); fi
  CHECK_LIDAR=1
fi

usb_label() {  # 友好名称
  case "$1" in
    0483:5740) echo "虚拟串口" ;;
    0483:5741) echo "机械臂" ;;
    *)         echo "USB" ;;
  esac
}

check_usb()   { lsusb -d "$1" >/dev/null 2>&1; }
check_lidar() { ping -c 1 -W 1 "${LIDAR_IP}" >/dev/null 2>&1; }

# 网卡未配 host IP 时告警一次 (仅提示, 不阻塞)
warn_nic_once=1
check_nic_warn() {
  if [ "${warn_nic_once}" -eq 1 ] && ! ip -4 addr show "${LIDAR_NIC}" 2>/dev/null | grep -q "${LIDAR_HOST_IP}"; then
    echo "[告警] 网卡 ${LIDAR_NIC} 未配置 ${LIDAR_HOST_IP}，雷达可能永远 ping 不通。"
    echo "       检查: ip -4 addr show ${LIDAR_NIC}   (可用 install_autostart.sh 配置 netplan)"
    warn_nic_once=0
  fi
}

# 组装本次检查项的描述, 供日志显示
desc=()
[ "${#CHECK_USB[@]}" -gt 0 ] && desc+=("USB:${CHECK_USB[*]}")
[ "${CHECK_LIDAR}" -eq 1 ]   && desc+=("雷达:${LIDAR_IP}")
echo "==> 预检 [${desc[*]}] (超时 ${PREFLIGHT_TIMEOUT}s)"

start=$(date +%s)
while true; do
  missing=()

  for id in "${CHECK_USB[@]}"; do
    check_usb "${id}" || missing+=("$(usb_label "${id}") USB ${id} 未检测到")
  done

  if [ "${CHECK_LIDAR}" -eq 1 ] && ! check_lidar; then
    missing+=("雷达 ${LIDAR_IP} 未连通")
    check_nic_warn
  fi

  if [ "${#missing[@]}" -eq 0 ]; then
    echo "==> [OK] ${desc[*]} 就绪"
    exit 0
  fi

  if [ "${ONCE}" -eq 1 ]; then
    echo "==> [缺项]"; printf '    - %s\n' "${missing[@]}"; exit 1
  fi

  now=$(date +%s)
  if [ $((now - start)) -ge "${PREFLIGHT_TIMEOUT}" ]; then
    echo "==> [失败] 等待 ${PREFLIGHT_TIMEOUT}s 后仍未就绪:" >&2
    printf '    - %s\n' "${missing[@]}" >&2
    exit 1
  fi

  echo "[等待] $(printf '%s; ' "${missing[@]}")"
  sleep 1
done
