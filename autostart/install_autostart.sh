#!/usr/bin/env bash
# 一键安装开机自启动三件套。
# 用法:
#   ./install_autostart.sh              # 全部安装 (systemd 用户服务 + udev + netplan)
#   ./install_autostart.sh --no-net     # 跳过 netplan 网卡配置 (你自己管网络)
#   ./install_autostart.sh --with-bt    # 顺带让行为树 bt_runner 开机自启
#   ./install_autostart.sh --no-bt      # 不让 bt_runner 自启 (即使 env 里设了 1)
#   ./install_autostart.sh --uninstall  # 卸载 (停用服务 + 删除已装文件)
#
# bt_runner 是否自启默认看 autostart.env 里的 ENABLE_BT_RUNNER (0/1);
# --with-bt / --no-bt 只是命令行临时覆盖它。
#
# systemd 用户服务需 sudo 装 udev/netplan; 会提示输入密码。

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_UNIT_DIR="${HOME}/.config/systemd/user"
# 核心件 (总是安装+启用) + target
UNITS=(atrc-virtual-serial.service atrc-navigation.service atrc-arm.service atrc-planner.service atrc-planner-qt-test.service atrc-serial-bridge-test.service atrc.target)
# 行为树 (可选, 由 ENABLE_BT_RUNNER / --with-bt / --no-bt 决定是否 enable)
BT_UNIT=atrc-bt-runner.service

# bt 自启默认取自 autostart.env; 命令行可覆盖 (空=不覆盖)
ENV_FILE="${SCRIPT_DIR}/autostart.env"
[ -f "${ENV_FILE}" ] && source "${ENV_FILE}"
BT_OVERRIDE=""

DO_NET=1
UNINSTALL=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-net)    DO_NET=0; shift ;;
    --with-bt)   BT_OVERRIDE=1; shift ;;
    --no-bt)     BT_OVERRIDE=0; shift ;;
    --uninstall) UNINSTALL=1; shift ;;
    -h|--help)   grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[WARN] 未知参数: $1"; shift ;;
  esac
done

# 命令行覆盖 env; 都没设则默认 0 (不自启)
ENABLE_BT_RUNNER="${BT_OVERRIDE:-${ENABLE_BT_RUNNER:-0}}"

# ---------- 卸载 ----------
if [ "${UNINSTALL}" -eq 1 ]; then
  echo "==> 停用并卸载 systemd 用户服务"
  systemctl --user disable --now atrc.target "${UNITS[@]}" "${BT_UNIT}" 2>/dev/null || true
  for u in "${UNITS[@]}" "${BT_UNIT}"; do rm -f "${USER_UNIT_DIR}/${u}"; done
  systemctl --user daemon-reload
  echo "==> 删除 udev / netplan (需 sudo)"
  sudo rm -f /etc/udev/rules.d/99-atrc-usb.rules
  sudo udevadm control --reload 2>/dev/null || true
  echo "    netplan 文件 /etc/netplan/99-atrc-lidar.yaml 未自动删除 (避免误改网络); 如需删除请手动处理。"
  echo "==> 卸载完成"
  exit 0
fi

# ---------- 0. 确保脚本可执行 ----------
echo "==> chmod +x 脚本"
chmod +x "${SCRIPT_DIR}"/*.sh
chmod +x "${SCRIPT_DIR}/.."/run_*.sh 2>/dev/null || true

# ---------- 1. systemd 用户服务 ----------
echo "==> 安装 systemd 用户单元到 ${USER_UNIT_DIR}"
mkdir -p "${USER_UNIT_DIR}"
cp "${SCRIPT_DIR}/systemd/"*.service "${SCRIPT_DIR}/systemd/"*.target "${USER_UNIT_DIR}/"
systemctl --user daemon-reload
systemctl --user enable atrc.target "${UNITS[@]}"
echo "    已 enable, 下次登录桌面自动启动。"

# 行为树 bt_runner: 按开关 enable / disable (不影响核心四件套)
if [ "${ENABLE_BT_RUNNER}" = "1" ]; then
  systemctl --user enable "${BT_UNIT}"
  echo "    bt_runner 已开启自启 (ENABLE_BT_RUNNER=1)。"
else
  systemctl --user disable "${BT_UNIT}" 2>/dev/null || true
  echo "    bt_runner 未自启 (ENABLE_BT_RUNNER=0)。要开: 改 autostart.env 或加 --with-bt 重装。"
fi

# 让用户服务能在登录时随图形会话拉起 (开机免登录场景可选开 linger)
loginctl enable-linger "$(whoami)" 2>/dev/null || true

# ---------- 2. udev 规则 (USB 授权) ----------
echo "==> 安装 udev 规则 (USB 0483:5740 / 0483:5741 授权, 需 sudo)"
sudo cp "${SCRIPT_DIR}/99-atrc-usb.rules" /etc/udev/rules.d/99-atrc-usb.rules
sudo udevadm control --reload
sudo udevadm trigger
echo "    (若 USB 当前已插着, 请拔插一次让规则生效)"

# ---------- 3. netplan 网卡静态 IP (雷达) ----------
if [ "${DO_NET}" -eq 1 ]; then
  echo "==> 配置雷达网卡 enp2s0=192.168.1.50/24 (netplan, 需 sudo)"
  echo "    如你已有网络方案, 可 Ctrl-C 中断并改用 --no-net 重装。"
  sudo cp "${SCRIPT_DIR}/99-atrc-lidar.yaml" /etc/netplan/99-atrc-lidar.yaml
  sudo chmod 600 /etc/netplan/99-atrc-lidar.yaml
  sudo netplan apply
  echo "    已应用。检查: ip -4 addr show enp2s0"
else
  echo "==> 跳过网卡配置 (--no-net)。请自行确保 enp2s0 有 192.168.1.50，否则雷达 ping 不通。"
fi

# ---------- 完成 ----------
cat <<EOF

========================================================
安装完成！

切蓝/红:   编辑 autostart/zone.conf 写 blue 或 red，然后
           systemctl --user restart atrc-navigation

看日志:    journalctl --user -u atrc-virtual-serial -f
           journalctl --user -u atrc-navigation -f
           journalctl --user -u atrc-arm -f
           journalctl --user -u atrc-planner -f
           journalctl --user -u atrc-planner-qt-test -f
           journalctl --user -u atrc-serial-bridge-test -f
           journalctl --user -u atrc-bt-runner -f

看状态:    systemctl --user status 'atrc-*'
立即启动:  systemctl --user start atrc.target
停止全部:  systemctl --user stop atrc.target

行为树自启: 当前 ENABLE_BT_RUNNER=${ENABLE_BT_RUNNER} (0=不自启 1=自启)
           单独开: systemctl --user enable --now atrc-bt-runner
           单独关: systemctl --user disable --now atrc-bt-runner

每个程序只等自己那一路硬件就绪就启动:
  虚拟串口 -> USB 0483:5740
  机械臂   -> USB 0483:5741
  导航     -> 雷达 ${LIDAR_IP:-192.168.1.154}
  行为树   -> 无硬件, 排在导航/规划器之后启动 (可选自启)
========================================================
EOF
