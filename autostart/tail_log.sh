#!/usr/bin/env bash
# 在终端里实时跟随某个 atrc 用户服务的日志。
# 用法: ./tail_log.sh <unit名>   例如 ./tail_log.sh atrc-navigation
#
# 被 ~/.config/autostart/atrc-log-*.desktop 调用, 每个服务弹一个终端窗口。
# 单独手动看某路日志也可以直接跑本脚本。

UNIT="${1:?用法: tail_log.sh <unit名>, 例如 atrc-navigation}"

echo "==================== ${UNIT} 日志 (Ctrl-C 退出窗口) ===================="
echo

# 先补看最近 200 行历史(-n 200), 再 -f 实时跟随。
# --no-hostname 让每行短一点; 服务没起来时 journalctl 也会等着, 一旦有日志就出。
exec journalctl --user -u "${UNIT}" -n 200 -f --no-hostname
