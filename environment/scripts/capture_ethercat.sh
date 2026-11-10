#!/usr/bin/env bash
# 被动记录 EtherCAT 帧：不打开主站、不发控制命令，不替代现场操作者启动控制栈。
set -euo pipefail
IFNAME="${1:-enp3s0}"
if [[ ! "$IFNAME" =~ ^[A-Za-z0-9._:-]+$ ]]; then
  echo '网口名无效' >&2
  exit 1
fi
command -v tcpdump >/dev/null
command -v timeout >/dev/null
# 每次使用独立目录；最多两份 50 MB 环形文件，最长 90 秒，避免无限占用磁盘。
CAPTURE_USER="$(id -un)"
CAPTURE_DIR="$(mktemp -d /tmp/x1-ethercat-capture.XXXXXX)"
echo "抓包目录：$CAPTURE_DIR"
echo '仅抓 EtherCAT。请在另一个终端按原流程启动；90 秒后自动结束。'
set +e
sudo timeout --signal=INT --kill-after=5s 90s \
  tcpdump -Z "$CAPTURE_USER" -i "$IFNAME" -p -nn -s 0 -B 4096 -U -C 50 -W 2 \
  -w "$CAPTURE_DIR/ethercat.pcap" 'ether proto 0x88a4' \
  2> >(tee "$CAPTURE_DIR/capture.stderr" >&2)
CAPTURE_RC=$?
set -e
# 124 是达到指定抓包时长，其他失败必须报告。
if [[ "$CAPTURE_RC" != 0 && "$CAPTURE_RC" != 124 ]]; then
  echo "抓包失败，退出码 $CAPTURE_RC；目录：$CAPTURE_DIR" >&2
  exit "$CAPTURE_RC"
fi
echo "抓包完成：$CAPTURE_DIR（capture.stderr 包含内核丢包统计）"
