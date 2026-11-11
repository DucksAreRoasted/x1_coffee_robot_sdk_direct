#!/usr/bin/env bash
# 用 tshark 对 SOEM/IgH 抓包做只读统计，不打开主站，也不发送 EtherCAT 帧。
set -euo pipefail
PCAP="${1:-}"
if [[ -z "$PCAP" || ! -f "$PCAP" ]]; then
  echo "用法：$0 capture.pcap" >&2
  exit 2
fi
command -v tshark >/dev/null
echo "文件：$PCAP"
echo "EtherCAT 帧数：$(tshark -n -r "$PCAP" -Y ecat -T fields -e frame.number 2>/dev/null | wc -l)"
echo "总帧数：$(tshark -n -r "$PCAP" -T fields -e frame.number 2>/dev/null | wc -l)"
echo "帧长分布："
tshark -n -r "$PCAP" -Y ecat -T fields -e frame.len 2>/dev/null |
  awk 'NF {count[$1]++} END {for (n in count) print "  " n " bytes: " count[n]}' | sort -n
echo "EtherCAT 帧间隔统计（由 frame.time_epoch 计算）："
tshark -n -r "$PCAP" -Y ecat -T fields -e frame.time_epoch 2>/dev/null |
  awk 'NF {if (last!="") {d=$1-last; n++; sum+=d; if (d<min||min==0) min=d; if (d>max) max=d} last=$1} END {if(n) printf "  samples=%d mean=%.6f ms min=%.6f ms max=%.6f ms\n",n,sum/n*1000,min*1000,max*1000; else print "  无可用 EtherCAT 时间戳"}'
echo "提示：用 Wireshark 打开该 pcap，可继续查看 ECAT datagram、WKC、命令类型和从站地址。"
