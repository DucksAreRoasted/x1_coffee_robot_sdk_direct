#!/usr/bin/env python3
"""从启动日志提取额定电流和版本；不访问硬件、不推定电机结构版本。"""
import argparse
import hashlib
import json
import re
from pathlib import Path
from model import JOINTS


def parse_diagnostics(text):
    currents = {}
    versions = {}
    for axis, value in re.findall(r'\[电机诊断\] axis=(\d+) index=0x6075 rated_current_ma=(\d+)', text):
        axis, value = int(axis), int(value)
        if not 1 <= axis <= 15 or not 0 < value <= 0xffffffff:
            raise ValueError('无效额定电流记录')
        currents.setdefault(axis, set()).add(value)
    for axis, obj, value in re.findall(r'\[电机诊断\] axis=(\d+) index=(0x100[89A]) value=([^\r\n]*)', text):
        axis = int(axis)
        if not 1 <= axis <= 15:
            raise ValueError('无效版本记录轴号')
        versions.setdefault((axis, obj), set()).add(value.strip())
    if set(currents) != set(range(1, 16)):
        raise ValueError('日志缺少全部15轴的有效额定电流')
    if any(len(v) != 1 for v in [*currents.values(), *versions.values()]):
        raise ValueError('多轮诊断存在冲突，不能混用')
    return {j: dict(hardware_axis=i, rated_current_ma=next(iter(currents[i])),
                   versions={o: next(iter(versions[(i, o)]))
                             for o in ['0x1008', '0x1009', '0x100A'] if (i, o) in versions},
                   structure_version=None)
            for i, j in enumerate(JOINTS, 1)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    raw = a.log.read_bytes()
    report = dict(joints=parse_diagnostics(raw.decode()), runtime_applied=False,
                  source_path=str(a.log), source_sha256=hashlib.sha256(raw).hexdigest())
    with a.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(a.output)


if __name__ == '__main__':
    main()
