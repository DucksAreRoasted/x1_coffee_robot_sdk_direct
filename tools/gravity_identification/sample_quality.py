#!/usr/bin/env python3
"""离线检查静态样本的力矩漂移；不将旧换算值视为已验证输出力矩。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from model import JOINTS


def quality(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row['pose_id'], int(row['approach'])), []).append(row)
    result = []
    for (pose, direction), batch in sorted(groups.items()):
        batch = sorted(batch, key=lambda r: int(r['time_ns']))
        if len(batch) < 20:
            raise ValueError('漂移分析至少需要20个样本')
        joints = []
        for j in JOINTS[8:]:
            q = np.array([float(r['q_'+j]) for r in batch])
            tau = np.array([float(r['tau_'+j]) for r in batch])
            if not np.isfinite(np.r_[q, tau]).all():
                raise ValueError('样本含非有限值')
            joints.append(dict(joint=j, q_span_rad=float(np.ptp(q)),
                legacy_tau_std=float(np.std(tau)),
                legacy_tau_end_minus_start=float(np.mean(tau[-10:])-np.mean(tau[:10]))))
        result.append(dict(pose_id=pose, approach=direction, samples=len(batch), joints=joints))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    rows = []
    for f in a.csv:
        with f.open() as stream:
            rows.extend(csv.DictReader(stream))
    report = dict(runtime_applied=False, note='legacy_tau 使用原 CSV 换算；不是已验证输出侧 Nm。前后各10个样本均值作漂移诊断，不据此自动剔除数据。', groups=quality(rows))
    with a.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(a.output)


if __name__ == '__main__':
    main()
