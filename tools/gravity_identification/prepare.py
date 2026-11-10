#!/usr/bin/env python3
"""从已有启动日志生成辨识换算草稿；不访问硬件，不自动确认扭矩方向。"""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from model import JOINTS


def conversion_from_log(text):
    # 拒绝多轮启动间不一致的读数，避免把硬件变更后的日志混为同一配置。
    values = {}
    for axis, value in re.findall(r'轴\s+(\d+)\s+额定力矩\s+([\d.eE+-]+)\s+N·m', text):
        axis, value = int(axis), float(value)
        if not 1 <= axis <= 15 or not math.isfinite(value) or value <= 0:
            raise ValueError('无效额定扭矩记录')
        if axis in values and values[axis] != value:
            raise ValueError(f'轴 {axis} 存在不同额定扭矩读数，请检查日志来源')
        values[axis] = value
    missing = sorted(set(range(1, 16))-set(values))
    if missing:
        raise ValueError(f'日志缺少额定扭矩，轴号：{missing}')
    return {'torque_signs_verified': False,
            'note': '额定扭矩来自历史启动日志，精度受日志小数位限制；sign 须独立实测。',
            'joints': {j: {'rated_nm': values[i+1], 'sign': None} for i,j in enumerate(JOINTS)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.log.read_bytes()
    result = conversion_from_log(raw.decode('utf-8'))
    result['source'] = {'log': str(args.log), 'sha256': hashlib.sha256(raw).hexdigest()}
    result['load_condition'] = {'left_gripper_kg': None, 'right_gripper_kg': None,
                                'payload_description': None}
    content = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    with args.output.open('x') as handle:
        handle.write(content)
    print(f'已生成草稿：{args.output}；方向和负载尚未确认，不能直接采样。')

if __name__ == '__main__':
    main()
