#!/usr/bin/env python3
"""离线比较电流换算假设；不选择结构版本，不生成可加载的控制参数。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from identify import paired, fit
from model import Model

# V3.45 手册表 25-1、25-2，印刷页 104–105；必须按铭牌结构版本选择。
VERSIONS = {
    'erob90': [('V3_MC1', .088), ('V3_MC2', .14)],
    'erob80': [('V3_MC1', .101), ('V4_MC2/V5_MC2', .134)],
    'erob70': [('V3_MC1', .075), ('V4_MC2', .132)],
    'erob70f': [('V3_MC2', .134)],
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path, nargs='+')
    p.add_argument('--fit-latest', action='store_true', help='按用户授权的最新版本近似重做离线辨识，保留物理及误差检查')
    p.add_argument('--latest-assumption', action='store_true', help='使用清单中的最新版本分析假设，不视为已核实参数')
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--inventory', type=Path, default=Path(__file__).with_name('motor_inventory.json'))
    p.add_argument('--conversion', type=Path, default=Path(__file__).with_name('right_arm_session.json'))
    a = p.parse_args()
    cfg = json.loads(a.conversion.read_text())
    inventory = json.loads(a.inventory.read_text())
    rows = []
    for path in a.csv:
        with path.open() as stream:
            rows.extend(csv.DictReader(stream))
    digest = hashlib.sha256(a.conversion.read_bytes()).hexdigest()
    if {r['conversion_sha256'] for r in rows} != {digest}:
        raise ValueError('样本与原始换算配置摘要不一致，不能恢复千分比')
    poses = paired(rows, 'r')
    model = Model(a.urdf, [1, 1])
    cad = []
    for _, qa, qb, _ in poses:
        ya, ka, prior, _ = model.regressor(qa, 'r')
        yb, kb, _, _ = model.regressor(qb, 'r')
        cad.append(((ya+yb)@prior+ka+kb)/2)
    cad = np.array(cad)
    results = []
    ideal_latest = np.zeros((len(poses), 7))
    for i in range(7):
        joint = f'Joint_r{i+1}'
        motor = inventory['joints'][joint]
        family = next(f for f in ['erob70f', 'erob70', 'erob80', 'erob90'] if motor['model'].startswith(f))
        # 旧 CSV 未保存电流：恢复的是带关节方向的扭矩千分比，并假设其等同电流千分比。
        fraction = np.array([t[8+i] for _, _, _, t in poses])/cfg['joints'][joint]['rated_nm']
        current = fraction*motor['drive_rated_current_a_0x6075']
        hypotheses = VERSIONS[family]
        if a.latest_assumption or a.fit_latest:
            h = motor['analysis_kt_hypothesis']
            hypotheses = [(motor['assumed_product_version']+' assumed; Kt row '+h['manual_structure_row'], h['value_nm_per_a'])]
        for version, kt in hypotheses:
            ideal = current*kt*motor['nominal_gear_ratio']
            ideal_latest[:, i] = ideal
            for efficiency in [.5, .7, 1.]:
                results.append(dict(joint=joint, structure_hypothesis=version,
                    kt_nm_per_a=kt, efficiency_hypothesis=efficiency,
                    cad_mean_nm=float(cad[:, i].mean()),
                    converted_mean_nm=float((ideal*efficiency).mean()),
                    rms_against_cad_nm=float(np.sqrt(np.mean((ideal*efficiency-cad[:, i])**2)))))
    report = dict(latest_version_assumption=a.latest_assumption, runtime_applied=False, deployment_allowed=False, results=results,
        assumptions=['夹爪各1kg，CAD不变', '历史扭矩千分比近似电流千分比',
                     '采样时额定电流与本轮诊断相同，尚未证明',
                     '效率0.5/0.7为敏感性假设，1.0为理想对照，不是标定结果'],
        source_sha256={str(f): hashlib.sha256(f.read_bytes()).hexdigest()
                       for f in [*a.csv, a.urdf, a.inventory, a.conversion]})
    if a.fit_latest:
        if inventory.get('version_policy', {}).get('allow_offline_identification') is not True:
            raise ValueError('清单未授权同系列版本近似辨识')
        report['latest_version_assumption'] = True
        report['identification_scenarios'] = []
        # 各效率单独报告，不按验证集误差挑选“获胜”效率或自动部署。
        for efficiency in [.5, .7, 1.]:
            converted = []
            for n, (name, qa, qb, original) in enumerate(poses):
                tau = original.copy()
                tau[8:] = ideal_latest[n]*efficiency
                converted.append((name, qa, qb, tau))
            candidate = fit(model, converted, 'r', .3)
            candidate['efficiency_hypothesis'] = efficiency
            candidate['deployment_allowed'] = False
            report['identification_scenarios'].append(candidate)
    with a.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(a.output)


if __name__ == '__main__':
    main()
