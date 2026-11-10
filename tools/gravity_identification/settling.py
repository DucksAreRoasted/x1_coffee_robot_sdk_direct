"""在驱动器千分比单位中检查窗口稳定性，不依赖未校准的 Nm 换算。"""
import numpy as np


def stability_reason(rows, joints, drift_limit=10., std_limit=15., duration=3.):
    """空字符串表示通过；否则返回未通过原因，供界面显示同一套判据。"""
    if len(rows)<20:
        return f'正在积累样本：{len(rows)}/20条'
    stamps=np.array([int(r['time_ns']) for r in rows],dtype=np.int64)
    if np.any(np.diff(stamps)<=0):
        raise ValueError('稳定窗口时间戳不递增')
    span=(stamps[-1]-stamps[0])*1e-9
    if span<duration:
        return f'正常积累窗口：{span:.2f}/{duration:.2f}秒'
    reasons=[]
    # 分别检查前后各三分之一窗口均值，避免单个瞬时噪声决定结论。
    for kind,label in [('current','电流'),('torque','扭矩')]:
        invalid=[j for j in joints if any(int(r[f'raw_{kind}_valid_'+j])!=1 for r in rows)]
        if invalid:
            reasons.append(f'{label}反馈无效：'+','.join(invalid))
            continue
        values=np.array([[float(r[f'raw_{kind}_permille_'+j]) for j in joints] for r in rows])
        if not np.isfinite(values).all():
            reasons.append(f'{label}反馈包含非有限数值')
            continue
        n=len(rows)//3
        drift=np.abs(values[-n:].mean(axis=0)-values[:n].mean(axis=0))
        std=values.std(axis=0)
        for metric,limit,name in [(drift,drift_limit,'前后均值漂移'),(std,std_limit,'标准差')]:
            index=int(np.argmax(metric))
            if metric[index]>limit:
                reasons.append(f'{joints[index]} {label}{name} {metric[index]:.2f}‰ > {limit:g}‰')
    return '；'.join(reasons)


def stable_window(rows, joints, drift_limit=10., std_limit=15., duration=3.):
    return not stability_reason(rows,joints,drift_limit,std_limit,duration)


class StableConfirmation:
    """连续通过窗口检查后才收样，避免在衰减过程首次越过阈值时立即结束。"""
    def __init__(self, seconds=3.):
        self.seconds=seconds
        self.since=None

    def update(self, now, passed):
        if not passed:
            self.since=None
            return False
        if self.since is None:
            self.since=now
        return now-self.since>=self.seconds
