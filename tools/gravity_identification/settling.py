"""在驱动器千分比单位中检查窗口稳定性，不依赖未校准的 Nm 换算。"""
import numpy as np


def stable_window(rows, joints, drift_limit=10., std_limit=15., duration=3.):
    if len(rows)<20:
        return False
    stamps=np.array([int(r['time_ns']) for r in rows],dtype=np.int64)
    if np.any(np.diff(stamps)<=0):
        raise ValueError('稳定窗口时间戳不递增')
    if (stamps[-1]-stamps[0])*1e-9<duration:
        return False
    # 分别检查前后各三分之一窗口均值，避免单个瞬时噪声决定结论。
    for kind in ['current','torque']:
        if any(int(r[f'raw_{kind}_valid_'+j])!=1 for r in rows for j in joints):
            return False
        values=np.array([[float(r[f'raw_{kind}_permille_'+j]) for j in joints] for r in rows])
        if not np.isfinite(values).all():
            return False
        n=len(rows)//3
        if np.max(np.abs(values[-n:].mean(axis=0)-values[:n].mean(axis=0)))>drift_limit:
            return False
        if np.max(values.std(axis=0))>std_limit:
            return False
    return True
