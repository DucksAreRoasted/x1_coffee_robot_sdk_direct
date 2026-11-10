"""X1 双臂离线静力学模型，坐标和角度直接使用 URDF；不连接机器人。"""
import xml.etree.ElementTree as ET
import numpy as np

JOINTS = ['Joint_Body'] + [f'Joint_{s}{j}' for s in 'lr' for j in range(1, 8)]


def vec(node, key, default='0 0 0'):
    return np.array([float(v) for v in (node.get(key, default) if node is not None else default).split()])


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def origin(node):
    r, p, y = vec(node, 'rpy')
    return vec(node, 'xyz'), rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)


class Model:
    def __init__(self, path, gripper_kg):
        root = ET.parse(path).getroot()
        self.links = {}
        for link in root.findall('link'):
            inertial = link.find('inertial')
            mass, com, inertia = 0., np.zeros(3), np.zeros((3, 3))
            if inertial is not None:
                mass = float(inertial.find('mass').get('value'))
                com, r = origin(inertial.find('origin'))
                a = inertial.find('inertia').attrib
                inertia = r @ np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],
                    [float(a['ixy']),float(a['iyy']),float(a['iyz'])],
                    [float(a['ixz']),float(a['iyz']),float(a['izz'])]]) @ r.T
            self.links[link.get('name')] = (mass, com, inertia)
        # 与现有 C++ 模型一致，但要求调用方显式声明夹爪质量，不把假设当实测。
        for name, mass in zip(['left_gripper_tcp', 'right_gripper_tcp'], gripper_kg):
            if name not in self.links or not np.isfinite(mass) or mass < 0:
                raise ValueError('缺少夹爪 TCP 或质量无效')
            self.links[name] = (float(mass), np.zeros(3), np.zeros((3, 3)))
        self.joints = root.findall('joint')
        self.by_name = {j.get('name'): j for j in self.joints}
        if not set(JOINTS).issubset(self.by_name):
            raise ValueError('URDF 不包含 coffee_15_no_slide 全部关节')
        children = {j.find('child').get('link') for j in self.joints}
        roots = set(self.links)-children
        if len(roots) != 1: raise ValueError('URDF 必须为单根树')
        self.root = roots.pop()

    def geometry(self, q):
        q = np.asarray(q, float)
        if q.shape != (15,) or not np.isfinite(q).all(): raise ValueError('需要 15 个有限弧度角')
        frames = {self.root: (np.zeros(3), np.eye(3), [])}
        axes = {}
        pending = list(self.joints)
        while pending:
            progress = False
            for j in list(pending):
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                if parent not in frames: continue
                p, r, ancestors = frames[parent]
                xyz, r0 = origin(j.find('origin')); p = p+r@xyz; r = r@r0
                chain = list(ancestors); name = j.get('name')
                if j.get('type') != 'fixed':
                    if name not in JOINTS: raise ValueError('未知可动关节 '+name)
                    idx = JOINTS.index(name); axis = vec(j.find('axis'), 'xyz', '1 0 0')
                    axis /= np.linalg.norm(axis); axes[idx] = (p, r@axis)
                    r = r@rotation(axis, q[idx]); chain.append(idx)
                frames[child] = (p, r, chain); pending.remove(j); progress = True
            if not progress: raise ValueError('URDF 拓扑断开')
        return frames, axes

    def regressor(self, q, side):
        frames, axes = self.geometry(q)
        selected = [self.by_name[f'Joint_{side}{i}'].find('child').get('link') for i in range(1,8)]
        y = np.zeros((15,28)); known = np.zeros(15); inertia = np.zeros(15); prior=[]
        for name in selected:
            mass, com, _ = self.links[name]; prior.extend([mass, *(mass*com)])
        # 重力补偿为势能梯度，向上力 m*g 与关节轴的力臂投影。
        up = np.array([0.,0.,9.80665])
        for name, (p,r,chain) in frames.items():
            mass, com, local_i = self.links[name]; world_com = p+r@com
            for idx in chain:
                pivot, axis = axes[idx]
                jac = np.cross(axis, world_com-pivot)
                inertia[idx] += mass*(jac@jac)+axis@(r@local_i@r.T)@axis
                if name in selected:
                    col = selected.index(name)*4
                    y[idx,col] = axis@np.cross(p-pivot,up)
                    for k in range(3): y[idx,col+1+k] = axis@np.cross(r[:,k],up)
                else: known[idx] += mass*(axis@np.cross(world_com-pivot,up))
        slots = slice(1,8) if side=='l' else slice(8,15)
        return y[slots], known[slots], np.array(prior), inertia[slots]
