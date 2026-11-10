"""采样专用躯干净距检查；包围盒给出保守下界，不替代MoveIt碰撞检查。"""
import itertools
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from model import origin


def mesh_tree(triangles):
    # 逐层分割真实三角面；近距离时细化到单面，避免整连杆空白区域误拒绝。
    low=triangles.min((0,1));high=triangles.max((0,1))
    corners=np.array(list(itertools.product(*zip(low,high))))
    if len(triangles)==1:return (corners,None,None)
    centers=triangles.mean(axis=1);axis=int(np.argmax(np.ptp(centers,axis=0)))
    order=np.argsort(centers[:,axis],kind='stable');mid=len(order)//2
    return (corners,mesh_tree(triangles[order[:mid]]),mesh_tree(triangles[order[mid:]]))


def separated(tree_a, tree_b, rotation, translation, margin):
    """证明全部三角面包围盒相距至少margin；无法证明时保守拒绝。"""
    pending=[(tree_a,tree_b)]
    while pending:
        a,b=pending.pop();corners=a[0]@rotation.T+translation
        lo_a=corners.min(0);hi_a=corners.max(0);lo_b=b[0].min(0);hi_b=b[0].max(0)
        gap=np.maximum(0,np.maximum(lo_b-hi_a,lo_a-hi_b))
        if np.linalg.norm(gap)>=margin:continue
        if a[1] is None and b[1] is None:return False
        if b[1] is None or (a[1] is not None and np.linalg.norm(hi_a-lo_a)>=np.linalg.norm(hi_b-lo_b)):
            pending.extend([(a[1],b),(a[2],b)])
        else:pending.extend([(a,b[1]),(a,b[2])])
    return True


class BodyClearance:
    def __init__(self, urdf, package_dir, margin=.04):
        if not np.isfinite(margin) or margin<.04:raise ValueError('躯干净距不得小于0.04m')
        self.margin=margin;self.boxes={};self.trees={}
        root=ET.parse(urdf).getroot()
        for link in root.findall('link'):
            name=link.get('name')
            if name not in ['Body','Waist']+[f'arm{s}{i}' for s in 'lr' for i in range(3,8)]:continue
            points=[]
            for collision in link.findall('collision'):
                mesh=collision.find('geometry/mesh')
                if mesh is None:raise ValueError('躯干净距检查需要网格碰撞几何')
                prefix='package://arm_describle/'
                filename=mesh.get('filename','')
                if not filename.startswith(prefix):raise ValueError('未知网格包')
                data=(Path(package_dir)/filename[len(prefix):]).read_bytes()
                n=struct.unpack_from('<I',data,80)[0]
                if len(data)!=84+50*n:raise ValueError('需要二进制STL')
                v=np.array([struct.unpack_from('<12f',data,84+i*50)[3:]for i in range(n)]).reshape(-1,3)
                v*=np.array([float(x)for x in mesh.get('scale','1 1 1').split()])
                xyz,r=origin(collision.find('origin'));v=v@r.T+xyz
                points.extend(v)
            v=np.array(points)
            self.trees[name]=mesh_tree(v.reshape(-1,3,3))
            self.boxes[name]=np.array(list(itertools.product(*zip(v.min(0),v.max(0)))))

    def minimum(self, model, q, side):
        frames,_=model.geometry(q);best=(float('inf'),'','')
        # 肩部1/2轴与躯干为连接结构，不施加分离距离；仍由MoveIt检查。
        for torso in ('Body','Waist'):
            tp,tr,_=frames[torso];box=self.boxes[torso];lo=box.min(0);hi=box.max(0)
            for i in range(3,8):
                arm=f'arm{side}{i}';ap,ar,_=frames[arm]
                corners=(self.boxes[arm]@ar.T+ap-tp)@tr
                gap=np.maximum(0,np.maximum(lo-corners.max(0),corners.min(0)-hi))
                distance=float(np.linalg.norm(gap))
                if distance<best[0]:best=(distance,arm,torso)
        return best

    def check(self, model, q, side):
        frames,_=model.geometry(q)
        for torso in ('Body','Waist'):
            tp,tr,_=frames[torso]
            for i in range(3,8):
                arm=f'arm{side}{i}';ap,ar,_=frames[arm]
                if not separated(self.trees[arm],self.trees[torso],tr.T@ar,tr.T@(ap-tp),self.margin):
                    raise ValueError(f'{arm}与{torso}网格细分检查不能保证{self.margin*1000:.0f}mm净距；请先远离躯干')
