"""用独立NumPy树形FK核对键盘旋转；不调用硬件或改变控制参数。"""
import runpy
import unittest
from pathlib import Path
import numpy as np
from model import Model,JOINTS,rotation

ROOT=Path(__file__).resolve().parents[2]
NS=runpy.run_path(str(ROOT/'src/x1_coffee_integration/scripts/keyboard_tcp_teleop.py'))

class KeyboardRotationTest(unittest.TestCase):
    def test_tcp_axes_both_arms_positive_and_negative(self):
        urdf=ROOT/'src/arm_describle/urdf/HRB3Q-LC.urdf'
        model=Model(urdf,[1,1])
        # 现场采样起点附近的非零姿态，含腰转角，避免只验证零位。
        q=np.array([-.2601,.9701,.0261,.0153,-.1999,-1.607,-1.1661,1.643,
                    -.9899,.00016,-.03998,.2002,1.6400,1.1501,-1.6601])
        frames,_=model.geometry(q)
        for side,short in [('left','l'),('right','r')]:
            tip=side+'_gripper_tcp'
            chain=NS['load_chain'](str(urdf),tip,[f'Joint_{short}{j}' for j in range(1,8)])
            position,orientation,_=frames[tip]
            for axis in range(3):
                for sign in (-1,1):
                    with self.subTest(side=side,axis=axis,sign=sign):
                        delta=np.eye(3)[axis]*sign*.03
                        result=chain.step(dict(zip(JOINTS,q)),[0,0,0],delta.tolist())
                        self.assertIsNotNone(result)
                        moved=q.copy()
                        for name,value in result.items():moved[JOINTS.index(name)]=value
                        actual,_=model.geometry(moved)
                        p,r,_=actual[tip]
                        expected=orientation@rotation(np.eye(3)[axis],sign*.03)
                        error=np.arccos(np.clip((np.trace(expected@r.T)-1)/2,-1,1))
                        self.assertLess(error,.001)
                        self.assertLess(np.linalg.norm(p-position),.0004)

if __name__=='__main__':unittest.main()
