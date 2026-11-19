#!/usr/bin/env python3
"""离线查看腰部碰撞体和右臂 TCP 采样点。"""
import argparse, json
from pathlib import Path
import mujoco
import mujoco.viewer
import pinocchio as pin
import numpy as np

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--urdf',type=Path,required=True)
    ap.add_argument('--plan',type=Path,required=True); ap.add_argument('--side',choices=['l','r'],default='r')
    a=ap.parse_args(); plan=json.loads(a.plan.read_text())
    robot=pin.buildModelFromUrdf(str(a.urdf)); data=robot.createData()
    frame=robot.getFrameId('right_gripper_tcp' if a.side=='r' else 'left_gripper_tcp')
    xml='''<mujoco model="x1_collision_view"><option timestep="0.002"/><visual><global offwidth="1600" offheight="1000"/></visual><worldbody>
      <geom name="ground" type="plane" size="2 2 .01" rgba=".18 .18 .18 1"/>
      <body name="waist_plate_to_axis" pos="0 0 .33857"><geom name="waist_collision" type="box" size=".1295 .087 .33857" rgba=".85 .25 .08 .45" contype="1" conaffinity="1"/></body>
      <body name="waist_axis" pos="0 0 .67714"><geom type="cylinder" size=".04 .20" rgba=".15 .65 .95 1" euler="0 1.5708 0"/></body>
      <body name="tcp" pos="0 0 1"><freejoint/><geom name="tcp" type="sphere" size=".035" rgba=".15 .9 .2 1"/></body>
    </worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml); d=mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m,d) as v:
        v.cam.azimuth=135; v.cam.elevation=-20; v.cam.distance=1.8; v.cam.lookat[:]=[0,0,.5]
        for step in [{'pose_id':'seed','q':plan['seed']},*plan['steps']]:
            q=np.asarray(step['q'],float); pin.forwardKinematics(robot,data,q); pin.updateFramePlacements(robot,data)
            d.qpos[:3]=data.oMf[frame].translation; mujoco.mj_forward(m,d); v.sync()
            print(step['pose_id'], np.round(d.qpos[:3],3), flush=True)
            while v.is_running():
                v.sync();
                if np.linalg.norm(d.qpos[:3]-data.oMf[frame].translation)>1e-6: break
                import time; time.sleep(.03)

if __name__=='__main__': main()
