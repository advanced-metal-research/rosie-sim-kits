"""Rosie 1400 in PyBullet: load the URDF, compute forward kinematics, move to a pose under position control.

    pip install pybullet numpy
    python examples/pybullet_demo.py          # add --gui for a window
"""
import math
import os
import sys

import pybullet as p

KIT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
p.connect(p.GUI if "--gui" in sys.argv else p.DIRECT)
p.setGravity(0, 0, -9.81)
robot = p.loadURDF(os.path.join(KIT, "urdf", "rosie_1400.urdf"), useFixedBase=True,
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
info = {p.getJointInfo(robot, i)[12].decode(): p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))}
joints = [info[f"link_{k}"] for k in range(1, 7)]           # J1..J6, in order
tool0 = info["tool0"][0]

# Forward kinematics: put the arm in the stretched pose (robot.json poses.stretched) and read tool0 (the flange face).
for j, q in zip(joints, [0, 1.570796, -1.570796, 0, 0, 0]):
    p.resetJointState(robot, j[0], q)
pos, orn = p.getLinkState(robot, tool0, computeForwardKinematics=True)[4:6]
print("tool0 at the stretched pose (m):", [round(x, 4) for x in pos])

# Move from home to a work pose with the robot's own torque and speed limits.
for j in joints:
    p.resetJointState(robot, j[0], 0.0)
target = [math.radians(a) for a in (30, 45, -10, 0, -35, 0)]
for j, q in zip(joints, target):
    p.setJointMotorControl2(robot, j[0], p.POSITION_CONTROL, targetPosition=q,
                            force=j[10], maxVelocity=j[11])     # URDF effort (N m) and velocity (rad/s)
for _ in range(240 * 3):                                       # 3 s at PyBullet's default 240 Hz
    p.stepSimulation()
q = [p.getJointState(robot, j[0])[0] for j in joints]
print("joint error (deg):", [round(math.degrees(a - b), 3) for a, b in zip(q, target)])
print("tool0 at the work pose (m):", [round(x, 4) for x in p.getLinkState(robot, tool0, computeForwardKinematics=True)[4]])
