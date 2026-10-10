"""Rosie 600 in MuJoCo: load the MJCF, hold a pose with the built-in position actuators, read tool0.

    pip install mujoco numpy
    python examples/mujoco_demo.py            # add --view for the interactive viewer
"""
import os
import sys

import mujoco
import numpy as np

KIT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
model = mujoco.MjModel.from_xml_path(os.path.join(KIT, "mjcf", "rosie_600.xml"))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

target = np.radians([30, 45, -10, 0, -35, 0])                 # J1..J6
data.ctrl[:] = target                                         # actuators are position servos: ctrl = target angle
for _ in range(int(3.0 / model.opt.timestep)):                # 3 s of simulated time
    mujoco.mj_step(model, data)

print("joint error (deg):", np.round(np.degrees(data.qpos - target), 3))
print("joint torque (N m):", np.round(data.actuator_force, 1))
print("tool0 position (m):", np.round(data.site("tool0").xpos, 4))
print("tool0 z axis (base frame):", np.round(data.site("tool0").xmat.reshape(3, 3)[:, 2], 4))

if "--view" in sys.argv:
    import mujoco.viewer
    mujoco.viewer.launch(model, data)
