"""Rosie 1000 kinematics in plain numpy, from robot.json: forward kinematics and a damped least-squares IK.

    pip install numpy
    python examples/fk_ik.py
"""
import json
import os

import numpy as np

KIT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT = json.load(open(os.path.join(KIT, "robot.json")))
JOINTS = ROBOT["joints"]
LOWER = np.array([j["lower"] for j in JOINTS])
UPPER = np.array([j["upper"] for j in JOINTS])


def rot(axis, q):
    """Rotation matrix about a unit axis by q radians."""
    x, y, z = axis
    c, s, t = np.cos(q), np.sin(q), 1 - np.cos(q)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


TOOL0 = np.eye(4)
TOOL0[:3, :3] = rot([1, 0, 0], ROBOT["frames"]["tool0"]["rpy"][0])   # tool0: z out of the flange


def fk(q):
    """4x4 pose of tool0 in base_link for joint angles q (rad). Link frames are parallel at zero, so rpy = 0."""
    T = np.eye(4)
    for j, qi in zip(JOINTS, q):
        A = np.eye(4)
        A[:3, 3] = j["origin_xyz"]
        A[:3, :3] = rot(j["axis"], qi)
        T = T @ A
    return T @ TOOL0


def pose_error(T, target):
    dp = target[:3, 3] - T[:3, 3]
    R = target[:3, :3] @ T[:3, :3].T
    dw = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return np.concatenate([dp, dw])


def ik(target, q0, iters=200, damping=1e-3):
    """Joint angles reaching the 4x4 target pose of tool0, starting from q0; stays inside the joint limits."""
    q = np.array(q0, float)
    for _ in range(iters):
        e = pose_error(fk(q), target)
        if np.linalg.norm(e) < 1e-9:
            break
        J = np.zeros((6, 6))
        for k in range(6):
            dq = np.zeros(6)
            dq[k] = 1e-6
            J[:, k] = (pose_error(fk(q), target) - pose_error(fk(q + dq), target)) / 1e-6
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), e), LOWER, UPPER)
    return q


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)
    stretched = ROBOT["poses"]["stretched"]
    wrist = np.eye(4)
    for j, qi in zip(JOINTS[:5], stretched):                    # wrist centre = the J5 frame origin
        A = np.eye(4); A[:3, 3] = j["origin_xyz"]; A[:3, :3] = rot(j["axis"], qi); wrist = wrist @ A
    print("home tool0 (m):", fk(ROBOT["home"])[:3, 3])
    print("stretched wrist centre (m):", wrist[:3, 3], "reach", round(float(np.hypot(*wrist[:2, 3])), 4))
    target = fk(ROBOT["poses"]["work"])
    q = ik(target, ROBOT["home"])
    print("IK to the work pose (deg):", np.degrees(q))
    print("IK pose error:", np.linalg.norm(pose_error(fk(q), target)))
