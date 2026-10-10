# Rosie 1400 simulation and CAD kit

Everything needed to put the Rosie 1400 into a simulator or a CAD layout: a URDF, a MuJoCo model, an SRDF, a
STEP assembly, meshes, and `robot.json` with the kinematics, joint limits, speeds, torques, payload ratings and
mass properties in one machine-readable file. The geometry is the robot's exterior, one filled solid per link, with
the structural parts' own CAD surfaces and the motors and gearboxes as plain envelopes; the kinematics, limits, ratings
and inertias are the robot's own.

Kit 2026-10-09 from Advanced Metal Research, https://advancedmetalresearch.com/rosie. Online copy of every file: https://advancedmetalresearch.com/assets/sim/1400/

## Files

| File | What it is |
|---|---|
| `urdf/rosie_1400.urdf` | URDF: joints J1 to J6, real inertials, visual and collision meshes, `tool0` |
| `mjcf/rosie_1400.xml` | MuJoCo model with position actuators, torque limits, armature and keyframes |
| `srdf/rosie_1400.srdf` | Planning group `manipulator` (base_link to tool0), named poses, adjacent-link collision pairs |
| `step/rosie_1400.step` | STEP AP214 assembly, mm, one solid per link (exact CAD surfaces), at the zero pose (7.5 MB) |
| `robot.json` | Kinematics, limits, speeds, torques, accelerations, payloads, mass properties, frames, FK reference poses, the benchmark cycle |
| `ik_cases.json` | IK reference cases: `tool0` target poses with every joint solution inside the limits and its branch |
| `benchmark/rosie_1400_cycle_1kg.csv`, `.json` | The benchmark pick-and-place cycle with 1 kg: t, q, qd, qdd every 1 ms, and the TCP |
| `glb/rosie_1400.glb` | glTF binary for browsers and game engines: one node per link in the joint hierarchy (1.1 MB) |
| `meshes/visual/*.stl` | One mesh per link, in the link frame, metres |
| `meshes/collision/*.stl` | Convex pieces per link, in the link frame, metres |
| `launch/display.launch.py`, `rviz/`, `package.xml`, `CMakeLists.txt` | ROS 2 package `rosie_1400_description` |
| `examples/` | The quickstart scripts below, ready to run |

## Conventions

- Units: metres, kilograms, radians, seconds (the STEP is in millimetres).
- Joints `J1` to `J6` connect `base_link`, `link_1` ... `link_6`. Every link frame is parallel to `base_link` at the
  zero pose and sits on its joint. Axes: J1 z, J2 y, J3 y, J4 x, J5 y, J6 z, positive by the right-hand rule.
- Zero pose = home: upper arm vertical, forearm along +x, tool flange facing down.
- `tool0` is on the tool flange face, z out of the flange (down at home), x along base +x at home. Add your tool's TCP
  as a fixed offset from `tool0`.
- `base_footprint` is the floor (the mounting face) under J1. `base_link` is the robot base frame,
  29.4 mm above it. The STEP's origin is `base_link`.
- Same joint names, link names, axes and zero pose as RosieOS's robot descriptions (`rosie_1400_v3`).

At home, `tool0` is at [0.8337, 0.0, 0.7557] m in `base_link`. At the stretched pose (J2 = 90°, J3 = -90°) the wrist centre is
at [1.4337, 0.0, 0.255] m: 1433.7 mm from the J1 axis
(published reach 1,400 mm to the wrist centre,
1,499 mm to the tool flange).

## Joints

| Joint | Range (°) | Max speed (°/s) | Continuous torque (N·m) | Peak torque (N·m) | Drive peak at joint (N·m) | Armature (kg·m²) |
|---|---|---|---|---|---|---|
| J1 base | -177.617 to 177.617 | 240 | 222.6 | 568 | 1113 | 6.78 |
| J2 shoulder | -108.862 to 108.862 | 240 | 222.6 | 568 | 1113 | 6.78 |
| J3 elbow | -148.969 to 148.969 | 240 | 222.6 | 568 | 1113 | 6.78 |
| J4 forearm roll | -177.617 to 177.617 | 336 | 73.9 | 204 | 351 | 2.87 |
| J5 wrist bend | -126.051 to 126.051 | 360 | 40 | 82 | 445 | 1.21 |
| J6 flange roll | -177.617 to 177.617 | 360 | 22.4 | 54 | 112 | 0.11 |

Peak torque (`effort_peak`, URDF `effort`) is what the gearbox passes to the arm. Drive peak at joint
(`drive_peak_torque_at_joint`, MuJoCo `forcerange`) is the motor's peak torque x the gear ratio: it also pays for
accelerating the motor's own rotor (`armature`). No gearbox efficiency, drag or speed derating applied. The Rosie 1400 J4 drive is set up with a motor torque limit that protects its gear unit, so its figure is that limit x the ratio. URDF `velocity` is the max speed. Joint damping is nominal
(not identified on hardware). Payload: 15 kg rated, 23.4 kg peak
(published ratings, with the J4 drive upgrade (J4 speed, torque and armature and the link_3 mass in this kit are for that drive)). Repeatability ±0.05 mm. Robot mass 77 kg.

## Links

| Link | Part of the robot | Mass (kg) | Visual triangles | Collision pieces |
|---|---|---|---|---|
| `base_link` | base | 22.256 | 12,768 | 10 |
| `link_1` | turret (J1) | 20.242 | 14,056 | 10 |
| `link_2` | upper arm (J2) | 10.937 | 14,102 | 3 |
| `link_3` | elbow (J3) | 14.468 | 11,076 | 10 |
| `link_4` | forearm (J4) | 7.914 | 9,704 | 3 |
| `link_5` | wrist (J5) | 0.811 | 2,808 | 10 |
| `link_6` | tool flange (J6) | 0.650 | 26,126 | 10 |

Inertials (mass, centre of mass, full inertia tensor in the link frame) come from the robot's CAD mass properties.

## Acceleration, jerk and motion profiles

| Joint | Derived max acceleration (°/s²) | Benchmark, 1 kg (°/s²) | Benchmark, rated payload (°/s²) |
|---|---|---|---|
| J1 | 478 | 5,074 | 3,691 |
| J2 | 161 | 2,777 | 2,659 |
| J3 | 1,314 | 3,803 | 1,425 |
| J4 | 3,291 | (does not move) | (does not move) |
| J5 | 2,281 | 13,106 | 5,238 |
| J6 | 17,887 | 41,729 | 24,511 |

- Derived max acceleration (`joints[].derived_max_acceleration`, rad/s²): at the stretched pose with
  15 kg. Peak-torque acceleration of one joint from standstill with the others held, from this file's own data: (effort_peak - |gravity torque|) / (armature + inertia about the joint axis of every link it moves, plus the payload), the weaker direction against gravity. Conservative: effort_peak is the torque the gearbox passes to the arm, while the motor accelerates its own rotor (most of armature) ahead of the gearbox with its own torque, up to drive_peak_torque_at_joint (the MuJoCo force limit). Torque falling with speed and friction are not included.
  `motion.derived_max_acceleration` also gives it at the `work` pose with 1 kg.
- Benchmark accelerations (`benchmarks.acceleration`): what the published cycle times use. 80 % of each joint's peak-torque acceleration with the payload, the lowest over the four cycle poses, from the drive model (motor peak torque through the gearbox ratio and efficiency, less drag, accelerating the motor's own inertia and the arm; capped by the gearbox start/stop torque on the arm).
- RosieOS simulator model settings (software planning settings, not ratings): rosie_1400_v3, the RosieOS simulator's Rosie 1400 cell model: J1 5, J2 5, J3 5, J4 10, J5 10, J6 10 rad/s² (planning settings for programs in that simulated cell, not a rating).
- Jerk: No RosieOS robot description states a per-joint jerk limit. The planner bounds jerk at 2 x the acceleration setting / 0.2 s as a smoothness backstop (planner_jerk_bound_rad_s3, for the settings above); jog, stop ramps, Move-to and Cartesian steps are jerk-limited by the cell's machine-level jog jerk where one is set.
- Profiles: RosieOS's planner outputs each program as joint-space cubic splines: free-space moves as clamped cubic B-splines (acceleration continuous), weld moves as cubic Hermite curves through the weld path. It optimises them inside each joint's velocity and acceleration settings with a jerk bound of 2 x the acceleration setting / 0.2 s. The real-time core plays them as planned, without further smoothing. Jog, stop ramps, Move-to and Cartesian steps are jerk-limited (double-S, every joint on one shared progress) where the cell sets a machine jog jerk, and acceleration-limited otherwise (the local simulator). A single-joint move by an angle is a trapezoid, or a triangle when it is too short to reach speed.

## Benchmark cycle

The published cycle times (0.803 s with 1 kg, 0.845 s at rated payload,
https://advancedmetalresearch.com/rosie#specs) are this cycle, from the model that computed them (0.8029 s and
0.8453 s unrounded). `robot.json` `benchmarks` holds every condition.

- Path: The tool points straight down (tool0 z = -z of base_link) and keeps its heading. Pick point A and place point B are 305 mm apart along y, centred 350 mm from the J1 axis on +x; the TCP lifts 25 mm at each. Sequence: A_low -> A_high -> B_high -> B_low -> B_high -> A_high -> A_low. TCP points in `base_link` (m): A low [0.35, -0.1525, 0.0755], A high [0.35, -0.1525, 0.1005],
  B high [0.35, 0.1525, 0.1005], B low [0.35, 0.1525, 0.0755]; the low points are 104.9 mm above the floor.
- TCP: [0.0, 0.0, 0.05] m in `tool0` (on the tool axis, 50 mm out from the flange face).
- Payload: Everything on the flange (gripper and part) as one body: 1 kg for the 1 kg cycle, the rated payload for the rated cycle. No other tool mass. CoM at [0.05, 0.0, 0.1] m in `tool0`; inertia of a
  uniform 100 mm cube about its CoM: I = m s^2 / 6 = 0.001667 kg m^2 per kg on each axis. AMR defines no standard gripper or tool; this payload is the benchmark's definition.
- Moves: Six joint-space point-to-point moves, each from rest to rest. All joints start and stop together. No blending, dwell or gripper time.
- Profile: Per segment, each joint follows a trapezoid at its own benchmark acceleration (a triangle if it cannot reach speed). The segment takes the slowest joint's time at max_velocity; the other joints lower their cruise speed to finish with it. Speeds: `joints[].max_velocity`.
- Segment times with 1 kg (s): 0.0790, 0.2435, 0.0790, 0.0790, 0.2435, 0.0790.

`benchmark/rosie_1400_cycle_1kg.csv` (and `.json`) is that cycle with 1 kg, sampled every 1 ms:
`t_s`, `q1..q6` (rad), `qd1..qd6` (rad/s), `qdd1..qdd6` (rad/s²) and the TCP in `base_link` (m), 0.8029 s
long. Following it in this kit's MuJoCo model needs at most 0.77 x the force limit
(`drive_peak_torque_at_joint`) on any joint (`benchmarks.kit_model_check`).

- The cycle has no corner blending, dwell or gripper time, so it is slower than a blended cycle over the same points.
- The rated-payload time is computed at 15.00 kg.
- J4 does not move: every cycle pose has J4 = 0 and J6 = -J1 (the tool keeps its heading along base x).

## IK reference cases

`ik_cases.json` has 12 cases solved by RosieOS's own IK (RosieOS Cartesian resolver: robot-v4-cartesiand --resolve-only, operation "move"; source: RosieOS 6d1fd9e4 motion-server/v1/src/robot_v4_cartesian_cli.hpp, motion-server/v1/src/cartesian_resolver.hpp, motion-server/v1/src/robot_v4_cartesian_nats_protocol.hpp). Each
gives a seed pose, the straight Cartesian move RosieOS resolved from it, the `tool0` target and the solution RosieOS reached
(`expected`), plus all 25 solutions inside the joint limits across the cases (`all_solutions`). Every solution
reproduces its target with this kit's FK within 1e-9. The file is self-contained (it carries the kinematic chain), and
its `solver` block states the method and how it relates to RosieOS's robot descriptions. Online copy: https://advancedmetalresearch.com/assets/sim/1400/ik_cases.json

```python
import json
import numpy as np

doc = json.load(open("ik_cases.json"))


def rot(axis, q):
    x, y, z = axis
    c, s, t = np.cos(q), np.sin(q), 1 - np.cos(q)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def fk(q):
    T = np.eye(4)
    for j, qi in zip(doc["chain"]["joints"], q):
        A = np.eye(4)
        A[:3, 3], A[:3, :3] = j["origin_xyz"], rot(j["axis"], qi)
        T = T @ A
    return T @ np.diag([1, -1, -1, 1])                 # tool0: link_6 turned 180 deg about x


worst = 0.0
for case in doc["cases"]:
    T = fk(case["expected"]["q"])
    worst = max(worst, np.abs(T[:3, 3] - case["target"]["xyz"]).max(),
                np.abs(T[:3, :3] - case["target"]["rotation"]).max())
print(len(doc["cases"]), "cases, worst error", worst)    # within doc["tolerance"]
```

Branch flags:

- `shoulder`: front: J1 points the arm plane toward the wrist centre (cos(J1 - atan2(y_w, x_w)) > 0, wrist centre = J5 origin in base_link); back: the arm reaches over the J1 axis (J1 turned 180 deg from that).
- `elbow`: up: sin(J3 - J3_straight) > 0, where J3_straight = -atan2(Lf, a3) is the J3 angle at which the forearm lines up with the upper arm (Lf = J4 origin x + J5 origin x, the J3 to wrist centre distance along the forearm; a3 = J4 origin z, the elbow offset; from joints[].origin_xyz); down: the other side.
- `wrist`: nonflip: cos(J5) > 0; flip: cos(J5) < 0 (J4 + 180 deg, J5 -> 180 deg - J5, J6 + 180 deg). The wrist is singular at J5 = +/-90 deg, where the J4 and J6 axes line up.
- Every solution inside the joint limits is listed, including angles 360 deg apart where a joint's range allows both.

## GLB

`glb/rosie_1400.glb` is the visual geometry as one glTF 2.0 binary for three.js, Babylon.js, `<model-viewer>`, Blender or a
game engine. Nodes follow the joint chain (`base_footprint` > `base_link` > `link_1` ... `link_6` > `tool0`); each link
node sits on its joint origin, so rotating it about its joint axis (`extras.axis`, in the node's own frame) by the joint
angle moves the robot. glTF is Y-up: the root node turns this kit's Z-up frames upright. Online copy: https://advancedmetalresearch.com/assets/sim/1400/glb/rosie_1400.glb

## Quickstart

### PyBullet

```bash
pip install pybullet numpy
python examples/pybullet_demo.py
```

```python
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
```

### MuJoCo

```bash
pip install mujoco numpy
python examples/mujoco_demo.py        # --view opens the viewer
```

```python
import os
import sys

import mujoco
import numpy as np

KIT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
model = mujoco.MjModel.from_xml_path(os.path.join(KIT, "mjcf", "rosie_1400.xml"))
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
```

### ROS 2 (RViz)

Copy or link this folder into a workspace as the package `rosie_1400_description`, then:

```bash
cd ~/ros2_ws && colcon build --packages-select rosie_1400_description && source install/setup.bash
ros2 launch rosie_1400_description display.launch.py
```

Or publish the description yourself:

```bash
sed 's|filename="../meshes/|filename="package://rosie_1400_description/meshes/|' urdf/rosie_1400.urdf > /tmp/rosie_1400.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/rosie_1400.urdf)"
```

For MoveIt, the SRDF has the `manipulator` group (`base_link` to `tool0`), the named poses and the disabled
collision pairs.

### Kinematics in plain Python

```bash
pip install numpy
python examples/fk_ik.py
```

```python
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
```

### RosieOS

RosieOS, the controller that runs every Rosie, has its own simulated cell: see
[Run everything in simulation](https://advancedmetalresearch.com/docs/guides/run-in-simulation) and
[For AI agents](https://advancedmetalresearch.com/docs/get-started/for-ai-agents). The joint names, axes and zero pose here are the
same as in RosieOS's robot descriptions, so joint values move between the two unchanged.

## Geometry

Per link, every part except fasteners, pulleys and belts fused into one solid. The structural parts (housings, arms, brackets, base, flange) keep their original CAD surfaces (the same planes, cylinders, cones, tori and B-splines), so faces, edges and hole axes can be picked and dimensioned. Motors and gearboxes are plain envelopes of the same outer size and position: stepped boxes and cylinders, without connectors or detail. Bolt holes, counterbores, openings into the inside and gaps around the motors are capped flush with the surfaces around them, open channels get a cover that follows their rim, and everything inside is solid (no internal parts or voids). The base mounting face with its holes and bore, and the tool flange, are unchanged. Visual meshes are this solid, tessellated; collision meshes are convex decompositions of it.

Fidelity against the full CAD: at least 44 % of every link's surface is the original CAD
surface (worst link), up to 44 % is motor and gearbox envelope, and the rest is flush caps and
covers over holes, openings and gaps. No exterior material of the structural parts was removed
(0.00 %), and link bounding boxes match the CAD (motors and gearboxes as their envelopes) within
0.00 mm.
The STEP is analytic B-rep: 1,413 faces
(675 plane, 456 cylinder, 148 cone, 121 bspline, 12 torus, 1 extrusion).

## Notes

- Published reach is 1,400 mm; the geometry stretches to 1,433.7 mm (J1 axis to the wrist centre at J2 = 90 deg, J3 = -90 deg).
- Joint limits are those of RosieOS's robot description of the physical Rosie 1400 arm: +/-3.1, +/-1.9, +/-2.6, +/-3.1, +/-2.2 and +/-3.1 rad. That description turns J2 about -y and J6 about -z (the same motion with the opposite J2 and J6 signs) and has the same geometry as this kit; its ranges are symmetric, so they are the same in this kit's joint coordinates.
- RosieOS's simulator model rosie_1400_v3 has narrower J3, J5 and J6 ranges and puts the flange 93.3 mm from J5 (an earlier wrist); this kit has 99.3 mm, as built.
- Mass properties are as built (77.0 kg) except link_3, which includes the J4 drive upgrade: 0.25 kg heavier, 77.28 kg in total.

## Licence

Free to use for simulation and integration. See `LICENSE.txt`.
