# Rosie robot simulation and CAD kits

URDF, MuJoCo MJCF, SRDF, STEP, GLB and STL meshes, `robot.json` (kinematics, joint limits, speeds, torques, payloads and mass properties) and quickstart code for the Rosie 600, 1000 and 1400 six-axis industrial robots from [Advanced Metal Research](https://advancedmetalresearch.com/).

This is a mirror of https://advancedmetalresearch.com/assets/sim/ and https://advancedmetalresearch.com/assets/cad/, for environments that can reach GitHub but not advancedmetalresearch.com. It is synced automatically from the website; the website is the canonical source. Guide for agents: https://advancedmetalresearch.com/agents.md.

## Quick start

```sh
curl -fsSLO https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/rosie-1000-sim-kit.zip
```

```python
import json, urllib.request
index = json.load(urllib.request.urlopen("https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/index.json"))
kit = next(k for k in index["kits"] if k["model"] == "rosie_1000")
robot = json.load(urllib.request.urlopen(kit["robot_json"]))
```

Every URL in `sim/index.json` points at this mirror; `canonical` gives the website's copy.

## Robots

| Robot | Reach | Rated payload | Mass | Kit ZIP | robot.json | URDF |
|---|---|---|---|---|---|---|
| Rosie 600 | 0.6 m | 9 kg | 29.3 kg | [683 KB](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/rosie-600-sim-kit.zip) | [robot.json](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/600/robot.json) | [URDF](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/600/urdf/rosie_600.urdf) |
| Rosie 1000 | 1 m | 7.4 kg | 38.6 kg | [725 KB](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/rosie-1000-sim-kit.zip) | [robot.json](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/1000/robot.json) | [URDF](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/1000/urdf/rosie_1000.urdf) |
| Rosie 1400 | 1.4 m | 15 kg | 77 kg | [3.6 MB](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/rosie-1400-sim-kit.zip) | [robot.json](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/1400/robot.json) | [URDF](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/1400/urdf/rosie_1400.urdf) |

## Layout

- `sim/index.json`: every kit and every file URL
- `sim/rosie-<model>-sim-kit.zip`: one robot, everything
- `sim/<model>/`: the unzipped kit (`robot.json`, `urdf/`, `mjcf/`, `srdf/`, `step/`, `glb/`, `meshes/`, `examples/`, a ROS 2 package)
- `cad/`: Rosie 1400 tool flange and base mounting interfaces (STEP, PDF, JSON)

Units are metres, kilograms and radians; the STEP is in millimetres. URDF and MJCF mesh paths are relative, so they load from their URL or from the unzipped kit.

## Licence

Free to use for simulation and integration, including commercial work. See each kit's `LICENSE.txt` (for example [sim/1000/LICENSE.txt](https://raw.githubusercontent.com/advanced-metal-research/rosie-sim-kits/main/sim/1000/LICENSE.txt)).

Contact: generalcontact@advancedmetalresearch.com
