# AGENTS.md

## Scope

Focus maintenance work on:

- `robot12idb.py`: beamline-specific UR3 and UR5 behavior.
- `common/`: shared robot, camera, dashboard, geometry, and pipette support used by `robot12idb.py`.
- `scripts/script_ur3_12idb.py`: high-level UR3 alignment and operating procedures.
- `ini/UR3.ini` and `ini/UR3path.ini`: live UR3 calibration and path data.
- `12idUR/`: EPICS IOC that will expose selected `robot12idb.py` behavior through pyDevSup.

RoboDK, `ur_rest_node.py`, old UI code, and other historical utilities are outside the normal scope unless a task explicitly concerns them. Avoid changing `urxe/` or `rtde/` unless the issue is in the low-level driver itself.

## 12idUR IOC

`12idUR` is a synApps-style IOC and the starting point for the future UR3 EPICS interface. It currently contains a verified pyDevSup hello-world integration:

- `12idUR/configure/RELEASE` defines `PYDEVSUP=$(TOP)/../pyDevSup`.
- `12idUR/12idURApp/src/Makefile` includes pyDevSup's Python configuration and links `pyDevSup$(PY_LD_VER)` into `ioc12idURLinux.dbd` for `rhel9-x86_64`.
- `12idUR/iocBoot/ioc12idUR/st.cmd.Linux` adds the boot directory to embedded Python's `sys.path` and loads `hello_world.db` before `iocInit()`.
- `hello_world.db` defines the `12idUR:Hello` `bo` record using `DTYP="Python Device"` and `OUT="@hello_world"`.
- `hello_world.py` supplies the direct pyDevSup support object. Writing `1` to `12idUR:Hello` has been runtime-tested and prints `hello world` in the IOC console.
- pyDevSup and the IOC have both been built successfully for `EPICS_HOST_ARCH=rhel9-x86_64`. pyDevSup must be built before rebuilding `12idUR` because the IOC consumes its installed DBD, library, and Python modules.
- The vendored pyDevSup configuration uses Python 3.9 and currently has target configuration for `rhel9-x86_64`, not `rhel9-x86_64-debug`.

For future robot support:

- Prefer direct pyDevSup device support with one shared driver instance rather than `devsup.ptable` or one independent robot object per record.
- Keep IOC startup disconnected. Robot construction is a hardware operation; connect only through an explicit command PV.
- Serialize robot, camera, TCP, dashboard, and gripper operations. Reject overlapping commands rather than allowing concurrent hardware access.
- Use asynchronous command records and explicit command state/completion identifiers so clients do not infer completion from a command `bo` value.
- Initially expose bounded, reviewed robot primitives. Do not expose arbitrary Cartesian poses, unrestricted relative motion, or complete pick-and-place orchestration.
- Keep multi-step procedures, trajectory decisions, retries, and alignment tolerances in an external Channel Access client unless a concrete IOC requirement justifies moving them into device support.
- Apply all hardware, geometry, unit, motion-path, TCP-restoration, and configuration constraints in this document to IOC code. An IOC process boundary does not make an operation safer.

## Architecture

The main inheritance chain is:

```text
PyQt5.QtCore.QObject
└── common.robUR.UR
    └── common.robUR.UR_grip
        └── common.robUR.UR_cam_grip
            ├── robot12idb.UR3
            └── robot12idb.UR5
```

- `common.robUR.UR` selects the `urxe` or `rtde` backend, connects to the controller, sets TCP/payload, creates the dashboard and optional camera/gripper, and wraps motion methods with safety-mode checks.
- `UR_grip` adds gripper behavior and Qt signals.
- `UR_cam_grip` adds camera-frame geometry, camera-relative motion, and AprilTag/QR alignment.
- `robot12idb.UR3` adds 12-ID-B magazine/stage geometry, frame indexing, transfer paths, and sample workflows.
- `robot12idb.UR5` adds 12-ID-C tool changing, pipette support, and ptychography sample workflows.
- Keep generic reusable motion behavior in `common/robUR.py`, beamline-specific primitives in `robot12idb.py`, and complete sequenced procedures in `scripts/script_ur3_12idb.py`.

Relevant support modules:

- `common/m3d.py`: the required math3d import point and math3d 4.x compatibility layer.
- `common/urcamera.py`: IP/USB capture and marker detection.
- `common/urdashboard.py`: UR dashboard connection and protective-stop commands.
- `common/tc_pipet.py`: Tricontinent pump protocol.
- `common/urmodes.py`: dependency-free robot and safety enums.
- `common/utils.py`: magazine index conversion and vector helpers.

## Hardware Safety

Treat this as live robot-control software. Do not perform hardware operations without explicit user approval and an operator present.

Hardware operations include:

- Constructing `UR3`, `UR5`, or `common.robUR.UR`.
- Running `test.py`, `TweakRobot.py`, or anything in `scripts/`.
- Calling robot motion, gripper, pipette, dashboard, TCP, payload, digital-output, camera-centering, or protective-stop methods.
- Running URScript programs from `urscripts/`.

Construction is not a harmless smoke test. It connects to a controller from `list_of_robots.json`, sets TCP and payload, opens dashboard and optional camera connections, and may attempt to clear a protective stop.

Before any approved physical test, require:

- Correct robot identity and configuration.
- Confirmed active TCP, payload, and attached tool.
- Cleared workspace and reviewed waypoints.
- Reduced speed for first execution.
- An operator with access to the emergency stop.

The safety-mode checks in `common/robUR.py` are only controller-state gates. They do not provide collision avoidance, workspace validation, tool detection, or sample detection.

## Geometry And Units

- A UR pose is `[x, y, z, rx, ry, rz]`.
- Cartesian positions are in metres.
- Pose rotation vectors and joint values are in radians.
- Public helpers such as `rotx`, `roty`, `rotz`, `rotj`, `rotate`, and Euler helpers generally accept degrees and convert internally.
- UR3 base axes are +X outboard and +Y along the X-ray direction.
- `translate` and `mvr2x/y/z` operate in the robot base frame.
- `translate_tool` and `mvr2x/y/zTCP` operate in the active TCP frame.
- Camera-frame operations temporarily switch to `camtcp`. Preserve restoration of the original/tool TCP, including on exceptions.
- INI pose and path vectors use metres/radians, while values such as `vert_offset`, `vert_samZ`, `vert_magZ`, `trans_X`, `magXgap`, and `magYgap` are millimetres and require division by 1000.
- Pipette volume APIs use microlitres; the full configured stroke is 1600 half-steps for 200 uL.

Do not silently change coordinate frames, units, waypoint ordering, TCP state, acceleration, velocity, or blend radius. These are behavior and safety changes.

## Motion Invariants

- Insert or remove samples vertically only at a known magazine or stage XY position.
- Perform long transport from elevated waypoints using the configured intermediate path.
- Preserve the ordering of `path1`, `path2`, `path3`, `middl_q`, and stage/magazine up/down waypoints.
- In remote-heater mode (`samZ == "stv"`), preserve the lateral +Y clearance move before vertical or long-distance motion.
- `whereisgripper()` is a heuristic based largely on position. Do not treat it as an independent hardware sensor.
- Gripper position is also only an inference of sample presence.
- Tool engagement on UR5 is software state plus digital output 0, not sensor-confirmed state.

For motion changes, statically inspect every generated pose and both directions of each path. A safe outbound path does not imply that reversing or partially entering it is safe.

## Configuration

Treat these as live operational data and edit them only when explicitly requested:

- `ini/UR3.ini`
- `ini/UR3path.ini`
- `list_of_robots.json`
- `urscripts/`

`UR3.readini()` creates attributes dynamically from lines formatted as:

```text
description, key : value(s)
```

Single numeric values become floats, non-numeric single values remain strings, and multiple values become lists of floats. `writeini()` rewrites numeric values to four decimal places, so do not use it casually for calibration updates.

## Import Behavior

`common/__init__.py` eagerly imports most modules in `common/`. Even a narrow import such as `from common import m3d` can therefore require PyQt5, OpenCV, SciPy, URX, camera dependencies, and network support.

- Always import math3d through `from common import m3d`. Its compatibility patch intentionally affects the shared `math3d.Transform` class used by this project and URX.
- `common/robUR.py` reads `urscripts/checkdistance.script` at import time.
- `common/ursocket.py` performs network discovery at import time through `8.8.8.8`.
- `robot12idb.py` optionally imports EPICS beamline integration and suppresses failures.
- Do not use importing the complete stack as the first validation step in an isolated environment.

## Editing Conventions

- Make the smallest change that solves the requested problem.
- Preserve local legacy style and avoid unrelated cleanup or reformatting.
- Use four spaces and double quotes for new isolated Python code, consistent with `pyproject.toml`.
- Keep compatibility behavior centralized in `common/m3d.py` and safety/robot enums in `common/urmodes.py`.
- Avoid new broad `except` blocks. Existing broad exception handling is not a pattern to extend.
- Add comments only when they explain a non-obvious safety, frame, unit, compatibility, or hardware constraint.
- Do not change calibrated numbers or motion speeds as incidental cleanup.
- Do not run repository-wide `ruff --fix` or `ruff format`; the legacy files do not fully conform and this creates unrelated churn.

## Validation

Prefer validation that cannot contact or move hardware.

Use `12idUR/python/.venv/bin/python` for Python commands that require project dependencies. The system Python does not contain all required packages, including OpenCV.

For syntax validation without importing modules:

```sh
python - <<'PY'
import ast
from pathlib import Path

for path in [Path("robot12idb.py"), *Path("common").glob("*.py")]:
    ast.parse(path.read_text(), filename=str(path))
PY
```

Run lint checks only on changed files where practical:

```sh
ruff check <changed-python-files>
ruff format --check <changed-python-files>
git diff --check
```

The existing safe unit tests are parser-only:

```sh
python -m pytest tests/test_urmon_parser.py
```

Do not run bare `pytest`. Root `test.py` is an unmarked interactive hardware diagnostic that constructs a live `UR5` during test collection. The configured `hardware` marker does not protect against it. Do not use `make test`; its Docker/WEI workflow is stale and unrelated to focused robot validation.

There are currently no automated tests for `robot12idb.py`, motion paths, cameras, grippers, dashboard behavior, INI persistence, tool changing, or pipette hardware. State that limitation explicitly and never claim physical behavior is validated by the parser tests.

## Historical Inconsistencies

The README, `setup.py`, `pyproject.toml`, Makefile, Docker configuration, and REST node contain stale or conflicting information. Do not resolve these inconsistencies unless the task explicitly includes them. In particular, README examples for UR3 alignment may still show procedures as members of `robot12idb`; the active procedures are in `scripts/script_ur3_12idb.py`.
