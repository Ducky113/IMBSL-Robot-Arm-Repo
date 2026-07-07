# Usage guide

How to run every part of the NCKU SO-101 pick-and-place project. For the
6-week roadmap and architecture, see [system-build-plan.md](system-build-plan.md).

All commands are run from the project's `Main/` directory unless noted.

## Contents

- [1. Install](#1-install)
- [2. Activate the environment](#2-activate-the-environment)
- [3. Connect the hardware](#3-connect-the-hardware)
- [4. Gamepad teleop](#4-gamepad-teleop)
- [5. Record demonstrations](#5-record-demonstrations)
- [6. Table-frame motion (goto x, y, z)](#6-table-frame-motion-goto-x-y-z)
- [7. Gripper open/close test](#7-gripper-openclose-test)
- [8. Camera vision (CV) pipeline](#8-camera-vision-cv-pipeline)
  - [8a. Calibrate the homography](#8a-calibrate-the-homography)
  - [8b. Check the homography](#8b-check-the-homography)
  - [8c. Tune HSV thresholds](#8c-tune-hsv-thresholds)
  - [8d. Detect samples](#8d-detect-samples)
- [9. Troubleshooting](#9-troubleshooting)

---

## 1. Install

After cloning the repo from GitHub:

```bash
cd Main
./setup.sh
```

`setup.sh` creates a Python environment (conda `lerobot`, or a `.venv` if conda
is missing), installs the vendored LeRobot with the `gamepad`, `kinematics`, and
`feetech` extras, then installs `placo>=0.9.16` for inverse kinematics and
verifies everything imports.

Manual equivalent:

```bash
conda create -n lerobot python=3.11
conda activate lerobot
pip install -e "lerobot[gamepad,kinematics,feetech]"
pip install 'placo>=0.9.16'
```

> The GUI HSV tuner (step 8c) needs a full OpenCV build. LeRobot installs
> `opencv-python-headless`, which has no window support. If you need the live
> tuner window, run `pip install opencv-python` in the same environment.

## 2. Activate the environment

Every session:

```bash
conda activate lerobot          # or: source .venv/bin/activate
```

## 3. Connect the hardware

Plug in the SO-101 follower and the gamepad, then find the serial port:

```bash
ls /dev/ttyACM*
```

The robot config files under `config/robot/` default to `/dev/ttyACM0`. If your
arm enumerates as a different port, edit the `port` field in:

- `config/robot/table_frame.json`
- `config/robot/env_config_so101_gamepad.json`
- `config/robot/env_config_so101_record.json`

The follower expects **6 Feetech motors** (IDs 1–5 = arm joints, ID 6 = gripper).
If a motor is missing, assign IDs with:

```bash
cd lerobot
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
```

## 4. Gamepad teleop

```bash
./config/robot/run_so101_teleop.sh
```

Controls:

| Input | Action |
|-------|--------|
| **Hold RB** | Enable teleop (nothing moves unless held) |
| RB + left stick | Move X–Y |
| RB + right stick (vertical) | Move Z |
| **RB + LB** + left stick (horizontal) | Wrist roll |
| **LT** / **RT** | Open / close gripper |
| B / Circle | Exit |
| Y / Triangle | End episode as SUCCESS |
| A / Cross | End episode as FAILURE |
| X / Square | Re-record episode |

Gripper mapping is configurable — see the table and JSON overrides in
[`config/robot/README.md`](../config/robot/README.md).

## 5. Record demonstrations

```bash
./config/robot/run_so101_teleop.sh record
```

Uses `config/robot/env_config_so101_record.json` (front camera enabled, episodes
saved under `data/datasets/`, gitignored). Adjust `num_episodes_to_record`,
`repo_id`, and camera settings in that file.

## 6. Table-frame motion (goto x, y, z)

Move the end-effector to absolute table coordinates. The origin `(0,0,0)` is the
end-effector pose with all arm joints at their calibration midpoint; `x/y/z` are
offsets in metres in the robot-base axes. See `config/robot/table_frame.json`.

```bash
./config/robot/run_goto_table.sh --home              # go to mid-joint home
./config/robot/run_goto_table.sh --x 0.2 --y 0.0     # absolute move
./config/robot/run_goto_table.sh --x 0.15 --y 0.1 --z 0.05 --gripper 100
./config/robot/run_goto_table.sh --where             # print current pose
./config/robot/run_goto_table.sh --interactive       # type "x y [z]" per line
```

Useful flags: `--wz-deg` (wrist roll), `--duration` (seconds), `--no-home`
(skip homing on connect), `--no-calibrate` (skip the calibration prompt).

## 7. Gripper open/close test

Quick standalone check that motor 6 works, without launching teleop. Make sure
no teleop process is holding the serial port first.

```bash
python - <<'PY'
import sys; sys.path.insert(0, "lerobot/src")
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

robot = SO101Follower(SO101FollowerConfig(port="/dev/ttyACM0", id="so101_follower",
                                          use_degrees=True, max_relative_target=None))
robot.connect(calibrate=False)

def move_gripper(target, duration=2.0, fps=30):
    motors = list(robot.bus.motors)
    start = {m: float(robot.get_observation()[f"{m}.pos"]) for m in motors}
    for i in range(1, int(duration*fps)+1):
        a = i/(duration*fps)
        act = {f"{m}.pos": start[m] for m in motors}
        act["gripper.pos"] = start["gripper"] + a*(target-start["gripper"])
        robot.send_action(act); precise_sleep(1/fps)

move_gripper(0)     # 0   = open
move_gripper(100)   # 100 = closed
robot.disconnect()
PY
```

Note the SO-101 convention: **gripper `0` = open, `100` = closed**.

## 8. Camera vision (CV) pipeline

The CV service maps table pixels to real table coordinates (homography) and
finds coloured samples (HSV blob detection). Config lives in `config/`.

### 8a. Calibrate the homography

Edit the four table corners and their pixel locations in
`scripts/calc_homography.py`:

- `Table_C` — real corner positions in metres, order **BL, TL, TR, BR**.
- `Image_C` — the same corners' pixel `(u, v)` in your overhead image.

Then generate the matrix:

```bash
python scripts/calc_homography.py
```

This writes `config/table_homography.json` (world corners, image corners, and
the 3×3 matrix `H`).

### 8b. Check the homography

Put a few known pixel→table measurements in the `TEST_POINTS` list of
`scripts/check_homography.py`, then:

```bash
python scripts/check_homography.py
```

It prints the reprojection error per point. Aim for < 5 mm; 1–2 cm at the edges
is common with a 4-corner fit and lens distortion. If errors are large, re-check
that `TEST_POINTS` and the corners in `calc_homography.py` come from the **same**
image.

### 8c. Tune HSV thresholds

Interactive tuner with a live mask preview (needs a display and full OpenCV — see
the note in step 1):

```bash
PYTHONPATH=. python -m cv_service.tune_hsv -i data/images/M1.jpg
```

Drag the trackbars, press `s` to save to `config/hsv.json`, `q` to quit.

### 8d. Detect samples

Run blob detection and print table coordinates for each detection:

```bash
# single image
PYTHONPATH=. python -m cv_service -i data/images/M1.jpg

# whole folder
PYTHONPATH=. python -m cv_service --image-dir data/images

# JSON output / save an annotated image / show windows
PYTHONPATH=. python -m cv_service -i data/images/M1.jpg --json
PYTHONPATH=. python -m cv_service -i data/images/M1.jpg --save out.jpg
PYTHONPATH=. python -m cv_service -i data/images/M1.jpg --show
```

Detections are filtered to the calibrated table quad and reported as
`(x_cm, y_cm)` in the table frame.

## 9. Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Missing motor IDs: 6` on connect | Gripper not on the bus — check power/cable, or re-assign IDs with `lerobot-setup-motors` (do the gripper last so it gets ID 6). |
| `placo failed to import` | `pip install 'placo>=0.9.16'`; re-run if `liburdfdom_sensor.so.4.0` is reported missing. |
| `termios.error` / port busy | Another process (often teleop) holds the port. Stop it: `pkill -f gym_manipulator`, then retry. |
| `Relative goal position ... clamped` warnings | `max_relative_target` is capping each step for safety. Expected; lower speed or raise the limit in the config if needed. |
| HSV tuner: no window / `cv2.imshow` error | You have `opencv-python-headless`. Install the full build: `pip install opencv-python`. |
| Wrong serial port | `ls /dev/ttyACM*` and update `port` in `config/robot/*.json`. |

For the SO-101 gamepad patches applied to the vendored LeRobot, see
[`patches/README.md`](../patches/README.md).
