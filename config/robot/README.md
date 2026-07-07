# SO-101 robot configs

Gamepad teleop and table-frame motion for the SO-101 follower. Project setup: [../../README.md](../../README.md).

## Setup

```bash
conda activate lerobot
cd lerobot && pip install -e ".[gamepad,kinematics,feetech]"
pip install 'placo>=0.9.16'
```

If `liburdfdom_sensor.so.4.0` is missing, run the `placo>=0.9.16` line again.

Edit `robot.port` in the JSON files if not on `/dev/ttyACM0`.

## Gamepad teleop

```bash
./config/robot/run_so101_teleop.sh          # teleop test
./config/robot/run_so101_teleop.sh record   # record demos → data/datasets/
```

| Input | Action |
|-------|--------|
| **Hold RB** | Enable teleop |
| RB + left stick | Move X–Y |
| RB + right stick (vertical) | Move Z |
| **RB + LB** + left stick (horizontal) | Wrist roll |
| **LT** / **RT** | Open / close gripper |

Optional overrides in `env_config_so101_gamepad.json` under `teleop`:

```json
"gripper_open_button": 7,
"gripper_close_button": 6,
"gripper_open_trigger_axis": 2,
"gripper_close_trigger_axis": 5
```

Leave `gripper_*_button` as `null` to use LT/RT triggers (default).

## Table-frame motion

Origin = end-effector at mid-joint home (0° on all arm joints). See `table_frame.json`.

```bash
./config/robot/run_goto_table.sh --home
./config/robot/run_goto_table.sh --x 0.2 --y 0.0
./config/robot/run_goto_table.sh --interactive
```

## Config files

| File | Purpose |
|------|---------|
| `table_frame.json` | Table-frame IK motion |
| `env_config_so101_gamepad.json` | Gamepad teleop (no recording) |
| `env_config_so101_record.json` | Record episodes with camera |
