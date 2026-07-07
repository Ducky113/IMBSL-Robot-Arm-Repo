# NCKU SO-101 Pick-and-Place

Overhead CV + SO-101 arm control for a table pick-and-place workflow. See [docs/system-build-plan.md](docs/system-build-plan.md) for the full roadmap.

## Layout

```
├── config/           # Homography, HSV, robot JSON configs
├── cv_service/       # HSV blob detection + table coordinates
├── mission/          # Table-frame IK motion (goto x,y,z)
├── scripts/          # Homography calibration utilities
├── data/
│   ├── images/       # Test images for CV
│   ├── calib/        # Calibration photos
│   └── datasets/     # Recorded demos (gitignored)
├── assets/           # SO-101 URDF + meshes (subset of SO-ARM100)
├── lerobot/          # Vendored LeRobot with SO-101 gamepad patches
├── patches/          # Patch file for lerobot changes
├── docs/             # USAGE.md + system-build-plan.md
└── setup.sh          # One-shot install (run after cloning)
```

## Quick start

After cloning from GitHub, from this `Main/` directory:

```bash
./setup.sh
conda activate lerobot        # or: source .venv/bin/activate
```

`setup.sh` builds the environment, installs the vendored LeRobot with the
`gamepad`, `kinematics`, and `feetech` extras, adds `placo>=0.9.16`, and verifies
the install. Then edit `robot.port` in `config/robot/*.json` if not on
`/dev/ttyACM0`.

| Task | Command |
|------|---------|
| Gamepad teleop | `./config/robot/run_so101_teleop.sh` |
| Table-frame moves | `./config/robot/run_goto_table.sh --home` |
| CV detection | `PYTHONPATH=. python -m cv_service -i data/images/M1.jpg` |
| Pick detected box | `./config/robot/run_cv_pick.sh --image data/images/M1.jpg` |
| Check homography | `python scripts/check_homography.py` |

Full how-to for every task: **[docs/USAGE.md](docs/USAGE.md)**.
Robot-specific details: [config/robot/README.md](config/robot/README.md).

## Coordinate frames

| Frame | Origin | Notes |
|-------|--------|-------|
| **CV / homography** | Bottom-left table corner | `config/table_homography.json` |
| **Robot table** | Mid-joint home EE pose | `mission/table_moves.py` |

Mapping between CV table coords and robot table coords is not yet implemented.
