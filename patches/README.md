# LeRobot patches

`lerobot-so101-gamepad.patch` contains local changes required for SO-101 gamepad teleop with wrist roll (5-DOF EE deltas via `gym_manipulator`).

## Modified files

| File | Change |
|------|--------|
| `teleoperators/gamepad/gamepad_utils.py` | RB-gated teleop, wrist roll (RB+LB + left stick) |
| `teleoperators/gamepad/teleop_gamepad.py` | `delta_wz` in action |
| `teleoperators/gamepad/configuration_gamepad.py` | `wz_step_size` |
| `processor/delta_action_processor.py` | `delta_wz` → `target_wz` |
| `processor/hil_processor.py` | 5-element intervention tensor |
| `rl/gym_manipulator.py` | Action shape + neutral action |
| `utils/utils.py` | Skip TTS if `spd-say` missing |
| `policies/groot/groot_n1.py` | Import fix (`@strict` removed) |

## Re-apply after upstream upgrade

```bash
cd lerobot
git apply --check ../patches/lerobot-so101-gamepad.patch   # dry run
git apply ../patches/lerobot-so101-gamepad.patch
```

The vendored `lerobot/` tree in this repo already includes these patches applied.
