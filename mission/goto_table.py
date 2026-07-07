#!/usr/bin/env python3
"""Move SO-101 to table-frame (x, y, z) coordinates.

Table frame: origin at mid-joint home pose (0,0,0).

Examples:
  python goto_table.py --x 0.2 --y 0.0
  python goto_table.py --x 0.15 --y 0.1 --z 0.05
  python goto_table.py where
  python goto_table.py --interactive
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running from Main/ or Main/mission/
_MAIN = Path(__file__).resolve().parents[1]
if str(_MAIN / "lerobot" / "src") not in sys.path:
    sys.path.insert(0, str(_MAIN / "lerobot" / "src"))

from mission.table_moves import TableMotionController, load_table_config  # noqa: E402


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SO-101 table-frame motion")
    p.add_argument(
        "--config",
        type=Path,
        default=_MAIN / "config/robot/table_frame.json",
        help="Path to table_frame.json",
    )
    p.add_argument("--x", type=float, help="Table x (m), offset from home pose")
    p.add_argument("--y", type=float, help="Table y (m), offset from home pose")
    p.add_argument("--z", type=float, help="Table z (m), offset from home pose; default from config")
    p.add_argument("--wz-deg", type=float, help="Wrist roll in degrees (rotvec z); default: keep current")
    p.add_argument("--gripper", type=float, help="Gripper 0-100")
    p.add_argument("--duration", type=float, help="Move duration in seconds")
    p.add_argument("--where", action="store_true", help="Print current table-frame pose and exit")
    p.add_argument("--interactive", "-i", action="store_true", help="REPL: enter x y [z] per line")
    p.add_argument("--no-calibrate", action="store_true", help="Skip calibration prompt on connect")
    p.add_argument("--no-home", action="store_true", help="Do not move to mid-joint home on connect")
    p.add_argument("--home", action="store_true", help="Only move to mid-joint home and exit")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    cfg = load_table_config(args.config)
    ctrl = TableMotionController(cfg)

    try:
        if args.home:
            home_on_connect = True
        elif args.no_home:
            home_on_connect = False
        else:
            home_on_connect = None

        ctrl.connect(calibrate=not args.no_calibrate, home=home_on_connect)

        if args.home:
            print("At mid-joint home (table origin).")
            pose = ctrl.get_table_pose()
            print(
                f"table pose: x={pose['x_m']:.4f} y={pose['y_m']:.4f} z={pose['z_m']:.4f} "
                f"gripper={pose['gripper']:.1f}"
            )
            return 0

        if args.where:
            pose = ctrl.get_table_pose()
            print(
                f"table pose: x={pose['x_m']:.4f} y={pose['y_m']:.4f} z={pose['z_m']:.4f} "
                f"gripper={pose['gripper']:.1f}"
            )
            return 0

        if args.interactive:
            print("Table frame: origin at mid-joint home. Enter: x y [z]  (or 'q')")
            while True:
                try:
                    line = input("> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if not line or line.lower() in {"q", "quit", "exit"}:
                    break
                parts = line.split()
                if len(parts) < 2:
                    print("Need at least x and y")
                    continue
                x_m, y_m = float(parts[0]), float(parts[1])
                z_m = float(parts[2]) if len(parts) > 2 else None
                target = ctrl.move_to_table(x_m, y_m, z_m, duration_s=args.duration)
                print(f"  -> x={target['x_m']:.4f} y={target['y_m']:.4f} z={target['z_m']:.4f}")
            return 0

        if args.x is None or args.y is None:
            print("Provide --x and --y, or use --where / --interactive", file=sys.stderr)
            return 2

        wz_rad = None
        if args.wz_deg is not None:
            import math

            wz_rad = math.radians(args.wz_deg)

        target = ctrl.move_to_table(
            args.x,
            args.y,
            args.z,
            wz_rad=wz_rad,
            gripper=args.gripper,
            duration_s=args.duration,
        )
        print(f"Moved to x={target['x_m']:.4f} y={target['y_m']:.4f} z={target['z_m']:.4f}")
        return 0
    finally:
        if ctrl.follower.is_connected:
            ctrl.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
