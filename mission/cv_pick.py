#!/usr/bin/env python3
"""Detect a box with the CV service and move the SO-101 to pick and place it.

Single shot:
  1. Grab a frame (image file or live camera).
  2. Detect the box inside the calibrated table quad (ignoring the drop zone).
  3. Map its CV table coord -> robot table coord (config/cv_to_robot.json).
  4. Hover ~5 cm above, then optionally descend, close gripper, lift, and place
     it in the predefined drop zone.

Continuous (--loop): every N seconds, look for a box; when one is found, pick it
and place it in the drop zone, then keep watching. Boxes already sitting in the
drop zone are ignored.

Safety: without --go this is a DRY RUN that only prints the target poses.

Examples:
  # Dry run on an image
  python mission/cv_pick.py --image ../data/images/M1.jpg

  # Watch a live camera every 2 s and pick+place each box into the drop zone
  python mission/cv_pick.py --camera 0 --loop --go

  # Solve the CV->robot transform from measured correspondences
  python mission/cv_pick.py --calibrate points.json --out ../config/cv_to_robot.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

# Make the project root and vendored lerobot importable.
_MAIN = Path(__file__).resolve().parents[1]
for _p in (_MAIN, _MAIN / "lerobot" / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import cv2  # noqa: E402

from cv_service.detect import detect_blobs, load_image  # noqa: E402
from cv_service.homography import (  # noqa: E402
    homography_matrix,
    load_homography,
    pixel_to_table,
    workspace_corners_px,
)

DEFAULT_CALIB = _MAIN / "config" / "table_homography.json"
DEFAULT_HSV = _MAIN / "config" / "hsv.json"
DEFAULT_CV2ROBOT = _MAIN / "config" / "cv_to_robot.json"
DEFAULT_PICK_PLACE = _MAIN / "config" / "pick_place.json"
DEFAULT_TABLE_CFG = _MAIN / "config" / "robot" / "table_frame.json"


# --------------------------------------------------------------------------- #
# CV helpers
# --------------------------------------------------------------------------- #
def load_hsv(path: Path):
    if not path.is_file():
        return (0, 80, 80), (180, 255, 255), 500.0
    data = json.loads(path.read_text())
    return (
        tuple(data["hsv_lower"]),
        tuple(data["hsv_upper"]),
        float(data.get("min_area", 500)),
    )


def in_zone(x_m: float, y_m: float, zone: dict | None) -> bool:
    """True if (x, y) in CV metres falls inside a {x:[lo,hi], y:[lo,hi]} zone."""
    if not zone:
        return False
    return (
        zone["x"][0] <= x_m <= zone["x"][1]
        and zone["y"][0] <= y_m <= zone["y"][1]
    )


def detect_target(frame, calib, hsv_path: Path, exclude_zone: dict | None = None):
    """Return (u, v, area, x_cv_m, y_cv_m) for the best detection, or None.

    Blobs whose CV-table centre falls inside `exclude_zone` (the drop area) are
    skipped, so a box already placed is not picked again.
    """
    H = homography_matrix(calib)
    workspace = workspace_corners_px(calib)
    hsv_lower, hsv_upper, min_area = load_hsv(hsv_path)

    blobs, _, _ = detect_blobs(
        frame,
        hsv_lower=hsv_lower,
        hsv_upper=hsv_upper,
        table_corners=workspace,
        min_area=min_area,
    )
    # detect_blobs sorts by area (largest first); take the first non-excluded one.
    for b in blobs:
        x_cv, y_cv = pixel_to_table(b.u, b.v, H)
        if in_zone(x_cv, y_cv, exclude_zone):
            continue
        return b.u, b.v, b.area, x_cv, y_cv
    return None


def open_camera(index: int):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {index}")
    return cap


def read_frame(cap):
    frame = None
    for _ in range(5):  # settle auto-exposure / flush buffer
        ok, frame = cap.read()
    if frame is None:
        raise RuntimeError("Could not read a frame from the camera")
    return frame


# --------------------------------------------------------------------------- #
# CV -> robot transform + task config
# --------------------------------------------------------------------------- #
def load_cv_to_robot(path: Path) -> dict:
    default = {
        "A": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        "pick_z_m": 0.0,
        "hover_m": 0.05,
        "gripper_open": 100.0,
        "gripper_closed": 0.0,
        "calibrated": False,
    }
    if path.is_file():
        default.update(json.loads(path.read_text()))
    return default


def load_pick_place(path: Path) -> dict:
    default = {
        "poll_interval_s": 2.0,
        "drop_zone_cv_m": None,
        "place": {"robot_xy_m": None, "z_m": 0.0, "hover_m": 0.05},
        "return_home_after_place": True,
    }
    if path.is_file():
        default.update(json.loads(path.read_text()))
    return default


def cv_to_robot_xy(x_cv: float, y_cv: float, A) -> tuple[float, float]:
    A = np.asarray(A, dtype=float)
    r = A @ np.array([x_cv, y_cv, 1.0], dtype=float)
    return float(r[0]), float(r[1])


def resolve_place_xy(pick_place: dict, A) -> tuple[float, float]:
    """Robot-frame (x, y) where boxes are placed."""
    place = pick_place.get("place", {})
    if place.get("robot_xy_m"):
        xy = place["robot_xy_m"]
        return float(xy[0]), float(xy[1])
    zone = pick_place.get("drop_zone_cv_m")
    if not zone:
        raise ValueError(
            "Define place.robot_xy_m or drop_zone_cv_m in config/pick_place.json"
        )
    cx = (zone["x"][0] + zone["x"][1]) / 2.0
    cy = (zone["y"][0] + zone["y"][1]) / 2.0
    return cv_to_robot_xy(cx, cy, A)


def solve_affine(cv_pts, robot_pts) -> np.ndarray:
    """Least-squares 2D affine (2x3) mapping cv_pts -> robot_pts. Needs >=3 pts."""
    cv = np.asarray(cv_pts, dtype=float)
    rb = np.asarray(robot_pts, dtype=float)
    if len(cv) < 3:
        raise ValueError("Need at least 3 non-collinear correspondences.")
    n = len(cv)
    M = np.zeros((2 * n, 6))
    b = np.zeros(2 * n)
    for i, (x, y) in enumerate(cv):
        M[2 * i] = [x, y, 1, 0, 0, 0]
        M[2 * i + 1] = [0, 0, 0, x, y, 1]
        b[2 * i] = rb[i, 0]
        b[2 * i + 1] = rb[i, 1]
    sol, *_ = np.linalg.lstsq(M, b, rcond=None)
    return np.array([[sol[0], sol[1], sol[2]], [sol[3], sol[4], sol[5]]])


def run_calibration(points_path: Path, out_path: Path) -> int:
    """points.json: [{"cv": [x_m, y_m], "robot": [x_m, y_m]}, ...] (>=3 entries)."""
    pairs = json.loads(points_path.read_text())
    cv_pts = [p["cv"] for p in pairs]
    robot_pts = [p["robot"] for p in pairs]
    A = solve_affine(cv_pts, robot_pts)

    residuals = [
        np.hypot(*(np.subtract(cv_to_robot_xy(x, y, A), rb)))
        for (x, y), rb in zip(cv_pts, robot_pts)
    ]
    rms_mm = float(np.sqrt(np.mean(np.square(residuals))) * 1000)

    existing = load_cv_to_robot(out_path)
    existing.update({"A": A.tolist(), "calibrated": True})
    out_path.write_text(json.dumps(existing, indent=2))
    print(f"Saved transform to {out_path}")
    print(f"  RMS residual over {len(cv_pts)} points: {rms_mm:.1f} mm")
    return 0


# --------------------------------------------------------------------------- #
# Motion primitives
# --------------------------------------------------------------------------- #
def pick_object(ctrl, rx, ry, pick_z, hover, g_open, g_closed, duration):
    """Hover above, descend, close gripper, lift. Leaves the arm holding the box."""
    ctrl.move_to_table(rx, ry, pick_z + hover, gripper=g_open, duration_s=duration)
    ctrl.move_to_table(rx, ry, pick_z, gripper=g_open, duration_s=duration)
    ctrl.set_gripper(g_closed)
    ctrl.move_to_table(rx, ry, pick_z + hover, gripper=g_closed, duration_s=duration)


def place_object(ctrl, dx, dy, place_z, hover, g_open, g_closed, duration):
    """Move to the drop zone, descend, open gripper, lift."""
    ctrl.move_to_table(dx, dy, place_z + hover, gripper=g_closed, duration_s=duration)
    ctrl.move_to_table(dx, dy, place_z, gripper=g_closed, duration_s=duration)
    ctrl.set_gripper(g_open)
    ctrl.move_to_table(dx, dy, place_z + hover, gripper=g_open, duration_s=duration)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pick-and-place a CV-detected box with the SO-101.")
    src = p.add_argument_group("input")
    src.add_argument("--image", type=Path, help="Detect in this image file")
    src.add_argument("--camera", type=int, help="Detect from this camera index")

    p.add_argument("--calib", type=Path, default=DEFAULT_CALIB, help="Homography JSON")
    p.add_argument("--hsv-config", type=Path, default=DEFAULT_HSV, help="HSV JSON")
    p.add_argument("--cv2robot", type=Path, default=DEFAULT_CV2ROBOT, help="CV->robot transform JSON")
    p.add_argument("--pick-place-config", type=Path, default=DEFAULT_PICK_PLACE, help="Task config JSON")
    p.add_argument("--config", type=Path, default=DEFAULT_TABLE_CFG, help="table_frame.json")

    p.add_argument("--hover", type=float, help="Hover height above the pick point (m); default from cv2robot")
    p.add_argument("--pick-z", type=float, help="Table-surface z in robot frame (m); default from cv2robot")
    p.add_argument("--gripper-open", type=float, help="Gripper open value; default from cv2robot")
    p.add_argument("--gripper-closed", type=float, help="Gripper closed value; default from cv2robot")
    p.add_argument("--duration", type=float, help="Per-move duration (s)")

    p.add_argument("--loop", action="store_true", help="Continuously poll the camera and pick+place")
    p.add_argument("--interval", type=float, help="Seconds between polls in --loop; default from config")
    p.add_argument("--go", action="store_true", help="Actually move (default: dry run, print only)")
    p.add_argument("--pick", action="store_true", help="Single shot: descend + close gripper + lift")
    p.add_argument("--place", action="store_true", help="Single shot: after pick, place in the drop zone")
    p.add_argument("--no-calibrate", action="store_true", help="Skip robot calibration prompt on connect")

    p.add_argument("--calibrate", type=Path, help="Solve CV->robot transform from a points JSON, then exit")
    p.add_argument("--out", type=Path, default=DEFAULT_CV2ROBOT, help="Where to write --calibrate result")
    return p.parse_args()


def _resolve_params(args, transform, pick_place):
    hover = args.hover if args.hover is not None else float(transform.get("hover_m", 0.05))
    pick_z = args.pick_z if args.pick_z is not None else float(transform.get("pick_z_m", 0.0))
    g_open = args.gripper_open if args.gripper_open is not None else float(transform.get("gripper_open", 100.0))
    g_closed = args.gripper_closed if args.gripper_closed is not None else float(transform.get("gripper_closed", 0.0))
    place_cfg = pick_place.get("place", {})
    place_z = float(place_cfg.get("z_m", 0.0))
    place_hover = float(place_cfg.get("hover_m", hover))
    return hover, pick_z, g_open, g_closed, place_z, place_hover


def _report(det, rx, ry, pick_z, hover):
    u, v, area, x_cv, y_cv = det
    print("Detection:")
    print(f"  pixel        = ({u:.0f}, {v:.0f})  area={area:.0f}px")
    print(f"  CV table     = ({x_cv * 100:.1f}, {y_cv * 100:.1f}) cm")
    print(f"  robot table  = ({rx * 100:.1f}, {ry * 100:.1f}) cm")
    print(f"  hover pose   = x={rx:.4f} y={ry:.4f} z={pick_z + hover:.4f}")
    print(f"  pick pose    = x={rx:.4f} y={ry:.4f} z={pick_z:.4f}")


def main() -> int:
    args = _parse_args()

    if args.calibrate is not None:
        return run_calibration(args.calibrate, args.out)

    if not args.image and args.camera is None:
        print("Provide --image PATH or --camera INDEX", file=sys.stderr)
        return 2

    calib = load_homography(args.calib)
    transform = load_cv_to_robot(args.cv2robot)
    pick_place = load_pick_place(args.pick_place_config)
    A = transform["A"]
    calibrated = bool(transform.get("calibrated", False))
    exclude_zone = pick_place.get("drop_zone_cv_m")

    hover, pick_z, g_open, g_closed, place_z, place_hover = _resolve_params(
        args, transform, pick_place
    )
    interval = args.interval if args.interval is not None else float(pick_place.get("poll_interval_s", 2.0))

    if args.go and not calibrated:
        print(
            "Refusing to move: CV->robot transform is not calibrated.\n"
            "Calibrate first: python mission/cv_pick.py --calibrate points.json",
            file=sys.stderr,
        )
        return 3

    # Connect the robot only when we actually intend to move.
    ctrl = None
    drop_xy = None
    if args.go:
        from mission.table_moves import TableMotionController, load_table_config  # noqa: E402

        drop_xy = resolve_place_xy(pick_place, A)
        cfg = load_table_config(args.config)
        ctrl = TableMotionController(cfg)
        ctrl.connect(calibrate=not args.no_calibrate)

    def cycle(frame, do_pick: bool, do_place: bool) -> bool:
        """One detect (+ optional pick/place). Returns True if a box was handled."""
        det = detect_target(frame, calib, args.hsv_config, exclude_zone)
        if det is None:
            print("No box detected (outside the drop zone).")
            return False
        rx, ry = cv_to_robot_xy(det[3], det[4], A)
        _report(det, rx, ry, pick_z, hover)
        if not calibrated:
            print(
                "  WARNING: transform not calibrated — robot coord is meaningless.",
                file=sys.stderr,
            )
        if not args.go:
            return True
        if not do_pick:
            print("  Moving to hover only (add --pick to grasp)...")
            ctrl.move_to_table(rx, ry, pick_z + hover, gripper=g_open, duration_s=args.duration)
            return True
        print("  Picking...")
        pick_object(ctrl, rx, ry, pick_z, hover, g_open, g_closed, args.duration)
        if do_place:
            print(f"  Placing in drop zone ({drop_xy[0] * 100:.1f}, {drop_xy[1] * 100:.1f}) cm...")
            place_object(ctrl, drop_xy[0], drop_xy[1], place_z, place_hover, g_open, g_closed, args.duration)
            if pick_place.get("return_home_after_place", True):
                ctrl.go_home()
        print("  Cycle complete.")
        return True

    try:
        if args.loop:
            cap = open_camera(args.camera) if args.camera is not None else None
            mode = "LIVE pick+place" if args.go else "DRY RUN (detection only)"
            print(f"Polling every {interval:.1f}s — {mode}. Ctrl-C to stop.")
            try:
                while True:
                    frame = read_frame(cap) if cap is not None else load_image(args.image)
                    print(f"\n[{time.strftime('%H:%M:%S')}]")
                    cycle(frame, do_pick=True, do_place=True)
                    time.sleep(interval)
            except KeyboardInterrupt:
                print("\nStopped.")
            finally:
                if cap is not None:
                    cap.release()
            return 0

        # Single shot
        frame = read_frame(open_camera(args.camera)) if args.camera is not None else load_image(args.image)
        handled = cycle(frame, do_pick=args.pick, do_place=args.place)
        if not args.go:
            print("\nDry run (no motion). Add --go to move; --loop --go for continuous.")
        return 0 if handled else 1
    finally:
        if ctrl is not None and ctrl.follower.is_connected:
            ctrl.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
