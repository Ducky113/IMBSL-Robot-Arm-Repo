"""Interactive HSV tuner. Press 's' to save, 'q' to quit."""

import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

from cv_service.detect import detect_blobs, draw_detections, load_image
from cv_service.homography import (
    DEFAULT_CALIB_PATH,
    homography_matrix,
    load_homography,
    pixel_to_table,
    workspace_corners_px,
)

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HSV_PATH = ROOT / "config" / "hsv.json"
WINDOW = "tune_hsv"


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Tune HSV thresholds with live preview.")
    parser.add_argument("--image", "-i", required=True, type=Path)
    parser.add_argument("--calib", type=Path, default=DEFAULT_CALIB_PATH)
    parser.add_argument("--out", type=Path, default=DEFAULT_HSV_PATH)
    args = parser.parse_args()

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print(
            "No display found (DISPLAY / WAYLAND_DISPLAY not set).\n"
            "tune_hsv needs a GUI — run from a normal desktop terminal, not with 'env -i'.\n"
            "Example:\n"
            "  cd <project-root>\n"
            "  PYTHONPATH=. python -m cv_service.tune_hsv -i data/images/M1.jpg",
            file=sys.stderr,
        )
        sys.exit(1)

    calib = load_homography(args.calib)
    H = homography_matrix(calib)
    workspace = workspace_corners_px(calib)
    image = load_image(args.image)

    if args.out.is_file():
        saved = json.loads(args.out.read_text())
        h0, s0, v0 = saved["hsv_lower"]
        h1, s1, v1 = saved["hsv_upper"]
        min_area = int(saved.get("min_area", 500))
    else:
        h0, s0, v0 = 0, 80, 80
        h1, s1, v1 = 180, 255, 255
        min_area = 500

    cv2.namedWindow(WINDOW)
    for name, val, maxval in [
        ("H lo", h0, 179),
        ("S lo", s0, 255),
        ("V lo", v0, 255),
        ("H hi", h1, 179),
        ("S hi", s1, 255),
        ("V hi", v1, 255),
        ("min area", min(min_area, 50000), 50000),
    ]:
        cv2.createTrackbar(name, WINDOW, val, maxval, lambda _: None)

    print("Adjust trackbars. 's' = save, 'q' = quit.")

    while True:
        lower = (
            cv2.getTrackbarPos("H lo", WINDOW),
            cv2.getTrackbarPos("S lo", WINDOW),
            cv2.getTrackbarPos("V lo", WINDOW),
        )
        upper = (
            cv2.getTrackbarPos("H hi", WINDOW),
            cv2.getTrackbarPos("S hi", WINDOW),
            cv2.getTrackbarPos("V hi", WINDOW),
        )
        min_area = max(1, cv2.getTrackbarPos("min area", WINDOW))

        blobs, mask, _ = detect_blobs(
            image,
            hsv_lower=lower,
            hsv_upper=upper,
            table_corners=workspace,
            min_area=float(min_area),
        )
        table_cm = []
        for b in blobs:
            x_m, y_m = pixel_to_table(b.u, b.v, H)
            table_cm.append((x_m * 100, y_m * 100))
        vis = draw_detections(image, blobs, workspace, table_cm)
        stacked = np.hstack([vis, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow(WINDOW, stacked)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            args.out.parent.mkdir(parents=True, exist_ok=True)
            payload = {"hsv_lower": list(lower), "hsv_upper": list(upper), "min_area": min_area}
            args.out.write_text(json.dumps(payload, indent=2))
            print(f"Saved {args.out}: {payload}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
