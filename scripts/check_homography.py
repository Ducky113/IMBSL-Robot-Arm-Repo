import json
from pathlib import Path

import cv2
import numpy as np

CALIB_PATH = Path(__file__).resolve().parent.parent / "config" / "table_homography.json"

# Verification points: pixel (u, v) -> table (x_cm, y_cm) from homography calibration
TEST_POINTS = [
    ((2542,952), (15,23.5)),
    ((348,2052), (68.5,49.3)),
    ((854,306), (56.5,7.3)),
]

data = json.load(CALIB_PATH.open())
H = np.array(data["H"], dtype=np.float64)

print(f"Loaded {CALIB_PATH}\n")
print( "The maximum error should be within 5mm ideally")

for (u, v), (x_cm, y_cm) in TEST_POINTS:
    pt = np.array([[[u, v]]], dtype=np.float64)
    got = cv2.perspectiveTransform(pt, H)[0, 0]
    got_cm = (got[0] * 100, got[1] * 100)
    err_mm = np.hypot(got_cm[0] - x_cm, got_cm[1] - y_cm) * 10
    print(f"pixel ({u}, {v})")
    print(f"  expected: ({x_cm}, {y_cm}) cm")
    print(f"  got:      ({got_cm[0]:.2f}, {got_cm[1]:.2f}) cm")
    print(f"  error:    {err_mm:.1f} mm\n")


