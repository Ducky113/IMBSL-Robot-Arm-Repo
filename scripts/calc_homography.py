import json
from pathlib import Path

import cv2
import numpy as np

# Corners in order: BL, TL, TR, BR (from Info_Sheet.txt)
Table_C = np.array([[0, 0], [0, 0.57], [.74, 0.57], [.74, 0]], dtype=np.float64)

Image_C = np.array([[3216,28], [3234,2394], [6,2504], [158,38]], dtype=np.float64)

# pixel (u, v) -> table (x, y) in meters
H, _ = cv2.findHomography(Image_C, Table_C, method=0)

out_path = Path(__file__).resolve().parent.parent / "config" / "table_homography.json"
out_path.write_text(
    json.dumps(
        {"units": "meters","world_corners_m": Table_C.tolist(),"image_corners_px": Image_C.astype(int).tolist(),"H": H.tolist(),},indent=2
        )
    )
print(f"Saved {out_path}")
