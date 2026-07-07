import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CALIB_PATH = ROOT / "config" / "table_homography.json"


def load_homography(path: Path | str | None = None) -> dict:
    path = Path(path) if path is not None else DEFAULT_CALIB_PATH
    with path.open() as f:
        return json.load(f)


def homography_matrix(data: dict) -> np.ndarray:
    return np.array(data["H"], dtype=np.float64)


def workspace_corners_px(data: dict) -> np.ndarray:
    return np.array(data["image_corners_px"], dtype=np.int32)


def pixel_to_table(u: float, v: float, H: np.ndarray) -> tuple[float, float]:
    pt = np.array([[[u, v]]], dtype=np.float64)
    xy = cv2.perspectiveTransform(pt, H)[0, 0]
    return float(xy[0]), float(xy[1])
