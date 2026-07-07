from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class Target_Cap:
    u: float
    v: float  # pixel coords
    area: float  # used to filter false positives


def make_table_mask(shape: tuple[int, ...], corners_px: np.ndarray) -> np.ndarray:
    h, w = shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [corners_px.reshape(-1, 1, 2)], 255)
    return mask


def point_locate(u: float, v: float, corners_px: np.ndarray) -> bool:
    return cv2.pointPolygonTest(corners_px.astype(np.float32), (u, v), False) >= 0


def detect_blobs(
    image: np.ndarray,
    *,
    hsv_lower: tuple[int, int, int],
    hsv_upper: tuple[int, int, int],
    table_corners: np.ndarray,
    min_area: float = 10.0,
    morph_kernel: int = 5,
) -> tuple[list[Target_Cap], np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))

    ws = make_table_mask(image.shape, table_corners)
    mask = cv2.bitwise_and(color_mask, ws)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs: list[Target_Cap] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        u = m["m10"] / m["m00"]
        v = m["m01"] / m["m00"]
        if not point_locate(u, v, table_corners):
            continue
        blobs.append(Target_Cap(u=u, v=v, area=area))

    blobs.sort(key=lambda b: b.area, reverse=True)
    return blobs, mask, ws


def load_image(path: Path | str) -> np.ndarray:
    path = Path(path)
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return image


def draw_detections(
    image: np.ndarray,
    blobs: list[Target_Cap],
    table_corners: np.ndarray,
    table_xy_cm: list[tuple[float, float]] | None = None,
) -> np.ndarray:
    out = image.copy()
    cv2.polylines(out, [table_corners.reshape(-1, 1, 2)], True, (0, 255, 255), 2)
    for i, blob in enumerate(blobs):
        u, v = int(blob.u), int(blob.v)
        cv2.circle(out, (u, v), 10, (0, 255, 0), 2)
        label = str(i)
        if table_xy_cm and i < len(table_xy_cm):
            x_cm, y_cm = table_xy_cm[i]
            label = f"{i} ({x_cm:.1f},{y_cm:.1f})cm"
        cv2.putText(
            out,
            label,
            (u + 12, v),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    return out
