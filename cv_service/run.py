"""Detect samples in an image and print table coordinates."""

import argparse
import json
from pathlib import Path

import cv2

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


def _parse_hsv(s: str) -> tuple[int, int, int]:
    parts = [int(x.strip()) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Expected H,S,V e.g. 0,100,100")
    return parts[0], parts[1], parts[2]


def _load_hsv(path: Path) -> tuple[tuple[int, int, int], tuple[int, int, int], float]:
    if not path.is_file():
        return (0, 80, 80), (180, 255, 255), 500.0
    data = json.loads(path.read_text())
    return tuple(data["hsv_lower"]), tuple(data["hsv_upper"]), float(data.get("min_area", 500))


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect colored samples on the table.")
    parser.add_argument("--image", "-i", type=Path, help="Input image path")
    parser.add_argument("--image-dir", type=Path, help="Run on all images in a folder")
    parser.add_argument("--calib", type=Path, default=DEFAULT_CALIB_PATH)
    parser.add_argument("--hsv-config", type=Path, default=DEFAULT_HSV_PATH)
    parser.add_argument("--hsv-lower", type=_parse_hsv)
    parser.add_argument("--hsv-upper", type=_parse_hsv)
    parser.add_argument("--min-area", type=float)
    parser.add_argument("--show", action="store_true", help="Show detection windows")
    parser.add_argument("--save", type=Path, help="Save annotated image")
    parser.add_argument("--json", action="store_true", help="Output JSON")
    args = parser.parse_args()

    if not args.image and not args.image_dir:
        parser.error("Provide --image or --image-dir")

    calib = load_homography(args.calib)
    H = homography_matrix(calib)
    workspace = workspace_corners_px(calib)

    hsv_lower, hsv_upper, min_area = _load_hsv(args.hsv_config)
    if args.hsv_lower:
        hsv_lower = args.hsv_lower
    if args.hsv_upper:
        hsv_upper = args.hsv_upper
    if args.min_area is not None:
        min_area = args.min_area

    if args.image_dir:
        images = sorted(
            p
            for p in args.image_dir.iterdir()
            if p.suffix.lower() in {".jpg", ".jpeg", ".png"}
        )
    else:
        images = [args.image]

    all_results = []

    for image_path in images:
        image = load_image(image_path)
        blobs, mask, _ = detect_blobs(
            image,
            hsv_lower=hsv_lower,
            hsv_upper=hsv_upper,
            table_corners=workspace,
            min_area=min_area,
        )

        table_cm = []
        for b in blobs:
            x_m, y_m = pixel_to_table(b.u, b.v, H)
            table_cm.append((x_m * 100, y_m * 100))
        annotated = draw_detections(image, blobs, workspace, table_cm)

        result = {
            "image": str(image_path),
            "detections": [
                {
                    "u": b.u,
                    "v": b.v,
                    "area": b.area,
                    "x_cm": xy[0],
                    "y_cm": xy[1],
                }
                for b, xy in zip(blobs, table_cm)
            ],
        }
        all_results.append(result)

        if args.json:
            continue

        print(f"\n{image_path.name}: {len(blobs)} detection(s)")
        print(f"  HSV lower={hsv_lower} upper={hsv_upper} min_area={min_area}")
        for i, (b, (x_cm, y_cm)) in enumerate(zip(blobs, table_cm)):
            print(
                f"  [{i}] pixel=({b.u:.0f}, {b.v:.0f}) "
                f"table=({x_cm:.1f}, {y_cm:.1f}) cm area={b.area:.0f}px"
            )

        if args.save:
            out = args.save if len(images) == 1 else args.save.parent / f"{image_path.stem}_out{image_path.suffix}"
            out.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out), annotated)
            print(f"  saved {out}")

        if args.show:
            cv2.imshow(f"detections: {image_path.name}", annotated)
            cv2.imshow(f"mask: {image_path.name}", mask)

    if args.json:
        print(json.dumps(all_results, indent=2))

    if args.show:
        print("Press any key in an image window to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
