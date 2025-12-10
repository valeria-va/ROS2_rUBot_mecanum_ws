#!/usr/bin/env python3
"""
Minimal trajectory generator (raster/zigzag) with map filtering.

Example:
ros2 run my_robot_co2map trajectory_generator \
  --xmin -7.35 --xmax 5.05 \
  --ymin -6.48 --ymax 2.92 \
  --spacing 0.5 \
  --sweep-axis y \
  --pattern zigzag \
  --yaw-mode fixed --yaw 0.0 \
  --map-yaml /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/slam_maps/test_rajoles1_justincase.yaml \
  --inflate 0.4 \
  --out /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/trajectories/trajectory1.csv
"""

from pathlib import Path
import argparse
import csv
import math
from typing import List, Tuple, Optional
import numpy as np
from PIL import Image, ImageFilter

try:
    import yaml
except Exception:
    yaml = None

try:
    from PIL import Image, ImageFilter
except Exception:
    Image = None


# ---------------- utilities ----------------

def frange(start: float, stop: float, step: float) -> List[float]:
    vals = []
    x = start
    # include endpoint with a tiny epsilon
    eps = 1e-9
    while x <= stop + eps:
        vals.append(round(x, 6))
        x += step
    return vals


def clamp_bounds(xmin, xmax, ymin, ymax):
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    return xmin, xmax, ymin, ymax


# ---------------- raster/zigzag ----------------

def generate_raster(
    xmin: float, xmax: float, ymin: float, ymax: float,
    spacing: float, sweep_axis: str, zigzag: bool,
    yaw_mode: str, fixed_yaw: float
) -> List[Tuple[float, float, float]]:
    x_vals = frange(xmin, xmax, spacing)
    y_vals = frange(ymin, ymax, spacing)
    waypoints: List[Tuple[float, float, float]] = []
    prev = None

    if sweep_axis == "y":
        for i, y in enumerate(y_vals):
            xs = x_vals if (not zigzag or i % 2 == 0) else list(reversed(x_vals))
            for x in xs:
                yaw = fixed_yaw
                if yaw_mode == "along_path" and prev is not None:
                    dx, dy = x - prev[0], y - prev[1]
                    if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                        yaw = math.atan2(dy, dx)
                waypoints.append((x, y, yaw))
                prev = (x, y, yaw)
    else:  # sweep_axis == "x"
        for j, x in enumerate(x_vals):
            ys = y_vals if (not zigzag or j % 2 == 0) else list(reversed(y_vals))
            for y in ys:
                yaw = fixed_yaw
                if yaw_mode == "along_path" and prev is not None:
                    dx, dy = x - prev[0], y - prev[1]
                    if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                        yaw = math.atan2(dy, dx)
                waypoints.append((x, y, yaw))
                prev = (x, y, yaw)
    return waypoints


# ---------------- map utils (fixed logic) ----------------

def load_map_yaml(map_yaml_path: Path):
    if yaml is None:
        raise RuntimeError("PyYAML is required. Install with: pip install pyyaml")
    with open(map_yaml_path, "r") as f:
        data = yaml.safe_load(f)
    required = ["image", "resolution", "origin"]
    for k in required:
        if k not in data:
            raise ValueError(f"map.yaml missing required key: {k}")
    # ensure numeric defaults exist
    data.setdefault("occupied_thresh", 0.65)
    data.setdefault("free_thresh", 0.25)
    data.setdefault("negate", 0)
    return data


def load_map_image(image_path: Path, negate: int):
    if Image is None:
        raise RuntimeError("Pillow is required. Install with: pip install pillow")
    if not image_path.exists():
        raise FileNotFoundError(f"Map image not found: {image_path}")
    img = Image.open(image_path)
    # convert to grayscale if needed
    if img.mode != "L":
        img = img.convert("L")
    if negate == 1:
        img = Image.eval(img, lambda p: 255 - p)
    return img



# paste/replace these functions in your trajectory_generator.py

import numpy as np
from PIL import Image, ImageFilter

def build_occupied_mask_np(img: Image.Image, occupied_thresh: float, free_thresh: float):
    """
    Return a boolean numpy array occ_mask (shape h,w), True = occupied (includes unknown).
    unknown (between occ and free) -> treated as occupied.
    img is a PIL L-mode image.
    """
    arr = np.array(img, dtype=np.uint8)        # shape (h,w), values 0..255
    occ_px = int(round(occupied_thresh * 255.0))
    free_px = int(round(free_thresh * 255.0))

    # occupied if val <= occ_px OR (val between occ_px+1 and free_px-1) -> unknown treated occupied
    occ_mask = (arr <= occ_px) | ((arr > occ_px) & (arr < free_px))
    # Note: free pixels are arr >= free_px
    return occ_mask  # dtype=bool, shape (h,w)


def inflate_occupied_mask_np(occ_mask: np.ndarray, inflation_radius_m: float, resolution: float):
    """
    Inflate boolean occ_mask (h,w) by inflation_radius_m using PIL MaxFilter.
    Returns inflated boolean numpy array.
    """
    if inflation_radius_m <= 0.0:
        return occ_mask

    radius_px = max(0, int(math.ceil(inflation_radius_m / resolution)))
    if radius_px == 0:
        return occ_mask

    # Convert to PIL image (mode 'L'): 0 free -> 0, occupied -> 255
    pil = Image.fromarray((occ_mask.astype(np.uint8) * 255))
    kernel = 2 * radius_px + 1
    inflated = pil.filter(ImageFilter.MaxFilter(size=kernel))
    inflated_np = np.array(inflated, dtype=np.uint8) > 0
    return inflated_np  # bool array


def world_to_map_pixel(x: float, y: float, origin, resolution: float, img_h: int):
    """
    Convert world coordinates (x,y) -> image pixel coords consistent with PIL indexing.
    Returns (mx, my_img) where mx in [0..w-1], my_img in [0..h-1] with top-left origin.
    """
    x0, y0, _ = origin
    mx = int(math.floor((x - x0) / resolution))
    my = int(math.floor((y - y0) / resolution))
    # flip Y for PIL image coordinates (top-left origin)
    my_img = img_h - 1 - my
    return mx, my_img


def filter_waypoints_by_map(
    waypoints: List[Tuple[float, float, float]],
    map_yaml_path: Optional[Path],
    inflation_radius_m: float = 0.0,
) -> List[Tuple[float, float, float]]:
    """
    Filters waypoints; returns list of accepted waypoints.
    Strict rules:
      - unknown (mid-gray) is treated occupied
      - outside-map is treated occupied (rejected)
    """
    if not map_yaml_path:
        return waypoints

    params = load_map_yaml(map_yaml_path)
    image_path = Path(map_yaml_path.parent) / params["image"]
    img = load_map_image(image_path, negate=int(params.get("negate", 0)))
    # img is PIL L-mode with shape (w,h) in previous code; we need numpy (h,w)
    arr_img = img  # PIL Image
    h, w = np.array(arr_img).shape

    resolution = float(params["resolution"])
    origin = tuple(params["origin"])
    occupied_thresh = float(params.get("occupied_thresh", 0.65))
    free_thresh = float(params.get("free_thresh", 0.25))

    # Build occupied mask (unknown => occupied)
    occ_mask = build_occupied_mask_np(arr_img, occupied_thresh, free_thresh)  # bool h,w

    # Inflate
    if inflation_radius_m > 0.0:
        occ_mask = inflate_occupied_mask_np(occ_mask, inflation_radius_m, resolution)

    # Now classify points
    filtered: List[Tuple[float, float, float]] = []
    for (x, y, yaw) in waypoints:
        mx, my_img = world_to_map_pixel(x, y, origin, resolution, h)
        # Reject if out of bounds
        if mx < 0 or my_img < 0 or mx >= w or my_img >= h:
            continue
        # occ_mask is indexed as [row=y_img, col=x]
        if not occ_mask[my_img, mx]:
            # not occupied => free
            filtered.append((x, y, yaw))
        # else occupied (including unknown) -> skip
    return filtered

# ---------------- IO ----------------

def write_csv(path: Path, waypoints: List[Tuple[float, float, float]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# x", "y", "yaw"])
        for x, y, yaw in waypoints:
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])


# ---------------- CLI ----------------

def parse_args():
    p = argparse.ArgumentParser(description="Minimal trajectory generator (raster/zigzag) with safe map filtering")
    p.add_argument("--xmin", type=float, required=True, help="min X (meters, map frame)")
    p.add_argument("--xmax", type=float, required=True, help="max X")
    p.add_argument("--ymin", type=float, required=True, help="min Y")
    p.add_argument("--ymax", type=float, required=True, help="max Y")
    p.add_argument("--spacing", type=float, default=0.5, help="grid/coverage spacing (meters)")
    p.add_argument("--pattern", choices=["raster", "zigzag"], default="raster",
                   help="coverage pattern (raster or zigzag)")
    p.add_argument("--sweep-axis", choices=["x", "y"], default="y",
                   help="for raster/zigzag: traverse rows (y) or columns (x)")
    p.add_argument("--yaw-mode", choices=["fixed", "along_path"], default="fixed",
                   help="orientation mode: fixed yaw or along direction of motion")
    p.add_argument("--yaw", type=float, default=0.0, help="fixed yaw (radians) if yaw-mode=fixed")
    p.add_argument("--out", type=str, default="trajectory.csv", help="output CSV path")
    p.add_argument("--map-yaml", type=str, default=None, help="SLAM map.yaml for obstacle/unknown filtering")
    p.add_argument("--inflate", type=float, default=0.0, help="inflation radius (meters) for obstacle clearance")
    return p.parse_args()


def main():
    args = parse_args()
    xmin, xmax, ymin, ymax = clamp_bounds(args.xmin, args.xmax, args.ymin, args.ymax)

    # generate waypoints
    if args.pattern in ("raster", "zigzag"):
        zigzag = (args.pattern == "zigzag")
        wps = generate_raster(
            xmin, xmax, ymin, ymax, args.spacing,
            sweep_axis=args.sweep_axis,
            zigzag=zigzag,
            yaw_mode=args.yaw_mode,
            fixed_yaw=args.yaw
        )
    else:
        raise ValueError("Unsupported pattern")

    # filter against map if provided
    map_yaml = Path(args.map_yaml) if args.map_yaml else None
    if map_yaml:
        try:
            wps = filter_waypoints_by_map(wps, map_yaml_path=map_yaml, inflation_radius_m=args.inflate)
        except Exception as e:
            print(f"[WARN] Map filtering failed: {e}. Proceeding without filtering.")
    # save
    out_path = Path(args.out)
    write_csv(out_path, wps)
    print(f"Saved {len(wps)} waypoints to {out_path}")


if __name__ == "__main__":
    main()
