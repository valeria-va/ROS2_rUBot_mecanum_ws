#!/usr/bin/env python3
"""
Trajectory generator that writes a CSV of (x,y,yaw) waypoints.

ros2 run my_robot_co2map trajectory_generator \
  --xmin 0.0 --xmax 3.0 --ymin 0.0 --ymax 2.0 \
  --spacing 0.1 --pattern zigzag --yaw-mode fixed --yaw 0.0 \
  --map-yaml test_delete.yaml \
  --inflate 0.25 \
  --out ../trajectories/test_delete.csv


ros2 run my_robot_co2map trajectory_generator \
  --xmin 0.0 \              # position: min X bound (meters, map frame)
  --xmax 3.0 \              # position: max X bound
  --ymin 0.0 \              # position: min Y bound
  --ymax 2.0 \              # position: max Y bound
  --spacing 0.5 \           # non-position: grid spacing (meters)

  --sweep-axis y \          # non-position: raster direction ('y' rows or 'x' columns)
  --zigzag \                # PATTERN CHOICE A: lawnmower (alternate direction each row)
  # (Alternative to --zigzag) OMIT THIS FLAG for straight raster (same direction each row)

  --yaw-mode fixed \        # non-position: orientation mode ('fixed' or 'along_path')
  --yaw 0.0 \               # non-position: yaw in radians if fixed (ignored in along_path)

  --map-yaml maps/room.yaml \  # non-position: SLAM map for obstacle/unknown filtering (map.yaml)
  --inflate 0.25 \          # non-position: extra clearance from obstacles (meters)
  --out sweep.csv           # non-position: output CSV filename (generated trajectory)


Outputs a CSV with rows: x,y,yaw

"""

import csv
import math
import argparse
from pathlib import Path
from typing import List, Tuple, Optional

try:
    import yaml
except ImportError:
    yaml = None

try:
    from PIL import Image, ImageFilter
except ImportError:
    Image = None


# ---------- utility ----------

def frange(start: float, stop: float, step: float) -> List[float]:
    vals = []
    x = start
    while x <= stop + 1e-9:
        vals.append(round(x, 6))
        x += step
    return vals


def clamp_bounds(xmin, xmax, ymin, ymax):
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    return xmin, xmax, ymin, ymax


# ---------- pattern generators ----------

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


def generate_spiral(
    xmin: float, xmax: float, ymin: float, ymax: float,
    spacing: float, yaw_mode: str, fixed_yaw: float
) -> List[Tuple[float, float, float]]:
    """
    Simple rectangular spiral: start near center and expand outward layer by layer.
    Produces smoother paths and fewer long transits than raster end-hops.
    """
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    # round center to nearest grid
    gx = round((cx - xmin) / spacing) * spacing + xmin
    gy = round((cy - ymin) / spacing) * spacing + ymin

    waypoints: List[Tuple[float, float, float]] = []
    prev = None

    # Create layers expanding from center
    max_half_x = math.ceil((xmax - gx) / spacing)
    max_half_y = math.ceil((ymax - gy) / spacing)
    max_half = int(max(max_half_x, max_half_y, math.ceil((gx - xmin) / spacing), math.ceil((gy - ymin) / spacing)))

    for k in range(1, max_half + 1):
        x_left = gx - k * spacing
        x_right = gx + k * spacing
        y_bottom = gy - k * spacing
        y_top = gy + k * spacing

        # Clamp within bounds
        x_left, x_right = max(x_left, xmin), min(x_right, xmax)
        y_bottom, y_top = max(y_bottom, ymin), min(y_top, ymax)

        # Traverse a rectangle perimeter counter-clockwise: bottom row, right col, top row, left col
        # bottom row (left -> right)
        x_vals = frange(x_left, x_right, spacing)
        for x in x_vals:
            y = y_bottom
            yaw = fixed_yaw
            if yaw_mode == "along_path" and prev is not None:
                dx, dy = x - prev[0], y - prev[1]
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    yaw = math.atan2(dy, dx)
            waypoints.append((x, y, yaw))
            prev = (x, y, yaw)
        # right column (bottom -> top)
        y_vals = frange(y_bottom + spacing, y_top, spacing)
        for y in y_vals:
            x = x_right
            yaw = fixed_yaw
            if yaw_mode == "along_path" and prev is not None:
                dx, dy = x - prev[0], y - prev[1]
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    yaw = math.atan2(dy, dx)
            waypoints.append((x, y, yaw))
            prev = (x, y, yaw)
        # top row (right -> left)
        x_vals_rev = list(reversed(frange(x_left, x_right - 1e-9, spacing)))
        for x in x_vals_rev:
            y = y_top
            yaw = fixed_yaw
            if yaw_mode == "along_path" and prev is not None:
                dx, dy = x - prev[0], y - prev[1]
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    yaw = math.atan2(dy, dx)
            waypoints.append((x, y, yaw))
            prev = (x, y, yaw)
        # left column (top -> bottom)
        y_vals_rev = list(reversed(frange(y_bottom + spacing, y_top - 1e-9, spacing)))
        for y in y_vals_rev:
            x = x_left
            yaw = fixed_yaw
            if yaw_mode == "along_path" and prev is not None:
                dx, dy = x - prev[0], y - prev[1]
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    yaw = math.atan2(dy, dx)
            waypoints.append((x, y, yaw))
            prev = (x, y, yaw)

    # Add the center if inside bounds
    if xmin <= gx <= xmax and ymin <= gy <= ymax:
        yaw = fixed_yaw
        if yaw_mode == "along_path" and prev is not None:
            dx, dy = gx - prev[0], gy - prev[1]
            if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                yaw = math.atan2(dy, dx)
        waypoints.insert(0, (gx, gy, yaw))

    return waypoints


def generate_poisson(
    xmin: float, xmax: float, ymin: float, ymax: float,
    spacing: float, yaw_mode: str, fixed_yaw: float, max_attempts: int = 20000
) -> List[Tuple[float, float, float]]:
    """
    Poisson-disk sampling: randomly distributed points with minimum spacing.
    Good for irregular rooms and uniform coverage density.
    """
    import random
    waypoints: List[Tuple[float, float, float]] = []

    def valid(pt):
        x, y = pt
        for (xp, yp, _) in waypoints:
            dx, dy = x - xp, y - yp
            if dx * dx + dy * dy < spacing * spacing:
                return False
        return True

    attempts = 0
    prev = None
    while attempts < max_attempts:
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        if valid((x, y)):
            yaw = fixed_yaw
            if yaw_mode == "along_path" and prev is not None:
                dx, dy = x - prev[0], y - prev[1]
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    yaw = math.atan2(dy, dx)
            waypoints.append((x, y, yaw))
            prev = (x, y, yaw)
        attempts += 1

    return waypoints


# ---------- map filtering ----------

def load_map_yaml(map_yaml_path: Path):
    if yaml is None:
        raise RuntimeError("PyYAML not installed. Install with: pip install pyyaml")
    with open(map_yaml_path, "r") as f:
        data = yaml.safe_load(f)
    required = ["image", "resolution", "origin"]
    for k in required:
        if k not in data:
            raise ValueError(f"map.yaml missing required key: {k}")
    data.setdefault("occupied_thresh", 0.65)
    data.setdefault("free_thresh", 0.196)
    data.setdefault("negate", 0)
    return data


def load_map_image(image_path: Path, negate: int):
    if Image is None:
        raise RuntimeError("Pillow not installed. Install with: pip install pillow")
    img = Image.open(image_path)
    if img.mode != "L":
        img = img.convert("L")
    if negate == 1:
        img = Image.eval(img, lambda p: 255 - p)
    return img


def inflate_mask(img: Image.Image, inflation_radius_m: float, resolution: float) -> Image.Image:
    radius_px = max(0, int(round(inflation_radius_m / resolution)))
    if radius_px == 0:
        return img
    # Build occupied mask using a conservative threshold
    occupied_thresh = 0.65
    w, h = img.size
    occ = Image.new("1", (w, h), 0)
    px = img.load()
    oc = occ.load()
    for x in range(w):
        for y in range(h):
            prob = px[x, y] / 255.0
            oc[x, y] = 1 if prob >= occupied_thresh else 0
    # Dilate occupied
    dilated = occ.filter(ImageFilter.MaxFilter(size=2 * radius_px + 1))
    out = Image.new("L", (w, h))
    out_px = out.load()
    dil_px = dilated.load()
    for x in range(w):
        for y in range(h):
            out_px[x, y] = 255 if dil_px[x, y] == 1 else px[x, y]
    return out


def world_to_map_xy(x: float, y: float, origin, resolution: float) -> Tuple[int, int]:
    x0, y0, _ = origin
    mx = int((x - x0) / resolution)
    my = int((y - y0) / resolution)
    return mx, my


def pixel_is_free(img: Image.Image, mx: int, my: int, free_thresh: float, occupied_thresh: float) -> Optional[bool]:
    w, h = img.size
    if mx < 0 or my < 0 or mx >= w or my >= h:
        return None
    # flip Y to match map coordinates vs. image rows
    pix = img.getpixel((mx, h - 1 - my))
    prob = pix / 255.0
    if prob >= occupied_thresh:
        return False
    elif prob <= free_thresh:
        return True
    else:
        return None


def filter_waypoints_by_map(
    waypoints: List[Tuple[float, float, float]],
    map_yaml_path: Optional[Path],
    inflation_radius_m: float = 0.0,
) -> List[Tuple[float, float, float]]:
    if map_yaml_path is None:
        return waypoints

    params = load_map_yaml(map_yaml_path)
    image_path = Path(map_yaml_path.parent) / params["image"]
    img = load_map_image(image_path, negate=int(params.get("negate", 0)))
    if inflation_radius_m > 0.0:
        img = inflate_mask(img, inflation_radius_m, float(params["resolution"]))

    origin = tuple(params["origin"])
    res = float(params["resolution"])
    free_thresh = float(params.get("free_thresh", 0.196))
    occupied_thresh = float(params.get("occupied_thresh", 0.65))

    filtered = []
    for (x, y, yaw) in waypoints:
        mx, my = world_to_map_xy(x, y, origin, res)
        ok = pixel_is_free(img, mx, my, free_thresh, occupied_thresh)
        if ok is True:
            filtered.append((x, y, yaw))
        # unknown/occupied skipped
    return filtered


# ---------- IO ----------

def write_csv(path: Path, waypoints: List[Tuple[float, float, float]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# x", "y", "yaw"])
        for x, y, yaw in waypoints:
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])


# ---------- CLI ----------

def parse_args():
    p = argparse.ArgumentParser(description="Trajectory generator with multiple patterns")
    # Position-related
    p.add_argument("--xmin", type=float, required=True, help="min X (meters, map frame)")
    p.add_argument("--xmax", type=float, required=True, help="max X")
    p.add_argument("--ymin", type=float, required=True, help="min Y")
    p.add_argument("--ymax", type=float, required=True, help="max Y")
    # Non-position
    p.add_argument("--spacing", type=float, default=0.5, help="grid/coverage spacing (meters)")
    p.add_argument("--pattern", choices=["raster", "zigzag", "spiral", "poisson"], default="raster",
                   help="coverage pattern")
    p.add_argument("--sweep-axis", choices=["x", "y"], default="y",
                   help="for raster/zigzag: traverse rows (y) or columns (x)")
    p.add_argument("--yaw-mode", choices=["fixed", "along_path"], default="fixed",
                   help="orientation mode: fixed yaw or along direction of motion")
    p.add_argument("--yaw", type=float, default=0.0, help="fixed yaw (radians) if yaw-mode=fixed")
    p.add_argument("--out", type=str, default="trajectory.csv", help="output CSV path")
    # Map filtering
    p.add_argument("--map-yaml", type=str, default=None,
                   help="SLAM map.yaml for obstacle/unknown filtering")
    p.add_argument("--inflate", type=float, default=0.0,
                   help="inflation radius (meters) for obstacle clearance")
    return p.parse_args()


def main():
    args = parse_args()
    xmin, xmax, ymin, ymax = clamp_bounds(args.xmin, args.xmax, args.ymin, args.ymax)

    # Generate waypoints by pattern
    if args.pattern in ("raster", "zigzag"):
        zigzag = (args.pattern == "zigzag")
        wps = generate_raster(
            xmin, xmax, ymin, ymax, args.spacing,
            sweep_axis=args.sweep_axis,
            zigzag=zigzag,
            yaw_mode=args.yaw_mode,
            fixed_yaw=args.yaw
        )
    elif args.pattern == "spiral":
        wps = generate_spiral(
            xmin, xmax, ymin, ymax, args.spacing,
            yaw_mode=args.yaw_mode, fixed_yaw=args.yaw
        )
    elif args.pattern == "poisson":
        wps = generate_poisson(
            xmin, xmax, ymin, ymax, args.spacing,
            yaw_mode=args.yaw_mode, fixed_yaw=args.yaw
        )
    else:
        raise ValueError(f"Unknown pattern: {args.pattern}")

    # Filter against map
    map_yaml = Path(args.map_yaml) if args.map_yaml else None
    if map_yaml:
        try:
            wps = filter_waypoints_by_map(wps, map_yaml_path=map_yaml, inflation_radius_m=args.inflate)
        except Exception as e:
            print(f"[WARN] Map filtering failed: {e}. Proceeding without filtering.")

    # Save
    out_path = Path(args.out)
    write_csv(out_path, wps)
    print(f"Saved {len(wps)} waypoints to {out_path}")


if __name__ == "__main__":
    main()

