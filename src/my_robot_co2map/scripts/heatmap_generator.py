

#!/usr/bin/env python3
"""
Generate CO2 heatmap over ROS2 SLAM map.

Usage:

python3 heatmap_generator.py \
    --csv /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/ens160_logs/sensor_log_20251212_113802.csv \
    --map /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/Navigation_Projects/my_robot_navigation2/map/1ECO2MAP.yaml \
    --out /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/co2maps/co2map1_overlay.png
"""

import argparse
import pandas as pd
import numpy as np
import cv2
import yaml
import os

# Offset between measurement start and SLAM map origin
OFFSET_X = 3.71  # meters
OFFSET_Y = -3.33 # meters

def load_map(yaml_file):
    with open(yaml_file, 'r') as f:
        map_data = yaml.safe_load(f)
    map_path = os.path.join(os.path.dirname(yaml_file), map_data['image'])
    map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        raise FileNotFoundError(f"Map image not found: {map_path}")
    return map_img, map_data

def poses_to_pixels(x, y, map_data, map_img):
    """
    Convert poses in meters to pixel coordinates in the map.
    """
    resolution = map_data['resolution']
    origin_x, origin_y = map_data['origin'][:2]
    
    px = ((x - origin_x) / resolution).astype(int)
    py = map_img.shape[0] - ((y - origin_y) / resolution).astype(int)  # invert y-axis
    
    return px, py

def generate_heatmap(df, map_img, map_data):
    """
    Generate a heatmap overlay of eCO2 values on the map.
    """
    # Average eCO2 across channels per timestamp
    df_grouped = df.groupby(['Pose_X', 'Pose_Y']).agg({'eCO2': 'mean'}).reset_index()

    # Apply offset to align with SLAM map
    x_aligned = df_grouped['Pose_X'] - OFFSET_X
    y_aligned = df_grouped['Pose_Y'] - OFFSET_Y
    eco2 = df_grouped['eCO2'].values

    # Convert to pixel coordinates
    px, py = poses_to_pixels(x_aligned, y_aligned, map_data, map_img)

    # Create empty heatmap
    heatmap = np.zeros_like(map_img, dtype=float)
    count = np.zeros_like(map_img, dtype=int)

    # Accumulate eCO2 values
    for i in range(len(px)):
        if 0 <= px[i] < map_img.shape[1] and 0 <= py[i] < map_img.shape[0]:
            heatmap[py[i], px[i]] += eco2[i]
            count[py[i], px[i]] += 1

    # Average values
    mask = count > 0
    heatmap[mask] = heatmap[mask] / count[mask]

    # Mask walls/unknown
    if map_data.get('negate', 0) == 0:
        free_mask = map_img < map_data['occupied_thresh'] * 255
    else:
        free_mask = map_img > map_data['occupied_thresh'] * 255
    heatmap[~free_mask] = 0

    # Normalize heatmap to 0-255 for coloring
    if np.any(mask):
        heatmap_norm = cv2.normalize(heatmap, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    else:
        heatmap_norm = heatmap.astype(np.uint8)

    # Apply color map
    heatmap_color = cv2.applyColorMap(heatmap_norm, cv2.COLORMAP_JET)

    # Overlay on SLAM map
    map_color = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
    overlay = cv2.addWeighted(map_color, 0.6, heatmap_color, 0.4, 0)

    return overlay

def main():
    parser = argparse.ArgumentParser(description="Generate eCO2 heatmap on top of ROS2 SLAM map")
    parser.add_argument("--csv", required=True, help="CSV log file with eCO2 and poses")
    parser.add_argument("--map", required=True, help="Map YAML file")
    parser.add_argument("--out", required=True, help="Output image file")
    args = parser.parse_args()

    # Load CSV
    df = pd.read_csv(args.csv)
    map_img, map_data = load_map(args.map)

    overlay = generate_heatmap(df, map_img, map_data)

    cv2.imwrite(args.out, overlay)
    print(f"Heatmap saved to {args.out}")

if __name__ == "__main__":
    main()
