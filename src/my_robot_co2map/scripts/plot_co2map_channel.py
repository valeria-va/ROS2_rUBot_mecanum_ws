#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from PIL import Image
import yaml

# -----------------------------
# CONFIGURATION
# -----------------------------
csv_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/ens160_logs/sensor_log_20251218_165354.csv")
yaml_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/Navigation_Projects/my_robot_navigation2/map/rajoles.yaml")

channel_to_plot = 5
square_size = 0.3
eco2_min, eco2_max = 400, 1200
max_path_gap = 0.5

# -----------------------------
# LOAD SENSOR DATA
# -----------------------------
df = pd.read_csv(csv_path)
df_ch = df[df['Channel'] == channel_to_plot]

x_data = df_ch['Pose_X'].to_numpy()
y_data = df_ch['Pose_Y'].to_numpy()
co2_data = df_ch['eCO2'].clip(eco2_min, eco2_max).to_numpy()

# -----------------------------
# LOAD MAP YAML
# -----------------------------
with open(yaml_path) as f:
    map_yaml = yaml.safe_load(f)

image_path = Path(map_yaml['image'])
if not image_path.is_absolute():
    image_path = yaml_path.parent / image_path

img = Image.open(image_path)
img = img.transpose(Image.FLIP_TOP_BOTTOM)
img = img.rotate(90, expand=True)  # <<< ROTATE IMAGE

res = map_yaml['resolution']
origin_x, origin_y, _ = map_yaml['origin']

# Original bounds
orig_width_px, orig_height_px = Image.open(image_path).size
x_min = origin_x
y_min = origin_y
x_max = origin_x + orig_width_px * res
y_max = origin_y + orig_height_px * res

# New bounds after rotation
new_x_min = 0.0
new_y_min = 0.0
new_x_max = y_max - y_min
new_y_max = x_max - x_min

extent = [new_x_min, new_x_max, new_y_min, new_y_max]

# -----------------------------
# ROTATE SENSOR DATA (90° CCW)
# -----------------------------
x_rot = y_max - y_data
y_rot = x_data - x_min

# -----------------------------
# CREATE GRID
# -----------------------------
x_bins = np.arange(new_x_min, new_x_max, square_size)
y_bins = np.arange(new_y_min, new_y_max, square_size)
xx, yy = np.meshgrid(x_bins + square_size / 2,
                     y_bins + square_size / 2)

co2_grid = np.zeros(xx.shape)

# -----------------------------
# INVERSE DISTANCE WEIGHTING
# -----------------------------
grid_points = np.column_stack([xx.ravel(), yy.ravel()])
sensor_points = np.column_stack([x_rot, y_rot])

for i, gp in enumerate(grid_points):
    dists = np.linalg.norm(sensor_points - gp, axis=1)
    weights = 1.0 / (dists**2 + 1e-6)
    co2_grid.ravel()[i] = np.sum(weights * co2_data) / np.sum(weights)

# -----------------------------
# PLOT
# -----------------------------
plt.figure(figsize=(15, 15))
plt.imshow(np.array(img), origin='lower', extent=extent, cmap='gray', alpha=0.3)

im = plt.imshow(co2_grid, origin='lower', extent=extent,
                cmap='rainbow', interpolation='nearest',
                alpha=0.7, vmin=eco2_min, vmax=eco2_max)

cbar = plt.colorbar(im, fraction=0.03, pad=0.02)
cbar.set_label('Average eCO₂ (PPM)')

# Rotated path
prev_x, prev_y = None, None
for x, y in zip(x_rot, y_rot):
    if prev_x is not None and np.hypot(x - prev_x, y - prev_y) < max_path_gap:
        plt.plot([prev_x, x], [prev_y, y], color='black', linewidth=2)
    prev_x, prev_y = x, y

plt.title(f"CO₂ Map (Channel {channel_to_plot}) – Rotated 90°")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.axis('equal')
plt.grid(False)
plt.show()
