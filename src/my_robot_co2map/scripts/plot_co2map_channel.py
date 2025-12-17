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
csv_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/ens160_logs/sensor_log_20251217_164439.csv")
yaml_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/Navigation_Projects/my_robot_navigation2/map/passadis.yaml")

channel_to_plot = 5  # which sensor channel to display
square_size = 0.3
eco2_min, eco2_max = 400, 1200
max_path_gap = 0.5

# -----------------------------
# LOAD SENSOR DATA
# -----------------------------
df = pd.read_csv(csv_path)

# Filter by channel
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

width_px, height_px = img.size
res = map_yaml['resolution']
origin_x, origin_y, _ = map_yaml['origin']

x_min = origin_x
y_min = origin_y
x_max = origin_x + width_px * res
y_max = origin_y + height_px * res

dx = res / 2
dy = res / 2
extent = [x_min - dx, x_max + dx, y_min - dy, y_max + dy]

# -----------------------------
# CREATE GRID
# -----------------------------
x_bins = np.arange(x_min, x_max, square_size)
y_bins = np.arange(y_min, y_max, square_size)
xx, yy = np.meshgrid(x_bins + square_size/2, y_bins + square_size/2)
co2_grid = np.zeros(xx.shape)

# -----------------------------
# WEIGHTED CO2
# -----------------------------
grid_points = np.column_stack([xx.ravel(), yy.ravel()])
sensor_points = np.column_stack([x_data, y_data])

for i, gp in enumerate(grid_points):
    dists = np.linalg.norm(sensor_points - gp, axis=1)
    weights = 1 / (dists**2 + 1e-6)
    co2_grid.ravel()[i] = np.sum(weights * co2_data) / np.sum(weights)

# -----------------------------
# PLOT
# -----------------------------
plt.figure(figsize=(15, 15))
plt.imshow(np.array(img), origin='lower', extent=extent, cmap='gray', alpha=0.3)

# Plot CO2 grid
im = plt.imshow(co2_grid, origin='lower', extent=extent, cmap='rainbow',
                interpolation='nearest', alpha=0.7, vmin=eco2_min, vmax=eco2_max, aspect='auto')

# Shorter colorbar
cbar = plt.colorbar(im, fraction=0.03, pad=0.02)  # fraction makes it thinner/shorter, pad is spacing
cbar.set_label('Average eCO2 Level (PPM)')

prev_x, prev_y = None, None
for x, y in zip(x_data, y_data):
    if prev_x is not None and np.hypot(x-prev_x, y-prev_y) < max_path_gap:
        plt.plot([prev_x, x], [prev_y, y], color='black', linewidth=2)
    prev_x, prev_y = x, y

plt.title(f"CO₂ Map (Channel {channel_to_plot}) with Robot Path")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.grid(False)
plt.axis('equal')
plt.show()
