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
csv_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/ens160_logs/sensor_log_20251217_142216.csv")
yaml_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/Navigation_Projects/my_robot_navigation2/map/passadis.yaml")

square_size = 0.3  # meters per grid square

# -----------------------------
# LOAD SENSOR DATA
# -----------------------------
df = pd.read_csv(csv_path)

# Group by Pose_X and Pose_Y, average eCO2 across all channels for that position
grouped = df.groupby(['Pose_X','Pose_Y'])['eCO2'].mean().reset_index()

x_data = grouped['Pose_X'].to_numpy()
y_data = grouped['Pose_Y'].to_numpy()
co2_data = grouped['eCO2'].to_numpy()

# -----------------------------
# LOAD MAP YAML
# -----------------------------
with open(yaml_path) as f:
    map_yaml = yaml.safe_load(f)

image_path = Path(map_yaml['image'])
if not image_path.is_absolute():
    image_path = yaml_path.parent / image_path

img = Image.open(image_path)
width_px, height_px = img.size
res = map_yaml['resolution']
origin_x, origin_y, _ = map_yaml['origin']

x_min = origin_x
y_min = origin_y
x_max = origin_x + width_px * res
y_max = origin_y + height_px * res

print(f"Map bounds (meters): x=[{x_min}, {x_max}], y=[{y_min}, {y_max}]")

# -----------------------------
# CREATE GRID COVERING FULL MAP
# -----------------------------
x_bins = np.arange(x_min, x_max, square_size)
y_bins = np.arange(y_min, y_max, square_size)

xx, yy = np.meshgrid(x_bins + square_size/2, y_bins + square_size/2)  # center of each square

grid_shape = xx.shape
co2_grid = np.zeros(grid_shape)

# -----------------------------
# COMPUTE WEIGHTED CO2 USING INVERSE DISTANCE SQUARED
# -----------------------------
grid_points = np.column_stack([xx.ravel(), yy.ravel()])
sensor_points = np.column_stack([x_data, y_data])

# Compute weighted average per square
for i, gp in enumerate(grid_points):
    dists = np.linalg.norm(sensor_points - gp, axis=1)
    weights = 1 / (dists**2 + 1e-6)  # avoid division by zero
    co2_grid.ravel()[i] = np.sum(weights * co2_data) / np.sum(weights)

# -----------------------------
# PLOT  MAP
# -----------------------------
plt.figure(figsize=(12, 6))
extent = [x_min, x_max, y_min, y_max]
plt.imshow(co2_grid, origin='lower', extent=extent, cmap='rainbow', interpolation='nearest')
plt.colorbar(label='Average eCO2 Level (PPM)')
plt.title("CO₂ Map")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.grid(False)
plt.show()
