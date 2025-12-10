import csv
from pathlib import Path
import matplotlib.pyplot as plt
from PIL import Image


#   python3 /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/scripts/plot_waypoints.py


# Paths
map_yaml_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/slam_maps/test_rajoles1_justincase.yaml")
csv_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/trajectories/trajectory1.csv")

# Load map
import yaml
with open(map_yaml_path, "r") as f:
    map_data = yaml.safe_load(f)

img_path = Path(map_yaml_path.parent) / map_data["image"]
img = Image.open(img_path).convert("L")
resolution = map_data["resolution"]
origin_x, origin_y, _ = map_data["origin"]

# Flip image to match world coordinates
img = img.transpose(Image.FLIP_TOP_BOTTOM)
img_w, img_h = img.size

# Load waypoints
wps = []
with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:
        if row[0].startswith("#"):
            continue
        x, y, yaw = map(float, row)
        wps.append((x, y))

# Convert world coordinates to pixels
px = [(int((x - origin_x) / resolution), int((y - origin_y) / resolution)) for x, y in wps]

# Plot
plt.figure(figsize=(8, 6))
plt.imshow(img, cmap="gray", origin="lower")
px_vals, py_vals = zip(*px)
plt.scatter(px_vals, py_vals, c="red", s=40)
plt.title(f"{len(wps)} waypoints on map")
plt.show()
