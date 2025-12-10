#!/usr/bin/env python3
"""
Standalone SLAM YAML analyzer for trajectory planning.
"""
 
# python3 ~/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/scripts/yaml_analyzer.py test_rajoles1_justincase.yaml


import yaml
from pathlib import Path
from PIL import Image

def analyze_map(yaml_path):
    yaml_path = Path(yaml_path)
    if not yaml_path.exists():
        print(f"[ERROR] File not found: {yaml_path}")
        return

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    print("=== Map Info ===")
    print(f"Image file: {data.get('image')}")
    print(f"Mode: {data.get('mode')}")
    print(f"Resolution: {data.get('resolution')} m/cell")
    origin = data.get('origin', [0,0,0])
    print(f"Origin (x, y, yaw): {origin}")
    print(f"Occupied threshold: {data.get('occupied_thresh')}")
    print(f"Free threshold: {data.get('free_thresh')}")
    print(f"Negate: {data.get('negate')}")

    # Estimate map size in meters
    image_path = yaml_path.parent / data['image']
    img = Image.open(image_path)
    width, height = img.size
    print(f"Map image size: {width} x {height} pixels")
    print(f"Map physical size: {width*data['resolution']:.2f} x {height*data['resolution']:.2f} meters")

    # Suggest trajectory bounds
    xmin = origin[0]
    ymin = origin[1]
    xmax = xmin + width * data['resolution']
    ymax = ymin + height * data['resolution']
    print(f"Suggested trajectory bounds:")
    print(f"  xmin: {xmin:.2f}, xmax: {xmax:.2f}")
    print(f"  ymin: {ymin:.2f}, ymax: {ymax:.2f}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Analyze SLAM map YAML for trajectory planning")
    parser.add_argument("yaml_file", help="Path to the map YAML file")
    args = parser.parse_args()
    analyze_map(args.yaml_file)
