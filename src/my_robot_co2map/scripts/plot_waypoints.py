import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# python3 /home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/scripts/plot_waypoints.py 

# --- CONFIGURATION ---
csv_path = Path("/home/valeria/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_co2map/ens160_logs/sensor_log_20251216_161421.csv")

# --- 1. LOAD DATA USING PANDAS ---
try:
    # Read the CSV file. The first row is the header.
    df = pd.read_csv(csv_path)

    # Check if the necessary columns exist
    required_cols = ['Pose_X', 'Pose_Y', 'eCO2']
    if not all(col in df.columns for col in required_cols):
        print(f"Error: CSV is missing one or more required columns: {required_cols}")
        print(f"Available columns: {df.columns.tolist()}")
        exit()

except FileNotFoundError:
    print(f"ERROR: CSV file not found at: {csv_path}")
    exit()
except Exception as e:
    print(f"An error occurred during file loading: {e}")
    exit()


# --- 2. PREPARE DATA FOR PLOTTING ---
# Extract the relevant columns
x_coords = df['Pose_X'].to_numpy()
y_coords = df['Pose_Y'].to_numpy()
co2_levels = df['eCO2'].to_numpy()

# --- 3. PLOT THE HEATMAP TRAJECTORY ---
plt.figure(figsize=(10, 8))

# Use a scatter plot to map colors to the eCO2 values.
# The 'c' argument specifies the data used for coloring, and 'cmap' defines the color scheme.
scatter = plt.scatter(x_coords, y_coords,
                      c=co2_levels,       # Color points based on eCO2
                      cmap='viridis',     # Colormap: Good contrast, low (blue) to high (yellow)
                      s=50,               # Size of the points
                      alpha=0.8,          # Transparency
                      label='Trajectory Points')

# Connect the points with a thin gray line to show the path
plt.plot(x_coords, y_coords, 
         linestyle='-', 
         color='gray', 
         linewidth=1, 
         alpha=0.5,
         zorder=0) # zorder=0 ensures the line is drawn beneath the colored points

# 4. Add Colorbar for Interpretation
cbar = plt.colorbar(scatter)
cbar.set_label('eCO2 Level (PPM)', fontsize=12)


# 5. Personalization and Display
plt.title(f"Robot Trajectory Colored by eCO2 Concentration")
plt.xlabel("Pose X (meters)", fontsize=12)
plt.ylabel("Pose Y (meters)", fontsize=12)
plt.grid(True, linestyle=':')
plt.axis('equal') # Ensure proportional scaling

plt.show()