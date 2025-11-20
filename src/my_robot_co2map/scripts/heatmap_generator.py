#!/usr/bin/env python3
"""
Generate a CO₂ heatmap from merged position+sensor CSV.

Usage:
  python3 heatmap_generator.py \
    --csv merged_for_heatmap.csv \
    --out heatmap.png \
    --black-for-empty

    png is optional, the script saves the plotted heatmap as an image file 
    instead of (or in addition to) showing it interactively.
"""

import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def build_heatmap(df, x_col="pose_x", y_col="pose_y", value_col="avg_eCO2",
                  black_for_empty=True, out=None):
    # Clean data
    df = df.dropna(subset=[x_col, y_col, value_col])
    df[x_col] = df[x_col].astype(int)
    df[y_col] = df[y_col].astype(int)

    min_x, min_y = df[x_col].min(), df[y_col].min()
    max_x, max_y = df[x_col].max(), df[y_col].max()

    cols = max_x - min_x + 1
    rows = max_y - min_y + 1
    print(f"Map X range: {min_x}–{max_x}, Y range: {min_y}–{max_y}")

    heatmap = np.full((rows, cols), np.nan)

    for _, row in df.iterrows():
        x_idx = row[x_col] - min_x
        y_idx = row[y_col] - min_y
        if 0 <= x_idx < cols and 0 <= y_idx < rows:
            heatmap[y_idx, x_idx] = row[value_col]

    plt.figure(figsize=(10, 8))
    cmap = plt.cm.jet.copy()
    if black_for_empty:
        cmap.set_bad(color="black")

    plt.imshow(heatmap, cmap=cmap, interpolation="nearest", origin="lower")
    plt.title("CO₂ Heatmap")

    # Custom ticks
    x_ticks = np.arange(0, cols, max(1, cols // 9))
    y_ticks = np.arange(0, rows, max(1, rows // 9))
    plt.xticks(x_ticks, x_ticks + min_x)
    plt.yticks(y_ticks, y_ticks + min_y)

    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.colorbar(label="eCO₂ (ppm)")
    plt.tight_layout()

    if out:
        plt.savefig(out, dpi=150)
        print(f"Saved heatmap to {out}")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Generate CO₂ heatmap from merged CSV")
    parser.add_argument("--csv", required=True, help="Merged CSV with pose_x, pose_y, avg_eCO2")
    parser.add_argument("--out", help="Optional output image file (PNG)")
    parser.add_argument("--black-for-empty", action="store_true",
                        help="Render empty cells as black")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    build_heatmap(df, black_for_empty=args.black_for_empty, out=args.out)

if __name__ == "__main__":
    main()
