#!/usr/bin/env python3
"""
This script:
 - Reads a raw sensor CSV (per-channel rows with Timestamp, Channel, eCO2, etc.)
 - Applies per-channel baseline drift removal (linear detrend + shift to 400 ppm)
 - Aggregates sensor readings by timestamp (mean across channels -> avg_eCO2)
 - Merges sensor averages with a position CSV (poses with Timestamp, X, Y)
 - Writes merged CSV ready for heatmap generation


terminal command format:
 python3 process_data.py \
  --pos_csv map_data_testrajoles2.csv \
  --sensor_csv sensor_data_testrajoles2.csv \
  --remove-drift \
  --out merged_for_heatmap.csv

"""

import argparse
import pandas as pd
import numpy as np
import os

def remove_drift_per_channel(df, time_col='Timestamp', value_col='eCO2',
                             channel_col='Channel', baseline_min=400.0):
    """
    Detrend each channel separately and return processed_df.
    """
    df = df.copy()
    df[time_col] = pd.to_datetime(df[time_col], errors='coerce')
    global_t0 = df[time_col].min()
    df['Time_s'] = (df[time_col] - global_t0).dt.total_seconds().astype(float)

    def detrend_group(g):
        X = g['Time_s'].values
        y = g[value_col].values.astype(float)
        if len(X) < 2 or np.all(np.isnan(y)):
            g[channel_col] = g.name
            return g
        try:
            m, b = np.polyfit(X, y, 1)
        except Exception:
            m, b = 0.0, np.nanmean(y)
        linear_drift = m * X
        detrended = y - linear_drift
        shift_amount = baseline_min - np.nanmin(detrended)
        g[value_col] = detrended + shift_amount
        g[channel_col] = g.name
        return g

    try:
        processed_df = df.groupby(channel_col, group_keys=False,
                                  include_groups=False).apply(detrend_group)
    except TypeError:
        # fallback for older pandas
        processed_df = df.groupby(channel_col, group_keys=False).apply(detrend_group)

    processed_df = processed_df.drop(columns=['Time_s'])
    return processed_df

def aggregate_sensor(df_sensor, sensor_time_col='Timestamp', agg_col='eCO2'):
    df_sensor_sorted = df_sensor.sort_values(sensor_time_col)
    eco2_avg = (df_sensor_sorted.groupby(sensor_time_col)[agg_col]
                .mean()
                .reset_index()
                .rename(columns={agg_col: 'avg_eCO2'}))
    return eco2_avg

def merge_positions_and_sensor(df_pos, eco2_avg,
                               pos_time_col='Timestamp',
                               sensor_time_col='Timestamp',
                               tolerance_s=1.0):
    df_pos_sorted = df_pos.sort_values(pos_time_col).reset_index(drop=True)
    eco2_sorted = eco2_avg.sort_values(sensor_time_col).reset_index(drop=True)
    tol = pd.Timedelta(seconds=tolerance_s)
    merged = pd.merge_asof(df_pos_sorted, eco2_sorted,
                           left_on=pos_time_col,
                           right_on=sensor_time_col,
                           direction='nearest',
                           tolerance=tol)
    merged = merged.dropna(subset=['avg_eCO2'])
    return merged

def main():
    parser = argparse.ArgumentParser(description="Prepare merged CSV for heatmap generation")
    parser.add_argument('--pos_csv', required=True, help='Position CSV with Timestamp and X/Y columns')
    parser.add_argument('--sensor_csv', required=True, help='Sensor CSV with Timestamp, Channel, eCO2, ...')
    parser.add_argument('--out', default='merged_for_heatmap.csv', help='Output merged CSV path')
    parser.add_argument('--remove-drift', action='store_true', help='Apply baseline drift removal per channel')
    parser.add_argument('--tolerance', type=float, default=1.0, help='merge_asof tolerance in seconds')
    args = parser.parse_args()

    # Load CSVs
    df_pos = pd.read_csv(args.pos_csv)
    df_sensor = pd.read_csv(args.sensor_csv)

    # Parse timestamps
    df_pos['Timestamp'] = pd.to_datetime(df_pos['Timestamp'], errors='coerce')
    df_sensor['Timestamp'] = pd.to_datetime(df_sensor['Timestamp'], errors='coerce')

    # Optionally detrend sensor data
    if args.remove_drift:
        print("Applying baseline drift removal...")
        df_sensor = remove_drift_per_channel(df_sensor, value_col='eCO2', channel_col='Channel')

    # Aggregate sensor readings
    eco2_avg = aggregate_sensor(df_sensor, sensor_time_col='Timestamp', agg_col='eCO2')

    # Merge with positions
    merged = merge_positions_and_sensor(df_pos, eco2_avg,
                                        pos_time_col='Timestamp',
                                        sensor_time_col='Timestamp',
                                        tolerance_s=args.tolerance)

    # Normalize column names
    if 'pose_x' not in merged.columns and 'PositionX' in merged.columns:
        merged = merged.rename(columns={'PositionX': 'pose_x', 'PositionY': 'pose_y'})

    # Save
    merged.to_csv(args.out, index=False)
    print(f"Saved merged CSV with {len(merged)} rows to {args.out}")

if __name__ == '__main__':
    main()
