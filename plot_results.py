#!/usr/bin/env python3
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_csv_flexible(path, expected_cols=15):
    """
    Loads a CSV that may:
      - have a header row (strings like 'quat_1')
      - have an extra first column (timestamp/index)
    Returns numpy array (N, expected_cols).
    """
    # First try reading with header=None
    df = pd.read_csv(path, header=None)

    # If the first row contains strings (e.g. 'quat_1'), re-read treating first row as header
    if df.shape[0] > 0 and any(isinstance(x, str) for x in df.iloc[0].values):
        df = pd.read_csv(path, header=0)   # use header row
    else:
        # no header row; keep header=None data
        pass

    # Coerce all to numeric (non-numeric become NaN)
    df = df.apply(pd.to_numeric, errors="coerce")

    # Drop fully empty rows (in case)
    df = df.dropna(how="all")

    # If there's an extra first column, drop it
    if df.shape[1] == expected_cols + 1:
        df = df.iloc[:, 1:]
    elif df.shape[1] != expected_cols:
        raise ValueError(
            f"{path}: Expected {expected_cols} cols (or {expected_cols+1} with an extra first col), "
            f"got {df.shape[1]} columns after parsing."
        )

    # If any NaNs remain, complain with a useful hint
    if df.isna().any().any():
        bad = df.isna().sum().sum()
        raise ValueError(f"{path}: Found {bad} non-numeric / missing entries after conversion to float.")

    return df.to_numpy(dtype=float)


def rotm_to_rpy(R):
    """
    Convert rotation matrices to roll-pitch-yaw (XYZ / roll-pitch-yaw) in radians.
    R: array (N, 3, 3)
    Returns rpy: array (N, 3) = [roll, pitch, yaw]
    Uses a common convention:
      roll  = atan2(R32, R33)
      pitch = asin(-R31)
      yaw   = atan2(R21, R11)
    """
    r11 = R[:, 0, 0]; r21 = R[:, 1, 0]; r31 = R[:, 2, 0]
    r32 = R[:, 2, 1]; r33 = R[:, 2, 2]

    pitch = np.arcsin(np.clip(-r31, -1.0, 1.0))
    roll  = np.arctan2(r32, r33)
    yaw   = np.arctan2(r21, r11)
    return np.stack([roll, pitch, yaw], axis=1)

def main():
    # Paths (edit if your filenames differ)
    est_path = os.path.join("log", "result.csv")

    # Ground truth: pick the first CSV inside data/ by default.
    # If you have multiple, either rename the GT file or set gt_path directly.
    gt_candidates = sorted(glob.glob(os.path.join("data", "*.csv")))
    if not gt_candidates:
        raise FileNotFoundError("No CSV found in data/. Put your GT csv there or update gt_path.")
    gt_path = gt_candidates[0]

    print(f"Estimate:    {est_path}")
    print(f"Ground truth:{gt_path}")

    est = load_csv_flexible(est_path, expected_cols=15)
    gt  = load_csv_flexible(gt_path, expected_cols=15)

    # Align length (if they differ)
    N = min(len(est), len(gt))
    est = est[:N]
    gt  = gt[:N]
    t = np.arange(N)

    # Split into components
    # Estimate: [R(9), p(3), v(3)]
    est_R = est[:, 0:9].reshape(N, 3, 3)
    est_p = est[:, 9:12]
    est_v = est[:, 12:15]

    # GT: columns quat_1..quat_9 are interpreted as the 9 elements of a 3x3 rotation matrix (not quaternion components).
    gt_R = gt[:, 0:9].reshape(N, 3, 3)
    gt_p = gt[:, 9:12]
    gt_v = gt[:, 12:15]

    # Orientation comparison as RPY (roll/pitch/yaw)
    est_rpy = rotm_to_rpy(est_R)
    gt_rpy  = rotm_to_rpy(gt_R)

    # ---- Plot 1: Orientation (RPY) ----
    fig1, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    labels = ["roll [rad]", "pitch [rad]", "yaw [rad]"]
    for i in range(3):
        ax[i].plot(t, gt_rpy[:, i], label="GT")
        ax[i].plot(t, est_rpy[:, i], label="Estimate", linestyle="--")
        ax[i].set_ylabel(labels[i])
        ax[i].grid(True)
    ax[0].legend()
    ax[-1].set_xlabel("sample")
    fig1.suptitle("Orientation comparison (RPY from rotation matrix)")

    # ---- Plot 2: Position ----
    fig2, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    labels = ["x [m]", "y [m]", "z [m]"]
    for i in range(3):
        ax[i].plot(t, gt_p[:, i], label="GT")
        ax[i].plot(t, est_p[:, i], label="Estimate", linestyle="--")
        ax[i].set_ylabel(labels[i])
        ax[i].grid(True)
    ax[0].legend()
    ax[-1].set_xlabel("sample")
    fig2.suptitle("Position comparison")

    # ---- Plot 3: Linear velocity ----
    fig3, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    labels = ["vx [m/s]", "vy [m/s]", "vz [m/s]"]
    for i in range(3):
        ax[i].plot(t, gt_v[:, i], label="GT")
        ax[i].plot(t, est_v[:, i], label="Estimate", linestyle="--")
        ax[i].set_ylabel(labels[i])
        ax[i].grid(True)
    ax[0].legend()
    ax[-1].set_xlabel("sample")
    fig3.suptitle("Velocity comparison")

    # ---- Plot 4: XY trajectory ----
    plt.figure(figsize=(8, 8))
    plt.plot(gt_p[:, 0], gt_p[:, 1], label="GT")
    plt.plot(est_p[:, 0], est_p[:, 1], label="Estimate", linestyle="--")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory (XY)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.show()

if __name__ == "__main__":
    main()
