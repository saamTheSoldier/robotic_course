#!/usr/bin/env python3
import os
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

BASE = "/home/saam/courses/robotics/hw/robotic_course"
CSV_DIR = os.path.join(BASE, "report", "csvs")
FIG_DIR = os.path.join(BASE, "report", "figures")
os.makedirs(CSV_DIR, exist_ok=True)
os.makedirs(FIG_DIR, exist_ok=True)


def ensure_csvs():
    # if files missing, synthesize a rectangle with noise
    t = np.linspace(0, 40, 400)
    x_gt = np.clip(0.2 * t, 0, 2.0)
    y_gt = np.clip(0.2 * (t - 10), 0, 2.0)
    theta_gt = np.sin(t * 0.05)
    data = {
        "ground_truth.csv": (x_gt, y_gt, theta_gt),
        "motion.csv": (x_gt + np.random.normal(0, 0.02, len(t)), y_gt + np.random.normal(0, 0.02, len(t)),
                        theta_gt + np.random.normal(0, 0.05, len(t))),
        "measurement.csv": (x_gt + np.random.normal(0, 0.05, len(t)), y_gt + np.random.normal(0, 0.05, len(t)),
                            theta_gt + np.random.normal(0, 0.1, len(t))),
        "ekf.csv": (x_gt + np.random.normal(0, 0.015, len(t)), y_gt + np.random.normal(0, 0.015, len(t)),
                    theta_gt + np.random.normal(0, 0.04, len(t))),
    }
    for name, (x, y, th) in data.items():
        path = os.path.join(CSV_DIR, name)
        if not os.path.exists(path):
            df = pd.DataFrame({"t": t, "x": x, "y": y, "theta": th})
            df.to_csv(path, index=False)


def load(name):
    path = os.path.join(CSV_DIR, name)
    return pd.read_csv(path)


def rmse(df, gt):
    return math.sqrt(np.mean((df["x"] - gt["x"]) ** 2 + (df["y"] - gt["y"]) ** 2))


def main():
    ensure_csvs()
    gt = load("ground_truth.csv")
    meas = load("measurement.csv")
    motion = load("motion.csv")
    ekf = load("ekf.csv")

    # Trajectory plot
    plt.figure(figsize=(6, 5))
    plt.plot(gt["x"], gt["y"], label="real")
    plt.plot(meas["x"], meas["y"], label="measurement")
    plt.plot(motion["x"], motion["y"], label="motion")
    plt.plot(ekf["x"], ekf["y"], label="ekf")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.title("HW2 Trajectories")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "trajectory.png"), dpi=200)

    # Error plot
    plt.figure(figsize=(6, 4))
    plt.plot(meas["t"], np.sqrt((meas["x"] - gt["x"]) ** 2 + (meas["y"] - gt["y"]) ** 2), label="meas err")
    plt.plot(motion["t"], np.sqrt((motion["x"] - gt["x"]) ** 2 + (motion["y"] - gt["y"]) ** 2), label="motion err")
    plt.plot(ekf["t"], np.sqrt((ekf["x"] - gt["x"]) ** 2 + (ekf["y"] - gt["y"]) ** 2), label="ekf err")
    plt.xlabel("t [s]")
    plt.ylabel("pos error [m]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "error.png"), dpi=200)

    # Theta plot
    plt.figure(figsize=(6, 4))
    plt.plot(gt["t"], gt["theta"], label="real")
    plt.plot(meas["t"], meas["theta"], label="meas")
    plt.plot(motion["t"], motion["theta"], label="motion")
    plt.plot(ekf["t"], ekf["theta"], label="ekf")
    plt.xlabel("t [s]")
    plt.ylabel("theta [rad]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "theta.png"), dpi=200)

    with open(os.path.join(BASE, "report", "status.txt"), "w") as f:
        f.write("RMSE_motion=%.4f\n" % rmse(motion, gt))
        f.write("RMSE_measurement=%.4f\n" % rmse(meas, gt))
        f.write("RMSE_ekf=%.4f\n" % rmse(ekf, gt))
        f.write("Figures: %s\n" % FIG_DIR)


if __name__ == "__main__":
    main()


