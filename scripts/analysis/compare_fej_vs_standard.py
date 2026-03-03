#!/usr/bin/env python3
import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# WGS-84
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def llh_to_ecef(lat_deg, lon_deg, h_m):
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - WGS84_E2) + h_m) * sin_lat
    return x, y, z


def load_sol(path):
    df = pd.read_csv(path, sep=r"\s+", comment="#")
    required = [
        "timestamp", "fused_x", "fused_y", "fused_z",
        "gnss_lever_x", "gnss_lever_y", "gnss_lever_z",
    ]
    for c in required:
        if c not in df.columns:
            raise ValueError(f"missing column {c} in {path}")
    return df


def load_truth(path):
    with open(path, "r", encoding="utf-8") as f:
        first = f.readline().strip().split()
    has_header = bool(first) and not first[0].replace('.', '', 1).replace('-', '', 1).isdigit()
    if has_header:
        df = pd.read_csv(path, sep=r"\s+")
        rename = {"x": "tx", "y": "ty", "z": "tz", "height": "h"}
        df = df.rename(columns=rename)
        if "timestamp" not in df.columns:
            raise ValueError(f"truth file has header but no timestamp column: {path}")
        has_ecef = all(c in df.columns for c in ["tx", "ty", "tz"])
        has_llh = all(c in df.columns for c in ["lat", "lon", "h"])
        if has_llh and not has_ecef:
            x, y, z = llh_to_ecef(
                df["lat"].to_numpy(),
                df["lon"].to_numpy(),
                df["h"].to_numpy(),
            )
            df["tx"] = x
            df["ty"] = y
            df["tz"] = z
    else:
        df = pd.read_csv(path, sep=r"\s+", header=None)
        if df.shape[1] < 4:
            raise ValueError(f"truth file needs at least 4 columns: {path}")
        df = df.iloc[:, :4]
        df.columns = ["timestamp", "tx", "ty", "tz"]
    return df[["timestamp", "tx", "ty", "tz"]]


def interp_truth(truth, t):
    tt = truth["timestamp"].to_numpy()
    tx = np.interp(t, tt, truth["tx"].to_numpy())
    ty = np.interp(t, tt, truth["ty"].to_numpy())
    tz = np.interp(t, tt, truth["tz"].to_numpy())
    return np.column_stack([tx, ty, tz])


def rmse(err):
    return np.sqrt(np.mean(err * err, axis=0))


def main():
    ap = argparse.ArgumentParser(description="Compare standard ESKF vs FEJ-ESKF outputs")
    ap.add_argument("--std", default="SOL_data4_gnss_std.txt")
    ap.add_argument("--fej", default="SOL_data4_gnss_fej.txt")
    ap.add_argument("--truth", default="dataset/data4_converted/POS_converted.txt")
    ap.add_argument("--outdir", default="output/compare_data4_std_vs_fej")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    std = load_sol(args.std)
    fej = load_sol(args.fej)
    truth = load_truth(args.truth)

    t_std = std["timestamp"].to_numpy()
    t_fej = fej["timestamp"].to_numpy()

    p_std = std[["fused_x", "fused_y", "fused_z"]].to_numpy()
    p_fej = fej[["fused_x", "fused_y", "fused_z"]].to_numpy()

    p_true_std = interp_truth(truth, t_std)
    p_true_fej = interp_truth(truth, t_fej)

    e_std = p_std - p_true_std
    e_fej = p_fej - p_true_fej

    en_std = np.linalg.norm(e_std, axis=1)
    en_fej = np.linalg.norm(e_fej, axis=1)

    rmse_std = rmse(e_std)
    rmse_fej = rmse(e_fej)

    summary = {
        "std": {
            "rmse_xyz_m": rmse_std.tolist(),
            "rmse_3d_m": float(np.sqrt(np.mean(en_std ** 2))),
            "mean_3d_m": float(np.mean(en_std)),
            "p95_3d_m": float(np.percentile(en_std, 95)),
        },
        "fej": {
            "rmse_xyz_m": rmse_fej.tolist(),
            "rmse_3d_m": float(np.sqrt(np.mean(en_fej ** 2))),
            "mean_3d_m": float(np.mean(en_fej)),
            "p95_3d_m": float(np.percentile(en_fej, 95)),
        },
    }

    with open(os.path.join(args.outdir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    with open(os.path.join(args.outdir, "summary.txt"), "w", encoding="utf-8") as f:
        f.write("Standard ESKF\n")
        f.write(f"  RMSE xyz [m]: {rmse_std[0]:.6f}, {rmse_std[1]:.6f}, {rmse_std[2]:.6f}\n")
        f.write(f"  RMSE 3D  [m]: {summary['std']['rmse_3d_m']:.6f}\n")
        f.write(f"  Mean 3D  [m]: {summary['std']['mean_3d_m']:.6f}\n")
        f.write(f"  P95  3D  [m]: {summary['std']['p95_3d_m']:.6f}\n\n")
        f.write("FEJ-ESKF\n")
        f.write(f"  RMSE xyz [m]: {rmse_fej[0]:.6f}, {rmse_fej[1]:.6f}, {rmse_fej[2]:.6f}\n")
        f.write(f"  RMSE 3D  [m]: {summary['fej']['rmse_3d_m']:.6f}\n")
        f.write(f"  Mean 3D  [m]: {summary['fej']['mean_3d_m']:.6f}\n")
        f.write(f"  P95  3D  [m]: {summary['fej']['p95_3d_m']:.6f}\n")

    # trajectory comparison (ECEF XY)
    plt.figure(figsize=(10, 8))
    plt.plot(p_true_std[:, 0], p_true_std[:, 1], "k-", lw=1.0, label="truth")
    plt.plot(p_std[:, 0], p_std[:, 1], "b-", lw=0.8, label="standard")
    plt.plot(p_fej[:, 0], p_fej[:, 1], "r-", lw=0.8, label="fej")
    plt.xlabel("ECEF X [m]")
    plt.ylabel("ECEF Y [m]")
    plt.title("Trajectory Comparison")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(args.outdir, "01_trajectory_compare.png"), dpi=150)
    plt.close()

    # position error norm comparison
    t0 = min(t_std[0], t_fej[0])
    plt.figure(figsize=(12, 5))
    plt.plot(t_std - t0, en_std, "b-", lw=0.8, label="standard |e_p|")
    plt.plot(t_fej - t0, en_fej, "r-", lw=0.8, label="fej |e_p|")
    plt.xlabel("time [s]")
    plt.ylabel("position error norm [m]")
    plt.title("Position Error Norm")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(args.outdir, "02_position_error_norm_compare.png"), dpi=150)
    plt.close()

    # xyz error comparison
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    labels = ["X", "Y", "Z"]
    for i, ax in enumerate(axes):
        ax.plot(t_std - t0, e_std[:, i], "b-", lw=0.8, label="standard")
        ax.plot(t_fej - t0, e_fej[:, i], "r-", lw=0.8, label="fej")
        ax.set_ylabel(f"{labels[i]} err [m]")
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend()
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Position Error XYZ")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(args.outdir, "03_position_error_xyz_compare.png"), dpi=150)
    plt.close(fig)

    # GNSS lever comparison
    gl_std = std[["gnss_lever_x", "gnss_lever_y", "gnss_lever_z"]].to_numpy()
    gl_fej = fej[["gnss_lever_x", "gnss_lever_y", "gnss_lever_z"]].to_numpy()
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t_std - t0, gl_std[:, i], "b-", lw=0.8, label="standard")
        ax.plot(t_fej - t0, gl_fej[:, i], "r-", lw=0.8, label="fej")
        ax.set_ylabel(f"l_gnss[{i}] [m]")
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend()
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("GNSS Lever Arm Estimate")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(os.path.join(args.outdir, "04_gnss_lever_compare.png"), dpi=150)
    plt.close(fig)

    print(json.dumps(summary, indent=2))
    print(f"saved to: {os.path.abspath(args.outdir)}")


if __name__ == "__main__":
    main()
