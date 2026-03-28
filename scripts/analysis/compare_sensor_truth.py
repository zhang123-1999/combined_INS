#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compare GNSS/ODO measurements against truth with time alignment and frame handling.

Outputs:
- JSON summary with robust statistics and top outliers
- Aligned CSV files
- Diagnostic figures (time series + histogram)

Default behavior:
- Read paths and IMU->Vehicle mounting angles from a YAML config if provided
- Fallback to explicit CLI paths
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import yaml
except Exception:
    yaml = None


# WGS-84
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def load_numeric_matrix(path: Path, min_cols: int) -> np.ndarray:
    rows: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            s = s.replace(",", " ")
            parts = s.split()
            vals: List[float] = []
            ok = True
            for p in parts:
                try:
                    vals.append(float(p))
                except ValueError:
                    ok = False
                    break
            if not ok or len(vals) < min_cols:
                continue
            rows.append(vals)

    if not rows:
        raise ValueError(f"no numeric rows in {path}")

    ncol = min(len(r) for r in rows)
    if ncol < min_cols:
        raise ValueError(f"insufficient numeric columns in {path}: {ncol} < {min_cols}")
    arr = np.array([r[:ncol] for r in rows], dtype=float)
    return arr


def llh_to_ecef(lat_rad: np.ndarray, lon_rad: np.ndarray, h_m: np.ndarray) -> np.ndarray:
    sl = np.sin(lat_rad)
    cl = np.cos(lat_rad)
    so = np.sin(lon_rad)
    co = np.cos(lon_rad)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sl * sl)
    x = (n + h_m) * cl * co
    y = (n + h_m) * cl * so
    z = (n * (1.0 - WGS84_E2) + h_m) * sl
    return np.column_stack([x, y, z])


def ecef_to_llh_single(x: float, y: float, z: float) -> Tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(6):
        sl = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
        lat = math.atan2(z + WGS84_E2 * n * sl, p)
    sl = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    h = p / max(1e-12, math.cos(lat)) - n
    return lat, lon, h


def rot_ned_to_ecef(lat: float, lon: float) -> np.ndarray:
    sl = math.sin(lat)
    cl = math.cos(lat)
    so = math.sin(lon)
    co = math.cos(lon)
    return np.array(
        [
            [-sl * co, -so, -cl * co],
            [-sl * so, co, -cl * so],
            [cl, 0.0, -sl],
        ],
        dtype=float,
    )


def ecef_vel_to_ned(v_ecef: np.ndarray, lat: float, lon: float) -> np.ndarray:
    r_ne = rot_ned_to_ecef(lat, lon)
    return r_ne.T @ v_ecef


def euler_to_rot_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def interp_cols(t_src: np.ndarray, x_src: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    out = np.zeros((t_query.size, x_src.shape[1]), dtype=float)
    for i in range(x_src.shape[1]):
        out[:, i] = np.interp(t_query, t_src, x_src[:, i])
    return out


def interp_angles_deg(t_src: np.ndarray, ang_deg: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    rad = np.deg2rad(ang_deg)
    rad_u = np.unwrap(rad, axis=0)
    out = np.zeros((t_query.size, ang_deg.shape[1]), dtype=float)
    for i in range(ang_deg.shape[1]):
        out[:, i] = np.rad2deg(np.interp(t_query, t_src, rad_u[:, i]))
    return out


def derive_gnss_velocity_ned(t_gnss: np.ndarray, p_gnss_ecef: np.ndarray, lat_rad: np.ndarray,
                             lon_rad: np.ndarray) -> np.ndarray:
    """Estimate GNSS NED velocity from position by finite differences."""
    n = t_gnss.size
    if n == 0:
        return np.zeros((0, 3), dtype=float)
    if n == 1:
        return np.zeros((1, 3), dtype=float)

    v_ecef = np.zeros((n, 3), dtype=float)
    dt01 = max(1e-9, t_gnss[1] - t_gnss[0])
    v_ecef[0] = (p_gnss_ecef[1] - p_gnss_ecef[0]) / dt01
    dtn = max(1e-9, t_gnss[-1] - t_gnss[-2])
    v_ecef[-1] = (p_gnss_ecef[-1] - p_gnss_ecef[-2]) / dtn
    for i in range(1, n - 1):
        dt = max(1e-9, t_gnss[i + 1] - t_gnss[i - 1])
        v_ecef[i] = (p_gnss_ecef[i + 1] - p_gnss_ecef[i - 1]) / dt

    v_ned = np.zeros_like(v_ecef)
    for i in range(n):
        v_ned[i] = ecef_vel_to_ned(v_ecef[i], float(lat_rad[i]), float(lon_rad[i]))
    return v_ned


def stats(x: np.ndarray) -> Dict[str, float]:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {}
    return {
        "count": int(x.size),
        "mean": float(np.mean(x)),
        "std": float(np.std(x)),
        "median": float(np.median(x)),
        "p95": float(np.percentile(x, 95)),
        "p99": float(np.percentile(x, 99)),
        "min": float(np.min(x)),
        "max": float(np.max(x)),
    }


def top_k_events(t: np.ndarray, v: np.ndarray, k: int, extra: Optional[np.ndarray] = None) -> List[Dict[str, float]]:
    if t.size == 0:
        return []
    k = max(1, min(k, t.size))
    idx = np.argsort(v)[::-1][:k]
    out: List[Dict[str, float]] = []
    for i in idx:
        item = {"t": float(t[i]), "value": float(v[i])}
        if extra is not None and extra.ndim == 2 and extra.shape[0] == t.size:
            item["e1"] = float(extra[i, 0])
            if extra.shape[1] > 1:
                item["e2"] = float(extra[i, 1])
            if extra.shape[1] > 2:
                item["e3"] = float(extra[i, 2])
        out.append(item)
    return out


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def resolve_path(base: Path, p: Optional[str]) -> Optional[Path]:
    if p is None or str(p).strip() == "":
        return None
    pp = Path(p)
    if not pp.is_absolute():
        pp = base / pp
    return pp


def load_config(path: Optional[Path]) -> Dict:
    if path is None or not path.exists() or yaml is None:
        return {}
    with path.open("r", encoding="utf-8") as f:
        obj = yaml.safe_load(f)
    return obj if isinstance(obj, dict) else {}


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare GNSS/ODO measurements vs truth.")
    parser.add_argument("--config", type=str, default="config_data2_gnss.yaml", help="YAML config path")
    parser.add_argument("--truth", type=str, default=None, help="truth POS_converted path")
    parser.add_argument("--gnss", type=str, default=None, help="GNSS_converted path")
    parser.add_argument("--odo", type=str, default=None, help="ODO_converted path")
    parser.add_argument("--out-dir", type=str, default="output/sensor_validation", help="output directory")
    parser.add_argument("--mounting-rpy-deg", type=float, nargs=3, default=None,
                        help="IMU->Vehicle mounting roll pitch yaw (deg), override config")
    parser.add_argument("--top-k", type=int, default=20, help="top outlier count")
    parser.add_argument(
        "--derive-gnss-vel-if-missing",
        action="store_true",
        help="derive GNSS velocity from position when GNSS file has no velocity columns",
    )
    args = parser.parse_args()

    cwd = Path.cwd()
    cfg = load_config(resolve_path(cwd, args.config))
    fusion_cfg = cfg.get("fusion", {}) if isinstance(cfg.get("fusion", {}), dict) else {}
    cst_cfg = fusion_cfg.get("constraints", {}) if isinstance(fusion_cfg.get("constraints", {}), dict) else {}

    truth_path = resolve_path(cwd, args.truth or fusion_cfg.get("pos_path"))
    gnss_path = resolve_path(cwd, args.gnss or fusion_cfg.get("gnss_path"))
    odo_path = resolve_path(cwd, args.odo or fusion_cfg.get("odo_path"))
    out_dir = resolve_path(cwd, args.out_dir)
    assert out_dir is not None
    ensure_dir(out_dir)

    if truth_path is None or not truth_path.exists():
        raise FileNotFoundError("truth file is required and not found")
    if gnss_path is None or not gnss_path.exists():
        raise FileNotFoundError("gnss file is required and not found")
    if odo_path is None or not odo_path.exists():
        raise FileNotFoundError("odo file is required and not found")

    # Stored mounting angle follows the same convention as the solver.
    # The runtime C_b_v uses the inverse Euler rotation.
    if args.mounting_rpy_deg is not None:
        mounting_rpy_deg = np.array(args.mounting_rpy_deg, dtype=float)
    else:
        mounting_rpy_deg = np.array(cst_cfg.get("imu_mounting_angle", [0.0, 0.0, 0.0]), dtype=float)
        if mounting_rpy_deg.size != 3:
            mounting_rpy_deg = np.array([0.0, 0.0, 0.0], dtype=float)
    c_b_v = euler_to_rot_zyx(
        math.radians(mounting_rpy_deg[0]),
        math.radians(mounting_rpy_deg[1]),
        math.radians(mounting_rpy_deg[2]),
    ).T

    # -------------------------
    # Truth
    # -------------------------
    truth = load_numeric_matrix(truth_path, min_cols=10)
    t_truth = truth[:, 0]
    pos_truth_raw = truth[:, 1:4]
    vel_truth_raw = truth[:, 4:7]
    rpy_truth_deg = truth[:, 7:10]

    truth_is_lla = np.max(np.abs(pos_truth_raw[0, :2])) < 200.0
    if truth_is_lla:
        lat_truth_rad = np.deg2rad(pos_truth_raw[:, 0])
        lon_truth_rad = np.deg2rad(pos_truth_raw[:, 1])
        h_truth = pos_truth_raw[:, 2]
        p_truth_ecef = llh_to_ecef(lat_truth_rad, lon_truth_rad, h_truth)
        v_truth_ned = vel_truth_raw.copy()
    else:
        p_truth_ecef = pos_truth_raw.copy()
        llh = np.array([ecef_to_llh_single(*p_truth_ecef[i]) for i in range(p_truth_ecef.shape[0])], dtype=float)
        lat_truth_rad = llh[:, 0]
        lon_truth_rad = llh[:, 1]
        v_truth_ned = np.zeros_like(vel_truth_raw)
        for i in range(vel_truth_raw.shape[0]):
            v_truth_ned[i] = ecef_vel_to_ned(vel_truth_raw[i], lat_truth_rad[i], lon_truth_rad[i])

    # -------------------------
    # GNSS
    # -------------------------
    gnss = load_numeric_matrix(gnss_path, min_cols=7)
    t_gnss = gnss[:, 0]
    pos_gnss_raw = gnss[:, 1:4]
    std_gnss_ned = gnss[:, 4:7]

    gnss_is_lla = np.max(np.abs(pos_gnss_raw[0, :2])) < 200.0
    if gnss_is_lla:
        lat_g_rad = np.deg2rad(pos_gnss_raw[:, 0])
        lon_g_rad = np.deg2rad(pos_gnss_raw[:, 1])
        h_g = pos_gnss_raw[:, 2]
        p_gnss_ecef = llh_to_ecef(lat_g_rad, lon_g_rad, h_g)
    else:
        p_gnss_ecef = pos_gnss_raw.copy()
        llh_g = np.array([ecef_to_llh_single(*p_gnss_ecef[i]) for i in range(p_gnss_ecef.shape[0])], dtype=float)
        lat_g_rad = llh_g[:, 0]
        lon_g_rad = llh_g[:, 1]

    gnss_has_vel = gnss.shape[1] >= 13
    gnss_vel_derived = False
    if gnss_has_vel:
        v_gnss_ned = gnss[:, 7:10]
        std_v_gnss_ned = gnss[:, 10:13]
    else:
        if args.derive_gnss_vel_if_missing:
            v_gnss_ned = derive_gnss_velocity_ned(t_gnss, p_gnss_ecef, lat_g_rad, lon_g_rad)
            std_v_gnss_ned = np.zeros((gnss.shape[0], 3), dtype=float)
            gnss_has_vel = True
            gnss_vel_derived = True
        else:
            v_gnss_ned = np.zeros((gnss.shape[0], 3), dtype=float)
            std_v_gnss_ned = np.zeros((gnss.shape[0], 3), dtype=float)

    mask_g = (t_gnss >= t_truth[0]) & (t_gnss <= t_truth[-1])
    t_g = t_gnss[mask_g]
    p_g = p_gnss_ecef[mask_g]
    std_g = std_gnss_ned[mask_g]
    v_g = v_gnss_ned[mask_g] if gnss_has_vel else np.zeros((0, 3))
    std_v_g = std_v_gnss_ned[mask_g] if gnss_has_vel else np.zeros((0, 3))

    p_truth_at_g = interp_cols(t_truth, p_truth_ecef, t_g)
    v_truth_ned_at_g = interp_cols(t_truth, v_truth_ned, t_g)
    lat_truth_at_g = np.interp(t_g, t_truth, lat_truth_rad)
    lon_truth_at_g = np.interp(t_g, t_truth, lon_truth_rad)

    err_pos_ned = np.zeros((t_g.size, 3), dtype=float)
    for i in range(t_g.size):
        dp_e = p_g[i] - p_truth_at_g[i]
        r_ne = rot_ned_to_ecef(float(lat_truth_at_g[i]), float(lon_truth_at_g[i]))
        err_pos_ned[i] = r_ne.T @ dp_e
    err_pos_h = np.linalg.norm(err_pos_ned[:, :2], axis=1)
    err_pos_3d = np.linalg.norm(err_pos_ned, axis=1)

    if gnss_has_vel:
        err_vel_ned = v_g - v_truth_ned_at_g
        err_vel_3d = np.linalg.norm(err_vel_ned, axis=1)
    else:
        err_vel_ned = np.zeros((0, 3), dtype=float)
        err_vel_3d = np.zeros((0,), dtype=float)

    # -------------------------
    # ODO
    # -------------------------
    odo = load_numeric_matrix(odo_path, min_cols=2)
    t_odo = odo[:, 0]
    v_odo = odo[:, 1]
    mask_o = (t_odo >= t_truth[0]) & (t_odo <= t_truth[-1])
    t_o = t_odo[mask_o]
    v_o = v_odo[mask_o]

    v_truth_ned_at_o = interp_cols(t_truth, v_truth_ned, t_o)
    rpy_truth_at_o_deg = interp_angles_deg(t_truth, rpy_truth_deg, t_o)
    rpy_truth_at_o_rad = np.deg2rad(rpy_truth_at_o_deg)

    v_truth_body = np.zeros((t_o.size, 3), dtype=float)
    v_truth_vehicle = np.zeros((t_o.size, 3), dtype=float)
    for i in range(t_o.size):
        c_nb = euler_to_rot_zyx(
            float(rpy_truth_at_o_rad[i, 0]),
            float(rpy_truth_at_o_rad[i, 1]),
            float(rpy_truth_at_o_rad[i, 2]),
        )
        v_truth_body[i] = c_nb.T @ v_truth_ned_at_o[i]
        v_truth_vehicle[i] = c_b_v @ v_truth_body[i]

    v_truth_fwd = v_truth_vehicle[:, 0]
    v_truth_horiz = np.linalg.norm(v_truth_ned_at_o[:, :2], axis=1)

    err_odo_signed = v_o - v_truth_fwd
    err_odo_abs = np.abs(v_o) - np.abs(v_truth_fwd)

    # -------------------------
    # Save aligned CSV
    # -------------------------
    np.savetxt(
        out_dir / "gnss_pos_vs_truth.csv",
        np.column_stack(
            [
                t_g,
                p_g,
                p_truth_at_g,
                err_pos_ned,
                err_pos_h,
                err_pos_3d,
                std_g,
            ]
        ),
        fmt="%.10f",
        header=(
            "t "
            "gnss_x gnss_y gnss_z truth_x truth_y truth_z "
            "err_n err_e err_d err_2d err_3d "
            "sigma_n sigma_e sigma_d"
        ),
        comments="",
    )

    if gnss_has_vel:
        np.savetxt(
            out_dir / "gnss_vel_vs_truth.csv",
            np.column_stack([t_g, v_g, v_truth_ned_at_g, err_vel_ned, err_vel_3d, std_v_g]),
            fmt="%.10f",
            header=(
                "t "
                "gnss_vn gnss_ve gnss_vd truth_vn truth_ve truth_vd "
                "err_vn err_ve err_vd err_v3d "
                "sigma_vn sigma_ve sigma_vd"
            ),
            comments="",
        )

    np.savetxt(
        out_dir / "odo_vs_truth.csv",
        np.column_stack([t_o, v_o, v_truth_fwd, v_truth_horiz, err_odo_signed, err_odo_abs]),
        fmt="%.10f",
        header="t odo_speed truth_fwd_vehicle truth_horiz_ned err_signed err_abs",
        comments="",
    )

    # -------------------------
    # Summary + outliers
    # -------------------------
    summary = {
        "inputs": {
            "truth_path": str(truth_path),
            "gnss_path": str(gnss_path),
            "odo_path": str(odo_path),
            "truth_is_lla": bool(truth_is_lla),
            "gnss_is_lla": bool(gnss_is_lla),
            "gnss_has_velocity": bool(gnss_has_vel),
            "gnss_velocity_derived": bool(gnss_vel_derived),
            "mounting_rpy_deg": mounting_rpy_deg.tolist(),
        },
        "counts": {
            "truth_rows": int(t_truth.size),
            "gnss_rows_raw": int(t_gnss.size),
            "gnss_rows_aligned": int(t_g.size),
            "odo_rows_raw": int(t_odo.size),
            "odo_rows_aligned": int(t_o.size),
        },
        "gnss_position_error_m": {
            "n": stats(err_pos_ned[:, 0]),
            "e": stats(err_pos_ned[:, 1]),
            "d": stats(err_pos_ned[:, 2]),
            "2d": stats(err_pos_h),
            "3d": stats(err_pos_3d),
        },
        "gnss_velocity_error_mps": {
            "n": stats(err_vel_ned[:, 0]) if gnss_has_vel else {},
            "e": stats(err_vel_ned[:, 1]) if gnss_has_vel else {},
            "d": stats(err_vel_ned[:, 2]) if gnss_has_vel else {},
            "3d": stats(err_vel_3d) if gnss_has_vel else {},
        },
        "odo_error_mps": {
            "signed": stats(err_odo_signed),
            "abs": stats(err_odo_abs),
        },
        "outliers": {
            "gnss_pos_3d_top": top_k_events(t_g, err_pos_3d, args.top_k, err_pos_ned),
            "gnss_vel_3d_top": top_k_events(t_g, err_vel_3d, args.top_k, err_vel_ned) if gnss_has_vel else [],
            "odo_signed_top": top_k_events(t_o, np.abs(err_odo_signed), args.top_k),
        },
    }

    with (out_dir / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    # -------------------------
    # Figures
    # -------------------------
    # GNSS position error
    tg_rel = t_g - t_g[0]
    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    ax[0].plot(tg_rel, err_pos_ned[:, 0], lw=0.8)
    ax[0].set_ylabel("err_N (m)")
    ax[1].plot(tg_rel, err_pos_ned[:, 1], lw=0.8)
    ax[1].set_ylabel("err_E (m)")
    ax[2].plot(tg_rel, err_pos_ned[:, 2], lw=0.8)
    ax[2].set_ylabel("err_D (m)")
    ax[3].plot(tg_rel, err_pos_3d, lw=0.8, label="3D error")
    ax[3].plot(tg_rel, err_pos_h, lw=0.8, label="2D error")
    ax[3].set_ylabel("norm (m)")
    ax[3].set_xlabel("time from first GNSS sample (s)")
    ax[3].legend(loc="upper right")
    fig.suptitle("GNSS Position Error vs Truth (NED)")
    fig.tight_layout()
    fig.savefig(out_dir / "gnss_position_error.png", dpi=150)
    plt.close(fig)

    # GNSS velocity error
    if gnss_has_vel:
        fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
        ax[0].plot(tg_rel, err_vel_ned[:, 0], lw=0.8)
        ax[0].set_ylabel("err_vN (m/s)")
        ax[1].plot(tg_rel, err_vel_ned[:, 1], lw=0.8)
        ax[1].set_ylabel("err_vE (m/s)")
        ax[2].plot(tg_rel, err_vel_ned[:, 2], lw=0.8)
        ax[2].set_ylabel("err_vD (m/s)")
        ax[3].plot(tg_rel, err_vel_3d, lw=0.8)
        ax[3].set_ylabel("err_v3D (m/s)")
        ax[3].set_xlabel("time from first GNSS sample (s)")
        fig.suptitle("GNSS Velocity Error vs Truth (NED)")
        fig.tight_layout()
        fig.savefig(out_dir / "gnss_velocity_error.png", dpi=150)
        plt.close(fig)

    # ODO vs truth
    to_rel = t_o - t_o[0]
    fig, ax = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    ax[0].plot(to_rel, v_o, lw=0.7, label="ODO")
    ax[0].plot(to_rel, v_truth_fwd, lw=0.7, label="truth forward (vehicle)")
    ax[0].plot(to_rel, v_truth_horiz, lw=0.7, label="truth horizontal (NED)")
    ax[0].set_ylabel("speed (m/s)")
    ax[0].legend(loc="upper right")
    ax[1].plot(to_rel, err_odo_signed, lw=0.7, label="ODO - truth forward")
    ax[1].plot(to_rel, err_odo_abs, lw=0.7, label="|ODO| - |truth forward|")
    ax[1].set_ylabel("error (m/s)")
    ax[1].set_xlabel("time from first ODO sample (s)")
    ax[1].legend(loc="upper right")
    fig.suptitle("ODO Speed vs Truth")
    fig.tight_layout()
    fig.savefig(out_dir / "odo_speed_compare.png", dpi=150)
    plt.close(fig)

    # ODO histogram
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    ax.hist(err_odo_signed, bins=120, alpha=0.6, label="signed")
    ax.hist(err_odo_abs, bins=120, alpha=0.6, label="abs")
    ax.set_xlabel("error (m/s)")
    ax.set_ylabel("count")
    ax.legend(loc="upper right")
    ax.set_title("ODO Error Histogram")
    fig.tight_layout()
    fig.savefig(out_dir / "odo_error_hist.png", dpi=150)
    plt.close(fig)

    # -------------------------
    # Console summary
    # -------------------------
    pos3 = summary["gnss_position_error_m"]["3d"]
    odo_s = summary["odo_error_mps"]["signed"]
    print("=== Sensor vs Truth Validation ===")
    print(f"truth rows: {summary['counts']['truth_rows']}")
    print(f"gnss aligned rows: {summary['counts']['gnss_rows_aligned']}")
    print(f"odo aligned rows: {summary['counts']['odo_rows_aligned']}")
    if pos3:
        print(
            "GNSS pos 3D error [m]: "
            f"mean={pos3['mean']:.3f}, p95={pos3['p95']:.3f}, p99={pos3['p99']:.3f}, max={pos3['max']:.3f}"
        )
    if gnss_has_vel:
        vel3 = summary["gnss_velocity_error_mps"]["3d"]
        if vel3:
            vel_tag = " (derived)" if gnss_vel_derived else ""
            print(
                f"GNSS vel 3D error{vel_tag} [m/s]: "
                f"mean={vel3['mean']:.3f}, p95={vel3['p95']:.3f}, p99={vel3['p99']:.3f}, max={vel3['max']:.3f}"
            )
    if odo_s:
        print(
            "ODO signed error [m/s]: "
            f"mean={odo_s['mean']:.3f}, p95={odo_s['p95']:.3f}, p99={odo_s['p99']:.3f}, max={odo_s['max']:.3f}"
        )
    print(f"saved to: {out_dir}")


if __name__ == "__main__":
    main()
