#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Export anomaly windows and joint diagnostics:
- state trajectories from SOL
- GNSS/ODO residual proxies from sensor validation CSVs
- automatic coarse diagnosis label per window
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


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
        raise ValueError(f"{path} has only {ncol} columns")
    return np.array([r[:ncol] for r in rows], dtype=float)


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


def truth_to_ecef(pos_raw: np.ndarray, vel_raw: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    truth_is_lla = np.max(np.abs(pos_raw[0, :2])) < 200.0
    if truth_is_lla:
        lat = np.deg2rad(pos_raw[:, 0])
        lon = np.deg2rad(pos_raw[:, 1])
        h = pos_raw[:, 2]
        p_ecef = llh_to_ecef(lat, lon, h)

        # NED -> ECEF
        v_ecef = np.zeros_like(vel_raw)
        for i in range(pos_raw.shape[0]):
            sl = math.sin(float(lat[i]))
            cl = math.cos(float(lat[i]))
            so = math.sin(float(lon[i]))
            co = math.cos(float(lon[i]))
            r_ne = np.array(
                [
                    [-sl * co, -so, -cl * co],
                    [-sl * so, co, -cl * so],
                    [cl, 0.0, -sl],
                ],
                dtype=float,
            )
            v_ecef[i] = r_ne @ vel_raw[i]
        return p_ecef, v_ecef

    return pos_raw.copy(), vel_raw.copy()


def interp_cols(t_src: np.ndarray, x_src: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    out = np.zeros((t_query.size, x_src.shape[1]), dtype=float)
    for i in range(x_src.shape[1]):
        out[:, i] = np.interp(t_query, t_src, x_src[:, i])
    return out


def build_candidates(summary: Dict, per_source: int) -> List[Tuple[float, float, str]]:
    out = []
    outliers = summary.get("outliers", {})
    mapping = [
        ("gnss_pos_3d_top", "gnss_pos_3d"),
        ("gnss_vel_3d_top", "gnss_vel_3d"),
        ("odo_signed_top", "odo_abs_err"),
    ]
    for key, name in mapping:
        lst = outliers.get(key, [])
        if not isinstance(lst, list):
            continue
        for i, item in enumerate(lst[:per_source]):
            t = float(item.get("t", np.nan))
            v = float(item.get("value", np.nan))
            if not np.isfinite(t) or not np.isfinite(v):
                continue
            # rank-weighted severity; keep magnitude scale modest
            sev = abs(v) * (1.0 / (i + 1))
            out.append((t, sev, name))
    return out


def cluster_times(cands: List[Tuple[float, float, str]], merge_gap: float, max_windows: int) -> List[Dict]:
    if not cands:
        return []
    cands = sorted(cands, key=lambda x: x[0])
    groups: List[List[Tuple[float, float, str]]] = []
    cur = [cands[0]]
    for item in cands[1:]:
        if abs(item[0] - cur[-1][0]) <= merge_gap:
            cur.append(item)
        else:
            groups.append(cur)
            cur = [item]
    groups.append(cur)

    windows: List[Dict] = []
    for g in groups:
        best = max(g, key=lambda x: x[1])
        center = best[0]
        total = float(sum(x[1] for x in g))
        srcs = sorted(set(x[2] for x in g))
        windows.append({"t_center": float(center), "severity": total, "sources": srcs})

    windows.sort(key=lambda x: x["severity"], reverse=True)
    windows = windows[:max_windows]
    windows.sort(key=lambda x: x["t_center"])
    return windows


def segment(arr: np.ndarray, t0: float, t1: float) -> np.ndarray:
    if arr.size == 0:
        return arr
    m = (arr[:, 0] >= t0) & (arr[:, 0] <= t1)
    return arr[m]


def save_csv(path: Path, arr: np.ndarray, header: str) -> None:
    if arr.size == 0:
        path.write_text(header + "\n", encoding="utf-8")
        return
    np.savetxt(path, arr, fmt="%.10f", header=header, comments="")


def classify_window(
    max_gnss_pos: float,
    max_gnss_vel: float,
    max_odo: float,
    state_mount_range: float,
    state_scale_range: float,
    ref_p99_pos: float,
    ref_p99_vel: float,
    ref_p99_odo: float,
) -> str:
    s_pos = max_gnss_pos / max(1e-6, ref_p99_pos)
    s_vel = max_gnss_vel / max(1e-6, ref_p99_vel)
    s_odo = max_odo / max(1e-6, ref_p99_odo)
    sensor_spike = max(s_pos, s_vel, s_odo)
    state_spike = max(
        state_mount_range / 5.0,     # 5 deg in 60s window is already large
        state_scale_range / 0.02,    # 0.02 scale change in 60s is large
    )
    if sensor_spike > 2.0 and state_spike < 1.0:
        return "likely_data_anomaly"
    if state_spike > 2.0 and sensor_spike < 1.3:
        return "likely_filter_state_anomaly"
    return "mixed_or_uncertain"


def main() -> None:
    parser = argparse.ArgumentParser(description="Export anomaly windows diagnostics.")
    parser.add_argument("--summary", type=str, default="output/sensor_validation_data4/summary.json")
    parser.add_argument("--sol", type=str, default="SOL_data4_gnss.txt")
    parser.add_argument("--truth", type=str, default=None, help="override truth path")
    parser.add_argument("--gnss-pos-csv", type=str, default=None)
    parser.add_argument("--gnss-vel-csv", type=str, default=None)
    parser.add_argument("--odo-csv", type=str, default=None)
    parser.add_argument("--out-dir", type=str, default="output/sensor_validation_data4/windows")
    parser.add_argument("--half-window", type=float, default=30.0)
    parser.add_argument("--merge-gap", type=float, default=8.0)
    parser.add_argument("--per-source", type=int, default=20)
    parser.add_argument("--max-windows", type=int, default=12)
    args = parser.parse_args()

    summary_path = Path(args.summary)
    with summary_path.open("r", encoding="utf-8") as f:
        summary = json.load(f)

    base = summary_path.parent
    sol_path = Path(args.sol)
    if not sol_path.is_absolute():
        sol_path = Path.cwd() / sol_path

    truth_path = Path(args.truth) if args.truth else Path(summary["inputs"]["truth_path"])
    gnss_pos_csv = Path(args.gnss_pos_csv) if args.gnss_pos_csv else base / "gnss_pos_vs_truth.csv"
    gnss_vel_csv = Path(args.gnss_vel_csv) if args.gnss_vel_csv else base / "gnss_vel_vs_truth.csv"
    odo_csv = Path(args.odo_csv) if args.odo_csv else base / "odo_vs_truth.csv"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load SOL and truth to compute overall solution errors
    sol = load_numeric_matrix(sol_path, min_cols=28)
    t_sol = sol[:, 0]
    p_sol = sol[:, 1:4]
    v_sol = sol[:, 4:7]
    mounting_pitch = sol[:, 10]
    mounting_yaw = sol[:, 11]
    odo_scale = sol[:, 12]

    truth = load_numeric_matrix(truth_path, min_cols=10)
    t_truth = truth[:, 0]
    p_truth_ecef, v_truth_ecef = truth_to_ecef(truth[:, 1:4], truth[:, 4:7])
    p_truth_at_sol = interp_cols(t_truth, p_truth_ecef, t_sol)
    v_truth_at_sol = interp_cols(t_truth, v_truth_ecef, t_sol)
    sol_pos_err_3d = np.linalg.norm(p_sol - p_truth_at_sol, axis=1)
    sol_vel_err_3d = np.linalg.norm(v_sol - v_truth_at_sol, axis=1)
    sol_aux = np.column_stack([t_sol, sol_pos_err_3d, sol_vel_err_3d, mounting_pitch, mounting_yaw, odo_scale])

    # Load residual proxy CSVs
    gnss_pos = load_numeric_matrix(gnss_pos_csv, min_cols=13) if gnss_pos_csv.exists() else np.zeros((0, 13))
    gnss_vel = load_numeric_matrix(gnss_vel_csv, min_cols=14) if gnss_vel_csv.exists() else np.zeros((0, 14))
    odo_mat = load_numeric_matrix(odo_csv, min_cols=6) if odo_csv.exists() else np.zeros((0, 6))

    # Build windows
    cands = build_candidates(summary, per_source=args.per_source)
    windows = cluster_times(cands, merge_gap=args.merge_gap, max_windows=args.max_windows)
    if not windows:
        print("No anomaly windows found in summary outliers.")
        return

    ref_p99_pos = float(summary["gnss_position_error_m"]["3d"].get("p99", 1.0))
    ref_p99_vel = float(summary["gnss_velocity_error_mps"]["3d"].get("p99", 1.0))
    ref_p99_odo = float(summary["odo_error_mps"]["signed"].get("p99", 1.0))

    report = {
        "summary_source": str(summary_path),
        "sol_path": str(sol_path),
        "truth_path": str(truth_path),
        "half_window_s": float(args.half_window),
        "windows": [],
    }

    for i, w in enumerate(windows, start=1):
        tc = float(w["t_center"])
        t0 = tc - args.half_window
        t1 = tc + args.half_window

        s_seg = segment(sol_aux, t0, t1)
        gpos_seg = segment(gnss_pos, t0, t1)
        gvel_seg = segment(gnss_vel, t0, t1)
        odo_seg = segment(odo_mat, t0, t1)

        if s_seg.size == 0:
            continue

        # stats in window
        max_sol_pos = float(np.max(s_seg[:, 1]))
        max_sol_vel = float(np.max(s_seg[:, 2]))
        mount_range = float(np.max(np.maximum(s_seg[:, 3], s_seg[:, 4])) - np.min(np.minimum(s_seg[:, 3], s_seg[:, 4])))
        scale_range = float(np.max(s_seg[:, 5]) - np.min(s_seg[:, 5]))
        max_gpos = float(np.max(gpos_seg[:, 11])) if gpos_seg.size else 0.0
        max_gvel = float(np.max(gvel_seg[:, 10])) if gvel_seg.size else 0.0
        max_odo = float(np.max(np.abs(odo_seg[:, 4]))) if odo_seg.size else 0.0

        label = classify_window(
            max_gnss_pos=max_gpos,
            max_gnss_vel=max_gvel,
            max_odo=max_odo,
            state_mount_range=mount_range,
            state_scale_range=scale_range,
            ref_p99_pos=ref_p99_pos,
            ref_p99_vel=ref_p99_vel,
            ref_p99_odo=ref_p99_odo,
        )

        # Save per-window csv
        sub = out_dir / f"window_{i:02d}_t{tc:.3f}"
        sub.mkdir(parents=True, exist_ok=True)
        save_csv(sub / "sol_window.csv", s_seg, "t sol_pos_err_3d sol_vel_err_3d mounting_pitch_deg mounting_yaw_deg odo_scale")
        save_csv(sub / "gnss_pos_window.csv", gpos_seg, "t gnss_x gnss_y gnss_z truth_x truth_y truth_z err_n err_e err_d err_2d err_3d sigma_n sigma_e sigma_d")
        if gvel_seg.size:
            save_csv(sub / "gnss_vel_window.csv", gvel_seg, "t gnss_vn gnss_ve gnss_vd truth_vn truth_ve truth_vd err_vn err_ve err_vd err_v3d sigma_vn sigma_ve sigma_vd")
        save_csv(sub / "odo_window.csv", odo_seg, "t odo_speed truth_fwd_vehicle truth_horiz_ned err_signed err_abs")

        # Plot
        tr = s_seg[:, 0] - tc
        fig, ax = plt.subplots(6, 1, figsize=(14, 14), sharex=True)

        ax[0].plot(tr, s_seg[:, 1], lw=1.0, label="SOL pos err 3D")
        ax[0].axvline(0.0, color="r", lw=0.8, ls="--")
        ax[0].set_ylabel("m")
        ax[0].legend(loc="upper right")

        ax[1].plot(tr, s_seg[:, 2], lw=1.0, label="SOL vel err 3D")
        ax[1].axvline(0.0, color="r", lw=0.8, ls="--")
        ax[1].set_ylabel("m/s")
        ax[1].legend(loc="upper right")

        if gpos_seg.size:
            tg = gpos_seg[:, 0] - tc
            ax[2].plot(tg, gpos_seg[:, 11], lw=0.9, label="GNSS pos err 3D")
            ax[2].plot(tg, gpos_seg[:, 7], lw=0.7, alpha=0.8, label="err_N")
            ax[2].plot(tg, gpos_seg[:, 8], lw=0.7, alpha=0.8, label="err_E")
            ax[2].plot(tg, gpos_seg[:, 9], lw=0.7, alpha=0.8, label="err_D")
            ax[2].legend(loc="upper right", ncol=4, fontsize=8)
        ax[2].axvline(0.0, color="r", lw=0.8, ls="--")
        ax[2].set_ylabel("m")

        if gvel_seg.size:
            tv = gvel_seg[:, 0] - tc
            ax[3].plot(tv, gvel_seg[:, 10], lw=0.9, label="GNSS vel err 3D")
            ax[3].plot(tv, gvel_seg[:, 7], lw=0.7, alpha=0.8, label="err_vN")
            ax[3].plot(tv, gvel_seg[:, 8], lw=0.7, alpha=0.8, label="err_vE")
            ax[3].plot(tv, gvel_seg[:, 9], lw=0.7, alpha=0.8, label="err_vD")
            ax[3].legend(loc="upper right", ncol=4, fontsize=8)
        ax[3].axvline(0.0, color="r", lw=0.8, ls="--")
        ax[3].set_ylabel("m/s")

        if odo_seg.size:
            to = odo_seg[:, 0] - tc
            ax[4].plot(to, odo_seg[:, 4], lw=0.8, label="ODO signed err")
            ax[4].plot(to, odo_seg[:, 5], lw=0.8, label="ODO abs err")
            ax[4].legend(loc="upper right")
        ax[4].axvline(0.0, color="r", lw=0.8, ls="--")
        ax[4].set_ylabel("m/s")

        ax[5].plot(tr, s_seg[:, 3], lw=0.8, label="mounting_pitch_deg")
        ax[5].plot(tr, s_seg[:, 4], lw=0.8, label="mounting_yaw_deg")
        ax[5].plot(tr, s_seg[:, 5], lw=0.8, label="odo_scale")
        ax[5].axvline(0.0, color="r", lw=0.8, ls="--")
        ax[5].legend(loc="upper right", ncol=3, fontsize=8)
        ax[5].set_ylabel("state")
        ax[5].set_xlabel("time relative to anomaly center (s)")

        fig.suptitle(
            f"Window {i:02d} @ t={tc:.3f}s | label={label} | sources={','.join(w['sources'])}",
            fontsize=11,
        )
        fig.tight_layout()
        fig.savefig(sub / "joint_diagnostics.png", dpi=150)
        plt.close(fig)

        report["windows"].append(
            {
                "index": i,
                "t_center": tc,
                "t0": t0,
                "t1": t1,
                "sources": w["sources"],
                "severity": w["severity"],
                "label": label,
                "metrics": {
                    "max_sol_pos_err_3d_m": max_sol_pos,
                    "max_sol_vel_err_3d_mps": max_sol_vel,
                    "max_gnss_pos_err_3d_m": max_gpos,
                    "max_gnss_vel_err_3d_mps": max_gvel,
                    "max_odo_signed_err_mps": max_odo,
                    "mounting_range_deg": mount_range,
                    "odo_scale_range": scale_range,
                },
                "files": {
                    "dir": str(sub),
                    "plot": str(sub / "joint_diagnostics.png"),
                    "sol_csv": str(sub / "sol_window.csv"),
                    "gnss_pos_csv": str(sub / "gnss_pos_window.csv"),
                    "gnss_vel_csv": str(sub / "gnss_vel_window.csv"),
                    "odo_csv": str(sub / "odo_window.csv"),
                },
            }
        )

    with (out_dir / "window_report.json").open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"generated windows: {len(report['windows'])}")
    print(f"report: {out_dir / 'window_report.json'}")


if __name__ == "__main__":
    main()
