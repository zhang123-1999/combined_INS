#!/usr/bin/env python3
"""Compare current-solver pure INS drift against KF-GINS under the same data2 contract."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import shutil
import sys
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_kf_gins_outage_compare import (
    FALLBACK_ROOT_DEFAULT,
    KF_GINS_EXE_DEFAULT,
    angle_wrap_deg,
    copy_file,
    copy_runtime_outputs,
    interp_angle_deg,
    load_kf_gins_imu_error,
    load_kf_gins_navresult,
    load_truth_nav,
    reset_run_dir,
    resolve_runtime_root,
    run_kf_gins,
)
from scripts.analysis.run_data2_rtk_outage_eval import json_safe, markdown_table


SOURCE_MANIFEST_DEFAULT = REPO_ROOT / "output" / "d2_baseline_pure_ins_90s_r2" / "manifest.json"
SOURCE_CONFIG_DEFAULT = (
    REPO_ROOT
    / "output"
    / "d2_baseline_pure_ins_90s_r2"
    / "artifacts"
    / "cases"
    / "d2_baseline_pure_ins_90s"
    / "config_d2_baseline_pure_ins_90s.yaml"
)
OUTPUT_DIR_DEFAULT = REPO_ROOT / "output" / "data2_kf_gins_pure_ins_compare_r1"
EXP_ID_DEFAULT = "EXP-20260328-data2-kf-gins-pure-ins-compare-r1"
WINDOW_HORIZONS_S = (30.0, 60.0, 90.0)


def sigma_radps_to_degph(value: float) -> float:
    return float(value) * 180.0 / math.pi * 3600.0


def sigma_mps2_to_mgal(value: float) -> float:
    return float(value) * 1.0e5


def sigma_scale_to_ppm(value: float) -> float:
    return float(value) * 1.0e6


def clean_float(value: float, ndigits: int = 12) -> float:
    return round(float(value), ndigits)


def format_fixed_9(value: float) -> str:
    return str(Decimal(str(float(value))).quantize(Decimal("0.000000001"), rounding=ROUND_HALF_UP))


def build_kf_gins_pure_ins_config(source_cfg: dict[str, Any], imupath: Path, gnsspath: Path, outputpath: Path) -> dict[str, Any]:
    fusion = source_cfg["fusion"]
    noise = fusion["noise"]
    init = fusion["init"]

    return {
        "imupath": str(imupath),
        "gnsspath": str(gnsspath),
        "outputpath": str(outputpath),
        "imudatalen": 7,
        "imudatarate": 200,
        "starttime": float(fusion["starttime"]),
        "endtime": float(fusion["finaltime"]),
        "initpos": [float(v) for v in init["init_pos_lla"]],
        "initvel": [float(v) for v in init["init_vel_ned"]],
        "initatt": [float(v) for v in init["init_att_rpy"]],
        "initgyrbias": [float(v) for v in init.get("bg0", [0.0, 0.0, 0.0])],
        "initaccbias": [float(v) for v in init.get("ba0", [0.0, 0.0, 0.0])],
        "initgyrscale": [float(v) for v in init.get("sg0", [0.0, 0.0, 0.0])],
        "initaccscale": [float(v) for v in init.get("sa0", [0.0, 0.0, 0.0])],
        "initposstd": [float(v) for v in init["std_pos"]],
        "initvelstd": [float(v) for v in init["std_vel"]],
        "initattstd": [float(v) for v in init["std_att"]],
        "initbgstd": [clean_float(sigma_radps_to_degph(v)) for v in init["std_bg"]],
        "initbastd": [clean_float(sigma_mps2_to_mgal(v)) for v in init["std_ba"]],
        "initsgstd": [clean_float(sigma_scale_to_ppm(v)) for v in init["std_sg"]],
        "initsastd": [clean_float(sigma_scale_to_ppm(v)) for v in init["std_sa"]],
        "imunoise": {
            "arw": [clean_float(float(noise["sigma_gyro"]) * 180.0 / math.pi * math.sqrt(3600.0))] * 3,
            "vrw": [clean_float(float(noise["sigma_acc"]) * math.sqrt(3600.0))] * 3,
            "gbstd": [clean_float(sigma_radps_to_degph(float(noise["sigma_bg"])))] * 3,
            "abstd": [clean_float(sigma_mps2_to_mgal(float(noise["sigma_ba"])))] * 3,
            "gsstd": [clean_float(sigma_scale_to_ppm(float(noise["sigma_sg"])))] * 3,
            "asstd": [clean_float(sigma_scale_to_ppm(float(noise["sigma_sa"])))] * 3,
            "corrtime": clean_float(float(noise["markov_corr_time"]) / 3600.0),
        },
        "antlever": [float(v) for v in init["gnss_lever_arm0"]],
    }


def write_dummy_gnss_file(
    output_path: Path,
    timestamp: float,
    lat_deg: float,
    lon_deg: float,
    h_m: float,
    std_xyz: tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> dict[str, float]:
    ensure_dir(output_path.parent)
    row = (
        f"{format_fixed_9(timestamp)} {format_fixed_9(lat_deg)} {format_fixed_9(lon_deg)} {format_fixed_9(h_m)} "
        f"{format_fixed_9(std_xyz[0])} {format_fixed_9(std_xyz[1])} {format_fixed_9(std_xyz[2])}\n"
    )
    output_path.write_text(row, encoding="utf-8")
    return {
        "rows_written": 1,
        "timestamp": float(timestamp),
        "lat_deg": float(lat_deg),
        "lon_deg": float(lon_deg),
        "h_m": float(h_m),
    }


def load_current_error_frame(all_states_path: Path) -> pd.DataFrame:
    frame = pd.read_csv(all_states_path)
    required = {
        "timestamp",
        "p_n_err_m",
        "p_e_err_m",
        "p_u_err_m",
        "v_n_mps",
        "v_e_mps",
        "v_u_mps",
        "truth_v_n_mps",
        "truth_v_e_mps",
        "truth_v_u_mps",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "truth_roll_deg",
        "truth_pitch_deg",
        "truth_yaw_deg",
    }
    missing = required.difference(frame.columns)
    if missing:
        raise RuntimeError(f"missing current all_states columns: {sorted(missing)}")

    out = pd.DataFrame()
    out["timestamp"] = frame["timestamp"].astype(float)
    out["pos_err_n_m"] = frame["p_n_err_m"].astype(float)
    out["pos_err_e_m"] = frame["p_e_err_m"].astype(float)
    out["pos_err_u_m"] = frame["p_u_err_m"].astype(float)
    out["vel_err_n_mps"] = frame["v_n_mps"].astype(float) - frame["truth_v_n_mps"].astype(float)
    out["vel_err_e_mps"] = frame["v_e_mps"].astype(float) - frame["truth_v_e_mps"].astype(float)
    out["vel_err_u_mps"] = frame["v_u_mps"].astype(float) - frame["truth_v_u_mps"].astype(float)
    out["roll_err_deg"] = angle_wrap_deg(frame["roll_deg"].astype(float).to_numpy() - frame["truth_roll_deg"].astype(float).to_numpy())
    out["pitch_err_deg"] = angle_wrap_deg(
        frame["pitch_deg"].astype(float).to_numpy() - frame["truth_pitch_deg"].astype(float).to_numpy()
    )
    out["yaw_err_deg"] = angle_wrap_deg(frame["yaw_deg"].astype(float).to_numpy() - frame["truth_yaw_deg"].astype(float).to_numpy())
    if {"bg_z_degh", "truth_bg_z_degh"}.issubset(frame.columns):
        out["bg_z_err_degh"] = frame["bg_z_degh"].astype(float) - frame["truth_bg_z_degh"].astype(float)
    return out


def rot_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    sl = math.sin(lat_rad)
    cl = math.cos(lat_rad)
    so = math.sin(lon_rad)
    co = math.cos(lon_rad)
    return np.array(
        [
            [-sl * co, -sl * so, cl],
            [-so, co, 0.0],
            [-cl * co, -cl * so, -sl],
        ],
        dtype=float,
    )


def ecef_delta_to_neu(delta_ecef: np.ndarray, lat_deg: np.ndarray, lon_deg: np.ndarray) -> np.ndarray:
    out = np.zeros_like(delta_ecef)
    for idx in range(delta_ecef.shape[0]):
        rot = rot_ecef_to_ned(math.radians(float(lat_deg[idx])), math.radians(float(lon_deg[idx])))
        ned = rot @ delta_ecef[idx]
        out[idx] = np.array([ned[0], ned[1], -ned[2]], dtype=float)
    return out


def build_kf_error_frame(nav_frame: pd.DataFrame, truth_frame: pd.DataFrame) -> pd.DataFrame:
    nav_t = nav_frame["timestamp"].to_numpy(dtype=float)
    truth_t = truth_frame["timestamp"].to_numpy(dtype=float)

    truth_ecef = np.column_stack(
        [
            np.interp(nav_t, truth_t, truth_frame["ecef_x"].to_numpy(dtype=float)),
            np.interp(nav_t, truth_t, truth_frame["ecef_y"].to_numpy(dtype=float)),
            np.interp(nav_t, truth_t, truth_frame["ecef_z"].to_numpy(dtype=float)),
        ]
    )
    nav_ecef = nav_frame[["ecef_x", "ecef_y", "ecef_z"]].to_numpy(dtype=float)
    delta_ecef = nav_ecef - truth_ecef

    truth_lat = np.interp(nav_t, truth_t, truth_frame["lat_deg"].to_numpy(dtype=float))
    truth_lon = np.interp(nav_t, truth_t, truth_frame["lon_deg"].to_numpy(dtype=float))
    pos_err_neu = ecef_delta_to_neu(delta_ecef, truth_lat, truth_lon)

    truth_vn = np.interp(nav_t, truth_t, truth_frame["vn_mps"].to_numpy(dtype=float))
    truth_ve = np.interp(nav_t, truth_t, truth_frame["ve_mps"].to_numpy(dtype=float))
    truth_vu = np.interp(nav_t, truth_t, -truth_frame["vd_mps"].to_numpy(dtype=float))
    nav_vu = -nav_frame["vd_mps"].to_numpy(dtype=float)

    roll_truth = interp_angle_deg(nav_t, truth_t, truth_frame["roll_deg"].to_numpy(dtype=float))
    pitch_truth = interp_angle_deg(nav_t, truth_t, truth_frame["pitch_deg"].to_numpy(dtype=float))
    yaw_truth = interp_angle_deg(nav_t, truth_t, truth_frame["yaw_deg"].to_numpy(dtype=float))

    out = pd.DataFrame()
    out["timestamp"] = nav_t
    out["pos_err_n_m"] = pos_err_neu[:, 0]
    out["pos_err_e_m"] = pos_err_neu[:, 1]
    out["pos_err_u_m"] = pos_err_neu[:, 2]
    out["vel_err_n_mps"] = nav_frame["vn_mps"].to_numpy(dtype=float) - truth_vn
    out["vel_err_e_mps"] = nav_frame["ve_mps"].to_numpy(dtype=float) - truth_ve
    out["vel_err_u_mps"] = nav_vu - truth_vu
    out["roll_err_deg"] = angle_wrap_deg(nav_frame["roll_deg"].to_numpy(dtype=float) - roll_truth)
    out["pitch_err_deg"] = angle_wrap_deg(nav_frame["pitch_deg"].to_numpy(dtype=float) - pitch_truth)
    out["yaw_err_deg"] = angle_wrap_deg(nav_frame["yaw_deg"].to_numpy(dtype=float) - yaw_truth)
    return out


def attach_bg_z_error(frame: pd.DataFrame, imu_err_frame: pd.DataFrame, truth_bg_z_degh: float) -> pd.DataFrame:
    out = frame.copy()
    imu_t = imu_err_frame["timestamp"].to_numpy(dtype=float)
    imu_err = imu_err_frame["bg_z_degh"].to_numpy(dtype=float) - float(truth_bg_z_degh)
    out["bg_z_err_degh"] = np.interp(out["timestamp"].to_numpy(dtype=float), imu_t, imu_err)
    return out


def compute_system_window_metrics(
    system: str,
    err_frame: pd.DataFrame,
    horizons_s: tuple[float, ...] = WINDOW_HORIZONS_S,
) -> list[dict[str, Any]]:
    t = err_frame["timestamp"].to_numpy(dtype=float)
    elapsed = t - float(t[0])
    pos = err_frame[["pos_err_n_m", "pos_err_e_m", "pos_err_u_m"]].to_numpy(dtype=float)
    vel = err_frame[["vel_err_n_mps", "vel_err_e_mps", "vel_err_u_mps"]].to_numpy(dtype=float)
    att = err_frame[["roll_err_deg", "pitch_err_deg", "yaw_err_deg"]].to_numpy(dtype=float)

    rows: list[dict[str, Any]] = []
    windows: list[tuple[str, np.ndarray]] = []
    for horizon_s in horizons_s:
        mask = elapsed <= float(horizon_s) + 1.0e-9
        if int(np.count_nonzero(mask)) >= 2:
            windows.append((f"{int(horizon_s)}s", mask))
    windows.append(("full", np.ones_like(elapsed, dtype=bool)))

    for label, mask in windows:
        pos_slice = pos[mask]
        vel_slice = vel[mask]
        att_slice = att[mask]
        pos_3d = np.linalg.norm(pos_slice, axis=1)
        vel_3d = np.linalg.norm(vel_slice, axis=1)
        att_3d = np.linalg.norm(att_slice, axis=1)
        row: dict[str, Any] = {
            "system": system,
            "window": label,
            "samples": int(pos_slice.shape[0]),
            "duration_s": float(elapsed[mask][-1]),
            "pos_rmse_3d_m": float(np.sqrt(np.mean(pos_3d * pos_3d))),
            "pos_final_3d_m": float(pos_3d[-1]),
            "vel_rmse_3d_mps": float(np.sqrt(np.mean(vel_3d * vel_3d))),
            "vel_final_3d_mps": float(vel_3d[-1]),
            "att_rmse_3d_deg": float(np.sqrt(np.mean(att_3d * att_3d))),
            "yaw_rmse_deg": float(np.sqrt(np.mean(att_slice[:, 2] ** 2))),
            "yaw_final_abs_deg": float(abs(att_slice[-1, 2])),
        }
        if "bg_z_err_degh" in err_frame.columns:
            row["bg_z_err_abs_max_degh"] = float(np.max(np.abs(err_frame.loc[mask, "bg_z_err_degh"].to_numpy(dtype=float))))
        rows.append(row)
    return rows


def build_comparison_window_frame(current_df: pd.DataFrame, kf_df: pd.DataFrame) -> pd.DataFrame:
    metrics = [
        "pos_rmse_3d_m",
        "pos_final_3d_m",
        "vel_rmse_3d_mps",
        "vel_final_3d_mps",
        "att_rmse_3d_deg",
        "yaw_rmse_deg",
        "yaw_final_abs_deg",
    ]
    optional = "bg_z_err_abs_max_degh"
    if optional in current_df.columns and optional in kf_df.columns:
        metrics.append(optional)

    merged = current_df.merge(kf_df, on="window", suffixes=("_current", "_kf"))
    rows: list[dict[str, Any]] = []
    for _, row in merged.iterrows():
        out: dict[str, Any] = {"window": row["window"]}
        for metric in metrics:
            current_value = float(row[f"{metric}_current"])
            kf_value = float(row[f"{metric}_kf"])
            out[f"{metric}_current"] = current_value
            out[f"{metric}_kf"] = kf_value
            out[f"{metric}_delta_kf_minus_current"] = kf_value - current_value
            out[f"{metric}_ratio_current_over_kf"] = float("inf") if abs(kf_value) < 1.0e-12 else current_value / kf_value
        rows.append(out)
    return pd.DataFrame(rows)


def plot_compare_overview(current_err_frame: pd.DataFrame, kf_err_frame: pd.DataFrame, output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    current_t = current_err_frame["timestamp"].to_numpy(dtype=float)
    kf_t = kf_err_frame["timestamp"].to_numpy(dtype=float)
    current_elapsed = current_t - float(current_t[0])
    kf_elapsed = kf_t - float(kf_t[0])

    current_pos = np.linalg.norm(
        current_err_frame[["pos_err_n_m", "pos_err_e_m", "pos_err_u_m"]].to_numpy(dtype=float),
        axis=1,
    )
    kf_pos = np.linalg.norm(kf_err_frame[["pos_err_n_m", "pos_err_e_m", "pos_err_u_m"]].to_numpy(dtype=float), axis=1)
    current_vel = np.linalg.norm(
        current_err_frame[["vel_err_n_mps", "vel_err_e_mps", "vel_err_u_mps"]].to_numpy(dtype=float),
        axis=1,
    )
    kf_vel = np.linalg.norm(kf_err_frame[["vel_err_n_mps", "vel_err_e_mps", "vel_err_u_mps"]].to_numpy(dtype=float), axis=1)
    current_yaw = np.abs(current_err_frame["yaw_err_deg"].to_numpy(dtype=float))
    kf_yaw = np.abs(kf_err_frame["yaw_err_deg"].to_numpy(dtype=float))

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(current_elapsed, current_pos, label="current solver", linewidth=1.2)
    axes[0].plot(kf_elapsed, kf_pos, label="KF-GINS", linewidth=1.2)
    axes[0].set_ylabel("pos err 3d [m]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(current_elapsed, current_vel, linewidth=1.2)
    axes[1].plot(kf_elapsed, kf_vel, linewidth=1.2)
    axes[1].set_ylabel("vel err 3d [m/s]")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(current_elapsed, current_yaw, linewidth=1.2)
    axes[2].plot(kf_elapsed, kf_yaw, linewidth=1.2)
    axes[2].set_ylabel("|yaw err| [deg]")
    axes[2].set_xlabel("elapsed time [s]")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("data2 pure INS drift: current solver vs KF-GINS")
    fig.tight_layout()
    ensure_dir(output_path.parent)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_bg_z_compare(current_err_frame: pd.DataFrame, kf_err_frame: pd.DataFrame, output_path: Path) -> None:
    if "bg_z_err_degh" not in current_err_frame.columns or "bg_z_err_degh" not in kf_err_frame.columns:
        return

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    current_t = current_err_frame["timestamp"].to_numpy(dtype=float)
    kf_t = kf_err_frame["timestamp"].to_numpy(dtype=float)
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(current_t - float(current_t[0]), current_err_frame["bg_z_err_degh"].to_numpy(dtype=float), label="current solver")
    ax.plot(kf_t - float(kf_t[0]), kf_err_frame["bg_z_err_degh"].to_numpy(dtype=float), label="KF-GINS")
    ax.set_xlabel("elapsed time [s]")
    ax.set_ylabel("bg_z err [deg/h]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    ensure_dir(output_path.parent)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_summary(
    source_manifest: dict[str, Any],
    runtime_mode: str,
    current_df: pd.DataFrame,
    kf_df: pd.DataFrame,
    comparison_df: pd.DataFrame,
    output_dir: Path,
) -> str:
    table_rows: list[list[str]] = []
    current_map = {row["window"]: row for row in current_df.to_dict("records")}
    kf_map = {row["window"]: row for row in kf_df.to_dict("records")}
    cmp_map = {row["window"]: row for row in comparison_df.to_dict("records")}
    for window in [f"{int(v)}s" for v in WINDOW_HORIZONS_S] + ["full"]:
        if window not in current_map or window not in kf_map or window not in cmp_map:
            continue
        table_rows.append(
            [
                window,
                f"{current_map[window]['pos_final_3d_m']:.6f}",
                f"{kf_map[window]['pos_final_3d_m']:.6f}",
                f"{cmp_map[window]['pos_final_3d_m_delta_kf_minus_current']:.6f}",
                f"{cmp_map[window]['pos_final_3d_m_ratio_current_over_kf']:.3f}",
                f"{current_map[window]['yaw_final_abs_deg']:.6f}",
                f"{kf_map[window]['yaw_final_abs_deg']:.6f}",
            ]
        )

    ratio_90 = float(cmp_map["90s"]["pos_final_3d_m_ratio_current_over_kf"]) if "90s" in cmp_map else float("nan")
    ratio_full = float(cmp_map["full"]["pos_final_3d_m_ratio_current_over_kf"]) if "full" in cmp_map else float("nan")
    if math.isfinite(ratio_90) and math.isfinite(ratio_full) and ratio_90 >= 2.0 and ratio_full >= 2.0:
        takeaway = (
            "pure INS gap already appears before any GNSS update path; next debugging should tighten on mechanization, "
            "bias/scale state propagation, and state feedback/reset semantics."
        )
    elif math.isfinite(ratio_90) and ratio_90 <= 1.5 and math.isfinite(ratio_full) and ratio_full <= 1.5:
        takeaway = "pure INS drift stays in the same order; next debugging should move to GNSS/combo update and outage runtime path."
    else:
        takeaway = (
            "pure INS gap is present but not yet enough to explain the whole outage discrepancy alone; next step should compare "
            "fixed-lever INS/GNSS under the same runtime contract."
        )

    lines = [
        "# data2 KF-GINS pure INS compare",
        "",
        f"- exp_id: `{EXP_ID_DEFAULT}`",
        f"- output_dir: `{rel_from_root(output_dir, REPO_ROOT)}`",
        f"- current_source_exp: `{source_manifest['exp_id']}`",
        f"- current_source_manifest: `{rel_from_root(REPO_ROOT / source_manifest['manifest_path'], REPO_ROOT) if 'manifest_path' in source_manifest else rel_from_root(SOURCE_MANIFEST_DEFAULT, REPO_ROOT)}`",
        f"- runtime_mode: `{runtime_mode}`",
        "- contract: current solver side reuses fresh pure INS `90 s` source artifact; `KF-GINS` uses the same IMU, same init PVA/noise contract, and a single dummy GNSS row placed after `endtime` so no GNSS update is consumed during the run.",
        "",
        "## Position Final Error Comparison",
        "## Takeaway",
        f"- {takeaway}",
    ]
    lines[lines.index("## Takeaway"):lines.index("## Takeaway")] = markdown_table(
        [
            "window",
            "current_pos_final_3d_m",
            "kf_pos_final_3d_m",
            "delta_kf_minus_current_m",
            "ratio_current_over_kf",
            "current_yaw_final_abs_deg",
            "kf_yaw_final_abs_deg",
        ],
        table_rows,
    ) + [""]
    return "\n".join(lines) + "\n"


def load_source_manifest(path: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest.setdefault("manifest_path", rel_from_root(path, REPO_ROOT))
    return manifest


def build_runtime_paths(
    runtime_mode: str,
    runtime_root: Path,
    output_dir: Path,
    kf_exe: Path,
    source_cfg: dict[str, Any],
    canonical_config_path: Path,
    canonical_dummy_gnss_path: Path,
    canonical_output_dir: Path,
    canonical_log_path: Path,
) -> dict[str, Path]:
    imu_src = (REPO_ROOT / source_cfg["fusion"]["imu_path"]).resolve()
    if runtime_mode == "preferred_root":
        ensure_dir(canonical_output_dir)
        return {
            "exe_path": kf_exe.resolve(),
            "config_path": canonical_config_path.resolve(),
            "output_dir": canonical_output_dir.resolve(),
            "imu_path": imu_src,
            "gnss_path": canonical_dummy_gnss_path.resolve(),
            "stdout_log_path": canonical_log_path.resolve(),
        }

    run_root = (runtime_root / output_dir.name).resolve()
    reset_run_dir(run_root, runtime_root.resolve())
    bin_dir = run_root / "bin"
    inputs_dir = run_root / "inputs"
    config_dir = run_root / "config"
    runtime_output_dir = run_root / "output"
    logs_dir = run_root / "logs"
    ensure_dir(bin_dir)
    ensure_dir(inputs_dir)
    ensure_dir(config_dir)
    ensure_dir(runtime_output_dir)
    ensure_dir(logs_dir)

    exe_dst = bin_dir / kf_exe.name
    imu_dst = inputs_dir / imu_src.name
    gnss_dst = inputs_dir / canonical_dummy_gnss_path.name
    config_dst = config_dir / canonical_config_path.name
    stdout_dst = logs_dir / "kf_gins_stdout.txt"

    copy_file(kf_exe.resolve(), exe_dst)
    copy_file(imu_src, imu_dst)
    copy_file(canonical_dummy_gnss_path.resolve(), gnss_dst)

    runtime_cfg = build_kf_gins_pure_ins_config(source_cfg, imu_dst, gnss_dst, runtime_output_dir)
    save_yaml(runtime_cfg, config_dst)

    return {
        "exe_path": exe_dst,
        "config_path": config_dst,
        "output_dir": runtime_output_dir,
        "imu_path": imu_dst,
        "gnss_path": gnss_dst,
        "stdout_log_path": stdout_dst,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare data2 pure INS drift between current solver and KF-GINS.")
    parser.add_argument("--source-manifest", type=Path, default=SOURCE_MANIFEST_DEFAULT)
    parser.add_argument("--source-config", type=Path, default=SOURCE_CONFIG_DEFAULT)
    parser.add_argument("--kf-exe", type=Path, default=KF_GINS_EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--fallback-root", type=Path, default=FALLBACK_ROOT_DEFAULT)
    args = parser.parse_args()
    args.source_manifest = Path(args.source_manifest)
    if not args.source_manifest.is_absolute():
        args.source_manifest = (REPO_ROOT / args.source_manifest).resolve()
    args.source_config = Path(args.source_config)
    if not args.source_config.is_absolute():
        args.source_config = (REPO_ROOT / args.source_config).resolve()
    args.kf_exe = Path(args.kf_exe)
    if not args.kf_exe.is_absolute():
        args.kf_exe = (REPO_ROOT / args.kf_exe).resolve()
    args.output_dir = Path(args.output_dir)
    if not args.output_dir.is_absolute():
        args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.fallback_root = Path(args.fallback_root)
    return args


def main() -> None:
    args = parse_args()
    if not args.source_manifest.exists():
        raise FileNotFoundError(f"missing source manifest: {args.source_manifest}")
    if not args.source_config.exists():
        raise FileNotFoundError(f"missing source config: {args.source_config}")
    if not args.kf_exe.exists():
        raise FileNotFoundError(f"missing KF-GINS exe: {args.kf_exe}")

    source_manifest = load_source_manifest(args.source_manifest)
    source_cfg = load_yaml(args.source_config)
    current_all_states_path = (REPO_ROOT / source_manifest["all_states_path"]).resolve()
    if not current_all_states_path.exists():
        raise FileNotFoundError(f"missing current all_states: {current_all_states_path}")

    if args.output_dir.exists():
        shutil.rmtree(args.output_dir)
    ensure_dir(args.output_dir)
    artifacts_dir = args.output_dir / "artifacts"
    plots_dir = args.output_dir / "plots"
    ensure_dir(artifacts_dir)
    ensure_dir(plots_dir)

    current_err_frame = load_current_error_frame(current_all_states_path)
    current_window_df = pd.DataFrame(compute_system_window_metrics("current_solver", current_err_frame))
    current_window_path = args.output_dir / "current_window_metrics.csv"
    current_window_df.to_csv(current_window_path, index=False, encoding="utf-8-sig")

    init = source_cfg["fusion"]["init"]
    endtime = float(source_cfg["fusion"]["finaltime"])
    dummy_gnss_path = artifacts_dir / "dummy_gnss_future_only.txt"
    dummy_gnss_meta = write_dummy_gnss_file(
        dummy_gnss_path,
        timestamp=endtime + 10.0,
        lat_deg=float(init["init_pos_lla"][0]),
        lon_deg=float(init["init_pos_lla"][1]),
        h_m=float(init["init_pos_lla"][2]),
    )

    canonical_kf_config = build_kf_gins_pure_ins_config(
        source_cfg,
        (REPO_ROOT / source_cfg["fusion"]["imu_path"]).resolve(),
        dummy_gnss_path.resolve(),
        args.output_dir.resolve(),
    )
    canonical_kf_config_path = artifacts_dir / "kf_gins_pure_ins_config.yaml"
    save_yaml(canonical_kf_config, canonical_kf_config_path)

    canonical_stdout_path = args.output_dir / "kf_gins_stdout.txt"
    runtime_root, runtime_mode = resolve_runtime_root(REPO_ROOT, args.fallback_root)
    runtime_paths = build_runtime_paths(
        runtime_mode=runtime_mode,
        runtime_root=runtime_root,
        output_dir=args.output_dir,
        kf_exe=args.kf_exe,
        source_cfg=source_cfg,
        canonical_config_path=canonical_kf_config_path,
        canonical_dummy_gnss_path=dummy_gnss_path,
        canonical_output_dir=args.output_dir,
        canonical_log_path=canonical_stdout_path,
    )

    stdout_text = run_kf_gins(runtime_paths["exe_path"], runtime_paths["config_path"], runtime_paths["stdout_log_path"])
    if runtime_mode != "preferred_root":
        copy_runtime_outputs(runtime_paths["output_dir"], args.output_dir)
        copy_file(runtime_paths["stdout_log_path"], canonical_stdout_path)

    navresult_path = args.output_dir / "KF_GINS_Navresult.nav"
    imu_error_path = args.output_dir / "KF_GINS_IMU_ERR.txt"
    std_path = args.output_dir / "KF_GINS_STD.txt"
    nav_frame = load_kf_gins_navresult(navresult_path)
    imu_err_frame = load_kf_gins_imu_error(imu_error_path)

    truth_path = (REPO_ROOT / source_cfg["fusion"]["pos_path"]).resolve()
    truth_frame = load_truth_nav(truth_path)
    kf_err_frame = build_kf_error_frame(nav_frame, truth_frame)

    current_raw = pd.read_csv(current_all_states_path)
    truth_bg_z_degh = float(current_raw["truth_bg_z_degh"].iloc[0]) if "truth_bg_z_degh" in current_raw.columns else float("nan")
    if math.isfinite(truth_bg_z_degh):
        kf_err_frame = attach_bg_z_error(kf_err_frame, imu_err_frame, truth_bg_z_degh)

    kf_window_df = pd.DataFrame(compute_system_window_metrics("kf_gins", kf_err_frame))
    kf_window_path = args.output_dir / "kf_window_metrics.csv"
    kf_window_df.to_csv(kf_window_path, index=False, encoding="utf-8-sig")

    comparison_window_df = build_comparison_window_frame(current_window_df, kf_window_df)
    comparison_window_path = args.output_dir / "comparison_window_metrics.csv"
    comparison_window_df.to_csv(comparison_window_path, index=False, encoding="utf-8-sig")

    plot_compare_overview(current_err_frame, kf_err_frame, plots_dir / "pure_ins_compare_overview.png")
    plot_bg_z_compare(current_err_frame, kf_err_frame, plots_dir / "bg_z_error_compare.png")

    summary_path = args.output_dir / "summary.md"
    summary_path.write_text(
        build_summary(source_manifest, runtime_mode, current_window_df, kf_window_df, comparison_window_df, args.output_dir),
        encoding="utf-8",
    )

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_manifest": rel_from_root(args.source_manifest, REPO_ROOT),
        "source_config": rel_from_root(args.source_config, REPO_ROOT),
        "kf_exe": rel_from_root(args.kf_exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "truth_path": rel_from_root(truth_path, REPO_ROOT),
        "dummy_gnss_path": rel_from_root(dummy_gnss_path, REPO_ROOT),
        "dummy_gnss_metadata": dummy_gnss_meta,
        "runtime_mode": runtime_mode,
        "runtime_root": str(runtime_root),
        "current_all_states_path": rel_from_root(current_all_states_path, REPO_ROOT),
        "current_window_metrics_path": rel_from_root(current_window_path, REPO_ROOT),
        "kf_window_metrics_path": rel_from_root(kf_window_path, REPO_ROOT),
        "comparison_window_metrics_path": rel_from_root(comparison_window_path, REPO_ROOT),
        "summary_path": rel_from_root(summary_path, REPO_ROOT),
        "navresult_path": rel_from_root(navresult_path, REPO_ROOT),
        "imu_error_path": rel_from_root(imu_error_path, REPO_ROOT),
        "std_path": rel_from_root(std_path, REPO_ROOT),
        "stdout_path": rel_from_root(canonical_stdout_path, REPO_ROOT),
        "canonical_kf_config_path": rel_from_root(canonical_kf_config_path, REPO_ROOT),
        "plot_paths": [
            rel_from_root(plots_dir / "pure_ins_compare_overview.png", REPO_ROOT),
            rel_from_root(plots_dir / "bg_z_error_compare.png", REPO_ROOT),
        ],
        "kf_stdout_bytes": len(stdout_text.encode("utf-8", errors="ignore")),
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    print(
        json.dumps(
            {
                "exp_id": args.exp_id,
                "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
                "summary": rel_from_root(summary_path, REPO_ROOT),
                "manifest": rel_from_root(manifest_path, REPO_ROOT),
                "comparison_window_metrics": rel_from_root(comparison_window_path, REPO_ROOT),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
