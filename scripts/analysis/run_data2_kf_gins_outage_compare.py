#!/usr/bin/env python3
"""Run a shared-subset KF-GINS vs current-solver data2 GNSS outage comparison."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import (
    ensure_dir,
    interp_truth,
    llh_deg_to_ecef,
    load_truth_ecef,
    load_yaml,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_rtk_outage_eval import (
    aggregate_outage_segments,
    compute_global_metrics,
    compute_outage_segments,
    extract_on_windows,
    format_metric,
    invert_windows,
    json_safe,
    markdown_table,
)


SOURCE_MANIFEST_DEFAULT = (
    REPO_ROOT / "output" / "data2_baseline_ins_gnss_outage_no_odo_nhc_r2" / "manifest.json"
)
SOURCE_CONFIG_DEFAULT = (
    REPO_ROOT
    / "output"
    / "data2_baseline_ins_gnss_outage_no_odo_nhc_r2"
    / "artifacts"
    / "cases"
    / "data2_baseline_ins_gnss_outage_no_odo_nhc"
    / "config_data2_baseline_ins_gnss_outage_no_odo_nhc.yaml"
)
KF_GINS_EXE_DEFAULT = REPO_ROOT / "KF-GINS" / "bin" / "Release" / "KF-GINS.exe"
FALLBACK_ROOT_DEFAULT = Path(r"C:\temp\uwb_kf_gins_compare")


def path_is_ascii_safe(path: Path) -> bool:
    try:
        str(path).encode("ascii")
    except UnicodeEncodeError:
        return False
    return True


def default_runtime_probe(root: Path) -> bool:
    return root.exists() and path_is_ascii_safe(root)


def resolve_runtime_root(
    preferred_root: Path,
    fallback_root: Path,
    probe: Callable[[Path], bool] | None = None,
) -> tuple[Path, str]:
    probe_fn = probe or default_runtime_probe
    preferred_root = Path(preferred_root)
    fallback_root = Path(fallback_root)
    if probe_fn(preferred_root):
        return preferred_root, "preferred_root"
    ensure_dir(fallback_root)
    return fallback_root, "fallback_ascii"


def load_truth_nav(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(path, sep=r"\s+", header=None)
    if raw.shape[1] < 10:
        raise RuntimeError(f"truth file columns < 10: {path}")
    raw = raw.iloc[:, :10].copy()
    raw.columns = [
        "timestamp",
        "lat_deg",
        "lon_deg",
        "h_m",
        "vn_mps",
        "ve_mps",
        "vd_mps",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    ]
    ecef = llh_deg_to_ecef(
        raw["lat_deg"].to_numpy(dtype=float),
        raw["lon_deg"].to_numpy(dtype=float),
        raw["h_m"].to_numpy(dtype=float),
    )
    raw["ecef_x"] = ecef[:, 0]
    raw["ecef_y"] = ecef[:, 1]
    raw["ecef_z"] = ecef[:, 2]
    return raw


def angle_wrap_deg(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def interp_angle_deg(target_t: np.ndarray, source_t: np.ndarray, source_angle_deg: np.ndarray) -> np.ndarray:
    source_angle_rad = np.unwrap(np.deg2rad(source_angle_deg))
    interp_rad = np.interp(target_t, source_t, source_angle_rad)
    return np.rad2deg(interp_rad)


def load_gnss_table(path: Path) -> tuple[np.ndarray, list[str]]:
    header: list[str] = []
    with path.open("r", encoding="utf-8") as f:
        first = f.readline().strip()
    has_header = bool(first) and (not first.split()[0].lstrip("-").replace(".", "", 1).isdigit())
    if has_header:
        with path.open("r", encoding="utf-8") as f:
            header = [f.readline().rstrip("\n")]
    data = np.loadtxt(path, comments="#", skiprows=1 if has_header else 0)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data, header


def filter_gnss_by_windows(
    input_path: Path,
    output_path: Path,
    enabled_windows: list[tuple[float, float]],
) -> dict[str, float]:
    data, header = load_gnss_table(input_path)
    timestamps = data[:, 0].astype(float)
    mask = np.zeros(timestamps.shape[0], dtype=bool)
    for start_t, end_t in enabled_windows:
        mask |= (timestamps >= float(start_t)) & (timestamps <= float(end_t))
    filtered = data[mask]
    ensure_dir(output_path.parent)
    with output_path.open("w", encoding="utf-8") as f:
        for line in header:
            f.write(line + "\n")
        np.savetxt(f, filtered, fmt="%.9f")
    return {
        "rows_raw": int(data.shape[0]),
        "rows_kept": int(filtered.shape[0]),
        "ratio_kept": float(filtered.shape[0] / max(1, data.shape[0])),
    }


def sigma_radps_to_degph(value: float) -> float:
    return float(value) * 180.0 / math.pi * 3600.0


def sigma_mps2_to_mgal(value: float) -> float:
    return float(value) * 1.0e5


def sigma_scale_to_ppm(value: float) -> float:
    return float(value) * 1.0e6


def std_var_radps_to_degph(var_value: float) -> float:
    return sigma_radps_to_degph(math.sqrt(float(var_value)))


def std_var_mps2_to_mgal(var_value: float) -> float:
    return sigma_mps2_to_mgal(math.sqrt(float(var_value)))


def std_var_scale_to_ppm(var_value: float) -> float:
    return sigma_scale_to_ppm(math.sqrt(float(var_value)))


def _convert_init_std_vec(
    init: dict[str, Any],
    key: str,
    fallback_var_indices: list[int],
    std_converter,
    var_converter,
) -> list[float]:
    values = init.get(key)
    if isinstance(values, list) and len(values) >= 3:
        return [float(std_converter(v)) for v in values[:3]]
    return [float(var_converter(init["P0_diag"][idx])) for idx in fallback_var_indices]


def _maybe_zero_vec(values: list[float], disabled: bool) -> list[float]:
    if disabled:
        return [0.0] * len(values)
    return [float(v) for v in values]


def build_kf_gins_config(source_cfg: dict[str, Any], imupath: Path, gnsspath: Path, outputpath: Path) -> dict[str, Any]:
    fusion = source_cfg["fusion"]
    noise = fusion["noise"]
    init = fusion["init"]
    ablation = fusion.get("ablation", {})

    init_bg_std = _convert_init_std_vec(
        init,
        "std_bg",
        [12, 13, 14],
        sigma_radps_to_degph,
        std_var_radps_to_degph,
    )
    init_ba_std = _convert_init_std_vec(
        init,
        "std_ba",
        [9, 10, 11],
        sigma_mps2_to_mgal,
        std_var_mps2_to_mgal,
    )
    init_sg_std = _convert_init_std_vec(
        init,
        "std_sg",
        [15, 16, 17],
        sigma_scale_to_ppm,
        std_var_scale_to_ppm,
    )
    init_sa_std = _convert_init_std_vec(
        init,
        "std_sa",
        [18, 19, 20],
        sigma_scale_to_ppm,
        std_var_scale_to_ppm,
    )

    arw = float(noise["sigma_gyro"]) * 180.0 / math.pi * math.sqrt(3600.0)
    vrw = float(noise["sigma_acc"]) * math.sqrt(3600.0)
    gbstd = _maybe_zero_vec(
        [sigma_radps_to_degph(float(noise["sigma_bg"]))] * 3,
        bool(ablation.get("disable_gyro_bias", False)),
    )
    abstd = _maybe_zero_vec(
        [sigma_mps2_to_mgal(float(noise["sigma_ba"]))] * 3,
        bool(ablation.get("disable_accel_bias", False)),
    )
    gsstd = _maybe_zero_vec(
        [sigma_scale_to_ppm(float(noise["sigma_sg"]))] * 3,
        bool(ablation.get("disable_gyro_scale", False)),
    )
    asstd = _maybe_zero_vec(
        [sigma_scale_to_ppm(float(noise["sigma_sa"]))] * 3,
        bool(ablation.get("disable_accel_scale", False)),
    )
    corrtime = float(noise["markov_corr_time"]) / 3600.0

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
        "initgyrbias": [sigma_radps_to_degph(float(v)) for v in init.get("bg0", [0.0, 0.0, 0.0])[:3]],
        "initaccbias": [sigma_mps2_to_mgal(float(v)) for v in init.get("ba0", [0.0, 0.0, 0.0])[:3]],
        "initgyrscale": [sigma_scale_to_ppm(float(v)) for v in init.get("sg0", [0.0, 0.0, 0.0])[:3]],
        "initaccscale": [sigma_scale_to_ppm(float(v)) for v in init.get("sa0", [0.0, 0.0, 0.0])[:3]],
        "initposstd": [float(v) for v in init["std_pos"]],
        "initvelstd": [float(v) for v in init["std_vel"]],
        "initattstd": [float(v) for v in init["std_att"]],
        "initbgstd": _maybe_zero_vec(
            init_bg_std,
            bool(ablation.get("disable_gyro_bias", False)),
        ),
        "initbastd": _maybe_zero_vec(
            init_ba_std,
            bool(ablation.get("disable_accel_bias", False)),
        ),
        "initsgstd": _maybe_zero_vec(
            init_sg_std,
            bool(ablation.get("disable_gyro_scale", False)),
        ),
        "initsastd": _maybe_zero_vec(
            init_sa_std,
            bool(ablation.get("disable_accel_scale", False)),
        ),
        "imunoise": {
            "arw": [arw] * 3,
            "vrw": [vrw] * 3,
            "gbstd": gbstd,
            "abstd": abstd,
            "gsstd": gsstd,
            "asstd": asstd,
            "corrtime": corrtime,
        },
        "antlever": [float(v) for v in init["gnss_lever_arm0"]],
    }


def load_kf_gins_navresult(nav_path: Path) -> pd.DataFrame:
    raw = pd.read_csv(nav_path, sep=r"\s+", header=None)
    if raw.shape[1] < 11:
        raise RuntimeError(f"KF-GINS navresult columns < 11: {nav_path}")
    raw = raw.iloc[:, :11].copy()
    raw.columns = [
        "gps_week",
        "timestamp",
        "lat_deg",
        "lon_deg",
        "h_m",
        "vn_mps",
        "ve_mps",
        "vd_mps",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
    ]
    ecef = llh_deg_to_ecef(
        raw["lat_deg"].to_numpy(dtype=float),
        raw["lon_deg"].to_numpy(dtype=float),
        raw["h_m"].to_numpy(dtype=float),
    )
    raw["ecef_x"] = ecef[:, 0]
    raw["ecef_y"] = ecef[:, 1]
    raw["ecef_z"] = ecef[:, 2]
    return raw


def load_kf_gins_imu_error(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(path, sep=r"\s+", header=None)
    if raw.shape[1] < 13:
        raise RuntimeError(f"KF-GINS imu error columns < 13: {path}")
    raw = raw.iloc[:, :13].copy()
    raw.columns = [
        "timestamp",
        "bg_x_degh",
        "bg_y_degh",
        "bg_z_degh",
        "ba_x_mgal",
        "ba_y_mgal",
        "ba_z_mgal",
        "sg_x_ppm",
        "sg_y_ppm",
        "sg_z_ppm",
        "sa_x_ppm",
        "sa_y_ppm",
        "sa_z_ppm",
    ]
    return raw


def copy_file(src: Path, dst: Path) -> None:
    ensure_dir(dst.parent)
    shutil.copy2(src, dst)


def reset_run_dir(path: Path, allowed_root: Path) -> None:
    path = path.resolve()
    allowed_root = allowed_root.resolve()
    if allowed_root not in path.parents and path != allowed_root:
        raise RuntimeError(f"refuse to reset path outside allowed root: {path}")
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def build_runtime_paths(
    runtime_mode: str,
    runtime_root: Path,
    output_dir: Path,
    kf_exe: Path,
    canonical_config_path: Path,
    canonical_gnss_path: Path,
    canonical_output_dir: Path,
    canonical_log_path: Path,
    source_cfg: dict[str, Any],
) -> dict[str, Path]:
    imu_src = (REPO_ROOT / source_cfg["fusion"]["imu_path"]).resolve()
    if runtime_mode == "preferred_root":
        ensure_dir(canonical_output_dir)
        return {
            "exe_path": kf_exe.resolve(),
            "config_path": canonical_config_path.resolve(),
            "output_dir": canonical_output_dir.resolve(),
            "imu_path": imu_src,
            "gnss_path": canonical_gnss_path.resolve(),
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
    gnss_dst = inputs_dir / canonical_gnss_path.name
    config_dst = config_dir / canonical_config_path.name
    stdout_dst = logs_dir / "kf_gins_stdout.txt"

    copy_file(kf_exe.resolve(), exe_dst)
    copy_file(imu_src, imu_dst)
    copy_file(canonical_gnss_path.resolve(), gnss_dst)

    runtime_cfg = build_kf_gins_config(source_cfg, imu_dst, gnss_dst, runtime_output_dir)
    save_yaml(runtime_cfg, config_dst)

    return {
        "exe_path": exe_dst,
        "config_path": config_dst,
        "output_dir": runtime_output_dir,
        "imu_path": imu_dst,
        "gnss_path": gnss_dst,
        "stdout_log_path": stdout_dst,
    }


def run_kf_gins(exe_path: Path, config_path: Path, stdout_log_path: Path) -> str:
    cmd = [str(exe_path), str(config_path)]
    proc = subprocess.run(
        cmd,
        cwd=str(exe_path.parent),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    stdout_log_path.write_text(merged, encoding="utf-8")
    if proc.returncode != 0:
        raise RuntimeError(f"KF-GINS failed ({proc.returncode}): {' '.join(cmd)}\n{merged}")
    return merged


def copy_runtime_outputs(runtime_output_dir: Path, repo_output_dir: Path) -> dict[str, Path]:
    ensure_dir(repo_output_dir)
    copied: dict[str, Path] = {}
    for name in ("KF_GINS_Navresult.nav", "KF_GINS_IMU_ERR.txt", "KF_GINS_STD.txt"):
        src = runtime_output_dir / name
        if not src.exists():
            raise FileNotFoundError(f"missing KF-GINS runtime output: {src}")
        dst = repo_output_dir / name
        copy_file(src, dst)
        copied[name] = dst
    return copied


def build_error_frame(nav_frame: pd.DataFrame, truth_frame: pd.DataFrame) -> pd.DataFrame:
    nav_t = nav_frame["timestamp"].to_numpy(dtype=float)
    truth_t = truth_frame["timestamp"].to_numpy(dtype=float)
    truth_ecef = truth_frame[["ecef_x", "ecef_y", "ecef_z"]].to_numpy(dtype=float)
    nav_ecef = nav_frame[["ecef_x", "ecef_y", "ecef_z"]].to_numpy(dtype=float)
    truth_interp = interp_truth(nav_t, truth_t, truth_ecef)
    err_xyz = nav_ecef - truth_interp
    err_3d = np.linalg.norm(err_xyz, axis=1)

    yaw_truth = interp_angle_deg(nav_t, truth_t, truth_frame["yaw_deg"].to_numpy(dtype=float))
    pitch_truth = interp_angle_deg(nav_t, truth_t, truth_frame["pitch_deg"].to_numpy(dtype=float))
    roll_truth = interp_angle_deg(nav_t, truth_t, truth_frame["roll_deg"].to_numpy(dtype=float))

    yaw_err = angle_wrap_deg(nav_frame["yaw_deg"].to_numpy(dtype=float) - yaw_truth)
    pitch_err = angle_wrap_deg(nav_frame["pitch_deg"].to_numpy(dtype=float) - pitch_truth)
    roll_err = angle_wrap_deg(nav_frame["roll_deg"].to_numpy(dtype=float) - roll_truth)

    out = nav_frame.copy()
    out["err_x_m"] = err_xyz[:, 0]
    out["err_y_m"] = err_xyz[:, 1]
    out["err_z_m"] = err_xyz[:, 2]
    out["err_3d_m"] = err_3d
    out["roll_err_deg"] = roll_err
    out["pitch_err_deg"] = pitch_err
    out["yaw_err_deg"] = yaw_err
    return out


def compute_window_rows(
    case_id: str,
    err_frame: pd.DataFrame,
    imu_err_frame: pd.DataFrame,
    windows: list[tuple[str, str, float, float]],
) -> list[dict[str, Any]]:
    t_nav = err_frame["timestamp"].to_numpy(dtype=float)
    t_imu = imu_err_frame["timestamp"].to_numpy(dtype=float)
    rows: list[dict[str, Any]] = []
    for idx, (window_type, window_name, start_t, end_t) in enumerate(windows):
        include_end = idx == len(windows) - 1
        if include_end:
            mask_nav = (t_nav >= start_t) & (t_nav <= end_t)
            mask_imu = (t_imu >= start_t) & (t_imu <= end_t)
        else:
            mask_nav = (t_nav >= start_t) & (t_nav < end_t)
            mask_imu = (t_imu >= start_t) & (t_imu < end_t)
        frame = err_frame.loc[mask_nav]
        if frame.empty:
            continue
        err_xyz = frame[["err_x_m", "err_y_m", "err_z_m"]].to_numpy(dtype=float)
        err3 = frame["err_3d_m"].to_numpy(dtype=float)
        imu_slice = imu_err_frame.loc[mask_imu]
        bg_z_abs_max = float(imu_slice["bg_z_degh"].abs().max()) if not imu_slice.empty else float("nan")
        rows.append(
            {
                "case_id": case_id,
                "window_type": window_type,
                "window_name": window_name,
                "start_time": float(start_t),
                "end_time": float(end_t),
                "samples": int(frame.shape[0]),
                "rmse_3d_m": float(np.sqrt(np.mean(err3 * err3))),
                "p95_3d_m": float(np.percentile(err3, 95)),
                "final_err_3d_m": float(err3[-1]),
                "yaw_err_abs_max_deg": float(frame["yaw_err_deg"].abs().max()),
                "bg_z_err_abs_max_degh": bg_z_abs_max,
            }
        )
    return rows


def build_phase_window_rows(
    phase_windows: dict[str, list[float]],
    off_windows: list[tuple[float, float]],
) -> list[tuple[str, str, float, float]]:
    windows: list[tuple[str, str, float, float]] = []
    phase_name_map = {
        "phase1": "phase1_ins_gnss",
        "phase2": "phase2_ins_gnss",
        "phase3": "phase3_periodic_gnss_outage",
    }
    for key in ("phase1", "phase2", "phase3"):
        if key not in phase_windows:
            continue
        start_t, end_t = phase_windows[key]
        windows.append(("phase", phase_name_map[key], float(start_t), float(end_t)))
    for idx, (start_t, end_t) in enumerate(off_windows, start=1):
        windows.append(("gnss_off", f"gnss_off_{idx:02d}", float(start_t), float(end_t)))
    return windows


def summarize_case_metrics(
    case_id: str,
    err_frame: pd.DataFrame,
    imu_err_frame: pd.DataFrame,
    off_windows: list[tuple[float, float]],
    rel_paths: dict[str, str],
) -> dict[str, Any]:
    err_xyz = err_frame[["err_x_m", "err_y_m", "err_z_m"]].to_numpy(dtype=float)
    metrics = compute_global_metrics(err_xyz)
    segment_rows = compute_outage_segments(
        err_frame["timestamp"].to_numpy(dtype=float),
        err_xyz,
        off_windows,
    )
    metrics.update(aggregate_outage_segments(segment_rows))
    metrics.update(
        {
            "case_id": case_id,
            "case_label": "KF-GINS shared-subset INS/GNSS outage",
            "navresult_path": rel_paths["navresult_path"],
            "imu_error_path": rel_paths["imu_error_path"],
            "std_path": rel_paths["std_path"],
            "stdout_path": rel_paths["stdout_path"],
            "config_path": rel_paths["config_path"],
            "segment_rows": json.dumps(json_safe(segment_rows), ensure_ascii=False),
            "yaw_err_max_abs_deg": float(err_frame["yaw_err_deg"].abs().max()),
            "pitch_err_max_abs_deg": float(err_frame["pitch_err_deg"].abs().max()),
            "roll_err_max_abs_deg": float(err_frame["roll_err_deg"].abs().max()),
            "bg_z_err_max_abs_degh": float(imu_err_frame["bg_z_degh"].abs().max()),
        }
    )
    return metrics


def normalize_current_case_metrics(row: pd.Series) -> dict[str, float]:
    bg_z_key = "bg_z_err_abs_max_degh"
    if bg_z_key not in row.index:
        bg_z_key = "bg_z_degh_err_max_abs"
    return {
        "overall_rmse_3d_m": float(row["overall_rmse_3d_m_aux"]),
        "overall_p95_3d_m": float(row["overall_p95_3d_m_aux"]),
        "overall_final_err_3d_m": float(row["overall_final_err_3d_m_aux"]),
        "mean_outage_final_err_3d_m": float(row["mean_outage_final_err_3d_m"]),
        "max_outage_final_err_3d_m": float(row["max_outage_final_err_3d_m"]),
        "mean_outage_rmse_3d_m": float(row["mean_outage_rmse_3d_m"]),
        "max_outage_rmse_3d_m": float(row["max_outage_rmse_3d_m"]),
        "yaw_err_max_abs_deg": float(row["yaw_err_max_abs_deg"]),
        "bg_z_err_max_abs_degh": float(row[bg_z_key]),
    }


def normalize_kf_case_metrics(row: dict[str, Any]) -> dict[str, float]:
    return {
        "overall_rmse_3d_m": float(row["overall_rmse_3d_m_aux"]),
        "overall_p95_3d_m": float(row["overall_p95_3d_m_aux"]),
        "overall_final_err_3d_m": float(row["overall_final_err_3d_m_aux"]),
        "mean_outage_final_err_3d_m": float(row["mean_outage_final_err_3d_m"]),
        "max_outage_final_err_3d_m": float(row["max_outage_final_err_3d_m"]),
        "mean_outage_rmse_3d_m": float(row["mean_outage_rmse_3d_m"]),
        "max_outage_rmse_3d_m": float(row["max_outage_rmse_3d_m"]),
        "yaw_err_max_abs_deg": float(row["yaw_err_max_abs_deg"]),
        "bg_z_err_max_abs_degh": float(row["bg_z_err_max_abs_degh"]),
    }


def build_comparison_case_frame(current_row: pd.Series, kf_metrics: dict[str, Any]) -> pd.DataFrame:
    current_norm = normalize_current_case_metrics(current_row)
    kf_norm = normalize_kf_case_metrics(kf_metrics)
    metric_names = list(current_norm.keys())
    rows = [
        {"system": "current_solver", **current_norm},
        {"system": "kf_gins_shared_subset", **kf_norm},
        {
            "system": "delta_kf_minus_current",
            **{name: float(kf_norm[name] - current_norm[name]) for name in metric_names},
        },
    ]
    return pd.DataFrame(rows)


def build_comparison_phase_frame(current_phase_df: pd.DataFrame, kf_phase_df: pd.DataFrame) -> pd.DataFrame:
    keep_cols = [
        "window_name",
        "rmse_3d_m",
        "p95_3d_m",
        "final_err_3d_m",
        "yaw_err_abs_max_deg",
        "bg_z_err_abs_max_degh",
    ]
    current_use = current_phase_df[keep_cols].copy()
    kf_use = kf_phase_df[keep_cols].copy()
    merged = current_use.merge(kf_use, on="window_name", suffixes=("_current", "_kf"))
    rows: list[dict[str, Any]] = []
    for _, row in merged.iterrows():
        out: dict[str, Any] = {"window_name": row["window_name"]}
        for metric in keep_cols[1:]:
            current_key = f"{metric}_current"
            kf_key = f"{metric}_kf"
            out[current_key] = float(row[current_key])
            out[kf_key] = float(row[kf_key])
            out[f"{metric}_delta_kf_minus_current"] = float(row[kf_key] - row[current_key])
        rows.append(out)
    return pd.DataFrame(rows)


def load_current_phase_metrics(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    numeric_cols = ["start_time", "end_time", "samples", "rmse_3d_m", "p95_3d_m", "final_err_3d_m"]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def load_current_sol(path: Path) -> tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(path, sep=r"\s+", comment="#")
    if {"timestamp", "fused_x", "fused_y", "fused_z"}.issubset(df.columns):
        return (
            df["timestamp"].to_numpy(dtype=float),
            df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float),
        )
    raw = pd.read_csv(path, sep=r"\s+", header=None)
    if raw.shape[1] < 4:
        raise RuntimeError(f"current solver solution columns < 4: {path}")
    return raw.iloc[:, 0].to_numpy(dtype=float), raw.iloc[:, 1:4].to_numpy(dtype=float)


def plot_error_time_compare(
    current_sol_path: Path,
    kf_err_frame: pd.DataFrame,
    truth_path: Path,
    phase_windows: dict[str, list[float]],
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    current_t, current_xyz = load_current_sol(current_sol_path)
    truth_t, truth_xyz = load_truth_ecef(truth_path)
    current_truth = interp_truth(current_t, truth_t, truth_xyz)
    current_err_3d = np.linalg.norm(current_xyz - current_truth, axis=1)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(current_t, current_err_3d, label="current solver", linewidth=1.0)
    ax.plot(
        kf_err_frame["timestamp"].to_numpy(dtype=float),
        kf_err_frame["err_3d_m"].to_numpy(dtype=float),
        label="KF-GINS shared subset",
        linewidth=1.0,
    )
    if "phase3" in phase_windows:
        start_t, end_t = phase_windows["phase3"]
        ax.axvspan(float(start_t), float(end_t), color="#f4f1d0", alpha=0.25, label="phase3")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("3D position error [m]")
    ax.set_title("data2 INS/GNSS outage 3D error comparison")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_outage_compare(
    current_phase_df: pd.DataFrame,
    kf_phase_df: pd.DataFrame,
    metric_col: str,
    ylabel: str,
    title: str,
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    current_off = current_phase_df[current_phase_df["window_type"] == "gnss_off"].copy()
    kf_off = kf_phase_df[kf_phase_df["window_type"] == "gnss_off"].copy()
    merged = current_off.merge(kf_off, on="window_name", suffixes=("_current", "_kf"))
    if merged.empty:
        return
    labels = merged["window_name"].tolist()
    x = np.arange(len(labels))
    width = 0.38

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.bar(x - width / 2, merged[f"{metric_col}_current"], width, label="current solver")
    ax.bar(x + width / 2, merged[f"{metric_col}_kf"], width, label="KF-GINS shared subset")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_summary(
    summary_path: Path,
    exp_id: str,
    output_dir: Path,
    source_manifest_rel: str,
    runtime_mode: str,
    current_case_row: pd.Series,
    current_phase_df: pd.DataFrame,
    kf_case_metrics: dict[str, Any],
    kf_phase_df: pd.DataFrame,
    comparison_case_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    lines = [
        "# data2 KF-GINS shared-subset outage comparison",
        "",
        f"- exp_id: `{exp_id}`",
        "- comparison_scope: `shared INS/GNSS subset only`",
        "- caveat: `KF-GINS is 21-state, no ODO/NHC/extrinsic-estimation runtime phases; this is not a 31-state like-for-like replication.`",
        "- KF-GINS lever handling: `fixed true GNSS lever [0.15, -0.22, -1.15] m`",
        f"- source_reference: `{source_manifest_rel}`",
        f"- output_dir: `{rel_from_root(output_dir, REPO_ROOT)}`",
        f"- runtime_mode: `{runtime_mode}`",
        f"- generated_at: `{dt.datetime.now().isoformat(timespec='seconds')}`",
        "",
        "## Headline Metrics",
    ]
    headline_rows = []
    for _, row in comparison_case_df.iterrows():
        headline_rows.append(
            [
                str(row["system"]),
                format_metric(row["overall_rmse_3d_m"]),
                format_metric(row["mean_outage_rmse_3d_m"]),
                format_metric(row["max_outage_rmse_3d_m"]),
                format_metric(row["mean_outage_final_err_3d_m"]),
                format_metric(row["max_outage_final_err_3d_m"]),
                format_metric(row["overall_final_err_3d_m"]),
            ]
        )
    lines.extend(
        markdown_table(
            [
                "system",
                "overall_rmse3d_m",
                "mean_outage_rmse3d_m",
                "max_outage_rmse3d_m",
                "mean_outage_final3d_m",
                "max_outage_final3d_m",
                "final_3d_m",
            ],
            headline_rows,
        )
    )

    current_off_01 = current_phase_df[current_phase_df["window_name"] == "gnss_off_01"].iloc[0]
    current_off_09 = current_phase_df[current_phase_df["window_name"] == "gnss_off_09"].iloc[0]
    kf_off_01 = kf_phase_df[kf_phase_df["window_name"] == "gnss_off_01"].iloc[0]
    kf_off_09 = kf_phase_df[kf_phase_df["window_name"] == "gnss_off_09"].iloc[0]
    current_phase3 = current_phase_df[current_phase_df["window_name"] == "phase3_periodic_gnss_outage"].iloc[0]
    kf_phase3 = kf_phase_df[kf_phase_df["window_name"] == "phase3_periodic_gnss_outage"].iloc[0]

    lines.extend(
        [
            "",
            "## Key Windows",
            "| window | current_rmse3d_m | kf_rmse3d_m | delta_rmse3d_m | current_final3d_m | kf_final3d_m | delta_final3d_m |",
            "| --- | --- | --- | --- | --- | --- | --- |",
            "| phase3_periodic_gnss_outage | "
            f"{format_metric(current_phase3['rmse_3d_m'])} | {format_metric(kf_phase3['rmse_3d_m'])} | "
            f"{format_metric(float(kf_phase3['rmse_3d_m']) - float(current_phase3['rmse_3d_m']))} | "
            f"{format_metric(current_phase3['final_err_3d_m'])} | {format_metric(kf_phase3['final_err_3d_m'])} | "
            f"{format_metric(float(kf_phase3['final_err_3d_m']) - float(current_phase3['final_err_3d_m']))} |",
            "| gnss_off_01 | "
            f"{format_metric(current_off_01['rmse_3d_m'])} | {format_metric(kf_off_01['rmse_3d_m'])} | "
            f"{format_metric(float(kf_off_01['rmse_3d_m']) - float(current_off_01['rmse_3d_m']))} | "
            f"{format_metric(current_off_01['final_err_3d_m'])} | {format_metric(kf_off_01['final_err_3d_m'])} | "
            f"{format_metric(float(kf_off_01['final_err_3d_m']) - float(current_off_01['final_err_3d_m']))} |",
            "| gnss_off_09 | "
            f"{format_metric(current_off_09['rmse_3d_m'])} | {format_metric(kf_off_09['rmse_3d_m'])} | "
            f"{format_metric(float(kf_off_09['rmse_3d_m']) - float(current_off_09['rmse_3d_m']))} | "
            f"{format_metric(current_off_09['final_err_3d_m'])} | {format_metric(kf_off_09['final_err_3d_m'])} | "
            f"{format_metric(float(kf_off_09['final_err_3d_m']) - float(current_off_09['final_err_3d_m']))} |",
            "",
            "## Current Reference Row",
            f"- case_id: `{current_case_row['case_id']}`",
            f"- config_path: `{current_case_row['config_path']}`",
            "",
            "## KF-GINS Result Row",
            f"- case_id: `{kf_case_metrics['case_id']}`",
            f"- config_path: `{kf_case_metrics['config_path']}`",
            f"- navresult_path: `{kf_case_metrics['navresult_path']}`",
            f"- imu_error_path: `{kf_case_metrics['imu_error_path']}`",
            f"- stdout_path: `{kf_case_metrics['stdout_path']}`",
            "",
            "## Plot Outputs",
        ]
    )
    for key, path in plot_paths.items():
        lines.append(f"- `{key}`: `{path}`")
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_manifest(
    exp_id: str,
    output_dir: Path,
    args: argparse.Namespace,
    source_manifest: dict[str, Any],
    runtime_mode: str,
    filter_stats: dict[str, float],
    rel_paths: dict[str, str],
    plot_paths: dict[str, str],
    comparison_case_path: Path,
    comparison_phase_path: Path,
) -> dict[str, Any]:
    return {
        "exp_id": exp_id,
        "output_dir": rel_from_root(output_dir, REPO_ROOT),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_manifest": rel_from_root(Path(args.source_manifest), REPO_ROOT),
        "source_case_config": rel_from_root(Path(args.source_config), REPO_ROOT),
        "source_exp_id": source_manifest.get("exp_id"),
        "comparison_scope": "shared_subset_ins_gnss_only",
        "runtime_mode": runtime_mode,
        "kf_gins_exe": rel_from_root(Path(args.kf_gins_exe), REPO_ROOT),
        "fallback_root": str(Path(args.fallback_root)),
        "gnss_filter_stats": json_safe(filter_stats),
        "phase_windows": source_manifest.get("phase_windows"),
        "gnss_on_windows": source_manifest.get("gnss_on_windows"),
        "gnss_off_windows": source_manifest.get("gnss_off_windows"),
        "artifacts": rel_paths,
        "comparison_case_metrics_path": rel_from_root(comparison_case_path, REPO_ROOT),
        "comparison_phase_metrics_path": rel_from_root(comparison_phase_path, REPO_ROOT),
        "plot_paths": plot_paths,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare current data2 INS/GNSS outage result against KF-GINS.")
    parser.add_argument("--source-manifest", type=str, default=str(SOURCE_MANIFEST_DEFAULT))
    parser.add_argument("--source-config", type=str, default=str(SOURCE_CONFIG_DEFAULT))
    parser.add_argument("--kf-gins-exe", type=str, default=str(KF_GINS_EXE_DEFAULT))
    parser.add_argument(
        "--output-dir",
        type=str,
        default=str(REPO_ROOT / "output" / "data2_kf_gins_outage_compare_r1"),
    )
    parser.add_argument("--fallback-root", type=str, default=str(FALLBACK_ROOT_DEFAULT))
    parser.add_argument(
        "--exp-id",
        type=str,
        default=f"EXP-{dt.date.today().strftime('%Y%m%d')}-data2-kf-gins-outage-compare-r1",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    source_manifest = json.loads(Path(args.source_manifest).read_text(encoding="utf-8"))
    source_cfg = load_yaml(Path(args.source_config))
    current_case_df = pd.read_csv(REPO_ROOT / source_manifest["case_metrics_path"])
    current_phase_df = load_current_phase_metrics(REPO_ROOT / source_manifest["phase_metrics_path"])
    current_case_row = current_case_df.iloc[0]

    output_dir = Path(args.output_dir).resolve()
    artifacts_dir = output_dir / "artifacts"
    plots_dir = output_dir / "plots"
    generated_dir = artifacts_dir / "generated"
    runtime_repo_dir = artifacts_dir / "runtime_output"
    ensure_dir(generated_dir)
    ensure_dir(plots_dir)
    ensure_dir(runtime_repo_dir)

    gnss_on_windows = source_manifest.get("gnss_on_windows") or extract_on_windows(source_cfg)
    phase_windows = source_manifest.get("phase_windows") or {
        "phase1": [528076.0, 528276.0],
        "phase2": [528276.0, 528776.0],
        "phase3": [528776.0, float(source_cfg["fusion"]["finaltime"])],
    }
    start_t = float(source_cfg["fusion"]["starttime"])
    final_t = float(source_cfg["fusion"]["finaltime"])
    gnss_off_windows = source_manifest.get("gnss_off_windows") or invert_windows(gnss_on_windows, start_t, final_t)

    source_gnss_path = (REPO_ROOT / source_cfg["fusion"]["gnss_path"]).resolve()
    filtered_gnss_path = generated_dir / "rtk_outage_filtered_for_kf_gins.txt"
    filter_stats = filter_gnss_by_windows(
        source_gnss_path,
        filtered_gnss_path,
        [(float(a), float(b)) for a, b in gnss_on_windows],
    )

    canonical_kf_output_dir = runtime_repo_dir
    canonical_kf_config_path = generated_dir / "kf-gins-data2-outage.yaml"
    canonical_kf_config = build_kf_gins_config(
        source_cfg,
        (REPO_ROOT / source_cfg["fusion"]["imu_path"]).resolve(),
        filtered_gnss_path.resolve(),
        canonical_kf_output_dir.resolve(),
    )
    save_yaml(canonical_kf_config, canonical_kf_config_path)

    runtime_root, runtime_mode = resolve_runtime_root(REPO_ROOT, Path(args.fallback_root))
    runtime_paths = build_runtime_paths(
        runtime_mode=runtime_mode,
        runtime_root=runtime_root,
        output_dir=output_dir,
        kf_exe=Path(args.kf_gins_exe),
        canonical_config_path=canonical_kf_config_path,
        canonical_gnss_path=filtered_gnss_path,
        canonical_output_dir=canonical_kf_output_dir,
        canonical_log_path=generated_dir / "kf_gins_stdout.txt",
        source_cfg=source_cfg,
    )
    stdout_text = run_kf_gins(
        runtime_paths["exe_path"],
        runtime_paths["config_path"],
        runtime_paths["stdout_log_path"],
    )
    if runtime_mode == "fallback_ascii":
        copy_runtime_outputs(runtime_paths["output_dir"], runtime_repo_dir)
        copy_file(runtime_paths["stdout_log_path"], generated_dir / "kf_gins_stdout.txt")

    navresult_path = runtime_repo_dir / "KF_GINS_Navresult.nav"
    imu_error_path = runtime_repo_dir / "KF_GINS_IMU_ERR.txt"
    std_path = runtime_repo_dir / "KF_GINS_STD.txt"
    if runtime_mode == "preferred_root":
        navresult_path = runtime_paths["output_dir"] / "KF_GINS_Navresult.nav"
        imu_error_path = runtime_paths["output_dir"] / "KF_GINS_IMU_ERR.txt"
        std_path = runtime_paths["output_dir"] / "KF_GINS_STD.txt"

    truth_path = (REPO_ROOT / source_cfg["fusion"]["pos_path"]).resolve()
    truth_frame = load_truth_nav(truth_path)
    nav_frame = load_kf_gins_navresult(navresult_path)
    imu_err_frame = load_kf_gins_imu_error(imu_error_path)
    err_frame = build_error_frame(nav_frame, truth_frame)

    case_id = "data2_kf_gins_outage_shared_subset"
    phase_window_rows = build_phase_window_rows(
        {key: [float(val[0]), float(val[1])] for key, val in phase_windows.items()},
        [(float(a), float(b)) for a, b in gnss_off_windows],
    )
    kf_phase_rows = compute_window_rows(case_id, err_frame, imu_err_frame, phase_window_rows)
    kf_phase_df = pd.DataFrame(kf_phase_rows)

    rel_paths = {
        "config_path": rel_from_root(canonical_kf_config_path, REPO_ROOT),
        "filtered_gnss_path": rel_from_root(filtered_gnss_path, REPO_ROOT),
        "navresult_path": rel_from_root(navresult_path, REPO_ROOT),
        "imu_error_path": rel_from_root(imu_error_path, REPO_ROOT),
        "std_path": rel_from_root(std_path, REPO_ROOT),
        "stdout_path": rel_from_root(generated_dir / "kf_gins_stdout.txt", REPO_ROOT),
    }
    kf_case_metrics = summarize_case_metrics(
        case_id,
        err_frame,
        imu_err_frame,
        [(float(a), float(b)) for a, b in gnss_off_windows],
        rel_paths,
    )
    kf_case_metrics["runtime_mode"] = runtime_mode
    kf_case_metrics["stdout_bytes"] = len(stdout_text.encode("utf-8", errors="ignore"))

    kf_case_metrics_path = output_dir / "case_metrics.csv"
    kf_phase_metrics_path = output_dir / "phase_metrics.csv"
    comparison_case_path = output_dir / "comparison_case_metrics.csv"
    comparison_phase_path = output_dir / "comparison_phase_metrics.csv"
    manifest_path = output_dir / "manifest.json"
    summary_path = output_dir / "summary.md"

    pd.DataFrame([kf_case_metrics]).to_csv(kf_case_metrics_path, index=False)
    kf_phase_df.to_csv(kf_phase_metrics_path, index=False)

    comparison_case_df = build_comparison_case_frame(current_case_row, kf_case_metrics)
    comparison_phase_df = build_comparison_phase_frame(current_phase_df, kf_phase_df)
    comparison_case_df.to_csv(comparison_case_path, index=False)
    comparison_phase_df.to_csv(comparison_phase_path, index=False)

    current_sol_path = (REPO_ROOT / current_case_row["sol_path"]).resolve()
    plot_paths = {
        "error_3d_compare": rel_from_root(plots_dir / "error_3d_compare.png", REPO_ROOT),
        "outage_rmse_3d_compare": rel_from_root(plots_dir / "outage_rmse_3d_compare.png", REPO_ROOT),
        "outage_final_err_3d_compare": rel_from_root(plots_dir / "outage_final_err_3d_compare.png", REPO_ROOT),
    }
    plot_error_time_compare(
        current_sol_path=current_sol_path,
        kf_err_frame=err_frame,
        truth_path=truth_path,
        phase_windows={key: [float(v[0]), float(v[1])] for key, v in phase_windows.items()},
        output_path=(plots_dir / "error_3d_compare.png"),
    )
    plot_outage_compare(
        current_phase_df=current_phase_df,
        kf_phase_df=kf_phase_df,
        metric_col="rmse_3d_m",
        ylabel="RMSE 3D [m]",
        title="GNSS outage RMSE comparison",
        output_path=(plots_dir / "outage_rmse_3d_compare.png"),
    )
    plot_outage_compare(
        current_phase_df=current_phase_df,
        kf_phase_df=kf_phase_df,
        metric_col="final_err_3d_m",
        ylabel="Final 3D error [m]",
        title="GNSS outage final error comparison",
        output_path=(plots_dir / "outage_final_err_3d_compare.png"),
    )

    write_summary(
        summary_path=summary_path,
        exp_id=args.exp_id,
        output_dir=output_dir,
        source_manifest_rel=rel_from_root(Path(args.source_manifest), REPO_ROOT),
        runtime_mode=runtime_mode,
        current_case_row=current_case_row,
        current_phase_df=current_phase_df,
        kf_case_metrics=kf_case_metrics,
        kf_phase_df=kf_phase_df,
        comparison_case_df=comparison_case_df,
        plot_paths=plot_paths,
    )

    manifest = build_manifest(
        exp_id=args.exp_id,
        output_dir=output_dir,
        args=args,
        source_manifest=source_manifest,
        runtime_mode=runtime_mode,
        filter_stats=filter_stats,
        rel_paths=rel_paths,
        plot_paths=plot_paths,
        comparison_case_path=comparison_case_path,
        comparison_phase_path=comparison_phase_path,
    )
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    phase3_rmse = float(
        kf_phase_df[kf_phase_df["window_name"] == "phase3_periodic_gnss_outage"].iloc[0]["rmse_3d_m"]
    )
    print(f"[kf_gins_compare] exp_id={args.exp_id}")
    print(f"[kf_gins_compare] output_dir={output_dir}")
    print(f"[kf_gins_compare] runtime_mode={runtime_mode}")
    print(
        "[kf_gins_compare] metrics: "
        f"overall_rmse3d={kf_case_metrics['overall_rmse_3d_m_aux']:.6f} m, "
        f"phase3_rmse3d={phase3_rmse:.6f} m"
    )


if __name__ == "__main__":
    main()
