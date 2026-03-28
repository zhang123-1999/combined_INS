from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    json_safe,
    reset_directory,
    run_command,
)


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F

EXP_ID_DEFAULT = "EXP-20260318-data2-turn-window-shared-correction-r1"
BASELINE_EXP_ID = "EXP-20260318-data2-ins-gnss-lever-noise-coupling-sweep-r1"
SOURCE_SWEEP_DIR = Path("output/data2_ins_gnss_lever_noise_coupling_sweep")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_shared_correction_probe")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
POS_PATH_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
ROLLING_WINDOW_S = 5.0
TOP_K_WINDOWS = 5
MIN_SPEED_M_S = 3.0
MIN_WINDOW_SEPARATION_S = 30.0
MEAS_TOL_S = 0.05


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    r_scale: float


CASE_SPECS_DEFAULT = [
    CaseSpec(case_id="eskf_rstd1_qlever1", label="baseline_r1_q1", r_scale=1.0),
    CaseSpec(case_id="eskf_rstd0p25_qlever1", label="r0p25_q1", r_scale=0.25),
    CaseSpec(case_id="eskf_rstd2_qlever1", label="r2_q1", r_scale=2.0),
]


def load_pos_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def load_gnss_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "sigma_n", "sigma_e", "sigma_d"],
        engine="python",
    )


def llh_to_ecef_deg(lat_deg: float, lon_deg: float, h_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl = math.sin(lat)
    cl = math.cos(lat)
    so = math.sin(lon)
    co = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    x = (n + h_m) * cl * co
    y = (n + h_m) * cl * so
    z = (n * (1.0 - WGS84_E2) + h_m) * sl
    return np.array([x, y, z], dtype=float)


def ecef_to_llh_single(x: float, y: float, z: float) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(6):
        sl = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
        lat = math.atan2(z + WGS84_E2 * n * sl, p)
    sl = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    h = p / max(1.0e-12, math.cos(lat)) - n
    return lat, lon, h


def rot_ned_to_ecef(lat_rad: float, lon_rad: float) -> np.ndarray:
    sl = math.sin(lat_rad)
    cl = math.cos(lat_rad)
    so = math.sin(lon_rad)
    co = math.cos(lon_rad)
    return np.array(
        [
            [-sl * co, -so, -cl * co],
            [-sl * so, co, -cl * so],
            [cl, 0.0, -sl],
        ],
        dtype=float,
    )


def quat_to_rot(q_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(q_wxyz, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def skew(vec: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vec, dtype=float)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def select_turn_windows(
    pos_df: pd.DataFrame,
    rolling_window_s: float,
    top_k: int,
    min_speed_m_s: float,
    min_separation_s: float,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    t = pos_df["t"].to_numpy(dtype=float)
    yaw_rad = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.rad2deg(np.gradient(yaw_rad, t))
    speed_m_s = np.linalg.norm(pos_df[["vn", "ve"]].to_numpy(dtype=float), axis=1)
    dt_med = float(np.median(np.diff(t)))
    window_samples = max(3, int(round(rolling_window_s / max(dt_med, 1.0e-3))))
    min_periods = max(3, window_samples // 2)
    turn_score_deg_s = (
        pd.Series(np.abs(yaw_rate_deg_s)).rolling(window_samples, center=True, min_periods=min_periods).mean().to_numpy()
    )
    series_df = pd.DataFrame(
        {
            "t": t,
            "speed_m_s": speed_m_s,
            "yaw_rate_deg_s": yaw_rate_deg_s,
            "turn_score_deg_s": turn_score_deg_s,
        }
    )
    valid_idx = np.where(np.isfinite(turn_score_deg_s) & (speed_m_s >= min_speed_m_s))[0]
    order = valid_idx[np.argsort(turn_score_deg_s[valid_idx])[::-1]]
    rows: list[dict[str, Any]] = []
    selected_centers: list[float] = []
    half_window = 0.5 * rolling_window_s
    for idx in order:
        center_t = float(t[idx])
        if any(abs(center_t - prev) < min_separation_s for prev in selected_centers):
            continue
        selected_centers.append(center_t)
        rows.append(
            {
                "window_id": f"turn_window_{len(rows) + 1}",
                "center_t": center_t,
                "start_t": center_t - half_window,
                "end_t": center_t + half_window,
                "turn_score_deg_s": float(turn_score_deg_s[idx]),
                "abs_yaw_rate_deg_s": float(abs(yaw_rate_deg_s[idx])),
                "speed_m_s": float(speed_m_s[idx]),
            }
        )
        if len(rows) >= top_k:
            break
    windows_df = pd.DataFrame(rows).sort_values(by="turn_score_deg_s", ascending=False).reset_index(drop=True)
    return windows_df, series_df


def find_gnss_measurement_row(gnss_df: pd.DataFrame, t_query: float, tol_s: float) -> pd.Series:
    times = gnss_df["t"].to_numpy(dtype=float)
    idx = int(np.searchsorted(times, t_query))
    candidates = [j for j in (idx - 1, idx, idx + 1) if 0 <= j < len(times)]
    best = min(candidates, key=lambda j: abs(float(times[j]) - t_query))
    if abs(float(times[best]) - t_query) > tol_s:
        raise RuntimeError(f"failed to locate GNSS measurement at t={t_query:.9f} within tol={tol_s}")
    return gnss_df.iloc[best]


def value_share(numerator: float, terms: list[float]) -> float:
    denom = float(sum(abs(term) for term in terms))
    if denom <= 0.0:
        return math.nan
    return abs(float(numerator)) / denom


def norm_share(numerator: np.ndarray, terms: list[np.ndarray]) -> float:
    denom = float(sum(np.linalg.norm(term) for term in terms))
    if denom <= 0.0:
        return math.nan
    return float(np.linalg.norm(numerator) / denom)


def parse_vector_field(value: Any, expected_len: int) -> np.ndarray:
    if not isinstance(value, str):
        return np.full(expected_len, math.nan, dtype=float)
    text = value.strip()
    if not (text.startswith("[") and text.endswith("]")):
        return np.full(expected_len, math.nan, dtype=float)
    body = text[1:-1].strip()
    if not body:
        return np.full(expected_len, math.nan, dtype=float)
    parts = [part for part in body.split(";") if part]
    if len(parts) != expected_len:
        return np.full(expected_len, math.nan, dtype=float)
    try:
        return np.array([float(part) for part in parts], dtype=float)
    except ValueError:
        return np.full(expected_len, math.nan, dtype=float)


def parse_matrix_field(value: Any, rows: int, cols: int) -> np.ndarray:
    return parse_vector_field(value, rows * cols).reshape(rows, cols)


def format_vector_field(vec: np.ndarray) -> str:
    arr = np.asarray(vec, dtype=float).reshape(-1)
    return "[" + ";".join(f"{value:.12g}" for value in arr) + "]"


def load_effective_gnss_pos_update_df(path: Path) -> pd.DataFrame:
    raw_df = pd.read_csv(path)
    if raw_df.empty or "tag" not in raw_df.columns:
        return raw_df

    pos_mask = raw_df["tag"].astype(str).str.contains("GNSS_POS", regex=False, na=False)
    pos_df = raw_df.loc[pos_mask].copy()
    if pos_df.empty:
        return pos_df
    if (pos_df["tag"] == "GNSS_POS").all():
        return pos_df.sort_values(by="gnss_t").reset_index(drop=True)

    pos_df["_raw_order"] = np.arange(len(pos_df), dtype=int)
    dx_cols = [
        "dx_pos_x",
        "dx_pos_y",
        "dx_pos_z",
        "dx_att_x",
        "dx_att_y",
        "dx_att_z",
        "dx_ba_x",
        "dx_ba_y",
        "dx_ba_z",
        "dx_bg_x",
        "dx_bg_y",
        "dx_bg_z",
        "dx_sg_x",
        "dx_sg_y",
        "dx_sg_z",
        "dx_gnss_lever_x",
        "dx_gnss_lever_y",
        "dx_gnss_lever_z",
    ]
    k_vec_cols = [
        "k_pos_x_vec",
        "k_pos_y_vec",
        "k_pos_z_vec",
        "k_att_x_vec",
        "k_att_y_vec",
        "k_att_z_vec",
        "k_gnss_lever_x_vec",
        "k_gnss_lever_y_vec",
        "k_gnss_lever_z_vec",
    ]

    rows: list[pd.Series] = []
    for _, group in pos_df.groupby("gnss_t", sort=False):
        group = group.sort_values(by=["_raw_order", "state_t"]).reset_index(drop=True)
        row = group.iloc[0].copy()
        row["tag"] = "GNSS_POS"
        for col in dx_cols:
            if col in group.columns:
                row[col] = float(group[col].sum())
        for col in k_vec_cols:
            if col in group.columns:
                vec_sum = np.zeros(3, dtype=float)
                any_valid = False
                for value in group[col]:
                    parsed = parse_vector_field(value, 3)
                    if np.isfinite(parsed).all():
                        vec_sum += parsed
                        any_valid = True
                if any_valid:
                    row[col] = format_vector_field(vec_sum)
        rows.append(row.drop(labels="_raw_order"))
    return pd.DataFrame(rows).sort_values(by="gnss_t").reset_index(drop=True)


def safe_corr(cov: float, std_a: float, std_b: float) -> float:
    denom = float(std_a) * float(std_b)
    if denom <= 0.0 or not np.isfinite(denom):
        return math.nan
    return float(cov / denom)


def build_case_config_from_source(spec: CaseSpec, source_cfg_path: Path, case_dir: Path) -> tuple[dict[str, Any], Path]:
    cfg = copy.deepcopy(load_yaml(source_cfg_path))
    fusion = cfg.setdefault("fusion", {})
    fusion["output_path"] = rel_from_root(case_dir / f"SOL_{spec.case_id}.txt", REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(case_dir / f"state_series_{spec.case_id}.csv", REPO_ROOT)
    fusion["first_update_debug_output_path"] = rel_from_root(
        case_dir / f"first_update_{spec.case_id}.csv", REPO_ROOT
    )
    fusion["gnss_update_debug_output_path"] = rel_from_root(
        case_dir / f"gnss_updates_{spec.case_id}.csv", REPO_ROOT
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg, cfg_path


def run_case(spec: CaseSpec, source_cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    cfg, cfg_path = build_case_config_from_source(spec, source_cfg_path, case_dir)
    stdout_path = case_dir / f"{spec.case_id}.stdout.txt"
    diag_path = case_dir / f"DIAG_{spec.case_id}.txt"
    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    first_update_path = case_dir / f"first_update_{spec.case_id}.csv"
    gnss_update_path = case_dir / f"gnss_updates_{spec.case_id}.csv"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    for required_path in [sol_path, state_series_path, first_update_path, gnss_update_path]:
        if not required_path.exists():
            raise RuntimeError(f"missing solver artifact for {spec.case_id}: {required_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after {spec.case_id}")
    diag_path.write_text(root_diag.read_text(encoding="utf-8"), encoding="utf-8")
    return {
        "case_id": spec.case_id,
        "label": spec.label,
        "r_scale": float(spec.r_scale),
        "source_config_path": rel_from_root(source_cfg_path, REPO_ROOT),
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "first_update_path": rel_from_root(first_update_path, REPO_ROOT),
        "gnss_update_path": rel_from_root(gnss_update_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "gnss_path": str(cfg["fusion"]["gnss_path"]),
        "config_mtime": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "first_update_mtime": dt.datetime.fromtimestamp(first_update_path.stat().st_mtime).isoformat(timespec="seconds"),
        "gnss_update_mtime": dt.datetime.fromtimestamp(gnss_update_path.stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(diag_path.stat().st_mtime).isoformat(timespec="seconds"),
    }


def build_diag_window_std_summary(case_rows: list[dict[str, Any]], windows_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_meta in case_rows:
        cfg_path = (REPO_ROOT / case_meta["config_path"]).resolve()
        diag_path = (REPO_ROOT / case_meta["diag_path"]).resolve()
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        start_time = float(cfg["fusion"]["starttime"])
        diag_df = pd.read_csv(diag_path, sep=r"\s+")
        for window in windows_df.itertuples(index=False):
            start_rel = float(window.start_t) - start_time
            end_rel = float(window.end_t) - start_time
            seg = diag_df.loc[(diag_df["t"] >= start_rel) & (diag_df["t"] <= end_rel)].copy()
            rows.append(
                {
                    "case_id": str(case_meta["case_id"]),
                    "label": str(case_meta["label"]),
                    "r_scale": float(case_meta["r_scale"]),
                    "window_id": str(window.window_id),
                    "samples": int(len(seg)),
                    "mean_std_px_m": float(seg["std_px"].mean()),
                    "mean_std_py_m": float(seg["std_py"].mean()),
                    "mean_std_pz_m": float(seg["std_pz"].mean()),
                    "mean_std_phiN_deg": float(seg["std_phiN"].mean() * 57.29577951308232),
                    "mean_std_phiE_deg": float(seg["std_phiE"].mean() * 57.29577951308232),
                    "mean_std_phiD_deg": float(seg["std_phiD"].mean() * 57.29577951308232),
                    "mean_std_lgx_m": float(seg["std_lgx"].mean()),
                    "mean_std_lgy_m": float(seg["std_lgy"].mean()),
                    "mean_std_lgz_m": float(seg["std_lgz"].mean()),
                    "min_std_lgz_m": float(seg["std_lgz"].min()),
                    "max_std_lgz_m": float(seg["std_lgz"].max()),
                }
            )
    return pd.DataFrame(rows)


def write_cause_judgement_summary(
    output_path: Path, case_summary_df: pd.DataFrame, diag_window_std_df: pd.DataFrame
) -> None:
    std_case = (
        diag_window_std_df.groupby(["case_id", "label", "r_scale"], as_index=False)
        .agg(
            mean_std_pz_m=("mean_std_pz_m", "mean"),
            mean_std_lgz_m=("mean_std_lgz_m", "mean"),
            mean_std_phiD_deg=("mean_std_phiD_deg", "mean"),
        )
        .sort_values(by="r_scale")
        .reset_index(drop=True)
    )
    merged = case_summary_df.merge(std_case, on=["case_id", "label", "r_scale"], how="left")
    lines: list[str] = []
    lines.append("# data2 turn-window cause judgement")
    lines.append("")
    lines.append("## Core judgement")
    lines.append("- Current evidence does not support `no turning excitation`.")
    lines.append("- Current evidence also does not support `GNSS lever covariance already collapsed so the state cannot move`.")
    lines.append(
        "- The stronger explanation is: turning excitation exists, but GNSS_POS innovation is still routed primarily into "
        "`position + attitude`, so lever updates do not accumulate toward truth across epochs."
    )
    lines.append("")
    lines.append("## Quantitative evidence")
    for row in merged.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}`: mean `std_pz={row.mean_std_pz_m:.6f} m`, `std_lgz={row.mean_std_lgz_m:.6f} m`, "
            f"`heading_std={row.mean_std_phiD_deg:.6f} deg`; mean `h_lever_z={row.mean_h_lever_z_norm:.6f}`, "
            f"`h_att_z={row.mean_h_att_z_norm:.6f}`, `pos_share_z={row.mean_pos_share_z:.6f}`, "
            f"`att_share_z={row.mean_att_share_z:.6f}`, `lever_share_z={row.mean_lever_share_z:.6f}`; "
            f"`prior_std_pz={row.mean_prior_std_pz_m:.6f} m`, `prior_std_lgz={row.mean_prior_std_lgz_m:.6f} m`, "
            f"`corr_pz_lgz={row.mean_prior_corr_pz_lgz:.6f}`, `k_pos_z_norm={row.mean_k_pos_z_norm:.6f}`, "
            f"`k_att_z_norm={row.mean_k_att_z_norm:.6f}`, `k_lever_z_norm={row.mean_k_lever_z_norm:.6f}`; "
            f"`lever_z_error_improve={row.mean_lever_z_error_improve_m:.6f} m`."
        )
    lines.append("")
    lines.append("## Interpretation boundary")
    lines.append(
        "- These numbers are sufficient to reject the two extreme explanations: `no excitation at all` and "
        "`lever covariance already frozen to near zero`."
    )
    lines.append(
        "- If the refreshed logs keep showing `prior_std_pz ~ prior_std_lgz` but `k_pos_z_norm >> k_lever_z_norm`, "
        "then the dominant issue is prior covariance routing / state competition, not instantaneous geometry magnitude."
    )
    lines.append(
        "- They are not yet sufficient to isolate whether the deeper implementation issue is mainly "
        "`geometry sensitivity too weak after mapping`, `cross-covariance evolution`, or `reset/correction semantics`; "
        "that still needs the planned per-epoch `pred_ant_ecef -> innovation -> dx` cross-check."
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def assign_turn_window(gnss_t: float, windows_df: pd.DataFrame) -> tuple[str | None, float | None]:
    for row in windows_df.itertuples(index=False):
        if float(row.start_t) <= gnss_t <= float(row.end_t):
            return str(row.window_id), float(row.turn_score_deg_s)
    return None, None


def analyze_case_updates(
    case_meta: dict[str, Any],
    windows_df: pd.DataFrame,
    turn_series_df: pd.DataFrame,
    truth_lever: np.ndarray,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    gnss_update_path = (REPO_ROOT / case_meta["gnss_update_path"]).resolve()
    cfg_path = (REPO_ROOT / case_meta["config_path"]).resolve()
    gnss_cfg = load_yaml(cfg_path)
    gnss_path = (REPO_ROOT / str(gnss_cfg["fusion"]["gnss_path"])).resolve()
    gnss_df = load_gnss_dataframe(gnss_path)
    update_df = load_effective_gnss_pos_update_df(gnss_update_path)
    update_df = update_df.loc[update_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)

    interp_t = turn_series_df["t"].to_numpy(dtype=float)
    interp_turn_score = turn_series_df["turn_score_deg_s"].to_numpy(dtype=float)
    interp_yaw_rate = turn_series_df["yaw_rate_deg_s"].to_numpy(dtype=float)
    interp_speed = turn_series_df["speed_m_s"].to_numpy(dtype=float)

    rows: list[dict[str, Any]] = []
    for row in update_df.itertuples(index=False):
        gnss_t = float(row.gnss_t)
        p_before = np.array([row.state_p_before_x, row.state_p_before_y, row.state_p_before_z], dtype=float)
        q_before = np.array(
            [row.state_q_before_w, row.state_q_before_x, row.state_q_before_y, row.state_q_before_z],
            dtype=float,
        )
        lever_before = np.array([row.lever_before_x, row.lever_before_y, row.lever_before_z], dtype=float)
        dx_pos = np.array([row.dx_pos_x, row.dx_pos_y, row.dx_pos_z], dtype=float)
        dx_att = np.array([row.dx_att_x, row.dx_att_y, row.dx_att_z], dtype=float)
        dx_bg = np.array([row.dx_bg_x, row.dx_bg_y, row.dx_bg_z], dtype=float)
        dx_sg = np.array([row.dx_sg_x, row.dx_sg_y, row.dx_sg_z], dtype=float)
        dx_lever = np.array([row.dx_gnss_lever_x, row.dx_gnss_lever_y, row.dx_gnss_lever_z], dtype=float)
        innovation = np.array([row.y_x, row.y_y, row.y_z], dtype=float)
        lever_after = lever_before + dx_lever

        lat_rad, lon_rad, _ = ecef_to_llh_single(*p_before)
        r_ne = rot_ned_to_ecef(lat_rad, lon_rad)
        c_be = quat_to_rot(q_before)
        c_bn = r_ne.T @ c_be
        lever_ned = c_bn @ lever_before
        h_att = skew(lever_ned)
        h_lever = c_bn
        meas_pos = dx_pos
        meas_att = skew(lever_ned) @ dx_att
        meas_lever = c_bn @ dx_lever
        meas_direct_total = meas_pos + meas_att + meas_lever
        prior_std_pos = parse_vector_field(getattr(row, "prior_std_pos_vec", None), 3)
        prior_std_att = parse_vector_field(getattr(row, "prior_std_att_vec", None), 3)
        prior_std_lever = parse_vector_field(getattr(row, "prior_std_gnss_lever_vec", None), 3)
        prior_cov_pos_att = parse_matrix_field(getattr(row, "prior_cov_pos_att_mat", None), 3, 3)
        prior_cov_pos_lever = parse_matrix_field(
            getattr(row, "prior_cov_pos_gnss_lever_mat", None), 3, 3
        )
        prior_cov_att_lever = parse_matrix_field(
            getattr(row, "prior_cov_att_gnss_lever_mat", None), 3, 3
        )
        k_pos_x = parse_vector_field(getattr(row, "k_pos_x_vec", None), 3)
        k_pos_y = parse_vector_field(getattr(row, "k_pos_y_vec", None), 3)
        k_pos_z = parse_vector_field(getattr(row, "k_pos_z_vec", None), 3)
        k_att_x = parse_vector_field(getattr(row, "k_att_x_vec", None), 3)
        k_att_y = parse_vector_field(getattr(row, "k_att_y_vec", None), 3)
        k_att_z = parse_vector_field(getattr(row, "k_att_z_vec", None), 3)
        k_lever_x = parse_vector_field(getattr(row, "k_gnss_lever_x_vec", None), 3)
        k_lever_y = parse_vector_field(getattr(row, "k_gnss_lever_y_vec", None), 3)
        k_lever_z = parse_vector_field(getattr(row, "k_gnss_lever_z_vec", None), 3)

        gnss_meas = find_gnss_measurement_row(gnss_df, gnss_t, MEAS_TOL_S)
        z_ecef = llh_to_ecef_deg(float(gnss_meas["lat"]), float(gnss_meas["lon"]), float(gnss_meas["h"]))
        pred_ant_ecef = p_before + c_be @ lever_before
        innovation_reconstructed = -r_ne.T @ (pred_ant_ecef - z_ecef)

        window_id, window_turn_score = assign_turn_window(gnss_t, windows_df)
        rows.append(
            {
                "case_id": case_meta["case_id"],
                "label": case_meta["label"],
                "r_scale": float(case_meta["r_scale"]),
                "gnss_t": gnss_t,
                "state_t": float(row.state_t),
                "window_id": window_id,
                "window_turn_score_deg_s": window_turn_score,
                "turn_score_deg_s": float(np.interp(gnss_t, interp_t, interp_turn_score)),
                "yaw_rate_deg_s": float(np.interp(gnss_t, interp_t, interp_yaw_rate)),
                "speed_m_s": float(np.interp(gnss_t, interp_t, interp_speed)),
                "y_x": float(innovation[0]),
                "y_y": float(innovation[1]),
                "y_z": float(innovation[2]),
                "y_norm": float(np.linalg.norm(innovation)),
                "pred_ant_ecef_x": float(pred_ant_ecef[0]),
                "pred_ant_ecef_y": float(pred_ant_ecef[1]),
                "pred_ant_ecef_z": float(pred_ant_ecef[2]),
                "meas_ecef_x": float(z_ecef[0]),
                "meas_ecef_y": float(z_ecef[1]),
                "meas_ecef_z": float(z_ecef[2]),
                "innovation_recon_x": float(innovation_reconstructed[0]),
                "innovation_recon_y": float(innovation_reconstructed[1]),
                "innovation_recon_z": float(innovation_reconstructed[2]),
                "innovation_recon_error_norm": float(np.linalg.norm(innovation_reconstructed - innovation)),
                "meas_pos_x": float(meas_pos[0]),
                "meas_pos_y": float(meas_pos[1]),
                "meas_pos_z": float(meas_pos[2]),
                "meas_att_x": float(meas_att[0]),
                "meas_att_y": float(meas_att[1]),
                "meas_att_z": float(meas_att[2]),
                "meas_lever_x": float(meas_lever[0]),
                "meas_lever_y": float(meas_lever[1]),
                "meas_lever_z": float(meas_lever[2]),
                "h_att_x_norm": float(np.linalg.norm(h_att[0, :])),
                "h_att_y_norm": float(np.linalg.norm(h_att[1, :])),
                "h_att_z_norm": float(np.linalg.norm(h_att[2, :])),
                "h_lever_x_norm": float(np.linalg.norm(h_lever[0, :])),
                "h_lever_y_norm": float(np.linalg.norm(h_lever[1, :])),
                "h_lever_z_norm": float(np.linalg.norm(h_lever[2, :])),
                "meas_direct_total_x": float(meas_direct_total[0]),
                "meas_direct_total_y": float(meas_direct_total[1]),
                "meas_direct_total_z": float(meas_direct_total[2]),
                "pos_share_x": value_share(meas_pos[0], [meas_pos[0], meas_att[0], meas_lever[0]]),
                "att_share_x": value_share(meas_att[0], [meas_pos[0], meas_att[0], meas_lever[0]]),
                "lever_share_x": value_share(meas_lever[0], [meas_pos[0], meas_att[0], meas_lever[0]]),
                "pos_share_y": value_share(meas_pos[1], [meas_pos[1], meas_att[1], meas_lever[1]]),
                "att_share_y": value_share(meas_att[1], [meas_pos[1], meas_att[1], meas_lever[1]]),
                "lever_share_y": value_share(meas_lever[1], [meas_pos[1], meas_att[1], meas_lever[1]]),
                "pos_share_z": value_share(meas_pos[2], [meas_pos[2], meas_att[2], meas_lever[2]]),
                "att_share_z": value_share(meas_att[2], [meas_pos[2], meas_att[2], meas_lever[2]]),
                "lever_share_z": value_share(meas_lever[2], [meas_pos[2], meas_att[2], meas_lever[2]]),
                "lever_norm_share": norm_share(meas_lever, [meas_pos, meas_att, meas_lever]),
                "dx_bg_norm": float(np.linalg.norm(dx_bg)),
                "dx_sg_norm": float(np.linalg.norm(dx_sg)),
                "dx_lever_norm": float(np.linalg.norm(dx_lever)),
                "prior_std_px_m": float(prior_std_pos[0]),
                "prior_std_py_m": float(prior_std_pos[1]),
                "prior_std_pz_m": float(prior_std_pos[2]),
                "prior_std_attx_rad": float(prior_std_att[0]),
                "prior_std_atty_rad": float(prior_std_att[1]),
                "prior_std_attz_rad": float(prior_std_att[2]),
                "prior_std_lgx_m": float(prior_std_lever[0]),
                "prior_std_lgy_m": float(prior_std_lever[1]),
                "prior_std_lgz_m": float(prior_std_lever[2]),
                "prior_corr_px_attx": safe_corr(prior_cov_pos_att[0, 0], prior_std_pos[0], prior_std_att[0]),
                "prior_corr_py_atty": safe_corr(prior_cov_pos_att[1, 1], prior_std_pos[1], prior_std_att[1]),
                "prior_corr_pz_attz": safe_corr(prior_cov_pos_att[2, 2], prior_std_pos[2], prior_std_att[2]),
                "prior_corr_px_lgx": safe_corr(prior_cov_pos_lever[0, 0], prior_std_pos[0], prior_std_lever[0]),
                "prior_corr_py_lgy": safe_corr(prior_cov_pos_lever[1, 1], prior_std_pos[1], prior_std_lever[1]),
                "prior_corr_pz_lgz": safe_corr(prior_cov_pos_lever[2, 2], prior_std_pos[2], prior_std_lever[2]),
                "prior_corr_attx_lgx": safe_corr(
                    prior_cov_att_lever[0, 0], prior_std_att[0], prior_std_lever[0]
                ),
                "prior_corr_atty_lgy": safe_corr(
                    prior_cov_att_lever[1, 1], prior_std_att[1], prior_std_lever[1]
                ),
                "prior_corr_attz_lgz": safe_corr(
                    prior_cov_att_lever[2, 2], prior_std_att[2], prior_std_lever[2]
                ),
                "k_pos_x_norm": float(np.linalg.norm(k_pos_x)),
                "k_pos_y_norm": float(np.linalg.norm(k_pos_y)),
                "k_pos_z_norm": float(np.linalg.norm(k_pos_z)),
                "k_att_x_norm": float(np.linalg.norm(k_att_x)),
                "k_att_y_norm": float(np.linalg.norm(k_att_y)),
                "k_att_z_norm": float(np.linalg.norm(k_att_z)),
                "k_lever_x_norm": float(np.linalg.norm(k_lever_x)),
                "k_lever_y_norm": float(np.linalg.norm(k_lever_y)),
                "k_lever_z_norm": float(np.linalg.norm(k_lever_z)),
                "lever_before_x": float(lever_before[0]),
                "lever_before_y": float(lever_before[1]),
                "lever_before_z": float(lever_before[2]),
                "lever_after_x": float(lever_after[0]),
                "lever_after_y": float(lever_after[1]),
                "lever_after_z": float(lever_after[2]),
                "lever_error_norm_before": float(np.linalg.norm(lever_before - truth_lever)),
                "lever_error_norm_after": float(np.linalg.norm(lever_after - truth_lever)),
                "lever_error_x_before": float(abs(lever_before[0] - truth_lever[0])),
                "lever_error_x_after": float(abs(lever_after[0] - truth_lever[0])),
                "lever_error_y_before": float(abs(lever_before[1] - truth_lever[1])),
                "lever_error_y_after": float(abs(lever_after[1] - truth_lever[1])),
                "lever_error_z_before": float(abs(lever_before[2] - truth_lever[2])),
                "lever_error_z_after": float(abs(lever_after[2] - truth_lever[2])),
            }
        )

    per_update_df = pd.DataFrame(rows)
    turn_updates_df = per_update_df.loc[per_update_df["window_id"].notna()].copy()
    if turn_updates_df.empty:
        raise RuntimeError(f"no GNSS_POS updates found inside selected turn windows for {case_meta['case_id']}")

    window_rows: list[dict[str, Any]] = []
    for window_id, group in turn_updates_df.groupby("window_id", sort=False):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        window_info = windows_df.loc[windows_df["window_id"] == window_id].iloc[0]
        first_row = group.iloc[0]
        last_row = group.iloc[-1]
        window_rows.append(
            {
                "case_id": case_meta["case_id"],
                "label": case_meta["label"],
                "r_scale": float(case_meta["r_scale"]),
                "window_id": window_id,
                "center_t": float(window_info["center_t"]),
                "start_t": float(window_info["start_t"]),
                "end_t": float(window_info["end_t"]),
                "turn_score_deg_s": float(window_info["turn_score_deg_s"]),
                "abs_yaw_rate_deg_s": float(window_info["abs_yaw_rate_deg_s"]),
                "updates": int(len(group)),
                "mean_y_norm": float(group["y_norm"].mean()),
                "mean_abs_y_x": float(group["y_x"].abs().mean()),
                "mean_abs_y_y": float(group["y_y"].abs().mean()),
                "mean_abs_y_z": float(group["y_z"].abs().mean()),
                "mean_abs_meas_pos_x": float(group["meas_pos_x"].abs().mean()),
                "mean_abs_meas_pos_y": float(group["meas_pos_y"].abs().mean()),
                "mean_abs_meas_att_x": float(group["meas_att_x"].abs().mean()),
                "mean_abs_meas_att_y": float(group["meas_att_y"].abs().mean()),
                "mean_abs_meas_lever_x": float(group["meas_lever_x"].abs().mean()),
                "mean_abs_meas_lever_y": float(group["meas_lever_y"].abs().mean()),
                "mean_abs_meas_pos_z": float(group["meas_pos_z"].abs().mean()),
                "mean_abs_meas_att_z": float(group["meas_att_z"].abs().mean()),
                "mean_abs_meas_lever_z": float(group["meas_lever_z"].abs().mean()),
                "mean_h_att_x_norm": float(group["h_att_x_norm"].mean()),
                "mean_h_att_y_norm": float(group["h_att_y_norm"].mean()),
                "mean_h_att_z_norm": float(group["h_att_z_norm"].mean()),
                "mean_h_lever_x_norm": float(group["h_lever_x_norm"].mean()),
                "mean_h_lever_y_norm": float(group["h_lever_y_norm"].mean()),
                "mean_h_lever_z_norm": float(group["h_lever_z_norm"].mean()),
                "mean_pos_share_x": float(group["pos_share_x"].mean()),
                "mean_att_share_x": float(group["att_share_x"].mean()),
                "mean_lever_share_x": float(group["lever_share_x"].mean()),
                "mean_pos_share_y": float(group["pos_share_y"].mean()),
                "mean_att_share_y": float(group["att_share_y"].mean()),
                "mean_lever_share_y": float(group["lever_share_y"].mean()),
                "mean_pos_share_z": float(group["pos_share_z"].mean()),
                "mean_att_share_z": float(group["att_share_z"].mean()),
                "mean_lever_share_z": float(group["lever_share_z"].mean()),
                "mean_lever_norm_share": float(group["lever_norm_share"].mean()),
                "mean_dx_bg_norm": float(group["dx_bg_norm"].mean()),
                "mean_dx_sg_norm": float(group["dx_sg_norm"].mean()),
                "mean_dx_lever_norm": float(group["dx_lever_norm"].mean()),
                "mean_prior_std_px_m": float(group["prior_std_px_m"].mean()),
                "mean_prior_std_py_m": float(group["prior_std_py_m"].mean()),
                "mean_prior_std_pz_m": float(group["prior_std_pz_m"].mean()),
                "mean_prior_std_lgx_m": float(group["prior_std_lgx_m"].mean()),
                "mean_prior_std_lgy_m": float(group["prior_std_lgy_m"].mean()),
                "mean_prior_std_lgz_m": float(group["prior_std_lgz_m"].mean()),
                "mean_prior_corr_px_attx": float(group["prior_corr_px_attx"].mean()),
                "mean_prior_corr_py_atty": float(group["prior_corr_py_atty"].mean()),
                "mean_prior_corr_pz_attz": float(group["prior_corr_pz_attz"].mean()),
                "mean_prior_corr_px_lgx": float(group["prior_corr_px_lgx"].mean()),
                "mean_prior_corr_py_lgy": float(group["prior_corr_py_lgy"].mean()),
                "mean_prior_corr_pz_lgz": float(group["prior_corr_pz_lgz"].mean()),
                "mean_prior_corr_attx_lgx": float(group["prior_corr_attx_lgx"].mean()),
                "mean_prior_corr_atty_lgy": float(group["prior_corr_atty_lgy"].mean()),
                "mean_prior_corr_attz_lgz": float(group["prior_corr_attz_lgz"].mean()),
                "mean_k_pos_x_norm": float(group["k_pos_x_norm"].mean()),
                "mean_k_pos_y_norm": float(group["k_pos_y_norm"].mean()),
                "mean_k_pos_z_norm": float(group["k_pos_z_norm"].mean()),
                "mean_k_att_x_norm": float(group["k_att_x_norm"].mean()),
                "mean_k_att_y_norm": float(group["k_att_y_norm"].mean()),
                "mean_k_att_z_norm": float(group["k_att_z_norm"].mean()),
                "mean_k_lever_x_norm": float(group["k_lever_x_norm"].mean()),
                "mean_k_lever_y_norm": float(group["k_lever_y_norm"].mean()),
                "mean_k_lever_z_norm": float(group["k_lever_z_norm"].mean()),
                "lever_x_error_improve_m": float(first_row["lever_error_x_before"] - last_row["lever_error_x_after"]),
                "lever_y_error_improve_m": float(first_row["lever_error_y_before"] - last_row["lever_error_y_after"]),
                "lever_z_error_improve_m": float(first_row["lever_error_z_before"] - last_row["lever_error_z_after"]),
                "lever_norm_error_improve_m": float(
                    first_row["lever_error_norm_before"] - last_row["lever_error_norm_after"]
                ),
                "innovation_recon_error_norm_max": float(group["innovation_recon_error_norm"].max()),
            }
        )
    window_summary_df = pd.DataFrame(window_rows)

    case_summary_df = (
        window_summary_df.groupby(["case_id", "label", "r_scale"], as_index=False)
        .agg(
            windows=("window_id", "count"),
            updates=("updates", "sum"),
            mean_turn_score_deg_s=("turn_score_deg_s", "mean"),
            mean_abs_y_x=("mean_abs_y_x", "mean"),
            mean_abs_y_y=("mean_abs_y_y", "mean"),
            mean_abs_y_z=("mean_abs_y_z", "mean"),
            mean_h_att_x_norm=("mean_h_att_x_norm", "mean"),
            mean_h_att_y_norm=("mean_h_att_y_norm", "mean"),
            mean_h_att_z_norm=("mean_h_att_z_norm", "mean"),
            mean_h_lever_x_norm=("mean_h_lever_x_norm", "mean"),
            mean_h_lever_y_norm=("mean_h_lever_y_norm", "mean"),
            mean_h_lever_z_norm=("mean_h_lever_z_norm", "mean"),
            mean_pos_share_x=("mean_pos_share_x", "mean"),
            mean_att_share_x=("mean_att_share_x", "mean"),
            mean_lever_share_x=("mean_lever_share_x", "mean"),
            mean_pos_share_y=("mean_pos_share_y", "mean"),
            mean_att_share_y=("mean_att_share_y", "mean"),
            mean_lever_share_y=("mean_lever_share_y", "mean"),
            mean_pos_share_z=("mean_pos_share_z", "mean"),
            mean_att_share_z=("mean_att_share_z", "mean"),
            mean_lever_share_z=("mean_lever_share_z", "mean"),
            mean_lever_norm_share=("mean_lever_norm_share", "mean"),
            mean_dx_bg_norm=("mean_dx_bg_norm", "mean"),
            mean_dx_sg_norm=("mean_dx_sg_norm", "mean"),
            mean_dx_lever_norm=("mean_dx_lever_norm", "mean"),
            mean_prior_std_px_m=("mean_prior_std_px_m", "mean"),
            mean_prior_std_py_m=("mean_prior_std_py_m", "mean"),
            mean_prior_std_pz_m=("mean_prior_std_pz_m", "mean"),
            mean_prior_std_lgx_m=("mean_prior_std_lgx_m", "mean"),
            mean_prior_std_lgy_m=("mean_prior_std_lgy_m", "mean"),
            mean_prior_std_lgz_m=("mean_prior_std_lgz_m", "mean"),
            mean_prior_corr_px_attx=("mean_prior_corr_px_attx", "mean"),
            mean_prior_corr_py_atty=("mean_prior_corr_py_atty", "mean"),
            mean_prior_corr_pz_attz=("mean_prior_corr_pz_attz", "mean"),
            mean_prior_corr_px_lgx=("mean_prior_corr_px_lgx", "mean"),
            mean_prior_corr_py_lgy=("mean_prior_corr_py_lgy", "mean"),
            mean_prior_corr_pz_lgz=("mean_prior_corr_pz_lgz", "mean"),
            mean_prior_corr_attx_lgx=("mean_prior_corr_attx_lgx", "mean"),
            mean_prior_corr_atty_lgy=("mean_prior_corr_atty_lgy", "mean"),
            mean_prior_corr_attz_lgz=("mean_prior_corr_attz_lgz", "mean"),
            mean_k_pos_x_norm=("mean_k_pos_x_norm", "mean"),
            mean_k_pos_y_norm=("mean_k_pos_y_norm", "mean"),
            mean_k_pos_z_norm=("mean_k_pos_z_norm", "mean"),
            mean_k_att_x_norm=("mean_k_att_x_norm", "mean"),
            mean_k_att_y_norm=("mean_k_att_y_norm", "mean"),
            mean_k_att_z_norm=("mean_k_att_z_norm", "mean"),
            mean_k_lever_x_norm=("mean_k_lever_x_norm", "mean"),
            mean_k_lever_y_norm=("mean_k_lever_y_norm", "mean"),
            mean_k_lever_z_norm=("mean_k_lever_z_norm", "mean"),
            mean_lever_x_error_improve_m=("lever_x_error_improve_m", "mean"),
            mean_lever_y_error_improve_m=("lever_y_error_improve_m", "mean"),
            mean_lever_z_error_improve_m=("lever_z_error_improve_m", "mean"),
            mean_lever_norm_error_improve_m=("lever_norm_error_improve_m", "mean"),
            max_innovation_recon_error_norm=("innovation_recon_error_norm_max", "max"),
        )
        .sort_values(by="r_scale")
        .reset_index(drop=True)
    )
    return per_update_df, window_summary_df, case_summary_df


def write_summary(
    summary_path: Path,
    case_summary_df: pd.DataFrame,
    window_summary_df: pd.DataFrame,
    windows_df: pd.DataFrame,
) -> None:
    lines: list[str] = []
    lines.append("# data2 turn-window GNSS shared correction probe")
    lines.append("")
    lines.append("## 1. Selected turn windows")
    for row in windows_df.itertuples(index=False):
        lines.append(
            f"- `{row.window_id}`: center=`{row.center_t:.3f}` s, window=`[{row.start_t:.3f}, {row.end_t:.3f}]` s, "
            f"turn_score=`{row.turn_score_deg_s:.3f}` deg/s, `|yaw_rate|=`{row.abs_yaw_rate_deg_s:.3f}` deg/s, "
            f"speed=`{row.speed_m_s:.3f}` m/s."
        )
    lines.append("")
    lines.append("## 2. Case summary")
    for row in case_summary_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}` (`R x {row.r_scale:.2f}`): mean `h_att_x/y/z=`"
            f"`{row.mean_h_att_x_norm:.4f}/{row.mean_h_att_y_norm:.4f}/{row.mean_h_att_z_norm:.4f}`, "
            f"`h_lever_x/y/z={row.mean_h_lever_x_norm:.4f}/{row.mean_h_lever_y_norm:.4f}/{row.mean_h_lever_z_norm:.4f}`, "
            f"`lever_share_x/y/z={row.mean_lever_share_x:.4f}/{row.mean_lever_share_y:.4f}/{row.mean_lever_share_z:.4f}`, "
            f"`pos_share_x/y/z={row.mean_pos_share_x:.4f}/{row.mean_pos_share_y:.4f}/{row.mean_pos_share_z:.4f}`, "
            f"`k_lever_x/y/z={row.mean_k_lever_x_norm:.4f}/{row.mean_k_lever_y_norm:.4f}/{row.mean_k_lever_z_norm:.4f}`, "
            f"`corr_p(x/y/z,lg)={row.mean_prior_corr_px_lgx:.4f}/{row.mean_prior_corr_py_lgy:.4f}/{row.mean_prior_corr_pz_lgz:.4f}`, "
            f"`lever_z_error_improve={row.mean_lever_z_error_improve_m:.6f}` m, "
            f"`lever_x_error_improve={row.mean_lever_x_error_improve_m:.6f}` m, "
            f"`lever_y_error_improve={row.mean_lever_y_error_improve_m:.6f}` m, "
            f"`dx_bg_norm={row.mean_dx_bg_norm:.6e}`, `dx_sg_norm={row.mean_dx_sg_norm:.6e}`."
        )
    lines.append("")
    strongest_window = window_summary_df.sort_values(by="turn_score_deg_s", ascending=False).iloc[0]
    lines.append("## 3. Representative strongest-window note")
    lines.append(
        f"- Strongest window is `{strongest_window['window_id']}` for `{strongest_window['case_id']}`: "
        f"`turn_score={strongest_window['turn_score_deg_s']:.3f}` deg/s, "
        f"`mean_abs_y_z={strongest_window['mean_abs_y_z']:.6f}` m, "
        f"`mean_lever_share_z={strongest_window['mean_lever_share_z']:.4f}`, "
        f"`mean_pos_share_z={strongest_window['mean_pos_share_z']:.4f}`, "
        f"`mean_att_share_z={strongest_window['mean_att_share_z']:.4f}`, "
        f"`lever_z_error_improve={strongest_window['lever_z_error_improve_m']:.6f}` m."
    )
    lines.append("")
    lines.append("## 4. Interpretation")
    lines.append(
        "- If `lever_share_z` stays far below `pos_share_z + att_share_z` while `lever_z_error_improve` remains near zero, "
        "the turn excitation is not being converted into an effective GNSS lever-arm decoupling."
    )
    lines.append(
        "- In this truth-modeled `INS/GNSS` probe, `dx_bg_norm` is effectively zero and `dx_sg_norm` stays tiny, so the "
        "dominant shared sinks are `position + attitude`, not the bias states."
    )
    lines.append(
        "- The geometry is axis-asymmetric: `h_att_x` stays large because the vertical lever component is large, while "
        "`h_att_z` stays much smaller because it only depends on the horizontal lever components. Even so, `position` "
        "still dominates both axes in the realized correction share."
    )
    lines.append(
        "- If `prior_std_p*` and `prior_std_lg*` stay comparable but `k_pos_*` still dominates `k_lever_*`, the core "
        "failure mode is that the filter's prior covariance keeps routing GNSS information into the directly observed "
        "`position` state instead of letting turn-induced multi-epoch information accumulate on `gnss_lever`."
    )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe GNSS_POS shared correction inside high-turn windows.")
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--source-sweep-dir", type=Path, default=SOURCE_SWEEP_DIR)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    source_sweep_dir = (REPO_ROOT / args.source_sweep_dir).resolve()
    exe_path = (REPO_ROOT / args.exe).resolve()
    base_cfg_path = (REPO_ROOT / args.base_config).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()

    if not exe_path.exists():
        raise RuntimeError(f"missing executable: {exe_path}")
    if not source_sweep_dir.exists():
        raise RuntimeError(f"missing source sweep dir: {source_sweep_dir}")

    reset_directory(output_dir)
    artifacts_dir = output_dir / "artifacts" / "cases"
    ensure_dir(artifacts_dir)

    base_cfg = load_yaml(base_cfg_path)
    truth_reference = build_truth_reference(base_cfg)
    truth_lever = np.array(
        [
            float(truth_reference["states"]["gnss_lever_x"]["reference_value_internal"]),
            float(truth_reference["states"]["gnss_lever_y"]["reference_value_internal"]),
            float(truth_reference["states"]["gnss_lever_z"]["reference_value_internal"]),
        ],
        dtype=float,
    )

    pos_df = load_pos_dataframe(pos_path)
    windows_df, turn_series_df = select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=ROLLING_WINDOW_S,
        top_k=TOP_K_WINDOWS,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    windows_path = output_dir / "turn_windows.csv"
    windows_df.to_csv(windows_path, index=False, encoding="utf-8-sig")

    case_rows: list[dict[str, Any]] = []
    all_updates: list[pd.DataFrame] = []
    all_window_summaries: list[pd.DataFrame] = []
    all_case_summaries: list[pd.DataFrame] = []

    for spec in CASE_SPECS_DEFAULT:
        source_cfg_path = source_sweep_dir / "artifacts" / "cases" / spec.case_id / f"config_{spec.case_id}.yaml"
        if not source_cfg_path.exists():
            raise RuntimeError(f"missing source config for {spec.case_id}: {source_cfg_path}")
        case_dir = artifacts_dir / spec.case_id
        ensure_dir(case_dir)
        case_meta = run_case(spec, source_cfg_path, case_dir, exe_path)
        case_rows.append(case_meta)
        per_update_df, window_summary_df, case_summary_df = analyze_case_updates(
            case_meta=case_meta,
            windows_df=windows_df,
            turn_series_df=turn_series_df,
            truth_lever=truth_lever,
        )
        all_updates.append(per_update_df)
        all_window_summaries.append(window_summary_df)
        all_case_summaries.append(case_summary_df)

    update_summary_df = pd.concat(all_updates, ignore_index=True)
    window_summary_df = pd.concat(all_window_summaries, ignore_index=True)
    case_summary_df = pd.concat(all_case_summaries, ignore_index=True).sort_values(by="r_scale").reset_index(drop=True)

    per_update_path = output_dir / "turn_window_update_breakdown.csv"
    window_summary_path = output_dir / "turn_window_summary.csv"
    case_summary_path = output_dir / "case_update_summary.csv"
    diag_window_std_path = output_dir / "diag_turn_window_std_summary.csv"
    cause_judgement_path = output_dir / "cause_judgement_summary.md"
    update_summary_df.to_csv(per_update_path, index=False, encoding="utf-8-sig")
    window_summary_df.to_csv(window_summary_path, index=False, encoding="utf-8-sig")
    case_summary_df.to_csv(case_summary_path, index=False, encoding="utf-8-sig")
    diag_window_std_df = build_diag_window_std_summary(case_rows, windows_df)
    diag_window_std_df.to_csv(diag_window_std_path, index=False, encoding="utf-8-sig")

    summary_path = output_dir / "summary.md"
    write_summary(summary_path, case_summary_df, window_summary_df, windows_df)
    write_cause_judgement_summary(cause_judgement_path, case_summary_df, diag_window_std_df)

    manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "baseline_exp_id": BASELINE_EXP_ID,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(base_cfg_path, REPO_ROOT),
        "exe": rel_from_root(exe_path, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "source_sweep_dir": rel_from_root(source_sweep_dir, REPO_ROOT),
        "rolling_window_s": ROLLING_WINDOW_S,
        "top_k_windows": TOP_K_WINDOWS,
        "min_speed_m_s": MIN_SPEED_M_S,
        "min_window_separation_s": MIN_WINDOW_SEPARATION_S,
        "truth_lever_internal_m": truth_lever.tolist(),
        "artifacts": {
            "turn_windows_csv": rel_from_root(windows_path, REPO_ROOT),
            "turn_window_update_breakdown_csv": rel_from_root(per_update_path, REPO_ROOT),
            "turn_window_summary_csv": rel_from_root(window_summary_path, REPO_ROOT),
            "case_update_summary_csv": rel_from_root(case_summary_path, REPO_ROOT),
            "diag_turn_window_std_summary_csv": rel_from_root(diag_window_std_path, REPO_ROOT),
            "cause_judgement_summary_md": rel_from_root(cause_judgement_path, REPO_ROOT),
            "summary_md": rel_from_root(summary_path, REPO_ROOT),
        },
        "cases": case_rows,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
