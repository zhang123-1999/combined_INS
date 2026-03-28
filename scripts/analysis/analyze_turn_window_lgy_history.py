from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.analyze_turn_vs_straight_xy_decoupling import assign_window_id
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root
from scripts.analysis.run_data2_state_sanity_matrix import json_safe
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (
    load_effective_gnss_pos_update_df,
    parse_matrix_field,
    parse_vector_field,
)


EXP_ID_DEFAULT = "EXP-20260319-data2-turn-window-lgy-history-r1"
SOURCE_PROBE_DEFAULT = Path("output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_lgy_history_analysis")
DEFAULT_CASE_IDS = [
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref",
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0",
]
FOCUS_WINDOWS = ["turn_window_1", "turn_window_2", "turn_window_3", "turn_window_4", "turn_window_5"]
TURN_WINDOW_3_CASE_DIFF = (
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref",
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0",
)
EPS = 1.0e-12


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Trace pre-window state/covariance history for `k_lgy_y` around the focused turn windows."
    )
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--source-probe", type=Path, default=SOURCE_PROBE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument(
        "--history-lead-s",
        type=float,
        default=20.0,
        help="Seconds of GNSS_POS history to keep before each focus window.",
    )
    parser.add_argument(
        "--case-id",
        action="append",
        default=None,
        help="Case id to analyze. Can be passed multiple times; defaults to the two focused rerun cases.",
    )
    return parser


def nonzero_sign(value: float) -> float:
    if not np.isfinite(value) or abs(value) <= EPS:
        return math.nan
    return 1.0 if value > 0.0 else -1.0


def load_case_meta(source_probe: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    manifest = json.loads((source_probe / "manifest.json").read_text(encoding="utf-8"))
    case_map = {str(row["case_id"]): row for row in manifest.get("case_rows", [])}
    return manifest, case_map


def load_focus_windows(source_probe: Path) -> pd.DataFrame:
    windows_path = source_probe / "turn_windows.csv"
    if not windows_path.exists():
        raise RuntimeError(f"missing turn windows csv: {windows_path}")
    windows_df = pd.read_csv(windows_path)
    focus = windows_df.loc[windows_df["window_id"].isin(FOCUS_WINDOWS)].copy()
    return focus.sort_values(by="center_t").reset_index(drop=True)


def load_source_window_summary(source_probe: Path) -> pd.DataFrame:
    path = source_probe / "turn_window_summary.csv"
    if not path.exists():
        raise RuntimeError(f"missing turn window summary csv: {path}")
    return pd.read_csv(path)


def load_source_breakdown(source_probe: Path) -> pd.DataFrame:
    path = source_probe / "turn_window_update_breakdown.csv"
    if not path.exists():
        raise RuntimeError(f"missing turn window update breakdown csv: {path}")
    return pd.read_csv(path)


def family_name(window_id: str) -> str:
    if window_id in {"turn_window_1", "turn_window_5"}:
        return "positive_family_1_5"
    if window_id == "turn_window_4":
        return "positive_family_4"
    if window_id in {"turn_window_2", "turn_window_3"}:
        return "negative_family_2_3"
    return "other"


def augment_case_history(
    update_df: pd.DataFrame,
    windows_df: pd.DataFrame,
    truth_lever_y: float,
    case_id: str,
    label: str,
    history_lead_s: float,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for window in windows_df.itertuples(index=False):
        start_t = float(window.start_t)
        end_t = float(window.end_t)
        mask = (update_df["gnss_t"] >= (start_t - history_lead_s)) & (update_df["gnss_t"] <= end_t)
        history_df = update_df.loc[mask].copy().sort_values(by="gnss_t").reset_index(drop=True)
        if history_df.empty:
            continue
        history_df["window_id"] = str(window.window_id)
        history_df["window_center_t"] = float(window.center_t)
        history_df["window_start_t"] = start_t
        history_df["window_end_t"] = end_t
        history_df["window_turn_score_deg_s"] = float(window.turn_score_deg_s)
        history_df["window_abs_yaw_rate_deg_s"] = float(window.abs_yaw_rate_deg_s)
        history_df["window_speed_m_s"] = float(window.speed_m_s)
        history_df["family_id"] = family_name(str(window.window_id))
        history_df["case_id"] = case_id
        history_df["label"] = label
        history_df["history_phase"] = np.where(history_df["gnss_t"] < start_t, "pre_window", "in_window")
        history_df["dt_to_window_start_s"] = history_df["gnss_t"] - start_t
        history_df["dt_to_window_center_s"] = history_df["gnss_t"] - float(window.center_t)
        history_df["update_index_in_history"] = np.arange(len(history_df), dtype=int)
        history_df["turn_direction_sign"] = history_df["yaw_rate_deg_s"].apply(nonzero_sign)

        for row in history_df.itertuples(index=False):
            prior_std_pos = parse_vector_field(getattr(row, "prior_std_pos_vec", None), 3)
            prior_std_lever = parse_vector_field(getattr(row, "prior_std_gnss_lever_vec", None), 3)
            prior_cov_pos_lever = parse_matrix_field(getattr(row, "prior_cov_pos_gnss_lever_mat", None), 3, 3)
            prior_cov_lever = parse_matrix_field(getattr(row, "prior_cov_gnss_lever_mat", None), 3, 3)
            h_pos_x = parse_vector_field(getattr(row, "h_pos_x_vec", None), 3)
            h_pos_y = parse_vector_field(getattr(row, "h_pos_y_vec", None), 3)
            h_pos_z = parse_vector_field(getattr(row, "h_pos_z_vec", None), 3)
            h_lever_x = parse_vector_field(getattr(row, "h_gnss_lever_x_vec", None), 3)
            h_lever_y = parse_vector_field(getattr(row, "h_gnss_lever_y_vec", None), 3)
            h_lever_z = parse_vector_field(getattr(row, "h_gnss_lever_z_vec", None), 3)
            raw_k_lgy = parse_vector_field(getattr(row, "raw_k_gnss_lever_y_vec", None), 3)
            post_k_lgy = parse_vector_field(getattr(row, "k_gnss_lever_y_vec", None), 3)
            num_pos = parse_vector_field(getattr(row, "num_gnss_lever_y_from_pos_vec", None), 3)
            num_att = parse_vector_field(getattr(row, "num_gnss_lever_y_from_att_vec", None), 3)
            num_lever = parse_vector_field(getattr(row, "num_gnss_lever_y_from_gnss_lever_vec", None), 3)
            num_lgy = parse_vector_field(getattr(row, "num_gnss_lever_y_from_lgy_vec", None), 3)

            std_px = float(prior_std_pos[0])
            std_py = float(prior_std_pos[1])
            std_pz = float(prior_std_pos[2])
            std_lgy = float(prior_std_lever[1])
            cov_lgy_px = float(prior_cov_pos_lever[0, 1])
            cov_lgy_py = float(prior_cov_pos_lever[1, 1])
            cov_lgy_pz = float(prior_cov_pos_lever[2, 1])
            cov_lgy_lgx = float(prior_cov_lever[1, 0])
            cov_lgy_lgy = float(prior_cov_lever[1, 1])
            cov_lgy_lgz = float(prior_cov_lever[1, 2])
            h_y_pos_x = float(h_pos_x[1])
            h_y_pos_y = float(h_pos_y[1])
            h_y_pos_z = float(h_pos_z[1])
            h_y_lever_x = float(h_lever_x[1])
            h_y_lever_y = float(h_lever_y[1])
            h_y_lever_z = float(h_lever_z[1])
            lever_before_y = float(row.lever_before_y)
            desired_dx_y = float(truth_lever_y - lever_before_y)
            lever_after_y = float(lever_before_y + row.dx_gnss_lever_y)

            rows.append(
                {
                    "case_id": case_id,
                    "label": label,
                    "window_id": str(window.window_id),
                    "family_id": family_name(str(window.window_id)),
                    "window_center_t": float(window.center_t),
                    "window_start_t": start_t,
                    "window_end_t": end_t,
                    "window_turn_score_deg_s": float(window.turn_score_deg_s),
                    "window_abs_yaw_rate_deg_s": float(window.abs_yaw_rate_deg_s),
                    "window_speed_m_s": float(window.speed_m_s),
                    "gnss_t": float(row.gnss_t),
                    "state_t": float(row.state_t),
                    "history_phase": str(row.history_phase),
                    "dt_to_window_start_s": float(row.dt_to_window_start_s),
                    "dt_to_window_center_s": float(row.dt_to_window_center_s),
                    "update_index_in_history": int(row.update_index_in_history),
                    "yaw_rate_deg_s": float(row.yaw_rate_deg_s),
                    "turn_direction_sign": float(nonzero_sign(float(row.yaw_rate_deg_s))),
                    "y_y": float(row.y_y),
                    "lever_before_y": lever_before_y,
                    "lever_after_y": lever_after_y,
                    "desired_dx_y": desired_dx_y,
                    "dx_gnss_lever_y": float(row.dx_gnss_lever_y),
                    "raw_k_lgy_y": float(raw_k_lgy[1]),
                    "post_k_lgy_y": float(post_k_lgy[1]),
                    "std_py_m": std_py,
                    "std_lgy_m": std_lgy,
                    "cov_lgy_px": cov_lgy_px,
                    "cov_lgy_py": cov_lgy_py,
                    "cov_lgy_pz": cov_lgy_pz,
                    "corr_lgy_px": float(cov_lgy_px / (std_px * std_lgy)) if std_px > 0.0 and std_lgy > 0.0 else math.nan,
                    "corr_lgy_py": float(cov_lgy_py / (std_py * std_lgy)) if std_py > 0.0 and std_lgy > 0.0 else math.nan,
                    "corr_lgy_pz": float(cov_lgy_pz / (std_pz * std_lgy)) if std_pz > 0.0 and std_lgy > 0.0 else math.nan,
                    "cov_lgy_lgx": cov_lgy_lgx,
                    "cov_lgy_lgy": cov_lgy_lgy,
                    "cov_lgy_lgz": cov_lgy_lgz,
                    "h_y_pos_x": h_y_pos_x,
                    "h_y_pos_y": h_y_pos_y,
                    "h_y_pos_z": h_y_pos_z,
                    "h_y_lever_x": h_y_lever_x,
                    "h_y_lever_y": h_y_lever_y,
                    "h_y_lever_z": h_y_lever_z,
                    "num_y_pos_total": float(num_pos[1]),
                    "num_y_att_total": float(num_att[1]),
                    "num_y_lever_total": float(num_lever[1]),
                    "num_y_lgy_self_total": float(num_lgy[1]),
                    "num_y_pos_from_px": float(cov_lgy_px * h_y_pos_x),
                    "num_y_pos_from_py": float(cov_lgy_py * h_y_pos_y),
                    "num_y_pos_from_pz": float(cov_lgy_pz * h_y_pos_z),
                    "num_y_lever_from_lgx": float(cov_lgy_lgx * h_y_lever_x),
                    "num_y_lever_from_lgy": float(cov_lgy_lgy * h_y_lever_y),
                    "num_y_lever_from_lgz": float(cov_lgy_lgz * h_y_lever_z),
                }
            )
    return pd.DataFrame(rows).sort_values(by=["case_id", "window_center_t", "gnss_t"]).reset_index(drop=True)


def summarize_window_history(
    history_df: pd.DataFrame,
    source_window_summary_df: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    summary_lookup = source_window_summary_df.set_index(["case_id", "window_id"])
    for (case_id, label, window_id, family_id), group in history_df.groupby(
        ["case_id", "label", "window_id", "family_id"], sort=False
    ):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        pre_df = group.loc[group["history_phase"] == "pre_window"].copy()
        in_df = group.loc[group["history_phase"] == "in_window"].copy()
        if in_df.empty:
            continue
        entry = in_df.iloc[0]
        last_in = in_df.iloc[-1]
        last_pre = pre_df.iloc[-1] if not pre_df.empty else entry
        first_pre = pre_df.iloc[0] if not pre_df.empty else entry
        source_row = summary_lookup.loc[(case_id, window_id)] if (case_id, window_id) in summary_lookup.index else None

        rows.append(
            {
                "case_id": str(case_id),
                "label": str(label),
                "window_id": str(window_id),
                "family_id": str(family_id),
                "turn_direction_sign": float(np.nanmean(in_df["turn_direction_sign"].to_numpy(dtype=float))),
                "history_updates": int(len(group)),
                "pre_updates": int(len(pre_df)),
                "in_updates": int(len(in_df)),
                "entry_gnss_t": float(entry["gnss_t"]),
                "entry_yaw_rate_deg_s": float(entry["yaw_rate_deg_s"]),
                "entry_y_y": float(entry["y_y"]),
                "entry_lever_before_y": float(entry["lever_before_y"]),
                "entry_desired_dx_y": float(entry["desired_dx_y"]),
                "entry_dx_gnss_lever_y": float(entry["dx_gnss_lever_y"]),
                "entry_raw_k_lgy_y": float(entry["raw_k_lgy_y"]),
                "entry_post_k_lgy_y": float(entry["post_k_lgy_y"]),
                "entry_cov_lgy_py": float(entry["cov_lgy_py"]),
                "entry_corr_lgy_py": float(entry["corr_lgy_py"]),
                "entry_cov_lgy_lgy": float(entry["cov_lgy_lgy"]),
                "entry_h_y_lever_y": float(entry["h_y_lever_y"]),
                "entry_num_y_pos_total": float(entry["num_y_pos_total"]),
                "entry_num_y_pos_from_py": float(entry["num_y_pos_from_py"]),
                "entry_num_y_lever_total": float(entry["num_y_lever_total"]),
                "entry_num_y_lever_from_lgy": float(entry["num_y_lever_from_lgy"]),
                "last_in_cov_lgy_py": float(last_in["cov_lgy_py"]),
                "last_in_h_y_lever_y": float(last_in["h_y_lever_y"]),
                "last_in_raw_k_lgy_y": float(last_in["raw_k_lgy_y"]),
                "pre_first_lever_before_y": float(first_pre["lever_before_y"]),
                "pre_last_lever_before_y": float(last_pre["lever_before_y"]),
                "pre_delta_lever_before_y": float(last_pre["lever_before_y"] - first_pre["lever_before_y"]),
                "pre_first_cov_lgy_py": float(first_pre["cov_lgy_py"]),
                "pre_last_cov_lgy_py": float(last_pre["cov_lgy_py"]),
                "pre_delta_cov_lgy_py": float(last_pre["cov_lgy_py"] - first_pre["cov_lgy_py"]),
                "pre_first_cov_lgy_lgy": float(first_pre["cov_lgy_lgy"]),
                "pre_last_cov_lgy_lgy": float(last_pre["cov_lgy_lgy"]),
                "pre_delta_cov_lgy_lgy": float(last_pre["cov_lgy_lgy"] - first_pre["cov_lgy_lgy"]),
                "pre_first_h_y_lever_y": float(first_pre["h_y_lever_y"]),
                "pre_last_h_y_lever_y": float(last_pre["h_y_lever_y"]),
                "pre_delta_h_y_lever_y": float(last_pre["h_y_lever_y"] - first_pre["h_y_lever_y"]),
                "pre_mean_cov_lgy_py": float(np.nanmean(pre_df["cov_lgy_py"].to_numpy(dtype=float))) if not pre_df.empty else math.nan,
                "pre_mean_corr_lgy_py": float(np.nanmean(pre_df["corr_lgy_py"].to_numpy(dtype=float))) if not pre_df.empty else math.nan,
                "pre_mean_cov_lgy_lgy": float(np.nanmean(pre_df["cov_lgy_lgy"].to_numpy(dtype=float))) if not pre_df.empty else math.nan,
                "pre_mean_h_y_lever_y": float(np.nanmean(pre_df["h_y_lever_y"].to_numpy(dtype=float))) if not pre_df.empty else math.nan,
                "pre_mean_raw_k_lgy_y": float(np.nanmean(pre_df["raw_k_lgy_y"].to_numpy(dtype=float))) if not pre_df.empty else math.nan,
                "pre_cov_lgy_py_positive_rate": float(
                    np.nanmean((pre_df["cov_lgy_py"].to_numpy(dtype=float) > 0.0).astype(float))
                )
                if not pre_df.empty
                else math.nan,
                "pre_h_y_lever_y_positive_rate": float(
                    np.nanmean((pre_df["h_y_lever_y"].to_numpy(dtype=float) > 0.0).astype(float))
                )
                if not pre_df.empty
                else math.nan,
                "pre_raw_k_lgy_y_positive_rate": float(
                    np.nanmean((pre_df["raw_k_lgy_y"].to_numpy(dtype=float) > 0.0).astype(float))
                )
                if not pre_df.empty
                else math.nan,
                "in_cov_lgy_py_positive_rate": float(
                    np.nanmean((in_df["cov_lgy_py"].to_numpy(dtype=float) > 0.0).astype(float))
                ),
                "in_h_y_lever_y_positive_rate": float(
                    np.nanmean((in_df["h_y_lever_y"].to_numpy(dtype=float) > 0.0).astype(float))
                ),
                "in_raw_k_lgy_y_positive_rate": float(
                    np.nanmean((in_df["raw_k_lgy_y"].to_numpy(dtype=float) > 0.0).astype(float))
                ),
                "window_lever_y_error_improve_m": float(source_row["lever_y_error_improve_m"])
                if source_row is not None
                else math.nan,
                "window_mean_k_lever_y_norm": float(source_row["mean_k_lever_y_norm"])
                if source_row is not None
                else math.nan,
            }
        )
    return pd.DataFrame(rows).sort_values(by=["case_id", "window_id"]).reset_index(drop=True)


def build_family_summary(window_summary_df: pd.DataFrame, case_id: str) -> pd.DataFrame:
    ref_df = window_summary_df.loc[window_summary_df["case_id"] == case_id].copy()
    if ref_df.empty:
        return pd.DataFrame()
    rows: list[dict[str, Any]] = []
    for family_id, group in ref_df.groupby("family_id", sort=False):
        rows.append(
            {
                "case_id": case_id,
                "family_id": str(family_id),
                "windows": ",".join(group["window_id"].astype(str).tolist()),
                "turn_direction_sign": float(np.nanmean(group["turn_direction_sign"].to_numpy(dtype=float))),
                "entry_cov_lgy_py_mean": float(np.nanmean(group["entry_cov_lgy_py"].to_numpy(dtype=float))),
                "entry_cov_lgy_lgy_mean": float(np.nanmean(group["entry_cov_lgy_lgy"].to_numpy(dtype=float))),
                "entry_h_y_lever_y_mean": float(np.nanmean(group["entry_h_y_lever_y"].to_numpy(dtype=float))),
                "entry_num_y_pos_total_mean": float(np.nanmean(group["entry_num_y_pos_total"].to_numpy(dtype=float))),
                "entry_num_y_lever_total_mean": float(np.nanmean(group["entry_num_y_lever_total"].to_numpy(dtype=float))),
                "window_lever_y_error_improve_mean_m": float(
                    np.nanmean(group["window_lever_y_error_improve_m"].to_numpy(dtype=float))
                ),
                "pre_cov_lgy_py_positive_rate_mean": float(
                    np.nanmean(group["pre_cov_lgy_py_positive_rate"].to_numpy(dtype=float))
                ),
                "pre_h_y_lever_y_positive_rate_mean": float(
                    np.nanmean(group["pre_h_y_lever_y_positive_rate"].to_numpy(dtype=float))
                ),
                "in_cov_lgy_py_positive_rate_mean": float(
                    np.nanmean(group["in_cov_lgy_py_positive_rate"].to_numpy(dtype=float))
                ),
                "in_h_y_lever_y_positive_rate_mean": float(
                    np.nanmean(group["in_h_y_lever_y_positive_rate"].to_numpy(dtype=float))
                ),
            }
        )
    return pd.DataFrame(rows).sort_values(by="family_id").reset_index(drop=True)


def build_turn_window_3_case_diff(history_df: pd.DataFrame) -> pd.DataFrame:
    window_id = "turn_window_3"
    case_a, case_b = TURN_WINDOW_3_CASE_DIFF
    a_df = history_df.loc[(history_df["case_id"] == case_a) & (history_df["window_id"] == window_id)].copy()
    b_df = history_df.loc[(history_df["case_id"] == case_b) & (history_df["window_id"] == window_id)].copy()
    if a_df.empty or b_df.empty:
        return pd.DataFrame()

    merge_cols = [
        "window_id",
        "history_phase",
        "dt_to_window_start_s",
        "dt_to_window_center_s",
        "yaw_rate_deg_s",
        "turn_direction_sign",
    ]
    value_cols = [
        "lever_before_y",
        "desired_dx_y",
        "dx_gnss_lever_y",
        "raw_k_lgy_y",
        "post_k_lgy_y",
        "cov_lgy_py",
        "corr_lgy_py",
        "cov_lgy_lgy",
        "h_y_lever_y",
        "num_y_pos_total",
        "num_y_pos_from_py",
        "num_y_lever_total",
        "num_y_lever_from_lgy",
        "y_y",
    ]
    merged = a_df[["gnss_t", *merge_cols, *value_cols]].merge(
        b_df[["gnss_t", *value_cols]],
        on="gnss_t",
        how="inner",
        suffixes=("_ref", "_lgy0"),
    )
    for col in value_cols:
        merged[f"delta_{col}"] = merged[f"{col}_lgy0"] - merged[f"{col}_ref"]
    return merged.sort_values(by="gnss_t").reset_index(drop=True)


def write_summary(
    summary_path: Path,
    window_summary_df: pd.DataFrame,
    family_summary_df: pd.DataFrame,
    turn_window_3_case_diff_df: pd.DataFrame,
) -> None:
    ref_case_id = TURN_WINDOW_3_CASE_DIFF[0]
    ref_windows = window_summary_df.loc[window_summary_df["case_id"] == ref_case_id].copy()
    ref_windows = ref_windows.sort_values(by="window_id").reset_index(drop=True)

    lines: list[str] = []
    lines.append("# turn-window lgy history analysis")
    lines.append("")
    lines.append("## Core reading")
    lines.append("- `cov_lgy_py` is taken from `prior_cov_pos_gnss_lever_mat[py,lgy]`, so for `GNSS_POS` measurement-`y` it is also the direct `P(l_gy,p_y)` path because `H_y(pos_y)=1` and `H_y(pos_x/z)=0`.")
    lines.append("- `cov_lgy_lgy * h_y_lever_y` is the dominant same-axis self block observed in the previous lever-axis focused rerun; here the question is whether its sign is set by covariance history or by the geometry term.")
    lines.append("- `turn_window_3` case diff compares the current best combined reference against `positive-turn lgy<-y = 0`. Since window 3 is not a positive turn, any difference at its entry already means the degradation was inherited from earlier history.")
    lines.append("")
    lines.append("## Reference Window Entries")
    for row in ref_windows.itertuples(index=False):
        lines.append(
            f"- `{row.window_id}` (`family={row.family_id}`, `turn_sign={row.turn_direction_sign:+.0f}`, "
            f"`entry_yaw={row.entry_yaw_rate_deg_s:.3f}` deg/s): "
            f"`entry cov(lgy,py)={row.entry_cov_lgy_py:.6f}`, `entry cov(lgy,lgy)={row.entry_cov_lgy_lgy:.6f}`, "
            f"`entry h_y_lever_y={row.entry_h_y_lever_y:.6f}`, "
            f"`last_in cov(lgy,py)/h_y_lever_y={row.last_in_cov_lgy_py:.6f}/{row.last_in_h_y_lever_y:.6f}`, "
            f"`entry num_y(pos/lever)={row.entry_num_y_pos_total:.6f}/{row.entry_num_y_lever_total:.6f}`, "
            f"`pre last cov(lgy,py)={row.pre_last_cov_lgy_py:.6f}`, `pre last h_y_lever_y={row.pre_last_h_y_lever_y:.6f}`, "
            f"`entry raw/post k_lgy_y={row.entry_raw_k_lgy_y:.6f}/{row.entry_post_k_lgy_y:.6f}`, "
            f"`in h_y>0 rate={row.in_h_y_lever_y_positive_rate:.2f}`, "
            f"`window lever_y_error_improve={row.window_lever_y_error_improve_m * 1.0e3:+.3f} mm`."
        )

    if not family_summary_df.empty:
        lines.append("")
        lines.append("## Family Summary")
        for row in family_summary_df.itertuples(index=False):
            lines.append(
                f"- `{row.family_id}` (`windows={row.windows}`, `turn_sign={row.turn_direction_sign:+.0f}`): "
                f"`entry cov(lgy,py) mean={row.entry_cov_lgy_py_mean:.6f}`, "
                f"`entry h_y_lever_y mean={row.entry_h_y_lever_y_mean:.6f}`, "
                f"`entry num_y(pos/lever) mean={row.entry_num_y_pos_total_mean:.6f}/{row.entry_num_y_lever_total_mean:.6f}`, "
                f"`pre cov(lgy,py)>0 rate={row.pre_cov_lgy_py_positive_rate_mean:.2f}`, "
                f"`pre h_y_lever_y>0 rate={row.pre_h_y_lever_y_positive_rate_mean:.2f}`, "
                f"`in cov(lgy,py)>0 rate={row.in_cov_lgy_py_positive_rate_mean:.2f}`, "
                f"`in h_y_lever_y>0 rate={row.in_h_y_lever_y_positive_rate_mean:.2f}`, "
                f"`mean lever_y_error_improve={row.window_lever_y_error_improve_mean_m * 1.0e3:+.3f} mm`."
            )

    if not turn_window_3_case_diff_df.empty:
        pre_df = turn_window_3_case_diff_df.loc[turn_window_3_case_diff_df["history_phase"] == "pre_window"].copy()
        in_df = turn_window_3_case_diff_df.loc[turn_window_3_case_diff_df["history_phase"] == "in_window"].copy()
        last_pre = pre_df.iloc[-1] if not pre_df.empty else in_df.iloc[0]
        entry = in_df.iloc[0]
        lines.append("")
        lines.append("## turn_window_3 Case Diff")
        lines.append(
            f"- Last pre-window update (`dt={last_pre.dt_to_window_start_s:.3f}` s): "
            f"`delta lever_before_y={last_pre.delta_lever_before_y * 1.0e3:+.3f} mm`, "
            f"`delta cov(lgy,py)={last_pre.delta_cov_lgy_py:.6f}`, "
            f"`delta cov(lgy,lgy)={last_pre.delta_cov_lgy_lgy:.6f}`, "
            f"`delta h_y_lever_y={last_pre.delta_h_y_lever_y:.6f}`, "
            f"`delta raw_k_lgy_y={last_pre.delta_raw_k_lgy_y:.6f}`."
        )
        lines.append(
            f"- Entry update (`dt={entry.dt_to_window_start_s:.3f}` s): "
            f"`delta lever_before_y={entry.delta_lever_before_y * 1.0e3:+.3f} mm`, "
            f"`delta desired_dx_y={entry.delta_desired_dx_y * 1.0e3:+.3f} mm`, "
            f"`delta cov(lgy,py)={entry.delta_cov_lgy_py:.6f}`, "
            f"`delta cov(lgy,lgy)={entry.delta_cov_lgy_lgy:.6f}`, "
            f"`delta h_y_lever_y={entry.delta_h_y_lever_y:.6f}`, "
            f"`delta raw/post k_lgy_y={entry.delta_raw_k_lgy_y:.6f}/{entry.delta_post_k_lgy_y:.6f}`, "
            f"`delta dx_gnss_lever_y={entry.delta_dx_gnss_lever_y * 1.0e3:+.3f} mm`."
        )
        lines.append(
            f"- Pre-window max abs delta: "
            f"`lever_before_y={pre_df['delta_lever_before_y'].abs().max() * 1.0e3:.3f} mm`, "
            f"`cov(lgy,py)={pre_df['delta_cov_lgy_py'].abs().max():.6f}`, "
            f"`cov(lgy,lgy)={pre_df['delta_cov_lgy_lgy'].abs().max():.6f}`, "
            f"`h_y_lever_y={pre_df['delta_h_y_lever_y'].abs().max():.6f}`, "
            f"`raw_k_lgy_y={pre_df['delta_raw_k_lgy_y'].abs().max():.6f}`."
        )

    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probe = (REPO_ROOT / args.source_probe).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    case_ids = args.case_id if args.case_id else DEFAULT_CASE_IDS

    if not source_probe.exists():
        raise RuntimeError(f"missing source probe dir: {source_probe}")

    ensure_dir(output_dir)
    manifest, case_meta_map = load_case_meta(source_probe)
    windows_df = load_focus_windows(source_probe)
    source_window_summary_df = load_source_window_summary(source_probe)
    source_breakdown_df = load_source_breakdown(source_probe)
    truth_lever_internal_m = manifest.get("truth_lever_internal_m")
    if not isinstance(truth_lever_internal_m, list) or len(truth_lever_internal_m) < 2:
        raise RuntimeError("manifest missing truth_lever_internal_m")
    truth_lever_y = float(truth_lever_internal_m[1])

    all_history_rows: list[pd.DataFrame] = []
    for case_id in case_ids:
        case_meta = case_meta_map.get(case_id)
        if case_meta is None:
            raise RuntimeError(f"missing case meta for {case_id}")
        raw_path = source_probe / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv"
        update_df = load_effective_gnss_pos_update_df(raw_path)
        update_df = update_df.loc[update_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)
        meta_df = source_breakdown_df.loc[source_breakdown_df["case_id"] == case_id, ["gnss_t", "yaw_rate_deg_s", "speed_m_s"]].copy()
        if meta_df.empty:
            raise RuntimeError(f"missing turn-window breakdown rows for {case_id}")
        update_df = update_df.merge(meta_df, on="gnss_t", how="left")
        if update_df["yaw_rate_deg_s"].isna().any():
            raise RuntimeError(f"failed to backfill yaw_rate_deg_s for {case_id}")
        update_df["window_id_from_source"] = assign_window_id(update_df["gnss_t"], windows_df, "window_id")
        case_history_df = augment_case_history(
            update_df=update_df,
            windows_df=windows_df,
            truth_lever_y=truth_lever_y,
            case_id=case_id,
            label=str(case_meta.get("label", case_id)),
            history_lead_s=float(args.history_lead_s),
        )
        if case_history_df.empty:
            raise RuntimeError(f"no history rows found for {case_id}")
        all_history_rows.append(case_history_df)

    history_df = pd.concat(all_history_rows, ignore_index=True)
    window_summary_df = summarize_window_history(history_df, source_window_summary_df)
    family_summary_df = build_family_summary(window_summary_df, TURN_WINDOW_3_CASE_DIFF[0])
    turn_window_3_case_diff_df = build_turn_window_3_case_diff(history_df)

    history_out = output_dir / "lgy_history_per_update.csv"
    window_summary_out = output_dir / "lgy_history_window_summary.csv"
    family_summary_out = output_dir / "lgy_history_family_summary.csv"
    turn_window_3_diff_out = output_dir / "turn_window_3_case_diff.csv"
    summary_out = output_dir / "summary.md"
    manifest_out = output_dir / "manifest.json"

    history_df.to_csv(history_out, index=False, encoding="utf-8-sig")
    window_summary_df.to_csv(window_summary_out, index=False, encoding="utf-8-sig")
    family_summary_df.to_csv(family_summary_out, index=False, encoding="utf-8-sig")
    turn_window_3_case_diff_df.to_csv(turn_window_3_diff_out, index=False, encoding="utf-8-sig")
    write_summary(summary_out, window_summary_df, family_summary_df, turn_window_3_case_diff_df)

    out_manifest = {
        "exp_id": str(args.exp_id),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probe": rel_from_root(source_probe, REPO_ROOT),
        "history_lead_s": float(args.history_lead_s),
        "case_ids": case_ids,
        "focus_windows": FOCUS_WINDOWS,
        "turn_window_3_case_diff": {
            "reference_case_id": TURN_WINDOW_3_CASE_DIFF[0],
            "comparison_case_id": TURN_WINDOW_3_CASE_DIFF[1],
        },
        "artifacts": {
            "history_per_update_csv": rel_from_root(history_out, REPO_ROOT),
            "window_summary_csv": rel_from_root(window_summary_out, REPO_ROOT),
            "family_summary_csv": rel_from_root(family_summary_out, REPO_ROOT),
            "turn_window_3_case_diff_csv": rel_from_root(turn_window_3_diff_out, REPO_ROOT),
            "summary_md": rel_from_root(summary_out, REPO_ROOT),
        },
    }
    manifest_out.write_text(json.dumps(json_safe(out_manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
