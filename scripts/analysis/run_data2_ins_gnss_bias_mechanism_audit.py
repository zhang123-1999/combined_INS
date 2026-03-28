from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_bias_range_sweep import (
    CONTROL_CASE_ID,
    CaseSpec as BiasCaseSpec,
    build_case_config,
)
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    json_safe,
    reset_directory,
    run_command,
)
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (
    load_effective_gnss_pos_update_df,
    load_pos_dataframe,
    parse_matrix_field,
    parse_vector_field,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-bias-mechanism-audit-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_bias_mechanism_audit")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
POS_PATH_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
EARLY_UPDATE_COUNT = 120
MIN_SPEED_M_S = 3.0
TURN_YAW_RATE_DEG_S = 8.0
STRAIGHT_YAW_RATE_DEG_S = 3.0
MEAS_AXIS_LABELS = ["x", "y", "z"]
BIAS_AXIS_META = [
    ("ba_x", "ba", 0),
    ("ba_y", "ba", 1),
    ("ba_z", "ba", 2),
    ("bg_x", "bg", 0),
    ("bg_y", "bg", 1),
    ("bg_z", "bg", 2),
]
SOURCE_LABELS = ["pos", "att", "lever"]


@dataclass(frozen=True)
class AuditCase:
    case_id: str
    label: str
    spec: BiasCaseSpec | None


CASE_SPECS_DEFAULT = [
    AuditCase(case_id=CONTROL_CASE_ID, label="truth_anchor_bias_control", spec=None),
    AuditCase(
        case_id=BiasCaseSpec(ba_p0_scale=1.0, bg_p0_scale=1.0, ba_q_scale=0.25, bg_q_scale=0.25).case_id,
        label="free_bias_safe_baseline",
        spec=BiasCaseSpec(ba_p0_scale=1.0, bg_p0_scale=1.0, ba_q_scale=0.25, bg_q_scale=0.25),
    ),
    AuditCase(
        case_id=BiasCaseSpec(ba_p0_scale=0.1, bg_p0_scale=1.0, ba_q_scale=0.25, bg_q_scale=0.25).case_id,
        label="free_bias_tight_ba_p0",
        spec=BiasCaseSpec(ba_p0_scale=0.1, bg_p0_scale=1.0, ba_q_scale=0.25, bg_q_scale=0.25),
    ),
]


def safe_corr(cov: float, std_a: float, std_b: float) -> float:
    denom = float(std_a) * float(std_b)
    if denom <= 0.0 or not np.isfinite(denom):
        return math.nan
    return float(cov / denom)


def safe_norm_share(vec: np.ndarray, vecs: list[np.ndarray]) -> float:
    denom = float(sum(np.linalg.norm(item) for item in vecs))
    if denom <= 0.0 or not np.isfinite(denom):
        return math.nan
    return float(np.linalg.norm(vec) / denom)


def classify_motion(speed_m_s: float, yaw_rate_deg_s: float) -> tuple[str, str]:
    abs_yaw = abs(float(yaw_rate_deg_s))
    if speed_m_s < MIN_SPEED_M_S:
        return "low_speed", "unknown"
    if abs_yaw >= TURN_YAW_RATE_DEG_S:
        return "turning", "positive" if yaw_rate_deg_s >= 0.0 else "negative"
    if abs_yaw <= STRAIGHT_YAW_RATE_DEG_S:
        return "straight", "neutral"
    return "transition", "neutral"


def phase_label(update_index: int, total_updates: int) -> str:
    early_count = min(EARLY_UPDATE_COUNT, max(10, total_updates // 5))
    return "early" if update_index < early_count else "later"


def load_motion_profile(path: Path) -> pd.DataFrame:
    pos_df = load_pos_dataframe(path).sort_values(by="t").reset_index(drop=True)
    t = pos_df["t"].to_numpy(dtype=float)
    yaw_rad = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.rad2deg(np.gradient(yaw_rad, t))
    speed_m_s = np.linalg.norm(pos_df[["vn", "ve"]].to_numpy(dtype=float), axis=1)
    motion_df = pos_df[["t"]].copy()
    motion_df["speed_m_s"] = speed_m_s
    motion_df["yaw_rate_deg_s"] = yaw_rate_deg_s
    return motion_df


def sample_motion(motion_df: pd.DataFrame, t_query: float) -> tuple[float, float]:
    t = motion_df["t"].to_numpy(dtype=float)
    speed = np.interp(float(t_query), t, motion_df["speed_m_s"].to_numpy(dtype=float))
    yaw_rate = np.interp(float(t_query), t, motion_df["yaw_rate_deg_s"].to_numpy(dtype=float))
    return float(speed), float(yaw_rate)


def run_case(
    audit_case: AuditCase,
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    exe_path: Path,
) -> dict[str, Any]:
    cfg, overrides = build_case_config(
        base_cfg=base_cfg,
        truth_reference=truth_reference,
        case_dir=case_dir,
        case_id=audit_case.case_id,
        spec=audit_case.spec,
    )
    fusion = cfg.setdefault("fusion", {})
    sol_path = case_dir / f"SOL_{audit_case.case_id}.txt"
    state_series_path = case_dir / f"state_series_{audit_case.case_id}.csv"
    diag_path = case_dir / f"DIAG_{audit_case.case_id}.txt"
    stdout_path = case_dir / f"{audit_case.case_id}.stdout.txt"
    gnss_update_path = case_dir / f"gnss_updates_{audit_case.case_id}.csv"
    first_update_path = case_dir / f"first_update_{audit_case.case_id}.csv"
    cfg_path = case_dir / f"config_{audit_case.case_id}.yaml"
    fusion["first_update_debug_output_path"] = rel_from_root(first_update_path, REPO_ROOT)
    fusion["gnss_update_debug_output_path"] = rel_from_root(gnss_update_path, REPO_ROOT)
    save_yaml(cfg, cfg_path)

    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")

    for required_path in [sol_path, state_series_path, gnss_update_path]:
        if not required_path.exists():
            raise RuntimeError(f"missing solver artifact for {audit_case.case_id}: {required_path}")
    if root_diag.exists():
        shutil.copy2(root_diag, diag_path)

    row: dict[str, Any] = {
        "case_id": audit_case.case_id,
        "label": audit_case.label,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT) if diag_path.exists() else "",
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "gnss_update_path": rel_from_root(gnss_update_path, REPO_ROOT),
        "first_update_path": rel_from_root(first_update_path, REPO_ROOT) if first_update_path.exists() else "",
        "config_mtime": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "gnss_update_mtime": dt.datetime.fromtimestamp(gnss_update_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(overrides)
    row.update(evaluate_navigation_metrics(cfg_path, sol_path))
    return row


def analyze_case_updates(case_meta: dict[str, Any], motion_df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    raw_updates = load_effective_gnss_pos_update_df((REPO_ROOT / case_meta["gnss_update_path"]).resolve())
    if raw_updates.empty:
        raise RuntimeError(f"no GNSS_POS updates found for {case_meta['case_id']}")

    raw_updates = raw_updates.sort_values(by="gnss_t").reset_index(drop=True)
    update_rows: list[dict[str, Any]] = []
    axis_rows: list[dict[str, Any]] = []

    total_updates = int(len(raw_updates))
    for update_index, row in raw_updates.iterrows():
        gnss_t = float(row["gnss_t"])
        speed_m_s, yaw_rate_deg_s = sample_motion(motion_df, gnss_t)
        motion_label, turn_sign = classify_motion(speed_m_s, yaw_rate_deg_s)
        phase = phase_label(update_index, total_updates)
        y_vec = np.array([float(row["y_x"]), float(row["y_y"]), float(row["y_z"])], dtype=float)
        s_mat = parse_matrix_field(row["s_mat"], 3, 3)
        prior_std_pos = parse_vector_field(row["prior_std_pos_vec"], 3)
        prior_std_att = parse_vector_field(row["prior_std_att_vec"], 3)
        prior_std_ba = parse_vector_field(row["prior_std_ba_vec"], 3)
        prior_std_bg = parse_vector_field(row["prior_std_bg_vec"], 3)
        prior_std_gnss_lever = parse_vector_field(row["prior_std_gnss_lever_vec"], 3)

        h_pos = np.column_stack(
            [
                parse_vector_field(row["h_pos_x_vec"], 3),
                parse_vector_field(row["h_pos_y_vec"], 3),
                parse_vector_field(row["h_pos_z_vec"], 3),
            ]
        )
        h_att = np.column_stack(
            [
                parse_vector_field(row["h_att_x_vec"], 3),
                parse_vector_field(row["h_att_y_vec"], 3),
                parse_vector_field(row["h_att_z_vec"], 3),
            ]
        )
        h_lever = np.column_stack(
            [
                parse_vector_field(row["h_gnss_lever_x_vec"], 3),
                parse_vector_field(row["h_gnss_lever_y_vec"], 3),
                parse_vector_field(row["h_gnss_lever_z_vec"], 3),
            ]
        )
        prior_cov = {
            "ba": {
                "pos": parse_matrix_field(row["prior_cov_ba_pos_mat"], 3, 3),
                "att": parse_matrix_field(row["prior_cov_ba_att_mat"], 3, 3),
                "lever": parse_matrix_field(row["prior_cov_ba_gnss_lever_mat"], 3, 3),
                "std": prior_std_ba,
            },
            "bg": {
                "pos": parse_matrix_field(row["prior_cov_bg_pos_mat"], 3, 3),
                "att": parse_matrix_field(row["prior_cov_bg_att_mat"], 3, 3),
                "lever": parse_matrix_field(row["prior_cov_bg_gnss_lever_mat"], 3, 3),
                "std": prior_std_bg,
            },
        }

        dx_ba = np.array([float(row["dx_ba_x"]), float(row["dx_ba_y"]), float(row["dx_ba_z"])], dtype=float)
        dx_bg = np.array([float(row["dx_bg_x"]), float(row["dx_bg_y"]), float(row["dx_bg_z"])], dtype=float)
        dx_pos = np.array([float(row["dx_pos_x"]), float(row["dx_pos_y"]), float(row["dx_pos_z"])], dtype=float)
        dx_att = np.array([float(row["dx_att_x"]), float(row["dx_att_y"]), float(row["dx_att_z"])], dtype=float)

        update_rows.append(
            {
                "case_id": case_meta["case_id"],
                "label": case_meta["label"],
                "update_index": update_index,
                "phase_label": phase,
                "gnss_t": gnss_t,
                "motion_label": motion_label,
                "turn_sign": turn_sign,
                "speed_m_s": speed_m_s,
                "yaw_rate_deg_s": yaw_rate_deg_s,
                "y_x": float(y_vec[0]),
                "y_y": float(y_vec[1]),
                "y_z": float(y_vec[2]),
                "abs_y_x": float(abs(y_vec[0])),
                "abs_y_y": float(abs(y_vec[1])),
                "abs_y_z": float(abs(y_vec[2])),
                "y_norm": float(np.linalg.norm(y_vec)),
                "dx_pos_norm": float(np.linalg.norm(dx_pos)),
                "dx_att_norm": float(np.linalg.norm(dx_att)),
                "dx_ba_norm": float(np.linalg.norm(dx_ba)),
                "dx_bg_norm": float(np.linalg.norm(dx_bg)),
                "prior_std_ba_norm": float(np.linalg.norm(prior_std_ba)),
                "prior_std_bg_norm": float(np.linalg.norm(prior_std_bg)),
            }
        )

        for state_name, group, axis in BIAS_AXIS_META:
            state_std = prior_cov[group]["std"]
            cov_pos = prior_cov[group]["pos"]
            cov_att = prior_cov[group]["att"]
            cov_lever = prior_cov[group]["lever"]
            num_pos = cov_pos[axis, :] @ h_pos.T
            num_att = cov_att[axis, :] @ h_att.T
            num_lever = cov_lever[axis, :] @ h_lever.T
            num_total = num_pos + num_att + num_lever
            k_row = parse_vector_field(row[f"k_{state_name}_vec"], 3)
            dx_from_meas = k_row * y_vec
            num_from_gain = s_mat @ k_row
            source_norms = {
                "pos": float(np.linalg.norm(num_pos)),
                "att": float(np.linalg.norm(num_att)),
                "lever": float(np.linalg.norm(num_lever)),
            }
            dominant_source = max(source_norms.items(), key=lambda item: item[1])[0]
            dominant_meas_axis = MEAS_AXIS_LABELS[int(np.argmax(np.abs(k_row)))]
            dominant_dx_meas_axis = MEAS_AXIS_LABELS[int(np.argmax(np.abs(dx_from_meas)))]
            num_terms = {
                "pos_x": float(num_pos[0]),
                "pos_y": float(num_pos[1]),
                "pos_z": float(num_pos[2]),
                "att_x": float(num_att[0]),
                "att_y": float(num_att[1]),
                "att_z": float(num_att[2]),
                "lever_x": float(num_lever[0]),
                "lever_y": float(num_lever[1]),
                "lever_z": float(num_lever[2]),
            }
            dominant_num_term, dominant_num_term_value = max(
                num_terms.items(), key=lambda item: abs(item[1])
            )
            sum_abs_num_terms = float(sum(abs(value) for value in num_terms.values()))
            dx_value = float(row[f"dx_{state_name}"])
            axis_rows.append(
                {
                    "case_id": case_meta["case_id"],
                    "label": case_meta["label"],
                    "state_name": state_name,
                    "group": group,
                    "axis": axis,
                    "update_index": update_index,
                    "phase_label": phase,
                    "gnss_t": gnss_t,
                    "motion_label": motion_label,
                    "turn_sign": turn_sign,
                    "speed_m_s": speed_m_s,
                    "yaw_rate_deg_s": yaw_rate_deg_s,
                    "y_norm": float(np.linalg.norm(y_vec)),
                    "y_x": float(y_vec[0]),
                    "y_y": float(y_vec[1]),
                    "y_z": float(y_vec[2]),
                    "abs_y_x": float(abs(y_vec[0])),
                    "abs_y_y": float(abs(y_vec[1])),
                    "abs_y_z": float(abs(y_vec[2])),
                    "dx_value": dx_value,
                    "abs_dx_value": abs(dx_value),
                    "k_norm": float(np.linalg.norm(k_row)),
                    "k_x": float(k_row[0]),
                    "k_y": float(k_row[1]),
                    "k_z": float(k_row[2]),
                    "abs_k_x": float(abs(k_row[0])),
                    "abs_k_y": float(abs(k_row[1])),
                    "abs_k_z": float(abs(k_row[2])),
                    "dx_from_meas_x": float(dx_from_meas[0]),
                    "dx_from_meas_y": float(dx_from_meas[1]),
                    "dx_from_meas_z": float(dx_from_meas[2]),
                    "abs_dx_from_meas_x": float(abs(dx_from_meas[0])),
                    "abs_dx_from_meas_y": float(abs(dx_from_meas[1])),
                    "abs_dx_from_meas_z": float(abs(dx_from_meas[2])),
                    "num_total_norm": float(np.linalg.norm(num_total)),
                    "num_pos_norm": float(np.linalg.norm(num_pos)),
                    "num_att_norm": float(np.linalg.norm(num_att)),
                    "num_lever_norm": float(np.linalg.norm(num_lever)),
                    "num_pos_x": float(num_pos[0]),
                    "num_pos_y": float(num_pos[1]),
                    "num_pos_z": float(num_pos[2]),
                    "num_att_x": float(num_att[0]),
                    "num_att_y": float(num_att[1]),
                    "num_att_z": float(num_att[2]),
                    "num_lever_x": float(num_lever[0]),
                    "num_lever_y": float(num_lever[1]),
                    "num_lever_z": float(num_lever[2]),
                    "share_pos": safe_norm_share(num_pos, [num_pos, num_att, num_lever]),
                    "share_att": safe_norm_share(num_att, [num_pos, num_att, num_lever]),
                    "share_lever": safe_norm_share(num_lever, [num_pos, num_att, num_lever]),
                    "dominant_source": dominant_source,
                    "dominant_meas_axis": dominant_meas_axis,
                    "dominant_dx_meas_axis": dominant_dx_meas_axis,
                    "dominant_num_term": dominant_num_term,
                    "dominant_num_term_share": (
                        float(abs(dominant_num_term_value) / sum_abs_num_terms)
                        if sum_abs_num_terms > 0.0
                        else math.nan
                    ),
                    "num_recon_error_norm": float(np.linalg.norm(num_total - num_from_gain)),
                    "prior_std_state": float(state_std[axis]),
                    "prior_std_pos_same": float(prior_std_pos[axis]),
                    "prior_std_att_same": float(prior_std_att[axis]),
                    "prior_std_lever_same": float(prior_std_gnss_lever[axis]),
                    "prior_corr_state_pos_same": safe_corr(
                        cov_pos[axis, axis], state_std[axis], prior_std_pos[axis]
                    ),
                    "prior_corr_state_att_same": safe_corr(
                        cov_att[axis, axis], state_std[axis], prior_std_att[axis]
                    ),
                    "prior_corr_state_lever_same": safe_corr(
                        cov_lever[axis, axis], state_std[axis], prior_std_gnss_lever[axis]
                    ),
                }
            )

    return pd.DataFrame(update_rows), pd.DataFrame(axis_rows)


def summarize_axis_groups(axis_df: pd.DataFrame) -> pd.DataFrame:
    def dominant_fraction(series: pd.Series, label: str) -> float:
        if series.empty:
            return math.nan
        return float((series.astype(str) == label).mean())

    rows: list[dict[str, Any]] = []
    num_term_cols = [
        "num_pos_x",
        "num_pos_y",
        "num_pos_z",
        "num_att_x",
        "num_att_y",
        "num_att_z",
        "num_lever_x",
        "num_lever_y",
        "num_lever_z",
    ]
    group_cols = ["case_id", "label", "state_name", "phase_label", "motion_label"]
    for keys, group in axis_df.groupby(group_cols, sort=False):
        case_id, label, state_name, phase_label_value, motion_label = keys
        term_means = {col: float(group[col].abs().mean()) for col in num_term_cols}
        ordered_terms = sorted(term_means.items(), key=lambda item: item[1], reverse=True)
        dominant_num_term_mode = group["dominant_num_term"].astype(str).value_counts(normalize=True)
        rows.append(
            {
                "case_id": case_id,
                "label": label,
                "state_name": state_name,
                "phase_label": phase_label_value,
                "motion_label": motion_label,
                "updates": int(len(group)),
                "mean_y_norm": float(group["y_norm"].mean()),
                "mean_abs_y_x": float(group["abs_y_x"].mean()),
                "mean_abs_y_y": float(group["abs_y_y"].mean()),
                "mean_abs_y_z": float(group["abs_y_z"].mean()),
                "mean_abs_dx_value": float(group["abs_dx_value"].mean()),
                "p95_abs_dx_value": float(group["abs_dx_value"].quantile(0.95)),
                "mean_k_norm": float(group["k_norm"].mean()),
                "mean_abs_k_x": float(group["abs_k_x"].mean()),
                "mean_abs_k_y": float(group["abs_k_y"].mean()),
                "mean_abs_k_z": float(group["abs_k_z"].mean()),
                "mean_abs_dx_from_meas_x": float(group["abs_dx_from_meas_x"].mean()),
                "mean_abs_dx_from_meas_y": float(group["abs_dx_from_meas_y"].mean()),
                "mean_abs_dx_from_meas_z": float(group["abs_dx_from_meas_z"].mean()),
                "mean_num_total_norm": float(group["num_total_norm"].mean()),
                "mean_share_pos": float(group["share_pos"].mean()),
                "mean_share_att": float(group["share_att"].mean()),
                "mean_share_lever": float(group["share_lever"].mean()),
                "mean_prior_std_state": float(group["prior_std_state"].mean()),
                "mean_prior_corr_state_pos_same": float(group["prior_corr_state_pos_same"].mean()),
                "mean_prior_corr_state_att_same": float(group["prior_corr_state_att_same"].mean()),
                "mean_prior_corr_state_lever_same": float(group["prior_corr_state_lever_same"].mean()),
                "mean_num_recon_error_norm": float(group["num_recon_error_norm"].mean()),
                "dominant_pos_ratio": dominant_fraction(group["dominant_source"], "pos"),
                "dominant_att_ratio": dominant_fraction(group["dominant_source"], "att"),
                "dominant_lever_ratio": dominant_fraction(group["dominant_source"], "lever"),
                "dominant_meas_x_ratio": dominant_fraction(group["dominant_meas_axis"], "x"),
                "dominant_meas_y_ratio": dominant_fraction(group["dominant_meas_axis"], "y"),
                "dominant_meas_z_ratio": dominant_fraction(group["dominant_meas_axis"], "z"),
                "dominant_dx_meas_x_ratio": dominant_fraction(group["dominant_dx_meas_axis"], "x"),
                "dominant_dx_meas_y_ratio": dominant_fraction(group["dominant_dx_meas_axis"], "y"),
                "dominant_dx_meas_z_ratio": dominant_fraction(group["dominant_dx_meas_axis"], "z"),
                "dominant_num_term_mode": str(dominant_num_term_mode.index[0]) if not dominant_num_term_mode.empty else "",
                "dominant_num_term_mode_ratio": float(dominant_num_term_mode.iloc[0]) if not dominant_num_term_mode.empty else math.nan,
                "top_num_term": ordered_terms[0][0].replace("num_", ""),
                "top_num_term_mean_abs": ordered_terms[0][1],
                "second_num_term": ordered_terms[1][0].replace("num_", ""),
                "second_num_term_mean_abs": ordered_terms[1][1],
                "third_num_term": ordered_terms[2][0].replace("num_", ""),
                "third_num_term_mean_abs": ordered_terms[2][1],
            }
        )
    return pd.DataFrame(rows)


def build_axis_term_summary(axis_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for source in SOURCE_LABELS:
        for meas_axis in MEAS_AXIS_LABELS:
            num_col = f"num_{source}_{meas_axis}"
            dx_col = f"abs_dx_from_meas_{meas_axis}"
            k_col = f"abs_k_{meas_axis}"
            y_col = f"abs_y_{meas_axis}"
            grouped = (
                axis_df.groupby(["case_id", "label", "state_name", "phase_label", "motion_label"], sort=False)
                .agg(
                    updates=("update_index", "count"),
                    mean_num=(num_col, "mean"),
                    mean_abs_num=(num_col, lambda s: float(np.mean(np.abs(s)))),
                    p95_abs_num=(num_col, lambda s: float(np.quantile(np.abs(s), 0.95))),
                    mean_abs_dx_component=(dx_col, "mean"),
                    mean_abs_k_component=(k_col, "mean"),
                    mean_abs_y_component=(y_col, "mean"),
                )
                .reset_index()
            )
            grouped["source"] = source
            grouped["meas_axis"] = meas_axis
            rows.append(grouped)
    term_df = pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()
    if not term_df.empty:
        term_df = term_df[
            [
                "case_id",
                "label",
                "state_name",
                "phase_label",
                "motion_label",
                "source",
                "meas_axis",
                "updates",
                "mean_num",
                "mean_abs_num",
                "p95_abs_num",
                "mean_abs_dx_component",
                "mean_abs_k_component",
                "mean_abs_y_component",
            ]
        ].sort_values(
            by=["case_id", "state_name", "phase_label", "motion_label", "mean_abs_num"],
            ascending=[True, True, True, True, False],
        )
    return term_df.reset_index(drop=True)


def extract_key_examples(axis_df: pd.DataFrame, top_k: int = 5) -> pd.DataFrame:
    rows: list[pd.DataFrame] = []
    for (case_id, state_name), group in axis_df.groupby(["case_id", "state_name"], sort=False):
        top_group = group.sort_values(by="abs_dx_value", ascending=False).head(top_k).copy()
        top_group.insert(0, "rank_in_case_state", np.arange(1, len(top_group) + 1, dtype=int))
        rows.append(top_group)
    return pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()


def write_summary(
    output_path: Path,
    case_df: pd.DataFrame,
    grouped_df: pd.DataFrame,
    term_df: pd.DataFrame,
    axis_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    lines: list[str] = []
    lines.append("# data2 pure INS/GNSS bias mechanism audit")
    lines.append("")
    lines.append("## 1. 审计设置")
    lines.append("- pipeline: `pure INS/GNSS`，固定 `gnss_lever` 真值并锚定 `sg/sa`，只比较 `ba/bg` 更新机理。")
    lines.append("- update chain: 当前实验显式关闭 `ODO/NHC` 且 `enable_gnss_velocity=false`，因此外部量测驱动只剩 `GNSS_POS`。")
    lines.append(
        f"- motion labels: `turning` 定义为 `speed>={MIN_SPEED_M_S:.1f} m/s` 且 `|yaw_rate|>={TURN_YAW_RATE_DEG_S:.1f} deg/s`；"
        f"`straight` 为 `speed>={MIN_SPEED_M_S:.1f} m/s` 且 `|yaw_rate|<={STRAIGHT_YAW_RATE_DEG_S:.1f} deg/s`。"
    )
    lines.append(
        f"- phase split: `early` 为前 `min({EARLY_UPDATE_COUNT}, 20%)` 个 `GNSS_POS` 更新，其余归入 `later`。"
    )
    lines.append("")
    lines.append("## 2. Case navigation reference")
    for row in case_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}` (`{row.label}`): `nav_rmse_3d_m={float(row.nav_rmse_3d_m):.6f}`, "
            f"`nav_final_err_3d_m={float(row.nav_final_err_3d_m):.6f}`."
        )

    baseline_case_id = CASE_SPECS_DEFAULT[1].case_id
    key_axes = ["ba_x", "ba_y", "bg_y"]
    lines.append("")
    lines.append("## 3. Baseline free-bias key-axis mechanism")
    for state_name in key_axes:
        overall = grouped_df[
            (grouped_df["case_id"] == baseline_case_id)
            & (grouped_df["state_name"] == state_name)
        ].copy()
        if overall.empty:
            continue
        overall_mean = (
            overall.groupby("state_name", as_index=False)
            .agg(
                mean_abs_dx_value=("mean_abs_dx_value", "mean"),
                mean_share_pos=("mean_share_pos", "mean"),
                mean_share_att=("mean_share_att", "mean"),
                mean_share_lever=("mean_share_lever", "mean"),
                mean_prior_corr_state_pos_same=("mean_prior_corr_state_pos_same", "mean"),
                mean_prior_corr_state_att_same=("mean_prior_corr_state_att_same", "mean"),
                mean_num_recon_error_norm=("mean_num_recon_error_norm", "mean"),
            )
            .iloc[0]
        )
        lines.append(
            f"- `{state_name}` overall: `mean|dx|={overall_mean['mean_abs_dx_value']:.6e}`, "
            f"`share_pos/att/lever={overall_mean['mean_share_pos']:.3f}/{overall_mean['mean_share_att']:.3f}/{overall_mean['mean_share_lever']:.3f}`, "
            f"`corr_same(pos/att)={overall_mean['mean_prior_corr_state_pos_same']:.3f}/{overall_mean['mean_prior_corr_state_att_same']:.3f}`, "
            f"`recon_err={overall_mean['mean_num_recon_error_norm']:.3e}`."
        )
        for phase_value in ["early", "later"]:
            for motion_value in ["straight", "turning"]:
                sub = overall[(overall["phase_label"] == phase_value) & (overall["motion_label"] == motion_value)]
                if sub.empty:
                    continue
                row = sub.iloc[0]
                lines.append(
                    f"  {phase_value}/{motion_value}: `updates={int(row['updates'])}`, "
                    f"`mean|dx|={row['mean_abs_dx_value']:.6e}`, `mean_k_norm={row['mean_k_norm']:.6e}`, "
                    f"`share_pos/att/lever={row['mean_share_pos']:.3f}/{row['mean_share_att']:.3f}/{row['mean_share_lever']:.3f}`, "
                    f"`dominant_pos/att/lever={row['dominant_pos_ratio']:.2f}/{row['dominant_att_ratio']:.2f}/{row['dominant_lever_ratio']:.2f}`, "
                    f"`dominant_kaxis_x/y/z={row['dominant_meas_x_ratio']:.2f}/{row['dominant_meas_y_ratio']:.2f}/{row['dominant_meas_z_ratio']:.2f}`, "
                    f"`dominant_dxaxis_x/y/z={row['dominant_dx_meas_x_ratio']:.2f}/{row['dominant_dx_meas_y_ratio']:.2f}/{row['dominant_dx_meas_z_ratio']:.2f}`, "
                    f"`top_terms={row['top_num_term']},{row['second_num_term']},{row['third_num_term']}`."
                )

    ba_tight_case_id = CASE_SPECS_DEFAULT[2].case_id
    lines.append("")
    lines.append("## 4. Tight `ba_p0` comparison")
    for state_name in ["ba_x", "ba_y", "ba_z"]:
        base_rows = grouped_df[
            (grouped_df["case_id"] == baseline_case_id)
            & (grouped_df["state_name"] == state_name)
        ]
        tight_rows = grouped_df[
            (grouped_df["case_id"] == ba_tight_case_id)
            & (grouped_df["state_name"] == state_name)
        ]
        if base_rows.empty or tight_rows.empty:
            continue
        base_mean = base_rows["mean_abs_dx_value"].mean()
        tight_mean = tight_rows["mean_abs_dx_value"].mean()
        base_std = base_rows["mean_prior_std_state"].mean()
        tight_std = tight_rows["mean_prior_std_state"].mean()
        base_share_pos = base_rows["mean_share_pos"].mean()
        tight_share_pos = tight_rows["mean_share_pos"].mean()
        base_share_att = base_rows["mean_share_att"].mean()
        tight_share_att = tight_rows["mean_share_att"].mean()
        lines.append(
            f"- `{state_name}`: `mean|dx| {base_mean:.6e} -> {tight_mean:.6e}`, "
            f"`prior_std {base_std:.6e} -> {tight_std:.6e}`, "
            f"`share_pos/att {base_share_pos:.3f}/{base_share_att:.3f} -> {tight_share_pos:.3f}/{tight_share_att:.3f}`."
        )

    lines.append("")
    lines.append("## 5. Term-level contrasts")
    for state_name in ["ba_x", "ba_y"]:
        sub = grouped_df[
            (grouped_df["case_id"] == baseline_case_id)
            & (grouped_df["state_name"] == state_name)
            & (grouped_df["phase_label"].isin(["early", "later"]))
            & (grouped_df["motion_label"].isin(["straight", "turning"]))
        ].copy()
        if sub.empty:
            continue
        straight = sub[(sub["phase_label"] == "later") & (sub["motion_label"] == "straight")]
        turning = sub[(sub["phase_label"] == "early") & (sub["motion_label"] == "turning")]
        if not straight.empty and not turning.empty:
            row_s = straight.iloc[0]
            row_t = turning.iloc[0]
            lines.append(
                f"- `{state_name}` later/straight vs early/turning: "
                f"`top_terms {row_s['top_num_term']},{row_s['second_num_term']} -> {row_t['top_num_term']},{row_t['second_num_term']}`, "
                f"`dxaxis_x/y/z {row_s['dominant_dx_meas_x_ratio']:.2f}/{row_s['dominant_dx_meas_y_ratio']:.2f}/{row_s['dominant_dx_meas_z_ratio']:.2f} -> "
                f"{row_t['dominant_dx_meas_x_ratio']:.2f}/{row_t['dominant_dx_meas_y_ratio']:.2f}/{row_t['dominant_dx_meas_z_ratio']:.2f}`."
            )

    bgy_early = grouped_df[
        (grouped_df["case_id"] == baseline_case_id)
        & (grouped_df["state_name"] == "bg_y")
        & (grouped_df["phase_label"] == "early")
        & (grouped_df["motion_label"] == "straight")
    ]
    bgy_later = grouped_df[
        (grouped_df["case_id"] == baseline_case_id)
        & (grouped_df["state_name"] == "bg_y")
        & (grouped_df["phase_label"] == "later")
        & (grouped_df["motion_label"] == "straight")
    ]
    if not bgy_early.empty and not bgy_later.empty:
        row_e = bgy_early.iloc[0]
        row_l = bgy_later.iloc[0]
        lines.append(
            f"- `bg_y` early/straight -> later/straight: "
            f"`mean|y|={row_e['mean_y_norm']:.6e} -> {row_l['mean_y_norm']:.6e}`, "
            f"`mean|k|={row_e['mean_k_norm']:.6e} -> {row_l['mean_k_norm']:.6e}`, "
            f"`mean|dx|={row_e['mean_abs_dx_value']:.6e} -> {row_l['mean_abs_dx_value']:.6e}`, "
            f"`top_terms={row_e['top_num_term']} -> {row_l['top_num_term']}`."
        )

    lines.append("")
    lines.append("## 6. Quick judgement")
    baseline_axis_df = axis_df[axis_df["case_id"] == baseline_case_id].copy()
    if not baseline_axis_df.empty:
        direct_like = (
            baseline_axis_df.groupby("state_name", as_index=False)[["share_pos", "share_att", "share_lever"]]
            .mean()
            .sort_values(by="state_name")
        )
        for row in direct_like.itertuples(index=False):
            lines.append(
                f"- `{row.state_name}` 平均 share=`pos {row.share_pos:.3f} / att {row.share_att:.3f} / lever {row.share_lever:.3f}`，"
                "若 `lever` 长期很小而 `pos/att` 占优，则说明 bias 主要通过 shared correction 被带动，而不是像直接观测量那样独立更新。"
            )

    lines.append("")
    lines.append("## Coverage")
    lines.append(f"- manifest: `{manifest['manifest_path']}`")
    lines.append(f"- per_update_metrics: `{manifest['per_update_metrics_csv']}`")
    lines.append(f"- axis_mechanism_metrics: `{manifest['axis_mechanism_metrics_csv']}`")
    lines.append(f"- axis_group_summary: `{manifest['axis_group_summary_csv']}`")
    lines.append(f"- axis_term_summary: `{manifest['axis_term_summary_csv']}`")
    lines.append(f"- key_update_examples: `{manifest['key_update_examples_csv']}`")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit ba/bg correction mechanism under pure INS/GNSS with truth-anchored gnss_lever and sg/sa."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.pos_path = (REPO_ROOT / args.pos_path).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")
    if not args.pos_path.exists():
        raise FileNotFoundError(f"missing POS path: {args.pos_path}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    motion_df = load_motion_profile(args.pos_path)

    case_rows: list[dict[str, Any]] = []
    per_update_frames: list[pd.DataFrame] = []
    axis_frames: list[pd.DataFrame] = []

    for audit_case in CASE_SPECS_DEFAULT:
        case_dir = args.case_root / audit_case.case_id
        ensure_dir(case_dir)
        case_meta = run_case(
            audit_case=audit_case,
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            case_dir=case_dir,
            exe_path=args.exe,
        )
        case_rows.append(case_meta)
        update_df, axis_df = analyze_case_updates(case_meta=case_meta, motion_df=motion_df)
        per_update_frames.append(update_df)
        axis_frames.append(axis_df)

    case_df = pd.DataFrame(case_rows)
    per_update_df = pd.concat(per_update_frames, ignore_index=True)
    axis_df = pd.concat(axis_frames, ignore_index=True)
    grouped_df = summarize_axis_groups(axis_df).sort_values(
        by=["case_id", "state_name", "phase_label", "motion_label"]
    ).reset_index(drop=True)
    term_df = build_axis_term_summary(axis_df)
    key_examples_df = extract_key_examples(axis_df)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    per_update_path = args.output_dir / "per_update_metrics.csv"
    axis_metrics_path = args.output_dir / "axis_mechanism_metrics.csv"
    grouped_path = args.output_dir / "axis_group_summary.csv"
    term_path = args.output_dir / "axis_term_summary.csv"
    key_examples_path = args.output_dir / "key_update_examples.csv"
    case_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    per_update_df.to_csv(per_update_path, index=False, encoding="utf-8-sig")
    axis_df.to_csv(axis_metrics_path, index=False, encoding="utf-8-sig")
    grouped_df.to_csv(grouped_path, index=False, encoding="utf-8-sig")
    term_df.to_csv(term_path, index=False, encoding="utf-8-sig")
    key_examples_df.to_csv(key_examples_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "mode": "pure_ins_gnss_bias_mechanism_audit",
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "exe": rel_from_root(args.exe, REPO_ROOT),
        "pos_path": rel_from_root(args.pos_path, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "per_update_metrics_csv": rel_from_root(per_update_path, REPO_ROOT),
        "axis_mechanism_metrics_csv": rel_from_root(axis_metrics_path, REPO_ROOT),
        "axis_group_summary_csv": rel_from_root(grouped_path, REPO_ROOT),
        "axis_term_summary_csv": rel_from_root(term_path, REPO_ROOT),
        "key_update_examples_csv": rel_from_root(key_examples_path, REPO_ROOT),
        "manifest_path": rel_from_root(args.output_dir / "manifest.json", REPO_ROOT),
        "early_update_count_cap": EARLY_UPDATE_COUNT,
        "min_speed_m_s": MIN_SPEED_M_S,
        "turn_yaw_rate_deg_s": TURN_YAW_RATE_DEG_S,
        "straight_yaw_rate_deg_s": STRAIGHT_YAW_RATE_DEG_S,
        "cases": case_rows,
    }
    summary_path = args.output_dir / "summary.md"
    write_summary(
        output_path=summary_path,
        case_df=case_df,
        grouped_df=grouped_df,
        term_df=term_df,
        axis_df=axis_df,
        manifest=manifest,
    )
    manifest["summary_md"] = rel_from_root(summary_path, REPO_ROOT)

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
