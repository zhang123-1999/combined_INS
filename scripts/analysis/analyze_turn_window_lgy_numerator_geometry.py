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
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.analyze_turn_vs_straight_xy_decoupling import assign_window_id
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (
    MIN_SPEED_M_S,
    MIN_WINDOW_SEPARATION_S,
    POS_PATH_DEFAULT,
    ROLLING_WINDOW_S,
    TOP_K_WINDOWS,
    ecef_to_llh_single,
    load_effective_gnss_pos_update_df,
    load_gnss_dataframe,
    llh_to_ecef_deg,
    load_pos_dataframe,
    parse_matrix_field,
    parse_vector_field,
    quat_to_rot,
    rot_ned_to_ecef,
    select_turn_windows,
)
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference, json_safe


EXP_ID_DEFAULT = "EXP-20260319-data2-turn-window-lgy-numerator-geometry-r1"
SOURCE_PROBE_DEFAULT = Path("output/data2_turn_window_lgy_from_y_gain_scale_probe")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_lgy_numerator_geometry_analysis")
FOCUS_CASES_DEFAULT = [
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref",
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p75",
    "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0",
]
FOCUS_WINDOWS = ["turn_window_1", "turn_window_2", "turn_window_3", "turn_window_4", "turn_window_5"]
EPS = 1.0e-12


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze `k_lgy_y` numerator / geometry generation across positive and negative turn windows."
    )
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--source-probe", type=Path, default=SOURCE_PROBE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    parser.add_argument(
        "--case-id",
        action="append",
        default=None,
        help="Case id to analyze. Can be passed multiple times; defaults to focused reference cases.",
    )
    return parser


def nonzero_sign(value: float) -> float:
    if not np.isfinite(value) or abs(value) <= EPS:
        return math.nan
    return 1.0 if value > 0.0 else -1.0


def load_case_meta(source_probe: Path) -> dict[str, dict[str, Any]]:
    manifest = json.loads((source_probe / "manifest.json").read_text(encoding="utf-8"))
    return {str(row["case_id"]): row for row in manifest.get("case_rows", [])}


def component_share(value: float, pool: list[float]) -> float:
    denom = float(sum(abs(v) for v in pool if np.isfinite(v)))
    if denom <= 0.0:
        return math.nan
    return abs(float(value)) / denom


def augment_update_rows(update_df: pd.DataFrame, gnss_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for row in update_df.itertuples(index=False):
        p_before = np.array([row.state_p_before_x, row.state_p_before_y, row.state_p_before_z], dtype=float)
        q_before = np.array(
            [row.state_q_before_w, row.state_q_before_x, row.state_q_before_y, row.state_q_before_z], dtype=float
        )
        lever_before = np.array([row.lever_before_x, row.lever_before_y, row.lever_before_z], dtype=float)

        s_mat = parse_matrix_field(getattr(row, "s_mat", None), 3, 3)
        s_inv = np.full((3, 3), math.nan, dtype=float)
        if np.isfinite(s_mat).all():
            try:
                s_inv = np.linalg.inv(s_mat)
            except np.linalg.LinAlgError:
                pass

        raw_k_lgy = parse_vector_field(getattr(row, "raw_k_gnss_lever_y_vec", None), 3)
        post_k_lgy = parse_vector_field(getattr(row, "k_gnss_lever_y_vec", None), 3)
        num_total = parse_vector_field(getattr(row, "num_gnss_lever_y_vec", None), 3)
        num_pos = parse_vector_field(getattr(row, "num_gnss_lever_y_from_pos_vec", None), 3)
        num_att = parse_vector_field(getattr(row, "num_gnss_lever_y_from_att_vec", None), 3)
        num_lever = parse_vector_field(getattr(row, "num_gnss_lever_y_from_gnss_lever_vec", None), 3)
        num_lgx = parse_vector_field(getattr(row, "num_gnss_lever_y_from_lgx_vec", None), 3)
        num_lgy = parse_vector_field(getattr(row, "num_gnss_lever_y_from_lgy_vec", None), 3)
        num_lgz = parse_vector_field(getattr(row, "num_gnss_lever_y_from_lgz_vec", None), 3)

        h_pos_x = parse_vector_field(getattr(row, "h_pos_x_vec", None), 3)
        h_pos_y = parse_vector_field(getattr(row, "h_pos_y_vec", None), 3)
        h_pos_z = parse_vector_field(getattr(row, "h_pos_z_vec", None), 3)
        h_att_x = parse_vector_field(getattr(row, "h_att_x_vec", None), 3)
        h_att_y = parse_vector_field(getattr(row, "h_att_y_vec", None), 3)
        h_att_z = parse_vector_field(getattr(row, "h_att_z_vec", None), 3)
        h_lg_x = parse_vector_field(getattr(row, "h_gnss_lever_x_vec", None), 3)
        h_lg_y = parse_vector_field(getattr(row, "h_gnss_lever_y_vec", None), 3)
        h_lg_z = parse_vector_field(getattr(row, "h_gnss_lever_z_vec", None), 3)
        prior_cov_lg = parse_matrix_field(getattr(row, "prior_cov_gnss_lever_mat", None), 3, 3)

        lat_rad, lon_rad, _ = ecef_to_llh_single(*p_before)
        r_ne = rot_ned_to_ecef(lat_rad, lon_rad)
        c_be = quat_to_rot(q_before)
        pred_ant_ecef = p_before + c_be @ lever_before

        gnss_meas = gnss_df.iloc[(gnss_df["t"] - float(row.gnss_t)).abs().argmin()]
        meas_ecef = llh_to_ecef_deg(float(gnss_meas["lat"]), float(gnss_meas["lon"]), float(gnss_meas["h"]))
        innovation_recon = -r_ne.T @ (pred_ant_ecef - meas_ecef)

        raw_k_y_from_num_x = float(num_total[0] * s_inv[0, 1]) if np.isfinite(num_total[0]) and np.isfinite(s_inv[0, 1]) else math.nan
        raw_k_y_from_num_y = float(num_total[1] * s_inv[1, 1]) if np.isfinite(num_total[1]) and np.isfinite(s_inv[1, 1]) else math.nan
        raw_k_y_from_num_z = float(num_total[2] * s_inv[2, 1]) if np.isfinite(num_total[2]) and np.isfinite(s_inv[2, 1]) else math.nan

        rows.append(
            {
                "case_id": str(row.case_id),
                "label": str(row.label),
                "r_scale": float(row.r_scale),
                "gnss_t": float(row.gnss_t),
                "state_t": float(row.state_t),
                "y_x": float(row.y_x),
                "y_y": float(row.y_y),
                "y_z": float(row.y_z),
                "yaw_rate_deg_s": float(row.yaw_rate_deg_s),
                "turn_direction_sign": nonzero_sign(float(row.yaw_rate_deg_s)),
                "window_id": row.window_id,
                "pred_ant_ecef_x": float(pred_ant_ecef[0]),
                "pred_ant_ecef_y": float(pred_ant_ecef[1]),
                "pred_ant_ecef_z": float(pred_ant_ecef[2]),
                "innovation_recon_y": float(innovation_recon[1]),
                "innovation_recon_error_norm": float(
                    np.linalg.norm(innovation_recon - np.array([row.y_x, row.y_y, row.y_z], dtype=float))
                ),
                "desired_dx_y": float(row.desired_dx_y),
                "dx_gnss_lever_y": float(row.dx_gnss_lever_y),
                "raw_k_lgy_x": float(raw_k_lgy[0]),
                "raw_k_lgy_y": float(raw_k_lgy[1]),
                "raw_k_lgy_z": float(raw_k_lgy[2]),
                "post_k_lgy_x": float(post_k_lgy[0]),
                "post_k_lgy_y": float(post_k_lgy[1]),
                "post_k_lgy_z": float(post_k_lgy[2]),
                "num_lgy_x": float(num_total[0]),
                "num_lgy_y": float(num_total[1]),
                "num_lgy_z": float(num_total[2]),
                "num_lgy_from_pos_x": float(num_pos[0]),
                "num_lgy_from_pos_y": float(num_pos[1]),
                "num_lgy_from_pos_z": float(num_pos[2]),
                "num_lgy_from_att_x": float(num_att[0]),
                "num_lgy_from_att_y": float(num_att[1]),
                "num_lgy_from_att_z": float(num_att[2]),
                "num_lgy_from_lever_x": float(num_lever[0]),
                "num_lgy_from_lever_y": float(num_lever[1]),
                "num_lgy_from_lever_z": float(num_lever[2]),
                "num_lgy_from_lgx_x": float(num_lgx[0]),
                "num_lgy_from_lgx_y": float(num_lgx[1]),
                "num_lgy_from_lgx_z": float(num_lgx[2]),
                "num_lgy_from_lgy_x": float(num_lgy[0]),
                "num_lgy_from_lgy_y": float(num_lgy[1]),
                "num_lgy_from_lgy_z": float(num_lgy[2]),
                "num_lgy_from_lgz_x": float(num_lgz[0]),
                "num_lgy_from_lgz_y": float(num_lgz[1]),
                "num_lgy_from_lgz_z": float(num_lgz[2]),
                "raw_k_y_from_num_x": raw_k_y_from_num_x,
                "raw_k_y_from_num_y": raw_k_y_from_num_y,
                "raw_k_y_from_num_z": raw_k_y_from_num_z,
                "s_xy": float(s_mat[0, 1]),
                "s_yy": float(s_mat[1, 1]),
                "s_zy": float(s_mat[2, 1]),
                "s_inv_xy": float(s_inv[0, 1]),
                "s_inv_yy": float(s_inv[1, 1]),
                "s_inv_zy": float(s_inv[2, 1]),
                "num_y_pos_share": component_share(float(num_pos[1]), [float(num_pos[1]), float(num_att[1]), float(num_lever[1])]),
                "num_y_att_share": component_share(float(num_att[1]), [float(num_pos[1]), float(num_att[1]), float(num_lever[1])]),
                "num_y_lever_share": component_share(float(num_lever[1]), [float(num_pos[1]), float(num_att[1]), float(num_lever[1])]),
                "num_y_lgx_share": component_share(float(num_lgx[1]), [float(num_lgx[1]), float(num_lgy[1]), float(num_lgz[1])]),
                "num_y_lgy_share": component_share(float(num_lgy[1]), [float(num_lgx[1]), float(num_lgy[1]), float(num_lgz[1])]),
                "num_y_lgz_share": component_share(float(num_lgz[1]), [float(num_lgx[1]), float(num_lgy[1]), float(num_lgz[1])]),
                "h_y_pos_x": float(h_pos_x[1]),
                "h_y_pos_y": float(h_pos_y[1]),
                "h_y_pos_z": float(h_pos_z[1]),
                "h_y_att_x": float(h_att_x[1]),
                "h_y_att_y": float(h_att_y[1]),
                "h_y_att_z": float(h_att_z[1]),
                "h_y_lever_x": float(h_lg_x[1]),
                "h_y_lever_y": float(h_lg_y[1]),
                "h_y_lever_z": float(h_lg_z[1]),
                "cov_lgy_lgx": float(prior_cov_lg[1, 0]),
                "cov_lgy_lgy": float(prior_cov_lg[1, 1]),
                "cov_lgy_lgz": float(prior_cov_lg[1, 2]),
                "num_y_lgx_from_cov_h": float(prior_cov_lg[1, 0] * h_lg_x[1]),
                "num_y_lgy_from_cov_h": float(prior_cov_lg[1, 1] * h_lg_y[1]),
                "num_y_lgz_from_cov_h": float(prior_cov_lg[1, 2] * h_lg_z[1]),
            }
        )
    return pd.DataFrame(rows).sort_values(by=["case_id", "gnss_t"]).reset_index(drop=True)


def summarize_windows(per_update_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for (case_id, label, window_id), group in per_update_df.groupby(["case_id", "label", "window_id"], sort=False):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        rows.append(
            {
                "case_id": str(case_id),
                "label": str(label),
                "window_id": str(window_id),
                "turn_direction_sign": float(np.nanmean(group["turn_direction_sign"].to_numpy(dtype=float))),
                "updates": int(len(group)),
                "mean_yaw_rate_deg_s": float(np.nanmean(group["yaw_rate_deg_s"].to_numpy(dtype=float))),
                "mean_innovation_y": float(np.nanmean(group["y_y"].to_numpy(dtype=float))),
                "mean_innovation_recon_y": float(np.nanmean(group["innovation_recon_y"].to_numpy(dtype=float))),
                "max_innovation_recon_error_norm": float(np.nanmax(group["innovation_recon_error_norm"].to_numpy(dtype=float))),
                "mean_desired_dx_y": float(np.nanmean(group["desired_dx_y"].to_numpy(dtype=float))),
                "mean_dx_gnss_lever_y": float(np.nanmean(group["dx_gnss_lever_y"].to_numpy(dtype=float))),
                "mean_num_lgy_y": float(np.nanmean(group["num_lgy_y"].to_numpy(dtype=float))),
                "mean_num_lgy_from_pos_y": float(np.nanmean(group["num_lgy_from_pos_y"].to_numpy(dtype=float))),
                "mean_num_lgy_from_att_y": float(np.nanmean(group["num_lgy_from_att_y"].to_numpy(dtype=float))),
                "mean_num_lgy_from_lever_y": float(np.nanmean(group["num_lgy_from_lever_y"].to_numpy(dtype=float))),
                "mean_num_lgy_from_lgx_y": float(np.nanmean(group["num_lgy_from_lgx_y"].to_numpy(dtype=float))),
                "mean_num_lgy_from_lgy_y": float(np.nanmean(group["num_lgy_from_lgy_y"].to_numpy(dtype=float))),
                "mean_num_lgy_from_lgz_y": float(np.nanmean(group["num_lgy_from_lgz_y"].to_numpy(dtype=float))),
                "mean_num_y_pos_share": float(np.nanmean(group["num_y_pos_share"].to_numpy(dtype=float))),
                "mean_num_y_att_share": float(np.nanmean(group["num_y_att_share"].to_numpy(dtype=float))),
                "mean_num_y_lever_share": float(np.nanmean(group["num_y_lever_share"].to_numpy(dtype=float))),
                "mean_num_y_lgx_share": float(np.nanmean(group["num_y_lgx_share"].to_numpy(dtype=float))),
                "mean_num_y_lgy_share": float(np.nanmean(group["num_y_lgy_share"].to_numpy(dtype=float))),
                "mean_num_y_lgz_share": float(np.nanmean(group["num_y_lgz_share"].to_numpy(dtype=float))),
                "mean_raw_k_lgy_y": float(np.nanmean(group["raw_k_lgy_y"].to_numpy(dtype=float))),
                "mean_post_k_lgy_y": float(np.nanmean(group["post_k_lgy_y"].to_numpy(dtype=float))),
                "mean_raw_k_y_from_num_x": float(np.nanmean(group["raw_k_y_from_num_x"].to_numpy(dtype=float))),
                "mean_raw_k_y_from_num_y": float(np.nanmean(group["raw_k_y_from_num_y"].to_numpy(dtype=float))),
                "mean_raw_k_y_from_num_z": float(np.nanmean(group["raw_k_y_from_num_z"].to_numpy(dtype=float))),
                "mean_s_yy": float(np.nanmean(group["s_yy"].to_numpy(dtype=float))),
                "mean_s_inv_yy": float(np.nanmean(group["s_inv_yy"].to_numpy(dtype=float))),
                "mean_h_y_pos_x": float(np.nanmean(group["h_y_pos_x"].to_numpy(dtype=float))),
                "mean_h_y_pos_y": float(np.nanmean(group["h_y_pos_y"].to_numpy(dtype=float))),
                "mean_h_y_pos_z": float(np.nanmean(group["h_y_pos_z"].to_numpy(dtype=float))),
                "mean_h_y_att_x": float(np.nanmean(group["h_y_att_x"].to_numpy(dtype=float))),
                "mean_h_y_att_y": float(np.nanmean(group["h_y_att_y"].to_numpy(dtype=float))),
                "mean_h_y_att_z": float(np.nanmean(group["h_y_att_z"].to_numpy(dtype=float))),
                "mean_h_y_lever_x": float(np.nanmean(group["h_y_lever_x"].to_numpy(dtype=float))),
                "mean_h_y_lever_y": float(np.nanmean(group["h_y_lever_y"].to_numpy(dtype=float))),
                "mean_h_y_lever_z": float(np.nanmean(group["h_y_lever_z"].to_numpy(dtype=float))),
                "mean_cov_lgy_lgx": float(np.nanmean(group["cov_lgy_lgx"].to_numpy(dtype=float))),
                "mean_cov_lgy_lgy": float(np.nanmean(group["cov_lgy_lgy"].to_numpy(dtype=float))),
                "mean_cov_lgy_lgz": float(np.nanmean(group["cov_lgy_lgz"].to_numpy(dtype=float))),
                "mean_num_y_lgx_from_cov_h": float(np.nanmean(group["num_y_lgx_from_cov_h"].to_numpy(dtype=float))),
                "mean_num_y_lgy_from_cov_h": float(np.nanmean(group["num_y_lgy_from_cov_h"].to_numpy(dtype=float))),
                "mean_num_y_lgz_from_cov_h": float(np.nanmean(group["num_y_lgz_from_cov_h"].to_numpy(dtype=float))),
            }
        )
    return pd.DataFrame(rows)


def build_focus_comparison(window_df: pd.DataFrame) -> pd.DataFrame:
    focus = window_df.loc[window_df["window_id"].isin(FOCUS_WINDOWS)].copy()
    return focus.sort_values(by=["case_id", "window_id"]).reset_index(drop=True)


def write_summary(summary_path: Path, focus_df: pd.DataFrame) -> None:
    lines: list[str] = []
    lines.append("# turn-window lgy numerator geometry analysis")
    lines.append("")
    lines.append("## Core reading")
    lines.append("- `num_lgy_*` is the raw pre-scale numerator `P(l_gy,*) H^T`; `raw_k_lgy_*` is the pre-scale Kalman gain row derived from that numerator and `S^{-1}`; `post_k_lgy_*` is the actually applied gain after solver-side selective scaling.")
    lines.append("- `num_lgy_from_pos/att/lever_y` separates the measurement-`y` numerator source by state block, while `raw_k_y_from_num_x/y/z` shows how numerator components are mixed into `raw K(l_gy, meas_y)` through `S^{-1}`.")
    lines.append("- `innovation_recon_y` is rebuilt from `pred_ant_ecef -> meas_ecef` geometry; a small reconstruction error means the geometry chain is internally consistent for that update.")
    lines.append("")
    lines.append("## Focus windows")
    for row in focus_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}:{row.window_id}` (`turn_sign={row.turn_direction_sign:+.0f}`, "
            f"`yaw_rate={row.mean_yaw_rate_deg_s:.3f}` deg/s): "
            f"`innov_y={row.mean_innovation_y:.6f}`, `desired_dx_y={row.mean_desired_dx_y:.6f}`, "
            f"`num_y(pos/att/lever)={row.mean_num_lgy_from_pos_y:.6f}/{row.mean_num_lgy_from_att_y:.6f}/{row.mean_num_lgy_from_lever_y:.6f}`, "
            f"`num_y(lever_lgx/lgy/lgz)={row.mean_num_lgy_from_lgx_y:.6f}/{row.mean_num_lgy_from_lgy_y:.6f}/{row.mean_num_lgy_from_lgz_y:.6f}`, "
            f"`cov_lgy(lgx/lgy/lgz)={row.mean_cov_lgy_lgx:.6f}/{row.mean_cov_lgy_lgy:.6f}/{row.mean_cov_lgy_lgz:.6f}`, "
            f"`raw_k_y={row.mean_raw_k_lgy_y:.6f}`, `post_k_y={row.mean_post_k_lgy_y:.6f}`, "
            f"`raw_k_y_from_num_x/y/z={row.mean_raw_k_y_from_num_x:.6f}/{row.mean_raw_k_y_from_num_y:.6f}/{row.mean_raw_k_y_from_num_z:.6f}`, "
            f"`h_y_pos={row.mean_h_y_pos_x:.6f},{row.mean_h_y_pos_y:.6f},{row.mean_h_y_pos_z:.6f}`, "
            f"`h_y_att={row.mean_h_y_att_x:.6f},{row.mean_h_y_att_y:.6f},{row.mean_h_y_att_z:.6f}`, "
            f"`h_y_lever={row.mean_h_y_lever_x:.6f},{row.mean_h_y_lever_y:.6f},{row.mean_h_y_lever_z:.6f}`, "
            f"`num_y(cov*h)_lgx/lgy/lgz={row.mean_num_y_lgx_from_cov_h:.6f}/{row.mean_num_y_lgy_from_cov_h:.6f}/{row.mean_num_y_lgz_from_cov_h:.6f}`, "
            f"`innov_recon_err_max={row.max_innovation_recon_error_norm:.3e}`."
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probe = (REPO_ROOT / args.source_probe).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()
    case_ids = args.case_id if args.case_id else FOCUS_CASES_DEFAULT

    if not source_probe.exists():
        raise RuntimeError(f"missing source probe dir: {source_probe}")

    ensure_dir(output_dir)

    pos_df = load_pos_dataframe(pos_path)
    windows_df, turn_series_df = select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=ROLLING_WINDOW_S,
        top_k=TOP_K_WINDOWS,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    case_meta_map = load_case_meta(source_probe)

    per_case_rows: list[pd.DataFrame] = []
    for case_id in case_ids:
        case_meta = case_meta_map.get(case_id)
        if case_meta is None:
            raise RuntimeError(f"missing case meta for {case_id}")
        raw_path = source_probe / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv"
        cfg_path = REPO_ROOT / str(case_meta["config_path"])
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        truth_reference = build_truth_reference(cfg)
        truth_lever_y = float(truth_reference["states"]["gnss_lever_y"]["reference_value_internal"])
        gnss_path = (REPO_ROOT / str(cfg["fusion"]["gnss_path"])).resolve()
        gnss_df = load_gnss_dataframe(gnss_path)
        update_df = load_effective_gnss_pos_update_df(raw_path)
        update_df = update_df.loc[update_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)
        update_df["yaw_rate_deg_s"] = np.interp(
            update_df["gnss_t"].to_numpy(dtype=float),
            turn_series_df["t"].to_numpy(dtype=float),
            turn_series_df["yaw_rate_deg_s"].to_numpy(dtype=float),
        )
        update_df["window_id"] = assign_window_id(update_df["gnss_t"], windows_df, "window_id")
        update_df = update_df.loc[update_df["window_id"].isin(FOCUS_WINDOWS)].copy().reset_index(drop=True)
        if update_df.empty:
            raise RuntimeError(f"no focus-window GNSS_POS updates found for {case_id}")
        update_df["case_id"] = case_id
        update_df["label"] = str(case_meta.get("label", case_id))
        update_df["r_scale"] = float(case_meta.get("r_scale", math.nan))
        desired_dx_y = []
        for row in update_df.itertuples(index=False):
            desired_dx_y.append(float(truth_lever_y - row.lever_before_y))
        update_df["desired_dx_y"] = desired_dx_y
        per_case_rows.append(augment_update_rows(update_df, gnss_df))

    per_update_df = pd.concat(per_case_rows, ignore_index=True)
    window_df = summarize_windows(per_update_df)
    focus_df = build_focus_comparison(window_df)

    per_update_out = output_dir / "lgy_numerator_geometry_per_update.csv"
    window_out = output_dir / "lgy_numerator_geometry_window_summary.csv"
    focus_out = output_dir / "lgy_numerator_geometry_focus_comparison.csv"
    summary_out = output_dir / "summary.md"
    manifest_out = output_dir / "manifest.json"

    per_update_df.to_csv(per_update_out, index=False, encoding="utf-8-sig")
    window_df.to_csv(window_out, index=False, encoding="utf-8-sig")
    focus_df.to_csv(focus_out, index=False, encoding="utf-8-sig")
    write_summary(summary_out, focus_df)

    manifest = {
        "exp_id": str(args.exp_id),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probe": rel_from_root(source_probe, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "case_ids": case_ids,
        "focus_windows": FOCUS_WINDOWS,
        "artifacts": {
            "per_update_csv": rel_from_root(per_update_out, REPO_ROOT),
            "window_summary_csv": rel_from_root(window_out, REPO_ROOT),
            "focus_comparison_csv": rel_from_root(focus_out, REPO_ROOT),
            "summary_md": rel_from_root(summary_out, REPO_ROOT),
        },
    }
    manifest_out.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
