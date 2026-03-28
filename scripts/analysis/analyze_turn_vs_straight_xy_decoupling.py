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

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference, json_safe
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (
    MIN_SPEED_M_S,
    MIN_WINDOW_SEPARATION_S,
    OUTPUT_DIR_DEFAULT,
    POS_PATH_DEFAULT,
    ROLLING_WINDOW_S,
    load_pos_dataframe,
    select_turn_windows,
)


STRAIGHT_COST_TURN_WEIGHT = 5.0
STRAIGHT_MATCH_SPEED_WEIGHT = 1.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare GNSS lever innovation allocation in turn vs straight windows.")
    parser.add_argument("--probe-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    return parser


def value_share_from_columns(df: pd.DataFrame, axis: str, prefix: str) -> pd.Series:
    pos = df[f"meas_pos_{axis}"].abs()
    att = df[f"meas_att_{axis}"].abs()
    lever = df[f"meas_lever_{axis}"].abs()
    denom = pos + att + lever
    out = np.full(len(df), math.nan, dtype=float)
    valid = denom > 0.0
    if prefix == "pos":
        out[valid] = (pos / denom)[valid]
    elif prefix == "att":
        out[valid] = (att / denom)[valid]
    else:
        out[valid] = (lever / denom)[valid]
    return pd.Series(out, index=df.index, dtype=float)


def assign_window_id(times: pd.Series, windows_df: pd.DataFrame, column_name: str) -> pd.Series:
    out = np.full(len(times), None, dtype=object)
    for row in windows_df.itertuples(index=False):
        mask = (times >= float(row.start_t)) & (times <= float(row.end_t))
        out[mask.to_numpy()] = str(getattr(row, column_name))
    return pd.Series(out, index=times.index, dtype=object)


def select_matched_straight_windows(
    series_df: pd.DataFrame,
    turn_windows_df: pd.DataFrame,
    rolling_window_s: float,
    min_speed_m_s: float,
    min_separation_s: float,
) -> pd.DataFrame:
    candidates = series_df.loc[
        np.isfinite(series_df["turn_score_deg_s"]) & (series_df["speed_m_s"] >= min_speed_m_s)
    ].copy()
    turn_centers = turn_windows_df["center_t"].to_numpy(dtype=float)
    if len(turn_centers) > 0:
        far_from_turn = np.ones(len(candidates), dtype=bool)
        cand_t = candidates["t"].to_numpy(dtype=float)
        for center_t in turn_centers:
            far_from_turn &= np.abs(cand_t - center_t) >= min_separation_s
        candidates = candidates.loc[far_from_turn].copy()
    candidates = candidates.sort_values(by="t").reset_index(drop=True)
    if candidates.empty:
        raise RuntimeError("no straight-window candidates available after excluding turn windows")

    selected_rows: list[dict[str, Any]] = []
    selected_centers: list[float] = []
    half_window = 0.5 * rolling_window_s
    for turn_row in turn_windows_df.itertuples(index=False):
        target_speed = float(turn_row.speed_m_s)
        candidate_df = candidates.copy()
        if selected_centers:
            cand_t = candidate_df["t"].to_numpy(dtype=float)
            far_from_selected = np.ones(len(candidate_df), dtype=bool)
            for center_t in selected_centers:
                far_from_selected &= np.abs(cand_t - center_t) >= min_separation_s
            candidate_df = candidate_df.loc[far_from_selected].copy()
        if candidate_df.empty:
            raise RuntimeError(f"failed to find matched straight window for {turn_row.window_id}")
        candidate_df["match_cost"] = (
            STRAIGHT_COST_TURN_WEIGHT * candidate_df["turn_score_deg_s"]
            + STRAIGHT_MATCH_SPEED_WEIGHT * (candidate_df["speed_m_s"] - target_speed).abs()
        )
        best = candidate_df.sort_values(
            by=["match_cost", "turn_score_deg_s", "speed_m_s"], ascending=[True, True, True]
        ).iloc[0]
        center_t = float(best["t"])
        selected_centers.append(center_t)
        selected_rows.append(
            {
                "window_id": f"straight_window_{len(selected_rows) + 1}",
                "matched_turn_window_id": str(turn_row.window_id),
                "center_t": center_t,
                "start_t": center_t - half_window,
                "end_t": center_t + half_window,
                "turn_score_deg_s": float(best["turn_score_deg_s"]),
                "abs_yaw_rate_deg_s": float(abs(best["yaw_rate_deg_s"])),
                "speed_m_s": float(best["speed_m_s"]),
                "matched_turn_speed_m_s": target_speed,
                "speed_delta_m_s": float(best["speed_m_s"] - target_speed),
                "match_cost": float(best["match_cost"]),
            }
        )
    return pd.DataFrame(selected_rows)


def add_xy_metrics(df: pd.DataFrame, truth_lever: np.ndarray) -> pd.DataFrame:
    out = df.copy()
    for axis_idx, axis in enumerate(["x", "y"]):
        out[f"pos_share_{axis}_calc"] = value_share_from_columns(out, axis, "pos")
        out[f"att_share_{axis}_calc"] = value_share_from_columns(out, axis, "att")
        out[f"lever_share_{axis}_calc"] = value_share_from_columns(out, axis, "lever")
        out[f"lever_error_{axis}_before_calc"] = (out[f"lever_before_{axis}"] - truth_lever[axis_idx]).abs()
        out[f"lever_error_{axis}_after_calc"] = (out[f"lever_after_{axis}"] - truth_lever[axis_idx]).abs()
    return out


def summarize_windows(
    df: pd.DataFrame,
    windows_df: pd.DataFrame,
    case_id: str,
    phase: str,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for row in windows_df.itertuples(index=False):
        window_id = str(row.window_id)
        sub = df.loc[df["analysis_window_id"] == window_id].sort_values(by="gnss_t").reset_index(drop=True)
        if sub.empty:
            continue
        first = sub.iloc[0]
        last = sub.iloc[-1]
        rows.append(
            {
                "case_id": case_id,
                "phase": phase,
                "window_id": window_id,
                "matched_turn_window_id": str(getattr(row, "matched_turn_window_id", "")),
                "center_t": float(row.center_t),
                "turn_score_deg_s": float(row.turn_score_deg_s),
                "speed_m_s": float(row.speed_m_s),
                "updates": int(len(sub)),
                "mean_abs_y_x": float(sub["y_x"].abs().mean()),
                "mean_abs_y_y": float(sub["y_y"].abs().mean()),
                "mean_pos_share_x": float(sub["pos_share_x_calc"].mean()),
                "mean_att_share_x": float(sub["att_share_x_calc"].mean()),
                "mean_lever_share_x": float(sub["lever_share_x_calc"].mean()),
                "mean_pos_share_y": float(sub["pos_share_y_calc"].mean()),
                "mean_att_share_y": float(sub["att_share_y_calc"].mean()),
                "mean_lever_share_y": float(sub["lever_share_y_calc"].mean()),
                "mean_abs_meas_pos_x": float(sub["meas_pos_x"].abs().mean()),
                "mean_abs_meas_att_x": float(sub["meas_att_x"].abs().mean()),
                "mean_abs_meas_lever_x": float(sub["meas_lever_x"].abs().mean()),
                "mean_abs_meas_pos_y": float(sub["meas_pos_y"].abs().mean()),
                "mean_abs_meas_att_y": float(sub["meas_att_y"].abs().mean()),
                "mean_abs_meas_lever_y": float(sub["meas_lever_y"].abs().mean()),
                "lever_x_error_improve_m": float(first["lever_error_x_before_calc"] - last["lever_error_x_after_calc"]),
                "lever_y_error_improve_m": float(first["lever_error_y_before_calc"] - last["lever_error_y_after_calc"]),
            }
        )
    return pd.DataFrame(rows)


def build_case_comparison(
    window_summary_df: pd.DataFrame,
    turn_windows_df: pd.DataFrame,
    straight_windows_df: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_id in sorted(window_summary_df["case_id"].unique()):
        turn_sub = window_summary_df.loc[(window_summary_df["case_id"] == case_id) & (window_summary_df["phase"] == "turn")]
        straight_sub = window_summary_df.loc[
            (window_summary_df["case_id"] == case_id) & (window_summary_df["phase"] == "straight")
        ]
        if turn_sub.empty or straight_sub.empty:
            continue
        row: dict[str, Any] = {
            "case_id": case_id,
            "turn_windows": int(len(turn_sub)),
            "straight_windows": int(len(straight_sub)),
            "mean_turn_score_turn_deg_s": float(turn_sub["turn_score_deg_s"].mean()),
            "mean_turn_score_straight_deg_s": float(straight_sub["turn_score_deg_s"].mean()),
            "mean_speed_turn_m_s": float(turn_sub["speed_m_s"].mean()),
            "mean_speed_straight_m_s": float(straight_sub["speed_m_s"].mean()),
        }
        metric_prefixes = [
            "mean_abs_y",
            "mean_pos_share",
            "mean_att_share",
            "mean_lever_share",
            "mean_abs_meas_pos",
            "mean_abs_meas_att",
            "mean_abs_meas_lever",
        ]
        for axis in ["x", "y"]:
            for metric in metric_prefixes:
                col = f"{metric}_{axis}"
                turn_val = float(turn_sub[col].mean())
                straight_val = float(straight_sub[col].mean())
                row[f"turn_{col}"] = turn_val
                row[f"straight_{col}"] = straight_val
                row[f"delta_{col}"] = turn_val - straight_val
            improve_col = f"lever_{axis}_error_improve_m"
            turn_improve = float(turn_sub[improve_col].mean())
            straight_improve = float(straight_sub[improve_col].mean())
            row[f"turn_{improve_col}"] = turn_improve
            row[f"straight_{improve_col}"] = straight_improve
            row[f"delta_{improve_col}"] = turn_improve - straight_improve
        rows.append(row)
    return pd.DataFrame(rows)


def write_summary(
    output_path: Path,
    turn_windows_df: pd.DataFrame,
    straight_windows_df: pd.DataFrame,
    case_comparison_df: pd.DataFrame,
) -> None:
    lines: list[str] = []
    lines.append("# turn-vs-straight xy decoupling summary")
    lines.append("")
    lines.append("## Matched straight windows")
    for row in straight_windows_df.itertuples(index=False):
        lines.append(
            f"- `{row.window_id}` matched to `{row.matched_turn_window_id}`: center=`{row.center_t:.3f}` s, "
            f"turn_score=`{row.turn_score_deg_s:.3f}` deg/s, `|yaw_rate|=`{row.abs_yaw_rate_deg_s:.3f}` deg/s, "
            f"speed=`{row.speed_m_s:.3f}` m/s, `speed_delta={row.speed_delta_m_s:.3f}` m/s."
        )
    lines.append("")
    lines.append("## Case comparison")
    for row in case_comparison_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}`: turn vs straight `x` lever-share=`{row.turn_mean_lever_share_x:.4f}` vs "
            f"`{row.straight_mean_lever_share_x:.4f}`, `y` lever-share=`{row.turn_mean_lever_share_y:.4f}` vs "
            f"`{row.straight_mean_lever_share_y:.4f}`; `x` pos-share=`{row.turn_mean_pos_share_x:.4f}` vs "
            f"`{row.straight_mean_pos_share_x:.4f}`, `y` pos-share=`{row.turn_mean_pos_share_y:.4f}` vs "
            f"`{row.straight_mean_pos_share_y:.4f}`; `x` error-improve=`{row.turn_lever_x_error_improve_m:.6f}` vs "
            f"`{row.straight_lever_x_error_improve_m:.6f}` m, `y` error-improve=`{row.turn_lever_y_error_improve_m:.6f}` vs "
            f"`{row.straight_lever_y_error_improve_m:.6f}` m."
        )
    lines.append("")
    lines.append("## Reading guide")
    lines.append(
        "- If turning creates real decoupling, `lever_share_x/y` and `lever_error_improve_x/y` should rise relative to straight windows while `pos_share_x/y` falls."
    )
    lines.append(
        "- If turning only increases innovation magnitude but `pos_share_x/y` remains dominant, the system still lacks effective lever-arm decoupling."
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    probe_dir = (REPO_ROOT / args.probe_dir).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()
    ensure_dir(probe_dir)

    breakdown_path = probe_dir / "turn_window_update_breakdown.csv"
    turn_windows_path = probe_dir / "turn_windows.csv"
    if not breakdown_path.exists():
        raise RuntimeError(f"missing breakdown csv: {breakdown_path}")
    if not turn_windows_path.exists():
        raise RuntimeError(f"missing turn windows csv: {turn_windows_path}")

    base_cfg = yaml.safe_load((REPO_ROOT / "config_data2_baseline_eskf.yaml").read_text(encoding="utf-8"))
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
    _, turn_series_df = select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=ROLLING_WINDOW_S,
        top_k=5,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    turn_windows_df = pd.read_csv(turn_windows_path)
    straight_windows_df = select_matched_straight_windows(
        series_df=turn_series_df,
        turn_windows_df=turn_windows_df,
        rolling_window_s=ROLLING_WINDOW_S,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )

    breakdown_df = pd.read_csv(breakdown_path)
    breakdown_df = add_xy_metrics(breakdown_df, truth_lever)
    breakdown_df["analysis_window_id"] = assign_window_id(breakdown_df["gnss_t"], turn_windows_df, "window_id")
    straight_ids = assign_window_id(breakdown_df["gnss_t"], straight_windows_df, "window_id")
    breakdown_df.loc[breakdown_df["analysis_window_id"].isna(), "analysis_window_id"] = straight_ids.loc[
        breakdown_df["analysis_window_id"].isna()
    ]

    window_rows: list[pd.DataFrame] = []
    for case_id in sorted(breakdown_df["case_id"].unique()):
        case_df = breakdown_df.loc[breakdown_df["case_id"] == case_id].copy()
        window_rows.append(summarize_windows(case_df, turn_windows_df, case_id, "turn"))
        window_rows.append(summarize_windows(case_df, straight_windows_df, case_id, "straight"))
    window_summary_df = pd.concat(window_rows, ignore_index=True)
    case_comparison_df = build_case_comparison(window_summary_df, turn_windows_df, straight_windows_df)

    straight_windows_out = probe_dir / "matched_straight_windows.csv"
    window_summary_out = probe_dir / "turn_vs_straight_xy_window_summary.csv"
    case_summary_out = probe_dir / "turn_vs_straight_xy_case_summary.csv"
    summary_md_out = probe_dir / "turn_vs_straight_xy_summary.md"
    straight_windows_df.to_csv(straight_windows_out, index=False, encoding="utf-8-sig")
    window_summary_df.to_csv(window_summary_out, index=False, encoding="utf-8-sig")
    case_comparison_df.to_csv(case_summary_out, index=False, encoding="utf-8-sig")
    write_summary(summary_md_out, turn_windows_df, straight_windows_df, case_comparison_df)

    manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "probe_dir": rel_from_root(probe_dir, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "inputs": {
            "turn_windows_csv": rel_from_root(turn_windows_path, REPO_ROOT),
            "turn_window_update_breakdown_csv": rel_from_root(breakdown_path, REPO_ROOT),
        },
        "artifacts": {
            "matched_straight_windows_csv": rel_from_root(straight_windows_out, REPO_ROOT),
            "turn_vs_straight_xy_window_summary_csv": rel_from_root(window_summary_out, REPO_ROOT),
            "turn_vs_straight_xy_case_summary_csv": rel_from_root(case_summary_out, REPO_ROOT),
            "turn_vs_straight_xy_summary_md": rel_from_root(summary_md_out, REPO_ROOT),
        },
        "params": {
            "rolling_window_s": ROLLING_WINDOW_S,
            "min_speed_m_s": MIN_SPEED_M_S,
            "min_window_separation_s": MIN_WINDOW_SEPARATION_S,
            "straight_cost_turn_weight": STRAIGHT_COST_TURN_WEIGHT,
            "straight_match_speed_weight": STRAIGHT_MATCH_SPEED_WEIGHT,
        },
    }
    manifest_path = probe_dir / "turn_vs_straight_xy_manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
