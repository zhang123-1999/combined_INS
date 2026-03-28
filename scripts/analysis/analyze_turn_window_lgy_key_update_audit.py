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


EXP_ID_DEFAULT = "EXP-20260319-data2-turn-window-lgy-key-update-audit-r1"
SOURCE_PROBE_DEFAULT = Path("output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1")
HISTORY_SOURCE_DEFAULT = Path("output/data2_turn_window_lgy_history_analysis/lgy_history_per_update.csv")
DIFF_SOURCE_DEFAULT = Path("output/data2_turn_window_lgy_transition_carryover_analysis/full_case_diff_per_update.csv")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_lgy_key_update_audit")

REF_CASE_ID = "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref"
ALT_CASE_ID = "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0"
WINDOW1_ID = "turn_window_1"
WINDOW1_TARGETS = [529531.0, 529532.0, 529536.0]
CARRYOVER_METRICS = [
    "delta_lever_before_y_m",
    "delta_cov_lgy_py",
    "delta_cov_lgy_lgy",
    "delta_post_k_lgy_y",
    "delta_raw_k_lgy_y",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Audit the key lgy sign-transition updates in window_1 and the survival chain from turn_window_4 to turn_window_3."
    )
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--source-probe", type=Path, default=SOURCE_PROBE_DEFAULT)
    parser.add_argument("--history-source", type=Path, default=HISTORY_SOURCE_DEFAULT)
    parser.add_argument("--diff-source", type=Path, default=DIFF_SOURCE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--cov-sign-eps", type=float, default=1.0e-4)
    parser.add_argument("--h-sign-eps", type=float, default=1.0e-2)
    parser.add_argument("--window1-context-pad", type=int, default=2)
    parser.add_argument("--top-k", type=int, default=6)
    return parser


def stable_sign(value: float, eps: float) -> int:
    if not np.isfinite(value) or abs(value) < eps:
        return 0
    return 1 if value > 0.0 else -1


def family_code(cov_value: float, h_value: float, cov_eps: float, h_eps: float) -> str:
    cov_sign = stable_sign(cov_value, cov_eps)
    h_sign = stable_sign(h_value, h_eps)
    if cov_sign == 0 or h_sign == 0:
        return "transient"
    return f"{'+' if cov_sign > 0 else '-'}{'+' if h_sign > 0 else '-'}"


def metric_label(metric: str) -> str:
    label_map = {
        "delta_lever_before_y_m": "dlever",
        "delta_cov_lgy_py": "dcov_py",
        "delta_cov_lgy_lgy": "dvar_lgy",
        "delta_post_k_lgy_y": "dpost_k",
        "delta_raw_k_lgy_y": "draw_k",
    }
    return label_map.get(metric, metric)


def load_turn_windows(source_probe: Path) -> pd.DataFrame:
    path = source_probe / "turn_windows.csv"
    if not path.exists():
        raise RuntimeError(f"missing turn windows csv: {path}")
    return pd.read_csv(path)


def load_history(history_source: Path) -> pd.DataFrame:
    if not history_source.exists():
        raise RuntimeError(f"missing history source csv: {history_source}")
    return pd.read_csv(history_source)


def load_diff(diff_source: Path) -> pd.DataFrame:
    if not diff_source.exists():
        raise RuntimeError(f"missing diff source csv: {diff_source}")
    return pd.read_csv(diff_source)


def load_case_update_context(source_probe: Path, windows_df: pd.DataFrame, case_id: str) -> pd.DataFrame:
    raw_path = source_probe / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv"
    update_df = load_effective_gnss_pos_update_df(raw_path)
    update_df = update_df.loc[update_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)
    if update_df.empty:
        raise RuntimeError(f"no GNSS_POS updates found for {case_id}")

    rows: list[dict[str, Any]] = []
    for row in update_df.itertuples(index=False):
        prior_cov_pos_lever = parse_matrix_field(getattr(row, "prior_cov_pos_gnss_lever_mat", None), 3, 3)
        prior_cov_lever = parse_matrix_field(getattr(row, "prior_cov_gnss_lever_mat", None), 3, 3)
        h_lever_y = parse_vector_field(getattr(row, "h_gnss_lever_y_vec", None), 3)
        raw_k_lgy = parse_vector_field(getattr(row, "raw_k_gnss_lever_y_vec", None), 3)
        post_k_lgy = parse_vector_field(getattr(row, "k_gnss_lever_y_vec", None), 3)
        num_pos = parse_vector_field(getattr(row, "num_gnss_lever_y_from_pos_vec", None), 3)
        num_att = parse_vector_field(getattr(row, "num_gnss_lever_y_from_att_vec", None), 3)
        num_lever = parse_vector_field(getattr(row, "num_gnss_lever_y_from_gnss_lever_vec", None), 3)

        rows.append(
            {
                "gnss_t": float(row.gnss_t),
                "window_id": None,
                "yaw_rate_deg_s": float(getattr(row, "yaw_rate_deg_s", math.nan)),
                "y_y": float(row.y_y),
                "lever_before_y": float(row.lever_before_y),
                "dx_gnss_lever_y": float(row.dx_gnss_lever_y),
                "cov_lgy_py": float(prior_cov_pos_lever[1, 1]),
                "cov_lgy_lgy": float(prior_cov_lever[1, 1]),
                "h_y_lever_y": float(h_lever_y[1]),
                "raw_k_lgy_y": float(raw_k_lgy[1]),
                "post_k_lgy_y": float(post_k_lgy[1]),
                "num_y_pos_total": float(num_pos[1]),
                "num_y_att_total": float(num_att[1]),
                "num_y_lever_total": float(num_lever[1]),
            }
        )
    context_df = pd.DataFrame(rows).sort_values(by="gnss_t").reset_index(drop=True)
    context_df["window_id"] = assign_window_id(context_df["gnss_t"], windows_df, "window_id")
    return context_df


def build_window1_context(
    history_df: pd.DataFrame,
    cov_eps: float,
    h_eps: float,
    context_pad: int,
) -> pd.DataFrame:
    df = history_df.loc[
        (history_df["case_id"] == REF_CASE_ID) & (history_df["window_id"] == WINDOW1_ID)
    ].copy()
    if df.empty:
        raise RuntimeError(f"missing history rows for {REF_CASE_ID}/{WINDOW1_ID}")
    df = df.sort_values(by="gnss_t").reset_index(drop=True)
    df["stable_family"] = [
        family_code(float(cov), float(h), cov_eps, h_eps)
        for cov, h in zip(df["cov_lgy_py"].to_numpy(dtype=float), df["h_y_lever_y"].to_numpy(dtype=float))
    ]
    df["prev_family"] = df["stable_family"].shift(1).fillna("")
    df["next_family"] = df["stable_family"].shift(-1).fillna("")
    df["same_axis_term_y_mm"] = df["post_k_lgy_y"] * df["y_y"] * 1.0e3

    delta_cols = [
        "cov_lgy_py",
        "cov_lgy_lgy",
        "h_y_lever_y",
        "num_y_pos_total",
        "num_y_att_total",
        "num_y_lever_total",
        "raw_k_lgy_y",
        "post_k_lgy_y",
        "y_y",
        "lever_before_y",
        "dx_gnss_lever_y",
        "same_axis_term_y_mm",
    ]
    for col in delta_cols:
        df[f"delta_{col}"] = df[col].diff()

    target_indices = df.index[df["gnss_t"].isin(WINDOW1_TARGETS)].tolist()
    if not target_indices:
        raise RuntimeError("window_1 targets not found in history source")
    start_idx = max(0, min(target_indices) - context_pad)
    end_idx = min(len(df), max(target_indices) + context_pad + 1)
    context_df = df.iloc[start_idx:end_idx].copy().reset_index(drop=True)
    context_df["is_target_update"] = context_df["gnss_t"].isin(WINDOW1_TARGETS)
    return context_df


def classify_window1_driver(prev_row: pd.Series, row: pd.Series, cov_eps: float, h_eps: float) -> str:
    cov_prev = stable_sign(float(prev_row["cov_lgy_py"]), cov_eps)
    cov_curr = stable_sign(float(row["cov_lgy_py"]), cov_eps)
    h_prev = stable_sign(float(prev_row["h_y_lever_y"]), h_eps)
    h_curr = stable_sign(float(row["h_y_lever_y"]), h_eps)
    cov_flip = cov_prev != 0 and cov_curr != 0 and cov_prev != cov_curr
    h_flip = h_prev != 0 and h_curr != 0 and h_prev != h_curr
    if cov_flip and h_flip:
        return "joint_covariance_and_geometry"
    if cov_flip:
        return "covariance_only"
    if h_flip:
        return "geometry_only"
    return "no_stable_flip"


def build_window1_target_summary(context_df: pd.DataFrame, cov_eps: float, h_eps: float) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    indexed = context_df.set_index("gnss_t")
    for target_t in WINDOW1_TARGETS:
        if target_t not in indexed.index:
            continue
        row = indexed.loc[target_t]
        prev_candidates = context_df.loc[context_df["gnss_t"] < target_t].copy()
        next_candidates = context_df.loc[context_df["gnss_t"] > target_t].copy()
        prev_row = prev_candidates.iloc[-1] if not prev_candidates.empty else row
        next_row = next_candidates.iloc[0] if not next_candidates.empty else row
        driver = classify_window1_driver(prev_row, row, cov_eps, h_eps)
        rows.append(
            {
                "target_gnss_t": float(target_t),
                "history_phase": str(row["history_phase"]),
                "prev_family": str(prev_row["stable_family"]),
                "current_family": str(row["stable_family"]),
                "next_family": str(next_row["stable_family"]),
                "driver_class": driver,
                "yaw_rate_deg_s": float(row["yaw_rate_deg_s"]),
                "y_y": float(row["y_y"]),
                "desired_dx_y_m": float(row["desired_dx_y"]),
                "dx_gnss_lever_y_mm": float(row["dx_gnss_lever_y"]) * 1.0e3,
                "same_axis_term_y_mm": float(row["same_axis_term_y_mm"]),
                "cov_lgy_py": float(row["cov_lgy_py"]),
                "delta_cov_lgy_py": float(row["delta_cov_lgy_py"]),
                "cov_lgy_lgy": float(row["cov_lgy_lgy"]),
                "delta_cov_lgy_lgy": float(row["delta_cov_lgy_lgy"]),
                "h_y_lever_y": float(row["h_y_lever_y"]),
                "delta_h_y_lever_y": float(row["delta_h_y_lever_y"]),
                "num_y_pos_total": float(row["num_y_pos_total"]),
                "delta_num_y_pos_total": float(row["delta_num_y_pos_total"]),
                "num_y_att_total": float(row["num_y_att_total"]),
                "delta_num_y_att_total": float(row["delta_num_y_att_total"]),
                "num_y_lever_total": float(row["num_y_lever_total"]),
                "delta_num_y_lever_total": float(row["delta_num_y_lever_total"]),
                "raw_k_lgy_y": float(row["raw_k_lgy_y"]),
                "delta_raw_k_lgy_y": float(row["delta_raw_k_lgy_y"]),
                "post_k_lgy_y": float(row["post_k_lgy_y"]),
                "delta_post_k_lgy_y": float(row["delta_post_k_lgy_y"]),
                "lever_before_y_m": float(row["lever_before_y"]),
                "delta_lever_before_y_m": float(row["delta_lever_before_y"]),
            }
        )
    return pd.DataFrame(rows).sort_values(by="target_gnss_t").reset_index(drop=True)


def augment_diff_with_context(diff_df: pd.DataFrame, ref_df: pd.DataFrame, alt_df: pd.DataFrame) -> pd.DataFrame:
    merged = diff_df.merge(
        ref_df.add_prefix("ref_"),
        left_on="gnss_t",
        right_on="ref_gnss_t",
        how="left",
    ).merge(
        alt_df.add_prefix("alt_"),
        left_on="gnss_t",
        right_on="alt_gnss_t",
        how="left",
    )
    merged = merged.drop(columns=["ref_gnss_t", "alt_gnss_t"], errors="ignore")
    for prefix in ["ref", "alt"]:
        merged[f"{prefix}_stable_family"] = [
            family_code(float(cov), float(h), 1.0e-4, 1.0e-2)
            for cov, h in zip(
                merged[f"{prefix}_cov_lgy_py"].to_numpy(dtype=float),
                merged[f"{prefix}_h_y_lever_y"].to_numpy(dtype=float),
            )
        ]
    return merged.sort_values(by="gnss_t").reset_index(drop=True)


def build_carryover_interval_summary(diff_df: pd.DataFrame) -> tuple[pd.DataFrame, list[dict[str, Any]]]:
    control_row = diff_df.loc[(diff_df["delta_post_k_lgy_y"].abs() > 1.0e-7)].iloc[0]
    state_mask = (
        (diff_df["delta_lever_before_y_m"].abs() > 1.0e-6)
        | (diff_df["delta_cov_lgy_py"].abs() > 1.0e-7)
        | (diff_df["delta_cov_lgy_lgy"].abs() > 1.0e-7)
    )
    state_row = diff_df.loc[state_mask].iloc[0]
    window5_row = diff_df.loc[diff_df["window_id"] == "turn_window_5"].iloc[0]
    window3_row = diff_df.loc[diff_df["window_id"] == "turn_window_3"].iloc[0]

    interval_specs = [
        {
            "interval_id": "control_divergence_to_state_divergence",
            "start_t": float(control_row["gnss_t"]),
            "end_t": float(state_row["gnss_t"]),
        },
        {
            "interval_id": "state_divergence_to_window5_entry",
            "start_t": float(state_row["gnss_t"]),
            "end_t": float(window5_row["gnss_t"]),
        },
        {
            "interval_id": "window5_entry_to_window3_entry",
            "start_t": float(window5_row["gnss_t"]),
            "end_t": float(window3_row["gnss_t"]),
        },
    ]

    rows: list[dict[str, Any]] = []
    for spec in interval_specs:
        seg = diff_df.loc[(diff_df["gnss_t"] >= spec["start_t"]) & (diff_df["gnss_t"] <= spec["end_t"])].copy()
        counts = seg["window_id"].fillna("non_window").value_counts().to_dict()
        row: dict[str, Any] = {
            "interval_id": str(spec["interval_id"]),
            "start_t": float(spec["start_t"]),
            "end_t": float(spec["end_t"]),
            "updates": int(len(seg)),
            "window_counts": "; ".join(f"{k}:{v}" for k, v in counts.items()),
        }
        for metric in CARRYOVER_METRICS:
            row[f"{metric}_start"] = float(seg[metric].iloc[0])
            row[f"{metric}_end"] = float(seg[metric].iloc[-1])
            row[f"{metric}_min"] = float(seg[metric].min())
            row[f"{metric}_max"] = float(seg[metric].max())
        rows.append(row)
    return pd.DataFrame(rows), interval_specs


def build_carryover_top_updates(
    diff_df: pd.DataFrame,
    interval_specs: list[dict[str, Any]],
    top_k: int,
) -> pd.DataFrame:
    df = diff_df.copy().sort_values(by="gnss_t").reset_index(drop=True)
    for metric in CARRYOVER_METRICS:
        df[f"step_{metric}"] = df[metric].diff()

    rows: list[dict[str, Any]] = []
    for spec in interval_specs:
        seg = df.loc[(df["gnss_t"] >= spec["start_t"]) & (df["gnss_t"] <= spec["end_t"])].copy()
        for metric in CARRYOVER_METRICS:
            step_col = f"step_{metric}"
            ranked = seg.loc[seg[step_col].notna()].copy()
            ranked = ranked.reindex(ranked[step_col].abs().sort_values(ascending=False).index).head(top_k)
            for rank, row in enumerate(ranked.itertuples(index=False), start=1):
                rows.append(
                    {
                        "interval_id": str(spec["interval_id"]),
                        "metric": metric,
                        "metric_label": metric_label(metric),
                        "rank": int(rank),
                        "gnss_t": float(row.gnss_t),
                        "window_id": "" if pd.isna(row.window_id) else str(row.window_id),
                        "ref_stable_family": str(row.ref_stable_family),
                        "alt_stable_family": str(row.alt_stable_family),
                        "ref_yaw_rate_deg_s": float(row.ref_yaw_rate_deg_s),
                        "ref_y_y": float(row.ref_y_y),
                        "step_value": float(getattr(row, step_col)),
                        "current_value": float(getattr(row, metric)),
                        "ref_lever_before_y_m": float(row.ref_lever_before_y),
                        "alt_lever_before_y_m": float(row.alt_lever_before_y),
                        "ref_raw_k_lgy_y": float(row.ref_raw_k_lgy_y),
                        "alt_raw_k_lgy_y": float(row.alt_raw_k_lgy_y),
                        "ref_num_y_pos_total": float(row.ref_num_y_pos_total),
                        "alt_num_y_pos_total": float(row.alt_num_y_pos_total),
                        "ref_num_y_lever_total": float(row.ref_num_y_lever_total),
                        "alt_num_y_lever_total": float(row.alt_num_y_lever_total),
                    }
                )
    return pd.DataFrame(rows).sort_values(by=["interval_id", "metric", "rank"]).reset_index(drop=True)


def top_update_line(top_df: pd.DataFrame, interval_id: str, metric: str) -> str:
    sub = top_df.loc[(top_df["interval_id"] == interval_id) & (top_df["metric"] == metric)].copy()
    if sub.empty:
        return ""
    row = sub.iloc[0]
    yaw_part = (
        f", `yaw={row.ref_yaw_rate_deg_s:+.3f}` deg/s"
        if np.isfinite(float(row.ref_yaw_rate_deg_s))
        else ""
    )
    return (
        f"`{metric_label(metric)}` top reinforcement at `t={row.gnss_t:.0f}`"
        + (f" (`{row.window_id}`)" if row.window_id else " (`non_window`)")
        + f": `step={row.step_value:+.6f}`, `current={row.current_value:+.6f}`, "
        f"`ref/alt family={row.ref_stable_family}/{row.alt_stable_family}`{yaw_part}."
    )


def write_summary(
    summary_path: Path,
    window1_targets_df: pd.DataFrame,
    interval_summary_df: pd.DataFrame,
    top_updates_df: pd.DataFrame,
) -> None:
    lines: list[str] = []
    lines.append("# turn-window lgy key-update audit")
    lines.append("")
    lines.append("## Core reading")
    lines.append("- `window_1` keeps the same focused reference case as the earlier history analysis; the target here is not a window-average metric but the exact update-level family transitions at `529531/529532/529536 s`.")
    lines.append("- `driver_class` is a strict sign-change classification between consecutive GNSS_POS updates: `geometry_only` means only `h_y_lever_y` changed stable sign, `covariance_only` means only `cov(lgy,p_y)` changed sign, and `joint_covariance_and_geometry` means both flipped together.")
    lines.append("- The carry-over section ranks step changes in the already-established case diff (`reference` vs `positive-turn lgy_from_y=0`) so the surviving divergence can be traced to concrete updates instead of only checkpoint snapshots.")
    lines.append("")
    lines.append("## window_1 Key Transition Updates")
    for row in window1_targets_df.itertuples(index=False):
        lines.append(
            f"- `t={row.target_gnss_t:.0f}` (`{row.history_phase}`): "
            f"`family {row.prev_family} -> {row.current_family} -> {row.next_family}`, "
            f"`driver={row.driver_class}`, `yaw={row.yaw_rate_deg_s:+.3f}` deg/s, "
            f"`cov_py={row.cov_lgy_py:+.6f}` (`d={row.delta_cov_lgy_py:+.6f}`), "
            f"`h_y={row.h_y_lever_y:+.6f}` (`d={row.delta_h_y_lever_y:+.6f}`), "
            f"`num_y(pos/att/lever)={row.num_y_pos_total:+.6f}/{row.num_y_att_total:+.6f}/{row.num_y_lever_total:+.6f}`, "
            f"`raw_k={row.raw_k_lgy_y:+.6f}`, `same_axis_term={row.same_axis_term_y_mm:+.3f} mm`, "
            f"`innovation_y={row.y_y:+.6f}`, `dx_lgy={row.dx_gnss_lever_y_mm:+.3f} mm`."
        )
    lines.append("")
    lines.append("## Carry-Over Survival Intervals")
    for row in interval_summary_df.itertuples(index=False):
        lines.append(
            f"- `{row.interval_id}` (`t={row.start_t:.0f}->{row.end_t:.0f}`, `updates={row.updates}`, `{row.window_counts}`): "
            f"`dlever end={row.delta_lever_before_y_m_end * 1.0e3:+.3f} mm`, "
            f"`dcov_py end={row.delta_cov_lgy_py_end:+.6f}`, `dvar_lgy end={row.delta_cov_lgy_lgy_end:+.6f}`, "
            f"`dpost_k end={row.delta_post_k_lgy_y_end:+.6f}`. "
            f"{top_update_line(top_updates_df, row.interval_id, 'delta_lever_before_y_m')} "
            f"{top_update_line(top_updates_df, row.interval_id, 'delta_cov_lgy_py')} "
            f"{top_update_line(top_updates_df, row.interval_id, 'delta_post_k_lgy_y')}"
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probe = (REPO_ROOT / args.source_probe).resolve()
    history_source = (REPO_ROOT / args.history_source).resolve()
    diff_source = (REPO_ROOT / args.diff_source).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()

    if not source_probe.exists():
        raise RuntimeError(f"missing source probe dir: {source_probe}")

    ensure_dir(output_dir)

    windows_df = load_turn_windows(source_probe)
    history_df = load_history(history_source)
    diff_df = load_diff(diff_source)
    ref_context_df = load_case_update_context(source_probe, windows_df, REF_CASE_ID)
    alt_context_df = load_case_update_context(source_probe, windows_df, ALT_CASE_ID)

    window1_context_df = build_window1_context(
        history_df,
        args.cov_sign_eps,
        args.h_sign_eps,
        args.window1_context_pad,
    )
    window1_targets_df = build_window1_target_summary(window1_context_df, args.cov_sign_eps, args.h_sign_eps)

    diff_aug_df = augment_diff_with_context(diff_df, ref_context_df, alt_context_df)
    interval_summary_df, interval_specs = build_carryover_interval_summary(diff_aug_df)
    top_updates_df = build_carryover_top_updates(diff_aug_df, interval_specs, args.top_k)

    context_out = output_dir / "window1_transition_context.csv"
    target_out = output_dir / "window1_transition_key_updates.csv"
    interval_summary_out = output_dir / "carryover_interval_summary.csv"
    top_updates_out = output_dir / "carryover_top_updates.csv"
    summary_out = output_dir / "summary.md"
    manifest_out = output_dir / "manifest.json"

    window1_context_df.to_csv(context_out, index=False, encoding="utf-8-sig")
    window1_targets_df.to_csv(target_out, index=False, encoding="utf-8-sig")
    interval_summary_df.to_csv(interval_summary_out, index=False, encoding="utf-8-sig")
    top_updates_df.to_csv(top_updates_out, index=False, encoding="utf-8-sig")
    write_summary(summary_out, window1_targets_df, interval_summary_df, top_updates_df)

    manifest = {
        "exp_id": str(args.exp_id),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probe": rel_from_root(source_probe, REPO_ROOT),
        "history_source": rel_from_root(history_source, REPO_ROOT),
        "diff_source": rel_from_root(diff_source, REPO_ROOT),
        "reference_case_id": REF_CASE_ID,
        "comparison_case_id": ALT_CASE_ID,
        "window1_targets": WINDOW1_TARGETS,
        "top_k": int(args.top_k),
        "artifacts": {
            "window1_transition_context_csv": rel_from_root(context_out, REPO_ROOT),
            "window1_transition_key_updates_csv": rel_from_root(target_out, REPO_ROOT),
            "carryover_interval_summary_csv": rel_from_root(interval_summary_out, REPO_ROOT),
            "carryover_top_updates_csv": rel_from_root(top_updates_out, REPO_ROOT),
            "summary_md": rel_from_root(summary_out, REPO_ROOT),
        },
    }
    manifest_out.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
