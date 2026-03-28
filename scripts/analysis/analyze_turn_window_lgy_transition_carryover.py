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


EXP_ID_DEFAULT = "EXP-20260319-data2-turn-window-lgy-transition-carryover-r1"
SOURCE_PROBE_DEFAULT = Path("output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1")
HISTORY_SOURCE_DEFAULT = Path("output/data2_turn_window_lgy_history_analysis/lgy_history_per_update.csv")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_lgy_transition_carryover_analysis")
REF_CASE_ID = "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref"
ALT_CASE_ID = "joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0"
POSITIVE_WINDOWS = ["turn_window_1", "turn_window_4", "turn_window_5"]
CHECKPOINT_WINDOW_IDS = ["turn_window_4", "turn_window_5", "turn_window_3"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze stable sign-family transitions for lgy and trace full carry-over from positive-turn suppression to turn_window_3."
    )
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--source-probe", type=Path, default=SOURCE_PROBE_DEFAULT)
    parser.add_argument("--history-source", type=Path, default=HISTORY_SOURCE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--cov-sign-eps", type=float, default=1.0e-4)
    parser.add_argument("--h-sign-eps", type=float, default=1.0e-2)
    parser.add_argument("--state-div-lever-eps-m", type=float, default=1.0e-6)
    parser.add_argument("--state-div-cov-eps", type=float, default=1.0e-7)
    parser.add_argument("--control-div-k-eps", type=float, default=1.0e-7)
    return parser


def load_case_meta(source_probe: Path) -> dict[str, dict[str, Any]]:
    manifest = json.loads((source_probe / "manifest.json").read_text(encoding="utf-8"))
    return {str(row["case_id"]): row for row in manifest.get("case_rows", [])}


def load_turn_windows(source_probe: Path) -> pd.DataFrame:
    windows_path = source_probe / "turn_windows.csv"
    if not windows_path.exists():
        raise RuntimeError(f"missing turn windows csv: {windows_path}")
    return pd.read_csv(windows_path)


def stable_sign(value: float, eps: float) -> int:
    if not np.isfinite(value) or abs(value) < eps:
        return 0
    return 1 if value > 0.0 else -1


def family_code(cov_value: float, h_value: float, cov_eps: float, h_eps: float) -> str:
    cov_sign = stable_sign(cov_value, cov_eps)
    h_sign = stable_sign(h_value, h_eps)
    if cov_sign == 0 or h_sign == 0:
        return "transient"
    cov_label = "+" if cov_sign > 0 else "-"
    h_label = "+" if h_sign > 0 else "-"
    return f"{cov_label}{h_label}"


def augment_history_families(history_df: pd.DataFrame, cov_eps: float, h_eps: float) -> pd.DataFrame:
    df = history_df.copy()
    df["stable_family"] = [
        family_code(float(cov), float(h), cov_eps, h_eps)
        for cov, h in zip(df["cov_lgy_py"].to_numpy(dtype=float), df["h_y_lever_y"].to_numpy(dtype=float))
    ]
    return df


def compress_family_segments(df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for window_id, group in df.groupby("window_id", sort=False):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        start_idx = 0
        current_family = str(group.loc[0, "stable_family"])
        for idx in range(1, len(group) + 1):
            is_break = idx == len(group) or str(group.loc[idx, "stable_family"]) != current_family
            if not is_break:
                continue
            segment = group.iloc[start_idx:idx]
            rows.append(
                {
                    "window_id": str(window_id),
                    "family_id": current_family,
                    "start_gnss_t": float(segment["gnss_t"].iloc[0]),
                    "end_gnss_t": float(segment["gnss_t"].iloc[-1]),
                    "start_phase": str(segment["history_phase"].iloc[0]),
                    "end_phase": str(segment["history_phase"].iloc[-1]),
                    "updates": int(len(segment)),
                    "mean_cov_lgy_py": float(np.nanmean(segment["cov_lgy_py"].to_numpy(dtype=float))),
                    "mean_h_y_lever_y": float(np.nanmean(segment["h_y_lever_y"].to_numpy(dtype=float))),
                    "mean_raw_k_lgy_y": float(np.nanmean(segment["raw_k_lgy_y"].to_numpy(dtype=float))),
                }
            )
            if idx < len(group):
                start_idx = idx
                current_family = str(group.loc[idx, "stable_family"])
    return pd.DataFrame(rows)


def first_family_flip(df: pd.DataFrame, column: str, eps: float) -> tuple[float, str, float] | None:
    prev_sign: int | None = None
    for row in df.itertuples(index=False):
        sign = stable_sign(float(getattr(row, column)), eps)
        if prev_sign is not None and sign != prev_sign and sign != 0:
            return float(row.gnss_t), str(row.history_phase), float(getattr(row, column))
        if sign != 0:
            prev_sign = sign
    return None


def summarize_positive_windows(history_df: pd.DataFrame, segments_df: pd.DataFrame, cov_eps: float, h_eps: float) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    ref_df = history_df.loc[history_df["window_id"].isin(POSITIVE_WINDOWS)].copy()
    for window_id, group in ref_df.groupby("window_id", sort=False):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        seg_group = segments_df.loc[segments_df["window_id"] == window_id].copy().reset_index(drop=True)
        pre_df = group.loc[group["history_phase"] == "pre_window"].copy()
        in_df = group.loc[group["history_phase"] == "in_window"].copy()
        path_labels = [f"{row.family_id}({row.updates})" for row in seg_group.itertuples(index=False)]
        entry_row = in_df.iloc[0]
        second_in_row = in_df.iloc[1] if len(in_df) > 1 else in_df.iloc[0]
        last_row = in_df.iloc[-1]
        rows.append(
            {
                "window_id": str(window_id),
                "segment_path": " -> ".join(path_labels),
                "dominant_pre_family": str(pre_df["stable_family"].mode(dropna=False).iloc[0]),
                "entry_family": str(entry_row["stable_family"]),
                "second_in_family": str(second_in_row["stable_family"]),
                "last_in_family": str(last_row["stable_family"]),
                "first_cov_flip_t": first_family_flip(group, "cov_lgy_py", cov_eps)[0]
                if first_family_flip(group, "cov_lgy_py", cov_eps) is not None
                else math.nan,
                "first_cov_flip_phase": first_family_flip(group, "cov_lgy_py", cov_eps)[1]
                if first_family_flip(group, "cov_lgy_py", cov_eps) is not None
                else "",
                "first_h_flip_t": first_family_flip(group, "h_y_lever_y", h_eps)[0]
                if first_family_flip(group, "h_y_lever_y", h_eps) is not None
                else math.nan,
                "first_h_flip_phase": first_family_flip(group, "h_y_lever_y", h_eps)[1]
                if first_family_flip(group, "h_y_lever_y", h_eps) is not None
                else "",
                "entry_cov_lgy_py": float(entry_row["cov_lgy_py"]),
                "entry_h_y_lever_y": float(entry_row["h_y_lever_y"]),
                "second_in_cov_lgy_py": float(second_in_row["cov_lgy_py"]),
                "second_in_h_y_lever_y": float(second_in_row["h_y_lever_y"]),
                "last_in_cov_lgy_py": float(last_row["cov_lgy_py"]),
                "last_in_h_y_lever_y": float(last_row["h_y_lever_y"]),
                "entry_raw_k_lgy_y": float(entry_row["raw_k_lgy_y"]),
                "second_in_raw_k_lgy_y": float(second_in_row["raw_k_lgy_y"]),
                "last_in_raw_k_lgy_y": float(last_row["raw_k_lgy_y"]),
            }
        )
    return pd.DataFrame(rows).sort_values(by="window_id").reset_index(drop=True)


def load_case_updates(source_probe: Path, case_id: str) -> pd.DataFrame:
    raw_path = source_probe / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv"
    update_df = load_effective_gnss_pos_update_df(raw_path)
    update_df = update_df.loc[update_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)
    if update_df.empty:
        raise RuntimeError(f"no GNSS_POS updates found for {case_id}")
    return update_df


def build_full_case_diff(source_probe: Path, windows_df: pd.DataFrame, ref_case_id: str, alt_case_id: str) -> pd.DataFrame:
    ref_df = load_case_updates(source_probe, ref_case_id)
    alt_df = load_case_updates(source_probe, alt_case_id)
    merged = ref_df.merge(alt_df, on="gnss_t", how="inner", suffixes=("_ref", "_alt"))
    rows: list[dict[str, Any]] = []
    for row in merged.itertuples(index=False):
        cov_pos_ref = parse_matrix_field(getattr(row, "prior_cov_pos_gnss_lever_mat_ref", None), 3, 3)
        cov_pos_alt = parse_matrix_field(getattr(row, "prior_cov_pos_gnss_lever_mat_alt", None), 3, 3)
        cov_lev_ref = parse_matrix_field(getattr(row, "prior_cov_gnss_lever_mat_ref", None), 3, 3)
        cov_lev_alt = parse_matrix_field(getattr(row, "prior_cov_gnss_lever_mat_alt", None), 3, 3)
        h_ref = parse_vector_field(getattr(row, "h_gnss_lever_y_vec_ref", None), 3)
        h_alt = parse_vector_field(getattr(row, "h_gnss_lever_y_vec_alt", None), 3)
        k_ref = parse_vector_field(getattr(row, "k_gnss_lever_y_vec_ref", None), 3)
        k_alt = parse_vector_field(getattr(row, "k_gnss_lever_y_vec_alt", None), 3)
        raw_k_ref = parse_vector_field(getattr(row, "raw_k_gnss_lever_y_vec_ref", None), 3)
        raw_k_alt = parse_vector_field(getattr(row, "raw_k_gnss_lever_y_vec_alt", None), 3)
        num_pos_ref = parse_vector_field(getattr(row, "num_gnss_lever_y_from_pos_vec_ref", None), 3)
        num_pos_alt = parse_vector_field(getattr(row, "num_gnss_lever_y_from_pos_vec_alt", None), 3)
        num_lever_ref = parse_vector_field(getattr(row, "num_gnss_lever_y_from_gnss_lever_vec_ref", None), 3)
        num_lever_alt = parse_vector_field(getattr(row, "num_gnss_lever_y_from_gnss_lever_vec_alt", None), 3)
        rows.append(
            {
                "gnss_t": float(row.gnss_t),
                "window_id": None,
                "delta_lever_before_y_m": float(row.lever_before_y_alt - row.lever_before_y_ref),
                "delta_dx_gnss_lever_y_m": float(row.dx_gnss_lever_y_alt - row.dx_gnss_lever_y_ref),
                "delta_cov_lgy_py": float(cov_pos_alt[1, 1] - cov_pos_ref[1, 1]),
                "delta_cov_lgy_lgy": float(cov_lev_alt[1, 1] - cov_lev_ref[1, 1]),
                "delta_h_y_lever_y": float(h_alt[1] - h_ref[1]),
                "delta_post_k_lgy_y": float(k_alt[1] - k_ref[1]),
                "delta_raw_k_lgy_y": float(raw_k_alt[1] - raw_k_ref[1]),
                "delta_num_y_pos_total": float(num_pos_alt[1] - num_pos_ref[1]),
                "delta_num_y_lever_total": float(num_lever_alt[1] - num_lever_ref[1]),
            }
        )
    diff_df = pd.DataFrame(rows).sort_values(by="gnss_t").reset_index(drop=True)
    diff_df["window_id"] = assign_window_id(diff_df["gnss_t"], windows_df, "window_id")
    return diff_df


def find_first_control_divergence(diff_df: pd.DataFrame, control_div_k_eps: float) -> pd.Series:
    mask = diff_df["delta_post_k_lgy_y"].abs() > control_div_k_eps
    if not mask.any():
        raise RuntimeError("failed to find first control divergence")
    return diff_df.loc[mask].iloc[0]


def find_first_state_divergence(
    diff_df: pd.DataFrame,
    state_div_lever_eps_m: float,
    state_div_cov_eps: float,
) -> pd.Series:
    mask = (
        (diff_df["delta_lever_before_y_m"].abs() > state_div_lever_eps_m)
        | (diff_df["delta_cov_lgy_py"].abs() > state_div_cov_eps)
        | (diff_df["delta_cov_lgy_lgy"].abs() > state_div_cov_eps)
    )
    if not mask.any():
        raise RuntimeError("failed to find first state divergence")
    return diff_df.loc[mask].iloc[0]


def checkpoint_row(label: str, row: pd.Series) -> dict[str, Any]:
    return {
        "checkpoint_id": label,
        "gnss_t": float(row["gnss_t"]),
        "window_id": "" if pd.isna(row["window_id"]) else str(row["window_id"]),
        "delta_lever_before_y_mm": float(row["delta_lever_before_y_m"]) * 1.0e3,
        "delta_cov_lgy_py": float(row["delta_cov_lgy_py"]),
        "delta_cov_lgy_lgy": float(row["delta_cov_lgy_lgy"]),
        "delta_h_y_lever_y": float(row["delta_h_y_lever_y"]),
        "delta_post_k_lgy_y": float(row["delta_post_k_lgy_y"]),
        "delta_raw_k_lgy_y": float(row["delta_raw_k_lgy_y"]),
        "delta_dx_gnss_lever_y_mm": float(row["delta_dx_gnss_lever_y_m"]) * 1.0e3,
        "delta_num_y_pos_total": float(row["delta_num_y_pos_total"]),
        "delta_num_y_lever_total": float(row["delta_num_y_lever_total"]),
    }


def build_carryover_checkpoints(
    diff_df: pd.DataFrame,
    windows_df: pd.DataFrame,
    control_div_k_eps: float,
    state_div_lever_eps_m: float,
    state_div_cov_eps: float,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    first_control = find_first_control_divergence(diff_df, control_div_k_eps)
    first_state = find_first_state_divergence(diff_df, state_div_lever_eps_m, state_div_cov_eps)
    rows.append(checkpoint_row("first_control_divergence", first_control))
    rows.append(checkpoint_row("first_state_divergence", first_state))

    for window_id in CHECKPOINT_WINDOW_IDS:
        sub = diff_df.loc[diff_df["window_id"] == window_id].copy().sort_values(by="gnss_t").reset_index(drop=True)
        if sub.empty:
            continue
        rows.append(checkpoint_row(f"{window_id}_entry", sub.iloc[0]))
        rows.append(checkpoint_row(f"{window_id}_last", sub.iloc[-1]))
        if window_id == "turn_window_3":
            window_row = windows_df.loc[windows_df["window_id"] == window_id].iloc[0]
            pre_sub = diff_df.loc[diff_df["gnss_t"] < float(window_row.start_t)].copy().sort_values(by="gnss_t").reset_index(drop=True)
            if not pre_sub.empty:
                rows.append(checkpoint_row(f"{window_id}_last_pre", pre_sub.iloc[-1]))
    return pd.DataFrame(rows)


def write_summary(
    summary_path: Path,
    pair_df: pd.DataFrame,
    checkpoint_df: pd.DataFrame,
) -> None:
    lines: list[str] = []
    lines.append("# turn-window lgy transition and carry-over analysis")
    lines.append("")
    lines.append("## Core reading")
    lines.append("- `stable_family` is defined by the sign pair of `cov(lgy,p_y)` and `h_y_lever_y`, with small magnitudes collapsed to `transient` so brief near-zero jitter does not dominate the path summary.")
    lines.append("- `first_control_divergence` means the first GNSS_POS update where the applied `post K(lgy, meas_y)` differs between the reference and `positive-turn lgy_from_y=0`; `first_state_divergence` is the first later update where `lever_before_y` or prior covariance already differs.")
    lines.append("")
    lines.append("## Positive-Turn Family Paths")
    for row in pair_df.itertuples(index=False):
        lines.append(
            f"- `{row.window_id}`: path `{row.segment_path}`, "
            f"`entry/second/last={row.entry_family}/{row.second_in_family}/{row.last_in_family}`, "
            f"`first_cov_flip={row.first_cov_flip_t:.0f} ({row.first_cov_flip_phase})`, "
            f"`first_h_flip={row.first_h_flip_t:.0f} ({row.first_h_flip_phase})`, "
            f"`entry cov/h={row.entry_cov_lgy_py:.6f}/{row.entry_h_y_lever_y:.6f}`, "
            f"`second_in cov/h={row.second_in_cov_lgy_py:.6f}/{row.second_in_h_y_lever_y:.6f}`, "
            f"`last_in cov/h={row.last_in_cov_lgy_py:.6f}/{row.last_in_h_y_lever_y:.6f}`."
        )
    lines.append("")
    lines.append("## Carry-Over Checkpoints")
    for row in checkpoint_df.itertuples(index=False):
        lines.append(
            f"- `{row.checkpoint_id}` at `t={row.gnss_t:.0f}`"
            + (f" (`{row.window_id}`)" if row.window_id else "")
            + f": `dlever={row.delta_lever_before_y_mm:+.3f} mm`, "
            f"`dcov_py={row.delta_cov_lgy_py:+.6f}`, `dvar_lgy={row.delta_cov_lgy_lgy:+.6f}`, "
            f"`dh={row.delta_h_y_lever_y:+.6f}`, `dpost_k={row.delta_post_k_lgy_y:+.6f}`, "
            f"`draw_k={row.delta_raw_k_lgy_y:+.6f}`, `ddx={row.delta_dx_gnss_lever_y_mm:+.3f} mm`."
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probe = (REPO_ROOT / args.source_probe).resolve()
    history_source = (REPO_ROOT / args.history_source).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()

    if not source_probe.exists():
        raise RuntimeError(f"missing source probe dir: {source_probe}")
    if not history_source.exists():
        raise RuntimeError(f"missing history source csv: {history_source}")

    ensure_dir(output_dir)
    windows_df = load_turn_windows(source_probe)
    history_df = pd.read_csv(history_source)
    history_df = history_df.loc[
        (history_df["case_id"] == REF_CASE_ID) & history_df["window_id"].isin(POSITIVE_WINDOWS)
    ].copy()
    history_df = augment_history_families(history_df, args.cov_sign_eps, args.h_sign_eps)
    segments_df = compress_family_segments(history_df)
    pair_df = summarize_positive_windows(history_df, segments_df, args.cov_sign_eps, args.h_sign_eps)

    diff_df = build_full_case_diff(source_probe, windows_df, REF_CASE_ID, ALT_CASE_ID)
    checkpoint_df = build_carryover_checkpoints(
        diff_df,
        windows_df,
        args.control_div_k_eps,
        args.state_div_lever_eps_m,
        args.state_div_cov_eps,
    )

    segments_out = output_dir / "positive_turn_family_segments.csv"
    pair_out = output_dir / "positive_turn_pair_comparison.csv"
    diff_out = output_dir / "full_case_diff_per_update.csv"
    checkpoint_out = output_dir / "carryover_checkpoint_summary.csv"
    summary_out = output_dir / "summary.md"
    manifest_out = output_dir / "manifest.json"

    segments_df.to_csv(segments_out, index=False, encoding="utf-8-sig")
    pair_df.to_csv(pair_out, index=False, encoding="utf-8-sig")
    diff_df.to_csv(diff_out, index=False, encoding="utf-8-sig")
    checkpoint_df.to_csv(checkpoint_out, index=False, encoding="utf-8-sig")
    write_summary(summary_out, pair_df, checkpoint_df)

    manifest = {
        "exp_id": str(args.exp_id),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probe": rel_from_root(source_probe, REPO_ROOT),
        "history_source": rel_from_root(history_source, REPO_ROOT),
        "reference_case_id": REF_CASE_ID,
        "comparison_case_id": ALT_CASE_ID,
        "cov_sign_eps": float(args.cov_sign_eps),
        "h_sign_eps": float(args.h_sign_eps),
        "artifacts": {
            "positive_turn_family_segments_csv": rel_from_root(segments_out, REPO_ROOT),
            "positive_turn_pair_comparison_csv": rel_from_root(pair_out, REPO_ROOT),
            "full_case_diff_per_update_csv": rel_from_root(diff_out, REPO_ROOT),
            "carryover_checkpoint_summary_csv": rel_from_root(checkpoint_out, REPO_ROOT),
            "summary_md": rel_from_root(summary_out, REPO_ROOT),
        },
    }
    manifest_out.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
