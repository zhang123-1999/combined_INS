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

from scripts.analysis.analyze_turn_vs_straight_xy_decoupling import (  # noqa: E402
    assign_window_id,
    select_matched_straight_windows,
)
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    build_truth_reference,
    json_safe,
)
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (  # noqa: E402
    MIN_SPEED_M_S,
    MIN_WINDOW_SEPARATION_S,
    OUTPUT_DIR_DEFAULT,
    POS_PATH_DEFAULT,
    ROLLING_WINDOW_S,
    load_pos_dataframe,
    load_effective_gnss_pos_update_df,
    parse_vector_field,
    select_turn_windows,
)


EPS = 1.0e-12


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect whether turn-window xy lever updates fail because of wrong single-step sign or later reversal."
    )
    parser.add_argument("--probe-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    return parser


def sign_match(actual: float, desired: float) -> float:
    if not np.isfinite(actual) or not np.isfinite(desired):
        return math.nan
    if abs(actual) <= EPS or abs(desired) <= EPS:
        return math.nan
    return float(math.copysign(1.0, actual) == math.copysign(1.0, desired))


def load_matched_windows(probe_dir: Path, pos_path: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    turn_windows_path = probe_dir / "turn_windows.csv"
    if not turn_windows_path.exists():
        raise RuntimeError(f"missing turn windows: {turn_windows_path}")
    turn_windows_df = pd.read_csv(turn_windows_path)

    straight_windows_path = probe_dir / "matched_straight_windows.csv"
    if straight_windows_path.exists():
        straight_windows_df = pd.read_csv(straight_windows_path)
        return turn_windows_df, straight_windows_df

    pos_df = load_pos_dataframe(pos_path)
    _, turn_series_df = select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=ROLLING_WINDOW_S,
        top_k=len(turn_windows_df),
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    straight_windows_df = select_matched_straight_windows(
        series_df=turn_series_df,
        turn_windows_df=turn_windows_df,
        rolling_window_s=ROLLING_WINDOW_S,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    straight_windows_df.to_csv(straight_windows_path, index=False, encoding="utf-8-sig")
    return turn_windows_df, straight_windows_df


def load_raw_gain_rows(probe_dir: Path, case_ids: list[str]) -> pd.DataFrame:
    rows: list[pd.DataFrame] = []
    for case_id in case_ids:
        raw_path = probe_dir / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv"
        if not raw_path.exists():
            raise RuntimeError(f"missing raw GNSS update log for {case_id}: {raw_path}")
        raw_df = load_effective_gnss_pos_update_df(raw_path)
        raw_df = raw_df.loc[raw_df["tag"] == "GNSS_POS"].copy()
        raw_df["case_id"] = case_id
        rows.append(
            raw_df[
                [
                    "case_id",
                    "gnss_t",
                    "dx_gnss_lever_x",
                    "dx_gnss_lever_y",
                    "k_gnss_lever_x_vec",
                    "k_gnss_lever_y_vec",
                ]
            ]
        )
    if not rows:
        raise RuntimeError("no case rows found for raw sign analysis")
    return pd.concat(rows, ignore_index=True)


def enrich_step_metrics(merged_df: pd.DataFrame, truth_lever_xy: dict[str, float]) -> pd.DataFrame:
    out = merged_df.copy().sort_values(by=["case_id", "gnss_t"]).reset_index(drop=True)
    innovation = out[["y_x", "y_y", "y_z"]].to_numpy(dtype=float)
    for axis_idx, axis in enumerate(["x", "y"]):
        k_rows = np.vstack(
            out[f"k_gnss_lever_{axis}_vec"].apply(lambda value: parse_vector_field(value, 3)).to_numpy()
        )
        dx = out[f"dx_gnss_lever_{axis}"].to_numpy(dtype=float)
        before = out[f"lever_before_{axis}"].to_numpy(dtype=float)
        after = out[f"lever_after_{axis}"].to_numpy(dtype=float)
        truth = float(truth_lever_xy[axis])
        desired = truth - before
        same_axis = k_rows[:, axis_idx] * innovation[:, axis_idx]
        cross_axis = dx - same_axis
        out[f"desired_dx_{axis}"] = desired
        out[f"err_before_{axis}"] = before - truth
        out[f"err_after_{axis}"] = after - truth
        out[f"step_improve_{axis}_m"] = np.abs(before - truth) - np.abs(after - truth)
        out[f"dx_sign_match_{axis}"] = [sign_match(a, d) for a, d in zip(dx, desired)]
        out[f"same_axis_sign_match_{axis}"] = [sign_match(a, d) for a, d in zip(same_axis, desired)]
        out[f"same_axis_dx_align_{axis}"] = [sign_match(a, b) for a, b in zip(same_axis, dx)]
        out[f"same_axis_contrib_{axis}"] = same_axis
        out[f"cross_axis_contrib_{axis}"] = cross_axis
        out[f"same_axis_abs_share_{axis}"] = np.abs(same_axis) / np.maximum(np.abs(dx), EPS)
        out[f"cross_axis_abs_share_{axis}"] = np.abs(cross_axis) / np.maximum(np.abs(dx), EPS)
    return out


def summarize_phase_windows(df: pd.DataFrame, window_col: str, phase: str) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    subset = df.loc[df[window_col].notna()].copy()
    for (case_id, window_id), group in subset.groupby(["case_id", window_col], sort=False):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        for axis in ["x", "y"]:
            step = group[f"step_improve_{axis}_m"].to_numpy(dtype=float)
            first_err_before = float(abs(group.iloc[0][f"err_before_{axis}"]))
            after_err = np.abs(group[f"err_after_{axis}"].to_numpy(dtype=float))
            total_improve = first_err_before - float(after_err[-1])
            best_prefix_improve = first_err_before - float(np.min(after_err))
            rows.append(
                {
                    "case_id": str(case_id),
                    "phase": phase,
                    "window_id": str(window_id),
                    "axis": axis,
                    "updates": int(len(group)),
                    "mean_abs_y": float(group[f"y_{axis}"].abs().mean()),
                    "positive_step_rate": float(np.mean(step > 0.0)),
                    "dx_sign_match_rate": float(np.nanmean(group[f"dx_sign_match_{axis}"].to_numpy(dtype=float))),
                    "same_axis_sign_match_rate": float(
                        np.nanmean(group[f"same_axis_sign_match_{axis}"].to_numpy(dtype=float))
                    ),
                    "same_axis_dx_align_rate": float(
                        np.nanmean(group[f"same_axis_dx_align_{axis}"].to_numpy(dtype=float))
                    ),
                    "mean_step_improve_m": float(np.mean(step)),
                    "first_step_improve_m": float(step[0]),
                    "last_step_improve_m": float(step[-1]),
                    "total_window_improve_m": total_improve,
                    "best_prefix_improve_m": best_prefix_improve,
                    "reversal_gap_m": best_prefix_improve - total_improve,
                    "has_temporary_good_then_bad": float(best_prefix_improve > 0.0 and total_improve < 0.0),
                    "mean_same_axis_abs_share": float(
                        np.nanmean(group[f"same_axis_abs_share_{axis}"].to_numpy(dtype=float))
                    ),
                    "mean_cross_axis_abs_share": float(
                        np.nanmean(group[f"cross_axis_abs_share_{axis}"].to_numpy(dtype=float))
                    ),
                    "mean_same_axis_contrib_m": float(
                        np.nanmean(group[f"same_axis_contrib_{axis}"].to_numpy(dtype=float))
                    ),
                    "mean_cross_axis_contrib_m": float(
                        np.nanmean(group[f"cross_axis_contrib_{axis}"].to_numpy(dtype=float))
                    ),
                }
            )
    return pd.DataFrame(rows)


def summarize_case_phase(window_summary_df: pd.DataFrame) -> pd.DataFrame:
    return (
        window_summary_df.groupby(["case_id", "phase", "axis"], as_index=False)
        .agg(
            windows=("window_id", "count"),
            updates=("updates", "sum"),
            mean_abs_y=("mean_abs_y", "mean"),
            mean_positive_step_rate=("positive_step_rate", "mean"),
            mean_dx_sign_match_rate=("dx_sign_match_rate", "mean"),
            mean_same_axis_sign_match_rate=("same_axis_sign_match_rate", "mean"),
            mean_same_axis_dx_align_rate=("same_axis_dx_align_rate", "mean"),
            mean_step_improve_m=("mean_step_improve_m", "mean"),
            mean_first_step_improve_m=("first_step_improve_m", "mean"),
            mean_last_step_improve_m=("last_step_improve_m", "mean"),
            mean_total_window_improve_m=("total_window_improve_m", "mean"),
            mean_best_prefix_improve_m=("best_prefix_improve_m", "mean"),
            mean_reversal_gap_m=("reversal_gap_m", "mean"),
            temporary_good_then_bad_rate=("has_temporary_good_then_bad", "mean"),
            mean_same_axis_abs_share=("mean_same_axis_abs_share", "mean"),
            mean_cross_axis_abs_share=("mean_cross_axis_abs_share", "mean"),
            mean_same_axis_contrib_m=("mean_same_axis_contrib_m", "mean"),
            mean_cross_axis_contrib_m=("mean_cross_axis_contrib_m", "mean"),
        )
        .sort_values(by=["case_id", "axis", "phase"])
        .reset_index(drop=True)
    )


def build_case_comparison(case_phase_df: pd.DataFrame) -> pd.DataFrame:
    turn_df = case_phase_df.loc[case_phase_df["phase"] == "turn"].copy()
    straight_df = case_phase_df.loc[case_phase_df["phase"] == "straight"].copy()
    merged = turn_df.merge(
        straight_df,
        on=["case_id", "axis"],
        suffixes=("_turn", "_straight"),
        how="inner",
    )
    rows: list[dict[str, Any]] = []
    metrics = [
        "mean_positive_step_rate",
        "mean_dx_sign_match_rate",
        "mean_same_axis_sign_match_rate",
        "mean_same_axis_dx_align_rate",
        "mean_step_improve_m",
        "mean_first_step_improve_m",
        "mean_last_step_improve_m",
        "mean_total_window_improve_m",
        "mean_best_prefix_improve_m",
        "mean_reversal_gap_m",
        "temporary_good_then_bad_rate",
        "mean_same_axis_abs_share",
        "mean_cross_axis_abs_share",
        "mean_same_axis_contrib_m",
        "mean_cross_axis_contrib_m",
    ]
    for row in merged.itertuples(index=False):
        out: dict[str, Any] = {
            "case_id": str(row.case_id),
            "axis": str(row.axis),
            "turn_windows": int(row.windows_turn),
            "straight_windows": int(row.windows_straight),
            "turn_updates": int(row.updates_turn),
            "straight_updates": int(row.updates_straight),
        }
        for metric in metrics:
            turn_val = float(getattr(row, f"{metric}_turn"))
            straight_val = float(getattr(row, f"{metric}_straight"))
            out[f"turn_{metric}"] = turn_val
            out[f"straight_{metric}"] = straight_val
            out[f"delta_{metric}"] = turn_val - straight_val
        rows.append(out)
    return pd.DataFrame(rows).sort_values(by=["case_id", "axis"]).reset_index(drop=True)


def write_summary(output_path: Path, comparison_df: pd.DataFrame) -> None:
    lines: list[str] = []
    lines.append("# turn-vs-straight xy sign consistency summary")
    lines.append("")
    lines.append("## Core judgement")
    lines.append(
        "- The main `x/y` failure mode is not `no turning excitation`, and it is not primarily `later epochs undo an otherwise correct turn update`."
    )
    lines.append(
        "- The stronger signal is that turn windows already have weak single-step direction quality: `dx_sign_match_rate` and `positive_step_rate` stay around or below `0.5`, so many lever updates point away from the truth on the same epoch."
    )
    lines.append(
        "- At the same time, `same_axis_sign_match_rate` is often higher than the final `dx_sign_match_rate`, which means turning does open a same-axis lever channel, but cross-axis innovation terms still contaminate the final `K*y` result."
    )
    lines.append(
        "- Some windows do show `best_prefix_improve > 0` while the final window total is negative, so later reversal exists, but it is secondary to the single-step routing/sign problem."
    )
    lines.append("")
    lines.append("## Case comparison")
    for row in comparison_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}` axis `{row.axis}`: turn `dx_sign_match={row.turn_mean_dx_sign_match_rate:.3f}` vs "
            f"straight `{row.straight_mean_dx_sign_match_rate:.3f}`, turn `positive_step={row.turn_mean_positive_step_rate:.3f}` vs "
            f"straight `{row.straight_mean_positive_step_rate:.3f}`; turn `total_window_improve={row.turn_mean_total_window_improve_m * 1000.0:.3f}` mm "
            f"vs straight `{row.straight_mean_total_window_improve_m * 1000.0:.3f}` mm; turn `best_prefix={row.turn_mean_best_prefix_improve_m * 1000.0:.3f}` mm, "
            f"`reversal_gap={row.turn_mean_reversal_gap_m * 1000.0:.3f}` mm, `temp_good_then_bad={row.turn_temporary_good_then_bad_rate:.3f}`; "
            f"turn `same_axis_sign={row.turn_mean_same_axis_sign_match_rate:.3f}`, `same_abs_share={row.turn_mean_same_axis_abs_share:.3f}`, "
            f"`cross_abs_share={row.turn_mean_cross_axis_abs_share:.3f}`."
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    probe_dir = (REPO_ROOT / args.probe_dir).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()
    base_config_path = (REPO_ROOT / args.base_config).resolve()

    if not probe_dir.exists():
        raise RuntimeError(f"missing probe dir: {probe_dir}")

    breakdown_path = probe_dir / "turn_window_update_breakdown.csv"
    if not breakdown_path.exists():
        raise RuntimeError(f"missing breakdown csv: {breakdown_path}")

    turn_windows_df, straight_windows_df = load_matched_windows(probe_dir, pos_path)
    breakdown_df = pd.read_csv(breakdown_path)
    breakdown_df["turn_window_id"] = assign_window_id(breakdown_df["gnss_t"], turn_windows_df, "window_id")
    breakdown_df["straight_window_id"] = assign_window_id(breakdown_df["gnss_t"], straight_windows_df, "window_id")

    cfg = yaml.safe_load(base_config_path.read_text(encoding="utf-8"))
    truth_reference = build_truth_reference(cfg)
    truth_lever_xy = {
        "x": float(truth_reference["states"]["gnss_lever_x"]["reference_value_internal"]),
        "y": float(truth_reference["states"]["gnss_lever_y"]["reference_value_internal"]),
    }

    case_ids = sorted(str(case_id) for case_id in breakdown_df["case_id"].unique())
    raw_gain_df = load_raw_gain_rows(probe_dir, case_ids)
    merged_df = breakdown_df.merge(raw_gain_df, on=["case_id", "gnss_t"], how="left", validate="many_to_one")
    if merged_df[["dx_gnss_lever_x", "dx_gnss_lever_y"]].isna().any().any():
        raise RuntimeError("raw gain merge produced missing dx rows")

    step_df = enrich_step_metrics(merged_df, truth_lever_xy)
    turn_window_summary_df = summarize_phase_windows(step_df, "turn_window_id", "turn")
    straight_window_summary_df = summarize_phase_windows(step_df, "straight_window_id", "straight")
    window_summary_df = pd.concat([turn_window_summary_df, straight_window_summary_df], ignore_index=True)
    case_phase_df = summarize_case_phase(window_summary_df)
    comparison_df = build_case_comparison(case_phase_df)

    ensure_dir(probe_dir)
    step_out = probe_dir / "turn_vs_straight_xy_sign_per_update.csv"
    window_out = probe_dir / "turn_vs_straight_xy_sign_window_summary.csv"
    case_phase_out = probe_dir / "turn_vs_straight_xy_sign_case_phase_summary.csv"
    comparison_out = probe_dir / "turn_vs_straight_xy_sign_case_summary.csv"
    summary_out = probe_dir / "turn_vs_straight_xy_sign_summary.md"
    manifest_out = probe_dir / "turn_vs_straight_xy_sign_manifest.json"

    step_df.to_csv(step_out, index=False, encoding="utf-8-sig")
    window_summary_df.to_csv(window_out, index=False, encoding="utf-8-sig")
    case_phase_df.to_csv(case_phase_out, index=False, encoding="utf-8-sig")
    comparison_df.to_csv(comparison_out, index=False, encoding="utf-8-sig")
    write_summary(summary_out, comparison_df)

    manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "probe_dir": rel_from_root(probe_dir, REPO_ROOT),
        "base_config": rel_from_root(base_config_path, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "truth_lever_xy_m": truth_lever_xy,
        "artifacts": {
            "turn_vs_straight_xy_sign_per_update_csv": rel_from_root(step_out, REPO_ROOT),
            "turn_vs_straight_xy_sign_window_summary_csv": rel_from_root(window_out, REPO_ROOT),
            "turn_vs_straight_xy_sign_case_phase_summary_csv": rel_from_root(case_phase_out, REPO_ROOT),
            "turn_vs_straight_xy_sign_case_summary_csv": rel_from_root(comparison_out, REPO_ROOT),
            "turn_vs_straight_xy_sign_summary_md": rel_from_root(summary_out, REPO_ROOT),
        },
    }
    manifest_out.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
