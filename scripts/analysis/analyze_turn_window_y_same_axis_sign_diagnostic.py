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

from scripts.analysis.analyze_turn_window_xy_sign_consistency import load_matched_windows  # noqa: E402
from scripts.analysis.analyze_turn_window_xy_term_source import (  # noqa: E402
    EPS,
    extract_case_rows,
    load_case_ids,
    load_case_meta,
)
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference, json_safe  # noqa: E402
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (  # noqa: E402
    MIN_SPEED_M_S,
    MIN_WINDOW_SEPARATION_S,
    POS_PATH_DEFAULT,
    ROLLING_WINDOW_S,
    TOP_K_WINDOWS,
    load_pos_dataframe,
    select_turn_windows,
)


EXP_ID_DEFAULT = "EXP-20260319-data2-turn-window-y-same-axis-sign-diagnostic-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_y_same_axis_sign_diagnostic")
DEFAULT_SOURCE_PROBES = [
    Path("output/data2_turn_window_position_gain_scale_probe"),
    Path("output/data2_turn_window_lgx_from_y_gain_scale_probe"),
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose y-axis same-axis sign instability by separating innovation sign, "
            "k_lgy_y sign, and turn-direction conditioning."
        )
    )
    parser.add_argument(
        "--source-probe",
        type=Path,
        action="append",
        default=None,
        help="Probe directory to analyze. Can be passed multiple times.",
    )
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    return parser


def nonzero_sign(value: float) -> float:
    if not np.isfinite(value) or abs(value) <= EPS:
        return math.nan
    return 1.0 if value > 0.0 else -1.0


def add_turn_direction_columns(df: pd.DataFrame, pos_path: Path) -> pd.DataFrame:
    pos_df = load_pos_dataframe(pos_path)
    _, turn_series_df = select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=ROLLING_WINDOW_S,
        top_k=TOP_K_WINDOWS,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    turn_t = turn_series_df["t"].to_numpy(dtype=float)
    yaw_rate = turn_series_df["yaw_rate_deg_s"].to_numpy(dtype=float)
    speed = turn_series_df["speed_m_s"].to_numpy(dtype=float)

    out = df.copy()
    out["yaw_rate_deg_s"] = np.interp(out["gnss_t"].to_numpy(dtype=float), turn_t, yaw_rate)
    out["speed_m_s"] = np.interp(out["gnss_t"].to_numpy(dtype=float), turn_t, speed)
    out["turn_direction_sign"] = out["yaw_rate_deg_s"].map(nonzero_sign)
    return out


def load_source_rows(
    source_probe: Path,
    pos_path: Path,
    truth_lever: dict[str, float],
) -> pd.DataFrame:
    turn_windows_df, straight_windows_df = load_matched_windows(source_probe, pos_path)
    case_meta_map = load_case_meta(source_probe)
    case_ids = load_case_ids(source_probe)
    df = extract_case_rows(
        source_probe=source_probe,
        case_ids=case_ids,
        case_meta_map=case_meta_map,
        truth_lever=truth_lever,
        turn_windows_df=turn_windows_df,
        straight_windows_df=straight_windows_df,
    )
    df = df.loc[df["axis"] == "y"].copy().reset_index(drop=True)
    df["source_probe"] = source_probe.name
    return df


def enrich_rows(df: pd.DataFrame, pos_path: Path) -> pd.DataFrame:
    out = add_turn_direction_columns(df, pos_path)
    out["k_lgy_y"] = out["term_y"] / out["innovation_axis_value"]
    out.loc[out["innovation_axis_value"].abs() <= EPS, "k_lgy_y"] = math.nan
    out["innovation_sign"] = out["innovation_axis_value"].map(nonzero_sign)
    out["desired_sign"] = out["desired_dx"].map(nonzero_sign)
    out["k_sign"] = out["k_lgy_y"].map(nonzero_sign)
    out["lever_err_sign"] = (-out["desired_dx"]).map(nonzero_sign)
    out["needed_k_sign"] = out["desired_sign"] * out["innovation_sign"]
    out.loc[
        out["desired_sign"].isna() | out["innovation_sign"].isna(),
        "needed_k_sign",
    ] = math.nan
    out["k_matches_needed_sign"] = np.where(
        out["k_sign"].notna() & out["needed_k_sign"].notna(),
        (out["k_sign"] == out["needed_k_sign"]).astype(float),
        math.nan,
    )
    out["innovation_matches_desired_sign"] = np.where(
        out["innovation_sign"].notna() & out["desired_sign"].notna(),
        (out["innovation_sign"] == out["desired_sign"]).astype(float),
        math.nan,
    )
    return out


def summarize_case_phase(per_update_df: pd.DataFrame) -> pd.DataFrame:
    def positive_rate(series: pd.Series) -> float:
        arr = series.to_numpy(dtype=float)
        arr = arr[np.isfinite(arr)]
        if len(arr) == 0:
            return math.nan
        return float(np.mean(arr > 0.0))

    rows: list[dict[str, Any]] = []
    group_cols = ["source_probe", "case_id", "label", "r_scale", "phase"]
    for key, group in per_update_df.groupby(group_cols, sort=False):
        source_probe, case_id, label, r_scale, phase = key
        rows.append(
            {
                "source_probe": str(source_probe),
                "case_id": str(case_id),
                "label": str(label),
                "r_scale": float(r_scale),
                "phase": str(phase),
                "updates": int(len(group)),
                "mean_same_axis_sign_match_rate": float(np.nanmean(group["same_axis_sign_match"].to_numpy(dtype=float))),
                "mean_k_matches_needed_sign_rate": float(
                    np.nanmean(group["k_matches_needed_sign"].to_numpy(dtype=float))
                ),
                "mean_innovation_matches_desired_sign_rate": float(
                    np.nanmean(group["innovation_matches_desired_sign"].to_numpy(dtype=float))
                ),
                "innovation_positive_rate": positive_rate(group["innovation_axis_value"]),
                "k_positive_rate": positive_rate(group["k_lgy_y"]),
                "k_negative_rate": positive_rate(-group["k_lgy_y"]),
                "desired_positive_rate": positive_rate(group["desired_dx"]),
                "mean_k_lgy_y": float(np.nanmean(group["k_lgy_y"].to_numpy(dtype=float))),
                "mean_innovation_y": float(np.nanmean(group["innovation_axis_value"].to_numpy(dtype=float))),
                "mean_same_axis_term_y": float(np.nanmean(group["term_y"].to_numpy(dtype=float))),
                "mean_step_improve_m": float(np.nanmean(group["step_improve_m"].to_numpy(dtype=float))),
                "mean_total_window_improve_m": float(
                    np.nanmean(
                        np.array(
                            [
                                abs(window_group.iloc[0]["err_before"]) - abs(window_group.iloc[-1]["err_after"])
                                for _, window_group in group.groupby("analysis_window_id", sort=False)
                            ],
                            dtype=float,
                        )
                    )
                ),
            }
        )
    return pd.DataFrame(rows).sort_values(by=["source_probe", "r_scale"]).reset_index(drop=True)


def summarize_by_turn_direction(per_update_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    group_cols = [
        "source_probe",
        "case_id",
        "label",
        "r_scale",
        "phase",
        "turn_direction_sign",
    ]
    for key, group in per_update_df.groupby(group_cols, sort=False):
        source_probe, case_id, label, r_scale, phase, turn_direction_sign = key
        rows.append(
            {
                "source_probe": str(source_probe),
                "case_id": str(case_id),
                "label": str(label),
                "r_scale": float(r_scale),
                "phase": str(phase),
                "turn_direction_sign": float(turn_direction_sign),
                "updates": int(len(group)),
                "mean_yaw_rate_deg_s": float(np.nanmean(group["yaw_rate_deg_s"].to_numpy(dtype=float))),
                "mean_same_axis_sign_match_rate": float(np.nanmean(group["same_axis_sign_match"].to_numpy(dtype=float))),
                "mean_k_matches_needed_sign_rate": float(
                    np.nanmean(group["k_matches_needed_sign"].to_numpy(dtype=float))
                ),
                "mean_innovation_matches_desired_sign_rate": float(
                    np.nanmean(group["innovation_matches_desired_sign"].to_numpy(dtype=float))
                ),
                "innovation_positive_rate": float(np.mean(group["innovation_axis_value"].to_numpy(dtype=float) > 0.0)),
                "k_negative_rate": float(np.mean(group["k_lgy_y"].to_numpy(dtype=float) < 0.0)),
                "mean_k_lgy_y": float(np.nanmean(group["k_lgy_y"].to_numpy(dtype=float))),
                "mean_innovation_y": float(np.nanmean(group["innovation_axis_value"].to_numpy(dtype=float))),
                "mean_same_axis_term_y": float(np.nanmean(group["term_y"].to_numpy(dtype=float))),
            }
        )
    return pd.DataFrame(rows).sort_values(by=["source_probe", "r_scale", "turn_direction_sign"]).reset_index(drop=True)


def summarize_by_window(per_update_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    group_cols = [
        "source_probe",
        "case_id",
        "label",
        "r_scale",
        "phase",
        "analysis_window_id",
    ]
    for key, group in per_update_df.groupby(group_cols, sort=False):
        source_probe, case_id, label, r_scale, phase, window_id = key
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        rows.append(
            {
                "source_probe": str(source_probe),
                "case_id": str(case_id),
                "label": str(label),
                "r_scale": float(r_scale),
                "phase": str(phase),
                "window_id": str(window_id),
                "updates": int(len(group)),
                "mean_yaw_rate_deg_s": float(np.nanmean(group["yaw_rate_deg_s"].to_numpy(dtype=float))),
                "turn_direction_sign": float(np.nanmean(group["turn_direction_sign"].to_numpy(dtype=float))),
                "mean_same_axis_sign_match_rate": float(np.nanmean(group["same_axis_sign_match"].to_numpy(dtype=float))),
                "mean_k_matches_needed_sign_rate": float(
                    np.nanmean(group["k_matches_needed_sign"].to_numpy(dtype=float))
                ),
                "innovation_positive_rate": float(np.mean(group["innovation_axis_value"].to_numpy(dtype=float) > 0.0)),
                "k_negative_rate": float(np.mean(group["k_lgy_y"].to_numpy(dtype=float) < 0.0)),
                "mean_k_lgy_y": float(np.nanmean(group["k_lgy_y"].to_numpy(dtype=float))),
                "mean_innovation_y": float(np.nanmean(group["innovation_axis_value"].to_numpy(dtype=float))),
                "total_window_improve_m": float(abs(group.iloc[0]["err_before"]) - abs(group.iloc[-1]["err_after"])),
            }
        )
    return pd.DataFrame(rows).sort_values(by=["source_probe", "r_scale", "window_id"]).reset_index(drop=True)


def build_reference_comparison(
    case_phase_df: pd.DataFrame,
    turn_dir_df: pd.DataFrame,
) -> pd.DataFrame:
    turn_case_df = case_phase_df.loc[case_phase_df["phase"] == "turn"].copy()
    turn_dir_turn_df = turn_dir_df.loc[turn_dir_df["phase"] == "turn"].copy()
    pos_turn_df = turn_dir_turn_df.loc[turn_dir_turn_df["turn_direction_sign"] > 0.0].copy()
    neg_turn_df = turn_dir_turn_df.loc[turn_dir_turn_df["turn_direction_sign"] < 0.0].copy()
    merged = (
        turn_case_df.merge(
            pos_turn_df[
                [
                    "source_probe",
                    "case_id",
                    "mean_same_axis_sign_match_rate",
                    "mean_k_matches_needed_sign_rate",
                    "innovation_positive_rate",
                    "k_negative_rate",
                ]
            ],
            on=["source_probe", "case_id"],
            how="left",
            suffixes=("", "_pos_turn"),
        )
        .merge(
            neg_turn_df[
                [
                    "source_probe",
                    "case_id",
                    "mean_same_axis_sign_match_rate",
                    "mean_k_matches_needed_sign_rate",
                    "innovation_positive_rate",
                    "k_negative_rate",
                ]
            ],
            on=["source_probe", "case_id"],
            how="left",
            suffixes=("", "_neg_turn"),
        )
    )
    rename_map = {
        "mean_same_axis_sign_match_rate_pos_turn": "pos_turn_same_axis_sign_match_rate",
        "mean_k_matches_needed_sign_rate_pos_turn": "pos_turn_k_matches_needed_sign_rate",
        "innovation_positive_rate_pos_turn": "pos_turn_innovation_positive_rate",
        "k_negative_rate_pos_turn": "pos_turn_k_negative_rate",
        "mean_same_axis_sign_match_rate_neg_turn": "neg_turn_same_axis_sign_match_rate",
        "mean_k_matches_needed_sign_rate_neg_turn": "neg_turn_k_matches_needed_sign_rate",
        "innovation_positive_rate_neg_turn": "neg_turn_innovation_positive_rate",
        "k_negative_rate_neg_turn": "neg_turn_k_negative_rate",
    }
    return merged.rename(columns=rename_map).sort_values(by=["source_probe", "r_scale"]).reset_index(drop=True)


def write_summary(summary_path: Path, comparison_df: pd.DataFrame) -> None:
    lines: list[str] = []
    lines.append("# turn-window y same-axis sign diagnostic")
    lines.append("")
    lines.append("## Core judgement")
    lines.append(
        "- In turn windows, `desired_dx_y` is almost always negative in the current free-PVA setup, so when `innovation_y` is positive, the same-axis channel only helps if `k_lgy_y` turns negative."
    )
    lines.append(
        "- The main `y` problem is not that `innovation_y` flips unpredictably. The stronger signal is that `k_lgy_y` does not match the sign needed by `desired_dx_y * innovation_y`, especially in one turn direction."
    )
    lines.append(
        "- `pos_gain=0.25` improves `y` mainly by shifting the sign distribution of `k_lgy_y`, not by substantially changing the sign distribution of `innovation_y`."
    )
    lines.append(
        "- The later `x`-only selective suppression leaves these `y` metrics nearly unchanged, which confirms that `y` needs its own targeted fix."
    )
    lines.append("")
    lines.append("## Turn-Conditioned Comparison")
    for row in comparison_df.itertuples(index=False):
        lines.append(
            f"- `{row.source_probe}:{row.case_id}`: turn `same_axis_sign={row.mean_same_axis_sign_match_rate:.3f}`, "
            f"`k_matches_needed={row.mean_k_matches_needed_sign_rate:.3f}`, "
            f"`innov_positive={row.innovation_positive_rate:.3f}`, "
            f"`k_negative={row.k_negative_rate:.3f}`, "
            f"`total_window_improve={row.mean_total_window_improve_m * 1e3:.3f}` mm; "
            f"positive-turn `same_axis_sign={row.pos_turn_same_axis_sign_match_rate:.3f}`, "
            f"`innov_positive={row.pos_turn_innovation_positive_rate:.3f}`, "
            f"`k_negative={row.pos_turn_k_negative_rate:.3f}`; "
            f"negative-turn `same_axis_sign={row.neg_turn_same_axis_sign_match_rate:.3f}`, "
            f"`innov_positive={row.neg_turn_innovation_positive_rate:.3f}`, "
            f"`k_negative={row.neg_turn_k_negative_rate:.3f}`."
        )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probes = args.source_probe if args.source_probe else DEFAULT_SOURCE_PROBES
    source_probes = [(REPO_ROOT / probe).resolve() for probe in source_probes]
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()
    base_config_path = (REPO_ROOT / args.base_config).resolve()

    ensure_dir(output_dir)
    for probe in source_probes:
        if not probe.exists():
            raise RuntimeError(f"missing source probe dir: {probe}")

    base_cfg = yaml.safe_load(base_config_path.read_text(encoding="utf-8"))
    truth_reference = build_truth_reference(base_cfg)
    truth_lever = {
        "x": float(truth_reference["states"]["gnss_lever_x"]["reference_value_internal"]),
        "y": float(truth_reference["states"]["gnss_lever_y"]["reference_value_internal"]),
    }

    per_update_df = pd.concat(
        [load_source_rows(probe, pos_path, truth_lever) for probe in source_probes],
        ignore_index=True,
    )
    per_update_df = enrich_rows(per_update_df, pos_path)
    case_phase_df = summarize_case_phase(per_update_df)
    turn_dir_df = summarize_by_turn_direction(per_update_df)
    window_df = summarize_by_window(per_update_df)
    comparison_df = build_reference_comparison(case_phase_df, turn_dir_df)

    per_update_out = output_dir / "y_same_axis_per_update.csv"
    case_phase_out = output_dir / "y_same_axis_case_phase_summary.csv"
    turn_dir_out = output_dir / "y_same_axis_turn_direction_summary.csv"
    window_out = output_dir / "y_same_axis_window_summary.csv"
    comparison_out = output_dir / "y_same_axis_case_comparison.csv"
    summary_out = output_dir / "summary.md"
    manifest_out = output_dir / "manifest.json"

    per_update_df.to_csv(per_update_out, index=False, encoding="utf-8-sig")
    case_phase_df.to_csv(case_phase_out, index=False, encoding="utf-8-sig")
    turn_dir_df.to_csv(turn_dir_out, index=False, encoding="utf-8-sig")
    window_df.to_csv(window_out, index=False, encoding="utf-8-sig")
    comparison_df.to_csv(comparison_out, index=False, encoding="utf-8-sig")
    write_summary(summary_out, comparison_df)

    manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probes": [rel_from_root(probe, REPO_ROOT) for probe in source_probes],
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "base_config": rel_from_root(base_config_path, REPO_ROOT),
        "truth_lever_xy_internal_m": truth_lever,
        "artifacts": {
            "per_update_csv": rel_from_root(per_update_out, REPO_ROOT),
            "case_phase_summary_csv": rel_from_root(case_phase_out, REPO_ROOT),
            "turn_direction_summary_csv": rel_from_root(turn_dir_out, REPO_ROOT),
            "window_summary_csv": rel_from_root(window_out, REPO_ROOT),
            "case_comparison_csv": rel_from_root(comparison_out, REPO_ROOT),
            "summary_md": rel_from_root(summary_out, REPO_ROOT),
        },
    }
    manifest_out.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
