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

from scripts.analysis.analyze_turn_vs_straight_xy_decoupling import assign_window_id  # noqa: E402
from scripts.analysis.analyze_turn_window_xy_sign_consistency import load_matched_windows  # noqa: E402
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference, json_safe  # noqa: E402
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (  # noqa: E402
    POS_PATH_DEFAULT,
    load_effective_gnss_pos_update_df,
    parse_vector_field,
)


EXP_ID_DEFAULT = "EXP-20260318-data2-turn-window-xy-term-source-r1"
SOURCE_PROBE_DEFAULT = Path("output/data2_turn_window_position_gain_scale_probe")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_xy_term_source_probe")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EPS = 1.0e-12
AXES = ["x", "y", "z"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Decompose turn-window GNSS lever updates into k*y term sources per axis."
    )
    parser.add_argument("--source-probe", type=Path, default=SOURCE_PROBE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    return parser


def nonzero_sign(value: float) -> int:
    if not np.isfinite(value) or abs(value) <= EPS:
        return 0
    return 1 if value > 0.0 else -1


def sign_match(actual: float, desired: float) -> float:
    sign_a = nonzero_sign(actual)
    sign_d = nonzero_sign(desired)
    if sign_a == 0 or sign_d == 0:
        return math.nan
    return float(sign_a == sign_d)


def load_case_meta(source_probe: Path) -> dict[str, dict[str, Any]]:
    manifest_path = source_probe / "manifest.json"
    if not manifest_path.exists():
        return {}
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    case_rows = manifest.get("case_rows", [])
    return {str(row["case_id"]): row for row in case_rows}


def load_case_ids(source_probe: Path) -> list[str]:
    summary_path = source_probe / "case_update_summary.csv"
    if summary_path.exists():
        summary_df = pd.read_csv(summary_path)
        if "case_id" in summary_df.columns:
            return [str(case_id) for case_id in summary_df["case_id"].dropna().unique().tolist()]
    case_dirs = sorted((source_probe / "artifacts" / "cases").glob("*"))
    return [case_dir.name for case_dir in case_dirs if case_dir.is_dir()]


def extract_case_rows(
    source_probe: Path,
    case_ids: list[str],
    case_meta_map: dict[str, dict[str, Any]],
    truth_lever: dict[str, float],
    turn_windows_df: pd.DataFrame,
    straight_windows_df: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_id in case_ids:
        raw_path = source_probe / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv"
        if not raw_path.exists():
            raise RuntimeError(f"missing raw GNSS update log for {case_id}: {raw_path}")
        raw_df = load_effective_gnss_pos_update_df(raw_path)
        raw_df = raw_df.loc[raw_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)
        case_meta = case_meta_map.get(case_id, {})
        label = str(case_meta.get("label", case_id))
        r_scale = float(case_meta.get("r_scale", math.nan))
        turn_ids = assign_window_id(raw_df["gnss_t"], turn_windows_df, "window_id")
        straight_ids = assign_window_id(raw_df["gnss_t"], straight_windows_df, "window_id")
        raw_df["turn_window_id"] = turn_ids
        raw_df["straight_window_id"] = straight_ids
        raw_df["phase"] = np.where(raw_df["turn_window_id"].notna(), "turn", np.where(raw_df["straight_window_id"].notna(), "straight", None))
        raw_df["analysis_window_id"] = raw_df["turn_window_id"].where(raw_df["turn_window_id"].notna(), raw_df["straight_window_id"])

        for row in raw_df.itertuples(index=False):
            phase = getattr(row, "phase")
            window_id = getattr(row, "analysis_window_id")
            if phase not in {"turn", "straight"} or not isinstance(window_id, str):
                continue
            innovation = np.array([float(row.y_x), float(row.y_y), float(row.y_z)], dtype=float)
            for axis_idx, axis in enumerate(["x", "y"]):
                k_vec = parse_vector_field(getattr(row, f"k_gnss_lever_{axis}_vec", None), 3)
                if not np.isfinite(k_vec).all():
                    continue
                contrib = k_vec * innovation
                dx = float(getattr(row, f"dx_gnss_lever_{axis}"))
                before = float(getattr(row, f"lever_before_{axis}"))
                after = before + dx
                desired = float(truth_lever[axis] - before)
                same = float(contrib[axis_idx])
                cross_terms = {
                    term_axis: float(contrib[idx]) for idx, term_axis in enumerate(AXES) if idx != axis_idx
                }
                cross_total = float(sum(cross_terms.values()))
                dominant_term = AXES[int(np.argmax(np.abs(contrib)))]
                dominant_cross_term = max(cross_terms, key=lambda key: abs(cross_terms[key]))

                same_match = sign_match(same, desired)
                final_match = sign_match(dx, desired)
                same_good_final_bad = float(same_match == 1.0 and final_match == 0.0)
                desired_sign = nonzero_sign(desired)
                opposite_cross = {
                    key: value
                    for key, value in cross_terms.items()
                    if desired_sign != 0 and nonzero_sign(value) == -desired_sign
                }
                dominant_bad_cross_term = (
                    max(opposite_cross, key=lambda key: abs(opposite_cross[key]))
                    if opposite_cross
                    else ""
                )
                rows.append(
                    {
                        "case_id": case_id,
                        "label": label,
                        "r_scale": r_scale,
                        "axis": axis,
                        "phase": phase,
                        "analysis_window_id": window_id,
                        "gnss_t": float(row.gnss_t),
                        "state_t": float(row.state_t),
                        "innovation_axis_value": float(innovation[axis_idx]),
                        "desired_dx": desired,
                        "dx": dx,
                        "lever_before": before,
                        "lever_after": after,
                        "err_before": before - float(truth_lever[axis]),
                        "err_after": after - float(truth_lever[axis]),
                        "step_improve_m": abs(before - float(truth_lever[axis])) - abs(after - float(truth_lever[axis])),
                        "term_x": float(contrib[0]),
                        "term_y": float(contrib[1]),
                        "term_z": float(contrib[2]),
                        "same_axis_term": same,
                        "cross_total_term": cross_total,
                        "cross_y_term": float(cross_terms.get("y", 0.0)),
                        "cross_z_term": float(cross_terms.get("z", 0.0)),
                        "dominant_term": dominant_term,
                        "dominant_cross_term": dominant_cross_term,
                        "dominant_bad_cross_term": dominant_bad_cross_term,
                        "dx_reconstruction_error": dx - float(contrib.sum()),
                        "final_sign_match": final_match,
                        "same_axis_sign_match": same_match,
                        "same_axis_dx_align": sign_match(same, dx),
                        "term_x_sign_match": sign_match(float(contrib[0]), desired),
                        "term_y_sign_match": sign_match(float(contrib[1]), desired),
                        "term_z_sign_match": sign_match(float(contrib[2]), desired),
                        "same_axis_good_final_bad": same_good_final_bad,
                        "same_axis_abs_share": abs(same) / max(abs(dx), EPS),
                        "cross_total_abs_share": abs(cross_total) / max(abs(dx), EPS),
                        "term_x_abs_share": abs(float(contrib[0])) / max(abs(dx), EPS),
                        "term_y_abs_share": abs(float(contrib[1])) / max(abs(dx), EPS),
                        "term_z_abs_share": abs(float(contrib[2])) / max(abs(dx), EPS),
                        "cross_x_abs_share": abs(float(cross_terms.get("x", 0.0))) / max(abs(dx), EPS),
                        "cross_y_abs_share": abs(float(cross_terms.get("y", 0.0))) / max(abs(dx), EPS),
                        "cross_z_abs_share": abs(float(cross_terms.get("z", 0.0))) / max(abs(dx), EPS),
                    }
                )
    if not rows:
        raise RuntimeError("no usable GNSS_POS term rows found")
    return pd.DataFrame(rows).sort_values(by=["case_id", "axis", "gnss_t"]).reset_index(drop=True)


def summarize_windows(per_update_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for (case_id, label, r_scale, axis, phase, window_id), group in per_update_df.groupby(
        ["case_id", "label", "r_scale", "axis", "phase", "analysis_window_id"], sort=False
    ):
        group = group.sort_values(by="gnss_t").reset_index(drop=True)
        dominant_term_counts = group["dominant_term"].value_counts(normalize=True)
        dominant_bad_cross = (
            group.loc[group["same_axis_good_final_bad"] > 0.5, "dominant_bad_cross_term"]
            .replace("", np.nan)
            .value_counts(normalize=True)
        )
        rows.append(
            {
                "case_id": str(case_id),
                "label": str(label),
                "r_scale": float(r_scale),
                "axis": str(axis),
                "phase": str(phase),
                "window_id": str(window_id),
                "updates": int(len(group)),
                "mean_abs_innovation_axis": float(group["innovation_axis_value"].abs().mean()),
                "mean_step_improve_m": float(group["step_improve_m"].mean()),
                "total_window_improve_m": float(abs(group.iloc[0]["err_before"]) - abs(group.iloc[-1]["err_after"])),
                "mean_final_sign_match_rate": float(np.nanmean(group["final_sign_match"].to_numpy(dtype=float))),
                "mean_same_axis_sign_match_rate": float(np.nanmean(group["same_axis_sign_match"].to_numpy(dtype=float))),
                "mean_same_axis_dx_align_rate": float(np.nanmean(group["same_axis_dx_align"].to_numpy(dtype=float))),
                "same_axis_good_final_bad_rate": float(group["same_axis_good_final_bad"].mean()),
                "mean_abs_term_x_m": float(group["term_x"].abs().mean()),
                "mean_abs_term_y_m": float(group["term_y"].abs().mean()),
                "mean_abs_term_z_m": float(group["term_z"].abs().mean()),
                "mean_term_x_m": float(group["term_x"].mean()),
                "mean_term_y_m": float(group["term_y"].mean()),
                "mean_term_z_m": float(group["term_z"].mean()),
                "mean_same_axis_abs_share": float(group["same_axis_abs_share"].mean()),
                "mean_cross_total_abs_share": float(group["cross_total_abs_share"].mean()),
                "mean_cross_x_abs_share": float(group["cross_x_abs_share"].mean()),
                "mean_cross_y_abs_share": float(group["cross_y_abs_share"].mean()),
                "mean_cross_z_abs_share": float(group["cross_z_abs_share"].mean()),
                "mean_dx_reconstruction_error": float(group["dx_reconstruction_error"].abs().mean()),
                "dominant_term_x_rate": float(dominant_term_counts.get("x", 0.0)),
                "dominant_term_y_rate": float(dominant_term_counts.get("y", 0.0)),
                "dominant_term_z_rate": float(dominant_term_counts.get("z", 0.0)),
                "dominant_bad_cross_x_rate": float(dominant_bad_cross.get("x", 0.0)),
                "dominant_bad_cross_y_rate": float(dominant_bad_cross.get("y", 0.0)),
                "dominant_bad_cross_z_rate": float(dominant_bad_cross.get("z", 0.0)),
            }
        )
    return pd.DataFrame(rows)


def summarize_case_phase(window_summary_df: pd.DataFrame) -> pd.DataFrame:
    return (
        window_summary_df.groupby(["case_id", "label", "r_scale", "axis", "phase"], as_index=False)
        .agg(
            windows=("window_id", "count"),
            updates=("updates", "sum"),
            mean_abs_innovation_axis=("mean_abs_innovation_axis", "mean"),
            mean_step_improve_m=("mean_step_improve_m", "mean"),
            mean_total_window_improve_m=("total_window_improve_m", "mean"),
            mean_final_sign_match_rate=("mean_final_sign_match_rate", "mean"),
            mean_same_axis_sign_match_rate=("mean_same_axis_sign_match_rate", "mean"),
            mean_same_axis_dx_align_rate=("mean_same_axis_dx_align_rate", "mean"),
            mean_same_axis_good_final_bad_rate=("same_axis_good_final_bad_rate", "mean"),
            mean_abs_term_x_m=("mean_abs_term_x_m", "mean"),
            mean_abs_term_y_m=("mean_abs_term_y_m", "mean"),
            mean_abs_term_z_m=("mean_abs_term_z_m", "mean"),
            mean_term_x_m=("mean_term_x_m", "mean"),
            mean_term_y_m=("mean_term_y_m", "mean"),
            mean_term_z_m=("mean_term_z_m", "mean"),
            mean_same_axis_abs_share=("mean_same_axis_abs_share", "mean"),
            mean_cross_total_abs_share=("mean_cross_total_abs_share", "mean"),
            mean_cross_x_abs_share=("mean_cross_x_abs_share", "mean"),
            mean_cross_y_abs_share=("mean_cross_y_abs_share", "mean"),
            mean_cross_z_abs_share=("mean_cross_z_abs_share", "mean"),
            mean_dx_reconstruction_error=("mean_dx_reconstruction_error", "mean"),
            mean_dominant_term_x_rate=("dominant_term_x_rate", "mean"),
            mean_dominant_term_y_rate=("dominant_term_y_rate", "mean"),
            mean_dominant_term_z_rate=("dominant_term_z_rate", "mean"),
            mean_dominant_bad_cross_x_rate=("dominant_bad_cross_x_rate", "mean"),
            mean_dominant_bad_cross_y_rate=("dominant_bad_cross_y_rate", "mean"),
            mean_dominant_bad_cross_z_rate=("dominant_bad_cross_z_rate", "mean"),
        )
        .sort_values(by=["case_id", "axis", "phase"])
        .reset_index(drop=True)
    )


def build_case_comparison(case_phase_df: pd.DataFrame) -> pd.DataFrame:
    turn_df = case_phase_df.loc[case_phase_df["phase"] == "turn"].copy()
    straight_df = case_phase_df.loc[case_phase_df["phase"] == "straight"].copy()
    merged = turn_df.merge(
        straight_df,
        on=["case_id", "label", "r_scale", "axis"],
        how="inner",
        suffixes=("_turn", "_straight"),
    )
    rows: list[dict[str, Any]] = []
    metrics = [
        "mean_total_window_improve_m",
        "mean_final_sign_match_rate",
        "mean_same_axis_sign_match_rate",
        "mean_same_axis_dx_align_rate",
        "mean_same_axis_good_final_bad_rate",
        "mean_abs_term_x_m",
        "mean_abs_term_y_m",
        "mean_abs_term_z_m",
        "mean_term_x_m",
        "mean_term_y_m",
        "mean_term_z_m",
        "mean_same_axis_abs_share",
        "mean_cross_total_abs_share",
        "mean_cross_x_abs_share",
        "mean_cross_y_abs_share",
        "mean_cross_z_abs_share",
        "mean_dominant_term_x_rate",
        "mean_dominant_term_y_rate",
        "mean_dominant_term_z_rate",
        "mean_dominant_bad_cross_x_rate",
        "mean_dominant_bad_cross_y_rate",
        "mean_dominant_bad_cross_z_rate",
    ]
    for row in merged.itertuples(index=False):
        out: dict[str, Any] = {
            "case_id": str(row.case_id),
            "label": str(row.label),
            "r_scale": float(row.r_scale),
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
    return pd.DataFrame(rows).sort_values(by=["axis", "r_scale"]).reset_index(drop=True)


def write_summary(summary_path: Path, case_comparison_df: pd.DataFrame) -> None:
    lines: list[str] = []
    lines.append("# turn-window xy term source summary")
    lines.append("")
    lines.append("## X-axis focus")
    x_df = case_comparison_df.loc[case_comparison_df["axis"] == "x"].copy()
    for row in x_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}`: turn `total_improve={row.turn_mean_total_window_improve_m * 1e3:.3f}` mm, "
            f"`dx_sign_match={row.turn_mean_final_sign_match_rate:.3f}`, "
            f"`same_axis_sign={row.turn_mean_same_axis_sign_match_rate:.3f}`, "
            f"`same_good_final_bad={row.turn_mean_same_axis_good_final_bad_rate:.3f}`; "
            f"turn `|term_x|/|term_y|/|term_z|={row.turn_mean_abs_term_x_m * 1e3:.3f}/"
            f"{row.turn_mean_abs_term_y_m * 1e3:.3f}/{row.turn_mean_abs_term_z_m * 1e3:.3f}` mm; "
            f"turn dominant bad cross `y/z={row.turn_mean_dominant_bad_cross_y_rate:.3f}/"
            f"{row.turn_mean_dominant_bad_cross_z_rate:.3f}`."
        )
    lines.append("")
    lines.append("## Y-axis focus")
    y_df = case_comparison_df.loc[case_comparison_df["axis"] == "y"].copy()
    for row in y_df.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}`: turn `total_improve={row.turn_mean_total_window_improve_m * 1e3:.3f}` mm, "
            f"`dx_sign_match={row.turn_mean_final_sign_match_rate:.3f}`, "
            f"`same_axis_sign={row.turn_mean_same_axis_sign_match_rate:.3f}`, "
            f"`same_good_final_bad={row.turn_mean_same_axis_good_final_bad_rate:.3f}`; "
            f"turn `|term_x|/|term_y|/|term_z|={row.turn_mean_abs_term_x_m * 1e3:.3f}/"
            f"{row.turn_mean_abs_term_y_m * 1e3:.3f}/{row.turn_mean_abs_term_z_m * 1e3:.3f}` mm; "
            f"turn dominant bad cross `x/z={row.turn_mean_dominant_bad_cross_x_rate:.3f}/"
            f"{row.turn_mean_dominant_bad_cross_z_rate:.3f}`."
        )
    lines.append("")
    lines.append("## Reading guide")
    lines.append(
        "- For axis `x`, `term_x` is the same-axis channel `k_lgx_x * y_x`; `term_y/term_z` are cross-axis contamination terms."
    )
    lines.append(
        "- `same_good_final_bad_rate` highlights updates where the same-axis term pointed toward truth but the final summed `dx` still pointed away."
    )
    lines.append(
        "- `dominant_bad_cross_y/z_rate` is computed only inside those `same_good_final_bad` updates, so it directly indicates which cross-axis term most often flipped the result."
    )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probe = (REPO_ROOT / args.source_probe).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()
    base_config_path = (REPO_ROOT / args.base_config).resolve()

    if not source_probe.exists():
        raise RuntimeError(f"missing source probe dir: {source_probe}")
    ensure_dir(output_dir)

    base_cfg = yaml.safe_load(base_config_path.read_text(encoding="utf-8"))
    truth_reference = build_truth_reference(base_cfg)
    truth_lever = {
        "x": float(truth_reference["states"]["gnss_lever_x"]["reference_value_internal"]),
        "y": float(truth_reference["states"]["gnss_lever_y"]["reference_value_internal"]),
    }

    turn_windows_df, straight_windows_df = load_matched_windows(source_probe, pos_path)
    case_meta_map = load_case_meta(source_probe)
    case_ids = load_case_ids(source_probe)
    per_update_df = extract_case_rows(
        source_probe=source_probe,
        case_ids=case_ids,
        case_meta_map=case_meta_map,
        truth_lever=truth_lever,
        turn_windows_df=turn_windows_df,
        straight_windows_df=straight_windows_df,
    )
    window_summary_df = summarize_windows(per_update_df)
    case_phase_df = summarize_case_phase(window_summary_df)
    case_summary_df = build_case_comparison(case_phase_df)

    per_update_out = output_dir / "turn_vs_straight_xy_term_per_update.csv"
    window_summary_out = output_dir / "turn_vs_straight_xy_term_window_summary.csv"
    case_phase_out = output_dir / "turn_vs_straight_xy_term_case_phase_summary.csv"
    case_summary_out = output_dir / "turn_vs_straight_xy_term_case_summary.csv"
    summary_md_out = output_dir / "turn_vs_straight_xy_term_summary.md"

    per_update_df.to_csv(per_update_out, index=False, encoding="utf-8-sig")
    window_summary_df.to_csv(window_summary_out, index=False, encoding="utf-8-sig")
    case_phase_df.to_csv(case_phase_out, index=False, encoding="utf-8-sig")
    case_summary_df.to_csv(case_summary_out, index=False, encoding="utf-8-sig")
    write_summary(summary_md_out, case_summary_df)

    manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probe": rel_from_root(source_probe, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "base_config": rel_from_root(base_config_path, REPO_ROOT),
        "truth_lever_xy_internal_m": truth_lever,
        "artifacts": {
            "per_update_csv": rel_from_root(per_update_out, REPO_ROOT),
            "window_summary_csv": rel_from_root(window_summary_out, REPO_ROOT),
            "case_phase_summary_csv": rel_from_root(case_phase_out, REPO_ROOT),
            "case_summary_csv": rel_from_root(case_summary_out, REPO_ROOT),
            "summary_md": rel_from_root(summary_md_out, REPO_ROOT),
        },
    }
    manifest_out = output_dir / "manifest.json"
    manifest_out.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
