import argparse
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

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import json_safe, reset_directory  # noqa: E402
from scripts.analysis.run_data2_staged_truth_ablation_probe import working_motion_truth_reference  # noqa: E402
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    build_motion_frame,
    build_truth_interp,
    load_imu_dataframe,
    load_pos_dataframe,
    merge_case_outputs,
    parse_diag_times,
)


CASE_ID_DEFAULT = "baseline_current_v_b_gate"
CASE_DIR_DEFAULT = Path(
    "output/data2_nhc_admission_alignment_r1_20260325/artifacts/cases/baseline_current_v_b_gate"
)
OUTPUT_DIR_DEFAULT = Path("output/data2_odo_reference_semantics_audit_r1_20260325")
EXP_ID_DEFAULT = "EXP-20260325-data2-odo-reference-semantics-audit-r1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit ODO residual semantics against current-mainline fresh outputs."
    )
    parser.add_argument("--case-id", default=CASE_ID_DEFAULT)
    parser.add_argument("--case-dir", type=Path, default=CASE_DIR_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    args = parser.parse_args()
    args.case_dir = (REPO_ROOT / args.case_dir).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    return args


def format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float, np.floating)):
        value = float(value)
        if not math.isfinite(value):
            return "NA"
        return f"{value:.6f}"
    return str(value)


def render_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def load_odo_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["timestamp", "odo_speed"],
        engine="python",
    )


def merge_case_frame(
    merged_df: pd.DataFrame,
    motion_df: pd.DataFrame,
    odo_df: pd.DataFrame,
) -> pd.DataFrame:
    frame = pd.merge_asof(
        merged_df.sort_values("timestamp"),
        motion_df.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=0.003,
    )
    frame = pd.merge_asof(
        frame.sort_values("timestamp"),
        odo_df.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=0.003,
    )
    required = [
        "odo_speed",
        "v_b_x_mps",
        "v_v_x_mps",
        "v_v_trans_x_mps",
        "v_v_rot_x_mps",
        "truth_v_v_x_mps",
        "odo_scale_state",
    ]
    if frame[required].isna().any().any():
        raise RuntimeError("failed to align ODO/motion/state data")
    frame["odo_model_est_mps"] = frame["odo_scale_state"] * frame["v_v_x_mps"]
    frame["odo_model_trans_only_est_mps"] = frame["odo_scale_state"] * frame["v_v_trans_x_mps"]
    frame["delta_v_v_minus_v_b_x_mps"] = frame["v_v_x_mps"] - frame["v_b_x_mps"]
    frame["delta_odo_model_minus_v_b_x_mps"] = frame["odo_model_est_mps"] - frame["v_b_x_mps"]
    return frame


def residual_metrics(subset: pd.DataFrame, prediction_col: str) -> dict[str, float]:
    residual = subset["odo_speed"].to_numpy(dtype=float) - subset[prediction_col].to_numpy(dtype=float)
    abs_residual = np.abs(residual)
    corr = float("nan")
    if residual.size > 1:
        x = subset["odo_speed"].to_numpy(dtype=float)
        y = subset[prediction_col].to_numpy(dtype=float)
        if np.std(x) > 1.0e-12 and np.std(y) > 1.0e-12:
            corr = float(np.corrcoef(x, y)[0, 1])
    return {
        "mean_signed_residual_mps": float(np.mean(residual)),
        "mean_abs_residual_mps": float(np.mean(abs_residual)),
        "rmse_residual_mps": float(np.sqrt(np.mean(residual * residual))),
        "p95_abs_residual_mps": float(np.percentile(abs_residual, 95)),
        "corr_with_odo": corr,
    }


def window_rows(frame: pd.DataFrame, window_specs: list[tuple[str, float, float]]) -> pd.DataFrame:
    candidate_specs = [
        ("odo_model_est_mps", "estimated model: odo_scale * v_v.x"),
        ("odo_model_trans_only_est_mps", "estimated model: odo_scale * v_v_trans.x"),
        ("v_v_x_mps", "estimated v_v.x"),
        ("v_v_trans_x_mps", "estimated v_v_trans.x"),
        ("v_b_x_mps", "estimated v_b.x"),
        ("truth_v_v_x_mps", "truth v_v.x"),
    ]
    rows: list[dict[str, Any]] = []
    for window_name, start_t, end_t in window_specs:
        subset = frame[(frame["timestamp"] >= start_t) & (frame["timestamp"] <= end_t)].copy()
        if subset.empty:
            continue
        for candidate_key, candidate_label in candidate_specs:
            metrics = residual_metrics(subset, candidate_key)
            rows.append(
                {
                    "window": window_name,
                    "start_t": start_t,
                    "end_t": end_t,
                    "candidate_key": candidate_key,
                    "candidate_label": candidate_label,
                    **metrics,
                }
            )
    return pd.DataFrame(rows)


def contribution_rows(frame: pd.DataFrame, window_specs: list[tuple[str, float, float]]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for window_name, start_t, end_t in window_specs:
        subset = frame[(frame["timestamp"] >= start_t) & (frame["timestamp"] <= end_t)].copy()
        if subset.empty:
            continue
        rows.append(
            {
                "window": window_name,
                "start_t": start_t,
                "end_t": end_t,
                "mean_abs_v_v_rot_x_mps": float(np.mean(np.abs(subset["v_v_rot_x_mps"]))),
                "max_abs_v_v_rot_x_mps": float(np.max(np.abs(subset["v_v_rot_x_mps"]))),
                "mean_abs_v_v_minus_v_b_x_mps": float(np.mean(np.abs(subset["delta_v_v_minus_v_b_x_mps"]))),
                "max_abs_v_v_minus_v_b_x_mps": float(np.max(np.abs(subset["delta_v_v_minus_v_b_x_mps"]))),
                "mean_abs_odo_model_minus_v_b_x_mps": float(
                    np.mean(np.abs(subset["delta_odo_model_minus_v_b_x_mps"]))
                ),
                "max_abs_odo_model_minus_v_b_x_mps": float(
                    np.max(np.abs(subset["delta_odo_model_minus_v_b_x_mps"]))
                ),
                "mean_odo_scale_state": float(np.mean(subset["odo_scale_state"])),
                "max_abs_odo_scale_offset": float(np.max(np.abs(subset["odo_scale_state"] - 1.0))),
            }
        )
    return pd.DataFrame(rows)


def write_summary(
    output_dir: Path,
    exp_id: str,
    metrics_df: pd.DataFrame,
    contribution_df: pd.DataFrame,
    metadata: dict[str, Any],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Setup",
        "",
        f"- case_id: `{metadata['case_id']}`",
        f"- case_config: `{metadata['case_config']}`",
        f"- sol_path: `{metadata['sol_path']}`",
        f"- state_series_path: `{metadata['state_series_path']}`",
        f"- stdout_path: `{metadata['stdout_path']}`",
        f"- first_divergence_start_t: `{format_metric(metadata['first_divergence_start_t'])}`",
        "",
        "## Residual Metrics",
        "",
    ]

    for window_name in metrics_df["window"].drop_duplicates().tolist():
        subset = metrics_df[metrics_df["window"] == window_name].sort_values("rmse_residual_mps")
        rows = [
            [
                str(row["candidate_key"]),
                format_metric(row["mean_signed_residual_mps"]),
                format_metric(row["mean_abs_residual_mps"]),
                format_metric(row["rmse_residual_mps"]),
                format_metric(row["p95_abs_residual_mps"]),
                format_metric(row["corr_with_odo"]),
            ]
            for _, row in subset.iterrows()
        ]
        lines.append(f"### {window_name}")
        lines.extend(
            render_table(
                [
                    "candidate",
                    "mean_signed_residual_mps",
                    "mean_abs_residual_mps",
                    "rmse_residual_mps",
                    "p95_abs_residual_mps",
                    "corr_with_odo",
                ],
                rows,
            )
        )
        lines.append("")

    lines.extend(["## Forward-Semantics Deltas", ""])
    contribution_rows_md = [
        [
            str(row["window"]),
            format_metric(row["mean_abs_v_v_rot_x_mps"]),
            format_metric(row["max_abs_v_v_rot_x_mps"]),
            format_metric(row["mean_abs_v_v_minus_v_b_x_mps"]),
            format_metric(row["max_abs_v_v_minus_v_b_x_mps"]),
            format_metric(row["mean_abs_odo_model_minus_v_b_x_mps"]),
            format_metric(row["max_abs_odo_model_minus_v_b_x_mps"]),
            format_metric(row["mean_odo_scale_state"]),
            format_metric(row["max_abs_odo_scale_offset"]),
        ]
        for _, row in contribution_df.iterrows()
    ]
    lines.extend(
        render_table(
            [
                "window",
                "mean_abs_v_v_rot_x_mps",
                "max_abs_v_v_rot_x_mps",
                "mean_abs_v_v_minus_v_b_x_mps",
                "max_abs_v_v_minus_v_b_x_mps",
                "mean_abs_odo_model_minus_v_b_x_mps",
                "max_abs_odo_model_minus_v_b_x_mps",
                "mean_odo_scale_state",
                "max_abs_odo_scale_offset",
            ],
            contribution_rows_md,
        )
    )

    lines.extend(["", "## Quick Read", ""])
    for window_name in metrics_df["window"].drop_duplicates().tolist():
        subset = metrics_df[metrics_df["window"] == window_name].sort_values("rmse_residual_mps")
        best = subset.iloc[0]
        lines.append(
            "- "
            f"`{window_name}` best candidate by RMSE is `{best['candidate_key']}` "
            f"with `rmse={format_metric(best['rmse_residual_mps'])} m/s`, "
            f"`mean|res|={format_metric(best['mean_abs_residual_mps'])} m/s`."
        )

    lines.extend(
        [
            "",
            "## Files",
            "",
            f"- semantics_metrics: `{rel_from_root(output_dir / 'semantics_metrics.csv', REPO_ROOT)}`",
            f"- contribution_summary: `{rel_from_root(output_dir / 'contribution_summary.csv', REPO_ROOT)}`",
            f"- manifest: `{rel_from_root(output_dir / 'manifest.json', REPO_ROOT)}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    reset_directory(args.output_dir)
    ensure_dir(args.output_dir)

    case_dir = args.case_dir
    case_id = args.case_id
    case_config = case_dir / f"config_{case_id}.yaml"
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    stdout_path = case_dir / f"solver_stdout_{case_id}.txt"
    if not case_config.exists():
        raise FileNotFoundError(f"missing case config: {case_config}")
    if not sol_path.exists():
        raise FileNotFoundError(f"missing sol file: {sol_path}")
    if not state_series_path.exists():
        raise FileNotFoundError(f"missing state series file: {state_series_path}")
    if not stdout_path.exists():
        raise FileNotFoundError(f"missing stdout file: {stdout_path}")

    cfg = load_yaml(case_config)
    truth_reference = working_motion_truth_reference(cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / cfg["fusion"]["imu_path"]).resolve())
    odo_df = load_odo_dataframe((REPO_ROOT / cfg["fusion"]["odo_path"]).resolve())
    merged_df = merge_case_outputs(sol_path, state_series_path)
    truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
    motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, truth_reference)
    frame = merge_case_frame(merged_df, motion_df, odo_df)

    phase_start = float(frame["timestamp"].min())
    phase_end = float(frame["timestamp"].max())
    phase1_end = float(cfg["fusion"]["starttime"]) + 200.0
    phase2_end = float(cfg["fusion"]["starttime"]) + 700.0
    stdout_text = stdout_path.read_text(encoding="utf-8", errors="ignore")
    first_divergence_start_t = parse_diag_times(stdout_text).get("first_divergence_start_t")
    divergence_end = phase_end
    if first_divergence_start_t is not None and math.isfinite(float(first_divergence_start_t)):
        divergence_end = min(phase_end, float(first_divergence_start_t) + 60.0)

    window_specs: list[tuple[str, float, float]] = [
        ("overall", phase_start, phase_end),
        ("phase2", phase1_end, phase2_end),
        ("phase3", phase2_end, phase_end),
    ]
    if first_divergence_start_t is not None and math.isfinite(float(first_divergence_start_t)):
        window_specs.append(("divergence_plus60s", float(first_divergence_start_t), divergence_end))

    metrics_df = window_rows(frame, window_specs)
    contribution_df = contribution_rows(frame, window_specs)

    metrics_df.to_csv(args.output_dir / "semantics_metrics.csv", index=False, encoding="utf-8-sig")
    contribution_df.to_csv(args.output_dir / "contribution_summary.csv", index=False, encoding="utf-8-sig")

    metadata = {
        "exp_id": args.exp_id,
        "case_id": case_id,
        "case_dir": rel_from_root(case_dir, REPO_ROOT),
        "case_config": rel_from_root(case_config, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "first_divergence_start_t": first_divergence_start_t,
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(json_safe(metadata), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    write_summary(args.output_dir, args.exp_id, metrics_df, contribution_df, metadata)
    print(rel_from_root(args.output_dir / "summary.md", REPO_ROOT))


if __name__ == "__main__":
    main()
