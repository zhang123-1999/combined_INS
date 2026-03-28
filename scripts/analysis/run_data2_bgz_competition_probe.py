import argparse
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml  # noqa: E402
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import (  # noqa: E402
    CaseSpec as BaselineCaseSpec,
    flatten_consistency_metrics,
    mtime_text,
    run_case as run_baseline_case,
)
from scripts.analysis.run_data2_staged_truth_ablation_probe import (  # noqa: E402
    CaseSpec as TruthCaseSpec,
    build_case_config as build_truth_case_config,
    working_motion_truth_reference,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    build_truth_reference,
    evaluate_navigation_metrics,
    json_safe,
    reset_directory,
)
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    build_motion_frame,
    build_plot_frame,
    build_truth_interp,
    load_imu_dataframe,
    load_pos_dataframe,
    merge_case_outputs,
    parse_diag_times,
)


EXP_ID_DEFAULT = "EXP-20260325-data2-bgz-state-competition-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_bgz_state_competition_r1_20260325")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 60.0
PHASE3_GNSS_OFF_DEFAULT = 60.0
CONVERGED_NOISE_SCALE_DEFAULT = 0.1
CRITICAL_WINDOW_OFFSET_DEFAULT = 0.0
CRITICAL_WINDOW_DURATION_DEFAULT = 60.0


@dataclass(frozen=True)
class CompetitionCaseSpec:
    case_id: str
    label: str
    color: str
    sort_order: int
    freeze_mounting: bool
    freeze_odo_lever: bool


CASE_SPECS: tuple[CompetitionCaseSpec, ...] = (
    CompetitionCaseSpec(
        case_id="mounting_fixed+lever_fixed",
        label="mounting fixed + lever fixed",
        color="#e45756",
        sort_order=0,
        freeze_mounting=True,
        freeze_odo_lever=True,
    ),
    CompetitionCaseSpec(
        case_id="mounting_open+lever_fixed",
        label="mounting open + lever fixed",
        color="#f58518",
        sort_order=1,
        freeze_mounting=False,
        freeze_odo_lever=True,
    ),
    CompetitionCaseSpec(
        case_id="mounting_fixed+lever_open",
        label="mounting fixed + lever open",
        color="#54a24b",
        sort_order=2,
        freeze_mounting=True,
        freeze_odo_lever=False,
    ),
    CompetitionCaseSpec(
        case_id="mounting_open+lever_open",
        label="mounting open + lever open",
        color="#4c78a8",
        sort_order=3,
        freeze_mounting=False,
        freeze_odo_lever=False,
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run bg_z state-competition probe on combined-truth geometry with freeze/open extrinsic variants."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--phase1-end-offset", type=float, default=PHASE1_END_OFFSET_DEFAULT)
    parser.add_argument("--phase2-end-offset", type=float, default=PHASE2_END_OFFSET_DEFAULT)
    parser.add_argument("--phase3-gnss-on", type=float, default=PHASE3_GNSS_ON_DEFAULT)
    parser.add_argument("--phase3-gnss-off", type=float, default=PHASE3_GNSS_OFF_DEFAULT)
    parser.add_argument("--converged-noise-scale", type=float, default=CONVERGED_NOISE_SCALE_DEFAULT)
    parser.add_argument("--critical-window-offset", type=float, default=CRITICAL_WINDOW_OFFSET_DEFAULT)
    parser.add_argument("--critical-window-duration", type=float, default=CRITICAL_WINDOW_DURATION_DEFAULT)
    parser.add_argument("--reuse-existing", action="store_true")
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    return args


def format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, str):
        return value
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


def rmse3d(plot_df: pd.DataFrame, start_time: float, end_time: float) -> float:
    subset = plot_df[(plot_df["timestamp"] >= start_time) & (plot_df["timestamp"] <= end_time)]
    if subset.empty:
        return float("nan")
    err = subset[["p_n_err_m", "p_e_err_m", "p_u_err_m"]].to_numpy(dtype=float)
    return float(np.sqrt(np.mean(np.sum(err * err, axis=1))))


def peak_abs(plot_df: pd.DataFrame, column: str, start_time: float, end_time: float | None = None) -> float:
    subset = plot_df[plot_df["timestamp"] >= start_time]
    if end_time is not None:
        subset = subset[subset["timestamp"] <= end_time]
    if subset.empty:
        return float("nan")
    return float(np.max(np.abs(subset[column].to_numpy(dtype=float))))


def load_existing_case_row(case_dir: Path, cfg_path: Path, spec: CompetitionCaseSpec) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
    diag_path = case_dir / f"DIAG_{spec.case_id}.txt"
    for path in (cfg_path, sol_path, state_series_path, stdout_path, diag_path):
        if not path.exists():
            raise FileNotFoundError(f"missing existing artifact for {spec.case_id}: {path}")
    nav_metrics, segment_rows = evaluate_navigation_metrics(cfg_path, sol_path)
    row: dict[str, Any] = {
        "case_id": spec.case_id,
        "case_label": spec.label,
        "filter_mode": "ESKF",
        "runtime_truth_anchor_pva": False,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "config_mtime": mtime_text(cfg_path),
        "sol_mtime": mtime_text(sol_path),
        "state_series_mtime": mtime_text(state_series_path),
        "diag_mtime": mtime_text(diag_path),
        "stdout_mtime": mtime_text(stdout_path),
        "segment_rows": segment_rows,
    }
    row.update(nav_metrics)
    return row


def build_competition_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CompetitionCaseSpec,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    truth_spec = TruthCaseSpec(
        case_id=spec.case_id,
        label=spec.label,
        color=spec.color,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
    )
    cfg, overrides, metadata = build_truth_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=truth_spec,
        args=args,
    )

    fusion = cfg.setdefault("fusion", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    final_time = float(fusion["finaltime"])
    critical_window_start = float(metadata["phase1_end_time"] + args.critical_window_offset)
    critical_window_end = float(
        min(final_time, critical_window_start + max(0.0, float(args.critical_window_duration)))
    )

    ablation_cfg["disable_mounting"] = bool(spec.freeze_mounting)
    ablation_cfg["disable_odo_lever_arm"] = bool(spec.freeze_odo_lever)
    overrides["fusion.ablation.disable_mounting"] = bool(spec.freeze_mounting)
    overrides["fusion.ablation.disable_odo_lever_arm"] = bool(spec.freeze_odo_lever)

    metadata["competition_note"] = (
        "combined-truth geometry retained; "
        f"mounting={'fixed' if spec.freeze_mounting else 'open'}, "
        f"odo_lever={'fixed' if spec.freeze_odo_lever else 'open'} in phase2/phase3"
    )
    metadata["truth_note"] = (
        "mounting base fixed to working total truth; "
        "ODO lever init fixed to README truth; "
        "open means estimable after phase1, not zeroed"
    )
    metadata["critical_window_start_time"] = critical_window_start
    metadata["critical_window_end_time"] = critical_window_end
    return cfg, overrides, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CompetitionCaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    cfg, overrides, metadata = build_competition_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=spec,
        args=args,
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides, metadata


def first_last_delta(plot_df: pd.DataFrame, column: str, start_time: float, end_time: float) -> float:
    subset = plot_df[(plot_df["timestamp"] >= start_time) & (plot_df["timestamp"] <= end_time)]
    if subset.empty:
        return float("nan")
    values = subset[column].to_numpy(dtype=float)
    return float(values[-1] - values[0])


def peak_abs_change_from_start(plot_df: pd.DataFrame, column: str, start_time: float, end_time: float) -> float:
    subset = plot_df[(plot_df["timestamp"] >= start_time) & (plot_df["timestamp"] <= end_time)]
    if subset.empty:
        return float("nan")
    values = subset[column].to_numpy(dtype=float)
    return float(np.max(np.abs(values - values[0])))


def value_at_or_before(plot_df: pd.DataFrame, column: str, time_point: float) -> float:
    subset = plot_df[plot_df["timestamp"] <= time_point]
    if subset.empty:
        return float("nan")
    return float(subset.iloc[-1][column])


def enrich_reference_deltas(case_metrics_df: pd.DataFrame) -> pd.DataFrame:
    df = case_metrics_df.copy()
    ref_case_id = "mounting_fixed+lever_fixed"
    df["phase3_rmse_improvement_vs_fixed_fixed_m"] = np.nan
    df["phase2_bg_z_abs_reduction_vs_fixed_fixed_degh"] = np.nan
    df["phase3_bg_z_abs_reduction_vs_fixed_fixed_degh"] = np.nan
    df["critical_window_bg_z_delta_reduction_vs_fixed_fixed_degh"] = np.nan
    df["critical_window_mounting_delta_vs_fixed_fixed_deg"] = np.nan
    df["critical_window_lever_y_delta_vs_fixed_fixed_m"] = np.nan
    if ref_case_id not in set(df["case_id"]):
        return df
    ref_row = df.set_index("case_id").loc[ref_case_id]
    for idx, row in df.iterrows():
        df.at[idx, "phase3_rmse_improvement_vs_fixed_fixed_m"] = (
            float(ref_row["phase3_rmse_3d_m"]) - float(row["phase3_rmse_3d_m"])
        )
        df.at[idx, "phase2_bg_z_abs_reduction_vs_fixed_fixed_degh"] = (
            float(ref_row["phase2_bg_z_abs_max_degh"]) - float(row["phase2_bg_z_abs_max_degh"])
        )
        df.at[idx, "phase3_bg_z_abs_reduction_vs_fixed_fixed_degh"] = (
            float(ref_row["phase3_bg_z_abs_max_degh"]) - float(row["phase3_bg_z_abs_max_degh"])
        )
        df.at[idx, "critical_window_bg_z_delta_reduction_vs_fixed_fixed_degh"] = (
            float(ref_row["critical_window_peak_abs_bg_z_change_degh"])
            - float(row["critical_window_peak_abs_bg_z_change_degh"])
        )
        df.at[idx, "critical_window_mounting_delta_vs_fixed_fixed_deg"] = (
            float(row["critical_window_peak_abs_mounting_yaw_change_deg"])
            - float(ref_row["critical_window_peak_abs_mounting_yaw_change_deg"])
        )
        df.at[idx, "critical_window_lever_y_delta_vs_fixed_fixed_m"] = (
            float(row["critical_window_peak_abs_odo_lever_y_change_m"])
            - float(ref_row["critical_window_peak_abs_odo_lever_y_change_m"])
        )
    return df


def build_takeaways(case_metrics_df: pd.DataFrame) -> list[str]:
    if case_metrics_df.empty:
        return ["- no cases available"]
    lookup = case_metrics_df.set_index("case_id")
    fixed = lookup.loc["mounting_fixed+lever_fixed"]
    lines: list[str] = []
    for case_id in (
        "mounting_open+lever_fixed",
        "mounting_fixed+lever_open",
        "mounting_open+lever_open",
    ):
        row = lookup.loc[case_id]
        lines.append(
            "- "
            f"`{case_id}`: phase3 RMSE change vs fixed-fixed = "
            f"`{format_metric(float(row['phase3_rmse_3d_m']) - float(fixed['phase3_rmse_3d_m']))} m`, "
            f"phase2 bg_z peak change = "
            f"`{format_metric(float(row['phase2_bg_z_abs_max_degh']) - float(fixed['phase2_bg_z_abs_max_degh']))} deg/h`, "
            f"critical-window mounting peak-abs-change delta = "
            f"`{format_metric(float(row['critical_window_peak_abs_mounting_yaw_change_deg']) - float(fixed['critical_window_peak_abs_mounting_yaw_change_deg']))} deg`, "
            f"lever-y peak-abs-change delta = "
            f"`{format_metric(float(row['critical_window_peak_abs_odo_lever_y_change_m']) - float(fixed['critical_window_peak_abs_odo_lever_y_change_m']))} m`."
        )
    return lines


def write_summary(
    output_dir: Path,
    exp_id: str,
    case_metrics_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Setup",
        "",
        "- all four cases keep the same staged controls: strict weak-excitation gate, phase2/phase3 freeze GNSS lever, `fusion.init.odo_scale=1.0`, `fusion.fej.enable=false`.",
        "- all four cases also keep the same combined-truth geometry at phase1 exit: mounting base uses working total truth, ODO lever init uses README truth.",
        "- only the phase2/phase3 extrinsic state policy changes: `open` means the state remains estimable after phase1, not that the geometry is reset to zero.",
        (
            "- critical window for state competition metrics: "
            f"`[{format_metric(manifest['critical_window']['start_time'])}, "
            f"{format_metric(manifest['critical_window']['end_time'])}] s`."
        ),
        "",
        "## Case Metrics",
        "",
    ]

    case_rows: list[list[str]] = []
    ordered_df = case_metrics_df.sort_values("sort_order")
    for _, row in ordered_df.iterrows():
        case_rows.append(
            [
                str(row["case_id"]),
                "fixed" if bool(row["freeze_mounting"]) else "open",
                "fixed" if bool(row["freeze_odo_lever"]) else "open",
                format_metric(row["phase2_rmse_3d_m"]),
                format_metric(row["phase3_rmse_3d_m"]),
                format_metric(row["first_divergence_start_t"]),
                format_metric(row["odo_accept_ratio"]),
                format_metric(row["nhc_accept_ratio"]),
                format_metric(row["phase2_bg_z_abs_max_degh"]),
                format_metric(row["phase3_bg_z_abs_max_degh"]),
                format_metric(row["peak_total_mounting_yaw_after_phase1_deg"]),
                format_metric(row["peak_odo_lever_y_after_phase1_m"]),
                format_metric(row["critical_window_peak_abs_bg_z_change_degh"]),
                format_metric(row["critical_window_peak_abs_mounting_yaw_change_deg"]),
                format_metric(row["critical_window_peak_abs_odo_lever_y_change_m"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "mounting",
                "lever",
                "phase2_rmse_3d_m",
                "phase3_rmse_3d_m",
                "first_div_t",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "phase2_bg_z_peak_degh",
                "phase3_bg_z_peak_degh",
                "peak_total_mounting_yaw_deg",
                "peak_odo_lever_y_m",
                "critical_bg_z_peak_change_degh",
                "critical_mounting_peak_change_deg",
                "critical_lever_y_peak_change_m",
            ],
            case_rows,
        )
    )

    lines.extend(["", "## End-Of-Phase2 Net Deltas", ""])
    delta_rows: list[list[str]] = []
    for _, row in ordered_df.iterrows():
        delta_rows.append(
            [
                str(row["case_id"]),
                format_metric(row["phase2_end_net_bg_z_delta_degh"]),
                format_metric(row["phase2_end_net_total_mounting_yaw_delta_deg"]),
                format_metric(row["phase2_end_net_odo_lever_y_delta_m"]),
                format_metric(row["phase3_rmse_improvement_vs_fixed_fixed_m"]),
                format_metric(row["phase2_bg_z_abs_reduction_vs_fixed_fixed_degh"]),
                format_metric(row["phase3_bg_z_abs_reduction_vs_fixed_fixed_degh"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "phase2_end_net_bg_z_delta_degh",
                "phase2_end_net_mounting_yaw_delta_deg",
                "phase2_end_net_odo_lever_y_delta_m",
                "phase3_rmse_improve_vs_fixed_fixed_m",
                "phase2_bg_z_peak_reduction_vs_fixed_fixed_degh",
                "phase3_bg_z_peak_reduction_vs_fixed_fixed_degh",
            ],
            delta_rows,
        )
    )

    lines.extend(["", "## Quick Read", ""])
    lines.extend(build_takeaways(ordered_df))
    lines.extend(
        [
            "",
            "## Manifest",
            "",
            f"- manifest: `{rel_from_root(output_dir / 'manifest.json', REPO_ROOT)}`",
            f"- case_metrics: `{rel_from_root(output_dir / 'case_metrics.csv', REPO_ROOT)}`",
            f"- generated_at: `{manifest['generated_at']}`",
        ]
    )
    (output_dir / "state_competition_summary.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")

    if args.reuse_existing:
        ensure_dir(args.output_dir)
        ensure_dir(args.artifacts_dir)
        ensure_dir(args.case_root)
    else:
        reset_directory(args.output_dir)
        ensure_dir(args.artifacts_dir)
        ensure_dir(args.case_root)

    base_cfg = load_yaml(args.base_config)
    truth_reference = working_motion_truth_reference(base_cfg)
    base_truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())

    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}
    case_metadata: dict[str, dict[str, Any]] = {}

    for spec in CASE_SPECS:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides, metadata = write_case_config(
            base_cfg=base_cfg,
            motion_truth_reference=truth_reference,
            case_dir=case_dir,
            spec=spec,
            args=args,
        )
        case_config_paths[spec.case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_overrides[spec.case_id] = overrides
        case_metadata[spec.case_id] = metadata

        if args.reuse_existing:
            row = load_existing_case_row(case_dir, cfg_path, spec)
        else:
            baseline_spec = BaselineCaseSpec(
                case_id=spec.case_id,
                label=spec.label,
                color=spec.color,
                filter_mode="ESKF",
                enable_fej=False,
                enable_runtime_anchor=False,
            )
            row = run_baseline_case(case_dir, cfg_path, args.exe, baseline_spec)

        stdout_text = (REPO_ROOT / row["stdout_path"]).read_text(encoding="utf-8", errors="ignore")
        row.update(flatten_consistency_metrics(stdout_text))
        row.update(parse_diag_times(stdout_text))
        row.setdefault("odo_accept_ratio", float("nan"))
        row.setdefault("nhc_accept_ratio", float("nan"))
        row.setdefault("first_divergence_start_t", float("nan"))

        sol_path = (REPO_ROOT / row["sol_path"]).resolve()
        state_series_path = (REPO_ROOT / row["state_series_path"]).resolve()
        merged_df = merge_case_outputs(sol_path, state_series_path)
        truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
        motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, truth_reference)
        plot_df = build_plot_frame(merged_df, truth_interp_df, base_truth_reference, motion_df)

        phase1_end_time = float(metadata["phase1_end_time"])
        phase2_end_time = float(metadata["phase2_end_time"])
        final_time = float(base_cfg["fusion"]["finaltime"])
        critical_window_start = float(metadata["critical_window_start_time"])
        critical_window_end = float(metadata["critical_window_end_time"])

        row["sort_order"] = spec.sort_order
        row["freeze_mounting"] = spec.freeze_mounting
        row["freeze_odo_lever"] = spec.freeze_odo_lever
        row["competition_note"] = metadata["competition_note"]
        row["truth_note"] = metadata["truth_note"]
        row["critical_window_start_time"] = critical_window_start
        row["critical_window_end_time"] = critical_window_end
        row["phase2_rmse_3d_m"] = rmse3d(plot_df, phase1_end_time, phase2_end_time)
        row["phase3_rmse_3d_m"] = rmse3d(plot_df, phase2_end_time, final_time)
        row["phase2_bg_z_abs_max_degh"] = peak_abs(merged_df, "bg_z_degh", phase1_end_time, phase2_end_time)
        row["phase3_bg_z_abs_max_degh"] = peak_abs(merged_df, "bg_z_degh", phase2_end_time, final_time)
        row["peak_bg_z_after_phase1_degh"] = peak_abs(merged_df, "bg_z_degh", phase1_end_time)
        row["peak_total_mounting_yaw_after_phase1_deg"] = peak_abs(
            merged_df, "total_mounting_yaw_deg", phase1_end_time
        )
        row["peak_odo_lever_y_after_phase1_m"] = peak_abs(merged_df, "odo_lever_y_m", phase1_end_time)
        row["critical_window_net_bg_z_delta_degh"] = first_last_delta(
            merged_df, "bg_z_degh", critical_window_start, critical_window_end
        )
        row["critical_window_net_total_mounting_yaw_delta_deg"] = first_last_delta(
            merged_df, "total_mounting_yaw_deg", critical_window_start, critical_window_end
        )
        row["critical_window_net_odo_lever_y_delta_m"] = first_last_delta(
            merged_df, "odo_lever_y_m", critical_window_start, critical_window_end
        )
        row["critical_window_peak_abs_bg_z_change_degh"] = peak_abs_change_from_start(
            merged_df, "bg_z_degh", critical_window_start, critical_window_end
        )
        row["critical_window_peak_abs_mounting_yaw_change_deg"] = peak_abs_change_from_start(
            merged_df, "total_mounting_yaw_deg", critical_window_start, critical_window_end
        )
        row["critical_window_peak_abs_odo_lever_y_change_m"] = peak_abs_change_from_start(
            merged_df, "odo_lever_y_m", critical_window_start, critical_window_end
        )
        row["phase2_end_bg_z_degh"] = value_at_or_before(merged_df, "bg_z_degh", phase2_end_time)
        row["phase2_end_total_mounting_yaw_deg"] = value_at_or_before(
            merged_df, "total_mounting_yaw_deg", phase2_end_time
        )
        row["phase2_end_odo_lever_y_m"] = value_at_or_before(merged_df, "odo_lever_y_m", phase2_end_time)
        row["phase2_start_bg_z_degh"] = value_at_or_before(merged_df, "bg_z_degh", phase1_end_time)
        row["phase2_start_total_mounting_yaw_deg"] = value_at_or_before(
            merged_df, "total_mounting_yaw_deg", phase1_end_time
        )
        row["phase2_start_odo_lever_y_m"] = value_at_or_before(merged_df, "odo_lever_y_m", phase1_end_time)
        row["phase2_end_net_bg_z_delta_degh"] = (
            float(row["phase2_end_bg_z_degh"]) - float(row["phase2_start_bg_z_degh"])
        )
        row["phase2_end_net_total_mounting_yaw_delta_deg"] = (
            float(row["phase2_end_total_mounting_yaw_deg"]) - float(row["phase2_start_total_mounting_yaw_deg"])
        )
        row["phase2_end_net_odo_lever_y_delta_m"] = (
            float(row["phase2_end_odo_lever_y_m"]) - float(row["phase2_start_odo_lever_y_m"])
        )
        case_rows.append(row)

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df = case_metrics_df.sort_values("sort_order").reset_index(drop=True)
    case_metrics_df = enrich_reference_deltas(case_metrics_df)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    first_case = case_metrics_df.iloc[0]
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_overrides": case_overrides,
        "case_metadata": case_metadata,
        "critical_window": {
            "start_time": float(first_case["critical_window_start_time"]),
            "end_time": float(first_case["critical_window_end_time"]),
            "duration_s": float(first_case["critical_window_end_time"] - first_case["critical_window_start_time"]),
        },
        "freshness": {
            "solver_exe_mtime": mtime_text(args.exe),
            "base_config_mtime": mtime_text(args.base_config),
        },
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(json_safe(manifest), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    write_summary(args.output_dir, args.exp_id, case_metrics_df, manifest)
    print(rel_from_root(args.output_dir / "state_competition_summary.md", REPO_ROOT))


if __name__ == "__main__":
    main()
