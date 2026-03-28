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


EXP_ID_DEFAULT = "EXP-20260325-data2-bgz-mitigation-validation-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_bgz_mitigation_validation_r1_20260325")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 60.0
PHASE3_GNSS_OFF_DEFAULT = 60.0
CONVERGED_NOISE_SCALE_DEFAULT = 0.1


@dataclass(frozen=True)
class ValidationCaseSpec:
    case_id: str
    label: str
    color: str
    sort_order: int
    pure_ins_gnss: bool = False
    enable_bgz_gate: bool = False
    enable_bgz_forgetting: bool = False
    bgz_gate_apply_to_odo: bool = True
    bgz_gate_apply_to_nhc: bool = True
    bgz_gate_forward_speed_min: float = 3.0
    bgz_gate_yaw_rate_min_deg_s: float = 8.0
    bgz_gate_lateral_acc_min: float = 0.3
    bgz_gate_min_scale: float = 0.0
    bgz_cov_forgetting_tau_s: float = 15.0


CASE_SET_R1: tuple[ValidationCaseSpec, ...] = (
    ValidationCaseSpec(
        case_id="ins_gnss_only",
        label="INS/GNSS only",
        color="#4c78a8",
        sort_order=0,
        pure_ins_gnss=True,
    ),
    ValidationCaseSpec(
        case_id="current_mainline_ins_gnss_odo_nhc",
        label="current mainline INS/GNSS/ODO/NHC",
        color="#f58518",
        sort_order=1,
    ),
    ValidationCaseSpec(
        case_id="mitigated_bgz_gate_only",
        label="mitigated: bg_z gate only",
        color="#54a24b",
        sort_order=2,
        enable_bgz_gate=True,
    ),
    ValidationCaseSpec(
        case_id="mitigated_bgz_gate_and_forgetting",
        label="mitigated: bg_z gate + covariance forgetting",
        color="#e45756",
        sort_order=3,
        enable_bgz_gate=True,
        enable_bgz_forgetting=True,
    ),
)

CASE_SET_ODO_ONLY_R2: tuple[ValidationCaseSpec, ...] = (
    ValidationCaseSpec(
        case_id="ins_gnss_only",
        label="INS/GNSS only",
        color="#4c78a8",
        sort_order=0,
        pure_ins_gnss=True,
    ),
    ValidationCaseSpec(
        case_id="current_mainline_ins_gnss_odo_nhc",
        label="current mainline INS/GNSS/ODO/NHC",
        color="#f58518",
        sort_order=1,
    ),
    ValidationCaseSpec(
        case_id="mitigated_bgz_gate_odo_only",
        label="mitigated: ODO-only bg_z gate",
        color="#54a24b",
        sort_order=2,
        enable_bgz_gate=True,
        bgz_gate_apply_to_odo=True,
        bgz_gate_apply_to_nhc=False,
    ),
)

CASE_SET_ODO_ONLY_TIGHT_R3: tuple[ValidationCaseSpec, ...] = (
    ValidationCaseSpec(
        case_id="ins_gnss_only",
        label="INS/GNSS only",
        color="#4c78a8",
        sort_order=0,
        pure_ins_gnss=True,
    ),
    ValidationCaseSpec(
        case_id="current_mainline_ins_gnss_odo_nhc",
        label="current mainline INS/GNSS/ODO/NHC",
        color="#f58518",
        sort_order=1,
    ),
    ValidationCaseSpec(
        case_id="mitigated_bgz_gate_odo_only_yaw12",
        label="mitigated: ODO-only yaw gate 12 deg/s",
        color="#54a24b",
        sort_order=2,
        enable_bgz_gate=True,
        bgz_gate_apply_to_odo=True,
        bgz_gate_apply_to_nhc=False,
        bgz_gate_yaw_rate_min_deg_s=12.0,
        bgz_gate_lateral_acc_min=1.0e9,
    ),
)

CASE_SET_MAP: dict[str, tuple[ValidationCaseSpec, ...]] = {
    "r1": CASE_SET_R1,
    "odo_only_r2": CASE_SET_ODO_ONLY_R2,
    "odo_only_tight_r3": CASE_SET_ODO_ONLY_TIGHT_R3,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate non-debug bg_z mitigation candidates against staged INS/GNSS and current mainline."
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
    parser.add_argument("--case-set", choices=sorted(CASE_SET_MAP.keys()), default="r1")
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


def peak_abs(df: pd.DataFrame, column: str, start_time: float, end_time: float | None = None) -> float:
    subset = df[df["timestamp"] >= start_time]
    if end_time is not None:
        subset = subset[subset["timestamp"] <= end_time]
    if subset.empty:
        return float("nan")
    return float(np.max(np.abs(subset[column].to_numpy(dtype=float))))


def build_validation_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: ValidationCaseSpec,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    truth_spec = TruthCaseSpec(
        case_id=spec.case_id,
        label=spec.label,
        color=spec.color,
        fix_mounting_truth=False,
        fix_odo_lever_truth=False,
    )
    cfg, overrides, metadata = build_truth_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=truth_spec,
        args=args,
    )

    fusion = cfg.setdefault("fusion", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    runtime_phases = fusion.setdefault("runtime_phases", [])

    variant_notes: list[str] = []

    if spec.pure_ins_gnss:
        constraints_cfg["enable_odo"] = False
        constraints_cfg["enable_nhc"] = False
        ablation_cfg["disable_odo_scale"] = True
        ablation_cfg["disable_mounting"] = True
        ablation_cfg["disable_odo_lever_arm"] = True
        overrides["fusion.constraints.enable_odo"] = False
        overrides["fusion.constraints.enable_nhc"] = False
        overrides["fusion.ablation.disable_odo_scale"] = True
        overrides["fusion.ablation.disable_mounting"] = True
        overrides["fusion.ablation.disable_odo_lever_arm"] = True
        for phase in runtime_phases:
            phase_constraints = phase.setdefault("constraints", {})
            phase_constraints["enable_odo"] = False
            phase_constraints["enable_nhc"] = False
            phase_ablation = phase.setdefault("ablation", {})
            phase_ablation["disable_odo_scale"] = True
            phase_ablation["disable_mounting"] = True
            phase_ablation["disable_odo_lever_arm"] = True
        overrides["fusion.runtime_phases"] = runtime_phases
        variant_notes.append("pure INS/GNSS: ODO/NHC disabled for all phases, odo-related extrinsics frozen")

    if spec.enable_bgz_gate:
        constraints_cfg["enable_bgz_observability_gate"] = True
        constraints_cfg["bgz_gate_apply_to_odo"] = bool(spec.bgz_gate_apply_to_odo)
        constraints_cfg["bgz_gate_apply_to_nhc"] = bool(spec.bgz_gate_apply_to_nhc)
        constraints_cfg["bgz_gate_forward_speed_min"] = float(spec.bgz_gate_forward_speed_min)
        constraints_cfg["bgz_gate_yaw_rate_min_deg_s"] = float(spec.bgz_gate_yaw_rate_min_deg_s)
        constraints_cfg["bgz_gate_lateral_acc_min"] = float(spec.bgz_gate_lateral_acc_min)
        constraints_cfg["bgz_gate_min_scale"] = float(spec.bgz_gate_min_scale)
        overrides["fusion.constraints.enable_bgz_observability_gate"] = True
        overrides["fusion.constraints.bgz_gate_apply_to_odo"] = bool(spec.bgz_gate_apply_to_odo)
        overrides["fusion.constraints.bgz_gate_apply_to_nhc"] = bool(spec.bgz_gate_apply_to_nhc)
        overrides["fusion.constraints.bgz_gate_forward_speed_min"] = float(spec.bgz_gate_forward_speed_min)
        overrides["fusion.constraints.bgz_gate_yaw_rate_min_deg_s"] = float(spec.bgz_gate_yaw_rate_min_deg_s)
        overrides["fusion.constraints.bgz_gate_lateral_acc_min"] = float(spec.bgz_gate_lateral_acc_min)
        overrides["fusion.constraints.bgz_gate_min_scale"] = float(spec.bgz_gate_min_scale)
        variant_notes.append(
            "bg_z gate "
            f"(odo={'on' if spec.bgz_gate_apply_to_odo else 'off'}, "
            f"nhc={'on' if spec.bgz_gate_apply_to_nhc else 'off'}, "
            f"vx>={spec.bgz_gate_forward_speed_min:.2f} m/s, "
            f"yaw>={spec.bgz_gate_yaw_rate_min_deg_s:.2f} deg/s or "
            f"|ay|>={spec.bgz_gate_lateral_acc_min:.2f} m/s^2, "
            f"min_scale={spec.bgz_gate_min_scale:.2f})"
        )

    if spec.enable_bgz_forgetting:
        constraints_cfg["enable_bgz_covariance_forgetting"] = True
        constraints_cfg["bgz_cov_forgetting_tau_s"] = float(spec.bgz_cov_forgetting_tau_s)
        overrides["fusion.constraints.enable_bgz_covariance_forgetting"] = True
        overrides["fusion.constraints.bgz_cov_forgetting_tau_s"] = float(spec.bgz_cov_forgetting_tau_s)
        variant_notes.append(f"bg_z covariance forgetting tau={spec.bgz_cov_forgetting_tau_s:.2f} s")

    metadata["variant_note"] = "; ".join(variant_notes) if variant_notes else "mainline staged control"
    return cfg, overrides, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: ValidationCaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    cfg, overrides, metadata = build_validation_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=spec,
        args=args,
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides, metadata


def enrich_reference_deltas(case_metrics_df: pd.DataFrame) -> pd.DataFrame:
    df = case_metrics_df.copy()
    df["phase3_rmse_delta_vs_ins_gnss_m"] = np.nan
    df["phase3_rmse_improvement_vs_current_m"] = np.nan
    df["phase3_bg_z_peak_reduction_vs_current_degh"] = np.nan
    df["phase3_yaw_peak_reduction_vs_current_deg"] = np.nan
    if "ins_gnss_only" not in set(df["case_id"]) or "current_mainline_ins_gnss_odo_nhc" not in set(df["case_id"]):
        return df
    lookup = df.set_index("case_id")
    ins_row = lookup.loc["ins_gnss_only"]
    current_row = lookup.loc["current_mainline_ins_gnss_odo_nhc"]
    for idx, row in df.iterrows():
        df.at[idx, "phase3_rmse_delta_vs_ins_gnss_m"] = (
            float(row["phase3_rmse_3d_m"]) - float(ins_row["phase3_rmse_3d_m"])
        )
        df.at[idx, "phase3_rmse_improvement_vs_current_m"] = (
            float(current_row["phase3_rmse_3d_m"]) - float(row["phase3_rmse_3d_m"])
        )
        df.at[idx, "phase3_bg_z_peak_reduction_vs_current_degh"] = (
            float(current_row["phase3_bg_z_abs_max_degh"]) - float(row["phase3_bg_z_abs_max_degh"])
        )
        df.at[idx, "phase3_yaw_peak_reduction_vs_current_deg"] = (
            float(current_row["phase3_yaw_abs_max_deg"]) - float(row["phase3_yaw_abs_max_deg"])
        )
    return df


def build_takeaways(case_metrics_df: pd.DataFrame) -> list[str]:
    if case_metrics_df.empty:
        return ["- no cases available"]
    lookup = case_metrics_df.set_index("case_id")
    lines: list[str] = []
    ins = lookup.loc["ins_gnss_only"]
    current = lookup.loc["current_mainline_ins_gnss_odo_nhc"]
    for case_id in lookup.index:
        if case_id in ("ins_gnss_only", "current_mainline_ins_gnss_odo_nhc"):
            continue
        row = lookup.loc[case_id]
        lines.append(
            "- "
            f"`{case_id}`: phase3 RMSE vs current = "
            f"`{format_metric(float(current['phase3_rmse_3d_m']) - float(row['phase3_rmse_3d_m']))} m` improvement, "
            f"vs INS/GNSS gap = "
            f"`{format_metric(float(row['phase3_rmse_3d_m']) - float(ins['phase3_rmse_3d_m']))} m`, "
            f"phase3 bg_z peak reduction = "
            f"`{format_metric(float(current['phase3_bg_z_abs_max_degh']) - float(row['phase3_bg_z_abs_max_degh']))} deg/h`, "
            f"phase3 yaw peak reduction = "
            f"`{format_metric(float(current['phase3_yaw_abs_max_deg']) - float(row['phase3_yaw_abs_max_deg']))} deg`."
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
        "- all cases keep the same staged schedule: `0-200 s` INS/GNSS, `200-700 s` staged joint phase, `700+ s` periodic GNSS outage.",
        "- shared controls remain: strict weak-excitation gate, phase2/phase3 freeze GNSS lever, `fusion.init.odo_scale=1.0`, `fusion.fej.enable=false`.",
        "- `ins_gnss_only` additionally disables ODO/NHC and freezes `odo_scale/mounting/lever_odo` for fairness as a pure INS/GNSS reference.",
        "- mitigation candidates reuse the existing non-debug `enable_bgz_observability_gate` path, with or without covariance forgetting.",
        "",
        "## Case Metrics",
        "",
    ]

    case_rows: list[list[str]] = []
    for _, row in case_metrics_df.sort_values("sort_order").iterrows():
        case_rows.append(
            [
                str(row["case_id"]),
                format_metric(row["phase2_rmse_3d_m"]),
                format_metric(row["phase3_rmse_3d_m"]),
                format_metric(row["phase3_rmse_delta_vs_ins_gnss_m"]),
                format_metric(row["phase3_rmse_improvement_vs_current_m"]),
                format_metric(row["first_divergence_start_t"]),
                format_metric(row["odo_accept_ratio"]),
                format_metric(row["nhc_accept_ratio"]),
                format_metric(row["phase2_bg_z_abs_max_degh"]),
                format_metric(row["phase3_bg_z_abs_max_degh"]),
                format_metric(row["phase2_yaw_abs_max_deg"]),
                format_metric(row["phase3_yaw_abs_max_deg"]),
                format_metric(row["peak_total_mounting_yaw_after_phase1_deg"]),
                format_metric(row["peak_odo_lever_y_after_phase1_m"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "phase2_rmse_3d_m",
                "phase3_rmse_3d_m",
                "phase3_delta_vs_ins_gnss_m",
                "phase3_improve_vs_current_m",
                "first_div_t",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "phase2_bg_z_peak_degh",
                "phase3_bg_z_peak_degh",
                "phase2_yaw_peak_deg",
                "phase3_yaw_peak_deg",
                "peak_total_mounting_yaw_deg",
                "peak_odo_lever_y_m",
            ],
            case_rows,
        )
    )

    lines.extend(["", "## Variant Notes", ""])
    for _, row in case_metrics_df.sort_values("sort_order").iterrows():
        lines.append(f"- `{row['case_id']}`: {row['variant_note']}")

    lines.extend(["", "## Quick Read", ""])
    lines.extend(build_takeaways(case_metrics_df))
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
    (output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


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
    case_specs = CASE_SET_MAP[args.case_set]
    truth_reference = working_motion_truth_reference(base_cfg)
    base_truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())

    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}
    case_metadata: dict[str, dict[str, Any]] = {}

    for spec in case_specs:
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

        row["sort_order"] = spec.sort_order
        row["variant_note"] = metadata["variant_note"]
        row["phase2_rmse_3d_m"] = rmse3d(plot_df, phase1_end_time, phase2_end_time)
        row["phase3_rmse_3d_m"] = rmse3d(plot_df, phase2_end_time, final_time)
        row["phase2_bg_z_abs_max_degh"] = peak_abs(merged_df, "bg_z_degh", phase1_end_time, phase2_end_time)
        row["phase3_bg_z_abs_max_degh"] = peak_abs(merged_df, "bg_z_degh", phase2_end_time, final_time)
        row["phase2_yaw_abs_max_deg"] = peak_abs(plot_df, "yaw_err_deg", phase1_end_time, phase2_end_time)
        row["phase3_yaw_abs_max_deg"] = peak_abs(plot_df, "yaw_err_deg", phase2_end_time, final_time)
        row["peak_total_mounting_yaw_after_phase1_deg"] = peak_abs(
            merged_df, "total_mounting_yaw_deg", phase1_end_time
        )
        row["peak_odo_lever_y_after_phase1_m"] = peak_abs(merged_df, "odo_lever_y_m", phase1_end_time)
        case_rows.append(row)

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df = case_metrics_df.sort_values("sort_order").reset_index(drop=True)
    case_metrics_df = enrich_reference_deltas(case_metrics_df)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

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
    print(rel_from_root(args.output_dir / "summary.md", REPO_ROOT))


if __name__ == "__main__":
    main()
