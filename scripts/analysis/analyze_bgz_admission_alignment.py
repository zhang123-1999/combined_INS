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


EXP_ID_DEFAULT = "EXP-20260325-data2-nhc-admission-alignment-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_nhc_admission_alignment_r1_20260325")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 60.0
PHASE3_GNSS_OFF_DEFAULT = 60.0
CONVERGED_NOISE_SCALE_DEFAULT = 0.1


@dataclass(frozen=True)
class AdmissionCaseSpec:
    case_id: str
    label: str
    color: str
    sort_order: int
    nhc_admission_velocity_source: str


CASE_SPECS: tuple[AdmissionCaseSpec, ...] = (
    AdmissionCaseSpec(
        case_id="baseline_current_v_b_gate",
        label="baseline: current v_b gate",
        color="#f58518",
        sort_order=0,
        nhc_admission_velocity_source="v_b",
    ),
    AdmissionCaseSpec(
        case_id="nhc_gate_use_v_wheel_b",
        label="variant: v_wheel_b gate",
        color="#54a24b",
        sort_order=1,
        nhc_admission_velocity_source="v_wheel_b",
    ),
    AdmissionCaseSpec(
        case_id="nhc_gate_use_v_v",
        label="variant: v_v gate",
        color="#4c78a8",
        sort_order=2,
        nhc_admission_velocity_source="v_v",
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run EXP-20260325-data2-nhc-admission-alignment-r1."
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


def peak_abs(df: pd.DataFrame, column: str, start_time: float, end_time: float) -> float:
    subset = df[(df["timestamp"] >= start_time) & (df["timestamp"] <= end_time)]
    if subset.empty:
        return float("nan")
    return float(np.max(np.abs(subset[column].to_numpy(dtype=float))))


def ratio_from_mask(mask: np.ndarray) -> float:
    if mask.size == 0:
        return float("nan")
    return float(np.mean(mask.astype(float)))


def build_admission_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: AdmissionCaseSpec,
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

    constraints_cfg = cfg.setdefault("fusion", {}).setdefault("constraints", {})
    constraints_cfg["enable_nhc_admission_log"] = True
    constraints_cfg["nhc_admission_velocity_source"] = spec.nhc_admission_velocity_source

    overrides["fusion.constraints.enable_nhc_admission_log"] = True
    overrides["fusion.constraints.nhc_admission_velocity_source"] = spec.nhc_admission_velocity_source
    metadata["variant_note"] = (
        "staged current-mainline control; "
        f"NHC admission source={spec.nhc_admission_velocity_source}"
    )
    return cfg, overrides, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: AdmissionCaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    cfg, overrides, metadata = build_admission_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=spec,
        args=args,
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides, metadata


def admission_log_path_from_sol(sol_rel_path: str) -> Path:
    sol_path = (REPO_ROOT / sol_rel_path).resolve()
    return sol_path.with_name(f"{sol_path.stem}_nhc_admission.csv")


def summarize_admission_log(case_id: str, log_path: Path) -> dict[str, Any]:
    if not log_path.exists():
        raise FileNotFoundError(f"missing nhc admission log: {log_path}")
    df = pd.read_csv(log_path)
    if df.empty:
        return {
            "case_id": case_id,
            "selected_source": "NA",
            "admission_samples": 0,
            "selected_precheck_accept_ratio": float("nan"),
            "accept_v_b_ratio": float("nan"),
            "accept_v_wheel_b_ratio": float("nan"),
            "accept_v_v_ratio": float("nan"),
            "current_vs_selected_mismatch_ratio": float("nan"),
            "current_accept_selected_reject_ratio": float("nan"),
            "current_reject_selected_accept_ratio": float("nan"),
            "current_vs_v_wheel_b_mismatch_ratio": float("nan"),
            "current_vs_v_v_mismatch_ratio": float("nan"),
            "selected_forward_reject_ratio": float("nan"),
            "selected_lat_vert_reject_ratio": float("nan"),
        }

    selected_source = str(df["selected_source"].iloc[0])
    selected_accept = df["selected_accept"].to_numpy(dtype=bool)
    accept_v_b = df["accept_v_b"].to_numpy(dtype=bool)
    accept_v_wheel_b = df["accept_v_wheel_b"].to_numpy(dtype=bool)
    accept_v_v = df["accept_v_v"].to_numpy(dtype=bool)
    below_forward_v_b = df["below_forward_v_b"].to_numpy(dtype=bool)
    below_forward_v_wheel_b = df["below_forward_v_wheel_b"].to_numpy(dtype=bool)
    below_forward_v_v = df["below_forward_v_v"].to_numpy(dtype=bool)
    exceed_v_b = df["exceed_lat_vert_v_b"].to_numpy(dtype=bool)
    exceed_v_wheel_b = df["exceed_lat_vert_v_wheel_b"].to_numpy(dtype=bool)
    exceed_v_v = df["exceed_lat_vert_v_v"].to_numpy(dtype=bool)

    selected_forward_reject = below_forward_v_b
    selected_lat_vert_reject = exceed_v_b
    if selected_source == "v_wheel_b":
        selected_forward_reject = below_forward_v_wheel_b
        selected_lat_vert_reject = exceed_v_wheel_b
    elif selected_source == "v_v":
        selected_forward_reject = below_forward_v_v
        selected_lat_vert_reject = exceed_v_v

    return {
        "case_id": case_id,
        "selected_source": selected_source,
        "admission_samples": int(df.shape[0]),
        "selected_precheck_accept_ratio": ratio_from_mask(selected_accept),
        "accept_v_b_ratio": ratio_from_mask(accept_v_b),
        "accept_v_wheel_b_ratio": ratio_from_mask(accept_v_wheel_b),
        "accept_v_v_ratio": ratio_from_mask(accept_v_v),
        "current_vs_selected_mismatch_ratio": ratio_from_mask(accept_v_b != selected_accept),
        "current_accept_selected_reject_ratio": ratio_from_mask(accept_v_b & (~selected_accept)),
        "current_reject_selected_accept_ratio": ratio_from_mask((~accept_v_b) & selected_accept),
        "current_vs_v_wheel_b_mismatch_ratio": ratio_from_mask(accept_v_b != accept_v_wheel_b),
        "current_vs_v_v_mismatch_ratio": ratio_from_mask(accept_v_b != accept_v_v),
        "selected_forward_reject_ratio": ratio_from_mask(selected_forward_reject),
        "selected_lat_vert_reject_ratio": ratio_from_mask(selected_lat_vert_reject),
    }


def enrich_case_deltas(case_metrics_df: pd.DataFrame) -> pd.DataFrame:
    df = case_metrics_df.copy()
    df["phase2_rmse_improvement_vs_current_m"] = np.nan
    df["phase3_rmse_improvement_vs_current_m"] = np.nan
    df["phase2_bg_z_reduction_vs_current_degh"] = np.nan
    df["phase3_bg_z_reduction_vs_current_degh"] = np.nan
    if "baseline_current_v_b_gate" not in set(df["case_id"]):
        return df
    current = df.set_index("case_id").loc["baseline_current_v_b_gate"]
    for idx, row in df.iterrows():
        df.at[idx, "phase2_rmse_improvement_vs_current_m"] = (
            float(current["phase2_rmse_3d_m"]) - float(row["phase2_rmse_3d_m"])
        )
        df.at[idx, "phase3_rmse_improvement_vs_current_m"] = (
            float(current["phase3_rmse_3d_m"]) - float(row["phase3_rmse_3d_m"])
        )
        df.at[idx, "phase2_bg_z_reduction_vs_current_degh"] = (
            float(current["phase2_bg_z_abs_max_degh"]) - float(row["phase2_bg_z_abs_max_degh"])
        )
        df.at[idx, "phase3_bg_z_reduction_vs_current_degh"] = (
            float(current["phase3_bg_z_abs_max_degh"]) - float(row["phase3_bg_z_abs_max_degh"])
        )
    return df


def build_takeaways(case_metrics_df: pd.DataFrame) -> list[str]:
    if case_metrics_df.empty:
        return ["- no cases available"]
    lookup = case_metrics_df.set_index("case_id")
    current = lookup.loc["baseline_current_v_b_gate"]
    lines: list[str] = []
    for case_id in lookup.index:
        if case_id == "baseline_current_v_b_gate":
            continue
        row = lookup.loc[case_id]
        lines.append(
            "- "
            f"`{case_id}`: admission mismatch vs current `v_b` = "
            f"`{format_metric(row['current_vs_selected_mismatch_ratio'])}`, "
            f"`current_accept->selected_reject={format_metric(row['current_accept_selected_reject_ratio'])}`, "
            f"`current_reject->selected_accept={format_metric(row['current_reject_selected_accept_ratio'])}`; "
            f"phase3 RMSE improvement vs current = "
            f"`{format_metric(float(current['phase3_rmse_3d_m']) - float(row['phase3_rmse_3d_m']))} m`, "
            f"phase3 bg_z peak reduction = "
            f"`{format_metric(float(current['phase3_bg_z_abs_max_degh']) - float(row['phase3_bg_z_abs_max_degh']))} deg/h`."
        )
    return lines


def write_summary(
    output_dir: Path,
    exp_id: str,
    case_metrics_df: pd.DataFrame,
    admission_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Setup",
        "",
        "- all cases keep the staged current-mainline controls unchanged: strict weak-excitation gate, phase2/phase3 freeze GNSS lever, `fusion.init.odo_scale=1.0`, `fusion.fej.enable=false`.",
        "- the only intentional runtime change is `fusion.constraints.nhc_admission_velocity_source`.",
        "- every case enables `fusion.constraints.enable_nhc_admission_log=true` and writes per-attempt precheck decisions for `v_b`, `v_wheel_b`, and `v_v`.",
        "",
        "## Case Metrics",
        "",
    ]

    case_rows: list[list[str]] = []
    for _, row in case_metrics_df.sort_values("sort_order").iterrows():
        case_rows.append(
            [
                str(row["case_id"]),
                str(row["selected_source"]),
                format_metric(row["phase2_rmse_3d_m"]),
                format_metric(row["phase3_rmse_3d_m"]),
                format_metric(row["phase2_rmse_improvement_vs_current_m"]),
                format_metric(row["phase3_rmse_improvement_vs_current_m"]),
                format_metric(row["first_divergence_start_t"]),
                format_metric(row["phase2_bg_z_abs_max_degh"]),
                format_metric(row["phase3_bg_z_abs_max_degh"]),
                format_metric(row["odo_accept_ratio"]),
                format_metric(row["nhc_accept_ratio"]),
                format_metric(row["selected_precheck_accept_ratio"]),
                format_metric(row["current_vs_selected_mismatch_ratio"]),
                format_metric(row["current_accept_selected_reject_ratio"]),
                format_metric(row["current_reject_selected_accept_ratio"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "selected_source",
                "phase2_rmse_3d_m",
                "phase3_rmse_3d_m",
                "phase2_improve_vs_current_m",
                "phase3_improve_vs_current_m",
                "first_div_t",
                "phase2_bg_z_peak_degh",
                "phase3_bg_z_peak_degh",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "selected_precheck_accept_ratio",
                "mismatch_vs_current",
                "current_accept_selected_reject",
                "current_reject_selected_accept",
            ],
            case_rows,
        )
    )

    lines.extend(["", "## Admission Audit", ""])
    audit_rows: list[list[str]] = []
    for _, row in admission_df.sort_values("sort_order").iterrows():
        audit_rows.append(
            [
                str(row["case_id"]),
                str(row["selected_source"]),
                format_metric(row["admission_samples"]),
                format_metric(row["accept_v_b_ratio"]),
                format_metric(row["accept_v_wheel_b_ratio"]),
                format_metric(row["accept_v_v_ratio"]),
                format_metric(row["current_vs_v_wheel_b_mismatch_ratio"]),
                format_metric(row["current_vs_v_v_mismatch_ratio"]),
                format_metric(row["selected_forward_reject_ratio"]),
                format_metric(row["selected_lat_vert_reject_ratio"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "selected_source",
                "samples",
                "accept_v_b",
                "accept_v_wheel_b",
                "accept_v_v",
                "current_vs_v_wheel_b",
                "current_vs_v_v",
                "selected_forward_reject",
                "selected_lat_vert_reject",
            ],
            audit_rows,
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
            f"- admission_mismatch_summary: `{rel_from_root(output_dir / 'admission_mismatch_summary.csv', REPO_ROOT)}`",
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
    truth_reference = working_motion_truth_reference(base_cfg)
    base_truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())

    case_rows: list[dict[str, Any]] = []
    admission_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_admission_logs: dict[str, str] = {}
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

        admission_log_path = admission_log_path_from_sol(row["sol_path"])
        case_admission_logs[spec.case_id] = rel_from_root(admission_log_path, REPO_ROOT)
        admission_row = summarize_admission_log(spec.case_id, admission_log_path)
        admission_row["sort_order"] = spec.sort_order
        admission_rows.append(admission_row)

        row["sort_order"] = spec.sort_order
        row["variant_note"] = metadata["variant_note"]
        row["selected_source"] = admission_row["selected_source"]
        row["selected_precheck_accept_ratio"] = admission_row["selected_precheck_accept_ratio"]
        row["current_vs_selected_mismatch_ratio"] = admission_row["current_vs_selected_mismatch_ratio"]
        row["current_accept_selected_reject_ratio"] = admission_row["current_accept_selected_reject_ratio"]
        row["current_reject_selected_accept_ratio"] = admission_row["current_reject_selected_accept_ratio"]
        row["phase2_rmse_3d_m"] = rmse3d(plot_df, phase1_end_time, phase2_end_time)
        row["phase3_rmse_3d_m"] = rmse3d(plot_df, phase2_end_time, final_time)
        row["phase2_bg_z_abs_max_degh"] = peak_abs(merged_df, "bg_z_degh", phase1_end_time, phase2_end_time)
        row["phase3_bg_z_abs_max_degh"] = peak_abs(merged_df, "bg_z_degh", phase2_end_time, final_time)
        case_rows.append(row)

    admission_df = pd.DataFrame(admission_rows).sort_values("sort_order").reset_index(drop=True)
    case_metrics_df = pd.DataFrame(case_rows).sort_values("sort_order").reset_index(drop=True)
    case_metrics_df = case_metrics_df.merge(
        admission_df[
            [
                "case_id",
                "selected_source",
                "selected_precheck_accept_ratio",
                "current_vs_selected_mismatch_ratio",
                "current_accept_selected_reject_ratio",
                "current_reject_selected_accept_ratio",
            ]
        ],
        on="case_id",
        how="left",
        suffixes=("", "_admission"),
    )
    for column in (
        "selected_source",
        "selected_precheck_accept_ratio",
        "current_vs_selected_mismatch_ratio",
        "current_accept_selected_reject_ratio",
        "current_reject_selected_accept_ratio",
    ):
        adm_col = f"{column}_admission"
        if adm_col in case_metrics_df.columns:
            case_metrics_df[column] = case_metrics_df[adm_col]
            case_metrics_df = case_metrics_df.drop(columns=[adm_col])
    case_metrics_df = enrich_case_deltas(case_metrics_df)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    admission_path = args.output_dir / "admission_mismatch_summary.csv"
    admission_df.to_csv(admission_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "admission_mismatch_summary_csv": rel_from_root(admission_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_admission_logs": case_admission_logs,
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
    write_summary(args.output_dir, args.exp_id, case_metrics_df, admission_df, manifest)
    print(rel_from_root(args.output_dir / "summary.md", REPO_ROOT))


if __name__ == "__main__":
    main()
