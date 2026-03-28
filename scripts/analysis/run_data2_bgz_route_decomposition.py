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


EXP_ID_DEFAULT = "EXP-20260325-data2-bgz-route-decomposition-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_bgz_route_decomposition_r1_20260325")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 60.0
PHASE3_GNSS_OFF_DEFAULT = 60.0
CONVERGED_NOISE_SCALE_DEFAULT = 0.1
MECHANISM_WINDOW_OFFSET_DEFAULT = 0.0
MECHANISM_WINDOW_DURATION_DEFAULT = 60.0
BGZ_WRITE_EPS_DEGH_DEFAULT = 1.0e-6
RADPS_TO_DEGH = 180.0 / math.pi * 3600.0


@dataclass(frozen=True)
class RouteCaseSpec:
    case_id: str
    label: str
    color: str
    family: str
    sort_order: int
    fix_mounting_truth: bool = False
    fix_odo_lever_truth: bool = False
    parent_case_id: str | None = None
    debug_odo_disable_bgz_jacobian: bool = False
    debug_odo_disable_bgz_state_update: bool = False
    debug_nhc_disable_bgz_state_update: bool = False


CASE_SPECS: tuple[RouteCaseSpec, ...] = (
    RouteCaseSpec(
        case_id="control_strict_gate_freeze_gnss_lever",
        label="control: strict gate + freeze GNSS lever",
        color="#4c78a8",
        family="control",
        sort_order=0,
    ),
    RouteCaseSpec(
        case_id="fix_odo_lever_truth",
        label="stress A: ODO lever truth fixed",
        color="#54a24b",
        family="fix_odo_lever_truth",
        sort_order=10,
        fix_odo_lever_truth=True,
    ),
    RouteCaseSpec(
        case_id="fix_odo_lever_truth_disable_odo_bgz_jacobian",
        label="stress A + disable ODO bg_z Jacobian",
        color="#2f7f3e",
        family="fix_odo_lever_truth",
        sort_order=11,
        fix_odo_lever_truth=True,
        parent_case_id="fix_odo_lever_truth",
        debug_odo_disable_bgz_jacobian=True,
    ),
    RouteCaseSpec(
        case_id="fix_odo_lever_truth_disable_odo_bgz_state_update",
        label="stress A + disable ODO bg_z state update",
        color="#3d9b55",
        family="fix_odo_lever_truth",
        sort_order=12,
        fix_odo_lever_truth=True,
        parent_case_id="fix_odo_lever_truth",
        debug_odo_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_odo_lever_truth_disable_nhc_bgz_state_update",
        label="stress A + disable NHC bg_z state update",
        color="#69b36d",
        family="fix_odo_lever_truth",
        sort_order=13,
        fix_odo_lever_truth=True,
        parent_case_id="fix_odo_lever_truth",
        debug_nhc_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_odo_lever_truth_disable_both_bgz_state_update",
        label="stress A + disable both bg_z state updates",
        color="#86c88c",
        family="fix_odo_lever_truth",
        sort_order=14,
        fix_odo_lever_truth=True,
        parent_case_id="fix_odo_lever_truth",
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_odo_lever_truth_disable_odo_jacobian_and_both_bgz_state_update",
        label="stress A + disable ODO Jacobian + both bg_z updates",
        color="#a4d9aa",
        family="fix_odo_lever_truth",
        sort_order=15,
        fix_odo_lever_truth=True,
        parent_case_id="fix_odo_lever_truth",
        debug_odo_disable_bgz_jacobian=True,
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_mounting_and_odo_lever_truth",
        label="stress B: mounting + ODO lever truth fixed",
        color="#e45756",
        family="fix_mounting_and_odo_lever_truth",
        sort_order=20,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
    ),
    RouteCaseSpec(
        case_id="fix_mounting_and_odo_lever_truth_disable_odo_bgz_jacobian",
        label="stress B + disable ODO bg_z Jacobian",
        color="#c74443",
        family="fix_mounting_and_odo_lever_truth",
        sort_order=21,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
        parent_case_id="fix_mounting_and_odo_lever_truth",
        debug_odo_disable_bgz_jacobian=True,
    ),
    RouteCaseSpec(
        case_id="fix_mounting_and_odo_lever_truth_disable_odo_bgz_state_update",
        label="stress B + disable ODO bg_z state update",
        color="#d65f5e",
        family="fix_mounting_and_odo_lever_truth",
        sort_order=22,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
        parent_case_id="fix_mounting_and_odo_lever_truth",
        debug_odo_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_mounting_and_odo_lever_truth_disable_nhc_bgz_state_update",
        label="stress B + disable NHC bg_z state update",
        color="#e07b7a",
        family="fix_mounting_and_odo_lever_truth",
        sort_order=23,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
        parent_case_id="fix_mounting_and_odo_lever_truth",
        debug_nhc_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_mounting_and_odo_lever_truth_disable_both_bgz_state_update",
        label="stress B + disable both bg_z state updates",
        color="#eaa0a0",
        family="fix_mounting_and_odo_lever_truth",
        sort_order=24,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
        parent_case_id="fix_mounting_and_odo_lever_truth",
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
    ),
    RouteCaseSpec(
        case_id="fix_mounting_and_odo_lever_truth_disable_odo_jacobian_and_both_bgz_state_update",
        label="stress B + disable ODO Jacobian + both bg_z updates",
        color="#f1bbbb",
        family="fix_mounting_and_odo_lever_truth",
        sort_order=25,
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
        parent_case_id="fix_mounting_and_odo_lever_truth",
        debug_odo_disable_bgz_jacobian=True,
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run bg_z route decomposition on staged data2 cases with mechanism-window logging."
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
    parser.add_argument("--mechanism-window-offset", type=float, default=MECHANISM_WINDOW_OFFSET_DEFAULT)
    parser.add_argument("--mechanism-window-duration", type=float, default=MECHANISM_WINDOW_DURATION_DEFAULT)
    parser.add_argument("--bgz-write-eps-degh", type=float, default=BGZ_WRITE_EPS_DEGH_DEFAULT)
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


def build_route_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: RouteCaseSpec,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    truth_spec = TruthCaseSpec(
        case_id=spec.case_id,
        label=spec.label,
        color=spec.color,
        fix_mounting_truth=spec.fix_mounting_truth,
        fix_odo_lever_truth=spec.fix_odo_lever_truth,
    )
    cfg, overrides, metadata = build_truth_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=truth_spec,
        args=args,
    )

    constraints_cfg = cfg.setdefault("fusion", {}).setdefault("constraints", {})
    window_start = float(metadata["phase1_end_time"] + args.mechanism_window_offset)
    window_end = float(
        min(
            float(base_cfg["fusion"]["finaltime"]),
            window_start + max(0.0, float(args.mechanism_window_duration)),
        )
    )

    constraints_cfg["enable_mechanism_log"] = True
    constraints_cfg["mechanism_log_stride"] = 1
    constraints_cfg["mechanism_log_post_gnss_only"] = False
    constraints_cfg["mechanism_log_start_time"] = window_start
    constraints_cfg["mechanism_log_end_time"] = window_end
    constraints_cfg["debug_odo_disable_bgz_jacobian"] = spec.debug_odo_disable_bgz_jacobian
    constraints_cfg["debug_odo_disable_bgz_state_update"] = spec.debug_odo_disable_bgz_state_update
    constraints_cfg["debug_nhc_disable_bgz_state_update"] = spec.debug_nhc_disable_bgz_state_update

    overrides["fusion.constraints.enable_mechanism_log"] = True
    overrides["fusion.constraints.mechanism_log_stride"] = 1
    overrides["fusion.constraints.mechanism_log_post_gnss_only"] = False
    overrides["fusion.constraints.mechanism_log_start_time"] = window_start
    overrides["fusion.constraints.mechanism_log_end_time"] = window_end
    overrides["fusion.constraints.debug_odo_disable_bgz_jacobian"] = spec.debug_odo_disable_bgz_jacobian
    overrides["fusion.constraints.debug_odo_disable_bgz_state_update"] = spec.debug_odo_disable_bgz_state_update
    overrides["fusion.constraints.debug_nhc_disable_bgz_state_update"] = spec.debug_nhc_disable_bgz_state_update

    route_note_parts: list[str] = []
    if spec.debug_odo_disable_bgz_jacobian:
        route_note_parts.append("disable_odo_bgz_jacobian")
    if spec.debug_odo_disable_bgz_state_update:
        route_note_parts.append("disable_odo_bgz_state_update")
    if spec.debug_nhc_disable_bgz_state_update:
        route_note_parts.append("disable_nhc_bgz_state_update")
    metadata["mechanism_window_start_time"] = window_start
    metadata["mechanism_window_end_time"] = window_end
    metadata["route_note"] = ",".join(route_note_parts) if route_note_parts else "baseline"
    return cfg, overrides, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: RouteCaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    cfg, overrides, metadata = build_route_case_config(
        base_cfg=base_cfg,
        motion_truth_reference=motion_truth_reference,
        case_dir=case_dir,
        spec=spec,
        args=args,
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides, metadata


def load_existing_case_row(case_dir: Path, cfg_path: Path, spec: RouteCaseSpec) -> dict[str, Any]:
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


def mechanism_path_from_sol(sol_path: Path) -> Path:
    return sol_path.parent / f"{sol_path.stem}_mechanism.csv"


def load_mechanism_dataframe(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if df.empty:
        return df
    for col in [
        "t_meas",
        "t_state",
        "dx_bg_z",
        "k_row_bg_z_norm",
        "h_col_bg_z_norm",
        "bg_z_before",
        "bg_z_after",
        "nis",
        "s_trace",
    ]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.sort_values(["t_meas", "tag"]).reset_index(drop=True)
    df["delta_bg_z_radps"] = df["bg_z_after"] - df["bg_z_before"]
    df["delta_bg_z_degh"] = df["delta_bg_z_radps"] * RADPS_TO_DEGH
    df["dx_bg_z_degh"] = df["dx_bg_z"] * RADPS_TO_DEGH
    return df


def summarize_mechanism(
    df: pd.DataFrame,
    case_id: str,
    eps_degh: float,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    base_summary: dict[str, Any] = {
        "mechanism_rows": 0,
        "first_bgz_write_tag": "NA",
        "first_bgz_write_t": float("nan"),
        "first_bgz_write_delta_bgz_degh": float("nan"),
        "first_bgz_write_h_col_bgz_norm": float("nan"),
        "first_bgz_write_k_row_bgz_norm": float("nan"),
        "nhc_mechanism_rows": 0,
        "nhc_bgz_write_count": 0,
        "nhc_first_bgz_write_t": float("nan"),
        "nhc_first_bgz_write_delta_bgz_degh": float("nan"),
        "nhc_cum_abs_delta_bgz_degh": 0.0,
        "nhc_cum_signed_delta_bgz_degh": 0.0,
        "nhc_max_abs_delta_bgz_degh": 0.0,
        "nhc_mean_h_col_bgz_norm": float("nan"),
        "nhc_mean_k_row_bgz_norm": float("nan"),
        "odo_mechanism_rows": 0,
        "odo_bgz_write_count": 0,
        "odo_first_bgz_write_t": float("nan"),
        "odo_first_bgz_write_delta_bgz_degh": float("nan"),
        "odo_cum_abs_delta_bgz_degh": 0.0,
        "odo_cum_signed_delta_bgz_degh": 0.0,
        "odo_max_abs_delta_bgz_degh": 0.0,
        "odo_mean_h_col_bgz_norm": float("nan"),
        "odo_mean_k_row_bgz_norm": float("nan"),
        "dominant_bgz_cum_abs_tag": "NA",
    }
    step_rows: list[dict[str, Any]] = []
    if df.empty:
        return base_summary, step_rows

    work = df[df["tag"].isin(["NHC", "ODO"])].copy()
    base_summary["mechanism_rows"] = int(len(work))
    if work.empty:
        return base_summary, step_rows

    work["bgz_write"] = work["delta_bg_z_degh"].abs() > eps_degh
    first_write_df = work[work["bgz_write"]]
    if not first_write_df.empty:
        first_row = first_write_df.iloc[0]
        base_summary["first_bgz_write_tag"] = str(first_row["tag"])
        base_summary["first_bgz_write_t"] = float(first_row["t_meas"])
        base_summary["first_bgz_write_delta_bgz_degh"] = float(first_row["delta_bg_z_degh"])
        base_summary["first_bgz_write_h_col_bgz_norm"] = float(first_row["h_col_bg_z_norm"])
        base_summary["first_bgz_write_k_row_bgz_norm"] = float(first_row["k_row_bg_z_norm"])

    dominant_tag = "NA"
    dominant_value = -1.0
    for tag in ("NHC", "ODO"):
        tag_df = work[work["tag"] == tag].copy()
        write_df = tag_df[tag_df["bgz_write"]]
        prefix = tag.lower()
        base_summary[f"{prefix}_mechanism_rows"] = int(len(tag_df))
        base_summary[f"{prefix}_bgz_write_count"] = int(len(write_df))
        base_summary[f"{prefix}_cum_abs_delta_bgz_degh"] = float(tag_df["delta_bg_z_degh"].abs().sum())
        base_summary[f"{prefix}_cum_signed_delta_bgz_degh"] = float(tag_df["delta_bg_z_degh"].sum())
        base_summary[f"{prefix}_max_abs_delta_bgz_degh"] = float(tag_df["delta_bg_z_degh"].abs().max()) if not tag_df.empty else 0.0
        base_summary[f"{prefix}_mean_h_col_bgz_norm"] = float(tag_df["h_col_bg_z_norm"].mean()) if not tag_df.empty else float("nan")
        base_summary[f"{prefix}_mean_k_row_bgz_norm"] = float(tag_df["k_row_bg_z_norm"].mean()) if not tag_df.empty else float("nan")
        if not write_df.empty:
            base_summary[f"{prefix}_first_bgz_write_t"] = float(write_df.iloc[0]["t_meas"])
            base_summary[f"{prefix}_first_bgz_write_delta_bgz_degh"] = float(write_df.iloc[0]["delta_bg_z_degh"])
        cum_abs = float(tag_df["delta_bg_z_degh"].abs().sum())
        if cum_abs > dominant_value + 1.0e-12:
            dominant_value = cum_abs
            dominant_tag = tag
        step_rows.append(
            {
                "case_id": case_id,
                "tag": tag,
                "mechanism_rows": int(len(tag_df)),
                "bgz_write_count": int(len(write_df)),
                "first_bgz_write_t": float(write_df.iloc[0]["t_meas"]) if not write_df.empty else float("nan"),
                "first_bgz_write_delta_bgz_degh": float(write_df.iloc[0]["delta_bg_z_degh"]) if not write_df.empty else float("nan"),
                "cum_abs_delta_bgz_degh": cum_abs,
                "cum_signed_delta_bgz_degh": float(tag_df["delta_bg_z_degh"].sum()),
                "max_abs_delta_bgz_degh": float(tag_df["delta_bg_z_degh"].abs().max()) if not tag_df.empty else 0.0,
                "mean_h_col_bgz_norm": float(tag_df["h_col_bg_z_norm"].mean()) if not tag_df.empty else float("nan"),
                "mean_k_row_bgz_norm": float(tag_df["k_row_bg_z_norm"].mean()) if not tag_df.empty else float("nan"),
            }
        )
    if dominant_value <= 1.0e-12:
        dominant_tag = "NA"
    base_summary["dominant_bgz_cum_abs_tag"] = dominant_tag
    return base_summary, step_rows


def enrich_parent_deltas(case_metrics_df: pd.DataFrame) -> pd.DataFrame:
    df = case_metrics_df.copy()
    df["phase3_rmse_improvement_vs_parent_m"] = np.nan
    df["first_divergence_delay_vs_parent_s"] = np.nan
    df["total_cum_abs_delta_bgz_reduction_vs_parent_degh"] = np.nan
    df["parent_first_bgz_write_tag"] = "NA"
    total_abs = df["nhc_cum_abs_delta_bgz_degh"].fillna(0.0) + df["odo_cum_abs_delta_bgz_degh"].fillna(0.0)
    df["total_cum_abs_delta_bgz_degh"] = total_abs
    lookup = df.set_index("case_id")
    for idx, row in df.iterrows():
        parent = row.get("parent_case_id")
        if not isinstance(parent, str) or parent == "":
            continue
        if parent not in lookup.index:
            continue
        parent_row = lookup.loc[parent]
        if math.isfinite(float(parent_row["phase3_rmse_3d_m"])) and math.isfinite(float(row["phase3_rmse_3d_m"])):
            df.at[idx, "phase3_rmse_improvement_vs_parent_m"] = float(parent_row["phase3_rmse_3d_m"]) - float(row["phase3_rmse_3d_m"])
        if math.isfinite(float(parent_row["first_divergence_start_t"])) and math.isfinite(float(row["first_divergence_start_t"])):
            df.at[idx, "first_divergence_delay_vs_parent_s"] = float(row["first_divergence_start_t"]) - float(parent_row["first_divergence_start_t"])
        elif not math.isfinite(float(row["first_divergence_start_t"])) and math.isfinite(float(parent_row["first_divergence_start_t"])):
            df.at[idx, "first_divergence_delay_vs_parent_s"] = float("inf")
        df.at[idx, "total_cum_abs_delta_bgz_reduction_vs_parent_degh"] = float(parent_row["total_cum_abs_delta_bgz_degh"]) - float(row["total_cum_abs_delta_bgz_degh"])
        df.at[idx, "parent_first_bgz_write_tag"] = str(parent_row["first_bgz_write_tag"])
    return df


def write_summary(
    output_dir: Path,
    exp_id: str,
    case_metrics_df: pd.DataFrame,
    bgz_step_metrics_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Setup",
        "",
        "- staged control stays unchanged: strict weak-excitation gate, phase2/phase3 freeze GNSS lever, `fusion.init.odo_scale=1.0`.",
        "- stressed cases include both `fix_odo_lever_truth` and `fix_mounting_and_odo_lever_truth`.",
        (
            "- mechanism log only records the phase2 entry window "
            f"`[{format_metric(manifest['mechanism_window']['start_time'])}, "
            f"{format_metric(manifest['mechanism_window']['end_time'])}] s` "
            "to keep per-update bg_z tracing tractable."
        ),
        "",
        "## Case Metrics",
        "",
    ]
    case_rows: list[list[str]] = []
    ordered_df = case_metrics_df.sort_values("sort_order")
    for _, row in ordered_df.iterrows():
        delay_value = row.get("first_divergence_delay_vs_parent_s")
        delay_text = "no_parent"
        if isinstance(row.get("parent_case_id"), str) and row.get("parent_case_id"):
            if math.isinf(float(delay_value)):
                delay_text = "no_divergence"
            else:
                delay_text = format_metric(delay_value)
        case_rows.append(
            [
                str(row["case_id"]),
                str(row["parent_case_id"]) if isinstance(row["parent_case_id"], str) and row["parent_case_id"] else "NA",
                format_metric(row["phase2_rmse_3d_m"]),
                format_metric(row["phase3_rmse_3d_m"]),
                format_metric(row["phase3_rmse_improvement_vs_parent_m"]),
                format_metric(row["first_divergence_start_t"]),
                delay_text,
                format_metric(row["odo_accept_ratio"]),
                format_metric(row["nhc_accept_ratio"]),
                str(row["first_bgz_write_tag"]),
                format_metric(row["first_bgz_write_t"]),
                format_metric(row["first_bgz_write_delta_bgz_degh"]),
                str(row["dominant_bgz_cum_abs_tag"]),
                format_metric(row["total_cum_abs_delta_bgz_degh"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "parent_case_id",
                "phase2_rmse_3d_m",
                "phase3_rmse_3d_m",
                "phase3_improve_vs_parent_m",
                "first_div_t",
                "first_div_delay_s",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "first_bgz_write_tag",
                "first_bgz_write_t",
                "first_bgz_write_delta_degh",
                "dominant_bgz_tag",
                "total_cum_abs_delta_degh",
            ],
            case_rows,
        )
    )
    lines.extend(["", "## Per-Sensor bg_z Step Metrics", ""])
    step_rows: list[list[str]] = []
    for _, row in bgz_step_metrics_df.sort_values(["case_id", "tag"]).iterrows():
        step_rows.append(
            [
                str(row["case_id"]),
                str(row["tag"]),
                format_metric(row["mechanism_rows"]),
                format_metric(row["bgz_write_count"]),
                format_metric(row["first_bgz_write_t"]),
                format_metric(row["first_bgz_write_delta_bgz_degh"]),
                format_metric(row["cum_abs_delta_bgz_degh"]),
                format_metric(row["cum_signed_delta_bgz_degh"]),
                format_metric(row["max_abs_delta_bgz_degh"]),
                format_metric(row["mean_h_col_bgz_norm"]),
                format_metric(row["mean_k_row_bgz_norm"]),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "tag",
                "mechanism_rows",
                "bgz_write_count",
                "first_write_t",
                "first_write_delta_degh",
                "cum_abs_delta_degh",
                "cum_signed_delta_degh",
                "max_abs_delta_degh",
                "mean_h_bgz_norm",
                "mean_k_bgz_norm",
            ],
            step_rows,
        )
    )
    lines.extend(
        [
            "",
            "## Manifest",
            "",
            f"- manifest: `{rel_from_root(output_dir / 'manifest.json', REPO_ROOT)}`",
            f"- case_metrics: `{rel_from_root(output_dir / 'case_metrics.csv', REPO_ROOT)}`",
            f"- bgz_step_metrics: `{rel_from_root(output_dir / 'bgz_step_metrics.csv', REPO_ROOT)}`",
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
    step_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}
    case_metadata: dict[str, dict[str, Any]] = {}
    case_mechanism_paths: dict[str, str] = {}

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
        mechanism_path = mechanism_path_from_sol(sol_path)
        if not mechanism_path.exists():
            raise RuntimeError(f"missing mechanism log for {spec.case_id}: {mechanism_path}")
        case_mechanism_paths[spec.case_id] = rel_from_root(mechanism_path, REPO_ROOT)

        merged_df = merge_case_outputs(sol_path, state_series_path)
        truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
        motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, truth_reference)
        plot_df = build_plot_frame(merged_df, truth_interp_df, base_truth_reference, motion_df)

        phase1_end_time = float(metadata["phase1_end_time"])
        phase2_end_time = float(metadata["phase2_end_time"])
        final_time = float(base_cfg["fusion"]["finaltime"])
        row["family"] = spec.family
        row["sort_order"] = spec.sort_order
        row["parent_case_id"] = spec.parent_case_id or ""
        row["route_note"] = metadata["route_note"]
        row["truth_note"] = metadata["truth_note"]
        row["mechanism_window_start_time"] = metadata["mechanism_window_start_time"]
        row["mechanism_window_end_time"] = metadata["mechanism_window_end_time"]
        row["phase2_rmse_3d_m"] = rmse3d(plot_df, phase1_end_time, phase2_end_time)
        row["phase3_rmse_3d_m"] = rmse3d(plot_df, phase2_end_time, final_time)
        row["phase2_bg_z_abs_max_degh"] = peak_abs(plot_df, "bg_z_degh", phase1_end_time, phase2_end_time)
        row["phase3_bg_z_abs_max_degh"] = peak_abs(plot_df, "bg_z_degh", phase2_end_time, final_time)
        row["phase2_yaw_abs_max_deg"] = peak_abs(plot_df, "yaw_err_deg", phase1_end_time, phase2_end_time)
        row["phase3_yaw_abs_max_deg"] = peak_abs(plot_df, "yaw_err_deg", phase2_end_time, final_time)

        mechanism_df = load_mechanism_dataframe(mechanism_path)
        mechanism_summary, case_step_rows = summarize_mechanism(
            mechanism_df,
            case_id=spec.case_id,
            eps_degh=float(args.bgz_write_eps_degh),
        )
        row.update(mechanism_summary)
        step_rows.extend(case_step_rows)
        case_rows.append(row)

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df = enrich_parent_deltas(case_metrics_df)
    case_metrics_df = case_metrics_df.sort_values("sort_order").reset_index(drop=True)
    bgz_step_metrics_df = pd.DataFrame(step_rows)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    bgz_step_metrics_path = args.output_dir / "bgz_step_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    bgz_step_metrics_df.to_csv(bgz_step_metrics_path, index=False, encoding="utf-8-sig")

    first_case = case_metrics_df.iloc[0]
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "bgz_step_metrics_csv": rel_from_root(bgz_step_metrics_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_overrides": case_overrides,
        "case_metadata": case_metadata,
        "case_mechanism_paths": case_mechanism_paths,
        "mechanism_window": {
            "start_time": float(first_case["mechanism_window_start_time"]),
            "end_time": float(first_case["mechanism_window_end_time"]),
            "duration_s": float(first_case["mechanism_window_end_time"] - first_case["mechanism_window_start_time"]),
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
    write_summary(args.output_dir, args.exp_id, case_metrics_df, bgz_step_metrics_df, manifest)
    print(rel_from_root(args.output_dir / "summary.md", REPO_ROOT))


if __name__ == "__main__":
    main()
