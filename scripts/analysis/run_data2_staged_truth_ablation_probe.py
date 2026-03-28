import argparse
import copy
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
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
    build_case_config as build_baseline_case_config,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    build_truth_reference,
    downsample_for_plot,
    json_safe,
    reset_directory,
    run_command,
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
from scripts.analysis.run_data2_staged_estimation import (  # noqa: E402
    build_periodic_gnss_windows,
    phase_absolute_times,
    scaled_noise_override,
)


EXP_ID_DEFAULT = "EXP-20260324-data2-staged-truth-ablation-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_staged_truth_ablation_r1_20260324")
BASE_CONFIG_DEFAULT = Path("config_data2_research_seed_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 60.0
PHASE3_GNSS_OFF_DEFAULT = 60.0
CONVERGED_NOISE_SCALE_DEFAULT = 0.1
WORKING_TOTAL_MOUNTING_DEG = {"roll": 0.0, "pitch": 0.36, "yaw": 0.84}

STRICT_WEAK_GATE = {
    "disable_nhc_when_weak_excitation": True,
    "disable_odo_when_weak_excitation": True,
    "excitation_min_speed": 2.0,
    "excitation_min_yaw_rate": 0.05,
    "excitation_min_lateral_acc": 0.5,
}


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    fix_mounting_truth: bool = False
    fix_odo_lever_truth: bool = False


@dataclass(frozen=True)
class PlotColumn:
    key: str
    label: str
    unit: str


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        case_id="control_strict_gate_freeze_gnss_lever",
        label="control: strict gate + freeze GNSS lever",
        color="#4c78a8",
    ),
    CaseSpec(
        case_id="fix_mounting_truth",
        label="mounting truth fixed",
        color="#f58518",
        fix_mounting_truth=True,
    ),
    CaseSpec(
        case_id="fix_odo_lever_truth",
        label="ODO lever truth fixed",
        color="#54a24b",
        fix_odo_lever_truth=True,
    ),
    CaseSpec(
        case_id="fix_mounting_and_odo_lever_truth",
        label="mounting + ODO lever truth fixed",
        color="#e45756",
        fix_mounting_truth=True,
        fix_odo_lever_truth=True,
    ),
)
CASE_MAP = {spec.case_id: spec for spec in CASE_SPECS}

NAV_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("p_n_err_m", "p_n error", "m"),
    PlotColumn("p_e_err_m", "p_e error", "m"),
    PlotColumn("p_u_err_m", "p_u error", "m"),
    PlotColumn("v_n_err_mps", "v_n error", "m/s"),
    PlotColumn("v_e_err_mps", "v_e error", "m/s"),
    PlotColumn("v_u_err_mps", "v_u error", "m/s"),
    PlotColumn("roll_err_deg", "roll error", "deg"),
    PlotColumn("pitch_err_deg", "pitch error", "deg"),
    PlotColumn("yaw_err_deg", "yaw error", "deg"),
)

KEY_STATE_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("ba_x_mgal", "ba_x", "mGal"),
    PlotColumn("bg_z_degh", "bg_z", "deg/h"),
    PlotColumn("sa_x_ppm", "sa_x", "ppm"),
    PlotColumn("odo_scale_state", "odo_scale", "1"),
    PlotColumn("total_mounting_yaw_deg", "total_mounting_yaw", "deg"),
    PlotColumn("odo_lever_y_m", "odo_lever_y", "m"),
    PlotColumn("gnss_lever_y_m", "gnss_lever_y", "m"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run staged truth-ablation probes with strict weak gate, phase2 GNSS lever freeze, and odo_scale=1.0."
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
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    return args


def working_motion_truth_reference(base_cfg: dict[str, Any]) -> dict[str, Any]:
    truth_reference = build_truth_reference(base_cfg)
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["roll"] = float(WORKING_TOTAL_MOUNTING_DEG["roll"])
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["pitch"] = float(WORKING_TOTAL_MOUNTING_DEG["pitch"])
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["yaw"] = float(WORKING_TOTAL_MOUNTING_DEG["yaw"])
    truth_reference["sources"]["mounting_total_truth"]["source"] = (
        "working ODO/NHC mounting convention used in this staged truth-ablation probe"
    )
    return truth_reference


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


def phase_constraints() -> dict[str, Any]:
    return {key: value for key, value in STRICT_WEAK_GATE.items()}


def build_runtime_phases(
    cfg: dict[str, Any],
    phase1_end_time: float,
    phase2_end_time: float,
    final_time: float,
    noise_scale: float,
) -> list[dict[str, Any]]:
    converged_noise = scaled_noise_override(cfg["fusion"]["noise"], noise_scale)
    phase_ablation = {"disable_gnss_lever_arm": True}
    phase_constraints_cfg = phase_constraints()
    return [
        {
            "name": "phase1_ins_gnss_freeze_odo_nhc_states",
            "start_time": float(cfg["fusion"]["starttime"]),
            "end_time": float(phase1_end_time),
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        },
        {
            "name": "phase2_strict_gate_freeze_gnss_lever",
            "start_time": float(phase1_end_time),
            "end_time": float(phase2_end_time),
            "noise": converged_noise,
            "ablation": phase_ablation,
            "constraints": phase_constraints_cfg,
        },
        {
            "name": "phase3_periodic_gnss_outage_strict_gate_freeze_gnss_lever",
            "start_time": float(phase2_end_time),
            "end_time": float(final_time),
            "noise": converged_noise,
            "ablation": phase_ablation,
            "constraints": phase_constraints_cfg,
        },
    ]


def build_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    baseline_spec = BaselineCaseSpec(
        case_id=spec.case_id,
        label=spec.label,
        color=spec.color,
        filter_mode="ESKF",
        enable_fej=False,
        enable_runtime_anchor=False,
    )
    cfg, overrides = build_baseline_case_config(base_cfg, case_dir, baseline_spec)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    start_time = float(fusion["starttime"])
    final_time = float(fusion["finaltime"])
    phase1_end_time, phase2_end_time = phase_absolute_times(
        start_time, args.phase1_end_offset, args.phase2_end_offset
    )
    on_windows, off_windows = build_periodic_gnss_windows(
        start_time=start_time,
        final_time=final_time,
        phase2_end_time=phase2_end_time,
        phase3_on_duration=args.phase3_gnss_on,
        phase3_off_duration=args.phase3_gnss_off,
    )

    ablation_cfg["disable_mounting_roll"] = False
    ablation_cfg["disable_gnss_lever_z"] = False
    init_cfg["odo_scale"] = 1.0
    fusion["gnss_schedule"] = {
        "enabled": True,
        "enabled_windows": [
            {"start_time": float(win_start), "end_time": float(win_end)}
            for win_start, win_end in on_windows
        ],
    }
    fusion["runtime_phases"] = build_runtime_phases(
        cfg=cfg,
        phase1_end_time=phase1_end_time,
        phase2_end_time=phase2_end_time,
        final_time=final_time,
        noise_scale=args.converged_noise_scale,
    )
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
    constraints_cfg["enable_mechanism_log"] = False

    overrides["fusion.ablation.disable_mounting_roll"] = False
    overrides["fusion.ablation.disable_gnss_lever_z"] = False
    overrides["fusion.init.odo_scale"] = 1.0
    overrides["fusion.gnss_schedule.enabled"] = True
    overrides["fusion.gnss_schedule.enabled_windows"] = [
        {"start_time": float(win_start), "end_time": float(win_end)}
        for win_start, win_end in on_windows
    ]
    overrides["fusion.runtime_phases"] = fusion["runtime_phases"]
    overrides["fusion.constraints.enable_diagnostics"] = True
    overrides["fusion.constraints.enable_consistency_log"] = True
    overrides["fusion.constraints.enable_mechanism_log"] = False

    truth_notes: list[str] = []
    if spec.fix_mounting_truth:
        constraints_mount = list(map(float, constraints_cfg.get("imu_mounting_angle", [0.0, 0.0, 0.0])))
        constraints_mount[0] = float(WORKING_TOTAL_MOUNTING_DEG["roll"])
        constraints_mount[1] = float(WORKING_TOTAL_MOUNTING_DEG["pitch"])
        constraints_mount[2] = float(WORKING_TOTAL_MOUNTING_DEG["yaw"])
        constraints_cfg["imu_mounting_angle"] = constraints_mount
        init_cfg["mounting_pitch0"] = 0.0
        init_cfg["mounting_yaw0"] = 0.0
        ablation_cfg["disable_mounting"] = True
        overrides["fusion.constraints.imu_mounting_angle"] = constraints_mount
        overrides["fusion.init.mounting_pitch0"] = 0.0
        overrides["fusion.init.mounting_yaw0"] = 0.0
        overrides["fusion.ablation.disable_mounting"] = True
        truth_notes.append("mounting fixed to working total truth (pitch=0.36 deg, yaw=0.84 deg)")

    if spec.fix_odo_lever_truth:
        odo_lever_truth = [float(x) for x in motion_truth_reference["sources"]["odo_lever_truth"]["value_m"]]
        constraints_cfg["odo_lever_arm"] = odo_lever_truth
        init_cfg["lever_arm0"] = odo_lever_truth
        ablation_cfg["disable_odo_lever_arm"] = True
        overrides["fusion.constraints.odo_lever_arm"] = odo_lever_truth
        overrides["fusion.init.lever_arm0"] = odo_lever_truth
        overrides["fusion.ablation.disable_odo_lever_arm"] = True
        truth_notes.append(f"ODO lever fixed to README truth {odo_lever_truth}")

    metadata = {
        "phase1_end_time": phase1_end_time,
        "phase2_end_time": phase2_end_time,
        "gnss_on_windows": on_windows,
        "gnss_off_windows": off_windows,
        "strict_gate": phase_constraints(),
        "truth_note": "; ".join(truth_notes) if truth_notes else "none",
    }
    return cfg, overrides, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    motion_truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    cfg, overrides, metadata = build_case_config(base_cfg, motion_truth_reference, case_dir, spec, args)
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides, metadata


def boundary_lines(ax: plt.Axes, phase1_end_time: float, phase2_end_time: float) -> None:
    ax.axvline(phase1_end_time, color="#333333", linestyle="--", linewidth=0.9)
    ax.axvline(phase2_end_time, color="#333333", linestyle="--", linewidth=0.9)


def shade_gnss_off_windows(ax: plt.Axes, off_windows: list[tuple[float, float]]) -> None:
    for start_time, end_time in off_windows:
        ax.axvspan(start_time, end_time, color="#f4d3d3", alpha=0.35)


def plot_column_grid(
    case_frames: dict[str, pd.DataFrame],
    case_ids: list[str],
    columns: tuple[PlotColumn, ...],
    output_path: Path,
    title: str,
    phase1_end_time: float,
    phase2_end_time: float,
    gnss_off_windows: list[tuple[float, float]],
    time_window: tuple[float, float] | None = None,
) -> None:
    n_cols = 3
    n_rows = math.ceil(len(columns) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 2.9 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, column in enumerate(columns):
        ax = axes_flat[idx]
        shade_gnss_off_windows(ax, gnss_off_windows)
        for case_id in case_ids:
            case_spec = CASE_MAP[case_id]
            df = case_frames[case_id]
            work = df
            if time_window is not None:
                work = work[(work["timestamp"] >= time_window[0]) & (work["timestamp"] <= time_window[1])]
            t = work["timestamp"].to_numpy(dtype=float)
            y = work[column.key].to_numpy(dtype=float)
            t_plot, y_plot = downsample_for_plot(t, y)
            ax.plot(t_plot, y_plot, linewidth=0.95, color=case_spec.color, label=case_spec.label)
        boundary_lines(ax, phase1_end_time, phase2_end_time)
        ax.set_title(f"{column.label} [{column.unit}]", fontsize=9)
        ax.grid(alpha=0.2)
        ax.tick_params(axis="both", labelsize=7)
        if time_window is not None:
            ax.set_xlim(time_window[0], time_window[1])
        if idx == 0:
            ax.legend(loc="best", fontsize=7)
    for idx in range(len(columns), len(axes_flat)):
        axes_flat[idx].axis("off")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_velocity_compare(
    case_frames: dict[str, pd.DataFrame],
    case_ids: list[str],
    output_path: Path,
    phase1_end_time: float,
    phase2_end_time: float,
    gnss_off_windows: list[tuple[float, float]],
) -> None:
    cols = [("v_v_x_mps", "v_v.x"), ("v_v_y_mps", "v_v.y"), ("v_v_z_mps", "v_v.z")]
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for ax, (column, label) in zip(axes, cols):
        shade_gnss_off_windows(ax, gnss_off_windows)
        for case_id in case_ids:
            case_spec = CASE_MAP[case_id]
            df = case_frames[case_id]
            t = df["timestamp"].to_numpy(dtype=float)
            y = df[column].to_numpy(dtype=float)
            t_plot, y_plot = downsample_for_plot(t, y)
            ax.plot(t_plot, y_plot, linewidth=1.0, color=case_spec.color, label=case_spec.label)
            if case_id == case_ids[0]:
                truth_key = column.replace("v_v_", "truth_v_v_")
                tt_plot, ty_plot = downsample_for_plot(t, df[truth_key].to_numpy(dtype=float))
                ax.plot(tt_plot, ty_plot, linewidth=1.0, color="black", linestyle="--", label="truth_v_v")
        boundary_lines(ax, phase1_end_time, phase2_end_time)
        ax.set_title(label)
        ax.set_ylabel("m/s")
        ax.grid(alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle("Vehicle-frame velocity comparison", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def rmse3d(plot_df: pd.DataFrame, start_time: float, end_time: float) -> float:
    subset = plot_df[(plot_df["timestamp"] >= start_time) & (plot_df["timestamp"] <= end_time)]
    if subset.empty:
        return float("nan")
    err = subset[["p_n_err_m", "p_e_err_m", "p_u_err_m"]].to_numpy(dtype=float)
    return float(np.sqrt(np.mean(np.sum(err * err, axis=1))))


def peak_abs(plot_df: pd.DataFrame, column: str, start_time: float) -> float:
    subset = plot_df[plot_df["timestamp"] >= start_time]
    if subset.empty:
        return float("nan")
    return float(np.max(np.abs(subset[column].to_numpy(dtype=float))))


def run_jump_audit(case_dir: Path, output_path: Path, min_time: float) -> None:
    run_command(
        [
            sys.executable,
            str((REPO_ROOT / "scripts/analysis/analyze_data2_jump_windows.py").resolve()),
            "--case-dir",
            str(case_dir.resolve()),
            "--output-md",
            str(output_path.resolve()),
            "--top-k",
            "8",
            "--cluster-gap",
            "5.0",
            "--min-time",
            f"{min_time:.6f}",
        ],
        REPO_ROOT,
    )


def write_summary(
    output_dir: Path,
    exp_id: str,
    case_metrics_df: pd.DataFrame,
    phase_metrics_df: pd.DataFrame,
    plots: dict[str, Path],
    manifest: dict[str, Any],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Setup",
        "",
        "- all cases use staged schedule (`0-200 s` INS/GNSS, `200-700 s` joint calibration, `700+ s` periodic GNSS outage).",
        "- phase2/phase3 always enable strict weak-excitation gate on both `ODO` and `NHC`.",
        "- phase2/phase3 always freeze `GNSS lever`.",
        "- all cases use `fusion.init.odo_scale=1.0`.",
        f"- working mounting truth assumption used for motion/vehicle-frame diagnostics: `pitch={WORKING_TOTAL_MOUNTING_DEG['pitch']:.2f} deg`, `yaw={WORKING_TOTAL_MOUNTING_DEG['yaw']:.2f} deg`.",
        "",
        "## Case Metrics",
        "",
    ]
    metric_rows = []
    for _, row in case_metrics_df.iterrows():
        metric_rows.append(
            [
                str(row["case_id"]),
                format_metric(row.get("overall_rmse_3d_m_aux")),
                format_metric(row.get("overall_final_err_3d_m_aux")),
                format_metric(row.get("phase2_rmse_3d_m")),
                format_metric(row.get("phase3_rmse_3d_m")),
                format_metric(row.get("first_divergence_start_t")),
                format_metric(row.get("first_div_gnss_pos_t")),
                format_metric(row.get("odo_accept_ratio")),
                format_metric(row.get("nhc_accept_ratio")),
                format_metric(row.get("peak_bg_z_after_phase1_degh")),
                format_metric(row.get("peak_sa_x_after_phase1_ppm")),
                format_metric(row.get("peak_total_mounting_yaw_after_phase1_deg")),
                format_metric(row.get("peak_odo_lever_y_after_phase1_m")),
                format_metric(row.get("peak_gnss_lever_y_after_phase1_m")),
            ]
        )
    lines.extend(
        render_table(
            [
                "case_id",
                "overall_rmse_3d_m",
                "final_err_3d_m",
                "phase2_rmse_3d_m",
                "phase3_rmse_3d_m",
                "first_div_t",
                "first_div_gnss_t",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "peak_bg_z_deg_h",
                "peak_sa_x_ppm",
                "peak_total_mount_yaw_deg",
                "peak_odo_lever_y_m",
                "peak_gnss_lever_y_m",
            ],
            metric_rows,
        )
    )
    lines.extend(
        [
            "",
            "## Phase Metrics",
            "",
        ]
    )
    phase_rows = []
    for _, row in phase_metrics_df.iterrows():
        phase_rows.append(
            [
                str(row["case_id"]),
                str(row["phase_name"]),
                format_metric(row["rmse_3d_m"]),
                format_metric(row["yaw_abs_max_deg"]),
                format_metric(row["bg_z_abs_max_degh"]),
                format_metric(row["sa_x_abs_max_ppm"]),
            ]
        )
    lines.extend(
        render_table(
            ["case_id", "phase_name", "rmse_3d_m", "yaw_abs_max_deg", "bg_z_abs_max_degh", "sa_x_abs_max_ppm"],
            phase_rows,
        )
    )
    lines.extend(
        [
            "",
            "## Plots",
            "",
        ]
    )
    for name, path in plots.items():
        lines.append(f"- {name}: `{rel_from_root(path, REPO_ROOT)}`")
    lines.extend(
        [
            "",
            "## Manifest",
            "",
            f"- manifest: `{rel_from_root(output_dir / 'manifest.json', REPO_ROOT)}`",
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

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    base_truth_reference = build_truth_reference(base_cfg)
    motion_truth_reference = working_motion_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())

    case_frames: dict[str, pd.DataFrame] = {}
    case_rows: list[dict[str, Any]] = []
    phase_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}
    case_metadata: dict[str, dict[str, Any]] = {}
    jump_reports: dict[str, str] = {}

    for spec in CASE_SPECS:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides, metadata = write_case_config(
            base_cfg=base_cfg,
            motion_truth_reference=motion_truth_reference,
            case_dir=case_dir,
            spec=spec,
            args=args,
        )
        case_config_paths[spec.case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_overrides[spec.case_id] = overrides
        case_metadata[spec.case_id] = metadata
        row = run_baseline_case(
            case_dir,
            cfg_path,
            args.exe,
            BaselineCaseSpec(
                case_id=spec.case_id,
                label=spec.label,
                color=spec.color,
                filter_mode="ESKF",
                enable_fej=False,
                enable_runtime_anchor=False,
            ),
        )

        sol_path = (REPO_ROOT / row["sol_path"]).resolve()
        state_series_path = (REPO_ROOT / row["state_series_path"]).resolve()
        merged_df = merge_case_outputs(sol_path, state_series_path)
        truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
        motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, motion_truth_reference)
        plot_df = build_plot_frame(merged_df, truth_interp_df, base_truth_reference, motion_df)
        for raw_col in [
            "odo_scale_state",
            "mounting_pitch_deg",
            "mounting_yaw_deg",
            "total_mounting_pitch_deg",
            "total_mounting_yaw_deg",
            "odo_lever_x_m",
            "odo_lever_y_m",
            "odo_lever_z_m",
            "gnss_lever_x_m",
            "gnss_lever_y_m",
            "gnss_lever_z_m",
        ]:
            plot_df[raw_col] = merged_df[raw_col].to_numpy(dtype=float)
        case_frames[spec.case_id] = plot_df

        phase1_end_time = metadata["phase1_end_time"]
        phase2_end_time = metadata["phase2_end_time"]
        final_time = float(base_cfg["fusion"]["finaltime"])
        row.update(flatten_consistency_metrics((REPO_ROOT / row["stdout_path"]).read_text(encoding="utf-8", errors="ignore")))
        row.update(parse_diag_times((REPO_ROOT / row["stdout_path"]).read_text(encoding="utf-8", errors="ignore")))
        row["phase2_rmse_3d_m"] = rmse3d(plot_df, phase1_end_time, phase2_end_time)
        row["phase3_rmse_3d_m"] = rmse3d(plot_df, phase2_end_time, final_time)
        row["peak_bg_z_after_phase1_degh"] = peak_abs(plot_df, "bg_z_degh", phase1_end_time)
        row["peak_sa_x_after_phase1_ppm"] = peak_abs(plot_df, "sa_x_ppm", phase1_end_time)
        row["peak_total_mounting_yaw_after_phase1_deg"] = peak_abs(plot_df, "total_mounting_yaw_deg", phase1_end_time)
        row["peak_odo_lever_y_after_phase1_m"] = peak_abs(plot_df, "odo_lever_y_m", phase1_end_time)
        row["peak_gnss_lever_y_after_phase1_m"] = peak_abs(plot_df, "gnss_lever_y_m", phase1_end_time)
        row["truth_note"] = metadata["truth_note"]
        case_rows.append(row)

        for phase_name, start_t, end_t in [
            ("phase1_ins_gnss", float(base_cfg["fusion"]["starttime"]), phase1_end_time),
            ("phase2_ins_gnss_odo_nhc", phase1_end_time, phase2_end_time),
            ("phase3_periodic_gnss_outage", phase2_end_time, final_time),
        ]:
            subset = plot_df[(plot_df["timestamp"] >= start_t) & (plot_df["timestamp"] <= end_t)]
            phase_rows.append(
                {
                    "case_id": spec.case_id,
                    "phase_name": phase_name,
                    "rmse_3d_m": rmse3d(plot_df, start_t, end_t),
                    "yaw_abs_max_deg": float(np.max(np.abs(subset["yaw_err_deg"]))) if not subset.empty else float("nan"),
                    "bg_z_abs_max_degh": float(np.max(np.abs(subset["bg_z_degh"]))) if not subset.empty else float("nan"),
                    "sa_x_abs_max_ppm": float(np.max(np.abs(subset["sa_x_ppm"]))) if not subset.empty else float("nan"),
                }
            )

        jump_report_path = case_dir / f"jump_window_audit_{spec.case_id}.md"
        run_jump_audit(case_dir, jump_report_path, phase1_end_time)
        jump_reports[spec.case_id] = rel_from_root(jump_report_path, REPO_ROOT)

    case_metrics_df = pd.DataFrame(case_rows)
    phase_metrics_df = pd.DataFrame(phase_rows)

    reference_case_id = CASE_SPECS[0].case_id
    phase1_end_time = float(case_metadata[reference_case_id]["phase1_end_time"])
    phase2_end_time = float(case_metadata[reference_case_id]["phase2_end_time"])
    gnss_off_windows = case_metadata[reference_case_id]["gnss_off_windows"]
    case_ids = [spec.case_id for spec in CASE_SPECS]

    plots = {
        "nav_errors_compare": args.plot_dir / "nav_errors_compare.png",
        "key_states_compare": args.plot_dir / "key_states_compare.png",
        "key_states_jump_zoom": args.plot_dir / "key_states_jump_zoom.png",
        "velocity_vehicle_compare": args.plot_dir / "velocity_vehicle_compare.png",
    }
    plot_column_grid(
        case_frames,
        case_ids,
        NAV_COLUMNS,
        plots["nav_errors_compare"],
        "Navigation error comparison",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_column_grid(
        case_frames,
        case_ids,
        KEY_STATE_COLUMNS,
        plots["key_states_compare"],
        "Key state comparison",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_column_grid(
        case_frames,
        case_ids,
        KEY_STATE_COLUMNS,
        plots["key_states_jump_zoom"],
        "Key state jump zoom (phase2/3)",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        time_window=(phase1_end_time, min(float(base_cfg["fusion"]["finaltime"]), phase2_end_time + 420.0)),
    )
    plot_velocity_compare(
        case_frames,
        case_ids,
        plots["velocity_vehicle_compare"],
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )

    case_metrics_path = args.output_dir / "case_metrics.csv"
    phase_metrics_path = args.output_dir / "phase_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    phase_metrics_df.to_csv(phase_metrics_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "phase_metrics_csv": rel_from_root(phase_metrics_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_overrides": case_overrides,
        "case_jump_reports": jump_reports,
        "working_mounting_total_truth_deg": WORKING_TOTAL_MOUNTING_DEG,
        "strict_gate": phase_constraints(),
        "phase_windows": {
            "phase1": [float(base_cfg["fusion"]["starttime"]), phase1_end_time],
            "phase2": [phase1_end_time, phase2_end_time],
            "phase3": [phase2_end_time, float(base_cfg["fusion"]["finaltime"])],
        },
        "phase3_periodic_gnss": {
            "on_duration_s": args.phase3_gnss_on,
            "off_duration_s": args.phase3_gnss_off,
            "off_windows": [[float(start), float(end)] for start, end in gnss_off_windows],
        },
        "freshness": {
            "solver_exe_mtime": mtime_text(args.exe),
            "base_config_mtime": mtime_text(args.base_config),
        },
    }
    (args.output_dir / "manifest.json").write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    write_summary(args.output_dir, args.exp_id, case_metrics_df, phase_metrics_df, plots, manifest)
    print(rel_from_root(args.output_dir / "manifest.json", REPO_ROOT))


if __name__ == "__main__":
    main()
