from __future__ import annotations

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
    build_case_config as build_baseline_case_config,
    flatten_consistency_metrics,
    metric_value,
    mtime_text,
    run_case as run_baseline_case,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    build_truth_reference,
    downsample_for_plot,
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
)


EXP_ID_DEFAULT = "EXP-20260324-data2-staged-estimation-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_staged_estimation_r1_20260324")
BASE_CONFIG_DEFAULT = Path("config_data2_research_seed_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 60.0
PHASE3_GNSS_OFF_DEFAULT = 60.0
CONVERGED_NOISE_SCALE_DEFAULT = 0.1
TRANSITION_PRE_SECONDS = 10.0
TRANSITION_POST_SECONDS = 30.0


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    staged: bool


@dataclass(frozen=True)
class PlotColumn:
    key: str
    label: str
    unit: str


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec("baseline_periodic_gnss", "baseline periodic GNSS", "#4c78a8", False),
    CaseSpec("staged_segmented", "staged segmented", "#f58518", True),
)

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

EXTRINSIC_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("odo_scale_err", "odo_scale error", "1"),
    PlotColumn("mounting_roll_err_deg", "mounting_roll error", "deg"),
    PlotColumn("mounting_pitch_err_deg", "mounting_pitch error", "deg"),
    PlotColumn("mounting_yaw_err_deg", "mounting_yaw error", "deg"),
    PlotColumn("odo_lever_x_err_m", "odo_lever_x error", "m"),
    PlotColumn("odo_lever_y_err_m", "odo_lever_y error", "m"),
    PlotColumn("odo_lever_z_err_m", "odo_lever_z error", "m"),
    PlotColumn("gnss_lever_x_err_m", "gnss_lever_x error", "m"),
    PlotColumn("gnss_lever_y_err_m", "gnss_lever_y error", "m"),
    PlotColumn("gnss_lever_z_err_m", "gnss_lever_z error", "m"),
)

IMU_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("ba_x_mgal", "ba_x", "mGal"),
    PlotColumn("ba_y_mgal", "ba_y", "mGal"),
    PlotColumn("ba_z_mgal", "ba_z", "mGal"),
    PlotColumn("bg_x_degh", "bg_x", "deg/h"),
    PlotColumn("bg_y_degh", "bg_y", "deg/h"),
    PlotColumn("bg_z_degh", "bg_z", "deg/h"),
    PlotColumn("sg_x_ppm", "sg_x", "ppm"),
    PlotColumn("sg_y_ppm", "sg_y", "ppm"),
    PlotColumn("sg_z_ppm", "sg_z", "ppm"),
    PlotColumn("sa_x_ppm", "sa_x", "ppm"),
    PlotColumn("sa_y_ppm", "sa_y", "ppm"),
    PlotColumn("sa_z_ppm", "sa_z", "ppm"),
)

KEY_STATE_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("yaw_err_deg", "yaw error", "deg"),
    PlotColumn("mounting_yaw_err_deg", "mounting_yaw error", "deg"),
    PlotColumn("gnss_lever_y_err_m", "gnss_lever_y error", "m"),
    PlotColumn("odo_scale_err", "odo_scale error", "1"),
    PlotColumn("bg_z_degh", "bg_z", "deg/h"),
)

ALL_COLUMNS: tuple[PlotColumn, ...] = NAV_COLUMNS + EXTRINSIC_COLUMNS + IMU_COLUMNS
CASE_MAP = {spec.case_id: spec for spec in CASE_SPECS}


def format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            return "NA"
        return f"{float(value):.6f}"
    return str(value)


def render_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def phase_absolute_times(start_time: float, phase1_end_offset: float, phase2_end_offset: float) -> tuple[float, float]:
    return start_time + phase1_end_offset, start_time + phase2_end_offset


def invert_windows(
    on_windows: list[tuple[float, float]],
    start_time: float,
    final_time: float,
) -> list[tuple[float, float]]:
    off_windows: list[tuple[float, float]] = []
    cursor = start_time
    for win_start, win_end in on_windows:
        if win_start > cursor:
            off_windows.append((cursor, win_start))
        cursor = max(cursor, win_end)
    if cursor < final_time:
        off_windows.append((cursor, final_time))
    return off_windows


def build_periodic_gnss_windows(
    start_time: float,
    final_time: float,
    phase2_end_time: float,
    phase3_on_duration: float,
    phase3_off_duration: float,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    if final_time <= start_time:
        return [], []
    first_on_end = min(final_time, phase2_end_time + max(0.0, phase3_on_duration))
    on_windows: list[tuple[float, float]] = [(start_time, first_on_end)]
    cycle = max(0.0, phase3_on_duration) + max(0.0, phase3_off_duration)
    cursor = first_on_end + max(0.0, phase3_off_duration)
    while cycle > 0.0 and cursor < final_time:
        win_end = min(final_time, cursor + max(0.0, phase3_on_duration))
        if win_end > cursor:
            on_windows.append((cursor, win_end))
        cursor = win_end + max(0.0, phase3_off_duration)
    return on_windows, invert_windows(on_windows, start_time, final_time)


def scaled_noise_override(noise_cfg: dict[str, Any], scale: float) -> dict[str, Any]:
    override: dict[str, Any] = {}
    scalar_keys = ["sigma_ba", "sigma_bg", "sigma_sg", "sigma_sa", "sigma_gnss_lever_arm"]
    vector_keys = [
        "sigma_ba_vec",
        "sigma_bg_vec",
        "sigma_sg_vec",
        "sigma_sa_vec",
        "sigma_gnss_lever_arm_vec",
    ]
    for key in scalar_keys:
        if key in noise_cfg:
            override[key] = float(noise_cfg[key]) * scale
    for key in vector_keys:
        if key not in noise_cfg:
            continue
        vec = [float(x) for x in noise_cfg[key]]
        if all(x < 0.0 for x in vec):
            continue
        override[key] = [float(x) * scale for x in vec]
    return override


def build_runtime_phases(
    cfg: dict[str, Any],
    phase1_end_time: float,
    phase2_end_time: float,
    final_time: float,
    noise_scale: float,
) -> list[dict[str, Any]]:
    converged_noise = scaled_noise_override(cfg["fusion"]["noise"], noise_scale)
    return [
        {
            "name": "phase1_ins_gnss_freeze_odo_nhc_states",
            "start_time": float(cfg["fusion"]["starttime"]),
            "end_time": phase1_end_time,
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        },
        {
            "name": "phase2_joint_calibration_reduced_converged_noise",
            "start_time": phase1_end_time,
            "end_time": phase2_end_time,
            "noise": converged_noise,
        },
        {
            "name": "phase3_periodic_gnss_outage_reduced_converged_noise",
            "start_time": phase2_end_time,
            "end_time": final_time,
            "noise": converged_noise,
        },
    ]


def build_case_config(
    base_cfg: dict[str, Any],
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
    fusion["gnss_schedule"] = {
        "enabled": True,
        "enabled_windows": [
            {"start_time": float(win_start), "end_time": float(win_end)}
            for win_start, win_end in on_windows
        ],
    }

    overrides["fusion.ablation.disable_mounting_roll"] = False
    overrides["fusion.ablation.disable_gnss_lever_z"] = False
    overrides["fusion.gnss_schedule.enabled"] = True
    overrides["fusion.gnss_schedule.enabled_windows"] = [
        {"start_time": float(win_start), "end_time": float(win_end)}
        for win_start, win_end in on_windows
    ]

    if spec.staged:
        runtime_phases = build_runtime_phases(
            cfg=cfg,
            phase1_end_time=phase1_end_time,
            phase2_end_time=phase2_end_time,
            final_time=final_time,
            noise_scale=args.converged_noise_scale,
        )
        fusion["runtime_phases"] = runtime_phases
        overrides["fusion.runtime_phases"] = runtime_phases
    else:
        fusion.pop("runtime_phases", None)

    metadata = {
        "phase1_end_time": phase1_end_time,
        "phase2_end_time": phase2_end_time,
        "gnss_on_windows": on_windows,
        "gnss_off_windows": off_windows,
        "converged_noise_override": (
            scaled_noise_override(cfg["fusion"]["noise"], args.converged_noise_scale)
            if spec.staged
            else {}
        ),
    }
    return cfg, overrides, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    cfg, overrides, metadata = build_case_config(base_cfg, case_dir, spec, args)
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
    single_case_id: str | None = None,
) -> None:
    n_cols = 3
    n_rows = math.ceil(len(columns) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 2.9 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, column in enumerate(columns):
        ax = axes_flat[idx]
        shade_gnss_off_windows(ax, gnss_off_windows)
        for case_id in case_ids:
            if single_case_id is not None and case_id != single_case_id:
                continue
            case_spec = CASE_MAP[case_id]
            df = case_frames[case_id]
            t = df["timestamp"].to_numpy(dtype=float)
            y = df[column.key].to_numpy(dtype=float)
            t_plot, y_plot = downsample_for_plot(t, y)
            ax.plot(t_plot, y_plot, linewidth=0.95, color=case_spec.color, label=case_spec.label)
        boundary_lines(ax, phase1_end_time, phase2_end_time)
        ax.set_title(f"{column.label} [{column.unit}]", fontsize=9)
        ax.grid(alpha=0.2)
        ax.tick_params(axis="both", labelsize=7)
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
    use_vehicle_frame: bool,
) -> None:
    cols = (
        [("v_v_x_mps", "v_v.x"), ("v_v_y_mps", "v_v.y"), ("v_v_z_mps", "v_v.z")]
        if use_vehicle_frame
        else [("v_b_x_mps", "v_b.x"), ("v_b_y_mps", "v_b.y"), ("v_b_z_mps", "v_b.z")]
    )
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
            truth_key = column.replace("v_v_", "truth_v_v_")
            if use_vehicle_frame and truth_key in df.columns and case_id == case_ids[0]:
                tt_plot, ty_plot = downsample_for_plot(t, df[truth_key].to_numpy(dtype=float))
                ax.plot(tt_plot, ty_plot, linewidth=1.0, color="black", linestyle="--", label="truth_v_v")
        boundary_lines(ax, phase1_end_time, phase2_end_time)
        ax.set_title(label)
        ax.set_ylabel("m/s")
        ax.grid(alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle(
        "Vehicle-frame velocity comparison" if use_vehicle_frame else "Body-frame velocity comparison",
        fontsize=14,
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def compute_phase_metrics(
    plot_df: pd.DataFrame,
    case_id: str,
    phase_windows: list[tuple[str, float, float]],
    gnss_off_windows: list[tuple[float, float]],
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for phase_name, start_time, end_time in phase_windows:
        subset = plot_df.loc[(plot_df["timestamp"] >= start_time) & (plot_df["timestamp"] <= end_time)]
        if subset.empty:
            continue
        err3 = np.linalg.norm(
            subset[["p_n_err_m", "p_e_err_m", "p_u_err_m"]].to_numpy(dtype=float),
            axis=1,
        )
        rows.append(
            {
                "case_id": case_id,
                "window_type": "phase",
                "window_name": phase_name,
                "start_time": start_time,
                "end_time": end_time,
                "samples": int(len(subset)),
                "rmse_3d_m": float(np.sqrt(np.mean(err3 * err3))),
                "final_err_3d_m": float(err3[-1]),
                "yaw_err_abs_max_deg": float(np.max(np.abs(subset["yaw_err_deg"]))),
                "mounting_yaw_err_abs_max_deg": float(np.max(np.abs(subset["mounting_yaw_err_deg"]))),
                "gnss_lever_y_err_abs_max_m": float(np.max(np.abs(subset["gnss_lever_y_err_m"]))),
                "odo_scale_err_abs_max": float(np.max(np.abs(subset["odo_scale_err"]))),
                "v_v_y_abs_max_mps": float(np.max(np.abs(subset["v_v_y_mps"]))),
                "v_v_z_abs_max_mps": float(np.max(np.abs(subset["v_v_z_mps"]))),
            }
        )
    for idx, (start_time, end_time) in enumerate(gnss_off_windows, start=1):
        subset = plot_df.loc[(plot_df["timestamp"] >= start_time) & (plot_df["timestamp"] <= end_time)]
        if subset.empty:
            continue
        err3 = np.linalg.norm(
            subset[["p_n_err_m", "p_e_err_m", "p_u_err_m"]].to_numpy(dtype=float),
            axis=1,
        )
        rows.append(
            {
                "case_id": case_id,
                "window_type": "gnss_off",
                "window_name": f"gnss_off_{idx:02d}",
                "start_time": start_time,
                "end_time": end_time,
                "samples": int(len(subset)),
                "rmse_3d_m": float(np.sqrt(np.mean(err3 * err3))),
                "final_err_3d_m": float(err3[-1]),
                "yaw_err_abs_max_deg": float(np.max(np.abs(subset["yaw_err_deg"]))),
                "mounting_yaw_err_abs_max_deg": float(np.max(np.abs(subset["mounting_yaw_err_deg"]))),
                "gnss_lever_y_err_abs_max_m": float(np.max(np.abs(subset["gnss_lever_y_err_m"]))),
                "odo_scale_err_abs_max": float(np.max(np.abs(subset["odo_scale_err"]))),
                "v_v_y_abs_max_mps": float(np.max(np.abs(subset["v_v_y_mps"]))),
                "v_v_z_abs_max_mps": float(np.max(np.abs(subset["v_v_z_mps"]))),
            }
        )
    return pd.DataFrame(rows)


def compute_transition_metrics(
    plot_df: pd.DataFrame,
    case_id: str,
    transitions: list[tuple[str, float]],
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    t = plot_df["timestamp"].to_numpy(dtype=float)
    sec_grid = np.arange(math.ceil(float(t[0])), math.floor(float(t[-1])) + 1.0, 1.0, dtype=float)
    if sec_grid.size < 2:
        return pd.DataFrame()
    for transition_name, center_time in transitions:
        mask = (sec_grid >= center_time - TRANSITION_PRE_SECONDS) & (
            sec_grid <= center_time + TRANSITION_POST_SECONDS
        )
        sec_window = sec_grid[mask]
        if sec_window.size < 2:
            continue
        for column in KEY_STATE_COLUMNS:
            sec_values = np.interp(sec_grid, t, plot_df[column.key].to_numpy(dtype=float))
            sec_subset = sec_values[mask]
            sec_diff = np.diff(sec_subset)
            diff_times = sec_window[1:]
            max_idx = int(np.argmax(np.abs(sec_diff)))
            rows.append(
                {
                    "case_id": case_id,
                    "transition": transition_name,
                    "state_key": column.key,
                    "center_time": center_time,
                    "window_start": float(sec_window[0]),
                    "window_end": float(sec_window[-1]),
                    "max_abs_jump_1hz": float(np.max(np.abs(sec_diff))),
                    "max_jump_t_1hz": float(diff_times[max_idx]),
                    "total_variation_1hz": float(np.sum(np.abs(sec_diff))),
                }
            )
    return pd.DataFrame(rows)


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    phase_metrics_df: pd.DataFrame,
    transition_metrics_df: pd.DataFrame,
) -> None:
    overall_rows = [
        [
            str(row["case_label"]),
            format_metric(row.get("overall_rmse_3d_m_aux")),
            format_metric(row.get("overall_final_err_3d_m_aux")),
            format_metric(row.get("odo_accept_ratio")),
            format_metric(row.get("nhc_accept_ratio")),
            format_metric(metric_value(row, "gnss_accept_ratio", "gnss_pos_accept_ratio")),
            format_metric(row.get("first_divergence_start_t")),
        ]
        for _, row in case_metrics_df.iterrows()
    ]

    phase_rows = [
        [
            str(row["case_id"]),
            str(row["window_name"]),
            format_metric(row["rmse_3d_m"]),
            format_metric(row["final_err_3d_m"]),
            format_metric(row["yaw_err_abs_max_deg"]),
            format_metric(row["mounting_yaw_err_abs_max_deg"]),
            format_metric(row["gnss_lever_y_err_abs_max_m"]),
        ]
        for _, row in phase_metrics_df.loc[phase_metrics_df["window_type"] == "phase"].iterrows()
    ]

    off_summary = (
        phase_metrics_df.loc[phase_metrics_df["window_type"] == "gnss_off"]
        .groupby("case_id", as_index=False)
        .agg(
            mean_gnss_off_rmse_3d_m=("rmse_3d_m", "mean"),
            max_gnss_off_final_err_3d_m=("final_err_3d_m", "max"),
            max_gnss_off_yaw_err_deg=("yaw_err_abs_max_deg", "max"),
        )
    )
    off_rows = [
        [
            str(row["case_id"]),
            format_metric(row["mean_gnss_off_rmse_3d_m"]),
            format_metric(row["max_gnss_off_final_err_3d_m"]),
            format_metric(row["max_gnss_off_yaw_err_deg"]),
        ]
        for _, row in off_summary.iterrows()
    ]

    transition_rows = [
        [
            str(row["case_id"]),
            str(row["transition"]),
            str(row["state_key"]),
            format_metric(row["max_abs_jump_1hz"]),
            format_metric(row["max_jump_t_1hz"]),
            format_metric(row["total_variation_1hz"]),
        ]
        for _, row in transition_metrics_df.iterrows()
    ]

    lines = [
        "# data2 staged estimation summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- phase windows: `phase1={manifest['phase_windows']['phase1']}`, `phase2={manifest['phase_windows']['phase2']}`, `phase3={manifest['phase_windows']['phase3']}`",
        (
            f"- phase3 periodic GNSS: `on={manifest['phase3_periodic_gnss']['on_duration_s']:.1f}s`, "
            f"`off={manifest['phase3_periodic_gnss']['off_duration_s']:.1f}s`"
        ),
        f"- converged-noise scale: `{manifest['converged_noise_scale']:.3f}`",
        "",
        "## Overall Metrics",
    ]
    lines.extend(
        render_table(
            ["case", "rmse3d", "final3d", "odo_accept", "nhc_accept", "gnss_accept", "t_div_first"],
            overall_rows,
        )
    )
    lines.extend(["", "## Phase Metrics"])
    lines.extend(
        render_table(
            ["case", "phase", "rmse3d", "final3d", "yaw_abs_max", "mount_yaw_abs_max", "lgy_abs_max"],
            phase_rows,
        )
    )
    lines.extend(["", "## GNSS Off Summary"])
    lines.extend(
        render_table(
            ["case", "mean_gnss_off_rmse3d", "max_gnss_off_final3d", "max_gnss_off_yaw"],
            off_rows,
        )
    )
    lines.extend(["", "## Transition Metrics"])
    lines.extend(
        render_table(
            ["case", "transition", "state_key", "max_abs_jump_1hz", "max_jump_t", "total_variation_1hz"],
            transition_rows,
        )
    )
    lines.extend(
        [
            "",
            "## Outputs",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- phase_metrics: `{manifest['phase_metrics_csv']}`",
            f"- transition_metrics: `{manifest['transition_metrics_csv']}`",
            f"- plots_dir: `{manifest['plots_dir']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the data2 three-phase staged estimation experiment with periodic GNSS outage."
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
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    truth_reference = build_truth_reference(base_cfg)
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())

    case_frames: dict[str, pd.DataFrame] = {}
    case_rows: list[dict[str, Any]] = []
    phase_metric_frames: list[pd.DataFrame] = []
    transition_metric_frames: list[pd.DataFrame] = []
    case_config_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}
    case_metadata: dict[str, dict[str, Any]] = {}

    for spec in CASE_SPECS:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides, metadata = write_case_config(base_cfg, case_dir, spec, args)
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
        motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, truth_reference)
        plot_df = build_plot_frame(merged_df, truth_interp_df, truth_reference, motion_df)
        case_frames[spec.case_id] = plot_df

        phase1_end_time = metadata["phase1_end_time"]
        phase2_end_time = metadata["phase2_end_time"]
        final_time = float(base_cfg["fusion"]["finaltime"])
        phase_windows = [
            ("phase1_ins_gnss", float(base_cfg["fusion"]["starttime"]), phase1_end_time),
            ("phase2_ins_gnss_odo_nhc", phase1_end_time, phase2_end_time),
            ("phase3_periodic_gnss_outage", phase2_end_time, final_time),
        ]
        phase_metric_frames.append(
            compute_phase_metrics(plot_df, spec.case_id, phase_windows, metadata["gnss_off_windows"])
        )
        transition_metric_frames.append(
            compute_transition_metrics(
                plot_df,
                spec.case_id,
                [("phase1_to_phase2", phase1_end_time), ("phase2_to_phase3", phase2_end_time)],
            )
        )

        row["phase1_end_time"] = phase1_end_time
        row["phase2_end_time"] = phase2_end_time
        row["gnss_off_window_count"] = len(metadata["gnss_off_windows"])
        stdout_text = (REPO_ROOT / row["stdout_path"]).resolve().read_text(encoding="utf-8", errors="ignore")
        row.update(flatten_consistency_metrics(stdout_text))
        case_rows.append(row)

    case_metrics_df = pd.DataFrame(case_rows)
    phase_metrics_df = pd.concat(phase_metric_frames, ignore_index=True) if phase_metric_frames else pd.DataFrame()
    transition_metrics_df = (
        pd.concat(transition_metric_frames, ignore_index=True) if transition_metric_frames else pd.DataFrame()
    )

    phase1_end_time = case_metadata["staged_segmented"]["phase1_end_time"]
    phase2_end_time = case_metadata["staged_segmented"]["phase2_end_time"]
    gnss_off_windows = case_metadata["staged_segmented"]["gnss_off_windows"]
    case_ids = [spec.case_id for spec in CASE_SPECS]

    plot_column_grid(
        case_frames,
        case_ids,
        NAV_COLUMNS,
        args.plot_dir / "nav_errors_compare.png",
        "Navigation error comparison",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_column_grid(
        case_frames,
        case_ids,
        EXTRINSIC_COLUMNS,
        args.plot_dir / "extrinsics_compare.png",
        "Extrinsic-state error comparison",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_column_grid(
        case_frames,
        case_ids,
        IMU_COLUMNS,
        args.plot_dir / "imu_states_compare.png",
        "IMU state comparison",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_column_grid(
        case_frames,
        case_ids,
        KEY_STATE_COLUMNS,
        args.plot_dir / "key_states_compare.png",
        "Key state comparison",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_velocity_compare(
        case_frames,
        case_ids,
        args.plot_dir / "velocity_vehicle_compare.png",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        use_vehicle_frame=True,
    )
    plot_velocity_compare(
        case_frames,
        case_ids,
        args.plot_dir / "velocity_body_compare.png",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        use_vehicle_frame=False,
    )
    plot_column_grid(
        case_frames,
        case_ids,
        ALL_COLUMNS,
        args.plot_dir / "staged_all_states_overview.png",
        "Staged case all-state overview",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        single_case_id="staged_segmented",
    )

    case_metrics_path = args.output_dir / "case_metrics.csv"
    phase_metrics_path = args.output_dir / "phase_metrics.csv"
    transition_metrics_path = args.output_dir / "transition_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    phase_metrics_df.to_csv(phase_metrics_path, index=False, encoding="utf-8-sig")
    transition_metrics_df.to_csv(transition_metrics_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "phase_metrics_csv": rel_from_root(phase_metrics_path, REPO_ROOT),
        "transition_metrics_csv": rel_from_root(transition_metrics_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_overrides": case_overrides,
        "phase_windows": {
            "phase1": [float(base_cfg["fusion"]["starttime"]), phase1_end_time],
            "phase2": [phase1_end_time, phase2_end_time],
            "phase3": [phase2_end_time, float(base_cfg["fusion"]["finaltime"])],
        },
        "phase3_periodic_gnss": {
            "on_duration_s": args.phase3_gnss_on,
            "off_duration_s": args.phase3_gnss_off,
            "gnss_on_windows": case_metadata["staged_segmented"]["gnss_on_windows"],
            "gnss_off_windows": case_metadata["staged_segmented"]["gnss_off_windows"],
        },
        "converged_noise_scale": args.converged_noise_scale,
        "staged_converged_noise_override": case_metadata["staged_segmented"]["converged_noise_override"],
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_exe_mtime": mtime_text(args.exe),
            "case_metrics_csv": mtime_text(case_metrics_path),
            "phase_metrics_csv": mtime_text(phase_metrics_path),
            "transition_metrics_csv": mtime_text(transition_metrics_path),
        },
        "assumptions": [
            "Baseline and staged cases share the same periodic GNSS schedule in phase3; only staged case adds runtime phases.",
            "Phase1 freezes odo_scale + mounting + lever_odo and disables ODO/NHC updates.",
            "Phase2/3 reduce GNSS lever and IMU bias/scale process noise to 0.1x by default.",
            "disable_mounting_roll and disable_gnss_lever_z are explicitly cleared so all 31 states remain visible in the plots.",
            "ODO scale keeps the repo-standard 1e-6 near-zero initialization to satisfy the current initialization guard.",
        ],
    }

    summary_path = args.output_dir / "summary.md"
    write_summary(summary_path, manifest, case_metrics_df, phase_metrics_df, transition_metrics_df)
    manifest["summary_md"] = rel_from_root(summary_path, REPO_ROOT)
    manifest["freshness"]["summary_md"] = mtime_text(summary_path)

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
