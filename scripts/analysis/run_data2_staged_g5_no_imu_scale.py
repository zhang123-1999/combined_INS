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
from scripts.analysis.run_data2_fullwindow_attitude_bias_coupling import (  # noqa: E402
    ALL_STATE_SPECS,
    GROUP_SPECS,
    GroupSpec,
    KEY_COUPLING_STATES,
    build_state_frame,
    compute_case_metrics,
    format_metric,
    render_table,
    scale_noise_fields,
    scale_optional_vector_noise,
    state_reference_value,
)
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import (  # noqa: E402
    CaseSpec as BaselineCaseSpec,
    mtime_text,
    run_case as run_baseline_case,
)
from scripts.analysis.run_data2_staged_estimation import (  # noqa: E402
    build_periodic_gnss_windows,
    phase_absolute_times,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    build_truth_reference,
    downsample_for_plot,
    json_safe,
    reset_directory,
)
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    build_truth_interp,
    load_pos_dataframe,
    merge_case_outputs,
    wrap_deg,
)


EXP_ID_DEFAULT = "EXP-20260325-data2-staged-g5-no-imu-scale-r2"
OUTPUT_DIR_DEFAULT = Path("output/data2_staged_g5_no_imu_scale_r2_20260325")
BASE_CONFIG_DEFAULT = Path("config_data2_research_seed_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 90.0
PHASE3_GNSS_OFF_DEFAULT = 90.0
G5_EXTRINSIC_NOISE_SCALE = 0.01
G5_MEASUREMENT_NOISE_SCALE = 5.0


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    description: str
    measurement_noise_scale: float
    measurement_noise_rel_to_g5: float
    phase3_enable_odo: bool = True
    phase3_enable_nhc: bool = True
    phase3_constraint_mode: str = "odo_nhc_enabled"


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        case_id="staged_g5_odo_nhc_noise_4x",
        label="staged G5 ODO/NHC noise 4x",
        color="#e45756",
        description=(
            "Three-phase staged G5 baseline with ODO/NHC measurement noise quadrupled relative to G5."
        ),
        measurement_noise_scale=G5_MEASUREMENT_NOISE_SCALE * 4.0,
        measurement_noise_rel_to_g5=4.0,
    ),
    CaseSpec(
        case_id="staged_g5_odo_nhc_noise_5x",
        label="staged G5 ODO/NHC noise 5x",
        color="#72b7b2",
        description=(
            "Three-phase staged G5 baseline with ODO/NHC measurement noise quintupled relative to G5."
        ),
        measurement_noise_scale=G5_MEASUREMENT_NOISE_SCALE * 5.0,
        measurement_noise_rel_to_g5=5.0,
    ),
    CaseSpec(
        case_id="staged_g5_odo_nhc_noise_6x",
        label="staged G5 ODO/NHC noise 6x",
        color="#54a24b",
        description=(
            "Three-phase staged G5 baseline with ODO/NHC measurement noise sextupled relative to G5."
        ),
        measurement_noise_scale=G5_MEASUREMENT_NOISE_SCALE * 6.0,
        measurement_noise_rel_to_g5=6.0,
    ),
    CaseSpec(
        case_id="staged_g5_odo_nhc_noise_6x_phase3_ins_only",
        label="staged G5 6x with phase3 INS-only",
        color="#4c78a8",
        description=(
            "Three-phase staged G5 baseline with 6x ODO/NHC noise in phase2, but phase3 disables "
            "ODO/NHC and keeps only INS/GNSS with the same periodic GNSS outage schedule."
        ),
        measurement_noise_scale=G5_MEASUREMENT_NOISE_SCALE * 6.0,
        measurement_noise_rel_to_g5=6.0,
        phase3_enable_odo=False,
        phase3_enable_nhc=False,
        phase3_constraint_mode="ins_gnss_outage_control",
    ),
)
CASE_MAP = {spec.case_id: spec for spec in CASE_SPECS}
PVA_ERROR_GROUP_SPECS: tuple[GroupSpec, ...] = (
    GroupSpec("position", "Position Error", GROUP_SPECS[0].states),
    GroupSpec("velocity", "Velocity Error", GROUP_SPECS[1].states),
    GroupSpec("attitude", "Attitude Error", GROUP_SPECS[2].states),
)
MAINLINE_TRUTH_KEYS_TO_HIDE = {
    "ba_x_mgal",
    "ba_y_mgal",
    "ba_z_mgal",
    "bg_x_degh",
    "bg_y_degh",
    "bg_z_degh",
}
MAINLINE_OBSOLETE_PLOT_FILES = ("sg.png", "sa.png")


@dataclass(frozen=True)
class MainlinePlotConfig:
    overview_states: tuple[Any, ...]
    group_specs: tuple[GroupSpec, ...]
    truth_keys_to_hide: set[str]


def build_mainline_plot_config() -> MainlinePlotConfig:
    kept_group_specs = tuple(group for group in GROUP_SPECS if group.group_id not in {"sg", "sa"})
    overview_states = tuple(state for group in kept_group_specs for state in group.states)
    return MainlinePlotConfig(
        overview_states=overview_states,
        group_specs=kept_group_specs,
        truth_keys_to_hide=set(MAINLINE_TRUTH_KEYS_TO_HIDE),
    )


def remove_obsolete_mainline_plot_files(plot_dir: Path) -> None:
    for filename in MAINLINE_OBSOLETE_PLOT_FILES:
        path = plot_dir / filename
        if path.exists():
            path.unlink()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the data2 staged G5 no-IMU-scale experiment with periodic GNSS outage."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--phase1-end-offset", type=float, default=PHASE1_END_OFFSET_DEFAULT)
    parser.add_argument("--phase2-end-offset", type=float, default=PHASE2_END_OFFSET_DEFAULT)
    parser.add_argument("--phase3-gnss-on", type=float, default=PHASE3_GNSS_ON_DEFAULT)
    parser.add_argument("--phase3-gnss-off", type=float, default=PHASE3_GNSS_OFF_DEFAULT)
    parser.add_argument("--cases", nargs="*", default=None)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    return args


def selected_case_specs(case_ids: list[str] | None) -> list[CaseSpec]:
    if not case_ids:
        return list(CASE_SPECS)
    seen: set[str] = set()
    selected: list[CaseSpec] = []
    for case_id in case_ids:
        if case_id not in CASE_MAP:
            raise ValueError(f"unsupported case_id: {case_id}")
        if case_id not in seen:
            selected.append(CASE_MAP[case_id])
            seen.add(case_id)
    return selected


def build_runtime_phases(
    cfg: dict[str, Any],
    phase1_end_time: float,
    phase2_end_time: float,
    final_time: float,
    spec: CaseSpec | None = None,
) -> list[dict[str, Any]]:
    start_time = float(cfg["fusion"]["starttime"])
    phase3_enable_odo = True if spec is None else bool(spec.phase3_enable_odo)
    phase3_enable_nhc = True if spec is None else bool(spec.phase3_enable_nhc)
    return [
        {
            "name": "phase1_ins_gnss_freeze_mounting_odo_states",
            "start_time": start_time,
            "end_time": float(phase1_end_time),
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        },
        {
            "name": "phase2_ins_gnss_odo_nhc_freeze_gnss_lever",
            "start_time": float(phase1_end_time),
            "end_time": float(phase2_end_time),
            "ablation": {
                "disable_gnss_lever_arm": True,
            },
            "constraints": {"enable_odo": True, "enable_nhc": True},
        },
        {
            "name": "phase3_periodic_gnss_outage_freeze_calibrated_extrinsics",
            "start_time": float(phase2_end_time),
            "end_time": float(final_time),
            "ablation": {
                "disable_gnss_lever_arm": True,
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
            },
            "constraints": {
                "enable_odo": phase3_enable_odo,
                "enable_nhc": phase3_enable_nhc,
            },
        },
    ]


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    fej_cfg = fusion.setdefault("fej", {})
    base_noise = base_cfg["fusion"]["noise"]
    base_constraints = base_cfg["fusion"]["constraints"]

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion.pop("post_gnss_ablation", None)

    fej_cfg["enable"] = False
    constraints_cfg["enable_odo"] = True
    constraints_cfg["enable_nhc"] = True
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["runtime_truth_anchor_pva"] = False
    init_cfg["runtime_truth_anchor_position"] = False
    init_cfg["runtime_truth_anchor_velocity"] = False
    init_cfg["runtime_truth_anchor_attitude"] = False
    init_cfg["runtime_truth_anchor_gnss_only"] = False
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    ablation_cfg["disable_mounting"] = False
    ablation_cfg["disable_mounting_roll"] = False
    ablation_cfg["disable_odo_lever_arm"] = False
    ablation_cfg["disable_gnss_lever_arm"] = False
    ablation_cfg["disable_gnss_lever_z"] = False
    ablation_cfg["disable_odo_scale"] = False
    ablation_cfg["disable_gyro_scale"] = True
    ablation_cfg["disable_accel_scale"] = True

    init_cfg["mounting_roll0"] = state_reference_value(truth_reference, "mounting_roll")
    init_cfg["mounting_pitch0"] = state_reference_value(truth_reference, "mounting_pitch")
    init_cfg["mounting_yaw0"] = state_reference_value(truth_reference, "mounting_yaw")
    odo_lever_truth = [float(x) for x in truth_reference["sources"]["odo_lever_truth"]["value_m"]]
    gnss_lever_truth = [float(x) for x in truth_reference["sources"]["gnss_lever_truth"]["value_m"]]
    init_cfg["lever_arm0"] = odo_lever_truth
    init_cfg["gnss_lever_arm0"] = gnss_lever_truth
    init_cfg["odo_scale"] = 1.0
    constraints_cfg["odo_lever_arm"] = odo_lever_truth

    scale_noise_fields(
        noise_cfg,
        base_noise,
        [
            "sigma_mounting",
            "sigma_mounting_roll",
            "sigma_mounting_pitch",
            "sigma_mounting_yaw",
            "sigma_lever_arm",
            "sigma_gnss_lever_arm",
        ],
        G5_EXTRINSIC_NOISE_SCALE,
    )
    scale_optional_vector_noise(noise_cfg, base_noise, "sigma_lever_arm_vec", G5_EXTRINSIC_NOISE_SCALE)
    scale_optional_vector_noise(noise_cfg, base_noise, "sigma_gnss_lever_arm_vec", G5_EXTRINSIC_NOISE_SCALE)

    constraints_cfg["sigma_odo"] = float(base_constraints["sigma_odo"]) * spec.measurement_noise_scale
    constraints_cfg["sigma_nhc_y"] = float(base_constraints["sigma_nhc_y"]) * spec.measurement_noise_scale
    constraints_cfg["sigma_nhc_z"] = float(base_constraints["sigma_nhc_z"]) * spec.measurement_noise_scale

    start_time = float(fusion["starttime"])
    final_time = float(fusion["finaltime"])
    phase1_end_time, phase2_end_time = phase_absolute_times(
        start_time,
        args.phase1_end_offset,
        args.phase2_end_offset,
    )
    on_windows, off_windows = build_periodic_gnss_windows(
        start_time=start_time,
        final_time=final_time,
        phase2_end_time=phase2_end_time,
        phase3_on_duration=args.phase3_gnss_on,
        phase3_off_duration=args.phase3_gnss_off,
    )
    fusion["gnss_schedule"] = {
        "enabled": True,
        "enabled_windows": [
            {"start_time": float(win_start), "end_time": float(win_end)}
            for win_start, win_end in on_windows
        ],
    }
    fusion["runtime_phases"] = build_runtime_phases(cfg, phase1_end_time, phase2_end_time, final_time, spec)

    metadata: dict[str, Any] = {
        "description": spec.description,
        "truth_initialized": [
            "mounting(22-24)",
            "lever_odo(25-27)",
            "lever_gnss(28-30)",
            "odo_scale(21)=1.0",
        ],
        "frozen_states": [
            "sg(15-17)",
            "sa(18-20)",
        ],
        "phase_frozen_states": {
            "phase1": ["odo_scale(21)", "mounting(22-24)", "lever_odo(25-27)"],
            "phase2": ["lever_gnss(28-30)"],
            "phase3": ["lever_gnss(28-30)", "odo_scale(21)", "mounting(22-24)", "lever_odo(25-27)"],
        },
        "noise_scales": {
            "extrinsics_q_scale": G5_EXTRINSIC_NOISE_SCALE,
            "odo_nhc_r_scale": spec.measurement_noise_scale,
            "odo_nhc_r_scale_rel_to_g5": spec.measurement_noise_rel_to_g5,
        },
        "phase3_constraint_mode": spec.phase3_constraint_mode,
        "phase1_end_time": float(phase1_end_time),
        "phase2_end_time": float(phase2_end_time),
        "gnss_on_windows": [(float(start), float(end)) for start, end in on_windows],
        "gnss_off_windows": [(float(start), float(end)) for start, end in off_windows],
        "config_path": rel_from_root(case_dir / f"config_{spec.case_id}.yaml", REPO_ROOT),
    }
    return cfg, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    args: argparse.Namespace,
) -> tuple[Path, dict[str, Any]]:
    cfg, metadata = build_case_config(base_cfg, truth_reference, case_dir, spec, args)
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    metadata["config_mtime"] = mtime_text(cfg_path)
    return cfg_path, metadata


def boundary_lines(ax: plt.Axes, phase1_end_time: float, phase2_end_time: float) -> None:
    ax.axvline(phase1_end_time, color="#333333", linestyle="--", linewidth=0.9)
    ax.axvline(phase2_end_time, color="#333333", linestyle="--", linewidth=0.9)


def shade_gnss_off_windows(ax: plt.Axes, off_windows: list[tuple[float, float]]) -> None:
    for start_time, end_time in off_windows:
        ax.axvspan(start_time, end_time, color="#f4d3d3", alpha=0.30)


def state_error_values(df: pd.DataFrame, state_spec: Any) -> np.ndarray:
    values = df[state_spec.key].to_numpy(dtype=float)
    truth = df[state_spec.truth_column].to_numpy(dtype=float)
    if state_spec.key in {"roll_deg", "pitch_deg", "yaw_deg"}:
        return wrap_deg(values - truth)
    return values - truth


def plot_state_grid(
    case_frames: dict[str, pd.DataFrame],
    case_specs: list[CaseSpec],
    state_specs: tuple[Any, ...],
    output_path: Path,
    title: str,
    phase1_end_time: float,
    phase2_end_time: float,
    gnss_off_windows: list[tuple[float, float]],
    plot_mode: str = "state",
    plot_modes_by_key: dict[str, str] | None = None,
    truth_keys_to_hide: set[str] | None = None,
) -> None:
    plot_modes_by_key = {} if plot_modes_by_key is None else dict(plot_modes_by_key)
    truth_keys_to_hide = set() if truth_keys_to_hide is None else set(truth_keys_to_hide)
    n_cols = 3
    n_rows = math.ceil(len(state_specs) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 2.9 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, state_spec in enumerate(state_specs):
        ax = axes_flat[idx]
        shade_gnss_off_windows(ax, gnss_off_windows)
        truth_plotted = False
        state_plot_mode = plot_modes_by_key.get(state_spec.key, plot_mode)
        for case_spec in case_specs:
            df = case_frames[case_spec.case_id]
            t = df["timestamp"].to_numpy(dtype=float)
            if state_plot_mode == "error":
                values = state_error_values(df, state_spec)
            else:
                values = df[state_spec.key].to_numpy(dtype=float)
            t_plot, v_plot = downsample_for_plot(t, values)
            ax.plot(t_plot, v_plot, linewidth=0.95, color=case_spec.color, label=case_spec.label)
            if (
                state_plot_mode == "state"
                and state_spec.truth_column
                and state_spec.key not in truth_keys_to_hide
                and not truth_plotted
            ):
                truth_values = df[state_spec.truth_column].to_numpy(dtype=float)
                tt_plot, tv_plot = downsample_for_plot(t, truth_values)
                ax.plot(tt_plot, tv_plot, linewidth=1.0, color="black", linestyle="--", label="truth")
                truth_plotted = True
        boundary_lines(ax, phase1_end_time, phase2_end_time)
        title_suffix = " err" if state_plot_mode == "error" else ""
        ax.set_title(f"{state_spec.label}{title_suffix} [{state_spec.unit}]", fontsize=9)
        ax.grid(alpha=0.2)
        ax.tick_params(axis="both", labelsize=7)
        if idx == 0:
            ax.legend(loc="best", fontsize=7)
    for idx in range(len(state_specs), len(axes_flat)):
        axes_flat[idx].axis("off")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def position_error_3d(state_frame: pd.DataFrame) -> np.ndarray:
    return np.linalg.norm(
        np.column_stack(
            [
                state_frame["p_n_m"].to_numpy(dtype=float) - state_frame["truth_p_n_m"].to_numpy(dtype=float),
                state_frame["p_e_m"].to_numpy(dtype=float) - state_frame["truth_p_e_m"].to_numpy(dtype=float),
                state_frame["p_u_m"].to_numpy(dtype=float) - state_frame["truth_p_u_m"].to_numpy(dtype=float),
            ]
        ),
        axis=1,
    )


def compute_phase_metrics(
    state_frame: pd.DataFrame,
    case_id: str,
    phase_windows: list[tuple[str, float, float]],
    gnss_off_windows: list[tuple[float, float]],
) -> pd.DataFrame:
    err3 = position_error_3d(state_frame)
    yaw_err = wrap_deg(
        state_frame["yaw_deg"].to_numpy(dtype=float) - state_frame["truth_yaw_deg"].to_numpy(dtype=float)
    )
    bg_z_err = (
        state_frame["bg_z_degh"].to_numpy(dtype=float) - state_frame["truth_bg_z_degh"].to_numpy(dtype=float)
    )
    timestamps = state_frame["timestamp"].to_numpy(dtype=float)

    rows: list[dict[str, Any]] = []
    for phase_name, start_time, end_time in phase_windows:
        mask = (timestamps >= start_time) & (timestamps <= end_time)
        if not np.any(mask):
            continue
        err_subset = err3[mask]
        rows.append(
            {
                "case_id": case_id,
                "window_type": "phase",
                "window_name": phase_name,
                "start_time": float(start_time),
                "end_time": float(end_time),
                "samples": int(np.count_nonzero(mask)),
                "rmse_3d_m": float(np.sqrt(np.mean(err_subset * err_subset))),
                "p95_3d_m": float(np.percentile(err_subset, 95.0)),
                "final_err_3d_m": float(err_subset[-1]),
                "yaw_err_abs_max_deg": float(np.max(np.abs(yaw_err[mask]))),
                "bg_z_err_abs_max_degh": float(np.max(np.abs(bg_z_err[mask]))),
            }
        )
    for idx, (start_time, end_time) in enumerate(gnss_off_windows, start=1):
        mask = (timestamps >= start_time) & (timestamps <= end_time)
        if not np.any(mask):
            continue
        err_subset = err3[mask]
        rows.append(
            {
                "case_id": case_id,
                "window_type": "gnss_off",
                "window_name": f"gnss_off_{idx:02d}",
                "start_time": float(start_time),
                "end_time": float(end_time),
                "samples": int(np.count_nonzero(mask)),
                "rmse_3d_m": float(np.sqrt(np.mean(err_subset * err_subset))),
                "p95_3d_m": float(np.percentile(err_subset, 95.0)),
                "final_err_3d_m": float(err_subset[-1]),
                "yaw_err_abs_max_deg": float(np.max(np.abs(yaw_err[mask]))),
                "bg_z_err_abs_max_degh": float(np.max(np.abs(bg_z_err[mask]))),
            }
        )
    return pd.DataFrame(rows)


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    phase_metrics_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    overall_rows = [
        [
            str(row["case_id"]),
            format_metric(row.get("measurement_noise_rel_to_g5")),
            str(row.get("phase3_constraint_mode", "")),
            format_metric(row.get("overall_rmse_3d_m_aux")),
            format_metric(row.get("phase2_rmse_3d_m")),
            format_metric(row.get("phase3_rmse_3d_m")),
            format_metric(row.get("overall_final_err_3d_m_aux")),
            format_metric(row.get("yaw_err_max_abs_deg")),
            format_metric(row.get("bg_z_degh_err_max_abs")),
            format_metric(row.get("odo_accept_ratio")),
            format_metric(row.get("nhc_accept_ratio")),
        ]
        for _, row in case_metrics_df.iterrows()
    ]
    phase_rows = [
        [
            str(row["case_id"]),
            str(row["window_name"]),
            format_metric(row["rmse_3d_m"]),
            format_metric(row["p95_3d_m"]),
            format_metric(row["final_err_3d_m"]),
            format_metric(row["yaw_err_abs_max_deg"]),
            format_metric(row["bg_z_err_abs_max_degh"]),
        ]
        for _, row in phase_metrics_df.iterrows()
    ]
    lines = [
        "# data2 staged G5 no-IMU-scale summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- phase windows: `phase1={manifest['phase_windows']['phase1']}`, `phase2={manifest['phase_windows']['phase2']}`, `phase3={manifest['phase_windows']['phase3']}`",
        (
            f"- phase3 periodic GNSS: `on={manifest['phase3_periodic_gnss']['on_duration_s']:.1f}s`, "
            f"`off={manifest['phase3_periodic_gnss']['off_duration_s']:.1f}s`"
        ),
        f"- generated_at: `{manifest['generated_at']}`",
        "",
        "## Case Metrics",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "noise_rel_g5",
                "phase3_mode",
                "rmse3d_m",
                "phase2_rmse3d_m",
                "phase3_rmse3d_m",
                "final_3d_m",
                "yaw_err_max_abs_deg",
                "bg_z_err_max_abs_degh",
                "odo_accept_ratio",
                "nhc_accept_ratio",
            ],
            overall_rows,
        )
    )
    lines.extend(["", "## Phase Metrics"])
    lines.extend(
        render_table(
            ["case_id", "window", "rmse3d_m", "p95_3d_m", "final_3d_m", "yaw_abs_max_deg", "bg_z_abs_max_degh"],
            phase_rows,
        )
    )
    lines.extend(["", "## Plot Outputs"])
    for key, path in plot_paths.items():
        lines.append(f"- `{key}`: `{path}`")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")

    selected_specs = selected_case_specs(args.cases)
    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())

    case_frames: dict[str, pd.DataFrame] = {}
    case_rows: list[dict[str, Any]] = []
    phase_metric_frames: list[pd.DataFrame] = []
    case_metadata: dict[str, dict[str, Any]] = {}

    for spec in selected_specs:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, metadata = write_case_config(base_cfg, truth_reference, case_dir, spec, args)
        case_metadata[spec.case_id] = metadata

        baseline_spec = BaselineCaseSpec(
            case_id=spec.case_id,
            label=spec.label,
            color=spec.color,
            filter_mode="ESKF",
            enable_fej=False,
            enable_runtime_anchor=False,
        )
        case_row = run_baseline_case(case_dir, cfg_path, args.exe, baseline_spec)

        sol_path = (REPO_ROOT / case_row["sol_path"]).resolve()
        state_series_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        merged_df = merge_case_outputs(sol_path, state_series_path)
        truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
        state_frame = build_state_frame(merged_df, truth_interp_df, truth_reference)

        state_frame_path = case_dir / f"all_states_{spec.case_id}.csv"
        state_frame.to_csv(state_frame_path, index=False, encoding="utf-8-sig")
        case_row["all_states_path"] = rel_from_root(state_frame_path, REPO_ROOT)
        case_row["all_states_mtime"] = mtime_text(state_frame_path)
        case_row["case_description"] = spec.description
        case_row["config_mtime"] = mtime_text(cfg_path)

        phase_windows = [
            ("phase1_ins_gnss", float(base_cfg["fusion"]["starttime"]), metadata["phase1_end_time"]),
            ("phase2_ins_gnss_odo_nhc", metadata["phase1_end_time"], metadata["phase2_end_time"]),
            ("phase3_periodic_gnss_outage", metadata["phase2_end_time"], float(base_cfg["fusion"]["finaltime"])),
        ]
        phase_df = compute_phase_metrics(state_frame, spec.case_id, phase_windows, metadata["gnss_off_windows"])
        phase_metric_frames.append(phase_df)

        metrics_row = compute_case_metrics(case_row, state_frame)
        metrics_row["measurement_noise_scale"] = spec.measurement_noise_scale
        metrics_row["measurement_noise_rel_to_g5"] = spec.measurement_noise_rel_to_g5
        metrics_row["phase3_constraint_mode"] = spec.phase3_constraint_mode
        metrics_row["phase2_rmse_3d_m"] = float(
            phase_df.loc[phase_df["window_name"] == "phase2_ins_gnss_odo_nhc", "rmse_3d_m"].iloc[0]
        )
        metrics_row["phase3_rmse_3d_m"] = float(
            phase_df.loc[phase_df["window_name"] == "phase3_periodic_gnss_outage", "rmse_3d_m"].iloc[0]
        )
        case_rows.append(metrics_row)
        case_frames[spec.case_id] = state_frame

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df = case_metrics_df.set_index("case_id").loc[[spec.case_id for spec in selected_specs]].reset_index()
    phase_metrics_df = pd.concat(phase_metric_frames, ignore_index=True) if phase_metric_frames else pd.DataFrame()

    case_metrics_path = args.output_dir / "case_metrics.csv"
    phase_metrics_path = args.output_dir / "phase_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    phase_metrics_df.to_csv(phase_metrics_path, index=False, encoding="utf-8-sig")

    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    ref_case_id = selected_specs[0].case_id
    phase1_end_time = float(case_metadata[ref_case_id]["phase1_end_time"])
    phase2_end_time = float(case_metadata[ref_case_id]["phase2_end_time"])
    gnss_off_windows = [(float(start), float(end)) for start, end in case_metadata[ref_case_id]["gnss_off_windows"]]
    plot_config = build_mainline_plot_config()
    remove_obsolete_mainline_plot_files(args.plot_dir)

    plot_paths: dict[str, str] = {}
    all_states_path = args.plot_dir / "all_states_overview.png"
    plot_state_grid(
        case_frames,
        selected_specs,
        plot_config.overview_states,
        all_states_path,
        "Staged G5 all-state overview",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["all_states_overview"] = rel_from_root(all_states_path, REPO_ROOT)

    key_states_path = args.plot_dir / "key_coupling_states.png"
    plot_state_grid(
        case_frames,
        selected_specs,
        KEY_COUPLING_STATES,
        key_states_path,
        "Staged G5 key coupling states",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["key_coupling_states"] = rel_from_root(key_states_path, REPO_ROOT)

    for group_spec in plot_config.group_specs:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        plot_mode = "error" if group_spec.group_id in {"position", "velocity", "attitude"} else "state"
        selected_group = group_spec
        if plot_mode == "error":
            selected_group = next(item for item in PVA_ERROR_GROUP_SPECS if item.group_id == group_spec.group_id)
        plot_state_grid(
            case_frames,
            selected_specs,
            selected_group.states,
            group_path,
            f"Staged G5 {selected_group.title}",
            phase1_end_time,
            phase2_end_time,
            gnss_off_windows,
            plot_mode=plot_mode,
            truth_keys_to_hide=plot_config.truth_keys_to_hide,
        )
        plot_paths[group_spec.group_id] = rel_from_root(group_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    manifest = {
        "exp_id": args.exp_id,
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "solver": rel_from_root(args.exe, REPO_ROOT),
        "cases": [spec.case_id for spec in selected_specs],
        "case_metadata": json_safe(case_metadata),
        "phase_windows": {
            "phase1": [float(base_cfg["fusion"]["starttime"]), phase1_end_time],
            "phase2": [phase1_end_time, phase2_end_time],
            "phase3": [phase2_end_time, float(base_cfg["fusion"]["finaltime"])],
        },
        "phase3_periodic_gnss": {
            "on_duration_s": float(args.phase3_gnss_on),
            "off_duration_s": float(args.phase3_gnss_off),
            "gnss_on_windows": json_safe(case_metadata[ref_case_id]["gnss_on_windows"]),
            "gnss_off_windows": json_safe(case_metadata[ref_case_id]["gnss_off_windows"]),
        },
        "truth_reference_path": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_path": rel_from_root(case_metrics_path, REPO_ROOT),
        "phase_metrics_path": rel_from_root(phase_metrics_path, REPO_ROOT),
        "plot_paths": plot_paths,
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_mtime": mtime_text(args.exe),
            "case_metrics_mtime": mtime_text(case_metrics_path),
            "phase_metrics_mtime": mtime_text(phase_metrics_path),
        },
    }
    write_summary(summary_path, manifest, case_metrics_df, phase_metrics_df, plot_paths)
    manifest["freshness"]["summary_md_mtime"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
