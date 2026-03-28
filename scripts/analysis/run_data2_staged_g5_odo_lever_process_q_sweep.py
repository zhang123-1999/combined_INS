import argparse
import copy
import datetime as dt
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.interactive_nav_report import euler_to_rotation, relative_euler_error_deg
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_fullwindow_attitude_bias_coupling import (
    GroupSpec,
    StateSpec,
    build_state_frame,
    compute_case_metrics,
    format_metric,
    render_table,
    scale_noise_fields,
    scale_optional_vector_noise,
)
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import (
    CaseSpec as BaselineCaseSpec,
    mtime_text,
    run_case as run_baseline_case,
)
from scripts.analysis.run_data2_staged_estimation import build_periodic_gnss_windows, phase_absolute_times
from scripts.analysis.run_data2_staged_g5_mounting_zero_init_q_sweep import (
    G5_EXTRINSIC_NOISE_SCALE,
    G5_ODO_NHC_MEASUREMENT_NOISE_SCALE,
    G5_ODO_NHC_R_SCALE_REL_TO_G5,
    MOUNTING_INIT_STD_DEG,
    large_mounting_variance,
)
from scripts.analysis.run_data2_staged_g5_no_imu_scale import (
    ALL_STATE_SPECS,
    GROUP_SPECS,
    KEY_COUPLING_STATES,
    PVA_ERROR_GROUP_SPECS,
    build_runtime_phases as build_base_runtime_phases,
    compute_phase_metrics,
    plot_state_grid,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    base_p0_diag_from_config,
    build_truth_reference,
    json_safe,
    reset_directory,
)
from scripts.analysis.run_nhc_state_convergence_research import (
    build_motion_frame,
    build_truth_interp,
    load_imu_dataframe,
    load_pos_dataframe,
    merge_case_outputs,
)


EXP_ID_DEFAULT = "EXP-20260326-data2-staged-g5-odo-lever-process-q-sweep-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_staged_g5_odo_lever_process_q_sweep_r1_20260326")
BASE_CONFIG_DEFAULT = Path("config_data2_research_seed_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PHASE2_END_OFFSET_DEFAULT = 700.0
PHASE3_GNSS_ON_DEFAULT = 90.0
PHASE3_GNSS_OFF_DEFAULT = 90.0
PHASE2_COV_SEED_DURATION_DEFAULT = 0.02
MOUNTING_STATE_INDICES = (22, 23, 24)
MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED = 1.0
ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED = 2.0
ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED = 4.0
ZERO_VEC3 = [0.0, 0.0, 0.0]
WORKING_TOTAL_MOUNTING_DEG = {"roll": 0.0, "pitch": 0.36, "yaw": 0.84}


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    description: str
    odo_lever_process_q_scale_rel_to_staged_g5: float
    measurement_noise_scale: float = G5_ODO_NHC_MEASUREMENT_NOISE_SCALE
    measurement_noise_rel_to_g5: float = G5_ODO_NHC_R_SCALE_REL_TO_G5


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        case_id="staged_g5_odo_lever_q_0p25x",
        label="staged G5 odo_lever q 0.25x",
        color="#e45756",
        description="Fixed odo_scale-seed-2x and odo_lever-seed-4x baseline with odo_lever process noise at 0.25x of staged G5.",
        odo_lever_process_q_scale_rel_to_staged_g5=0.25,
    ),
    CaseSpec(
        case_id="staged_g5_odo_lever_q_0p5x",
        label="staged G5 odo_lever q 0.5x",
        color="#f58518",
        description="Fixed odo_scale-seed-2x and odo_lever-seed-4x baseline with odo_lever process noise at 0.5x of staged G5.",
        odo_lever_process_q_scale_rel_to_staged_g5=0.5,
    ),
    CaseSpec(
        case_id="staged_g5_odo_lever_q_1x",
        label="staged G5 odo_lever q 1x",
        color="#54a24b",
        description="Fixed odo_scale-seed-2x and odo_lever-seed-4x baseline with nominal staged G5 odo_lever process noise.",
        odo_lever_process_q_scale_rel_to_staged_g5=1.0,
    ),
    CaseSpec(
        case_id="staged_g5_odo_lever_q_2x",
        label="staged G5 odo_lever q 2x",
        color="#4c78a8",
        description="Fixed odo_scale-seed-2x and odo_lever-seed-4x baseline with odo_lever process noise at 2x of staged G5.",
        odo_lever_process_q_scale_rel_to_staged_g5=2.0,
    ),
    CaseSpec(
        case_id="staged_g5_odo_lever_q_4x",
        label="staged G5 odo_lever q 4x",
        color="#b279a2",
        description="Fixed odo_scale-seed-2x and odo_lever-seed-4x baseline with odo_lever process noise at 4x of staged G5.",
        odo_lever_process_q_scale_rel_to_staged_g5=4.0,
    ),
)
CASE_MAP = {spec.case_id: spec for spec in CASE_SPECS}
TRUTH_KEYS_TO_HIDE = {
    "ba_x_mgal",
    "ba_y_mgal",
    "ba_z_mgal",
    "bg_x_degh",
    "bg_y_degh",
    "bg_z_degh",
}
VEHICLE_VELOCITY_ERROR_STATES: tuple[StateSpec, ...] = (
    StateSpec("vv_x_err_mps", "v_v.x", "m/s", None, None),
    StateSpec("vv_y_err_mps", "v_v.y", "m/s", None, None),
    StateSpec("vv_z_err_mps", "v_v.z", "m/s", None, None),
)
VEHICLE_HEADING_ERROR_STATES: tuple[StateSpec, ...] = (
    StateSpec("vehicle_heading_err_deg", "vehicle heading", "deg", None, None),
)


@dataclass(frozen=True)
class BestRerunPlotConfig:
    overview_states: tuple[StateSpec, ...]
    plot_modes_by_key: dict[str, str]
    group_specs: tuple[GroupSpec, ...]
    truth_keys_to_hide: set[str]
    vehicle_velocity_error_states: tuple[StateSpec, ...]
    vehicle_heading_error_states: tuple[StateSpec, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the data2 staged G5 odo_lever process-noise sweep on top of the fixed odo_scale-seed-2x and odo_lever-seed-4x baseline."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--phase1-end-offset", type=float, default=PHASE1_END_OFFSET_DEFAULT)
    parser.add_argument("--phase2-end-offset", type=float, default=PHASE2_END_OFFSET_DEFAULT)
    parser.add_argument("--phase3-gnss-on", type=float, default=PHASE3_GNSS_ON_DEFAULT)
    parser.add_argument("--phase3-gnss-off", type=float, default=PHASE3_GNSS_OFF_DEFAULT)
    parser.add_argument("--phase2-cov-seed-duration", type=float, default=PHASE2_COV_SEED_DURATION_DEFAULT)
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


def build_best_rerun_plot_config() -> BestRerunPlotConfig:
    kept_group_specs = tuple(group for group in GROUP_SPECS if group.group_id not in {"sg", "sa"})
    pva_error_keys = {
        state.key
        for group in PVA_ERROR_GROUP_SPECS
        for state in group.states
    }
    overview_states = tuple(state for group in kept_group_specs for state in group.states)
    plot_modes_by_key = {state_key: "error" for state_key in pva_error_keys}
    return BestRerunPlotConfig(
        overview_states=overview_states,
        plot_modes_by_key=plot_modes_by_key,
        group_specs=kept_group_specs,
        truth_keys_to_hide=set(TRUTH_KEYS_TO_HIDE),
        vehicle_velocity_error_states=VEHICLE_VELOCITY_ERROR_STATES,
        vehicle_heading_error_states=VEHICLE_HEADING_ERROR_STATES,
    )


def build_vehicle_truth_reference(truth_reference: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(truth_reference)
    out.setdefault("sources", {}).setdefault("mounting_total_truth", {})["value_deg"] = dict(WORKING_TOTAL_MOUNTING_DEG)
    return out


def build_vehicle_error_frame(
    merged_df: pd.DataFrame,
    truth_interp_df: pd.DataFrame,
    imu_df: pd.DataFrame,
    truth_reference: dict[str, Any],
) -> pd.DataFrame:
    vehicle_truth_reference = build_vehicle_truth_reference(truth_reference)
    motion_df = build_motion_frame(merged_df, truth_interp_df, imu_df, vehicle_truth_reference)

    fused_roll = np.deg2rad(merged_df["fused_roll"].to_numpy(dtype=float))
    fused_pitch = np.deg2rad(merged_df["fused_pitch"].to_numpy(dtype=float))
    fused_yaw = np.deg2rad(merged_df["fused_yaw"].to_numpy(dtype=float))
    truth_roll = np.deg2rad(truth_interp_df["roll"].to_numpy(dtype=float))
    truth_pitch = np.deg2rad(truth_interp_df["pitch"].to_numpy(dtype=float))
    truth_yaw = np.deg2rad(truth_interp_df["yaw"].to_numpy(dtype=float))

    body_to_nav = euler_to_rotation(fused_roll, fused_pitch, fused_yaw)
    truth_body_to_nav = euler_to_rotation(truth_roll, truth_pitch, truth_yaw)
    body_to_vehicle = euler_to_rotation(
        np.deg2rad(merged_df["total_mounting_roll_deg"].to_numpy(dtype=float)),
        np.deg2rad(merged_df["total_mounting_pitch_deg"].to_numpy(dtype=float)),
        np.deg2rad(merged_df["total_mounting_yaw_deg"].to_numpy(dtype=float)),
    )
    truth_body_to_vehicle = euler_to_rotation(
        np.full(len(merged_df), np.deg2rad(WORKING_TOTAL_MOUNTING_DEG["roll"]), dtype=float),
        np.full(len(merged_df), np.deg2rad(WORKING_TOTAL_MOUNTING_DEG["pitch"]), dtype=float),
        np.full(len(merged_df), np.deg2rad(WORKING_TOTAL_MOUNTING_DEG["yaw"]), dtype=float),
    )
    vehicle_to_nav = np.einsum("...ij,...jk->...ik", body_to_nav, np.swapaxes(body_to_vehicle, -1, -2))
    truth_vehicle_to_nav = np.einsum(
        "...ij,...jk->...ik",
        truth_body_to_nav,
        np.swapaxes(truth_body_to_vehicle, -1, -2),
    )
    _, _, vehicle_heading_err_deg = relative_euler_error_deg(vehicle_to_nav, truth_vehicle_to_nav)

    return pd.DataFrame(
        {
            "timestamp": merged_df["timestamp"].to_numpy(dtype=float),
            "vv_x_err_mps": motion_df["v_v_x_mps"].to_numpy(dtype=float)
            - motion_df["truth_v_v_x_mps"].to_numpy(dtype=float),
            "vv_y_err_mps": motion_df["v_v_y_mps"].to_numpy(dtype=float)
            - motion_df["truth_v_v_y_mps"].to_numpy(dtype=float),
            "vv_z_err_mps": motion_df["v_v_z_mps"].to_numpy(dtype=float)
            - motion_df["truth_v_v_z_mps"].to_numpy(dtype=float),
            "vehicle_heading_err_deg": vehicle_heading_err_deg,
        }
    )


def build_runtime_phases(
    cfg: dict[str, Any],
    phase1_end_time: float,
    phase2_end_time: float,
    final_time: float,
    phase2_cov_seed_duration: float,
    phase2_odo_scale_seed_var: float,
    phase2_lever_seed_var_vec: list[float],
) -> list[dict[str, Any]]:
    phase1, phase2, phase3 = build_base_runtime_phases(cfg, phase1_end_time, phase2_end_time, final_time)
    phase2_seed_end_time = min(
        float(phase2_end_time),
        float(phase1_end_time) + max(0.0, float(phase2_cov_seed_duration)),
    )
    phase2_seed = copy.deepcopy(phase2)
    phase2_seed["name"] = "phase2_cov_seed"
    phase2_seed["start_time"] = float(phase1_end_time)
    phase2_seed["end_time"] = float(phase2_seed_end_time)
    phase2_seed.setdefault("constraints", {})["enable_covariance_floor"] = True
    phase2_seed["constraints"]["p_floor_mounting_deg"] = float(MOUNTING_INIT_STD_DEG)
    phase2_seed["constraints"]["p_floor_odo_scale_var"] = float(phase2_odo_scale_seed_var)
    phase2_seed["constraints"]["p_floor_lever_arm_vec"] = [float(x) for x in phase2_lever_seed_var_vec]

    phase2_main = copy.deepcopy(phase2)
    phase2_main["start_time"] = float(phase2_seed_end_time)
    return [phase1, phase2_seed, phase2_main, phase3]


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
    base_init = base_cfg["fusion"]["init"]
    p0_diag = base_p0_diag_from_config(base_cfg)

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

    init_cfg["mounting_roll0"] = 0.0
    init_cfg["mounting_pitch0"] = 0.0
    init_cfg["mounting_yaw0"] = 0.0
    init_cfg["std_mounting_roll"] = MOUNTING_INIT_STD_DEG
    init_cfg["std_mounting_pitch"] = MOUNTING_INIT_STD_DEG
    init_cfg["std_mounting_yaw"] = MOUNTING_INIT_STD_DEG
    constraints_cfg["imu_mounting_angle"] = [0.0, 0.0, 0.0]

    gnss_lever_truth = [float(x) for x in truth_reference["sources"]["gnss_lever_truth"]["value_m"]]
    init_cfg["lever_arm0"] = list(ZERO_VEC3)
    init_cfg["gnss_lever_arm0"] = gnss_lever_truth
    init_cfg["odo_scale"] = 1.0
    constraints_cfg["odo_lever_arm"] = list(ZERO_VEC3)

    mount_var = large_mounting_variance()
    for idx in MOUNTING_STATE_INDICES:
        p0_diag[idx] = mount_var
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    lever_q_scale_rel_to_base_q = G5_EXTRINSIC_NOISE_SCALE * spec.odo_lever_process_q_scale_rel_to_staged_g5
    scale_noise_fields(noise_cfg, base_noise, ["sigma_lever_arm"], lever_q_scale_rel_to_base_q)
    scale_optional_vector_noise(noise_cfg, base_noise, "sigma_lever_arm_vec", lever_q_scale_rel_to_base_q)
    scale_noise_fields(noise_cfg, base_noise, ["sigma_gnss_lever_arm"], G5_EXTRINSIC_NOISE_SCALE)
    scale_optional_vector_noise(noise_cfg, base_noise, "sigma_gnss_lever_arm_vec", G5_EXTRINSIC_NOISE_SCALE)
    scale_noise_fields(
        noise_cfg,
        base_noise,
        ["sigma_mounting", "sigma_mounting_roll", "sigma_mounting_pitch", "sigma_mounting_yaw"],
        G5_EXTRINSIC_NOISE_SCALE * MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED,
    )

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

    phase2_odo_scale_seed_std = float(base_init["std_odo_scale"]) * ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED
    phase2_odo_scale_seed_var = phase2_odo_scale_seed_std * phase2_odo_scale_seed_std
    phase2_lever_seed_std_vec = [
        float(value) * ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED for value in base_init["std_lever_arm"]
    ]
    phase2_lever_seed_var_vec = [value * value for value in phase2_lever_seed_std_vec]

    fusion["gnss_schedule"] = {
        "enabled": True,
        "enabled_windows": [
            {"start_time": float(win_start), "end_time": float(win_end)}
            for win_start, win_end in on_windows
        ],
    }
    fusion["runtime_phases"] = build_runtime_phases(
        cfg,
        phase1_end_time,
        phase2_end_time,
        final_time,
        args.phase2_cov_seed_duration,
        phase2_odo_scale_seed_var,
        phase2_lever_seed_var_vec,
    )

    metadata: dict[str, Any] = {
        "description": spec.description,
        "mounting_init_mode": "zero_init_large_p0",
        "mounting_zero_init_semantics": "total_and_state_zero",
        "odo_lever_zero_init_semantics": "total_and_state_zero",
        "truth_initialized": ["lever_gnss(28-30)", "odo_scale(21)=1.0"],
        "zero_initialized": ["mounting(22-24)", "lever_odo(25-27)"],
        "frozen_states": ["sg(15-17)", "sa(18-20)"],
        "phase_frozen_states": {
            "phase1": ["odo_scale(21)", "mounting(22-24)", "lever_odo(25-27)"],
            "phase2": ["lever_gnss(28-30)"],
            "phase3": ["lever_gnss(28-30)", "odo_scale(21)", "mounting(22-24)", "lever_odo(25-27)"],
        },
        "large_initial_covariance": {
            "mounting_std_deg": float(MOUNTING_INIT_STD_DEG),
            "mounting_var_rad2": float(mount_var),
            "p0_indices": list(MOUNTING_STATE_INDICES),
        },
        "mounting_total_base_deg": [0.0, 0.0, 0.0],
        "mounting_total_init_deg": [0.0, 0.0, 0.0],
        "odo_lever_total_base_m": list(ZERO_VEC3),
        "odo_lever_total_init_m": list(ZERO_VEC3),
        "mounting_q_scale_rel_to_staged_6x": float(MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED),
        "odo_scale_phase2_seed": {
            "std": float(phase2_odo_scale_seed_std),
            "var": float(phase2_odo_scale_seed_var),
            "scale_rel_to_base_init": float(ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED),
        },
        "odo_lever_phase2_seed": {
            "std_vec_m": [float(x) for x in phase2_lever_seed_std_vec],
            "var_vec_m2": [float(x) for x in phase2_lever_seed_var_vec],
            "scale_rel_to_base_init": float(ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED),
        },
        "noise_scales": {
            "lever_q_scale_rel_to_staged_g5": float(spec.odo_lever_process_q_scale_rel_to_staged_g5),
            "lever_q_scale_rel_to_base_q": float(lever_q_scale_rel_to_base_q),
            "lever_q_sigma_m_sqrt_hz": float(noise_cfg["sigma_lever_arm"]),
            "gnss_lever_q_scale_rel_to_base_q": float(G5_EXTRINSIC_NOISE_SCALE),
            "mounting_q_scale_rel_to_staged_6x": float(MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED),
            "mounting_q_scale_rel_to_base_q": float(G5_EXTRINSIC_NOISE_SCALE * MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED),
            "odo_nhc_r_scale": spec.measurement_noise_scale,
            "odo_nhc_r_scale_rel_to_g5": spec.measurement_noise_rel_to_g5,
        },
        "phase1_end_time": float(phase1_end_time),
        "phase2_main_start_time": float(phase1_end_time + max(0.0, args.phase2_cov_seed_duration)),
        "phase2_end_time": float(phase2_end_time),
        "phase2_cov_seed_duration_s": float(args.phase2_cov_seed_duration),
        "phase2_mounting_cov_seed_floor_deg": float(MOUNTING_INIT_STD_DEG),
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
            format_metric(row.get("odo_lever_process_q_scale_rel_to_staged_g5")),
            format_metric(row.get("odo_lever_phase2_seed_scale_rel_to_base_init")),
            format_metric(row.get("measurement_noise_rel_to_g5")),
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
        "# data2 staged G5 odo_lever process-noise sweep summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- phase windows: `phase1={manifest['phase_windows']['phase1']}`, `phase2={manifest['phase_windows']['phase2']}`, `phase3={manifest['phase_windows']['phase3']}`",
        (
            f"- phase3 periodic GNSS: `on={manifest['phase3_periodic_gnss']['on_duration_s']:.1f}s`, "
            f"`off={manifest['phase3_periodic_gnss']['off_duration_s']:.1f}s`"
        ),
        (
            f"- fixed baseline: `mounting_q_rel_staged_6x={MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED:.1f}x`, "
            f"`odo_scale_phase2_seed_rel_init={ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED:.1f}x`, "
            f"`odo_lever_phase2_seed_rel_init={ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED:.1f}x`, "
            f"`odo_lever total/state init=[0,0,0] m`"
        ),
        (
            f"- fixed phase2 covariance seed: `duration={manifest['experiment_controls']['phase2_cov_seed_duration_s']:.3f}s`, "
            f"`p_floor_mounting_deg={manifest['experiment_controls']['phase2_mounting_cov_seed_floor_deg']:.1f}`, "
            f"`odo_scale_seed_var_2x={manifest['experiment_controls']['fixed_odo_scale_seed_var_2x']:.12g}`, "
            f"`odo_lever_seed_var_4x={manifest['experiment_controls']['fixed_odo_lever_seed_var_4x']}`"
        ),
        f"- fixed ODO/NHC measurement noise: `{G5_ODO_NHC_R_SCALE_REL_TO_G5:.1f}x relative to G5`",
        f"- generated_at: `{manifest['generated_at']}`",
        "",
        "## Case Metrics",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "lever_q_rel_staged_g5",
                "lever_seed_rel_init",
                "odo_nhc_r_rel_g5",
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
    base_truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())
    plot_config = build_best_rerun_plot_config()

    case_frames: dict[str, pd.DataFrame] = {}
    case_rows: list[dict[str, Any]] = []
    phase_metric_frames: list[pd.DataFrame] = []
    case_metadata: dict[str, dict[str, Any]] = {}
    output_truth_reference: dict[str, Any] | None = None

    for spec in selected_specs:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, metadata = write_case_config(base_cfg, base_truth_reference, case_dir, spec, args)
        case_metadata[spec.case_id] = metadata
        case_cfg = load_yaml(cfg_path)
        case_truth_reference = build_truth_reference(case_cfg)
        if output_truth_reference is None:
            output_truth_reference = case_truth_reference

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
        state_frame = build_state_frame(merged_df, truth_interp_df, case_truth_reference)
        vehicle_error_frame = build_vehicle_error_frame(merged_df, truth_interp_df, imu_df, case_truth_reference)
        state_frame = state_frame.merge(vehicle_error_frame, on="timestamp", how="left", validate="one_to_one")

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
        metrics_row["odo_lever_process_q_scale_rel_to_staged_g5"] = spec.odo_lever_process_q_scale_rel_to_staged_g5
        metrics_row["mounting_q_scale_rel_to_staged_6x"] = float(MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED)
        metrics_row["odo_scale_phase2_seed_scale_rel_to_base_init"] = float(ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED)
        metrics_row["odo_lever_phase2_seed_scale_rel_to_base_init"] = float(ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED)
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

    if output_truth_reference is None:
        raise RuntimeError("no case truth reference generated")

    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(output_truth_reference), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    ref_case_id = selected_specs[0].case_id
    phase1_end_time = float(case_metadata[ref_case_id]["phase1_end_time"])
    phase2_end_time = float(case_metadata[ref_case_id]["phase2_end_time"])
    gnss_off_windows = [(float(start), float(end)) for start, end in case_metadata[ref_case_id]["gnss_off_windows"]]

    plot_paths: dict[str, str] = {}
    all_states_path = args.plot_dir / "all_states_overview.png"
    plot_state_grid(
        case_frames,
        selected_specs,
        plot_config.overview_states,
        all_states_path,
        "Staged G5 odo_lever process-noise sweep: PVA errors + calibration states",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        plot_modes_by_key=plot_config.plot_modes_by_key,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["all_states_overview"] = rel_from_root(all_states_path, REPO_ROOT)

    key_states_path = args.plot_dir / "key_coupling_states.png"
    plot_state_grid(
        case_frames,
        selected_specs,
        KEY_COUPLING_STATES,
        key_states_path,
        "Staged G5 odo_lever process-noise sweep: key coupling states",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["key_coupling_states"] = rel_from_root(key_states_path, REPO_ROOT)

    for group_spec in plot_config.group_specs:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        plot_mode = "error" if group_spec.group_id in {"position", "velocity", "attitude"} else "state"
        selected_group: GroupSpec = group_spec
        if plot_mode == "error":
            selected_group = next(item for item in PVA_ERROR_GROUP_SPECS if item.group_id == group_spec.group_id)
        plot_state_grid(
            case_frames,
            selected_specs,
            selected_group.states,
            group_path,
            f"Staged G5 odo_lever process-noise sweep: {selected_group.title}",
            phase1_end_time,
            phase2_end_time,
            gnss_off_windows,
            plot_mode=plot_mode,
            truth_keys_to_hide=plot_config.truth_keys_to_hide,
        )
        plot_paths[group_spec.group_id] = rel_from_root(group_path, REPO_ROOT)

    vehicle_velocity_error_path = args.plot_dir / "vehicle_velocity_error.png"
    plot_state_grid(
        case_frames,
        selected_specs,
        plot_config.vehicle_velocity_error_states,
        vehicle_velocity_error_path,
        "Staged G5 odo_lever process-noise sweep: vehicle-frame velocity error",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_paths["vehicle_velocity_error"] = rel_from_root(vehicle_velocity_error_path, REPO_ROOT)

    vehicle_heading_error_path = args.plot_dir / "vehicle_heading_error.png"
    plot_state_grid(
        case_frames,
        selected_specs,
        plot_config.vehicle_heading_error_states,
        vehicle_heading_error_path,
        "Staged G5 odo_lever process-noise sweep: vehicle heading error",
        phase1_end_time,
        phase2_end_time,
        gnss_off_windows,
    )
    plot_paths["vehicle_heading_error"] = rel_from_root(vehicle_heading_error_path, REPO_ROOT)

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
        "experiment_controls": {
            "disable_gyro_scale": True,
            "disable_accel_scale": True,
            "odo_nhc_r_scale_rel_to_g5": G5_ODO_NHC_R_SCALE_REL_TO_G5,
            "mounting_q_scale_rel_to_staged_6x": float(MOUNTING_Q_SCALE_REL_TO_STAGED_6X_FIXED),
            "mounting_init_zero_deg": [0.0, 0.0, 0.0],
            "mounting_init_std_deg": MOUNTING_INIT_STD_DEG,
            "phase2_cov_seed_duration_s": float(args.phase2_cov_seed_duration),
            "phase2_mounting_cov_seed_floor_deg": float(MOUNTING_INIT_STD_DEG),
            "fixed_odo_scale_seed_scale_rel_to_base_init": float(ODO_SCALE_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED),
            "fixed_odo_scale_seed_var_2x": float(base_cfg["fusion"]["init"]["std_odo_scale"] * 2.0) ** 2,
            "fixed_odo_lever_seed_scale_rel_to_base_init": float(ODO_LEVER_PHASE2_SEED_SCALE_REL_TO_BASE_INIT_FIXED),
            "fixed_odo_lever_seed_var_4x": [float(value * 4.0) ** 2 for value in base_cfg["fusion"]["init"]["std_lever_arm"]],
            "base_odo_lever_q_staged_g5": float(base_cfg["fusion"]["noise"]["sigma_lever_arm"] * G5_EXTRINSIC_NOISE_SCALE),
        },
    }
    write_summary(summary_path, manifest, case_metrics_df, phase_metrics_df, plot_paths)
    manifest["freshness"]["summary_md_mtime"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
