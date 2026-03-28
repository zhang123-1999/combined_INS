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
    build_truth_interp,
    load_pos_dataframe,
    merge_case_outputs,
    llh_deg_to_ecef,
    rot_ned_to_ecef,
    wrap_deg,
)


EXP_ID_DEFAULT = "EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_fullwindow_attitude_bias_coupling_r1_20260325")
BASE_CONFIG_DEFAULT = Path("config_data2_research_seed_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
EXTRINSIC_NOISE_SCALE = 0.01
MEASUREMENT_NOISE_SCALE = 5.0


@dataclass(frozen=True)
class StateSpec:
    key: str
    label: str
    unit: str
    truth_state_key: str | None
    truth_column: str | None


@dataclass(frozen=True)
class GroupSpec:
    group_id: str
    title: str
    states: tuple[StateSpec, ...]


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    description: str
    enable_odo: bool
    enable_nhc: bool
    truth_init_mounting: bool = False
    truth_init_odo_lever: bool = False
    truth_init_gnss_lever: bool = False
    freeze_mounting: bool = False
    freeze_odo_lever: bool = False
    freeze_gnss_lever: bool = False
    freeze_odo_scale: bool = False
    disable_gyro_scale: bool = False
    disable_accel_scale: bool = False
    extrinsic_noise_scale: float | None = None
    measurement_noise_scale: float = 1.0


GROUP_SPECS: tuple[GroupSpec, ...] = (
    GroupSpec(
        "position",
        "Position",
        (
            StateSpec("p_n_m", "p_n", "m", "position_n_truth", "truth_p_n_m"),
            StateSpec("p_e_m", "p_e", "m", "position_e_truth", "truth_p_e_m"),
            StateSpec("p_u_m", "p_u", "m", "position_u_truth", "truth_p_u_m"),
        ),
    ),
    GroupSpec(
        "velocity",
        "Velocity",
        (
            StateSpec("v_n_mps", "v_n", "m/s", "velocity_n_truth", "truth_v_n_mps"),
            StateSpec("v_e_mps", "v_e", "m/s", "velocity_e_truth", "truth_v_e_mps"),
            StateSpec("v_u_mps", "v_u", "m/s", "velocity_u_truth", "truth_v_u_mps"),
        ),
    ),
    GroupSpec(
        "attitude",
        "Attitude",
        (
            StateSpec("roll_deg", "roll", "deg", "attitude_roll_truth", "truth_roll_deg"),
            StateSpec("pitch_deg", "pitch", "deg", "attitude_pitch_truth", "truth_pitch_deg"),
            StateSpec("yaw_deg", "yaw", "deg", "attitude_yaw_truth", "truth_yaw_deg"),
        ),
    ),
    GroupSpec(
        "ba",
        "Accel Bias",
        (
            StateSpec("ba_x_mgal", "ba_x", "mGal", "ba_x", "truth_ba_x_mgal"),
            StateSpec("ba_y_mgal", "ba_y", "mGal", "ba_y", "truth_ba_y_mgal"),
            StateSpec("ba_z_mgal", "ba_z", "mGal", "ba_z", "truth_ba_z_mgal"),
        ),
    ),
    GroupSpec(
        "bg",
        "Gyro Bias",
        (
            StateSpec("bg_x_degh", "bg_x", "deg/h", "bg_x", "truth_bg_x_degh"),
            StateSpec("bg_y_degh", "bg_y", "deg/h", "bg_y", "truth_bg_y_degh"),
            StateSpec("bg_z_degh", "bg_z", "deg/h", "bg_z", "truth_bg_z_degh"),
        ),
    ),
    GroupSpec(
        "sg",
        "Gyro Scale",
        (
            StateSpec("sg_x_ppm", "sg_x", "ppm", "sg_x", "truth_sg_x_ppm"),
            StateSpec("sg_y_ppm", "sg_y", "ppm", "sg_y", "truth_sg_y_ppm"),
            StateSpec("sg_z_ppm", "sg_z", "ppm", "sg_z", "truth_sg_z_ppm"),
        ),
    ),
    GroupSpec(
        "sa",
        "Accel Scale",
        (
            StateSpec("sa_x_ppm", "sa_x", "ppm", "sa_x", "truth_sa_x_ppm"),
            StateSpec("sa_y_ppm", "sa_y", "ppm", "sa_y", "truth_sa_y_ppm"),
            StateSpec("sa_z_ppm", "sa_z", "ppm", "sa_z", "truth_sa_z_ppm"),
        ),
    ),
    GroupSpec(
        "odo_scale",
        "ODO Scale",
        (StateSpec("odo_scale_state", "odo_scale", "1", "odo_scale", "truth_odo_scale_state"),),
    ),
    GroupSpec(
        "mounting",
        "Mounting (Total)",
        (
            StateSpec("mounting_roll_deg", "mounting_roll_total", "deg", "mounting_roll", "truth_mounting_roll_deg"),
            StateSpec("mounting_pitch_deg", "mounting_pitch_total", "deg", "mounting_pitch", "truth_mounting_pitch_deg"),
            StateSpec("mounting_yaw_deg", "mounting_yaw_total", "deg", "mounting_yaw", "truth_mounting_yaw_deg"),
        ),
    ),
    GroupSpec(
        "odo_lever",
        "ODO Lever",
        (
            StateSpec("odo_lever_x_m", "odo_lever_x", "m", "odo_lever_x", "truth_odo_lever_x_m"),
            StateSpec("odo_lever_y_m", "odo_lever_y", "m", "odo_lever_y", "truth_odo_lever_y_m"),
            StateSpec("odo_lever_z_m", "odo_lever_z", "m", "odo_lever_z", "truth_odo_lever_z_m"),
        ),
    ),
    GroupSpec(
        "gnss_lever",
        "GNSS Lever",
        (
            StateSpec("gnss_lever_x_m", "gnss_lever_x", "m", "gnss_lever_x", "truth_gnss_lever_x_m"),
            StateSpec("gnss_lever_y_m", "gnss_lever_y", "m", "gnss_lever_y", "truth_gnss_lever_y_m"),
            StateSpec("gnss_lever_z_m", "gnss_lever_z", "m", "gnss_lever_z", "truth_gnss_lever_z_m"),
        ),
    ),
)
ALL_STATE_SPECS: tuple[StateSpec, ...] = tuple(state for group in GROUP_SPECS for state in group.states)

KEY_COUPLING_STATES: tuple[StateSpec, ...] = (
    GROUP_SPECS[2].states[0],
    GROUP_SPECS[2].states[1],
    GROUP_SPECS[2].states[2],
    GROUP_SPECS[4].states[0],
    GROUP_SPECS[4].states[1],
    GROUP_SPECS[4].states[2],
    GROUP_SPECS[3].states[0],
    GROUP_SPECS[3].states[1],
    GROUP_SPECS[3].states[2],
    GROUP_SPECS[8].states[2],
    GROUP_SPECS[9].states[1],
    GROUP_SPECS[10].states[1],
)

CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        case_id="group1_ins_gnss_truth_gnss_lever_fixed",
        label="G1 INS/GNSS truth GNSS lever fixed",
        color="#4c78a8",
        description="INS/GNSS control with GNSS lever set to truth and fixed.",
        enable_odo=False,
        enable_nhc=False,
        truth_init_gnss_lever=True,
        freeze_gnss_lever=True,
    ),
    CaseSpec(
        case_id="group2_ins_gnss_odo_nhc_truth_extrinsics_fixed",
        label="G2 truth extrinsics fixed",
        color="#f58518",
        description="INS/GNSS/ODO/NHC with truth mounting and both levers fixed.",
        enable_odo=True,
        enable_nhc=True,
        truth_init_mounting=True,
        truth_init_odo_lever=True,
        truth_init_gnss_lever=True,
        freeze_mounting=True,
        freeze_odo_lever=True,
        freeze_gnss_lever=True,
        freeze_odo_scale=True,
    ),
    CaseSpec(
        case_id="group3_ins_gnss_odo_nhc_truth_extrinsics_soft",
        label="G3 truth extrinsics soft",
        color="#54a24b",
        description="INS/GNSS/ODO/NHC with truth-init extrinsics released under 0.01x process noise.",
        enable_odo=True,
        enable_nhc=True,
        truth_init_mounting=True,
        truth_init_odo_lever=True,
        truth_init_gnss_lever=True,
        freeze_odo_scale=True,
        extrinsic_noise_scale=EXTRINSIC_NOISE_SCALE,
    ),
    CaseSpec(
        case_id="group4_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise",
        label="G4 truth extrinsics soft + high ODO/NHC noise",
        color="#e45756",
        description="Group3 plus 5x ODO/NHC measurement noise.",
        enable_odo=True,
        enable_nhc=True,
        truth_init_mounting=True,
        truth_init_odo_lever=True,
        truth_init_gnss_lever=True,
        freeze_odo_scale=True,
        extrinsic_noise_scale=EXTRINSIC_NOISE_SCALE,
        measurement_noise_scale=MEASUREMENT_NOISE_SCALE,
    ),
    CaseSpec(
        case_id="group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale",
        label="G5 G4 + disable sg/sa",
        color="#72b7b2",
        description="Group4 plus gyro/accel scale-factor states ablated.",
        enable_odo=True,
        enable_nhc=True,
        truth_init_mounting=True,
        truth_init_odo_lever=True,
        truth_init_gnss_lever=True,
        freeze_odo_scale=True,
        disable_gyro_scale=True,
        disable_accel_scale=True,
        extrinsic_noise_scale=EXTRINSIC_NOISE_SCALE,
        measurement_noise_scale=MEASUREMENT_NOISE_SCALE,
    ),
)
CASE_MAP = {spec.case_id: spec for spec in CASE_SPECS}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run full-window attitude/bias coupling experiments for INS/GNSS/ODO/NHC on data2."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--cases", nargs="*", default=None)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    return args


def render_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float, np.floating)):
        value = float(value)
        if not math.isfinite(value):
            return "NA"
        return f"{value:.6f}"
    return str(value)


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


def state_reference_value(truth_reference: dict[str, Any], state_key: str) -> float:
    return float(truth_reference["states"][state_key]["reference_value"])


def truth_vector(truth_reference: dict[str, Any], prefix: str) -> list[float]:
    return [
        state_reference_value(truth_reference, f"{prefix}_x"),
        state_reference_value(truth_reference, f"{prefix}_y"),
        state_reference_value(truth_reference, f"{prefix}_z"),
    ]


def scale_noise_fields(noise_cfg: dict[str, Any], base_noise: dict[str, Any], keys: list[str], scale: float) -> None:
    for key in keys:
        if key in base_noise:
            noise_cfg[key] = float(base_noise[key]) * scale


def scale_optional_vector_noise(noise_cfg: dict[str, Any], base_noise: dict[str, Any], key: str, scale: float) -> None:
    if key in base_noise:
        noise_cfg[key] = [float(x) * scale for x in base_noise[key]]


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
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
    fusion["gnss_schedule"] = {"enabled": False}
    fusion.pop("runtime_phases", None)
    fusion.pop("post_gnss_ablation", None)

    fej_cfg["enable"] = False
    constraints_cfg["enable_odo"] = bool(spec.enable_odo)
    constraints_cfg["enable_nhc"] = bool(spec.enable_nhc)
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

    ablation_cfg["disable_mounting"] = bool(spec.freeze_mounting)
    ablation_cfg["disable_mounting_roll"] = False
    ablation_cfg["disable_odo_lever_arm"] = bool(spec.freeze_odo_lever)
    ablation_cfg["disable_gnss_lever_arm"] = bool(spec.freeze_gnss_lever)
    ablation_cfg["disable_gnss_lever_z"] = False
    ablation_cfg["disable_odo_scale"] = bool(spec.freeze_odo_scale)
    ablation_cfg["disable_gyro_scale"] = bool(spec.disable_gyro_scale)
    ablation_cfg["disable_accel_scale"] = bool(spec.disable_accel_scale)

    metadata: dict[str, Any] = {
        "description": spec.description,
        "truth_initialized": [],
        "frozen_states": [],
        "noise_scales": {},
    }

    if spec.truth_init_mounting:
        init_cfg["mounting_roll0"] = state_reference_value(truth_reference, "mounting_roll")
        init_cfg["mounting_pitch0"] = state_reference_value(truth_reference, "mounting_pitch")
        init_cfg["mounting_yaw0"] = state_reference_value(truth_reference, "mounting_yaw")
        metadata["truth_initialized"].append("mounting(22-24)")
    if spec.truth_init_odo_lever:
        odo_lever_truth = [float(x) for x in truth_reference["sources"]["odo_lever_truth"]["value_m"]]
        init_cfg["lever_arm0"] = odo_lever_truth
        constraints_cfg["odo_lever_arm"] = odo_lever_truth
        metadata["truth_initialized"].append("lever_odo(25-27)")
    if spec.truth_init_gnss_lever:
        gnss_lever_truth = [float(x) for x in truth_reference["sources"]["gnss_lever_truth"]["value_m"]]
        init_cfg["gnss_lever_arm0"] = gnss_lever_truth
        metadata["truth_initialized"].append("lever_gnss(28-30)")

    if spec.freeze_mounting:
        metadata["frozen_states"].append("mounting(22-24)")
    if spec.freeze_odo_lever:
        metadata["frozen_states"].append("lever_odo(25-27)")
    if spec.freeze_gnss_lever:
        metadata["frozen_states"].append("lever_gnss(28-30)")
    if spec.freeze_odo_scale:
        init_cfg["odo_scale"] = 1.0
        metadata["frozen_states"].append("odo_scale(21)")
    if spec.disable_gyro_scale:
        metadata["frozen_states"].append("sg(15-17)")
    if spec.disable_accel_scale:
        metadata["frozen_states"].append("sa(18-20)")

    if spec.extrinsic_noise_scale is not None:
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
            spec.extrinsic_noise_scale,
        )
        scale_optional_vector_noise(noise_cfg, base_noise, "sigma_lever_arm_vec", spec.extrinsic_noise_scale)
        scale_optional_vector_noise(noise_cfg, base_noise, "sigma_gnss_lever_arm_vec", spec.extrinsic_noise_scale)
        metadata["noise_scales"]["extrinsics_q_scale"] = spec.extrinsic_noise_scale

    if spec.measurement_noise_scale != 1.0:
        constraints_cfg["sigma_odo"] = float(base_constraints["sigma_odo"]) * spec.measurement_noise_scale
        constraints_cfg["sigma_nhc_y"] = float(base_constraints["sigma_nhc_y"]) * spec.measurement_noise_scale
        constraints_cfg["sigma_nhc_z"] = float(base_constraints["sigma_nhc_z"]) * spec.measurement_noise_scale
        metadata["noise_scales"]["odo_nhc_r_scale"] = spec.measurement_noise_scale

    metadata["config_path"] = rel_from_root(case_dir / f"config_{spec.case_id}.yaml", REPO_ROOT)
    return cfg, metadata


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
) -> tuple[Path, dict[str, Any]]:
    cfg, metadata = build_case_config(base_cfg, truth_reference, case_dir, spec)
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    metadata["config_mtime"] = mtime_text(cfg_path)
    return cfg_path, metadata


def build_state_frame(
    merged_df: pd.DataFrame,
    truth_interp_df: pd.DataFrame,
    truth_reference: dict[str, Any],
) -> pd.DataFrame:
    truth_xyz = llh_deg_to_ecef(
        truth_interp_df["lat"].to_numpy(dtype=float),
        truth_interp_df["lon"].to_numpy(dtype=float),
        truth_interp_df["h"].to_numpy(dtype=float),
    )
    est_xyz = merged_df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float)
    origin_xyz = truth_xyz[0]
    lat0 = np.deg2rad(np.array([truth_interp_df["lat"].iloc[0]], dtype=float))
    lon0 = np.deg2rad(np.array([truth_interp_df["lon"].iloc[0]], dtype=float))
    rot_ne0 = rot_ned_to_ecef(lat0, lon0)[0]
    rot_en0 = rot_ne0.T

    est_ned = (rot_en0 @ (est_xyz - origin_xyz).T).T
    truth_ned = (rot_en0 @ (truth_xyz - origin_xyz).T).T

    lat_rad = np.deg2rad(truth_interp_df["lat"].to_numpy(dtype=float))
    lon_rad = np.deg2rad(truth_interp_df["lon"].to_numpy(dtype=float))
    rot_ne = rot_ned_to_ecef(lat_rad, lon_rad)

    est_v_ecef = merged_df[["fused_vx", "fused_vy", "fused_vz"]].to_numpy(dtype=float)
    est_v_ned = np.einsum("nji,nj->ni", rot_ne, est_v_ecef)
    truth_v_ned = truth_interp_df[["vn", "ve", "vd"]].to_numpy(dtype=float)

    frame = pd.DataFrame({"timestamp": merged_df["timestamp"].to_numpy(dtype=float)})
    frame["p_n_m"] = est_ned[:, 0]
    frame["p_e_m"] = est_ned[:, 1]
    frame["p_u_m"] = -est_ned[:, 2]
    frame["truth_p_n_m"] = truth_ned[:, 0]
    frame["truth_p_e_m"] = truth_ned[:, 1]
    frame["truth_p_u_m"] = -truth_ned[:, 2]

    frame["v_n_mps"] = est_v_ned[:, 0]
    frame["v_e_mps"] = est_v_ned[:, 1]
    frame["v_u_mps"] = -est_v_ned[:, 2]
    frame["truth_v_n_mps"] = truth_v_ned[:, 0]
    frame["truth_v_e_mps"] = truth_v_ned[:, 1]
    frame["truth_v_u_mps"] = -truth_v_ned[:, 2]

    frame["roll_deg"] = merged_df["fused_roll"].to_numpy(dtype=float)
    frame["pitch_deg"] = merged_df["fused_pitch"].to_numpy(dtype=float)
    frame["yaw_deg"] = merged_df["fused_yaw"].to_numpy(dtype=float)
    frame["truth_roll_deg"] = truth_interp_df["roll"].to_numpy(dtype=float)
    frame["truth_pitch_deg"] = truth_interp_df["pitch"].to_numpy(dtype=float)
    frame["truth_yaw_deg"] = truth_interp_df["yaw"].to_numpy(dtype=float)

    mounting_total_truth = truth_reference["sources"]["mounting_total_truth"]["value_deg"]
    frame["mounting_state_roll_deg"] = merged_df["mounting_roll_deg"].to_numpy(dtype=float)
    frame["mounting_state_pitch_deg"] = merged_df["mounting_pitch_deg"].to_numpy(dtype=float)
    frame["mounting_state_yaw_deg"] = merged_df["mounting_yaw_deg"].to_numpy(dtype=float)
    frame["truth_mounting_state_roll_deg"] = state_reference_value(truth_reference, "mounting_roll")
    frame["truth_mounting_state_pitch_deg"] = state_reference_value(truth_reference, "mounting_pitch")
    frame["truth_mounting_state_yaw_deg"] = state_reference_value(truth_reference, "mounting_yaw")

    scalar_truth_map = {
        "ba_x_mgal": state_reference_value(truth_reference, "ba_x"),
        "ba_y_mgal": state_reference_value(truth_reference, "ba_y"),
        "ba_z_mgal": state_reference_value(truth_reference, "ba_z"),
        "bg_x_degh": state_reference_value(truth_reference, "bg_x"),
        "bg_y_degh": state_reference_value(truth_reference, "bg_y"),
        "bg_z_degh": state_reference_value(truth_reference, "bg_z"),
        "sg_x_ppm": state_reference_value(truth_reference, "sg_x"),
        "sg_y_ppm": state_reference_value(truth_reference, "sg_y"),
        "sg_z_ppm": state_reference_value(truth_reference, "sg_z"),
        "sa_x_ppm": state_reference_value(truth_reference, "sa_x"),
        "sa_y_ppm": state_reference_value(truth_reference, "sa_y"),
        "sa_z_ppm": state_reference_value(truth_reference, "sa_z"),
        "odo_scale_state": state_reference_value(truth_reference, "odo_scale"),
        "mounting_roll_deg": float(mounting_total_truth["roll"]),
        "mounting_pitch_deg": float(mounting_total_truth["pitch"]),
        "mounting_yaw_deg": float(mounting_total_truth["yaw"]),
        "odo_lever_x_m": state_reference_value(truth_reference, "odo_lever_x"),
        "odo_lever_y_m": state_reference_value(truth_reference, "odo_lever_y"),
        "odo_lever_z_m": state_reference_value(truth_reference, "odo_lever_z"),
        "gnss_lever_x_m": state_reference_value(truth_reference, "gnss_lever_x"),
        "gnss_lever_y_m": state_reference_value(truth_reference, "gnss_lever_y"),
        "gnss_lever_z_m": state_reference_value(truth_reference, "gnss_lever_z"),
    }
    for col, truth_value in scalar_truth_map.items():
        if col == "mounting_roll_deg":
            frame[col] = merged_df["total_mounting_roll_deg"].to_numpy(dtype=float)
        elif col == "mounting_pitch_deg":
            frame[col] = merged_df["total_mounting_pitch_deg"].to_numpy(dtype=float)
        elif col == "mounting_yaw_deg":
            frame[col] = merged_df["total_mounting_yaw_deg"].to_numpy(dtype=float)
        else:
            frame[col] = merged_df[col].to_numpy(dtype=float)
        frame[f"truth_{col}"] = truth_value
    return frame


def compute_case_metrics(case_row: dict[str, Any], state_frame: pd.DataFrame) -> dict[str, Any]:
    metrics = dict(case_row)
    roll_err = wrap_deg(state_frame["roll_deg"].to_numpy(dtype=float) - state_frame["truth_roll_deg"].to_numpy(dtype=float))
    pitch_err = wrap_deg(
        state_frame["pitch_deg"].to_numpy(dtype=float) - state_frame["truth_pitch_deg"].to_numpy(dtype=float)
    )
    yaw_err = wrap_deg(state_frame["yaw_deg"].to_numpy(dtype=float) - state_frame["truth_yaw_deg"].to_numpy(dtype=float))
    metrics["roll_err_max_abs_deg"] = float(np.max(np.abs(roll_err)))
    metrics["pitch_err_max_abs_deg"] = float(np.max(np.abs(pitch_err)))
    metrics["yaw_err_max_abs_deg"] = float(np.max(np.abs(yaw_err)))
    for key in ["ba_x_mgal", "ba_y_mgal", "ba_z_mgal", "bg_x_degh", "bg_y_degh", "bg_z_degh"]:
        diff = state_frame[key].to_numpy(dtype=float) - state_frame[f"truth_{key}"].to_numpy(dtype=float)
        metrics[f"{key}_err_max_abs"] = float(np.max(np.abs(diff)))
    metrics["odo_accept_ratio"] = metric_value(pd.Series(case_row), "odo_accept_ratio")
    metrics["nhc_accept_ratio"] = metric_value(pd.Series(case_row), "nhc_accept_ratio")
    return metrics


def plot_state_grid(
    case_frames: dict[str, pd.DataFrame],
    case_specs: list[CaseSpec],
    state_specs: tuple[StateSpec, ...],
    output_path: Path,
    title: str,
) -> None:
    n_cols = 3
    n_rows = math.ceil(len(state_specs) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 2.9 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, state_spec in enumerate(state_specs):
        ax = axes_flat[idx]
        truth_plotted = False
        for case_spec in case_specs:
            df = case_frames[case_spec.case_id]
            t = df["timestamp"].to_numpy(dtype=float)
            values = df[state_spec.key].to_numpy(dtype=float)
            t_plot, v_plot = downsample_for_plot(t, values)
            ax.plot(t_plot, v_plot, linewidth=0.95, color=case_spec.color, label=case_spec.label)
            if state_spec.truth_column and not truth_plotted:
                truth_values = df[state_spec.truth_column].to_numpy(dtype=float)
                tt_plot, tv_plot = downsample_for_plot(t, truth_values)
                ax.plot(tt_plot, tv_plot, linewidth=1.0, color="black", linestyle="--", label="truth")
                truth_plotted = True
        ax.set_title(f"{state_spec.label} [{state_spec.unit}]", fontsize=9)
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


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    rows = [
        [
            str(row["case_id"]),
            format_metric(row.get("overall_rmse_3d_m_aux")),
            format_metric(row.get("overall_p95_3d_m_aux")),
            format_metric(row.get("overall_final_err_3d_m_aux")),
            format_metric(row.get("yaw_err_max_abs_deg")),
            format_metric(row.get("bg_z_degh_err_max_abs")),
            format_metric(row.get("odo_accept_ratio")),
            format_metric(row.get("nhc_accept_ratio")),
        ]
        for _, row in case_metrics_df.iterrows()
    ]
    lines = [
        "# data2 full-window attitude/bias coupling summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- generated_at: `{manifest['generated_at']}`",
        "",
        "## Case Metrics",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "rmse3d_m",
                "p95_3d_m",
                "final_3d_m",
                "yaw_err_max_abs_deg",
                "bg_z_err_max_abs_degh",
                "odo_accept_ratio",
                "nhc_accept_ratio",
            ],
            rows,
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
    case_metadata: dict[str, dict[str, Any]] = {}

    for spec in selected_specs:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, metadata = write_case_config(base_cfg, truth_reference, case_dir, spec)
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

        case_rows.append(compute_case_metrics(case_row, state_frame))
        case_frames[spec.case_id] = state_frame

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    plot_paths: dict[str, str] = {}
    all_states_path = args.plot_dir / "all_states_overview.png"
    plot_state_grid(case_frames, selected_specs, ALL_STATE_SPECS, all_states_path, "All 31 states with truth")
    plot_paths["all_states_overview"] = rel_from_root(all_states_path, REPO_ROOT)

    key_states_path = args.plot_dir / "key_coupling_states.png"
    plot_state_grid(case_frames, selected_specs, KEY_COUPLING_STATES, key_states_path, "Key coupling states")
    plot_paths["key_coupling_states"] = rel_from_root(key_states_path, REPO_ROOT)

    for group_spec in GROUP_SPECS:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        plot_state_grid(case_frames, selected_specs, group_spec.states, group_path, group_spec.title)
        plot_paths[group_spec.group_id] = rel_from_root(group_path, REPO_ROOT)

    manifest = {
        "exp_id": args.exp_id,
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "solver": rel_from_root(args.exe, REPO_ROOT),
        "cases": [spec.case_id for spec in selected_specs],
        "case_metadata": json_safe(case_metadata),
        "truth_reference_path": rel_from_root(truth_reference_path, REPO_ROOT),
        "plot_paths": plot_paths,
        "case_metrics_path": rel_from_root(case_metrics_path, REPO_ROOT),
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")

    summary_path = args.output_dir / "summary.md"
    write_summary(summary_path, manifest, case_metrics_df, plot_paths)


if __name__ == "__main__":
    main()
