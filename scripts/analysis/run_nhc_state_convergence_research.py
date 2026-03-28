from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import re
import shutil
import sys
from dataclasses import dataclass, field
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


EXP_ID = "EXP-20260323-data2-nhc-state-convergence-research-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_nhc_state_convergence_research_r1_20260323")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
JUMP_WINDOW_START = 528450.0
JUMP_WINDOW_END = 528580.0
RESTART_DISABLE_START = 528548.0
RESTART_DISABLE_END = 528558.0
MECH_FINALTIME = 528560.0
MECH_PLOT_START = 528520.0
TAIL_SECONDS = 120.0
KEY_STATES_FOR_JUMP = (
    ("yaw_err_deg", 1.0),
    ("mounting_yaw_err_deg", 0.2),
    ("gnss_lever_y_err_m", 0.2),
    ("ba_x_mgal", 100.0),
    ("bg_z_degh", 50.0),
)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
OMEGA_IE = 7.292115e-5
ODO_SCALE_ZERO_EPS = 1.0e-6
MOUNTING_SIGMA_SCALE = 0.1


@dataclass(frozen=True)
class PlotColumn:
    key: str
    label: str
    unit: str
    kind: str


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    family: str
    color: str
    description: str
    enable_runtime_anchor: bool = False
    enable_fej: bool = False
    overrides: tuple[tuple[str, Any], ...] = ()
    mechanism_case: bool = False


@dataclass(frozen=True)
class FamilySpec:
    family_id: str
    title: str
    case_ids: tuple[str, ...]
    focus: str


NAV_ERROR_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("p_n_err_m", "p_n error", "m", "error"),
    PlotColumn("p_e_err_m", "p_e error", "m", "error"),
    PlotColumn("p_u_err_m", "p_u error", "m", "error"),
    PlotColumn("v_n_err_mps", "v_n error", "m/s", "error"),
    PlotColumn("v_e_err_mps", "v_e error", "m/s", "error"),
    PlotColumn("v_u_err_mps", "v_u error", "m/s", "error"),
    PlotColumn("roll_err_deg", "roll error", "deg", "error"),
    PlotColumn("pitch_err_deg", "pitch error", "deg", "error"),
    PlotColumn("yaw_err_deg", "yaw error", "deg", "error"),
)

STATIC_ERROR_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("odo_scale_err", "odo_scale error", "1", "error"),
    PlotColumn("mounting_roll_err_deg", "mounting_roll error", "deg", "error"),
    PlotColumn("mounting_pitch_err_deg", "mounting_pitch error", "deg", "error"),
    PlotColumn("mounting_yaw_err_deg", "mounting_yaw error", "deg", "error"),
    PlotColumn("odo_lever_x_err_m", "odo_lever_x error", "m", "error"),
    PlotColumn("odo_lever_y_err_m", "odo_lever_y error", "m", "error"),
    PlotColumn("odo_lever_z_err_m", "odo_lever_z error", "m", "error"),
    PlotColumn("gnss_lever_x_err_m", "gnss_lever_x error", "m", "error"),
    PlotColumn("gnss_lever_y_err_m", "gnss_lever_y error", "m", "error"),
    PlotColumn("gnss_lever_z_err_m", "gnss_lever_z error", "m", "error"),
)

NO_TRUTH_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("ba_x_mgal", "ba_x", "mGal", "state"),
    PlotColumn("ba_y_mgal", "ba_y", "mGal", "state"),
    PlotColumn("ba_z_mgal", "ba_z", "mGal", "state"),
    PlotColumn("bg_x_degh", "bg_x", "deg/h", "state"),
    PlotColumn("bg_y_degh", "bg_y", "deg/h", "state"),
    PlotColumn("bg_z_degh", "bg_z", "deg/h", "state"),
    PlotColumn("sg_x_ppm", "sg_x", "ppm", "state"),
    PlotColumn("sg_y_ppm", "sg_y", "ppm", "state"),
    PlotColumn("sg_z_ppm", "sg_z", "ppm", "state"),
    PlotColumn("sa_x_ppm", "sa_x", "ppm", "state"),
    PlotColumn("sa_y_ppm", "sa_y", "ppm", "state"),
    PlotColumn("sa_z_ppm", "sa_z", "ppm", "state"),
)

ALL_OVERVIEW_COLUMNS: tuple[PlotColumn, ...] = NAV_ERROR_COLUMNS + STATIC_ERROR_COLUMNS + NO_TRUTH_COLUMNS
VELOCITY_COLUMNS: tuple[PlotColumn, ...] = (
    PlotColumn("v_v_x_mps", "v_v.x", "m/s", "velocity"),
    PlotColumn("v_v_y_mps", "v_v.y", "m/s", "velocity"),
    PlotColumn("v_v_z_mps", "v_v.z", "m/s", "velocity"),
)


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        "baseline_free_run",
        "baseline free-run",
        "P0_redraw",
        "#4c78a8",
        "ESKF free-run baseline with zero-init non-PVA states and 0.1x mounting process noise.",
    ),
    CaseSpec(
        "eskf_pva_truth_anchor",
        "ESKF PVA anchor",
        "P0_redraw",
        "#f58518",
        "Continuous runtime P/V/A truth anchor reference case.",
        enable_runtime_anchor=True,
    ),
    CaseSpec(
        "nhc_off_below_5p0",
        "NHC off <5.0 m/s",
        "P0_redraw",
        "#e45756",
        "Negative-result blunt low-speed gate.",
        overrides=(("fusion.constraints.nhc_disable_below_forward_speed", 5.0),),
    ),
    CaseSpec(
        "nhc_window_off",
        "NHC window off",
        "P0_window_disable",
        "#72b7b2",
        "Disable NHC only inside the restart jump window.",
        overrides=(
            ("fusion.constraints.debug_nhc_disable_start_time", RESTART_DISABLE_START),
            ("fusion.constraints.debug_nhc_disable_end_time", RESTART_DISABLE_END),
        ),
    ),
    CaseSpec(
        "odo_window_off",
        "ODO window off",
        "P0_window_disable",
        "#54a24b",
        "Disable ODO only inside the restart jump window.",
        overrides=(
            ("fusion.constraints.debug_odo_disable_start_time", RESTART_DISABLE_START),
            ("fusion.constraints.debug_odo_disable_end_time", RESTART_DISABLE_END),
        ),
    ),
    CaseSpec(
        "gnss_pos_staged",
        "GNSS staged",
        "P0_gnss_lever",
        "#b279a2",
        "Use stage_nonpos_then_pos instead of joint GNSS position update.",
        overrides=(("fusion.gnss_pos_update_mode", "stage_nonpos_then_pos"),),
    ),
    CaseSpec(
        "gnss_lever_window_freeze",
        "GNSS lever freeze window",
        "P0_gnss_lever",
        "#ff9da6",
        "Freeze GNSS lever state only around the jump window.",
        overrides=(
            ("fusion.constraints.debug_gnss_lever_arm_disable_start_time", RESTART_DISABLE_START),
            ("fusion.constraints.debug_gnss_lever_arm_disable_end_time", RESTART_DISABLE_END),
        ),
    ),
    CaseSpec(
        "nhc_speed_0p5",
        "NHC off <0.5 m/s",
        "P1_speed_threshold",
        "#9d755d",
        "Narrow low-speed gate at 0.5 m/s.",
        overrides=(("fusion.constraints.nhc_disable_below_forward_speed", 0.5),),
    ),
    CaseSpec(
        "nhc_speed_1p0",
        "NHC off <1.0 m/s",
        "P1_speed_threshold",
        "#bab0ab",
        "Narrow low-speed gate at 1.0 m/s.",
        overrides=(("fusion.constraints.nhc_disable_below_forward_speed", 1.0),),
    ),
    CaseSpec(
        "nhc_speed_1p5",
        "NHC off <1.5 m/s",
        "P1_speed_threshold",
        "#2f4b7c",
        "Narrow low-speed gate at 1.5 m/s.",
        overrides=(("fusion.constraints.nhc_disable_below_forward_speed", 1.5),),
    ),
    CaseSpec(
        "nhc_speed_2p0",
        "NHC off <2.0 m/s",
        "P1_speed_threshold",
        "#665191",
        "Narrow low-speed gate at 2.0 m/s.",
        overrides=(("fusion.constraints.nhc_disable_below_forward_speed", 2.0),),
    ),
    CaseSpec(
        "nhc_weak_gate_default",
        "NHC weak-excitation gate",
        "P1_joint_gate",
        "#a05195",
        "Disable NHC whenever the existing weak-excitation thresholds are met.",
        overrides=(("fusion.constraints.disable_nhc_when_weak_excitation", True),),
    ),
    CaseSpec(
        "nhc_weak_gate_strict",
        "NHC+ODO strict weak gate",
        "P1_joint_gate",
        "#d45087",
        "Disable NHC/ODO under stricter weak-excitation thresholds.",
        overrides=(
            ("fusion.constraints.disable_nhc_when_weak_excitation", True),
            ("fusion.constraints.disable_odo_when_weak_excitation", True),
            ("fusion.constraints.excitation_min_speed", 2.0),
            ("fusion.constraints.excitation_min_yaw_rate", 0.05),
            ("fusion.constraints.excitation_min_lateral_acc", 0.5),
        ),
    ),
    CaseSpec(
        "nhc_interval_0p05",
        "NHC dt>=0.05 s",
        "P1_joint_gate",
        "#f95d6a",
        "Reduce NHC update density with 0.05 s minimum interval.",
        overrides=(("fusion.constraints.nhc_min_update_interval", 0.05),),
    ),
    CaseSpec(
        "nhc_interval_0p10",
        "NHC dt>=0.10 s",
        "P1_joint_gate",
        "#ff7c43",
        "Reduce NHC update density with 0.10 s minimum interval.",
        overrides=(("fusion.constraints.nhc_min_update_interval", 0.10),),
    ),
    CaseSpec(
        "nhc_interval_0p20",
        "NHC dt>=0.20 s",
        "P1_joint_gate",
        "#ffa600",
        "Reduce NHC update density with 0.20 s minimum interval.",
        overrides=(("fusion.constraints.nhc_min_update_interval", 0.20),),
    ),
    CaseSpec(
        "gnss_pos_gain_0p5",
        "GNSS pos gain 0.5x",
        "P1_gnss_gain",
        "#003f5c",
        "Scale GNSS position-state gain to 0.5x.",
        overrides=(("fusion.gnss_pos_position_gain_scale", 0.5),),
    ),
    CaseSpec(
        "gnss_pos_gain_0p25",
        "GNSS pos gain 0.25x",
        "P1_gnss_gain",
        "#374c80",
        "Scale GNSS position-state gain to 0.25x.",
        overrides=(("fusion.gnss_pos_position_gain_scale", 0.25),),
    ),
    CaseSpec(
        "lgy_from_y_0p5",
        "lgy_from_y 0.5x",
        "P1_gnss_gain",
        "#7a5195",
        "Scale GNSS lever-y from measurement-y gain to 0.5x.",
        overrides=(("fusion.gnss_pos_lgy_from_y_gain_scale", 0.5),),
    ),
    CaseSpec(
        "lgy_from_y_0p0",
        "lgy_from_y 0.0x",
        "P1_gnss_gain",
        "#bc5090",
        "Remove GNSS lever-y from measurement-y correction path.",
        overrides=(("fusion.gnss_pos_lgy_from_y_gain_scale", 0.0),),
    ),
    CaseSpec(
        "odo_before_nhc",
        "ODO before NHC",
        "P1_order_freeze",
        "#ef5675",
        "Swap local order so ODO runs before NHC.",
        overrides=(("fusion.constraints.debug_run_odo_before_nhc", True),),
    ),
    CaseSpec(
        "freeze_mounting",
        "freeze mounting",
        "P1_order_freeze",
        "#ff764a",
        "Freeze all mounting state components.",
        overrides=(("fusion.ablation.disable_mounting", True),),
    ),
    CaseSpec(
        "freeze_mounting_yaw",
        "freeze mounting_yaw",
        "P1_order_freeze",
        "#ffa600",
        "Freeze mounting_yaw only.",
        overrides=(("fusion.ablation.disable_mounting_yaw", True),),
    ),
    CaseSpec(
        "freeze_gnss_lever",
        "freeze gnss lever",
        "P1_order_freeze",
        "#00a6fb",
        "Freeze GNSS lever state through ablation.",
        overrides=(("fusion.ablation.disable_gnss_lever_arm", True),),
    ),
    CaseSpec(
        "freeze_odo_scale",
        "freeze odo_scale",
        "P1_order_freeze",
        "#0582ca",
        "Freeze ODO scale state.",
        overrides=(("fusion.ablation.disable_odo_scale", True),),
    ),
    CaseSpec(
        "bgz_p0_30degh",
        "bg_z P0 30 deg/h",
        "P2_bgz",
        "#006494",
        "Tighten initial bg_z prior to 30 deg/h.",
    ),
    CaseSpec(
        "bgz_q_0p1x",
        "bg_z Q 0.1x",
        "P2_bgz",
        "#247ba0",
        "Reduce bg_z process noise to 0.1x on z axis.",
    ),
    CaseSpec(
        "bgz_p0_30degh_q_0p1x",
        "bg_z P0+Q",
        "P2_bgz",
        "#70c1b3",
        "Combine tight bg_z prior and reduced bg_z process noise.",
    ),
    CaseSpec(
        "bgz_observability_gate",
        "bg_z observability gate",
        "P2_bgz",
        "#b2dbbf",
        "Enable ODO/NHC to bg_z observability gate.",
        overrides=(("fusion.constraints.enable_bgz_observability_gate", True),),
    ),
    CaseSpec(
        "odo_time_offset_neg10ms",
        "ODO time -10 ms",
        "P2_time_alignment",
        "#f3ffbd",
        "Advance ODO measurement time by 10 ms.",
        overrides=(("fusion.constraints.odo_time_offset", -0.01),),
    ),
    CaseSpec(
        "odo_time_offset_pos10ms",
        "ODO time +10 ms",
        "P2_time_alignment",
        "#ffb703",
        "Delay ODO measurement time by 10 ms.",
        overrides=(("fusion.constraints.odo_time_offset", 0.01),),
    ),
    CaseSpec(
        "calibration_init",
        "calibration init",
        "P2_init_anchor_reset",
        "#8ecae6",
        "Initialize non-PVA states from calibration/truth proxy instead of zeros.",
    ),
    CaseSpec(
        "anchor_pos_only",
        "anchor pos only",
        "P2_init_anchor_reset",
        "#219ebc",
        "Runtime truth anchor on position only.",
        enable_runtime_anchor=True,
        overrides=(
            ("fusion.init.runtime_truth_anchor_position", True),
            ("fusion.init.runtime_truth_anchor_velocity", False),
            ("fusion.init.runtime_truth_anchor_attitude", False),
        ),
    ),
    CaseSpec(
        "anchor_vel_only",
        "anchor vel only",
        "P2_init_anchor_reset",
        "#023047",
        "Runtime truth anchor on velocity only.",
        enable_runtime_anchor=True,
        overrides=(
            ("fusion.init.runtime_truth_anchor_position", False),
            ("fusion.init.runtime_truth_anchor_velocity", True),
            ("fusion.init.runtime_truth_anchor_attitude", False),
        ),
    ),
    CaseSpec(
        "anchor_att_only",
        "anchor att only",
        "P2_init_anchor_reset",
        "#fb8500",
        "Runtime truth anchor on attitude only.",
        enable_runtime_anchor=True,
        overrides=(
            ("fusion.init.runtime_truth_anchor_position", False),
            ("fusion.init.runtime_truth_anchor_velocity", False),
            ("fusion.init.runtime_truth_anchor_attitude", True),
        ),
    ),
    CaseSpec(
        "standard_reset_gamma",
        "standard reset gamma",
        "P2_init_anchor_reset",
        "#e63946",
        "Enable standard reset Gamma for ESKF update injection.",
        overrides=(("fusion.fej.debug_enable_standard_reset_gamma", True),),
    ),
    CaseSpec(
        "mech_baseline_short",
        "mechanism baseline",
        "P1F1_P2J1_mechanism",
        "#4c78a8",
        "Short-window baseline with full mechanism logging.",
        mechanism_case=True,
    ),
    CaseSpec(
        "mech_nhc_window_off_short",
        "mechanism NHC window off",
        "P1F1_P2J1_mechanism",
        "#72b7b2",
        "Short-window case with NHC disabled only in the restart window.",
        mechanism_case=True,
        overrides=(
            ("fusion.constraints.debug_nhc_disable_start_time", RESTART_DISABLE_START),
            ("fusion.constraints.debug_nhc_disable_end_time", RESTART_DISABLE_END),
        ),
    ),
    CaseSpec(
        "mech_gnss_pos_staged_short",
        "mechanism GNSS staged",
        "P1F1_P2J1_mechanism",
        "#b279a2",
        "Short-window case with staged GNSS position update.",
        mechanism_case=True,
        overrides=(("fusion.gnss_pos_update_mode", "stage_nonpos_then_pos"),),
    ),
    CaseSpec(
        "mech_standard_reset_gamma_short",
        "mechanism reset gamma",
        "P1F1_P2J1_mechanism",
        "#e45756",
        "Short-window case with standard reset Gamma enabled.",
        mechanism_case=True,
        overrides=(("fusion.fej.debug_enable_standard_reset_gamma", True),),
    ),
)


FAMILY_SPECS: tuple[FamilySpec, ...] = (
    FamilySpec(
        "P0_redraw",
        "P0 A: baseline redraw",
        ("baseline_free_run", "eskf_pva_truth_anchor", "nhc_off_below_5p0"),
        "Unify the plotting/metric aperture and retain the blunt 5 m/s negative result.",
    ),
    FamilySpec(
        "P0_window_disable",
        "P0 B: local window disable",
        ("baseline_free_run", "nhc_window_off", "odo_window_off"),
        "Separate NHC and ODO local necessity near 528551~528556.",
    ),
    FamilySpec(
        "P0_gnss_lever",
        "P0 C: GNSS/lever execution path",
        ("baseline_free_run", "gnss_pos_staged", "gnss_lever_window_freeze"),
        "Check whether GNSS joint att+lever is the direct jump executor.",
    ),
    FamilySpec(
        "P1_speed_threshold",
        "P1 B3: narrow speed threshold scan",
        ("baseline_free_run", "nhc_speed_0p5", "nhc_speed_1p0", "nhc_speed_1p5", "nhc_speed_2p0"),
        "Replace the blunt 5 m/s gate with narrow low-speed thresholds.",
    ),
    FamilySpec(
        "P1_joint_gate",
        "P1 B4/B5: weak-excitation and interval",
        ("baseline_free_run", "nhc_weak_gate_default", "nhc_weak_gate_strict", "nhc_interval_0p05", "nhc_interval_0p10", "nhc_interval_0p20"),
        "Compare joint weak-excitation gates and NHC update down-sampling.",
    ),
    FamilySpec(
        "P1_gnss_gain",
        "P1 C3/C4: GNSS gain path",
        ("baseline_free_run", "gnss_pos_gain_0p5", "gnss_pos_gain_0p25", "lgy_from_y_0p5", "lgy_from_y_0p0"),
        "Separate position gain scaling from lever-y write-back scaling.",
    ),
    FamilySpec(
        "P1_order_freeze",
        "P1 D2/E1: order and freeze",
        ("baseline_free_run", "odo_before_nhc", "freeze_mounting", "freeze_mounting_yaw", "freeze_gnss_lever", "freeze_odo_scale"),
        "Test route nodes by order changes and state freezing.",
    ),
    FamilySpec(
        "P2_bgz",
        "P2 F: bg_z deep route",
        ("baseline_free_run", "bgz_p0_30degh", "bgz_q_0p1x", "bgz_p0_30degh_q_0p1x", "bgz_observability_gate"),
        "Separate basin effect from active bg_z reinjection.",
    ),
    FamilySpec(
        "P2_time_alignment",
        "P2 H: time alignment",
        ("baseline_free_run", "odo_time_offset_neg10ms", "odo_time_offset_pos10ms"),
        "Check ODO time-offset sensitivity around stop-start.",
    ),
    FamilySpec(
        "P2_init_anchor_reset",
        "P2 I/J: init, anchor, reset",
        ("baseline_free_run", "calibration_init", "anchor_pos_only", "anchor_vel_only", "anchor_att_only", "standard_reset_gamma"),
        "Check basin, component anchors, and reset semantics.",
    ),
    FamilySpec(
        "P1F1_P2J1_mechanism",
        "P1F1/P2J1 mechanism",
        ("mech_baseline_short", "mech_nhc_window_off_short", "mech_gnss_pos_staged_short", "mech_standard_reset_gamma_short"),
        "Short-window mechanism audit with rich per-update logging.",
    ),
)


CASE_MAP = {spec.case_id: spec for spec in CASE_SPECS}
FAMILY_MAP = {spec.family_id: spec for spec in FAMILY_SPECS}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the NHC state-convergence experiment matrix from 实验研究.md and generate standardized artifacts."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID)
    parser.add_argument(
        "--cases",
        nargs="*",
        default=None,
        help="Optional subset of case ids. Default runs the whole matrix.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.case_derived_root = args.artifacts_dir / "derived"
    args.family_root = args.output_dir / "families"
    args.plot_dir = args.output_dir / "plots"
    return args


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


def wrap_deg(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def load_pos_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["timestamp", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def load_imu_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["timestamp", "dtheta_x", "dtheta_y", "dtheta_z", "dv_x", "dv_y", "dv_z"],
        engine="python",
    )


def llh_deg_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, h_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * cos_lon
    y = (n + h_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + h_m) * sin_lat
    return np.column_stack((x, y, z))


def rot_ned_to_ecef(lat_rad: np.ndarray, lon_rad: np.ndarray) -> np.ndarray:
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)
    rot = np.empty((lat_rad.size, 3, 3), dtype=float)
    rot[:, 0, 0] = -sin_lat * cos_lon
    rot[:, 0, 1] = -sin_lon
    rot[:, 0, 2] = -cos_lat * cos_lon
    rot[:, 1, 0] = -sin_lat * sin_lon
    rot[:, 1, 1] = cos_lon
    rot[:, 1, 2] = -cos_lat * sin_lon
    rot[:, 2, 0] = cos_lat
    rot[:, 2, 1] = 0.0
    rot[:, 2, 2] = -sin_lat
    return rot


def euler_to_rot_zyx(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def compute_earth_radius(lat_rad: float) -> tuple[float, float]:
    sin_lat = math.sin(lat_rad)
    sin2 = sin_lat * sin_lat
    rm = WGS84_A * (1.0 - WGS84_E2) / ((1.0 - WGS84_E2 * sin2) ** 1.5)
    rn = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin2)
    return rm, rn


def omega_ie_ned(lat_rad: float) -> np.ndarray:
    return np.array([OMEGA_IE * math.cos(lat_rad), 0.0, -OMEGA_IE * math.sin(lat_rad)], dtype=float)


def omega_en_ned(v_ned: np.ndarray, lat_rad: float, h_m: float) -> np.ndarray:
    rm, rn = compute_earth_radius(lat_rad)
    v_n, v_e, _ = v_ned
    return np.array(
        [v_e / (rn + h_m), -v_n / (rm + h_m), -v_e * math.tan(lat_rad) / (rn + h_m)],
        dtype=float,
    )


def set_nested(cfg: dict[str, Any], dotted_path: str, value: Any) -> None:
    parts = dotted_path.split(".")
    cursor: dict[str, Any] = cfg
    for key in parts[:-1]:
        cursor = cursor.setdefault(key, {})
    cursor[parts[-1]] = value


def parse_diag_times(stdout_text: str) -> dict[str, float | None]:
    patterns = {
        "first_divergence_start_t": r"\[Diag\] First divergence start t=([-+0-9eE\.]+)",
        "first_div_gnss_pos_t": r"\[Diag\] Div GNSS_POS update t=([-+0-9eE\.]+)",
    }
    out: dict[str, float | None] = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, stdout_text)
        out[key] = float(match.group(1)) if match else None
    return out


def human_bg_degph_to_radps(value_degph: float) -> float:
    return math.radians(value_degph) / 3600.0


def selected_case_specs(case_ids: list[str] | None) -> list[CaseSpec]:
    if not case_ids:
        return list(CASE_SPECS)
    selected: list[CaseSpec] = []
    seen: set[str] = set()
    for case_id in case_ids:
        if case_id not in CASE_MAP:
            raise ValueError(f"unsupported case_id: {case_id}")
        if case_id not in seen:
            selected.append(CASE_MAP[case_id])
            seen.add(case_id)
    return selected


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    case_spec: CaseSpec,
) -> tuple[dict[str, Any], dict[str, Any]]:
    baseline_spec = BaselineCaseSpec(
        case_id=case_spec.case_id,
        label=case_spec.label,
        color=case_spec.color,
        filter_mode="ESKF",
        enable_fej=case_spec.enable_fej,
        enable_runtime_anchor=case_spec.enable_runtime_anchor,
    )
    cfg, overrides = build_baseline_case_config(base_cfg, case_dir, baseline_spec)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    fej_cfg = fusion.setdefault("fej", {})

    if case_spec.mechanism_case:
        fusion["finaltime"] = MECH_FINALTIME
        constraints_cfg["enable_mechanism_log"] = True
        constraints_cfg["mechanism_log_post_gnss_only"] = False
        overrides["fusion.finaltime"] = MECH_FINALTIME
        overrides["fusion.constraints.enable_mechanism_log"] = True
        overrides["fusion.constraints.mechanism_log_post_gnss_only"] = False

    if case_spec.case_id in {"bgz_p0_30degh", "bgz_p0_30degh_q_0p1x"}:
        p0_diag = list(fusion["init"]["P0_diag"])
        p0_diag[14] = human_bg_degph_to_radps(30.0) ** 2
        init_cfg["P0_diag"] = p0_diag
        overrides["fusion.init.P0_diag[14]"] = p0_diag[14]

    if case_spec.case_id in {"bgz_q_0p1x", "bgz_p0_30degh_q_0p1x"}:
        base_sigma_bg = base_cfg["fusion"]["noise"].get("sigma_bg_vec")
        if base_sigma_bg:
            sigma_bg_vec = list(map(float, base_sigma_bg))
        else:
            sigma_bg_scalar = float(base_cfg["fusion"]["noise"]["sigma_bg"])
            sigma_bg_vec = [sigma_bg_scalar, sigma_bg_scalar, sigma_bg_scalar]
        sigma_bg_vec[2] = float(sigma_bg_vec[2]) * 0.1
        noise_cfg["sigma_bg_vec"] = sigma_bg_vec
        overrides["fusion.noise.sigma_bg_vec"] = sigma_bg_vec

    if case_spec.case_id == "calibration_init":
        init_cfg["ba0"] = [
            float(truth_reference["states"]["ba_x"]["reference_value_internal"]),
            float(truth_reference["states"]["ba_y"]["reference_value_internal"]),
            float(truth_reference["states"]["ba_z"]["reference_value_internal"]),
        ]
        init_cfg["bg0"] = [
            float(truth_reference["states"]["bg_x"]["reference_value_internal"]),
            float(truth_reference["states"]["bg_y"]["reference_value_internal"]),
            float(truth_reference["states"]["bg_z"]["reference_value_internal"]),
        ]
        init_cfg["sg0"] = [
            float(truth_reference["states"]["sg_x"]["reference_value_internal"]),
            float(truth_reference["states"]["sg_y"]["reference_value_internal"]),
            float(truth_reference["states"]["sg_z"]["reference_value_internal"]),
        ]
        init_cfg["sa0"] = [
            float(truth_reference["states"]["sa_x"]["reference_value_internal"]),
            float(truth_reference["states"]["sa_y"]["reference_value_internal"]),
            float(truth_reference["states"]["sa_z"]["reference_value_internal"]),
        ]
        init_cfg["odo_scale"] = 1.0
        init_cfg["mounting_roll0"] = float(truth_reference["states"]["mounting_roll"]["reference_value"])
        init_cfg["mounting_pitch0"] = float(truth_reference["states"]["mounting_pitch"]["reference_value"])
        init_cfg["mounting_yaw0"] = float(truth_reference["states"]["mounting_yaw"]["reference_value"])
        init_cfg["lever_arm0"] = [float(x) for x in truth_reference["sources"]["odo_lever_truth"]["value_m"]]
        init_cfg["gnss_lever_arm0"] = [float(x) for x in truth_reference["sources"]["gnss_lever_truth"]["value_m"]]
        overrides["special_init"] = "calibration_init"

    for dotted_path, value in case_spec.overrides:
        set_nested(cfg, dotted_path, value)
        overrides[dotted_path] = value

    if "sigma_mounting" not in noise_cfg:
        base_noise = base_cfg["fusion"]["noise"]
        noise_cfg["sigma_mounting"] = float(base_noise["sigma_mounting"]) * MOUNTING_SIGMA_SCALE
        noise_cfg["sigma_mounting_roll"] = float(base_noise["sigma_mounting_roll"]) * MOUNTING_SIGMA_SCALE
        noise_cfg["sigma_mounting_pitch"] = float(base_noise["sigma_mounting_pitch"]) * MOUNTING_SIGMA_SCALE
        noise_cfg["sigma_mounting_yaw"] = float(base_noise["sigma_mounting_yaw"]) * MOUNTING_SIGMA_SCALE

    if case_spec.enable_runtime_anchor:
        init_cfg["runtime_truth_anchor_pva"] = True
    elif case_spec.case_id not in {"anchor_pos_only", "anchor_vel_only", "anchor_att_only"}:
        init_cfg["runtime_truth_anchor_position"] = False
        init_cfg["runtime_truth_anchor_velocity"] = False
        init_cfg["runtime_truth_anchor_attitude"] = False

    fej_cfg["enable"] = bool(case_spec.enable_fej)
    ablation_cfg.setdefault("disable_mounting_roll", False)
    ablation_cfg.setdefault("disable_gnss_lever_z", False)
    return cfg, overrides


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    case_spec: CaseSpec,
) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(base_cfg, truth_reference, case_dir, case_spec)
    cfg_path = case_dir / f"config_{case_spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides


def build_truth_interp(query_t: np.ndarray, truth_df: pd.DataFrame) -> pd.DataFrame:
    truth_t = truth_df["timestamp"].to_numpy(dtype=float)
    out = pd.DataFrame({"timestamp": query_t})
    for col in ["lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"]:
        src = truth_df[col].to_numpy(dtype=float)
        if col == "yaw":
            src = np.rad2deg(np.unwrap(np.deg2rad(src)))
        values = np.interp(query_t, truth_t, src)
        if col == "yaw":
            values = wrap_deg(values)
        out[col] = values
    return out


def merge_case_outputs(sol_path: Path, state_series_path: Path) -> pd.DataFrame:
    sol_df = pd.read_csv(sol_path, sep=r"\s+", engine="python")
    state_df = pd.read_csv(state_series_path)
    if "odo_scale" in state_df.columns and "odo_scale_state" not in state_df.columns:
        state_df = state_df.rename(columns={"odo_scale": "odo_scale_state"})
    merged = pd.merge_asof(
        sol_df.sort_values("timestamp"),
        state_df.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=1.0e-6,
    )
    required = [
        "fused_x",
        "fused_y",
        "fused_z",
        "fused_vx",
        "fused_vy",
        "fused_vz",
        "fused_roll",
        "fused_pitch",
        "fused_yaw",
        "ba_x_mgal",
        "ba_y_mgal",
        "ba_z_mgal",
        "bg_x_degh",
        "bg_y_degh",
        "bg_z_degh",
        "sg_x_ppm",
        "sg_y_ppm",
        "sg_z_ppm",
        "sa_x_ppm",
        "sa_y_ppm",
        "sa_z_ppm",
        "odo_scale_state",
        "mounting_roll_deg",
        "mounting_pitch_deg",
        "mounting_yaw_deg",
        "odo_lever_x_m",
        "odo_lever_y_m",
        "odo_lever_z_m",
        "gnss_lever_x_m",
        "gnss_lever_y_m",
        "gnss_lever_z_m",
    ]
    missing = [col for col in required if col not in merged.columns]
    if missing:
        raise RuntimeError(f"missing merged columns: {missing}")
    if "total_mounting_roll_deg" not in merged.columns:
        merged["total_mounting_roll_deg"] = merged["mounting_roll_deg"]
    if "total_mounting_pitch_deg" not in merged.columns:
        merged["total_mounting_pitch_deg"] = merged["mounting_pitch_deg"]
    if "total_mounting_yaw_deg" not in merged.columns:
        merged["total_mounting_yaw_deg"] = merged["mounting_yaw_deg"]
    return merged


def build_motion_frame(
    merged_df: pd.DataFrame,
    truth_interp_df: pd.DataFrame,
    imu_df: pd.DataFrame,
    truth_reference: dict[str, Any],
) -> pd.DataFrame:
    motion = pd.merge_asof(
        merged_df[["timestamp"]].sort_values("timestamp"),
        imu_df.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=0.002,
    )
    if motion[["dtheta_x", "dtheta_y", "dtheta_z", "dv_x", "dv_y", "dv_z"]].isna().any().any():
        raise RuntimeError("failed to align IMU samples for motion reconstruction")

    t = motion["timestamp"].to_numpy(dtype=float)
    dt = np.diff(t, prepend=t[0])
    if dt.size > 1:
        dt[0] = dt[1]
    dt = np.clip(dt, 1.0e-6, None)
    raw_omega = motion[["dtheta_x", "dtheta_y", "dtheta_z"]].to_numpy(dtype=float) / dt[:, None]
    raw_acc = motion[["dv_x", "dv_y", "dv_z"]].to_numpy(dtype=float) / dt[:, None]

    roll_rad = np.deg2rad(merged_df["fused_roll"].to_numpy(dtype=float))
    pitch_rad = np.deg2rad(merged_df["fused_pitch"].to_numpy(dtype=float))
    yaw_rad = np.deg2rad(merged_df["fused_yaw"].to_numpy(dtype=float))
    lat_rad = np.deg2rad(truth_interp_df["lat"].to_numpy(dtype=float))
    lon_rad = np.deg2rad(truth_interp_df["lon"].to_numpy(dtype=float))
    h_m = truth_interp_df["h"].to_numpy(dtype=float)
    c_bn = np.stack([euler_to_rot_zyx(r, p, y) for r, p, y in zip(roll_rad, pitch_rad, yaw_rad)], axis=0)
    v_ecef = merged_df[["fused_vx", "fused_vy", "fused_vz"]].to_numpy(dtype=float)
    rot_ne = rot_ned_to_ecef(lat_rad, lon_rad)
    v_ned = np.einsum("nij,nj->ni", np.transpose(rot_ne, (0, 2, 1)), v_ecef)
    v_body = np.einsum("nij,nj->ni", np.transpose(c_bn, (0, 2, 1)), v_ned)
    rm, rn = [], []
    for lat_value in lat_rad:
        rm_i, rn_i = compute_earth_radius(float(lat_value))
        rm.append(rm_i)
        rn.append(rn_i)
    rm_arr = np.asarray(rm, dtype=float)
    rn_arr = np.asarray(rn, dtype=float)
    omega_ie = np.column_stack((OMEGA_IE * np.cos(lat_rad), np.zeros_like(lat_rad), -OMEGA_IE * np.sin(lat_rad)))
    omega_en = np.column_stack(
        (
            v_ned[:, 1] / (rn_arr + h_m),
            -v_ned[:, 0] / (rm_arr + h_m),
            -v_ned[:, 1] * np.tan(lat_rad) / (rn_arr + h_m),
        )
    )
    omega_in_b = np.einsum("nij,nj->ni", np.transpose(c_bn, (0, 2, 1)), omega_ie + omega_en)

    bg_radps = np.deg2rad(
        merged_df[["bg_x_degh", "bg_y_degh", "bg_z_degh"]].to_numpy(dtype=float) / 3600.0
    )
    sg_frac = merged_df[["sg_x_ppm", "sg_y_ppm", "sg_z_ppm"]].to_numpy(dtype=float) * 1.0e-6
    omega_ib_unbiased = raw_omega - bg_radps
    sf_g = 1.0 - sg_frac
    omega_ib_corr = sf_g * omega_ib_unbiased
    omega_nb_b = omega_ib_corr - omega_in_b

    lever = merged_df[["odo_lever_x_m", "odo_lever_y_m", "odo_lever_z_m"]].to_numpy(dtype=float)
    v_rot = np.cross(omega_nb_b, lever)
    v_wheel_b = v_body + v_rot

    mount_roll = np.deg2rad(merged_df["total_mounting_roll_deg"].to_numpy(dtype=float))
    mount_pitch = np.deg2rad(merged_df["total_mounting_pitch_deg"].to_numpy(dtype=float))
    mount_yaw = np.deg2rad(merged_df["total_mounting_yaw_deg"].to_numpy(dtype=float))
    c_b_v = np.stack(
        [np.transpose(euler_to_rot_zyx(r, p, y)) for r, p, y in zip(mount_roll, mount_pitch, mount_yaw)],
        axis=0,
    )
    v_vehicle = np.einsum("nij,nj->ni", c_b_v, v_wheel_b)
    v_vehicle_trans = np.einsum("nij,nj->ni", c_b_v, v_body)
    v_vehicle_rot = np.einsum("nij,nj->ni", c_b_v, v_rot)

    truth_roll_rad = np.deg2rad(truth_interp_df["roll"].to_numpy(dtype=float))
    truth_pitch_rad = np.deg2rad(truth_interp_df["pitch"].to_numpy(dtype=float))
    truth_yaw_rad = np.deg2rad(truth_interp_df["yaw"].to_numpy(dtype=float))
    truth_c_bn = np.stack(
        [euler_to_rot_zyx(r, p, y) for r, p, y in zip(truth_roll_rad, truth_pitch_rad, truth_yaw_rad)],
        axis=0,
    )
    truth_v_ned = truth_interp_df[["vn", "ve", "vd"]].to_numpy(dtype=float)
    truth_v_body = np.einsum("nij,nj->ni", np.transpose(truth_c_bn, (0, 2, 1)), truth_v_ned)
    truth_bg = np.array(
        [
            float(truth_reference["states"]["bg_x"]["reference_value_internal"]),
            float(truth_reference["states"]["bg_y"]["reference_value_internal"]),
            float(truth_reference["states"]["bg_z"]["reference_value_internal"]),
        ],
        dtype=float,
    )
    truth_sg = np.array(
        [
            float(truth_reference["states"]["sg_x"]["reference_value_internal"]),
            float(truth_reference["states"]["sg_y"]["reference_value_internal"]),
            float(truth_reference["states"]["sg_z"]["reference_value_internal"]),
        ],
        dtype=float,
    )
    truth_rm, truth_rn = [], []
    for lat_value in lat_rad:
        rm_i, rn_i = compute_earth_radius(float(lat_value))
        truth_rm.append(rm_i)
        truth_rn.append(rn_i)
    truth_rm_arr = np.asarray(truth_rm, dtype=float)
    truth_rn_arr = np.asarray(truth_rn, dtype=float)
    truth_omega_en = np.column_stack(
        (
            truth_v_ned[:, 1] / (truth_rn_arr + h_m),
            -truth_v_ned[:, 0] / (truth_rm_arr + h_m),
            -truth_v_ned[:, 1] * np.tan(lat_rad) / (truth_rn_arr + h_m),
        )
    )
    truth_omega_in_b = np.einsum("nij,nj->ni", np.transpose(truth_c_bn, (0, 2, 1)), omega_ie + truth_omega_en)
    truth_omega_ib_unbiased = raw_omega - truth_bg[None, :]
    truth_omega_ib_corr = (1.0 - truth_sg[None, :]) * truth_omega_ib_unbiased
    truth_omega_nb_b = truth_omega_ib_corr - truth_omega_in_b
    truth_lever = np.array(truth_reference["sources"]["odo_lever_truth"]["value_m"], dtype=float)
    truth_v_rot = np.cross(truth_omega_nb_b, np.repeat(truth_lever[None, :], len(t), axis=0))
    truth_v_wheel_b = truth_v_body + truth_v_rot
    total_truth = truth_reference["sources"]["mounting_total_truth"]["value_deg"]
    truth_c_b_v = np.transpose(
        euler_to_rot_zyx(
            math.radians(float(total_truth["roll"])),
            math.radians(float(total_truth["pitch"])),
            math.radians(float(total_truth["yaw"])),
        )
    )
    truth_v_vehicle = np.einsum("ij,nj->ni", truth_c_b_v, truth_v_wheel_b)

    motion_df = pd.DataFrame(
        {
            "timestamp": t,
            "v_b_x_mps": v_body[:, 0],
            "v_b_y_mps": v_body[:, 1],
            "v_b_z_mps": v_body[:, 2],
            "v_v_x_mps": v_vehicle[:, 0],
            "v_v_y_mps": v_vehicle[:, 1],
            "v_v_z_mps": v_vehicle[:, 2],
            "v_v_trans_x_mps": v_vehicle_trans[:, 0],
            "v_v_trans_y_mps": v_vehicle_trans[:, 1],
            "v_v_trans_z_mps": v_vehicle_trans[:, 2],
            "v_v_rot_x_mps": v_vehicle_rot[:, 0],
            "v_v_rot_y_mps": v_vehicle_rot[:, 1],
            "v_v_rot_z_mps": v_vehicle_rot[:, 2],
            "truth_v_v_x_mps": truth_v_vehicle[:, 0],
            "truth_v_v_y_mps": truth_v_vehicle[:, 1],
            "truth_v_v_z_mps": truth_v_vehicle[:, 2],
            "body_forward_speed_abs_mps": np.abs(v_body[:, 0]),
            "yaw_rate_raw_deg_s": np.rad2deg(raw_omega[:, 2]),
            "lateral_acc_raw_mps2": raw_acc[:, 1],
        }
    )
    return motion_df


def build_plot_frame(
    merged_df: pd.DataFrame,
    truth_interp_df: pd.DataFrame,
    truth_reference: dict[str, Any],
    motion_df: pd.DataFrame,
) -> pd.DataFrame:
    t = merged_df["timestamp"].to_numpy(dtype=float)
    truth_xyz = llh_deg_to_ecef(
        truth_interp_df["lat"].to_numpy(dtype=float),
        truth_interp_df["lon"].to_numpy(dtype=float),
        truth_interp_df["h"].to_numpy(dtype=float),
    )
    sol_xyz = merged_df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float)
    err_ecef = sol_xyz - truth_xyz
    rot_ne = rot_ned_to_ecef(
        np.deg2rad(truth_interp_df["lat"].to_numpy(dtype=float)),
        np.deg2rad(truth_interp_df["lon"].to_numpy(dtype=float)),
    )
    err_ned = np.einsum("nji,nj->ni", rot_ne, err_ecef)
    sol_v_ecef = merged_df[["fused_vx", "fused_vy", "fused_vz"]].to_numpy(dtype=float)
    sol_v_ned = np.einsum("nji,nj->ni", rot_ne, sol_v_ecef)
    truth_v_ned = truth_interp_df[["vn", "ve", "vd"]].to_numpy(dtype=float)

    plot_df = pd.DataFrame({"timestamp": t})
    plot_df["p_n_err_m"] = err_ned[:, 0]
    plot_df["p_e_err_m"] = err_ned[:, 1]
    plot_df["p_u_err_m"] = -err_ned[:, 2]
    plot_df["v_n_err_mps"] = sol_v_ned[:, 0] - truth_v_ned[:, 0]
    plot_df["v_e_err_mps"] = sol_v_ned[:, 1] - truth_v_ned[:, 1]
    plot_df["v_u_err_mps"] = -(sol_v_ned[:, 2] - truth_v_ned[:, 2])
    plot_df["roll_err_deg"] = wrap_deg(
        merged_df["fused_roll"].to_numpy(dtype=float) - truth_interp_df["roll"].to_numpy(dtype=float)
    )
    plot_df["pitch_err_deg"] = wrap_deg(
        merged_df["fused_pitch"].to_numpy(dtype=float) - truth_interp_df["pitch"].to_numpy(dtype=float)
    )
    plot_df["yaw_err_deg"] = wrap_deg(
        merged_df["fused_yaw"].to_numpy(dtype=float) - truth_interp_df["yaw"].to_numpy(dtype=float)
    )
    plot_df["odo_scale_err"] = merged_df["odo_scale_state"].to_numpy(dtype=float) - 1.0
    plot_df["mounting_roll_err_deg"] = (
        merged_df["mounting_roll_deg"].to_numpy(dtype=float)
        - float(truth_reference["states"]["mounting_roll"]["reference_value"])
    )
    plot_df["mounting_pitch_err_deg"] = (
        merged_df["mounting_pitch_deg"].to_numpy(dtype=float)
        - float(truth_reference["states"]["mounting_pitch"]["reference_value"])
    )
    plot_df["mounting_yaw_err_deg"] = (
        merged_df["mounting_yaw_deg"].to_numpy(dtype=float)
        - float(truth_reference["states"]["mounting_yaw"]["reference_value"])
    )
    plot_df["odo_lever_x_err_m"] = (
        merged_df["odo_lever_x_m"].to_numpy(dtype=float)
        - float(truth_reference["states"]["odo_lever_x"]["reference_value"])
    )
    plot_df["odo_lever_y_err_m"] = (
        merged_df["odo_lever_y_m"].to_numpy(dtype=float)
        - float(truth_reference["states"]["odo_lever_y"]["reference_value"])
    )
    plot_df["odo_lever_z_err_m"] = (
        merged_df["odo_lever_z_m"].to_numpy(dtype=float)
        - float(truth_reference["states"]["odo_lever_z"]["reference_value"])
    )
    plot_df["gnss_lever_x_err_m"] = (
        merged_df["gnss_lever_x_m"].to_numpy(dtype=float)
        - float(truth_reference["states"]["gnss_lever_x"]["reference_value"])
    )
    plot_df["gnss_lever_y_err_m"] = (
        merged_df["gnss_lever_y_m"].to_numpy(dtype=float)
        - float(truth_reference["states"]["gnss_lever_y"]["reference_value"])
    )
    plot_df["gnss_lever_z_err_m"] = (
        merged_df["gnss_lever_z_m"].to_numpy(dtype=float)
        - float(truth_reference["states"]["gnss_lever_z"]["reference_value"])
    )
    for col in [
        "ba_x_mgal",
        "ba_y_mgal",
        "ba_z_mgal",
        "bg_x_degh",
        "bg_y_degh",
        "bg_z_degh",
        "sg_x_ppm",
        "sg_y_ppm",
        "sg_z_ppm",
        "sa_x_ppm",
        "sa_y_ppm",
        "sa_z_ppm",
        "total_mounting_roll_deg",
        "total_mounting_pitch_deg",
        "total_mounting_yaw_deg",
    ]:
        plot_df[col] = merged_df[col].to_numpy(dtype=float)
    for col in motion_df.columns:
        if col != "timestamp":
            plot_df[col] = motion_df[col].to_numpy(dtype=float)
    return plot_df


def compute_jump_metrics(plot_df: pd.DataFrame, case_id: str) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    t = plot_df["timestamp"].to_numpy(dtype=float)
    t0 = math.ceil(float(t[0]))
    t1 = math.floor(float(t[-1]))
    sec_grid = np.arange(t0, t1 + 1.0, 1.0, dtype=float)
    for state_key, threshold in KEY_STATES_FOR_JUMP:
        values = plot_df[state_key].to_numpy(dtype=float)
        sec_values = np.interp(sec_grid, t, values)
        if sec_values.size >= 2:
            sec_diff = np.diff(sec_values)
            diff_times = sec_grid[1:]
            max_idx = int(np.argmax(np.abs(sec_diff)))
            max_jump = float(sec_diff[max_idx])
            max_jump_abs = float(np.abs(sec_diff[max_idx]))
            max_jump_t = float(diff_times[max_idx])
            above = np.where(np.abs(sec_diff) >= threshold)[0]
            first_jump_t = float(diff_times[above[0]]) if above.size else math.nan
        else:
            sec_diff = np.array([], dtype=float)
            max_jump = math.nan
            max_jump_abs = math.nan
            max_jump_t = math.nan
            first_jump_t = math.nan

        tail_mask = sec_grid >= max(sec_grid[-1] - TAIL_SECONDS, sec_grid[0]) if sec_grid.size else np.array([], dtype=bool)
        tail_values = sec_values[tail_mask] if sec_grid.size else np.array([], dtype=float)
        if tail_values.size >= 2:
            slope = float(np.polyfit(sec_grid[tail_mask], tail_values, 1)[0])
            tail_std = float(np.std(tail_values))
            tail_abs_mean = float(np.mean(np.abs(tail_values)))
        elif tail_values.size == 1:
            slope = 0.0
            tail_std = 0.0
            tail_abs_mean = float(np.abs(tail_values[0]))
        else:
            slope = math.nan
            tail_std = math.nan
            tail_abs_mean = math.nan

        rows.append(
            {
                "case_id": case_id,
                "state_key": state_key,
                "jump_threshold": threshold,
                "first_jump_t_1hz": first_jump_t,
                "max_jump_1hz": max_jump,
                "max_abs_jump_1hz": max_jump_abs,
                "max_jump_t_1hz": max_jump_t,
                "total_variation_1hz": float(np.sum(np.abs(sec_diff))) if sec_diff.size else math.nan,
                "tail_std_last120s": tail_std,
                "tail_abs_mean_last120s": tail_abs_mean,
                "tail_slope_last120s_per_s": slope,
                "overall_abs_max": float(np.max(np.abs(values))),
            }
        )
    return pd.DataFrame(rows)


def compute_motion_metrics(plot_df: pd.DataFrame, case_id: str) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    windows = {
        "overall": np.ones(len(plot_df), dtype=bool),
        "jump_window": (plot_df["timestamp"] >= JUMP_WINDOW_START) & (plot_df["timestamp"] <= JUMP_WINDOW_END),
        "restart_window": (plot_df["timestamp"] >= RESTART_DISABLE_START) & (plot_df["timestamp"] <= RESTART_DISABLE_END),
    }
    for window_name, mask in windows.items():
        if not np.any(mask):
            continue
        subset = plot_df.loc[mask]
        rows.append(
            {
                "case_id": case_id,
                "window": window_name,
                "samples": int(len(subset)),
                "forward_speed_abs_max_mps": float(np.max(subset["body_forward_speed_abs_mps"])),
                "forward_speed_abs_min_mps": float(np.min(subset["body_forward_speed_abs_mps"])),
                "yaw_rate_abs_max_deg_s": float(np.max(np.abs(subset["yaw_rate_raw_deg_s"]))),
                "lateral_acc_abs_max_mps2": float(np.max(np.abs(subset["lateral_acc_raw_mps2"]))),
                "v_v_y_abs_max_mps": float(np.max(np.abs(subset["v_v_y_mps"]))),
                "v_v_z_abs_max_mps": float(np.max(np.abs(subset["v_v_z_mps"]))),
                "truth_v_v_y_abs_max_mps": float(np.max(np.abs(subset["truth_v_v_y_mps"]))),
                "truth_v_v_z_abs_max_mps": float(np.max(np.abs(subset["truth_v_v_z_mps"]))),
                "v_v_rot_y_abs_max_mps": float(np.max(np.abs(subset["v_v_rot_y_mps"]))),
                "v_v_rot_z_abs_max_mps": float(np.max(np.abs(subset["v_v_rot_z_mps"]))),
                "v_v_trans_y_abs_max_mps": float(np.max(np.abs(subset["v_v_trans_y_mps"]))),
                "v_v_trans_z_abs_max_mps": float(np.max(np.abs(subset["v_v_trans_z_mps"]))),
            }
        )
    return pd.DataFrame(rows)


def extract_mechanism_metrics(mechanism_path: Path, case_id: str) -> dict[str, Any]:
    if not mechanism_path.exists():
        return {"case_id": case_id}
    mech_df = pd.read_csv(mechanism_path)
    rows: dict[str, Any] = {"case_id": case_id}
    window = mech_df[(mech_df["t_meas"] >= RESTART_DISABLE_START) & (mech_df["t_meas"] <= RESTART_DISABLE_END)]
    if window.empty:
        return rows
    for tag in ["NHC", "ODO", "GNSS_POS"]:
        subset = window.loc[window["tag"] == tag]
        if subset.empty:
            continue
        if "info_mount_yaw" in subset.columns:
            rows[f"{tag.lower()}_max_info_mount_yaw_restart"] = float(subset["info_mount_yaw"].max())
        if "k_row_bg_z_norm" in subset.columns:
            rows[f"{tag.lower()}_max_k_row_bg_z_norm_restart"] = float(subset["k_row_bg_z_norm"].max())
        if "dx_bg_z" in subset.columns:
            rows[f"{tag.lower()}_max_abs_dx_bg_z_restart"] = float(np.max(np.abs(subset["dx_bg_z"])))
        if "dx_mount_yaw" in subset.columns:
            rows[f"{tag.lower()}_max_abs_dx_mount_yaw_restart"] = float(np.max(np.abs(subset["dx_mount_yaw"])))
    for col in ["prior_cov_att_z_bg_z", "prior_cov_mount_yaw_bg_z", "post_cov_att_z_bg_z", "post_cov_mount_yaw_bg_z"]:
        if col in window.columns:
            rows[f"max_abs_{col}_restart"] = float(np.max(np.abs(window[col])))
    return rows


def save_case_products(
    case_dir: Path,
    case_spec: CaseSpec,
    plot_df: pd.DataFrame,
    jump_df: pd.DataFrame,
    motion_df: pd.DataFrame,
) -> dict[str, str]:
    plot_frame_path = case_dir / f"plot_frame_{case_spec.case_id}.csv"
    plot_jump_path = case_dir / f"plot_frame_jump_{case_spec.case_id}.csv"
    jump_metrics_path = case_dir / f"jump_metrics_{case_spec.case_id}.csv"
    motion_metrics_path = case_dir / f"motion_metrics_{case_spec.case_id}.csv"
    plot_df.to_csv(plot_frame_path, index=False, encoding="utf-8-sig")
    plot_df.loc[(plot_df["timestamp"] >= JUMP_WINDOW_START) & (plot_df["timestamp"] <= JUMP_WINDOW_END)].to_csv(
        plot_jump_path, index=False, encoding="utf-8-sig"
    )
    jump_df.to_csv(jump_metrics_path, index=False, encoding="utf-8-sig")
    motion_df.to_csv(motion_metrics_path, index=False, encoding="utf-8-sig")
    return {
        "plot_frame_path": rel_from_root(plot_frame_path, REPO_ROOT),
        "plot_frame_jump_path": rel_from_root(plot_jump_path, REPO_ROOT),
        "jump_metrics_path": rel_from_root(jump_metrics_path, REPO_ROOT),
        "motion_metrics_path": rel_from_root(motion_metrics_path, REPO_ROOT),
    }


def plot_column_grid(
    case_frames: dict[str, pd.DataFrame],
    case_ids: list[str],
    columns: tuple[PlotColumn, ...],
    output_path: Path,
    title: str,
    window: tuple[float, float] | None = None,
) -> None:
    n_cols = 3
    n_rows = math.ceil(len(columns) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(17, 2.8 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, column in enumerate(columns):
        ax = axes_flat[idx]
        for case_id in case_ids:
            case_spec = CASE_MAP[case_id]
            df = case_frames[case_id]
            if window is not None:
                df = df.loc[(df["timestamp"] >= window[0]) & (df["timestamp"] <= window[1])]
            if df.empty:
                continue
            t = df["timestamp"].to_numpy(dtype=float)
            v = df[column.key].to_numpy(dtype=float)
            t_plot, v_plot = downsample_for_plot(t, v)
            ax.plot(t_plot, v_plot, linewidth=0.95, color=case_spec.color, label=case_spec.label)
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


def plot_velocity_family(
    case_frames: dict[str, pd.DataFrame],
    case_ids: list[str],
    output_path: Path,
    window: tuple[float, float] | None = None,
    use_body_frame: bool = False,
) -> None:
    if use_body_frame:
        cols = [("v_b_x_mps", "v_b.x"), ("v_b_y_mps", "v_b.y"), ("v_b_z_mps", "v_b.z")]
        title = "Body-frame velocity comparison"
    else:
        cols = [("v_v_x_mps", "v_v.x"), ("v_v_y_mps", "v_v.y"), ("v_v_z_mps", "v_v.z")]
        title = "Vehicle-frame velocity comparison"
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for ax, (col, label) in zip(axes, cols):
        for case_id in case_ids:
            case_spec = CASE_MAP[case_id]
            df = case_frames[case_id]
            if window is not None:
                df = df.loc[(df["timestamp"] >= window[0]) & (df["timestamp"] <= window[1])]
            if df.empty:
                continue
            t = df["timestamp"].to_numpy(dtype=float)
            y = df[col].to_numpy(dtype=float)
            t_plot, y_plot = downsample_for_plot(t, y)
            ax.plot(t_plot, y_plot, linewidth=1.0, color=case_spec.color, label=case_spec.label)
            if not use_body_frame:
                truth_col = f"truth_{col}"
                if truth_col in df.columns and case_id == case_ids[0]:
                    tt_plot, ty_plot = downsample_for_plot(t, df[truth_col].to_numpy(dtype=float))
                    ax.plot(tt_plot, ty_plot, linewidth=1.0, color="black", linestyle="--", label="truth_v_v")
        ax.set_ylabel("m/s")
        ax.set_title(label)
        ax.grid(alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle(title if window is None else f"{title} [{window[0]:.0f}, {window[1]:.0f}] s", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_key_jump_states(
    case_frames: dict[str, pd.DataFrame],
    case_ids: list[str],
    output_path: Path,
    window: tuple[float, float],
) -> None:
    fig, axes = plt.subplots(len(KEY_STATES_FOR_JUMP), 1, figsize=(13, 3.0 * len(KEY_STATES_FOR_JUMP)), sharex=True)
    for ax, (state_key, _) in zip(np.atleast_1d(axes), KEY_STATES_FOR_JUMP):
        for case_id in case_ids:
            case_spec = CASE_MAP[case_id]
            df = case_frames[case_id]
            df = df.loc[(df["timestamp"] >= window[0]) & (df["timestamp"] <= window[1])]
            if df.empty:
                continue
            t = df["timestamp"].to_numpy(dtype=float)
            y = df[state_key].to_numpy(dtype=float)
            t_plot, y_plot = downsample_for_plot(t, y)
            ax.plot(t_plot, y_plot, linewidth=1.0, color=case_spec.color, label=case_spec.label)
        ax.set_title(state_key)
        ax.grid(alpha=0.25)
    np.atleast_1d(axes)[0].legend(loc="best", fontsize=8)
    np.atleast_1d(axes)[-1].set_xlabel("timestamp [s]")
    fig.suptitle(f"Jump-window key states [{window[0]:.0f}, {window[1]:.0f}] s", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_family_summary(
    output_path: Path,
    family_spec: FamilySpec,
    family_case_df: pd.DataFrame,
    family_jump_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    metric_rows: list[list[str]] = []
    for _, row in family_case_df.iterrows():
        metric_rows.append(
            [
                str(row["case_id"]),
                format_metric(row.get("overall_rmse_3d_m_aux")),
                format_metric(row.get("overall_final_err_3d_m_aux")),
                format_metric(row.get("nhc_accept_ratio")),
                format_metric(row.get("odo_accept_ratio")),
                format_metric(row.get("first_divergence_start_t")),
                format_metric(row.get("yaw_err_deg_first_jump_t_1hz")),
                format_metric(row.get("yaw_err_deg_max_abs_jump_1hz")),
                format_metric(row.get("mounting_yaw_err_deg_max_abs_jump_1hz")),
                format_metric(row.get("gnss_lever_y_err_m_max_abs_jump_1hz")),
            ]
        )
    jump_focus = family_jump_df.loc[
        family_jump_df["state_key"].isin(["yaw_err_deg", "mounting_yaw_err_deg", "gnss_lever_y_err_m", "bg_z_degh"])
    ].copy()
    jump_focus = jump_focus.sort_values(by=["state_key", "max_abs_jump_1hz"], ascending=[True, False])
    jump_rows = [
        [
            str(row["case_id"]),
            str(row["state_key"]),
            format_metric(row["first_jump_t_1hz"]),
            format_metric(row["max_abs_jump_1hz"]),
            format_metric(row["tail_std_last120s"]),
            format_metric(row["tail_slope_last120s_per_s"]),
        ]
        for _, row in jump_focus.iterrows()
    ]

    lines = [
        f"# {family_spec.title}",
        "",
        f"- focus: {family_spec.focus}",
        "",
        "## Case Summary",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "rmse3d",
                "final3d",
                "nhc_accept",
                "odo_accept",
                "t_div_first",
                "yaw_first_jump",
                "yaw_max_jump",
                "mount_yaw_max_jump",
                "lgy_max_jump",
            ],
            metric_rows,
        )
    )
    lines.extend(["", "## Jump Metrics"])
    lines.extend(
        render_table(
            ["case_id", "state_key", "first_jump_t_1hz", "max_abs_jump_1hz", "tail_std_last120s", "tail_slope"],
            jump_rows,
        )
    )
    lines.extend(["", "## Plots"])
    for key, value in plot_paths.items():
        lines.append(f"- {key}: `{value}`")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_case_metrics(
    case_row: dict[str, Any],
    jump_df: pd.DataFrame,
    motion_df: pd.DataFrame,
    mechanism_metrics: dict[str, Any],
) -> dict[str, Any]:
    out = dict(case_row)
    for _, row in jump_df.iterrows():
        prefix = str(row["state_key"])
        out[f"{prefix}_first_jump_t_1hz"] = row["first_jump_t_1hz"]
        out[f"{prefix}_max_abs_jump_1hz"] = row["max_abs_jump_1hz"]
        out[f"{prefix}_tail_std_last120s"] = row["tail_std_last120s"]
        out[f"{prefix}_tail_slope_last120s_per_s"] = row["tail_slope_last120s_per_s"]
    for _, row in motion_df.iterrows():
        window = str(row["window"])
        out[f"{window}_v_v_y_abs_max_mps"] = row["v_v_y_abs_max_mps"]
        out[f"{window}_v_v_z_abs_max_mps"] = row["v_v_z_abs_max_mps"]
        out[f"{window}_truth_v_v_y_abs_max_mps"] = row["truth_v_v_y_abs_max_mps"]
        out[f"{window}_truth_v_v_z_abs_max_mps"] = row["truth_v_v_z_abs_max_mps"]
    out.update(mechanism_metrics)
    return out


def write_root_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    family_paths: dict[str, dict[str, str]],
) -> None:
    ranked = case_metrics_df.sort_values(by=["yaw_err_deg_max_abs_jump_1hz", "mounting_yaw_err_deg_max_abs_jump_1hz"], ascending=[True, True]).head(12)
    metric_rows = [
        [
            str(row["case_id"]),
            str(row["family"]),
            format_metric(row.get("overall_rmse_3d_m_aux")),
            format_metric(row.get("overall_final_err_3d_m_aux")),
            format_metric(row.get("first_divergence_start_t")),
            format_metric(row.get("yaw_err_deg_first_jump_t_1hz")),
            format_metric(row.get("yaw_err_deg_max_abs_jump_1hz")),
            format_metric(row.get("mounting_yaw_err_deg_max_abs_jump_1hz")),
            format_metric(row.get("gnss_lever_y_err_m_max_abs_jump_1hz")),
            format_metric(row.get("restart_window_v_v_y_abs_max_mps")),
        ]
        for _, row in ranked.iterrows()
    ]
    lines = [
        "# data2 NHC state-convergence research summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        "",
        "## Best jump-behavior cases",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "family",
                "rmse3d",
                "final3d",
                "t_div_first",
                "yaw_first_jump",
                "yaw_max_jump",
                "mount_yaw_max_jump",
                "lgy_max_jump",
                "restart_vv_y_abs_max",
            ],
            metric_rows,
        )
    )
    lines.extend(["", "## Family Outputs"])
    for family_id, paths in family_paths.items():
        lines.append(f"- `{family_id}`: summary=`{paths['summary']}` plots_dir=`{paths['plots_dir']}`")
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
    ensure_dir(args.case_derived_root)
    ensure_dir(args.family_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())

    case_frames: dict[str, pd.DataFrame] = {}
    case_metrics_rows: list[dict[str, Any]] = []
    jump_metric_frames: list[pd.DataFrame] = []
    motion_metric_frames: list[pd.DataFrame] = []
    case_overrides: dict[str, dict[str, Any]] = {}
    case_config_paths: dict[str, str] = {}
    family_paths: dict[str, dict[str, str]] = {}

    for case_spec in selected_specs:
        case_dir = args.case_root / case_spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides = write_case_config(base_cfg, truth_reference, case_dir, case_spec)
        case_overrides[case_spec.case_id] = overrides
        case_config_paths[case_spec.case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_row = run_baseline_case(case_dir, cfg_path, args.exe, BaselineCaseSpec(
            case_id=case_spec.case_id,
            label=case_spec.label,
            color=case_spec.color,
            filter_mode="ESKF",
            enable_fej=case_spec.enable_fej,
            enable_runtime_anchor=case_spec.enable_runtime_anchor,
        ))
        case_row["family"] = case_spec.family
        case_row["description"] = case_spec.description
        stdout_path = (REPO_ROOT / case_row["stdout_path"]).resolve()
        stdout_text = stdout_path.read_text(encoding="utf-8", errors="ignore")
        case_row.update(parse_diag_times(stdout_text))

        sol_path = (REPO_ROOT / case_row["sol_path"]).resolve()
        state_series_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        merged_df = merge_case_outputs(sol_path, state_series_path)
        truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
        motion_frame = build_motion_frame(merged_df, truth_interp_df, imu_df, truth_reference)
        plot_frame = build_plot_frame(merged_df, truth_interp_df, truth_reference, motion_frame)
        jump_df = compute_jump_metrics(plot_frame, case_spec.case_id)
        motion_metrics_df = compute_motion_metrics(plot_frame, case_spec.case_id)

        mechanism_path = case_dir / f"SOL_{case_spec.case_id}_mechanism.csv"
        mechanism_metrics = extract_mechanism_metrics(mechanism_path, case_spec.case_id)
        product_paths = save_case_products(case_dir, case_spec, plot_frame, jump_df, motion_metrics_df)
        case_row.update(product_paths)

        case_frames[case_spec.case_id] = plot_frame
        jump_metric_frames.append(jump_df)
        motion_metric_frames.append(motion_metrics_df)
        case_metrics_rows.append(summarize_case_metrics(case_row, jump_df, motion_metrics_df, mechanism_metrics))

    case_metrics_df = pd.DataFrame(case_metrics_rows)
    jump_metrics_df = pd.concat(jump_metric_frames, ignore_index=True) if jump_metric_frames else pd.DataFrame()
    motion_metrics_df = pd.concat(motion_metric_frames, ignore_index=True) if motion_metric_frames else pd.DataFrame()

    case_metrics_path = args.output_dir / "case_metrics.csv"
    jump_metrics_path = args.output_dir / "jump_metrics.csv"
    motion_metrics_path = args.output_dir / "motion_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    jump_metrics_df.to_csv(jump_metrics_path, index=False, encoding="utf-8-sig")
    motion_metrics_df.to_csv(motion_metrics_path, index=False, encoding="utf-8-sig")

    for family_spec in FAMILY_SPECS:
        family_case_ids = [case_id for case_id in family_spec.case_ids if case_id in case_frames]
        if not family_case_ids:
            continue
        family_dir = args.family_root / family_spec.family_id
        family_plot_dir = family_dir / "plots"
        ensure_dir(family_dir)
        ensure_dir(family_plot_dir)
        plot_paths: dict[str, str] = {}

        all_overview_path = family_plot_dir / "all_states_overview.png"
        plot_column_grid(case_frames, family_case_ids, ALL_OVERVIEW_COLUMNS, all_overview_path, f"{family_spec.title}: all states")
        plot_paths["all_states_overview"] = rel_from_root(all_overview_path, REPO_ROOT)

        jump_overview_path = family_plot_dir / "all_states_jump_window.png"
        plot_column_grid(
            case_frames,
            family_case_ids,
            ALL_OVERVIEW_COLUMNS,
            jump_overview_path,
            f"{family_spec.title}: jump window",
            window=(JUMP_WINDOW_START, JUMP_WINDOW_END),
        )
        plot_paths["all_states_jump_window"] = rel_from_root(jump_overview_path, REPO_ROOT)

        vv_path = family_plot_dir / "velocity_v_frame.png"
        plot_velocity_family(case_frames, family_case_ids, vv_path, window=None, use_body_frame=False)
        plot_paths["velocity_v_frame"] = rel_from_root(vv_path, REPO_ROOT)

        vv_jump_path = family_plot_dir / "velocity_v_frame_jump_window.png"
        plot_velocity_family(
            case_frames,
            family_case_ids,
            vv_jump_path,
            window=(JUMP_WINDOW_START, JUMP_WINDOW_END),
            use_body_frame=False,
        )
        plot_paths["velocity_v_frame_jump_window"] = rel_from_root(vv_jump_path, REPO_ROOT)

        vb_jump_path = family_plot_dir / "velocity_b_frame_jump_window.png"
        plot_velocity_family(
            case_frames,
            family_case_ids,
            vb_jump_path,
            window=(JUMP_WINDOW_START, JUMP_WINDOW_END),
            use_body_frame=True,
        )
        plot_paths["velocity_b_frame_jump_window"] = rel_from_root(vb_jump_path, REPO_ROOT)

        key_jump_path = family_plot_dir / "key_jump_states.png"
        plot_key_jump_states(case_frames, family_case_ids, key_jump_path, (JUMP_WINDOW_START, JUMP_WINDOW_END))
        plot_paths["key_jump_states"] = rel_from_root(key_jump_path, REPO_ROOT)

        family_case_df = case_metrics_df.loc[case_metrics_df["case_id"].isin(family_case_ids)].copy()
        family_case_df = family_case_df.set_index("case_id").loc[family_case_ids].reset_index()
        family_jump_df = jump_metrics_df.loc[jump_metrics_df["case_id"].isin(family_case_ids)].copy()
        family_case_metrics_path = family_dir / "case_metrics.csv"
        family_jump_metrics_path = family_dir / "jump_metrics.csv"
        family_case_df.to_csv(family_case_metrics_path, index=False, encoding="utf-8-sig")
        family_jump_df.to_csv(family_jump_metrics_path, index=False, encoding="utf-8-sig")
        summary_path = family_dir / "summary.md"
        write_family_summary(summary_path, family_spec, family_case_df, family_jump_df, plot_paths)
        family_paths[family_spec.family_id] = {
            "summary": rel_from_root(summary_path, REPO_ROOT),
            "plots_dir": rel_from_root(family_plot_dir, REPO_ROOT),
        }

    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "jump_metrics_csv": rel_from_root(jump_metrics_path, REPO_ROOT),
        "motion_metrics_csv": rel_from_root(motion_metrics_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_overrides": case_overrides,
        "families": family_paths,
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_exe_mtime": mtime_text(args.exe),
            "case_metrics_mtime": mtime_text(case_metrics_path),
            "jump_metrics_mtime": mtime_text(jump_metrics_path),
            "motion_metrics_mtime": mtime_text(motion_metrics_path),
        },
    }
    write_root_summary(summary_path, manifest, case_metrics_df, family_paths)
    manifest["summary_md"] = rel_from_root(summary_path, REPO_ROOT)
    manifest["manifest_path"] = rel_from_root(manifest_path, REPO_ROOT)
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
