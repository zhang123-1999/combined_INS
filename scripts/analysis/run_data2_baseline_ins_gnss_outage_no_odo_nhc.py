from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import (
    ensure_dir,
    load_yaml,
    parse_consistency_summary,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_fullwindow_attitude_bias_coupling import (
    GROUP_SPECS,
    build_state_frame,
    compute_case_metrics,
)
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import mtime_text
from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import (
    imu_truth_model_internal,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_staged_estimation import build_periodic_gnss_windows
from scripts.analysis.run_data2_staged_g5_no_imu_scale import (
    KEY_COUPLING_STATES,
    PVA_ERROR_GROUP_SPECS,
    build_mainline_plot_config,
    compute_phase_metrics,
    format_metric,
    plot_state_grid,
    remove_obsolete_mainline_plot_files,
    render_table,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    evaluate_navigation_metrics,
    json_safe,
    run_command,
)
from scripts.analysis.run_nhc_state_convergence_research import (
    build_truth_interp,
    load_pos_dataframe,
    merge_case_outputs,
)


EXP_ID_DEFAULT = "EXP-20260326-data2-baseline-ins-gnss-outage-no-odo-nhc-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_baseline_ins_gnss_outage_no_odo_nhc")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
README_PATH_DEFAULT = Path("dataset/data2/README.md")
IMU_MODEL_DEFAULT = "POS-320"
CASE_ID_DEFAULT = "data2_baseline_ins_gnss_outage_no_odo_nhc"
PHASE3_GNSS_ON_DEFAULT = 90.0
PHASE3_GNSS_OFF_DEFAULT = 90.0
PHASE_NAME_MAP = {
    "phase1_ins_gnss_freeze_odo_states": "phase1_window",
    "phase2_ins_gnss_freeze_odo_states": "phase2_window",
    "phase3_periodic_gnss_outage_freeze_gnss_lever": "phase3_window",
}


@dataclass(frozen=True)
class PlotCase:
    case_id: str
    label: str
    color: str


@dataclass(frozen=True)
class RunnerPlotConfig:
    overview_states: tuple[Any, ...]
    group_specs: tuple[Any, ...]
    truth_keys_to_hide: set[str]


def normalize_repo_path(path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def invert_enabled_windows(
    start_time: float,
    final_time: float,
    enabled_windows: list[list[float]],
) -> list[list[float]]:
    off_windows: list[list[float]] = []
    cursor = float(start_time)
    for win_start, win_end in enabled_windows:
        start = float(win_start)
        end = float(win_end)
        if start > cursor:
            off_windows.append([cursor, start])
        cursor = max(cursor, end)
    if cursor < final_time:
        off_windows.append([cursor, float(final_time)])
    return off_windows


def case_id_for_mode(fix_gnss_lever_truth: bool) -> str:
    if fix_gnss_lever_truth:
        return f"{CASE_ID_DEFAULT}_gnss_lever_truth_fixed"
    return CASE_ID_DEFAULT


def case_label_for_mode(fix_gnss_lever_truth: bool) -> str:
    if fix_gnss_lever_truth:
        return "INS/GNSS no ODO/NHC with GNSS lever fixed to truth"
    return "INS/GNSS no ODO/NHC"


def build_ins_gnss_plot_config() -> RunnerPlotConfig:
    mainline_plot_config = build_mainline_plot_config()
    truth_keys_to_hide = set(mainline_plot_config.truth_keys_to_hide)
    truth_keys_to_hide.update(
        {
            "sg_x_ppm",
            "sg_y_ppm",
            "sg_z_ppm",
            "sa_x_ppm",
            "sa_y_ppm",
            "sa_z_ppm",
        }
    )
    return RunnerPlotConfig(
        overview_states=tuple(state for group in GROUP_SPECS for state in group.states),
        group_specs=GROUP_SPECS,
        truth_keys_to_hide=truth_keys_to_hide,
    )


def readme_white_noise_internal(readme_params: dict[str, Any]) -> tuple[float, float]:
    sigma_gyro = math.radians(float(readme_params["sigma_arw_deg_sqrt_hr"])) / math.sqrt(3600.0)
    sigma_acc = float(readme_params["sigma_vrw_mps_sqrt_hr"]) / math.sqrt(3600.0)
    return sigma_acc, sigma_gyro


def sigma_bg_internal_from_degh(sigma_bg_degh: float) -> float:
    sigma_bg_degh = float(sigma_bg_degh)
    if sigma_bg_degh <= 0.0:
        raise ValueError("sigma_bg_degh_override must be positive")
    return math.radians(sigma_bg_degh) / 3600.0


def sigma_ba_internal_from_mgal(sigma_ba_mgal: float) -> float:
    sigma_ba_mgal = float(sigma_ba_mgal)
    if sigma_ba_mgal <= 0.0:
        raise ValueError("sigma_ba_mgal_override must be positive")
    return sigma_ba_mgal * 1.0e-5


def sigma_scale_internal_from_ppm(sigma_scale_ppm: float) -> float:
    sigma_scale_ppm = float(sigma_scale_ppm)
    if sigma_scale_ppm <= 0.0:
        raise ValueError("sigma_scale_ppm_override must be positive")
    return sigma_scale_ppm * 1.0e-6


def sigma_gyro_white_internal_from_deg_sqrt_hr(sigma_gyro_deg_sqrt_hr: float) -> float:
    sigma_gyro_deg_sqrt_hr = float(sigma_gyro_deg_sqrt_hr)
    if sigma_gyro_deg_sqrt_hr <= 0.0:
        raise ValueError("sigma_gyro_deg_sqrt_hr_override must be positive")
    return math.radians(sigma_gyro_deg_sqrt_hr) / math.sqrt(3600.0)


def sigma_acc_white_internal_from_mps_sqrt_hr(sigma_acc_mps_sqrt_hr: float) -> float:
    sigma_acc_mps_sqrt_hr = float(sigma_acc_mps_sqrt_hr)
    if sigma_acc_mps_sqrt_hr <= 0.0:
        raise ValueError("sigma_acc_mps_sqrt_hr_override must be positive")
    return sigma_acc_mps_sqrt_hr / math.sqrt(3600.0)


def apply_readme_imu_contract(cfg: dict[str, Any], readme_path: Path, imu_model: str) -> dict[str, Any]:
    readme_params = parse_data2_readme_imu_params(readme_path.resolve(), imu_model)
    readme_internal = imu_truth_model_internal(readme_params)
    sigma_acc, sigma_gyro = readme_white_noise_internal(readme_params)

    fusion = cfg.setdefault("fusion", {})
    noise = fusion.setdefault("noise", {})
    init_cfg = fusion.setdefault("init", {})
    ablation = fusion.setdefault("ablation", {})

    noise["sigma_acc"] = float(sigma_acc)
    noise["sigma_gyro"] = float(sigma_gyro)
    for group in ["ba", "bg", "sg", "sa"]:
        sigma_value = float(readme_internal[f"sigma_{group}"])
        noise[f"sigma_{group}"] = sigma_value
        noise[f"sigma_{group}_vec"] = [sigma_value, sigma_value, sigma_value]
    noise["markov_corr_time"] = float(readme_internal["corr_time_s"])

    ablation["disable_gyro_scale"] = False
    ablation["disable_accel_scale"] = False

    for field in ["ba0", "bg0", "sg0", "sa0", "std_ba", "std_bg", "std_sg", "std_sa"]:
        init_cfg.pop(field, None)

    p0_diag = list(init_cfg.get("P0_diag", []))
    if len(p0_diag) < 21:
        raise RuntimeError("init.P0_diag is missing IMU state slots required for README-driven INS/GNSS contract")
    p0_diag[9:12] = [float(readme_internal["sigma_ba"] ** 2)] * 3
    p0_diag[12:15] = [float(readme_internal["sigma_bg"] ** 2)] * 3
    p0_diag[15:18] = [float(readme_internal["sigma_sg"] ** 2)] * 3
    p0_diag[18:21] = [float(readme_internal["sigma_sa"] ** 2)] * 3
    init_cfg["P0_diag"] = p0_diag

    return {
        "readme_path": rel_from_root(readme_path.resolve(), REPO_ROOT),
        "imu_model": imu_model,
        "sigma_acc": float(sigma_acc),
        "sigma_gyro": float(sigma_gyro),
        "sigma_ba": float(readme_internal["sigma_ba"]),
        "sigma_bg": float(readme_internal["sigma_bg"]),
        "sigma_sg": float(readme_internal["sigma_sg"]),
        "sigma_sa": float(readme_internal["sigma_sa"]),
        "markov_corr_time": float(readme_internal["corr_time_s"]),
    }


def apply_bias_overrides(
    cfg: dict[str, Any],
    sigma_bg_degh_override: float | None = None,
    sigma_ba_mgal_override: float | None = None,
    process_sigma_bg_degh_override: float | None = None,
    process_sigma_ba_mgal_override: float | None = None,
    init_std_bg_degh_override: float | None = None,
    init_std_ba_mgal_override: float | None = None,
) -> dict[str, Any]:
    effective_process_sigma_bg_degh = (
        process_sigma_bg_degh_override if process_sigma_bg_degh_override is not None else sigma_bg_degh_override
    )
    effective_process_sigma_ba_mgal = (
        process_sigma_ba_mgal_override if process_sigma_ba_mgal_override is not None else sigma_ba_mgal_override
    )
    effective_init_std_bg_degh = (
        init_std_bg_degh_override if init_std_bg_degh_override is not None else sigma_bg_degh_override
    )
    effective_init_std_ba_mgal = (
        init_std_ba_mgal_override if init_std_ba_mgal_override is not None else sigma_ba_mgal_override
    )

    if (
        effective_process_sigma_bg_degh is None
        and effective_process_sigma_ba_mgal is None
        and effective_init_std_bg_degh is None
        and effective_init_std_ba_mgal is None
    ):
        return {}

    fusion = cfg.setdefault("fusion", {})
    noise = fusion.setdefault("noise", {})
    init_cfg = fusion.setdefault("init", {})
    p0_diag = list(init_cfg.get("P0_diag", []))
    if len(p0_diag) < 15:
        raise RuntimeError("init.P0_diag is missing ba/bg slots required for INS/GNSS bias override contract")

    metadata: dict[str, Any] = {}
    if sigma_ba_mgal_override is not None:
        metadata["sigma_ba_mgal_override"] = float(sigma_ba_mgal_override)
    if sigma_bg_degh_override is not None:
        metadata["sigma_bg_degh_override"] = float(sigma_bg_degh_override)
    if process_sigma_ba_mgal_override is not None:
        metadata["process_sigma_ba_mgal_override"] = float(process_sigma_ba_mgal_override)
    if process_sigma_bg_degh_override is not None:
        metadata["process_sigma_bg_degh_override"] = float(process_sigma_bg_degh_override)
    if init_std_ba_mgal_override is not None:
        metadata["init_std_ba_mgal_override"] = float(init_std_ba_mgal_override)
    if init_std_bg_degh_override is not None:
        metadata["init_std_bg_degh_override"] = float(init_std_bg_degh_override)

    if effective_process_sigma_ba_mgal is not None:
        sigma_ba = sigma_ba_internal_from_mgal(effective_process_sigma_ba_mgal)
        noise["sigma_ba"] = sigma_ba
        noise["sigma_ba_vec"] = [sigma_ba, sigma_ba, sigma_ba]
        metadata["sigma_ba"] = sigma_ba
    if effective_process_sigma_bg_degh is not None:
        sigma_bg = sigma_bg_internal_from_degh(effective_process_sigma_bg_degh)
        noise["sigma_bg"] = sigma_bg
        noise["sigma_bg_vec"] = [sigma_bg, sigma_bg, sigma_bg]
        metadata["sigma_bg"] = sigma_bg
    if effective_init_std_ba_mgal is not None:
        init_std_ba = sigma_ba_internal_from_mgal(effective_init_std_ba_mgal)
        p0_diag[9:12] = [init_std_ba**2] * 3
        metadata["init_std_ba"] = init_std_ba
    if effective_init_std_bg_degh is not None:
        init_std_bg = sigma_bg_internal_from_degh(effective_init_std_bg_degh)
        p0_diag[12:15] = [init_std_bg**2] * 3
        metadata["init_std_bg"] = init_std_bg
    init_cfg["P0_diag"] = p0_diag
    return metadata


def apply_scale_factor_overrides(
    cfg: dict[str, Any],
    sigma_sg_ppm_override: float | None = None,
    sigma_sa_ppm_override: float | None = None,
    process_sigma_sg_ppm_override: float | None = None,
    process_sigma_sa_ppm_override: float | None = None,
    init_std_sg_ppm_override: float | None = None,
    init_std_sa_ppm_override: float | None = None,
) -> dict[str, Any]:
    effective_process_sigma_sg_ppm = (
        process_sigma_sg_ppm_override if process_sigma_sg_ppm_override is not None else sigma_sg_ppm_override
    )
    effective_process_sigma_sa_ppm = (
        process_sigma_sa_ppm_override if process_sigma_sa_ppm_override is not None else sigma_sa_ppm_override
    )
    effective_init_std_sg_ppm = (
        init_std_sg_ppm_override if init_std_sg_ppm_override is not None else sigma_sg_ppm_override
    )
    effective_init_std_sa_ppm = (
        init_std_sa_ppm_override if init_std_sa_ppm_override is not None else sigma_sa_ppm_override
    )

    if (
        effective_process_sigma_sg_ppm is None
        and effective_process_sigma_sa_ppm is None
        and effective_init_std_sg_ppm is None
        and effective_init_std_sa_ppm is None
    ):
        return {}

    fusion = cfg.setdefault("fusion", {})
    noise = fusion.setdefault("noise", {})
    init_cfg = fusion.setdefault("init", {})
    p0_diag = list(init_cfg.get("P0_diag", []))
    if len(p0_diag) < 21:
        raise RuntimeError("init.P0_diag is missing sg/sa slots required for INS/GNSS scale-factor override contract")

    metadata: dict[str, Any] = {}
    if sigma_sg_ppm_override is not None:
        metadata["sigma_sg_ppm_override"] = float(sigma_sg_ppm_override)
    if sigma_sa_ppm_override is not None:
        metadata["sigma_sa_ppm_override"] = float(sigma_sa_ppm_override)
    if process_sigma_sg_ppm_override is not None:
        metadata["process_sigma_sg_ppm_override"] = float(process_sigma_sg_ppm_override)
    if process_sigma_sa_ppm_override is not None:
        metadata["process_sigma_sa_ppm_override"] = float(process_sigma_sa_ppm_override)
    if init_std_sg_ppm_override is not None:
        metadata["init_std_sg_ppm_override"] = float(init_std_sg_ppm_override)
    if init_std_sa_ppm_override is not None:
        metadata["init_std_sa_ppm_override"] = float(init_std_sa_ppm_override)

    if effective_process_sigma_sg_ppm is not None:
        sigma_sg = sigma_scale_internal_from_ppm(effective_process_sigma_sg_ppm)
        noise["sigma_sg"] = sigma_sg
        noise["sigma_sg_vec"] = [sigma_sg, sigma_sg, sigma_sg]
        metadata["sigma_sg"] = sigma_sg
    if effective_process_sigma_sa_ppm is not None:
        sigma_sa = sigma_scale_internal_from_ppm(effective_process_sigma_sa_ppm)
        noise["sigma_sa"] = sigma_sa
        noise["sigma_sa_vec"] = [sigma_sa, sigma_sa, sigma_sa]
        metadata["sigma_sa"] = sigma_sa
    if effective_init_std_sg_ppm is not None:
        init_std_sg = sigma_scale_internal_from_ppm(effective_init_std_sg_ppm)
        p0_diag[15:18] = [init_std_sg**2] * 3
        metadata["init_std_sg"] = init_std_sg
    if effective_init_std_sa_ppm is not None:
        init_std_sa = sigma_scale_internal_from_ppm(effective_init_std_sa_ppm)
        p0_diag[18:21] = [init_std_sa**2] * 3
        metadata["init_std_sa"] = init_std_sa

    init_cfg["P0_diag"] = p0_diag
    return metadata


def build_phase3_runtime_bias_process_override(
    phase3_process_sigma_bg_degh_override: float | None = None,
    phase3_process_sigma_ba_mgal_override: float | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    phase3_noise_override: dict[str, Any] = {}
    metadata: dict[str, Any] = {}

    if phase3_process_sigma_ba_mgal_override is not None:
        sigma_ba = sigma_ba_internal_from_mgal(phase3_process_sigma_ba_mgal_override)
        phase3_noise_override["sigma_ba"] = sigma_ba
        phase3_noise_override["sigma_ba_vec"] = [sigma_ba, sigma_ba, sigma_ba]
        metadata["phase3_process_sigma_ba_mgal_override"] = float(phase3_process_sigma_ba_mgal_override)
        metadata["phase3_process_sigma_ba"] = sigma_ba
    if phase3_process_sigma_bg_degh_override is not None:
        sigma_bg = sigma_bg_internal_from_degh(phase3_process_sigma_bg_degh_override)
        phase3_noise_override["sigma_bg"] = sigma_bg
        phase3_noise_override["sigma_bg_vec"] = [sigma_bg, sigma_bg, sigma_bg]
        metadata["phase3_process_sigma_bg_degh_override"] = float(phase3_process_sigma_bg_degh_override)
        metadata["phase3_process_sigma_bg"] = sigma_bg

    return phase3_noise_override, metadata


def build_runtime_noise_override(
    prefix: str,
    sigma_acc_mps_sqrt_hr_override: float | None = None,
    sigma_gyro_deg_sqrt_hr_override: float | None = None,
    process_sigma_bg_degh_override: float | None = None,
    process_sigma_ba_mgal_override: float | None = None,
    process_sigma_sg_ppm_override: float | None = None,
    process_sigma_sa_ppm_override: float | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    noise_override: dict[str, Any] = {}
    metadata: dict[str, Any] = {}

    if sigma_acc_mps_sqrt_hr_override is not None:
        sigma_acc = sigma_acc_white_internal_from_mps_sqrt_hr(sigma_acc_mps_sqrt_hr_override)
        noise_override["sigma_acc"] = sigma_acc
        metadata[f"{prefix}_sigma_acc_mps_sqrt_hr_override"] = float(sigma_acc_mps_sqrt_hr_override)
        metadata[f"{prefix}_sigma_acc"] = sigma_acc
    if sigma_gyro_deg_sqrt_hr_override is not None:
        sigma_gyro = sigma_gyro_white_internal_from_deg_sqrt_hr(sigma_gyro_deg_sqrt_hr_override)
        noise_override["sigma_gyro"] = sigma_gyro
        metadata[f"{prefix}_sigma_gyro_deg_sqrt_hr_override"] = float(sigma_gyro_deg_sqrt_hr_override)
        metadata[f"{prefix}_sigma_gyro"] = sigma_gyro
    if process_sigma_ba_mgal_override is not None:
        sigma_ba = sigma_ba_internal_from_mgal(process_sigma_ba_mgal_override)
        noise_override["sigma_ba"] = sigma_ba
        noise_override["sigma_ba_vec"] = [sigma_ba, sigma_ba, sigma_ba]
        metadata[f"{prefix}_process_sigma_ba_mgal_override"] = float(process_sigma_ba_mgal_override)
        metadata[f"{prefix}_process_sigma_ba"] = sigma_ba
    if process_sigma_bg_degh_override is not None:
        sigma_bg = sigma_bg_internal_from_degh(process_sigma_bg_degh_override)
        noise_override["sigma_bg"] = sigma_bg
        noise_override["sigma_bg_vec"] = [sigma_bg, sigma_bg, sigma_bg]
        metadata[f"{prefix}_process_sigma_bg_degh_override"] = float(process_sigma_bg_degh_override)
        metadata[f"{prefix}_process_sigma_bg"] = sigma_bg
    if process_sigma_sg_ppm_override is not None:
        sigma_sg = sigma_scale_internal_from_ppm(process_sigma_sg_ppm_override)
        noise_override["sigma_sg"] = sigma_sg
        noise_override["sigma_sg_vec"] = [sigma_sg, sigma_sg, sigma_sg]
        metadata[f"{prefix}_process_sigma_sg_ppm_override"] = float(process_sigma_sg_ppm_override)
        metadata[f"{prefix}_process_sigma_sg"] = sigma_sg
    if process_sigma_sa_ppm_override is not None:
        sigma_sa = sigma_scale_internal_from_ppm(process_sigma_sa_ppm_override)
        noise_override["sigma_sa"] = sigma_sa
        noise_override["sigma_sa_vec"] = [sigma_sa, sigma_sa, sigma_sa]
        metadata[f"{prefix}_process_sigma_sa_ppm_override"] = float(process_sigma_sa_ppm_override)
        metadata[f"{prefix}_process_sigma_sa"] = sigma_sa

    return noise_override, metadata


def apply_fixed_gnss_lever_truth_contract(cfg: dict[str, Any]) -> None:
    fusion = cfg.setdefault("fusion", {})
    noise = fusion.setdefault("noise", {})
    init_cfg = fusion.setdefault("init", {})
    ablation = fusion.setdefault("ablation", {})

    ablation["disable_gnss_lever_arm"] = True
    noise["sigma_gnss_lever_arm"] = 0.0
    noise["sigma_gnss_lever_arm_vec"] = [0.0, 0.0, 0.0]
    init_cfg["std_gnss_lever_arm"] = [0.0, 0.0, 0.0]

    p0_diag = list(init_cfg.get("P0_diag", []))
    if len(p0_diag) < 31:
        raise RuntimeError("init.P0_diag is missing GNSS lever slots required for fixed-truth contract")
    p0_diag[28:31] = [0.0, 0.0, 0.0]
    init_cfg["P0_diag"] = p0_diag


def extract_baseline_transition_times(cfg: dict[str, Any]) -> tuple[float, float, float, float]:
    fusion = cfg["fusion"]
    runtime_phases = {str(phase["name"]): phase for phase in fusion.get("runtime_phases", [])}
    phase1 = runtime_phases.get("phase1_ins_gnss_freeze_mounting_odo_states")
    phase2_seed = runtime_phases.get("phase2_cov_seed")
    phase3 = runtime_phases.get("phase3_periodic_gnss_outage_freeze_calibrated_extrinsics")
    if phase1 is None or phase2_seed is None or phase3 is None:
        raise RuntimeError("missing canonical baseline runtime phases required to derive INS/GNSS outage windows")
    start_time = float(fusion["starttime"])
    phase1_end_time = float(phase1["end_time"])
    phase2_end_time = float(phase3["start_time"])
    final_time = float(fusion["finaltime"])
    if abs(phase1_end_time - float(phase2_seed["start_time"])) > 1.0e-6:
        raise RuntimeError("phase2 seed start is inconsistent with phase1 end in base config")
    return start_time, phase1_end_time, phase2_end_time, final_time


def build_runtime_phases(
    start_time: float,
    phase1_end_time: float,
    phase2_end_time: float,
    final_time: float,
    phase12_noise_override: dict[str, Any] | None = None,
    phase3_noise_override: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    phases = [
        {
            "name": "phase1_ins_gnss_freeze_odo_states",
            "start_time": float(start_time),
            "end_time": float(phase1_end_time),
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
                "disable_gnss_lever_arm": False,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        },
        {
            "name": "phase2_ins_gnss_freeze_odo_states",
            "start_time": float(phase1_end_time),
            "end_time": float(phase2_end_time),
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
                "disable_gnss_lever_arm": False,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        },
        {
            "name": "phase3_periodic_gnss_outage_freeze_gnss_lever",
            "start_time": float(phase2_end_time),
            "end_time": float(final_time),
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
                "disable_gnss_lever_arm": True,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        },
    ]
    if phase12_noise_override:
        phases[0]["noise"] = copy.deepcopy(phase12_noise_override)
        phases[1]["noise"] = copy.deepcopy(phase12_noise_override)
    if phase3_noise_override:
        phases[2]["noise"] = copy.deepcopy(phase3_noise_override)
    return phases


def extract_phase_windows(cfg: dict[str, Any]) -> dict[str, list[float]]:
    runtime_phases = cfg["fusion"].get("runtime_phases", [])
    windows: dict[str, list[float]] = {}
    for phase in runtime_phases:
        phase_name = str(phase["name"])
        mapped = PHASE_NAME_MAP.get(phase_name)
        if mapped is None:
            continue
        windows[mapped] = [float(phase["start_time"]), float(phase["end_time"])]
    missing = [field for field in PHASE_NAME_MAP.values() if field not in windows]
    if missing:
        raise RuntimeError(f"missing INS/GNSS outage runtime phase windows: {missing}")
    return windows


def extract_run_metadata(
    cfg: dict[str, Any],
    case_id: str,
    fix_gnss_lever_truth: bool,
    phase3_gnss_on: float,
    phase3_gnss_off: float,
) -> dict[str, Any]:
    fusion = cfg["fusion"]
    enabled_windows = [
        [float(window["start_time"]), float(window["end_time"])]
        for window in fusion["gnss_schedule"]["enabled_windows"]
    ]
    metadata = {
        "case_id": case_id,
        "fix_gnss_lever_truth": bool(fix_gnss_lever_truth),
        "phase3_gnss_on_s": float(phase3_gnss_on),
        "phase3_gnss_off_s": float(phase3_gnss_off),
        **extract_phase_windows(cfg),
        "gnss_on_windows": enabled_windows,
        "gnss_off_windows": invert_enabled_windows(
            float(fusion["starttime"]),
            float(fusion["finaltime"]),
            enabled_windows,
        ),
    }
    return metadata


def build_run_config(
    base_cfg: dict[str, Any],
    output_dir: Path,
    readme_path: Path = README_PATH_DEFAULT,
    imu_model: str = IMU_MODEL_DEFAULT,
    fix_gnss_lever_truth: bool = False,
    sigma_bg_degh_override: float | None = None,
    sigma_ba_mgal_override: float | None = None,
    process_sigma_bg_degh_override: float | None = None,
    process_sigma_ba_mgal_override: float | None = None,
    init_std_bg_degh_override: float | None = None,
    init_std_ba_mgal_override: float | None = None,
    sigma_sg_ppm_override: float | None = None,
    sigma_sa_ppm_override: float | None = None,
    process_sigma_sg_ppm_override: float | None = None,
    process_sigma_sa_ppm_override: float | None = None,
    init_std_sg_ppm_override: float | None = None,
    init_std_sa_ppm_override: float | None = None,
    phase12_process_sigma_bg_degh_override: float | None = None,
    phase12_process_sigma_ba_mgal_override: float | None = None,
    phase12_process_sigma_sg_ppm_override: float | None = None,
    phase12_process_sigma_sa_ppm_override: float | None = None,
    phase3_sigma_acc_mps_sqrt_hr_override: float | None = None,
    phase3_sigma_gyro_deg_sqrt_hr_override: float | None = None,
    phase3_process_sigma_bg_degh_override: float | None = None,
    phase3_process_sigma_ba_mgal_override: float | None = None,
    phase3_gnss_on: float = PHASE3_GNSS_ON_DEFAULT,
    phase3_gnss_off: float = PHASE3_GNSS_OFF_DEFAULT,
    enable_gnss_update_debug: bool = False,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    output_dir_abs = normalize_repo_path(output_dir)
    fusion = cfg.setdefault("fusion", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    case_id = case_id_for_mode(fix_gnss_lever_truth)
    start_time, phase1_end_time, phase2_end_time, final_time = extract_baseline_transition_times(cfg)

    fusion["output_path"] = rel_from_root(output_dir_abs / f"SOL_{case_id}.txt", REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(
        output_dir_abs / f"state_series_{case_id}.csv",
        REPO_ROOT,
    )
    if enable_gnss_update_debug:
        fusion["gnss_update_debug_output_path"] = rel_from_root(
            output_dir_abs / "artifacts" / "cases" / case_id / f"gnss_updates_{case_id}.csv",
            REPO_ROOT,
        )
    else:
        fusion.pop("gnss_update_debug_output_path", None)
    fusion.pop("post_gnss_ablation", None)

    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_nhc"] = False

    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_mounting"] = True
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_gnss_lever_arm"] = False
    imu_contract = apply_readme_imu_contract(cfg, normalize_repo_path(readme_path), imu_model)
    bias_override_contract = apply_bias_overrides(
        cfg,
        sigma_bg_degh_override=sigma_bg_degh_override,
        sigma_ba_mgal_override=sigma_ba_mgal_override,
        process_sigma_bg_degh_override=process_sigma_bg_degh_override,
        process_sigma_ba_mgal_override=process_sigma_ba_mgal_override,
        init_std_bg_degh_override=init_std_bg_degh_override,
        init_std_ba_mgal_override=init_std_ba_mgal_override,
    )
    scale_factor_override_contract = apply_scale_factor_overrides(
        cfg,
        sigma_sg_ppm_override=sigma_sg_ppm_override,
        sigma_sa_ppm_override=sigma_sa_ppm_override,
        process_sigma_sg_ppm_override=process_sigma_sg_ppm_override,
        process_sigma_sa_ppm_override=process_sigma_sa_ppm_override,
        init_std_sg_ppm_override=init_std_sg_ppm_override,
        init_std_sa_ppm_override=init_std_sa_ppm_override,
    )
    phase12_noise_override, phase12_noise_metadata = build_runtime_noise_override(
        "phase12",
        process_sigma_bg_degh_override=phase12_process_sigma_bg_degh_override,
        process_sigma_ba_mgal_override=phase12_process_sigma_ba_mgal_override,
        process_sigma_sg_ppm_override=phase12_process_sigma_sg_ppm_override,
        process_sigma_sa_ppm_override=phase12_process_sigma_sa_ppm_override,
    )
    phase3_nav_noise_override, phase3_nav_noise_metadata = build_runtime_noise_override(
        "phase3",
        sigma_acc_mps_sqrt_hr_override=phase3_sigma_acc_mps_sqrt_hr_override,
        sigma_gyro_deg_sqrt_hr_override=phase3_sigma_gyro_deg_sqrt_hr_override,
    )
    phase3_noise_override, phase3_noise_metadata = build_phase3_runtime_bias_process_override(
        phase3_process_sigma_bg_degh_override=phase3_process_sigma_bg_degh_override,
        phase3_process_sigma_ba_mgal_override=phase3_process_sigma_ba_mgal_override,
    )
    phase3_noise_override = {**phase3_nav_noise_override, **phase3_noise_override}

    enabled_windows, _ = build_periodic_gnss_windows(
        start_time=start_time,
        final_time=final_time,
        phase2_end_time=phase2_end_time,
        phase3_on_duration=phase3_gnss_on,
        phase3_off_duration=phase3_gnss_off,
    )
    fusion["gnss_schedule"] = {
        "enabled": True,
        "enabled_windows": [
            {"start_time": float(win_start), "end_time": float(win_end)}
            for win_start, win_end in enabled_windows
        ],
    }
    fusion["runtime_phases"] = build_runtime_phases(
        start_time,
        phase1_end_time,
        phase2_end_time,
        final_time,
        phase12_noise_override=phase12_noise_override,
        phase3_noise_override=phase3_noise_override,
    )
    if fix_gnss_lever_truth:
        apply_fixed_gnss_lever_truth_contract(cfg)
        for phase in fusion["runtime_phases"]:
            phase.setdefault("ablation", {})["disable_gnss_lever_arm"] = True

    metadata = extract_run_metadata(
        cfg,
        case_id,
        fix_gnss_lever_truth,
        phase3_gnss_on=phase3_gnss_on,
        phase3_gnss_off=phase3_gnss_off,
    )
    metadata.update(imu_contract)
    metadata.update(bias_override_contract)
    metadata.update(scale_factor_override_contract)
    metadata.update(phase12_noise_metadata)
    metadata.update(phase3_nav_noise_metadata)
    metadata.update(phase3_noise_metadata)
    metadata["enable_gnss_update_debug"] = bool(enable_gnss_update_debug)
    if enable_gnss_update_debug:
        metadata["gnss_update_debug_output_path"] = fusion["gnss_update_debug_output_path"]
    return cfg, metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the data2 INS/GNSS baseline with no ODO/NHC and phase3 GNSS outage plus frozen GNSS lever."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--fix-gnss-lever-truth", action="store_true")
    parser.add_argument("--sigma-bg-degh-override", type=float, default=None)
    parser.add_argument("--sigma-ba-mgal-override", type=float, default=None)
    parser.add_argument("--process-sigma-bg-degh-override", type=float, default=None)
    parser.add_argument("--process-sigma-ba-mgal-override", type=float, default=None)
    parser.add_argument("--init-std-bg-degh-override", type=float, default=None)
    parser.add_argument("--init-std-ba-mgal-override", type=float, default=None)
    parser.add_argument("--sigma-sg-ppm-override", type=float, default=None)
    parser.add_argument("--sigma-sa-ppm-override", type=float, default=None)
    parser.add_argument("--process-sigma-sg-ppm-override", type=float, default=None)
    parser.add_argument("--process-sigma-sa-ppm-override", type=float, default=None)
    parser.add_argument("--init-std-sg-ppm-override", type=float, default=None)
    parser.add_argument("--init-std-sa-ppm-override", type=float, default=None)
    parser.add_argument("--phase12-process-sigma-bg-degh-override", type=float, default=None)
    parser.add_argument("--phase12-process-sigma-ba-mgal-override", type=float, default=None)
    parser.add_argument("--phase12-process-sigma-sg-ppm-override", type=float, default=None)
    parser.add_argument("--phase12-process-sigma-sa-ppm-override", type=float, default=None)
    parser.add_argument("--phase3-sigma-acc-mps-sqrt-hr-override", type=float, default=None)
    parser.add_argument("--phase3-sigma-gyro-deg-sqrt-hr-override", type=float, default=None)
    parser.add_argument("--phase3-process-sigma-bg-degh-override", type=float, default=None)
    parser.add_argument("--phase3-process-sigma-ba-mgal-override", type=float, default=None)
    parser.add_argument("--phase3-gnss-on", type=float, default=PHASE3_GNSS_ON_DEFAULT)
    parser.add_argument("--phase3-gnss-off", type=float, default=PHASE3_GNSS_OFF_DEFAULT)
    parser.add_argument("--enable-gnss-update-debug", action="store_true")
    args = parser.parse_args()
    args.base_config = normalize_repo_path(args.base_config)
    args.exe = normalize_repo_path(args.exe)
    args.output_dir = normalize_repo_path(args.output_dir)
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_id = case_id_for_mode(args.fix_gnss_lever_truth)
    args.case_label = case_label_for_mode(args.fix_gnss_lever_truth)
    args.case_root = args.artifacts_dir / "cases" / args.case_id
    args.plot_dir = args.output_dir / "plots"
    return args


def run_case(cfg_path: Path, output_dir: Path, case_dir: Path, exe_path: Path, case_id: str, case_label: str) -> dict[str, Any]:
    sol_path = output_dir / f"SOL_{case_id}.txt"
    state_series_path = output_dir / f"state_series_{case_id}.csv"
    stdout_path = case_dir / f"solver_stdout_{case_id}.txt"
    diag_path = case_dir / f"DIAG_{case_id}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output: {state_series_path}")
    if not root_diag.exists():
        raise RuntimeError("missing DIAG.txt after INS/GNSS outage run")
    shutil.copy2(root_diag, diag_path)

    nav_metrics, segment_rows = evaluate_navigation_metrics(cfg_path, sol_path)
    row: dict[str, Any] = {
        "case_id": case_id,
        "case_label": case_label,
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
    for sensor_name, metrics in parse_consistency_summary(stdout_text).items():
        prefix = sensor_name.lower()
        for metric_name, metric_value in metrics.items():
            row[f"{prefix}_{metric_name}"] = float(metric_value)
    return row


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    phase_metrics_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    case_rows = [
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
        "# data2 INS/GNSS outage without ODO/NHC summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- case_id: `{manifest['case_id']}`",
        (
            f"- phase windows: `phase1={manifest['phase_windows']['phase1']}`, "
            f"`phase2={manifest['phase_windows']['phase2']}`, "
            f"`phase3={manifest['phase_windows']['phase3']}`"
        ),
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
                "rmse3d_m",
                "p95_3d_m",
                "final_3d_m",
                "yaw_err_max_abs_deg",
                "bg_z_err_max_abs_degh",
                "odo_accept_ratio",
                "nhc_accept_ratio",
            ],
            case_rows,
        )
    )
    lines.extend(["", "## Phase Metrics"])
    lines.extend(
        render_table(
            ["case_id", "window", "rmse3d_m", "p95_3d_m", "final_3d_m", "yaw_abs_max_deg", "bg_z_abs_max_degh"],
            phase_rows,
        )
    )
    if plot_paths:
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

    ensure_dir(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    cfg, metadata = build_run_config(
        base_cfg,
        args.output_dir,
        fix_gnss_lever_truth=args.fix_gnss_lever_truth,
        sigma_bg_degh_override=args.sigma_bg_degh_override,
        sigma_ba_mgal_override=args.sigma_ba_mgal_override,
        process_sigma_bg_degh_override=args.process_sigma_bg_degh_override,
        process_sigma_ba_mgal_override=args.process_sigma_ba_mgal_override,
        init_std_bg_degh_override=args.init_std_bg_degh_override,
        init_std_ba_mgal_override=args.init_std_ba_mgal_override,
        sigma_sg_ppm_override=args.sigma_sg_ppm_override,
        sigma_sa_ppm_override=args.sigma_sa_ppm_override,
        process_sigma_sg_ppm_override=args.process_sigma_sg_ppm_override,
        process_sigma_sa_ppm_override=args.process_sigma_sa_ppm_override,
        init_std_sg_ppm_override=args.init_std_sg_ppm_override,
        init_std_sa_ppm_override=args.init_std_sa_ppm_override,
        phase12_process_sigma_bg_degh_override=args.phase12_process_sigma_bg_degh_override,
        phase12_process_sigma_ba_mgal_override=args.phase12_process_sigma_ba_mgal_override,
        phase12_process_sigma_sg_ppm_override=args.phase12_process_sigma_sg_ppm_override,
        phase12_process_sigma_sa_ppm_override=args.phase12_process_sigma_sa_ppm_override,
        phase3_sigma_acc_mps_sqrt_hr_override=args.phase3_sigma_acc_mps_sqrt_hr_override,
        phase3_sigma_gyro_deg_sqrt_hr_override=args.phase3_sigma_gyro_deg_sqrt_hr_override,
        phase3_process_sigma_bg_degh_override=args.phase3_process_sigma_bg_degh_override,
        phase3_process_sigma_ba_mgal_override=args.phase3_process_sigma_ba_mgal_override,
        phase3_gnss_on=args.phase3_gnss_on,
        phase3_gnss_off=args.phase3_gnss_off,
        enable_gnss_update_debug=args.enable_gnss_update_debug,
    )

    cfg_path = args.case_root / f"config_{args.case_id}.yaml"
    save_yaml(cfg, cfg_path)

    truth_reference = build_truth_reference(cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    case_row = run_case(cfg_path, args.output_dir, args.case_root, args.exe, args.case_id, args.case_label)
    sol_path = args.output_dir / f"SOL_{args.case_id}.txt"
    state_series_path = args.output_dir / f"state_series_{args.case_id}.csv"
    merged_df = merge_case_outputs(sol_path, state_series_path)

    truth_df = load_pos_dataframe((REPO_ROOT / cfg["fusion"]["pos_path"]).resolve())
    truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
    state_frame = build_state_frame(merged_df, truth_interp_df, truth_reference)

    all_states_path = args.case_root / f"all_states_{args.case_id}.csv"
    state_frame.to_csv(all_states_path, index=False, encoding="utf-8-sig")

    phase_windows = [
        ("phase1_ins_gnss", *metadata["phase1_window"]),
        ("phase2_ins_gnss", *metadata["phase2_window"]),
        ("phase3_periodic_gnss_outage", *metadata["phase3_window"]),
    ]
    gnss_off_windows = [tuple(window) for window in metadata["gnss_off_windows"]]
    phase_metrics_df = compute_phase_metrics(state_frame, args.case_id, phase_windows, gnss_off_windows)

    metrics_row = compute_case_metrics(case_row, state_frame)
    metrics_row["config_mtime"] = mtime_text(cfg_path)
    metrics_row["all_states_path"] = rel_from_root(all_states_path, REPO_ROOT)
    metrics_row["all_states_mtime"] = mtime_text(all_states_path)
    case_metrics_df = pd.DataFrame([metrics_row])

    case_metrics_path = args.output_dir / "case_metrics.csv"
    phase_metrics_path = args.output_dir / "phase_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    phase_metrics_df.to_csv(phase_metrics_path, index=False, encoding="utf-8-sig")

    plot_case = PlotCase(case_id=args.case_id, label=args.case_label, color="#d62728")
    case_frames = {args.case_id: state_frame}
    plot_paths: dict[str, str] = {}
    plot_config = build_ins_gnss_plot_config()

    overview_path = args.plot_dir / "all_states_overview.png"
    plot_state_grid(
        case_frames,
        [plot_case],
        plot_config.overview_states,
        overview_path,
        f"data2 {args.case_label} all-state overview",
        metadata["phase1_window"][1],
        metadata["phase3_window"][0],
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["all_states_overview"] = rel_from_root(overview_path, REPO_ROOT)

    key_path = args.plot_dir / "key_coupling_states.png"
    plot_state_grid(
        case_frames,
        [plot_case],
        KEY_COUPLING_STATES,
        key_path,
        f"data2 {args.case_label} key coupling states",
        metadata["phase1_window"][1],
        metadata["phase3_window"][0],
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["key_coupling_states"] = rel_from_root(key_path, REPO_ROOT)

    for group_spec in plot_config.group_specs:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        selected_group = group_spec
        plot_mode = "state"
        if group_spec.group_id in {"position", "velocity", "attitude"}:
            plot_mode = "error"
            selected_group = next(item for item in PVA_ERROR_GROUP_SPECS if item.group_id == group_spec.group_id)
        plot_state_grid(
            case_frames,
            [plot_case],
            selected_group.states,
            group_path,
            f"data2 {args.case_label} {selected_group.title}",
            metadata["phase1_window"][1],
            metadata["phase3_window"][0],
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
        "case_id": args.case_id,
        "case_config": rel_from_root(cfg_path, REPO_ROOT),
        "truth_reference_path": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_path": rel_from_root(case_metrics_path, REPO_ROOT),
        "phase_metrics_path": rel_from_root(phase_metrics_path, REPO_ROOT),
        "phase_windows": {
            "phase1": metadata["phase1_window"],
            "phase2": metadata["phase2_window"],
            "phase3": metadata["phase3_window"],
        },
        "phase3_periodic_gnss": {
            "on_duration_s": metadata["phase3_gnss_on_s"],
            "off_duration_s": metadata["phase3_gnss_off_s"],
        },
        "gnss_on_windows": metadata["gnss_on_windows"],
        "gnss_off_windows": metadata["gnss_off_windows"],
        "plot_paths": plot_paths,
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_mtime": mtime_text(args.exe),
            "case_config_mtime": mtime_text(cfg_path),
            "case_metrics_mtime": mtime_text(case_metrics_path),
            "phase_metrics_mtime": mtime_text(phase_metrics_path),
            "truth_reference_mtime": mtime_text(truth_reference_path),
            "sol_mtime": metrics_row.get("sol_mtime"),
            "state_series_mtime": metrics_row.get("state_series_mtime"),
            "all_states_mtime": metrics_row.get("all_states_mtime"),
        },
    }
    write_summary(summary_path, manifest, case_metrics_df, phase_metrics_df, plot_paths)
    manifest["freshness"]["summary_md_mtime"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
