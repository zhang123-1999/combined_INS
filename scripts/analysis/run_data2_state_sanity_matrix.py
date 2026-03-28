from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import BoundaryNorm, ListedColormap

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.filter_gnss_outage import filter_gnss
from scripts.analysis.odo_nhc_update_sweep import (
    ensure_dir,
    interp_truth,
    load_sol,
    load_truth_ecef,
    load_yaml,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_rtk_outage_eval import (
    aggregate_outage_segments,
    compute_global_metrics,
    compute_outage_segments,
    extract_on_windows,
    invert_windows,
)


K_STATE_DIM = 31
K_RAD_TO_DEG = 180.0 / math.pi
K_DEG_TO_RAD = math.pi / 180.0
K_MGAL_TO_MS2 = 1.0e-5
DATA2_README_DEFAULT = REPO_ROOT / "dataset/data2/README.md"
DATA2_EXTRINSIC_IMU_DEFAULT = "POS-320"

STATE_ORDER = [
    "ba_x",
    "ba_y",
    "ba_z",
    "bg_x",
    "bg_y",
    "bg_z",
    "sg_x",
    "sg_y",
    "sg_z",
    "sa_x",
    "sa_y",
    "sa_z",
    "odo_scale",
    "mounting_roll",
    "mounting_pitch",
    "mounting_yaw",
    "odo_lever_x",
    "odo_lever_y",
    "odo_lever_z",
    "gnss_lever_x",
    "gnss_lever_y",
    "gnss_lever_z",
]

STATE_META: dict[str, dict[str, Any]] = {
    "ba_x": {"group": "ba", "axis": 0, "dynamic": True, "unit": "mGal", "csv": "ba_x_mgal"},
    "ba_y": {"group": "ba", "axis": 1, "dynamic": True, "unit": "mGal", "csv": "ba_y_mgal"},
    "ba_z": {"group": "ba", "axis": 2, "dynamic": True, "unit": "mGal", "csv": "ba_z_mgal"},
    "bg_x": {"group": "bg", "axis": 0, "dynamic": True, "unit": "deg/h", "csv": "bg_x_degh"},
    "bg_y": {"group": "bg", "axis": 1, "dynamic": True, "unit": "deg/h", "csv": "bg_y_degh"},
    "bg_z": {"group": "bg", "axis": 2, "dynamic": True, "unit": "deg/h", "csv": "bg_z_degh"},
    "sg_x": {"group": "sg", "axis": 0, "dynamic": True, "unit": "ppm", "csv": "sg_x_ppm"},
    "sg_y": {"group": "sg", "axis": 1, "dynamic": True, "unit": "ppm", "csv": "sg_y_ppm"},
    "sg_z": {"group": "sg", "axis": 2, "dynamic": True, "unit": "ppm", "csv": "sg_z_ppm"},
    "sa_x": {"group": "sa", "axis": 0, "dynamic": True, "unit": "ppm", "csv": "sa_x_ppm"},
    "sa_y": {"group": "sa", "axis": 1, "dynamic": True, "unit": "ppm", "csv": "sa_y_ppm"},
    "sa_z": {"group": "sa", "axis": 2, "dynamic": True, "unit": "ppm", "csv": "sa_z_ppm"},
    "odo_scale": {"group": "odo_scale", "axis": None, "dynamic": False, "unit": "1", "csv": "odo_scale"},
    "mounting_roll": {"group": "mounting", "axis": 0, "dynamic": False, "unit": "deg", "csv": "mounting_roll_deg"},
    "mounting_pitch": {"group": "mounting", "axis": 1, "dynamic": False, "unit": "deg", "csv": "mounting_pitch_deg"},
    "mounting_yaw": {"group": "mounting", "axis": 2, "dynamic": False, "unit": "deg", "csv": "mounting_yaw_deg"},
    "odo_lever_x": {"group": "odo_lever", "axis": 0, "dynamic": False, "unit": "m", "csv": "odo_lever_x_m"},
    "odo_lever_y": {"group": "odo_lever", "axis": 1, "dynamic": False, "unit": "m", "csv": "odo_lever_y_m"},
    "odo_lever_z": {"group": "odo_lever", "axis": 2, "dynamic": False, "unit": "m", "csv": "odo_lever_z_m"},
    "gnss_lever_x": {"group": "gnss_lever", "axis": 0, "dynamic": False, "unit": "m", "csv": "gnss_lever_x_m"},
    "gnss_lever_y": {"group": "gnss_lever", "axis": 1, "dynamic": False, "unit": "m", "csv": "gnss_lever_y_m"},
    "gnss_lever_z": {"group": "gnss_lever", "axis": 2, "dynamic": False, "unit": "m", "csv": "gnss_lever_z_m"},
}

LABEL_TO_SCORE = {"normal": 0, "borderline": 1, "abnormal": 2}
SCORE_TO_LABEL = {value: key for key, value in LABEL_TO_SCORE.items()}


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: json_safe(val) for key, val in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def run_command(cmd: list[str], cwd: Path) -> str:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(cmd)}\n{merged}")
    return merged


def reset_directory(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def format_metric(value: float) -> str:
    if value is None or not math.isfinite(float(value)):
        return "NA"
    return f"{float(value):.6f}"


def markdown_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    header = "| " + " | ".join(columns) + " |"
    separator = "| " + " | ".join(["---"] * len(columns)) + " |"
    body = ["| " + " | ".join(row) + " |" for row in rows]
    return [header, separator] + body


def euler_to_rotation(roll_rad: np.ndarray, pitch_rad: np.ndarray, yaw_rad: np.ndarray) -> np.ndarray:
    cr = np.cos(roll_rad)
    sr = np.sin(roll_rad)
    cp = np.cos(pitch_rad)
    sp = np.sin(pitch_rad)
    cy = np.cos(yaw_rad)
    sy = np.sin(yaw_rad)
    rot = np.empty((roll_rad.size, 3, 3), dtype=float)
    rot[:, 0, 0] = cy * cp
    rot[:, 0, 1] = cy * sp * sr - sy * cr
    rot[:, 0, 2] = cy * sp * cr + sy * sr
    rot[:, 1, 0] = sy * cp
    rot[:, 1, 1] = sy * sp * sr + cy * cr
    rot[:, 1, 2] = sy * sp * cr - cy * sr
    rot[:, 2, 0] = -sp
    rot[:, 2, 1] = cp * sr
    rot[:, 2, 2] = cp * cr
    return rot


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


def llh_deg_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, h_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    a = 6378137.0
    e2 = 6.69437999014e-3
    n = a / np.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * cos_lon
    y = (n + h_m) * cos_lat * sin_lon
    z = (n * (1.0 - e2) + h_m) * sin_lat
    return np.column_stack((x, y, z))


def compute_body_frame_gnss_lever(
    rtk_path: Path,
    pos_path: Path,
    start_time: float,
    final_time: float,
) -> dict[str, Any]:
    rtk_df = pd.read_csv(
        rtk_path,
        sep=r"\s+",
        header=None,
        usecols=list(range(7)),
        names=["timestamp", "lat", "lon", "h", "std_n", "std_e", "std_d"],
    )
    pos_df = pd.read_csv(
        pos_path,
        sep=r"\s+",
        header=None,
        names=[
            "timestamp",
            "lat",
            "lon",
            "h",
            "vn",
            "ve",
            "vd",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
        ],
    )
    rtk_df = rtk_df[(rtk_df["timestamp"] >= start_time) & (rtk_df["timestamp"] <= final_time)].copy()
    pos_df = pos_df[(pos_df["timestamp"] >= start_time) & (pos_df["timestamp"] <= final_time)].copy()
    if rtk_df.empty or pos_df.empty:
        raise RuntimeError("failed to compute GNSS lever truth: RTK/POS overlap is empty")

    t = rtk_df["timestamp"].to_numpy(dtype=float)
    pos_t = pos_df["timestamp"].to_numpy(dtype=float)
    pos_lat = np.interp(t, pos_t, pos_df["lat"].to_numpy(dtype=float))
    pos_lon = np.interp(t, pos_t, pos_df["lon"].to_numpy(dtype=float))
    pos_h = np.interp(t, pos_t, pos_df["h"].to_numpy(dtype=float))
    pos_roll = np.interp(t, pos_t, pos_df["roll_deg"].to_numpy(dtype=float))
    pos_pitch = np.interp(t, pos_t, pos_df["pitch_deg"].to_numpy(dtype=float))
    pos_yaw = np.interp(t, pos_t, pos_df["yaw_deg"].to_numpy(dtype=float))

    pos_ecef = llh_deg_to_ecef(pos_lat, pos_lon, pos_h)
    rtk_ecef = llh_deg_to_ecef(
        rtk_df["lat"].to_numpy(dtype=float),
        rtk_df["lon"].to_numpy(dtype=float),
        rtk_df["h"].to_numpy(dtype=float),
    )
    rot_ne = rot_ned_to_ecef(np.deg2rad(pos_lat), np.deg2rad(pos_lon))
    rot_nb = euler_to_rotation(np.deg2rad(pos_roll), np.deg2rad(pos_pitch), np.deg2rad(pos_yaw))
    rot_eb = np.einsum("nij,njk->nik", rot_ne, rot_nb)
    diff_ecef = rtk_ecef - pos_ecef
    lever_body = np.einsum("nji,nj->ni", rot_eb, diff_ecef)
    median = np.median(lever_body, axis=0)
    return {
        "value_m": [float(median[0]), float(median[1]), float(median[2])],
        "samples": int(lever_body.shape[0]),
        "source": "dataset/data2/rtk.txt + dataset/data2_converted/POS_converted.txt body-frame median",
    }


def normalize_readme_key(value: str) -> str:
    return value.strip().lower().replace("-", "").replace(" ", "")


def parse_data2_readme_static_vector(
    readme_path: Path,
    section_title: str,
    imu_model: str,
) -> list[float]:
    lines = readme_path.read_text(encoding="utf-8").splitlines()
    target = normalize_readme_key(imu_model)
    in_section = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("## "):
            in_section = stripped == f"## {section_title}"
            continue
        if not in_section or not stripped.startswith("|"):
            continue
        if ":---" in stripped or "---" in stripped:
            continue
        parts = [part.strip() for part in stripped.strip("|").split("|")]
        if len(parts) < 4:
            continue
        row_key = normalize_readme_key(parts[0])
        if row_key in {"imu", "1to2"}:
            continue
        if row_key == target:
            return [float(parts[1]), float(parts[2]), float(parts[3])]
    raise RuntimeError(f"failed to locate `{imu_model}` in section `{section_title}` from {readme_path}")


def parse_data2_readme_imu_process_params(readme_path: Path, imu_model: str) -> dict[str, float]:
    lines = readme_path.read_text(encoding="utf-8").splitlines()
    target = normalize_readme_key(imu_model)
    in_section = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("## "):
            in_section = stripped == "## IMU 参数"
            continue
        if not in_section or "|" not in stripped:
            continue
        parts = [part.strip() for part in stripped.strip("|").split("|")]
        if len(parts) < 8:
            continue
        row_key = normalize_readme_key(parts[0])
        if row_key in {"imu", ""}:
            continue
        if row_key == target:
            return {
                "imu_model": imu_model,
                "sigma_bg_degh": float(parts[3]),
                "sigma_ba_mgal": float(parts[4]),
                "corr_time_hr": float(parts[5]),
                "sigma_sg_ppm": float(parts[6]),
                "sigma_sa_ppm": float(parts[7]),
                "corr_time_s": float(parts[5]) * 3600.0,
            }
    raise RuntimeError(f"failed to locate IMU row `{imu_model}` in {readme_path}")


def human_to_internal(group: str, value: float) -> float:
    if group == "ba":
        return float(value * K_MGAL_TO_MS2)
    if group == "bg":
        return float(math.radians(value) / 3600.0)
    if group in {"sg", "sa"}:
        return float(value * 1.0e-6)
    if group == "mounting":
        return float(math.radians(value))
    return float(value)


def family_static_eps_human(group: str) -> float:
    if group == "odo_scale":
        return 1.0e-3
    if group == "mounting":
        return 0.02
    if group in {"odo_lever", "gnss_lever"}:
        return 0.01
    return 1.0e-6


def build_truth_reference(
    base_cfg: dict[str, Any],
    readme_path: Path | None = None,
    extrinsic_imu_model: str = DATA2_EXTRINSIC_IMU_DEFAULT,
) -> dict[str, Any]:
    fusion = base_cfg["fusion"]
    start_time = float(fusion["starttime"])
    final_time = float(fusion["finaltime"])
    constraints_mount = fusion["constraints"]["imu_mounting_angle"]
    readme_path = readme_path.resolve() if readme_path is not None else DATA2_README_DEFAULT.resolve()
    if not readme_path.exists():
        raise FileNotFoundError(f"missing data2 README: {readme_path}")
    readme_rel = rel_from_root(readme_path, REPO_ROOT)
    gnss_lever_nominal = parse_data2_readme_static_vector(readme_path, "天线杆臂", extrinsic_imu_model)
    odo_lever_nominal = parse_data2_readme_static_vector(readme_path, "里程计杆臂", extrinsic_imu_model)
    imu_process_params = parse_data2_readme_imu_process_params(readme_path, extrinsic_imu_model)
    gnss_lever_info = compute_body_frame_gnss_lever(
        (REPO_ROOT / "dataset/data2/rtk.txt").resolve(),
        (REPO_ROOT / fusion["pos_path"]).resolve(),
        start_time,
        final_time,
    )

    total_mounting_truth_deg = {"roll": 0.0, "pitch": 0.36, "yaw": 1.37}
    mounting_state_truth_deg = {
        "roll": 0.0,
        "pitch": total_mounting_truth_deg["pitch"] - float(constraints_mount[1]),
        "yaw": total_mounting_truth_deg["yaw"] - float(constraints_mount[2]),
    }

    dynamic_truth_human = {
        "ba": [-1500.0, -600.0, 0.0],
        "bg": [100.0, -300.0, -100.0],
        "sg": [1000.0, 6000.0, -4000.0],
        "sa": [2600.0, 8400.0, 300.0],
    }
    dynamic_mean_source = "docs/notes/初始姿态和参数.md"
    dynamic_process_source = f"{readme_rel} {extrinsic_imu_model} IMU parameters"
    instability_human = {
        "ba": float(imu_process_params["sigma_ba_mgal"]),
        "bg": float(imu_process_params["sigma_bg_degh"]),
        "sg": float(imu_process_params["sigma_sg_ppm"]),
        "sa": float(imu_process_params["sigma_sa_ppm"]),
    }
    markov_corr_time_s = float(imu_process_params["corr_time_s"])
    static_truth_human = {
        "odo_scale": 1.0,
        "mounting": [
            mounting_state_truth_deg["roll"],
            mounting_state_truth_deg["pitch"],
            mounting_state_truth_deg["yaw"],
        ],
        "odo_lever": odo_lever_nominal,
        "gnss_lever": gnss_lever_nominal,
    }

    state_refs: dict[str, Any] = {}
    for state_name in STATE_ORDER:
        meta = STATE_META[state_name]
        group = meta["group"]
        axis = meta["axis"]
        dynamic = bool(meta["dynamic"])
        if dynamic:
            value_human = float(dynamic_truth_human[group][axis])
            source = f"{dynamic_mean_source} (mean) + {dynamic_process_source} (sigma/tau)"
            instability = float(instability_human[group])
        elif group == "odo_scale":
            value_human = float(static_truth_human["odo_scale"])
            source = "fixed nominal scale prior"
            instability = None
        elif group == "mounting":
            value_human = float(static_truth_human["mounting"][axis])
            source = "dataset/data2/README.md commented mounting table - fusion.constraints.imu_mounting_angle offset"
            instability = None
        elif group == "odo_lever":
            value_human = float(static_truth_human["odo_lever"][axis])
            source = f"{readme_rel} {extrinsic_imu_model} ODO lever arm"
            instability = None
        elif group == "gnss_lever":
            value_human = float(static_truth_human["gnss_lever"][axis])
            source = f"{readme_rel} {extrinsic_imu_model} GNSS lever arm"
            instability = None
        else:
            raise KeyError(f"unsupported state group: {group}")

        state_refs[state_name] = {
            "state_name": state_name,
            "group": group,
            "axis": axis,
            "dynamic": dynamic,
            "unit": meta["unit"],
            "reference_value": value_human,
            "reference_value_internal": human_to_internal(group, value_human),
            "source": source,
        }
        if dynamic:
            state_refs[state_name]["reference_source"] = dynamic_mean_source
            state_refs[state_name]["instability_source"] = dynamic_process_source
            state_refs[state_name]["instability"] = instability
            state_refs[state_name]["instability_internal"] = human_to_internal(group, instability)
            state_refs[state_name]["markov_corr_time_s"] = markov_corr_time_s
            state_refs[state_name]["markov_corr_time_hr"] = float(imu_process_params["corr_time_hr"])

    gnss_lever_diag_delta = (
        np.array(gnss_lever_info["value_m"], dtype=float) - np.array(gnss_lever_nominal, dtype=float)
    ).tolist()

    return {
        "catalog_generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "sources": {
            "dynamic_initial_and_instability": f"{dynamic_mean_source} (mean) + {dynamic_process_source} (sigma/tau)",
            "dynamic_initial_value_source": dynamic_mean_source,
            "dynamic_instability_and_corr_time": {
                "imu_model": extrinsic_imu_model,
                "source": dynamic_process_source,
                "sigma_ba_mgal": float(imu_process_params["sigma_ba_mgal"]),
                "sigma_bg_degh": float(imu_process_params["sigma_bg_degh"]),
                "sigma_sg_ppm": float(imu_process_params["sigma_sg_ppm"]),
                "sigma_sa_ppm": float(imu_process_params["sigma_sa_ppm"]),
                "corr_time_hr": float(imu_process_params["corr_time_hr"]),
            },
            "extrinsic_truth_readme": readme_rel,
            "extrinsic_truth_target_imu": extrinsic_imu_model,
            "odo_lever_truth": {
                "imu_model": extrinsic_imu_model,
                "value_m": [float(x) for x in odo_lever_nominal],
                "source": f"{readme_rel} {extrinsic_imu_model} ODO lever arm",
                "role": "official_nominal_truth",
            },
            "gnss_lever_truth": {
                "imu_model": extrinsic_imu_model,
                "value_m": [float(x) for x in gnss_lever_nominal],
                "source": f"{readme_rel} {extrinsic_imu_model} GNSS lever arm",
                "role": "official_nominal_truth",
            },
            "gnss_lever_diagnostic_body_median": {
                **gnss_lever_info,
                "role": "diagnostic_only",
                "delta_vs_nominal_m": [float(x) for x in gnss_lever_diag_delta],
            },
            "mounting_total_truth": {
                "value_deg": total_mounting_truth_deg,
                "source": "dataset/data2/README.md commented mounting table",
            },
            "constraints_mounting_base_deg": list(map(float, constraints_mount)),
        },
        "states": state_refs,
    }


def base_p0_diag_from_config(base_cfg: dict[str, Any]) -> list[float]:
    p0_diag = base_cfg["fusion"]["init"].get("P0_diag")
    if not p0_diag or len(p0_diag) != K_STATE_DIM:
        raise RuntimeError("base config must provide init.P0_diag for explicit experiment generation")
    return [float(x) for x in p0_diag]


def default_ablation_flags() -> dict[str, bool]:
    return {
        "disable_gnss_lever_arm": False,
        "disable_gnss_lever_z": False,
        "disable_odo_lever_arm": False,
        "disable_odo_scale": False,
        "disable_gyro_bias": False,
        "disable_gyro_scale": False,
        "disable_accel_scale": False,
        "disable_mounting": False,
        "disable_mounting_roll": False,
    }


def get_group_vector_internal(truth_reference: dict[str, Any], group: str) -> np.ndarray:
    if group == "odo_scale":
        return np.array([truth_reference["states"]["odo_scale"]["reference_value_internal"]], dtype=float)
    if group == "mounting":
        return np.array(
            [
                truth_reference["states"]["mounting_roll"]["reference_value_internal"],
                truth_reference["states"]["mounting_pitch"]["reference_value_internal"],
                truth_reference["states"]["mounting_yaw"]["reference_value_internal"],
            ],
            dtype=float,
        )
    if group == "odo_lever":
        return np.array(
            [
                truth_reference["states"]["odo_lever_x"]["reference_value_internal"],
                truth_reference["states"]["odo_lever_y"]["reference_value_internal"],
                truth_reference["states"]["odo_lever_z"]["reference_value_internal"],
            ],
            dtype=float,
        )
    if group == "gnss_lever":
        return np.array(
            [
                truth_reference["states"]["gnss_lever_x"]["reference_value_internal"],
                truth_reference["states"]["gnss_lever_y"]["reference_value_internal"],
                truth_reference["states"]["gnss_lever_z"]["reference_value_internal"],
            ],
            dtype=float,
        )
    return np.array(
        [
            truth_reference["states"][f"{group}_x"]["reference_value_internal"],
            truth_reference["states"][f"{group}_y"]["reference_value_internal"],
            truth_reference["states"][f"{group}_z"]["reference_value_internal"],
        ],
        dtype=float,
    )


def get_group_instability_internal(truth_reference: dict[str, Any], group: str) -> np.ndarray:
    return np.array(
        [
            truth_reference["states"][f"{group}_x"]["instability_internal"],
            truth_reference["states"][f"{group}_y"]["instability_internal"],
            truth_reference["states"][f"{group}_z"]["instability_internal"],
        ],
        dtype=float,
    )


def family_anchored_process_internal(group: str) -> float:
    if group == "odo_scale":
        return 1.0e-7
    if group == "mounting":
        return human_to_internal("mounting", 1.0e-6)
    if group in {"odo_lever", "gnss_lever"}:
        return 1.0e-6
    raise KeyError(group)


def family_target_std_human(group: str, axis: int | None) -> float:
    if group == "odo_scale":
        return 0.05
    if group == "mounting":
        return [0.5, 0.5, 0.3][int(axis)]
    if group in {"odo_lever", "gnss_lever"}:
        return 0.2
    raise KeyError(group)


def case_release_state_name(case_id: str) -> str | None:
    if case_id == "truth_anchor_all_non_pva":
        return None
    if not case_id.startswith("release_"):
        raise ValueError(f"unexpected case_id: {case_id}")
    return case_id[len("release_") :]


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
    outage_gnss_path: Path,
) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_noise = base_cfg["fusion"]["noise"]
    base_p0 = base_p0_diag_from_config(base_cfg)
    p0_diag = list(base_p0)

    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    fusion["enable_gnss_velocity"] = False
    fusion["gnss_path"] = rel_from_root(outage_gnss_path, REPO_ROOT)
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["ablation"] = default_ablation_flags()
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False
    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = True
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    release_state = case_release_state_name(case_id)
    ba_init = get_group_vector_internal(truth_reference, "ba")
    bg_init = get_group_vector_internal(truth_reference, "bg")
    sg_init = get_group_vector_internal(truth_reference, "sg")
    sa_init = get_group_vector_internal(truth_reference, "sa")
    ba_inst = get_group_instability_internal(truth_reference, "ba")
    bg_inst = get_group_instability_internal(truth_reference, "bg")
    sg_inst = get_group_instability_internal(truth_reference, "sg")
    sa_inst = get_group_instability_internal(truth_reference, "sa")
    ba_std = 0.1 * ba_inst
    bg_std = 0.1 * bg_inst
    sg_std = 0.1 * sg_inst
    sa_std = 0.1 * sa_inst
    ba_noise = 0.1 * ba_inst
    bg_noise = 0.1 * bg_inst
    sg_noise = 0.1 * sg_inst
    sa_noise = 0.1 * sa_inst
    if release_state:
        meta = STATE_META[release_state]
        group = meta["group"]
        axis = int(meta["axis"]) if meta["axis"] is not None else None
        if group == "ba":
            ba_init[axis] = 0.0
            ba_std[axis] = max(abs(get_group_vector_internal(truth_reference, "ba")[axis]), 3.0 * ba_inst[axis])
            ba_noise[axis] = ba_inst[axis]
        elif group == "bg":
            bg_init[axis] = 0.0
            bg_std[axis] = max(abs(get_group_vector_internal(truth_reference, "bg")[axis]), 3.0 * bg_inst[axis])
            bg_noise[axis] = bg_inst[axis]
        elif group == "sg":
            sg_init[axis] = 0.0
            sg_std[axis] = max(abs(get_group_vector_internal(truth_reference, "sg")[axis]), 3.0 * sg_inst[axis])
            sg_noise[axis] = sg_inst[axis]
        elif group == "sa":
            sa_init[axis] = 0.0
            sa_std[axis] = max(abs(get_group_vector_internal(truth_reference, "sa")[axis]), 3.0 * sa_inst[axis])
            sa_noise[axis] = sa_inst[axis]

    init_cfg["ba0"] = [float(x) for x in ba_init]
    init_cfg["bg0"] = [float(x) for x in bg_init]
    init_cfg["sg0"] = [float(x) for x in sg_init]
    init_cfg["sa0"] = [float(x) for x in sa_init]
    p0_diag[9:12] = [float(x * x) for x in ba_std]
    p0_diag[12:15] = [float(x * x) for x in bg_std]
    p0_diag[15:18] = [float(x * x) for x in sg_std]
    p0_diag[18:21] = [float(x * x) for x in sa_std]

    odo_scale_ref = float(truth_reference["states"]["odo_scale"]["reference_value_internal"])
    odo_scale_init = odo_scale_ref
    odo_scale_std = 1.0e-3
    odo_scale_noise = 1.0e-7
    if release_state == "odo_scale":
        odo_scale_init = 1.0e-6
        odo_scale_std = 0.05
        odo_scale_noise = float(base_noise["sigma_odo_scale"])
    init_cfg["odo_scale"] = float(odo_scale_init)
    p0_diag[21] = float(odo_scale_std * odo_scale_std)

    mount_ref = get_group_vector_internal(truth_reference, "mounting")
    mount_init_deg = np.rad2deg(mount_ref)
    mount_std_deg = np.array([0.02, 0.02, 0.02], dtype=float)
    mount_noise = np.full(3, family_anchored_process_internal("mounting"), dtype=float)
    if release_state and STATE_META[release_state]["group"] == "mounting":
        axis = int(STATE_META[release_state]["axis"])
        mount_init_deg[axis] = 0.0
        mount_std_deg[axis] = family_target_std_human("mounting", axis)
        if axis == 0:
            mount_noise[axis] = float(base_noise.get("sigma_mounting_roll", base_noise["sigma_mounting"]))
        elif axis == 1:
            mount_noise[axis] = float(base_noise.get("sigma_mounting_pitch", base_noise["sigma_mounting"]))
        else:
            mount_noise[axis] = float(base_noise.get("sigma_mounting_yaw", base_noise["sigma_mounting"]))
    init_cfg["mounting_roll0"] = float(mount_init_deg[0])
    init_cfg["mounting_pitch0"] = float(mount_init_deg[1])
    init_cfg["mounting_yaw0"] = float(mount_init_deg[2])
    p0_diag[22] = float(math.radians(mount_std_deg[0]) ** 2)
    p0_diag[23] = float(math.radians(mount_std_deg[1]) ** 2)
    p0_diag[24] = float(math.radians(mount_std_deg[2]) ** 2)

    odo_lever_init = get_group_vector_internal(truth_reference, "odo_lever")
    odo_lever_std = np.full(3, 0.01, dtype=float)
    odo_lever_noise = np.full(3, family_anchored_process_internal("odo_lever"), dtype=float)
    if release_state and STATE_META[release_state]["group"] == "odo_lever":
        axis = int(STATE_META[release_state]["axis"])
        odo_lever_init[axis] = 0.0
        odo_lever_std[axis] = 0.2
        odo_lever_noise[axis] = float(base_noise["sigma_lever_arm"])
    init_cfg["lever_arm0"] = [float(x) for x in odo_lever_init]
    p0_diag[25:28] = [float(x * x) for x in odo_lever_std]

    gnss_lever_init = get_group_vector_internal(truth_reference, "gnss_lever")
    gnss_lever_std = np.full(3, 0.01, dtype=float)
    gnss_lever_noise = np.full(3, family_anchored_process_internal("gnss_lever"), dtype=float)
    if release_state and STATE_META[release_state]["group"] == "gnss_lever":
        axis = int(STATE_META[release_state]["axis"])
        gnss_lever_init[axis] = 0.0
        gnss_lever_std[axis] = 0.2
        gnss_lever_noise[axis] = float(base_noise["sigma_gnss_lever_arm"])
    init_cfg["gnss_lever_arm0"] = [float(x) for x in gnss_lever_init]
    p0_diag[28:31] = [float(x * x) for x in gnss_lever_std]
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    noise_cfg["sigma_ba"] = float(np.max(ba_noise))
    noise_cfg["sigma_bg"] = float(np.max(bg_noise))
    noise_cfg["sigma_sg"] = float(np.max(sg_noise))
    noise_cfg["sigma_sa"] = float(np.max(sa_noise))
    noise_cfg["sigma_ba_vec"] = [float(x) for x in ba_noise]
    noise_cfg["sigma_bg_vec"] = [float(x) for x in bg_noise]
    noise_cfg["sigma_sg_vec"] = [float(x) for x in sg_noise]
    noise_cfg["sigma_sa_vec"] = [float(x) for x in sa_noise]
    noise_cfg["sigma_odo_scale"] = float(odo_scale_noise)
    noise_cfg["sigma_mounting_roll"] = float(mount_noise[0])
    noise_cfg["sigma_mounting_pitch"] = float(mount_noise[1])
    noise_cfg["sigma_mounting_yaw"] = float(mount_noise[2])
    noise_cfg["sigma_lever_arm"] = float(np.max(odo_lever_noise))
    noise_cfg["sigma_lever_arm_vec"] = [float(x) for x in odo_lever_noise]
    noise_cfg["sigma_gnss_lever_arm"] = float(np.max(gnss_lever_noise))
    noise_cfg["sigma_gnss_lever_arm_vec"] = [float(x) for x in gnss_lever_noise]
    noise_cfg["markov_corr_time"] = 3600.0
    return cfg


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
    outage_gnss_path: Path,
) -> Path:
    cfg = build_case_config(base_cfg, truth_reference, case_id, case_dir, outage_gnss_path)
    cfg_path = case_dir / f"config_{case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path


def load_diag(path: Path) -> pd.DataFrame:
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_csv(path, sep=r"\s+")


def evaluate_navigation_metrics(cfg_path: Path, sol_path: Path) -> tuple[dict[str, Any], list[dict[str, float]]]:
    cfg = load_yaml(cfg_path)
    fusion = cfg["fusion"]
    truth_path = (REPO_ROOT / fusion["pos_path"]).resolve()
    truth_t, truth_xyz = load_truth_ecef(truth_path)
    sol_t, sol_xyz = load_sol(sol_path)
    truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
    err_xyz = sol_xyz - truth_interp
    outage_windows = invert_windows(extract_on_windows(cfg), float(fusion["starttime"]), float(fusion["finaltime"]))
    segment_rows = compute_outage_segments(sol_t, err_xyz, outage_windows)
    row: dict[str, Any] = {}
    row.update(compute_global_metrics(err_xyz))
    row.update(aggregate_outage_segments(segment_rows))
    return row, segment_rows


def run_case(case_id: str, cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    stdout_path = case_dir / f"{case_id}.stdout.txt"
    diag_path = case_dir / f"DIAG_{case_id}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output for {case_id}: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output for {case_id}: {state_series_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {case_id}")
    shutil.copy2(root_diag, diag_path)
    nav_metrics, segment_rows = evaluate_navigation_metrics(cfg_path, sol_path)
    row: dict[str, Any] = {
        "case_id": case_id,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(diag_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
        "segment_rows": segment_rows,
    }
    row.update(nav_metrics)
    return row


def static_behavior_label(state_name: str, truth_reference: dict[str, Any], values: np.ndarray) -> tuple[str, dict[str, float]]:
    group = STATE_META[state_name]["group"]
    truth = float(truth_reference["states"][state_name]["reference_value"])
    eps = family_static_eps_human(group)
    final_value = float(values[-1])
    value_range = float(values.max() - values.min())
    recovery_ratio = abs(final_value - truth) / max(abs(truth), eps)
    max_allowed_range = 1.5 * max(abs(truth), eps)
    if recovery_ratio <= 0.3 and value_range <= max_allowed_range:
        label = "normal"
    elif recovery_ratio <= 0.7:
        label = "borderline"
    else:
        label = "abnormal"
    return label, {
        "truth_value": truth,
        "final_value": final_value,
        "range": value_range,
        "recovery_ratio": float(recovery_ratio),
        "max_allowed_range": float(max_allowed_range),
    }


def dynamic_behavior_label(
    state_name: str,
    truth_reference: dict[str, Any],
    timestamps: np.ndarray,
    values: np.ndarray,
) -> tuple[str, dict[str, float]]:
    ref = truth_reference["states"][state_name]
    ref0 = float(ref["reference_value"])
    sigma_ss = float(ref["instability"])
    corr_time = float(ref["markov_corr_time_s"])
    elapsed = timestamps - float(timestamps[0])
    half_width = 3.0 * sigma_ss * np.sqrt(np.maximum(0.0, 1.0 - np.exp(-2.0 * elapsed / corr_time)))
    diff = np.abs(values - ref0)
    outside_ratio = float(np.mean(diff > half_width))
    eps = max(1.0e-9, 1.0e-6 * max(1.0, abs(ref0), sigma_ss))
    denom = np.maximum(half_width, eps)
    final_ratio = float(diff[-1] / denom[-1])
    max_ratio = float(np.max(diff / denom))
    if outside_ratio <= 0.05 and final_ratio <= 1.0:
        label = "normal"
    elif outside_ratio <= 0.20 and final_ratio <= 1.5:
        label = "borderline"
    else:
        label = "abnormal"
    return label, {
        "truth_value": ref0,
        "final_value": float(values[-1]),
        "outside_ratio": outside_ratio,
        "final_over_bound_ratio": final_ratio,
        "max_over_bound_ratio": max_ratio,
        "bound_final": float(half_width[-1]),
    }


def impact_label(case_row: dict[str, Any], control_row: dict[str, Any]) -> tuple[str, dict[str, float]]:
    control_mean = float(control_row["mean_outage_rmse_3d_m"])
    control_final = float(control_row["max_outage_final_err_3d_m"])
    case_mean = float(case_row["mean_outage_rmse_3d_m"])
    case_final = float(case_row["max_outage_final_err_3d_m"])
    delta_mean = (case_mean - control_mean) / max(control_mean, 1.0e-9)
    delta_final = (case_final - control_final) / max(control_final, 1.0e-9)
    if delta_mean <= 0.25 and delta_final <= 0.25:
        label = "normal"
    elif delta_mean <= 1.0 and delta_final <= 1.0:
        label = "borderline"
    else:
        label = "abnormal"
    return label, {
        "delta_mean_rmse3d": float(delta_mean),
        "delta_max_final3d": float(delta_final),
        "control_mean_rmse3d": control_mean,
        "case_mean_rmse3d": case_mean,
        "control_max_final3d": control_final,
        "case_max_final3d": case_final,
    }


def overall_label(behavior: str, impact: str) -> str:
    if "abnormal" in {behavior, impact}:
        return "abnormal"
    if "borderline" in {behavior, impact}:
        return "borderline"
    return "normal"


def evaluate_state_case(
    state_name: str,
    case_row: dict[str, Any],
    control_row: dict[str, Any],
    truth_reference: dict[str, Any],
) -> dict[str, Any]:
    state_csv_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    state_df = pd.read_csv(state_csv_path, usecols=["timestamp", STATE_META[state_name]["csv"]])
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    values = state_df[STATE_META[state_name]["csv"]].to_numpy(dtype=float)
    if STATE_META[state_name]["dynamic"]:
        behavior, behavior_metrics = dynamic_behavior_label(state_name, truth_reference, timestamps, values)
    else:
        behavior, behavior_metrics = static_behavior_label(state_name, truth_reference, values)
    impact, impact_metrics = impact_label(case_row, control_row)
    overall = overall_label(behavior, impact)

    diag_head_std_mr = math.nan
    diag_tail_std_mr = math.nan
    if state_name == "mounting_roll":
        diag_df = load_diag((REPO_ROOT / case_row["diag_path"]).resolve())
        if not diag_df.empty and "std_mr" in diag_df.columns:
            span = max(1, len(diag_df) // 5)
            diag_head_std_mr = float(np.nanmedian(diag_df["std_mr"].iloc[:span].to_numpy(dtype=float)))
            diag_tail_std_mr = float(np.nanmedian(diag_df["std_mr"].iloc[-span:].to_numpy(dtype=float)))
            if math.isfinite(diag_head_std_mr) and math.isfinite(diag_tail_std_mr):
                if diag_tail_std_mr >= 0.9 * max(diag_head_std_mr, 1.0e-12) and impact == "abnormal":
                    behavior = "abnormal"
                    overall = "abnormal"

    row: dict[str, Any] = {
        "state_name": state_name,
        "case_id": case_row["case_id"],
        "truth_source": truth_reference["states"][state_name]["source"],
        "behavior_label": behavior,
        "impact_label": impact,
        "overall_label": overall,
        "diag_head_std_mr": diag_head_std_mr,
        "diag_tail_std_mr": diag_tail_std_mr,
    }
    row.update(behavior_metrics)
    row.update(impact_metrics)
    return row


def downsample_for_plot(x: np.ndarray, y: np.ndarray, max_points: int = 5000) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= max_points:
        return x, y
    idx = np.linspace(0, x.size - 1, max_points, dtype=int)
    return x[idx], y[idx]


def plot_state_comparison(
    state_name: str,
    control_state_path: Path,
    case_state_path: Path,
    truth_reference: dict[str, Any],
    output_path: Path,
) -> None:
    column = STATE_META[state_name]["csv"]
    control_df = pd.read_csv(control_state_path, usecols=["timestamp", column])
    case_df = pd.read_csv(case_state_path, usecols=["timestamp", column])
    ct = control_df["timestamp"].to_numpy(dtype=float)
    cv = control_df[column].to_numpy(dtype=float)
    tt = case_df["timestamp"].to_numpy(dtype=float)
    tv = case_df[column].to_numpy(dtype=float)
    ct_plot, cv_plot = downsample_for_plot(ct, cv)
    tt_plot, tv_plot = downsample_for_plot(tt, tv)

    ref = truth_reference["states"][state_name]
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(ct_plot, cv_plot, label="control", linewidth=1.2)
    ax.plot(tt_plot, tv_plot, label=f"release_{state_name}", linewidth=1.2)
    if STATE_META[state_name]["dynamic"]:
        elapsed = tt_plot - tt_plot[0]
        sigma_ss = float(ref["instability"])
        corr_time = float(ref["markov_corr_time_s"])
        bound = 3.0 * sigma_ss * np.sqrt(np.maximum(0.0, 1.0 - np.exp(-2.0 * elapsed / corr_time)))
        ref_line = np.full_like(tt_plot, float(ref["reference_value"]))
        ax.plot(tt_plot, ref_line, linestyle="--", color="black", linewidth=1.0, label="ref0")
        ax.fill_between(tt_plot, ref_line - bound, ref_line + bound, color="#cccccc", alpha=0.3, label="3sigma envelope")
    else:
        ax.axhline(float(ref["reference_value"]), linestyle="--", color="black", linewidth=1.0, label="truth")
    ax.set_title(f"{state_name}: control vs release")
    ax.set_xlabel("timestamp [s]")
    ax.set_ylabel(STATE_META[state_name]["unit"])
    ax.grid(alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_heatmap(judgement_df: pd.DataFrame, output_path: Path) -> None:
    heat = np.column_stack(
        [
            judgement_df["behavior_label"].map(LABEL_TO_SCORE).to_numpy(dtype=float),
            judgement_df["impact_label"].map(LABEL_TO_SCORE).to_numpy(dtype=float),
        ]
    )
    fig, ax = plt.subplots(figsize=(5.5, max(6.0, 0.35 * len(judgement_df))))
    cmap = ListedColormap(["#4daf4a", "#ffcc33", "#e41a1c"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    im = ax.imshow(heat, cmap=cmap, norm=norm, aspect="auto")
    ax.set_xticks([0, 1], labels=["behavior", "impact"])
    ax.set_yticks(np.arange(len(judgement_df)), labels=judgement_df["state_name"].tolist())
    for i in range(len(judgement_df)):
        for j in range(2):
            ax.text(j, i, SCORE_TO_LABEL[int(heat[i, j])], ha="center", va="center", fontsize=8)
    ax.set_title("State judgement heatmap")
    fig.colorbar(im, ax=ax, ticks=[0, 1, 2], label="severity")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_mounting_roll_std(control_diag_path: Path, case_diag_path: Path, output_path: Path) -> None:
    control_diag = load_diag(control_diag_path)
    case_diag = load_diag(case_diag_path)
    if control_diag.empty or case_diag.empty or "std_mr" not in control_diag.columns or "std_mr" not in case_diag.columns:
        return
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(control_diag["t"], np.rad2deg(control_diag["std_mr"]), label="control std_mr", linewidth=1.2)
    ax.plot(case_diag["t"], np.rad2deg(case_diag["std_mr"]), label="release_mounting_roll std_mr", linewidth=1.2)
    ax.set_xlabel("elapsed time [s]")
    ax.set_ylabel("std_mr [deg]")
    ax.set_title("mounting_roll std_mr comparison")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def requested_case_ids(cases_arg: list[str] | None) -> list[str]:
    all_cases = ["truth_anchor_all_non_pva"] + [f"release_{state_name}" for state_name in STATE_ORDER]
    if not cases_arg:
        return all_cases
    unique: list[str] = []
    seen: set[str] = set()
    for case_id in cases_arg:
        normalized = case_id.strip()
        if not normalized:
            continue
        if normalized not in all_cases:
            raise ValueError(f"unsupported case_id: {normalized}")
        if normalized not in seen:
            unique.append(normalized)
            seen.add(normalized)
    if "truth_anchor_all_non_pva" not in seen:
        unique.insert(0, "truth_anchor_all_non_pva")
    return unique


def case_sort_key(case_id: str) -> tuple[int, int]:
    if case_id == "truth_anchor_all_non_pva":
        return (0, -1)
    state_name = case_release_state_name(case_id)
    return (1, STATE_ORDER.index(state_name))


def write_case_segments(case_dir: Path, case_id: str, segment_rows: list[dict[str, float]]) -> Path:
    segment_path = case_dir / f"outage_segments_{case_id}.csv"
    pd.DataFrame(segment_rows).to_csv(segment_path, index=False, encoding="utf-8-sig")
    return segment_path


def build_case_metrics_table(case_rows: list[dict[str, Any]]) -> pd.DataFrame:
    records: list[dict[str, Any]] = []
    for row in sorted(case_rows, key=lambda item: case_sort_key(item["case_id"])):
        record = {key: value for key, value in row.items() if key != "segment_rows"}
        records.append(record)
    return pd.DataFrame(records)


def write_summary(
    output_path: Path,
    truth_reference: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    judgement_df: pd.DataFrame,
    requested_cases: list[str],
    manifest: dict[str, Any],
) -> None:
    control_row = case_metrics_df.loc[case_metrics_df["case_id"] == "truth_anchor_all_non_pva"].iloc[0]
    available_states = [case_release_state_name(case_id) for case_id in requested_cases if case_id != "truth_anchor_all_non_pva"]
    available_states = [state for state in available_states if state is not None]
    abnormal_df = judgement_df[judgement_df["overall_label"] == "abnormal"].copy()
    borderline_df = judgement_df[judgement_df["overall_label"] == "borderline"].copy()
    normal_df = judgement_df[judgement_df["overall_label"] == "normal"].copy()

    lines: list[str] = [
        "# data2 ESKF non-PVA state sanity summary",
        "",
        "## 1. 真值锚定控制组表现",
        (
            f"- 控制组 `truth_anchor_all_non_pva` 在固定场景 "
            f"`data2 + ESKF + corrected RTK + 300s on / 100s off / 150s on` 下，"
            f"`mean_outage_rmse_3d_m={format_metric(float(control_row['mean_outage_rmse_3d_m']))}`，"
            f"`max_outage_final_err_3d_m={format_metric(float(control_row['max_outage_final_err_3d_m']))}`。"
        ),
        (
            f"- 控制组 auxiliary overall RMSE3D="
            f"`{format_metric(float(control_row['overall_rmse_3d_m_aux']))}`，"
            f"共 `outage_segment_count={int(float(control_row['outage_segment_count']))}` 段。"
        ),
        (
            "- 关闭状态本轮语义已改为“真值锚定 + 小初始/过程噪声”，"
            "不再使用状态 mask freeze。"
        ),
        "",
        "## 2. 最异常的状态列表",
    ]

    if abnormal_df.empty and borderline_df.empty:
        lines.append("- 当前已运行释放状态中，尚未出现 `abnormal/borderline`。")
    else:
        ranked_df = pd.concat([abnormal_df, borderline_df], ignore_index=True)
        ranked_df["severity_score"] = ranked_df["overall_label"].map(LABEL_TO_SCORE)
        ranked_df = ranked_df.sort_values(
            by=["severity_score", "delta_mean_rmse3d", "delta_max_final3d"],
            ascending=[False, False, False],
        )
        for _, row in ranked_df.head(8).iterrows():
            state_name = str(row["state_name"])
            lines.append(
                f"- `{state_name}`: overall=`{row['overall_label']}`, "
                f"behavior=`{row['behavior_label']}`, impact=`{row['impact_label']}`, "
                f"`delta_mean_rmse3d={format_metric(float(row['delta_mean_rmse3d']))}`, "
                f"`delta_max_final3d={format_metric(float(row['delta_max_final3d']))}`。"
            )

    lines.extend(
        [
            "",
            "## 3. 表现正常、可继续保留在线估计的状态",
        ]
    )
    if normal_df.empty:
        lines.append("- 当前已运行释放状态中，暂无可直接判为 `normal` 的状态。")
    else:
        normal_states = normal_df["state_name"].tolist()
        lines.append("- " + "、".join(f"`{state}`" for state in normal_states))

    lines.extend(
        [
            "",
            "## 4. 建议下一步冻结或重调的状态",
        ]
    )
    recommendations: list[str] = []
    if not abnormal_df.empty:
        recommendations.append(
            "- 对 `abnormal` 状态优先做针对性复查：先看初始化/单位/过程噪声映射，再看该状态观测方程是否在 outage 段缺少有效激励。"
        )
    if not borderline_df.empty:
        recommendations.append(
            "- 对 `borderline` 状态优先保留“真值锚定对照”作为回归基线，再做单状态噪声细扫，避免一次同时放开多个弱可观量。"
        )
    if "mounting_roll" in available_states:
        mr_rows = judgement_df[judgement_df["state_name"] == "mounting_roll"]
        if not mr_rows.empty:
            mr_row = mr_rows.iloc[0]
            recommendations.append(
                f"- `mounting_roll` 需结合 `std_mr` 诊断一起看；当前 "
                f"`diag_head_std_mr={format_metric(float(mr_row['diag_head_std_mr']))}`，"
                f"`diag_tail_std_mr={format_metric(float(mr_row['diag_tail_std_mr']))}`。"
            )
    if "release_odo_scale" in requested_cases:
        recommendations.append(
            "- `release_odo_scale` 为兼容当前初始化保护，实验里使用 `odo_scale=1e-6` 代替精确 `0`；该假设已记录到 manifest。"
        )
    if not recommendations:
        recommendations.append("- 当前只完成了 smoke subset，建议先扩展到全 23 case 后再冻结最终策略。")
    lines.extend(recommendations)

    lines.extend(
        [
            "",
            "## Coverage",
            f"- requested_cases: {', '.join(f'`{case_id}`' for case_id in requested_cases)}",
            f"- judged_states: {', '.join(f'`{state}`' for state in available_states) if available_states else 'none'}",
            f"- truth_reference: `{manifest['truth_reference_json']}`",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- state_judgement: `{manifest['state_judgement_csv']}`",
            "",
            "## Notes",
            (
                f"- GNSS lever official nominal (`{truth_reference['sources']['gnss_lever_truth']['imu_model']}`): "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_truth']['value_m'][0]))}`, "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_truth']['value_m'][1]))}`, "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_truth']['value_m'][2]))}` m."
            ),
            (
                "- GNSS lever diagnostic RTK/POS body median (do not use for ranking): "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_diagnostic_body_median']['value_m'][0]))}`, "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_diagnostic_body_median']['value_m'][1]))}`, "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_diagnostic_body_median']['value_m'][2]))}` m."
            ),
            (
                f"- manifest freshness: `{manifest['generated_at']}`; "
                f"schedule=`300s on / 100s off / 150s on`."
            ),
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(
        description="Run data2 ESKF non-PVA truth-anchor control + single-state release sanity matrix."
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=Path("config_data2_baseline_eskf.yaml"),
        help="Baseline config relative to repo root.",
    )
    parser.add_argument(
        "--exe",
        type=Path,
        default=Path("build/Release/eskf_fusion.exe"),
        help="Solver executable relative to repo root.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/data2_eskf_state_sanity"),
        help="Experiment output directory relative to repo root.",
    )
    parser.add_argument(
        "--exp-id",
        default=f"EXP-{today}-data2-state-sanity-r1",
        help="Experiment identifier recorded in manifest.",
    )
    parser.add_argument(
        "--initial-on",
        type=float,
        default=300.0,
        help="Initial GNSS on duration in seconds.",
    )
    parser.add_argument("--off", type=float, default=100.0, help="GNSS off duration in seconds.")
    parser.add_argument("--on", type=float, default=150.0, help="GNSS on duration in seconds after each outage.")
    parser.add_argument("--cycle-start", choices=("off", "on"), default="off")
    parser.add_argument(
        "--cases",
        nargs="*",
        help="Optional subset of case ids. Control is auto-inserted when omitted.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    args.case_ids = requested_case_ids(args.cases)
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    outage_gnss_path = args.artifacts_dir / "GNSS_outage_cycle_rtk_300on_100off_150on.txt"
    filter_stats = filter_gnss(
        input_path=(REPO_ROOT / "dataset/data2/rtk.txt").resolve(),
        output_path=outage_gnss_path,
        start_time=float(base_cfg["fusion"]["starttime"]),
        final_time=float(base_cfg["fusion"]["finaltime"]),
        initial_on=args.initial_on,
        on_dur=args.on,
        off_dur=args.off,
        cycle_starts_with=args.cycle_start,
    )
    gnss_stats_path = args.artifacts_dir / "gnss_outage_stats.json"
    gnss_stats_path.write_text(json.dumps(json_safe(filter_stats), indent=2, ensure_ascii=False), encoding="utf-8")

    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_segment_paths: dict[str, str] = {}
    release_state_rows: list[dict[str, Any]] = []

    for case_id in args.case_ids:
        case_dir = args.case_root / case_id
        ensure_dir(case_dir)
        cfg_path = write_case_config(base_cfg, truth_reference, case_id, case_dir, outage_gnss_path)
        case_config_paths[case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_row = run_case(case_id=case_id, cfg_path=cfg_path, case_dir=case_dir, exe_path=args.exe)
        segment_path = write_case_segments(case_dir, case_id, case_row["segment_rows"])
        case_row["outage_segments_path"] = rel_from_root(segment_path, REPO_ROOT)
        case_segment_paths[case_id] = case_row["outage_segments_path"]
        case_rows.append(case_row)

    control_candidates = [row for row in case_rows if row["case_id"] == "truth_anchor_all_non_pva"]
    if not control_candidates:
        raise RuntimeError("control case truth_anchor_all_non_pva was not executed")
    control_row = control_candidates[0]

    for case_row in sorted(case_rows, key=lambda item: case_sort_key(item["case_id"])):
        if case_row["case_id"] == "truth_anchor_all_non_pva":
            continue
        state_name = case_release_state_name(case_row["case_id"])
        release_state_rows.append(evaluate_state_case(state_name, case_row, control_row, truth_reference))
        control_state_path = (REPO_ROOT / control_row["state_series_path"]).resolve()
        case_state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        plot_state_comparison(
            state_name=state_name,
            control_state_path=control_state_path,
            case_state_path=case_state_path,
            truth_reference=truth_reference,
            output_path=args.plot_dir / f"{state_name}_control_vs_release.png",
        )
        if state_name == "mounting_roll":
            plot_mounting_roll_std(
                control_diag_path=(REPO_ROOT / control_row["diag_path"]).resolve(),
                case_diag_path=(REPO_ROOT / case_row["diag_path"]).resolve(),
                output_path=args.plot_dir / "mounting_roll_std_compare.png",
            )

    case_metrics_df = build_case_metrics_table(case_rows)
    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    judgement_df = pd.DataFrame(release_state_rows)
    if not judgement_df.empty:
        judgement_df = judgement_df.sort_values(by="state_name").reset_index(drop=True)
        plot_heatmap(judgement_df, args.plot_dir / "state_judgement_heatmap.png")
    state_judgement_path = args.output_dir / "state_judgement.csv"
    judgement_df.to_csv(state_judgement_path, index=False, encoding="utf-8-sig")

    freshness: dict[str, str] = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_judgement_csv": dt.datetime.fromtimestamp(state_judgement_path.stat().st_mtime).isoformat(timespec="seconds"),
        "gnss_outage_stats_json": dt.datetime.fromtimestamp(gnss_stats_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    if judgement_df.empty:
        freshness["plots_dir"] = dt.datetime.now().isoformat(timespec="seconds")
    else:
        freshness["plots_dir"] = dt.datetime.fromtimestamp((args.plot_dir / "state_judgement_heatmap.png").stat().st_mtime).isoformat(timespec="seconds")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "state_judgement_csv": rel_from_root(state_judgement_path, REPO_ROOT),
        "gnss_outage_stats_json": rel_from_root(gnss_stats_path, REPO_ROOT),
        "truth_catalog_source": truth_reference["sources"],
        "requested_case_ids": args.case_ids,
        "case_config_paths": case_config_paths,
        "case_outage_segments_paths": case_segment_paths,
        "freshness": freshness,
        "outage_schedule": {
            "initial_on_s": args.initial_on,
            "off_s": args.off,
            "on_s": args.on,
            "cycle_start": args.cycle_start,
            "first_off_start_time": filter_stats.get("first_off_start_time"),
            "first_off_end_time": filter_stats.get("first_off_end_time"),
        },
        "assumptions": [
            "关闭状态语义为真值锚定 + 小初始/过程噪声，而非 mask freeze。",
            "mounting 状态按相对 constraints.imu_mounting_angle 的状态坐标处理。",
            "release_odo_scale 使用 1e-6 代替精确 0，以兼容当前初始化保护逻辑。",
        ],
    }

    summary_path = args.output_dir / "summary.md"
    write_summary(
        output_path=summary_path,
        truth_reference=truth_reference,
        case_metrics_df=case_metrics_df,
        judgement_df=judgement_df,
        requested_cases=args.case_ids,
        manifest=manifest,
    )
    freshness["summary_md"] = dt.datetime.fromtimestamp(summary_path.stat().st_mtime).isoformat(timespec="seconds")

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
