from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_lever_truth_init_probe import (
    AXIS_ORDER,
    EARLY_WINDOW_S,
    FIRST_CHANGE_THRESHOLD_M,
    detect_first_change,
    plot_gnss_lever_deviation,
    plot_gnss_lever_first_window,
    plot_gnss_lever_truth_series,
)
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    STATE_META,
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    family_anchored_process_internal,
    family_target_std_human,
    format_metric,
    get_group_instability_internal,
    get_group_vector_internal,
    human_to_internal,
    json_safe,
    reset_directory,
    run_command,
    static_behavior_label,
)


CASE_ID = "release_gnss_lever_truth_imu_params"
GNSS_LEVER_COLUMNS = {
    "x": "gnss_lever_x_m",
    "y": "gnss_lever_y_m",
    "z": "gnss_lever_z_m",
}
REFERENCE_PROBES = {
    "truth_init_free_run": "output/data2_ins_gnss_lever_truth_init_probe/summary.md",
    "runtime_anchor_stage_b": "output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/summary.md",
}
IMU_GROUP_COLUMNS = {
    "ba": ["ba_x_mgal", "ba_y_mgal", "ba_z_mgal"],
    "bg": ["bg_x_degh", "bg_y_degh", "bg_z_degh"],
    "sg": ["sg_x_ppm", "sg_y_ppm", "sg_z_ppm"],
    "sa": ["sa_x_ppm", "sa_y_ppm", "sa_z_ppm"],
}
IMU_GROUP_UNITS = {
    "ba": "mGal",
    "bg": "deg/h",
    "sg": "ppm",
    "sa": "ppm",
}
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_lever_truth_imu_params_probe_pos320_mainline")
EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-lever-truth-imu-params-r4-pos320-mainline"
IMU_MODEL_DEFAULT = "POS-320"


def parse_data2_readme_imu_params(readme_path: Path, imu_model: str) -> dict[str, Any]:
    lines = readme_path.read_text(encoding="utf-8").splitlines()
    model_key = imu_model.strip().lower().replace("-", "").replace(" ", "")
    for line in lines:
        striped = line.strip()
        if not striped.startswith("|") or "Angle Random Walk" in striped or ":---" in striped:
            continue
        parts = [part.strip() for part in striped.strip("|").split("|")]
        if len(parts) != 8:
            continue
        row_key = parts[0].lower().replace("-", "").replace(" ", "")
        if row_key != model_key:
            continue
        return {
            "imu_model": imu_model,
            "source": rel_from_root(readme_path, REPO_ROOT),
            "sigma_arw_deg_sqrt_hr": float(parts[1]),
            "sigma_vrw_mps_sqrt_hr": float(parts[2]),
            "sigma_bg_degh": float(parts[3]),
            "sigma_ba_mgal": float(parts[4]),
            "corr_time_hr": float(parts[5]),
            "sigma_sg_ppm": float(parts[6]),
            "sigma_sa_ppm": float(parts[7]),
            "corr_time_s": float(parts[5]) * 3600.0,
        }
    raise RuntimeError(f"failed to locate IMU model `{imu_model}` in {readme_path}")


def imu_truth_model_internal(readme_params: dict[str, Any]) -> dict[str, float]:
    return {
        "sigma_ba": human_to_internal("ba", float(readme_params["sigma_ba_mgal"])),
        "sigma_bg": human_to_internal("bg", float(readme_params["sigma_bg_degh"])),
        "sigma_sg": human_to_internal("sg", float(readme_params["sigma_sg_ppm"])),
        "sigma_sa": human_to_internal("sa", float(readme_params["sigma_sa_ppm"])),
        "corr_time_s": float(readme_params["corr_time_s"]),
    }


def modeled_truth_reference_series(
    timestamps: np.ndarray,
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
) -> pd.DataFrame:
    elapsed = timestamps - float(timestamps[0])
    corr_time = float(readme_params["corr_time_s"])
    decay = np.exp(-elapsed / corr_time)
    sigma_factor = np.sqrt(np.maximum(0.0, 1.0 - np.exp(-2.0 * elapsed / corr_time)))

    ref_df = pd.DataFrame({"timestamp": timestamps})
    sigma_map = {
        "ba": float(readme_params["sigma_ba_mgal"]),
        "bg": float(readme_params["sigma_bg_degh"]),
        "sg": float(readme_params["sigma_sg_ppm"]),
        "sa": float(readme_params["sigma_sa_ppm"]),
    }
    for group, columns in IMU_GROUP_COLUMNS.items():
        sigma_ss = sigma_map[group]
        for axis_idx, axis in enumerate(["x", "y", "z"]):
            state_name = f"{group}_{axis}"
            initial_value = float(truth_reference["states"][state_name]["reference_value"])
            mean = initial_value * decay
            bound = 3.0 * sigma_ss * sigma_factor
            ref_df[f"{group}_{axis}_mean"] = mean
            ref_df[f"{group}_{axis}_lower"] = mean - bound
            ref_df[f"{group}_{axis}_upper"] = mean + bound
    return ref_df


def plot_imu_group_series(
    state_df: pd.DataFrame,
    ref_df: pd.DataFrame,
    group: str,
    output_path: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    for axis_idx, axis in enumerate(["x", "y", "z"]):
        column = IMU_GROUP_COLUMNS[group][axis_idx]
        mean_col = f"{group}_{axis}_mean"
        lower_col = f"{group}_{axis}_lower"
        upper_col = f"{group}_{axis}_upper"
        axes[axis_idx].plot(timestamps, state_df[column].to_numpy(dtype=float), linewidth=1.1, label=f"est {group}_{axis}")
        axes[axis_idx].plot(
            ref_df["timestamp"].to_numpy(dtype=float),
            ref_df[mean_col].to_numpy(dtype=float),
            linestyle="--",
            color="black",
            linewidth=1.0,
            label="modeled truth mean",
        )
        axes[axis_idx].fill_between(
            ref_df["timestamp"].to_numpy(dtype=float),
            ref_df[lower_col].to_numpy(dtype=float),
            ref_df[upper_col].to_numpy(dtype=float),
            color="#bdbdbd",
            alpha=0.25,
            label="modeled truth ±3σ",
        )
        axes[axis_idx].set_ylabel(IMU_GROUP_UNITS[group])
        axes[axis_idx].grid(alpha=0.25)
        axes[axis_idx].legend(loc="best")
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle(f"{group} state series vs README-based modeled truth")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    case_dir: Path,
    gnss_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_noise = base_cfg["fusion"]["noise"]
    p0_diag = list(base_p0_diag_from_config(base_cfg))

    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
    first_update_path = case_dir / f"first_update_{CASE_ID}.csv"

    fusion["enable_gnss_velocity"] = False
    fusion["gnss_path"] = rel_from_root(gnss_path, REPO_ROOT)
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["first_update_debug_output_path"] = rel_from_root(first_update_path, REPO_ROOT)
    fusion["gnss_update_debug_output_path"] = ""
    fusion["gnss_schedule"] = {"enabled": False}

    ablation_cfg = default_ablation_flags()
    ablation_cfg["disable_mounting"] = True
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_gnss_lever_z"] = False
    fusion["ablation"] = ablation_cfg
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = False
    init_cfg["runtime_truth_anchor_gnss_only"] = False
    init_cfg["runtime_truth_anchor_position"] = False
    init_cfg["runtime_truth_anchor_velocity"] = False
    init_cfg["runtime_truth_anchor_attitude"] = False
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    ba_init = get_group_vector_internal(truth_reference, "ba")
    bg_init = get_group_vector_internal(truth_reference, "bg")
    sg_init = get_group_vector_internal(truth_reference, "sg")
    sa_init = get_group_vector_internal(truth_reference, "sa")
    readme_internal = imu_truth_model_internal(readme_params)
    ba_sigma = float(readme_internal["sigma_ba"])
    bg_sigma = float(readme_internal["sigma_bg"])
    sg_sigma = float(readme_internal["sigma_sg"])
    sa_sigma = float(readme_internal["sigma_sa"])
    ba_std = np.full(3, 0.1 * ba_sigma, dtype=float)
    bg_std = np.full(3, 0.1 * bg_sigma, dtype=float)
    sg_std = np.full(3, 0.1 * sg_sigma, dtype=float)
    sa_std = np.full(3, 0.1 * sa_sigma, dtype=float)
    ba_noise = np.full(3, ba_sigma, dtype=float)
    bg_noise = np.full(3, bg_sigma, dtype=float)
    sg_noise = np.full(3, sg_sigma, dtype=float)
    sa_noise = np.full(3, sa_sigma, dtype=float)

    init_cfg["ba0"] = [float(x) for x in ba_init]
    init_cfg["bg0"] = [float(x) for x in bg_init]
    init_cfg["sg0"] = [float(x) for x in sg_init]
    init_cfg["sa0"] = [float(x) for x in sa_init]
    p0_diag[9:12] = [float(x * x) for x in ba_std]
    p0_diag[12:15] = [float(x * x) for x in bg_std]
    p0_diag[15:18] = [float(x * x) for x in sg_std]
    p0_diag[18:21] = [float(x * x) for x in sa_std]

    init_cfg["gnss_lever_arm0"] = [0.0, 0.0, 0.0]
    gnss_lever_std = np.full(3, family_target_std_human("gnss_lever", 0), dtype=float)
    gnss_lever_noise = np.full(3, float(base_noise["sigma_gnss_lever_arm"]), dtype=float)
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
    noise_cfg["sigma_gnss_lever_arm"] = float(np.max(gnss_lever_noise))
    noise_cfg["sigma_gnss_lever_arm_vec"] = [float(x) for x in gnss_lever_noise]
    noise_cfg["markov_corr_time"] = float(readme_internal["corr_time_s"])

    overrides = {
        "fusion.enable_gnss_velocity": False,
        "fusion.gnss_path": rel_from_root(gnss_path, REPO_ROOT),
        "fusion.gnss_schedule.enabled": False,
        "fusion.constraints.enable_odo": False,
        "fusion.constraints.enable_nhc": False,
        "fusion.constraints.enable_diagnostics": True,
        "fusion.ablation.disable_mounting": True,
        "fusion.ablation.disable_odo_lever_arm": True,
        "fusion.ablation.disable_odo_scale": True,
        "fusion.init.use_truth_pva": True,
        "fusion.init.runtime_truth_anchor_pva": False,
        "fusion.init.ba0": [float(x) for x in ba_init],
        "fusion.init.bg0": [float(x) for x in bg_init],
        "fusion.init.sg0": [float(x) for x in sg_init],
        "fusion.init.sa0": [float(x) for x in sa_init],
        "fusion.init.gnss_lever_arm0": [0.0, 0.0, 0.0],
        "fusion.init.P0_diag[9:21]": [float(x) for x in p0_diag[9:21]],
        "fusion.init.P0_diag[28:31]": [float(x) for x in p0_diag[28:31]],
        "fusion.noise.sigma_ba_vec": [float(x) for x in ba_noise],
        "fusion.noise.sigma_bg_vec": [float(x) for x in bg_noise],
        "fusion.noise.sigma_sg_vec": [float(x) for x in sg_noise],
        "fusion.noise.sigma_sa_vec": [float(x) for x in sa_noise],
        "fusion.noise.markov_corr_time": float(readme_internal["corr_time_s"]),
        "fusion.noise.sigma_gnss_lever_arm": float(np.max(gnss_lever_noise)),
        "fusion.noise.sigma_gnss_lever_arm_vec": [float(x) for x in gnss_lever_noise],
    }
    return cfg, overrides


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    case_dir: Path,
    gnss_path: Path,
) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(base_cfg, truth_reference, readme_params, case_dir, gnss_path)
    cfg_path = case_dir / f"config_{CASE_ID}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides


def run_probe_case(case_dir: Path, cfg_path: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
    first_update_path = case_dir / f"first_update_{CASE_ID}.csv"
    stdout_path = case_dir / f"{CASE_ID}.stdout.txt"
    diag_path = case_dir / f"DIAG_{CASE_ID}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output: {state_series_path}")
    if not first_update_path.exists():
        raise RuntimeError(f"missing first update debug output: {first_update_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {CASE_ID}")
    shutil.copy2(root_diag, diag_path)

    row: dict[str, Any] = {
        "case_id": CASE_ID,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "first_update_path": rel_from_root(first_update_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "first_update_mtime": dt.datetime.fromtimestamp(first_update_path.stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(diag_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(evaluate_navigation_metrics(cfg_path, sol_path))
    return row


def summarize_axis_behavior(
    state_name: str,
    timestamps: np.ndarray,
    values: np.ndarray,
    truth_reference: dict[str, Any],
    first_update_row: pd.Series,
) -> dict[str, Any]:
    truth_value = float(truth_reference["states"][state_name]["reference_value"])
    initial_value = float(values[0])
    first_change_time, first_change_value = detect_first_change(timestamps, values)
    if math.isfinite(first_change_time):
        early_mask = (timestamps >= first_change_time) & (timestamps <= first_change_time + EARLY_WINDOW_S)
    else:
        early_mask = timestamps <= timestamps[0] + EARLY_WINDOW_S
    if not np.any(early_mask):
        early_mask = np.ones_like(timestamps, dtype=bool)
    early_plateau_median = float(np.median(values[early_mask]))
    final_value = float(values[-1])
    final_abs_error = float(abs(final_value - truth_value))
    early_abs_error = float(abs(early_plateau_median - truth_value))
    label, behavior_metrics = static_behavior_label(state_name, truth_reference, values)

    lever_before = float(first_update_row["lever_before"])
    lever_after = float(first_update_row["lever_after"])
    dx_gnss_lever = float(first_update_row["dx_gnss_lever"])
    after_minus_before = float(lever_after - lever_before)

    return {
        "state_name": state_name,
        "axis": state_name[-1],
        "truth_source": truth_reference["states"][state_name]["source"],
        "initial_value": initial_value,
        "truth_value": truth_value,
        "first_change_time": first_change_time,
        "first_change_value": first_change_value,
        "early_plateau_median_10s": early_plateau_median,
        "early_plateau_abs_error_m": early_abs_error,
        "final_value": final_value,
        "final_abs_error_m": final_abs_error,
        "behavior_label": label,
        "recovery_ratio": float(behavior_metrics["recovery_ratio"]),
        "range_m": float(behavior_metrics["range"]),
        "lever_before": lever_before,
        "lever_after": lever_after,
        "dx_gnss_lever": dx_gnss_lever,
        "after_minus_before": after_minus_before,
        "dx_consistency_error_m": float(abs(after_minus_before - dx_gnss_lever)),
    }


def build_lever_metrics(
    truth_reference: dict[str, Any],
    state_series_path: Path,
    first_update_path: Path,
) -> pd.DataFrame:
    state_df = pd.read_csv(
        state_series_path,
        usecols=["timestamp", "gnss_lever_x_m", "gnss_lever_y_m", "gnss_lever_z_m"],
    )
    first_update_df = pd.read_csv(first_update_path)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    rows: list[dict[str, Any]] = []
    for axis in AXIS_ORDER:
        state_name = f"gnss_lever_{axis}"
        first_update_row = first_update_df.loc[first_update_df["gnss_axis"] == axis].iloc[0]
        values = state_df[GNSS_LEVER_COLUMNS[axis]].to_numpy(dtype=float)
        rows.append(summarize_axis_behavior(state_name, timestamps, values, truth_reference, first_update_row))
    return pd.DataFrame(rows).sort_values(by="axis").reset_index(drop=True)


def write_summary(
    output_path: Path,
    probe_row: dict[str, Any],
    lever_df: pd.DataFrame,
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    manifest: dict[str, Any],
) -> None:
    lines: list[str] = [
        "# data2 INS/GNSS GNSS lever probe with truth bias/scale modeling",
        "",
        "## 1. 本轮配置",
        (
            "- 纯 `INS/GNSS` 口径：`enable_odo=false`、`enable_nhc=false`、"
            "`disable_mounting=true`、`disable_odo_lever_arm=true`、`disable_odo_scale=true`、"
            "`enable_gnss_velocity=false`、`gnss_path=dataset/data2/rtk.txt`。"
        ),
        (
            "- `PVA` 不做 runtime truth anchor："
            "`use_truth_pva=true`，`runtime_truth_anchor_pva=false`。"
        ),
        (
            "- `ba/bg/sg/sa` 全部按真值锚定建模：真值初值 + 小 `P0/Q`；"
            "`GNSS lever` 三轴从零初值释放，使用较大的 `P0/Q` 在线估计。"
        ),
        (
            f"- `ba/bg/sg/sa` 的动态模型参数取自 `{readme_params['source']}` 的 "
            f"`{readme_params['imu_model']}` 行："
            f"`sigma_bg={format_metric(float(readme_params['sigma_bg_degh']))} deg/h`, "
            f"`sigma_ba={format_metric(float(readme_params['sigma_ba_mgal']))} mGal`, "
            f"`sigma_sg={format_metric(float(readme_params['sigma_sg_ppm']))} ppm`, "
            f"`sigma_sa={format_metric(float(readme_params['sigma_sa_ppm']))} ppm`, "
            f"`corr_time={format_metric(float(readme_params['corr_time_hr']))} h`。"
        ),
        f"- comparison references: `{REFERENCE_PROBES['truth_init_free_run']}`、`{REFERENCE_PROBES['runtime_anchor_stage_b']}`",
        "",
        "## 2. GNSS 杆臂是否能在无 PVA 锚定下恢复到真值附近",
    ]

    for _, row in lever_df.iterrows():
        lines.append(
            f"- `{row['state_name']}`: behavior=`{row['behavior_label']}`, "
            f"`truth={format_metric(float(row['truth_value']))}` m, "
            f"`early_plateau_10s={format_metric(float(row['early_plateau_median_10s']))}` m, "
            f"`final={format_metric(float(row['final_value']))}` m, "
            f"`final_abs_error={format_metric(float(row['final_abs_error_m']))}` m, "
            f"`recovery_ratio={format_metric(float(row['recovery_ratio']))}`."
        )

    lines.extend(["", "## 3. 首个 GNSS_POS 更新是否仍形成错误平台"])
    for _, row in lever_df.iterrows():
        lines.append(
            f"- `{row['state_name']}`: "
            f"`lever_before={format_metric(float(row['lever_before']))}` m, "
            f"`lever_after={format_metric(float(row['lever_after']))}` m, "
            f"`dx={format_metric(float(row['dx_gnss_lever']))}` m, "
            f"`early_abs_error={format_metric(float(row['early_plateau_abs_error_m']))}` m, "
            f"`dx_consistency_error={format_metric(float(row['dx_consistency_error_m']))}` m."
        )

    abnormal_df = lever_df[lever_df["behavior_label"] == "abnormal"]
    borderline_df = lever_df[lever_df["behavior_label"] == "borderline"]
    max_final_abs_error = float(lever_df["final_abs_error_m"].max())

    lines.extend(["", "## 4. 结果解读"])
    if abnormal_df.empty and borderline_df.empty and max_final_abs_error <= 0.05:
        lines.append(
            "- 当前结果说明：即使没有 runtime `PVA` 锚定，只要把 `ba/bg/sg/sa` 作为真值已知建模，"
            "`GNSS lever` 也能基本恢复到真值附近。"
        )
    elif abnormal_df.empty and borderline_df.empty:
        worst_row = lever_df.loc[lever_df["final_abs_error_m"].idxmax()]
        lines.append(
            "- 当前结果更准确的表述是“可以明显恢复，但尚未完全贴近真值”。"
            f" 其中偏差最大的仍是 `{worst_row['state_name']}`，最终绝对误差约 "
            f"`{format_metric(float(worst_row['final_abs_error_m']))}` m。"
        )
    else:
        labels = abnormal_df["state_name"].tolist() + borderline_df["state_name"].tolist()
        lines.append(
            "- 当前仍有未恢复到正常范围的轴："
            + "、".join(f"`{name}`" for name in labels)
            + "。这说明只修正 `ba/bg/sg/sa` 还不足以完全消除杆臂偏差。"
        )

    if float(probe_row["nav_rmse_3d_m"]) <= 0.05:
        lines.append(
            "- 导航误差已压到较低水平，若杆臂仍有明显平台偏差，更应优先怀疑 `GNSS_POS residual / free PVA shared correction`，"
            "而不是单纯归因于零偏比例因子本身。"
        )
    else:
        lines.append(
            "- 导航误差仍不低，因此杆臂异常可能同时混有姿态/位置传播误差，不能只用本轮就排除 `PVA shared correction`。"
        )

    lines.extend(
        [
            "",
            "## 5. Notes",
            (
                "- 本轮使用 `dataset/data2/rtk.txt`，它不含 GNSS 速度列；"
                "因此 `GNSS_POS` 首更仍可能带有已知的时间对齐残差。"
            ),
            "- 图像现已同时导出 `ba/bg/sg/sa` 四组状态曲线，并附 README 参数驱动的 Markov 均值轨迹与 `±3σ` 包络。",
            "- `behavior_label` 采用的是“从零初值恢复到真值比例”的静态判据，"
            "不等同于厘米级绝对精度判定；解释时仍需同时看 `final_abs_error_m`。",
            f"- nav_rmse_3d_m=`{format_metric(float(probe_row['nav_rmse_3d_m']))}`",
            f"- nav_final_err_3d_m=`{format_metric(float(probe_row['nav_final_err_3d_m']))}`",
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
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a data2 INS/GNSS-only probe with truth bias/scale modeling and free GNSS lever estimation."
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
        default=OUTPUT_DIR_DEFAULT,
        help="Output directory relative to repo root.",
    )
    parser.add_argument(
        "--gnss-path",
        type=Path,
        default=Path("dataset/data2/rtk.txt"),
        help="GNSS file used by the probe.",
    )
    parser.add_argument(
        "--imu-path",
        type=Path,
        default=None,
        help="Optional IMU text path overriding fusion.imu_path in the base config.",
    )
    parser.add_argument(
        "--exp-id",
        default=EXP_ID_DEFAULT,
        help="Experiment identifier recorded in manifest.",
    )
    parser.add_argument(
        "--readme-path",
        type=Path,
        default=Path("dataset/data2/README.md"),
        help="README used to derive IMU Markov parameters.",
    )
    parser.add_argument(
        "--imu-model",
        default=IMU_MODEL_DEFAULT,
        help="IMU model row name in README used for bias/scale truth modeling.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.gnss_path = (REPO_ROOT / args.gnss_path).resolve()
    if args.imu_path is not None:
        args.imu_path = (REPO_ROOT / args.imu_path).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")
    if not args.gnss_path.exists():
        raise FileNotFoundError(f"missing GNSS file: {args.gnss_path}")
    if args.imu_path is not None and not args.imu_path.exists():
        raise FileNotFoundError(f"missing IMU file: {args.imu_path}")
    if not args.readme_path.exists():
        raise FileNotFoundError(f"missing README file: {args.readme_path}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    if args.imu_path is not None:
        base_cfg.setdefault("fusion", {})["imu_path"] = rel_from_root(args.imu_path, REPO_ROOT)
    truth_reference = build_truth_reference(base_cfg)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    case_dir = args.case_root / CASE_ID
    ensure_dir(case_dir)
    cfg_path, overrides = write_case_config(base_cfg, truth_reference, readme_params, case_dir, args.gnss_path)
    probe_row = run_probe_case(case_dir=case_dir, cfg_path=cfg_path, exe_path=args.exe)

    state_series_path = (REPO_ROOT / probe_row["state_series_path"]).resolve()
    first_update_path = (REPO_ROOT / probe_row["first_update_path"]).resolve()
    state_df = pd.read_csv(
        state_series_path,
        usecols=[
            "timestamp",
            "gnss_lever_x_m",
            "gnss_lever_y_m",
            "gnss_lever_z_m",
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
        ],
    )
    lever_df = build_lever_metrics(truth_reference, state_series_path, first_update_path)
    imu_ref_df = modeled_truth_reference_series(state_df["timestamp"].to_numpy(dtype=float), truth_reference, readme_params)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    pd.DataFrame([probe_row]).to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    lever_metrics_path = args.output_dir / "lever_metrics.csv"
    lever_df.to_csv(lever_metrics_path, index=False, encoding="utf-8-sig")
    imu_truth_model_path = args.output_dir / "imu_truth_model_reference.csv"
    imu_ref_df.to_csv(imu_truth_model_path, index=False, encoding="utf-8-sig")

    plot_gnss_lever_truth_series(state_df, truth_reference, args.plot_dir / "gnss_lever_xyz_vs_truth.png")
    plot_gnss_lever_first_window(state_df, truth_reference, args.plot_dir / "gnss_lever_xyz_first_10s.png")
    plot_gnss_lever_deviation(state_df, truth_reference, args.plot_dir / "gnss_lever_xyz_deviation.png")
    for group in ["ba", "bg", "sg", "sa"]:
        plot_imu_group_series(state_df, imu_ref_df, group, args.plot_dir / f"{group}_modeled_truth_compare.png")

    freshness = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "config_yaml": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "lever_metrics_csv": dt.datetime.fromtimestamp(lever_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "imu_truth_model_reference_csv": dt.datetime.fromtimestamp(imu_truth_model_path.stat().st_mtime).isoformat(timespec="seconds"),
        "plots_dir": dt.datetime.fromtimestamp((args.plot_dir / "gnss_lever_xyz_vs_truth.png").stat().st_mtime).isoformat(timespec="seconds"),
    }

    manifest_path = args.output_dir / "manifest.json"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "imu_path_effective": base_cfg.get("fusion", {}).get("imu_path", ""),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "gnss_path": rel_from_root(args.gnss_path, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "lever_metrics_csv": rel_from_root(lever_metrics_path, REPO_ROOT),
        "imu_truth_model_reference_csv": rel_from_root(imu_truth_model_path, REPO_ROOT),
        "comparison_references": REFERENCE_PROBES,
        "truth_catalog_source": truth_reference["sources"],
        "imu_truth_model_from_readme": readme_params,
        "exact_overrides": overrides,
        "case_config_path": rel_from_root(cfg_path, REPO_ROOT),
        "case_artifacts": {
            "sol_path": probe_row["sol_path"],
            "state_series_path": probe_row["state_series_path"],
            "first_update_path": probe_row["first_update_path"],
            "diag_path": probe_row["diag_path"],
            "stdout_path": probe_row["stdout_path"],
        },
        "freshness": freshness,
        "mode": "ins_gnss_gnss_lever_truth_imu_params_probe",
        "assumptions": [
            "本轮按“零偏和比例因子带真值进行建模”解释为：ba/bg/sg/sa 使用已有真值初值，并把过程模型参数切换到 data2 README 的 IMU 参数。",
            "runtime PVA truth anchor 保持关闭，只保留 use_truth_pva=true 的真值首点初始化。",
            "mounting/odo_scale/odo_lever 继续禁用，不纳入本轮解释。",
            "默认采用经数据来源确认的 IMULog/POS-320 主线，并读取 README `POS-320` 行作为 IMU 参数来源。",
            "GNSS lever 判优真值采用 README `POS320` 标称外参；RTK/POS body-frame median 仅作诊断参考。",
        ],
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
    }
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    summary_path = args.output_dir / "summary.md"
    write_summary(summary_path, probe_row, lever_df, truth_reference, readme_params, manifest)

    print(json.dumps(json_safe({
        "exp_id": args.exp_id,
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "summary": rel_from_root(summary_path, REPO_ROOT),
        "manifest": rel_from_root(manifest_path, REPO_ROOT),
        "case_metrics": rel_from_root(case_metrics_path, REPO_ROOT),
        "lever_metrics": rel_from_root(lever_metrics_path, REPO_ROOT),
        "imu_truth_model_reference": rel_from_root(imu_truth_model_path, REPO_ROOT),
    }), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
