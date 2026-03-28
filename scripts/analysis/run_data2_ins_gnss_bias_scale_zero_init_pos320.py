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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import (
    IMU_GROUP_COLUMNS,
    IMU_GROUP_UNITS,
    imu_truth_model_internal,
    modeled_truth_reference_series,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    STATE_META,
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    format_metric,
    get_group_vector_internal,
    json_safe,
    reset_directory,
    run_command,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-bias-scale-zero-init-pos320-r3"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_bias_scale_zero_init_pos320_r3")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
README_PATH_DEFAULT = Path("dataset/data2/README.md")
IMU_MODEL_DEFAULT = "POS-320"
IMU_P0_SCALE_DEFAULT = 1.0
Y_PRIOR_SCALE_DEFAULT = 0.10
ACTIVITY_FLOOR_SCALE_DEFAULT = 0.05
ACTIVITY_CEILING_SCALE_DEFAULT = 3.0
CASE_ID = "zero_init_pos320_gm_large_p0"
IMU_GROUP_ORDER = ["ba", "bg", "sg", "sa"]
AXIS_ORDER = ["x", "y", "z"]
IMU_STATE_ORDER = [f"{group}_{axis}" for group in IMU_GROUP_ORDER for axis in AXIS_ORDER]


def system_markov_sigma_note(readme_params: dict[str, Any]) -> str:
    return (
        f"{readme_params['source']} {readme_params['imu_model']}: "
        f"ba sigma={float(readme_params['sigma_ba_mgal']):g} mGal, "
        f"bg sigma={float(readme_params['sigma_bg_degh']):g} deg/h, "
        f"sg sigma={float(readme_params['sigma_sg_ppm']):g} ppm, "
        f"sa sigma={float(readme_params['sigma_sa_ppm']):g} ppm, "
        f"corr_time={float(readme_params['corr_time_hr']):g} h"
    )


def large_zero_init_std(reference_internal: np.ndarray, sigma_internal: np.ndarray, prior_scale: float) -> np.ndarray:
    base = np.maximum(np.abs(reference_internal), 3.0 * sigma_internal)
    return np.asarray(float(prior_scale) * base, dtype=float)


def downsample_for_plot(x: np.ndarray, y: np.ndarray, max_points: int = 5000) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= max_points:
        return x, y
    idx = np.linspace(0, x.size - 1, max_points, dtype=int)
    return x[idx], y[idx]


def plot_imu_group_estimated_only(
    state_df: pd.DataFrame,
    group: str,
    output_path: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    elapsed_min = (timestamps - float(timestamps[0])) / 60.0
    for axis_idx, axis in enumerate(AXIS_ORDER):
        column = IMU_GROUP_COLUMNS[group][axis_idx]
        values = state_df[column].to_numpy(dtype=float)
        plot_x, plot_y = downsample_for_plot(elapsed_min, values)
        axes[axis_idx].plot(plot_x, plot_y, linewidth=1.1, color="#1f77b4", label=f"est {group}_{axis}")
        axes[axis_idx].axhline(0.0, linestyle="--", color="black", linewidth=0.9, alpha=0.6)
        axes[axis_idx].set_ylabel(IMU_GROUP_UNITS[group])
        axes[axis_idx].grid(alpha=0.25)
        axes[axis_idx].legend(loc="best")
    axes[-1].set_xlabel("elapsed time [min]")
    fig.suptitle(f"{group} state series (estimated only)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    case_dir: Path,
    imu_p0_scale: float,
    y_prior_scale: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    p0_diag = list(base_p0_diag_from_config(base_cfg))

    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
    stdout_path = case_dir / f"{CASE_ID}.stdout.txt"
    diag_path = case_dir / f"DIAG_{CASE_ID}.txt"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["gnss_schedule"] = {"enabled": False}
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

    ablation_cfg = default_ablation_flags()
    ablation_cfg["disable_mounting"] = True
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_gnss_lever_arm"] = True
    ablation_cfg["disable_gnss_lever_z"] = False
    fusion["ablation"] = ablation_cfg

    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_diagnostics"] = False
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

    readme_internal = imu_truth_model_internal(readme_params)
    imu_sigma_internal = {
        "ba": np.full(3, float(readme_internal["sigma_ba"]), dtype=float),
        "bg": np.full(3, float(readme_internal["sigma_bg"]), dtype=float),
        "sg": np.full(3, float(readme_internal["sigma_sg"]), dtype=float),
        "sa": np.full(3, float(readme_internal["sigma_sa"]), dtype=float),
    }
    imu_reference_internal = {
        "ba": get_group_vector_internal(truth_reference, "ba"),
        "bg": get_group_vector_internal(truth_reference, "bg"),
        "sg": get_group_vector_internal(truth_reference, "sg"),
        "sa": get_group_vector_internal(truth_reference, "sa"),
    }
    imu_std_internal = {
        group: large_zero_init_std(imu_reference_internal[group], sigma_internal, imu_p0_scale)
        for group, sigma_internal in imu_sigma_internal.items()
    }
    for group in IMU_GROUP_ORDER:
        imu_std_internal[group][1] *= float(y_prior_scale)
    zero_init = np.zeros(3, dtype=float)
    gnss_lever_ref = get_group_vector_internal(truth_reference, "gnss_lever")

    init_cfg["ba0"] = [float(x) for x in zero_init]
    init_cfg["bg0"] = [float(x) for x in zero_init]
    init_cfg["sg0"] = [float(x) for x in zero_init]
    init_cfg["sa0"] = [float(x) for x in zero_init]
    init_cfg["gnss_lever_arm0"] = [float(x) for x in gnss_lever_ref]
    init_cfg["std_ba"] = [float(x) for x in imu_std_internal["ba"]]
    init_cfg["std_bg"] = [float(x) for x in imu_std_internal["bg"]]
    init_cfg["std_sg"] = [float(x) for x in imu_std_internal["sg"]]
    init_cfg["std_sa"] = [float(x) for x in imu_std_internal["sa"]]
    init_cfg["std_gnss_lever_arm"] = [0.0, 0.0, 0.0]

    p0_diag[9:12] = [float(x * x) for x in imu_std_internal["ba"]]
    p0_diag[12:15] = [float(x * x) for x in imu_std_internal["bg"]]
    p0_diag[15:18] = [float(x * x) for x in imu_std_internal["sg"]]
    p0_diag[18:21] = [float(x * x) for x in imu_std_internal["sa"]]
    p0_diag[28:31] = [0.0, 0.0, 0.0]
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    noise_cfg["sigma_ba"] = float(np.max(imu_sigma_internal["ba"]))
    noise_cfg["sigma_bg"] = float(np.max(imu_sigma_internal["bg"]))
    noise_cfg["sigma_sg"] = float(np.max(imu_sigma_internal["sg"]))
    noise_cfg["sigma_sa"] = float(np.max(imu_sigma_internal["sa"]))
    noise_cfg["sigma_ba_vec"] = [float(x) for x in imu_sigma_internal["ba"]]
    noise_cfg["sigma_bg_vec"] = [float(x) for x in imu_sigma_internal["bg"]]
    noise_cfg["sigma_sg_vec"] = [float(x) for x in imu_sigma_internal["sg"]]
    noise_cfg["sigma_sa_vec"] = [float(x) for x in imu_sigma_internal["sa"]]
    noise_cfg["sigma_gnss_lever_arm"] = 0.0
    noise_cfg["sigma_gnss_lever_arm_vec"] = [0.0, 0.0, 0.0]
    noise_cfg["markov_corr_time"] = float(readme_internal["corr_time_s"])

    metadata = {
        "case_id": CASE_ID,
        "description": "INS/GNSS zero-init ba/bg/sg/sa with POS-320 GM and fixed GNSS lever",
        "imu_init_mode": "zero",
        "imu_p0_scale": float(imu_p0_scale),
        "y_prior_scale": float(y_prior_scale),
        "diag_output_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "gnss_lever_mode": "fixed_truth_constant",
        "gnss_lever_truth_internal_m": [float(x) for x in gnss_lever_ref],
        "imu_model": readme_params["imu_model"],
        "readme_path": readme_params["source"],
        "process_noise_reference": system_markov_sigma_note(readme_params),
        "ba_init_internal": [0.0, 0.0, 0.0],
        "bg_init_internal": [0.0, 0.0, 0.0],
        "sg_init_internal": [0.0, 0.0, 0.0],
        "sa_init_internal": [0.0, 0.0, 0.0],
        "ba_std_internal": [float(x) for x in imu_std_internal["ba"]],
        "bg_std_internal": [float(x) for x in imu_std_internal["bg"]],
        "sg_std_internal": [float(x) for x in imu_std_internal["sg"]],
        "sa_std_internal": [float(x) for x in imu_std_internal["sa"]],
        "ba_sigma_internal": [float(x) for x in imu_sigma_internal["ba"]],
        "bg_sigma_internal": [float(x) for x in imu_sigma_internal["bg"]],
        "sg_sigma_internal": [float(x) for x in imu_sigma_internal["sg"]],
        "sa_sigma_internal": [float(x) for x in imu_sigma_internal["sa"]],
        "ba_reference_internal": [float(x) for x in imu_reference_internal["ba"]],
        "bg_reference_internal": [float(x) for x in imu_reference_internal["bg"]],
        "sg_reference_internal": [float(x) for x in imu_reference_internal["sg"]],
        "sa_reference_internal": [float(x) for x in imu_reference_internal["sa"]],
        "ba_std_human": [float(x) for x in imu_std_internal["ba"] / 1.0e-5],
        "bg_std_human": [float(x) for x in np.rad2deg(imu_std_internal["bg"]) * 3600.0],
        "sg_std_human": [float(x) for x in imu_std_internal["sg"] * 1.0e6],
        "sa_std_human": [float(x) for x in imu_std_internal["sa"] * 1.0e6],
    }
    return cfg, metadata


def run_case(cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
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
    if root_diag.exists():
        shutil.copy2(root_diag, diag_path)
    row: dict[str, Any] = {
        "case_id": CASE_ID,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT) if diag_path.exists() else "",
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "config_mtime": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(evaluate_navigation_metrics(cfg_path, sol_path))
    return row


def classify_state_behavior(outside_ratio: float, final_ratio: float) -> str:
    if outside_ratio <= 0.05 and final_ratio <= 1.0:
        return "normal"
    if outside_ratio <= 0.20 and final_ratio <= 1.5:
        return "borderline"
    return "abnormal"


def activity_range_metrics(
    range_value: float,
    mean: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    floor_scale: float,
    ceiling_scale: float,
) -> dict[str, float | str]:
    reference_range = float(np.max(upper) - np.min(lower))
    reference_range = max(reference_range, 1.0e-9)
    range_ratio = float(range_value / reference_range)
    lower_bound = float(floor_scale * reference_range)
    upper_bound = float(ceiling_scale * reference_range)
    if range_ratio < float(floor_scale):
        label = "too_low"
        penalty = float(abs(math.log(max(range_ratio, 1.0e-12) / max(float(floor_scale), 1.0e-12))))
    elif range_ratio > float(ceiling_scale):
        label = "too_high"
        penalty = float(abs(math.log(range_ratio / max(float(ceiling_scale), 1.0e-12))))
    else:
        label = "range_sane"
        penalty = 0.0
    return {
        "activity_reference_range": reference_range,
        "activity_lower_bound": lower_bound,
        "activity_upper_bound": upper_bound,
        "activity_range_ratio": range_ratio,
        "activity_penalty": penalty,
        "activity_label": label,
        "activity_reference_start": float(np.min(lower)),
        "activity_reference_end": float(np.max(upper)),
        "activity_reference_mean_excursion": float(np.max(mean) - np.min(mean)),
    }


def evaluate_imu_states(
    state_df: pd.DataFrame,
    ref_df: pd.DataFrame,
    readme_params: dict[str, Any],
    activity_floor_scale: float = ACTIVITY_FLOOR_SCALE_DEFAULT,
    activity_ceiling_scale: float = ACTIVITY_CEILING_SCALE_DEFAULT,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    corr_time_hr = float(readme_params["corr_time_hr"])
    sigma_map = {
        "ba": float(readme_params["sigma_ba_mgal"]),
        "bg": float(readme_params["sigma_bg_degh"]),
        "sg": float(readme_params["sigma_sg_ppm"]),
        "sa": float(readme_params["sigma_sa_ppm"]),
    }
    timestamps = state_df["timestamp"].to_numpy(dtype=float)

    for group in IMU_GROUP_ORDER:
        for axis_idx, axis in enumerate(AXIS_ORDER):
            state_name = f"{group}_{axis}"
            column = IMU_GROUP_COLUMNS[group][axis_idx]
            values = state_df[column].to_numpy(dtype=float)
            mean = ref_df[f"{group}_{axis}_mean"].to_numpy(dtype=float)
            lower = ref_df[f"{group}_{axis}_lower"].to_numpy(dtype=float)
            upper = ref_df[f"{group}_{axis}_upper"].to_numpy(dtype=float)
            half_width = np.maximum(upper - mean, 1.0e-9)
            diff = np.abs(values - mean)
            outside_ratio = float(np.mean((values < lower) | (values > upper)))
            final_ratio = float(diff[-1] / half_width[-1])
            max_ratio = float(np.max(diff / half_width))
            label = classify_state_behavior(outside_ratio, final_ratio)
            range_value = float(np.max(values) - np.min(values))
            activity_metrics = activity_range_metrics(
                range_value=range_value,
                mean=mean,
                lower=lower,
                upper=upper,
                floor_scale=activity_floor_scale,
                ceiling_scale=activity_ceiling_scale,
            )
            rows.append(
                {
                    "state_name": state_name,
                    "group": group,
                    "axis": axis,
                    "unit": IMU_GROUP_UNITS[group],
                    "behavior_label": label,
                    "start_value": float(values[0]),
                    "end_value": float(values[-1]),
                    "mean_value": float(np.mean(values)),
                    "std_value": float(np.std(values, ddof=0)),
                    "min_value": float(np.min(values)),
                    "max_value": float(np.max(values)),
                    "range_value": range_value,
                    "head_to_tail_delta": float(values[-1] - values[0]),
                    "modeled_truth_start": float(mean[0]),
                    "modeled_truth_end": float(mean[-1]),
                    "bound_final": float(half_width[-1]),
                    "outside_ratio": outside_ratio,
                    "final_over_bound_ratio": final_ratio,
                    "max_over_bound_ratio": max_ratio,
                    "sigma_ss": sigma_map[group],
                    "corr_time_hr": corr_time_hr,
                    "csv_column": column,
                    "timestamp_start": float(timestamps[0]),
                    "timestamp_end": float(timestamps[-1]),
                    "activity_floor_scale": float(activity_floor_scale),
                    "activity_ceiling_scale": float(activity_ceiling_scale),
                    **activity_metrics,
                }
            )
    return pd.DataFrame(rows)


def write_summary(
    output_path: Path,
    case_row: dict[str, Any],
    imu_state_df: pd.DataFrame,
    readme_params: dict[str, Any],
    imu_p0_scale: float,
    y_prior_scale: float,
    manifest: dict[str, Any],
) -> None:
    label_counts = (
        imu_state_df.groupby(["group", "behavior_label"]).size().unstack(fill_value=0).reindex(IMU_GROUP_ORDER, fill_value=0)
    )

    lines: list[str] = [
        "# data2 INS/GNSS bias-scale zero-init POS-320 GM summary",
        "",
        "## 1. 实验设置",
        "- pipeline: `INS/GNSS only`, 显式关闭 `ODO/NHC`。",
        "- fixed lever: `gnss_lever(28:30)` 固定为 `README POS-320` 真值，并通过 `disable_gnss_lever_arm=true` 关闭估计。",
        "- estimated states: `ba/bg/sg/sa` 共 `12` 轴全部从 `0` 初值开始估计。",
        (
            f"- README-based GM process model: `{system_markov_sigma_note(readme_params)}`。"
        ),
        (
            f"- large prior std: `P0 std = {imu_p0_scale:g} x max(|init_ref|, 3 sigma_readme)`，"
            f"`ba/bg/sg/sa` 当前默认 prior std 取值分别为 "
            f"`{', '.join(format_metric(float(x)) for x in case_row['ba_std_human'])} mGal`, "
            f"`{', '.join(format_metric(float(x)) for x in case_row['bg_std_human'])} deg/h`, "
            f"`{', '.join(format_metric(float(x)) for x in case_row['sg_std_human'])} ppm`, "
            f"`{', '.join(format_metric(float(x)) for x in case_row['sa_std_human'])} ppm`。"
        ),
        f"- axis override: `y_prior_scale={y_prior_scale:g}`（仅缩放 `Y` 轴 prior，`X/Z` 保持不变）。",
        "",
        "## 2. Navigation",
        (
            f"- `{case_row['case_id']}`: `nav_rmse_3d_m={format_metric(float(case_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(case_row['nav_final_err_3d_m']))}`。"
        ),
        "",
        "## 3. IMU state behavior summary",
    ]

    for group in IMU_GROUP_ORDER:
        row = label_counts.loc[group]
        lines.append(
            f"- `{group}`: `normal={int(row.get('normal', 0))}`, "
            f"`borderline={int(row.get('borderline', 0))}`, "
            f"`abnormal={int(row.get('abnormal', 0))}`。"
        )

    lines.append("")
    lines.append("## 4. IMU state snapshot")
    for _, row in imu_state_df.iterrows():
        lines.append(
            f"- `{row['state_name']}`: "
            f"`label={row['behavior_label']}`, "
            f"`start={format_metric(float(row['start_value']))} {row['unit']}`, "
            f"`end={format_metric(float(row['end_value']))} {row['unit']}`, "
            f"`range={format_metric(float(row['range_value']))} {row['unit']}`, "
            f"`activity={row['activity_label']}`, "
            f"`activity_ratio={format_metric(float(row['activity_range_ratio']))}`, "
            f"`outside_ratio={format_metric(float(row['outside_ratio']))}`, "
            f"`final_ratio={format_metric(float(row['final_over_bound_ratio']))}`。"
        )

    lines.extend(
        [
            "",
            "## 5. Artifacts",
            f"- summary: `{manifest['summary_path']}`",
            f"- case metrics: `{manifest['case_metrics_csv']}`",
            f"- imu state metrics: `{manifest['imu_state_metrics_csv']}`",
            f"- estimated-only plots: `{manifest['plots_dir']}`",
            f"- modeled reference csv (metrics only, not plotted): `{manifest['modeled_truth_reference_csv']}`",
            f"- config: `{manifest['config_path']}`",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run INS/GNSS ba/bg/sg/sa zero-init experiment with POS-320 GM and fixed GNSS lever."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--readme-path", type=Path, default=README_PATH_DEFAULT)
    parser.add_argument("--imu-model", type=str, default=IMU_MODEL_DEFAULT)
    parser.add_argument("--imu-p0-scale", dest="imu_p0_scale", type=float, default=IMU_P0_SCALE_DEFAULT)
    parser.add_argument("--imu-p0-sigma-scale", dest="imu_p0_scale", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--y-prior-scale", type=float, default=Y_PRIOR_SCALE_DEFAULT)
    parser.add_argument("--exp-id", type=str, default=EXP_ID_DEFAULT)
    args = parser.parse_args()

    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg, readme_path=args.readme_path, extrinsic_imu_model=args.imu_model)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)
    reset_directory(args.output_dir)

    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False), encoding="utf-8")

    case_dir = args.output_dir / "artifacts" / "cases" / CASE_ID
    case_dir.mkdir(parents=True, exist_ok=True)
    plot_dir = args.output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    cfg, metadata = build_case_config(
        base_cfg=base_cfg,
        truth_reference=truth_reference,
        readme_params=readme_params,
        case_dir=case_dir,
        imu_p0_scale=float(args.imu_p0_scale),
        y_prior_scale=float(args.y_prior_scale),
    )
    cfg_path = case_dir / f"config_{CASE_ID}.yaml"
    save_yaml(cfg, cfg_path)

    case_row = run_case(cfg_path, case_dir, args.exe)
    case_row.update(metadata)

    usecols = ["timestamp"]
    for group in IMU_GROUP_ORDER:
        usecols.extend(IMU_GROUP_COLUMNS[group])
    state_df = pd.read_csv((REPO_ROOT / case_row["state_series_path"]).resolve(), usecols=usecols)
    modeled_truth_df = modeled_truth_reference_series(
        state_df["timestamp"].to_numpy(dtype=float),
        truth_reference,
        readme_params,
    )
    imu_state_df = evaluate_imu_states(state_df, modeled_truth_df, readme_params)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    imu_state_metrics_path = args.output_dir / "imu_state_metrics.csv"
    modeled_truth_path = args.output_dir / "modeled_truth_reference.csv"
    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"

    pd.DataFrame([case_row]).to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    imu_state_df.to_csv(imu_state_metrics_path, index=False, encoding="utf-8-sig")
    modeled_truth_df.to_csv(modeled_truth_path, index=False, encoding="utf-8-sig")

    for group in IMU_GROUP_ORDER:
        plot_imu_group_estimated_only(state_df, group, plot_dir / f"{group}_estimated_only.png")

    freshness = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "config_yaml": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "imu_state_metrics_csv": dt.datetime.fromtimestamp(imu_state_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "modeled_truth_reference_csv": dt.datetime.fromtimestamp(modeled_truth_path.stat().st_mtime).isoformat(timespec="seconds"),
        "plots_dir": dt.datetime.fromtimestamp((plot_dir / "ba_estimated_only.png").stat().st_mtime).isoformat(timespec="seconds"),
    }

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.output_dir / "artifacts", REPO_ROOT),
        "plots_dir": rel_from_root(plot_dir, REPO_ROOT),
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "imu_state_metrics_csv": rel_from_root(imu_state_metrics_path, REPO_ROOT),
        "modeled_truth_reference_csv": rel_from_root(modeled_truth_path, REPO_ROOT),
        "summary_path": rel_from_root(summary_path, REPO_ROOT),
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        "readme_imu_params": readme_params,
        "imu_p0_scale": float(args.imu_p0_scale),
        "y_prior_scale": float(args.y_prior_scale),
        "system_imu_q_note": system_markov_sigma_note(readme_params),
        "exact_overrides": json_safe(metadata),
        "case": json_safe(case_row),
        "freshness": freshness,
    }

    write_summary(
        output_path=summary_path,
        case_row=case_row,
        imu_state_df=imu_state_df,
        readme_params=readme_params,
        imu_p0_scale=float(args.imu_p0_scale),
        y_prior_scale=float(args.y_prior_scale),
        manifest=manifest,
    )
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"[done] wrote summary: {summary_path}")
    print(f"[done] wrote case metrics: {case_metrics_path}")
    print(f"[done] wrote imu state metrics: {imu_state_metrics_path}")
    print(f"[done] wrote modeled truth reference: {modeled_truth_path}")


if __name__ == "__main__":
    main()
