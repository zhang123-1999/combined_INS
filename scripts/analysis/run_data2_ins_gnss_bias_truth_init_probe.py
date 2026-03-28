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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import (
    imu_truth_model_internal,
    modeled_truth_reference_series,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    STATE_META,
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    format_metric,
    get_group_vector_internal,
    human_to_internal,
    json_safe,
    reset_directory,
    run_command,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-bias-truth-init-probe-r3-pos320-mainline"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_bias_truth_init_probe_pos320_mainline")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
README_PATH_DEFAULT = Path("dataset/data2/README.md")
IMU_MODEL_DEFAULT = "POS-320"
BIAS_STATE_ORDER = ["ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"]
SEGMENT_COUNT = 10


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    bias_init_mode: str
    description: str


CASE_SPECS = [
    CaseSpec(
        case_id="truth_init_system_q",
        bias_init_mode="truth",
        description="ba/bg truth init + system-consistent Markov sigma",
    ),
    CaseSpec(
        case_id="zero_init_system_q_ref",
        bias_init_mode="zero",
        description="ba/bg zero init + same system-consistent Markov sigma",
    ),
]


def system_markov_sigma_note(readme_params: dict[str, Any]) -> str:
    return (
        f"{readme_params['source']} {readme_params['imu_model']}: "
        f"ba sigma={float(readme_params['sigma_ba_mgal']):g} mGal, "
        f"bg sigma={float(readme_params['sigma_bg_degh']):g} deg/h, "
        f"corr_time={float(readme_params['corr_time_hr']):g} h"
    )


def anchored_dynamic_std(sigma_internal: np.ndarray) -> np.ndarray:
    return np.asarray(0.1 * sigma_internal, dtype=float)


def free_bias_std(reference: np.ndarray, sigma_internal: np.ndarray) -> np.ndarray:
    base = np.maximum(np.abs(reference), 3.0 * sigma_internal)
    return np.asarray(base, dtype=float)


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    p0_diag = list(base_p0_diag_from_config(base_cfg))

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    stdout_path = case_dir / f"{spec.case_id}.stdout.txt"
    diag_path = case_dir / f"DIAG_{spec.case_id}.txt"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["gnss_schedule"] = {"enabled": False}

    ablation_cfg = default_ablation_flags()
    ablation_cfg["disable_mounting"] = True
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_gnss_lever_z"] = False
    fusion["ablation"] = ablation_cfg
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

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

    ba_ref = get_group_vector_internal(truth_reference, "ba")
    bg_ref = get_group_vector_internal(truth_reference, "bg")
    sg_ref = get_group_vector_internal(truth_reference, "sg")
    sa_ref = get_group_vector_internal(truth_reference, "sa")
    gnss_lever_ref = get_group_vector_internal(truth_reference, "gnss_lever")
    readme_internal = imu_truth_model_internal(readme_params)
    ba_sigma = np.full(3, float(readme_internal["sigma_ba"]), dtype=float)
    bg_sigma = np.full(3, float(readme_internal["sigma_bg"]), dtype=float)
    sg_sigma = np.full(3, float(readme_internal["sigma_sg"]), dtype=float)
    sa_sigma = np.full(3, float(readme_internal["sigma_sa"]), dtype=float)

    if spec.bias_init_mode == "truth":
        ba_init = ba_ref.copy()
        bg_init = bg_ref.copy()
        ba_std = anchored_dynamic_std(ba_sigma)
        bg_std = anchored_dynamic_std(bg_sigma)
    elif spec.bias_init_mode == "zero":
        ba_init = np.zeros(3, dtype=float)
        bg_init = np.zeros(3, dtype=float)
        ba_std = free_bias_std(ba_ref, ba_sigma)
        bg_std = free_bias_std(bg_ref, bg_sigma)
    else:
        raise ValueError(f"unsupported bias_init_mode: {spec.bias_init_mode}")

    ba_noise = ba_sigma.copy()
    bg_noise = bg_sigma.copy()
    sg_std = anchored_dynamic_std(sg_sigma)
    sa_std = anchored_dynamic_std(sa_sigma)
    sg_noise = sg_sigma.copy()
    sa_noise = sa_sigma.copy()
    gnss_lever_std = np.full(3, 0.01, dtype=float)
    gnss_lever_noise = np.full(3, 1.0e-6, dtype=float)

    init_cfg["ba0"] = [float(x) for x in ba_init]
    init_cfg["bg0"] = [float(x) for x in bg_init]
    init_cfg["sg0"] = [float(x) for x in sg_ref]
    init_cfg["sa0"] = [float(x) for x in sa_ref]
    init_cfg["gnss_lever_arm0"] = [float(x) for x in gnss_lever_ref]

    p0_diag[9:12] = [float(x * x) for x in ba_std]
    p0_diag[12:15] = [float(x * x) for x in bg_std]
    p0_diag[15:18] = [float(x * x) for x in sg_std]
    p0_diag[18:21] = [float(x * x) for x in sa_std]
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

    metadata = {
        "case_id": spec.case_id,
        "description": spec.description,
        "bias_init_mode": spec.bias_init_mode,
        "diag_output_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "ba_init_internal": [float(x) for x in ba_init],
        "bg_init_internal": [float(x) for x in bg_init],
        "ba_std_internal": [float(x) for x in ba_std],
        "bg_std_internal": [float(x) for x in bg_std],
        "ba_sigma_internal": [float(x) for x in ba_noise],
        "bg_sigma_internal": [float(x) for x in bg_noise],
        "sg_truth_internal": [float(x) for x in sg_ref],
        "sa_truth_internal": [float(x) for x in sa_ref],
        "gnss_lever_truth_internal_m": [float(x) for x in gnss_lever_ref],
        "imu_model": readme_params["imu_model"],
        "readme_path": readme_params["source"],
        "process_noise_reference": system_markov_sigma_note(readme_params),
    }
    return cfg, metadata


def run_case(spec: CaseSpec, cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    stdout_path = case_dir / f"{spec.case_id}.stdout.txt"
    diag_path = case_dir / f"DIAG_{spec.case_id}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output for {spec.case_id}: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output for {spec.case_id}: {state_series_path}")
    if root_diag.exists():
        shutil.copy2(root_diag, diag_path)
    row: dict[str, Any] = {
        "case_id": spec.case_id,
        "description": spec.description,
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


def split_segment_means(values: np.ndarray, segment_count: int = SEGMENT_COUNT) -> np.ndarray:
    if values.size == 0:
        return np.array([], dtype=float)
    indices = np.array_split(np.arange(values.size), segment_count)
    means = [float(np.mean(values[idx])) for idx in indices if idx.size > 0]
    return np.asarray(means, dtype=float)


def classify_trend(
    segment_means: np.ndarray,
    range_value: float,
    sigma_ss: float,
    head_mean: float,
    tail_mean: float,
) -> tuple[str, int, float]:
    eps = max(1.0e-9, 1.0e-3 * max(1.0, abs(head_mean), abs(tail_mean), abs(sigma_ss)))
    drift_ratio = abs(tail_mean - head_mean) / max(range_value, eps)
    threshold = max(0.08 * max(range_value, eps), 0.20 * max(sigma_ss, eps))
    if segment_means.size <= 1:
        return "mixed", 0, float(drift_ratio)
    delta = np.diff(segment_means)
    signs = []
    for value in delta:
        if abs(float(value)) < threshold:
            continue
        signs.append(1 if value > 0.0 else -1)
    direction_changes = 0
    for idx in range(1, len(signs)):
        if signs[idx] != signs[idx - 1]:
            direction_changes += 1
    if drift_ratio >= 0.65 and direction_changes <= 1:
        trend = "persistent_drift"
    elif drift_ratio <= 0.55 and direction_changes >= 2:
        trend = "fluctuating"
    else:
        trend = "mixed"
    return trend, direction_changes, float(drift_ratio)


def modeled_behavior_label(values: np.ndarray, mean: np.ndarray, bound: np.ndarray) -> tuple[str, dict[str, float]]:
    diff = np.abs(values - mean)
    eps = np.maximum(1.0e-9, 1.0e-6 * np.maximum(1.0, np.abs(mean)))
    denom = np.maximum(bound, eps)
    outside_ratio = float(np.mean(diff > bound))
    final_ratio = float(diff[-1] / denom[-1])
    max_ratio = float(np.max(diff / denom))
    if outside_ratio <= 0.05 and final_ratio <= 1.0:
        label = "normal"
    elif outside_ratio <= 0.20 and final_ratio <= 1.5:
        label = "borderline"
    else:
        label = "abnormal"
    return label, {
        "modeled_truth_mean_start": float(mean[0]),
        "modeled_truth_mean_final": float(mean[-1]),
        "modeled_bound_final": float(bound[-1]),
        "final_value": float(values[-1]),
        "outside_ratio": outside_ratio,
        "final_over_bound_ratio": final_ratio,
        "max_over_bound_ratio": max_ratio,
    }


def evaluate_bias_axes(
    case_row: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
) -> pd.DataFrame:
    state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    usecols = ["timestamp"] + [STATE_META[state_name]["csv"] for state_name in BIAS_STATE_ORDER]
    state_df = pd.read_csv(state_path, usecols=usecols)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    modeled_truth_df = modeled_truth_reference_series(timestamps, truth_reference, readme_params)
    elapsed_h = (timestamps - float(timestamps[0])) / 3600.0
    head_count = max(1, len(state_df) // 10)

    rows: list[dict[str, Any]] = []
    for state_name in BIAS_STATE_ORDER:
        csv_col = STATE_META[state_name]["csv"]
        values = state_df[csv_col].to_numpy(dtype=float)
        ref = truth_reference["states"][state_name]
        group = STATE_META[state_name]["group"]
        axis = ["x", "y", "z"][int(STATE_META[state_name]["axis"])]
        mean = modeled_truth_df[f"{group}_{axis}_mean"].to_numpy(dtype=float)
        lower = modeled_truth_df[f"{group}_{axis}_lower"].to_numpy(dtype=float)
        upper = modeled_truth_df[f"{group}_{axis}_upper"].to_numpy(dtype=float)
        bound = upper - mean
        behavior_label, behavior_metrics = modeled_behavior_label(values, mean, bound)
        sigma_ss = float(bound[-1] / 3.0) if bound.size > 0 else 0.0
        head_mean = float(np.mean(values[:head_count]))
        tail_mean = float(np.mean(values[-head_count:]))
        mean_value = float(np.mean(values))
        std_value = float(np.std(values, ddof=0))
        range_value = float(np.max(values) - np.min(values))
        if values.size >= 2 and np.unique(elapsed_h).size >= 2:
            trend_slope_per_h = float(np.polyfit(elapsed_h, values, 1)[0])
        else:
            trend_slope_per_h = math.nan
        segment_means = split_segment_means(values)
        trend_label, direction_changes, drift_ratio = classify_trend(
            segment_means=segment_means,
            range_value=range_value,
            sigma_ss=sigma_ss,
            head_mean=head_mean,
            tail_mean=tail_mean,
        )
        rows.append(
            {
                "case_id": case_row["case_id"],
                "state_name": state_name,
                "group": STATE_META[state_name]["group"],
                "axis": STATE_META[state_name]["axis"],
                "unit": STATE_META[state_name]["unit"],
                "behavior_label": behavior_label,
                "trend_label": trend_label,
                "truth_source": f"{ref['source']} + {readme_params['source']} ({readme_params['imu_model']})",
                "initial_truth_value": float(mean[0]),
                "tail_truth_mean": float(np.mean(mean[-head_count:])),
                "sigma_ss": sigma_ss,
                "start_value": float(values[0]),
                "head_mean": head_mean,
                "tail_mean": tail_mean,
                "end_value": float(values[-1]),
                "mean_value": mean_value,
                "std_value": std_value,
                "range_value": range_value,
                "trend_slope_per_h": trend_slope_per_h,
                "head_to_tail_delta": float(tail_mean - head_mean),
                "mean_truth_deviation": float(np.mean(values - mean)),
                "tail_truth_deviation": float(tail_mean - np.mean(mean[-head_count:])),
                "direction_changes": int(direction_changes),
                "drift_ratio": drift_ratio,
                **behavior_metrics,
            }
        )
    return pd.DataFrame(rows)


def build_case_summary(case_row: dict[str, Any], axis_df: pd.DataFrame) -> dict[str, Any]:
    abnormal_count = int((axis_df["behavior_label"] == "abnormal").sum())
    trend_counts = axis_df["trend_label"].value_counts()
    row = dict(case_row)
    row.update(
        {
            "abnormal_axis_count": abnormal_count,
            "mean_outside_ratio": float(axis_df["outside_ratio"].mean()),
            "max_outside_ratio": float(axis_df["outside_ratio"].max()),
            "mean_abs_tail_truth_dev": float(axis_df["tail_truth_deviation"].abs().mean()),
            "persistent_drift_axes": int(trend_counts.get("persistent_drift", 0)),
            "fluctuating_axes": int(trend_counts.get("fluctuating", 0)),
            "mixed_axes": int(trend_counts.get("mixed", 0)),
        }
    )
    return row


def downsample_for_plot(x: np.ndarray, y: np.ndarray, max_points: int = 4000) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= max_points:
        return x, y
    idx = np.linspace(0, x.size - 1, max_points, dtype=int)
    return x[idx], y[idx]


def plot_bias_comparison(
    truth_case_row: dict[str, Any],
    zero_case_row: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    output_path: Path,
) -> None:
    truth_df = pd.read_csv((REPO_ROOT / truth_case_row["state_series_path"]).resolve())
    zero_df = pd.read_csv((REPO_ROOT / zero_case_row["state_series_path"]).resolve())
    truth_ref_df = modeled_truth_reference_series(
        truth_df["timestamp"].to_numpy(dtype=float),
        truth_reference,
        readme_params,
    )
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for idx, state_name in enumerate(BIAS_STATE_ORDER):
        ax = axes[idx]
        column = STATE_META[state_name]["csv"]
        truth_t = truth_df["timestamp"].to_numpy(dtype=float)
        truth_v = truth_df[column].to_numpy(dtype=float)
        zero_t = zero_df["timestamp"].to_numpy(dtype=float)
        zero_v = zero_df[column].to_numpy(dtype=float)
        t0 = float(truth_t[0])
        truth_x, truth_y = downsample_for_plot((truth_t - t0) / 60.0, truth_v)
        zero_x, zero_y = downsample_for_plot((zero_t - t0) / 60.0, zero_v)
        group = STATE_META[state_name]["group"]
        axis = ["x", "y", "z"][int(STATE_META[state_name]["axis"])]
        ref_t = truth_ref_df["timestamp"].to_numpy(dtype=float)
        ref_mean = truth_ref_df[f"{group}_{axis}_mean"].to_numpy(dtype=float)
        ref_lower = truth_ref_df[f"{group}_{axis}_lower"].to_numpy(dtype=float)
        ref_upper = truth_ref_df[f"{group}_{axis}_upper"].to_numpy(dtype=float)
        ref_x, ref_mean_ds = downsample_for_plot((ref_t - t0) / 60.0, ref_mean)
        _, ref_lower_ds = downsample_for_plot((ref_t - t0) / 60.0, ref_lower)
        _, ref_upper_ds = downsample_for_plot((ref_t - t0) / 60.0, ref_upper)

        ax.plot(truth_x, truth_y, linewidth=1.1, label="truth-init + system Q")
        ax.plot(zero_x, zero_y, linewidth=1.1, label="zero-init + system Q ref", alpha=0.85)
        ax.plot(ref_x, ref_mean_ds, linestyle="--", color="black", linewidth=0.9, label="modeled truth mean")
        ax.fill_between(ref_x, ref_lower_ds, ref_upper_ds, color="#d9d9d9", alpha=0.35, label="modeled truth ±3σ")
        ax.set_title(state_name)
        ax.set_ylabel(STATE_META[state_name]["unit"])
        ax.grid(alpha=0.25)

    axes[-2].set_xlabel("elapsed time [min]")
    axes[-1].set_xlabel("elapsed time [min]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_summary(
    output_path: Path,
    truth_case_row: pd.Series,
    zero_case_row: pd.Series,
    truth_axis_df: pd.DataFrame,
    readme_params: dict[str, Any],
    manifest: dict[str, Any],
) -> None:
    lines: list[str] = [
        "# data2 pure INS/GNSS bias truth-init probe summary",
        "",
        "## 1. 实验设置",
        "- pipeline: `pure INS/GNSS`, 显式关闭 `ODO/NHC`。",
        "- fixed-truth states: `gnss_lever(28:30)` 使用真值初值 + 小 `P0/Q` 固定；`sg(15:17), sa(18:20)` 继续按真值建模。",
        "- probed states: `ba(9:11), bg(12:14)` 不做 runtime truth anchor，只改变初值与过程噪声设定。",
        f"- README-based bias Q: `{system_markov_sigma_note(readme_params)}`；代码按 GM 稳态不稳定度解释 `sigma_ba/sigma_bg`，再乘 `sqrt(2/T)` 进入驱动噪声。",
        "- 图中的 truth 已改为 README 参数驱动的时变 `modeled truth mean ±3σ`，不再用常数水平线。",
        "",
        "## 2. case 对比",
        (
            f"- truth-init case `{truth_case_row['case_id']}`: "
            f"`nav_rmse_3d_m={format_metric(float(truth_case_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(truth_case_row['nav_final_err_3d_m']))}`, "
            f"`abnormal_axes={int(truth_case_row['abnormal_axis_count'])}`, "
            f"`persistent_drift_axes={int(truth_case_row['persistent_drift_axes'])}`, "
            f"`fluctuating_axes={int(truth_case_row['fluctuating_axes'])}`。"
        ),
        (
            f"- zero-init reference `{zero_case_row['case_id']}`: "
            f"`nav_rmse_3d_m={format_metric(float(zero_case_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(zero_case_row['nav_final_err_3d_m']))}`, "
            f"`abnormal_axes={int(zero_case_row['abnormal_axis_count'])}`, "
            f"`persistent_drift_axes={int(zero_case_row['persistent_drift_axes'])}`, "
            f"`fluctuating_axes={int(zero_case_row['fluctuating_axes'])}`。"
        ),
        "",
        "## 3. truth-init case 各轴行为",
    ]

    ordered_axis_df = truth_axis_df.sort_values(by=["trend_label", "state_name"]).reset_index(drop=True)
    for _, row in ordered_axis_df.iterrows():
        lines.append(
            f"- `{row['state_name']}`: trend=`{row['trend_label']}`, behavior=`{row['behavior_label']}`, "
            f"`slope={format_metric(float(row['trend_slope_per_h']))} {row['unit']}/h`, "
            f"`head_to_tail={format_metric(float(row['head_to_tail_delta']))} {row['unit']}`, "
            f"`range={format_metric(float(row['range_value']))} {row['unit']}`, "
            f"`outside_ratio={format_metric(float(row['outside_ratio']))}`, "
            f"`tail_modeled_truth_dev={format_metric(float(row['tail_truth_deviation']))} {row['unit']}`。"
        )

    lines.extend(
        [
            "",
            "## 4. 结论草案",
            (
                "- `truth-init + README Q` 可以把“初值错误”从问题里剥离出来，但并不会自动把全部 bias 变成围绕时变 GM truth 的正常波动；"
                "哪些轴仍呈持续漂移，需要直接以 `trend_label` 与 `tail_modeled_truth_dev` 为准。"
            ),
            (
                "- 若 truth-init case 仍有明显 `persistent_drift`，更可能说明更新链 / shared-cov / gain collapse 在主导，"
                "而不是单纯因为一开始从 0 出发。"
            ),
            "",
            "## 5. Artifacts",
            f"- case metrics: `{manifest['case_metrics_csv']}`",
            f"- bias axis metrics: `{manifest['bias_axis_metrics_csv']}`",
            f"- modeled truth reference: `{manifest['modeled_truth_reference_csv']}`",
            f"- comparison plot: `{manifest['comparison_plot']}`",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe truth-init bias evolution with system-consistent Markov noise.")
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--readme-path", type=Path, default=README_PATH_DEFAULT)
    parser.add_argument("--imu-model", type=str, default=IMU_MODEL_DEFAULT)
    parser.add_argument("--exp-id", type=str, default=EXP_ID_DEFAULT)
    args = parser.parse_args()

    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)
    reset_directory(args.output_dir)
    cases_root = args.output_dir / "artifacts" / "cases"
    cases_root.mkdir(parents=True, exist_ok=True)

    case_rows: list[dict[str, Any]] = []
    axis_frames: list[pd.DataFrame] = []

    for spec in CASE_SPECS:
        case_dir = cases_root / spec.case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        cfg, metadata = build_case_config(base_cfg, truth_reference, readme_params, case_dir, spec)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)
        case_row = run_case(spec, cfg_path, case_dir, args.exe)
        case_row.update(metadata)
        axis_df = evaluate_bias_axes(case_row, truth_reference, readme_params)
        axis_frames.append(axis_df)
        case_rows.append(build_case_summary(case_row, axis_df))

    case_df = pd.DataFrame(case_rows)
    axis_df = pd.concat(axis_frames, ignore_index=True)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    axis_metrics_path = args.output_dir / "bias_axis_metrics.csv"
    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"
    modeled_truth_path = args.output_dir / "modeled_truth_reference.csv"
    comparison_plot_path = args.output_dir / "plots" / "bias_truth_init_vs_zero_init.png"
    case_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    axis_df.to_csv(axis_metrics_path, index=False, encoding="utf-8-sig")

    truth_case_row = case_df.loc[case_df["case_id"] == "truth_init_system_q"].iloc[0]
    zero_case_row = case_df.loc[case_df["case_id"] == "zero_init_system_q_ref"].iloc[0]
    truth_axis_df = axis_df.loc[axis_df["case_id"] == "truth_init_system_q"].copy()
    truth_case_state_df = pd.read_csv((REPO_ROOT / truth_case_row["state_series_path"]).resolve(), usecols=["timestamp"])
    modeled_truth_df = modeled_truth_reference_series(
        truth_case_state_df["timestamp"].to_numpy(dtype=float),
        truth_reference,
        readme_params,
    )
    modeled_truth_df.to_csv(modeled_truth_path, index=False, encoding="utf-8-sig")

    plot_bias_comparison(
        truth_case_row=truth_case_row.to_dict(),
        zero_case_row=zero_case_row.to_dict(),
        truth_reference=truth_reference,
        readme_params=readme_params,
        output_path=comparison_plot_path,
    )

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config.resolve(), REPO_ROOT),
        "exe_path": rel_from_root(args.exe.resolve(), REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "bias_axis_metrics_csv": rel_from_root(axis_metrics_path, REPO_ROOT),
        "modeled_truth_reference_csv": rel_from_root(modeled_truth_path, REPO_ROOT),
        "comparison_plot": rel_from_root(comparison_plot_path, REPO_ROOT),
        "summary_path": rel_from_root(summary_path, REPO_ROOT),
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        "readme_imu_params": readme_params,
        "system_bias_q_note": system_markov_sigma_note(readme_params),
        "cases": json_safe(case_rows),
    }
    write_summary(summary_path, truth_case_row, zero_case_row, truth_axis_df, readme_params, manifest)
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"[done] wrote summary: {summary_path}")
    print(f"[done] wrote case metrics: {case_metrics_path}")
    print(f"[done] wrote bias axis metrics: {axis_metrics_path}")
    print(f"[done] wrote comparison plot: {comparison_plot_path}")


if __name__ == "__main__":
    main()
