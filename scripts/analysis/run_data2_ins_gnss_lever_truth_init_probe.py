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
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    downsample_for_plot,
    format_metric,
    get_group_vector_internal,
    json_safe,
    reset_directory,
    run_command,
)


CASE_ID = "free_gnss_lever_truth_init"
FIRST_CHANGE_THRESHOLD_M = 1.0e-5
EARLY_WINDOW_S = 10.0
FIRST_MINUTE_S = 60.0
FIRST_UPDATE_IMMEDIATE_RATIO = 0.7
FIRST_UPDATE_IMMEDIATE_FLOOR_M = 0.01
GNSS_LEVER_STATES = ["gnss_lever_x", "gnss_lever_y", "gnss_lever_z"]
GNSS_LEVER_COLUMNS = {
    "gnss_lever_x": "gnss_lever_x_m",
    "gnss_lever_y": "gnss_lever_y_m",
    "gnss_lever_z": "gnss_lever_z_m",
}
AXIS_ORDER = ["x", "y", "z"]
RUNTIME_ANCHOR_REFERENCE = (
    "output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/summary.md"
)


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    gnss_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_noise = base_cfg["fusion"]["noise"]
    p0_diag = base_p0_diag_from_config(base_cfg)

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
    fusion["ablation"] = ablation_cfg
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = False
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    gnss_lever_truth = get_group_vector_internal(truth_reference, "gnss_lever")
    init_cfg["gnss_lever_arm0"] = [float(x) for x in gnss_lever_truth]
    p0_diag[28:31] = [0.04, 0.04, 0.04]
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    sigma_gnss_lever = float(base_noise["sigma_gnss_lever_arm"])
    noise_cfg["sigma_gnss_lever_arm"] = sigma_gnss_lever
    noise_cfg["sigma_gnss_lever_arm_vec"] = [sigma_gnss_lever, sigma_gnss_lever, sigma_gnss_lever]

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
        "fusion.init.gnss_lever_arm0": [float(x) for x in gnss_lever_truth],
        "fusion.init.P0_diag[28:31]": [0.04, 0.04, 0.04],
        "fusion.noise.sigma_gnss_lever_arm": sigma_gnss_lever,
        "fusion.noise.sigma_gnss_lever_arm_vec": [sigma_gnss_lever, sigma_gnss_lever, sigma_gnss_lever],
    }
    return cfg, overrides


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    gnss_path: Path,
) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(base_cfg, truth_reference, case_dir, gnss_path)
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


def classify_axis(final_deviation: float, max_abs_deviation: float, early_plateau_deviation: float) -> str:
    if final_deviation <= 0.03 and max_abs_deviation <= 0.10:
        if early_plateau_deviation > 0.03:
            return "early_offset_recovered"
        return "near_truth"
    if early_plateau_deviation > 0.03 and final_deviation <= 0.03:
        return "early_offset_recovered"
    return "persistent_departure"


def detect_first_change(timestamps: np.ndarray, values: np.ndarray) -> tuple[float, float]:
    initial_value = float(values[0])
    delta = np.abs(values - initial_value)
    changed = np.where(delta > FIRST_CHANGE_THRESHOLD_M)[0]
    if changed.size == 0:
        return math.nan, math.nan
    first_idx = int(changed[0])
    return float(timestamps[first_idx]), float(values[first_idx])


def summarize_axis_behavior(
    timestamps: np.ndarray,
    values: np.ndarray,
    truth_value: float,
    first_update_row: pd.Series,
) -> dict[str, Any]:
    first_change_time, first_change_value = detect_first_change(timestamps, values)
    if math.isfinite(first_change_time):
        plateau_mask = (timestamps >= first_change_time) & (timestamps <= first_change_time + EARLY_WINDOW_S)
    else:
        plateau_mask = timestamps <= timestamps[0] + EARLY_WINDOW_S
    plateau_values = values[plateau_mask]
    if plateau_values.size == 0:
        plateau_values = values[:1]

    first_minute_mask = timestamps <= timestamps[0] + FIRST_MINUTE_S
    if not np.any(first_minute_mask):
        first_minute_mask = np.ones_like(timestamps, dtype=bool)

    initial_value = float(values[0])
    early_plateau_median = float(np.median(plateau_values))
    max_abs_deviation = float(np.max(np.abs(values - truth_value)))
    final_deviation = float(abs(values[-1] - truth_value))
    mean_abs_deviation_first_60s = float(np.mean(np.abs(values[first_minute_mask] - truth_value)))

    lever_before = float(first_update_row["lever_before"])
    lever_after = float(first_update_row["lever_after"])
    dx_gnss_lever = float(first_update_row["dx_gnss_lever"])
    after_minus_before = float(lever_after - lever_before)
    dx_consistency_error = float(abs(after_minus_before - dx_gnss_lever))
    first_update_abs_deviation = float(abs(lever_after - truth_value))
    early_plateau_deviation = float(abs(early_plateau_median - truth_value))
    immediate_threshold = max(FIRST_UPDATE_IMMEDIATE_FLOOR_M, FIRST_UPDATE_IMMEDIATE_RATIO * max_abs_deviation)
    immediate_departure = bool(
        (first_update_abs_deviation >= immediate_threshold) or (early_plateau_deviation >= immediate_threshold)
    )

    label = classify_axis(final_deviation, max_abs_deviation, early_plateau_deviation)
    drift_source = "first_gnss_pos_update" if immediate_departure else "post_first_update_accumulation"
    return {
        "initial_value": initial_value,
        "truth_value": truth_value,
        "first_change_time": first_change_time,
        "first_change_value": first_change_value,
        "early_plateau_median_10s": early_plateau_median,
        "max_abs_deviation_from_truth_m": max_abs_deviation,
        "final_deviation_from_truth_m": final_deviation,
        "mean_abs_deviation_first_60s_m": mean_abs_deviation_first_60s,
        "lever_before": lever_before,
        "lever_after": lever_after,
        "dx_gnss_lever": dx_gnss_lever,
        "after_minus_before": after_minus_before,
        "dx_consistency_error_m": dx_consistency_error,
        "first_update_abs_deviation_m": first_update_abs_deviation,
        "early_plateau_deviation_m": early_plateau_deviation,
        "drift_source": drift_source,
        "label": label,
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
    if set(first_update_df["gnss_axis"].astype(str).tolist()) != set(AXIS_ORDER):
        raise RuntimeError(f"unexpected first_update axes in {first_update_path}")

    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    rows: list[dict[str, Any]] = []
    for axis in AXIS_ORDER:
        state_name = f"gnss_lever_{axis}"
        column = GNSS_LEVER_COLUMNS[state_name]
        truth_value = float(truth_reference["states"][state_name]["reference_value"])
        values = state_df[column].to_numpy(dtype=float)
        first_update_row = first_update_df.loc[first_update_df["gnss_axis"] == axis].iloc[0]
        row = {
            "state_name": state_name,
            "axis": axis,
            "truth_source": truth_reference["states"][state_name]["source"],
        }
        row.update(summarize_axis_behavior(timestamps, values, truth_value, first_update_row))
        rows.append(row)
    return pd.DataFrame(rows).sort_values(by="axis").reset_index(drop=True)


def plot_gnss_lever_truth_series(state_df: pd.DataFrame, truth_reference: dict[str, Any], output_path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    for idx, axis in enumerate(AXIS_ORDER):
        state_name = f"gnss_lever_{axis}"
        column = GNSS_LEVER_COLUMNS[state_name]
        truth_value = float(truth_reference["states"][state_name]["reference_value"])
        t_plot, y_plot = downsample_for_plot(timestamps, state_df[column].to_numpy(dtype=float))
        axes[idx].plot(t_plot, y_plot, linewidth=1.2, label=state_name)
        axes[idx].axhline(truth_value, linestyle="--", color="black", linewidth=1.0, label="truth")
        axes[idx].set_ylabel("m")
        axes[idx].grid(alpha=0.25)
        axes[idx].legend(loc="best")
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle("GNSS lever state series vs truth")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_gnss_lever_first_window(state_df: pd.DataFrame, truth_reference: dict[str, Any], output_path: Path) -> None:
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    window_mask = timestamps <= timestamps[0] + EARLY_WINDOW_S
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for idx, axis in enumerate(AXIS_ORDER):
        state_name = f"gnss_lever_{axis}"
        column = GNSS_LEVER_COLUMNS[state_name]
        truth_value = float(truth_reference["states"][state_name]["reference_value"])
        t_plot, y_plot = downsample_for_plot(
            timestamps[window_mask],
            state_df.loc[window_mask, column].to_numpy(dtype=float),
        )
        axes[idx].plot(t_plot, y_plot, linewidth=1.3, label=f"{state_name} first 10s")
        axes[idx].axhline(truth_value, linestyle="--", color="black", linewidth=1.0, label="truth")
        axes[idx].set_ylabel("m")
        axes[idx].grid(alpha=0.25)
        axes[idx].legend(loc="best")
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle("GNSS lever first 10 seconds")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_gnss_lever_deviation(state_df: pd.DataFrame, truth_reference: dict[str, Any], output_path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    for idx, axis in enumerate(AXIS_ORDER):
        state_name = f"gnss_lever_{axis}"
        column = GNSS_LEVER_COLUMNS[state_name]
        truth_value = float(truth_reference["states"][state_name]["reference_value"])
        deviation = state_df[column].to_numpy(dtype=float) - truth_value
        t_plot, y_plot = downsample_for_plot(timestamps, deviation)
        axes[idx].plot(t_plot, y_plot, linewidth=1.2, label=f"{state_name} - truth")
        axes[idx].axhline(0.0, linestyle="--", color="black", linewidth=1.0)
        axes[idx].set_ylabel("m")
        axes[idx].grid(alpha=0.25)
        axes[idx].legend(loc="best")
    axes[-1].set_xlabel("timestamp [s]")
    fig.suptitle("GNSS lever deviation from truth")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_path: Path,
    probe_row: dict[str, Any],
    lever_df: pd.DataFrame,
    truth_reference: dict[str, Any],
    manifest: dict[str, Any],
) -> None:
    lines: list[str] = [
        "# data2 INS/GNSS GNSS lever truth-init free-run probe",
        "",
        "## 1. 与旧 runtime anchor 实验的区别",
        (
            "- 本轮与旧 `runtime_anchor` 口径相比，唯一区别是显式关闭 "
            "`fusion.init.runtime_truth_anchor_pva`；仍保留 `use_truth_pva=true` 的真值首点初始化。"
        ),
        (
            "- 其它核心口径保持为纯 `INS/GNSS`："
            "`enable_odo=false`、`enable_nhc=false`、"
            "`disable_mounting=true`、`disable_odo_lever_arm=true`、`disable_odo_scale=true`、"
            "`enable_gnss_velocity=false`、`gnss_path=dataset/data2/rtk.txt`。"
        ),
        f"- runtime-anchor reference: `{RUNTIME_ANCHOR_REFERENCE}`",
        "",
        "## 2. 三轴是否从真值出发后仍被拉离",
    ]

    for _, row in lever_df.iterrows():
        lines.append(
            f"- `{row['state_name']}`: label=`{row['label']}`, "
            f"`initial={format_metric(float(row['initial_value']))}` m, "
            f"`truth={format_metric(float(row['truth_value']))}` m, "
            f"`early_plateau_10s={format_metric(float(row['early_plateau_median_10s']))}` m, "
            f"`final_dev={format_metric(float(row['final_deviation_from_truth_m']))}` m, "
            f"`max_dev={format_metric(float(row['max_abs_deviation_from_truth_m']))}` m."
        )

    lines.extend(["", "## 3. 偏离主要发生在首个 GNSS_POS 更新还是后续累计"])
    for _, row in lever_df.iterrows():
        source_text = (
            "首个 `GNSS_POS` 更新已把杆臂拉离真值"
            if row["drift_source"] == "first_gnss_pos_update"
            else "主要是后续累计漂移，而非首更平台"
        )
        lines.append(
            f"- `{row['state_name']}`: {source_text}；"
            f"`lever_before={format_metric(float(row['lever_before']))}` m, "
            f"`lever_after={format_metric(float(row['lever_after']))}` m, "
            f"`dx={format_metric(float(row['dx_gnss_lever']))}` m, "
            f"`after_minus_before={format_metric(float(row['after_minus_before']))}` m, "
            f"`dx_consistency_error={format_metric(float(row['dx_consistency_error_m']))}` m."
        )

    lines.extend(["", "## 4. 是否达到持续性偏离真值"])
    persistent_df = lever_df[lever_df["label"] == "persistent_departure"]
    if persistent_df.empty:
        lines.append("- 当前三轴都没有达到 `persistent_departure`。")
    else:
        lines.append("- " + "、".join(f"`{state}`" for state in persistent_df["state_name"].tolist()))

    lines.extend(["", "## 5. 下一步优先方向"])
    if not persistent_df.empty and (lever_df["drift_source"] == "first_gnss_pos_update").any():
        lines.append(
            "- 当前结果支持把下一步排查重点优先转向 `GNSS_POS residual / PVA shared correction`，"
            "因为在取消 runtime PVA 锚定后，杆臂即使从真值出发也会在首更或首轮早期更新中被系统性拉离。"
        )
    elif not persistent_df.empty:
        lines.append(
            "- 当前结果更像“后续累计更新链”导致的持续偏离，下一步应补看长时间窗内的 `K*y` 累积而不只盯首更。"
        )
    else:
        lines.append(
            "- 当前结果没有显示强持续偏离，下一步不宜直接把根因升级为 solver bug；"
            "应继续区分厘米级首更残差与真正的系统性平台偏差。"
        )

    lines.extend(
        [
            "",
            "## Notes",
            (
                "- 本轮使用 `dataset/data2/rtk.txt`，它不含 GNSS 速度列；"
                "因此首更厘米级偏差可能混入已知的 `GNSS_POS` 时间对齐残差，"
                "不能把小残差直接升级为 solver bug。"
            ),
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
        description="Run a data2 INS/GNSS-only probe with GNSS lever truth initialization but no runtime PVA anchor."
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
        default=Path("output/data2_ins_gnss_lever_truth_init_probe"),
        help="Output directory relative to repo root.",
    )
    parser.add_argument(
        "--gnss-path",
        type=Path,
        default=Path("dataset/data2/rtk.txt"),
        help="GNSS file used by the probe.",
    )
    parser.add_argument(
        "--exp-id",
        default="EXP-20260318-data2-ins-gnss-lever-truth-init-free-run-r1",
        help="Experiment identifier recorded in manifest.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.gnss_path = (REPO_ROOT / args.gnss_path).resolve()
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

    case_dir = args.case_root / CASE_ID
    ensure_dir(case_dir)
    cfg_path, overrides = write_case_config(base_cfg, truth_reference, case_dir, args.gnss_path)
    probe_row = run_probe_case(case_dir=case_dir, cfg_path=cfg_path, exe_path=args.exe)

    state_series_path = (REPO_ROOT / probe_row["state_series_path"]).resolve()
    first_update_path = (REPO_ROOT / probe_row["first_update_path"]).resolve()
    state_df = pd.read_csv(
        state_series_path,
        usecols=["timestamp", "gnss_lever_x_m", "gnss_lever_y_m", "gnss_lever_z_m"],
    )
    lever_df = build_lever_metrics(truth_reference, state_series_path, first_update_path)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    pd.DataFrame([probe_row]).to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    lever_metrics_path = args.output_dir / "lever_deviation_metrics.csv"
    lever_df.to_csv(lever_metrics_path, index=False, encoding="utf-8-sig")

    plot_gnss_lever_truth_series(state_df, truth_reference, args.plot_dir / "gnss_lever_xyz_vs_truth.png")
    plot_gnss_lever_first_window(state_df, truth_reference, args.plot_dir / "gnss_lever_xyz_first_10s.png")
    plot_gnss_lever_deviation(state_df, truth_reference, args.plot_dir / "gnss_lever_xyz_deviation.png")

    freshness = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "config_yaml": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "lever_deviation_metrics_csv": dt.datetime.fromtimestamp(lever_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "plots_dir": dt.datetime.fromtimestamp((args.plot_dir / "gnss_lever_xyz_vs_truth.png").stat().st_mtime).isoformat(timespec="seconds"),
    }

    manifest_path = args.output_dir / "manifest.json"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "gnss_path": rel_from_root(args.gnss_path, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "lever_deviation_metrics_csv": rel_from_root(lever_metrics_path, REPO_ROOT),
        "comparison_reference": RUNTIME_ANCHOR_REFERENCE,
        "truth_catalog_source": truth_reference["sources"],
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
        "mode": "ins_gnss_gnss_lever_truth_init_probe",
        "assumptions": [
            "关闭 runtime PVA 真值锚定，但保留 use_truth_pva=true 的真值首点初始化。",
            "mounting/odo_scale/odo_lever 继续禁用，不纳入本轮解释。",
            "只做分析脚本层实验，不改 solver 数学。",
        ],
    }
    manifest["manifest_path"] = rel_from_root(manifest_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    write_summary(summary_path, probe_row, lever_df, truth_reference, manifest)
    freshness["summary_md"] = dt.datetime.fromtimestamp(summary_path.stat().st_mtime).isoformat(timespec="seconds")
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
