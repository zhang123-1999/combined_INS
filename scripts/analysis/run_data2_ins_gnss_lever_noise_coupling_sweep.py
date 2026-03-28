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

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_gnss_lever_bias_attribution import apply_model_variant
from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import (
    IMU_GROUP_COLUMNS,
    IMU_MODEL_DEFAULT,
    modeled_truth_reference_series,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_ins_gnss_lever_truth_init_probe import (
    AXIS_ORDER,
    build_lever_metrics,
    plot_gnss_lever_deviation,
    plot_gnss_lever_first_window,
    plot_gnss_lever_truth_series,
)
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    downsample_for_plot,
    format_metric,
    get_group_vector_internal,
    human_to_internal,
    json_safe,
    reset_directory,
    run_command,
)


EXP_ID_DEFAULT = "EXP-20260318-data2-ins-gnss-lever-noise-coupling-sweep-r1"
BASELINE_EXP_ID = "EXP-20260320-data2-ins-gnss-lever-truth-imu-params-r2-t3-truth"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_lever_noise_coupling_sweep")
R_SCALES_DEFAULT = [0.25, 0.5, 1.0, 2.0, 4.0]
Q_SCALES_DEFAULT = [0.1, 0.25, 1.0, 4.0, 10.0]
LOCAL_R_SCALES = {0.5, 1.0, 2.0}
LOCAL_Q_SCALES = {0.25, 1.0, 4.0}
MODELS_DEFAULT = ["eskf", "true_iekf"]
GNSS_COLUMNS = ["timestamp", "lat", "lon", "h", "sigma_n", "sigma_e", "sigma_d"]
SIGMA_COLUMNS = ["sigma_n", "sigma_e", "sigma_d"]
CASE_ID_FMT = "{model}_rstd{r_slug}_qlever{q_slug}"
BASELINE_REFERENCE_DIR = REPO_ROOT / "output/data2_ins_gnss_lever_truth_imu_params_probe"
FIRST_UPDATE_Q_FIXED = 1.0
FIRST_UPDATE_R_FIXED = 1.0
EARLY_Z_UNCHANGED_IMPROVEMENT_THRESHOLD = 0.10
R_DOMINANT_MIN_REL_RANGE = 0.20
Q_NEGLIGIBLE_MAX_REL_RANGE = 0.02
MODEL_SENSITIVITY_FACTOR = 1.5


@dataclass(frozen=True)
class CaseSpec:
    model_variant: str
    r_scale: float
    q_scale: float

    @property
    def case_id(self) -> str:
        return CASE_ID_FMT.format(
            model=self.model_variant,
            r_slug=scale_slug(self.r_scale),
            q_slug=scale_slug(self.q_scale),
        )


def scale_slug(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def parse_scale_list(raw: str) -> list[float]:
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if not items:
        raise ValueError("scale list must not be empty")
    return [float(item) for item in items]


def read_rtk_file(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, sep=r"\s+", header=None, names=GNSS_COLUMNS, comment="#", engine="python")
    if df.shape[1] != 7:
        raise RuntimeError(f"unexpected GNSS column count in {path}: {df.shape[1]}")
    return df


def write_scaled_gnss_variant(src_path: Path, dst_path: Path, scale: float) -> dict[str, Any]:
    source_df = read_rtk_file(src_path)
    df = source_df.copy()
    for col in SIGMA_COLUMNS:
        df[col] = df[col].astype(float) * scale
    ensure_dir(dst_path.parent)
    np.savetxt(dst_path, df.to_numpy(dtype=float), fmt=["%.2f", "%.10f", "%.10f", "%.3f", "%.6f", "%.6f", "%.6f"])
    return {
        "gnss_variant_path": rel_from_root(dst_path, REPO_ROOT),
        "r_scale": float(scale),
        "rows": int(df.shape[0]),
        "sigma_median_source": {col: float(source_df[col].median()) for col in SIGMA_COLUMNS},
        "sigma_median_scaled": {col: float(df[col].median()) for col in SIGMA_COLUMNS},
    }


def imu_truth_model_internal(readme_params: dict[str, Any]) -> dict[str, float]:
    return {
        "sigma_ba": human_to_internal("ba", float(readme_params["sigma_ba_mgal"])),
        "sigma_bg": human_to_internal("bg", float(readme_params["sigma_bg_degh"])),
        "sigma_sg": human_to_internal("sg", float(readme_params["sigma_sg_ppm"])),
        "sigma_sa": human_to_internal("sa", float(readme_params["sigma_sa_ppm"])),
        "corr_time_s": float(readme_params["corr_time_s"]),
    }


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    gnss_path: Path,
    case_dir: Path,
    spec: CaseSpec,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_noise = base_cfg["fusion"]["noise"]
    p0_diag = list(base_p0_diag_from_config(base_cfg))

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    first_update_path = case_dir / f"first_update_{spec.case_id}.csv"

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
    p0_diag[28:31] = [0.04, 0.04, 0.04]
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    gnss_lever_noise = float(base_noise["sigma_gnss_lever_arm"]) * float(spec.q_scale)
    noise_cfg["sigma_ba"] = float(np.max(ba_noise))
    noise_cfg["sigma_bg"] = float(np.max(bg_noise))
    noise_cfg["sigma_sg"] = float(np.max(sg_noise))
    noise_cfg["sigma_sa"] = float(np.max(sa_noise))
    noise_cfg["sigma_ba_vec"] = [float(x) for x in ba_noise]
    noise_cfg["sigma_bg_vec"] = [float(x) for x in bg_noise]
    noise_cfg["sigma_sg_vec"] = [float(x) for x in sg_noise]
    noise_cfg["sigma_sa_vec"] = [float(x) for x in sa_noise]
    noise_cfg["sigma_gnss_lever_arm"] = float(gnss_lever_noise)
    noise_cfg["sigma_gnss_lever_arm_vec"] = [float(gnss_lever_noise)] * 3
    noise_cfg["markov_corr_time"] = float(readme_internal["corr_time_s"])

    apply_model_variant(cfg, spec.model_variant)

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
        "fusion.init.P0_diag[28:31]": [0.04, 0.04, 0.04],
        "fusion.noise.sigma_ba_vec": [float(x) for x in ba_noise],
        "fusion.noise.sigma_bg_vec": [float(x) for x in bg_noise],
        "fusion.noise.sigma_sg_vec": [float(x) for x in sg_noise],
        "fusion.noise.sigma_sa_vec": [float(x) for x in sa_noise],
        "fusion.noise.markov_corr_time": float(readme_internal["corr_time_s"]),
        "fusion.noise.sigma_gnss_lever_arm": float(gnss_lever_noise),
        "fusion.noise.sigma_gnss_lever_arm_vec": [float(gnss_lever_noise)] * 3,
        "effective_gnss_pos_std_scale": float(spec.r_scale),
        "model_variant": spec.model_variant,
    }
    return cfg, overrides


def run_case(case_dir: Path, cfg_path: Path, exe_path: Path, case_id: str) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    first_update_path = case_dir / f"first_update_{case_id}.csv"
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
    if not first_update_path.exists():
        raise RuntimeError(f"missing first update output for {case_id}: {first_update_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {case_id}")
    shutil.copy2(root_diag, diag_path)

    row: dict[str, Any] = {
        "case_id": case_id,
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


def build_first_update_metrics(spec: CaseSpec, lever_df: pd.DataFrame, first_update_path: Path) -> pd.DataFrame:
    raw_df = pd.read_csv(first_update_path)
    rows: list[dict[str, Any]] = []
    for axis in AXIS_ORDER:
        lever_row = lever_df.loc[lever_df["axis"] == axis].iloc[0]
        first_row = raw_df.loc[raw_df["gnss_axis"] == axis].iloc[0]
        rows.append(
            {
                "case_id": spec.case_id,
                "model_variant": spec.model_variant,
                "r_scale": float(spec.r_scale),
                "q_scale": float(spec.q_scale),
                "axis": axis,
                "state_name": str(lever_row["state_name"]),
                "gnss_t": float(first_row["gnss_t"]),
                "state_t": float(first_row["state_t"]),
                "lever_before": float(first_row["lever_before"]),
                "lever_after": float(first_row["lever_after"]),
                "dx_gnss_lever": float(first_row["dx_gnss_lever"]),
                "after_minus_before": float(lever_row["after_minus_before"]),
                "dx_consistency_error_m": float(lever_row["dx_consistency_error_m"]),
                "first_change_time": float(lever_row["first_change_time"]),
                "first_change_value": float(lever_row["first_change_value"]),
                "early_plateau_median_10s": float(lever_row["early_plateau_median_10s"]),
                "early_plateau_abs_error_m": float(lever_row["early_plateau_deviation_m"]),
                "first_update_abs_deviation_m": float(lever_row["first_update_abs_deviation_m"]),
                "drift_source": str(lever_row["drift_source"]),
            }
        )
    return pd.DataFrame(rows).sort_values(by=["axis"]).reset_index(drop=True)


def merge_case_metrics(spec: CaseSpec, case_row: dict[str, Any], lever_df: pd.DataFrame) -> dict[str, Any]:
    row = dict(case_row)
    row["model_variant"] = spec.model_variant
    row["r_scale"] = float(spec.r_scale)
    row["q_scale"] = float(spec.q_scale)
    final_devs = lever_df["final_deviation_from_truth_m"].to_numpy(dtype=float)
    row["final_lever_dev_norm_m"] = float(np.linalg.norm(final_devs))
    row["final_lever_dev_max_m"] = float(np.max(final_devs))
    for axis in AXIS_ORDER:
        lever_row = lever_df.loc[lever_df["axis"] == axis].iloc[0]
        prefix = f"gnss_lever_{axis}"
        row[f"{prefix}_final_abs_error_m"] = float(lever_row["final_deviation_from_truth_m"])
        row[f"{prefix}_max_abs_deviation_m"] = float(lever_row["max_abs_deviation_from_truth_m"])
        row[f"{prefix}_early_plateau_abs_error_m"] = float(lever_row["early_plateau_deviation_m"])
    return row


def aggregate_lever_rows(spec: CaseSpec, lever_df: pd.DataFrame) -> pd.DataFrame:
    df = lever_df.copy()
    df.insert(0, "q_scale", float(spec.q_scale))
    df.insert(0, "r_scale", float(spec.r_scale))
    df.insert(0, "model_variant", spec.model_variant)
    df.insert(0, "case_id", spec.case_id)
    return df


def pivot_metric(case_metrics_df: pd.DataFrame, model_variant: str, value_col: str) -> pd.DataFrame:
    subset = case_metrics_df.loc[case_metrics_df["model_variant"] == model_variant].copy()
    return subset.pivot(index="q_scale", columns="r_scale", values=value_col).sort_index().sort_index(axis=1)


def plot_metric_heatmap(
    case_metrics_df: pd.DataFrame,
    model_variant: str,
    value_col: str,
    title: str,
    cbar_label: str,
    output_path: Path,
) -> None:
    heat = pivot_metric(case_metrics_df, model_variant, value_col)
    fig, ax = plt.subplots(figsize=(7, 5.5))
    im = ax.imshow(heat.to_numpy(dtype=float), aspect="auto", origin="lower", cmap="viridis")
    ax.set_xticks(np.arange(len(heat.columns)), labels=[scale_slug(float(x)) for x in heat.columns])
    ax.set_yticks(np.arange(len(heat.index)), labels=[scale_slug(float(y)) for y in heat.index])
    ax.set_xlabel("GNSS_POS effective std scale")
    ax.set_ylabel("GNSS lever process noise scale")
    ax.set_title(title)
    for i, q_scale in enumerate(heat.index):
        for j, r_scale in enumerate(heat.columns):
            ax.text(j, i, f"{heat.loc[q_scale, r_scale]:.3f}", ha="center", va="center", color="white", fontsize=8)
    fig.colorbar(im, ax=ax, label=cbar_label)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_slice_curves(case_metrics_df: pd.DataFrame, model_variant: str, output_path: Path) -> None:
    subset = case_metrics_df.loc[case_metrics_df["model_variant"] == model_variant].copy()
    q_slice = subset.loc[np.isclose(subset["q_scale"], FIRST_UPDATE_Q_FIXED)].sort_values("r_scale")
    r_slice = subset.loc[np.isclose(subset["r_scale"], FIRST_UPDATE_R_FIXED)].sort_values("q_scale")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    q_x = q_slice["r_scale"].to_numpy(dtype=float)
    r_x = r_slice["q_scale"].to_numpy(dtype=float)
    metrics = [
        ("final_lever_dev_norm_m", "final lever dev norm [m]"),
        ("gnss_lever_z_final_abs_error_m", "lever z final abs error [m]"),
        ("gnss_lever_z_early_plateau_abs_error_m", "lever z early plateau abs error [m]"),
        ("nav_rmse_3d_m", "nav rmse 3d [m]"),
    ]
    for ax, (metric, ylabel) in zip(axes.flat, metrics):
        ax.plot(q_x, q_slice[metric].to_numpy(dtype=float), marker="o", linewidth=1.2, label="vary R, Q=1")
        ax.plot(r_x, r_slice[metric].to_numpy(dtype=float), marker="s", linewidth=1.2, label="vary Q, R=1")
        ax.set_xscale("log")
        ax.grid(alpha=0.25)
        ax.set_ylabel(ylabel)
        ax.legend(loc="best")
    axes[0, 0].set_title(f"{model_variant}: fixed-slice sensitivity")
    axes[1, 0].set_xlabel("scale")
    axes[1, 1].set_xlabel("scale")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_best_case_comparison(
    truth_reference: dict[str, Any],
    cases: list[tuple[str, Path]],
    output_path: Path,
    title: str,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for label, state_path in cases:
        state_df = pd.read_csv(state_path, usecols=["timestamp", "gnss_lever_x_m", "gnss_lever_y_m", "gnss_lever_z_m"])
        timestamps = state_df["timestamp"].to_numpy(dtype=float)
        for idx, axis in enumerate(AXIS_ORDER):
            state_name = f"gnss_lever_{axis}"
            column = f"{state_name}_m"
            truth_value = float(truth_reference["states"][state_name]["reference_value"])
            t_plot, y_plot = downsample_for_plot(timestamps, state_df[column].to_numpy(dtype=float))
            axes[idx].plot(t_plot, y_plot, linewidth=1.2, label=label)
            axes[idx].axhline(truth_value, linestyle="--", color="black", linewidth=1.0, label="truth" if label == cases[0][0] else None)
            axes[idx].set_ylabel("m")
            axes[idx].grid(alpha=0.25)
    axes[-1].set_xlabel("timestamp [s]")
    axes[0].set_title(title)
    for ax in axes:
        handles, labels = ax.get_legend_handles_labels()
        uniq: dict[str, Any] = {}
        for handle, label in zip(handles, labels):
            if label and label not in uniq:
                uniq[label] = handle
        ax.legend(uniq.values(), uniq.keys(), loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_model_best_imu_groups(state_path: Path, ref_df: pd.DataFrame, model_variant: str, output_dir: Path) -> None:
    state_df = pd.read_csv(state_path, usecols=["timestamp"] + [col for cols in IMU_GROUP_COLUMNS.values() for col in cols])
    for group, columns in IMU_GROUP_COLUMNS.items():
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        timestamps = state_df["timestamp"].to_numpy(dtype=float)
        for idx, axis in enumerate(AXIS_ORDER):
            column = columns[idx]
            mean_col = f"{group}_{axis}_mean"
            lower_col = f"{group}_{axis}_lower"
            upper_col = f"{group}_{axis}_upper"
            axes[idx].plot(timestamps, state_df[column].to_numpy(dtype=float), linewidth=1.1, label=f"est {group}_{axis}")
            axes[idx].plot(
                ref_df["timestamp"].to_numpy(dtype=float),
                ref_df[mean_col].to_numpy(dtype=float),
                linestyle="--",
                color="black",
                linewidth=1.0,
                label="modeled truth mean",
            )
            axes[idx].fill_between(
                ref_df["timestamp"].to_numpy(dtype=float),
                ref_df[lower_col].to_numpy(dtype=float),
                ref_df[upper_col].to_numpy(dtype=float),
                color="#bdbdbd",
                alpha=0.25,
                label="modeled truth ±3σ",
            )
            axes[idx].grid(alpha=0.25)
            axes[idx].legend(loc="best")
        axes[-1].set_xlabel("timestamp [s]")
        fig.suptitle(f"{model_variant} {group} vs modeled truth")
        fig.tight_layout()
        fig.savefig(output_dir / f"{model_variant}_{group}_modeled_truth_compare.png", dpi=160)
        plt.close(fig)


def select_best_case(case_metrics_df: pd.DataFrame, model_variant: str, local_only: bool) -> pd.Series:
    subset = case_metrics_df.loc[case_metrics_df["model_variant"] == model_variant].copy()
    if local_only:
        subset = subset.loc[
            subset["r_scale"].isin(LOCAL_R_SCALES) & subset["q_scale"].isin(LOCAL_Q_SCALES)
        ].copy()
    subset = subset.sort_values(
        by=[
            "final_lever_dev_norm_m",
            "gnss_lever_z_final_abs_error_m",
            "nav_rmse_3d_m",
            "nav_final_err_3d_m",
        ],
        ascending=[True, True, True, True],
    )
    if subset.empty:
        raise RuntimeError(f"no cases available for model={model_variant}, local_only={local_only}")
    return subset.iloc[0]


def describe_first_update_sensitivity(first_update_df: pd.DataFrame, model_variant: str) -> dict[str, Any]:
    subset = first_update_df.loc[first_update_df["model_variant"] == model_variant].copy()
    z_df = subset.loc[subset["axis"] == "z"].copy()
    r_slice = z_df.loc[np.isclose(z_df["q_scale"], FIRST_UPDATE_Q_FIXED)].sort_values("r_scale")
    q_slice = z_df.loc[np.isclose(z_df["r_scale"], FIRST_UPDATE_R_FIXED)].sort_values("q_scale")
    r_values = r_slice["dx_gnss_lever"].abs().to_numpy(dtype=float)
    q_values = q_slice["dx_gnss_lever"].abs().to_numpy(dtype=float)
    baseline = float(
        z_df.loc[
            np.isclose(z_df["r_scale"], FIRST_UPDATE_R_FIXED) & np.isclose(z_df["q_scale"], FIRST_UPDATE_Q_FIXED),
            "dx_gnss_lever",
        ]
        .abs()
        .iloc[0]
    )
    baseline = max(baseline, 1.0e-9)
    r_rel_range = float((np.max(r_values) - np.min(r_values)) / baseline) if r_values.size else 0.0
    q_rel_range = float((np.max(q_values) - np.min(q_values)) / baseline) if q_values.size else 0.0
    if r_rel_range >= R_DOMINANT_MIN_REL_RANGE and q_rel_range <= Q_NEGLIGIBLE_MAX_REL_RANGE:
        label = "R_dominant_first_update"
    elif q_rel_range > r_rel_range:
        label = "Q_non_negligible_even_on_first_update"
    else:
        label = "mixed_or_low_sensitivity"
    return {
        "first_update_sensitivity_label": label,
        "first_update_z_r_rel_range": float(r_rel_range),
        "first_update_z_q_rel_range": float(q_rel_range),
    }


def tuning_conclusion(baseline_row: pd.Series, local_best_row: pd.Series, global_best_row: pd.Series) -> dict[str, Any]:
    baseline_final_norm = float(baseline_row["final_lever_dev_norm_m"])
    baseline_final_z = float(baseline_row["gnss_lever_z_final_abs_error_m"])
    baseline_nav = float(baseline_row["nav_rmse_3d_m"])
    baseline_early_z = float(baseline_row["gnss_lever_z_early_plateau_abs_error_m"])

    def improvement(candidate: pd.Series, col: str, baseline: float) -> float:
        return float((baseline - float(candidate[col])) / max(baseline, 1.0e-9))

    def nav_degrade(candidate: pd.Series) -> float:
        return float((float(candidate["nav_rmse_3d_m"]) - baseline_nav) / max(baseline_nav, 1.0e-9))

    local_final_improve = improvement(local_best_row, "final_lever_dev_norm_m", baseline_final_norm)
    local_z_improve = improvement(local_best_row, "gnss_lever_z_final_abs_error_m", baseline_final_z)
    local_nav_degrade = nav_degrade(local_best_row)

    global_final_improve = improvement(global_best_row, "final_lever_dev_norm_m", baseline_final_norm)
    global_z_improve = improvement(global_best_row, "gnss_lever_z_final_abs_error_m", baseline_final_z)
    global_nav_degrade = nav_degrade(global_best_row)
    global_early_z_improve = improvement(global_best_row, "gnss_lever_z_early_plateau_abs_error_m", baseline_early_z)

    local_pass = local_final_improve >= 0.40 and local_z_improve >= 0.50 and local_nav_degrade <= 0.25
    global_pass = global_final_improve >= 0.40 and global_z_improve >= 0.50 and global_nav_degrade <= 0.25

    if local_pass:
        label = "tuning_plausible_primary"
    elif global_pass and global_early_z_improve >= EARLY_Z_UNCHANGED_IMPROVEMENT_THRESHOLD:
        label = "tuning_only_extreme_sensitive"
    else:
        label = "structural_or_non_tuning_primary"

    return {
        "tuning_conclusion": label,
        "baseline_final_lever_dev_norm_m": baseline_final_norm,
        "baseline_final_lever_z_abs_error_m": baseline_final_z,
        "baseline_nav_rmse_3d_m": baseline_nav,
        "baseline_early_plateau_z_abs_error_m": baseline_early_z,
        "local_best_final_improve_ratio": float(local_final_improve),
        "local_best_z_improve_ratio": float(local_z_improve),
        "local_best_nav_degrade_ratio": float(local_nav_degrade),
        "global_best_final_improve_ratio": float(global_final_improve),
        "global_best_z_improve_ratio": float(global_z_improve),
        "global_best_nav_degrade_ratio": float(global_nav_degrade),
        "global_best_early_z_improve_ratio": float(global_early_z_improve),
    }


def build_judgement(case_metrics_df: pd.DataFrame, first_update_df: pd.DataFrame, models: list[str]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    model_to_range: dict[str, float] = {}
    for model_variant in models:
        subset = case_metrics_df.loc[case_metrics_df["model_variant"] == model_variant].copy()
        baseline = float(
            subset.loc[np.isclose(subset["r_scale"], 1.0) & np.isclose(subset["q_scale"], 1.0), "final_lever_dev_norm_m"].iloc[0]
        )
        model_to_range[model_variant] = float(
            (subset["final_lever_dev_norm_m"].max() - subset["final_lever_dev_norm_m"].min()) / max(baseline, 1.0e-9)
        )

    for model_variant in models:
        baseline_row = case_metrics_df.loc[
            (case_metrics_df["model_variant"] == model_variant)
            & np.isclose(case_metrics_df["r_scale"], 1.0)
            & np.isclose(case_metrics_df["q_scale"], 1.0)
        ].iloc[0]
        local_best_row = select_best_case(case_metrics_df, model_variant, local_only=True)
        global_best_row = select_best_case(case_metrics_df, model_variant, local_only=False)
        row = {
            "model_variant": model_variant,
            "baseline_case_id": str(baseline_row["case_id"]),
            "local_best_case_id": str(local_best_row["case_id"]),
            "global_best_case_id": str(global_best_row["case_id"]),
            "local_best_r_scale": float(local_best_row["r_scale"]),
            "local_best_q_scale": float(local_best_row["q_scale"]),
            "global_best_r_scale": float(global_best_row["r_scale"]),
            "global_best_q_scale": float(global_best_row["q_scale"]),
        }
        row.update(tuning_conclusion(baseline_row, local_best_row, global_best_row))
        row.update(describe_first_update_sensitivity(first_update_df, model_variant))
        other_models = [item for item in models if item != model_variant]
        if other_models:
            other = other_models[0]
            own_range = model_to_range[model_variant]
            other_range = model_to_range[other]
            if own_range >= MODEL_SENSITIVITY_FACTOR * other_range:
                relation = f"more_sensitive_than_{other}"
            elif other_range >= MODEL_SENSITIVITY_FACTOR * own_range:
                relation = f"less_sensitive_than_{other}"
            else:
                relation = f"similar_to_{other}"
            row["model_sensitivity_vs_other"] = relation
        else:
            row["model_sensitivity_vs_other"] = "single_model_run"
        row["grid_final_dev_rel_range"] = float(model_to_range[model_variant])
        rows.append(row)
    return pd.DataFrame(rows)


def load_baseline_reference() -> dict[str, Any]:
    out: dict[str, Any] = {"available": False}
    case_path = BASELINE_REFERENCE_DIR / "case_metrics.csv"
    lever_path = BASELINE_REFERENCE_DIR / "lever_metrics.csv"
    if not case_path.exists() or not lever_path.exists():
        return out
    case_df = pd.read_csv(case_path)
    lever_df = pd.read_csv(lever_path)
    out["available"] = True
    out["nav_rmse_3d_m"] = float(case_df["nav_rmse_3d_m"].iloc[0])
    out["nav_final_err_3d_m"] = float(case_df["nav_final_err_3d_m"].iloc[0])
    for axis in AXIS_ORDER:
        row = lever_df.loc[lever_df["axis"] == axis].iloc[0]
        out[f"gnss_lever_{axis}_final_abs_error_m"] = float(row["final_abs_error_m"])
        out[f"gnss_lever_{axis}_early_plateau_abs_error_m"] = float(row["early_plateau_abs_error_m"])
    out["final_lever_dev_norm_m"] = float(
        math.sqrt(sum(out[f"gnss_lever_{axis}_final_abs_error_m"] ** 2 for axis in AXIS_ORDER))
    )
    return out


def baseline_reproduction_check(case_metrics_df: pd.DataFrame, baseline_ref: dict[str, Any]) -> dict[str, Any]:
    row = case_metrics_df.loc[
        (case_metrics_df["model_variant"] == "eskf")
        & np.isclose(case_metrics_df["r_scale"], 1.0)
        & np.isclose(case_metrics_df["q_scale"], 1.0)
    ].iloc[0]
    if not baseline_ref.get("available", False):
        return {"baseline_reproduction_available": False}
    check = {
        "baseline_reproduction_available": True,
        "nav_rmse_3d_delta_m": float(float(row["nav_rmse_3d_m"]) - baseline_ref["nav_rmse_3d_m"]),
        "nav_final_err_3d_delta_m": float(float(row["nav_final_err_3d_m"]) - baseline_ref["nav_final_err_3d_m"]),
        "final_lever_dev_norm_delta_m": float(float(row["final_lever_dev_norm_m"]) - baseline_ref["final_lever_dev_norm_m"]),
    }
    for axis in AXIS_ORDER:
        check[f"gnss_lever_{axis}_final_abs_error_delta_m"] = float(
            float(row[f"gnss_lever_{axis}_final_abs_error_m"]) - baseline_ref[f"gnss_lever_{axis}_final_abs_error_m"]
        )
    return check


def write_summary(output_path: Path, judgement_df: pd.DataFrame, case_metrics_df: pd.DataFrame, manifest: dict[str, Any], models: list[str]) -> None:
    lines: list[str] = [
        "# data2 INS/GNSS GNSS lever noise coupling sweep",
        "",
        "## 1. 当前 baseline 下的杆臂偏差规模",
    ]
    for model_variant in models:
        baseline = case_metrics_df.loc[
            (case_metrics_df["model_variant"] == model_variant)
            & np.isclose(case_metrics_df["r_scale"], 1.0)
            & np.isclose(case_metrics_df["q_scale"], 1.0)
        ].iloc[0]
        lines.append(
            f"- `{model_variant}` baseline=`{baseline['case_id']}`: "
            f"`final_lever_dev_norm={format_metric(float(baseline['final_lever_dev_norm_m']))}` m, "
            f"`lever_z_final_abs_error={format_metric(float(baseline['gnss_lever_z_final_abs_error_m']))}` m, "
            f"`early_plateau_z_abs_error={format_metric(float(baseline['gnss_lever_z_early_plateau_abs_error_m']))}` m, "
            f"`nav_rmse_3d={format_metric(float(baseline['nav_rmse_3d_m']))}` m."
        )

    lines.extend(["", "## 2. 合理 R/Q 范围内是否能显著改善"])
    for _, row in judgement_df.iterrows():
        lines.append(
            f"- `{row['model_variant']}`: `local_best={row['local_best_case_id']}` "
            f"(R=`{format_metric(float(row['local_best_r_scale']))}`x, Q=`{format_metric(float(row['local_best_q_scale']))}`x), "
            f"`global_best={row['global_best_case_id']}` "
            f"(R=`{format_metric(float(row['global_best_r_scale']))}`x, Q=`{format_metric(float(row['global_best_q_scale']))}`x), "
            f"`tuning_conclusion={row['tuning_conclusion']}`, "
            f"`local_final_improve={format_metric(float(row['local_best_final_improve_ratio']))}`, "
            f"`global_final_improve={format_metric(float(row['global_best_final_improve_ratio']))}`."
        )

    lines.extend(["", "## 3. 首更错误平台对 R 与 Q 的敏感性"])
    for _, row in judgement_df.iterrows():
        lines.append(
            f"- `{row['model_variant']}`: `first_update_label={row['first_update_sensitivity_label']}`, "
            f"`z_r_rel_range={format_metric(float(row['first_update_z_r_rel_range']))}`, "
            f"`z_q_rel_range={format_metric(float(row['first_update_z_q_rel_range']))}`."
        )

    lines.extend(["", "## 4. ESKF 与 true_iekf 是否表现出不同的参数敏感性"])
    for _, row in judgement_df.iterrows():
        lines.append(
            f"- `{row['model_variant']}`: `grid_final_dev_rel_range={format_metric(float(row['grid_final_dev_rel_range']))}`, "
            f"`model_sensitivity_vs_other={row['model_sensitivity_vs_other']}`."
        )

    lines.extend(["", "## 5. 最终判断"])
    if (judgement_df["tuning_conclusion"] == "tuning_plausible_primary").any():
        lines.append("- 至少有一个模型在合理 `R/Q` 局部范围内可显著改善，参数设置对当前耦合具有强解释力。")
    elif (judgement_df["tuning_conclusion"] == "tuning_only_extreme_sensitive").any():
        lines.append("- 只有极端参数组合才明显改善，说明调参有影响，但不足以把问题解释成单纯参数设置不当。")
    else:
        lines.append("- 两个模型都更接近 `structural_or_non_tuning_primary`，说明仅靠 `GNSS_POS` 有效量测噪声与杆臂过程噪声，不足以解释当前杆臂异常。")

    lines.extend(
        [
            "",
            "## Notes",
            f"- baseline experiment reference: `{BASELINE_EXP_ID}`",
            "- 本轮 `GNSS_POS` 噪声扫描是通过缩放 `dataset/data2/rtk.txt` 每历元 `sigma_n/e/d` 实现，而不是改 fallback `sigma_gnss_pos`。",
            "- 本轮未修改任何 solver 数学，只增加 Python 侧实验驱动与结果汇总。",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run GNSS effective-noise x GNSS lever process-noise coupling sweep on data2 INS/GNSS.")
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--gnss-path", type=Path, default=Path("dataset/data2/rtk.txt"))
    parser.add_argument("--readme-path", type=Path, default=Path("dataset/data2/README.md"))
    parser.add_argument("--imu-model", default=IMU_MODEL_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--models", default=",".join(MODELS_DEFAULT))
    parser.add_argument("--r-scales", default=",".join(str(x) for x in R_SCALES_DEFAULT))
    parser.add_argument("--q-scales", default=",".join(str(x) for x in Q_SCALES_DEFAULT))
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.gnss_path = (REPO_ROOT / args.gnss_path).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()
    args.models = [item.strip() for item in args.models.split(",") if item.strip()]
    args.r_scales = parse_scale_list(args.r_scales)
    args.q_scales = parse_scale_list(args.q_scales)
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    args.gnss_variant_dir = args.output_dir / "gnss_variants"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")
    if not args.gnss_path.exists():
        raise FileNotFoundError(f"missing GNSS file: {args.gnss_path}")
    if not args.readme_path.exists():
        raise FileNotFoundError(f"missing README file: {args.readme_path}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)
    ensure_dir(args.gnss_variant_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False), encoding="utf-8")

    source_gnss_df = read_rtk_file(args.gnss_path)
    source_sigma_median = {col: float(source_gnss_df[col].median()) for col in SIGMA_COLUMNS}
    gnss_variant_rows: list[dict[str, Any]] = []
    r_scale_to_path: dict[float, Path] = {}
    for r_scale in args.r_scales:
        variant_path = args.gnss_variant_dir / f"rtk_sigma_scale_{scale_slug(r_scale)}.txt"
        variant_row = write_scaled_gnss_variant(args.gnss_path, variant_path, r_scale)
        gnss_variant_rows.append(variant_row)
        r_scale_to_path[float(r_scale)] = variant_path

    specs = [
        CaseSpec(model_variant=model, r_scale=float(r_scale), q_scale=float(q_scale))
        for model in args.models
        for r_scale in args.r_scales
        for q_scale in args.q_scales
    ]

    case_rows: list[dict[str, Any]] = []
    lever_rows: list[pd.DataFrame] = []
    first_update_rows: list[pd.DataFrame] = []
    case_overrides: dict[str, dict[str, Any]] = {}

    for idx, spec in enumerate(specs, start=1):
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg, overrides = build_case_config(
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            readme_params=readme_params,
            gnss_path=r_scale_to_path[float(spec.r_scale)],
            case_dir=case_dir,
            spec=spec,
        )
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)
        probe_row = run_case(case_dir=case_dir, cfg_path=cfg_path, exe_path=args.exe, case_id=spec.case_id)
        state_series_path = (REPO_ROOT / probe_row["state_series_path"]).resolve()
        first_update_path = (REPO_ROOT / probe_row["first_update_path"]).resolve()
        lever_df = build_lever_metrics(truth_reference, state_series_path, first_update_path)
        first_update_df = build_first_update_metrics(spec, lever_df, first_update_path)
        case_rows.append(merge_case_metrics(spec, probe_row, lever_df))
        lever_rows.append(aggregate_lever_rows(spec, lever_df))
        first_update_rows.append(first_update_df)
        case_overrides[spec.case_id] = overrides
        print(f"[{idx}/{len(specs)}] completed {spec.case_id}")

    case_metrics_df = pd.DataFrame(case_rows).sort_values(by=["model_variant", "r_scale", "q_scale"]).reset_index(drop=True)
    lever_metrics_df = pd.concat(lever_rows, ignore_index=True)
    first_update_metrics_df = pd.concat(first_update_rows, ignore_index=True)
    judgement_df = build_judgement(case_metrics_df, first_update_metrics_df, args.models)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    lever_metrics_path = args.output_dir / "lever_metrics.csv"
    first_update_metrics_path = args.output_dir / "first_update_metrics.csv"
    judgement_path = args.output_dir / "sweep_judgement.csv"
    gnss_variants_path = args.output_dir / "gnss_variant_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    lever_metrics_df.to_csv(lever_metrics_path, index=False, encoding="utf-8-sig")
    first_update_metrics_df.to_csv(first_update_metrics_path, index=False, encoding="utf-8-sig")
    judgement_df.to_csv(judgement_path, index=False, encoding="utf-8-sig")
    pd.DataFrame(gnss_variant_rows).to_csv(gnss_variants_path, index=False, encoding="utf-8-sig")

    for model_variant in args.models:
        plot_metric_heatmap(case_metrics_df, model_variant, "final_lever_dev_norm_m", f"{model_variant} final lever deviation norm", "m", args.plot_dir / f"{model_variant}_final_lever_dev_norm_heatmap.png")
        plot_metric_heatmap(case_metrics_df, model_variant, "gnss_lever_z_final_abs_error_m", f"{model_variant} lever z final abs error", "m", args.plot_dir / f"{model_variant}_lever_z_final_abs_error_heatmap.png")
        plot_metric_heatmap(case_metrics_df, model_variant, "gnss_lever_z_early_plateau_abs_error_m", f"{model_variant} lever z early plateau abs error", "m", args.plot_dir / f"{model_variant}_lever_z_early_plateau_heatmap.png")
        plot_metric_heatmap(case_metrics_df, model_variant, "nav_rmse_3d_m", f"{model_variant} nav rmse 3d", "m", args.plot_dir / f"{model_variant}_nav_rmse3d_heatmap.png")
        plot_slice_curves(case_metrics_df, model_variant, args.plot_dir / f"{model_variant}_rq_slice_curves.png")

        baseline_row = case_metrics_df.loc[
            (case_metrics_df["model_variant"] == model_variant)
            & np.isclose(case_metrics_df["r_scale"], 1.0)
            & np.isclose(case_metrics_df["q_scale"], 1.0)
        ].iloc[0]
        local_best_row = select_best_case(case_metrics_df, model_variant, local_only=True)
        global_best_row = select_best_case(case_metrics_df, model_variant, local_only=False)
        plot_best_case_comparison(
            truth_reference,
            [
                ("baseline", (REPO_ROOT / baseline_row["state_series_path"]).resolve()),
                ("local_best", (REPO_ROOT / local_best_row["state_series_path"]).resolve()),
                ("global_best", (REPO_ROOT / global_best_row["state_series_path"]).resolve()),
            ],
            args.plot_dir / f"{model_variant}_baseline_local_global_lever_compare.png",
            f"{model_variant} lever series: baseline vs local/global best",
        )
        best_state_path = (REPO_ROOT / global_best_row["state_series_path"]).resolve()
        best_state_df = pd.read_csv(best_state_path, usecols=["timestamp", "gnss_lever_x_m", "gnss_lever_y_m", "gnss_lever_z_m"])
        plot_gnss_lever_truth_series(best_state_df, truth_reference, args.plot_dir / f"{model_variant}_global_best_gnss_lever_xyz_vs_truth.png")
        plot_gnss_lever_first_window(best_state_df, truth_reference, args.plot_dir / f"{model_variant}_global_best_gnss_lever_xyz_first_10s.png")
        plot_gnss_lever_deviation(best_state_df, truth_reference, args.plot_dir / f"{model_variant}_global_best_gnss_lever_xyz_deviation.png")
        ref_df = modeled_truth_reference_series(
            pd.read_csv(best_state_path, usecols=["timestamp"])["timestamp"].to_numpy(dtype=float),
            truth_reference,
            readme_params,
        )
        plot_model_best_imu_groups(best_state_path, ref_df, model_variant, args.plot_dir)

    baseline_ref = load_baseline_reference()
    baseline_check = baseline_reproduction_check(case_metrics_df, baseline_ref) if "eskf" in args.models else {"baseline_reproduction_available": False}

    freshness = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "lever_metrics_csv": dt.datetime.fromtimestamp(lever_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "first_update_metrics_csv": dt.datetime.fromtimestamp(first_update_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "sweep_judgement_csv": dt.datetime.fromtimestamp(judgement_path.stat().st_mtime).isoformat(timespec="seconds"),
        "gnss_variant_metrics_csv": dt.datetime.fromtimestamp(gnss_variants_path.stat().st_mtime).isoformat(timespec="seconds"),
    }

    manifest_path = args.output_dir / "manifest.json"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "source_gnss_path": rel_from_root(args.gnss_path, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "lever_metrics_csv": rel_from_root(lever_metrics_path, REPO_ROOT),
        "first_update_metrics_csv": rel_from_root(first_update_metrics_path, REPO_ROOT),
        "sweep_judgement_csv": rel_from_root(judgement_path, REPO_ROOT),
        "gnss_variant_metrics_csv": rel_from_root(gnss_variants_path, REPO_ROOT),
        "baseline_experiment_reference": BASELINE_EXP_ID,
        "truth_catalog_source": truth_reference["sources"],
        "imu_truth_model_from_readme": readme_params,
        "models": args.models,
        "r_scales": [float(x) for x in args.r_scales],
        "q_scales": [float(x) for x in args.q_scales],
        "effective_gnss_pos_noise_method": "scaled_rtk_sigma_columns",
        "source_gnss_sigma_median": source_sigma_median,
        "case_overrides": case_overrides,
        "baseline_reproduction_check": baseline_check,
        "freshness": freshness,
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        "assumptions": [
            "本轮固定建立在 EXP-20260320-data2-ins-gnss-lever-truth-imu-params-r2-t3-truth 的 README Markov + T3 lever truth 语义上。",
            "GNSS_POS 有效量测噪声通过缩放 rtk.txt 第 5-7 列 sigma_n/e/d 实现，不依赖 sigma_gnss_pos fallback。",
            "GNSS lever 的 P0 固定为 std=0.2 m，只扫描过程噪声 sigma_gnss_lever_arm。",
            "GNSS_VEL、ODO、NHC、mounting、odo_lever、odo_scale 本轮均不参与。",
            "若 global_best 对 early plateau z 的改善低于 10%，按“首更平台基本未变”处理。",
        ],
    }
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    summary_path = args.output_dir / "summary.md"
    write_summary(summary_path, judgement_df, case_metrics_df, manifest, args.models)

    print(
        json.dumps(
            json_safe(
                {
                    "exp_id": args.exp_id,
                    "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
                    "summary": rel_from_root(summary_path, REPO_ROOT),
                    "manifest": rel_from_root(manifest_path, REPO_ROOT),
                    "case_metrics": rel_from_root(case_metrics_path, REPO_ROOT),
                    "lever_metrics": rel_from_root(lever_metrics_path, REPO_ROOT),
                    "first_update_metrics": rel_from_root(first_update_metrics_path, REPO_ROOT),
                    "sweep_judgement": rel_from_root(judgement_path, REPO_ROOT),
                }
            ),
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
