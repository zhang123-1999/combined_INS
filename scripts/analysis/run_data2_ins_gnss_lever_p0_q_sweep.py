from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import shutil
import sys
from collections import defaultdict
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
    IMU_MODEL_DEFAULT,
    imu_truth_model_internal,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_ins_gnss_lever_truth_init_probe import (
    AXIS_ORDER,
    build_lever_metrics,
)
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    downsample_for_plot,
    family_target_std_human,
    format_metric,
    get_group_vector_internal,
    json_safe,
    reset_directory,
    run_command,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r3-pos320-mainline"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_lever_p0_q_sweep_pos320_mainline")
P0_STD_SCALES_DEFAULT = [0.05, 0.1, 0.2, 0.5, 1.0]
Q_SCALES_DEFAULT = [0.1, 0.25, 1.0, 4.0, 10.0]
STAGE1_Q_SCALE_DEFAULT = 1.0
STAGE2_TOP_K_DEFAULT = 2
BASELINE_REFERENCE_DIR = REPO_ROOT / "output/data2_ins_gnss_lever_truth_imu_params_probe_r2_t3_truth"
GAIN_REFERENCE_CASE_DIR = (
    REPO_ROOT
    / "output/data2_turn_window_y_turn_conditioned_position_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25"
)
BEST_NOISE_LABEL = "noise_only_best"
BEST_P0_ONLY_LABEL = "p0_only_best"
GAIN_REFERENCE_LABEL = "gain_tuned_reference"
BASELINE_REFERENCE_LABEL = "truth_imu_t3_reference"


@dataclass(frozen=True)
class CaseSpec:
    model_variant: str
    p0_std_m: float
    q_scale: float

    @property
    def case_id(self) -> str:
        return (
            f"{self.model_variant}_p0std{scale_slug(self.p0_std_m)}"
            f"_qlever{scale_slug(self.q_scale)}"
        )


def scale_slug(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def parse_scale_list(raw: str) -> list[float]:
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if not items:
        raise ValueError("scale list must not be empty")
    return [float(item) for item in items]


def unique_sorted(values: list[float], baseline: float | None = None) -> list[float]:
    items = {float(value) for value in values}
    if baseline is not None:
        items.add(float(baseline))
    return sorted(items)


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
    init_cfg["std_gnss_lever_arm"] = [float(spec.p0_std_m)] * 3
    p0_diag[28:31] = [float(spec.p0_std_m * spec.p0_std_m)] * 3
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
        "model_variant": spec.model_variant,
        "effective_init_std_gnss_lever_m": [float(spec.p0_std_m)] * 3,
        "effective_P0_diag[28:31]": [float(spec.p0_std_m * spec.p0_std_m)] * 3,
        "effective_q_scale": float(spec.q_scale),
        "effective_sigma_gnss_lever_arm": float(gnss_lever_noise),
        "effective_sigma_gnss_lever_arm_vec": [float(gnss_lever_noise)] * 3,
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
        "fusion.init.gnss_lever_arm0": [0.0, 0.0, 0.0],
        "fusion.init.std_gnss_lever_arm": [float(spec.p0_std_m)] * 3,
        "fusion.init.P0_diag[28:31]": [float(spec.p0_std_m * spec.p0_std_m)] * 3,
        "fusion.noise.sigma_gnss_lever_arm": float(gnss_lever_noise),
        "fusion.noise.sigma_gnss_lever_arm_vec": [float(gnss_lever_noise)] * 3,
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


def lever_summary_from_df(lever_df: pd.DataFrame) -> dict[str, Any]:
    row: dict[str, Any] = {}
    final_errors: list[float] = []
    for axis in AXIS_ORDER:
        axis_row = lever_df.loc[lever_df["axis"] == axis].iloc[0]
        final_col = (
            "final_deviation_from_truth_m"
            if "final_deviation_from_truth_m" in axis_row.index
            else "final_abs_error_m"
        )
        early_col = (
            "early_plateau_deviation_m"
            if "early_plateau_deviation_m" in axis_row.index
            else "early_plateau_abs_error_m"
        )
        final_error = float(axis_row[final_col])
        final_errors.append(final_error)
        row[f"gnss_lever_{axis}_final_abs_error_m"] = final_error
        row[f"gnss_lever_{axis}_early_plateau_abs_error_m"] = float(axis_row[early_col])
        if "first_change_time" in axis_row.index:
            row[f"gnss_lever_{axis}_first_change_time"] = float(axis_row["first_change_time"])
        if "first_update_abs_deviation_m" in axis_row.index:
            row[f"gnss_lever_{axis}_first_update_abs_deviation_m"] = float(
                axis_row["first_update_abs_deviation_m"]
            )
        if "max_abs_deviation_from_truth_m" in axis_row.index:
            row[f"gnss_lever_{axis}_max_abs_deviation_m"] = float(
                axis_row["max_abs_deviation_from_truth_m"]
            )
    row["final_lever_dev_norm_m"] = float(np.linalg.norm(final_errors))
    row["final_lever_dev_max_m"] = float(np.max(final_errors))
    return row


def aggregate_lever_rows(spec: CaseSpec, lever_df: pd.DataFrame) -> pd.DataFrame:
    df = lever_df.copy()
    df.insert(0, "q_scale", float(spec.q_scale))
    df.insert(0, "p0_std_m", float(spec.p0_std_m))
    df.insert(0, "model_variant", spec.model_variant)
    df.insert(0, "case_id", spec.case_id)
    return df


def merge_case_metrics(spec: CaseSpec, case_row: dict[str, Any], lever_df: pd.DataFrame) -> dict[str, Any]:
    row = dict(case_row)
    row["model_variant"] = spec.model_variant
    row["p0_std_m"] = float(spec.p0_std_m)
    row["q_scale"] = float(spec.q_scale)
    row.update(lever_summary_from_df(lever_df))
    return row


def rank_case_table(case_metrics_df: pd.DataFrame) -> pd.DataFrame:
    return case_metrics_df.sort_values(
        by=[
            "final_lever_dev_norm_m",
            "gnss_lever_y_final_abs_error_m",
            "gnss_lever_z_final_abs_error_m",
            "gnss_lever_x_final_abs_error_m",
            "nav_rmse_3d_m",
            "nav_final_err_3d_m",
        ],
        ascending=[True, True, True, True, True, True],
    ).reset_index(drop=True)


def select_stage2_candidates(stage1_df: pd.DataFrame, top_k: int) -> pd.DataFrame:
    ranked = rank_case_table(stage1_df).copy()
    ranked["stage1_rank"] = np.arange(1, len(ranked) + 1)
    return ranked.head(top_k).reset_index(drop=True)


def load_archived_baseline_reference() -> dict[str, Any]:
    case_path = BASELINE_REFERENCE_DIR / "case_metrics.csv"
    lever_path = BASELINE_REFERENCE_DIR / "lever_metrics.csv"
    out: dict[str, Any] = {
        "label": BASELINE_REFERENCE_LABEL,
        "benchmark_case_type": "truth_modeled_baseline_reference",
        "available": False,
    }
    if not case_path.exists() or not lever_path.exists():
        return out
    case_df = pd.read_csv(case_path)
    lever_df = pd.read_csv(lever_path)
    if case_df.empty or lever_df.empty:
        return out
    row = case_df.iloc[0]
    out["available"] = True
    out["case_id"] = str(row["case_id"])
    out["config_path"] = str(row["config_path"])
    out["sol_path"] = str(row["sol_path"])
    out["state_series_path"] = str(row["state_series_path"])
    out["first_update_path"] = str(row["first_update_path"])
    out["nav_rmse_3d_m"] = float(row["nav_rmse_3d_m"])
    out["nav_final_err_3d_m"] = float(row["nav_final_err_3d_m"])
    out.update(lever_summary_from_df(lever_df))
    return out


def load_reference_case_metrics(
    case_dir: Path,
    truth_reference: dict[str, Any],
    label: str,
    benchmark_case_type: str,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "label": label,
        "benchmark_case_type": benchmark_case_type,
        "available": False,
    }
    if not case_dir.exists():
        return out
    config_files = sorted(case_dir.glob("config_*.yaml"))
    sol_files = sorted(case_dir.glob("SOL_*.txt"))
    state_series_files = sorted(case_dir.glob("state_series_*.csv"))
    first_update_files = sorted(case_dir.glob("first_update_*.csv"))
    if not config_files or not sol_files or not state_series_files or not first_update_files:
        return out
    cfg_path = config_files[0]
    sol_path = sol_files[0]
    state_series_path = state_series_files[0]
    first_update_path = first_update_files[0]
    row: dict[str, Any] = {
        "available": True,
        "case_id": case_dir.name,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "first_update_path": rel_from_root(first_update_path, REPO_ROOT),
    }
    row.update(evaluate_navigation_metrics(cfg_path, sol_path))
    lever_df = build_lever_metrics(truth_reference, state_series_path, first_update_path)
    row.update(lever_summary_from_df(lever_df))
    out.update(row)
    return out


def build_reference_metrics_df(
    truth_reference: dict[str, Any],
    gain_reference_case_dir: Path,
) -> pd.DataFrame:
    rows = [load_archived_baseline_reference()]
    rows.append(
        load_reference_case_metrics(
            gain_reference_case_dir,
            truth_reference,
            label=GAIN_REFERENCE_LABEL,
            benchmark_case_type="gain_tuned_reference",
        )
    )
    return pd.DataFrame(rows)


def baseline_reproduction_check(
    baseline_row: pd.Series,
    reference_metrics_df: pd.DataFrame,
) -> dict[str, Any]:
    ref_df = reference_metrics_df.loc[
        (reference_metrics_df["label"] == BASELINE_REFERENCE_LABEL)
        & reference_metrics_df["available"].astype(bool)
    ]
    if ref_df.empty:
        return {"baseline_reproduction_available": False}
    ref = ref_df.iloc[0]
    result = {
        "baseline_reproduction_available": True,
        "baseline_case_id": str(baseline_row["case_id"]),
        "reference_case_id": str(ref["case_id"]),
        "nav_rmse_3d_delta_m": float(float(baseline_row["nav_rmse_3d_m"]) - float(ref["nav_rmse_3d_m"])),
        "nav_final_err_3d_delta_m": float(
            float(baseline_row["nav_final_err_3d_m"]) - float(ref["nav_final_err_3d_m"])
        ),
        "final_lever_dev_norm_delta_m": float(
            float(baseline_row["final_lever_dev_norm_m"]) - float(ref["final_lever_dev_norm_m"])
        ),
    }
    for axis in AXIS_ORDER:
        result[f"gnss_lever_{axis}_final_abs_error_delta_m"] = float(
            float(baseline_row[f"gnss_lever_{axis}_final_abs_error_m"])
            - float(ref[f"gnss_lever_{axis}_final_abs_error_m"])
        )
    return result


def compare_rows(label: str, row: pd.Series | dict[str, Any]) -> dict[str, Any]:
    src = row if isinstance(row, dict) else row.to_dict()
    out = {
        "label": label,
        "case_id": str(src["case_id"]),
        "nav_rmse_3d_m": float(src["nav_rmse_3d_m"]),
        "nav_final_err_3d_m": float(src["nav_final_err_3d_m"]),
        "final_lever_dev_norm_m": float(src["final_lever_dev_norm_m"]),
    }
    for axis in AXIS_ORDER:
        out[f"gnss_lever_{axis}_final_abs_error_m"] = float(src[f"gnss_lever_{axis}_final_abs_error_m"])
        out[f"gnss_lever_{axis}_early_plateau_abs_error_m"] = float(
            src[f"gnss_lever_{axis}_early_plateau_abs_error_m"]
        )
    return out


def build_best_comparison_df(
    baseline_row: pd.Series,
    best_p0_row: pd.Series,
    best_noise_row: pd.Series,
    reference_metrics_df: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = [
        compare_rows("current_baseline", baseline_row),
        compare_rows(BEST_P0_ONLY_LABEL, best_p0_row),
        compare_rows(BEST_NOISE_LABEL, best_noise_row),
    ]
    for _, ref in reference_metrics_df.loc[reference_metrics_df["available"].astype(bool)].iterrows():
        rows.append(compare_rows(str(ref["label"]), ref))
    return pd.DataFrame(rows)


def build_interpretation(
    baseline_row: pd.Series,
    best_p0_row: pd.Series,
    best_noise_row: pd.Series,
    reference_metrics_df: pd.DataFrame,
) -> dict[str, Any]:
    baseline_norm = float(baseline_row["final_lever_dev_norm_m"])
    best_p0_norm = float(best_p0_row["final_lever_dev_norm_m"])
    best_noise_norm = float(best_noise_row["final_lever_dev_norm_m"])
    p0_improve = float((baseline_norm - best_p0_norm) / max(baseline_norm, 1.0e-9))
    q_incremental = float((best_p0_norm - best_noise_norm) / max(best_p0_norm, 1.0e-9))

    if p0_improve >= 0.10 and q_incremental <= 0.03:
        label = "p0_dominant"
    elif p0_improve <= 0.03 and q_incremental >= 0.10:
        label = "q_dominant"
    elif p0_improve >= 0.05 and q_incremental >= 0.05:
        label = "p0_and_q_both_matter"
    else:
        label = "limited_noise_only_gain"

    out = {
        "noise_interpretation": label,
        "baseline_final_lever_dev_norm_m": baseline_norm,
        "best_p0_only_final_lever_dev_norm_m": best_p0_norm,
        "best_noise_final_lever_dev_norm_m": best_noise_norm,
        "p0_only_improve_ratio": p0_improve,
        "q_incremental_improve_ratio": q_incremental,
        "best_noise_p0_std_m": float(best_noise_row["p0_std_m"]),
        "best_noise_q_scale": float(best_noise_row["q_scale"]),
        "best_p0_only_p0_std_m": float(best_p0_row["p0_std_m"]),
    }
    gain_ref_df = reference_metrics_df.loc[
        (reference_metrics_df["label"] == GAIN_REFERENCE_LABEL)
        & reference_metrics_df["available"].astype(bool)
    ]
    if not gain_ref_df.empty:
        gain_ref = gain_ref_df.iloc[0]
        out["gain_reference_final_lever_dev_norm_m"] = float(gain_ref["final_lever_dev_norm_m"])
        out["best_noise_vs_gain_reference_gap_m"] = float(
            best_noise_norm - float(gain_ref["final_lever_dev_norm_m"])
        )
    return out


def plot_stage1_p0_sweep(stage1_df: pd.DataFrame, output_path: Path) -> None:
    df = stage1_df.sort_values("p0_std_m")
    x = df["p0_std_m"].to_numpy(dtype=float)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    metrics = [
        ("final_lever_dev_norm_m", "final lever dev norm [m]"),
        ("gnss_lever_y_final_abs_error_m", "lever y final abs error [m]"),
        ("gnss_lever_z_final_abs_error_m", "lever z final abs error [m]"),
        ("nav_rmse_3d_m", "nav rmse 3d [m]"),
    ]
    for ax, (metric, ylabel) in zip(axes.flat, metrics):
        ax.plot(x, df[metric].to_numpy(dtype=float), marker="o", linewidth=1.3)
        ax.set_xscale("log")
        ax.set_xlabel("GNSS lever init std [m]")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.25)
    axes[0, 0].set_title("Stage-1 P0 sweep (Q fixed at baseline)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_stage2_q_sweeps(stage2_df: pd.DataFrame, candidate_df: pd.DataFrame, output_path: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    metrics = [
        ("final_lever_dev_norm_m", "final lever dev norm [m]"),
        ("gnss_lever_y_final_abs_error_m", "lever y final abs error [m]"),
        ("gnss_lever_z_final_abs_error_m", "lever z final abs error [m]"),
        ("nav_rmse_3d_m", "nav rmse 3d [m]"),
    ]
    for _, candidate in candidate_df.iterrows():
        subset = stage2_df.loc[np.isclose(stage2_df["p0_std_m"], float(candidate["p0_std_m"]))].sort_values("q_scale")
        q = subset["q_scale"].to_numpy(dtype=float)
        label = f"p0_std={float(candidate['p0_std_m']):g} m"
        for ax, (metric, ylabel) in zip(axes.flat, metrics):
            ax.plot(q, subset[metric].to_numpy(dtype=float), marker="o", linewidth=1.2, label=label)
            ax.set_xscale("log")
            ax.set_xlabel("GNSS lever process-noise scale")
            ax.set_ylabel(ylabel)
            ax.grid(alpha=0.25)
    axes[0, 0].set_title("Stage-2 Q sweeps on selected P0 candidates")
    for ax in axes.flat:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_case_series_comparison(
    truth_reference: dict[str, Any],
    cases: list[tuple[str, Path]],
    output_path: Path,
    title: str,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for label, state_path in cases:
        state_df = pd.read_csv(
            state_path,
            usecols=["timestamp", "gnss_lever_x_m", "gnss_lever_y_m", "gnss_lever_z_m"],
        )
        timestamps = state_df["timestamp"].to_numpy(dtype=float)
        for idx, axis in enumerate(AXIS_ORDER):
            state_name = f"gnss_lever_{axis}"
            column = f"{state_name}_m"
            truth_value = float(truth_reference["states"][state_name]["reference_value_internal"])
            t_plot, y_plot = downsample_for_plot(timestamps, state_df[column].to_numpy(dtype=float))
            axes[idx].plot(t_plot, y_plot, linewidth=1.2, label=label)
            axes[idx].axhline(
                truth_value,
                linestyle="--",
                color="black",
                linewidth=1.0,
                label="truth" if idx == 0 else None,
            )
            axes[idx].set_ylabel("m")
            axes[idx].grid(alpha=0.25)
    axes[-1].set_xlabel("timestamp [s]")
    axes[0].set_title(title)
    for ax in axes:
        handles, labels = ax.get_legend_handles_labels()
        unique: dict[str, Any] = {}
        for handle, label in zip(handles, labels):
            if label and label not in unique:
                unique[label] = handle
        ax.legend(unique.values(), unique.keys(), loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_path: Path,
    truth_reference: dict[str, Any],
    candidate_df: pd.DataFrame,
    case_metrics_df: pd.DataFrame,
    best_comparison_df: pd.DataFrame,
    reference_metrics_df: pd.DataFrame,
    interpretation: dict[str, Any],
    manifest: dict[str, Any],
) -> None:
    baseline_row = case_metrics_df.loc[
        np.isclose(case_metrics_df["p0_std_m"], float(manifest["baseline_p0_std_m"]))
        & np.isclose(case_metrics_df["q_scale"], float(manifest["stage1_q_scale"]))
    ].iloc[0]
    best_p0_row = candidate_df.iloc[0]
    best_noise_row = rank_case_table(case_metrics_df).iloc[0]

    lines: list[str] = [
        "# data2 INS/GNSS GNSS lever P0-Q sweep",
        "",
        "## 1. Baseline and stage winners",
        (
            f"- current baseline `{baseline_row['case_id']}`: "
            f"`p0_std={format_metric(float(baseline_row['p0_std_m']))}` m, "
            f"`q_scale={format_metric(float(baseline_row['q_scale']))}`x, "
            f"`final_lever_dev_norm={format_metric(float(baseline_row['final_lever_dev_norm_m']))}` m, "
            f"`nav_rmse_3d={format_metric(float(baseline_row['nav_rmse_3d_m']))}` m."
        ),
        (
            f"- best P0-only `{best_p0_row['case_id']}`: "
            f"`p0_std={format_metric(float(best_p0_row['p0_std_m']))}` m, "
            f"`final_lever_dev_norm={format_metric(float(best_p0_row['final_lever_dev_norm_m']))}` m, "
            f"`improve={format_metric(float(interpretation['p0_only_improve_ratio']))}`."
        ),
        (
            f"- best noise-only `{best_noise_row['case_id']}`: "
            f"`p0_std={format_metric(float(best_noise_row['p0_std_m']))}` m, "
            f"`q_scale={format_metric(float(best_noise_row['q_scale']))}`x, "
            f"`final_lever_dev_norm={format_metric(float(best_noise_row['final_lever_dev_norm_m']))}` m, "
            f"`incremental_q_gain={format_metric(float(interpretation['q_incremental_improve_ratio']))}`."
        ),
        "",
        "## 2. Stage-1 candidate selection",
    ]
    for _, row in candidate_df.iterrows():
        lines.append(
            f"- rank {int(row['stage1_rank'])}: `{row['case_id']}` "
            f"(`p0_std={format_metric(float(row['p0_std_m']))}` m, "
            f"`final_norm={format_metric(float(row['final_lever_dev_norm_m']))}` m, "
            f"`lever_y={format_metric(float(row['gnss_lever_y_final_abs_error_m']))}` m, "
            f"`lever_z={format_metric(float(row['gnss_lever_z_final_abs_error_m']))}` m)."
        )

    lines.extend(["", "## 3. Noise interpretation"])
    lines.append(
        f"- interpretation=`{interpretation['noise_interpretation']}`; "
        f"`baseline_norm={format_metric(float(interpretation['baseline_final_lever_dev_norm_m']))}` m, "
        f"`best_p0_only_norm={format_metric(float(interpretation['best_p0_only_final_lever_dev_norm_m']))}` m, "
        f"`best_noise_norm={format_metric(float(interpretation['best_noise_final_lever_dev_norm_m']))}` m."
    )
    if "best_noise_vs_gain_reference_gap_m" in interpretation:
        lines.append(
            f"- vs gain reference: "
            f"`gain_ref_norm={format_metric(float(interpretation['gain_reference_final_lever_dev_norm_m']))}` m, "
            f"`noise_minus_gain_gap={format_metric(float(interpretation['best_noise_vs_gain_reference_gap_m']))}` m."
        )

    lines.extend(["", "## 4. Reference comparison"])
    for _, row in best_comparison_df.iterrows():
        lines.append(
            f"- `{row['label']}` (`{row['case_id']}`): "
            f"`final_norm={format_metric(float(row['final_lever_dev_norm_m']))}` m, "
            f"`lever_x/y/z={format_metric(float(row['gnss_lever_x_final_abs_error_m']))}/"
            f"{format_metric(float(row['gnss_lever_y_final_abs_error_m']))}/"
            f"{format_metric(float(row['gnss_lever_z_final_abs_error_m']))}` m, "
            f"`nav_rmse_3d={format_metric(float(row['nav_rmse_3d_m']))}` m."
        )

    lines.extend(["", "## 5. Notes"])
    baseline_ref_df = reference_metrics_df.loc[
        (reference_metrics_df["label"] == BASELINE_REFERENCE_LABEL)
        & reference_metrics_df["available"].astype(bool)
    ]
    if not baseline_ref_df.empty:
        ref = baseline_ref_df.iloc[0]
        lines.append(
            f"- truth-modeled baseline reference available: `{ref['case_id']}`; "
            f"`current-baseline delta norm="
            f"{format_metric(float(baseline_row['final_lever_dev_norm_m']) - float(ref['final_lever_dev_norm_m']))}` m."
        )
    lines.extend(
        [
            "- 本轮固定 `GNSS_POS R`，不再混入量测噪声自由度。",
            "- 本轮未修改 solver 数学路径，只增加 staged P0-Q 实验编排与对照汇总。",
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
            "- `gain_tuned_reference` 仍来自 turn-window 目标下的增益后处理 case；这里把它只当作 external heuristic reference，而不是同目标最优解。",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run staged GNSS lever initial-noise (P0) and process-noise (Q) sweeps on data2 INS/GNSS."
    )
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--gnss-path", type=Path, default=Path("dataset/data2/rtk.txt"))
    parser.add_argument("--readme-path", type=Path, default=Path("dataset/data2/README.md"))
    parser.add_argument("--imu-model", default=IMU_MODEL_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--model-variant", default="eskf")
    parser.add_argument("--p0-std-scales", default=",".join(str(x) for x in P0_STD_SCALES_DEFAULT))
    parser.add_argument("--q-scales", default=",".join(str(x) for x in Q_SCALES_DEFAULT))
    parser.add_argument("--stage1-q-scale", type=float, default=STAGE1_Q_SCALE_DEFAULT)
    parser.add_argument("--stage2-top-k", type=int, default=STAGE2_TOP_K_DEFAULT)
    parser.add_argument("--gain-reference-case-dir", type=Path, default=GAIN_REFERENCE_CASE_DIR)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.gnss_path = (REPO_ROOT / args.gnss_path).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()
    args.gain_reference_case_dir = (REPO_ROOT / args.gain_reference_case_dir).resolve()
    args.p0_std_scales = unique_sorted(parse_scale_list(args.p0_std_scales), family_target_std_human("gnss_lever", 0))
    args.q_scales = unique_sorted(parse_scale_list(args.q_scales), args.stage1_q_scale)
    args.plot_dir = args.output_dir / "plots"
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
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
    if args.stage2_top_k <= 0:
        raise ValueError("stage2-top-k must be positive")

    reset_directory(args.output_dir)
    ensure_dir(args.plot_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)

    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    stage_membership: dict[str, set[str]] = defaultdict(set)
    run_specs: dict[str, CaseSpec] = {}
    case_rows: list[dict[str, Any]] = []
    lever_rows: list[pd.DataFrame] = []
    case_overrides: dict[str, dict[str, Any]] = {}

    stage1_specs = [
        CaseSpec(
            model_variant=args.model_variant,
            p0_std_m=float(p0_std),
            q_scale=float(args.stage1_q_scale),
        )
        for p0_std in args.p0_std_scales
    ]

    for spec in stage1_specs:
        stage_membership[spec.case_id].add("stage1_p0")
        run_specs.setdefault(spec.case_id, spec)

    for idx, spec in enumerate(stage1_specs, start=1):
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg, overrides = build_case_config(
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            readme_params=readme_params,
            gnss_path=args.gnss_path,
            case_dir=case_dir,
            spec=spec,
        )
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)
        probe_row = run_case(case_dir=case_dir, cfg_path=cfg_path, exe_path=args.exe, case_id=spec.case_id)
        state_series_path = (REPO_ROOT / probe_row["state_series_path"]).resolve()
        first_update_path = (REPO_ROOT / probe_row["first_update_path"]).resolve()
        lever_df = build_lever_metrics(truth_reference, state_series_path, first_update_path)
        case_rows.append(merge_case_metrics(spec, probe_row, lever_df))
        lever_rows.append(aggregate_lever_rows(spec, lever_df))
        case_overrides[spec.case_id] = overrides
        print(f"[stage1 {idx}/{len(stage1_specs)}] completed {spec.case_id}")

    case_metrics_df = pd.DataFrame(case_rows)
    stage1_df = case_metrics_df.loc[case_metrics_df["q_scale"].map(float).eq(float(args.stage1_q_scale))].copy()
    candidate_df = select_stage2_candidates(stage1_df, args.stage2_top_k)

    stage2_specs: list[CaseSpec] = []
    for _, candidate in candidate_df.iterrows():
        p0_std_m = float(candidate["p0_std_m"])
        for q_scale in args.q_scales:
            spec = CaseSpec(
                model_variant=args.model_variant,
                p0_std_m=p0_std_m,
                q_scale=float(q_scale),
            )
            stage_membership[spec.case_id].add("stage2_q")
            if spec.case_id not in run_specs:
                run_specs[spec.case_id] = spec
                stage2_specs.append(spec)

    for idx, spec in enumerate(stage2_specs, start=1):
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg, overrides = build_case_config(
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            readme_params=readme_params,
            gnss_path=args.gnss_path,
            case_dir=case_dir,
            spec=spec,
        )
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)
        probe_row = run_case(case_dir=case_dir, cfg_path=cfg_path, exe_path=args.exe, case_id=spec.case_id)
        state_series_path = (REPO_ROOT / probe_row["state_series_path"]).resolve()
        first_update_path = (REPO_ROOT / probe_row["first_update_path"]).resolve()
        lever_df = build_lever_metrics(truth_reference, state_series_path, first_update_path)
        case_rows.append(merge_case_metrics(spec, probe_row, lever_df))
        lever_rows.append(aggregate_lever_rows(spec, lever_df))
        case_overrides[spec.case_id] = overrides
        print(f"[stage2 {idx}/{len(stage2_specs)}] completed {spec.case_id}")

    case_metrics_df = pd.DataFrame(case_rows).sort_values(
        by=["p0_std_m", "q_scale", "case_id"]
    ).reset_index(drop=True)
    lever_metrics_df = pd.concat(lever_rows, ignore_index=True)
    case_metrics_df["included_in_stage1"] = case_metrics_df["case_id"].map(
        lambda case_id: "stage1_p0" in stage_membership[case_id]
    )
    case_metrics_df["included_in_stage2"] = case_metrics_df["case_id"].map(
        lambda case_id: "stage2_q" in stage_membership[case_id]
    )

    reference_metrics_df = build_reference_metrics_df(truth_reference, args.gain_reference_case_dir)
    baseline_row = case_metrics_df.loc[
        np.isclose(case_metrics_df["p0_std_m"], family_target_std_human("gnss_lever", 0))
        & np.isclose(case_metrics_df["q_scale"], args.stage1_q_scale)
    ].iloc[0]
    best_p0_row = candidate_df.iloc[0]
    best_noise_row = rank_case_table(case_metrics_df).iloc[0]
    interpretation = build_interpretation(baseline_row, best_p0_row, best_noise_row, reference_metrics_df)
    best_comparison_df = build_best_comparison_df(
        baseline_row, best_p0_row, best_noise_row, reference_metrics_df
    )

    case_metrics_path = args.output_dir / "case_metrics.csv"
    lever_metrics_path = args.output_dir / "lever_metrics.csv"
    candidate_path = args.output_dir / "candidate_selection.csv"
    reference_metrics_path = args.output_dir / "reference_metrics.csv"
    best_comparison_path = args.output_dir / "best_case_comparison.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    lever_metrics_df.to_csv(lever_metrics_path, index=False, encoding="utf-8-sig")
    candidate_df.to_csv(candidate_path, index=False, encoding="utf-8-sig")
    reference_metrics_df.to_csv(reference_metrics_path, index=False, encoding="utf-8-sig")
    best_comparison_df.to_csv(best_comparison_path, index=False, encoding="utf-8-sig")

    plot_stage1_p0_sweep(stage1_df, args.plot_dir / "stage1_p0_sweep.png")
    plot_stage2_q_sweeps(
        case_metrics_df.loc[case_metrics_df["included_in_stage2"]].copy(),
        candidate_df,
        args.plot_dir / "stage2_q_sweeps.png",
    )
    comparison_cases: list[tuple[str, Path]] = [
        ("baseline", (REPO_ROOT / baseline_row["state_series_path"]).resolve()),
        ("best_noise", (REPO_ROOT / best_noise_row["state_series_path"]).resolve()),
    ]
    gain_ref_df = reference_metrics_df.loc[
        (reference_metrics_df["label"] == GAIN_REFERENCE_LABEL)
        & reference_metrics_df["available"].astype(bool)
    ]
    if not gain_ref_df.empty:
        comparison_cases.append(
            ("gain_ref", (REPO_ROOT / gain_ref_df.iloc[0]["state_series_path"]).resolve())
        )
    plot_case_series_comparison(
        truth_reference,
        comparison_cases,
        args.plot_dir / "best_noise_vs_references_gnss_lever_xyz.png",
        "GNSS lever series: baseline vs best-noise vs references",
    )

    freshness = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "lever_metrics_csv": dt.datetime.fromtimestamp(lever_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "candidate_selection_csv": dt.datetime.fromtimestamp(candidate_path.stat().st_mtime).isoformat(timespec="seconds"),
        "reference_metrics_csv": dt.datetime.fromtimestamp(reference_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "best_case_comparison_csv": dt.datetime.fromtimestamp(best_comparison_path.stat().st_mtime).isoformat(timespec="seconds"),
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
        "candidate_selection_csv": rel_from_root(candidate_path, REPO_ROOT),
        "reference_metrics_csv": rel_from_root(reference_metrics_path, REPO_ROOT),
        "best_case_comparison_csv": rel_from_root(best_comparison_path, REPO_ROOT),
        "baseline_p0_std_m": float(family_target_std_human("gnss_lever", 0)),
        "stage1_q_scale": float(args.stage1_q_scale),
        "stage2_top_k": int(args.stage2_top_k),
        "model_variant": args.model_variant,
        "requested_p0_std_scales_m": [float(x) for x in args.p0_std_scales],
        "requested_q_scales": [float(x) for x in args.q_scales],
        "selected_stage2_candidates": candidate_df[
            ["case_id", "stage1_rank", "p0_std_m", "final_lever_dev_norm_m"]
        ].to_dict(orient="records"),
        "imu_truth_model_from_readme": readme_params,
        "truth_catalog_source": truth_reference["sources"],
        "case_overrides": case_overrides,
        "reference_paths": {
            "truth_imu_baseline_reference_dir": rel_from_root(BASELINE_REFERENCE_DIR, REPO_ROOT),
            "gain_reference_case_dir": rel_from_root(args.gain_reference_case_dir, REPO_ROOT),
        },
        "baseline_reproduction_check": baseline_reproduction_check(baseline_row, reference_metrics_df),
        "interpretation": interpretation,
        "freshness": freshness,
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        "assumptions": [
            "本轮固定为 pure INS/GNSS + README truth-modeled IMU 口径。",
            "本轮固定 GNSS_POS measurement noise，不再扫描 R。",
            "GNSS lever init std 通过 std_gnss_lever_arm + P0_diag[28:31] 一致写入，实际以 P0_diag 为准。",
            "GNSS lever process noise 只扫描 sigma_gnss_lever_arm(_vec) 的等轴缩放。",
            "ODO、NHC、mounting、odo_lever、odo_scale、runtime_truth_anchor_pva 本轮均保持关闭或固定。",
            "GNSS lever 判优真值采用 README `T3` 标称外参；RTK/POS body-frame median 仅作诊断参考。",
        ],
    }
    manifest_path.write_text(
        json.dumps(json_safe(manifest), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    summary_path = args.output_dir / "summary.md"
    write_summary(
        summary_path,
        truth_reference=truth_reference,
        candidate_df=candidate_df,
        case_metrics_df=case_metrics_df,
        best_comparison_df=best_comparison_df,
        reference_metrics_df=reference_metrics_df,
        interpretation=interpretation,
        manifest=manifest,
    )

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
                    "candidate_selection": rel_from_root(candidate_path, REPO_ROOT),
                    "reference_metrics": rel_from_root(reference_metrics_path, REPO_ROOT),
                    "best_case_comparison": rel_from_root(best_comparison_path, REPO_ROOT),
                }
            ),
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
