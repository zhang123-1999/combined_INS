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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
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
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    evaluate_navigation_metrics,
    json_safe,
    reset_directory,
)


AXES = ("pitch", "yaw")
AXIS_TO_STATE_NAME = {"pitch": "mounting_pitch", "yaw": "mounting_yaw"}
AXIS_TO_STATE_COL = {"pitch": "mounting_pitch_deg", "yaw": "mounting_yaw_deg"}
AXIS_TO_TOTAL_COL = {"pitch": "total_mounting_pitch_deg", "yaw": "total_mounting_yaw_deg"}
AXIS_TO_DIAG_COL = {"pitch": "std_mp", "yaw": "std_my"}

TARGET_TOTAL_MOUNTING_DEG = {"pitch": 0.36, "yaw": 0.84}
INIT_TOTAL_MOUNTING_DEG = {"pitch": 0.0, "yaw": 0.0}
STD_MOUNTING_DEG = 3.0
SIGMA_MOUNTING_RAD = 1.0e-6
P0_INDEX = {"pitch": 23, "yaw": 24}

EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-pitch-yaw-fullgnss-zeroinit-r1"
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_pitch_yaw_fullgnss_20260322")
CASE_ID = "release_mounting_pitch_yaw_full_gnss_zero_init"


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


def format_metric(value: float) -> str:
    if value is None or not math.isfinite(float(value)):
        return "NA"
    return f"{float(value):.6f}"


def render_table(df: pd.DataFrame, columns: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for _, row in df.iterrows():
        vals: list[str] = []
        for col in columns:
            value = row[col]
            if isinstance(value, str):
                vals.append(value)
            else:
                vals.append(format_metric(float(value)))
        lines.append("| " + " | ".join(vals) + " |")
    return lines


def summarize_series(values: np.ndarray) -> dict[str, float]:
    return {
        "mean_deg": float(np.mean(values)),
        "std_deg": float(np.std(values)),
        "min_deg": float(np.min(values)),
        "max_deg": float(np.max(values)),
        "p05_deg": float(np.percentile(values, 5.0)),
        "p50_deg": float(np.percentile(values, 50.0)),
        "p95_deg": float(np.percentile(values, 95.0)),
        "peak_to_peak_deg": float(np.ptp(values)),
    }


def classify_motion(speed_m_s: np.ndarray, yaw_rate_deg_s: np.ndarray) -> np.ndarray:
    labels = np.full(speed_m_s.shape, "transition", dtype=object)
    labels[speed_m_s < 3.0] = "low_speed"
    labels[(speed_m_s >= 3.0) & (np.abs(yaw_rate_deg_s) <= 3.0)] = "straight"
    labels[(speed_m_s >= 3.0) & (yaw_rate_deg_s >= 8.0)] = "turn_pos"
    labels[(speed_m_s >= 3.0) & (yaw_rate_deg_s <= -8.0)] = "turn_neg"
    return labels


def load_pos_motion(pos_path: Path) -> pd.DataFrame:
    return pd.read_csv(
        pos_path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def build_truth_reference_for_this_run(base_cfg: dict[str, Any]) -> dict[str, Any]:
    truth_reference = build_truth_reference(base_cfg)
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["pitch"] = float(TARGET_TOTAL_MOUNTING_DEG["pitch"])
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["yaw"] = float(TARGET_TOTAL_MOUNTING_DEG["yaw"])
    truth_reference["sources"]["mounting_total_truth"]["source"] = (
        "current ODO/NHC mounting study truth convention under full-GNSS zero-init run"
    )
    truth_reference["sources"]["constraints_mounting_base_deg"] = [0.0, 0.0, 0.0]
    truth_reference["states"]["mounting_pitch"]["reference_value"] = float(TARGET_TOTAL_MOUNTING_DEG["pitch"])
    truth_reference["states"]["mounting_pitch"]["reference_value_internal"] = float(
        math.radians(TARGET_TOTAL_MOUNTING_DEG["pitch"])
    )
    truth_reference["states"]["mounting_pitch"]["source"] = "constraints base zero, so state equals total pitch"
    truth_reference["states"]["mounting_yaw"]["reference_value"] = float(TARGET_TOTAL_MOUNTING_DEG["yaw"])
    truth_reference["states"]["mounting_yaw"]["reference_value_internal"] = float(
        math.radians(TARGET_TOTAL_MOUNTING_DEG["yaw"])
    )
    truth_reference["states"]["mounting_yaw"]["source"] = "constraints base zero, so state equals total yaw"
    truth_reference["states"]["mounting_roll"]["reference_value"] = 0.0
    truth_reference["states"]["mounting_roll"]["reference_value_internal"] = 0.0
    truth_reference["states"]["mounting_roll"]["source"] = "constraints base zero, roll fixed"
    return truth_reference


def reset_post_gnss_ablation(post_cfg: dict[str, Any]) -> None:
    post_cfg["enabled"] = False
    post_cfg["disable_gnss_lever_arm"] = False
    post_cfg["disable_gnss_lever_z"] = False
    post_cfg["disable_odo_lever_arm"] = False
    post_cfg["disable_odo_scale"] = False
    post_cfg["disable_gyro_bias"] = False
    post_cfg["disable_gyro_scale"] = False
    post_cfg["disable_accel_scale"] = False
    post_cfg["disable_mounting"] = False
    post_cfg["disable_mounting_roll"] = False
    post_cfg["disable_mounting_pitch"] = False
    post_cfg["disable_mounting_yaw"] = False


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    post_ablation_cfg = fusion.setdefault("post_gnss_ablation", {})
    p0_diag = [float(x) for x in init_cfg["P0_diag"]]

    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"

    gnss_path = (REPO_ROOT / "dataset/data2/rtk.txt").resolve()
    fusion["gnss_path"] = rel_from_root(gnss_path, REPO_ROOT)
    fusion["enable_gnss_velocity"] = False
    fusion["gnss_schedule"] = {"enabled": False}
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)

    odo_lever_truth = [float(x) for x in truth_reference["sources"]["odo_lever_truth"]["value_m"]]
    gnss_lever_truth = [float(x) for x in truth_reference["sources"]["gnss_lever_truth"]["value_m"]]

    constraints_cfg["imu_mounting_angle"] = [
        float(INIT_TOTAL_MOUNTING_DEG["pitch"] * 0.0),
        float(INIT_TOTAL_MOUNTING_DEG["pitch"]),
        float(INIT_TOTAL_MOUNTING_DEG["yaw"]),
    ]
    constraints_cfg["odo_lever_arm"] = odo_lever_truth
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
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
    init_cfg["odo_scale"] = 1.0
    init_cfg["mounting_roll0"] = 0.0
    init_cfg["mounting_pitch0"] = float(INIT_TOTAL_MOUNTING_DEG["pitch"])
    init_cfg["mounting_yaw0"] = float(INIT_TOTAL_MOUNTING_DEG["yaw"])
    init_cfg["lever_arm0"] = odo_lever_truth
    init_cfg["gnss_lever_arm0"] = gnss_lever_truth
    init_cfg["std_mounting_pitch"] = float(STD_MOUNTING_DEG)
    init_cfg["std_mounting_yaw"] = float(STD_MOUNTING_DEG)

    p0_diag[P0_INDEX["pitch"]] = float(np.deg2rad(STD_MOUNTING_DEG) ** 2)
    p0_diag[P0_INDEX["yaw"]] = float(np.deg2rad(STD_MOUNTING_DEG) ** 2)
    init_cfg["P0_diag"] = p0_diag

    noise_cfg["sigma_mounting_pitch"] = float(SIGMA_MOUNTING_RAD)
    noise_cfg["sigma_mounting_yaw"] = float(SIGMA_MOUNTING_RAD)

    ablation_cfg["disable_gnss_lever_arm"] = True
    ablation_cfg["disable_gnss_lever_z"] = False
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_mounting"] = False
    ablation_cfg["disable_mounting_roll"] = True
    ablation_cfg["disable_mounting_pitch"] = False
    ablation_cfg["disable_mounting_yaw"] = False

    reset_post_gnss_ablation(post_ablation_cfg)
    return cfg


def run_solver(exe_path: Path, cfg_path: Path) -> str:
    proc = subprocess.run(
        [str(exe_path.resolve()), "--config", str(cfg_path.resolve())],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    if proc.returncode != 0:
        raise RuntimeError(f"solver failed for {cfg_path.name}: returncode={proc.returncode}\n{merged}")
    return merged


def run_case(case_dir: Path, cfg_path: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
    stdout_path = case_dir / f"solver_stdout_{CASE_ID}.txt"
    diag_path = case_dir / f"DIAG_{CASE_ID}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_solver(exe_path, cfg_path)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output: {state_series_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {CASE_ID}")
    shutil.copy2(root_diag, diag_path)
    nav_metrics, segment_rows = evaluate_navigation_metrics(cfg_path, sol_path)
    case_row: dict[str, Any] = {
        "case_id": CASE_ID,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "sol_mtime": mtime_text(sol_path),
        "state_series_mtime": mtime_text(state_series_path),
        "diag_mtime": mtime_text(diag_path),
        "stdout_mtime": mtime_text(stdout_path),
        "segment_rows": segment_rows,
    }
    case_row.update(nav_metrics)
    consistency = parse_consistency_summary(stdout_text)
    case_row["odo_accept_ratio"] = float(consistency.get("ODO", {}).get("accept_ratio", float("nan")))
    case_row["odo_nis_mean"] = float(consistency.get("ODO", {}).get("nis_mean", float("nan")))
    case_row["nhc_accept_ratio"] = float(consistency.get("NHC", {}).get("accept_ratio", float("nan")))
    case_row["nhc_nis_mean"] = float(consistency.get("NHC", {}).get("nis_mean", float("nan")))
    return case_row


def compute_axis_metrics(
    *,
    axis_name: str,
    state_df: pd.DataFrame,
    diag_df: pd.DataFrame,
    pos_df: pd.DataFrame,
    truth_reference: dict[str, Any],
) -> dict[str, Any]:
    state_col = AXIS_TO_STATE_COL[axis_name]
    total_col = AXIS_TO_TOTAL_COL[axis_name]
    diag_col = AXIS_TO_DIAG_COL[axis_name]

    state_t = state_df["timestamp"].to_numpy(dtype=float)
    state_deg = state_df[state_col].to_numpy(dtype=float)
    total_deg = state_df[total_col].to_numpy(dtype=float)

    pos_t = pos_df["t"].to_numpy(dtype=float)
    yaw_truth = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.interp(state_t, pos_t, np.rad2deg(np.gradient(yaw_truth, pos_t)))
    speed_m_s = np.interp(
        state_t,
        pos_t,
        np.hypot(pos_df["vn"].to_numpy(dtype=float), pos_df["ve"].to_numpy(dtype=float)),
    )
    motion_label = classify_motion(speed_m_s, yaw_rate_deg_s)

    overall_state = summarize_series(state_deg)
    overall_total = summarize_series(total_deg)
    after_200_mask = (state_t - state_t[0]) >= 200.0
    if not np.any(after_200_mask):
        after_200_mask = np.ones_like(state_t, dtype=bool)
    after_200_state = summarize_series(state_deg[after_200_mask])
    after_200_total = summarize_series(total_deg[after_200_mask])

    def motion_mean(values: np.ndarray, label: str) -> float:
        mask = motion_label == label
        if not np.any(mask):
            return float("nan")
        return float(np.mean(values[mask]))

    if diag_col in diag_df.columns:
        diag_std_deg = diag_df[diag_col].to_numpy(dtype=float) * 180.0 / math.pi
        diag_std_median_deg = float(np.median(diag_std_deg))
        diag_std_p95_deg = float(np.percentile(diag_std_deg, 95.0))
        diag_std_final_deg = float(diag_std_deg[-1])
    else:
        diag_std_median_deg = float("nan")
        diag_std_p95_deg = float("nan")
        diag_std_final_deg = float("nan")

    total_truth_deg = float(truth_reference["sources"]["mounting_total_truth"]["value_deg"][axis_name])
    state_truth_deg = float(truth_reference["states"][AXIS_TO_STATE_NAME[axis_name]]["reference_value"])
    steady_center_state_deg = 0.5 * (after_200_state["p05_deg"] + after_200_state["p95_deg"])
    steady_center_total_deg = 0.5 * (after_200_total["p05_deg"] + after_200_total["p95_deg"])

    return {
        "axis_name": axis_name,
        "state_truth_deg": state_truth_deg,
        "state_final_deg": float(state_deg[-1]),
        "state_final_error_deg": float(state_deg[-1] - state_truth_deg),
        "state_steady_center_deg": float(steady_center_state_deg),
        "state_steady_center_error_deg": float(steady_center_state_deg - state_truth_deg),
        "state_overall_mean_deg": overall_state["mean_deg"],
        "state_overall_std_deg": overall_state["std_deg"],
        "state_overall_peak_to_peak_deg": overall_state["peak_to_peak_deg"],
        "state_after_200_p05_deg": after_200_state["p05_deg"],
        "state_after_200_p95_deg": after_200_state["p95_deg"],
        "state_after_200_peak_to_peak_deg": after_200_state["peak_to_peak_deg"],
        "target_total_deg": total_truth_deg,
        "final_total_deg": float(total_deg[-1]),
        "final_total_error_deg": float(total_deg[-1] - total_truth_deg),
        "steady_center_deg": float(steady_center_total_deg),
        "steady_center_error_deg": float(steady_center_total_deg - total_truth_deg),
        "overall_mean_deg": overall_total["mean_deg"],
        "overall_std_deg": overall_total["std_deg"],
        "overall_peak_to_peak_deg": overall_total["peak_to_peak_deg"],
        "after_200_p05_deg": after_200_total["p05_deg"],
        "after_200_p95_deg": after_200_total["p95_deg"],
        "after_200_peak_to_peak_deg": after_200_total["peak_to_peak_deg"],
        "straight_mean_deg": motion_mean(total_deg, "straight"),
        "turn_pos_mean_deg": motion_mean(total_deg, "turn_pos"),
        "turn_neg_mean_deg": motion_mean(total_deg, "turn_neg"),
        "turn_neg_minus_turn_pos_deg": motion_mean(total_deg, "turn_neg") - motion_mean(total_deg, "turn_pos"),
        "diag_std_median_deg": diag_std_median_deg,
        "diag_std_p95_deg": diag_std_p95_deg,
        "diag_std_final_deg": diag_std_final_deg,
    }


def build_case_metrics_row(case_row: dict[str, Any], axis_metrics_rows: list[dict[str, Any]]) -> dict[str, Any]:
    metrics = {key: value for key, value in case_row.items() if key != "segment_rows"}
    for axis_row in axis_metrics_rows:
        axis_name = str(axis_row["axis_name"])
        metrics[f"{axis_name}_steady_center_deg"] = float(axis_row["steady_center_deg"])
        metrics[f"{axis_name}_steady_center_error_deg"] = float(axis_row["steady_center_error_deg"])
        metrics[f"{axis_name}_final_total_deg"] = float(axis_row["final_total_deg"])
        metrics[f"{axis_name}_final_total_error_deg"] = float(axis_row["final_total_error_deg"])
        metrics[f"{axis_name}_after_200_peak_to_peak_deg"] = float(axis_row["after_200_peak_to_peak_deg"])
        metrics[f"{axis_name}_overall_std_deg"] = float(axis_row["overall_std_deg"])
        metrics[f"{axis_name}_diag_std_final_deg"] = float(axis_row["diag_std_final_deg"])
    return metrics


def plot_mounting_total_timeseries(state_df: pd.DataFrame, truth_reference: dict[str, Any], output_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for idx, axis_name in enumerate(AXES):
        total_col = AXIS_TO_TOTAL_COL[axis_name]
        truth_deg = float(truth_reference["sources"]["mounting_total_truth"]["value_deg"][axis_name])
        axes[idx].plot(state_df["timestamp"], state_df[total_col], linewidth=1.2, label=f"total_{axis_name}")
        axes[idx].axhline(truth_deg, linestyle="--", color="black", linewidth=1.0, label="truth")
        axes[idx].set_ylabel(f"{axis_name} [deg]")
        axes[idx].set_title(f"total mounting {axis_name}")
        axes[idx].grid(alpha=0.25)
        axes[idx].legend(loc="best")
    axes[-1].set_xlabel("timestamp [s]")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_mounting_state_timeseries(state_df: pd.DataFrame, truth_reference: dict[str, Any], output_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for idx, axis_name in enumerate(AXES):
        state_col = AXIS_TO_STATE_COL[axis_name]
        truth_deg = float(truth_reference["states"][AXIS_TO_STATE_NAME[axis_name]]["reference_value"])
        axes[idx].plot(state_df["timestamp"], state_df[state_col], linewidth=1.2, label=f"state_{axis_name}")
        axes[idx].axhline(truth_deg, linestyle="--", color="black", linewidth=1.0, label="truth")
        axes[idx].set_ylabel(f"{axis_name} [deg]")
        axes[idx].set_title(f"mounting state {axis_name}")
        axes[idx].grid(alpha=0.25)
        axes[idx].legend(loc="best")
    axes[-1].set_xlabel("timestamp [s]")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_dir: Path,
    *,
    exp_id: str,
    base_config_path: Path,
    exe_path: Path,
    truth_reference: dict[str, Any],
    case_metrics_row: dict[str, Any],
    axis_metrics_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    lines = [
        "# mounting pitch/yaw full-GNSS zero-init run",
        "",
        f"Date: `{dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{exp_id}`",
        f"- base config: `{rel_from_root(base_config_path, REPO_ROOT)}`",
        f"- solver: `{rel_from_root(exe_path, REPO_ROOT)}`",
        f"- semantics: `INS/GNSS/NHC/ODO`, `use_truth_pva=true`, `runtime_truth_anchor_pva=false`, `gnss_schedule.enabled=false`",
        (
            f"- fixed blocks: `odo_scale(21)=1.0`, `mounting_roll(22)=0`, "
            f"`odo_lever(25:27)={truth_reference['sources']['odo_lever_truth']['value_m']}`, "
            f"`gnss_lever(28:30)={truth_reference['sources']['gnss_lever_truth']['value_m']}`"
        ),
        (
            f"- released blocks: `mounting_pitch(23)` and `mounting_yaw(24)` with "
            f"`constraints.imu_mounting_angle=[0,0,0]`, `mounting_pitch0=0`, `mounting_yaw0=0`, "
            f"`std_mounting_pitch=std_mounting_yaw={STD_MOUNTING_DEG:.1f} deg`, "
            f"`sigma_mounting_pitch=sigma_mounting_yaw={SIGMA_MOUNTING_RAD:.1e}`"
        ),
        "",
        "## Navigation",
        "",
        (
            f"- `overall_rmse_3d_m_aux={format_metric(float(case_metrics_row['overall_rmse_3d_m_aux']))} m`, "
            f"`overall_final_err_3d_m_aux={format_metric(float(case_metrics_row['overall_final_err_3d_m_aux']))} m`, "
            f"`mean_outage_rmse_3d_m={format_metric(float(case_metrics_row['mean_outage_rmse_3d_m']))} m`, "
            f"`max_outage_final_err_3d_m={format_metric(float(case_metrics_row['max_outage_final_err_3d_m']))} m`."
        ),
        (
            f"- consistency: `ODO accept_ratio={format_metric(float(case_metrics_row['odo_accept_ratio']))}`, "
            f"`ODO nis_mean={format_metric(float(case_metrics_row['odo_nis_mean']))}`, "
            f"`NHC accept_ratio={format_metric(float(case_metrics_row['nhc_accept_ratio']))}`, "
            f"`NHC nis_mean={format_metric(float(case_metrics_row['nhc_nis_mean']))}`."
        ),
        "",
        "## Axis Metrics",
        "",
    ]
    axis_columns = [
        "axis_name",
        "state_truth_deg",
        "state_final_deg",
        "target_total_deg",
        "steady_center_deg",
        "final_total_deg",
        "after_200_peak_to_peak_deg",
        "overall_std_deg",
        "diag_std_final_deg",
    ]
    lines.extend(render_table(axis_metrics_df[axis_columns], axis_columns))
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            (
                f"- 这轮是“总安装角从 0 起步”的口径，因此 state truth 直接等于 total truth: "
                f"`pitch={TARGET_TOTAL_MOUNTING_DEG['pitch']:.2f} deg`, `yaw={TARGET_TOTAL_MOUNTING_DEG['yaw']:.2f} deg`."
            ),
            "- `mean_outage_rmse_3d_m` 和 `max_outage_final_err_3d_m` 在 full-GNSS 口径下可能为 `NA`，此时以 overall auxiliary 导航指标为主。",
            "",
            "## Freshness",
            "",
            f"- base config mtime: `{mtime_text(base_config_path)}`",
            f"- solver mtime: `{mtime_text(exe_path)}`",
            f"- metrics csv: `{manifest['metrics_csv']}`",
            f"- axis metrics csv: `{manifest['axis_metrics_csv']}`",
            f"- summary md: `{manifest['summary_md']}`",
            f"- plots dir: `{manifest['plots_dir']}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run data2 INS/GNSS/ODO/NHC mounting pitch+yaw study under full GNSS with zero total-angle init."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.pos = (REPO_ROOT / args.pos).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.case_root = args.output_dir / "cases"
    args.case_dir = args.case_root / CASE_ID
    args.plot_dir = args.output_dir / "plots"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")
    if not args.pos.exists():
        raise FileNotFoundError(f"missing POS file: {args.pos}")

    reset_directory(args.output_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.case_dir)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference_for_this_run(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    cfg = build_case_config(base_cfg, truth_reference, args.case_dir)
    cfg_path = args.case_dir / f"config_{CASE_ID}.yaml"
    save_yaml(cfg, cfg_path)

    case_row = run_case(args.case_dir, cfg_path, args.exe)
    state_df = pd.read_csv(
        (REPO_ROOT / case_row["state_series_path"]).resolve(),
        usecols=["timestamp", *AXIS_TO_STATE_COL.values(), *AXIS_TO_TOTAL_COL.values()],
    )
    diag_df = pd.read_csv((REPO_ROOT / case_row["diag_path"]).resolve(), sep=r"\s+")
    pos_df = load_pos_motion(args.pos)

    axis_metrics_rows = [
        compute_axis_metrics(
            axis_name=axis_name,
            state_df=state_df,
            diag_df=diag_df,
            pos_df=pos_df,
            truth_reference=truth_reference,
        )
        for axis_name in AXES
    ]
    axis_metrics_df = pd.DataFrame(axis_metrics_rows).sort_values(by="axis_name").reset_index(drop=True)

    case_metrics_row = build_case_metrics_row(case_row, axis_metrics_rows)
    metrics_df = pd.DataFrame([case_metrics_row])
    metrics_path = args.output_dir / "metrics.csv"
    metrics_df.to_csv(metrics_path, index=False, encoding="utf-8-sig")

    axis_metrics_path = args.output_dir / "axis_metrics.csv"
    axis_metrics_df.to_csv(axis_metrics_path, index=False, encoding="utf-8-sig")

    total_plot_path = args.plot_dir / "mounting_total_timeseries.png"
    state_plot_path = args.plot_dir / "mounting_state_timeseries.png"
    plot_mounting_total_timeseries(state_df, truth_reference, total_plot_path)
    plot_mounting_state_timeseries(state_df, truth_reference, state_plot_path)

    metrics_json_path = args.output_dir / "metrics.json"
    metrics_json_path.write_text(
        json.dumps(json_safe(case_metrics_row), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    summary_path = args.output_dir / "summary.md"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "pos_path": rel_from_root(args.pos, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "case_dir": rel_from_root(args.case_dir, REPO_ROOT),
        "case_config": rel_from_root(cfg_path, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "metrics_csv": rel_from_root(metrics_path, REPO_ROOT),
        "axis_metrics_csv": rel_from_root(axis_metrics_path, REPO_ROOT),
        "metrics_json": rel_from_root(metrics_json_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "plots": {
            "mounting_total_timeseries": rel_from_root(total_plot_path, REPO_ROOT),
            "mounting_state_timeseries": rel_from_root(state_plot_path, REPO_ROOT),
        },
        "assumptions": [
            "Full GNSS means fusion.gnss_schedule.enabled=false and GNSS position updates are provided by dataset/data2/rtk.txt over the whole run.",
            "Non-anchor semantics follow use_truth_pva=true and runtime_truth_anchor_pva=false.",
            "odo_scale, mounting_roll, odo_lever, and gnss_lever are hard-frozen through ablation flags.",
            "constraints.imu_mounting_angle is forced to [0, 0, 0], so mounting state equals total mounting angle in this run.",
        ],
        "freshness": {
            "truth_reference_json": mtime_text(truth_reference_path),
            "config_yaml": mtime_text(cfg_path),
            "metrics_csv": mtime_text(metrics_path),
            "axis_metrics_csv": mtime_text(axis_metrics_path),
            "summary_md": None,
        },
    }

    write_summary(
        args.output_dir,
        exp_id=args.exp_id,
        base_config_path=args.base_config,
        exe_path=args.exe,
        truth_reference=truth_reference,
        case_metrics_row=case_metrics_row,
        axis_metrics_df=axis_metrics_df,
        manifest=manifest,
    )
    manifest["freshness"]["summary_md"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
