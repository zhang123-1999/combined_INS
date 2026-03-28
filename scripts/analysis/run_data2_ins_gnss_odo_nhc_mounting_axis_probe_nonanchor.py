from __future__ import annotations

import argparse
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

from scripts.analysis.filter_gnss_outage import filter_gnss
from scripts.analysis.odo_nhc_update_sweep import (
    ensure_dir,
    load_yaml,
    parse_consistency_summary,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_case_config as build_state_sanity_case_config,
    build_truth_reference,
    evaluate_navigation_metrics,
    evaluate_state_case,
    json_safe,
    plot_heatmap,
    reset_directory,
)


CONTROL_CASE_ID = "control_all_mounting_fixed"
CASE_IDS = [CONTROL_CASE_ID, "release_mounting_pitch", "release_mounting_yaw"]
AXES = ("pitch", "yaw")
AXIS_TO_STATE_NAME = {"pitch": "mounting_pitch", "yaw": "mounting_yaw"}
AXIS_TO_STATE_COL = {"pitch": "mounting_pitch_deg", "yaw": "mounting_yaw_deg"}
AXIS_TO_TOTAL_COL = {"pitch": "total_mounting_pitch_deg", "yaw": "total_mounting_yaw_deg"}
AXIS_TO_DIAG_COL = {"pitch": "std_mp", "yaw": "std_my"}
TARGET_TOTAL_MOUNTING_DEG = {"pitch": 0.36, "yaw": 0.84}
STD_MOUNTING_DEG = 3.0
SIGMA_MOUNTING_RAD = 0.0
P0_INDEX = {"roll": 22, "pitch": 23, "yaw": 24}

EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-axis-nonanchor-probe-r1"
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_axis_nonanchor_probe_20260322")


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


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
            elif isinstance(value, (int, np.integer)):
                vals.append(str(int(value)))
            else:
                vals.append(f"{float(value):.6f}")
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


def case_release_state_name(case_id: str) -> str | None:
    if case_id == CONTROL_CASE_ID:
        return None
    if not case_id.startswith("release_"):
        raise ValueError(f"unexpected case_id: {case_id}")
    return case_id[len("release_") :]


def case_sort_key(case_id: str) -> tuple[int, int]:
    if case_id == CONTROL_CASE_ID:
        return (0, -1)
    return (1, CASE_IDS.index(case_id))


def build_nonanchor_truth_reference(base_cfg: dict[str, Any]) -> dict[str, Any]:
    truth_reference = build_truth_reference(base_cfg)
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["pitch"] = float(TARGET_TOTAL_MOUNTING_DEG["pitch"])
    truth_reference["sources"]["mounting_total_truth"]["value_deg"]["yaw"] = float(TARGET_TOTAL_MOUNTING_DEG["yaw"])
    truth_reference["sources"]["mounting_total_truth"]["source"] = "non-anchor working nominal used in post-fix ODO/NHC mounting study"
    truth_reference["sources"]["constraints_mounting_base_deg"] = [
        0.0,
        float(TARGET_TOTAL_MOUNTING_DEG["pitch"]),
        float(TARGET_TOTAL_MOUNTING_DEG["yaw"]),
    ]
    for axis in AXES:
        state_name = AXIS_TO_STATE_NAME[axis]
        truth_reference["states"][state_name]["reference_value"] = 0.0
        truth_reference["states"][state_name]["reference_value_internal"] = 0.0
        truth_reference["states"][state_name]["source"] = (
            "non-anchor working nominal total mounting fixed equal to constraints base"
        )
    truth_reference["states"]["mounting_roll"]["reference_value"] = 0.0
    truth_reference["states"]["mounting_roll"]["reference_value_internal"] = 0.0
    truth_reference["states"]["mounting_roll"]["source"] = (
        "non-anchor working nominal total mounting fixed equal to constraints base"
    )
    return truth_reference


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
    outage_gnss_path: Path,
) -> dict[str, Any]:
    internal_case_id = "truth_anchor_all_non_pva" if case_id == CONTROL_CASE_ID else case_id
    cfg = build_state_sanity_case_config(
        base_cfg=base_cfg,
        truth_reference=truth_reference,
        case_id=internal_case_id,
        case_dir=case_dir,
        outage_gnss_path=outage_gnss_path,
    )
    fusion = cfg["fusion"]
    ablation_cfg = fusion.setdefault("ablation", {})
    post_ablation_cfg = fusion.setdefault("post_gnss_ablation", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    p0_diag = [float(x) for x in init_cfg["P0_diag"]]

    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["enable_gnss_velocity"] = False
    fusion["gnss_path"] = rel_from_root(outage_gnss_path, REPO_ROOT)

    constraints_mount = list(map(float, constraints_cfg.get("imu_mounting_angle", [0.0, 0.0, 0.0])))
    constraints_mount[1] = float(TARGET_TOTAL_MOUNTING_DEG["pitch"])
    constraints_mount[2] = float(TARGET_TOTAL_MOUNTING_DEG["yaw"])
    constraints_cfg["imu_mounting_angle"] = constraints_mount
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = False
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False
    init_cfg["mounting_roll0"] = 0.0
    init_cfg["mounting_pitch0"] = 0.0
    init_cfg["mounting_yaw0"] = 0.0
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
    ablation_cfg["disable_mounting_roll"] = True
    ablation_cfg["disable_mounting_pitch"] = True
    ablation_cfg["disable_mounting_yaw"] = True
    post_ablation_cfg["enabled"] = False
    post_ablation_cfg["disable_gnss_lever_arm"] = False
    post_ablation_cfg["disable_gnss_lever_z"] = False
    post_ablation_cfg["disable_odo_lever_arm"] = False
    post_ablation_cfg["disable_odo_scale"] = False
    post_ablation_cfg["disable_gyro_bias"] = False
    post_ablation_cfg["disable_gyro_scale"] = False
    post_ablation_cfg["disable_accel_scale"] = False
    post_ablation_cfg["disable_mounting"] = False
    post_ablation_cfg["disable_mounting_roll"] = False
    post_ablation_cfg["disable_mounting_pitch"] = False
    post_ablation_cfg["disable_mounting_yaw"] = False

    if case_id == CONTROL_CASE_ID:
        ablation_cfg["disable_mounting"] = True
        return cfg

    ablation_cfg["disable_mounting"] = False
    release_state = case_release_state_name(case_id)
    if release_state == "mounting_pitch":
        ablation_cfg["disable_mounting_pitch"] = False
    elif release_state == "mounting_yaw":
        ablation_cfg["disable_mounting_yaw"] = False
    else:
        raise ValueError(f"unsupported release_state for case_id={case_id}")
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
        raise RuntimeError(f"solver failed for {cfg_path.name}: returncode={proc.returncode}")
    return merged


def run_case(case_id: str, cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    stdout_path = case_dir / f"solver_stdout_{case_id}.txt"
    diag_path = case_dir / f"DIAG_{case_id}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_solver(exe_path, cfg_path)
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
        "sol_mtime": mtime_text(sol_path),
        "state_series_mtime": mtime_text(state_series_path),
        "diag_mtime": mtime_text(diag_path),
        "stdout_mtime": mtime_text(stdout_path),
        "segment_rows": segment_rows,
    }
    row.update(nav_metrics)
    return row


def enrich_case_with_consistency(case_row: dict[str, Any]) -> dict[str, Any]:
    stdout_path = (REPO_ROOT / case_row["stdout_path"]).resolve()
    stdout_text = stdout_path.read_text(encoding="utf-8", errors="ignore")
    consistency = parse_consistency_summary(stdout_text)
    enriched = dict(case_row)
    enriched["odo_accept_ratio"] = float(consistency.get("ODO", {}).get("accept_ratio", float("nan")))
    enriched["odo_nis_mean"] = float(consistency.get("ODO", {}).get("nis_mean", float("nan")))
    enriched["nhc_accept_ratio"] = float(consistency.get("NHC", {}).get("accept_ratio", float("nan")))
    enriched["nhc_nis_mean"] = float(consistency.get("NHC", {}).get("nis_mean", float("nan")))
    return enriched


def compute_axis_metrics(
    *,
    case_id: str,
    axis_name: str,
    case_row: dict[str, Any],
    truth_reference: dict[str, Any],
    pos_df: pd.DataFrame,
) -> dict[str, Any]:
    state_series_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    diag_path = (REPO_ROOT / case_row["diag_path"]).resolve()
    state_df = pd.read_csv(
        state_series_path,
        usecols=["timestamp", AXIS_TO_STATE_COL[axis_name], AXIS_TO_TOTAL_COL[axis_name]],
    )
    diag_df = pd.read_csv(diag_path, sep=r"\s+")

    state_t = state_df["timestamp"].to_numpy(dtype=float)
    state_deg = state_df[AXIS_TO_STATE_COL[axis_name]].to_numpy(dtype=float)
    total_deg = state_df[AXIS_TO_TOTAL_COL[axis_name]].to_numpy(dtype=float)

    pos_t = pos_df["t"].to_numpy(dtype=float)
    yaw_truth = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.interp(state_t, pos_t, np.rad2deg(np.gradient(yaw_truth, pos_t)))
    speed_m_s = np.interp(
        state_t,
        pos_t,
        np.hypot(pos_df["vn"].to_numpy(dtype=float), pos_df["ve"].to_numpy(dtype=float)),
    )
    motion_label = classify_motion(speed_m_s, yaw_rate_deg_s)

    overall = summarize_series(total_deg)
    after_200_mask = (state_t - state_t[0]) >= 200.0
    if not np.any(after_200_mask):
        after_200_mask = np.ones_like(state_t, dtype=bool)
    after_200 = summarize_series(total_deg[after_200_mask])

    def motion_mean(label: str) -> float:
        mask = motion_label == label
        if not np.any(mask):
            return float("nan")
        return float(np.mean(total_deg[mask]))

    diag_col = AXIS_TO_DIAG_COL[axis_name]
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
    steady_center_deg = 0.5 * (after_200["p05_deg"] + after_200["p95_deg"])
    return {
        "case_id": case_id,
        "axis_name": axis_name,
        "state_truth_deg": float(truth_reference["states"][AXIS_TO_STATE_NAME[axis_name]]["reference_value"]),
        "state_final_deg": float(state_deg[-1]),
        "target_total_deg": total_truth_deg,
        "final_total_deg": float(total_deg[-1]),
        "final_total_error_deg": float(total_deg[-1] - total_truth_deg),
        "steady_center_deg": float(steady_center_deg),
        "steady_center_error_deg": float(steady_center_deg - total_truth_deg),
        "overall_mean_deg": overall["mean_deg"],
        "overall_std_deg": overall["std_deg"],
        "overall_peak_to_peak_deg": overall["peak_to_peak_deg"],
        "after_200_p05_deg": after_200["p05_deg"],
        "after_200_p95_deg": after_200["p95_deg"],
        "after_200_peak_to_peak_deg": after_200["peak_to_peak_deg"],
        "turn_pos_mean_deg": motion_mean("turn_pos"),
        "turn_neg_mean_deg": motion_mean("turn_neg"),
        "turn_neg_minus_turn_pos_deg": motion_mean("turn_neg") - motion_mean("turn_pos"),
        "diag_std_median_deg": diag_std_median_deg,
        "diag_std_p95_deg": diag_std_p95_deg,
        "diag_std_final_deg": diag_std_final_deg,
    }


def build_case_metrics_table(case_rows: list[dict[str, Any]]) -> pd.DataFrame:
    records: list[dict[str, Any]] = []
    for row in sorted(case_rows, key=lambda item: case_sort_key(item["case_id"])):
        record = {key: value for key, value in row.items() if key != "segment_rows"}
        records.append(record)
    return pd.DataFrame(records)


def write_case_segments(case_dir: Path, case_id: str, segment_rows: list[dict[str, float]]) -> Path:
    segment_path = case_dir / f"outage_segments_{case_id}.csv"
    pd.DataFrame(segment_rows).to_csv(segment_path, index=False, encoding="utf-8-sig")
    return segment_path


def evaluate_mounting_axis_case(
    state_name: str,
    case_row: dict[str, Any],
    control_row: dict[str, Any],
    truth_reference: dict[str, Any],
    axis_metrics_map: dict[tuple[str, str], dict[str, Any]],
) -> dict[str, Any]:
    row = evaluate_state_case(state_name, case_row, control_row, truth_reference)
    axis_name = state_name.split("_", 1)[1]
    axis_metrics = axis_metrics_map[(case_row["case_id"], axis_name)]
    row["total_truth_deg"] = float(axis_metrics["target_total_deg"])
    row["total_final_deg"] = float(axis_metrics["final_total_deg"])
    row["total_range_deg"] = float(axis_metrics["overall_peak_to_peak_deg"])
    row["steady_center_deg"] = float(axis_metrics["steady_center_deg"])
    row["steady_center_error_deg"] = float(axis_metrics["steady_center_error_deg"])
    row["after_200_peak_to_peak_deg"] = float(axis_metrics["after_200_peak_to_peak_deg"])
    row["overall_std_total_deg"] = float(axis_metrics["overall_std_deg"])
    row["turn_neg_minus_turn_pos_deg"] = float(axis_metrics["turn_neg_minus_turn_pos_deg"])
    row["odo_accept_ratio"] = float(case_row.get("odo_accept_ratio", float("nan")))
    row["nhc_accept_ratio"] = float(case_row.get("nhc_accept_ratio", float("nan")))
    row["odo_nis_mean"] = float(case_row.get("odo_nis_mean", float("nan")))
    row["nhc_nis_mean"] = float(case_row.get("nhc_nis_mean", float("nan")))
    return row


def plot_mounting_axis_comparison(
    axis_name: str,
    control_row: dict[str, Any],
    case_row: dict[str, Any],
    truth_reference: dict[str, Any],
    output_path: Path,
) -> None:
    control_df = pd.read_csv(
        (REPO_ROOT / control_row["state_series_path"]).resolve(),
        usecols=["timestamp", AXIS_TO_STATE_COL[axis_name], AXIS_TO_TOTAL_COL[axis_name]],
    )
    case_df = pd.read_csv(
        (REPO_ROOT / case_row["state_series_path"]).resolve(),
        usecols=["timestamp", AXIS_TO_STATE_COL[axis_name], AXIS_TO_TOTAL_COL[axis_name]],
    )

    state_col = AXIS_TO_STATE_COL[axis_name]
    total_col = AXIS_TO_TOTAL_COL[axis_name]
    state_truth = float(truth_reference["states"][AXIS_TO_STATE_NAME[axis_name]]["reference_value"])
    total_truth = float(truth_reference["sources"]["mounting_total_truth"]["value_deg"][axis_name])

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(control_df["timestamp"], control_df[state_col], linewidth=1.2, label="control")
    axes[0].plot(case_df["timestamp"], case_df[state_col], linewidth=1.2, label=f"release_{axis_name}")
    axes[0].axhline(state_truth, linestyle="--", color="black", linewidth=1.0, label="state truth")
    axes[0].set_ylabel("state offset [deg]")
    axes[0].set_title(f"mounting_{axis_name} state trajectory")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="best")

    axes[1].plot(control_df["timestamp"], control_df[total_col], linewidth=1.2, label="control")
    axes[1].plot(case_df["timestamp"], case_df[total_col], linewidth=1.2, label=f"release_{axis_name}")
    axes[1].axhline(total_truth, linestyle="--", color="black", linewidth=1.0, label="total truth")
    axes[1].set_xlabel("timestamp [s]")
    axes[1].set_ylabel("total angle [deg]")
    axes[1].set_title(f"total mounting_{axis_name} trajectory")
    axes[1].grid(alpha=0.25)
    axes[1].legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_dir: Path,
    *,
    exp_id: str,
    base_config_path: Path,
    exe_path: Path,
    metrics_df: pd.DataFrame,
    judgement_df: pd.DataFrame,
    axis_metrics_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    control_row = metrics_df.loc[metrics_df["case_id"] == CONTROL_CASE_ID].iloc[0]
    control_pitch = axis_metrics_df.loc[
        (axis_metrics_df["case_id"] == CONTROL_CASE_ID) & (axis_metrics_df["axis_name"] == "pitch")
    ].iloc[0]
    control_yaw = axis_metrics_df.loc[
        (axis_metrics_df["case_id"] == CONTROL_CASE_ID) & (axis_metrics_df["axis_name"] == "yaw")
    ].iloc[0]

    lines = [
        "# non-anchor mounting-axis probe",
        "",
        f"Date: `{dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{exp_id}`",
        f"- base config: `{rel_from_root(base_config_path, REPO_ROOT)}`",
        f"- solver: `{rel_from_root(exe_path, REPO_ROOT)}`",
        f"- non-anchor semantics: `use_truth_pva=true`, `runtime_truth_anchor_pva=false`",
        f"- total mounting nominal/truth: `pitch={TARGET_TOTAL_MOUNTING_DEG['pitch']:.2f} deg`, `yaw={TARGET_TOTAL_MOUNTING_DEG['yaw']:.2f} deg`",
        f"- shared mounting prior/noise: `std_pitch=std_yaw={STD_MOUNTING_DEG:.1f} deg`, `sigma_pitch=sigma_yaw={SIGMA_MOUNTING_RAD:.1f}`",
        "",
        "## Control",
        "",
        (
            f"- `{CONTROL_CASE_ID}` navigation: "
            f"`mean_outage_rmse_3d={control_row['mean_outage_rmse_3d_m']:.6f} m`, "
            f"`max_outage_final_err_3d={control_row['max_outage_final_err_3d_m']:.6f} m`, "
            f"`overall_rmse_3d_aux={control_row['overall_rmse_3d_m_aux']:.6f} m`, "
            f"`ODO/NHC accept={control_row['odo_accept_ratio']:.6f}/{control_row['nhc_accept_ratio']:.6f}`."
        ),
        (
            f"- control total pitch/yaw: "
            f"`pitch steady/final={control_pitch['steady_center_deg']:.6f}/{control_pitch['final_total_deg']:.6f} deg`, "
            f"`yaw steady/final={control_yaw['steady_center_deg']:.6f}/{control_yaw['final_total_deg']:.6f} deg`."
        ),
        "",
        "## Case Metrics",
        "",
    ]
    lines.extend(
        render_table(
            metrics_df[
                [
                    "case_id",
                    "mean_outage_rmse_3d_m",
                    "max_outage_final_err_3d_m",
                    "overall_rmse_3d_m_aux",
                    "overall_final_err_3d_m_aux",
                    "odo_accept_ratio",
                    "nhc_accept_ratio",
                    "pitch_steady_center_deg",
                    "pitch_after_200_peak_to_peak_deg",
                    "pitch_final_total_deg",
                    "yaw_steady_center_deg",
                    "yaw_after_200_peak_to_peak_deg",
                    "yaw_final_total_deg",
                ]
            ],
            [
                "case_id",
                "mean_outage_rmse_3d_m",
                "max_outage_final_err_3d_m",
                "overall_rmse_3d_m_aux",
                "overall_final_err_3d_m_aux",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "pitch_steady_center_deg",
                "pitch_after_200_peak_to_peak_deg",
                "pitch_final_total_deg",
                "yaw_steady_center_deg",
                "yaw_after_200_peak_to_peak_deg",
                "yaw_final_total_deg",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Release Judgement",
            "",
        ]
    )
    for _, row in judgement_df.iterrows():
        lines.append(
            (
                f"- `{row['state_name']}`: overall=`{row['overall_label']}`, "
                f"behavior=`{row['behavior_label']}`, impact=`{row['impact_label']}`, "
                f"state final/truth=`{float(row['final_value']):.6f}/{float(row['truth_value']):.6f} deg`, "
                f"total steady/final/truth=`{float(row['steady_center_deg']):.6f}/{float(row['total_final_deg']):.6f}/{float(row['total_truth_deg']):.6f} deg`, "
                f"`after_200_peak_to_peak={float(row['after_200_peak_to_peak_deg']):.6f} deg`, "
                f"`turn_neg_minus_turn_pos={float(row['turn_neg_minus_turn_pos_deg']):.6f} deg`, "
                f"`delta_mean_rmse3d={float(row['delta_mean_rmse3d']):.6f}`, "
                f"`delta_max_final3d={float(row['delta_max_final3d']):.6f}`, "
                f"`ODO/NHC accept={float(row['odo_accept_ratio']):.6f}/{float(row['nhc_accept_ratio']):.6f}`."
            )
        )
    lines.extend(
        [
            "",
            "## Interpretation Guardrails",
            "",
            "- 先看 control 是否已失稳，再解释 pitch/yaw 单轴释放结果；这轮结果不再沿用 runtime-anchor screen 的排序口径。",
            "- `ODO/NHC accept_ratio` 只是 consistency 辅助证据，不能单独说明安装角估计正确。",
            "- 本轮已使用代码级 `disable_mounting_pitch/yaw`，不再依赖 tiny `P0/Q` 近似冻结非目标轴。",
            "",
            "## Freshness",
            "",
            f"- base config mtime: `{mtime_text(base_config_path)}`",
            f"- solver mtime: `{mtime_text(exe_path)}`",
            f"- metrics csv: `{manifest['metrics_csv']}`",
            f"- axis metrics csv: `{manifest['axis_metrics_csv']}`",
            f"- state judgement csv: `{manifest['state_judgement_csv']}`",
            f"- truth reference json: `{manifest['truth_reference_json']}`",
            f"- plots dir: `{manifest['plots_dir']}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run data2 INS/GNSS/ODO/NHC mounting pitch/yaw probe under non-anchor semantics."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--initial-on", type=float, default=300.0)
    parser.add_argument("--off", type=float, default=100.0)
    parser.add_argument("--on", type=float, default=150.0)
    parser.add_argument("--cycle-start", choices=("off", "on"), default="off")
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.pos = (REPO_ROOT / args.pos).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.case_root = args.output_dir / "cases"
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
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_nonanchor_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    outage_gnss_path = args.output_dir / "GNSS_outage_cycle_rtk_300on_100off_150on.txt"
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
    gnss_stats_path = args.output_dir / "gnss_outage_stats.json"
    gnss_stats_path.write_text(json.dumps(json_safe(filter_stats), indent=2, ensure_ascii=False), encoding="utf-8")

    pos_df = load_pos_motion(args.pos)

    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_segment_paths: dict[str, str] = {}
    for case_id in CASE_IDS:
        case_dir = args.case_root / case_id
        ensure_dir(case_dir)
        cfg = build_case_config(base_cfg, truth_reference, case_id, case_dir, outage_gnss_path)
        cfg_path = case_dir / f"config_{case_id}.yaml"
        save_yaml(cfg, cfg_path)
        case_config_paths[case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_row = run_case(case_id=case_id, cfg_path=cfg_path, case_dir=case_dir, exe_path=args.exe)
        case_row = enrich_case_with_consistency(case_row)
        segment_path = write_case_segments(case_dir, case_id, case_row["segment_rows"])
        case_row["outage_segments_path"] = rel_from_root(segment_path, REPO_ROOT)
        case_segment_paths[case_id] = case_row["outage_segments_path"]
        case_rows.append(case_row)

    case_rows = sorted(case_rows, key=lambda item: case_sort_key(item["case_id"]))
    control_row = next(row for row in case_rows if row["case_id"] == CONTROL_CASE_ID)

    axis_metrics_rows: list[dict[str, Any]] = []
    axis_metrics_map: dict[tuple[str, str], dict[str, Any]] = {}
    for case_row in case_rows:
        for axis_name in AXES:
            axis_metrics = compute_axis_metrics(
                case_id=case_row["case_id"],
                axis_name=axis_name,
                case_row=case_row,
                truth_reference=truth_reference,
                pos_df=pos_df,
            )
            axis_metrics_rows.append(axis_metrics)
            axis_metrics_map[(case_row["case_id"], axis_name)] = axis_metrics
            for key, value in axis_metrics.items():
                if key in {"case_id", "axis_name"}:
                    continue
                case_row[f"{axis_name}_{key}"] = value

    judgement_rows: list[dict[str, Any]] = []
    for case_row in case_rows:
        if case_row["case_id"] == CONTROL_CASE_ID:
            continue
        state_name = case_release_state_name(case_row["case_id"])
        judgement_rows.append(
            evaluate_mounting_axis_case(
                state_name=state_name,
                case_row=case_row,
                control_row=control_row,
                truth_reference=truth_reference,
                axis_metrics_map=axis_metrics_map,
            )
        )
        axis_name = state_name.split("_", 1)[1]
        plot_mounting_axis_comparison(
            axis_name=axis_name,
            control_row=control_row,
            case_row=case_row,
            truth_reference=truth_reference,
            output_path=args.plot_dir / f"mounting_{axis_name}_comparison.png",
        )

    metrics_df = build_case_metrics_table(case_rows)
    metrics_path = args.output_dir / "metrics.csv"
    metrics_df.to_csv(metrics_path, index=False, encoding="utf-8-sig")

    axis_metrics_df = pd.DataFrame(axis_metrics_rows).sort_values(by=["axis_name", "case_id"]).reset_index(drop=True)
    axis_metrics_path = args.output_dir / "axis_metrics.csv"
    axis_metrics_df.to_csv(axis_metrics_path, index=False, encoding="utf-8-sig")

    judgement_df = pd.DataFrame(judgement_rows).sort_values(by="state_name").reset_index(drop=True)
    if not judgement_df.empty:
        plot_heatmap(judgement_df, args.plot_dir / "state_judgement_heatmap.png")
    state_judgement_path = args.output_dir / "state_judgement.csv"
    judgement_df.to_csv(state_judgement_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "pos_path": rel_from_root(args.pos, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "case_root": rel_from_root(args.case_root, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "metrics_csv": rel_from_root(metrics_path, REPO_ROOT),
        "axis_metrics_csv": rel_from_root(axis_metrics_path, REPO_ROOT),
        "state_judgement_csv": rel_from_root(state_judgement_path, REPO_ROOT),
        "gnss_outage_path": rel_from_root(outage_gnss_path, REPO_ROOT),
        "gnss_outage_stats_json": rel_from_root(gnss_stats_path, REPO_ROOT),
        "case_ids": CASE_IDS,
        "case_config_paths": case_config_paths,
        "case_outage_segments_paths": case_segment_paths,
        "outage_schedule": {
            "initial_on_s": args.initial_on,
            "off_s": args.off,
            "on_s": args.on,
            "cycle_start": args.cycle_start,
            "first_off_start_time": filter_stats.get("first_off_start_time"),
            "first_off_end_time": filter_stats.get("first_off_end_time"),
        },
        "assumptions": [
            "use_truth_pva=true and runtime_truth_anchor_pva=false define the non-anchor semantics of this probe.",
            "odo_scale, mounting_roll, odo_lever, and gnss_lever are hard-frozen through ablation flags.",
            "target total mounting nominal/truth is fixed to pitch=0.36 deg and yaw=0.84 deg for this study line.",
            "shared mounting prior uses std=3 deg and sigma=0 for both pitch and yaw.",
        ],
    }

    write_summary(
        args.output_dir,
        exp_id=args.exp_id,
        base_config_path=args.base_config,
        exe_path=args.exe,
        metrics_df=metrics_df,
        judgement_df=judgement_df,
        axis_metrics_df=axis_metrics_df,
        manifest=manifest,
    )
    summary_path = args.output_dir / "summary.md"
    manifest["summary_md"] = rel_from_root(summary_path, REPO_ROOT)
    manifest["freshness"] = {
        "truth_reference_json": mtime_text(truth_reference_path),
        "metrics_csv": mtime_text(metrics_path),
        "axis_metrics_csv": mtime_text(axis_metrics_path),
        "state_judgement_csv": mtime_text(state_judgement_path),
        "gnss_outage_stats_json": mtime_text(gnss_stats_path),
        "summary_md": mtime_text(summary_path),
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
