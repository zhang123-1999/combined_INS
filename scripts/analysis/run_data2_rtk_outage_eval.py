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

import numpy as np
import pandas as pd

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


EXPECTED_PLOT_FILES = [
    "error_position_xyz.png",
    "error_velocity_xyz.png",
    "error_attitude_xyz.png",
    "state_gyro_bias.png",
    "state_accel_bias.png",
    "state_gyro_scale.png",
    "state_accel_scale.png",
    "state_mounting_odo_scale.png",
    "state_odo_lever_arm.png",
    "state_gnss_lever_arm.png",
    "outage_segment_rmse_xyz_bar.png",
    "outage_segment_final_err_xyz_bar.png",
]


def detect_has_header(path: Path) -> bool:
    with path.open("r", encoding="utf-8") as f:
        first = f.readline().strip()
    if not first:
        return False
    token = first.split()[0]
    return not token.lstrip("-").replace(".", "", 1).isdigit()


def load_gnss_timestamps(path: Path) -> np.ndarray:
    has_header = detect_has_header(path)
    df = pd.read_csv(path, sep=r"\s+", header=0 if has_header else None, comment="#", usecols=[0])
    timestamps = df.iloc[:, 0].to_numpy(dtype=float)
    timestamps = timestamps[np.isfinite(timestamps)]
    if timestamps.size == 0:
        return timestamps
    return np.unique(timestamps)


def timestamps_to_windows(timestamps: np.ndarray) -> list[tuple[float, float]]:
    if timestamps.size == 0:
        return []
    if timestamps.size == 1:
        t = float(timestamps[0])
        return [(t, t)]

    dt_arr = np.diff(timestamps)
    positive_dt = dt_arr[dt_arr > 0.0]
    nominal_dt = float(np.median(positive_dt)) if positive_dt.size else 0.0
    gap_threshold = max(nominal_dt * 10.0, nominal_dt + 10.0) if nominal_dt > 0.0 else 10.0
    gap_indices = np.where(dt_arr > gap_threshold)[0]
    start_indices = np.concatenate(([0], gap_indices + 1))
    end_indices = np.concatenate((gap_indices, [timestamps.size - 1]))
    extension = nominal_dt if nominal_dt > 0.0 else 0.0

    windows: list[tuple[float, float]] = []
    for start_idx, end_idx in zip(start_indices, end_indices):
        start_t = float(timestamps[start_idx])
        end_t = float(timestamps[end_idx] + extension)
        windows.append((start_t, max(start_t, end_t)))
    return windows


def merge_windows(windows: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not windows:
        return []
    sorted_windows = sorted(windows)
    merged: list[list[float]] = [[float(sorted_windows[0][0]), float(sorted_windows[0][1])]]
    for start_t, end_t in sorted_windows[1:]:
        start_t = float(start_t)
        end_t = float(end_t)
        if start_t <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], end_t)
        else:
            merged.append([start_t, end_t])
    return [(item[0], item[1]) for item in merged]


def extract_on_windows(config: dict[str, Any]) -> list[tuple[float, float]]:
    fusion = config.get("fusion") or {}
    start_time = float(fusion.get("starttime", 0.0))
    final_time = float(fusion.get("finaltime", start_time))

    schedule_cfg = fusion.get("gnss_schedule") or {}
    if schedule_cfg.get("enabled", False):
        head_ratio = float(schedule_cfg.get("head_ratio", 0.0) or 0.0)
        if head_ratio > 0.0:
            split_t = start_time + head_ratio * (final_time - start_time)
            return [(start_time, split_t)]
        return []

    gnss_rel = fusion.get("gnss_path")
    if not gnss_rel:
        return []
    gnss_path = (REPO_ROOT / str(gnss_rel)).resolve()
    if not gnss_path.exists():
        return []
    return merge_windows(timestamps_to_windows(load_gnss_timestamps(gnss_path)))


def invert_windows(
    on_windows: list[tuple[float, float]], start_time: float, final_time: float
) -> list[tuple[float, float]]:
    if final_time <= start_time:
        return []
    merged = merge_windows(
        [
            (max(start_time, float(window_start)), min(final_time, float(window_end)))
            for window_start, window_end in on_windows
            if float(window_end) > start_time and float(window_start) < final_time
        ]
    )

    off_windows: list[tuple[float, float]] = []
    cursor = start_time
    for window_start, window_end in merged:
        if window_start > cursor:
            off_windows.append((cursor, window_start))
        cursor = max(cursor, window_end)
    if cursor < final_time:
        off_windows.append((cursor, final_time))
    return off_windows


def compute_global_metrics(err_xyz: np.ndarray) -> dict[str, float]:
    err3 = np.linalg.norm(err_xyz, axis=1)
    rmse_xyz = np.sqrt(np.mean(err_xyz * err_xyz, axis=0))
    return {
        "overall_rmse_x_m_aux": float(rmse_xyz[0]),
        "overall_rmse_y_m_aux": float(rmse_xyz[1]),
        "overall_rmse_z_m_aux": float(rmse_xyz[2]),
        "overall_rmse_3d_m_aux": float(np.sqrt(np.mean(err3 * err3))),
        "overall_p95_3d_m_aux": float(np.percentile(err3, 95)),
        "overall_final_err_3d_m_aux": float(err3[-1]) if err3.size else float("nan"),
    }


def compute_outage_segments(
    sol_t: np.ndarray, err_xyz: np.ndarray, outage_windows: list[tuple[float, float]]
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    if sol_t.size == 0:
        return rows

    for idx, (start_t, end_t) in enumerate(outage_windows, start=1):
        if idx == len(outage_windows):
            mask = (sol_t >= start_t) & (sol_t <= end_t)
        else:
            mask = (sol_t >= start_t) & (sol_t < end_t)
        seg_err = err_xyz[mask]
        if seg_err.size == 0:
            continue
        seg_err3 = np.linalg.norm(seg_err, axis=1)
        rmse_xyz = np.sqrt(np.mean(seg_err * seg_err, axis=0))
        final_abs = np.abs(seg_err[-1])
        rows.append(
            {
                "segment_id": idx,
                "start_time": float(start_t),
                "end_time": float(end_t),
                "duration_s": float(end_t - start_t),
                "samples": int(seg_err.shape[0]),
                "rmse_x_m": float(rmse_xyz[0]),
                "rmse_y_m": float(rmse_xyz[1]),
                "rmse_z_m": float(rmse_xyz[2]),
                "rmse_3d_m": float(np.sqrt(np.mean(seg_err3 * seg_err3))),
                "final_err_x_m": float(final_abs[0]),
                "final_err_y_m": float(final_abs[1]),
                "final_err_z_m": float(final_abs[2]),
                "final_err_3d_m": float(seg_err3[-1]),
            }
        )
    return rows


def aggregate_outage_segments(rows: list[dict[str, float]]) -> dict[str, float]:
    if not rows:
        return {
            "outage_segment_count": 0.0,
            "mean_outage_rmse_x_m": float("nan"),
            "mean_outage_rmse_y_m": float("nan"),
            "mean_outage_rmse_z_m": float("nan"),
            "mean_outage_rmse_3d_m": float("nan"),
            "max_outage_rmse_x_m": float("nan"),
            "max_outage_rmse_y_m": float("nan"),
            "max_outage_rmse_z_m": float("nan"),
            "max_outage_rmse_3d_m": float("nan"),
            "mean_outage_final_err_x_m": float("nan"),
            "mean_outage_final_err_y_m": float("nan"),
            "mean_outage_final_err_z_m": float("nan"),
            "mean_outage_final_err_3d_m": float("nan"),
            "max_outage_final_err_x_m": float("nan"),
            "max_outage_final_err_y_m": float("nan"),
            "max_outage_final_err_z_m": float("nan"),
            "max_outage_final_err_3d_m": float("nan"),
        }

    df = pd.DataFrame(rows)
    return {
        "outage_segment_count": float(len(rows)),
        "mean_outage_rmse_x_m": float(df["rmse_x_m"].mean()),
        "mean_outage_rmse_y_m": float(df["rmse_y_m"].mean()),
        "mean_outage_rmse_z_m": float(df["rmse_z_m"].mean()),
        "mean_outage_rmse_3d_m": float(df["rmse_3d_m"].mean()),
        "max_outage_rmse_x_m": float(df["rmse_x_m"].max()),
        "max_outage_rmse_y_m": float(df["rmse_y_m"].max()),
        "max_outage_rmse_z_m": float(df["rmse_z_m"].max()),
        "max_outage_rmse_3d_m": float(df["rmse_3d_m"].max()),
        "mean_outage_final_err_x_m": float(df["final_err_x_m"].mean()),
        "mean_outage_final_err_y_m": float(df["final_err_y_m"].mean()),
        "mean_outage_final_err_z_m": float(df["final_err_z_m"].mean()),
        "mean_outage_final_err_3d_m": float(df["final_err_3d_m"].mean()),
        "max_outage_final_err_x_m": float(df["final_err_x_m"].max()),
        "max_outage_final_err_y_m": float(df["final_err_y_m"].max()),
        "max_outage_final_err_z_m": float(df["final_err_z_m"].max()),
        "max_outage_final_err_3d_m": float(df["final_err_3d_m"].max()),
    }


def format_metric(value: float) -> str:
    if value is None or not math.isfinite(float(value)):
        return "NA"
    return f"{float(value):.6f}"


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: json_safe(val) for key, val in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def markdown_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    header = "| " + " | ".join(columns) + " |"
    separator = "| " + " | ".join(["---"] * len(columns)) + " |"
    body = ["| " + " | ".join(row) + " |" for row in rows]
    return [header, separator] + body


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


def build_case_configs(args: argparse.Namespace) -> tuple[Path, Path, dict[str, Any]]:
    base_cfg = load_yaml(args.base_config)
    base_fusion = base_cfg.setdefault("fusion", {})
    base_ablation = base_fusion.setdefault("ablation", {})
    base_ablation["disable_mounting_roll"] = True
    base_ablation["disable_gnss_lever_z"] = True
    base_fusion["enable_gnss_velocity"] = False
    base_fusion["gnss_path"] = "dataset/data2/rtk.txt"

    outage_gnss_path = args.artifacts_dir / "GNSS_outage_cycle_rtk_300on_100off_150on.txt"
    filter_stats = filter_gnss(
        input_path=(REPO_ROOT / "dataset/data2/rtk.txt").resolve(),
        output_path=outage_gnss_path,
        start_time=float(base_fusion["starttime"]),
        final_time=float(base_fusion["finaltime"]),
        initial_on=args.initial_on,
        on_dur=args.on,
        off_dur=args.off,
        cycle_starts_with=args.cycle_start,
    )

    full_cfg = copy.deepcopy(base_cfg)
    full_sol_path = args.artifacts_dir / "SOL_data2_baseline_eskf_full_control.txt"
    full_cfg["fusion"]["output_path"] = rel_from_root(full_sol_path, REPO_ROOT)

    outage_cfg = copy.deepcopy(base_cfg)
    outage_sol_path = args.artifacts_dir / "SOL_data2_eskf_outage_official.txt"
    outage_cfg["fusion"]["gnss_path"] = rel_from_root(outage_gnss_path, REPO_ROOT)
    outage_cfg["fusion"]["output_path"] = rel_from_root(outage_sol_path, REPO_ROOT)

    full_cfg_path = args.artifacts_dir / "cfg_data2_baseline_eskf_full_control.yaml"
    outage_cfg_path = args.artifacts_dir / "cfg_data2_eskf_outage_official.yaml"
    save_yaml(full_cfg, full_cfg_path)
    save_yaml(outage_cfg, outage_cfg_path)
    return full_cfg_path, outage_cfg_path, filter_stats


def evaluate_case(
    case_id: str,
    role: str,
    label: str,
    cfg_path: Path,
    stdout_path: Path,
    args: argparse.Namespace,
) -> tuple[dict[str, Any], list[dict[str, float]]]:
    cfg = load_yaml(cfg_path)
    fusion = cfg.get("fusion") or {}
    sol_path = (REPO_ROOT / str(fusion["output_path"])).resolve()
    truth_path = (REPO_ROOT / str(fusion["pos_path"])).resolve()

    merged = run_command([str(args.exe.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(merged, encoding="utf-8")

    truth_t, truth_xyz = load_truth_ecef(truth_path)
    sol_t, sol_xyz = load_sol(sol_path)
    truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
    err_xyz = sol_xyz - truth_interp

    start_time = float(fusion["starttime"])
    final_time = float(fusion["finaltime"])
    on_windows = extract_on_windows(cfg)
    outage_windows = invert_windows(on_windows, start_time, final_time)
    segment_rows = compute_outage_segments(sol_t, err_xyz, outage_windows)

    row: dict[str, Any] = {
        "case_id": case_id,
        "role": role,
        "label": label,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "truth_path": rel_from_root(truth_path, REPO_ROOT),
        "gnss_path": str(fusion["gnss_path"]),
        "enable_gnss_velocity": bool(fusion.get("enable_gnss_velocity", True)),
        "disable_mounting_roll": bool((fusion.get("ablation") or {}).get("disable_mounting_roll", False)),
        "disable_gnss_lever_z": bool((fusion.get("ablation") or {}).get("disable_gnss_lever_z", False)),
        "artifact_mtime_sol": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "artifact_mtime_stdout": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(compute_global_metrics(err_xyz))
    row.update(aggregate_outage_segments(segment_rows))
    return row, segment_rows


def build_metrics_row(
    args: argparse.Namespace,
    official_row: dict[str, Any],
    control_row: dict[str, Any],
) -> dict[str, Any]:
    metrics_row: dict[str, Any] = {
        "result_name": args.result_name,
        "dataset": "data2",
        "method": "ESKF",
        "evaluation_mode": "gnss_outage",
        "exp_id": args.exp_id,
        "official_case_id": official_row["case_id"],
        "official_config_path": official_row["config_path"],
        "official_sol_path": official_row["sol_path"],
        "official_stdout_path": official_row["stdout_path"],
        "control_case_id": control_row["case_id"],
        "control_config_path": control_row["config_path"],
        "control_sol_path": control_row["sol_path"],
        "control_stdout_path": control_row["stdout_path"],
        "enable_gnss_velocity": official_row["enable_gnss_velocity"],
        "disable_mounting_roll": official_row["disable_mounting_roll"],
        "disable_gnss_lever_z": official_row["disable_gnss_lever_z"],
        "artifact_mtime_official_sol": official_row["artifact_mtime_sol"],
        "artifact_mtime_official_stdout": official_row["artifact_mtime_stdout"],
        "artifact_mtime_control_sol": control_row["artifact_mtime_sol"],
        "artifact_mtime_control_stdout": control_row["artifact_mtime_stdout"],
    }

    for key in [
        "outage_segment_count",
        "mean_outage_rmse_x_m",
        "mean_outage_rmse_y_m",
        "mean_outage_rmse_z_m",
        "mean_outage_rmse_3d_m",
        "max_outage_rmse_x_m",
        "max_outage_rmse_y_m",
        "max_outage_rmse_z_m",
        "max_outage_rmse_3d_m",
        "mean_outage_final_err_x_m",
        "mean_outage_final_err_y_m",
        "mean_outage_final_err_z_m",
        "mean_outage_final_err_3d_m",
        "max_outage_final_err_x_m",
        "max_outage_final_err_y_m",
        "max_outage_final_err_z_m",
        "max_outage_final_err_3d_m",
    ]:
        metrics_row[key] = official_row[key]

    for prefix, source in [("official", official_row), ("control", control_row)]:
        for aux_key in [
            "overall_rmse_x_m_aux",
            "overall_rmse_y_m_aux",
            "overall_rmse_z_m_aux",
            "overall_rmse_3d_m_aux",
            "overall_p95_3d_m_aux",
            "overall_final_err_3d_m_aux",
        ]:
            metrics_row[f"{prefix}_{aux_key}"] = source[aux_key]

    return metrics_row


def write_summary(
    path: Path,
    metrics_row: dict[str, Any],
    official_row: dict[str, Any],
    control_row: dict[str, Any],
    segment_rows: list[dict[str, float]],
    filter_stats: dict[str, float | str],
    args: argparse.Namespace,
) -> None:
    lines: list[str] = [f"# {args.result_name} official GNSS outage evaluation", ""]
    lines.append("- 正式排序口径：先看 `mean_outage_rmse_3d_m`，再看 `max_outage_final_err_3d_m`。")
    lines.append("- `metrics.csv` 仅保留一行正式结果；full GNSS control 仅作为辅助诊断保留。")
    lines.append("- 整体 RMSE 继续存在，但只保留为 `_aux` 诊断字段，不作为 headline 指标。")
    lines.append("")

    lines.append("## official outage metrics")
    lines.extend(
        markdown_table(
            ["metric", "x [m]", "y [m]", "z [m]", "3d [m]"],
            [
                [
                    "mean outage RMSE",
                    format_metric(metrics_row["mean_outage_rmse_x_m"]),
                    format_metric(metrics_row["mean_outage_rmse_y_m"]),
                    format_metric(metrics_row["mean_outage_rmse_z_m"]),
                    format_metric(metrics_row["mean_outage_rmse_3d_m"]),
                ],
                [
                    "max outage RMSE",
                    format_metric(metrics_row["max_outage_rmse_x_m"]),
                    format_metric(metrics_row["max_outage_rmse_y_m"]),
                    format_metric(metrics_row["max_outage_rmse_z_m"]),
                    format_metric(metrics_row["max_outage_rmse_3d_m"]),
                ],
                [
                    "mean outage final err",
                    format_metric(metrics_row["mean_outage_final_err_x_m"]),
                    format_metric(metrics_row["mean_outage_final_err_y_m"]),
                    format_metric(metrics_row["mean_outage_final_err_z_m"]),
                    format_metric(metrics_row["mean_outage_final_err_3d_m"]),
                ],
                [
                    "max outage final err",
                    format_metric(metrics_row["max_outage_final_err_x_m"]),
                    format_metric(metrics_row["max_outage_final_err_y_m"]),
                    format_metric(metrics_row["max_outage_final_err_z_m"]),
                    format_metric(metrics_row["max_outage_final_err_3d_m"]),
                ],
            ],
        )
    )
    lines.append(f"- outage_segment_count: {int(metrics_row['outage_segment_count'])}")
    lines.append("")

    lines.append("## outage segments")
    if segment_rows:
        segment_table_rows: list[list[str]] = []
        for row in segment_rows:
            segment_table_rows.append(
                [
                    str(int(row["segment_id"])),
                    format_metric(row["start_time"]),
                    format_metric(row["end_time"]),
                    format_metric(row["duration_s"]),
                    str(int(row["samples"])),
                    format_metric(row["rmse_x_m"]),
                    format_metric(row["rmse_y_m"]),
                    format_metric(row["rmse_z_m"]),
                    format_metric(row["rmse_3d_m"]),
                    format_metric(row["final_err_x_m"]),
                    format_metric(row["final_err_y_m"]),
                    format_metric(row["final_err_z_m"]),
                    format_metric(row["final_err_3d_m"]),
                ]
            )
        lines.extend(
            markdown_table(
                [
                    "segment_id",
                    "start_time",
                    "end_time",
                    "duration_s",
                    "samples",
                    "rmse_x_m",
                    "rmse_y_m",
                    "rmse_z_m",
                    "rmse_3d_m",
                    "final_err_x_m",
                    "final_err_y_m",
                    "final_err_z_m",
                    "final_err_3d_m",
                ],
                segment_table_rows,
            )
        )
    else:
        lines.append("- no outage segments found")
    lines.append("")

    lines.append("## auxiliary diagnostics")
    lines.extend(
        markdown_table(
            ["case", "overall_rmse_x", "overall_rmse_y", "overall_rmse_z", "overall_rmse_3d", "overall_final_3d"],
            [
                [
                    "official_outage",
                    format_metric(metrics_row["official_overall_rmse_x_m_aux"]),
                    format_metric(metrics_row["official_overall_rmse_y_m_aux"]),
                    format_metric(metrics_row["official_overall_rmse_z_m_aux"]),
                    format_metric(metrics_row["official_overall_rmse_3d_m_aux"]),
                    format_metric(metrics_row["official_overall_final_err_3d_m_aux"]),
                ],
                [
                    "full_gnss_control",
                    format_metric(metrics_row["control_overall_rmse_x_m_aux"]),
                    format_metric(metrics_row["control_overall_rmse_y_m_aux"]),
                    format_metric(metrics_row["control_overall_rmse_z_m_aux"]),
                    format_metric(metrics_row["control_overall_rmse_3d_m_aux"]),
                    format_metric(metrics_row["control_overall_final_err_3d_m_aux"]),
                ],
            ],
        )
    )
    lines.append("")

    lines.append("## schedule and artifacts")
    lines.append(
        f"- official GNSS source: `{official_row['gnss_path']}`; control GNSS source: `{control_row['gnss_path']}`"
    )
    lines.append(
        f"- outage template: initial {format_metric(float(filter_stats['initial_on_s']))} s on, "
        f"then {format_metric(float(filter_stats['off_s']))} s off + {format_metric(float(filter_stats['on_s']))} s on"
    )
    lines.append(
        f"- first outage window: [{format_metric(float(filter_stats['first_off_start_time']))}, "
        f"{format_metric(float(filter_stats['first_off_end_time']))})"
    )
    lines.append(
        f"- kept GNSS rows: {int(float(filter_stats['rows_kept']))}/{int(float(filter_stats['rows_raw']))} "
        f"({format_metric(float(filter_stats['ratio_kept']))})"
    )
    lines.append(f"- plot dir: `{rel_from_root(args.plot_dir, REPO_ROOT)}`")
    lines.append(f"- artifacts dir: `{rel_from_root(args.artifacts_dir, REPO_ROOT)}`")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def run_standard_plot(
    args: argparse.Namespace,
    official_cfg_path: Path,
    official_row: dict[str, Any],
    outage_segments_path: Path,
) -> Path:
    plot_stdout_path = args.artifacts_dir / "plot_generation.stdout.txt"
    plot_cmd = [
        sys.executable,
        str((REPO_ROOT / "plot_navresult.py").resolve()),
        str((REPO_ROOT / official_row["sol_path"]).resolve()),
        str(official_cfg_path.resolve()),
        "--mode",
        "standard",
        "--output-dir",
        str(args.plot_dir.resolve()),
        "--outage-segments",
        str(outage_segments_path.resolve()),
    ]
    plot_stdout = run_command(plot_cmd, REPO_ROOT)
    plot_stdout_path.write_text(plot_stdout, encoding="utf-8")

    missing = [name for name in EXPECTED_PLOT_FILES if not (args.plot_dir / name).exists()]
    if missing:
        raise RuntimeError(f"missing standard plots: {missing}")
    return plot_stdout_path


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(description="Run the official standardized data2 RTK ESKF GNSS outage evaluation.")
    parser.add_argument(
        "--base-config",
        type=Path,
        default=Path("config_data2_baseline_eskf.yaml"),
        help="Base data2 ESKF config relative to repo root.",
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
        default=Path("output/data2_eskf_baseline"),
        help="Standard result directory relative to repo root.",
    )
    parser.add_argument(
        "--result-name",
        default="data2_eskf_baseline",
        help="Canonical standardized result name.",
    )
    parser.add_argument(
        "--exp-id",
        default=f"EXP-{today}-data2-eskf-baseline-output-r2",
        help="Experiment identifier recorded in manifest and walkthrough.",
    )
    parser.add_argument("--initial-on", type=float, default=300.0)
    parser.add_argument("--off", type=float, default=100.0)
    parser.add_argument("--on", type=float, default=150.0)
    parser.add_argument("--cycle-start", choices=("off", "on"), default="off")
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.plot_dir = args.output_dir / "plot"
    args.artifacts_dir = args.output_dir / "artifacts"
    return args


def main() -> None:
    args = parse_args()
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    reset_directory(args.output_dir)
    ensure_dir(args.plot_dir)
    ensure_dir(args.artifacts_dir)

    full_cfg_path, outage_cfg_path, filter_stats = build_case_configs(args)

    control_row, _ = evaluate_case(
        case_id="data2_baseline_eskf_full_control",
        role="control",
        label="data2 full GNSS ESKF control",
        cfg_path=full_cfg_path,
        stdout_path=args.artifacts_dir / "data2_baseline_eskf_full_control.stdout.txt",
        args=args,
    )
    official_row, segment_rows = evaluate_case(
        case_id="data2_eskf_outage_official",
        role="official_outage",
        label="data2 ESKF outage official",
        cfg_path=outage_cfg_path,
        stdout_path=args.artifacts_dir / "data2_eskf_outage_official.stdout.txt",
        args=args,
    )

    metrics_row = build_metrics_row(args, official_row=official_row, control_row=control_row)

    metrics_path = args.output_dir / "metrics.csv"
    pd.DataFrame([metrics_row]).to_csv(metrics_path, index=False, encoding="utf-8-sig")

    outage_segments_path = args.output_dir / "outage_segments.csv"
    pd.DataFrame(segment_rows).to_csv(outage_segments_path, index=False, encoding="utf-8-sig")

    plot_stdout_path = run_standard_plot(
        args=args,
        official_cfg_path=outage_cfg_path,
        official_row=official_row,
        outage_segments_path=outage_segments_path,
    )

    summary_path = args.output_dir / "summary.md"
    write_summary(
        summary_path,
        metrics_row=metrics_row,
        official_row=official_row,
        control_row=control_row,
        segment_rows=segment_rows,
        filter_stats=filter_stats,
        args=args,
    )

    stats_path = args.artifacts_dir / "gnss_outage_stats.json"
    stats_path.write_text(json.dumps(filter_stats, indent=2, ensure_ascii=False), encoding="utf-8")

    manifest = {
        "result_name": args.result_name,
        "dataset": "data2",
        "method": "ESKF",
        "evaluation_mode": "gnss_outage",
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "metrics_csv": rel_from_root(metrics_path, REPO_ROOT),
        "outage_segments_csv": rel_from_root(outage_segments_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "plot_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "official_config_path": metrics_row["official_config_path"],
        "official_sol_path": metrics_row["official_sol_path"],
        "official_stdout_path": metrics_row["official_stdout_path"],
        "control_config_path": metrics_row["control_config_path"],
        "control_sol_path": metrics_row["control_sol_path"],
        "control_stdout_path": metrics_row["control_stdout_path"],
        "gnss_outage_stats_json": rel_from_root(stats_path, REPO_ROOT),
        "plot_generation_stdout_path": rel_from_root(plot_stdout_path, REPO_ROOT),
        "schedule": {
            "initial_on_s": args.initial_on,
            "off_s": args.off,
            "on_s": args.on,
            "cycle_start": args.cycle_start,
            "first_off_start_time": filter_stats.get("first_off_start_time"),
            "first_off_end_time": filter_stats.get("first_off_end_time"),
        },
        "metrics_row": metrics_row,
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
