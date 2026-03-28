import argparse
import copy
import datetime as dt
import json
import math
import re
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

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml  # noqa: E402
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import (  # noqa: E402
    build_case_config as build_baseline_case_config,
)
from scripts.analysis.run_data2_state_sanity_matrix import reset_directory, run_command  # noqa: E402
from scripts.analysis.run_nhc_state_convergence_research import parse_diag_times  # noqa: E402
from scripts.analysis.run_data2_staged_estimation import phase_absolute_times, scaled_noise_override  # noqa: E402


EXP_ID_DEFAULT = "EXP-20260324-data2-phase2-bridge-probe-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_phase2_bridge_probe_r1_20260324")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
PHASE1_END_OFFSET_DEFAULT = 200.0
PROBE_END_OFFSET_DEFAULT = 484.6
CONVERGED_NOISE_SCALE_DEFAULT = 0.1
BRIDGE_DURATION_DEFAULT = 10.0
BRIDGE_SIGMA_ODO_SCALE_DEFAULT = 5.0e-3
BRIDGE_SIGMA_MOUNTING_DEFAULT = 4.6e-3
BRIDGE_SIGMA_LEVER_DEFAULT = 2.8e-2

STD_PROBE_TIMES = (528276.0, 528300.0, 528400.0, 528487.0, 528551.0)
THRESHOLD_SPECS = (
    ("ba_x_mgal", 500.0),
    ("ba_x_mgal", 1000.0),
    ("ba_x_mgal", 2000.0),
    ("bg_z_degh", 10.0),
    ("bg_z_degh", 100.0),
    ("bg_z_degh", 300.0),
    ("sa_x_ppm", 5000.0),
    ("sa_x_ppm", 10000.0),
    ("sa_x_ppm", 20000.0),
    ("gnss_lever_y_m", 0.5),
    ("gnss_lever_y_m", 1.0),
    ("gnss_lever_y_m", 5.0),
)
STATE_PLOT_COLUMNS = (
    ("ba_x_mgal", "ba_x", "mGal"),
    ("bg_z_degh", "bg_z", "deg/h"),
    ("sa_x_ppm", "sa_x", "ppm"),
    ("gnss_lever_y_m", "gnss_lever_y", "m"),
)
COV_PLOT_COLUMNS = (
    ("std_odo_s", "std_odo_scale", ""),
    ("std_my", "std_mounting_yaw", "rad"),
    ("std_ly", "std_odo_lever_y", "m"),
    ("std_lgy", "std_gnss_lever_y", "m"),
)


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    bridge_duration: float
    bridge_sigma_odo_scale: float | None = None
    bridge_sigma_mounting: float | None = None
    bridge_sigma_lever: float | None = None
    freeze_gnss_lever_phase2: bool = False


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        case_id="staged_release_baseline",
        label="staged release baseline",
        color="#4c78a8",
        bridge_duration=0.0,
    ),
    CaseSpec(
        case_id="staged_release_bridge_cov_seed_like",
        label="staged release + 10s bridge covariance seed",
        color="#f58518",
        bridge_duration=BRIDGE_DURATION_DEFAULT,
        bridge_sigma_odo_scale=BRIDGE_SIGMA_ODO_SCALE_DEFAULT,
        bridge_sigma_mounting=BRIDGE_SIGMA_MOUNTING_DEFAULT,
        bridge_sigma_lever=BRIDGE_SIGMA_LEVER_DEFAULT,
    ),
    CaseSpec(
        case_id="staged_release_phase2_freeze_gnss_lever",
        label="staged release + phase2 freeze GNSS lever",
        color="#54a24b",
        bridge_duration=0.0,
        freeze_gnss_lever_phase2=True,
    ),
)


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


def format_float(value: Any, digits: int = 6) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float, np.floating)):
        value = float(value)
        if not math.isfinite(value):
            return "NA"
        return f"{value:.{digits}f}"
    return str(value)


def render_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def bridge_noise_override(
    noise_cfg: dict[str, Any],
    converged_scale: float,
    odo_sigma: float | None,
    mounting_sigma: float | None,
    lever_sigma: float | None,
) -> dict[str, Any]:
    override = scaled_noise_override(noise_cfg, converged_scale)
    if odo_sigma is not None:
        override["sigma_odo_scale"] = float(odo_sigma)
    if mounting_sigma is not None:
        override["sigma_mounting"] = float(mounting_sigma)
        override["sigma_mounting_roll"] = float(mounting_sigma)
        override["sigma_mounting_pitch"] = float(mounting_sigma)
        override["sigma_mounting_yaw"] = float(mounting_sigma)
    if lever_sigma is not None:
        override["sigma_lever_arm"] = float(lever_sigma)
        override["sigma_lever_arm_vec"] = [float(lever_sigma)] * 3
    return override


def build_runtime_phases(
    cfg: dict[str, Any],
    phase1_end_time: float,
    probe_end_time: float,
    converged_noise_scale: float,
    spec: CaseSpec,
) -> list[dict[str, Any]]:
    base_noise = cfg["fusion"]["noise"]
    converged_noise = scaled_noise_override(base_noise, converged_noise_scale)
    phases: list[dict[str, Any]] = [
        {
            "name": "phase1_ins_gnss_freeze_odo_nhc_states",
            "start_time": float(cfg["fusion"]["starttime"]),
            "end_time": phase1_end_time,
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        }
    ]
    if spec.bridge_duration > 0.0:
        bridge_end = min(probe_end_time, phase1_end_time + spec.bridge_duration)
        phases.append(
            {
                "name": "phase2_bridge_cov_seed_like_boost",
                "start_time": phase1_end_time,
                "end_time": bridge_end,
                "noise": bridge_noise_override(
                    noise_cfg=base_noise,
                    converged_scale=converged_noise_scale,
                    odo_sigma=spec.bridge_sigma_odo_scale,
                    mounting_sigma=spec.bridge_sigma_mounting,
                    lever_sigma=spec.bridge_sigma_lever,
                ),
            }
        )
        if bridge_end < probe_end_time:
            phases.append(
                {
                    "name": "phase2_joint_calibration_reduced_converged_noise",
                    "start_time": bridge_end,
                    "end_time": probe_end_time,
                    "noise": converged_noise,
                }
            )
    else:
        phases.append(
            {
                "name": "phase2_joint_calibration_reduced_converged_noise",
                "start_time": phase1_end_time,
                "end_time": probe_end_time,
                "noise": converged_noise,
            }
        )
    return phases


def build_case_artifacts(case_dir: Path, case_id: str) -> dict[str, Path]:
    return {
        "config": case_dir / f"config_{case_id}.yaml",
        "sol": case_dir / f"SOL_{case_id}.txt",
        "state_series": case_dir / f"state_series_{case_id}.csv",
        "stdout": case_dir / f"solver_stdout_{case_id}.txt",
        "diag": case_dir / f"DIAG_{case_id}.txt",
    }


def build_case_config(
    base_cfg: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    phase1_end_time: float,
    probe_end_time: float,
    converged_noise_scale: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    baseline_spec = type(
        "BaselineSpec",
        (),
        {
            "case_id": spec.case_id,
            "label": spec.label,
            "color": spec.color,
            "filter_mode": "ESKF",
            "enable_fej": False,
            "enable_runtime_anchor": False,
        },
    )()
    cfg, overrides = build_baseline_case_config(base_cfg, case_dir, baseline_spec)
    fusion = cfg.setdefault("fusion", {})
    artifacts = build_case_artifacts(case_dir, spec.case_id)
    fusion["finaltime"] = float(probe_end_time)
    fusion["output_path"] = rel_from_root(artifacts["sol"], REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(artifacts["state_series"], REPO_ROOT)
    fusion["gnss_schedule"] = {"enabled": False}
    fusion["runtime_phases"] = build_runtime_phases(
        cfg=cfg,
        phase1_end_time=phase1_end_time,
        probe_end_time=probe_end_time,
        converged_noise_scale=converged_noise_scale,
        spec=spec,
    )
    fusion.setdefault("constraints", {})["enable_diagnostics"] = True
    fusion["constraints"]["enable_consistency_log"] = True
    fusion["constraints"]["enable_mechanism_log"] = False
    if spec.freeze_gnss_lever_phase2:
        fusion["constraints"]["debug_gnss_lever_arm_disable_start_time"] = float(phase1_end_time)
        fusion["constraints"]["debug_gnss_lever_arm_disable_end_time"] = float(probe_end_time)
    fusion.setdefault("ablation", {})["disable_mounting_roll"] = False
    fusion["ablation"]["disable_gnss_lever_z"] = False

    overrides["fusion.finaltime"] = float(probe_end_time)
    overrides["fusion.output_path"] = rel_from_root(artifacts["sol"], REPO_ROOT)
    overrides["fusion.state_series_output_path"] = rel_from_root(artifacts["state_series"], REPO_ROOT)
    overrides["fusion.gnss_schedule.enabled"] = False
    overrides["fusion.runtime_phases"] = fusion["runtime_phases"]
    overrides["fusion.constraints.enable_diagnostics"] = True
    overrides["fusion.constraints.enable_consistency_log"] = True
    overrides["fusion.constraints.enable_mechanism_log"] = False
    if spec.freeze_gnss_lever_phase2:
        overrides["fusion.constraints.debug_gnss_lever_arm_disable_start_time"] = float(phase1_end_time)
        overrides["fusion.constraints.debug_gnss_lever_arm_disable_end_time"] = float(probe_end_time)
    overrides["fusion.ablation.disable_mounting_roll"] = False
    overrides["fusion.ablation.disable_gnss_lever_z"] = False
    return cfg, overrides


def run_case(case_dir: Path, cfg_path: Path, exe_path: Path, spec: CaseSpec) -> dict[str, Any]:
    artifacts = build_case_artifacts(case_dir, spec.case_id)
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    artifacts["stdout"].write_text(stdout_text, encoding="utf-8")
    for key in ("sol", "state_series"):
        if not artifacts[key].exists():
            raise RuntimeError(f"missing solver artifact for {spec.case_id}: {artifacts[key]}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after {spec.case_id}")
    shutil.copy2(root_diag, artifacts["diag"])
    metrics = {
        "case_id": spec.case_id,
        "case_label": spec.label,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(artifacts["sol"], REPO_ROOT),
        "state_series_path": rel_from_root(artifacts["state_series"], REPO_ROOT),
        "stdout_path": rel_from_root(artifacts["stdout"], REPO_ROOT),
        "diag_path": rel_from_root(artifacts["diag"], REPO_ROOT),
        "config_mtime": mtime_text(cfg_path),
        "sol_mtime": mtime_text(artifacts["sol"]),
        "state_series_mtime": mtime_text(artifacts["state_series"]),
        "stdout_mtime": mtime_text(artifacts["stdout"]),
        "diag_mtime": mtime_text(artifacts["diag"]),
    }
    metrics.update(parse_diag_times(stdout_text))
    return metrics


def load_case_data(case_dir: Path, spec: CaseSpec, start_time: float) -> dict[str, Any]:
    artifacts = build_case_artifacts(case_dir, spec.case_id)
    series_df = pd.read_csv(artifacts["state_series"])
    diag_df = pd.read_csv(artifacts["diag"], sep=r"\s+", engine="python")
    stdout_text = artifacts["stdout"].read_text(encoding="utf-8", errors="ignore")
    diag_df["abs_t"] = start_time + diag_df["t"].to_numpy(dtype=float)
    return {
        "series": series_df,
        "diag": diag_df,
        "stdout": stdout_text,
        "artifacts": artifacts,
    }


def first_threshold_crossing(df: pd.DataFrame, column: str, threshold: float) -> float | None:
    values = np.abs(df[column].to_numpy(dtype=float))
    idx = np.flatnonzero(values >= threshold)
    if idx.size == 0:
        return None
    return float(df["timestamp"].iloc[int(idx[0])])


def nearest_value(df: pd.DataFrame, time_col: str, value_col: str, target_t: float) -> float | None:
    if df.empty:
        return None
    idx = (df[time_col] - target_t).abs().idxmin()
    value = df.loc[idx, value_col]
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def parse_gnss_h_att(stdout_text: str) -> pd.DataFrame:
    pattern = re.compile(r"\[GNSS_POS\] t=([-+0-9eE\.]+).*?\|\|H_att\|\|_F=([-+0-9eE\.]+)")
    rows: list[dict[str, float]] = []
    for match in pattern.finditer(stdout_text):
        rows.append({"t": float(match.group(1)), "h_att_norm": float(match.group(2))})
    return pd.DataFrame(rows)


def enrich_case_metrics(
    metrics_row: dict[str, Any],
    case_data: dict[str, Any],
    probe_times: tuple[float, ...],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    series_df = case_data["series"]
    diag_df = case_data["diag"]
    stdout_text = case_data["stdout"]
    h_att_df = parse_gnss_h_att(stdout_text)

    threshold_rows: list[dict[str, Any]] = []
    for column, threshold in THRESHOLD_SPECS:
        crossing_t = first_threshold_crossing(series_df, column, threshold)
        threshold_rows.append(
            {
                "case_id": metrics_row["case_id"],
                "state": column,
                "threshold": threshold,
                "first_cross_t": crossing_t,
            }
        )
        metrics_row[f"{column}_cross_{str(threshold).replace('.', 'p')}"] = crossing_t

    for target_t in probe_times:
        suffix = str(int(target_t))
        metrics_row[f"std_odo_s_at_{suffix}"] = nearest_value(diag_df, "abs_t", "std_odo_s", target_t)
        metrics_row[f"std_my_at_{suffix}"] = nearest_value(diag_df, "abs_t", "std_my", target_t)
        metrics_row[f"std_ly_at_{suffix}"] = nearest_value(diag_df, "abs_t", "std_ly", target_t)
        metrics_row[f"std_lgy_at_{suffix}"] = nearest_value(diag_df, "abs_t", "std_lgy", target_t)

    for column, _, _ in STATE_PLOT_COLUMNS:
        metrics_row[f"max_abs_{column}"] = float(np.max(np.abs(series_df[column].to_numpy(dtype=float))))

    if not h_att_df.empty:
        max_idx = int(h_att_df["h_att_norm"].idxmax())
        metrics_row["max_h_att_norm"] = float(h_att_df.loc[max_idx, "h_att_norm"])
        metrics_row["max_h_att_t"] = float(h_att_df.loc[max_idx, "t"])
        first_div_t = metrics_row.get("first_div_gnss_pos_t")
        metrics_row["h_att_at_first_div_gnss_pos"] = (
            nearest_value(h_att_df, "t", "h_att_norm", float(first_div_t))
            if first_div_t is not None
            else None
        )
    else:
        metrics_row["max_h_att_norm"] = None
        metrics_row["max_h_att_t"] = None
        metrics_row["h_att_at_first_div_gnss_pos"] = None
    return metrics_row, threshold_rows


def plot_case_comparison(
    output_path: Path,
    case_series: dict[str, pd.DataFrame],
    cases: tuple[CaseSpec, ...],
    columns: tuple[tuple[str, str, str], ...],
    xlim: tuple[float, float],
    phase1_end_time: float,
) -> None:
    fig, axes = plt.subplots(len(columns), 1, figsize=(11, 2.6 * len(columns)), sharex=True)
    if len(columns) == 1:
        axes = [axes]
    for ax, (column, label, unit) in zip(axes, columns):
        for spec in cases:
            df = case_series[spec.case_id]
            ax.plot(df["timestamp"], df[column], color=spec.color, linewidth=1.0, label=spec.label)
        ax.axvline(phase1_end_time, color="#333333", linestyle="--", linewidth=0.9)
        ax.set_ylabel(f"{label}\n{unit}".strip())
        ax.grid(alpha=0.25, linewidth=0.4)
    axes[0].legend(loc="upper left", ncol=2, frameon=False)
    axes[-1].set_xlim(xlim[0], xlim[1])
    axes[-1].set_xlabel("timestamp (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_covariance_comparison(
    output_path: Path,
    case_diags: dict[str, pd.DataFrame],
    cases: tuple[CaseSpec, ...],
    columns: tuple[tuple[str, str, str], ...],
    xlim: tuple[float, float],
    phase1_end_time: float,
) -> None:
    fig, axes = plt.subplots(len(columns), 1, figsize=(11, 2.6 * len(columns)), sharex=True)
    if len(columns) == 1:
        axes = [axes]
    for ax, (column, label, unit) in zip(axes, columns):
        for spec in cases:
            df = case_diags[spec.case_id]
            ax.plot(df["abs_t"], df[column], color=spec.color, linewidth=1.0, label=spec.label)
        ax.axvline(phase1_end_time, color="#333333", linestyle="--", linewidth=0.9)
        ax.set_ylabel(f"{label}\n{unit}".strip())
        ax.grid(alpha=0.25, linewidth=0.4)
    axes[0].legend(loc="upper left", ncol=2, frameon=False)
    axes[-1].set_xlim(xlim[0], xlim[1])
    axes[-1].set_xlabel("timestamp (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_dir: Path,
    exp_id: str,
    cases: tuple[CaseSpec, ...],
    metrics_df: pd.DataFrame,
    thresholds_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    summary_path = output_dir / "summary.md"
    rows = []
    for spec in cases:
        row = metrics_df.loc[metrics_df["case_id"] == spec.case_id].iloc[0]
        rows.append(
            [
                spec.case_id,
                format_float(row.get("first_divergence_start_t"), 3),
                format_float(row.get("first_div_gnss_pos_t"), 3),
                format_float(row.get("max_h_att_norm"), 3),
                format_float(row.get("max_h_att_t"), 3),
                format_float(row.get("max_abs_ba_x_mgal"), 1),
                format_float(row.get("max_abs_bg_z_degh"), 1),
                format_float(row.get("max_abs_sa_x_ppm"), 1),
                format_float(row.get("max_abs_gnss_lever_y_m"), 3),
            ]
        )

    threshold_focus = thresholds_df[
        thresholds_df["threshold"].isin([500.0, 10.0, 5000.0, 0.5])
    ].copy()
    threshold_rows = []
    for spec in cases:
        for state_name in ["ba_x_mgal", "bg_z_degh", "sa_x_ppm", "gnss_lever_y_m"]:
            match = threshold_focus[
                (threshold_focus["case_id"] == spec.case_id)
                & (threshold_focus["state"] == state_name)
            ]
            if match.empty:
                continue
            row = match.iloc[0]
            threshold_rows.append(
                [
                    spec.case_id,
                    state_name,
                    format_float(row["threshold"], 1),
                    format_float(row["first_cross_t"], 3),
                ]
            )

    std_rows = []
    for target_t in STD_PROBE_TIMES:
        suffix = str(int(target_t))
        for spec in cases:
            row = metrics_df.loc[metrics_df["case_id"] == spec.case_id].iloc[0]
            std_rows.append(
                [
                    spec.case_id,
                    suffix,
                    format_float(row.get(f"std_odo_s_at_{suffix}"), 6),
                    format_float(row.get(f"std_my_at_{suffix}"), 6),
                    format_float(row.get(f"std_ly_at_{suffix}"), 6),
                    format_float(row.get(f"std_lgy_at_{suffix}"), 6),
                ]
            )

    lines = [
        f"# {exp_id}",
        "",
        "## Purpose",
        "",
        "- Validate whether the staged `phase2` jump family is driven by released `odo_scale/mounting/lever_odo` states staying covariance-pinned after `phase1` freeze.",
        "- Use two narrow controls instead of broad retuning: `A)` a short `phase2` bridge that boosts ODO-related process noise toward the baseline covariance scale, and `B)` a `phase2` case that freezes `GNSS lever` to test whether it is an upstream source or a downstream jump executor.",
        "",
        "## Case Metrics",
        "",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "first_div_start_t",
                "first_div_gnss_pos_t",
                "max_H_att",
                "max_H_att_t",
                "max|ba_x|",
                "max|bg_z|",
                "max|sa_x|",
                "max|gnss_lever_y|",
            ],
            rows,
        )
    )
    lines.extend(
        [
            "",
            "## First Threshold Crossing",
            "",
        ]
    )
    lines.extend(render_table(["case_id", "state", "threshold", "first_cross_t"], threshold_rows))
    lines.extend(
        [
            "",
            "## ODO Block Std Snapshot",
            "",
        ]
    )
    lines.extend(
        render_table(
            ["case_id", "abs_t", "std_odo_s", "std_my", "std_ly", "std_lgy"],
            std_rows,
        )
    )
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            f"- manifest: `{rel_from_root(output_dir / 'manifest.json', REPO_ROOT)}`",
            f"- case_metrics: `{rel_from_root(output_dir / 'case_metrics.csv', REPO_ROOT)}`",
            f"- threshold_crossings: `{rel_from_root(output_dir / 'threshold_crossings.csv', REPO_ROOT)}`",
            f"- state_plot: `{rel_from_root(output_dir / 'plots' / 'key_state_bridge_probe.png', REPO_ROOT)}`",
            f"- covariance_plot: `{rel_from_root(output_dir / 'plots' / 'odo_block_std_bridge_probe.png', REPO_ROOT)}`",
            f"- freshness: `summary={mtime_text(output_dir / 'case_metrics.csv')}` `manifest={mtime_text(output_dir / 'manifest.json')}`",
        ]
    )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe the staged phase2 bridge failure mechanism.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--solver", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--phase1-end-offset", type=float, default=PHASE1_END_OFFSET_DEFAULT)
    parser.add_argument("--probe-end-offset", type=float, default=PROBE_END_OFFSET_DEFAULT)
    parser.add_argument("--converged-noise-scale", type=float, default=CONVERGED_NOISE_SCALE_DEFAULT)
    args = parser.parse_args()

    base_cfg = load_yaml(args.base_config)
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    reset_directory(output_dir)
    plots_dir = output_dir / "plots"
    ensure_dir(plots_dir)

    start_time = float(base_cfg["fusion"]["starttime"])
    phase1_end_time, _ = phase_absolute_times(
        start_time, args.phase1_end_offset, args.phase1_end_offset
    )
    probe_end_time = start_time + args.probe_end_offset

    metrics_rows: list[dict[str, Any]] = []
    threshold_rows: list[dict[str, Any]] = []
    case_series: dict[str, pd.DataFrame] = {}
    case_diags: dict[str, pd.DataFrame] = {}
    case_overrides: dict[str, Any] = {}

    for spec in CASE_SPECS:
        case_dir = output_dir / "artifacts" / "cases" / spec.case_id
        ensure_dir(case_dir)
        cfg, overrides = build_case_config(
            base_cfg=base_cfg,
            case_dir=case_dir,
            spec=spec,
            phase1_end_time=phase1_end_time,
            probe_end_time=probe_end_time,
            converged_noise_scale=args.converged_noise_scale,
        )
        cfg_path = build_case_artifacts(case_dir, spec.case_id)["config"]
        save_yaml(cfg, cfg_path)
        metrics_row = run_case(case_dir, cfg_path, args.solver, spec)
        case_data = load_case_data(case_dir, spec, start_time)
        metrics_row, case_threshold_rows = enrich_case_metrics(
            metrics_row=metrics_row,
            case_data=case_data,
            probe_times=STD_PROBE_TIMES,
        )
        metrics_rows.append(metrics_row)
        threshold_rows.extend(case_threshold_rows)
        case_series[spec.case_id] = case_data["series"]
        case_diags[spec.case_id] = case_data["diag"]
        case_overrides[spec.case_id] = overrides

    metrics_df = pd.DataFrame(metrics_rows)
    thresholds_df = pd.DataFrame(threshold_rows)
    metrics_df.to_csv(output_dir / "case_metrics.csv", index=False)
    thresholds_df.to_csv(output_dir / "threshold_crossings.csv", index=False)

    plot_case_comparison(
        output_path=plots_dir / "key_state_bridge_probe.png",
        case_series=case_series,
        cases=CASE_SPECS,
        columns=STATE_PLOT_COLUMNS,
        xlim=(phase1_end_time - 5.0, probe_end_time),
        phase1_end_time=phase1_end_time,
    )
    plot_covariance_comparison(
        output_path=plots_dir / "odo_block_std_bridge_probe.png",
        case_diags=case_diags,
        cases=CASE_SPECS,
        columns=COV_PLOT_COLUMNS,
        xlim=(phase1_end_time - 5.0, probe_end_time),
        phase1_end_time=phase1_end_time,
    )

    manifest = {
        "exp_id": args.exp_id,
        "base_config": rel_from_root((REPO_ROOT / args.base_config).resolve(), REPO_ROOT),
        "solver": rel_from_root((REPO_ROOT / args.solver).resolve(), REPO_ROOT),
        "phase_window": {
            "phase1_start": start_time,
            "phase1_end": phase1_end_time,
            "probe_end": probe_end_time,
        },
        "converged_noise_scale": args.converged_noise_scale,
        "bridge_cases": {
            spec.case_id: {
                "bridge_duration": spec.bridge_duration,
                "bridge_sigma_odo_scale": spec.bridge_sigma_odo_scale,
                "bridge_sigma_mounting": spec.bridge_sigma_mounting,
                "bridge_sigma_lever": spec.bridge_sigma_lever,
                "overrides": case_overrides[spec.case_id],
            }
            for spec in CASE_SPECS
        },
        "artifacts": {
            "case_metrics": rel_from_root(output_dir / "case_metrics.csv", REPO_ROOT),
            "threshold_crossings": rel_from_root(output_dir / "threshold_crossings.csv", REPO_ROOT),
            "plots": {
                "key_states": rel_from_root(plots_dir / "key_state_bridge_probe.png", REPO_ROOT),
                "odo_block_std": rel_from_root(plots_dir / "odo_block_std_bridge_probe.png", REPO_ROOT),
            },
        },
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_summary(
        output_dir=output_dir,
        exp_id=args.exp_id,
        cases=CASE_SPECS,
        metrics_df=metrics_df,
        thresholds_df=thresholds_df,
        manifest=manifest,
    )


if __name__ == "__main__":
    main()
