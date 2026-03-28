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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import (  # noqa: E402
    ensure_dir,
    load_yaml,
    parse_consistency_summary,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    downsample_for_plot,
    evaluate_navigation_metrics,
    json_safe,
    reset_directory,
    run_command,
)


@dataclass(frozen=True)
class StateSpec:
    key: str
    label: str
    unit: str
    source: str
    column: str


@dataclass(frozen=True)
class GroupSpec:
    group_id: str
    title: str
    states: tuple[StateSpec, ...]


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    filter_mode: str
    enable_fej: bool
    enable_runtime_anchor: bool


SOL_SOURCE = "sol"
SERIES_SOURCE = "state_series"
ODO_SCALE_ZERO_EPS = 1.0e-6
MOUNTING_SIGMA_SCALE = 0.1


GROUP_SPECS: tuple[GroupSpec, ...] = (
    GroupSpec(
        "position",
        "Position",
        (
            StateSpec("pos_x", "p_x", "m", SOL_SOURCE, "fused_x"),
            StateSpec("pos_y", "p_y", "m", SOL_SOURCE, "fused_y"),
            StateSpec("pos_z", "p_z", "m", SOL_SOURCE, "fused_z"),
        ),
    ),
    GroupSpec(
        "velocity",
        "Velocity",
        (
            StateSpec("vel_x", "v_x", "m/s", SOL_SOURCE, "fused_vx"),
            StateSpec("vel_y", "v_y", "m/s", SOL_SOURCE, "fused_vy"),
            StateSpec("vel_z", "v_z", "m/s", SOL_SOURCE, "fused_vz"),
        ),
    ),
    GroupSpec(
        "attitude",
        "Attitude",
        (
            StateSpec("att_roll", "roll", "deg", SOL_SOURCE, "fused_roll"),
            StateSpec("att_pitch", "pitch", "deg", SOL_SOURCE, "fused_pitch"),
            StateSpec("att_yaw", "yaw", "deg", SOL_SOURCE, "fused_yaw"),
        ),
    ),
    GroupSpec(
        "ba",
        "Accel Bias",
        (
            StateSpec("ba_x", "ba_x", "mGal", SERIES_SOURCE, "ba_x_mgal"),
            StateSpec("ba_y", "ba_y", "mGal", SERIES_SOURCE, "ba_y_mgal"),
            StateSpec("ba_z", "ba_z", "mGal", SERIES_SOURCE, "ba_z_mgal"),
        ),
    ),
    GroupSpec(
        "bg",
        "Gyro Bias",
        (
            StateSpec("bg_x", "bg_x", "deg/h", SERIES_SOURCE, "bg_x_degh"),
            StateSpec("bg_y", "bg_y", "deg/h", SERIES_SOURCE, "bg_y_degh"),
            StateSpec("bg_z", "bg_z", "deg/h", SERIES_SOURCE, "bg_z_degh"),
        ),
    ),
    GroupSpec(
        "sg",
        "Gyro Scale",
        (
            StateSpec("sg_x", "sg_x", "ppm", SERIES_SOURCE, "sg_x_ppm"),
            StateSpec("sg_y", "sg_y", "ppm", SERIES_SOURCE, "sg_y_ppm"),
            StateSpec("sg_z", "sg_z", "ppm", SERIES_SOURCE, "sg_z_ppm"),
        ),
    ),
    GroupSpec(
        "sa",
        "Accel Scale",
        (
            StateSpec("sa_x", "sa_x", "ppm", SERIES_SOURCE, "sa_x_ppm"),
            StateSpec("sa_y", "sa_y", "ppm", SERIES_SOURCE, "sa_y_ppm"),
            StateSpec("sa_z", "sa_z", "ppm", SERIES_SOURCE, "sa_z_ppm"),
        ),
    ),
    GroupSpec(
        "odo_scale",
        "ODO Scale",
        (StateSpec("odo_scale", "odo_scale", "1", SERIES_SOURCE, "odo_scale_state"),),
    ),
    GroupSpec(
        "mounting",
        "Mounting",
        (
            StateSpec("mounting_roll", "mounting_roll", "deg", SERIES_SOURCE, "mounting_roll_deg"),
            StateSpec("mounting_pitch", "mounting_pitch", "deg", SERIES_SOURCE, "mounting_pitch_deg"),
            StateSpec("mounting_yaw", "mounting_yaw", "deg", SERIES_SOURCE, "mounting_yaw_deg"),
        ),
    ),
    GroupSpec(
        "odo_lever",
        "ODO Lever",
        (
            StateSpec("odo_lever_x", "odo_lever_x", "m", SERIES_SOURCE, "odo_lever_x_m"),
            StateSpec("odo_lever_y", "odo_lever_y", "m", SERIES_SOURCE, "odo_lever_y_m"),
            StateSpec("odo_lever_z", "odo_lever_z", "m", SERIES_SOURCE, "odo_lever_z_m"),
        ),
    ),
    GroupSpec(
        "gnss_lever",
        "GNSS Lever",
        (
            StateSpec("gnss_lever_x", "gnss_lever_x", "m", SERIES_SOURCE, "gnss_lever_x_m"),
            StateSpec("gnss_lever_y", "gnss_lever_y", "m", SERIES_SOURCE, "gnss_lever_y_m"),
            StateSpec("gnss_lever_z", "gnss_lever_z", "m", SERIES_SOURCE, "gnss_lever_z_m"),
        ),
    ),
)

ALL_STATE_SPECS: tuple[StateSpec, ...] = tuple(
    state for group in GROUP_SPECS for state in group.states
)

CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec("group1_eskf_free_run", "group1: ESKF free-run", "#4c78a8", "ESKF", False, False),
    CaseSpec("group2_eskf_pva_truth_anchor", "group2: ESKF PVA truth anchor", "#f58518", "ESKF", False, True),
)


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


def format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            return "NA"
        return f"{float(value):.6f}"
    return str(value)


def flatten_consistency_metrics(stdout_text: str) -> dict[str, Any]:
    parsed = parse_consistency_summary(stdout_text)
    row: dict[str, Any] = {}
    for sensor_name, metrics in parsed.items():
        prefix = sensor_name.lower()
        for metric_name, metric_value in metrics.items():
            row[f"{prefix}_{metric_name}"] = float(metric_value)
    return row


def metric_value(row: pd.Series, *keys: str) -> Any:
    for key in keys:
        if key in row.index:
            return row.get(key)
    return None


def build_case_config(base_cfg: dict[str, Any], case_dir: Path, spec: CaseSpec) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    ablation_cfg = fusion.setdefault("ablation", {})
    post_ablation_cfg = fusion.setdefault("post_gnss_ablation", {})
    fej_cfg = fusion.setdefault("fej", {})
    base_noise = base_cfg["fusion"]["noise"]

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["gnss_schedule"] = {"enabled": False}
    fej_cfg["enable"] = spec.enable_fej
    constraints_cfg["enable_odo"] = True
    constraints_cfg["enable_nhc"] = True
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = spec.enable_runtime_anchor
    init_cfg["runtime_truth_anchor_position"] = spec.enable_runtime_anchor
    init_cfg["runtime_truth_anchor_velocity"] = spec.enable_runtime_anchor
    init_cfg["runtime_truth_anchor_attitude"] = spec.enable_runtime_anchor
    init_cfg["runtime_truth_anchor_gnss_only"] = False
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    init_cfg["ba0"] = [0.0, 0.0, 0.0]
    init_cfg["bg0"] = [0.0, 0.0, 0.0]
    init_cfg["sg0"] = [0.0, 0.0, 0.0]
    init_cfg["sa0"] = [0.0, 0.0, 0.0]
    init_cfg["odo_scale"] = ODO_SCALE_ZERO_EPS
    init_cfg["mounting_roll0"] = 0.0
    init_cfg["mounting_pitch0"] = 0.0
    init_cfg["mounting_yaw0"] = 0.0
    init_cfg["lever_arm0"] = [0.0, 0.0, 0.0]
    init_cfg["gnss_lever_arm0"] = [0.0, 0.0, 0.0]

    noise_cfg["sigma_mounting"] = float(base_noise.get("sigma_mounting", 0.0)) * MOUNTING_SIGMA_SCALE
    noise_cfg["sigma_mounting_roll"] = (
        float(base_noise.get("sigma_mounting_roll", base_noise.get("sigma_mounting", 0.0)))
        * MOUNTING_SIGMA_SCALE
    )
    noise_cfg["sigma_mounting_pitch"] = (
        float(base_noise.get("sigma_mounting_pitch", base_noise.get("sigma_mounting", 0.0)))
        * MOUNTING_SIGMA_SCALE
    )
    noise_cfg["sigma_mounting_yaw"] = (
        float(base_noise.get("sigma_mounting_yaw", base_noise.get("sigma_mounting", 0.0)))
        * MOUNTING_SIGMA_SCALE
    )

    ablation_cfg["disable_mounting"] = False
    ablation_cfg["disable_mounting_roll"] = False
    ablation_cfg["disable_odo_scale"] = False
    ablation_cfg["disable_odo_lever_arm"] = False
    ablation_cfg["disable_gnss_lever_arm"] = False
    ablation_cfg["disable_gnss_lever_z"] = False

    post_ablation_cfg["enabled"] = False

    overrides = {
        "fusion.output_path": rel_from_root(sol_path, REPO_ROOT),
        "fusion.state_series_output_path": rel_from_root(state_series_path, REPO_ROOT),
        "fusion.gnss_schedule.enabled": False,
        "fusion.fej.enable": spec.enable_fej,
        "fusion.constraints.enable_odo": True,
        "fusion.constraints.enable_nhc": True,
        "fusion.constraints.enable_diagnostics": True,
        "fusion.constraints.enable_consistency_log": True,
        "fusion.init.use_truth_pva": True,
        "fusion.init.runtime_truth_anchor_pva": spec.enable_runtime_anchor,
        "fusion.init.runtime_truth_anchor_position": spec.enable_runtime_anchor,
        "fusion.init.runtime_truth_anchor_velocity": spec.enable_runtime_anchor,
        "fusion.init.runtime_truth_anchor_attitude": spec.enable_runtime_anchor,
        "fusion.init.runtime_truth_anchor_gnss_only": False,
        "fusion.init.ba0": [0.0, 0.0, 0.0],
        "fusion.init.bg0": [0.0, 0.0, 0.0],
        "fusion.init.sg0": [0.0, 0.0, 0.0],
        "fusion.init.sa0": [0.0, 0.0, 0.0],
        "fusion.init.odo_scale": ODO_SCALE_ZERO_EPS,
        "fusion.init.mounting_roll0": 0.0,
        "fusion.init.mounting_pitch0": 0.0,
        "fusion.init.mounting_yaw0": 0.0,
        "fusion.init.lever_arm0": [0.0, 0.0, 0.0],
        "fusion.init.gnss_lever_arm0": [0.0, 0.0, 0.0],
        "fusion.noise.sigma_mounting": noise_cfg["sigma_mounting"],
        "fusion.noise.sigma_mounting_roll": noise_cfg["sigma_mounting_roll"],
        "fusion.noise.sigma_mounting_pitch": noise_cfg["sigma_mounting_pitch"],
        "fusion.noise.sigma_mounting_yaw": noise_cfg["sigma_mounting_yaw"],
        "fusion.ablation.disable_mounting_roll": False,
        "fusion.ablation.disable_gnss_lever_z": False,
    }
    return cfg, overrides


def write_case_config(base_cfg: dict[str, Any], case_dir: Path, spec: CaseSpec) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(base_cfg, case_dir, spec)
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides


def run_case(case_dir: Path, cfg_path: Path, exe_path: Path, spec: CaseSpec) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
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
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {spec.case_id}")
    shutil.copy2(root_diag, diag_path)

    nav_metrics, segment_rows = evaluate_navigation_metrics(cfg_path, sol_path)
    row: dict[str, Any] = {
        "case_id": spec.case_id,
        "case_label": spec.label,
        "filter_mode": spec.filter_mode,
        "runtime_truth_anchor_pva": spec.enable_runtime_anchor,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "config_mtime": mtime_text(cfg_path),
        "sol_mtime": mtime_text(sol_path),
        "state_series_mtime": mtime_text(state_series_path),
        "diag_mtime": mtime_text(diag_path),
        "stdout_mtime": mtime_text(stdout_path),
        "segment_rows": segment_rows,
    }
    row.update(nav_metrics)
    row.update(flatten_consistency_metrics(stdout_text))
    return row


def build_unified_state_df(sol_path: Path, state_series_path: Path) -> pd.DataFrame:
    sol_df = pd.read_csv(sol_path, sep=r"\s+", engine="python")
    state_df = pd.read_csv(state_series_path)
    if "odo_scale" in state_df.columns:
        state_df = state_df.rename(columns={"odo_scale": "odo_scale_state"})
    merged = pd.merge_asof(
        sol_df.sort_values("timestamp"),
        state_df.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=1.0e-6,
    )
    missing = [spec.column for spec in ALL_STATE_SPECS if spec.column not in merged.columns]
    if missing:
        raise RuntimeError(f"missing columns after merge: {missing}")
    unified = pd.DataFrame({"timestamp": merged["timestamp"].to_numpy(dtype=float)})
    for spec in ALL_STATE_SPECS:
        unified[spec.key] = merged[spec.column].to_numpy(dtype=float)
    return unified


def save_unified_state_df(case_dir: Path, spec: CaseSpec, state_df: pd.DataFrame) -> Path:
    path = case_dir / f"all_states_{spec.case_id}.csv"
    state_df.to_csv(path, index=False, encoding="utf-8-sig")
    return path


def render_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def plot_group(case_frames: dict[str, pd.DataFrame], group_spec: GroupSpec, output_path: Path) -> None:
    fig, axes = plt.subplots(len(group_spec.states), 1, figsize=(12, 3.2 * len(group_spec.states)), sharex=True)
    axes_arr = np.atleast_1d(axes)
    for idx, state_spec in enumerate(group_spec.states):
        ax = axes_arr[idx]
        for case_spec in CASE_SPECS:
            df = case_frames[case_spec.case_id]
            t = df["timestamp"].to_numpy(dtype=float)
            v = df[state_spec.key].to_numpy(dtype=float)
            t_plot, v_plot = downsample_for_plot(t, v)
            ax.plot(t_plot, v_plot, linewidth=1.2, color=case_spec.color, label=case_spec.label)
        ax.set_title(f"{state_spec.label} [{state_spec.unit}]")
        ax.set_ylabel(state_spec.unit)
        ax.grid(alpha=0.25)
        if idx == 0:
            ax.legend(loc="best")
    axes_arr[-1].set_xlabel("timestamp [s]")
    fig.suptitle(group_spec.title, fontsize=13)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_all_states_overview(case_frames: dict[str, pd.DataFrame], output_path: Path) -> None:
    n_cols = 3
    n_rows = math.ceil(len(ALL_STATE_SPECS) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 2.6 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, state_spec in enumerate(ALL_STATE_SPECS):
        ax = axes_flat[idx]
        for case_spec in CASE_SPECS:
            df = case_frames[case_spec.case_id]
            t = df["timestamp"].to_numpy(dtype=float)
            v = df[state_spec.key].to_numpy(dtype=float)
            t_plot, v_plot = downsample_for_plot(t, v)
            ax.plot(t_plot, v_plot, linewidth=0.9, color=case_spec.color, label=case_spec.label)
        ax.set_title(state_spec.label, fontsize=9)
        ax.set_ylabel(state_spec.unit, fontsize=8)
        ax.grid(alpha=0.2)
        ax.tick_params(axis="both", labelsize=7)
        if idx == 0:
            ax.legend(loc="best", fontsize=8)
    for idx in range(len(ALL_STATE_SPECS), len(axes_flat)):
        axes_flat[idx].axis("off")
    fig.suptitle("All 31 states overview", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_final_state_comparison(case_frames: dict[str, pd.DataFrame]) -> pd.DataFrame:
    base_case = CASE_SPECS[0].case_id
    anchor_case = CASE_SPECS[1].case_id
    rows: list[dict[str, Any]] = []
    for state_spec in ALL_STATE_SPECS:
        base_val = float(case_frames[base_case][state_spec.key].iloc[-1])
        anchor_val = float(case_frames[anchor_case][state_spec.key].iloc[-1])
        values = [base_val, anchor_val]
        rows.append(
            {
                "state_name": state_spec.key,
                "label": state_spec.label,
                "unit": state_spec.unit,
                f"{base_case}_final": base_val,
                f"{anchor_case}_final": anchor_val,
                "anchor_minus_eskf_free_run": anchor_val - base_val,
                "max_span_across_cases": max(values) - min(values),
            }
        )
    return pd.DataFrame(rows)


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    final_state_df: pd.DataFrame,
) -> None:
    metric_rows: list[list[str]] = []
    metric_columns = [
        "case_label",
        "filter_mode",
        "overall_rmse_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "gnss_accept_ratio",
    ]
    for _, row in case_metrics_df.iterrows():
        metric_rows.append(
            [
                str(row.get("case_label", "")),
                str(row.get("filter_mode", "")),
                format_metric(row.get("overall_rmse_3d_m_aux")),
                format_metric(row.get("overall_final_err_3d_m_aux")),
                format_metric(row.get("odo_accept_ratio")),
                format_metric(row.get("nhc_accept_ratio")),
                format_metric(metric_value(row, "gnss_accept_ratio", "gnss_pos_accept_ratio")),
            ]
        )

    top_delta = final_state_df.sort_values(by="max_span_across_cases", ascending=False).head(8)
    delta_rows = [
        [
            str(row["state_name"]),
            str(row["unit"]),
            format_metric(row["group1_eskf_free_run_final"]),
            format_metric(row["group2_eskf_pva_truth_anchor_final"]),
            format_metric(row["anchor_minus_eskf_free_run"]),
            format_metric(row["max_span_across_cases"]),
        ]
        for _, row in top_delta.iterrows()
    ]

    lines: list[str] = [
        "# data2 INS/GNSS/ODO/NHC two-way baseline comparison",
        "",
        "## 实验设计",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base config: `{manifest['base_config']}`",
        "- 两组都保持 `INS/GNSS/ODO/NHC`、同一套噪声配置、`use_truth_pva=true`。",
        "- 两组都将非 PVA 状态从零初值开始估计：`ba/bg/sg/sa=0`、`mounting=0`、`odo_lever=0`、`gnss_lever=0`。",
        f"- `odo_scale` 因当前初始化保护不能直接取 `0`，本轮按仓内既有口径使用 `odo_scale={ODO_SCALE_ZERO_EPS:.0e}` 近似零初值。",
        "- 为保证 31 维状态都能参与估计，本轮显式关闭基线中的 `disable_mounting_roll` 与 `disable_gnss_lever_z` ablation。",
        "- 安装角过程噪声统一下调到 baseline 的 `0.1x`。",
        "- 组一：`ESKF free-run`，`fusion.fej.enable=false`，`runtime_truth_anchor_pva=false`。",
        "- 组二：`ESKF + continuous PVA truth anchor`，`fusion.fej.enable=false`，连续锚定 `position+velocity+attitude`。",
        "",
        "## 导航与一致性指标",
    ]
    lines.extend(render_table(metric_columns, metric_rows))
    lines.extend(
        [
            "",
            "## 末态差异最大的状态",
        ]
    )
    lines.extend(
        render_table(
            [
                "state_name",
                "unit",
                "group1_eskf_free_run_final",
                "group2_eskf_pva_truth_anchor_final",
                "anchor_minus_eskf_free_run",
                "max_span_across_cases",
            ],
            delta_rows,
        )
    )
    lines.extend(
        [
            "",
            "## 产物",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- final_state_comparison: `{manifest['final_state_comparison_csv']}`",
            f"- plots_dir: `{manifest['plots_dir']}`",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run two data2 INS/GNSS/ODO/NHC baselines: ESKF free-run and ESKF PVA truth anchor, then plot all 31 states."
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
        default=Path("output/data2_ins_gnss_odo_nhc_pva_anchor_compare_r2_20260323"),
        help="Output directory relative to repo root.",
    )
    parser.add_argument(
        "--exp-id",
        default="EXP-20260323-data2-ins-gnss-odo-nhc-pva-anchor-compare-r2",
        help="Experiment identifier recorded in manifest.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)

    case_rows: list[dict[str, Any]] = []
    case_frames: dict[str, pd.DataFrame] = {}
    case_config_paths: dict[str, str] = {}
    case_unified_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}

    for spec in CASE_SPECS:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides = write_case_config(base_cfg, case_dir, spec)
        case_config_paths[spec.case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_overrides[spec.case_id] = overrides
        case_row = run_case(case_dir, cfg_path, args.exe, spec)
        unified_df = build_unified_state_df(
            (REPO_ROOT / case_row["sol_path"]).resolve(),
            (REPO_ROOT / case_row["state_series_path"]).resolve(),
        )
        unified_path = save_unified_state_df(case_dir, spec, unified_df)
        case_row["all_states_path"] = rel_from_root(unified_path, REPO_ROOT)
        case_row["all_states_mtime"] = mtime_text(unified_path)
        case_frames[spec.case_id] = unified_df
        case_unified_paths[spec.case_id] = case_row["all_states_path"]
        case_rows.append(case_row)

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df = case_metrics_df.set_index("case_id").loc[[spec.case_id for spec in CASE_SPECS]].reset_index()
    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    final_state_df = build_final_state_comparison(case_frames)
    final_state_comparison_path = args.output_dir / "final_state_comparison.csv"
    final_state_df.to_csv(final_state_comparison_path, index=False, encoding="utf-8-sig")

    plot_paths: dict[str, str] = {}
    for group_spec in GROUP_SPECS:
        plot_path = args.plot_dir / f"{group_spec.group_id}_compare.png"
        plot_group(case_frames, group_spec, plot_path)
        plot_paths[group_spec.group_id] = rel_from_root(plot_path, REPO_ROOT)
    overview_path = args.plot_dir / "all_states_overview.png"
    plot_all_states_overview(case_frames, overview_path)
    plot_paths["all_states_overview"] = rel_from_root(overview_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "final_state_comparison_csv": rel_from_root(final_state_comparison_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_all_states_paths": case_unified_paths,
        "case_overrides": case_overrides,
        "plots": plot_paths,
        "assumptions": [
            "Both cases keep INS/GNSS/ODO/NHC enabled and share the same non-mounting noise settings.",
            "Non-PVA states start from zero; odo_scale uses 1e-6 instead of exact zero to satisfy the current initialization guard.",
            "runtime_truth_anchor_gnss_only=false means the anchored case applies continuous runtime P/V/A truth anchoring.",
            "Mounting process noise is reduced to 0.1x of the baseline config for both cases.",
            "disable_mounting_roll and disable_gnss_lever_z are explicitly cleared so all 31 states are estimated and can be plotted.",
        ],
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_exe_mtime": mtime_text(args.exe),
            "case_metrics_csv": mtime_text(case_metrics_path),
            "final_state_comparison_csv": mtime_text(final_state_comparison_path),
            "plots_dir": mtime_text(overview_path),
        },
    }
    manifest["manifest_path"] = rel_from_root(manifest_path, REPO_ROOT)

    write_summary(summary_path, manifest, case_metrics_df, final_state_df)
    manifest["freshness"]["summary_md"] = mtime_text(summary_path)
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
