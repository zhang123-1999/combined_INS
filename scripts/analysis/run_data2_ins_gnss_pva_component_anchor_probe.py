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
from scripts.analysis.run_data2_ins_gnss_lever_truth_init_probe import build_lever_metrics
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


GNSS_LEVER_COLUMNS = {
    "x": "gnss_lever_x_m",
    "y": "gnss_lever_y_m",
    "z": "gnss_lever_z_m",
}
CASE_COLORS = {
    "baseline_free_run": "#4c78a8",
    "anchor_attitude_only": "#f58518",
    "anchor_position_only": "#54a24b",
    "anchor_velocity_only": "#e45756",
    "anchor_pva_all": "#72b7b2",
}


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    anchor_component: str | None
    experiment_name: str


CASE_SPECS = [
    CaseSpec("baseline_free_run", "baseline free-run", None, "baseline"),
    CaseSpec("anchor_attitude_only", "experiment A", "attitude", "A_only_attitude"),
    CaseSpec("anchor_position_only", "experiment B", "position", "B_only_position"),
    CaseSpec("anchor_velocity_only", "experiment C", "velocity", "C_only_velocity"),
    CaseSpec("anchor_pva_all", "full PVA anchor", "all", "D_all_components"),
]


def anchor_component_flags(anchor_component: str | None) -> dict[str, bool]:
    return {
        "position": anchor_component in {"position", "all"},
        "velocity": anchor_component in {"velocity", "all"},
        "attitude": anchor_component in {"attitude", "all"},
    }


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    gnss_path: Path,
    spec: CaseSpec,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_noise = base_cfg["fusion"]["noise"]
    p0_diag = base_p0_diag_from_config(base_cfg)

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
    fusion["ablation"] = ablation_cfg
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = spec.anchor_component is not None
    init_cfg["runtime_truth_anchor_gnss_only"] = spec.anchor_component is not None
    anchor_flags = anchor_component_flags(spec.anchor_component)
    init_cfg["runtime_truth_anchor_position"] = anchor_flags["position"]
    init_cfg["runtime_truth_anchor_velocity"] = anchor_flags["velocity"]
    init_cfg["runtime_truth_anchor_attitude"] = anchor_flags["attitude"]
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
        "fusion.init.runtime_truth_anchor_pva": spec.anchor_component is not None,
        "fusion.init.runtime_truth_anchor_gnss_only": spec.anchor_component is not None,
        "fusion.init.runtime_truth_anchor_position": anchor_flags["position"],
        "fusion.init.runtime_truth_anchor_velocity": anchor_flags["velocity"],
        "fusion.init.runtime_truth_anchor_attitude": anchor_flags["attitude"],
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
    spec: CaseSpec,
) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(base_cfg, truth_reference, case_dir, gnss_path, spec)
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides


def run_case(spec: CaseSpec, case_dir: Path, cfg_path: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    first_update_path = case_dir / f"first_update_{spec.case_id}.csv"
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
    if not first_update_path.exists():
        raise RuntimeError(f"missing first update output for {spec.case_id}: {first_update_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {spec.case_id}")
    shutil.copy2(root_diag, diag_path)

    row: dict[str, Any] = {
        "case_id": spec.case_id,
        "case_label": spec.label,
        "experiment_name": spec.experiment_name,
        "anchor_component": spec.anchor_component or "none",
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


def summarize_case(lever_df: pd.DataFrame) -> dict[str, Any]:
    final_dev = lever_df["final_deviation_from_truth_m"].to_numpy(dtype=float)
    max_dev = lever_df["max_abs_deviation_from_truth_m"].to_numpy(dtype=float)
    first60 = lever_df["mean_abs_deviation_first_60s_m"].to_numpy(dtype=float)
    return {
        "final_dev_norm_m": float(np.linalg.norm(final_dev)),
        "final_dev_max_m": float(np.max(final_dev)),
        "max_abs_dev_max_m": float(np.max(max_dev)),
        "first60_dev_norm_m": float(np.linalg.norm(first60)),
        "persistent_departure_count": int((lever_df["label"] == "persistent_departure").sum()),
    }


def compare_against_baseline(
    case_metrics_df: pd.DataFrame,
    lever_case_df: pd.DataFrame,
) -> pd.DataFrame:
    baseline_row = case_metrics_df.loc[case_metrics_df["case_id"] == "baseline_free_run"].iloc[0]
    baseline_axes = lever_case_df[lever_case_df["case_id"] == "baseline_free_run"].set_index("state_name")
    rows: list[dict[str, Any]] = []
    for _, row in case_metrics_df.iterrows():
        case_id = str(row["case_id"])
        case_axes = lever_case_df[lever_case_df["case_id"] == case_id].set_index("state_name")
        out = row.to_dict()
        out["delta_final_dev_norm_vs_baseline"] = float(
            row["final_dev_norm_m"] - float(baseline_row["final_dev_norm_m"])
        )
        out["ratio_final_dev_norm_vs_baseline"] = float(
            row["final_dev_norm_m"] / max(float(baseline_row["final_dev_norm_m"]), 1.0e-9)
        )
        out["delta_final_dev_max_vs_baseline"] = float(
            row["final_dev_max_m"] - float(baseline_row["final_dev_max_m"])
        )
        out["ratio_nav_rmse3d_vs_baseline"] = float(
            row["nav_rmse_3d_m"] / max(float(baseline_row["nav_rmse_3d_m"]), 1.0e-9)
        )
        for axis in ["x", "y", "z"]:
            state_name = f"gnss_lever_{axis}"
            out[f"{axis}_final_dev_delta_vs_baseline"] = float(
                case_axes.loc[state_name, "final_deviation_from_truth_m"]
                - baseline_axes.loc[state_name, "final_deviation_from_truth_m"]
            )
        rows.append(out)
    return pd.DataFrame(rows)


def plot_axis_comparison(
    case_rows: list[dict[str, Any]],
    truth_reference: dict[str, Any],
    axis: str,
    output_path: Path,
) -> None:
    state_name = f"gnss_lever_{axis}"
    column = GNSS_LEVER_COLUMNS[axis]
    truth_value = float(truth_reference["states"][state_name]["reference_value"])
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for case_row in case_rows:
        state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        df = pd.read_csv(state_path, usecols=["timestamp", column])
        t = df["timestamp"].to_numpy(dtype=float)
        v = df[column].to_numpy(dtype=float)
        t_plot, v_plot = downsample_for_plot(t, v)
        ax.plot(
            t_plot,
            v_plot,
            linewidth=1.2,
            label=str(case_row["case_label"]),
            color=CASE_COLORS.get(str(case_row["case_id"]), None),
        )
    ax.axhline(truth_value, linestyle="--", color="black", linewidth=1.0, label="truth")
    ax.set_title(f"{state_name}: baseline vs A/B/C + full PVA anchor")
    ax.set_xlabel("timestamp [s]")
    ax.set_ylabel("m")
    ax.grid(alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_final_dev_bar(case_metrics_df: pd.DataFrame, output_path: Path) -> None:
    ordered = case_metrics_df.set_index("case_id").loc[[spec.case_id for spec in CASE_SPECS]].reset_index()
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(
        ordered["case_label"],
        ordered["final_dev_norm_m"],
        color=[CASE_COLORS.get(case_id, "#999999") for case_id in ordered["case_id"]],
    )
    ax.set_ylabel("m")
    ax.set_title("GNSS lever final deviation norm")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_path: Path,
    case_compare_df: pd.DataFrame,
    lever_case_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    baseline_row = case_compare_df.loc[case_compare_df["case_id"] == "baseline_free_run"].iloc[0]
    anchored_df = case_compare_df[case_compare_df["case_id"] != "baseline_free_run"].copy()
    best_row = anchored_df.sort_values(
        by=["final_dev_norm_m", "final_dev_max_m", "nav_rmse_3d_m"],
        ascending=[True, True, True],
    ).iloc[0]

    axis_best: dict[str, pd.Series] = {}
    for axis in ["x", "y", "z"]:
        state_name = f"gnss_lever_{axis}"
        axis_df = lever_case_df[
            (lever_case_df["state_name"] == state_name)
            & (lever_case_df["case_id"] != "baseline_free_run")
        ].copy()
        axis_best[axis] = axis_df.sort_values(
            by=["final_deviation_from_truth_m", "max_abs_deviation_from_truth_m"],
            ascending=[True, True],
        ).iloc[0]

    lines: list[str] = [
        "# data2 INS/GNSS PVA component-anchor probe",
        "",
        "## 1. 实验设计",
        (
            "- 基线 case 保持上一轮 `GNSS lever 真值初值但不锚定` 口径："
            "`use_truth_pva=true`、`runtime_truth_anchor_pva=false`、"
            "`enable_odo=false`、`enable_nhc=false`、"
            "`disable_mounting/odo_lever/odo_scale=true`、`enable_gnss_velocity=false`。"
        ),
        (
            "- 三个对照 case 仅在成功 `GNSS` 更新后执行单分量真值回写："
            "实验 A 只锚定姿态，实验 B 只锚定位置，实验 C 只锚定速度；"
            "另外新增 `full PVA anchor`，在同一触发条件下同时锚定 `position+velocity+attitude`。"
        ),
        "",
        "## 2. 哪个 P/V/A 分量最能改善杆臂偏差",
        (
            f"- 基线 `final_dev_norm_m={format_metric(float(baseline_row['final_dev_norm_m']))}`，"
            f"当前所有对照中最优的是 `{best_row['case_label']}`，"
            f"`final_dev_norm_m={format_metric(float(best_row['final_dev_norm_m']))}`，"
            f"`nav_rmse_3d_m={format_metric(float(best_row['nav_rmse_3d_m']))}`。"
        ),
    ]
    single_component_df = anchored_df[anchored_df["anchor_component"].isin(["attitude", "position", "velocity"])].copy()
    if not single_component_df.empty:
        best_single_row = single_component_df.sort_values(
            by=["final_dev_norm_m", "final_dev_max_m", "nav_rmse_3d_m"],
            ascending=[True, True, True],
        ).iloc[0]
        lines.append(
            f"- 若只在单分量里比较，最优的是 `{best_single_row['case_label']}`，"
            f"anchor=`{best_single_row['anchor_component']}`，"
            f"`final_dev_norm_m={format_metric(float(best_single_row['final_dev_norm_m']))}`。"
        )
    for _, row in anchored_df.sort_values(by="final_dev_norm_m").iterrows():
        lines.append(
            f"- `{row['case_label']}`: anchor=`{row['anchor_component']}`, "
            f"`final_dev_norm={format_metric(float(row['final_dev_norm_m']))}` m, "
            f"`final_dev_max={format_metric(float(row['final_dev_max_m']))}` m, "
            f"`delta_vs_baseline={format_metric(float(row['delta_final_dev_norm_vs_baseline']))}` m, "
            f"`nav_rmse_3d={format_metric(float(row['nav_rmse_3d_m']))}` m."
        )

    lines.extend(["", "## 3. 分轴结果"])
    for axis in ["x", "y", "z"]:
        row = axis_best[axis]
        lines.append(
            f"- `gnss_lever_{axis}` 改善最明显的是 `{row['case_label']}`，"
            f"`final_dev={format_metric(float(row['final_deviation_from_truth_m']))}` m, "
            f"`max_dev={format_metric(float(row['max_abs_deviation_from_truth_m']))}` m, "
            f"`drift_source={row['drift_source']}`。"
        )

    lines.extend(["", "## 4. 源头判断"])
    if str(best_row["anchor_component"]) == "all":
        lines.append(
            "- `full PVA anchor` 给出了最强 overall 改善，说明多分量同时分担时效果最好；"
            "但单分量比较里仍应分别看 `position` 和 `attitude` 的相对作用。"
        )
    elif str(best_row["anchor_component"]) == "attitude":
        lines.append("- 当前 strongest evidence 指向 `姿态` 是主偏差源头；只拉回姿态时，杆臂偏差改善最明显。")
    elif str(best_row["anchor_component"]) == "position":
        lines.append("- 当前 strongest evidence 指向 `位置` 是主偏差源头；只拉回位置时，杆臂偏差改善最明显。")
    elif str(best_row["anchor_component"]) == "velocity":
        lines.append("- 当前 strongest evidence 指向 `速度` 是主偏差源头；只拉回速度时，杆臂偏差改善最明显。")
    else:
        lines.append("- 单分量锚定没有形成清晰优势，暂时不能把源头单独归到某一个 P/V/A 分量。")
    lines.append(
        "- 若某一轴与 overall 最优实验不一致，应优先解释为多分量耦合或不同轴对 `GNSS_POS` 残差分担机制不同。"
    )

    lines.extend(
        [
            "",
            "## Notes",
            (
                "- 本轮仍使用 `dataset/data2/rtk.txt`，它不含 GNSS 速度列；"
                "因此所有结论都只涉及 `GNSS_POS` 更新链，不涉及 `GNSS_VEL`。"
            ),
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- lever_case_metrics: `{manifest['lever_case_metrics_csv']}`",
            f"- comparison_summary: `{manifest['comparison_summary_csv']}`",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run data2 INS/GNSS experiments with GNSS-only single-component P/V/A runtime truth anchors."
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
        default=Path("output/data2_ins_gnss_pva_component_anchor_probe"),
        help="Output directory relative to repo root.",
    )
    parser.add_argument(
        "--gnss-path",
        type=Path,
        default=Path("dataset/data2/rtk.txt"),
        help="GNSS path used by the experiments.",
    )
    parser.add_argument(
        "--exp-id",
        default="EXP-20260318-data2-ins-gnss-pva-component-anchor-r1",
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

    case_rows: list[dict[str, Any]] = []
    lever_rows: list[pd.DataFrame] = []
    case_config_paths: dict[str, str] = {}
    override_table: dict[str, dict[str, Any]] = {}

    for spec in CASE_SPECS:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides = write_case_config(base_cfg, truth_reference, case_dir, args.gnss_path, spec)
        case_config_paths[spec.case_id] = rel_from_root(cfg_path, REPO_ROOT)
        override_table[spec.case_id] = overrides
        case_row = run_case(spec, case_dir, cfg_path, args.exe)
        lever_df = build_lever_metrics(
            truth_reference,
            (REPO_ROOT / case_row["state_series_path"]).resolve(),
            (REPO_ROOT / case_row["first_update_path"]).resolve(),
        )
        lever_df.insert(0, "case_label", spec.label)
        lever_df.insert(0, "case_id", spec.case_id)
        case_row.update(summarize_case(lever_df))
        case_rows.append(case_row)
        lever_rows.append(lever_df)

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df["case_order"] = range(len(case_metrics_df))
    case_metrics_df = case_metrics_df.set_index("case_id").loc[[spec.case_id for spec in CASE_SPECS]].reset_index()
    lever_case_df = pd.concat(lever_rows, ignore_index=True)
    comparison_df = compare_against_baseline(case_metrics_df, lever_case_df)
    comparison_df = comparison_df.set_index("case_id").loc[[spec.case_id for spec in CASE_SPECS]].reset_index()

    case_metrics_path = args.output_dir / "case_metrics.csv"
    lever_case_metrics_path = args.output_dir / "lever_case_metrics.csv"
    comparison_summary_path = args.output_dir / "comparison_summary.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    lever_case_df.to_csv(lever_case_metrics_path, index=False, encoding="utf-8-sig")
    comparison_df.to_csv(comparison_summary_path, index=False, encoding="utf-8-sig")

    for axis in ["x", "y", "z"]:
        plot_axis_comparison(case_rows, truth_reference, axis, args.plot_dir / f"gnss_lever_{axis}_compare.png")
    plot_final_dev_bar(case_metrics_df, args.plot_dir / "final_deviation_norm_bar.png")

    freshness = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "lever_case_metrics_csv": dt.datetime.fromtimestamp(lever_case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "comparison_summary_csv": dt.datetime.fromtimestamp(comparison_summary_path.stat().st_mtime).isoformat(timespec="seconds"),
        "plots_dir": dt.datetime.fromtimestamp((args.plot_dir / "final_deviation_norm_bar.png").stat().st_mtime).isoformat(timespec="seconds"),
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
        "lever_case_metrics_csv": rel_from_root(lever_case_metrics_path, REPO_ROOT),
        "comparison_summary_csv": rel_from_root(comparison_summary_path, REPO_ROOT),
        "truth_catalog_source": truth_reference["sources"],
        "case_config_paths": case_config_paths,
        "case_overrides": override_table,
        "freshness": freshness,
        "mode": "ins_gnss_pva_component_anchor_probe",
        "assumptions": [
            "所有 case 都使用 GNSS lever truth init + large P0/process noise 口径。",
            "A/B/C 只在成功 GNSS 更新后锚定对应单个 P/V/A 分量。",
            "仍只研究 GNSS_POS 链，不引入 GNSS_VEL 更新。",
        ],
    }
    manifest["manifest_path"] = rel_from_root(manifest_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    write_summary(summary_path, comparison_df, lever_case_df, manifest)
    freshness["summary_md"] = dt.datetime.fromtimestamp(summary_path.stat().st_mtime).isoformat(timespec="seconds")
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
