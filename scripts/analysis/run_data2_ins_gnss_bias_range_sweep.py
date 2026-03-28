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

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics, impact_label
from scripts.analysis.run_data2_state_sanity_matrix import (
    STATE_META,
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    dynamic_behavior_label,
    format_metric,
    get_group_instability_internal,
    get_group_vector_internal,
    json_safe,
    reset_directory,
    run_command,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-bias-range-sweep-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_bias_range_sweep")
CONTROL_CASE_ID = "truth_anchor_bias_control"
FREE_CASE_PREFIX = "free_bias"
BIAS_STATE_ORDER = ["ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"]
BA_Q_SCALES_DEFAULT = [0.25, 1.0, 4.0]
BG_Q_SCALES_DEFAULT = [0.25, 1.0, 4.0]
BA_P0_SCALES_DEFAULT = [1.0]
BG_P0_SCALES_DEFAULT = [1.0]
LABEL_SCORE = {"normal": 0, "borderline": 1, "abnormal": 2}


@dataclass(frozen=True)
class CaseSpec:
    ba_p0_scale: float
    bg_p0_scale: float
    ba_q_scale: float
    bg_q_scale: float

    @property
    def case_id(self) -> str:
        return (
            f"{FREE_CASE_PREFIX}_"
            f"bap0{scale_slug(self.ba_p0_scale)}_"
            f"bgp0{scale_slug(self.bg_p0_scale)}_"
            f"baq{scale_slug(self.ba_q_scale)}_"
            f"bgq{scale_slug(self.bg_q_scale)}"
        )


def scale_slug(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def parse_scale_list(raw: str) -> list[float]:
    values = [item.strip() for item in raw.split(",") if item.strip()]
    if not values:
        raise ValueError("scale list must not be empty")
    return [float(item) for item in values]


def free_bias_std(reference: np.ndarray, instability: np.ndarray, scale: float) -> np.ndarray:
    base = np.maximum(np.abs(reference), 3.0 * instability)
    return np.asarray(base * float(scale), dtype=float)


def anchored_dynamic_std(instability: np.ndarray) -> np.ndarray:
    return np.asarray(0.1 * instability, dtype=float)


def anchored_dynamic_noise(instability: np.ndarray) -> np.ndarray:
    return np.asarray(0.1 * instability, dtype=float)


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_dir: Path,
    case_id: str,
    spec: CaseSpec | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_p0 = list(base_p0_diag_from_config(base_cfg))
    p0_diag = list(base_p0)

    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    diag_path = case_dir / f"DIAG_{case_id}.txt"

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

    ba_ref = get_group_vector_internal(truth_reference, "ba")
    bg_ref = get_group_vector_internal(truth_reference, "bg")
    sg_ref = get_group_vector_internal(truth_reference, "sg")
    sa_ref = get_group_vector_internal(truth_reference, "sa")
    gnss_lever_ref = get_group_vector_internal(truth_reference, "gnss_lever")

    ba_inst = get_group_instability_internal(truth_reference, "ba")
    bg_inst = get_group_instability_internal(truth_reference, "bg")
    sg_inst = get_group_instability_internal(truth_reference, "sg")
    sa_inst = get_group_instability_internal(truth_reference, "sa")

    if spec is None:
        ba_init = ba_ref
        bg_init = bg_ref
        ba_std = anchored_dynamic_std(ba_inst)
        bg_std = anchored_dynamic_std(bg_inst)
        ba_noise = anchored_dynamic_noise(ba_inst)
        bg_noise = anchored_dynamic_noise(bg_inst)
        case_type = "control"
        scale_meta = {
            "ba_p0_scale": 0.1,
            "bg_p0_scale": 0.1,
            "ba_q_scale": 0.1,
            "bg_q_scale": 0.1,
        }
    else:
        ba_init = np.zeros(3, dtype=float)
        bg_init = np.zeros(3, dtype=float)
        ba_std = free_bias_std(ba_ref, ba_inst, spec.ba_p0_scale)
        bg_std = free_bias_std(bg_ref, bg_inst, spec.bg_p0_scale)
        ba_noise = ba_inst * float(spec.ba_q_scale)
        bg_noise = bg_inst * float(spec.bg_q_scale)
        case_type = "free_bias"
        scale_meta = {
            "ba_p0_scale": float(spec.ba_p0_scale),
            "bg_p0_scale": float(spec.bg_p0_scale),
            "ba_q_scale": float(spec.ba_q_scale),
            "bg_q_scale": float(spec.bg_q_scale),
        }

    sg_std = anchored_dynamic_std(sg_inst)
    sa_std = anchored_dynamic_std(sa_inst)
    sg_noise = anchored_dynamic_noise(sg_inst)
    sa_noise = anchored_dynamic_noise(sa_inst)
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
    noise_cfg["markov_corr_time"] = 3600.0

    overrides = {
        "case_type": case_type,
        "case_id": case_id,
        "diag_output_path": rel_from_root(diag_path, REPO_ROOT),
        "ba_init_internal": [float(x) for x in ba_init],
        "bg_init_internal": [float(x) for x in bg_init],
        "ba_std_internal": [float(x) for x in ba_std],
        "bg_std_internal": [float(x) for x in bg_std],
        "ba_noise_internal": [float(x) for x in ba_noise],
        "bg_noise_internal": [float(x) for x in bg_noise],
        "sg_truth_internal": [float(x) for x in sg_ref],
        "sa_truth_internal": [float(x) for x in sa_ref],
        "gnss_lever_truth_internal_m": [float(x) for x in gnss_lever_ref],
        **scale_meta,
    }
    return cfg, overrides


def run_case(case_id: str, cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
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
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {case_id}")
    shutil.copy2(root_diag, diag_path)
    row: dict[str, Any] = {
        "case_id": case_id,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(diag_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(evaluate_navigation_metrics(cfg_path, sol_path))
    return row


def evaluate_bias_axes(case_row: dict[str, Any], truth_reference: dict[str, Any]) -> pd.DataFrame:
    state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    usecols = ["timestamp"] + [STATE_META[state_name]["csv"] for state_name in BIAS_STATE_ORDER]
    state_df = pd.read_csv(state_path, usecols=usecols)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)

    rows: list[dict[str, Any]] = []
    for state_name in BIAS_STATE_ORDER:
        csv_col = STATE_META[state_name]["csv"]
        values = state_df[csv_col].to_numpy(dtype=float)
        behavior_label, behavior_metrics = dynamic_behavior_label(
            state_name=state_name,
            truth_reference=truth_reference,
            timestamps=timestamps,
            values=values,
        )
        ref = truth_reference["states"][state_name]
        rows.append(
            {
                "case_id": case_row["case_id"],
                "state_name": state_name,
                "group": STATE_META[state_name]["group"],
                "axis": STATE_META[state_name]["axis"],
                "unit": STATE_META[state_name]["unit"],
                "behavior_label": behavior_label,
                "truth_source": ref["source"],
                "nominal_center": float(ref["reference_value"]),
                "instability": float(ref["instability"]),
                "mean_value": float(np.mean(values)),
                "std_value": float(np.std(values, ddof=0)),
                "p05_value": float(np.quantile(values, 0.05)),
                "p50_value": float(np.quantile(values, 0.50)),
                "p95_value": float(np.quantile(values, 0.95)),
                "range_value": float(np.max(values) - np.min(values)),
                "abs_mean_center_dev": float(abs(np.mean(values) - float(ref["reference_value"]))),
                **behavior_metrics,
            }
        )
    return pd.DataFrame(rows)


def build_case_summary(
    case_row: dict[str, Any],
    axis_df: pd.DataFrame,
    control_row: dict[str, Any],
    overrides: dict[str, Any],
) -> dict[str, Any]:
    impact, impact_metrics = impact_label(case_row, control_row)
    normal_count = int((axis_df["behavior_label"] == "normal").sum())
    borderline_count = int((axis_df["behavior_label"] == "borderline").sum())
    abnormal_count = int((axis_df["behavior_label"] == "abnormal").sum())
    bg_df = axis_df[axis_df["group"] == "bg"]
    ba_df = axis_df[axis_df["group"] == "ba"]
    worst_axis_row = axis_df.sort_values(
        by=["outside_ratio", "final_over_bound_ratio", "max_over_bound_ratio"],
        ascending=[False, False, False],
    ).iloc[0]
    if abnormal_count > 0 or impact == "abnormal":
        overall = "abnormal"
    elif borderline_count > 0 or impact == "borderline":
        overall = "borderline"
    else:
        overall = "normal"

    row = dict(case_row)
    row.update(overrides)
    row.update(impact_metrics)
    row.update(
        {
            "impact_label": impact,
            "overall_label": overall,
            "normal_axis_count": normal_count,
            "borderline_axis_count": borderline_count,
            "abnormal_axis_count": abnormal_count,
            "worst_outside_ratio": float(axis_df["outside_ratio"].max()),
            "mean_outside_ratio": float(axis_df["outside_ratio"].mean()),
            "worst_final_over_bound_ratio": float(axis_df["final_over_bound_ratio"].max()),
            "worst_max_over_bound_ratio": float(axis_df["max_over_bound_ratio"].max()),
            "bg_worst_outside_ratio": float(bg_df["outside_ratio"].max()),
            "ba_worst_outside_ratio": float(ba_df["outside_ratio"].max()),
            "bg_mean_outside_ratio": float(bg_df["outside_ratio"].mean()),
            "ba_mean_outside_ratio": float(ba_df["outside_ratio"].mean()),
            "worst_axis_name": str(worst_axis_row["state_name"]),
            "worst_axis_behavior": str(worst_axis_row["behavior_label"]),
            "worst_axis_final_value": float(worst_axis_row["final_value"]),
        }
    )
    return row


def rank_case_table(case_df: pd.DataFrame) -> pd.DataFrame:
    ranked = case_df.copy()
    ranked["impact_score"] = ranked["impact_label"].map(LABEL_SCORE).fillna(99).astype(int)
    return (
        ranked.sort_values(
            by=[
                "impact_score",
                "abnormal_axis_count",
                "borderline_axis_count",
                "mean_outside_ratio",
                "worst_outside_ratio",
                "delta_nav_rmse3d",
                "delta_nav_final3d",
            ],
            ascending=[True, True, True, True, True, True, True],
        )
        .drop(columns=["impact_score"])
        .reset_index(drop=True)
    )


def pick_reference_scale(scales: list[float], preferred: float = 1.0) -> float:
    if any(math.isclose(float(scale), preferred, rel_tol=0.0, abs_tol=1.0e-9) for scale in scales):
        return float(preferred)
    return float(scales[0])


def pick_reference_case(
    free_case_df: pd.DataFrame,
    ba_p0_scales: list[float],
    bg_p0_scales: list[float],
    ba_q_scales: list[float],
    bg_q_scales: list[float],
) -> pd.Series:
    ba_p0_ref = pick_reference_scale(ba_p0_scales)
    bg_p0_ref = pick_reference_scale(bg_p0_scales)
    ba_q_ref = pick_reference_scale(ba_q_scales)
    bg_q_ref = pick_reference_scale(bg_q_scales)
    mask = (
        np.isclose(free_case_df["ba_p0_scale"].astype(float), ba_p0_ref)
        & np.isclose(free_case_df["bg_p0_scale"].astype(float), bg_p0_ref)
        & np.isclose(free_case_df["ba_q_scale"].astype(float), ba_q_ref)
        & np.isclose(free_case_df["bg_q_scale"].astype(float), bg_q_ref)
    )
    if not mask.any():
        raise RuntimeError(
            "failed to locate reference free-bias case "
            f"(ba_p0={ba_p0_ref}, bg_p0={bg_p0_ref}, ba_q={ba_q_ref}, bg_q={bg_q_ref})"
        )
    return free_case_df.loc[mask].iloc[0]


def write_summary(
    output_path: Path,
    control_row: pd.Series,
    baseline_row: pd.Series,
    ranked_df: pd.DataFrame,
    axis_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    acceptable_df = ranked_df.loc[ranked_df["impact_label"] != "abnormal"].copy()
    best_row = acceptable_df.iloc[0] if not acceptable_df.empty else ranked_df.iloc[0]
    best_axes = axis_df.loc[axis_df["case_id"] == best_row["case_id"]].sort_values(
        by=["outside_ratio", "final_over_bound_ratio"],
        ascending=[False, False],
    )
    baseline_axes = axis_df.loc[axis_df["case_id"] == baseline_row["case_id"]].sort_values(
        by=["outside_ratio", "final_over_bound_ratio"],
        ascending=[False, False],
    )
    aggressive_df = (
        ranked_df.loc[ranked_df["impact_label"] == "abnormal"]
        .sort_values(
            by=[
                "abnormal_axis_count",
                "borderline_axis_count",
                "mean_outside_ratio",
                "worst_outside_ratio",
            ],
            ascending=[True, True, True, True],
        )
        .reset_index(drop=True)
    )
    min_abnormal = int(ranked_df["abnormal_axis_count"].min())

    lines: list[str] = [
        "# data2 pure INS/GNSS bias range sweep summary",
        "",
        "## 1. 实验设置",
        "- pipeline: `pure INS/GNSS`, 显式关闭 `ODO/NHC`，不涉及 `odo_scale` 和 mounting 参数调节。",
        "- truth-anchored states: `gnss_lever(28:30)`、`sg(15:17)`、`sa(18:20)`；它们采用真值初值 + 小 `P0/Q`。",
        "- tuned states: `ba(9:11)`、`bg(12:14)`；它们从零初值释放，通过 `P0/Q` 尺度扫描检查估计均值与波动范围。",
        "- normal-range criterion: 继续沿用仓库现有 `dynamic_behavior_label`，以 `docs/notes/初始姿态和参数.md` 的名义中心值 + `3σ * sqrt(1-exp(-2t/tau))` 波动带判断 `outside_ratio / final_over_bound_ratio`，而不是要求收敛到单个固定真值点。",
        "",
        "## 2. 控制组与基线自由 bias case",
        (
            f"- control `{control_row['case_id']}`: "
            f"`nav_rmse_3d_m={format_metric(float(control_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(control_row['nav_final_err_3d_m']))}`。"
        ),
        (
            f"- baseline free-bias `{baseline_row['case_id']}`: "
            f"`impact={baseline_row['impact_label']}`, "
            f"`overall={baseline_row['overall_label']}`, "
            f"`abnormal_axes={int(baseline_row['abnormal_axis_count'])}`, "
            f"`worst_outside_ratio={format_metric(float(baseline_row['worst_outside_ratio']))}`, "
            f"`delta_nav_rmse3d={format_metric(float(baseline_row['delta_nav_rmse3d']))}`。"
        ),
        "",
        "## 3. 推荐候选（先过导航影响门槛）",
    ]

    for _, row in ranked_df.head(3).iterrows():
        lines.append(
            f"- `{row['case_id']}`: impact=`{row['impact_label']}`, overall=`{row['overall_label']}`, "
            f"`abnormal/borderline/normal={int(row['abnormal_axis_count'])}/{int(row['borderline_axis_count'])}/{int(row['normal_axis_count'])}`, "
            f"`worst_outside_ratio={format_metric(float(row['worst_outside_ratio']))}`, "
            f"`bg_mean_outside_ratio={format_metric(float(row['bg_mean_outside_ratio']))}`, "
            f"`ba_mean_outside_ratio={format_metric(float(row['ba_mean_outside_ratio']))}`, "
            f"`delta_nav_rmse3d={format_metric(float(row['delta_nav_rmse3d']))}`, "
            f"`delta_nav_final3d={format_metric(float(row['delta_nav_final3d']))}`。"
        )

    lines.extend(["", "## 4. 当前 best case 的最难轴"])
    for _, row in best_axes.head(4).iterrows():
        lines.append(
            f"- `{row['state_name']}`: behavior=`{row['behavior_label']}`, "
            f"`mean={format_metric(float(row['mean_value']))} {row['unit']}`, "
            f"`p05/p95={format_metric(float(row['p05_value']))}/{format_metric(float(row['p95_value']))} {row['unit']}`, "
            f"`outside_ratio={format_metric(float(row['outside_ratio']))}`, "
            f"`final_over_bound_ratio={format_metric(float(row['final_over_bound_ratio']))}`。"
        )

    lines.extend(["", "## 5. baseline free-bias 最难轴"])
    for _, row in baseline_axes.head(4).iterrows():
        lines.append(
            f"- `{row['state_name']}`: behavior=`{row['behavior_label']}`, "
            f"`mean={format_metric(float(row['mean_value']))} {row['unit']}`, "
            f"`p05/p95={format_metric(float(row['p05_value']))}/{format_metric(float(row['p95_value']))} {row['unit']}`, "
            f"`outside_ratio={format_metric(float(row['outside_ratio']))}`, "
            f"`final_over_bound_ratio={format_metric(float(row['final_over_bound_ratio']))}`。"
        )

    if not aggressive_df.empty:
        aggressive_row = aggressive_df.iloc[0]
        lines.extend(
            [
                "",
                "## 6. 不推荐的激进候选",
                (
                    f"- `{aggressive_row['case_id']}` 在 behavior 指标上略有改善，"
                    f"但 `impact={aggressive_row['impact_label']}`，"
                    f"`delta_nav_rmse3d={format_metric(float(aggressive_row['delta_nav_rmse3d']))}`, "
                    f"`delta_nav_final3d={format_metric(float(aggressive_row['delta_nav_final3d']))}`，"
                    "因此不作为调参推荐。"
                ),
            ]
        )

    lines.extend(
        [
            "",
            "## 7. 结论草案",
            (
                f"- 当前 sweep 中最少仍有 `{min_abnormal}` 个 abnormal axes，"
                "说明这一轮参数扫描仍不足以把 free `ba/bg` 全部拉回现有 normal-range 判据内。"
            ),
            (
                f"- 当前 ranking 第一名是 `{best_row['case_id']}`，"
                f"它满足 `impact={best_row['impact_label']}`，"
                f"`abnormal_axes={int(best_row['abnormal_axis_count'])}`，"
                f"`worst_outside_ratio={format_metric(float(best_row['worst_outside_ratio']))}`。"
            ),
            (
                f"- 相比 baseline free-bias case，best case 的 "
                f"`mean_outside_ratio` 从 "
                f"`{format_metric(float(baseline_row['mean_outside_ratio']))}` "
                f"变到 `{format_metric(float(best_row['mean_outside_ratio']))}`，"
                f"`delta_nav_rmse3d` 从 `{format_metric(float(baseline_row['delta_nav_rmse3d']))}` "
                f"变到 `{format_metric(float(best_row['delta_nav_rmse3d']))}`。"
            ),
            "- 当前参数判断必须同时看 behavior 与 navigation impact，不能只按 `outside_ratio` 或单一 axis 的表面改善排序。",
            "- 若后续还需要继续调参，优先围绕当前 top case 再做 `ba/bg P0` 细扫，而不是重新放开 `sg/sa` 或 `gnss_lever`。",
            "",
            "## Coverage",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- bias_axis_metrics: `{manifest['bias_axis_metrics_csv']}`",
            f"- truth_reference: `{manifest['truth_reference_json']}`",
            f"- generated_at: `{manifest['generated_at']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep ba/bg bias noise settings under pure INS/GNSS with truth-anchored GNSS lever and sg/sa."
    )
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--ba-q-scales", default="0.25,1,4")
    parser.add_argument("--bg-q-scales", default="0.25,1,4")
    parser.add_argument("--ba-p0-scales", default="1")
    parser.add_argument("--bg-p0-scales", default="1")
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.ba_q_scales = parse_scale_list(str(args.ba_q_scales))
    args.bg_q_scales = parse_scale_list(str(args.bg_q_scales))
    args.ba_p0_scales = parse_scale_list(str(args.ba_p0_scales))
    args.bg_p0_scales = parse_scale_list(str(args.bg_p0_scales))
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    case_rows: list[dict[str, Any]] = []
    axis_rows: list[pd.DataFrame] = []

    control_dir = args.case_root / CONTROL_CASE_ID
    ensure_dir(control_dir)
    control_cfg, control_overrides = build_case_config(
        base_cfg=base_cfg,
        truth_reference=truth_reference,
        case_dir=control_dir,
        case_id=CONTROL_CASE_ID,
        spec=None,
    )
    control_cfg_path = control_dir / f"config_{CONTROL_CASE_ID}.yaml"
    save_yaml(control_cfg, control_cfg_path)
    control_case_row = run_case(CONTROL_CASE_ID, control_cfg_path, control_dir, args.exe)
    control_axis_df = evaluate_bias_axes(control_case_row, truth_reference)
    control_summary = build_case_summary(control_case_row, control_axis_df, control_case_row, control_overrides)
    case_rows.append(control_summary)
    axis_rows.append(control_axis_df)

    specs: list[CaseSpec] = []
    for ba_p0_scale in args.ba_p0_scales:
        for bg_p0_scale in args.bg_p0_scales:
            for ba_q_scale in args.ba_q_scales:
                for bg_q_scale in args.bg_q_scales:
                    specs.append(
                        CaseSpec(
                            ba_p0_scale=float(ba_p0_scale),
                            bg_p0_scale=float(bg_p0_scale),
                            ba_q_scale=float(ba_q_scale),
                            bg_q_scale=float(bg_q_scale),
                        )
                    )

    for spec in specs:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg, overrides = build_case_config(
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            case_dir=case_dir,
            case_id=spec.case_id,
            spec=spec,
        )
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)
        case_row = run_case(spec.case_id, cfg_path, case_dir, args.exe)
        axis_df = evaluate_bias_axes(case_row, truth_reference)
        summary_row = build_case_summary(case_row, axis_df, control_case_row, overrides)
        case_rows.append(summary_row)
        axis_rows.append(axis_df)

    case_metrics_df = pd.DataFrame(case_rows)
    bias_axis_df = pd.concat(axis_rows, ignore_index=True)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    bias_axis_metrics_path = args.output_dir / "bias_axis_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    bias_axis_df.to_csv(bias_axis_metrics_path, index=False, encoding="utf-8-sig")

    free_case_df = case_metrics_df.loc[case_metrics_df["case_type"] == "free_bias"].copy()
    ranked_df = rank_case_table(free_case_df)
    ranked_path = args.output_dir / "ranked_case_metrics.csv"
    ranked_df.to_csv(ranked_path, index=False, encoding="utf-8-sig")

    baseline_row = pick_reference_case(
        free_case_df=free_case_df,
        ba_p0_scales=args.ba_p0_scales,
        bg_p0_scales=args.bg_p0_scales,
        ba_q_scales=args.ba_q_scales,
        bg_q_scales=args.bg_q_scales,
    )

    manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "exp_id": args.exp_id,
        "mode": "pure_ins_gnss_bias_range_sweep",
        "assumption": "scale factors refer to sg/sa only; odo_scale and mounting are excluded in pure INS/GNSS mode",
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "exe": rel_from_root(args.exe, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "bias_axis_metrics_csv": rel_from_root(bias_axis_metrics_path, REPO_ROOT),
        "ranked_case_metrics_csv": rel_from_root(ranked_path, REPO_ROOT),
        "control_case_id": CONTROL_CASE_ID,
        "free_case_count": int(len(free_case_df)),
        "ba_q_scales": args.ba_q_scales,
        "bg_q_scales": args.bg_q_scales,
        "ba_p0_scales": args.ba_p0_scales,
        "bg_p0_scales": args.bg_p0_scales,
        "reference_case_id": str(baseline_row["case_id"]),
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    summary_path = args.output_dir / "summary.md"
    write_summary(
        output_path=summary_path,
        control_row=case_metrics_df.loc[case_metrics_df["case_id"] == CONTROL_CASE_ID].iloc[0],
        baseline_row=baseline_row,
        ranked_df=ranked_df,
        axis_df=bias_axis_df,
        manifest=manifest,
    )


if __name__ == "__main__":
    main()
