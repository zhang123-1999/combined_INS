from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_bias_scale_zero_init_pos320 import (
    ACTIVITY_CEILING_SCALE_DEFAULT,
    ACTIVITY_FLOOR_SCALE_DEFAULT,
    IMU_GROUP_ORDER,
    build_case_config,
    evaluate_imu_states,
    run_case,
)
from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import (
    IMU_GROUP_COLUMNS,
    modeled_truth_reference_series,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    format_metric,
    json_safe,
    reset_directory,
)


EXP_ID_DEFAULT = "EXP-20260321-data2-ins-gnss-bias-scale-initial-noise-scan-r2"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_bias_scale_initial_noise_scan_r2")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
README_PATH_DEFAULT = Path("dataset/data2/README.md")
IMU_MODEL_DEFAULT = "POS-320"
IMU_P0_SCALES_DEFAULT = "0.5,1.0,1.5"
Y_PRIOR_SCALES_DEFAULT = "0.05,0.10,0.25"


@dataclass(frozen=True)
class CaseSpec:
    imu_p0_scale: float
    y_prior_scale: float

    @property
    def case_id(self) -> str:
        return f"p0s{scale_slug(self.imu_p0_scale)}_ys{scale_slug(self.y_prior_scale)}"


def scale_slug(value: float) -> str:
    return f"{value:g}".replace("-", "m").replace(".", "p")


def parse_scale_list(raw: str) -> list[float]:
    values = [item.strip() for item in str(raw).split(",") if item.strip()]
    if not values:
        raise ValueError("scale list must not be empty")
    return [float(item) for item in values]


def summarize_case(case_row: dict[str, Any], imu_state_df: pd.DataFrame) -> dict[str, Any]:
    label_counts = imu_state_df["behavior_label"].value_counts()
    activity_counts = imu_state_df["activity_label"].value_counts()
    y_df = imu_state_df.loc[imu_state_df["axis"] == "y"].copy()
    worst_row = imu_state_df.sort_values(
        by=["final_over_bound_ratio", "outside_ratio", "range_value"],
        ascending=[False, False, False],
    ).iloc[0]
    worst_y_row = y_df.sort_values(
        by=["final_over_bound_ratio", "outside_ratio", "range_value"],
        ascending=[False, False, False],
    ).iloc[0]
    row = dict(case_row)
    row.update(
        {
            "normal_axis_count": int(label_counts.get("normal", 0)),
            "borderline_axis_count": int(label_counts.get("borderline", 0)),
            "abnormal_axis_count": int(label_counts.get("abnormal", 0)),
            "activity_sane_axis_count": int(activity_counts.get("range_sane", 0)),
            "activity_low_axis_count": int(activity_counts.get("too_low", 0)),
            "activity_high_axis_count": int(activity_counts.get("too_high", 0)),
            "mean_outside_ratio": float(imu_state_df["outside_ratio"].mean()),
            "worst_outside_ratio": float(imu_state_df["outside_ratio"].max()),
            "mean_final_over_bound_ratio": float(imu_state_df["final_over_bound_ratio"].mean()),
            "worst_final_over_bound_ratio": float(imu_state_df["final_over_bound_ratio"].max()),
            "mean_activity_penalty": float(imu_state_df["activity_penalty"].mean()),
            "worst_activity_penalty": float(imu_state_df["activity_penalty"].max()),
            "mean_activity_ratio": float(imu_state_df["activity_range_ratio"].mean()),
            "y_mean_final_over_bound_ratio": float(y_df["final_over_bound_ratio"].mean()),
            "y_worst_final_over_bound_ratio": float(y_df["final_over_bound_ratio"].max()),
            "y_mean_range_value": float(y_df["range_value"].mean()),
            "y_mean_activity_penalty": float(y_df["activity_penalty"].mean()),
            "y_worst_activity_penalty": float(y_df["activity_penalty"].max()),
            "y_mean_activity_ratio": float(y_df["activity_range_ratio"].mean()),
            "y_activity_low_axis_count": int((y_df["activity_label"] == "too_low").sum()),
            "y_activity_high_axis_count": int((y_df["activity_label"] == "too_high").sum()),
            "worst_state_name": str(worst_row["state_name"]),
            "worst_state_label": str(worst_row["behavior_label"]),
            "worst_state_final_ratio": float(worst_row["final_over_bound_ratio"]),
            "worst_y_state_name": str(worst_y_row["state_name"]),
            "worst_y_state_label": str(worst_y_row["behavior_label"]),
            "worst_y_state_final_ratio": float(worst_y_row["final_over_bound_ratio"]),
        }
    )
    return row


def rank_nav_cases(case_df: pd.DataFrame) -> pd.DataFrame:
    return case_df.sort_values(
        by=[
            "nav_rmse_3d_m",
            "nav_final_err_3d_m",
            "y_worst_final_over_bound_ratio",
            "mean_final_over_bound_ratio",
            "worst_final_over_bound_ratio",
        ],
        ascending=[True, True, True, True, True],
    ).reset_index(drop=True)


def choose_balanced_case(ranked_df: pd.DataFrame) -> pd.Series:
    best_nav = ranked_df.iloc[0]
    rmse_threshold = float(best_nav["nav_rmse_3d_m"]) * 1.03
    final_threshold = max(float(best_nav["nav_final_err_3d_m"]) * 1.25, float(best_nav["nav_final_err_3d_m"]) + 1.0e-4)
    candidate_df = ranked_df.loc[
        (ranked_df["nav_rmse_3d_m"] <= rmse_threshold)
        & (ranked_df["nav_final_err_3d_m"] <= final_threshold)
    ].copy()
    if candidate_df.empty:
        candidate_df = ranked_df.copy()
    return (
        candidate_df.sort_values(
            by=[
                "y_mean_activity_penalty",
                "mean_activity_penalty",
                "y_activity_low_axis_count",
                "activity_low_axis_count",
                "y_worst_final_over_bound_ratio",
                "mean_final_over_bound_ratio",
                "worst_final_over_bound_ratio",
                "nav_rmse_3d_m",
                "nav_final_err_3d_m",
            ],
            ascending=[True, True, True, True, True, True, True, True, True],
        )
        .iloc[0]
    )


def pick_reference_case(case_df: pd.DataFrame, imu_p0_scale: float, y_prior_scale: float) -> pd.Series:
    mask = (
        case_df["imu_p0_scale"].astype(float).sub(float(imu_p0_scale)).abs() <= 1.0e-12
    ) & (
        case_df["y_prior_scale"].astype(float).sub(float(y_prior_scale)).abs() <= 1.0e-12
    )
    if not mask.any():
        raise RuntimeError(f"missing reference case for imu_p0_scale={imu_p0_scale}, y_prior_scale={y_prior_scale}")
    return case_df.loc[mask].iloc[0]


def boundary_flag(value: float, grid: list[float]) -> str:
    if not grid:
        return "unknown"
    if math.isclose(float(value), min(grid), rel_tol=0.0, abs_tol=1.0e-12):
        return "lower_boundary"
    if math.isclose(float(value), max(grid), rel_tol=0.0, abs_tol=1.0e-12):
        return "upper_boundary"
    return "interior"


def write_summary(
    output_path: Path,
    ranked_df: pd.DataFrame,
    imu_state_df: pd.DataFrame,
    reference_row: pd.Series,
    best_nav_row: pd.Series,
    recommended_row: pd.Series,
    manifest: dict[str, Any],
) -> None:
    ref_axes = imu_state_df.loc[imu_state_df["case_id"] == reference_row["case_id"]].sort_values(
        by=["final_over_bound_ratio", "outside_ratio", "range_value"],
        ascending=[False, False, False],
    )
    rec_axes = imu_state_df.loc[imu_state_df["case_id"] == recommended_row["case_id"]].sort_values(
        by=["final_over_bound_ratio", "outside_ratio", "range_value"],
        ascending=[False, False, False],
    )
    delta_rmse = float(recommended_row["nav_rmse_3d_m"]) - float(reference_row["nav_rmse_3d_m"])
    delta_final = float(recommended_row["nav_final_err_3d_m"]) - float(reference_row["nav_final_err_3d_m"])
    lines: list[str] = [
        "# data2 INS/GNSS bias-scale initial-noise scan summary",
        "",
        "## 1. 扫描设置",
        "- pipeline: `INS/GNSS only`，固定 `gnss_lever(28:30)`，只扫描 `ba/bg/sg/sa` 的 zero-init 初始噪声。",
        (
            f"- grid: `imu_p0_scale ∈ {manifest['imu_p0_scales']}`，"
            f"`y_prior_scale ∈ {manifest['y_prior_scales']}`，共 `{manifest['case_count']}` 个 fresh case。"
        ),
        (
            f"- activity window: 对每个 `ba/bg/sg/sa` 轴，先用 `POS-320` modeled reference envelope span "
            f"`max(upper)-min(lower)` 定义参考活动量，再用 "
            f"`[{manifest['activity_floor_scale']:g}x, {manifest['activity_ceiling_scale']:g}x]` "
            "作为宽松工程窗口；低于下界记为 `too_low`（prior 压得过死），高于上界记为 `too_high`。"
        ),
        "- ranking rule: `best-nav` 仍按 `nav_rmse_3d/nav_final_err_3d` 选；`balanced` 推荐则限制在 near-best-nav 集合内，优先最小化 `Y` 轴与全体状态的 activity penalty，再看 `final_over_bound_ratio`，避免把“越小越好”的僵硬解误判为优解。",
        "",
        "## 2. 参考点 / 最优点",
        (
            f"- reference `{reference_row['case_id']}`: "
            f"`imu_p0_scale={reference_row['imu_p0_scale']:g}`, "
            f"`y_prior_scale={reference_row['y_prior_scale']:g}`, "
            f"`nav_rmse_3d_m={format_metric(float(reference_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(reference_row['nav_final_err_3d_m']))}`, "
            f"`y_mean_activity_penalty={format_metric(float(reference_row['y_mean_activity_penalty']))}`, "
            f"`activity_low={int(reference_row['activity_low_axis_count'])}`。"
        ),
        (
            f"- best-nav `{best_nav_row['case_id']}`: "
            f"`imu_p0_scale={best_nav_row['imu_p0_scale']:g}`, "
            f"`y_prior_scale={best_nav_row['y_prior_scale']:g}`, "
            f"`nav_rmse_3d_m={format_metric(float(best_nav_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(best_nav_row['nav_final_err_3d_m']))}`, "
            f"`y_mean_activity_penalty={format_metric(float(best_nav_row['y_mean_activity_penalty']))}`, "
            f"`activity_low={int(best_nav_row['activity_low_axis_count'])}`。"
        ),
        (
            f"- recommended `{recommended_row['case_id']}`: "
            f"`imu_p0_scale={recommended_row['imu_p0_scale']:g}`, "
            f"`y_prior_scale={recommended_row['y_prior_scale']:g}`, "
            f"`nav_rmse_3d_m={format_metric(float(recommended_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(recommended_row['nav_final_err_3d_m']))}`, "
            f"`y_mean_activity_penalty={format_metric(float(recommended_row['y_mean_activity_penalty']))}`, "
            f"`activity_low={int(recommended_row['activity_low_axis_count'])}`, "
            f"`activity_high={int(recommended_row['activity_high_axis_count'])}`, "
            f"`y_worst_final_ratio={format_metric(float(recommended_row['y_worst_final_over_bound_ratio']))}`, "
            f"`mean_final_ratio={format_metric(float(recommended_row['mean_final_over_bound_ratio']))}`。"
        ),
        (
            f"- recommended vs reference: `delta_nav_rmse_3d_m={format_metric(delta_rmse)}`, "
            f"`delta_nav_final_err_3d_m={format_metric(delta_final)}`。"
        ),
        "",
        "## 3. 排名前五",
    ]

    for _, row in ranked_df.head(5).iterrows():
        lines.append(
            f"- `{row['case_id']}`: "
            f"`p0={row['imu_p0_scale']:g}`, `y={row['y_prior_scale']:g}`, "
            f"`rmse3d={format_metric(float(row['nav_rmse_3d_m']))}`, "
            f"`final3d={format_metric(float(row['nav_final_err_3d_m']))}`, "
            f"`y_activity_pen={format_metric(float(row['y_mean_activity_penalty']))}`, "
            f"`low/high={int(row['activity_low_axis_count'])}/{int(row['activity_high_axis_count'])}`, "
            f"`y_worst_final_ratio={format_metric(float(row['y_worst_final_over_bound_ratio']))}`, "
            f"`mean_final_ratio={format_metric(float(row['mean_final_over_bound_ratio']))}`。"
        )

    lines.extend(["", "## 4. recommended case 的最难轴"])
    for _, row in rec_axes.head(6).iterrows():
        lines.append(
            f"- `{row['state_name']}`: "
            f"`label={row['behavior_label']}`, "
            f"`range={format_metric(float(row['range_value']))} {row['unit']}`, "
            f"`activity={row['activity_label']}`, "
            f"`activity_ratio={format_metric(float(row['activity_range_ratio']))}`, "
            f"`outside_ratio={format_metric(float(row['outside_ratio']))}`, "
            f"`final_ratio={format_metric(float(row['final_over_bound_ratio']))}`。"
        )

    lines.extend(["", "## 5. reference case 的最难轴"])
    for _, row in ref_axes.head(6).iterrows():
        lines.append(
            f"- `{row['state_name']}`: "
            f"`label={row['behavior_label']}`, "
            f"`range={format_metric(float(row['range_value']))} {row['unit']}`, "
            f"`activity={row['activity_label']}`, "
            f"`activity_ratio={format_metric(float(row['activity_range_ratio']))}`, "
            f"`outside_ratio={format_metric(float(row['outside_ratio']))}`, "
            f"`final_ratio={format_metric(float(row['final_over_bound_ratio']))}`。"
        )

    lines.extend(
        [
            "",
            "## 6. 结论草案",
            (
                f"- 若按导航主排序，当前小范围扫描最优点是 "
                f"`imu_p0_scale={best_nav_row['imu_p0_scale']:g}`, "
                f"`y_prior_scale={best_nav_row['y_prior_scale']:g}`。"
            ),
            (
                f"- 若要求导航几乎不退化，同时避免 `ba/bg/sg/sa` 被 prior 压到过低，"
                f"推荐点是 `imu_p0_scale={recommended_row['imu_p0_scale']:g}`, "
                f"`y_prior_scale={recommended_row['y_prior_scale']:g}`。"
            ),
            (
                f"- 本轮 recommended case 仍有 `activity_low_axis_count={int(recommended_row['activity_low_axis_count'])}`，"
                f"`activity_high_axis_count={int(recommended_row['activity_high_axis_count'])}`；"
                "若 low-count 仍偏多，说明当前网格里还没有真正同时满足“导航 + 物理活动量”双目标的解。"
            ),
            (
                f"- recommended 点在 `imu_p0_scale` 上属于 `{manifest['recommended_p0_boundary']}`，"
                f"`y_prior_scale` 上属于 `{manifest['recommended_y_boundary']}`；"
                "若命中边界，下一轮细扫应向对应方向外扩。"
            ),
            "",
            "## Coverage",
            f"- summary: `{manifest['summary_path']}`",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- imu_state_metrics: `{manifest['imu_state_metrics_csv']}`",
            f"- ranked_case_metrics: `{manifest['ranked_case_metrics_csv']}`",
            f"- candidate_selection: `{manifest['candidate_selection_csv']}`",
            f"- truth_reference: `{manifest['truth_reference_json']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a local initial-noise scan for data2 INS/GNSS zero-init ba/bg/sg/sa under POS-320 modeling."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--readme-path", type=Path, default=README_PATH_DEFAULT)
    parser.add_argument("--imu-model", type=str, default=IMU_MODEL_DEFAULT)
    parser.add_argument("--exp-id", type=str, default=EXP_ID_DEFAULT)
    parser.add_argument("--imu-p0-scales", type=str, default=IMU_P0_SCALES_DEFAULT)
    parser.add_argument("--y-prior-scales", type=str, default=Y_PRIOR_SCALES_DEFAULT)
    parser.add_argument("--activity-floor-scale", type=float, default=ACTIVITY_FLOOR_SCALE_DEFAULT)
    parser.add_argument("--activity-ceiling-scale", type=float, default=ACTIVITY_CEILING_SCALE_DEFAULT)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()
    args.imu_p0_scales = parse_scale_list(args.imu_p0_scales)
    args.y_prior_scales = parse_scale_list(args.y_prior_scales)
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")
    if not args.readme_path.exists():
        raise FileNotFoundError(f"missing readme path: {args.readme_path}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg, readme_path=args.readme_path, extrinsic_imu_model=args.imu_model)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)

    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    case_rows: list[dict[str, Any]] = []
    imu_rows: list[pd.DataFrame] = []

    specs = [
        CaseSpec(imu_p0_scale=float(imu_p0_scale), y_prior_scale=float(y_prior_scale))
        for imu_p0_scale in args.imu_p0_scales
        for y_prior_scale in args.y_prior_scales
    ]

    for index, spec in enumerate(specs, start=1):
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg, metadata = build_case_config(
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            readme_params=readme_params,
            case_dir=case_dir,
            imu_p0_scale=spec.imu_p0_scale,
            y_prior_scale=spec.y_prior_scale,
        )
        metadata["case_id"] = spec.case_id
        metadata["scan_case_id"] = spec.case_id
        metadata["scan_case_index"] = index
        metadata["scan_case_count"] = len(specs)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        case_row = run_case(cfg_path, case_dir, args.exe)
        case_row["case_id"] = spec.case_id
        case_row.update(metadata)
        case_row["config_path"] = rel_from_root(cfg_path, REPO_ROOT)

        state_series_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        usecols = ["timestamp"]
        for group in IMU_GROUP_ORDER:
            usecols.extend(IMU_GROUP_COLUMNS[group])
        state_df = pd.read_csv(state_series_path, usecols=usecols)
        modeled_truth_df = modeled_truth_reference_series(
            state_df["timestamp"].to_numpy(dtype=float),
            truth_reference,
            readme_params,
        )
        imu_state_df = evaluate_imu_states(
            state_df,
            modeled_truth_df,
            readme_params,
            activity_floor_scale=float(args.activity_floor_scale),
            activity_ceiling_scale=float(args.activity_ceiling_scale),
        )
        imu_state_df.insert(0, "case_id", spec.case_id)
        case_rows.append(summarize_case(case_row, imu_state_df))
        imu_rows.append(imu_state_df)

    case_metrics_df = pd.DataFrame(case_rows)
    imu_state_metrics_df = pd.concat(imu_rows, ignore_index=True)
    ranked_df = rank_nav_cases(case_metrics_df)

    reference_row = pick_reference_case(case_metrics_df, imu_p0_scale=1.0, y_prior_scale=0.10)
    best_nav_row = ranked_df.iloc[0]
    recommended_row = choose_balanced_case(ranked_df)

    candidate_df = ranked_df.copy()
    candidate_df.insert(0, "rank_nav", range(1, len(candidate_df) + 1))
    candidate_df["is_reference"] = candidate_df["case_id"] == str(reference_row["case_id"])
    candidate_df["is_best_nav"] = candidate_df["case_id"] == str(best_nav_row["case_id"])
    candidate_df["is_recommended"] = candidate_df["case_id"] == str(recommended_row["case_id"])

    case_metrics_path = args.output_dir / "case_metrics.csv"
    imu_state_metrics_path = args.output_dir / "imu_state_metrics.csv"
    ranked_path = args.output_dir / "ranked_case_metrics.csv"
    candidate_path = args.output_dir / "candidate_selection.csv"
    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"

    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    imu_state_metrics_df.to_csv(imu_state_metrics_path, index=False, encoding="utf-8-sig")
    ranked_df.to_csv(ranked_path, index=False, encoding="utf-8-sig")
    candidate_df.to_csv(candidate_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "readme_path": rel_from_root(args.readme_path, REPO_ROOT),
        "imu_model": args.imu_model,
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "imu_state_metrics_csv": rel_from_root(imu_state_metrics_path, REPO_ROOT),
        "ranked_case_metrics_csv": rel_from_root(ranked_path, REPO_ROOT),
        "candidate_selection_csv": rel_from_root(candidate_path, REPO_ROOT),
        "summary_path": rel_from_root(summary_path, REPO_ROOT),
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        "imu_p0_scales": args.imu_p0_scales,
        "y_prior_scales": args.y_prior_scales,
        "case_count": len(specs),
        "reference_case_id": str(reference_row["case_id"]),
        "best_nav_case_id": str(best_nav_row["case_id"]),
        "recommended_case_id": str(recommended_row["case_id"]),
        "recommended_p0_boundary": boundary_flag(float(recommended_row["imu_p0_scale"]), args.imu_p0_scales),
        "recommended_y_boundary": boundary_flag(float(recommended_row["y_prior_scale"]), args.y_prior_scales),
        "activity_floor_scale": float(args.activity_floor_scale),
        "activity_ceiling_scale": float(args.activity_ceiling_scale),
        "readme_imu_params": readme_params,
    }
    write_summary(
        output_path=summary_path,
        ranked_df=ranked_df,
        imu_state_df=imu_state_metrics_df,
        reference_row=reference_row,
        best_nav_row=best_nav_row,
        recommended_row=recommended_row,
        manifest=manifest,
    )
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"[done] wrote summary: {summary_path}")
    print(f"[done] wrote case metrics: {case_metrics_path}")
    print(f"[done] wrote imu state metrics: {imu_state_metrics_path}")
    print(f"[done] wrote ranked case metrics: {ranked_path}")
    print(f"[done] wrote candidate selection: {candidate_path}")


if __name__ == "__main__":
    main()
