from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.append(str(REPO_ROOT / "scripts/analysis"))

import interactive_nav_report as nav_report  # noqa: E402

DEFAULT_OUTPUT_DIR = REPO_ROOT / "output/review/EXP-20260310-inekf-mechanism-r1"
DEFAULT_TARGET_POINTS = 10_000_000
DIFF_EPS = 1e-6


@dataclass(frozen=True)
class MechanismCase:
    case_id: str
    label: str
    sol_path: str
    config_path: str
    dataset: str
    method: str


CASES: tuple[MechanismCase, ...] = (
    MechanismCase(
        case_id="data4_gnss30_eskf",
        label="data4 GNSS30 ESKF",
        sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_eskf_doc.txt",
        config_path="output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml",
        dataset="data4",
        method="eskf",
    ),
    MechanismCase(
        case_id="data4_gnss30_true_full",
        label="data4 true_iekf full",
        sol_path="output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_full.txt",
        config_path="output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_full.yaml",
        dataset="data4",
        method="true_iekf",
    ),
    MechanismCase(
        case_id="data4_gnss30_true_freeze_bg",
        label="data4 freeze_bg",
        sol_path="output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_freeze_bg.txt",
        config_path="output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_freeze_bg.yaml",
        dataset="data4",
        method="freeze_bg",
    ),
    MechanismCase(
        case_id="data4_gnss30_true_freeze_mount",
        label="data4 freeze_mount",
        sol_path="output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_freeze_mount.txt",
        config_path="output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_freeze_mount.yaml",
        dataset="data4",
        method="freeze_mount",
    ),
    MechanismCase(
        case_id="data2_gnss30_eskf",
        label="data2 GNSS30 ESKF 参考",
        sol_path="output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_eskf_ref.txt",
        config_path="output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml",
        dataset="data2",
        method="eskf",
    ),
    MechanismCase(
        case_id="data2_gnss30_true_full",
        label="data2 true_iekf full",
        sol_path="output/review/20260306-phase2c-bg-freeze/SOL_full.txt",
        config_path="output/review/20260306-phase2c-bg-freeze/cfg_full.yaml",
        dataset="data2",
        method="true_iekf",
    ),
    MechanismCase(
        case_id="data2_gnss30_true_freeze_bg",
        label="data2 freeze_bg",
        sol_path="output/review/20260306-phase2c-bg-freeze/SOL_freeze_bg.txt",
        config_path="output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg.yaml",
        dataset="data2",
        method="freeze_bg",
    ),
    MechanismCase(
        case_id="data2_gnss30_true_freeze_mount",
        label="data2 freeze_mount",
        sol_path="output/review/20260306-phase2c-bg-freeze/SOL_freeze_mount.txt",
        config_path="output/review/20260306-phase2c-bg-freeze/cfg_freeze_mount.yaml",
        dataset="data2",
        method="freeze_mount",
    ),
)


def compute_monotonicity_ratio(values: np.ndarray) -> tuple[float, int]:
    diffs = np.diff(values)
    if diffs.size == 0:
        return float("nan"), 0
    mask = np.abs(diffs) > DIFF_EPS
    diffs = diffs[mask]
    if diffs.size == 0:
        return float("nan"), 0
    signs = np.sign(diffs)
    pos = int(np.sum(signs > 0))
    neg = int(np.sum(signs < 0))
    total = pos + neg
    if total == 0:
        return float("nan"), 0
    ratio = max(pos, neg) / total
    flips = int(np.sum(signs[1:] * signs[:-1] < 0)) if signs.size > 1 else 0
    return ratio, flips


def compute_heading_metrics(
    time_s: np.ndarray,
    heading_deg: np.ndarray,
    mono_step_s: float = 1.0,
) -> dict[str, float]:
    if time_s.size < 2:
        return {
            "heading_slope_deg_per_ks": float("nan"),
            "heading_monotonicity_ratio": float("nan"),
            "heading_sign_flip_count": float("nan"),
            "heading_detrended_rms_deg": float("nan"),
            "heading_final_deg": float("nan"),
        }
    heading_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(heading_deg)))
    t_rel = time_s - time_s[0]
    slope, intercept = np.polyfit(t_rel, heading_unwrapped, 1)
    trend = slope * t_rel + intercept
    residual = heading_unwrapped - trend
    if mono_step_s > 0.0 and time_s.size >= 2:
        t_sample = np.arange(time_s[0], time_s[-1] + mono_step_s, mono_step_s)
        heading_sample = np.interp(t_sample, time_s, heading_unwrapped)
    else:
        heading_sample = heading_unwrapped
    mono_ratio, flip_count = compute_monotonicity_ratio(heading_sample)
    return {
        "heading_slope_deg_per_ks": float(slope * 1000.0),
        "heading_monotonicity_ratio": float(mono_ratio),
        "heading_sign_flip_count": float(flip_count),
        "heading_detrended_rms_deg": float(math.sqrt(np.mean(residual * residual))),
        "heading_final_deg": float(heading_unwrapped[-1]),
    }


def compute_drift(values: np.ndarray) -> float:
    if values.size < 2:
        return float("nan")
    return float(values[-1] - values[0])


def select_post_gnss_mask(time_s: np.ndarray, split_time_s: float | None) -> np.ndarray:
    if split_time_s is None:
        return np.ones_like(time_s, dtype=bool)
    return time_s >= split_time_s


def format_float(value: float, fmt: str) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "nan"
    return fmt.format(value)


def render_markdown_table(df: pd.DataFrame, columns: Iterable[tuple[str, str]]) -> str:
    headers = [col for col, _ in columns]
    header_line = "| " + " | ".join(headers) + " |"
    sep_line = "| " + " | ".join(["---"] * len(headers)) + " |"
    rows = []
    for _, row in df.iterrows():
        values = []
        for col, fmt in columns:
            val = row[col]
            if isinstance(val, (float, np.floating)):
                values.append(format_float(float(val), fmt))
            else:
                values.append(str(val))
        rows.append("| " + " | ".join(values) + " |")
    return "\n".join([header_line, sep_line] + rows)


def build_case_metrics(
    case: MechanismCase,
    truth_cache: dict[Path, nav_report.TruthBundle],
    mounting_base_cache: dict[str, np.ndarray],
    target_points: int,
) -> dict[str, float | str]:
    sol_path = nav_report.repo_path(case.sol_path)
    cfg_path = nav_report.repo_path(case.config_path)
    config = nav_report.load_yaml(cfg_path)
    truth_path = nav_report.extract_truth_path(config, cfg_path)
    truth = truth_cache.setdefault(truth_path, nav_report.load_truth_bundle(truth_path, target_points))
    split_time_s = nav_report.extract_split_time_s(config, truth)

    nav_case = nav_report.CaseSpec(
        case_id=case.case_id,
        label=case.label,
        sol_path=case.sol_path,
        config_path=case.config_path,
        color="#000000",
    )
    case_result = nav_report.build_case_result(nav_case, truth, target_points, mounting_base_cache)
    plot_df = case_result.plot_df

    time_s = plot_df["time_sec"].to_numpy(dtype=float)
    heading_err = plot_df["vehicle_heading_err_deg"].to_numpy(dtype=float)
    mask = select_post_gnss_mask(time_s, split_time_s)
    time_post = time_s[mask]
    heading_post = heading_err[mask]

    heading_metrics = compute_heading_metrics(time_post, heading_post)

    sol_df = nav_report.load_solution_frame(sol_path)
    sol_time_s = sol_df["timestamp"].to_numpy(dtype=float) - truth.time[0]
    sol_mask = select_post_gnss_mask(sol_time_s, split_time_s)
    sol_mount = sol_df["mounting_yaw"].to_numpy(dtype=float)[sol_mask]
    mount_drift = compute_drift(sol_mount)
    bg_drift = float("nan")
    if "bg_z" in sol_df.columns:
        sol_bg = sol_df["bg_z"].to_numpy(dtype=float)[sol_mask]
        bg_drift = compute_drift(sol_bg)

    duration = float(time_post[-1] - time_post[0]) if time_post.size > 1 else float("nan")
    return {
        "case": case.label,
        "dataset": case.dataset,
        "method": case.method,
        "samples_post_gnss": int(time_post.size),
        "post_gnss_start_s": float(split_time_s) if split_time_s is not None else float("nan"),
        "post_gnss_duration_s": duration,
        "heading_slope_deg_per_ks": heading_metrics["heading_slope_deg_per_ks"],
        "heading_monotonicity_ratio": heading_metrics["heading_monotonicity_ratio"],
        "heading_sign_flip_count": heading_metrics["heading_sign_flip_count"],
        "heading_detrended_rms_deg": heading_metrics["heading_detrended_rms_deg"],
        "heading_final_deg": heading_metrics["heading_final_deg"],
        "mounting_yaw_drift_deg": mount_drift,
        "bg_z_drift_rad_s": bg_drift,
        "sol_path": case.sol_path,
        "config_path": case.config_path,
    }


def build_summary(metrics: pd.DataFrame) -> str:
    def pick(dataset: str, method: str) -> dict[str, float] | None:
        subset = metrics[(metrics["dataset"] == dataset) & (metrics["method"] == method)]
        if subset.empty:
            return None
        return subset.iloc[0].to_dict()

    lines = [
        "# EXP-20260310-inekf-mechanism-r1 summary",
        "",
        "本文档量化 post-GNSS 时间窗内的 v 系航向误差形态，并结合冻结实验解释 InEKF 的作用机理。",
        "",
        "## 指标说明",
        "- heading_slope_deg_per_ks: v 系航向误差线性漂移斜率（deg/ks）",
        "- heading_monotonicity_ratio: 导数符号一致率（越接近 1 越单调）",
        "- heading_sign_flip_count: 导数符号变化次数（越少越单调）",
        "  - 两项在 1 s 重采样后计算，以抑制高频噪声干扰。",
        "- heading_detrended_rms_deg: 去趋势后的残差 RMS（越小越“少波动”）",
        "- mounting_yaw_drift_deg: post-GNSS 内 mounting_yaw 末-初漂移（deg）",
        "- bg_z_drift_rad_s: post-GNSS 内 bg_z 末-初漂移（rad/s）",
        "",
    ]

    table_cols = [
        ("case", "{}"),
        ("heading_slope_deg_per_ks", "{:.3f}"),
        ("heading_monotonicity_ratio", "{:.3f}"),
        ("heading_sign_flip_count", "{:.0f}"),
        ("heading_detrended_rms_deg", "{:.3f}"),
        ("mounting_yaw_drift_deg", "{:.3f}"),
        ("bg_z_drift_rad_s", "{:.6e}"),
    ]

    for dataset in ("data4", "data2"):
        subset = metrics[metrics["dataset"] == dataset].copy()
        if subset.empty:
            continue
        split_time = subset["post_gnss_start_s"].dropna().unique()
        split_note = f"{split_time[0]:.3f} s" if split_time.size > 0 else "N/A"
        lines.append(f"## {dataset} GNSS30（post-GNSS 起始 {split_note}）")
        lines.append(render_markdown_table(subset, table_cols))
        lines.append("")

        row_eskf = pick(dataset, "eskf")
        row_full = pick(dataset, "true_iekf")
        row_bg = pick(dataset, "freeze_bg")
        row_mount = pick(dataset, "freeze_mount")
        if row_eskf and row_full and row_bg and row_mount:
            lines.append("关键观察：")
            lines.append(
                (
                    f"- ESKF vs true_iekf：漂移斜率 {row_eskf['heading_slope_deg_per_ks']:.2f} → "
                    f"{row_full['heading_slope_deg_per_ks']:.2f} deg/ks，"
                    f"去趋势 RMS {row_eskf['heading_detrended_rms_deg']:.3f} → "
                    f"{row_full['heading_detrended_rms_deg']:.3f} deg。"
                )
            )
            lines.append(
                (
                    f"- freeze_bg：斜率降至 {row_bg['heading_slope_deg_per_ks']:.2f} deg/ks，"
                    f"bg_z 漂移 {row_bg['bg_z_drift_rad_s']:.3e} rad/s，"
                    f"残差 RMS {row_bg['heading_detrended_rms_deg']:.3f} deg。"
                )
            )
            lines.append(
                (
                    f"- freeze_mount：mounting_yaw 漂移 {row_mount['mounting_yaw_drift_deg']:.3f} deg，"
                    f"斜率 {row_mount['heading_slope_deg_per_ks']:.2f} deg/ks。"
                )
            )
            lines.append("")

    row_data4_full = pick("data4", "true_iekf")
    row_data4_bg = pick("data4", "freeze_bg")
    row_data4_mount = pick("data4", "freeze_mount")
    row_data2_full = pick("data2", "true_iekf")
    row_data2_bg = pick("data2", "freeze_bg")
    row_data2_mount = pick("data2", "freeze_mount")

    lines += [
        "## 机理归纳（与《可观性分析讨论与InEKF算法.tex》对应）",
        "1. InEKF 采用右不变误差定义后，李群核心的误差动力学在结构上不依赖当前估计，",
        "   因而 post-GNSS 时段更容易呈现“单调漂移而非来回摆动”的误差形态。",
        "   这一点对应文档中“群仿射性质 ⇒ 误差动力学常系数”的结论。",
    ]

    if row_data2_full and row_data2_bg and row_data2_mount:
        lines.append(
            (
                "2. data2 的漂移主要来自 bg：full 的漂移斜率 "
                f"{row_data2_full['heading_slope_deg_per_ks']:.2f} deg/ks，"
                f"freeze_bg 降至 {row_data2_bg['heading_slope_deg_per_ks']:.2f} deg/ks，"
                f"freeze_mount 仍为 {row_data2_mount['heading_slope_deg_per_ks']:.2f} deg/ks；"
                "同时 freeze_mount 将 mounting_yaw 漂移降为 0，但对斜率几乎无影响。"
            )
        )
    if row_data4_full and row_data4_bg and row_data4_mount:
        lines.append(
            (
                "3. data4 呈现同样趋势：freeze_bg 将漂移斜率压到 "
                f"{row_data4_bg['heading_slope_deg_per_ks']:.2f} deg/ks，"
                f"freeze_mount 仍约 {row_data4_mount['heading_slope_deg_per_ks']:.2f} deg/ks，"
                "说明 bg_z 仍是主导漂移通道。"
            )
        )
    if row_data2_full and row_data4_full:
        lines.append(
            (
                "4. data2 的 bg_z 漂移量级高于 data4："
                f"{row_data2_full['bg_z_drift_rad_s']:.3e} vs {row_data4_full['bg_z_drift_rad_s']:.3e} rad/s，"
                "与文档中“扩展 InEKF 仍对欧氏参数敏感”的论断一致。"
            )
        )

    lines += [
        "",
        "## 备注",
        "- 本实验仅统计 post-GNSS 时间窗（由配置中的 gnss_schedule.head_ratio 决定）。",
        "- HTML 报告用于呈现图像证据，本摘要用于给出机理解释与定量指标。",
    ]
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate InEKF mechanism metrics and summary.")
    parser.add_argument("--output-dir", type=str, default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--target-points", type=int, default=DEFAULT_TARGET_POINTS)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    truth_cache: dict[Path, nav_report.TruthBundle] = {}
    mounting_base_cache: dict[str, np.ndarray] = {}
    rows = [
        build_case_metrics(case, truth_cache, mounting_base_cache, args.target_points)
        for case in CASES
    ]
    metrics = pd.DataFrame(rows)
    metrics_path = output_dir / "metrics.csv"
    metrics.to_csv(metrics_path, index=False)

    summary_path = output_dir / "summary.md"
    summary_path.write_text(build_summary(metrics), encoding="utf-8")

    print(f"[inekf_mechanism_report] wrote {metrics_path}")
    print(f"[inekf_mechanism_report] wrote {summary_path}")


if __name__ == "__main__":
    main()
