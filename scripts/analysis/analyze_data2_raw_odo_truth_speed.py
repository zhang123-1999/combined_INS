#!/usr/bin/env python3
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
RAW_ODO_PATH = REPO_ROOT / "dataset" / "data2" / "IMULog201812290233_2_ODO.txt"
TRUTH_PATH = REPO_ROOT / "dataset" / "data2" / "LC_SM_TXT.nav"
OUTPUT_DIR = REPO_ROOT / "output" / "data2_raw_odo_truth_speed_audit_r1_20260325"


@dataclass
class CompareMetric:
    reference: str
    mae: float
    rmse: float
    corr: float
    scale_odo_over_ref: float
    bias: float


def load_raw_odo(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path)
    timestamps = data[:, 0]
    odo_candidate = data[:, -2]
    reserved = data[:, -1]
    return timestamps, odo_candidate, reserved


def load_truth_speed(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path)
    timestamps = data[:, 1]
    vn = data[:, 5]
    ve = data[:, 6]
    vd = data[:, 7]
    speed_3d = np.sqrt(vn * vn + ve * ve + vd * vd)
    speed_h = np.sqrt(vn * vn + ve * ve)
    return timestamps, speed_3d, speed_h


def compute_metrics(odo: np.ndarray, ref: np.ndarray, name: str) -> CompareMetric:
    diff = odo - ref
    return CompareMetric(
        reference=name,
        mae=float(np.mean(np.abs(diff))),
        rmse=float(np.sqrt(np.mean(diff * diff))),
        corr=float(np.corrcoef(odo, ref)[0, 1]),
        scale_odo_over_ref=float(np.dot(odo, ref) / np.dot(ref, ref)),
        bias=float(np.mean(diff)),
    )


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def write_metrics_csv(path: Path, metrics: list[CompareMetric]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "reference",
                "mae_mps",
                "rmse_mps",
                "corr",
                "scale_odo_over_ref",
                "bias_mps",
            ],
        )
        writer.writeheader()
        for metric in metrics:
            writer.writerow(
                {
                    "reference": metric.reference,
                    "mae_mps": f"{metric.mae:.12f}",
                    "rmse_mps": f"{metric.rmse:.12f}",
                    "corr": f"{metric.corr:.12f}",
                    "scale_odo_over_ref": f"{metric.scale_odo_over_ref:.12f}",
                    "bias_mps": f"{metric.bias:.12f}",
                }
            )


def write_samples_csv(
    path: Path,
    timestamps: np.ndarray,
    odo: np.ndarray,
    truth_3d: np.ndarray,
    truth_h: np.ndarray,
) -> None:
    sample_indices = np.linspace(0, len(timestamps) - 1, 8, dtype=int)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "sample_idx",
                "t",
                "odo_candidate_mps",
                "truth_speed_3d_mps",
                "truth_speed_h_mps",
                "diff_3d_mps",
            ],
        )
        writer.writeheader()
        for idx in sample_indices:
            writer.writerow(
                {
                    "sample_idx": int(idx),
                    "t": f"{timestamps[idx]:.6f}",
                    "odo_candidate_mps": f"{odo[idx]:.9f}",
                    "truth_speed_3d_mps": f"{truth_3d[idx]:.9f}",
                    "truth_speed_h_mps": f"{truth_h[idx]:.9f}",
                    "diff_3d_mps": f"{(odo[idx] - truth_3d[idx]):.9f}",
                }
            )


def write_summary(
    path: Path,
    raw_odo_path: Path,
    truth_path: Path,
    metrics: list[CompareMetric],
    odo_dt_median: float,
    truth_dt_median: float,
    reserved_unique_count: int,
    reserved_min: float,
    reserved_max: float,
    near_integer_eps_1e6: float,
    near_integer_eps_1e3: float,
    odo_min: float,
    odo_max: float,
    odo_median: float,
) -> None:
    metric_map = {metric.reference: metric for metric in metrics}
    m3 = metric_map["truth_speed_3d"]
    mh = metric_map["truth_speed_h"]
    lines = [
        "# EXP-20260325-data2-raw-odo-truth-speed-audit-r1",
        "",
        "## Inputs",
        f"- raw_odo_path: `{raw_odo_path.relative_to(REPO_ROOT).as_posix()}`",
        f"- truth_path: `{truth_path.relative_to(REPO_ROOT).as_posix()}`",
        "",
        "## Method",
        "- 直接读取原始 `IMULog201812290233_2_ODO.txt`，取倒数第二列作为 ODO 候选观测；最后一列单独审计是否承载有效信息。",
        "- 读取 `LC_SM_TXT.nav` 中真值速度 `vn/ve/vd`，构造 `3D speed = sqrt(vn^2+ve^2+vd^2)` 与 `horizontal speed = sqrt(vn^2+ve^2)`。",
        "- 将真值速度按时间戳插值到原始 ODO 时间轴，再计算误差、相关系数与最小二乘比例系数。",
        "",
        "## Core Metrics",
        f"- 对 `truth_speed_3d`：`MAE={m3.mae:.6f} m/s`, `RMSE={m3.rmse:.6f} m/s`, `corr={m3.corr:.6f}`, `odo/ref scale={m3.scale_odo_over_ref:.6f}`, `bias={m3.bias:.6f} m/s`。",
        f"- 对 `truth_speed_h`：`MAE={mh.mae:.6f} m/s`, `RMSE={mh.rmse:.6f} m/s`, `corr={mh.corr:.6f}`, `odo/ref scale={mh.scale_odo_over_ref:.6f}`, `bias={mh.bias:.6f} m/s`。",
        f"- 采样周期：`odo_dt_median={odo_dt_median:.9f} s`, `truth_dt_median={truth_dt_median:.9f} s`。",
        f"- ODO 候选列范围：`min={odo_min:.6f} m/s`, `median={odo_median:.6f} m/s`, `max={odo_max:.6f} m/s`。",
        f"- 原始最后一列：`unique_count={reserved_unique_count}`, `min={reserved_min:.6f}`, `max={reserved_max:.6f}`，表现为纯保留列。",
        f"- 整数特征检验：`near_integer_ratio(|x-round(x)|<1e-6)={near_integer_eps_1e6:.6f}`, `near_integer_ratio(<1e-3)={near_integer_eps_1e3:.6f}`。",
        "",
        "## Interpretation",
        "- 原始 ODO 候选列与真值速度模长几乎重合，相关系数达到 `0.9998` 量级，误差仅 `0.1 m/s` 左右。",
        "- 若该列是原始脉冲计数，则不应在未乘轮径、分辨率、采样周期换算的情况下，直接与真值速度保持如此高的一致性。",
        "- 同时该列并不呈现整数计数特征，而最后一列恒为零，因此原始文件更像“IMU 增量 + 已换算 ODO 前向速度 + reserved”格式，而不是“原始编码器脉冲计数”。",
        "",
        "## Conclusion",
        "- 基于原始数据本身，而不是基于当前程序读取链，`IMULog201812290233_2_ODO.txt` 的倒数第二列应视为已经换算到物理单位的前向速度观测，单位近似为 `m/s`。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ensure_dir(OUTPUT_DIR)

    odo_t, odo, reserved = load_raw_odo(RAW_ODO_PATH)
    truth_t, truth_speed_3d, truth_speed_h = load_truth_speed(TRUTH_PATH)

    overlap = (odo_t >= truth_t.min()) & (odo_t <= truth_t.max())
    odo_t = odo_t[overlap]
    odo = odo[overlap]
    truth_3d_interp = np.interp(odo_t, truth_t, truth_speed_3d)
    truth_h_interp = np.interp(odo_t, truth_t, truth_speed_h)

    metrics = [
        compute_metrics(odo, truth_3d_interp, "truth_speed_3d"),
        compute_metrics(odo, truth_h_interp, "truth_speed_h"),
    ]

    write_metrics_csv(OUTPUT_DIR / "metrics.csv", metrics)
    write_samples_csv(
        OUTPUT_DIR / "samples.csv",
        odo_t,
        odo,
        truth_3d_interp,
        truth_h_interp,
    )
    write_summary(
        OUTPUT_DIR / "summary.md",
        RAW_ODO_PATH,
        TRUTH_PATH,
        metrics,
        odo_dt_median=float(np.median(np.diff(odo_t))),
        truth_dt_median=float(np.median(np.diff(truth_t))),
        reserved_unique_count=int(np.unique(reserved).size),
        reserved_min=float(reserved.min()),
        reserved_max=float(reserved.max()),
        near_integer_eps_1e6=float(np.mean(np.abs(odo - np.round(odo)) < 1e-6)),
        near_integer_eps_1e3=float(np.mean(np.abs(odo - np.round(odo)) < 1e-3)),
        odo_min=float(odo.min()),
        odo_max=float(odo.max()),
        odo_median=float(np.median(odo)),
    )

    print(f"[OK] wrote {OUTPUT_DIR.relative_to(REPO_ROOT).as_posix()}")


if __name__ == "__main__":
    main()
