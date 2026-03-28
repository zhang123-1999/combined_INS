from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import subprocess
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
    evaluate_navigation_metrics,
)


EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-yaw-bgz-coupling-straight-probe-r1"
BASE_CASE_CONFIG = Path(
    "output/debug_mounting_yaw_fullgnss_sensor_split_20260322/"
    "cases/yaw_only_both_nominal_aligned/config_yaw_only_both_nominal_aligned.yaml"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_bgz_coupling_straight_probe_20260322")

STRAIGHT_SPEED_MIN_M_S = 3.0
STRAIGHT_FAST_SPEED_MIN_M_S = 5.0
STRAIGHT_YAW_RATE_MAX_DEG_S = 3.0
TURN_YAW_RATE_MIN_DEG_S = 8.0
ZOOM_PADDING_S = 5.0


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    disable_mounting_yaw: bool = False
    disable_gyro_bias: bool = False


CASE_SPECS = [
    CaseSpec("free_yaw_free_bgz", "yaw free, bg_z free", "#d62728"),
    CaseSpec(
        "free_yaw_frozen_bgz",
        "yaw free, bg_z frozen",
        "#1f77b4",
        disable_gyro_bias=True,
    ),
    CaseSpec(
        "frozen_yaw_free_bgz",
        "yaw frozen, bg_z free",
        "#ff7f0e",
        disable_mounting_yaw=True,
    ),
    CaseSpec(
        "frozen_yaw_frozen_bgz",
        "yaw frozen, bg_z frozen",
        "#2ca02c",
        disable_mounting_yaw=True,
        disable_gyro_bias=True,
    ),
]


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


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
        raise RuntimeError(
            f"solver failed for {cfg_path.name}: returncode={proc.returncode}\n{merged}"
        )
    return merged


def load_pos_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def classify_motion(speed_m_s: np.ndarray, yaw_rate_deg_s: np.ndarray) -> np.ndarray:
    labels = np.full(speed_m_s.shape, "transition", dtype=object)
    labels[speed_m_s < STRAIGHT_SPEED_MIN_M_S] = "low_speed"
    straight_mask = (speed_m_s >= STRAIGHT_SPEED_MIN_M_S) & (
        np.abs(yaw_rate_deg_s) <= STRAIGHT_YAW_RATE_MAX_DEG_S
    )
    labels[straight_mask] = "straight"
    labels[(speed_m_s >= STRAIGHT_SPEED_MIN_M_S) & (yaw_rate_deg_s >= TURN_YAW_RATE_MIN_DEG_S)] = "turn_pos"
    labels[(speed_m_s >= STRAIGHT_SPEED_MIN_M_S) & (yaw_rate_deg_s <= -TURN_YAW_RATE_MIN_DEG_S)] = "turn_neg"
    return labels


def contiguous_windows(mask: np.ndarray, time_s: np.ndarray) -> pd.DataFrame:
    rows: list[dict[str, float | int]] = []
    if mask.size == 0:
        return pd.DataFrame(columns=["start_t", "end_t", "duration_s", "n"])
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return pd.DataFrame(columns=["start_t", "end_t", "duration_s", "n"])
    start = idx[0]
    prev = idx[0]
    for curr in idx[1:]:
        if curr == prev + 1:
            prev = curr
            continue
        rows.append(
            {
                "start_t": float(time_s[start]),
                "end_t": float(time_s[prev]),
                "duration_s": float(time_s[prev] - time_s[start]),
                "n": int(prev - start + 1),
            }
        )
        start = curr
        prev = curr
    rows.append(
        {
            "start_t": float(time_s[start]),
            "end_t": float(time_s[prev]),
            "duration_s": float(time_s[prev] - time_s[start]),
            "n": int(prev - start + 1),
        }
    )
    return pd.DataFrame(rows)


def safe_corr(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 3 or b.size < 3:
        return float("nan")
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def safe_diff_corr(a: np.ndarray, b: np.ndarray, mask: np.ndarray) -> float:
    idx = np.flatnonzero(mask)
    if idx.size < 4:
        return float("nan")
    keep_prev: list[int] = []
    keep_curr: list[int] = []
    for i0, i1 in zip(idx[:-1], idx[1:]):
        if i1 == i0 + 1:
            keep_prev.append(i0)
            keep_curr.append(i1)
    if len(keep_prev) < 3:
        return float("nan")
    da = a[np.asarray(keep_curr)] - a[np.asarray(keep_prev)]
    db = b[np.asarray(keep_curr)] - b[np.asarray(keep_prev)]
    return safe_corr(da, db)


def build_case_config(base_cfg: dict[str, Any], case_dir: Path, spec: CaseSpec) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg["fusion"]
    ablation = fusion.setdefault("ablation", {})
    constraints = fusion.setdefault("constraints", {})

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    constraints["enable_consistency_log"] = True
    constraints["enable_mechanism_log"] = False
    ablation["disable_mounting_yaw"] = bool(spec.disable_mounting_yaw)
    ablation["disable_gyro_bias"] = bool(spec.disable_gyro_bias)
    return cfg


def compute_case_metrics(
    *,
    spec: CaseSpec,
    cfg_path: Path,
    sol_path: Path,
    state_series_path: Path,
    stdout_text: str,
    pos_df: pd.DataFrame,
) -> tuple[dict[str, Any], pd.DataFrame]:
    nav_metrics, _ = evaluate_navigation_metrics(cfg_path, sol_path)
    consistency = parse_consistency_summary(stdout_text)
    state_df = pd.read_csv(state_series_path)

    state_t = state_df["timestamp"].to_numpy(dtype=float)
    yaw_total_deg = state_df["total_mounting_yaw_deg"].to_numpy(dtype=float)
    bg_z_degh = state_df["bg_z_degh"].to_numpy(dtype=float)

    pos_t = pos_df["t"].to_numpy(dtype=float)
    speed_truth = np.hypot(
        pos_df["vn"].to_numpy(dtype=float),
        pos_df["ve"].to_numpy(dtype=float),
    )
    yaw_unwrapped = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s_truth = np.rad2deg(np.gradient(yaw_unwrapped, pos_t))

    speed_m_s = np.interp(state_t, pos_t, speed_truth)
    yaw_rate_deg_s = np.interp(state_t, pos_t, yaw_rate_deg_s_truth)
    motion = classify_motion(speed_m_s, yaw_rate_deg_s)

    straight_mask = motion == "straight"
    straight_fast_mask = straight_mask & (speed_m_s >= STRAIGHT_FAST_SPEED_MIN_M_S)
    after_200_mask = (state_t - state_t[0]) >= 200.0
    if not np.any(after_200_mask):
        after_200_mask = np.ones_like(state_t, dtype=bool)
    straight_fast_after_200_mask = straight_fast_mask & after_200_mask
    if not np.any(straight_fast_after_200_mask):
        straight_fast_after_200_mask = straight_fast_mask
    if not np.any(straight_fast_after_200_mask):
        straight_fast_after_200_mask = straight_mask

    def masked_mean(values: np.ndarray, mask: np.ndarray) -> float:
        if not np.any(mask):
            return float("nan")
        return float(np.mean(values[mask]))

    def masked_std(values: np.ndarray, mask: np.ndarray) -> float:
        if not np.any(mask):
            return float("nan")
        return float(np.std(values[mask]))

    metrics: dict[str, Any] = {
        "case_id": spec.case_id,
        "label": spec.label,
        "disable_mounting_yaw": spec.disable_mounting_yaw,
        "disable_gyro_bias": spec.disable_gyro_bias,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "sol_mtime": mtime_text(sol_path),
        "state_series_mtime": mtime_text(state_series_path),
        "stdout_mtime": None,
        "yaw_final_total_deg": float(yaw_total_deg[-1]),
        "yaw_peak_to_peak_deg": float(np.ptp(yaw_total_deg)),
        "bg_z_final_degh": float(bg_z_degh[-1]),
        "bg_z_absmax_degh": float(np.max(np.abs(bg_z_degh))),
        "straight_ratio": float(np.mean(straight_mask)),
        "straight_fast_ratio": float(np.mean(straight_fast_mask)),
        "straight_fast_yaw_mean_deg": masked_mean(yaw_total_deg, straight_fast_after_200_mask),
        "straight_fast_yaw_std_deg": masked_std(yaw_total_deg, straight_fast_after_200_mask),
        "straight_fast_bg_z_mean_degh": masked_mean(bg_z_degh, straight_fast_after_200_mask),
        "straight_fast_bg_z_std_degh": masked_std(bg_z_degh, straight_fast_after_200_mask),
        "straight_fast_yaw_bg_corr": safe_corr(
            yaw_total_deg[straight_fast_after_200_mask],
            bg_z_degh[straight_fast_after_200_mask],
        ),
        "straight_fast_dyaw_dbg_corr": safe_diff_corr(
            yaw_total_deg, bg_z_degh, straight_fast_after_200_mask
        ),
        "odo_accept_ratio": consistency.get("ODO", {}).get("accept_ratio"),
        "nhc_accept_ratio": consistency.get("NHC", {}).get("accept_ratio"),
        "odo_nis_mean": consistency.get("ODO", {}).get("nis_mean"),
        "nhc_nis_mean": consistency.get("NHC", {}).get("nis_mean"),
    }
    metrics.update(nav_metrics)

    plot_df = pd.DataFrame(
        {
            "timestamp": state_t,
            "t_rel_s": state_t - state_t[0],
            "speed_m_s": speed_m_s,
            "yaw_rate_deg_s": yaw_rate_deg_s,
            "motion_label": motion,
            "straight_mask": straight_mask.astype(int),
            "straight_fast_mask": straight_fast_mask.astype(int),
            "yaw_total_deg": yaw_total_deg,
            "bg_z_degh": bg_z_degh,
            "case_id": spec.case_id,
            "label": spec.label,
        }
    )
    return metrics, plot_df


def render_summary(metrics_df: pd.DataFrame, longest_straight_fast: dict[str, float] | None) -> str:
    cols = [
        "case_id",
        "overall_rmse_3d_m_aux",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "yaw_final_total_deg",
        "bg_z_final_degh",
        "straight_fast_yaw_mean_deg",
        "straight_fast_bg_z_mean_degh",
        "straight_fast_dyaw_dbg_corr",
    ]
    lines = [
        "# yaw-bg_z straight-motion coupling probe",
        "",
        "本实验矩阵：`yaw/bg_z` 同时自由、只冻 `bg_z`、只冻 `yaw`、两者都冻。",
        "指标中的 `straight_fast_*` 统一按 `speed>=5 m/s` 且 `|yaw_rate|<=3 deg/s` 的直线段统计。",
        "",
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for _, row in metrics_df.iterrows():
        vals: list[str] = []
        for col in cols:
            value = row[col]
            if isinstance(value, str):
                vals.append(value)
            elif isinstance(value, (int, np.integer)):
                vals.append(str(int(value)))
            else:
                vals.append("NA" if not np.isfinite(float(value)) else f"{float(value):.6f}")
        lines.append("| " + " | ".join(vals) + " |")
    if longest_straight_fast is not None:
        lines.extend(
            [
                "",
                (
                    "最长 `straight_fast` 窗口："
                    f"`[{longest_straight_fast['start_t']:.3f}, {longest_straight_fast['end_t']:.3f}] s`, "
                    f"`duration={longest_straight_fast['duration_s']:.3f} s`。"
                ),
            ]
        )
    lines.append("")
    return "\n".join(lines)


def shade_windows(ax: plt.Axes, windows_df: pd.DataFrame, t0: float, color: str, alpha: float, label: str | None) -> None:
    first = True
    for _, row in windows_df.iterrows():
        ax.axvspan(
            float(row["start_t"]) - t0,
            float(row["end_t"]) - t0,
            color=color,
            alpha=alpha,
            linewidth=0.0,
            label=label if first else None,
        )
        first = False


def plot_overall_timeseries(
    *,
    merged_df: pd.DataFrame,
    case_specs: list[CaseSpec],
    straight_fast_windows: pd.DataFrame,
    zoom_window: dict[str, float] | None,
    output_path: Path,
) -> None:
    t0 = float(merged_df["timestamp"].min())
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)

    speed_df = merged_df[merged_df["case_id"] == case_specs[0].case_id]
    shade_windows(axes[0], straight_fast_windows, t0, "#d9f2d9", 0.45, "straight_fast")
    axes[0].plot(speed_df["t_rel_s"], speed_df["speed_m_s"], color="#444444", linewidth=1.0, label="speed")
    ax_rate = axes[0].twinx()
    ax_rate.plot(
        speed_df["t_rel_s"],
        speed_df["yaw_rate_deg_s"],
        color="#7f7f7f",
        linewidth=0.9,
        linestyle="--",
        label="yaw_rate",
    )
    axes[0].set_ylabel("speed [m/s]")
    ax_rate.set_ylabel("yaw rate [deg/s]")
    axes[0].grid(alpha=0.3)
    if zoom_window is not None:
        for ax in (axes[0], axes[1], axes[2], ax_rate):
            ax.axvline(zoom_window["start_t"] - t0, color="#9467bd", linewidth=1.0, linestyle=":")
            ax.axvline(zoom_window["end_t"] - t0, color="#9467bd", linewidth=1.0, linestyle=":")

    for spec in case_specs:
        case_df = merged_df[merged_df["case_id"] == spec.case_id]
        axes[1].plot(
            case_df["t_rel_s"],
            case_df["yaw_total_deg"],
            color=spec.color,
            linewidth=1.1,
            label=spec.label,
        )
        axes[2].plot(
            case_df["t_rel_s"],
            case_df["bg_z_degh"],
            color=spec.color,
            linewidth=1.1,
            label=spec.label,
        )
    shade_windows(axes[1], straight_fast_windows, t0, "#d9f2d9", 0.18, None)
    shade_windows(axes[2], straight_fast_windows, t0, "#d9f2d9", 0.18, None)
    axes[1].set_ylabel("total yaw [deg]")
    axes[2].set_ylabel("bg_z [deg/h]")
    axes[2].set_xlabel("time since start [s]")
    axes[1].grid(alpha=0.3)
    axes[2].grid(alpha=0.3)
    axes[0].legend(loc="upper left")
    ax_rate.legend(loc="upper right")
    axes[1].legend(loc="best", ncol=2, fontsize=9)
    fig.suptitle("yaw/bg_z coupling probe with straight_fast windows")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_zoom_timeseries(
    *,
    merged_df: pd.DataFrame,
    case_specs: list[CaseSpec],
    zoom_window: dict[str, float],
    output_path: Path,
) -> None:
    t0 = float(merged_df["timestamp"].min())
    zoom_start = zoom_window["start_t"] - ZOOM_PADDING_S
    zoom_end = zoom_window["end_t"] + ZOOM_PADDING_S
    zoom_df = merged_df[(merged_df["timestamp"] >= zoom_start) & (merged_df["timestamp"] <= zoom_end)].copy()

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    speed_df = zoom_df[zoom_df["case_id"] == case_specs[0].case_id]
    axes[0].plot(speed_df["t_rel_s"], speed_df["speed_m_s"], color="#444444", linewidth=1.0, label="speed")
    ax_rate = axes[0].twinx()
    ax_rate.plot(
        speed_df["t_rel_s"],
        speed_df["yaw_rate_deg_s"],
        color="#7f7f7f",
        linewidth=0.9,
        linestyle="--",
        label="yaw_rate",
    )
    for ax in (axes[0], axes[1], axes[2], ax_rate):
        ax.axvspan(
            zoom_window["start_t"] - t0,
            zoom_window["end_t"] - t0,
            color="#d9f2d9",
            alpha=0.35,
            linewidth=0.0,
        )
    axes[0].set_ylabel("speed [m/s]")
    ax_rate.set_ylabel("yaw rate [deg/s]")
    axes[0].grid(alpha=0.3)
    axes[0].legend(loc="upper left")
    ax_rate.legend(loc="upper right")

    for spec in case_specs:
        case_df = zoom_df[zoom_df["case_id"] == spec.case_id]
        axes[1].plot(case_df["t_rel_s"], case_df["yaw_total_deg"], color=spec.color, linewidth=1.1, label=spec.label)
        axes[2].plot(case_df["t_rel_s"], case_df["bg_z_degh"], color=spec.color, linewidth=1.1, label=spec.label)
    axes[1].set_ylabel("total yaw [deg]")
    axes[2].set_ylabel("bg_z [deg/h]")
    axes[2].set_xlabel("time since start [s]")
    axes[1].grid(alpha=0.3)
    axes[2].grid(alpha=0.3)
    axes[1].legend(loc="best", ncol=2, fontsize=9)
    fig.suptitle("longest straight_fast window zoom")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_phase(
    *,
    merged_df: pd.DataFrame,
    case_specs: list[CaseSpec],
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 7))
    for spec in case_specs:
        case_df = merged_df[(merged_df["case_id"] == spec.case_id) & (merged_df["straight_fast_mask"] == 1)]
        if case_df.empty:
            continue
        ax.plot(
            case_df["yaw_total_deg"],
            case_df["bg_z_degh"],
            color=spec.color,
            linewidth=1.1,
            alpha=0.9,
            label=spec.label,
        )
        ax.scatter(
            case_df["yaw_total_deg"].iloc[0],
            case_df["bg_z_degh"].iloc[0],
            color=spec.color,
            marker="o",
            s=28,
        )
        ax.scatter(
            case_df["yaw_total_deg"].iloc[-1],
            case_df["bg_z_degh"].iloc[-1],
            color=spec.color,
            marker="s",
            s=28,
        )
    ax.set_xlabel("total yaw [deg]")
    ax.set_ylabel("bg_z [deg/h]")
    ax.set_title("straight_fast phase trajectory: yaw vs bg_z")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe yaw-bg_z coupling during straight motion.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=BASE_CASE_CONFIG)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    base_cfg = load_yaml(REPO_ROOT / args.base_config)
    pos_df = load_pos_dataframe(REPO_ROOT / args.pos).sort_values(by="t").reset_index(drop=True)

    out_dir = REPO_ROOT / args.output_dir
    cases_dir = out_dir / "cases"
    plots_dir = out_dir / "plots"
    ensure_dir(cases_dir)
    ensure_dir(plots_dir)

    metrics_rows: list[dict[str, Any]] = []
    plot_frames: list[pd.DataFrame] = []
    manifest: list[dict[str, Any]] = []

    for spec in CASE_SPECS:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        cfg = build_case_config(base_cfg, case_dir, spec)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        stdout_text = run_solver(REPO_ROOT / args.exe, cfg_path)
        stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
        stdout_path.write_text(stdout_text, encoding="utf-8")

        sol_path = case_dir / f"SOL_{spec.case_id}.txt"
        state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
        if not sol_path.exists() or not state_series_path.exists():
            raise RuntimeError(f"missing solver outputs for {spec.case_id}")

        metrics, plot_df = compute_case_metrics(
            spec=spec,
            cfg_path=cfg_path,
            sol_path=sol_path,
            state_series_path=state_series_path,
            stdout_text=stdout_text,
            pos_df=pos_df,
        )
        metrics["stdout_path"] = rel_from_root(stdout_path, REPO_ROOT)
        metrics["stdout_mtime"] = mtime_text(stdout_path)
        metrics_rows.append(metrics)
        plot_frames.append(plot_df)
        manifest.append(
            {
                "case_id": spec.case_id,
                "config_path": rel_from_root(cfg_path, REPO_ROOT),
                "sol_path": rel_from_root(sol_path, REPO_ROOT),
                "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
                "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
            }
        )

    metrics_df = pd.DataFrame(metrics_rows)
    merged_df = pd.concat(plot_frames, ignore_index=True)

    ref_df = merged_df[merged_df["case_id"] == CASE_SPECS[0].case_id].copy()
    straight_windows = contiguous_windows(ref_df["motion_label"].eq("straight"), ref_df["timestamp"].to_numpy(dtype=float))
    straight_fast_windows = contiguous_windows(ref_df["straight_fast_mask"].eq(1), ref_df["timestamp"].to_numpy(dtype=float))
    longest_straight_fast = None
    if not straight_fast_windows.empty:
        longest_straight_fast = (
            straight_fast_windows.sort_values(by="duration_s", ascending=False).iloc[0].to_dict()
        )
    elif not straight_windows.empty:
        longest_straight_fast = straight_windows.sort_values(by="duration_s", ascending=False).iloc[0].to_dict()

    overall_plot = plots_dir / "yaw_bgz_timeseries_all_cases.png"
    zoom_plot = plots_dir / "yaw_bgz_longest_straight_fast_zoom.png"
    phase_plot = plots_dir / "yaw_bgz_phase_straight_fast.png"
    plot_overall_timeseries(
        merged_df=merged_df,
        case_specs=CASE_SPECS,
        straight_fast_windows=straight_fast_windows,
        zoom_window=longest_straight_fast,
        output_path=overall_plot,
    )
    if longest_straight_fast is not None:
        plot_zoom_timeseries(
            merged_df=merged_df,
            case_specs=CASE_SPECS,
            zoom_window=longest_straight_fast,
            output_path=zoom_plot,
        )
    plot_phase(merged_df=merged_df, case_specs=CASE_SPECS, output_path=phase_plot)

    metrics_path = out_dir / "metrics.csv"
    merged_path = out_dir / "timeseries_merged.csv"
    manifest_path = out_dir / "manifest.json"
    summary_path = out_dir / "summary.md"
    windows_path = out_dir / "straight_windows.csv"
    straight_fast_windows_path = out_dir / "straight_fast_windows.csv"

    metrics_df.to_csv(metrics_path, index=False)
    merged_df.to_csv(merged_path, index=False)
    straight_windows.to_csv(windows_path, index=False)
    straight_fast_windows.to_csv(straight_fast_windows_path, index=False)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    summary_path.write_text(render_summary(metrics_df, longest_straight_fast), encoding="utf-8")

    print(f"[done] metrics: {metrics_path}")
    print(f"[done] summary: {summary_path}")
    print(f"[done] overall_plot: {overall_plot}")
    if longest_straight_fast is not None:
        print(f"[done] zoom_plot: {zoom_plot}")
    print(f"[done] phase_plot: {phase_plot}")


if __name__ == "__main__":
    main()
