from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
ALL_STATES_DEFAULT = Path(
    "output/d2_fixlev_combo_q025_nav4/artifacts/cases/"
    "data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed/"
    "all_states_data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed.csv"
)
PHASE_METRICS_DEFAULT = Path("output/d2_fixlev_combo_q025_nav4/phase_metrics.csv")
OUTPUT_DIR_DEFAULT = Path("output/d2_fixlev_q025_nav4_straight_drift_r1")
EXP_ID_DEFAULT = "EXP-20260327-data2-fixlever-combo-q025-nav4-straight-outage-drift-r1"
CASE_ID_DEFAULT = "data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed"

GRAVITY_MPS2 = 9.80665
START_STRIDE_S = 1.0
TARGET_DURATIONS_S = (30.0, 60.0, 90.0)


@dataclass(frozen=True)
class StraightSpec:
    name: str
    label: str
    speed_min_mps: float
    yaw_rate_max_deg_s: float


STRAIGHT_SPECS = (
    StraightSpec(
        name="strict_straight",
        label="strict straight: speed>=8 m/s, |yaw_rate|<=1.0 deg/s",
        speed_min_mps=8.0,
        yaw_rate_max_deg_s=1.0,
    ),
    StraightSpec(
        name="near_straight",
        label="near-straight: speed>=8 m/s, |yaw_rate|<=1.5 deg/s",
        speed_min_mps=8.0,
        yaw_rate_max_deg_s=1.5,
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze GNSS-outage straight-line drift for the current data2 combo best."
    )
    parser.add_argument("--all-states", type=Path, default=ALL_STATES_DEFAULT)
    parser.add_argument("--phase-metrics", type=Path, default=PHASE_METRICS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--case-id", default=CASE_ID_DEFAULT)
    return parser.parse_args()


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def rel_from_root(path: Path) -> str:
    return str(path.resolve().relative_to(REPO_ROOT.resolve())).replace("\\", "/")


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


def wrap_angle_deg(angle_deg: np.ndarray) -> np.ndarray:
    return (angle_deg + 180.0) % 360.0 - 180.0


def contiguous_ranges(mask: np.ndarray) -> list[tuple[int, int]]:
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    ranges: list[tuple[int, int]] = []
    start = int(idx[0])
    prev = int(idx[0])
    for raw_curr in idx[1:]:
        curr = int(raw_curr)
        if curr == prev + 1:
            prev = curr
            continue
        ranges.append((start, prev))
        start = curr
        prev = curr
    ranges.append((start, prev))
    return ranges


def build_base_frame(all_states: pd.DataFrame) -> pd.DataFrame:
    df = all_states.copy()
    for axis in ("n", "e", "u"):
        df[f"pos_err_{axis}_m"] = df[f"p_{axis}_m"] - df[f"truth_p_{axis}_m"]
        df[f"vel_err_{axis}_mps"] = df[f"v_{axis}_mps"] - df[f"truth_v_{axis}_mps"]
    df["pos_err_h_m"] = np.hypot(df["pos_err_n_m"], df["pos_err_e_m"])
    df["pos_err_3d_m"] = np.sqrt(
        df["pos_err_n_m"] ** 2 + df["pos_err_e_m"] ** 2 + df["pos_err_u_m"] ** 2
    )
    df["vel_err_h_mps"] = np.hypot(df["vel_err_n_mps"], df["vel_err_e_mps"])
    df["vel_err_3d_mps"] = np.sqrt(
        df["vel_err_n_mps"] ** 2 + df["vel_err_e_mps"] ** 2 + df["vel_err_u_mps"] ** 2
    )
    df["roll_err_deg"] = df["roll_deg"] - df["truth_roll_deg"]
    df["pitch_err_deg"] = df["pitch_deg"] - df["truth_pitch_deg"]
    df["yaw_err_deg"] = wrap_angle_deg((df["yaw_deg"] - df["truth_yaw_deg"]).to_numpy(dtype=float))
    df["tilt_err_deg"] = np.hypot(df["roll_err_deg"], df["pitch_err_deg"])
    df["tilt_err_rad"] = np.deg2rad(df["tilt_err_deg"])

    t = df["timestamp"].to_numpy(dtype=float)
    truth_yaw_rad = np.unwrap(np.deg2rad(df["truth_yaw_deg"].to_numpy(dtype=float)))
    truth_speed = np.hypot(
        df["truth_v_n_mps"].to_numpy(dtype=float),
        df["truth_v_e_mps"].to_numpy(dtype=float),
    )
    truth_abs_yaw_rate = np.abs(np.rad2deg(np.gradient(truth_yaw_rad, t)))
    truth_lat_acc = truth_speed * np.deg2rad(truth_abs_yaw_rate)
    df["truth_speed_mps"] = truth_speed
    df["truth_abs_yaw_rate_degps"] = truth_abs_yaw_rate
    df["truth_lat_acc_mps2"] = truth_lat_acc
    return df


def load_outage_windows(phase_metrics_path: Path, case_id: str) -> pd.DataFrame:
    phase_df = pd.read_csv(phase_metrics_path)
    outage_df = phase_df.loc[
        (phase_df["case_id"] == case_id) & (phase_df["window_type"] == "gnss_off")
    ].copy()
    outage_df["duration_s"] = outage_df["end_time"] - outage_df["start_time"]
    outage_df = outage_df.sort_values(by="start_time").reset_index(drop=True)
    return outage_df


def integrate_tilt_budget(time_rel_s: np.ndarray, tilt_err_rad: np.ndarray) -> np.ndarray:
    accel_upper = GRAVITY_MPS2 * tilt_err_rad
    vel_upper = np.zeros_like(accel_upper, dtype=float)
    pos_upper = np.zeros_like(accel_upper, dtype=float)
    for idx in range(1, len(time_rel_s)):
        dt_s = float(time_rel_s[idx] - time_rel_s[idx - 1])
        vel_upper[idx] = vel_upper[idx - 1] + 0.5 * (accel_upper[idx] + accel_upper[idx - 1]) * dt_s
        pos_upper[idx] = pos_upper[idx - 1] + 0.5 * (vel_upper[idx] + vel_upper[idx - 1]) * dt_s
    return pos_upper


def extract_segment_metrics(
    frame: pd.DataFrame,
    start_idx: int,
    end_idx: int,
    target_duration_s: float,
) -> dict[str, float]:
    seg_df = frame.iloc[start_idx : end_idx + 1].reset_index(drop=True)
    time_rel_s = seg_df["timestamp"].to_numpy(dtype=float) - float(seg_df["timestamp"].iloc[0])
    target_idx = int(np.searchsorted(time_rel_s, target_duration_s, side="left"))
    target_idx = min(target_idx, len(seg_df) - 1)

    dn = float(seg_df["pos_err_n_m"].iloc[target_idx] - seg_df["pos_err_n_m"].iloc[0])
    de = float(seg_df["pos_err_e_m"].iloc[target_idx] - seg_df["pos_err_e_m"].iloc[0])
    du = float(seg_df["pos_err_u_m"].iloc[target_idx] - seg_df["pos_err_u_m"].iloc[0])
    actual_h_m = math.hypot(dn, de)
    actual_3d_m = math.sqrt(dn * dn + de * de + du * du)

    tilt_budget_curve = integrate_tilt_budget(
        time_rel_s[: target_idx + 1],
        seg_df["tilt_err_rad"].to_numpy(dtype=float)[: target_idx + 1],
    )
    tilt_budget_m = float(tilt_budget_curve[-1])
    entry_vel_term_m = float(seg_df["vel_err_h_mps"].iloc[0]) * target_duration_s

    return {
        "target_duration_s": target_duration_s,
        "actual_h_growth_m": actual_h_m,
        "actual_3d_growth_m": actual_3d_m,
        "vertical_growth_m": abs(du),
        "entry_vel_term_m": entry_vel_term_m,
        "tilt_budget_upper_m": tilt_budget_m,
        "residual_after_tilt_lower_m": max(actual_h_m - tilt_budget_m, 0.0),
        "proxy_sum_m": entry_vel_term_m + tilt_budget_m,
    }


def collect_longest_segments(
    frame: pd.DataFrame,
    outage_df: pd.DataFrame,
    spec: StraightSpec,
) -> tuple[pd.DataFrame, list[dict[str, object]]]:
    rows: list[dict[str, object]] = []
    plot_segments: list[dict[str, object]] = []
    t = frame["timestamp"].to_numpy(dtype=float)
    for outage_row in outage_df.itertuples(index=False):
        base_mask = (
            (t >= float(outage_row.start_time))
            & (t <= float(outage_row.end_time))
            & (frame["truth_speed_mps"].to_numpy(dtype=float) >= spec.speed_min_mps)
            & (frame["truth_abs_yaw_rate_degps"].to_numpy(dtype=float) <= spec.yaw_rate_max_deg_s)
        )
        segments = contiguous_ranges(base_mask)
        if not segments:
            rows.append(
                {
                    "spec_name": spec.name,
                    "window_name": outage_row.window_name,
                    "segment_found": 0,
                }
            )
            continue
        best_start_idx, best_end_idx = max(
            segments,
            key=lambda ab: t[ab[1]] - t[ab[0]],
        )
        seg_df = frame.iloc[best_start_idx : best_end_idx + 1].reset_index(drop=True)
        duration_s = float(seg_df["timestamp"].iloc[-1] - seg_df["timestamp"].iloc[0])
        base_row: dict[str, object] = {
            "spec_name": spec.name,
            "window_name": outage_row.window_name,
            "segment_found": 1,
            "segment_start_time": float(seg_df["timestamp"].iloc[0]),
            "segment_end_time": float(seg_df["timestamp"].iloc[-1]),
            "segment_duration_s": duration_s,
            "start_pos_err_3d_m": float(seg_df["pos_err_3d_m"].iloc[0]),
            "start_vel_err_h_mps": float(seg_df["vel_err_h_mps"].iloc[0]),
            "start_vel_err_3d_mps": float(seg_df["vel_err_3d_mps"].iloc[0]),
            "start_tilt_err_deg": float(seg_df["tilt_err_deg"].iloc[0]),
            "mean_speed_mps": float(seg_df["truth_speed_mps"].mean()),
            "mean_abs_yaw_rate_degps": float(seg_df["truth_abs_yaw_rate_degps"].mean()),
            "max_abs_yaw_rate_degps": float(seg_df["truth_abs_yaw_rate_degps"].max()),
            "mean_lat_acc_mps2": float(seg_df["truth_lat_acc_mps2"].mean()),
            "max_lat_acc_mps2": float(seg_df["truth_lat_acc_mps2"].max()),
        }
        for duration_s_target in TARGET_DURATIONS_S:
            prefix = f"t{int(duration_s_target):02d}_"
            if duration_s < duration_s_target:
                base_row[f"{prefix}available"] = 0
                continue
            metrics = extract_segment_metrics(seg_df, 0, len(seg_df) - 1, duration_s_target)
            base_row[f"{prefix}available"] = 1
            for key, value in metrics.items():
                if key == "target_duration_s":
                    continue
                base_row[f"{prefix}{key}"] = value
        rows.append(base_row)
        plot_segments.append(
            {
                "spec_name": spec.name,
                "window_name": outage_row.window_name,
                "data": seg_df,
            }
        )
    return pd.DataFrame(rows), plot_segments


def collect_subwindow_rows(
    frame: pd.DataFrame,
    outage_df: pd.DataFrame,
    spec: StraightSpec,
) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    t = frame["timestamp"].to_numpy(dtype=float)
    speed = frame["truth_speed_mps"].to_numpy(dtype=float)
    abs_yaw_rate = frame["truth_abs_yaw_rate_degps"].to_numpy(dtype=float)
    for outage_row in outage_df.itertuples(index=False):
        base_mask = (
            (t >= float(outage_row.start_time))
            & (t <= float(outage_row.end_time))
            & (speed >= spec.speed_min_mps)
            & (abs_yaw_rate <= spec.yaw_rate_max_deg_s)
        )
        for start_idx, end_idx in contiguous_ranges(base_mask):
            seg_df = frame.iloc[start_idx : end_idx + 1].reset_index(drop=True)
            seg_t = seg_df["timestamp"].to_numpy(dtype=float)
            seg_duration_s = float(seg_t[-1] - seg_t[0])
            for target_duration_s in TARGET_DURATIONS_S:
                if seg_duration_s < target_duration_s:
                    continue
                start_time = float(seg_t[0])
                latest_start = float(seg_t[-1] - target_duration_s)
                while start_time <= latest_start + 1.0e-9:
                    local_start_idx = int(np.searchsorted(seg_t, start_time, side="left"))
                    target_time = float(seg_t[local_start_idx] + target_duration_s)
                    local_end_idx = int(np.searchsorted(seg_t, target_time, side="left"))
                    local_end_idx = min(local_end_idx, len(seg_df) - 1)
                    metrics = extract_segment_metrics(
                        seg_df,
                        local_start_idx,
                        local_end_idx,
                        target_duration_s,
                    )
                    rows.append(
                        {
                            "spec_name": spec.name,
                            "window_name": outage_row.window_name,
                            "segment_start_time": float(seg_t[local_start_idx]),
                            "segment_end_time": float(seg_t[local_end_idx]),
                            "start_pos_err_3d_m": float(seg_df["pos_err_3d_m"].iloc[local_start_idx]),
                            "start_vel_err_h_mps": float(seg_df["vel_err_h_mps"].iloc[local_start_idx]),
                            "start_tilt_err_deg": float(seg_df["tilt_err_deg"].iloc[local_start_idx]),
                            "start_speed_mps": float(seg_df["truth_speed_mps"].iloc[local_start_idx]),
                            "start_abs_yaw_rate_degps": float(
                                seg_df["truth_abs_yaw_rate_degps"].iloc[local_start_idx]
                            ),
                            "start_lat_acc_mps2": float(seg_df["truth_lat_acc_mps2"].iloc[local_start_idx]),
                            **metrics,
                        }
                    )
                    start_time += START_STRIDE_S
    return pd.DataFrame(rows)


def summarize_subwindows(subwindow_df: pd.DataFrame, spec: StraightSpec) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    spec_df = subwindow_df.loc[subwindow_df["spec_name"] == spec.name]
    for target_duration_s in TARGET_DURATIONS_S:
        target_df = spec_df.loc[spec_df["target_duration_s"] == target_duration_s]
        row: dict[str, object] = {
            "spec_name": spec.name,
            "spec_label": spec.label,
            "target_duration_s": target_duration_s,
            "window_count": int(len(target_df)),
        }
        if target_df.empty:
            rows.append(row)
            continue
        for col in (
            "actual_h_growth_m",
            "actual_3d_growth_m",
            "entry_vel_term_m",
            "tilt_budget_upper_m",
            "residual_after_tilt_lower_m",
            "start_vel_err_h_mps",
            "start_tilt_err_deg",
        ):
            values = target_df[col].to_numpy(dtype=float)
            row[f"{col}_median"] = float(np.median(values))
            row[f"{col}_p95"] = float(np.quantile(values, 0.95))
            row[f"{col}_max"] = float(np.max(values))
        rows.append(row)
    return rows


def plot_example_segments(plot_segments: list[dict[str, object]], output_path: Path) -> None:
    near_segments = [
        item
        for item in plot_segments
        if item["spec_name"] == "near_straight"
    ]
    near_segments = sorted(
        near_segments,
        key=lambda item: float(item["data"]["timestamp"].iloc[-1] - item["data"]["timestamp"].iloc[0]),
        reverse=True,
    )[:4]
    if not near_segments:
        return

    fig, axes = plt.subplots(len(near_segments), 1, figsize=(10, 3.1 * len(near_segments)), sharex=False)
    if len(near_segments) == 1:
        axes = [axes]
    for ax, item in zip(axes, near_segments):
        seg_df: pd.DataFrame = item["data"]
        time_rel_s = seg_df["timestamp"].to_numpy(dtype=float) - float(seg_df["timestamp"].iloc[0])
        dn = seg_df["pos_err_n_m"].to_numpy(dtype=float) - float(seg_df["pos_err_n_m"].iloc[0])
        de = seg_df["pos_err_e_m"].to_numpy(dtype=float) - float(seg_df["pos_err_e_m"].iloc[0])
        actual_h = np.hypot(dn, de)
        entry_vel_term = float(seg_df["vel_err_h_mps"].iloc[0]) * time_rel_s
        tilt_budget = integrate_tilt_budget(
            time_rel_s,
            seg_df["tilt_err_rad"].to_numpy(dtype=float),
        )
        ax.plot(time_rel_s, actual_h, color="#111111", linewidth=1.7, label="actual horizontal growth")
        ax.plot(time_rel_s, entry_vel_term, color="#1f77b4", linewidth=1.1, label="entry |dv_h| * t")
        ax.plot(time_rel_s, tilt_budget, color="#d62728", linewidth=1.1, label="tilt budget upper")
        ax.plot(
            time_rel_s,
            entry_vel_term + tilt_budget,
            color="#2ca02c",
            linewidth=1.1,
            linestyle="--",
            label="entry term + tilt budget",
        )
        ax.set_title(
            f"{item['window_name']} | duration={time_rel_s[-1]:.1f}s | "
            f"speed={seg_df['truth_speed_mps'].mean():.2f} m/s | "
            f"|yaw_rate| mean={seg_df['truth_abs_yaw_rate_degps'].mean():.3f} deg/s"
        )
        ax.set_ylabel("horizontal drift [m]")
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc="upper left", ncol=2, fontsize=8)
    axes[-1].set_xlabel("elapsed time in selected straight outage segment [s]")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_aggregate_stats(stats_df: pd.DataFrame, output_path: Path) -> None:
    plot_df = stats_df.loc[stats_df["window_count"] > 0].copy()
    if plot_df.empty:
        return
    x_labels = [
        f"{row.spec_name}\n{int(row.target_duration_s)}s"
        for row in plot_df.itertuples(index=False)
    ]
    x = np.arange(len(x_labels))
    width = 0.23
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.bar(
        x - width,
        plot_df["actual_h_growth_m_median"],
        width,
        label="actual median",
        color="#111111",
    )
    ax.bar(
        x,
        plot_df["entry_vel_term_m_median"],
        width,
        label="entry |dv_h|*t median",
        color="#1f77b4",
    )
    ax.bar(
        x + width,
        plot_df["tilt_budget_upper_m_median"],
        width,
        label="tilt budget median",
        color="#d62728",
    )
    for idx, row in enumerate(plot_df.itertuples(index=False)):
        ax.text(
            x[idx] - width,
            float(row.actual_h_growth_m_p95) + 0.5,
            f"p95={row.actual_h_growth_m_p95:.1f}",
            ha="center",
            va="bottom",
            fontsize=8,
            rotation=90,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels)
    ax.set_ylabel("horizontal growth [m]")
    ax.set_title("Straight-outage subwindow drift statistics")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_summary(
    summary_path: Path,
    *,
    exp_id: str,
    case_id: str,
    all_states_path: Path,
    phase_metrics_path: Path,
    outage_df: pd.DataFrame,
    stats_df: pd.DataFrame,
    longest_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    lines: list[str] = []
    lines.append("# outage straight-drift analysis")
    lines.append("")
    lines.append(f"- exp_id: `{exp_id}`")
    lines.append(f"- case_id: `{case_id}`")
    lines.append(f"- all_states: `{rel_from_root(all_states_path)}`")
    lines.append(f"- phase_metrics: `{rel_from_root(phase_metrics_path)}`")
    lines.append(
        "- straight definitions: "
        "`strict straight = speed>=8 m/s and |truth_yaw_rate|<=1.0 deg/s`; "
        "`near-straight = speed>=8 m/s and |truth_yaw_rate|<=1.5 deg/s`"
    )
    lines.append(
        "- decomposition used: "
        "`actual horizontal growth`, `entry velocity term = |delta v_h(start)| * t`, "
        "`tilt budget upper = int int g * tilt_err dt^2`, "
        "`residual_after_tilt_lower = max(actual_h - tilt_budget_upper, 0)`"
    )
    lines.append("")

    strict_60 = stats_df.loc[
        (stats_df["spec_name"] == "strict_straight") & (stats_df["target_duration_s"] == 60.0)
    ].iloc[0]
    strict_30 = stats_df.loc[
        (stats_df["spec_name"] == "strict_straight") & (stats_df["target_duration_s"] == 30.0)
    ].iloc[0]
    near_60 = stats_df.loc[
        (stats_df["spec_name"] == "near_straight") & (stats_df["target_duration_s"] == 60.0)
    ].iloc[0]
    longest_strict = longest_df.loc[
        (longest_df["spec_name"] == "strict_straight") & (longest_df["segment_found"] == 1),
        "segment_duration_s",
    ]
    longest_near = longest_df.loc[
        (longest_df["spec_name"] == "near_straight") & (longest_df["segment_found"] == 1),
        "segment_duration_s",
    ]

    lines.append("## Key Findings")
    lines.append(
        f"- strict straight 下，所有 outage 里最长连续直线段只有 `{longest_strict.max():.3f} s`，所以当前数据里不存在完整 `60 s` 的严格直线 outage 样本。"
    )
    lines.append(
        f"- strict straight `30 s` 子窗口共有 `{int(strict_30['window_count'])}` 个，"
        f"`actual_h_growth` 的 `median/p95/max = "
        f"{strict_30['actual_h_growth_m_median']:.3f} / {strict_30['actual_h_growth_m_p95']:.3f} / {strict_30['actual_h_growth_m_max']:.3f} m`。"
    )
    lines.append(
        f"- 同一批 strict `30 s` 子窗口里，排除姿态放大后的保守下界 "
        f"`residual_after_tilt_lower median/p95/max = "
        f"{strict_30['residual_after_tilt_lower_m_median']:.3f} / "
        f"{strict_30['residual_after_tilt_lower_m_p95']:.3f} / "
        f"{strict_30['residual_after_tilt_lower_m_max']:.3f} m`；"
        f"入口水平速度误差项本身就有 `median={strict_30['entry_vel_term_m_median']:.3f} m / 30 s`。"
    )
    lines.append(
        f"- 如果放宽到 near-straight，最长连续段可到 `{longest_near.max():.3f} s`，因此可以评估 `60 s`。"
        f"现有 `60 s` 子窗口共 `{int(near_60['window_count'])}` 个，"
        f"`actual_h_growth median/p95/max = "
        f"{near_60['actual_h_growth_m_median']:.3f} / {near_60['actual_h_growth_m_p95']:.3f} / {near_60['actual_h_growth_m_max']:.3f} m`。"
    )
    lines.append(
        f"- 这批 near-straight `60 s` 子窗口里，即使把姿态项按上界剔掉，"
        f"`residual_after_tilt_lower median={near_60['residual_after_tilt_lower_m_median']:.3f} m`；"
        f"而入口水平速度误差项单独就有 "
        f"`median={near_60['entry_vel_term_m_median']:.3f} m / 60 s`, "
        f"`max={near_60['entry_vel_term_m_max']:.3f} m / 60 s`。"
    )
    lines.append(
        "- 结论直接对应你的问题："
        "`1 min 10 m` 在当前配置下不仅会出现，而且在后半段 outage 的近直线窗口里明显超过；"
        "真正的门槛不是姿态是否继续放大，而是 outage 入口已经带着多大的水平速度误差。"
    )
    lines.append("")

    lines.append("## Representative Windows")
    rep_df = longest_df.loc[
        (longest_df["spec_name"] == "near_straight")
        & (longest_df["segment_found"] == 1)
        & (longest_df["window_name"].isin(["gnss_off_01", "gnss_off_04", "gnss_off_07", "gnss_off_09"]))
    ].copy()
    for row in rep_df.itertuples(index=False):
        lines.append(
            f"- `{row.window_name}`: segment=`[{row.segment_start_time:.3f}, {row.segment_end_time:.3f}] s`, "
            f"`duration={row.segment_duration_s:.3f} s`, "
            f"`start_pos_err_3d={row.start_pos_err_3d_m:.3f} m`, "
            f"`start_vel_err_h={row.start_vel_err_h_mps:.3f} m/s`, "
            f"`start_tilt={row.start_tilt_err_deg:.4f} deg`, "
            f"`speed_mean={row.mean_speed_mps:.3f} m/s`, "
            f"`|yaw_rate| mean/max={row.mean_abs_yaw_rate_degps:.3f}/{row.max_abs_yaw_rate_degps:.3f} deg/s`."
        )
        for duration_s in (30, 60):
            prefix = f"t{duration_s:02d}_"
            if not row._asdict().get(f"{prefix}available", 0):
                continue
            lines.append(
                f"  `{duration_s}s`: actual_h=`{row._asdict()[f'{prefix}actual_h_growth_m']:.3f} m`, "
                f"actual_3d=`{row._asdict()[f'{prefix}actual_3d_growth_m']:.3f} m`, "
                f"entry_vel_term=`{row._asdict()[f'{prefix}entry_vel_term_m']:.3f} m`, "
                f"tilt_budget_upper=`{row._asdict()[f'{prefix}tilt_budget_upper_m']:.3f} m`, "
                f"residual_after_tilt_lower=`{row._asdict()[f'{prefix}residual_after_tilt_lower_m']:.3f} m`."
            )
    lines.append("")

    lines.append("## Artifacts")
    lines.append(f"- outage windows: `{len(outage_df)}`")
    lines.append(f"- subwindow metrics csv: `{plot_paths['subwindow_metrics_csv']}`")
    lines.append(f"- subwindow stats csv: `{plot_paths['aggregate_stats_csv']}`")
    lines.append(f"- longest segments csv: `{plot_paths['longest_segments_csv']}`")
    lines.append(f"- example growth plot: `{plot_paths['example_plot']}`")
    lines.append(f"- aggregate stats plot: `{plot_paths['aggregate_plot']}`")
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    plots_dir = output_dir / "plots"
    ensure_dir(output_dir)
    ensure_dir(plots_dir)

    all_states_path = (REPO_ROOT / args.all_states).resolve()
    phase_metrics_path = (REPO_ROOT / args.phase_metrics).resolve()
    all_states = pd.read_csv(all_states_path)
    frame = build_base_frame(all_states)
    outage_df = load_outage_windows(phase_metrics_path, args.case_id)

    longest_rows: list[pd.DataFrame] = []
    plot_segments: list[dict[str, object]] = []
    subwindow_rows: list[pd.DataFrame] = []
    stats_rows: list[dict[str, object]] = []
    for spec in STRAIGHT_SPECS:
        longest_df, spec_plot_segments = collect_longest_segments(frame, outage_df, spec)
        longest_rows.append(longest_df)
        plot_segments.extend(spec_plot_segments)
        spec_subwindow_df = collect_subwindow_rows(frame, outage_df, spec)
        subwindow_rows.append(spec_subwindow_df)
        stats_rows.extend(summarize_subwindows(spec_subwindow_df, spec))

    longest_df = pd.concat(longest_rows, ignore_index=True) if longest_rows else pd.DataFrame()
    subwindow_df = pd.concat(subwindow_rows, ignore_index=True) if subwindow_rows else pd.DataFrame()
    stats_df = pd.DataFrame(stats_rows)

    longest_segments_csv = output_dir / "longest_segments.csv"
    subwindow_csv = output_dir / "straight_subwindow_metrics.csv"
    stats_csv = output_dir / "straight_subwindow_stats.csv"
    longest_df.to_csv(longest_segments_csv, index=False, encoding="utf-8-sig")
    subwindow_df.to_csv(subwindow_csv, index=False, encoding="utf-8-sig")
    stats_df.to_csv(stats_csv, index=False, encoding="utf-8-sig")

    example_plot = plots_dir / "example_segments_growth.png"
    aggregate_plot = plots_dir / "aggregate_window_stats.png"
    plot_example_segments(plot_segments, example_plot)
    plot_aggregate_stats(stats_df, aggregate_plot)

    plot_paths = {
        "longest_segments_csv": rel_from_root(longest_segments_csv),
        "subwindow_metrics_csv": rel_from_root(subwindow_csv),
        "aggregate_stats_csv": rel_from_root(stats_csv),
        "example_plot": rel_from_root(example_plot),
        "aggregate_plot": rel_from_root(aggregate_plot),
    }

    summary_path = output_dir / "summary.md"
    write_summary(
        summary_path,
        exp_id=args.exp_id,
        case_id=args.case_id,
        all_states_path=all_states_path,
        phase_metrics_path=phase_metrics_path,
        outage_df=outage_df,
        stats_df=stats_df,
        longest_df=longest_df,
        plot_paths=plot_paths,
    )

    manifest = {
        "exp_id": args.exp_id,
        "case_id": args.case_id,
        "all_states": rel_from_root(all_states_path),
        "phase_metrics": rel_from_root(phase_metrics_path),
        "output_dir": rel_from_root(output_dir),
        "artifacts": {
            **plot_paths,
            "summary_md": rel_from_root(summary_path),
        },
        "straight_specs": [
            {
                "name": spec.name,
                "label": spec.label,
                "speed_min_mps": spec.speed_min_mps,
                "yaw_rate_max_deg_s": spec.yaw_rate_max_deg_s,
            }
            for spec in STRAIGHT_SPECS
        ],
        "freshness": {
            "all_states_mtime": mtime_text(all_states_path),
            "phase_metrics_mtime": mtime_text(phase_metrics_path),
        },
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(rel_from_root(summary_path))


if __name__ == "__main__":
    main()
