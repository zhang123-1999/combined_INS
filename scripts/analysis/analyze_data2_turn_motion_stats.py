#!/usr/bin/env python3
"""Summarize turning-time ratio and left/right turn counts for data2."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (  # noqa: E402
    POS_PATH_DEFAULT,
    load_pos_dataframe,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-turn-motion-stats-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_motion_stats")
DEFAULT_SPEED_THRESHOLD_M_S = 3.0
DEFAULT_TURN_RATE_THRESHOLD_DEG_S = 8.0
DEFAULT_MIN_TURN_DURATION_S = 1.0
DEFAULT_SMOOTH_WINDOW_S = 0.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compute turning-time ratio and left/right turn counts for data2 POS trajectory."
    )
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--speed-threshold-m-s", type=float, default=DEFAULT_SPEED_THRESHOLD_M_S)
    parser.add_argument("--turn-rate-threshold-deg-s", type=float, default=DEFAULT_TURN_RATE_THRESHOLD_DEG_S)
    parser.add_argument("--min-turn-duration-s", type=float, default=DEFAULT_MIN_TURN_DURATION_S)
    parser.add_argument("--smooth-window-s", type=float, default=DEFAULT_SMOOTH_WINDOW_S)
    return parser


def build_sample_dt(t: np.ndarray) -> np.ndarray:
    if len(t) < 2:
        return np.zeros_like(t)
    dt_forward = np.diff(t, append=t[-1])
    dt_forward[-1] = float(np.median(np.diff(t)))
    return dt_forward


def compute_turn_signal_deg_s(t: np.ndarray, yaw_deg: np.ndarray, smooth_window_s: float) -> np.ndarray:
    yaw_rad = np.unwrap(np.deg2rad(yaw_deg))
    yaw_rate_deg_s = np.rad2deg(np.gradient(yaw_rad, t))
    if smooth_window_s <= 0.0 or len(t) < 3:
        return yaw_rate_deg_s
    dt_med = float(np.median(np.diff(t)))
    window_samples = max(3, int(round(smooth_window_s / max(dt_med, 1.0e-6))))
    min_periods = max(3, window_samples // 2)
    return (
        pd.Series(yaw_rate_deg_s)
        .rolling(window_samples, center=True, min_periods=min_periods)
        .mean()
        .to_numpy(dtype=float)
    )


def compute_curvature_cross_ne(t: np.ndarray, vn: np.ndarray, ve: np.ndarray) -> np.ndarray:
    an = np.gradient(vn, t)
    ae = np.gradient(ve, t)
    return vn * ae - ve * an


def turn_label_from_sign(sign: float) -> str:
    return "right" if sign > 0.0 else "left"


def build_segment_rows(
    t: np.ndarray,
    dt_s: np.ndarray,
    speed_m_s: np.ndarray,
    turn_signal_deg_s: np.ndarray,
    motion_mask: np.ndarray,
    turn_rate_threshold_deg_s: float,
    min_turn_duration_s: float,
) -> tuple[pd.DataFrame, np.ndarray]:
    signed_mask = motion_mask & np.isfinite(turn_signal_deg_s) & (np.abs(turn_signal_deg_s) >= turn_rate_threshold_deg_s)
    sign = np.sign(turn_signal_deg_s)
    sign[~signed_mask] = 0.0

    segments: list[dict[str, Any]] = []
    kept_mask = np.zeros(len(t), dtype=bool)

    start_idx: int | None = None
    current_sign = 0.0
    for idx, value in enumerate(sign):
        if value == 0.0:
            if start_idx is not None:
                end_idx = idx - 1
                duration_s = float(np.sum(dt_s[start_idx : end_idx + 1]))
                if duration_s >= min_turn_duration_s:
                    kept_mask[start_idx : end_idx + 1] = True
                    segments.append(
                        {
                            "segment_id": f"turn_segment_{len(segments) + 1}",
                            "turn_direction": turn_label_from_sign(current_sign),
                            "turn_sign": int(current_sign),
                            "start_t": float(t[start_idx]),
                            "end_t": float(t[end_idx] + dt_s[end_idx]),
                            "duration_s": duration_s,
                            "samples": int(end_idx - start_idx + 1),
                            "mean_speed_m_s": float(np.mean(speed_m_s[start_idx : end_idx + 1])),
                            "max_speed_m_s": float(np.max(speed_m_s[start_idx : end_idx + 1])),
                            "mean_yaw_rate_deg_s": float(np.mean(turn_signal_deg_s[start_idx : end_idx + 1])),
                            "mean_abs_yaw_rate_deg_s": float(np.mean(np.abs(turn_signal_deg_s[start_idx : end_idx + 1]))),
                            "max_abs_yaw_rate_deg_s": float(np.max(np.abs(turn_signal_deg_s[start_idx : end_idx + 1]))),
                        }
                    )
                start_idx = None
                current_sign = 0.0
            continue

        if start_idx is None:
            start_idx = idx
            current_sign = float(value)
            continue

        if value != current_sign:
            end_idx = idx - 1
            duration_s = float(np.sum(dt_s[start_idx : end_idx + 1]))
            if duration_s >= min_turn_duration_s:
                kept_mask[start_idx : end_idx + 1] = True
                segments.append(
                    {
                        "segment_id": f"turn_segment_{len(segments) + 1}",
                        "turn_direction": turn_label_from_sign(current_sign),
                        "turn_sign": int(current_sign),
                        "start_t": float(t[start_idx]),
                        "end_t": float(t[end_idx] + dt_s[end_idx]),
                        "duration_s": duration_s,
                        "samples": int(end_idx - start_idx + 1),
                        "mean_speed_m_s": float(np.mean(speed_m_s[start_idx : end_idx + 1])),
                        "max_speed_m_s": float(np.max(speed_m_s[start_idx : end_idx + 1])),
                        "mean_yaw_rate_deg_s": float(np.mean(turn_signal_deg_s[start_idx : end_idx + 1])),
                        "mean_abs_yaw_rate_deg_s": float(np.mean(np.abs(turn_signal_deg_s[start_idx : end_idx + 1]))),
                        "max_abs_yaw_rate_deg_s": float(np.max(np.abs(turn_signal_deg_s[start_idx : end_idx + 1]))),
                    }
                )
            start_idx = idx
            current_sign = float(value)

    if start_idx is not None:
        end_idx = len(t) - 1
        duration_s = float(np.sum(dt_s[start_idx : end_idx + 1]))
        if duration_s >= min_turn_duration_s:
            kept_mask[start_idx : end_idx + 1] = True
            segments.append(
                {
                    "segment_id": f"turn_segment_{len(segments) + 1}",
                    "turn_direction": turn_label_from_sign(current_sign),
                    "turn_sign": int(current_sign),
                    "start_t": float(t[start_idx]),
                    "end_t": float(t[end_idx] + dt_s[end_idx]),
                    "duration_s": duration_s,
                    "samples": int(end_idx - start_idx + 1),
                    "mean_speed_m_s": float(np.mean(speed_m_s[start_idx : end_idx + 1])),
                    "max_speed_m_s": float(np.max(speed_m_s[start_idx : end_idx + 1])),
                    "mean_yaw_rate_deg_s": float(np.mean(turn_signal_deg_s[start_idx : end_idx + 1])),
                    "mean_abs_yaw_rate_deg_s": float(np.mean(np.abs(turn_signal_deg_s[start_idx : end_idx + 1]))),
                    "max_abs_yaw_rate_deg_s": float(np.max(np.abs(turn_signal_deg_s[start_idx : end_idx + 1]))),
                }
            )

    return pd.DataFrame(segments), kept_mask


def summarize_segments(
    t: np.ndarray,
    dt_s: np.ndarray,
    motion_mask: np.ndarray,
    turn_mask: np.ndarray,
    segments_df: pd.DataFrame,
) -> dict[str, Any]:
    log_duration_s = float(t[-1] - t[0]) if len(t) >= 2 else 0.0
    motion_duration_s = float(np.sum(dt_s[motion_mask]))
    turn_duration_s = float(np.sum(dt_s[turn_mask]))
    left_segments = segments_df.loc[segments_df["turn_direction"] == "left"].copy()
    right_segments = segments_df.loc[segments_df["turn_direction"] == "right"].copy()

    left_turn_duration_s = float(left_segments["duration_s"].sum()) if not left_segments.empty else 0.0
    right_turn_duration_s = float(right_segments["duration_s"].sum()) if not right_segments.empty else 0.0

    return {
        "log_duration_s": log_duration_s,
        "motion_duration_s": motion_duration_s,
        "turn_duration_s": turn_duration_s,
        "turn_time_ratio_of_motion": turn_duration_s / motion_duration_s if motion_duration_s > 0.0 else math.nan,
        "turn_time_ratio_of_log": turn_duration_s / log_duration_s if log_duration_s > 0.0 else math.nan,
        "left_turn_duration_s": left_turn_duration_s,
        "right_turn_duration_s": right_turn_duration_s,
        "left_turn_ratio_of_motion": left_turn_duration_s / motion_duration_s if motion_duration_s > 0.0 else math.nan,
        "right_turn_ratio_of_motion": right_turn_duration_s / motion_duration_s if motion_duration_s > 0.0 else math.nan,
        "left_turn_count": int(len(left_segments)),
        "right_turn_count": int(len(right_segments)),
        "turn_segment_count": int(len(segments_df)),
        "mean_turn_segment_duration_s": float(segments_df["duration_s"].mean()) if not segments_df.empty else math.nan,
        "median_turn_segment_duration_s": float(segments_df["duration_s"].median()) if not segments_df.empty else math.nan,
    }


def build_sensitivity_rows(
    t: np.ndarray,
    dt_s: np.ndarray,
    speed_m_s: np.ndarray,
    turn_signal_deg_s: np.ndarray,
    motion_mask: np.ndarray,
    min_turn_duration_s: float,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for threshold in (5.0, 8.0, 10.0):
        segments_df, kept_mask = build_segment_rows(
            t=t,
            dt_s=dt_s,
            speed_m_s=speed_m_s,
            turn_signal_deg_s=turn_signal_deg_s,
            motion_mask=motion_mask,
            turn_rate_threshold_deg_s=threshold,
            min_turn_duration_s=min_turn_duration_s,
        )
        summary = summarize_segments(t=t, dt_s=dt_s, motion_mask=motion_mask, turn_mask=kept_mask, segments_df=segments_df)
        rows.append(
            {
                "turn_rate_threshold_deg_s": threshold,
                "turn_time_ratio_of_motion": float(summary["turn_time_ratio_of_motion"]),
                "left_turn_count": int(summary["left_turn_count"]),
                "right_turn_count": int(summary["right_turn_count"]),
                "turn_segment_count": int(summary["turn_segment_count"]),
                "turn_duration_s": float(summary["turn_duration_s"]),
            }
        )
    return pd.DataFrame(rows)


def write_summary(
    output_path: Path,
    pos_path: Path,
    summary: dict[str, Any],
    sensitivity_df: pd.DataFrame,
    sample_dt_median_s: float,
    smooth_window_s: float,
    speed_threshold_m_s: float,
    turn_rate_threshold_deg_s: float,
    min_turn_duration_s: float,
    curvature_match_rate: float,
) -> None:
    lines: list[str] = []
    lines.append("# data2 turn-motion statistics")
    lines.append("")
    lines.append("## Core result")
    lines.append(
        f"- Source trajectory: `{rel_from_root(pos_path, REPO_ROOT)}` with median sample dt "
        f"`{sample_dt_median_s:.6f} s`."
    )
    lines.append(
        f"- Motion is defined as `speed >= {speed_threshold_m_s:.1f} m/s`; turning is defined as "
        f"`|yaw_rate| >= {turn_rate_threshold_deg_s:.1f} deg/s` with minimum segment duration "
        f"`{min_turn_duration_s:.1f} s`."
    )
    lines.append(
        f"- Under this rule, turning occupies `{summary['turn_time_ratio_of_motion']:.4%}` of motion time "
        f"(`{summary['turn_duration_s']:.3f} s / {summary['motion_duration_s']:.3f} s`)."
    )
    lines.append(
        f"- Turn events: `left={summary['left_turn_count']}`, `right={summary['right_turn_count']}`, "
        f"`total={summary['turn_segment_count']}`."
    )
    lines.append(
        f"- Left-turn time: `{summary['left_turn_duration_s']:.3f} s` "
        f"(`{summary['left_turn_ratio_of_motion']:.4%}` of motion); right-turn time: "
        f"`{summary['right_turn_duration_s']:.3f} s` "
        f"(`{summary['right_turn_ratio_of_motion']:.4%}` of motion)."
    )
    lines.append("")
    lines.append("## Sign convention")
    lines.append(
        f"- `yaw_rate > 0` is labeled as `right turn`, `yaw_rate < 0` as `left turn`. "
        f"As a sanity check, POS `yaw_rate` sign matches the planar velocity-curvature sign "
        f"`{curvature_match_rate:.3f}` over moving samples with `|yaw_rate| >= 3 deg/s`, "
        "so the sign is consistent with clockwise/right-turn positive."
    )
    if smooth_window_s > 0.0:
        lines.append(f"- A centered rolling mean of `{smooth_window_s:.3f} s` was applied before thresholding.")
    else:
        lines.append("- No extra smoothing was applied before thresholding.")
    lines.append("")
    lines.append("## Threshold sensitivity")
    for row in sensitivity_df.itertuples(index=False):
        lines.append(
            f"- `|yaw_rate| >= {row.turn_rate_threshold_deg_s:.1f} deg/s`: "
            f"turn ratio=`{row.turn_time_ratio_of_motion:.4%}`, "
            f"`left/right/total={row.left_turn_count}/{row.right_turn_count}/{row.turn_segment_count}`, "
            f"turn duration=`{row.turn_duration_s:.3f} s`."
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    pos_path = (REPO_ROOT / args.pos_path).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    ensure_dir(output_dir)

    pos_df = load_pos_dataframe(pos_path)
    t = pos_df["t"].to_numpy(dtype=float)
    vn = pos_df["vn"].to_numpy(dtype=float)
    ve = pos_df["ve"].to_numpy(dtype=float)
    yaw_deg = pos_df["yaw"].to_numpy(dtype=float)
    speed_m_s = np.hypot(vn, ve)
    dt_s = build_sample_dt(t)
    sample_dt_median_s = float(np.median(np.diff(t))) if len(t) >= 2 else math.nan

    motion_mask = speed_m_s >= float(args.speed_threshold_m_s)
    turn_signal_deg_s = compute_turn_signal_deg_s(t=t, yaw_deg=yaw_deg, smooth_window_s=float(args.smooth_window_s))
    segments_df, kept_mask = build_segment_rows(
        t=t,
        dt_s=dt_s,
        speed_m_s=speed_m_s,
        turn_signal_deg_s=turn_signal_deg_s,
        motion_mask=motion_mask,
        turn_rate_threshold_deg_s=float(args.turn_rate_threshold_deg_s),
        min_turn_duration_s=float(args.min_turn_duration_s),
    )
    summary = summarize_segments(t=t, dt_s=dt_s, motion_mask=motion_mask, turn_mask=kept_mask, segments_df=segments_df)

    curvature_cross_ne = compute_curvature_cross_ne(t=t, vn=vn, ve=ve)
    sign_check_mask = motion_mask & np.isfinite(turn_signal_deg_s) & (np.abs(turn_signal_deg_s) >= 3.0)
    sign_check_mask &= np.isfinite(curvature_cross_ne) & (np.abs(curvature_cross_ne) >= 1.0e-3)
    if np.any(sign_check_mask):
        curvature_match_rate = float(
            np.mean(np.sign(turn_signal_deg_s[sign_check_mask]) == np.sign(curvature_cross_ne[sign_check_mask]))
        )
    else:
        curvature_match_rate = math.nan

    sensitivity_df = build_sensitivity_rows(
        t=t,
        dt_s=dt_s,
        speed_m_s=speed_m_s,
        turn_signal_deg_s=turn_signal_deg_s,
        motion_mask=motion_mask,
        min_turn_duration_s=float(args.min_turn_duration_s),
    )

    segments_path = output_dir / "turn_segments.csv"
    sensitivity_path = output_dir / "threshold_sensitivity.csv"
    summary_path = output_dir / "summary.md"
    manifest_path = output_dir / "manifest.json"

    segments_df.to_csv(segments_path, index=False, encoding="utf-8-sig")
    sensitivity_df.to_csv(sensitivity_path, index=False, encoding="utf-8-sig")
    write_summary(
        output_path=summary_path,
        pos_path=pos_path,
        summary=summary,
        sensitivity_df=sensitivity_df,
        sample_dt_median_s=sample_dt_median_s,
        smooth_window_s=float(args.smooth_window_s),
        speed_threshold_m_s=float(args.speed_threshold_m_s),
        turn_rate_threshold_deg_s=float(args.turn_rate_threshold_deg_s),
        min_turn_duration_s=float(args.min_turn_duration_s),
        curvature_match_rate=curvature_match_rate,
    )

    manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "parameters": {
            "speed_threshold_m_s": float(args.speed_threshold_m_s),
            "turn_rate_threshold_deg_s": float(args.turn_rate_threshold_deg_s),
            "min_turn_duration_s": float(args.min_turn_duration_s),
            "smooth_window_s": float(args.smooth_window_s),
        },
        "summary": summary,
        "curvature_match_rate": curvature_match_rate,
        "artifacts": {
            "turn_segments_csv": rel_from_root(segments_path, REPO_ROOT),
            "threshold_sensitivity_csv": rel_from_root(sensitivity_path, REPO_ROOT),
            "summary_md": rel_from_root(summary_path, REPO_ROOT),
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
