from __future__ import annotations

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-yaw-oscillation-audit-r1"
CONFIG_DEFAULT = Path(
    "output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r3_axis_hard_freeze/"
    "artifacts/cases/release_mounting_yaw/config_release_mounting_yaw.yaml"
)
STATE_SERIES_DEFAULT = Path(
    "output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r3_axis_hard_freeze/"
    "artifacts/cases/release_mounting_yaw/state_series_release_mounting_yaw.csv"
)
DIAG_DEFAULT = Path(
    "output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r3_axis_hard_freeze/"
    "artifacts/cases/release_mounting_yaw/DIAG_release_mounting_yaw.txt"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_oscillation_20260322")


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT)).replace("\\", "/")
    except ValueError:
        return str(path).replace("\\", "/")


def mtime_text(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")


def classify_motion(speed_m_s: np.ndarray, yaw_rate_deg_s: np.ndarray) -> np.ndarray:
    labels = np.full(speed_m_s.shape, "transition", dtype=object)
    labels[speed_m_s < 3.0] = "low_speed"
    labels[(speed_m_s >= 3.0) & (np.abs(yaw_rate_deg_s) <= 3.0)] = "straight"
    labels[(speed_m_s >= 3.0) & (yaw_rate_deg_s >= 8.0)] = "turn_pos"
    labels[(speed_m_s >= 3.0) & (yaw_rate_deg_s <= -8.0)] = "turn_neg"
    return labels


def compute_gnss_schedule_mask(state_t: np.ndarray, fusion_cfg: dict) -> np.ndarray:
    schedule = fusion_cfg.get("gnss_schedule", {})
    initial_on = float(schedule.get("initial_on_duration", 300.0))
    off_duration = float(schedule.get("off_duration", 100.0))
    on_duration = float(schedule.get("on_duration", 150.0))
    cycle_start = str(schedule.get("cycle_start", "off"))
    t_rel = state_t - state_t[0]
    mask = np.ones_like(state_t, dtype=bool)
    if cycle_start != "off":
        return mask
    cycle = off_duration + on_duration
    mask = t_rel < initial_on
    rem = ~mask
    phase = np.mod(t_rel[rem] - initial_on, cycle)
    mask[rem] = phase >= off_duration
    return mask


def summarize_series(values: np.ndarray) -> dict[str, float]:
    return {
        "mean_deg": float(np.mean(values)),
        "std_deg": float(np.std(values)),
        "min_deg": float(np.min(values)),
        "max_deg": float(np.max(values)),
        "p05_deg": float(np.percentile(values, 5.0)),
        "p50_deg": float(np.percentile(values, 50.0)),
        "p95_deg": float(np.percentile(values, 95.0)),
        "peak_to_peak_deg": float(np.ptp(values)),
    }


def render_table(df: pd.DataFrame, columns: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for _, row in df.iterrows():
        vals: list[str] = []
        for col in columns:
            value = row[col]
            if isinstance(value, str):
                vals.append(value)
            elif isinstance(value, (int, np.integer)):
                vals.append(str(int(value)))
            else:
                vals.append(f"{float(value):.6f}")
        lines.append("| " + " | ".join(vals) + " |")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit mounting yaw oscillation source and scale.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--config", type=Path, default=CONFIG_DEFAULT)
    parser.add_argument("--state-series", type=Path, default=STATE_SERIES_DEFAULT)
    parser.add_argument("--diag", type=Path, default=DIAG_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    config_path = (REPO_ROOT / args.config).resolve()
    state_series_path = (REPO_ROOT / args.state_series).resolve()
    diag_path = (REPO_ROOT / args.diag).resolve()
    pos_path = (REPO_ROOT / args.pos).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    ensure_dir(output_dir)

    raw_cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    fusion_cfg = raw_cfg["fusion"]
    state_df = pd.read_csv(state_series_path)
    diag_df = pd.read_csv(diag_path, sep=r"\s+")
    pos_df = pd.read_csv(
        pos_path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )

    state_t = state_df["timestamp"].to_numpy(dtype=float)
    yaw_deg = state_df["total_mounting_yaw_deg"].to_numpy(dtype=float)
    pos_t = pos_df["t"].to_numpy(dtype=float)
    yaw_truth = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.interp(state_t, pos_t, np.rad2deg(np.gradient(yaw_truth, pos_t)))
    speed_m_s = np.interp(
        state_t,
        pos_t,
        np.hypot(pos_df["vn"].to_numpy(dtype=float), pos_df["ve"].to_numpy(dtype=float)),
    )
    motion_label = classify_motion(speed_m_s, yaw_rate_deg_s)
    gnss_on = compute_gnss_schedule_mask(state_t, fusion_cfg)

    global_stats = summarize_series(yaw_deg)
    after_200_mask = (state_t - state_t[0]) >= 200.0
    after_200_stats = summarize_series(yaw_deg[after_200_mask])
    final_deg = float(yaw_deg[-1])

    motion_rows: list[dict[str, float | int | str]] = []
    for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]:
        mask = motion_label == label
        if not np.any(mask):
            continue
        stats = summarize_series(yaw_deg[mask])
        motion_rows.append(
            {
                "motion_label": label,
                "n": int(mask.sum()),
                "count_ratio": float(mask.mean()),
                "mean_yaw_deg": stats["mean_deg"],
                "std_yaw_deg": stats["std_deg"],
                "p05_deg": stats["p05_deg"],
                "p95_deg": stats["p95_deg"],
                "mean_speed_mps": float(np.mean(speed_m_s[mask])),
                "mean_yaw_rate_deg_s": float(np.mean(yaw_rate_deg_s[mask])),
            }
        )
    motion_df = pd.DataFrame(motion_rows)

    schedule_rows: list[dict[str, float | int | str]] = []
    for label, mask in [("gnss_on", gnss_on), ("gnss_off", ~gnss_on)]:
        stats = summarize_series(yaw_deg[mask])
        schedule_rows.append(
            {
                "schedule_label": label,
                "n": int(mask.sum()),
                "count_ratio": float(mask.mean()),
                "mean_yaw_deg": stats["mean_deg"],
                "std_yaw_deg": stats["std_deg"],
                "p05_deg": stats["p05_deg"],
                "p95_deg": stats["p95_deg"],
            }
        )
    schedule_df = pd.DataFrame(schedule_rows)

    dy = np.diff(yaw_deg)
    dt = np.diff(state_t)
    rate = dy / np.maximum(dt, 1e-9)
    step_stats = {
        "abs_step_p95_deg": float(np.percentile(np.abs(dy), 95.0)),
        "abs_step_max_deg": float(np.max(np.abs(dy))),
        "abs_rate_p95_deg_s": float(np.percentile(np.abs(rate), 95.0)),
        "abs_rate_max_deg_s": float(np.max(np.abs(rate))),
    }

    std_my_deg = diag_df["std_my"].to_numpy(dtype=float) * 180.0 / math.pi
    covariance_stats = {
        "mean_std_my_deg": float(np.mean(std_my_deg)),
        "median_std_my_deg": float(np.median(std_my_deg)),
        "p95_std_my_deg": float(np.percentile(std_my_deg, 95.0)),
        "max_std_my_deg": float(np.max(std_my_deg)),
        "final_std_my_deg": float(std_my_deg[-1]),
    }

    sigma_mounting_yaw = float(fusion_cfg["noise"]["sigma_mounting_yaw"])
    dataset_duration = float(state_t[-1] - state_t[0])
    process_noise_theory = {
        "sigma_mounting_yaw_rad_sqrtHz": sigma_mounting_yaw,
        "sigma_mounting_yaw_deg_sqrtHz": float(sigma_mounting_yaw * 180.0 / math.pi),
        "rw_1s_deg": float(sigma_mounting_yaw * math.sqrt(1.0) * 180.0 / math.pi),
        "rw_10s_deg": float(sigma_mounting_yaw * math.sqrt(10.0) * 180.0 / math.pi),
        "rw_100s_deg": float(sigma_mounting_yaw * math.sqrt(100.0) * 180.0 / math.pi),
        "rw_total_1sigma_deg": float(sigma_mounting_yaw * math.sqrt(dataset_duration) * 180.0 / math.pi),
        "dataset_duration_s": dataset_duration,
        "max_mounting_step_deg": float(fusion_cfg["constraints"].get("max_mounting_step_deg", 0.5)),
        "p_floor_mounting_deg": float(fusion_cfg["constraints"].get("p_floor_mounting_deg", 0.1)),
        "std_mounting_yaw_init_deg": float(fusion_cfg["init"].get("std_mounting_yaw", 0.0)),
    }

    summary = {
        "exp_id": args.exp_id,
        "series_column": "total_mounting_yaw_deg",
        "global_stats": global_stats | {"final_deg": final_deg},
        "after_200s_stats": after_200_stats,
        "motion_stats": motion_rows,
        "schedule_stats": schedule_rows,
        "step_stats": step_stats,
        "covariance_stats": covariance_stats,
        "process_noise_theory": process_noise_theory,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    motion_df.to_csv(output_dir / "motion_breakdown.csv", index=False)
    schedule_df.to_csv(output_dir / "schedule_breakdown.csv", index=False)

    lines = [
        "# mounting_yaw oscillation audit",
        "",
        f"Date: `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{args.exp_id}`",
        f"- config: `{rel(config_path)}`",
        f"- state series: `{rel(state_series_path)}`",
        f"- diag: `{rel(diag_path)}`",
        f"- truth motion: `{rel(pos_path)}`",
        "",
        "## Global stats",
        "",
        f"- `total_mounting_yaw_deg` overall: mean `{global_stats['mean_deg']:.6f} deg`, std `{global_stats['std_deg']:.6f} deg`, min/max `{global_stats['min_deg']:.6f}/{global_stats['max_deg']:.6f} deg`, peak-to-peak `{global_stats['peak_to_peak_deg']:.6f} deg`, final `{final_deg:.6f} deg`.",
        f"- After first `200 s`: std `{after_200_stats['std_deg']:.6f} deg`, `p05/p95 = {after_200_stats['p05_deg']:.6f}/{after_200_stats['p95_deg']:.6f} deg`, peak-to-peak `{after_200_stats['peak_to_peak_deg']:.6f} deg`.",
        f"- Pointwise step size is small most of the time: `|Δyaw| p95 = {step_stats['abs_step_p95_deg']:.6f} deg`, `|Δyaw| max = {step_stats['abs_step_max_deg']:.6f} deg`; rare transients exist but the trace is not high-frequency white jitter.",
        "",
        "## Motion coupling",
        "",
        "- If this were dominated by process noise, the mean should not shift strongly with motion class. The actual trace does shift strongly with turn direction.",
        "",
    ]
    lines.extend(
        render_table(
            motion_df,
            ["motion_label", "count_ratio", "mean_yaw_deg", "std_yaw_deg", "p05_deg", "p95_deg", "mean_speed_mps", "mean_yaw_rate_deg_s"],
        )
    )
    lines.extend(
        [
            "",
            "## GNSS schedule coupling",
            "",
        ]
    )
    lines.extend(
        render_table(
            schedule_df,
            ["schedule_label", "count_ratio", "mean_yaw_deg", "std_yaw_deg", "p05_deg", "p95_deg"],
        )
    )
    lines.extend(
        [
            "",
            "## Covariance vs actual motion",
            "",
            f"- `std_my` from diagnostic covariance: median `{covariance_stats['median_std_my_deg']:.6f} deg`, p95 `{covariance_stats['p95_std_my_deg']:.6f} deg`, final `{covariance_stats['final_std_my_deg']:.6f} deg`.",
            f"- Actual state variation is much larger than covariance-reported uncertainty: trace std `{global_stats['std_deg']:.6f} deg`, motion-conditioned mean shift `turn_pos -> turn_neg = {motion_df.loc[motion_df['motion_label']=='turn_neg','mean_yaw_deg'].iloc[0] - motion_df.loc[motion_df['motion_label']=='turn_pos','mean_yaw_deg'].iloc[0]:.6f} deg`.",
            "",
            "## Process-noise scale check",
            "",
            f"- `sigma_mounting_yaw = {process_noise_theory['sigma_mounting_yaw_rad_sqrtHz']:.6e} rad/sqrt(Hz) = {process_noise_theory['sigma_mounting_yaw_deg_sqrtHz']:.6f} deg/sqrt(Hz)`.",
            f"- Equivalent random-walk 1-sigma: `1 s = {process_noise_theory['rw_1s_deg']:.6f} deg`, `10 s = {process_noise_theory['rw_10s_deg']:.6f} deg`, `100 s = {process_noise_theory['rw_100s_deg']:.6f} deg`, full-run `{process_noise_theory['rw_total_1sigma_deg']:.6f} deg`.",
            f"- Guardrails: `std_mounting_yaw_init = {process_noise_theory['std_mounting_yaw_init_deg']:.6f} deg`, `p_floor_mounting_deg = {process_noise_theory['p_floor_mounting_deg']:.6f} deg`, `max_mounting_step_deg = {process_noise_theory['max_mounting_step_deg']:.6f} deg`.",
            "",
            "## Assessment",
            "",
            "- The observed `~0.6 deg` peak-to-peak is real, but it is not a uniform random jitter. The trace follows motion class: `turn_pos` pushes the estimate down, `turn_neg` pushes it up, while `low_speed` stays very stable.",
            "- GNSS on/off only changes the mean by about `0.009 deg`, much smaller than the turn-conditioned shift. So the main driver is not GNSS schedule.",
            "- Current process noise is not negligible, but the stronger evidence points to measurement-driven motion coupling, i.e. the state is being pulled by turn-direction-dependent NHC optimum changes on top of the straight-line `0.84 deg` basin.",
            "- In short: `sigma_mounting_yaw` may be somewhat loose, but it is probably not the primary root cause of the large envelope. The first-order cause is still model/data mismatch in the yaw channel, especially turn asymmetry.",
            "",
            "## Freshness",
            "",
            f"- config mtime: `{mtime_text(config_path)}`",
            f"- state series mtime: `{mtime_text(state_series_path)}`",
            f"- diag mtime: `{mtime_text(diag_path)}`",
            f"- POS mtime: `{mtime_text(pos_path)}`",
            f"- generated artifacts: `{rel(output_dir / 'summary.md')}`, `{rel(output_dir / 'summary.json')}`, `{rel(output_dir / 'motion_breakdown.csv')}`, `{rel(output_dir / 'schedule_breakdown.csv')}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
