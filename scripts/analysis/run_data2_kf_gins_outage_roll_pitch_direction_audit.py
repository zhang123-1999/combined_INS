#!/usr/bin/env python3
"""Audit whether roll/pitch-only attitude differences explain outage dv_sf direction."""

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

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root


EXP_ID_DEFAULT = "EXP-20260329-data2-kf-gins-outage-roll-pitch-direction-audit-r1"
OUTPUT_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_rpdir_r1"
CONTRIB_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_contrib_r1"
CURRENT_SOL_DEFAULT = (
    REPO_ROOT / "output" / "d2_outage_fix_dbg_r4"
    / "SOL_data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed.txt"
)
KF_NAV_DEFAULT = (
    REPO_ROOT / "output" / "d2_kf_outage_dbg_r2" / "runtime_output" / "KF_GINS_Navresult.nav"
)
TRUTH_DEFAULT = REPO_ROOT / "dataset" / "data2_converted" / "POS_converted.txt"

WINDOW_SPECS: dict[str, tuple[int, int]] = {
    "w2": (528270, 528280),
    "w1": (528180, 528210),
}


def load_truth(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+", header=None).iloc[:, :10].copy()
    frame.columns = ["timestamp", "lat_deg", "lon_deg", "h_m",
                     "vn_mps", "ve_mps", "vd_mps",
                     "roll_deg", "pitch_deg", "yaw_deg"]
    return frame


def load_current_sol(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+")
    expected = [
        "timestamp", "fused_x", "fused_y", "fused_z",
        "fused_vx", "fused_vy", "fused_vz",
        "fused_roll", "fused_pitch", "fused_yaw",
        "mounting_pitch", "mounting_yaw", "odo_scale",
        "sg_x", "sg_y", "sg_z", "sa_x", "sa_y", "sa_z",
        "ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z",
        "lever_x", "lever_y", "lever_z",
        "gnss_lever_x", "gnss_lever_y", "gnss_lever_z",
    ]
    if len(frame.columns) == len(expected):
        frame.columns = expected
    frame["roll_deg"] = frame["fused_roll"]
    frame["pitch_deg"] = frame["fused_pitch"]
    frame["yaw_deg"] = frame["fused_yaw"]
    return frame


def load_kf_nav(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+", header=None).iloc[:, :11].copy()
    frame.columns = [
        "week", "timestamp",
        "lat_deg", "lon_deg", "h_m",
        "vn_mps", "ve_mps", "vd_mps",
        "roll_deg", "pitch_deg", "yaw_deg",
    ]
    return frame


def along_cross_basis(truth: pd.DataFrame, start_t: float, end_t: float) -> tuple[np.ndarray, np.ndarray]:
    mid_t = 0.5 * (start_t + end_t)
    along = np.array([
        np.interp([mid_t], truth["timestamp"].to_numpy(float), truth["vn_mps"].to_numpy(float))[0],
        np.interp([mid_t], truth["timestamp"].to_numpy(float), truth["ve_mps"].to_numpy(float))[0],
    ], dtype=float)
    norm = float(np.linalg.norm(along))
    if norm < 1e-9:
        raise ValueError(f"near-zero truth horizontal velocity at t={mid_t}")
    along /= norm
    cross = np.array([-along[1], along[0]], dtype=float)
    return along, cross


def euler_to_rotation_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def acd_to_ned(vec_acd: np.ndarray, along_2d: np.ndarray, cross_2d: np.ndarray) -> np.ndarray:
    return np.array(
        [
            vec_acd[0] * along_2d[0] + vec_acd[1] * cross_2d[0],
            vec_acd[0] * along_2d[1] + vec_acd[1] * cross_2d[1],
            vec_acd[2],
        ],
        dtype=float,
    )


def decompose_to_acd(vec_ned: np.ndarray, along_2d: np.ndarray, cross_2d: np.ndarray) -> dict[str, float]:
    return {
        "along": float(vec_ned[0] * along_2d[0] + vec_ned[1] * along_2d[1]),
        "cross": float(vec_ned[0] * cross_2d[0] + vec_ned[1] * cross_2d[1]),
        "down": float(vec_ned[2]),
        "norm": float(np.linalg.norm(vec_ned)),
    }


def predict_roll_pitch_only_delta_ned(
    cur_roll_deg: float,
    cur_pitch_deg: float,
    kf_roll_deg: float,
    kf_pitch_deg: float,
    kf_yaw_deg: float,
    kf_vec_ned: np.ndarray,
) -> np.ndarray:
    r_kf = euler_to_rotation_deg(kf_roll_deg, kf_pitch_deg, kf_yaw_deg)
    r_cur_rp = euler_to_rotation_deg(cur_roll_deg, cur_pitch_deg, kf_yaw_deg)
    return r_cur_rp @ r_kf.T @ kf_vec_ned - kf_vec_ned


def horizontal_cos_and_ratio(pred_acd: np.ndarray, obs_acd: np.ndarray) -> tuple[float, float]:
    pred_h = pred_acd[:2]
    obs_h = obs_acd[:2]
    pred_norm = float(np.linalg.norm(pred_h))
    obs_norm = float(np.linalg.norm(obs_h))
    if pred_norm < 1e-12 or obs_norm < 1e-12:
        return float("nan"), float("nan")
    cos = float(np.dot(pred_h, obs_h) / (pred_norm * obs_norm))
    ratio = pred_norm / obs_norm
    return cos, ratio


def pick_attitude_start(frame: pd.DataFrame, start_t: float) -> pd.Series:
    return frame[frame["timestamp"].astype(float) >= start_t - 0.1].iloc[0]


def build_window_start_rows(
    truth: pd.DataFrame,
    current_sol: pd.DataFrame,
    kf_nav: pd.DataFrame,
    summary_df: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for window_name, (start_t, end_t) in WINDOW_SPECS.items():
        along_2d, cross_2d = along_cross_basis(truth, start_t, end_t)
        cur_att = pick_attitude_start(current_sol, start_t)
        kf_att = pick_attitude_start(kf_nav, start_t)
        kf_row = summary_df[
            (summary_df["window"] == window_name)
            & (summary_df["solver"] == "kf")
            & (summary_df["contributor"] == "dv_sf")
        ].iloc[0]
        obs_row = summary_df[
            (summary_df["window"] == window_name)
            & (summary_df["solver"] == "delta")
            & (summary_df["contributor"] == "dv_sf")
        ].iloc[0]
        kf_acd = np.array([kf_row["cum_along"], kf_row["cum_cross"], kf_row["cum_down"]], dtype=float)
        obs_acd = np.array([obs_row["cum_along"], obs_row["cum_cross"], obs_row["cum_down"]], dtype=float)
        kf_ned = acd_to_ned(kf_acd, along_2d, cross_2d)
        pred_ned = predict_roll_pitch_only_delta_ned(
            float(cur_att["roll_deg"]),
            float(cur_att["pitch_deg"]),
            float(kf_att["roll_deg"]),
            float(kf_att["pitch_deg"]),
            float(kf_att["yaw_deg"]),
            kf_ned,
        )
        pred = decompose_to_acd(pred_ned, along_2d, cross_2d)
        cos_h, ratio_h = horizontal_cos_and_ratio(
            np.array([pred["along"], pred["cross"], pred["down"]], dtype=float),
            obs_acd,
        )
        rows.append(
            {
                "window": window_name,
                "mode": "window_start_roll_pitch_only",
                "pred_along": pred["along"],
                "pred_cross": pred["cross"],
                "pred_down": pred["down"],
                "pred_norm": pred["norm"],
                "obs_along": float(obs_acd[0]),
                "obs_cross": float(obs_acd[1]),
                "obs_down": float(obs_acd[2]),
                "obs_norm": float(np.linalg.norm(obs_acd)),
                "horizontal_cos": cos_h,
                "horizontal_norm_ratio": ratio_h,
                "cur_roll_deg": float(cur_att["roll_deg"]),
                "cur_pitch_deg": float(cur_att["pitch_deg"]),
                "kf_roll_deg": float(kf_att["roll_deg"]),
                "kf_pitch_deg": float(kf_att["pitch_deg"]),
                "kf_yaw_deg": float(kf_att["yaw_deg"]),
            }
        )
    return pd.DataFrame(rows)


def build_interval_rows(
    window_name: str,
    truth: pd.DataFrame,
    current_sol: pd.DataFrame,
    kf_nav: pd.DataFrame,
    step_df: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    grouped = step_df.groupby(["interval_start_t", "interval_end_t"], sort=True)
    for (start_t, end_t), group in grouped:
        along_2d, cross_2d = along_cross_basis(truth, float(start_t), float(end_t))
        cur_att = pick_attitude_start(current_sol, float(start_t))
        kf_att = pick_attitude_start(kf_nav, float(start_t))
        kf_acd = np.array(
            [
                group["kf_dv_sf_along"].sum(),
                group["kf_dv_sf_cross"].sum(),
                group["kf_dv_sf_down"].sum(),
            ],
            dtype=float,
        )
        obs_acd = np.array(
            [
                group["delta_dv_sf_along"].sum(),
                group["delta_dv_sf_cross"].sum(),
                group["delta_dv_sf_down"].sum(),
            ],
            dtype=float,
        )
        pred_ned = predict_roll_pitch_only_delta_ned(
            float(cur_att["roll_deg"]),
            float(cur_att["pitch_deg"]),
            float(kf_att["roll_deg"]),
            float(kf_att["pitch_deg"]),
            float(kf_att["yaw_deg"]),
            acd_to_ned(kf_acd, along_2d, cross_2d),
        )
        pred = decompose_to_acd(pred_ned, along_2d, cross_2d)
        cos_h, ratio_h = horizontal_cos_and_ratio(
            np.array([pred["along"], pred["cross"], pred["down"]], dtype=float),
            obs_acd,
        )
        rows.append(
            {
                "window": window_name,
                "interval_start_t": float(start_t),
                "interval_end_t": float(end_t),
                "pred_along": pred["along"],
                "pred_cross": pred["cross"],
                "pred_down": pred["down"],
                "pred_norm": pred["norm"],
                "obs_along": float(obs_acd[0]),
                "obs_cross": float(obs_acd[1]),
                "obs_down": float(obs_acd[2]),
                "obs_norm": float(np.linalg.norm(obs_acd)),
                "horizontal_cos": cos_h,
                "horizontal_norm_ratio": ratio_h,
                "cur_roll_deg": float(cur_att["roll_deg"]),
                "cur_pitch_deg": float(cur_att["pitch_deg"]),
                "kf_roll_deg": float(kf_att["roll_deg"]),
                "kf_pitch_deg": float(kf_att["pitch_deg"]),
                "kf_yaw_deg": float(kf_att["yaw_deg"]),
            }
        )
    return pd.DataFrame(rows)


def build_interval_summary(interval_rows: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for window_name, group in interval_rows.groupby("window", sort=True):
        pred_med = group[["pred_along", "pred_cross", "pred_down"]].median()
        obs_med = group[["obs_along", "obs_cross", "obs_down"]].median()
        rows.append(
            {
                "window": window_name,
                "mode": "interval_local_roll_pitch_only",
                "pred_along_median": float(pred_med["pred_along"]),
                "pred_cross_median": float(pred_med["pred_cross"]),
                "pred_down_median": float(pred_med["pred_down"]),
                "pred_norm_median": float(np.median(group["pred_norm"])),
                "obs_along_median": float(obs_med["obs_along"]),
                "obs_cross_median": float(obs_med["obs_cross"]),
                "obs_down_median": float(obs_med["obs_down"]),
                "obs_norm_median": float(np.median(group["obs_norm"])),
                "horizontal_cos_median": float(np.nanmedian(group["horizontal_cos"])),
                "horizontal_norm_ratio_median": float(np.nanmedian(group["horizontal_norm_ratio"])),
                "n_intervals": int(group.shape[0]),
            }
        )
    return pd.DataFrame(rows)


def build_summary(
    exp_id: str,
    inputs: dict[str, Path],
    window_start_rows: pd.DataFrame,
    interval_summary: pd.DataFrame,
) -> list[str]:
    lines = [
        "# outage roll/pitch direction audit",
        "",
        f"- exp_id: `{exp_id}`",
        "- goal: check whether roll/pitch-only attitude differences already explain the shared-chain dv_sf horizontal pattern",
        "- model: keep KF yaw fixed, replace only roll/pitch, rotate KF cumulative dv_sf into current roll/pitch attitude, compare predicted delta against observed delta_dv_sf",
        "",
        "## Inputs",
    ]
    for key, value in inputs.items():
        lines.append(f"  - {key}: `{rel_from_root(value, REPO_ROOT)}`")
    lines.append("")

    lines.append("## Window-Start Check")
    for row in window_start_rows.itertuples(index=False):
        lines += [
            f"### {row.window}",
            f"  - predicted delta_sf (roll/pitch only): along={row.pred_along:+.6f}  cross={row.pred_cross:+.6f}  down={row.pred_down:+.6f}  norm={row.pred_norm:.6f}",
            f"  - observed  delta_sf              : along={row.obs_along:+.6f}  cross={row.obs_cross:+.6f}  down={row.obs_down:+.6f}  norm={row.obs_norm:.6f}",
            f"  - horizontal cos(pred, obs)={row.horizontal_cos:.3f}  horizontal_norm_ratio={row.horizontal_norm_ratio:.3f}",
            f"  - roll/pitch current vs KF = {row.cur_roll_deg:.6f}/{row.cur_pitch_deg:.6f} vs {row.kf_roll_deg:.6f}/{row.kf_pitch_deg:.6f} deg",
        ]
        lines.append("")

    lines.append("## Interval-Local Check")
    for row in interval_summary.itertuples(index=False):
        lines += [
            f"### {row.window}",
            f"  - median predicted delta_sf: along={row.pred_along_median:+.6f}  cross={row.pred_cross_median:+.6f}  down={row.pred_down_median:+.6f}  norm={row.pred_norm_median:.6f}",
            f"  - median observed  delta_sf: along={row.obs_along_median:+.6f}  cross={row.obs_cross_median:+.6f}  down={row.obs_down_median:+.6f}  norm={row.obs_norm_median:.6f}",
            f"  - median horizontal cos(pred, obs)={row.horizontal_cos_median:.3f}  median horizontal_norm_ratio={row.horizontal_norm_ratio_median:.3f}",
            f"  - intervals={row.n_intervals}",
        ]
        lines.append("")

    lines += [
        "## Interpretation",
        "",
        "- If the roll/pitch-only prediction already gives the same horizontal sign pattern (`-along`, `+cross`), then yaw is not required to explain direction.",
        "- If interval-local horizontal cosine stays close to `1.0`, then local roll/pitch offsets are enough to explain the shared-chain horizontal direction.",
        "- Any remaining large `down` mismatch should be treated as a separate vertical/timing term, not as evidence against the horizontal roll/pitch explanation.",
    ]
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Outage roll/pitch direction audit")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--output-dir", default=str(OUTPUT_DIR_DEFAULT), type=Path)
    parser.add_argument("--contrib-dir", default=str(CONTRIB_DIR_DEFAULT), type=Path)
    parser.add_argument("--sol", default=str(CURRENT_SOL_DEFAULT), type=Path)
    parser.add_argument("--kf-nav", default=str(KF_NAV_DEFAULT), type=Path)
    parser.add_argument("--truth", default=str(TRUTH_DEFAULT), type=Path)
    args = parser.parse_args()

    ensure_dir(args.output_dir)
    inputs = {
        "contrib_dir": args.contrib_dir,
        "current_sol": args.sol,
        "kf_nav": args.kf_nav,
        "truth": args.truth,
    }
    for path in inputs.values():
        if not path.exists():
            print(f"ERROR: input not found: {path}", file=sys.stderr)
            return 1

    summary_path = args.contrib_dir / "contributor_summary.csv"
    if not summary_path.exists():
        print(f"ERROR: contributor summary not found: {summary_path}", file=sys.stderr)
        return 1

    truth = load_truth(args.truth)
    current_sol = load_current_sol(args.sol)
    kf_nav = load_kf_nav(args.kf_nav)
    summary_df = pd.read_csv(summary_path)

    window_start_rows = build_window_start_rows(truth, current_sol, kf_nav, summary_df)
    window_start_rows.to_csv(args.output_dir / "window_start_roll_pitch_prediction.csv", index=False)

    interval_frames: list[pd.DataFrame] = []
    for window_name in WINDOW_SPECS:
        step_path = args.contrib_dir / f"contributor_breakdown_{window_name}.csv"
        if not step_path.exists():
            print(f"ERROR: missing contributor breakdown: {step_path}", file=sys.stderr)
            return 1
        step_df = pd.read_csv(step_path)
        interval_rows = build_interval_rows(window_name, truth, current_sol, kf_nav, step_df)
        interval_rows.to_csv(args.output_dir / f"interval_local_roll_pitch_prediction_{window_name}.csv", index=False)
        interval_frames.append(interval_rows)

    all_intervals = pd.concat(interval_frames, ignore_index=True)
    interval_summary = build_interval_summary(all_intervals)
    interval_summary.to_csv(args.output_dir / "interval_local_roll_pitch_summary.csv", index=False)

    summary_lines = build_summary(args.exp_id, inputs, window_start_rows, interval_summary)
    (args.output_dir / "summary.md").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    manifest = {
        "exp_id": args.exp_id,
        "timestamp": dt.datetime.now().isoformat(),
        "inputs": {key: str(path) for key, path in inputs.items()},
        "outputs": {
            "window_start_roll_pitch_prediction": str(args.output_dir / "window_start_roll_pitch_prediction.csv"),
            "interval_local_roll_pitch_prediction_w2": str(args.output_dir / "interval_local_roll_pitch_prediction_w2.csv"),
            "interval_local_roll_pitch_prediction_w1": str(args.output_dir / "interval_local_roll_pitch_prediction_w1.csv"),
            "interval_local_roll_pitch_summary": str(args.output_dir / "interval_local_roll_pitch_summary.csv"),
            "summary": str(args.output_dir / "summary.md"),
        },
    }
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
