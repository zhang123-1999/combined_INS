#!/usr/bin/env python3
"""Audit whether stepwise local roll/pitch deltas explain outage dv_sf magnitude."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root
from scripts.analysis.run_data2_kf_gins_outage_roll_pitch_direction_audit import (
    CURRENT_SOL_DEFAULT,
    KF_NAV_DEFAULT,
    TRUTH_DEFAULT,
    acd_to_ned,
    decompose_to_acd,
    euler_to_rotation_deg,
    horizontal_cos_and_ratio,
    load_current_sol,
    load_kf_nav,
    load_truth,
)


EXP_ID_DEFAULT = "EXP-20260329-data2-kf-gins-outage-roll-pitch-stepwise-audit-r1"
OUTPUT_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_rpstep_r1"
CONTRIB_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_contrib_r1"
SHARED_STEP_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_substep_r1"

WINDOW_SPECS: dict[str, tuple[int, int]] = {
    "w2": (528270, 528280),
    "w1": (528180, 528210),
}


def truth_along_cross_basis_at(truth: pd.DataFrame, t: float) -> tuple[np.ndarray, np.ndarray]:
    along = np.array(
        [
            np.interp([t], truth["timestamp"].to_numpy(float), truth["vn_mps"].to_numpy(float))[0],
            np.interp([t], truth["timestamp"].to_numpy(float), truth["ve_mps"].to_numpy(float))[0],
        ],
        dtype=float,
    )
    norm = float(np.linalg.norm(along))
    if norm < 1e-9:
        raise ValueError(f"near-zero truth horizontal velocity at t={t}")
    along /= norm
    cross = np.array([-along[1], along[0]], dtype=float)
    return along, cross


def nearest_row(frame: pd.DataFrame, t: float) -> pd.Series:
    idx = int(np.abs(frame["timestamp"].to_numpy(float) - t).argmin())
    return frame.iloc[idx]


def predict_step_delta_acd(
    cur_roll_deg: float,
    cur_pitch_deg: float,
    kf_roll_deg: float,
    kf_pitch_deg: float,
    kf_yaw_deg: float,
    kf_step_acd: np.ndarray,
    along_2d: np.ndarray,
    cross_2d: np.ndarray,
) -> np.ndarray:
    kf_step_ned = acd_to_ned(kf_step_acd, along_2d, cross_2d)
    r_kf = euler_to_rotation_deg(kf_roll_deg, kf_pitch_deg, kf_yaw_deg)
    r_cur_rp = euler_to_rotation_deg(cur_roll_deg, cur_pitch_deg, kf_yaw_deg)
    pred_step_ned = r_cur_rp @ r_kf.T @ kf_step_ned - kf_step_ned
    return np.array(
        [
            decompose_to_acd(pred_step_ned, along_2d, cross_2d)["along"],
            decompose_to_acd(pred_step_ned, along_2d, cross_2d)["cross"],
            decompose_to_acd(pred_step_ned, along_2d, cross_2d)["down"],
        ],
        dtype=float,
    )


def accumulate_stepwise_prediction(step_inputs: Iterable[dict[str, Any]]) -> np.ndarray:
    total = np.zeros(3, dtype=float)
    for step in step_inputs:
        total += predict_step_delta_acd(**step)
    return total


def load_step_inputs(
    window_name: str,
    truth: pd.DataFrame,
    current_sol: pd.DataFrame,
    kf_nav: pd.DataFrame,
    contrib_dir: Path,
    shared_step_dir: Path,
) -> pd.DataFrame:
    contrib_path = contrib_dir / f"contributor_breakdown_{window_name}.csv"
    shared_path = shared_step_dir / f"shared_step_rows_{window_name}.csv"
    contrib_df = pd.read_csv(contrib_path)
    shared_df = pd.read_csv(shared_path)

    merged = contrib_df.merge(
        shared_df[["interval_start_t", "interval_end_t", "step_idx", "current_t", "kf_t"]],
        on=["interval_start_t", "interval_end_t", "step_idx"],
        how="left",
        validate="one_to_one",
    )
    if merged[["current_t", "kf_t"]].isna().any().any():
        raise RuntimeError(f"missing shared-step timestamps in merge for {window_name}")
    if not np.allclose(
        merged["t_curr"].to_numpy(float),
        merged["current_t"].to_numpy(float),
        atol=1e-6,
        rtol=0.0,
    ):
        raise RuntimeError(f"t_curr/current_t mismatch detected in {window_name}")

    rows: list[dict[str, Any]] = []
    for row in merged.itertuples(index=False):
        step_mid_t = float(row.current_t) - 0.5 * float(row.dt_current)
        along_2d, cross_2d = truth_along_cross_basis_at(truth, step_mid_t)
        cur_att = nearest_row(current_sol, float(row.current_t))
        kf_att = nearest_row(kf_nav, float(row.kf_t))

        kf_step_acd = np.array(
            [row.kf_dv_sf_along, row.kf_dv_sf_cross, row.kf_dv_sf_down],
            dtype=float,
        )
        obs_acd = np.array(
            [row.delta_dv_sf_along, row.delta_dv_sf_cross, row.delta_dv_sf_down],
            dtype=float,
        )
        pred_acd = predict_step_delta_acd(
            cur_roll_deg=float(cur_att["roll_deg"]),
            cur_pitch_deg=float(cur_att["pitch_deg"]),
            kf_roll_deg=float(kf_att["roll_deg"]),
            kf_pitch_deg=float(kf_att["pitch_deg"]),
            kf_yaw_deg=float(kf_att["yaw_deg"]),
            kf_step_acd=kf_step_acd,
            along_2d=along_2d,
            cross_2d=cross_2d,
        )
        cos_h, ratio_h = horizontal_cos_and_ratio(pred_acd, obs_acd)

        rows.append(
            {
                "window": window_name,
                "interval_start_t": float(row.interval_start_t),
                "interval_end_t": float(row.interval_end_t),
                "step_idx": int(row.step_idx),
                "current_t": float(row.current_t),
                "kf_t": float(row.kf_t),
                "step_mid_t": float(step_mid_t),
                "pred_along": float(pred_acd[0]),
                "pred_cross": float(pred_acd[1]),
                "pred_down": float(pred_acd[2]),
                "pred_norm": float(np.linalg.norm(pred_acd)),
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


def build_interval_rows(step_rows: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    grouped = step_rows.groupby(["window", "interval_start_t", "interval_end_t"], sort=True)
    for (window_name, start_t, end_t), group in grouped:
        pred = group[["pred_along", "pred_cross", "pred_down"]].sum().to_numpy(float)
        obs = group[["obs_along", "obs_cross", "obs_down"]].sum().to_numpy(float)
        cos_h, ratio_h = horizontal_cos_and_ratio(pred, obs)
        rows.append(
            {
                "window": window_name,
                "interval_start_t": float(start_t),
                "interval_end_t": float(end_t),
                "pred_along": float(pred[0]),
                "pred_cross": float(pred[1]),
                "pred_down": float(pred[2]),
                "pred_norm": float(np.linalg.norm(pred)),
                "obs_along": float(obs[0]),
                "obs_cross": float(obs[1]),
                "obs_down": float(obs[2]),
                "obs_norm": float(np.linalg.norm(obs)),
                "horizontal_cos": cos_h,
                "horizontal_norm_ratio": ratio_h,
                "n_steps": int(group.shape[0]),
            }
        )
    return pd.DataFrame(rows)


def build_window_summary(interval_rows: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for window_name, group in interval_rows.groupby("window", sort=True):
        pred_med = group[["pred_along", "pred_cross", "pred_down"]].median()
        obs_med = group[["obs_along", "obs_cross", "obs_down"]].median()
        rows.append(
            {
                "window": window_name,
                "mode": "stepwise_local_roll_pitch_only",
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
                "down_residual_median": float(np.median(group["pred_down"] - group["obs_down"])),
                "n_intervals": int(group.shape[0]),
                "n_steps_per_interval_median": int(np.median(group["n_steps"])),
            }
        )
    return pd.DataFrame(rows)


def build_summary(
    exp_id: str,
    inputs: dict[str, Path],
    window_summary: pd.DataFrame,
) -> list[str]:
    lines = [
        "# outage roll/pitch stepwise audit",
        "",
        f"- exp_id: `{exp_id}`",
        "- goal: tighten the roll/pitch-only model from one interval-level rotation to a shared-step local rotation",
        "- model: for each shared step, use the local truth along/cross basis at step midpoint, keep KF yaw fixed, replace only roll/pitch, rotate KF step dv_sf into current roll/pitch attitude, then accumulate predicted delta_dv_sf over each interval",
        "",
        "## Inputs",
    ]
    for key, value in inputs.items():
        lines.append(f"  - {key}: `{rel_from_root(value, REPO_ROOT)}`")
    lines.append("")

    lines.append("## Window Summary")
    for row in window_summary.itertuples(index=False):
        lines += [
            f"### {row.window}",
            f"  - median predicted delta_sf: along={row.pred_along_median:+.6f}  cross={row.pred_cross_median:+.6f}  down={row.pred_down_median:+.6f}  norm={row.pred_norm_median:.6f}",
            f"  - median observed  delta_sf: along={row.obs_along_median:+.6f}  cross={row.obs_cross_median:+.6f}  down={row.obs_down_median:+.6f}  norm={row.obs_norm_median:.6f}",
            f"  - median horizontal cos(pred, obs)={row.horizontal_cos_median:.3f}  median horizontal_norm_ratio={row.horizontal_norm_ratio_median:.3f}",
            f"  - median down residual (pred-obs)={row.down_residual_median:+.6f}",
            f"  - intervals={row.n_intervals}  shared_steps_per_interval≈{row.n_steps_per_interval_median}",
        ]
        lines.append("")

    lines += [
        "## Interpretation",
        "",
        "- If the stepwise local model materially improves horizontal_norm_ratio toward `1.0`, then the remaining gap was mainly an artifact of the old one-shot interval rotation.",
        "- If horizontal cosine stays high but horizontal_norm_ratio remains far from `1.0`, then roll/pitch still explains direction but not stepwise magnitude.",
        "- If predicted down remains near zero while observed down stays positive, the vertical residue is still not closed by local roll/pitch rotation alone.",
    ]
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Outage roll/pitch stepwise audit")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--output-dir", default=str(OUTPUT_DIR_DEFAULT), type=Path)
    parser.add_argument("--contrib-dir", default=str(CONTRIB_DIR_DEFAULT), type=Path)
    parser.add_argument("--shared-step-dir", default=str(SHARED_STEP_DIR_DEFAULT), type=Path)
    parser.add_argument("--sol", default=str(CURRENT_SOL_DEFAULT), type=Path)
    parser.add_argument("--kf-nav", default=str(KF_NAV_DEFAULT), type=Path)
    parser.add_argument("--truth", default=str(TRUTH_DEFAULT), type=Path)
    args = parser.parse_args()

    ensure_dir(args.output_dir)
    inputs = {
        "contrib_dir": args.contrib_dir,
        "shared_step_dir": args.shared_step_dir,
        "current_sol": args.sol,
        "kf_nav": args.kf_nav,
        "truth": args.truth,
    }
    for path in inputs.values():
        if not path.exists():
            print(f"ERROR: input not found: {path}", file=sys.stderr)
            return 1

    truth = load_truth(args.truth)
    current_sol = load_current_sol(args.sol)
    kf_nav = load_kf_nav(args.kf_nav)

    stepwise_frames: list[pd.DataFrame] = []
    interval_frames: list[pd.DataFrame] = []
    for window_name in WINDOW_SPECS:
        step_rows = load_step_inputs(
            window_name=window_name,
            truth=truth,
            current_sol=current_sol,
            kf_nav=kf_nav,
            contrib_dir=args.contrib_dir,
            shared_step_dir=args.shared_step_dir,
        )
        step_rows.to_csv(args.output_dir / f"stepwise_roll_pitch_prediction_{window_name}.csv", index=False)
        stepwise_frames.append(step_rows)

        interval_rows = build_interval_rows(step_rows)
        interval_rows.to_csv(args.output_dir / f"interval_stepwise_roll_pitch_prediction_{window_name}.csv", index=False)
        interval_frames.append(interval_rows)

    all_intervals = pd.concat(interval_frames, ignore_index=True)
    all_intervals.to_csv(args.output_dir / "interval_stepwise_roll_pitch_summary.csv", index=False)
    window_summary = build_window_summary(all_intervals)
    window_summary.to_csv(args.output_dir / "window_stepwise_roll_pitch_summary.csv", index=False)

    summary_lines = build_summary(args.exp_id, inputs, window_summary)
    (args.output_dir / "summary.md").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    manifest = {
        "exp_id": args.exp_id,
        "timestamp": dt.datetime.now().isoformat(),
        "inputs": {key: str(path) for key, path in inputs.items()},
        "outputs": {
            "stepwise_roll_pitch_prediction_w2": str(args.output_dir / "stepwise_roll_pitch_prediction_w2.csv"),
            "stepwise_roll_pitch_prediction_w1": str(args.output_dir / "stepwise_roll_pitch_prediction_w1.csv"),
            "interval_stepwise_roll_pitch_prediction_w2": str(args.output_dir / "interval_stepwise_roll_pitch_prediction_w2.csv"),
            "interval_stepwise_roll_pitch_prediction_w1": str(args.output_dir / "interval_stepwise_roll_pitch_prediction_w1.csv"),
            "interval_stepwise_roll_pitch_summary": str(args.output_dir / "interval_stepwise_roll_pitch_summary.csv"),
            "window_stepwise_roll_pitch_summary": str(args.output_dir / "window_stepwise_roll_pitch_summary.csv"),
            "summary": str(args.output_dir / "summary.md"),
        },
    }
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
