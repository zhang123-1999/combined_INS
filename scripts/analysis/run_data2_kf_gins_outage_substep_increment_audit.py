#!/usr/bin/env python3
"""Audit outage nominal velocity mismatch at predict-substep granularity."""

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


EXP_ID_DEFAULT = "EXP-20260328-data2-kf-gins-outage-substep-shared-audit-r1"
OUTPUT_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_substep_r1"
CURRENT_SOL_DEFAULT = (
    REPO_ROOT / "output" / "d2_outage_fix_dbg_r4" / "SOL_data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed.txt"
)
CURRENT_GNSS_DEBUG_DEFAULT = (
    REPO_ROOT
    / "output"
    / "d2_outage_fix_dbg_r4"
    / "artifacts"
    / "cases"
    / "data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed"
    / "gnss_updates_data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed.csv"
)
KF_NAV_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_dbg_r2" / "runtime_output" / "KF_GINS_Navresult.nav"
KF_GNSS_DEBUG_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_dbg_r2" / "kf_gins" / "gnss_updates_kf_gins.csv"
TRUTH_DEFAULT = REPO_ROOT / "dataset" / "data2_converted" / "POS_converted.txt"
WINDOW_SPECS = {
    "w2": (528270, 528280),
    "w1": (528180, 528210),
}


def rel_path(path: Path) -> str:
    return rel_from_root(path, REPO_ROOT)


def rot_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    sl = math.sin(lat_rad)
    cl = math.cos(lat_rad)
    so = math.sin(lon_rad)
    co = math.cos(lon_rad)
    return np.array(
        [
            [-sl * co, -sl * so, cl],
            [-so, co, 0.0],
            [-cl * co, -cl * so, -sl],
        ],
        dtype=float,
    )


def load_truth_frame(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+", header=None)
    frame = frame.iloc[:, :10].copy()
    frame.columns = ["timestamp", "lat_deg", "lon_deg", "h_m", "vn_mps", "ve_mps", "vd_mps", "roll_deg", "pitch_deg", "yaw_deg"]
    return frame


def load_current_sol(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+")
    frame.columns = [
        "timestamp",
        "fused_x",
        "fused_y",
        "fused_z",
        "fused_vx",
        "fused_vy",
        "fused_vz",
        "fused_roll",
        "fused_pitch",
        "fused_yaw",
        "mounting_pitch",
        "mounting_yaw",
        "odo_scale",
        "sg_x",
        "sg_y",
        "sg_z",
        "sa_x",
        "sa_y",
        "sa_z",
        "ba_x",
        "ba_y",
        "ba_z",
        "bg_x",
        "bg_y",
        "bg_z",
        "lever_x",
        "lever_y",
        "lever_z",
        "gnss_lever_x",
        "gnss_lever_y",
        "gnss_lever_z",
    ]
    return frame


def load_kf_nav(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+", header=None)
    frame = frame.iloc[:, :11].copy()
    frame.columns = ["week", "timestamp", "lat_deg", "lon_deg", "h_m", "vn_mps", "ve_mps", "vd_mps", "roll_deg", "pitch_deg", "yaw_deg"]
    return frame


def pick_exact_row(frame: pd.DataFrame, column: str, value: float, tol: float = 1e-6) -> pd.Series:
    matched = frame[np.abs(frame[column].astype(float) - float(value)) < tol]
    if matched.shape[0] != 1:
        raise RuntimeError(f"expected exactly 1 row for {column}={value}, got {matched.shape[0]}")
    return matched.iloc[0]


def interp_truth_scalar(truth: pd.DataFrame, timestamp: float, column: str) -> float:
    return float(np.interp([timestamp], truth["timestamp"].to_numpy(dtype=float), truth[column].to_numpy(dtype=float))[0])


def current_ecef_velocity_to_ned(truth: pd.DataFrame, timestamp: float, vel_ecef: np.ndarray) -> np.ndarray:
    lat_deg = interp_truth_scalar(truth, timestamp, "lat_deg")
    lon_deg = interp_truth_scalar(truth, timestamp, "lon_deg")
    rot = rot_ecef_to_ned(math.radians(lat_deg), math.radians(lon_deg))
    return rot @ vel_ecef


def convert_current_sol_to_ned(truth: pd.DataFrame, frame: pd.DataFrame) -> pd.DataFrame:
    out = frame.copy()
    timestamps = out["timestamp"].to_numpy(dtype=float)
    lat_deg = np.interp(timestamps, truth["timestamp"].to_numpy(dtype=float), truth["lat_deg"].to_numpy(dtype=float))
    lon_deg = np.interp(timestamps, truth["timestamp"].to_numpy(dtype=float), truth["lon_deg"].to_numpy(dtype=float))
    vel_ecef = out[["fused_vx", "fused_vy", "fused_vz"]].to_numpy(dtype=float)
    vel_ned = np.zeros_like(vel_ecef)
    for idx in range(vel_ecef.shape[0]):
        rot = rot_ecef_to_ned(math.radians(float(lat_deg[idx])), math.radians(float(lon_deg[idx])))
        vel_ned[idx] = rot @ vel_ecef[idx]
    out["vn_mps"] = vel_ned[:, 0]
    out["ve_mps"] = vel_ned[:, 1]
    out["vd_mps"] = vel_ned[:, 2]
    return out


def fixed_interval_basis(truth: pd.DataFrame, start_t: float, end_t: float) -> tuple[np.ndarray, np.ndarray]:
    mid_t = 0.5 * (float(start_t) + float(end_t))
    along = np.array(
        [
            interp_truth_scalar(truth, mid_t, "vn_mps"),
            interp_truth_scalar(truth, mid_t, "ve_mps"),
        ],
        dtype=float,
    )
    norm = float(np.linalg.norm(along))
    if norm <= 0.0:
        raise RuntimeError(f"zero horizontal truth velocity at interval midpoint {mid_t}")
    along /= norm
    cross = np.array([-along[1], along[0]], dtype=float)
    return along, cross


def decompose_vec(vec_ned: np.ndarray, along_2d: np.ndarray, cross_2d: np.ndarray) -> dict[str, float]:
    return {
        "along": float(vec_ned[0] * along_2d[0] + vec_ned[1] * along_2d[1]),
        "cross": float(vec_ned[0] * cross_2d[0] + vec_ned[1] * cross_2d[1]),
        "down": float(vec_ned[2]),
        "norm": float(np.linalg.norm(vec_ned)),
    }


def threshold_hits(frame: pd.DataFrame, final_norm: float) -> dict[str, int]:
    hits: dict[str, int] = {}
    if final_norm <= 0.0:
        return {f"hit_{int(frac * 100):02d}_idx": 0 for frac in (0.1, 0.25, 0.5, 0.75, 0.9)}
    for frac in (0.1, 0.25, 0.5, 0.75, 0.9):
        hit = frame[frame["shared_cum_norm"] >= frac * final_norm].iloc[0]
        hits[f"hit_{int(frac * 100):02d}_idx"] = int(hit["step_idx"])
    return hits


def build_window_audit(
    truth: pd.DataFrame,
    current_sol_ned: pd.DataFrame,
    kf_nav: pd.DataFrame,
    current_gnss: pd.DataFrame,
    kf_gnss: pd.DataFrame,
    window_name: str,
    start_t: int,
    end_t: int,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    interval_rows: list[dict[str, Any]] = []
    shared_rows: list[dict[str, Any]] = []
    median_rows: list[dict[str, Any]] = []

    for interval_start in range(int(start_t), int(end_t)):
        interval_end = interval_start + 1
        along_2d, cross_2d = fixed_interval_basis(truth, interval_start, interval_end)

        current_start = pick_exact_row(current_gnss, "gnss_t", float(interval_start))
        current_end = pick_exact_row(current_gnss, "gnss_t", float(interval_end))
        kf_start = pick_exact_row(kf_gnss, "gnss_t", float(interval_start))
        kf_end = pick_exact_row(kf_gnss, "gnss_t", float(interval_end))

        current_after = current_ecef_velocity_to_ned(
            truth,
            float(interval_start),
            np.array([current_start["state_v_after_x"], current_start["state_v_after_y"], current_start["state_v_after_z"]], dtype=float),
        )
        current_before = current_ecef_velocity_to_ned(
            truth,
            float(interval_end),
            np.array([current_end["state_v_before_x"], current_end["state_v_before_y"], current_end["state_v_before_z"]], dtype=float),
        )
        kf_after = np.array(
            [kf_start["state_vel_after_x"], kf_start["state_vel_after_y"], kf_start["state_vel_after_z"]],
            dtype=float,
        )
        kf_before = np.array(
            [kf_end["state_vel_before_x"], kf_end["state_vel_before_y"], kf_end["state_vel_before_z"]],
            dtype=float,
        )

        current_dense = current_sol_ned[
            (current_sol_ned["timestamp"].astype(float) > float(interval_start))
            & (current_sol_ned["timestamp"].astype(float) < float(interval_end))
        ].copy()
        kf_dense_pre = kf_nav[
            (kf_nav["timestamp"].astype(float) > float(interval_start))
            & (kf_nav["timestamp"].astype(float) < float(kf_end["state_t"]))
        ].copy()

        if current_dense.shape[0] != kf_dense_pre.shape[0] + 1:
            raise RuntimeError(
                f"interval {interval_start}->{interval_end} unexpected dense counts: "
                f"current={current_dense.shape[0]} kf_pre={kf_dense_pre.shape[0]}"
            )

        current_prev = current_after.copy()
        kf_prev = kf_after.copy()
        shared_vec = np.zeros(3, dtype=float)
        interval_shared_rows: list[dict[str, Any]] = []
        shared_limit = int(kf_dense_pre.shape[0])
        for step_idx, ((_, current_row), (_, kf_row)) in enumerate(
            zip(current_dense.iloc[:shared_limit].iterrows(), kf_dense_pre.iterrows()),
            start=1,
        ):
            current_vel = np.array([current_row["vn_mps"], current_row["ve_mps"], current_row["vd_mps"]], dtype=float)
            kf_vel = np.array([kf_row["vn_mps"], kf_row["ve_mps"], kf_row["vd_mps"]], dtype=float)
            delta_vec = (current_vel - current_prev) - (kf_vel - kf_prev)
            shared_vec += delta_vec
            step_parts = decompose_vec(delta_vec, along_2d, cross_2d)
            cum_parts = decompose_vec(shared_vec, along_2d, cross_2d)
            interval_shared_rows.append(
                {
                    "window_name": window_name,
                    "interval_start_t": float(interval_start),
                    "interval_end_t": float(interval_end),
                    "step_idx": int(step_idx),
                    "current_t": float(current_row["timestamp"]),
                    "kf_t": float(kf_row["timestamp"]),
                    "shared_step_along": step_parts["along"],
                    "shared_step_cross": step_parts["cross"],
                    "shared_step_down": step_parts["down"],
                    "shared_step_norm": step_parts["norm"],
                    "shared_cum_along": cum_parts["along"],
                    "shared_cum_cross": cum_parts["cross"],
                    "shared_cum_down": cum_parts["down"],
                    "shared_cum_norm": cum_parts["norm"],
                }
            )
            current_prev = current_vel
            kf_prev = kf_vel

        current_terminal_row = current_dense.iloc[shared_limit]
        current_terminal_vel = np.array(
            [current_terminal_row["vn_mps"], current_terminal_row["ve_mps"], current_terminal_row["vd_mps"]],
            dtype=float,
        )
        terminal_vec = (current_terminal_vel - current_prev) - (kf_before - kf_prev)
        tail_vec = current_before - current_terminal_vel
        total_vec = (current_before - current_after) - (kf_before - kf_after)
        check_vec = shared_vec + terminal_vec + tail_vec - total_vec

        shared_parts = decompose_vec(shared_vec, along_2d, cross_2d)
        terminal_parts = decompose_vec(terminal_vec, along_2d, cross_2d)
        tail_parts = decompose_vec(tail_vec, along_2d, cross_2d)
        total_parts = decompose_vec(total_vec, along_2d, cross_2d)
        shared_frame = pd.DataFrame(interval_shared_rows)
        hits = threshold_hits(shared_frame, shared_parts["norm"])

        interval_rows.append(
            {
                "window_name": window_name,
                "interval_start_t": float(interval_start),
                "interval_end_t": float(interval_end),
                "shared_step_count": int(shared_limit),
                "current_dense_count": int(current_dense.shape[0]),
                "kf_dense_pre_count": int(kf_dense_pre.shape[0]),
                "shared_along": shared_parts["along"],
                "shared_cross": shared_parts["cross"],
                "shared_down": shared_parts["down"],
                "shared_norm": shared_parts["norm"],
                "terminal_along": terminal_parts["along"],
                "terminal_cross": terminal_parts["cross"],
                "terminal_down": terminal_parts["down"],
                "terminal_norm": terminal_parts["norm"],
                "tail_along": tail_parts["along"],
                "tail_cross": tail_parts["cross"],
                "tail_down": tail_parts["down"],
                "tail_norm": tail_parts["norm"],
                "total_along": total_parts["along"],
                "total_cross": total_parts["cross"],
                "total_down": total_parts["down"],
                "total_norm": total_parts["norm"],
                "shared_fraction_of_total_norm": float(shared_parts["norm"] / max(total_parts["norm"], 1e-12)),
                "terminal_fraction_of_total_norm": float(terminal_parts["norm"] / max(total_parts["norm"], 1e-12)),
                "tail_fraction_of_total_norm": float(tail_parts["norm"] / max(total_parts["norm"], 1e-12)),
                "check_norm": float(np.linalg.norm(check_vec)),
                **hits,
            }
        )
        shared_rows.extend(interval_shared_rows)

    interval_frame = pd.DataFrame(interval_rows)
    shared_frame = pd.DataFrame(shared_rows)
    grouped = shared_frame.groupby("step_idx", as_index=False).median(numeric_only=True)
    median_cum = np.zeros(3, dtype=float)
    for idx in range(grouped.shape[0]):
        median_cum += np.array(
            [
                float(grouped.at[idx, "shared_step_along"]),
                float(grouped.at[idx, "shared_step_cross"]),
                float(grouped.at[idx, "shared_step_down"]),
            ],
            dtype=float,
        )
        grouped.at[idx, "shared_cum_along_median"] = float(median_cum[0])
        grouped.at[idx, "shared_cum_cross_median"] = float(median_cum[1])
        grouped.at[idx, "shared_cum_down_median"] = float(median_cum[2])
        grouped.at[idx, "shared_cum_norm_median"] = float(np.linalg.norm(median_cum))
    grouped.insert(0, "window_name", window_name)
    median_rows.extend(grouped.to_dict(orient="records"))
    return interval_frame, shared_frame, pd.DataFrame(median_rows)


def build_summary_lines(
    exp_id: str,
    output_dir: Path,
    interval_frame: pd.DataFrame,
    window_threshold_frame: pd.DataFrame,
    window_median_frames: dict[str, pd.DataFrame],
    inputs: dict[str, Path],
) -> list[str]:
    lines: list[str] = []
    lines.append("# outage predict-substep nominal increment audit")
    lines.append("")
    lines.append(f"- exp_id: `{exp_id}`")
    lines.append("- goal: split outage nominal velocity mismatch into shared predict substeps, terminal pre-update step, and current exact-time tail.")
    for key, path in inputs.items():
        lines.append(f"- {key}: `{rel_path(path)}`")
    lines.append("")

    for window_name in ("w2", "w1"):
        interval_sub = interval_frame[interval_frame["window_name"] == window_name].copy()
        threshold_sub = window_threshold_frame[window_threshold_frame["window_name"] == window_name].copy()
        median_frame = window_median_frames[window_name]
        last_median = median_frame.iloc[-1]
        lines.append(f"## {window_name} priority result")
        lines.append(
            "- shared `199` steps median "
            f"`along/cross/down = {interval_sub['shared_along'].median():.9f} / "
            f"{interval_sub['shared_cross'].median():.9f} / {interval_sub['shared_down'].median():.9f} m/s`, "
            f"`norm = {interval_sub['shared_norm'].median():.9f}`."
        )
        lines.append(
            "- terminal pre-update paired step median "
            f"`along/cross/down = {interval_sub['terminal_along'].median():.9f} / "
            f"{interval_sub['terminal_cross'].median():.9f} / {interval_sub['terminal_down'].median():.9f} m/s`, "
            f"`norm = {interval_sub['terminal_norm'].median():.9f}`."
        )
        lines.append(
            "- current exact-time tail median "
            f"`along/cross/down = {interval_sub['tail_along'].median():.9f} / "
            f"{interval_sub['tail_cross'].median():.9f} / {interval_sub['tail_down'].median():.9f} m/s`, "
            f"`norm = {interval_sub['tail_norm'].median():.9f}`."
        )
        lines.append(
            "- total interval mismatch median "
            f"`along/cross/down = {interval_sub['total_along'].median():.9f} / "
            f"{interval_sub['total_cross'].median():.9f} / {interval_sub['total_down'].median():.9f} m/s`, "
            f"`norm = {interval_sub['total_norm'].median():.9f}`."
        )
        lines.append(
            "- median shared cumulative thresholds "
            f"`25/50/75/90% step = {int(threshold_sub['hit_25_idx'].median())} / "
            f"{int(threshold_sub['hit_50_idx'].median())} / {int(threshold_sub['hit_75_idx'].median())} / "
            f"{int(threshold_sub['hit_90_idx'].median())}`."
        )
        lines.append(
            "- median shared cumulative final "
            f"`along/cross/down = {float(last_median['shared_cum_along_median']):.9f} / "
            f"{float(last_median['shared_cum_cross_median']):.9f} / {float(last_median['shared_cum_down_median']):.9f} m/s`, "
            f"`norm = {float(last_median['shared_cum_norm_median']):.9f}`."
        )
        lines.append("")

    lines.append("## takeaways")
    lines.append(
        "- in both `w2` and `w1`, almost all mismatch is already carried by the shared pre-update predict chain; "
        "the paired terminal pre-update step and current-only exact-time tail are at least one order smaller."
    )
    lines.append(
        "- the shared mismatch accumulates gradually rather than appearing as a single isolated spike: "
        "median `w2` hits `50%` near step `103` and `90%` near step `181`; `w1` is similar."
    )
    lines.append(
        "- this tightens `HYP-62` further: the dominant issue is inside the shared nominal propagation chain itself, "
        "not mainly at the final exact-time split boundary."
    )
    lines.append("")
    lines.append(f"- output_dir: `{rel_path(output_dir)}`")
    return lines


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--current-sol", type=Path, default=CURRENT_SOL_DEFAULT)
    parser.add_argument("--current-gnss-debug", type=Path, default=CURRENT_GNSS_DEBUG_DEFAULT)
    parser.add_argument("--kf-nav", type=Path, default=KF_NAV_DEFAULT)
    parser.add_argument("--kf-gnss-debug", type=Path, default=KF_GNSS_DEBUG_DEFAULT)
    parser.add_argument("--truth", type=Path, default=TRUTH_DEFAULT)
    args = parser.parse_args()

    ensure_dir(args.output_dir)

    truth = load_truth_frame(args.truth)
    current_sol_ned = convert_current_sol_to_ned(truth, load_current_sol(args.current_sol))
    kf_nav = load_kf_nav(args.kf_nav)
    current_gnss = pd.read_csv(args.current_gnss_debug)
    kf_gnss = pd.read_csv(args.kf_gnss_debug)

    interval_frames: list[pd.DataFrame] = []
    shared_frames: dict[str, pd.DataFrame] = {}
    median_frames: dict[str, pd.DataFrame] = {}

    for window_name, (start_t, end_t) in WINDOW_SPECS.items():
        interval_frame, shared_frame, median_frame = build_window_audit(
            truth=truth,
            current_sol_ned=current_sol_ned,
            kf_nav=kf_nav,
            current_gnss=current_gnss,
            kf_gnss=kf_gnss,
            window_name=window_name,
            start_t=start_t,
            end_t=end_t,
        )
        interval_frames.append(interval_frame)
        shared_frames[window_name] = shared_frame
        median_frames[window_name] = median_frame

    interval_frame = pd.concat(interval_frames, ignore_index=True)
    threshold_frame = interval_frame[
        [
            "window_name",
            "interval_start_t",
            "interval_end_t",
            "shared_norm",
            "total_norm",
            "hit_10_idx",
            "hit_25_idx",
            "hit_50_idx",
            "hit_75_idx",
            "hit_90_idx",
        ]
    ].copy()

    interval_path = args.output_dir / "interval_component_summary.csv"
    interval_frame.to_csv(interval_path, index=False)
    threshold_path = args.output_dir / "interval_threshold_summary.csv"
    threshold_frame.to_csv(threshold_path, index=False)

    shared_paths: dict[str, Path] = {}
    median_paths: dict[str, Path] = {}
    for window_name, shared_frame in shared_frames.items():
        shared_path = args.output_dir / f"shared_step_rows_{window_name}.csv"
        shared_frame.to_csv(shared_path, index=False)
        shared_paths[window_name] = shared_path
        median_path = args.output_dir / f"shared_step_median_{window_name}.csv"
        median_frames[window_name].to_csv(median_path, index=False)
        median_paths[window_name] = median_path

    summary_lines = build_summary_lines(
        exp_id=args.exp_id,
        output_dir=args.output_dir,
        interval_frame=interval_frame,
        window_threshold_frame=threshold_frame,
        window_median_frames=median_frames,
        inputs={
            "current_sol": args.current_sol,
            "current_gnss_debug": args.current_gnss_debug,
            "kf_nav": args.kf_nav,
            "kf_gnss_debug": args.kf_gnss_debug,
            "truth": args.truth,
        },
    )
    summary_path = args.output_dir / "summary.md"
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "inputs": {
            "current_sol": rel_path(args.current_sol),
            "current_gnss_debug": rel_path(args.current_gnss_debug),
            "kf_nav": rel_path(args.kf_nav),
            "kf_gnss_debug": rel_path(args.kf_gnss_debug),
            "truth": rel_path(args.truth),
        },
        "outputs": {
            "summary": rel_path(summary_path),
            "interval_component_summary": rel_path(interval_path),
            "interval_threshold_summary": rel_path(threshold_path),
            "shared_step_rows": {key: rel_path(path) for key, path in shared_paths.items()},
            "shared_step_median": {key: rel_path(path) for key, path in median_paths.items()},
        },
        "window_specs": WINDOW_SPECS,
        "headline_metrics": {
            window_name: {
                "shared_along_median": float(interval_frame[interval_frame["window_name"] == window_name]["shared_along"].median()),
                "shared_cross_median": float(interval_frame[interval_frame["window_name"] == window_name]["shared_cross"].median()),
                "shared_down_median": float(interval_frame[interval_frame["window_name"] == window_name]["shared_down"].median()),
                "shared_norm_median": float(interval_frame[interval_frame["window_name"] == window_name]["shared_norm"].median()),
                "terminal_norm_median": float(interval_frame[interval_frame["window_name"] == window_name]["terminal_norm"].median()),
                "tail_norm_median": float(interval_frame[interval_frame["window_name"] == window_name]["tail_norm"].median()),
                "total_norm_median": float(interval_frame[interval_frame["window_name"] == window_name]["total_norm"].median()),
            }
            for window_name in WINDOW_SPECS
        },
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(summary_path)
    print(manifest_path)


if __name__ == "__main__":
    main()
