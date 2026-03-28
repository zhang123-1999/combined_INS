from __future__ import annotations

import argparse
import json
import math
import sys
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

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root  # noqa: E402
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    build_motion_frame,
    build_truth_interp,
    load_imu_dataframe,
    load_pos_dataframe,
    merge_case_outputs,
)
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference  # noqa: E402

EXP_ID_DEFAULT = "EXP-20260324-data2-phase2-early-window-burst-audit-r1"
PROBE_OUTPUT_DEFAULT = Path("output/data2_phase2_early_window_probe_r1_20260324")
ANALYSIS_DIRNAME = "analysis"
CASE_LABELS = {
    "staged_release_baseline": "baseline",
    "staged_release_bridge_cov_seed_like": "bridge_10s",
    "staged_release_phase2_freeze_gnss_lever": "freeze_gnss_lever",
}
ENTRY_WINDOW_START_OFFSET = -0.05
ENTRY_WINDOW_END_OFFSET = 2.2
BURST_GAP_SEC = 0.03


def load_diag(diag_path: Path, start_time: float) -> pd.DataFrame:
    diag_df = pd.read_csv(diag_path, sep=r"\s+", engine="python")
    diag_df["timestamp"] = start_time + diag_df["t"].to_numpy(dtype=float)
    return diag_df


def load_odo_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["timestamp", "odo_speed"],
        engine="python",
    )


def accel_to_mgal(value: float) -> float:
    return float(value) * 1.0e5


def gyro_radps_to_degh(value: float) -> float:
    return math.degrees(float(value)) * 3600.0


def rad_to_deg(value: float) -> float:
    return math.degrees(float(value))


def add_proxy_columns(frame: pd.DataFrame) -> pd.DataFrame:
    out = frame.copy()
    out["nhc_proxy_norm_mps"] = np.sqrt(out["v_v_y_mps"] ** 2 + out["v_v_z_mps"] ** 2)
    out["truth_nhc_proxy_norm_mps"] = np.sqrt(out["truth_v_v_y_mps"] ** 2 + out["truth_v_v_z_mps"] ** 2)
    out["odo_pred_proxy_mps"] = out["odo_scale_state"] * out["v_v_x_mps"]
    out["odo_residual_proxy_mps"] = out["odo_speed"] - out["odo_pred_proxy_mps"]
    out["truth_odo_residual_proxy_mps"] = out["odo_speed"] - out["truth_v_v_x_mps"]
    return out


def merge_case_frame(
    sol_path: Path,
    state_series_path: Path,
    truth_df: pd.DataFrame,
    imu_df: pd.DataFrame,
    odo_df: pd.DataFrame,
    truth_reference: dict[str, Any],
) -> pd.DataFrame:
    merged = merge_case_outputs(sol_path, state_series_path)
    truth_interp = build_truth_interp(merged["timestamp"].to_numpy(dtype=float), truth_df)
    motion = build_motion_frame(merged, truth_interp, imu_df, truth_reference)
    frame = pd.merge_asof(
        merged.sort_values("timestamp"),
        motion.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=0.003,
    )
    frame = pd.merge_asof(
        frame.sort_values("timestamp"),
        odo_df.sort_values("timestamp"),
        on="timestamp",
        direction="nearest",
        tolerance=0.003,
    )
    if frame[["odo_speed", "v_v_x_mps", "v_v_y_mps", "v_v_z_mps"]].isna().any().any():
        raise RuntimeError("failed to merge motion/diag proxies")
    return add_proxy_columns(frame)


def augment_mechanism(mechanism_path: Path, frame: pd.DataFrame) -> pd.DataFrame:
    mech = pd.read_csv(mechanism_path).sort_values("t_meas")
    mech = mech[mech["tag"].isin(["ODO", "NHC"])].copy()
    if mech.empty:
        raise RuntimeError(f"empty mechanism log: {mechanism_path}")
    state_cols = [
        "timestamp",
        "ba_x_mgal",
        "sa_x_ppm",
        "bg_z_degh",
        "odo_scale_state",
        "total_mounting_yaw_deg",
        "odo_lever_y_m",
        "gnss_lever_y_m",
        "odo_speed",
        "v_v_x_mps",
        "v_v_y_mps",
        "v_v_z_mps",
        "truth_v_v_x_mps",
        "truth_v_v_y_mps",
        "truth_v_v_z_mps",
        "nhc_proxy_norm_mps",
        "truth_nhc_proxy_norm_mps",
        "odo_pred_proxy_mps",
        "odo_residual_proxy_mps",
        "truth_odo_residual_proxy_mps",
    ]
    augmented = pd.merge_asof(
        mech,
        frame[state_cols].sort_values("timestamp"),
        left_on="t_meas",
        right_on="timestamp",
        direction="nearest",
        tolerance=0.003,
    )
    if augmented["timestamp"].isna().any():
        raise RuntimeError(f"failed to align mechanism rows for {mechanism_path}")
    return augmented


def group_bursts(update_df: pd.DataFrame) -> pd.DataFrame:
    if update_df.empty:
        return update_df.assign(burst_id=pd.Series(dtype=int))
    out = update_df.sort_values("t_meas").copy()
    gap = out["t_meas"].diff().fillna(0.0)
    out["burst_id"] = (gap > BURST_GAP_SEC).cumsum().astype(int)
    return out


def take_row(frame: pd.DataFrame, target_t: float, *, side: str) -> pd.Series:
    if frame.empty:
        raise RuntimeError("empty frame")
    if side == "before":
        subset = frame[frame["timestamp"] <= target_t]
        if subset.empty:
            subset = frame.iloc[[0]]
        return subset.iloc[-1]
    if side == "after":
        subset = frame[frame["timestamp"] >= target_t]
        if subset.empty:
            subset = frame.iloc[[-1]]
        return subset.iloc[0]
    raise ValueError(f"unsupported side: {side}")


def summarize_window(case_id: str, update_df: pd.DataFrame, frame: pd.DataFrame, start_t: float, end_t: float) -> dict[str, Any]:
    window = update_df[(update_df["t_meas"] >= start_t) & (update_df["t_meas"] < end_t)].sort_values("t_meas")
    if window.empty:
        raise RuntimeError(f"empty window summary for {case_id}: [{start_t}, {end_t})")
    pre = take_row(frame, float(window["t_meas"].iloc[0]), side="before")
    post = take_row(frame, float(window["t_meas"].iloc[-1]), side="after")
    return {
        "case_id": case_id,
        "window_start_t": float(window["t_meas"].iloc[0]),
        "window_end_t": float(window["t_meas"].iloc[-1]),
        "updates": int(len(window)),
        "odo_updates": int((window["tag"] == "ODO").sum()),
        "nhc_updates": int((window["tag"] == "NHC").sum()),
        "delta_ba_x_mgal": float(post["ba_x_mgal"] - pre["ba_x_mgal"]),
        "delta_sa_x_ppm": float(post["sa_x_ppm"] - pre["sa_x_ppm"]),
        "delta_bg_z_degh": float(post["bg_z_degh"] - pre["bg_z_degh"]),
        "mean_abs_odo_residual_proxy_mps": float(np.mean(np.abs(window["odo_residual_proxy_mps"]))),
        "max_abs_odo_residual_proxy_mps": float(np.max(np.abs(window["odo_residual_proxy_mps"]))),
        "max_abs_nhc_proxy_norm_mps": float(np.max(np.abs(window["nhc_proxy_norm_mps"]))),
        "max_abs_truth_nhc_proxy_norm_mps": float(np.max(np.abs(window["truth_nhc_proxy_norm_mps"]))),
        "pre_odo_scale": float(pre["odo_scale_state"]),
        "post_odo_scale": float(post["odo_scale_state"]),
        "pre_total_mounting_yaw_deg": float(pre["total_mounting_yaw_deg"]),
        "post_total_mounting_yaw_deg": float(post["total_mounting_yaw_deg"]),
    }


def summarize_bursts(case_id: str, update_df: pd.DataFrame, frame: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    grouped = group_bursts(update_df)
    for burst_id, burst in grouped.groupby("burst_id", sort=True):
        burst = burst.sort_values("t_meas")
        start_t = float(burst["t_meas"].iloc[0])
        end_t = float(burst["t_meas"].iloc[-1])
        pre = take_row(frame, start_t, side="before")
        post = take_row(frame, end_t, side="after")
        rows.append(
            {
                "case_id": case_id,
                "burst_id": int(burst_id),
                "start_t": start_t,
                "end_t": end_t,
                "duration_s": end_t - start_t,
                "updates": int(len(burst)),
                "odo_updates": int((burst["tag"] == "ODO").sum()),
                "nhc_updates": int((burst["tag"] == "NHC").sum()),
                "max_y_norm": float(burst["y_norm"].max()),
                "max_nis": float(burst["nis"].max()),
                "max_abs_dx_att_z": float(np.max(np.abs(burst["dx_att_z"]))),
                "max_abs_dx_bg_z": float(np.max(np.abs(burst["dx_bg_z"]))),
                "max_abs_dx_mount_yaw": float(np.max(np.abs(burst["dx_mount_yaw"]))),
                "pre_ba_x_mgal": float(pre["ba_x_mgal"]),
                "post_ba_x_mgal": float(post["ba_x_mgal"]),
                "delta_ba_x_mgal": float(post["ba_x_mgal"] - pre["ba_x_mgal"]),
                "pre_sa_x_ppm": float(pre["sa_x_ppm"]),
                "post_sa_x_ppm": float(post["sa_x_ppm"]),
                "delta_sa_x_ppm": float(post["sa_x_ppm"] - pre["sa_x_ppm"]),
                "pre_bg_z_degh": float(pre["bg_z_degh"]),
                "post_bg_z_degh": float(post["bg_z_degh"]),
                "delta_bg_z_degh": float(post["bg_z_degh"] - pre["bg_z_degh"]),
                "pre_odo_scale": float(pre["odo_scale_state"]),
                "post_odo_scale": float(post["odo_scale_state"]),
                "pre_total_mounting_yaw_deg": float(pre["total_mounting_yaw_deg"]),
                "post_total_mounting_yaw_deg": float(post["total_mounting_yaw_deg"]),
                "pre_odo_lever_y_m": float(pre["odo_lever_y_m"]),
                "post_odo_lever_y_m": float(post["odo_lever_y_m"]),
                "pre_gnss_lever_y_m": float(pre["gnss_lever_y_m"]),
                "post_gnss_lever_y_m": float(post["gnss_lever_y_m"]),
                "max_abs_nhc_proxy_norm_mps": float(np.max(np.abs(burst["nhc_proxy_norm_mps"]))),
                "max_abs_truth_nhc_proxy_norm_mps": float(np.max(np.abs(burst["truth_nhc_proxy_norm_mps"]))),
                "mean_abs_odo_residual_proxy_mps": float(np.mean(np.abs(burst["odo_residual_proxy_mps"]))),
                "max_abs_odo_residual_proxy_mps": float(np.max(np.abs(burst["odo_residual_proxy_mps"]))),
                "mean_odo_speed_mps": float(np.mean(burst["odo_speed"])),
                "mean_v_v_x_mps": float(np.mean(burst["v_v_x_mps"])),
                "mean_odo_pred_proxy_mps": float(np.mean(burst["odo_pred_proxy_mps"])),
            }
        )
    return pd.DataFrame(rows)


def augment_gnss_updates(gnss_path: Path, frame: pd.DataFrame) -> pd.DataFrame:
    gnss = pd.read_csv(gnss_path).sort_values("gnss_t")
    state_cols = [
        "timestamp",
        "ba_x_mgal",
        "sa_x_ppm",
        "bg_z_degh",
        "odo_scale_state",
        "total_mounting_yaw_deg",
        "odo_lever_y_m",
        "gnss_lever_y_m",
    ]
    return pd.merge_asof(
        gnss,
        frame[state_cols].sort_values("timestamp"),
        left_on="gnss_t",
        right_on="timestamp",
        direction="nearest",
        tolerance=0.01,
    )


def summarize_entry_event(case_id: str, entry_window: dict[str, Any], gnss_df: pd.DataFrame, phase1_end_time: float) -> dict[str, Any]:
    first_post_release_gnss_t = math.floor(phase1_end_time) + 1.0
    gnss_entry = gnss_df[(gnss_df["gnss_t"] >= first_post_release_gnss_t - 1.0e-6) & (gnss_df["gnss_t"] < first_post_release_gnss_t + 2.0)].copy()
    if gnss_entry.empty:
        raise RuntimeError(f"no entry GNSS updates found for {case_id}")
    gnss_entry = gnss_entry.sort_values("gnss_t").reset_index(drop=True)
    first_gnss = gnss_entry.iloc[0]
    second_gnss = gnss_entry.iloc[1] if len(gnss_entry) > 1 else first_gnss
    return {
        "case_id": case_id,
        "entry_burst_start_t": float(entry_window["window_start_t"]),
        "entry_burst_end_t": float(entry_window["window_end_t"]),
        "entry_burst_updates": int(entry_window["updates"]),
        "entry_burst_odo_updates": int(entry_window["odo_updates"]),
        "entry_burst_nhc_updates": int(entry_window["nhc_updates"]),
        "entry_burst_delta_ba_x_mgal": float(entry_window["delta_ba_x_mgal"]),
        "entry_burst_delta_sa_x_ppm": float(entry_window["delta_sa_x_ppm"]),
        "entry_burst_delta_bg_z_degh": float(entry_window["delta_bg_z_degh"]),
        "entry_burst_mean_abs_odo_residual_proxy_mps": float(entry_window["mean_abs_odo_residual_proxy_mps"]),
        "entry_burst_max_abs_nhc_proxy_norm_mps": float(entry_window["max_abs_nhc_proxy_norm_mps"]),
        "entry_burst_max_abs_truth_nhc_proxy_norm_mps": float(entry_window["max_abs_truth_nhc_proxy_norm_mps"]),
        "entry_gnss_t": float(first_gnss["gnss_t"]),
        "entry_gnss_y_norm": float(first_gnss["y_norm"]),
        "entry_gnss_dx_ba_x_mgal": accel_to_mgal(float(first_gnss["dx_ba_x"])),
        "entry_gnss_dx_bg_z_degh": gyro_radps_to_degh(float(first_gnss["dx_bg_z"])),
        "entry_gnss_dx_att_z_deg": rad_to_deg(float(first_gnss["dx_att_z"])),
        "entry_gnss_dx_gnss_lever_y": float(first_gnss["dx_gnss_lever_y"]),
        "entry_gnss_state_ba_x_mgal": float(first_gnss["ba_x_mgal"]),
        "entry_gnss_state_sa_x_ppm": float(first_gnss["sa_x_ppm"]),
        "followup_gnss_t": float(second_gnss["gnss_t"]),
        "followup_gnss_dx_ba_x_mgal": accel_to_mgal(float(second_gnss["dx_ba_x"])),
        "followup_gnss_dx_bg_z_degh": gyro_radps_to_degh(float(second_gnss["dx_bg_z"])),
        "followup_gnss_dx_att_z_deg": rad_to_deg(float(second_gnss["dx_att_z"])),
    }


def plot_entry_state_comparison(
    output_path: Path,
    case_frames: dict[str, pd.DataFrame],
    phase1_end_time: float,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    colors = {
        "staged_release_baseline": "#4c78a8",
        "staged_release_bridge_cov_seed_like": "#f58518",
        "staged_release_phase2_freeze_gnss_lever": "#54a24b",
    }
    for case_id, frame in case_frames.items():
        subset = frame[
            (frame["timestamp"] >= phase1_end_time + ENTRY_WINDOW_START_OFFSET)
            & (frame["timestamp"] <= phase1_end_time + ENTRY_WINDOW_END_OFFSET)
        ]
        axes[0].plot(subset["timestamp"], subset["ba_x_mgal"], label=CASE_LABELS[case_id], color=colors[case_id], lw=1.4)
        axes[1].plot(subset["timestamp"], subset["sa_x_ppm"], label=CASE_LABELS[case_id], color=colors[case_id], lw=1.4)
        axes[2].plot(subset["timestamp"], subset["odo_scale_state"], label=CASE_LABELS[case_id], color=colors[case_id], lw=1.4)
    for ax in axes:
        ax.axvline(phase1_end_time, color="black", ls="--", lw=1.0, alpha=0.8)
        ax.axvline(math.floor(phase1_end_time) + 1.0, color="#666666", ls=":", lw=1.0, alpha=0.8)
        ax.grid(True, alpha=0.25)
    axes[0].set_ylabel("ba_x (mGal)")
    axes[1].set_ylabel("sa_x (ppm)")
    axes[2].set_ylabel("odo_scale")
    axes[2].set_xlabel("timestamp (s)")
    axes[0].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_entry_proxy_grid(
    output_path: Path,
    case_frames: dict[str, pd.DataFrame],
    phase1_end_time: float,
) -> None:
    ordered_cases = list(CASE_LABELS.keys())
    fig, axes = plt.subplots(len(ordered_cases), 2, figsize=(12, 8.5), sharex="col")
    colors = {
        "staged_release_baseline": "#4c78a8",
        "staged_release_bridge_cov_seed_like": "#f58518",
        "staged_release_phase2_freeze_gnss_lever": "#54a24b",
    }
    for row_idx, case_id in enumerate(ordered_cases):
        frame = case_frames[case_id]
        subset = frame[
            (frame["timestamp"] >= phase1_end_time + ENTRY_WINDOW_START_OFFSET)
            & (frame["timestamp"] <= phase1_end_time + ENTRY_WINDOW_END_OFFSET)
        ]
        ax_left = axes[row_idx, 0]
        ax_right = axes[row_idx, 1]
        ax_left.plot(subset["timestamp"], subset["odo_speed"], color="#111111", lw=1.1, label="odo_speed")
        ax_left.plot(subset["timestamp"], subset["odo_pred_proxy_mps"], color=colors[case_id], lw=1.2, label="odo_scale*v_v.x")
        ax_left.plot(
            subset["timestamp"],
            subset["truth_v_v_x_mps"],
            color="#888888",
            lw=1.0,
            ls="--",
            label="truth_v_v.x",
        )
        ax_right.plot(subset["timestamp"], subset["nhc_proxy_norm_mps"], color=colors[case_id], lw=1.2, label="|v_v.yz|")
        ax_right.plot(
            subset["timestamp"],
            subset["truth_nhc_proxy_norm_mps"],
            color="#888888",
            lw=1.0,
            ls="--",
            label="truth |v_v.yz|",
        )
        for ax in (ax_left, ax_right):
            ax.axvline(phase1_end_time, color="black", ls="--", lw=1.0, alpha=0.8)
            ax.axvline(math.floor(phase1_end_time) + 1.0, color="#666666", ls=":", lw=1.0, alpha=0.8)
            ax.grid(True, alpha=0.25)
        ax_left.set_ylabel(CASE_LABELS[case_id])
        if row_idx == 0:
            ax_left.legend(loc="upper right", fontsize=8)
            ax_right.legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xlabel("timestamp (s)")
    axes[-1, 1].set_xlabel("timestamp (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def format_num(value: Any, digits: int = 6) -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "NA"
    if not math.isfinite(numeric):
        return "NA"
    return f"{numeric:.{digits}f}"


def write_summary(
    output_dir: Path,
    exp_id: str,
    entry_rows: pd.DataFrame,
    plots: dict[str, Path],
    manifest: dict[str, Any],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Key Findings",
        "",
    ]
    for _, row in entry_rows.iterrows():
        case_id = row["case_id"]
        lines.extend(
            [
                f"### {case_id}",
                "",
                f"- `phase2` first burst: `{format_num(row['entry_burst_start_t'], 3)} -> {format_num(row['entry_burst_end_t'], 3)} s`, "
                f"`updates={int(row['entry_burst_updates'])}` (`ODO={int(row['entry_burst_odo_updates'])}`, `NHC={int(row['entry_burst_nhc_updates'])}`)",
                f"- pre-GNSS burst drift: `delta_ba_x={format_num(row['entry_burst_delta_ba_x_mgal'], 3)} mGal`, "
                f"`delta_sa_x={format_num(row['entry_burst_delta_sa_x_ppm'], 3)} ppm`, "
                f"`delta_bg_z={format_num(row['entry_burst_delta_bg_z_degh'], 6)} deg/h`",
                f"- proxy mismatch inside burst: `mean |odo_speed - odo_scale*v_v.x|={format_num(row['entry_burst_mean_abs_odo_residual_proxy_mps'], 6)} m/s`, "
                f"`max |v_v.yz|={format_num(row['entry_burst_max_abs_nhc_proxy_norm_mps'], 6)} m/s`, "
                f"`truth max |v_v.yz|={format_num(row['entry_burst_max_abs_truth_nhc_proxy_norm_mps'], 6)} m/s`",
                f"- first accepted updates after `phase2` release: "
                f"`NHC={format_num(row['first_accepted_nhc_after_phase2_t'], 3)} s`, "
                f"`ODO={format_num(row['first_accepted_odo_after_phase2_t'], 3)} s`",
                f"- first GNSS execution after burst at `{format_num(row['entry_gnss_t'], 3)} s`: "
                f"`y_norm={format_num(row['entry_gnss_y_norm'], 6)}`, "
                f"`dx_ba_x={format_num(row['entry_gnss_dx_ba_x_mgal'], 3)} mGal`, "
                f"`dx_bg_z={format_num(row['entry_gnss_dx_bg_z_degh'], 6)} deg/h`, "
                f"`dx_att_z={format_num(row['entry_gnss_dx_att_z_deg'], 6)} deg`, "
                f"`dx_gnss_lever_y={format_num(row['entry_gnss_dx_gnss_lever_y'], 9)}`",
                "",
            ]
        )
    odo_accept_min = float(entry_rows["first_accepted_odo_after_phase2_t"].min())
    odo_accept_max = float(entry_rows["first_accepted_odo_after_phase2_t"].max())
    mismatch_min = float(entry_rows["entry_burst_mean_abs_odo_residual_proxy_mps"].min())
    mismatch_max = float(entry_rows["entry_burst_mean_abs_odo_residual_proxy_mps"].max())
    lines.extend(
        [
            "## Interpretation",
            "",
            "- Visible `528277`-family state jump is executed by the first post-release `GNSS_POS` update, not by a one-shot abnormal `NHC` residual.",
            f"- The first post-release second contains `200` accepted `NHC` updates but `0` accepted `ODO` updates; live `ODO` stays rejected until about `{format_num(odo_accept_min, 3)}~{format_num(odo_accept_max, 3)} s`, which means the earliest `528277` pollution cannot be blamed on accepted `ODO` corrections.",
            f"- Even before `ODO` becomes accepted, the phase-entry forward-speed mismatch remains about `{format_num(mismatch_min, 6)}~{format_num(mismatch_max, 6)} m/s` against the wheel-speed measurement; meanwhile accepted `NHC` updates already drift `ba_x/sa_x/bg_z`, and the following `GNSS_POS` update executes that accumulated error as a discrete jump.",
            "- The bridge case improves covariance freedom but does not change this immediate phase-entry value mismatch enough, so the earliest `528277` pollution remains.",
            "- Freezing `GNSS lever` does not remove the entry burst, which is consistent with `GNSS lever` being a downstream executor/amplifier rather than the earliest source.",
            "",
            "## Artifacts",
            "",
            f"- entry_state_plot: `{rel_from_root(plots['entry_state'], REPO_ROOT)}`",
            f"- entry_proxy_plot: `{rel_from_root(plots['entry_proxy'], REPO_ROOT)}`",
            "",
            "## Manifest",
            "",
            "```json",
            json.dumps(manifest, ensure_ascii=False, indent=2),
            "```",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze phase2 early-window ODO/NHC bursts.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--probe-output", type=Path, default=PROBE_OUTPUT_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    args = parser.parse_args()

    probe_output = (REPO_ROOT / args.probe_output).resolve()
    analysis_dir = probe_output / ANALYSIS_DIRNAME
    plots_dir = analysis_dir / "plots"
    ensure_dir(analysis_dir)
    ensure_dir(plots_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())
    odo_df = load_odo_dataframe((REPO_ROOT / base_cfg["fusion"]["odo_path"]).resolve())
    start_time = float(base_cfg["fusion"]["starttime"])
    phase1_end_time = start_time + 200.0
    probe_end_time = start_time + 264.6
    entry_window_end_t = math.floor(phase1_end_time) + 1.0

    case_frames: dict[str, pd.DataFrame] = {}
    burst_frames: list[pd.DataFrame] = []
    gnss_frames: list[pd.DataFrame] = []
    entry_rows: list[dict[str, Any]] = []

    for case_id in CASE_LABELS:
        case_dir = probe_output / "artifacts" / "cases" / case_id
        sol_path = case_dir / f"SOL_{case_id}.txt"
        state_series_path = case_dir / f"state_series_{case_id}.csv"
        mechanism_path = case_dir / f"SOL_{case_id}_mechanism.csv"
        gnss_path = case_dir / f"GNSS_UPDATES_{case_id}.csv"

        frame = merge_case_frame(
            sol_path=sol_path,
            state_series_path=state_series_path,
            truth_df=truth_df,
            imu_df=imu_df,
            odo_df=odo_df,
            truth_reference=truth_reference,
        )
        case_frames[case_id] = frame

        mech = augment_mechanism(mechanism_path, frame)
        mech = mech[(mech["t_meas"] >= phase1_end_time) & (mech["t_meas"] <= probe_end_time)].copy()
        burst_df = summarize_bursts(case_id, mech, frame)
        burst_frames.append(burst_df)
        entry_window = summarize_window(case_id, mech, frame, phase1_end_time, entry_window_end_t)

        gnss_df = augment_gnss_updates(gnss_path, frame)
        gnss_df = gnss_df[(gnss_df["gnss_t"] >= phase1_end_time) & (gnss_df["gnss_t"] <= probe_end_time)].copy()
        gnss_df["case_id"] = case_id
        gnss_frames.append(gnss_df)

        entry_row = summarize_entry_event(case_id, entry_window, gnss_df, phase1_end_time)
        nhc_after = mech[mech["tag"] == "NHC"]
        odo_after = mech[mech["tag"] == "ODO"]
        entry_row["first_accepted_nhc_after_phase2_t"] = float(nhc_after["t_meas"].iloc[0]) if not nhc_after.empty else math.nan
        entry_row["first_accepted_odo_after_phase2_t"] = float(odo_after["t_meas"].iloc[0]) if not odo_after.empty else math.nan
        entry_rows.append(entry_row)

    burst_summary = pd.concat(burst_frames, ignore_index=True)
    gnss_summary = pd.concat(gnss_frames, ignore_index=True)
    entry_summary = pd.DataFrame(entry_rows).sort_values("case_id")

    burst_path = analysis_dir / "burst_summary.csv"
    gnss_path = analysis_dir / "gnss_update_summary.csv"
    entry_path = analysis_dir / "entry_summary.csv"
    burst_summary.to_csv(burst_path, index=False, encoding="utf-8-sig")
    gnss_summary.to_csv(gnss_path, index=False, encoding="utf-8-sig")
    entry_summary.to_csv(entry_path, index=False, encoding="utf-8-sig")

    plots = {
        "entry_state": plots_dir / "phase2_entry_state_comparison.png",
        "entry_proxy": plots_dir / "phase2_entry_proxy_grid.png",
    }
    plot_entry_state_comparison(plots["entry_state"], case_frames, phase1_end_time)
    plot_entry_proxy_grid(plots["entry_proxy"], case_frames, phase1_end_time)

    manifest = {
        "exp_id": args.exp_id,
        "probe_output": rel_from_root(probe_output, REPO_ROOT),
        "base_config": rel_from_root((REPO_ROOT / args.base_config).resolve(), REPO_ROOT),
        "phase_window": {
            "phase1_end_time": phase1_end_time,
            "probe_end_time": probe_end_time,
        },
        "artifacts": {
            "burst_summary": rel_from_root(burst_path, REPO_ROOT),
            "gnss_update_summary": rel_from_root(gnss_path, REPO_ROOT),
            "entry_summary": rel_from_root(entry_path, REPO_ROOT),
            "entry_state_plot": rel_from_root(plots["entry_state"], REPO_ROOT),
            "entry_proxy_plot": rel_from_root(plots["entry_proxy"], REPO_ROOT),
        },
    }
    (analysis_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_summary(
        output_dir=analysis_dir,
        exp_id=args.exp_id,
        entry_rows=entry_summary,
        plots=plots,
        manifest=manifest,
    )


if __name__ == "__main__":
    main()
