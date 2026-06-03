from __future__ import annotations

import ast
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
WORK_ROOT = REPO_ROOT.parent
ASSET_DIR = REPO_ROOT / "assets"

CASES = {
    "ESKF": {
        "color": "#d1495b",
        "root": WORK_ROOT / "d2_p10n_va0_t23_t4_60o200_glf_eskf_r1",
        "all_states": WORK_ROOT
        / "d2_p10n_va0_t23_t4_60o200_glf_eskf_r1"
        / "artifacts/cases/data2_ins_gnss_odo_nhc_staged_estimation_eskf"
        / "all_states_data2_ins_gnss_odo_nhc_staged_estimation_eskf.csv",
    },
    "InEKF": {
        "color": "#0077b6",
        "root": WORK_ROOT / "d2_p10n_va0_t23_t4_60o200_glf_inekf_r1",
        "all_states": WORK_ROOT
        / "d2_p10n_va0_t23_t4_60o200_glf_inekf_r1"
        / "artifacts/cases/data2_ins_gnss_odo_nhc_staged_estimation_inekf"
        / "all_states_data2_ins_gnss_odo_nhc_staged_estimation_inekf.csv",
    },
}


def read_case_metrics(root: Path) -> pd.Series:
    df = pd.read_csv(root / "case_metrics.csv", encoding="utf-8-sig")
    return df.iloc[0]


def read_phase_metrics(root: Path) -> pd.DataFrame:
    return pd.read_csv(root / "phase_metrics.csv", encoding="utf-8-sig")


def read_all_states(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, encoding="utf-8-sig")
    df["time_min"] = (df["timestamp"] - df["timestamp"].iloc[0]) / 60.0
    for axis in ("n", "e", "u"):
        df[f"p_{axis}_err_m"] = df[f"p_{axis}_m"] - df[f"truth_p_{axis}_m"]
        df[f"v_{axis}_err_mps"] = df[f"v_{axis}_mps"] - df[f"truth_v_{axis}_mps"]
    df["pos_err_3d_m"] = np.sqrt(
        df["p_n_err_m"] ** 2 + df["p_e_err_m"] ** 2 + df["p_u_err_m"] ** 2
    )
    return df


def load_data() -> tuple[dict[str, pd.Series], dict[str, pd.DataFrame], dict[str, pd.DataFrame]]:
    case_metrics = {}
    phase_metrics = {}
    all_states = {}
    for label, meta in CASES.items():
        case_metrics[label] = read_case_metrics(meta["root"])
        phase_metrics[label] = read_phase_metrics(meta["root"])
        all_states[label] = read_all_states(meta["all_states"])
    return case_metrics, phase_metrics, all_states


def add_outage_spans(ax, segment_rows: str) -> None:
    try:
        rows = ast.literal_eval(segment_rows)
    except (SyntaxError, ValueError):
        return
    if not rows:
        return
    t0 = rows[0]["start_time"] - ((rows[0]["start_time"] - 528076.009368) % 1)
    for row in rows:
        start = (float(row["start_time"]) - 528076.009368) / 60.0
        end = (float(row["end_time"]) - 528076.009368) / 60.0
        ax.axvspan(start, end, color="#e9ecef", alpha=0.45, linewidth=0)


def style_axes(ax, *, xlabel: str | None = None, ylabel: str | None = None) -> None:
    ax.grid(True, color="#d7dce2", linewidth=0.8, alpha=0.7)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)


def savefig(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=180, bbox_inches="tight")
    plt.close()


def plot_position_error(case_metrics, all_states) -> None:
    fig, ax = plt.subplots(figsize=(11.0, 4.8))
    add_outage_spans(ax, case_metrics["ESKF"]["segment_rows"])
    for label, df in all_states.items():
        ax.plot(
            df["time_min"],
            df["pos_err_3d_m"],
            label=label,
            color=CASES[label]["color"],
            linewidth=1.2,
        )
    ax.set_title("3D Position Error under GNSS Outage")
    ax.set_yscale("symlog", linthresh=10)
    ax.legend(frameon=False, loc="upper right")
    style_axes(ax, xlabel="Time from start (min)", ylabel="3D position error (m)")
    savefig(ASSET_DIR / "position_error_outage.png")


def plot_local_trajectory(all_states) -> None:
    fig, ax = plt.subplots(figsize=(7.8, 7.2))
    ref = next(iter(all_states.values()))
    ax.plot(
        ref["truth_p_e_m"],
        ref["truth_p_n_m"],
        color="#2b2d42",
        linewidth=1.6,
        label="Reference",
    )
    for label, df in all_states.items():
        sample = df.iloc[::20]
        ax.plot(
            sample["p_e_m"],
            sample["p_n_m"],
            color=CASES[label]["color"],
            linewidth=1.1,
            alpha=0.9,
            label=label,
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Local Trajectory Comparison")
    ax.legend(frameon=False, loc="best")
    style_axes(ax, xlabel="East (m)", ylabel="North (m)")
    savefig(ASSET_DIR / "trajectory_comparison.png")


def plot_metrics_summary(case_metrics) -> None:
    metrics = [
        ("overall_rmse_3d_m_aux", "Overall RMSE"),
        ("mean_outage_rmse_3d_m", "Mean outage RMSE"),
        ("max_outage_rmse_3d_m", "Worst outage RMSE"),
        ("overall_final_err_3d_m_aux", "Final error"),
    ]
    labels = [name for _, name in metrics]
    x = np.arange(len(metrics))
    width = 0.34
    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    for idx, case in enumerate(("ESKF", "InEKF")):
        values = [float(case_metrics[case][key]) for key, _ in metrics]
        offset = (idx - 0.5) * width
        ax.bar(
            x + offset,
            values,
            width=width,
            label=case,
            color=CASES[case]["color"],
        )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=12, ha="right")
    ax.set_title("Navigation Accuracy Summary")
    ax.set_yscale("log")
    ax.legend(frameon=False)
    style_axes(ax, ylabel="Error (m, log scale)")
    savefig(ASSET_DIR / "metrics_summary.png")


def plot_outage_segments(case_metrics) -> None:
    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    x = None
    for label, metrics in case_metrics.items():
        rows = ast.literal_eval(metrics["segment_rows"])
        seg_ids = [int(row["segment_id"]) for row in rows]
        values = [float(row["rmse_3d_m"]) for row in rows]
        if x is None:
            x = np.array(seg_ids)
        ax.plot(
            seg_ids,
            values,
            marker="o",
            label=label,
            color=CASES[label]["color"],
            linewidth=1.8,
        )
    ax.set_xticks(x)
    ax.set_title("Per-Outage Segment RMSE")
    ax.set_yscale("log")
    ax.legend(frameon=False)
    style_axes(ax, xlabel="GNSS outage segment", ylabel="3D RMSE (m, log scale)")
    savefig(ASSET_DIR / "outage_segment_rmse.png")


def plot_state_convergence(all_states) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 8.0), sharex=True)
    columns = [
        ("yaw_deg", "Yaw (deg)"),
        ("bg_z_degh", "Gyro z bias (deg/h)"),
        ("odo_scale_state", "ODO scale"),
    ]
    for ax, (column, ylabel) in zip(axes, columns):
        for label, df in all_states.items():
            ax.plot(
                df["time_min"],
                df[column],
                label=label,
                color=CASES[label]["color"],
                linewidth=1.2,
            )
        style_axes(ax, ylabel=ylabel)
    axes[0].set_title("State Convergence under Large Initial Error")
    axes[-1].set_xlabel("Time from start (min)")
    axes[0].legend(frameon=False, loc="upper right")
    savefig(ASSET_DIR / "state_convergence.png")


def write_summary(case_metrics) -> None:
    rows = []
    for label, row in case_metrics.items():
        rows.append(
            {
                "method": label,
                "overall_rmse_3d_m": round(float(row["overall_rmse_3d_m_aux"]), 3),
                "mean_outage_rmse_3d_m": round(float(row["mean_outage_rmse_3d_m"]), 3),
                "max_outage_rmse_3d_m": round(float(row["max_outage_rmse_3d_m"]), 3),
                "final_error_3d_m": round(float(row["overall_final_err_3d_m_aux"]), 3),
                "nhc_accept_ratio": round(float(row["nhc_accept_ratio"]), 3),
                "odo_accept_ratio": round(float(row["odo_accept_ratio"]), 3),
            }
        )
    pd.DataFrame(rows).to_csv(ASSET_DIR / "metrics_summary.csv", index=False)


def main() -> None:
    case_metrics, _, all_states = load_data()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    plot_position_error(case_metrics, all_states)
    plot_local_trajectory(all_states)
    plot_metrics_summary(case_metrics)
    plot_outage_segments(case_metrics)
    plot_state_convergence(all_states)
    write_summary(case_metrics)


if __name__ == "__main__":
    main()
