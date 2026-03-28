from __future__ import annotations

import argparse
import json
import math
import re
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

from scripts.analysis.analyze_data2_phase2_early_window_bursts import (  # noqa: E402
    ENTRY_WINDOW_END_OFFSET,
    ENTRY_WINDOW_START_OFFSET,
    augment_gnss_updates,
    augment_mechanism,
    format_num,
    load_odo_dataframe,
    merge_case_frame,
    summarize_entry_event,
    summarize_window,
)
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import build_truth_reference  # noqa: E402
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    load_imu_dataframe,
    load_pos_dataframe,
)


EXP_ID_DEFAULT = "EXP-20260324-data2-phase2-entry-intervention-analysis-r1"
PROBE_OUTPUT_DEFAULT = Path("output/data2_phase2_entry_intervention_r1_20260324")
ANALYSIS_DIRNAME = "analysis"
ODO_REJECT_PATTERN = re.compile(
    r"\[Consistency\]\s+(?P<tag>ODO|NHC)\s+reject\s+t=(?P<t>[-+0-9eE\.]+)\s+NIS=(?P<nis>[-+0-9eE\.]+)\s+gate=(?P<gate>[-+0-9eE\.]+)"
)


def parse_rejects(stdout_path: Path) -> pd.DataFrame:
    rows: list[dict[str, float | str]] = []
    text = stdout_path.read_text(encoding="utf-8", errors="ignore")
    for match in ODO_REJECT_PATTERN.finditer(text):
        rows.append(
            {
                "tag": match.group("tag"),
                "t": float(match.group("t")),
                "nis": float(match.group("nis")),
                "gate": float(match.group("gate")),
            }
        )
    return pd.DataFrame(rows, columns=["tag", "t", "nis", "gate"])


def plot_entry_state_comparison(
    output_path: Path,
    case_frames: dict[str, pd.DataFrame],
    case_order: list[str],
    case_meta: dict[str, Any],
    phase1_end_time: float,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.8), sharex=True)
    columns = (
        ("ba_x_mgal", "ba_x (mGal)"),
        ("sa_x_ppm", "sa_x (ppm)"),
        ("odo_scale_state", "odo_scale"),
    )
    for case_id in case_order:
        frame = case_frames[case_id]
        subset = frame[
            (frame["timestamp"] >= phase1_end_time + ENTRY_WINDOW_START_OFFSET)
            & (frame["timestamp"] <= phase1_end_time + ENTRY_WINDOW_END_OFFSET)
        ]
        for axis_idx, (column, ylabel) in enumerate(columns):
            axes[axis_idx].plot(
                subset["timestamp"],
                subset[column],
                label=case_meta[case_id]["label"],
                color=case_meta[case_id]["color"],
                lw=1.4,
            )
            axes[axis_idx].set_ylabel(ylabel)
    for ax in axes:
        ax.axvline(phase1_end_time, color="black", ls="--", lw=1.0, alpha=0.8)
        ax.axvline(math.floor(phase1_end_time) + 1.0, color="#666666", ls=":", lw=1.0, alpha=0.8)
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("timestamp (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_entry_proxy_grid(
    output_path: Path,
    case_frames: dict[str, pd.DataFrame],
    case_order: list[str],
    case_meta: dict[str, Any],
    phase1_end_time: float,
) -> None:
    fig, axes = plt.subplots(len(case_order), 2, figsize=(12, 2.2 * len(case_order)), sharex="col")
    axes = np.atleast_2d(axes)
    for row_idx, case_id in enumerate(case_order):
        frame = case_frames[case_id]
        subset = frame[
            (frame["timestamp"] >= phase1_end_time + ENTRY_WINDOW_START_OFFSET)
            & (frame["timestamp"] <= phase1_end_time + ENTRY_WINDOW_END_OFFSET)
        ]
        color = case_meta[case_id]["color"]
        ax_left = axes[row_idx, 0]
        ax_right = axes[row_idx, 1]
        ax_left.plot(subset["timestamp"], subset["odo_speed"], color="#111111", lw=1.1, label="odo_speed")
        ax_left.plot(subset["timestamp"], subset["odo_pred_proxy_mps"], color=color, lw=1.2, label="odo_scale*v_v.x")
        ax_left.plot(
            subset["timestamp"],
            subset["truth_v_v_x_mps"],
            color="#888888",
            lw=1.0,
            ls="--",
            label="truth_v_v.x",
        )
        ax_right.plot(subset["timestamp"], subset["nhc_proxy_norm_mps"], color=color, lw=1.2, label="|v_v.yz|")
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
        ax_left.set_ylabel(case_meta[case_id]["label"])
        if row_idx == 0:
            ax_left.legend(loc="upper right", fontsize=8)
            ax_right.legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xlabel("timestamp (s)")
    axes[-1, 1].set_xlabel("timestamp (s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def plot_odo_reject_nis(
    output_path: Path,
    reject_frames: dict[str, pd.DataFrame],
    entry_rows: pd.DataFrame,
    case_order: list[str],
    case_meta: dict[str, Any],
    phase1_end_time: float,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 5.8))
    for case_id in case_order:
        rejects = reject_frames.get(case_id)
        if rejects is None or rejects.empty:
            continue
        odo_rejects = rejects[rejects["tag"] == "ODO"].copy()
        if odo_rejects.empty:
            continue
        ax.plot(
            odo_rejects["t"],
            odo_rejects["nis"],
            color=case_meta[case_id]["color"],
            lw=1.1,
            label=f"{case_meta[case_id]['label']} reject NIS",
        )
        gate = float(odo_rejects["gate"].iloc[0])
        ax.axhline(gate, color=case_meta[case_id]["color"], ls=":", lw=0.9, alpha=0.5)
        accepted_t = float(entry_rows.loc[entry_rows["case_id"] == case_id, "first_accepted_odo_after_phase2_t"].iloc[0])
        if math.isfinite(accepted_t):
            ax.axvline(accepted_t, color=case_meta[case_id]["color"], ls="--", lw=0.9, alpha=0.8)
    ax.axvline(phase1_end_time, color="black", ls="--", lw=1.0, alpha=0.8)
    ax.set_yscale("log")
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("ODO reject NIS (log)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_summary(
    output_dir: Path,
    exp_id: str,
    entry_rows: pd.DataFrame,
    case_order: list[str],
    case_meta: dict[str, Any],
    plots: dict[str, Path],
    manifest: dict[str, Any],
) -> None:
    baseline_case_id = "staged_release_baseline"
    baseline_odo_t = float(
        entry_rows.loc[entry_rows["case_id"] == baseline_case_id, "first_accepted_odo_after_phase2_t"].iloc[0]
    )
    best_case = entry_rows.sort_values("first_accepted_odo_after_phase2_t").iloc[0]
    lines = [
        f"# {exp_id}",
        "",
        "## Key Findings",
        "",
    ]
    for case_id in case_order:
        row = entry_rows.loc[entry_rows["case_id"] == case_id].iloc[0]
        lines.extend(
            [
                f"### {case_id}",
                "",
                f"- intervention: `{case_meta[case_id]['label']}`",
                f"- first accepted after `phase2` release: "
                f"`NHC={format_num(row['first_accepted_nhc_after_phase2_t'], 3)} s`, "
                f"`ODO={format_num(row['first_accepted_odo_after_phase2_t'], 3)} s`, "
                f"`advance_vs_baseline={format_num(row['odo_accept_advance_vs_baseline_s'], 3)} s`",
                f"- first ODO reject: `t={format_num(row['first_rejected_odo_after_phase2_t'], 3)} s`, "
                f"`NIS={format_num(row['first_rejected_odo_nis'], 3)}`, "
                f"`gate={format_num(row['first_rejected_odo_gate'], 6)}`, "
                f"`ratio={format_num(row['first_rejected_odo_nis_over_gate'], 1)}`",
                f"- min ODO reject NIS in first 1 s: `{format_num(row['min_rejected_odo_nis_first_1s'], 3)}`",
                f"- phase2 first burst: `updates={int(row['entry_burst_updates'])}` "
                f"(`ODO={int(row['entry_burst_odo_updates'])}`, `NHC={int(row['entry_burst_nhc_updates'])}`), "
                f"`mean |odo_speed-odo_scale*v_v.x|={format_num(row['entry_burst_mean_abs_odo_residual_proxy_mps'], 6)} m/s`",
                f"- first post-release GNSS execution at `{format_num(row['entry_gnss_t'], 3)} s`: "
                f"`dx_ba_x={format_num(row['entry_gnss_dx_ba_x_mgal'], 3)} mGal`, "
                f"`dx_bg_z={format_num(row['entry_gnss_dx_bg_z_degh'], 6)} deg/h`, "
                f"`dx_att_z={format_num(row['entry_gnss_dx_att_z_deg'], 6)} deg`",
                "",
            ]
        )

    lines.extend(
        [
            "## Interpretation",
            "",
            f"- Baseline first live accepted `ODO` stays at `{format_num(baseline_odo_t, 3)} s`.",
            f"- Earliest case is `{best_case['case_id']}` at `{format_num(best_case['first_accepted_odo_after_phase2_t'], 3)} s`, "
            f"which advances acceptance by `{format_num(best_case['odo_accept_advance_vs_baseline_s'], 3)} s` relative to baseline.",
            "- If a case still shows `entry_burst ODO=0` while `NHC=200`, then it has not cut the real phase-entry ODO mismatch chain.",
            "- The reject `NIS/gate` ratio directly indicates whether the intervention moved the mismatch itself, or only changed downstream state motion.",
            "",
            "## Artifacts",
            "",
            f"- entry_state_plot: `{rel_from_root(plots['entry_state'], REPO_ROOT)}`",
            f"- entry_proxy_plot: `{rel_from_root(plots['entry_proxy'], REPO_ROOT)}`",
            f"- odo_reject_nis_plot: `{rel_from_root(plots['odo_reject_nis'], REPO_ROOT)}`",
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
    parser = argparse.ArgumentParser(description="Analyze phase2 entry intervention probe.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--probe-output", type=Path, default=PROBE_OUTPUT_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    args = parser.parse_args()

    probe_output = (REPO_ROOT / args.probe_output).resolve()
    manifest = json.loads((probe_output / "manifest.json").read_text(encoding="utf-8"))
    analysis_dir = probe_output / ANALYSIS_DIRNAME
    plots_dir = analysis_dir / "plots"
    ensure_dir(analysis_dir)
    ensure_dir(plots_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())
    imu_df = load_imu_dataframe((REPO_ROOT / base_cfg["fusion"]["imu_path"]).resolve())
    odo_df = load_odo_dataframe((REPO_ROOT / base_cfg["fusion"]["odo_path"]).resolve())

    phase1_end_time = float(manifest["phase_window"]["phase1_end_time"])
    probe_end_time = float(manifest["phase_window"]["probe_end_time"])
    entry_window_end_t = math.floor(phase1_end_time) + 1.0
    case_order = list(manifest["case_order"])
    case_meta = manifest["cases"]

    case_frames: dict[str, pd.DataFrame] = {}
    reject_frames: dict[str, pd.DataFrame] = {}
    entry_rows: list[dict[str, Any]] = []

    for case_id in case_order:
        case_dir = probe_output / "artifacts" / "cases" / case_id
        sol_path = case_dir / f"SOL_{case_id}.txt"
        state_series_path = case_dir / f"state_series_{case_id}.csv"
        mechanism_path = case_dir / f"SOL_{case_id}_mechanism.csv"
        gnss_path = case_dir / f"GNSS_UPDATES_{case_id}.csv"
        stdout_path = case_dir / f"solver_stdout_{case_id}.txt"

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
        entry_window = summarize_window(case_id, mech, frame, phase1_end_time, entry_window_end_t)

        gnss_df = augment_gnss_updates(gnss_path, frame)
        gnss_df = gnss_df[(gnss_df["gnss_t"] >= phase1_end_time) & (gnss_df["gnss_t"] <= probe_end_time)].copy()

        entry_row = summarize_entry_event(case_id, entry_window, gnss_df, phase1_end_time)
        nhc_after = mech[mech["tag"] == "NHC"]
        odo_after = mech[mech["tag"] == "ODO"]
        entry_row["label"] = case_meta[case_id]["label"]
        entry_row["first_accepted_nhc_after_phase2_t"] = float(nhc_after["t_meas"].iloc[0]) if not nhc_after.empty else math.nan
        entry_row["first_accepted_odo_after_phase2_t"] = float(odo_after["t_meas"].iloc[0]) if not odo_after.empty else math.nan

        rejects = parse_rejects(stdout_path)
        rejects = rejects[(rejects["t"] >= phase1_end_time) & (rejects["t"] <= probe_end_time)].copy()
        reject_frames[case_id] = rejects
        odo_rejects = rejects[rejects["tag"] == "ODO"].copy()
        if odo_rejects.empty:
            entry_row["first_rejected_odo_after_phase2_t"] = math.nan
            entry_row["first_rejected_odo_nis"] = math.nan
            entry_row["first_rejected_odo_gate"] = math.nan
            entry_row["first_rejected_odo_nis_over_gate"] = math.nan
            entry_row["min_rejected_odo_nis_first_1s"] = math.nan
        else:
            first_reject = odo_rejects.iloc[0]
            entry_row["first_rejected_odo_after_phase2_t"] = float(first_reject["t"])
            entry_row["first_rejected_odo_nis"] = float(first_reject["nis"])
            entry_row["first_rejected_odo_gate"] = float(first_reject["gate"])
            entry_row["first_rejected_odo_nis_over_gate"] = float(first_reject["nis"] / first_reject["gate"])
            first_second = odo_rejects[
                (odo_rejects["t"] >= phase1_end_time) & (odo_rejects["t"] < math.floor(phase1_end_time) + 1.0)
            ]
            entry_row["min_rejected_odo_nis_first_1s"] = (
                float(first_second["nis"].min()) if not first_second.empty else math.nan
            )
        entry_rows.append(entry_row)

    entry_summary = pd.DataFrame(entry_rows).sort_values("case_id").reset_index(drop=True)
    baseline_odo_t = float(
        entry_summary.loc[
            entry_summary["case_id"] == "staged_release_baseline",
            "first_accepted_odo_after_phase2_t",
        ].iloc[0]
    )
    entry_summary["odo_accept_advance_vs_baseline_s"] = baseline_odo_t - entry_summary["first_accepted_odo_after_phase2_t"]

    entry_path = analysis_dir / "entry_summary.csv"
    reject_path = analysis_dir / "odo_reject_summary.csv"
    entry_summary.to_csv(entry_path, index=False, encoding="utf-8-sig")
    reject_rows = []
    for case_id, rejects in reject_frames.items():
        if rejects.empty:
            continue
        rejects = rejects.copy()
        rejects["case_id"] = case_id
        reject_rows.append(rejects)
    if reject_rows:
        pd.concat(reject_rows, ignore_index=True).to_csv(reject_path, index=False, encoding="utf-8-sig")

    plots = {
        "entry_state": plots_dir / "phase2_entry_state_comparison.png",
        "entry_proxy": plots_dir / "phase2_entry_proxy_grid.png",
        "odo_reject_nis": plots_dir / "phase2_odo_reject_nis.png",
    }
    plot_entry_state_comparison(plots["entry_state"], case_frames, case_order, case_meta, phase1_end_time)
    plot_entry_proxy_grid(plots["entry_proxy"], case_frames, case_order, case_meta, phase1_end_time)
    plot_odo_reject_nis(plots["odo_reject_nis"], reject_frames, entry_summary, case_order, case_meta, phase1_end_time)

    write_summary(
        output_dir=analysis_dir,
        exp_id=args.exp_id,
        entry_rows=entry_summary,
        case_order=case_order,
        case_meta=case_meta,
        plots=plots,
        manifest=manifest,
    )


if __name__ == "__main__":
    main()
