from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.filter_gnss_outage import filter_gnss
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, parse_consistency_summary, rel_from_root
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_case_config as build_state_sanity_case_config,
    build_case_metrics_table,
    build_truth_reference,
    case_release_state_name,
    case_sort_key,
    evaluate_state_case,
    format_metric,
    json_safe,
    plot_heatmap,
    reset_directory,
    run_case,
    write_case_segments,
)


EXPERIMENT_CASES = [
    "truth_anchor_all_non_pva",
    "release_mounting_pitch",
    "release_mounting_yaw",
]
AXIS_TO_TOTAL_COLUMN = {
    "pitch": "total_mounting_pitch_deg",
    "yaw": "total_mounting_yaw_deg",
}


def baseline_mounting_std_deg(base_cfg: dict[str, Any], axis_name: str) -> float:
    init_cfg = base_cfg["fusion"]["init"]
    return float(init_cfg[f"std_mounting_{axis_name}"])


def baseline_mounting_sigma(base_cfg: dict[str, Any], axis_name: str) -> float:
    noise_cfg = base_cfg["fusion"]["noise"]
    return float(noise_cfg.get(f"sigma_mounting_{axis_name}", noise_cfg["sigma_mounting"]))


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
    outage_gnss_path: Path,
) -> dict[str, Any]:
    cfg = build_state_sanity_case_config(
        base_cfg=base_cfg,
        truth_reference=truth_reference,
        case_id=case_id,
        case_dir=case_dir,
        outage_gnss_path=outage_gnss_path,
    )
    fusion = cfg["fusion"]
    ablation_cfg = fusion.setdefault("ablation", {})
    post_ablation_cfg = fusion.setdefault("post_gnss_ablation", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    p0_diag = [float(x) for x in init_cfg["P0_diag"]]

    ablation_cfg["disable_gnss_lever_arm"] = True
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_mounting_roll"] = True
    ablation_cfg["disable_gnss_lever_z"] = False
    post_ablation_cfg["enabled"] = False

    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
    constraints_cfg["enable_mechanism_log"] = False
    init_cfg["runtime_truth_anchor_pva"] = True
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"

    if case_id == "truth_anchor_all_non_pva":
        ablation_cfg["disable_mounting"] = True
        return cfg

    ablation_cfg["disable_mounting"] = False
    target_state = case_release_state_name(case_id)
    if target_state not in {"mounting_pitch", "mounting_yaw"}:
        raise ValueError(f"unsupported mounting-axis case_id: {case_id}")

    target_axis = target_state.split("_", 1)[1]
    frozen_axis = "yaw" if target_axis == "pitch" else "pitch"
    ablation_cfg[f"disable_mounting_{target_axis}"] = False
    ablation_cfg[f"disable_mounting_{frozen_axis}"] = True
    init_cfg[f"std_mounting_{target_axis}"] = baseline_mounting_std_deg(base_cfg, target_axis)
    noise_cfg[f"sigma_mounting_{target_axis}"] = baseline_mounting_sigma(base_cfg, target_axis)

    mount_indices = {"roll": 22, "pitch": 23, "yaw": 24}
    p0_diag[mount_indices[target_axis]] = float(np.deg2rad(init_cfg[f"std_mounting_{target_axis}"]) ** 2)
    init_cfg["P0_diag"] = p0_diag
    return cfg


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
    outage_gnss_path: Path,
) -> Path:
    from scripts.analysis.odo_nhc_update_sweep import save_yaml

    cfg = build_case_config(base_cfg, truth_reference, case_id, case_dir, outage_gnss_path)
    cfg_path = case_dir / f"config_{case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path


def load_mounting_axis_series(case_row: dict[str, Any], axis_name: str) -> pd.DataFrame:
    state_series_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    state_col = f"mounting_{axis_name}_deg"
    total_col = AXIS_TO_TOTAL_COLUMN[axis_name]
    return pd.read_csv(state_series_path, usecols=["timestamp", state_col, total_col])


def enrich_case_with_consistency(case_row: dict[str, Any]) -> dict[str, Any]:
    stdout_path = (REPO_ROOT / case_row["stdout_path"]).resolve()
    stdout_text = stdout_path.read_text(encoding="utf-8", errors="ignore")
    consistency = parse_consistency_summary(stdout_text)
    enriched = dict(case_row)
    enriched["odo_accept_ratio"] = float(consistency.get("ODO", {}).get("accept_ratio", float("nan")))
    enriched["odo_nis_mean"] = float(consistency.get("ODO", {}).get("nis_mean", float("nan")))
    enriched["nhc_accept_ratio"] = float(consistency.get("NHC", {}).get("accept_ratio", float("nan")))
    enriched["nhc_nis_mean"] = float(consistency.get("NHC", {}).get("nis_mean", float("nan")))
    return enriched


def evaluate_mounting_axis_case(
    state_name: str,
    case_row: dict[str, Any],
    control_row: dict[str, Any],
    truth_reference: dict[str, Any],
) -> dict[str, Any]:
    row = evaluate_state_case(state_name, case_row, control_row, truth_reference)
    axis_name = state_name.split("_", 1)[1]
    axis_df = load_mounting_axis_series(case_row, axis_name)
    total_col = AXIS_TO_TOTAL_COLUMN[axis_name]
    total_values = axis_df[total_col].to_numpy(dtype=float)
    row["total_truth_deg"] = float(truth_reference["sources"]["mounting_total_truth"]["value_deg"][axis_name])
    row["total_final_deg"] = float(total_values[-1])
    row["total_range_deg"] = float(total_values.max() - total_values.min())
    row["odo_accept_ratio"] = float(case_row.get("odo_accept_ratio", float("nan")))
    row["nhc_accept_ratio"] = float(case_row.get("nhc_accept_ratio", float("nan")))
    row["odo_nis_mean"] = float(case_row.get("odo_nis_mean", float("nan")))
    row["nhc_nis_mean"] = float(case_row.get("nhc_nis_mean", float("nan")))
    return row


def plot_mounting_axis_comparison(
    axis_name: str,
    control_row: dict[str, Any],
    case_row: dict[str, Any],
    truth_reference: dict[str, Any],
    output_path: Path,
) -> None:
    control_df = load_mounting_axis_series(control_row, axis_name)
    case_df = load_mounting_axis_series(case_row, axis_name)
    state_col = f"mounting_{axis_name}_deg"
    total_col = AXIS_TO_TOTAL_COLUMN[axis_name]
    state_truth = float(truth_reference["states"][f"mounting_{axis_name}"]["reference_value"])
    total_truth = float(truth_reference["sources"]["mounting_total_truth"]["value_deg"][axis_name])

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(control_df["timestamp"], control_df[state_col], linewidth=1.2, label="control")
    axes[0].plot(case_df["timestamp"], case_df[state_col], linewidth=1.2, label=f"release_{axis_name}")
    axes[0].axhline(state_truth, linestyle="--", color="black", linewidth=1.0, label="state truth")
    axes[0].set_ylabel("state offset [deg]")
    axes[0].set_title(f"mounting_{axis_name} state trajectory")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="best")

    axes[1].plot(control_df["timestamp"], control_df[total_col], linewidth=1.2, label="control")
    axes[1].plot(case_df["timestamp"], case_df[total_col], linewidth=1.2, label=f"release_{axis_name}")
    axes[1].axhline(total_truth, linestyle="--", color="black", linewidth=1.0, label="total truth")
    axes[1].set_xlabel("timestamp [s]")
    axes[1].set_ylabel("total angle [deg]")
    axes[1].set_title(f"total mounting_{axis_name} trajectory")
    axes[1].grid(alpha=0.25)
    axes[1].legend(loc="best")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_path: Path,
    case_metrics_df: pd.DataFrame,
    judgement_df: pd.DataFrame,
    truth_reference: dict[str, Any],
    manifest: dict[str, Any],
) -> None:
    control_row = case_metrics_df.loc[case_metrics_df["case_id"] == "truth_anchor_all_non_pva"].iloc[0]
    lines = [
        "# data2 INS/GNSS/ODO/NHC mounting-axis probe",
        "",
        "## 1. Control",
        (
            "- `truth_anchor_all_non_pva`: fixed `odo_scale(21)` + `mounting(22:24)` + "
            "`odo_lever(25:27)` + `gnss_lever(28:30)` control."
        ),
        (
            f"- control `mean_outage_rmse_3d_m={format_metric(float(control_row['mean_outage_rmse_3d_m']))}`, "
            f"`max_outage_final_err_3d_m={format_metric(float(control_row['max_outage_final_err_3d_m']))}`, "
            f"`odo_accept_ratio={format_metric(float(control_row['odo_accept_ratio']))}`, "
            f"`nhc_accept_ratio={format_metric(float(control_row['nhc_accept_ratio']))}`."
        ),
        (
            "- This `r1` keeps `use_truth_pva=true` and `runtime_truth_anchor_pva=true`, so outage "
            "navigation metrics are mainly a sanity guard; the primary evidence is mounting-state stability, "
            "not navigation-error ranking."
        ),
        "",
        "## 2. Axis Results",
    ]
    for state_name in ("mounting_pitch", "mounting_yaw"):
        row_df = judgement_df.loc[judgement_df["state_name"] == state_name]
        if row_df.empty:
            continue
        row = row_df.iloc[0]
        lines.append(
            (
                f"- `{state_name}`: overall=`{row['overall_label']}`, behavior=`{row['behavior_label']}`, "
                f"impact=`{row['impact_label']}`, state final/truth="
                f"`{format_metric(float(row['final_value']))}/{format_metric(float(row['truth_value']))} deg`, "
                f"total final/truth=`{format_metric(float(row['total_final_deg']))}/{format_metric(float(row['total_truth_deg']))} deg`, "
                f"`recovery_ratio={format_metric(float(row['recovery_ratio']))}`, "
                f"`delta_mean_rmse3d={format_metric(float(row['delta_mean_rmse3d']))}`, "
                f"`delta_max_final3d={format_metric(float(row['delta_max_final3d']))}`, "
                f"`odo_accept_ratio={format_metric(float(row['odo_accept_ratio']))}`, "
                f"`nhc_accept_ratio={format_metric(float(row['nhc_accept_ratio']))}`."
            )
        )
    lines.extend(
        [
            "",
            "## 3. Interpretation Guardrails",
            (
                "- This run uses hard per-axis mounting ablation: the non-target pitch/yaw axis in each "
                "release case is masked through `disable_mounting_pitch/yaw`, not approximated by tiny `P0/Q`."
            ),
            (
                f"- total mounting truth: pitch=`{format_metric(float(truth_reference['sources']['mounting_total_truth']['value_deg']['pitch']))}` deg, "
                f"yaw=`{format_metric(float(truth_reference['sources']['mounting_total_truth']['value_deg']['yaw']))}` deg."
            ),
            "",
            "## 4. Artifacts",
            f"- `manifest`: `{manifest['manifest_path']}`",
            f"- `case_metrics`: `{manifest['case_metrics_csv']}`",
            f"- `state_judgement`: `{manifest['state_judgement_csv']}`",
            f"- `plots`: `{manifest['plots_dir']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(
        description="Probe ODO/NHC-related mounting pitch/yaw recoverability under fixed GNSS/ODO lever arms."
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=Path("config_data2_baseline_eskf.yaml"),
        help="Base data2 ESKF config relative to repo root.",
    )
    parser.add_argument(
        "--exe",
        type=Path,
        default=Path("build/Release/eskf_fusion.exe"),
        help="Solver executable relative to repo root.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r1"),
        help="Experiment output directory relative to repo root.",
    )
    parser.add_argument(
        "--exp-id",
        default=f"EXP-{today}-data2-ins-gnss-odo-nhc-mounting-axis-probe-r1",
        help="Experiment identifier recorded in manifest.",
    )
    parser.add_argument("--initial-on", type=float, default=300.0, help="Initial GNSS on duration in seconds.")
    parser.add_argument("--off", type=float, default=100.0, help="GNSS off duration in seconds.")
    parser.add_argument("--on", type=float, default=150.0, help="GNSS on duration in seconds after each outage.")
    parser.add_argument("--cycle-start", choices=("off", "on"), default="off")
    parser.add_argument("--cases", nargs="*", choices=EXPERIMENT_CASES, help="Optional subset of experiment cases.")
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    selected_cases = args.cases or EXPERIMENT_CASES
    if "truth_anchor_all_non_pva" not in selected_cases:
        selected_cases = ["truth_anchor_all_non_pva", *selected_cases]
    args.case_ids = selected_cases
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    outage_gnss_path = args.artifacts_dir / "GNSS_outage_cycle_rtk_300on_100off_150on.txt"
    filter_stats = filter_gnss(
        input_path=(REPO_ROOT / "dataset/data2/rtk.txt").resolve(),
        output_path=outage_gnss_path,
        start_time=float(base_cfg["fusion"]["starttime"]),
        final_time=float(base_cfg["fusion"]["finaltime"]),
        initial_on=args.initial_on,
        on_dur=args.on,
        off_dur=args.off,
        cycle_starts_with=args.cycle_start,
    )
    gnss_stats_path = args.artifacts_dir / "gnss_outage_stats.json"
    gnss_stats_path.write_text(json.dumps(json_safe(filter_stats), indent=2, ensure_ascii=False), encoding="utf-8")

    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    case_segment_paths: dict[str, str] = {}
    release_state_rows: list[dict[str, Any]] = []

    for case_id in args.case_ids:
        case_dir = args.case_root / case_id
        ensure_dir(case_dir)
        cfg_path = write_case_config(base_cfg, truth_reference, case_id, case_dir, outage_gnss_path)
        case_config_paths[case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_row = run_case(case_id=case_id, cfg_path=cfg_path, case_dir=case_dir, exe_path=args.exe)
        case_row = enrich_case_with_consistency(case_row)
        segment_path = write_case_segments(case_dir, case_id, case_row["segment_rows"])
        case_row["outage_segments_path"] = rel_from_root(segment_path, REPO_ROOT)
        case_segment_paths[case_id] = case_row["outage_segments_path"]
        case_rows.append(case_row)

    control_row = next((row for row in case_rows if row["case_id"] == "truth_anchor_all_non_pva"), None)
    if control_row is None:
        raise RuntimeError("control case truth_anchor_all_non_pva was not executed")

    for case_row in sorted(case_rows, key=lambda item: case_sort_key(item["case_id"])):
        if case_row["case_id"] == "truth_anchor_all_non_pva":
            continue
        state_name = case_release_state_name(case_row["case_id"])
        release_state_rows.append(evaluate_mounting_axis_case(state_name, case_row, control_row, truth_reference))
        axis_name = state_name.split("_", 1)[1]
        plot_mounting_axis_comparison(
            axis_name=axis_name,
            control_row=control_row,
            case_row=case_row,
            truth_reference=truth_reference,
            output_path=args.plot_dir / f"mounting_{axis_name}_compare.png",
        )

    case_metrics_df = build_case_metrics_table(case_rows)
    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    judgement_df = pd.DataFrame(release_state_rows)
    if not judgement_df.empty:
        judgement_df = judgement_df.sort_values(by="state_name").reset_index(drop=True)
        plot_heatmap(judgement_df, args.plot_dir / "state_judgement_heatmap.png")
    state_judgement_path = args.output_dir / "state_judgement.csv"
    judgement_df.to_csv(state_judgement_path, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "state_judgement_csv": rel_from_root(state_judgement_path, REPO_ROOT),
        "gnss_outage_stats_json": rel_from_root(gnss_stats_path, REPO_ROOT),
        "requested_case_ids": args.case_ids,
        "case_config_paths": case_config_paths,
        "case_outage_segments_paths": case_segment_paths,
        "outage_schedule": {
            "initial_on_s": args.initial_on,
            "off_s": args.off,
            "on_s": args.on,
            "cycle_start": args.cycle_start,
            "first_off_start_time": filter_stats.get("first_off_start_time"),
            "first_off_end_time": filter_stats.get("first_off_end_time"),
        },
        "assumptions": [
            "GNSS lever and ODO lever are hard-frozen through ablation flags.",
            "odo_scale is hard-frozen through ablation flags.",
            "mounting_roll is hard-frozen through disable_mounting_roll=true.",
            "use_truth_pva=true and runtime_truth_anchor_pva=true are intentionally preserved in this r1, so navigation metrics act as a sanity guard rather than the primary ranking signal.",
            "Non-target mounting pitch/yaw axis is hard-frozen through disable_mounting_pitch/disable_mounting_yaw.",
        ],
    }

    summary_path = args.output_dir / "summary.md"
    manifest["manifest_path"] = rel_from_root(args.output_dir / "manifest.json", REPO_ROOT)
    write_summary(summary_path, case_metrics_df, judgement_df, truth_reference, manifest)
    manifest["summary_md"] = rel_from_root(summary_path, REPO_ROOT)
    manifest["freshness"] = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_judgement_csv": dt.datetime.fromtimestamp(state_judgement_path.stat().st_mtime).isoformat(timespec="seconds"),
        "gnss_outage_stats_json": dt.datetime.fromtimestamp(gnss_stats_path.stat().st_mtime).isoformat(timespec="seconds"),
        "summary_md": dt.datetime.fromtimestamp(summary_path.stat().st_mtime).isoformat(timespec="seconds"),
    }

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
