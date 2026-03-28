from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import (
    ensure_dir,
    load_yaml,
    parse_consistency_summary,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_fullwindow_attitude_bias_coupling import (
    build_state_frame,
    compute_case_metrics,
)
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import mtime_text
from scripts.analysis.run_data2_staged_g5_no_imu_scale import (
    KEY_COUPLING_STATES,
    PVA_ERROR_GROUP_SPECS,
    build_mainline_plot_config,
    compute_phase_metrics,
    format_metric,
    plot_state_grid,
    remove_obsolete_mainline_plot_files,
    render_table,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    evaluate_navigation_metrics,
    json_safe,
    run_command,
)
from scripts.analysis.run_nhc_state_convergence_research import (
    build_truth_interp,
    load_pos_dataframe,
    merge_case_outputs,
)


EXP_ID_DEFAULT = "EXP-20260326-data2-baseline-current-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_baseline_current")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
SOLVER_DEFAULT = Path("build/Release/eskf_fusion.exe")
BASELINE_CASE_ID = "data2_baseline_current"
PHASE_NAME_MAP = {
    "phase1_ins_gnss_freeze_mounting_odo_states": "phase1_window",
    "phase2_cov_seed": "phase2_seed_window",
    "phase2_ins_gnss_odo_nhc_freeze_gnss_lever": "phase2_main_window",
    "phase3_periodic_gnss_outage_freeze_calibrated_extrinsics": "phase3_window",
}


@dataclass(frozen=True)
class PlotCase:
    case_id: str
    label: str
    color: str


def normalize_repo_path(path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def invert_enabled_windows(
    start_time: float,
    final_time: float,
    enabled_windows: list[list[float]],
) -> list[list[float]]:
    off_windows: list[list[float]] = []
    cursor = float(start_time)
    for win_start, win_end in enabled_windows:
        start = float(win_start)
        end = float(win_end)
        if start > cursor:
            off_windows.append([cursor, start])
        cursor = max(cursor, end)
    if cursor < final_time:
        off_windows.append([cursor, float(final_time)])
    return off_windows


def extract_phase_windows(cfg: dict[str, Any]) -> dict[str, list[float]]:
    runtime_phases = cfg["fusion"].get("runtime_phases", [])
    windows: dict[str, list[float]] = {}
    for phase in runtime_phases:
        phase_name = str(phase["name"])
        mapped = PHASE_NAME_MAP.get(phase_name)
        if mapped is None:
            continue
        windows[mapped] = [float(phase["start_time"]), float(phase["end_time"])]
    missing = [field for field in PHASE_NAME_MAP.values() if field not in windows]
    if missing:
        raise RuntimeError(f"missing canonical runtime phase windows: {missing}")
    return windows


def extract_run_metadata(cfg: dict[str, Any]) -> dict[str, Any]:
    fusion = cfg["fusion"]
    enabled_windows = [
        [float(window["start_time"]), float(window["end_time"])]
        for window in fusion["gnss_schedule"]["enabled_windows"]
    ]
    metadata = {
        "case_id": BASELINE_CASE_ID,
        **extract_phase_windows(cfg),
        "gnss_on_windows": enabled_windows,
        "gnss_off_windows": invert_enabled_windows(
            float(fusion["starttime"]),
            float(fusion["finaltime"]),
            enabled_windows,
        ),
    }
    return metadata


def build_run_config(base_cfg: dict[str, Any], output_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    output_dir_abs = normalize_repo_path(output_dir)
    fusion = cfg.setdefault("fusion", {})
    fusion["output_path"] = rel_from_root(output_dir_abs / f"SOL_{BASELINE_CASE_ID}.txt", REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(
        output_dir_abs / f"state_series_{BASELINE_CASE_ID}.csv",
        REPO_ROOT,
    )
    return cfg, extract_run_metadata(cfg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the canonical current-best data2 baseline as a single-case baseline artifact set.")
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    args = parser.parse_args()
    args.base_config = normalize_repo_path(args.base_config)
    args.exe = normalize_repo_path(args.exe)
    args.output_dir = normalize_repo_path(args.output_dir)
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases" / BASELINE_CASE_ID
    args.plot_dir = args.output_dir / "plots"
    return args


def run_case(cfg_path: Path, output_dir: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = output_dir / f"SOL_{BASELINE_CASE_ID}.txt"
    state_series_path = output_dir / f"state_series_{BASELINE_CASE_ID}.csv"
    stdout_path = case_dir / f"solver_stdout_{BASELINE_CASE_ID}.txt"
    diag_path = case_dir / f"DIAG_{BASELINE_CASE_ID}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output: {state_series_path}")
    if not root_diag.exists():
        raise RuntimeError("missing DIAG.txt after canonical baseline run")
    shutil.copy2(root_diag, diag_path)

    nav_metrics, segment_rows = evaluate_navigation_metrics(cfg_path, sol_path)
    row: dict[str, Any] = {
        "case_id": BASELINE_CASE_ID,
        "case_label": "data2 baseline current",
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "config_mtime": mtime_text(cfg_path),
        "sol_mtime": mtime_text(sol_path),
        "state_series_mtime": mtime_text(state_series_path),
        "diag_mtime": mtime_text(diag_path),
        "stdout_mtime": mtime_text(stdout_path),
        "segment_rows": segment_rows,
    }
    row.update(nav_metrics)
    for sensor_name, metrics in parse_consistency_summary(stdout_text).items():
        prefix = sensor_name.lower()
        for metric_name, metric_value in metrics.items():
            row[f"{prefix}_{metric_name}"] = float(metric_value)
    return row


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    phase_metrics_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    case_rows = [
        [
            str(row["case_id"]),
            format_metric(row.get("overall_rmse_3d_m_aux")),
            format_metric(row.get("overall_p95_3d_m_aux")),
            format_metric(row.get("overall_final_err_3d_m_aux")),
            format_metric(row.get("yaw_err_max_abs_deg")),
            format_metric(row.get("bg_z_degh_err_max_abs")),
            format_metric(row.get("odo_accept_ratio")),
            format_metric(row.get("nhc_accept_ratio")),
        ]
        for _, row in case_metrics_df.iterrows()
    ]
    phase_rows = [
        [
            str(row["case_id"]),
            str(row["window_name"]),
            format_metric(row["rmse_3d_m"]),
            format_metric(row["p95_3d_m"]),
            format_metric(row["final_err_3d_m"]),
            format_metric(row["yaw_err_abs_max_deg"]),
            format_metric(row["bg_z_err_abs_max_degh"]),
        ]
        for _, row in phase_metrics_df.iterrows()
    ]

    lines = [
        "# data2 canonical baseline summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- case_id: `{BASELINE_CASE_ID}`",
        (
            f"- phase windows: `phase1={manifest['phase_windows']['phase1']}`, "
            f"`phase2_seed={manifest['phase_windows']['phase2_seed']}`, "
            f"`phase2_main={manifest['phase_windows']['phase2_main']}`, "
            f"`phase3={manifest['phase_windows']['phase3']}`"
        ),
        f"- generated_at: `{manifest['generated_at']}`",
        "",
        "## Case Metrics",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "rmse3d_m",
                "p95_3d_m",
                "final_3d_m",
                "yaw_err_max_abs_deg",
                "bg_z_err_max_abs_degh",
                "odo_accept_ratio",
                "nhc_accept_ratio",
            ],
            case_rows,
        )
    )
    lines.extend(["", "## Phase Metrics"])
    lines.extend(
        render_table(
            ["case_id", "window", "rmse3d_m", "p95_3d_m", "final_3d_m", "yaw_abs_max_deg", "bg_z_abs_max_degh"],
            phase_rows,
        )
    )
    if plot_paths:
        lines.extend(["", "## Plot Outputs"])
        for key, path in plot_paths.items():
            lines.append(f"- `{key}`: `{path}`")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")

    ensure_dir(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    cfg, metadata = build_run_config(base_cfg, args.output_dir)

    cfg_path = args.case_root / f"config_{BASELINE_CASE_ID}.yaml"
    save_yaml(cfg, cfg_path)

    truth_reference = build_truth_reference(cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    case_row = run_case(cfg_path, args.output_dir, args.case_root, args.exe)
    sol_path = args.output_dir / f"SOL_{BASELINE_CASE_ID}.txt"
    state_series_path = args.output_dir / f"state_series_{BASELINE_CASE_ID}.csv"
    merged_df = merge_case_outputs(sol_path, state_series_path)

    truth_df = load_pos_dataframe((REPO_ROOT / cfg["fusion"]["pos_path"]).resolve())
    truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
    state_frame = build_state_frame(merged_df, truth_interp_df, truth_reference)

    all_states_path = args.case_root / f"all_states_{BASELINE_CASE_ID}.csv"
    state_frame.to_csv(all_states_path, index=False, encoding="utf-8-sig")

    phase_windows = [
        ("phase1_ins_gnss", *metadata["phase1_window"]),
        ("phase2_cov_seed", *metadata["phase2_seed_window"]),
        ("phase2_ins_gnss_odo_nhc", *metadata["phase2_main_window"]),
        ("phase3_periodic_gnss_outage", *metadata["phase3_window"]),
    ]
    gnss_off_windows = [tuple(window) for window in metadata["gnss_off_windows"]]
    phase_metrics_df = compute_phase_metrics(state_frame, BASELINE_CASE_ID, phase_windows, gnss_off_windows)

    metrics_row = compute_case_metrics(case_row, state_frame)
    metrics_row["config_mtime"] = mtime_text(cfg_path)
    metrics_row["all_states_path"] = rel_from_root(all_states_path, REPO_ROOT)
    metrics_row["all_states_mtime"] = mtime_text(all_states_path)
    case_metrics_df = pd.DataFrame([metrics_row])

    case_metrics_path = args.output_dir / "case_metrics.csv"
    phase_metrics_path = args.output_dir / "phase_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    phase_metrics_df.to_csv(phase_metrics_path, index=False, encoding="utf-8-sig")

    plot_case = PlotCase(case_id=BASELINE_CASE_ID, label="data2 baseline current", color="#1f77b4")
    case_frames = {BASELINE_CASE_ID: state_frame}
    plot_paths: dict[str, str] = {}
    plot_config = build_mainline_plot_config()
    remove_obsolete_mainline_plot_files(args.plot_dir)

    overview_path = args.plot_dir / "all_states_overview.png"
    plot_state_grid(
        case_frames,
        [plot_case],
        plot_config.overview_states,
        overview_path,
        "data2 canonical baseline all-state overview",
        metadata["phase1_window"][1],
        metadata["phase3_window"][0],
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["all_states_overview"] = rel_from_root(overview_path, REPO_ROOT)

    key_path = args.plot_dir / "key_coupling_states.png"
    plot_state_grid(
        case_frames,
        [plot_case],
        KEY_COUPLING_STATES,
        key_path,
        "data2 canonical baseline key coupling states",
        metadata["phase1_window"][1],
        metadata["phase3_window"][0],
        gnss_off_windows,
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["key_coupling_states"] = rel_from_root(key_path, REPO_ROOT)

    for group_spec in plot_config.group_specs:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        selected_group = group_spec
        plot_mode = "state"
        if group_spec.group_id in {"position", "velocity", "attitude"}:
            plot_mode = "error"
            selected_group = next(item for item in PVA_ERROR_GROUP_SPECS if item.group_id == group_spec.group_id)
        plot_state_grid(
            case_frames,
            [plot_case],
            selected_group.states,
            group_path,
            f"data2 canonical baseline {selected_group.title}",
            metadata["phase1_window"][1],
            metadata["phase3_window"][0],
            gnss_off_windows,
            plot_mode=plot_mode,
            truth_keys_to_hide=plot_config.truth_keys_to_hide,
        )
        plot_paths[group_spec.group_id] = rel_from_root(group_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    manifest = {
        "exp_id": args.exp_id,
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "solver": rel_from_root(args.exe, REPO_ROOT),
        "case_id": BASELINE_CASE_ID,
        "case_config": rel_from_root(cfg_path, REPO_ROOT),
        "truth_reference_path": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_path": rel_from_root(case_metrics_path, REPO_ROOT),
        "phase_metrics_path": rel_from_root(phase_metrics_path, REPO_ROOT),
        "phase_windows": {
            "phase1": metadata["phase1_window"],
            "phase2_seed": metadata["phase2_seed_window"],
            "phase2_main": metadata["phase2_main_window"],
            "phase3": metadata["phase3_window"],
        },
        "gnss_on_windows": metadata["gnss_on_windows"],
        "gnss_off_windows": metadata["gnss_off_windows"],
        "plot_paths": plot_paths,
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_mtime": mtime_text(args.exe),
            "case_config_mtime": mtime_text(cfg_path),
            "case_metrics_mtime": mtime_text(case_metrics_path),
            "phase_metrics_mtime": mtime_text(phase_metrics_path),
            "truth_reference_mtime": mtime_text(truth_reference_path),
            "sol_mtime": metrics_row.get("sol_mtime"),
            "state_series_mtime": metrics_row.get("state_series_mtime"),
            "all_states_mtime": metrics_row.get("all_states_mtime"),
        },
    }
    write_summary(summary_path, manifest, case_metrics_df, phase_metrics_df, plot_paths)
    manifest["freshness"]["summary_md_mtime"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
