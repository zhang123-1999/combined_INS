from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.filter_gnss_outage import filter_gnss
from scripts.analysis.odo_nhc_update_sweep import (
    Scenario,
    SweepRunner,
    ensure_dir,
    hz_label,
    interp_truth,
    load_sol,
    load_truth_ecef,
    parse_consistency_summary,
    parse_split_t,
    plot_outputs,
    rel_from_root,
    save_yaml,
    write_csv,
    write_summary,
    load_yaml,
)


ESKF_FIXED_ODO_NHC_HZ = [100.0, 75.0, 60.0, 50.0, 40.0, 30.0, 20.0, 10.0, 5.0, 2.0, 1.0]
TRUE_FIXED_ODO_NHC_HZ = [50.0, 20.0, 10.0, 5.0, 2.0, 1.333, 1.0, 0.75, 0.667, 0.5, 0.333, 0.25]
STATE_PLOT_FILES = [
    "01_trajectory_altitude.png",
    "02_velocity_NED.png",
    "02b_velocity_vehicle_v.png",
    "03_attitude.png",
    "04_gyro_bias.png",
    "05_accel_bias.png",
    "06_gyro_scale_factor.png",
    "07_accel_scale_factor.png",
    "08_mount_angles.png",
    "09_lever_arm.png",
    "10_gnss_lever_arm.png",
    "11_ecef_position.png",
]


@dataclass(frozen=True)
class CasePlan:
    case_id: str
    dataset_id: str
    family_id: str
    label: str
    base_config_rel: str
    sol_stub: str
    profile_case_id: str | None = None
    disable_gnss_schedule: bool = False
    gnss_path_rel: str | None = None
    odo_interval_s: float | None = None
    nhc_interval_s: float | None = None


PROFILE_CONSTRAINT_KEYS = [
    "sigma_odo",
    "sigma_nhc_y",
    "sigma_nhc_z",
    "odo_min_update_interval",
    "nhc_min_update_interval",
]


def hz_to_interval_s(hz: float) -> float:
    return 1.0 / hz


def load_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def load_noise_profiles(manifest_path: Path | None) -> dict[str, dict[str, Any]]:
    if manifest_path is None:
        return {}
    with manifest_path.open("r", encoding="utf-8") as f:
        manifest = json.load(f)
    profiles: dict[str, dict[str, Any]] = {}
    for row in manifest.get("cases", []):
        case_id = str(row.get("case_id", ""))
        cfg_rel = row.get("best_config_path")
        if not case_id or not cfg_rel:
            continue
        cfg_path = (REPO_ROOT / str(cfg_rel)).resolve()
        if not cfg_path.exists():
            raise FileNotFoundError(f"missing best config for profile {case_id}: {cfg_path}")
        profiles[case_id] = {
            "config_path": cfg_path,
            "config_rel": rel_from_root(cfg_path, REPO_ROOT),
            "config": load_yaml(cfg_path),
        }
    if not profiles:
        raise RuntimeError(f"noise profile manifest has no usable cases: {manifest_path}")
    return profiles


def apply_noise_profile(cfg: dict[str, Any], profile_cfg: dict[str, Any]) -> None:
    fusion = cfg.setdefault("fusion", {})
    profile_fusion = profile_cfg.get("fusion") or {}

    if "noise" in profile_fusion:
        fusion["noise"] = copy.deepcopy(profile_fusion["noise"])

    constraints = fusion.setdefault("constraints", {})
    profile_constraints = profile_fusion.get("constraints") or {}
    for key in PROFILE_CONSTRAINT_KEYS:
        if key in profile_constraints:
            constraints[key] = copy.deepcopy(profile_constraints[key])


def select_best_row(rows: list[dict[str, Any]], scenario_name: str) -> dict[str, Any]:
    df = pd.DataFrame(rows)
    sdf = df[df["scenario"] == scenario_name].copy()
    if sdf.empty:
        raise RuntimeError(f"missing sweep rows for scenario: {scenario_name}")
    candidates = sdf[sdf["group"].isin(["matched", "fixed_odo_nhc_sweep"])].copy()
    candidates = candidates.sort_values(
        by=["rmse_3d_m", "tail70_rmse_3d_m", "final_err_3d_m", "nhc_min_update_interval_s"],
        ascending=[True, True, True, True],
        na_position="last",
    )
    return candidates.iloc[0].to_dict()


def solver_cmd(exe: Path, cfg_path: Path) -> list[str]:
    return [str(exe.resolve()), "--config", str(cfg_path.resolve())]


def plot_cmd(sol_path: Path, cfg_path: Path) -> list[str]:
    return [sys.executable, str((REPO_ROOT / "plot_navresult.py").resolve()), str(sol_path.resolve()), str(cfg_path.resolve())]


def run_process(cmd: list[str], *, cwd: Path) -> str:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(cmd)}\n{merged}")
    return merged


def infer_plot_dir(sol_path: Path) -> Path:
    basename = sol_path.stem
    parts = basename.replace("SOL_", "").replace("_gnss", "").replace("_uwb", "")
    return REPO_ROOT / "output" / f"result_{parts}"


def compute_case_metrics(sol_path: Path, truth_path: Path, merged_log: str) -> dict[str, Any]:
    truth_t, truth_xyz = load_truth_ecef(truth_path)
    sol_t, sol_xyz = load_sol(sol_path)
    truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
    err = sol_xyz - truth_interp
    err3 = np.linalg.norm(err, axis=1)
    nav_duration_s = float(sol_t[-1] - sol_t[0]) if sol_t.size > 1 else 0.0
    tail_start = int(max(0, math.floor(err3.size * 0.3)))
    tail = err3[tail_start:]
    consistency = parse_consistency_summary(merged_log)
    split_t = parse_split_t(merged_log)
    rmse_xyz = np.sqrt(np.mean(err * err, axis=0))

    row: dict[str, Any] = {
        "rmse_x_m": float(rmse_xyz[0]),
        "rmse_y_m": float(rmse_xyz[1]),
        "rmse_z_m": float(rmse_xyz[2]),
        "rmse_3d_m": float(np.sqrt(np.mean(err3 * err3))),
        "p95_3d_m": float(np.percentile(err3, 95)),
        "tail70_rmse_3d_m": float(np.sqrt(np.mean(tail * tail))) if tail.size else float("nan"),
        "final_err_3d_m": float(err3[-1]) if err3.size else float("nan"),
        "samples": int(err3.size),
        "gnss_stop_s": float(split_t) if split_t is not None else float("nan"),
        "nav_duration_s": nav_duration_s,
    }

    for sensor in ("ODO", "NHC"):
        stats = consistency.get(sensor, {})
        seen = float(stats.get("seen", float("nan")))
        accepted = float(stats.get("accepted", float("nan")))
        row[f"{sensor.lower()}_seen"] = seen
        row[f"{sensor.lower()}_accepted"] = accepted
        row[f"{sensor.lower()}_accept_ratio"] = float(stats.get("accept_ratio", float("nan")))
        row[f"{sensor.lower()}_reject_nis"] = float(stats.get("reject_nis", float("nan")))
        row[f"{sensor.lower()}_reject_numeric"] = float(stats.get("reject_numeric", float("nan")))
        row[f"{sensor.lower()}_nis_mean"] = float(stats.get("nis_mean", float("nan")))
        row[f"{sensor.lower()}_nis_max"] = float(stats.get("nis_max", float("nan")))
        row[f"{sensor.lower()}_accepted_hz"] = (
            accepted / nav_duration_s if nav_duration_s > 0.0 and math.isfinite(accepted) else float("nan")
        )
    return row


def build_data4_sweep(args: argparse.Namespace) -> tuple[Path, dict[str, Any], dict[str, Any]]:
    ensure_dir(args.sweep_outdir)
    metrics_path = args.sweep_outdir / "metrics.csv"
    summary_path = args.sweep_outdir / "summary.md"
    manifest_path = args.sweep_outdir / "manifest.json"
    if metrics_path.exists() and summary_path.exists() and manifest_path.exists():
        existing_rows = pd.read_csv(metrics_path, encoding="utf-8-sig").to_dict(orient="records")
        return args.sweep_outdir, select_best_row(existing_rows, "data4_gnss30_eskf"), select_best_row(existing_rows, "data4_gnss30_true_iekf")

    runner = SweepRunner(REPO_ROOT, args.exe, args.sweep_outdir)
    rows: list[dict[str, Any]] = []

    eskf_scenario = Scenario("data4_gnss30_eskf", (REPO_ROOT / "config_data4_gnss30_eskf.yaml").resolve())
    true_scenario = Scenario("data4_gnss30_true_iekf", (REPO_ROOT / "config_data4_gnss30_true_iekf.yaml").resolve())

    rows.append(runner._run_case(eskf_scenario, "matched", 0.0, 0.0))
    rows.extend(
        runner.run_fixed_odo_nhc_sweep(
            eskf_scenario,
            fixed_odo_interval_s=0.0,
            nhc_intervals_s=[hz_to_interval_s(hz) for hz in ESKF_FIXED_ODO_NHC_HZ],
        )
    )
    rows.append(runner._run_case(true_scenario, "matched", 0.0, 0.0))
    rows.extend(
        runner.run_fixed_odo_nhc_sweep(
            true_scenario,
            fixed_odo_interval_s=0.0,
            nhc_intervals_s=[hz_to_interval_s(hz) for hz in TRUE_FIXED_ODO_NHC_HZ],
        )
    )

    rows.sort(key=lambda x: (x["scenario"], x["group"], x["odo_min_update_interval_s"], x["nhc_min_update_interval_s"]))
    write_csv(metrics_path, rows)
    write_summary(summary_path, rows)
    plot_outputs(args.sweep_outdir, rows)
    sweep_manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "scenarios": [
            {"name": eskf_scenario.name, "base_config": rel_from_root(eskf_scenario.base_config, REPO_ROOT)},
            {"name": true_scenario.name, "base_config": rel_from_root(true_scenario.base_config, REPO_ROOT)},
        ],
        "metrics_csv": rel_from_root(metrics_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "plots_dir": rel_from_root(args.sweep_outdir / "plots", REPO_ROOT),
    }
    manifest_path.write_text(json.dumps(sweep_manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    eskf_best = select_best_row(rows, "data4_gnss30_eskf")
    true_best = select_best_row(rows, "data4_gnss30_true_iekf")
    return args.sweep_outdir, eskf_best, true_best


def prepare_outage_file(base_config_rel: str, output_path: Path) -> dict[str, Any]:
    cfg = load_yaml(REPO_ROOT / base_config_rel)
    fusion = cfg.get("fusion") or {}
    input_path = (REPO_ROOT / str(fusion["gnss_path"])).resolve()
    start_time = float(fusion["starttime"])
    final_time = float(fusion["finaltime"])
    stats = filter_gnss(
        input_path=input_path,
        output_path=output_path,
        start_time=start_time,
        final_time=final_time,
        first_on=900.0,
        on_dur=300.0,
        off_dur=120.0,
    )
    stats["input_path"] = rel_from_root(input_path, REPO_ROOT)
    stats["output_path"] = rel_from_root(output_path, REPO_ROOT)
    stats["start_time"] = start_time
    stats["final_time"] = final_time
    return stats


def case_plans(data4_eskf_best: dict[str, Any], data4_true_best: dict[str, Any], cases_outdir: Path) -> tuple[list[CasePlan], list[dict[str, Any]]]:
    data2_outage_gnss = cases_outdir / "GNSS_outage_cycle_data2.txt"
    data4_outage_gnss = cases_outdir / "GNSS_outage_cycle_data4.txt"
    outage_stats = [
        prepare_outage_file("config_data2_gnss30_eskf_nofreeze.yaml", data2_outage_gnss),
        prepare_outage_file("config_data4_gnss30_eskf.yaml", data4_outage_gnss),
    ]

    plans = [
        CasePlan(
            case_id="data2_full_gnss_eskf",
            dataset_id="data2",
            family_id="full_vs_gnss30",
            label="data2 全程GNSS ESKF",
            base_config_rel="config_data2_baseline_eskf.yaml",
            sol_stub="data2_report_full_eskf",
            profile_case_id="data2_gnss30_eskf_best",
            disable_gnss_schedule=True,
        ),
        CasePlan(
            case_id="data2_gnss30_eskf_best",
            dataset_id="data2",
            family_id="full_vs_gnss30",
            label="data2 GNSS30 ESKF 最优",
            base_config_rel="config_data2_gnss30_eskf_nofreeze.yaml",
            sol_stub="data2_report_sparse30_eskf_best",
            profile_case_id="data2_gnss30_eskf_best",
            odo_interval_s=0.0,
            nhc_interval_s=hz_to_interval_s(30.0),
        ),
        CasePlan(
            case_id="data2_gnss30_true_iekf_best",
            dataset_id="data2",
            family_id="gnss30_compare",
            label="data2 GNSS30 true_iekf 最优",
            base_config_rel="config_data2_gnss30_true_iekf.yaml",
            sol_stub="data2_report_sparse30_true_iekf_best",
            profile_case_id="data2_gnss30_true_iekf_best",
            odo_interval_s=0.0,
            nhc_interval_s=hz_to_interval_s(0.75),
        ),
        CasePlan(
            case_id="data2_gnss_outage_eskf_best",
            dataset_id="data2",
            family_id="outage_cycle",
            label="data2 周期开断 ESKF 最优",
            base_config_rel="config_data2_gnss30_eskf_nofreeze.yaml",
            sol_stub="data2_report_outage_cycle_eskf_best",
            profile_case_id="data2_gnss30_eskf_best",
            disable_gnss_schedule=True,
            gnss_path_rel=rel_from_root(data2_outage_gnss, REPO_ROOT),
            odo_interval_s=0.0,
            nhc_interval_s=hz_to_interval_s(30.0),
        ),
        CasePlan(
            case_id="data2_gnss_outage_true_iekf_best",
            dataset_id="data2",
            family_id="outage_cycle",
            label="data2 周期开断 true_iekf 最优",
            base_config_rel="config_data2_gnss30_true_iekf.yaml",
            sol_stub="data2_report_outage_cycle_true_iekf_best",
            profile_case_id="data2_gnss30_true_iekf_best",
            disable_gnss_schedule=True,
            gnss_path_rel=rel_from_root(data2_outage_gnss, REPO_ROOT),
            odo_interval_s=0.0,
            nhc_interval_s=hz_to_interval_s(0.75),
        ),
        CasePlan(
            case_id="data4_full_gnss_eskf",
            dataset_id="data4",
            family_id="full_vs_gnss30",
            label="data4 全程GNSS ESKF",
            base_config_rel="config_data4_gnss30_eskf.yaml",
            sol_stub="data4_report_full_eskf",
            profile_case_id="data4_gnss30_eskf_best",
            disable_gnss_schedule=True,
        ),
        CasePlan(
            case_id="data4_gnss30_eskf_best",
            dataset_id="data4",
            family_id="full_vs_gnss30",
            label="data4 GNSS30 ESKF 最优",
            base_config_rel="config_data4_gnss30_eskf.yaml",
            sol_stub="data4_report_sparse30_eskf_best",
            profile_case_id="data4_gnss30_eskf_best",
            odo_interval_s=0.0,
            nhc_interval_s=float(data4_eskf_best["nhc_min_update_interval_s"]),
        ),
        CasePlan(
            case_id="data4_gnss30_true_iekf_best",
            dataset_id="data4",
            family_id="gnss30_compare",
            label="data4 GNSS30 true_iekf 最优",
            base_config_rel="config_data4_gnss30_true_iekf.yaml",
            sol_stub="data4_report_sparse30_true_iekf_best",
            profile_case_id="data4_gnss30_true_iekf_best",
            odo_interval_s=0.0,
            nhc_interval_s=float(data4_true_best["nhc_min_update_interval_s"]),
        ),
        CasePlan(
            case_id="data4_gnss_outage_eskf_best",
            dataset_id="data4",
            family_id="outage_cycle",
            label="data4 周期开断 ESKF 最优",
            base_config_rel="config_data4_gnss30_eskf.yaml",
            sol_stub="data4_report_outage_cycle_eskf_best",
            profile_case_id="data4_gnss30_eskf_best",
            disable_gnss_schedule=True,
            gnss_path_rel=rel_from_root(data4_outage_gnss, REPO_ROOT),
            odo_interval_s=0.0,
            nhc_interval_s=float(data4_eskf_best["nhc_min_update_interval_s"]),
        ),
        CasePlan(
            case_id="data4_gnss_outage_true_iekf_best",
            dataset_id="data4",
            family_id="outage_cycle",
            label="data4 周期开断 true_iekf 最优",
            base_config_rel="config_data4_gnss30_true_iekf.yaml",
            sol_stub="data4_report_outage_cycle_true_iekf_best",
            profile_case_id="data4_gnss30_true_iekf_best",
            disable_gnss_schedule=True,
            gnss_path_rel=rel_from_root(data4_outage_gnss, REPO_ROOT),
            odo_interval_s=0.0,
            nhc_interval_s=float(data4_true_best["nhc_min_update_interval_s"]),
        ),
    ]
    return plans, outage_stats


def run_case(plan: CasePlan, args: argparse.Namespace) -> dict[str, Any]:
    cfg = copy.deepcopy(load_yaml(REPO_ROOT / plan.base_config_rel))
    fusion = cfg.setdefault("fusion", {})
    constraints = fusion.setdefault("constraints", {})
    profile_meta = None
    if plan.profile_case_id is not None:
        profile_meta = args.noise_profiles.get(plan.profile_case_id)
        if profile_meta is None and args.noise_profile_manifest is not None:
            raise KeyError(f"missing noise profile for {plan.profile_case_id}")
        if profile_meta is not None:
            apply_noise_profile(cfg, profile_meta["config"])
            fusion = cfg.setdefault("fusion", {})
            constraints = fusion.setdefault("constraints", {})
    constraints["enable_consistency_log"] = True
    constraints["enable_diagnostics"] = False
    if plan.odo_interval_s is not None:
        constraints["odo_min_update_interval"] = float(plan.odo_interval_s)
    if plan.nhc_interval_s is not None:
        constraints["nhc_min_update_interval"] = float(plan.nhc_interval_s)

    if plan.disable_gnss_schedule or plan.gnss_path_rel is not None:
        gnss_schedule = fusion.setdefault("gnss_schedule", {})
        gnss_schedule["enabled"] = False
    if plan.gnss_path_rel is not None:
        fusion["gnss_path"] = plan.gnss_path_rel

    sol_path = args.cases_outdir / f"SOL_{plan.sol_stub}.txt"
    cfg_path = args.cases_outdir / f"cfg_{plan.case_id}.yaml"
    log_path = args.cases_outdir / f"{plan.case_id}.log"
    plot_log_path = args.cases_outdir / f"{plan.case_id}.plot.log"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    plot_dir = infer_plot_dir(sol_path)
    reused_existing = False

    if cfg_path.exists():
        try:
            existing_cfg = load_yaml(cfg_path)
        except Exception:
            existing_cfg = None
        if existing_cfg == cfg and log_path.exists() and sol_path.exists() and plot_log_path.exists():
            missing_plots = [name for name in STATE_PLOT_FILES if not (plot_dir / name).exists()]
            if not missing_plots:
                reused_existing = True

    if not reused_existing:
        save_yaml(cfg, cfg_path)
        merged = run_process(solver_cmd(args.exe, cfg_path), cwd=REPO_ROOT)
        log_path.write_text(merged, encoding="utf-8")
        if not sol_path.exists():
            raise FileNotFoundError(f"missing solution file: {sol_path}")
        plot_log = run_process(plot_cmd(sol_path, cfg_path), cwd=REPO_ROOT)
        missing_plots = [name for name in STATE_PLOT_FILES if not (plot_dir / name).exists()]
        if missing_plots:
            raise RuntimeError(f"missing state plots for {plan.case_id}: {missing_plots}")
        plot_log_path.write_text(plot_log, encoding="utf-8")

    merged = log_path.read_text(encoding="utf-8", errors="ignore")
    truth_path = (REPO_ROOT / str(fusion["pos_path"])).resolve()
    metrics = compute_case_metrics(sol_path, truth_path, merged)

    row = {
        "case_id": plan.case_id,
        "dataset_id": plan.dataset_id,
        "family_id": plan.family_id,
        "label": plan.label,
        "method": "true_iekf" if "true_iekf" in plan.case_id else "eskf",
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "base_config_rel": plan.base_config_rel,
        "noise_profile_case_id": plan.profile_case_id or "",
        "noise_profile_config_path": profile_meta["config_rel"] if profile_meta is not None else "",
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "log_path": rel_from_root(log_path, REPO_ROOT),
        "plot_log_path": rel_from_root(plot_log_path, REPO_ROOT),
        "image_dir": rel_from_root(plot_dir, REPO_ROOT),
        "odo_min_update_interval_s": float(constraints.get("odo_min_update_interval", float("nan"))),
        "nhc_min_update_interval_s": float(constraints.get("nhc_min_update_interval", float("nan"))),
        "odo_label": hz_label(float(constraints.get("odo_min_update_interval", 0.0))),
        "nhc_label": hz_label(float(constraints.get("nhc_min_update_interval", 0.0))),
        "gnss_path": str(fusion.get("gnss_path", "")),
        "gnss_schedule_enabled": bool((fusion.get("gnss_schedule") or {}).get("enabled", False)),
        "artifact_mtime_sol": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "artifact_mtime_log": dt.datetime.fromtimestamp(log_path.stat().st_mtime).isoformat(timespec="seconds"),
        "reused_existing": reused_existing,
    }
    row.update(metrics)
    return row


def write_cases_summary(path: Path, rows: list[dict[str, Any]], sweep_best: dict[str, dict[str, Any]], outage_stats: list[dict[str, Any]]) -> None:
    df = pd.DataFrame(rows)
    lines: list[str] = ["# Dataset report canonical cases", ""]
    profile_paths = sorted({str(x) for x in df["noise_profile_config_path"].tolist() if str(x)})
    if profile_paths:
        lines.append("## shared GNSS30 best profiles")
        for profile_path in profile_paths:
            lines.append(f"- `{profile_path}`")
        lines.append("")
    lines.append("## data4 GNSS30 best NHC")
    for key, row in sweep_best.items():
        lines.append(
            f"- {key}: case=`{row['case']}`, nhc=`{row['nhc_label']}`, RMSE3D={row['rmse_3d_m']:.6f} m, "
            f"tail70={row['tail70_rmse_3d_m']:.6f} m, final={row['final_err_3d_m']:.6f} m"
        )
    lines.append("")
    lines.append("## outage files")
    for stat in outage_stats:
        lines.append(
            f"- {stat['output_path']}: kept={stat['rows_kept']}/{stat['rows_raw']} ({stat['ratio_kept']:.3f}), "
            f"window=[{stat['start_time']:.3f}, {stat['final_time']:.3f}]"
        )
    lines.append("")
    for dataset_id in ("data2", "data4"):
        subset = df[df["dataset_id"] == dataset_id].copy()
        lines.append(f"## {dataset_id}")
        lines.append("")
        lines.append("| case_id | family | nhc | rmse_3d_m | p95_3d_m | tail70_rmse_3d_m | final_err_3d_m |")
        lines.append("| --- | --- | --- | ---: | ---: | ---: | ---: |")
        for _, row in subset.iterrows():
            lines.append(
                f"| {row['case_id']} | {row['family_id']} | {row['nhc_label']} | "
                f"{row['rmse_3d_m']:.6f} | {row['p95_3d_m']:.6f} | {row['tail70_rmse_3d_m']:.6f} | {row['final_err_3d_m']:.6f} |"
            )
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(description="Generate fresh cases and assets for the dataset-partitioned HTML report.")
    parser.add_argument(
        "--sweep-outdir",
        type=Path,
        default=Path(f"output/review/EXP-{today}-data4-gnss30-nhc-sweep-r1"),
        help="Output directory for the data4 GNSS30 NHC sweep.",
    )
    parser.add_argument(
        "--cases-outdir",
        type=Path,
        default=Path(f"output/review/EXP-{today}-dataset-report-cases-r1"),
        help="Output directory for the canonical report cases.",
    )
    parser.add_argument(
        "--exe",
        type=Path,
        default=Path("build/Release/eskf_fusion.exe"),
        help="Solver executable relative to repo root.",
    )
    parser.add_argument(
        "--noise-sweep-manifest",
        type=Path,
        default=None,
        help="Optional noise sweep manifest; when provided, full/outage cases inherit the matching GNSS30 best profile.",
    )
    args = parser.parse_args()
    args.sweep_outdir = (REPO_ROOT / args.sweep_outdir).resolve()
    args.cases_outdir = (REPO_ROOT / args.cases_outdir).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    if args.noise_sweep_manifest is not None:
        args.noise_sweep_manifest = (REPO_ROOT / args.noise_sweep_manifest).resolve()
    return args


def main() -> None:
    args = parse_args()
    ensure_dir(args.cases_outdir)
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")
    args.noise_profile_manifest = args.noise_sweep_manifest
    args.noise_profiles = load_noise_profiles(args.noise_sweep_manifest)

    sweep_outdir, data4_eskf_best, data4_true_best = build_data4_sweep(args)
    plans, outage_stats = case_plans(data4_eskf_best, data4_true_best, args.cases_outdir)

    rows = [run_case(plan, args) for plan in plans]
    rows.sort(key=lambda row: (row["dataset_id"], row["family_id"], row["case_id"]))

    metrics_path = args.cases_outdir / "metrics.csv"
    write_csv(metrics_path, rows)
    summary_path = args.cases_outdir / "summary.md"
    write_cases_summary(
        summary_path,
        rows,
        {
            "data4_gnss30_eskf_best": data4_eskf_best,
            "data4_gnss30_true_iekf_best": data4_true_best,
        },
        outage_stats,
    )
    manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "sweep_outdir": rel_from_root(sweep_outdir, REPO_ROOT),
        "metrics_csv": rel_from_root(metrics_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "noise_profile_manifest": rel_from_root(args.noise_sweep_manifest, REPO_ROOT) if args.noise_sweep_manifest else "",
        "data4_best": {
            "eskf": {
                "case": data4_eskf_best["case"],
                "nhc_label": data4_eskf_best["nhc_label"],
                "nhc_min_update_interval_s": float(data4_eskf_best["nhc_min_update_interval_s"]),
                "rmse_3d_m": float(data4_eskf_best["rmse_3d_m"]),
            },
            "true_iekf": {
                "case": data4_true_best["case"],
                "nhc_label": data4_true_best["nhc_label"],
                "nhc_min_update_interval_s": float(data4_true_best["nhc_min_update_interval_s"]),
                "rmse_3d_m": float(data4_true_best["rmse_3d_m"]),
            },
        },
        "outage_files": outage_stats,
        "cases": rows,
    }
    manifest_path = args.cases_outdir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(manifest_path.resolve())


if __name__ == "__main__":
    main()
