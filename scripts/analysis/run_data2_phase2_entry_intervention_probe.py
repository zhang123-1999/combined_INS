from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml  # noqa: E402
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import (  # noqa: E402
    build_case_config as build_baseline_case_config,
)
from scripts.analysis.run_data2_phase2_bridge_probe import (  # noqa: E402
    BASE_CONFIG_DEFAULT,
    CONVERGED_NOISE_SCALE_DEFAULT,
    PHASE1_END_OFFSET_DEFAULT,
    SOLVER_DEFAULT,
    build_case_artifacts,
    mtime_text,
)
from scripts.analysis.run_data2_phase2_early_window_probe import (  # noqa: E402
    PROBE_END_OFFSET_DEFAULT,
    build_extra_artifacts,
)
from scripts.analysis.run_data2_staged_estimation import phase_absolute_times, scaled_noise_override  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import reset_directory, run_command  # noqa: E402
from scripts.analysis.run_nhc_state_convergence_research import parse_diag_times  # noqa: E402


EXP_ID_DEFAULT = "EXP-20260324-data2-phase2-entry-intervention-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_phase2_entry_intervention_r1_20260324")
PARTIAL_FREEZE_DURATION_DEFAULT = 30.0
RELAXED_ODO_GATE_PROB_DEFAULT = 0.999999999999


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    init_odo_scale: float | None = None
    partial_freeze_duration: float = 0.0
    partial_freeze_ablation: tuple[tuple[str, bool], ...] = ()
    phase2_enable_nis_gating: bool | None = None
    phase2_odo_nis_gate_prob: float | None = None
    phase2_nhc_nis_gate_prob: float | None = None


CASE_SPECS: tuple[CaseSpec, ...] = (
    CaseSpec(
        case_id="staged_release_baseline",
        label="baseline release",
        color="#4c78a8",
    ),
    CaseSpec(
        case_id="value_reseed_odo_scale_1p0",
        label="value reseed odo_scale=1.0",
        color="#f58518",
        init_odo_scale=1.0,
    ),
    CaseSpec(
        case_id="partial_freeze_accel_bias_scale_30s",
        label="partial freeze accel_bias+accel_scale 30s",
        color="#54a24b",
        partial_freeze_duration=PARTIAL_FREEZE_DURATION_DEFAULT,
        partial_freeze_ablation=(
            ("disable_accel_bias", True),
            ("disable_accel_scale", True),
        ),
    ),
    CaseSpec(
        case_id="relaxed_odo_gate_prob",
        label="relaxed phase2 odo gate prob",
        color="#e45756",
        phase2_odo_nis_gate_prob=RELAXED_ODO_GATE_PROB_DEFAULT,
    ),
)


def build_phase2_constraints(spec: CaseSpec) -> dict[str, Any]:
    constraints: dict[str, Any] = {}
    if spec.phase2_enable_nis_gating is not None:
        constraints["enable_nis_gating"] = bool(spec.phase2_enable_nis_gating)
    if spec.phase2_odo_nis_gate_prob is not None:
        constraints["odo_nis_gate_prob"] = float(spec.phase2_odo_nis_gate_prob)
    if spec.phase2_nhc_nis_gate_prob is not None:
        constraints["nhc_nis_gate_prob"] = float(spec.phase2_nhc_nis_gate_prob)
    return constraints


def build_runtime_phases(
    cfg: dict[str, Any],
    phase1_end_time: float,
    probe_end_time: float,
    converged_noise_scale: float,
    spec: CaseSpec,
) -> list[dict[str, Any]]:
    base_noise = cfg["fusion"]["noise"]
    converged_noise = scaled_noise_override(base_noise, converged_noise_scale)
    phase2_constraints = build_phase2_constraints(spec)
    phases: list[dict[str, Any]] = [
        {
            "name": "phase1_ins_gnss_freeze_odo_nhc_states",
            "start_time": float(cfg["fusion"]["starttime"]),
            "end_time": float(phase1_end_time),
            "ablation": {
                "disable_odo_scale": True,
                "disable_mounting": True,
                "disable_odo_lever_arm": True,
            },
            "constraints": {"enable_odo": False, "enable_nhc": False},
        }
    ]
    if spec.partial_freeze_duration > 0.0:
        freeze_end = min(probe_end_time, phase1_end_time + spec.partial_freeze_duration)
        freeze_phase: dict[str, Any] = {
            "name": "phase2_partial_freeze",
            "start_time": float(phase1_end_time),
            "end_time": float(freeze_end),
            "noise": converged_noise,
        }
        if spec.partial_freeze_ablation:
            freeze_phase["ablation"] = {key: value for key, value in spec.partial_freeze_ablation}
        if phase2_constraints:
            freeze_phase["constraints"] = phase2_constraints
        phases.append(freeze_phase)
        if freeze_end < probe_end_time:
            release_phase: dict[str, Any] = {
                "name": "phase2_joint_calibration_reduced_converged_noise",
                "start_time": float(freeze_end),
                "end_time": float(probe_end_time),
                "noise": converged_noise,
            }
            if phase2_constraints:
                release_phase["constraints"] = phase2_constraints
            phases.append(release_phase)
    else:
        phase2: dict[str, Any] = {
            "name": "phase2_joint_calibration_reduced_converged_noise",
            "start_time": float(phase1_end_time),
            "end_time": float(probe_end_time),
            "noise": converged_noise,
        }
        if phase2_constraints:
            phase2["constraints"] = phase2_constraints
        phases.append(phase2)
    return phases


def build_case_config(
    base_cfg: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    phase1_end_time: float,
    probe_end_time: float,
    converged_noise_scale: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    baseline_spec = type(
        "BaselineSpec",
        (),
        {
            "case_id": spec.case_id,
            "label": spec.label,
            "color": spec.color,
            "filter_mode": "ESKF",
            "enable_fej": False,
            "enable_runtime_anchor": False,
        },
    )()
    cfg, overrides = build_baseline_case_config(base_cfg, case_dir, baseline_spec)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    constraints = fusion.setdefault("constraints", {})
    artifacts = build_case_artifacts(case_dir, spec.case_id)
    extra = build_extra_artifacts(case_dir, spec.case_id)

    if spec.init_odo_scale is not None:
        init_cfg["odo_scale"] = float(spec.init_odo_scale)

    fusion["finaltime"] = float(probe_end_time)
    fusion["output_path"] = rel_from_root(artifacts["sol"], REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(artifacts["state_series"], REPO_ROOT)
    fusion["gnss_schedule"] = {"enabled": False}
    fusion["runtime_phases"] = build_runtime_phases(
        cfg=cfg,
        phase1_end_time=phase1_end_time,
        probe_end_time=probe_end_time,
        converged_noise_scale=converged_noise_scale,
        spec=spec,
    )
    constraints["enable_diagnostics"] = True
    constraints["enable_consistency_log"] = True
    constraints["enable_mechanism_log"] = True
    constraints["mechanism_log_post_gnss_only"] = False
    constraints["mechanism_log_stride"] = 1
    fusion["first_update_debug_output_path"] = rel_from_root(extra["first_update"], REPO_ROOT)
    fusion["gnss_update_debug_output_path"] = rel_from_root(extra["gnss_updates"], REPO_ROOT)
    fusion.setdefault("ablation", {})["disable_mounting_roll"] = False
    fusion["ablation"]["disable_gnss_lever_z"] = False

    overrides["fusion.finaltime"] = float(probe_end_time)
    overrides["fusion.output_path"] = rel_from_root(artifacts["sol"], REPO_ROOT)
    overrides["fusion.state_series_output_path"] = rel_from_root(artifacts["state_series"], REPO_ROOT)
    overrides["fusion.gnss_schedule.enabled"] = False
    overrides["fusion.runtime_phases"] = fusion["runtime_phases"]
    overrides["fusion.constraints.enable_diagnostics"] = True
    overrides["fusion.constraints.enable_consistency_log"] = True
    overrides["fusion.constraints.enable_mechanism_log"] = True
    overrides["fusion.constraints.mechanism_log_post_gnss_only"] = False
    overrides["fusion.constraints.mechanism_log_stride"] = 1
    overrides["fusion.first_update_debug_output_path"] = rel_from_root(extra["first_update"], REPO_ROOT)
    overrides["fusion.gnss_update_debug_output_path"] = rel_from_root(extra["gnss_updates"], REPO_ROOT)
    overrides["fusion.ablation.disable_mounting_roll"] = False
    overrides["fusion.ablation.disable_gnss_lever_z"] = False
    if spec.init_odo_scale is not None:
        overrides["fusion.init.odo_scale"] = float(spec.init_odo_scale)
    return cfg, overrides


def run_case(case_dir: Path, cfg_path: Path, exe_path: Path, spec: CaseSpec) -> dict[str, Any]:
    artifacts = build_case_artifacts(case_dir, spec.case_id)
    extra = build_extra_artifacts(case_dir, spec.case_id)
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    artifacts["stdout"].write_text(stdout_text, encoding="utf-8")
    for key in ("sol", "state_series"):
        if not artifacts[key].exists():
            raise RuntimeError(f"missing solver artifact for {spec.case_id}: {artifacts[key]}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after {spec.case_id}")
    shutil.copy2(root_diag, artifacts["diag"])
    for key, path in extra.items():
        if not path.exists():
            raise RuntimeError(f"missing {key} artifact for {spec.case_id}: {path}")
    metrics = {
        "case_id": spec.case_id,
        "case_label": spec.label,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(artifacts["sol"], REPO_ROOT),
        "state_series_path": rel_from_root(artifacts["state_series"], REPO_ROOT),
        "stdout_path": rel_from_root(artifacts["stdout"], REPO_ROOT),
        "diag_path": rel_from_root(artifacts["diag"], REPO_ROOT),
        "mechanism_path": rel_from_root(extra["mechanism"], REPO_ROOT),
        "gnss_updates_path": rel_from_root(extra["gnss_updates"], REPO_ROOT),
        "first_update_path": rel_from_root(extra["first_update"], REPO_ROOT),
        "config_mtime": mtime_text(cfg_path),
        "sol_mtime": mtime_text(artifacts["sol"]),
        "state_series_mtime": mtime_text(artifacts["state_series"]),
        "stdout_mtime": mtime_text(artifacts["stdout"]),
        "diag_mtime": mtime_text(artifacts["diag"]),
        "mechanism_mtime": mtime_text(extra["mechanism"]),
        "gnss_updates_mtime": mtime_text(extra["gnss_updates"]),
        "first_update_mtime": mtime_text(extra["first_update"]),
    }
    metrics.update(parse_diag_times(stdout_text))
    return metrics


def write_summary(
    output_dir: Path,
    exp_id: str,
    manifest: dict[str, Any],
    metrics_rows: list[dict[str, Any]],
) -> None:
    lines = [
        f"# {exp_id}",
        "",
        "## Purpose",
        "",
        "- Compare phase2 value reseed / partial freeze / relaxed gate interventions on the fresh bug-fixed staged release pipeline.",
        "- Focus on whether the first live accepted ODO update can be moved clearly earlier than baseline `528300.965`.",
        "",
        "## Cases",
        "",
    ]
    for row in metrics_rows:
        case_meta = manifest["cases"][row["case_id"]]
        lines.extend(
            [
                f"### {row['case_id']}",
                "",
                f"- label: `{case_meta['label']}`",
                f"- config: `{row['config_path']}`",
                f"- runtime_intervention: `{json.dumps(case_meta['intervention'], ensure_ascii=False)}`",
                f"- sol: `{row['sol_path']}`",
                f"- state_series: `{row['state_series_path']}`",
                f"- stdout: `{row['stdout_path']}`",
                f"- mechanism: `{row['mechanism_path']}`",
                f"- gnss_updates: `{row['gnss_updates_path']}`",
                f"- first_update: `{row['first_update_path']}`",
                f"- first_divergence_start_t: `{row.get('first_divergence_start_t')}`",
                f"- first_div_gnss_pos_t: `{row.get('first_div_gnss_pos_t')}`",
                "",
            ]
        )
    lines.extend(
        [
            "## Manifest",
            "",
            "```json",
            json.dumps(manifest, ensure_ascii=False, indent=2),
            "```",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run phase2 entry intervention probe cases.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--solver", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--phase1-end-offset", type=float, default=PHASE1_END_OFFSET_DEFAULT)
    parser.add_argument("--probe-end-offset", type=float, default=PROBE_END_OFFSET_DEFAULT)
    parser.add_argument("--converged-noise-scale", type=float, default=CONVERGED_NOISE_SCALE_DEFAULT)
    args = parser.parse_args()

    base_cfg = load_yaml(args.base_config)
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    reset_directory(output_dir)

    start_time = float(base_cfg["fusion"]["starttime"])
    phase1_end_time, _ = phase_absolute_times(
        start_time,
        args.phase1_end_offset,
        args.phase1_end_offset,
    )
    probe_end_time = start_time + args.probe_end_offset

    metrics_rows: list[dict[str, Any]] = []
    manifest_cases: dict[str, Any] = {}

    for spec in CASE_SPECS:
        case_dir = output_dir / "artifacts" / "cases" / spec.case_id
        ensure_dir(case_dir)
        cfg, overrides = build_case_config(
            base_cfg=base_cfg,
            case_dir=case_dir,
            spec=spec,
            phase1_end_time=phase1_end_time,
            probe_end_time=probe_end_time,
            converged_noise_scale=args.converged_noise_scale,
        )
        cfg_path = build_case_artifacts(case_dir, spec.case_id)["config"]
        save_yaml(cfg, cfg_path)
        metrics_row = run_case(case_dir, cfg_path, args.solver, spec)
        metrics_rows.append(metrics_row)

        manifest_cases[spec.case_id] = {
            "label": spec.label,
            "color": spec.color,
            "intervention": {
                "init_odo_scale": spec.init_odo_scale,
                "partial_freeze_duration": spec.partial_freeze_duration,
                "partial_freeze_ablation": {key: value for key, value in spec.partial_freeze_ablation},
                "phase2_enable_nis_gating": spec.phase2_enable_nis_gating,
                "phase2_odo_nis_gate_prob": spec.phase2_odo_nis_gate_prob,
                "phase2_nhc_nis_gate_prob": spec.phase2_nhc_nis_gate_prob,
            },
            "config_path": metrics_row["config_path"],
            "sol_path": metrics_row["sol_path"],
            "state_series_path": metrics_row["state_series_path"],
            "stdout_path": metrics_row["stdout_path"],
            "diag_path": metrics_row["diag_path"],
            "mechanism_path": metrics_row["mechanism_path"],
            "gnss_updates_path": metrics_row["gnss_updates_path"],
            "first_update_path": metrics_row["first_update_path"],
            "overrides": overrides,
        }

    metrics_path = output_dir / "case_metrics.json"
    metrics_path.write_text(json.dumps(metrics_rows, ensure_ascii=False, indent=2), encoding="utf-8")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root((REPO_ROOT / args.base_config).resolve(), REPO_ROOT),
        "solver": rel_from_root((REPO_ROOT / args.solver).resolve(), REPO_ROOT),
        "phase_window": {
            "start_time": start_time,
            "phase1_end_time": phase1_end_time,
            "probe_end_time": probe_end_time,
        },
        "converged_noise_scale": args.converged_noise_scale,
        "case_order": [spec.case_id for spec in CASE_SPECS],
        "cases": manifest_cases,
        "artifacts": {
            "case_metrics_json": rel_from_root(metrics_path, REPO_ROOT),
        },
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_summary(
        output_dir=output_dir,
        exp_id=args.exp_id,
        manifest=manifest,
        metrics_rows=metrics_rows,
    )


if __name__ == "__main__":
    main()
