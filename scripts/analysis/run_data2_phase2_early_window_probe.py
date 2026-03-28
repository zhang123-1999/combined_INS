from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml  # noqa: E402
from scripts.analysis.run_data2_phase2_bridge_probe import (  # noqa: E402
    BASE_CONFIG_DEFAULT,
    CASE_SPECS,
    CONVERGED_NOISE_SCALE_DEFAULT,
    PHASE1_END_OFFSET_DEFAULT,
    REPO_ROOT as BRIDGE_REPO_ROOT,
    SOLVER_DEFAULT,
    build_case_artifacts,
    build_case_config,
    mtime_text,
    run_case,
)
from scripts.analysis.run_data2_staged_estimation import phase_absolute_times  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import reset_directory  # noqa: E402

assert REPO_ROOT == BRIDGE_REPO_ROOT

EXP_ID_DEFAULT = "EXP-20260324-data2-phase2-early-window-probe-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_phase2_early_window_probe_r1_20260324")
PROBE_END_OFFSET_DEFAULT = 264.6


def build_extra_artifacts(case_dir: Path, case_id: str) -> dict[str, Path]:
    return {
        "mechanism": case_dir / f"SOL_{case_id}_mechanism.csv",
        "gnss_updates": case_dir / f"GNSS_UPDATES_{case_id}.csv",
        "first_update": case_dir / f"GNSS_FIRST_UPDATE_{case_id}.csv",
    }


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
        "- Run a fresh narrow `phase2` probe with `mechanism_log` and GNSS-update debug enabled.",
        "- Keep the same three bridge cases as `EXP-20260324-data2-phase2-bridge-probe-r1`, but truncate to the early window only.",
        "",
        "## Cases",
        "",
    ]
    for row in metrics_rows:
        lines.extend(
            [
                f"### {row['case_id']}",
                "",
                f"- config: `{row['config_path']}`",
                f"- sol: `{row['sol_path']}`",
                f"- state_series: `{row['state_series_path']}`",
                f"- stdout: `{row['stdout_path']}`",
                f"- diag: `{row['diag_path']}`",
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
    parser = argparse.ArgumentParser(description="Run a narrow phase2 early-window mechanism probe.")
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
        extra = build_extra_artifacts(case_dir, spec.case_id)
        fusion = cfg.setdefault("fusion", {})
        constraints = fusion.setdefault("constraints", {})
        constraints["enable_mechanism_log"] = True
        constraints["mechanism_log_post_gnss_only"] = False
        constraints["mechanism_log_stride"] = 1
        fusion["first_update_debug_output_path"] = rel_from_root(extra["first_update"], REPO_ROOT)
        fusion["gnss_update_debug_output_path"] = rel_from_root(extra["gnss_updates"], REPO_ROOT)

        overrides["fusion.constraints.enable_mechanism_log"] = True
        overrides["fusion.constraints.mechanism_log_post_gnss_only"] = False
        overrides["fusion.constraints.mechanism_log_stride"] = 1
        overrides["fusion.first_update_debug_output_path"] = rel_from_root(extra["first_update"], REPO_ROOT)
        overrides["fusion.gnss_update_debug_output_path"] = rel_from_root(extra["gnss_updates"], REPO_ROOT)

        cfg_path = build_case_artifacts(case_dir, spec.case_id)["config"]
        save_yaml(cfg, cfg_path)
        metrics_row = run_case(case_dir, cfg_path, args.solver, spec)

        for key, artifact_path in extra.items():
            if not artifact_path.exists():
                raise RuntimeError(f"missing {key} artifact for {spec.case_id}: {artifact_path}")

        metrics_row["mechanism_path"] = rel_from_root(extra["mechanism"], REPO_ROOT)
        metrics_row["gnss_updates_path"] = rel_from_root(extra["gnss_updates"], REPO_ROOT)
        metrics_row["first_update_path"] = rel_from_root(extra["first_update"], REPO_ROOT)
        metrics_row["mechanism_mtime"] = mtime_text(extra["mechanism"])
        metrics_row["gnss_updates_mtime"] = mtime_text(extra["gnss_updates"])
        metrics_row["first_update_mtime"] = mtime_text(extra["first_update"])
        metrics_rows.append(metrics_row)

        manifest_cases[spec.case_id] = {
            "bridge_duration": spec.bridge_duration,
            "bridge_sigma_odo_scale": spec.bridge_sigma_odo_scale,
            "bridge_sigma_mounting": spec.bridge_sigma_mounting,
            "bridge_sigma_lever": spec.bridge_sigma_lever,
            "freeze_gnss_lever_phase2": spec.freeze_gnss_lever_phase2,
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
        "base_config": rel_from_root((REPO_ROOT / args.base_config).resolve(), REPO_ROOT),
        "solver": rel_from_root((REPO_ROOT / args.solver).resolve(), REPO_ROOT),
        "phase_window": {
            "start_time": start_time,
            "phase1_end_time": phase1_end_time,
            "probe_end_time": probe_end_time,
        },
        "converged_noise_scale": args.converged_noise_scale,
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
