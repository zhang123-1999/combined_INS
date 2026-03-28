from __future__ import annotations

import argparse
import datetime as dt
import json
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

from scripts.analysis.odo_nhc_update_sweep import (  # noqa: E402
    ensure_dir,
    load_yaml,
    parse_consistency_summary,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    evaluate_navigation_metrics,
)


BASE_CASE_CONFIG = Path(
    "output/debug_mounting_yaw_fullgnss_sensor_split_20260322/"
    "cases/yaw_only_both_nominal_aligned/config_yaw_only_both_nominal_aligned.yaml"
)
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_intervention_compare_r2_20260322")


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    disable_gyro_bias: bool = False
    debug_odo_disable_bgz_jacobian: bool = False
    debug_odo_disable_bgz_state_update: bool = False
    debug_run_odo_before_nhc: bool = False


CASE_SPECS = [
    CaseSpec("baseline"),
    CaseSpec("freeze_gyro_bias", disable_gyro_bias=True),
    CaseSpec("odo_disable_bgz_jacobian", debug_odo_disable_bgz_jacobian=True),
    CaseSpec("odo_disable_bgz_state_update", debug_odo_disable_bgz_state_update=True),
    CaseSpec("odo_before_nhc", debug_run_odo_before_nhc=True),
    CaseSpec(
        "odo_before_nhc_disable_bgz_state_update",
        debug_odo_disable_bgz_state_update=True,
        debug_run_odo_before_nhc=True,
    ),
]


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


def run_solver(exe_path: Path, cfg_path: Path) -> str:
    proc = subprocess.run(
        [str(exe_path.resolve()), "--config", str(cfg_path.resolve())],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    if proc.returncode != 0:
        raise RuntimeError(
            f"solver failed for {cfg_path.name}: returncode={proc.returncode}\n{merged}"
        )
    return merged


def build_case_config(base_cfg: dict[str, Any], case_dir: Path, spec: CaseSpec) -> dict[str, Any]:
    cfg = json.loads(json.dumps(base_cfg))
    fusion = cfg["fusion"]
    constraints = fusion.setdefault("constraints", {})
    ablation = fusion.setdefault("ablation", {})

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)

    constraints["enable_consistency_log"] = True
    constraints["enable_mechanism_log"] = False
    constraints["debug_odo_disable_bgz_jacobian"] = bool(spec.debug_odo_disable_bgz_jacobian)
    constraints["debug_odo_disable_bgz_state_update"] = bool(
        spec.debug_odo_disable_bgz_state_update
    )
    constraints["debug_run_odo_before_nhc"] = bool(spec.debug_run_odo_before_nhc)

    ablation["disable_gyro_bias"] = bool(spec.disable_gyro_bias)
    return cfg


def summarize_state_series(path: Path) -> dict[str, float]:
    df = pd.read_csv(path)
    yaw = df["total_mounting_yaw_deg"].to_numpy(dtype=float)
    bgz = df["bg_z_degh"].to_numpy(dtype=float)
    tail = yaw[200:] if yaw.size > 200 else yaw
    return {
        "yaw_steady_center_deg": float(np.mean(tail)),
        "yaw_final_total_deg": float(yaw[-1]),
        "yaw_after_200_peak_to_peak_deg": float(np.ptp(tail)),
        "yaw_overall_std_deg": float(np.std(yaw)),
        "bg_z_final_degh": float(bgz[-1]),
        "bg_z_absmax_degh": float(np.max(np.abs(bgz))),
    }


def format_float(value: Any) -> str:
    try:
        return f"{float(value):.6f}"
    except Exception:
        return "NA"


def render_summary(df: pd.DataFrame) -> str:
    cols = [
        "case_id",
        "overall_rmse_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "yaw_steady_center_deg",
        "yaw_final_total_deg",
        "yaw_after_200_peak_to_peak_deg",
        "bg_z_final_degh",
    ]
    lines = [
        "# Mounting Yaw Intervention Compare",
        "",
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for _, row in df.iterrows():
        vals: list[str] = []
        for col in cols:
            value = row[col]
            vals.append(value if isinstance(value, str) else format_float(value))
        lines.append("| " + " | ".join(vals) + " |")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", type=Path, default=BASE_CASE_CONFIG)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    base_cfg = load_yaml(REPO_ROOT / args.base_config)
    outdir = REPO_ROOT / args.output_dir
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    rows: list[dict[str, Any]] = []
    manifest: list[dict[str, Any]] = []

    for spec in CASE_SPECS:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        cfg = build_case_config(base_cfg, case_dir, spec)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        stdout_text = run_solver(args.exe, cfg_path)
        stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
        stdout_path.write_text(stdout_text, encoding="utf-8")

        sol_path = case_dir / f"SOL_{spec.case_id}.txt"
        state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
        if not sol_path.exists() or not state_series_path.exists():
            raise RuntimeError(f"missing solver outputs for {spec.case_id}")

        nav_metrics, _ = evaluate_navigation_metrics(cfg_path, sol_path)
        consistency = parse_consistency_summary(stdout_text)
        state_metrics = summarize_state_series(state_series_path)

        row: dict[str, Any] = {
            "case_id": spec.case_id,
            "config_path": rel_from_root(cfg_path, REPO_ROOT),
            "sol_path": rel_from_root(sol_path, REPO_ROOT),
            "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
            "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
            "sol_mtime": mtime_text(sol_path),
            "state_series_mtime": mtime_text(state_series_path),
            "stdout_mtime": mtime_text(stdout_path),
            "disable_gyro_bias": spec.disable_gyro_bias,
            "debug_odo_disable_bgz_jacobian": spec.debug_odo_disable_bgz_jacobian,
            "debug_odo_disable_bgz_state_update": spec.debug_odo_disable_bgz_state_update,
            "debug_run_odo_before_nhc": spec.debug_run_odo_before_nhc,
        }
        row.update(nav_metrics)
        row.update(state_metrics)
        row["odo_accept_ratio"] = consistency.get("ODO", {}).get("accept_ratio")
        row["odo_nis_mean"] = consistency.get("ODO", {}).get("nis_mean")
        row["nhc_accept_ratio"] = consistency.get("NHC", {}).get("accept_ratio")
        row["nhc_nis_mean"] = consistency.get("NHC", {}).get("nis_mean")
        rows.append(row)
        manifest.append(
            {
                "case_id": spec.case_id,
                "config_path": row["config_path"],
                "sol_path": row["sol_path"],
                "state_series_path": row["state_series_path"],
                "stdout_path": row["stdout_path"],
            }
        )

    df = pd.DataFrame(rows)
    metrics_path = outdir / "metrics.csv"
    df.to_csv(metrics_path, index=False)
    summary_path = outdir / "summary.md"
    summary_path.write_text(render_summary(df), encoding="utf-8")
    manifest_path = outdir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[done] metrics: {metrics_path}")
    print(f"[done] summary: {summary_path}")
    print(f"[done] manifest: {manifest_path}")


if __name__ == "__main__":
    main()
