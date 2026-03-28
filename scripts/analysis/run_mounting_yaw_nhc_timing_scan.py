from __future__ import annotations

import argparse
import copy
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
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import mtime_text  # noqa: E402


BASE_CONFIG_DEFAULT = Path(
    "output/debug_mounting_yaw_fullgnss_sensor_split_20260322/"
    "cases/yaw_only_both_nominal_aligned/config_yaw_only_both_nominal_aligned.yaml"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_nhc_timing_scan_r1_20260323")

STRAIGHT_SPEED_MIN_M_S = 3.0
TURN_YAW_RATE_MIN_DEG_S = 8.0


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    nhc_delay_s: float | None = None
    use_first_turn_gate: bool = False
    debug_run_odo_before_nhc: bool = False


CASE_SPECS = [
    CaseSpec("baseline", "NHC delay 0s", nhc_delay_s=0.0),
    CaseSpec("nhc_delay_1s", "NHC delay 1s", nhc_delay_s=1.0),
    CaseSpec("nhc_delay_3s", "NHC delay 3s", nhc_delay_s=3.0),
    CaseSpec("nhc_delay_first_turn", "NHC delay until first turn", use_first_turn_gate=True),
    CaseSpec("odo_before_nhc", "odo_before_nhc", debug_run_odo_before_nhc=True),
]


def load_pos_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def find_first_turn_time(pos_df: pd.DataFrame, start_t: float) -> float:
    t = pos_df["t"].to_numpy(dtype=float)
    speed = np.hypot(pos_df["vn"].to_numpy(dtype=float), pos_df["ve"].to_numpy(dtype=float))
    yaw = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.rad2deg(np.gradient(yaw, t))
    mask = (t >= start_t) & (speed >= STRAIGHT_SPEED_MIN_M_S) & (
        np.abs(yaw_rate_deg_s) >= TURN_YAW_RATE_MIN_DEG_S
    )
    hits = np.flatnonzero(mask)
    if hits.size == 0:
        raise RuntimeError("failed to detect first turn time from POS_converted.txt")
    return float(t[hits[0]])


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
        raise RuntimeError(f"solver failed for {cfg_path.name}: returncode={proc.returncode}\n{merged}")
    return merged


def build_case_config(
    base_cfg: dict[str, Any],
    case_dir: Path,
    spec: CaseSpec,
    *,
    gate_time: float | None,
) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg["fusion"]
    constraints = fusion.setdefault("constraints", {})

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)

    constraints["enable_consistency_log"] = True
    constraints["enable_diagnostics"] = False
    constraints["enable_mechanism_log"] = False
    constraints["debug_run_odo_before_nhc"] = bool(spec.debug_run_odo_before_nhc)
    if gate_time is not None and math.isfinite(gate_time):
        constraints["debug_nhc_enable_after_time"] = float(gate_time)
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


def render_summary(df: pd.DataFrame) -> str:
    cols = [
        "case_id",
        "label",
        "nhc_enable_after_time",
        "overall_rmse_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "yaw_final_total_deg",
        "bg_z_final_degh",
        "bg_z_absmax_degh",
    ]
    lines = [
        "# NHC Timing Scan",
        "",
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for _, row in df.iterrows():
        rendered: list[str] = []
        for col in cols:
            value = row[col]
            if isinstance(value, str):
                rendered.append(value)
            else:
                rendered.append(f"{float(value):.6f}")
        lines.append("| " + " | ".join(rendered) + " |")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Scan NHC activation timing on the full-GNSS mounting_yaw/bg_z baseline."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    base_cfg_path = REPO_ROOT / args.base_config
    pos_path = REPO_ROOT / args.pos
    exe_path = REPO_ROOT / args.exe
    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)

    base_cfg = load_yaml(base_cfg_path)
    pos_df = load_pos_dataframe(pos_path)
    start_t = float(base_cfg["fusion"]["starttime"])
    first_turn_t = find_first_turn_time(pos_df, start_t)
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    rows: list[dict[str, Any]] = []
    manifest: list[dict[str, Any]] = []

    for spec in CASE_SPECS:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        gate_time: float | None = None
        if spec.use_first_turn_gate:
            gate_time = first_turn_t
        elif spec.nhc_delay_s is not None:
            gate_time = start_t + float(spec.nhc_delay_s)
        cfg = build_case_config(base_cfg, case_dir, spec, gate_time=gate_time)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        stdout_text = run_solver(exe_path, cfg_path)
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
            "label": spec.label,
            "config_path": rel_from_root(cfg_path, REPO_ROOT),
            "sol_path": rel_from_root(sol_path, REPO_ROOT),
            "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
            "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
            "config_mtime": mtime_text(cfg_path),
            "sol_mtime": mtime_text(sol_path),
            "state_series_mtime": mtime_text(state_series_path),
            "stdout_mtime": mtime_text(stdout_path),
            "start_t": start_t,
            "first_turn_t": first_turn_t,
            "nhc_enable_after_time": float("nan") if gate_time is None else float(gate_time),
            "debug_run_odo_before_nhc": bool(spec.debug_run_odo_before_nhc),
            "odo_accept_ratio": consistency.get("ODO", {}).get("accept_ratio"),
            "odo_nis_mean": consistency.get("ODO", {}).get("nis_mean"),
            "nhc_accept_ratio": consistency.get("NHC", {}).get("accept_ratio"),
            "nhc_nis_mean": consistency.get("NHC", {}).get("nis_mean"),
        }
        row.update(nav_metrics)
        row.update(state_metrics)
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
