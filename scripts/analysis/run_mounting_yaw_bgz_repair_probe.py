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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
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
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_bgz_repair_probe_r1_20260323")

A_P0_BGZ_STD_DEGH = 30.0
B_GATE_FORWARD_SPEED_MIN = 3.0
B_GATE_YAW_RATE_MIN_DEG_S = 8.0
B_GATE_LATERAL_ACC_MIN = 0.3
B_GATE_MIN_SCALE = 0.0
B_COV_FORGET_TAU_S = 15.0
TAIL_WINDOW_S = 120.0


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    p0_bgz_std_degh: float | None = None
    enable_bgz_gate: bool = False
    enable_bgz_forgetting: bool = False
    gate_forward_speed_min: float = B_GATE_FORWARD_SPEED_MIN
    gate_yaw_rate_min_deg_s: float = B_GATE_YAW_RATE_MIN_DEG_S
    gate_lateral_acc_min: float = B_GATE_LATERAL_ACC_MIN
    gate_min_scale: float = B_GATE_MIN_SCALE
    cov_forget_tau_s: float = B_COV_FORGET_TAU_S


CASE_SPECS = [
    CaseSpec("baseline", "baseline"),
    CaseSpec("a_p0_bgz_30degh", "A: tighten P0(bg_z) to 30 deg/h", p0_bgz_std_degh=A_P0_BGZ_STD_DEGH),
    CaseSpec("b_gate_only", "B1: bg_z observability gate", enable_bgz_gate=True),
    CaseSpec(
        "ab_p0_gate_only",
        "A+B1: tightened P0 + current gate",
        p0_bgz_std_degh=A_P0_BGZ_STD_DEGH,
        enable_bgz_gate=True,
    ),
    CaseSpec(
        "b_gate_forget",
        "B1+B2: gate + covariance forgetting",
        enable_bgz_gate=True,
        enable_bgz_forgetting=True,
    ),
    CaseSpec(
        "ab_p0_gate_forget",
        "A+B: tightened P0 + gate + forgetting",
        p0_bgz_std_degh=A_P0_BGZ_STD_DEGH,
        enable_bgz_gate=True,
        enable_bgz_forgetting=True,
    ),
    CaseSpec(
        "b_gate_only_yaw12",
        "B1-strict: yaw-only gate (12 deg/s)",
        enable_bgz_gate=True,
        gate_yaw_rate_min_deg_s=12.0,
        gate_lateral_acc_min=1e9,
    ),
    CaseSpec(
        "ab_p0_gate_only_yaw12",
        "A+B1-strict: tightened P0 + yaw-only gate (12 deg/s)",
        p0_bgz_std_degh=A_P0_BGZ_STD_DEGH,
        enable_bgz_gate=True,
        gate_yaw_rate_min_deg_s=12.0,
        gate_lateral_acc_min=1e9,
    ),
]


def degh_to_rad_s(value_degh: float) -> float:
    return value_degh * math.pi / 180.0 / 3600.0


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


def fit_slope(time_s: np.ndarray, values: np.ndarray) -> float:
    if time_s.size < 3:
        return float("nan")
    x = time_s - time_s[0]
    coeff = np.polyfit(x, values, 1)
    return float(coeff[0])


def summarize_state_series(path: Path) -> dict[str, float]:
    df = pd.read_csv(path)
    t = df["timestamp"].to_numpy(dtype=float)
    yaw = df["total_mounting_yaw_deg"].to_numpy(dtype=float)
    bgz = df["bg_z_degh"].to_numpy(dtype=float)
    tail_mask = (t >= t[-1] - TAIL_WINDOW_S) if t.size > 0 else np.zeros(0, dtype=bool)
    if not np.any(tail_mask):
        tail_mask = np.ones_like(t, dtype=bool)
    t_tail = t[tail_mask]
    yaw_tail = yaw[tail_mask]
    bgz_tail = bgz[tail_mask]
    return {
        "yaw_final_total_deg": float(yaw[-1]),
        "yaw_absmax_deg": float(np.max(np.abs(yaw))),
        "yaw_tail_abs_mean_deg": float(np.mean(np.abs(yaw_tail))),
        "yaw_tail_slope_deg_s": fit_slope(t_tail, yaw_tail),
        "bg_z_final_degh": float(bgz[-1]),
        "bg_z_absmax_degh": float(np.max(np.abs(bgz))),
        "bg_z_tail_abs_mean_degh": float(np.mean(np.abs(bgz_tail))),
        "bg_z_tail_slope_degh_s": fit_slope(t_tail, bgz_tail),
    }


def build_case_config(base_cfg: dict[str, Any], case_dir: Path, spec: CaseSpec) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg["fusion"]
    constraints = fusion.setdefault("constraints", {})
    init_cfg = fusion.setdefault("init", {})
    p0_diag = list(init_cfg["P0_diag"])

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)

    constraints["enable_consistency_log"] = True
    constraints["enable_diagnostics"] = False
    constraints["enable_mechanism_log"] = False

    if spec.p0_bgz_std_degh is not None:
        std_rad_s = degh_to_rad_s(spec.p0_bgz_std_degh)
        p0_diag[14] = float(std_rad_s * std_rad_s)
        init_cfg["P0_diag"] = p0_diag

    if spec.enable_bgz_gate:
        constraints["enable_bgz_observability_gate"] = True
        constraints["bgz_gate_forward_speed_min"] = float(spec.gate_forward_speed_min)
        constraints["bgz_gate_yaw_rate_min_deg_s"] = float(spec.gate_yaw_rate_min_deg_s)
        constraints["bgz_gate_lateral_acc_min"] = float(spec.gate_lateral_acc_min)
        constraints["bgz_gate_min_scale"] = float(spec.gate_min_scale)
    if spec.enable_bgz_forgetting:
        constraints["enable_bgz_covariance_forgetting"] = True
        constraints["bgz_cov_forgetting_tau_s"] = float(spec.cov_forget_tau_s)

    return cfg


def render_summary(df: pd.DataFrame) -> str:
    cols = [
        "case_id",
        "label",
        "yaw_final_total_deg",
        "yaw_tail_abs_mean_deg",
        "yaw_tail_slope_deg_s",
        "bg_z_final_degh",
        "bg_z_tail_abs_mean_degh",
        "bg_z_tail_slope_degh_s",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "overall_rmse_3d_m_aux",
    ]
    lines = [
        "# bg_z Repair Probe",
        "",
        "- primary readout = state convergence (`bg_z`, `mounting_yaw`) instead of nav RMSE.",
        f"- A uses `P0(bg_z)` tightening to `{A_P0_BGZ_STD_DEGH:.1f} deg/h`.",
        (
            "- default B cases use `bg_z` observability gate with "
            f"`v_x>={B_GATE_FORWARD_SPEED_MIN:.1f} m/s`, "
            f"`yaw_rate>={B_GATE_YAW_RATE_MIN_DEG_S:.1f} deg/s` or "
            f"`|a_y|>={B_GATE_LATERAL_ACC_MIN:.1f} m/s^2`, "
            f"plus covariance forgetting tau `{B_COV_FORGET_TAU_S:.1f} s` when enabled; "
            "strict yaw-only variants are called out in the case label."
        ),
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


def plot_state_series(case_frames: dict[str, pd.DataFrame], output_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for case_id, df in case_frames.items():
        t = df["timestamp"].to_numpy(dtype=float)
        axes[0].plot(t, df["bg_z_degh"].to_numpy(dtype=float), label=case_id, linewidth=1.6)
        axes[1].plot(
            t,
            df["total_mounting_yaw_deg"].to_numpy(dtype=float),
            label=case_id,
            linewidth=1.6,
        )
    axes[0].set_ylabel("bg_z (deg/h)")
    axes[1].set_ylabel("total_mounting_yaw (deg)")
    axes[1].set_xlabel("timestamp (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[0].legend(ncol=2)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare A/B repair variants for full-window bg_z and mounting_yaw convergence."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    base_cfg_path = REPO_ROOT / args.base_config
    exe_path = REPO_ROOT / args.exe
    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    base_cfg = load_yaml(base_cfg_path)
    rows: list[dict[str, Any]] = []
    manifest: list[dict[str, Any]] = []
    case_frames: dict[str, pd.DataFrame] = {}

    for spec in CASE_SPECS:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        cfg = build_case_config(base_cfg, case_dir, spec)
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
        state_df = pd.read_csv(state_series_path)
        case_frames[spec.case_id] = state_df

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
            "odo_accept_ratio": consistency.get("ODO", {}).get("accept_ratio"),
            "nhc_accept_ratio": consistency.get("NHC", {}).get("accept_ratio"),
            "p0_bgz_std_degh": float("nan") if spec.p0_bgz_std_degh is None else float(spec.p0_bgz_std_degh),
            "enable_bgz_gate": bool(spec.enable_bgz_gate),
            "enable_bgz_forgetting": bool(spec.enable_bgz_forgetting),
            "bgz_gate_forward_speed_min": float(spec.gate_forward_speed_min),
            "bgz_gate_yaw_rate_min_deg_s": float(spec.gate_yaw_rate_min_deg_s),
            "bgz_gate_lateral_acc_min": float(spec.gate_lateral_acc_min),
            "bgz_gate_min_scale": float(spec.gate_min_scale),
            "bgz_cov_forget_tau_s": float(spec.cov_forget_tau_s),
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
    plot_path = outdir / "bgz_yaw_repair_compare.png"
    plot_state_series(case_frames, plot_path)

    print(f"[done] metrics: {metrics_path}")
    print(f"[done] summary: {summary_path}")
    print(f"[done] manifest: {manifest_path}")
    print(f"[done] plot: {plot_path}")


if __name__ == "__main__":
    main()
