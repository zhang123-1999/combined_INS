from __future__ import annotations

import argparse
import copy
import json
import math
import re
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
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_bgz_midrun_reset_probe_r1_20260323")

STRAIGHT_SPEED_MIN_M_S = 3.0
TURN_YAW_RATE_MIN_DEG_S = 8.0
PLOT_WINDOW_BEFORE_S = 20.0
PLOT_WINDOW_AFTER_S = 80.0
RESET_EVAL_WINDOW_S = 30.0
THRESHOLDS_DEGH = (1_000.0, 5_000.0, 10_000.0)


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    reset_delay_after_first_turn_s: float | None = None
    reset_bg_z_value_rad_s: float | None = None


CASE_SPECS = [
    CaseSpec("baseline", "baseline"),
    CaseSpec(
        "reset_after_first_turn_plus_10s",
        "reset bg_z/P(bg_z,*) at first_turn+10s",
        reset_delay_after_first_turn_s=10.0,
        reset_bg_z_value_rad_s=0.0,
    ),
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
    reset_time: float | None,
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
    if reset_time is not None and math.isfinite(reset_time):
        constraints["debug_reset_bg_z_state_and_cov_after_time"] = float(reset_time)
        constraints["debug_reset_bg_z_value"] = float(spec.reset_bg_z_value_rad_s)
    return cfg


def parse_reset_event(stdout_text: str) -> dict[str, float]:
    pattern = re.compile(
        r"\[Debug\] Mid-run bg_z reset applied at t=([-+0-9eE\.]+)\s+"
        r"target_t=([-+0-9eE\.]+)\s+bg_z_before=([-+0-9eE\.]+)\s+"
        r"bg_z_after=([-+0-9eE\.]+)\s+bg_var=([-+0-9eE\.]+)\s+"
        r"cleared_cross_cov_norm=([-+0-9eE\.]+)"
    )
    match = pattern.search(stdout_text)
    if match is None:
        return {}
    return {
        "reset_event_time": float(match.group(1)),
        "reset_target_time": float(match.group(2)),
        "reset_bg_z_before_rad_s": float(match.group(3)),
        "reset_bg_z_after_rad_s": float(match.group(4)),
        "reset_bg_z_var": float(match.group(5)),
        "reset_cleared_cross_cov_norm": float(match.group(6)),
    }


def interp_abs_at_time(time_s: np.ndarray, values: np.ndarray, target_t: float) -> float:
    if time_s.size == 0:
        return float("nan")
    return float(abs(np.interp(target_t, time_s, values)))


def first_crossing_delay_s(
    time_s: np.ndarray,
    values: np.ndarray,
    start_t: float,
    threshold: float,
) -> float:
    mask = time_s >= start_t
    if not np.any(mask):
        return float("nan")
    time_post = time_s[mask]
    value_post = np.abs(values[mask])
    hits = np.flatnonzero(value_post >= threshold)
    if hits.size == 0:
        return float("nan")
    return float(time_post[hits[0]] - start_t)


def fit_slope_degh_per_s(time_s: np.ndarray, values: np.ndarray, start_t: float, duration_s: float) -> float:
    mask = (time_s >= start_t) & (time_s <= start_t + duration_s)
    if np.count_nonzero(mask) < 3:
        return float("nan")
    x = time_s[mask] - start_t
    y = values[mask]
    coeff = np.polyfit(x, y, 1)
    return float(coeff[0])


def summarize_case(
    *,
    spec: CaseSpec,
    cfg_path: Path,
    sol_path: Path,
    state_series_path: Path,
    stdout_path: Path,
    stdout_text: str,
    reset_time_nominal: float | None,
) -> tuple[dict[str, Any], pd.DataFrame]:
    nav_metrics, _ = evaluate_navigation_metrics(cfg_path, sol_path)
    consistency = parse_consistency_summary(stdout_text)
    state_df = pd.read_csv(state_series_path)
    state_t = state_df["timestamp"].to_numpy(dtype=float)
    yaw_deg = state_df["total_mounting_yaw_deg"].to_numpy(dtype=float)
    bgz_degh = state_df["bg_z_degh"].to_numpy(dtype=float)

    reset_event = parse_reset_event(stdout_text)
    reset_eval_t = reset_event.get("reset_event_time")
    if reset_eval_t is None and reset_time_nominal is not None:
        reset_eval_t = float(reset_time_nominal)

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
        "yaw_final_total_deg": float(yaw_deg[-1]),
        "bg_z_final_degh": float(bgz_degh[-1]),
        "bg_z_absmax_degh": float(np.max(np.abs(bgz_degh))),
        "reset_time_nominal": float("nan") if reset_time_nominal is None else float(reset_time_nominal),
        "reset_event_time": float("nan") if reset_eval_t is None else float(reset_eval_t),
    }
    row.update(nav_metrics)
    row.update(reset_event)

    if reset_eval_t is not None and math.isfinite(reset_eval_t):
        row["bg_z_abs_at_reset_plus_1s_degh"] = interp_abs_at_time(state_t, bgz_degh, reset_eval_t + 1.0)
        row["bg_z_abs_at_reset_plus_5s_degh"] = interp_abs_at_time(state_t, bgz_degh, reset_eval_t + 5.0)
        row["bg_z_abs_at_reset_plus_10s_degh"] = interp_abs_at_time(state_t, bgz_degh, reset_eval_t + 10.0)
        row["bg_z_abs_at_reset_plus_30s_degh"] = interp_abs_at_time(
            state_t, bgz_degh, reset_eval_t + RESET_EVAL_WINDOW_S
        )
        row["bg_z_post_reset_absmax_30s_degh"] = float(
            np.max(np.abs(bgz_degh[(state_t >= reset_eval_t) & (state_t <= reset_eval_t + RESET_EVAL_WINDOW_S)]))
        )
        row["bg_z_post_reset_slope_degh_per_s"] = fit_slope_degh_per_s(
            state_t, bgz_degh, reset_eval_t, RESET_EVAL_WINDOW_S
        )
        row["yaw_abs_at_reset_plus_30s_deg"] = interp_abs_at_time(
            state_t, yaw_deg, reset_eval_t + RESET_EVAL_WINDOW_S
        )
        for threshold in THRESHOLDS_DEGH:
            key = f"rehit_{int(threshold):d}_degh_delay_s"
            row[key] = first_crossing_delay_s(state_t, bgz_degh, reset_eval_t, threshold)
    else:
        for offset in (1.0, 5.0, 10.0, 30.0):
            row[f"bg_z_abs_at_reset_plus_{int(offset):d}s_degh"] = float("nan")
        row["bg_z_post_reset_absmax_30s_degh"] = float("nan")
        row["bg_z_post_reset_slope_degh_per_s"] = float("nan")
        row["yaw_abs_at_reset_plus_30s_deg"] = float("nan")
        for threshold in THRESHOLDS_DEGH:
            row[f"rehit_{int(threshold):d}_degh_delay_s"] = float("nan")

    return row, state_df


def plot_cases(
    case_series: dict[str, pd.DataFrame],
    rows_df: pd.DataFrame,
    output_path: Path,
) -> None:
    baseline_row = rows_df[rows_df["case_id"] == "baseline"]
    reset_row = rows_df[rows_df["case_id"] != "baseline"]
    if baseline_row.empty or reset_row.empty:
        return
    reset_t = float(reset_row.iloc[0]["reset_event_time"])
    if not math.isfinite(reset_t):
        return

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    colors = {
        "baseline": "#d62728",
        "reset_after_first_turn_plus_10s": "#1f77b4",
    }
    for case_id, df in case_series.items():
        t = df["timestamp"].to_numpy(dtype=float)
        mask = (t >= reset_t - PLOT_WINDOW_BEFORE_S) & (t <= reset_t + PLOT_WINDOW_AFTER_S)
        if not np.any(mask):
            continue
        tt = t[mask] - reset_t
        axes[0].plot(
            tt,
            df.loc[mask, "bg_z_degh"].to_numpy(dtype=float),
            label=case_id,
            linewidth=2.0,
            color=colors.get(case_id),
        )
        axes[1].plot(
            tt,
            df.loc[mask, "total_mounting_yaw_deg"].to_numpy(dtype=float),
            label=case_id,
            linewidth=2.0,
            color=colors.get(case_id),
        )

    for ax in axes:
        ax.axvline(0.0, color="black", linestyle="--", linewidth=1.0)
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("bg_z (deg/h)")
    axes[1].set_ylabel("total_mounting_yaw (deg)")
    axes[1].set_xlabel("time from reset (s)")
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def render_summary(df: pd.DataFrame, first_turn_t: float) -> str:
    cols = [
        "case_id",
        "label",
        "reset_event_time",
        "overall_rmse_3d_m_aux",
        "yaw_final_total_deg",
        "bg_z_final_degh",
        "bg_z_abs_at_reset_plus_10s_degh",
        "bg_z_abs_at_reset_plus_30s_degh",
        "rehit_5000_degh_delay_s",
        "rehit_10000_degh_delay_s",
    ]
    lines = [
        "# Mid-Run bg_z Reset Probe",
        "",
        f"- first_turn_t = `{first_turn_t:.6f}`",
        f"- reset design: zero `bg_z` nominal and zero all off-diagonal `P(bg_z,*)` once after `first_turn+10s`.",
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

    if "reset_after_first_turn_plus_10s" in set(df["case_id"].tolist()):
        reset_row = df[df["case_id"] == "reset_after_first_turn_plus_10s"].iloc[0]
        lines.extend(
            [
                "",
                "## Readout",
                "",
                f"- reset event time = `{float(reset_row['reset_event_time']):.6f}`.",
                f"- `bg_z` after reset reaches `5000 deg/h` again in `{float(reset_row['rehit_5000_degh_delay_s']):.6f} s` and `10000 deg/h` in `{float(reset_row['rehit_10000_degh_delay_s']):.6f} s`.",
                f"- `bg_z_abs_at_reset_plus_30s = {float(reset_row['bg_z_abs_at_reset_plus_30s_degh']):.6f} deg/h`, `yaw_final = {float(reset_row['yaw_final_total_deg']):.6f} deg`.",
            ]
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Probe whether full-window bg_z divergence reappears after a mid-run bg_z/P(bg_z,*) reset."
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
    case_series: dict[str, pd.DataFrame] = {}

    for spec in CASE_SPECS:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        reset_time: float | None = None
        if spec.reset_delay_after_first_turn_s is not None:
            reset_time = first_turn_t + float(spec.reset_delay_after_first_turn_s)
        cfg = build_case_config(base_cfg, case_dir, spec, reset_time=reset_time)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        stdout_text = run_solver(exe_path, cfg_path)
        stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
        stdout_path.write_text(stdout_text, encoding="utf-8")

        sol_path = case_dir / f"SOL_{spec.case_id}.txt"
        state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
        if not sol_path.exists() or not state_series_path.exists():
            raise RuntimeError(f"missing solver outputs for {spec.case_id}")

        row, state_df = summarize_case(
            spec=spec,
            cfg_path=cfg_path,
            sol_path=sol_path,
            state_series_path=state_series_path,
            stdout_path=stdout_path,
            stdout_text=stdout_text,
            reset_time_nominal=reset_time,
        )
        row["first_turn_t"] = first_turn_t
        rows.append(row)
        case_series[spec.case_id] = state_df
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
    summary_path.write_text(render_summary(df, first_turn_t), encoding="utf-8")
    manifest_path = outdir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    plot_path = outdir / "bgz_yaw_reset_window.png"
    plot_cases(case_series, df, plot_path)

    print(f"[done] metrics: {metrics_path}")
    print(f"[done] summary: {summary_path}")
    print(f"[done] manifest: {manifest_path}")
    print(f"[done] plot: {plot_path}")


if __name__ == "__main__":
    main()
