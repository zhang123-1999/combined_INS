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


BASE_FULL_CONFIG = Path(
    "output/debug_mounting_yaw_fullgnss_sensor_split_20260322/"
    "cases/yaw_only_both_nominal_aligned/config_yaw_only_both_nominal_aligned.yaml"
)
BASE_SHORT_CONFIG = Path(
    "output/debug_mounting_yaw_mechanism_short_20260322/"
    "cases/yaw_both_short/config_yaw_both_short.yaml"
)
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_nhc_bgz_compare_r1_20260322")


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    disable_gyro_bias: bool = False
    debug_odo_disable_bgz_state_update: bool = False
    debug_nhc_disable_bgz_state_update: bool = False
    debug_run_odo_before_nhc: bool = False
    debug_seed_mount_yaw_bgz_cov_before_first_nhc: float | None = None
    debug_seed_bg_z_before_first_nhc: float | None = None
    debug_seed_bg_z_att_cov_before_first_nhc: tuple[float, float, float] | None = None
    init_mounting_yaw0_deg: float | None = None


FULL_CASES = [
    CaseSpec("baseline"),
    CaseSpec("freeze_gyro_bias", disable_gyro_bias=True),
    CaseSpec("nhc_disable_bgz_state_update", debug_nhc_disable_bgz_state_update=True),
    CaseSpec("odo_disable_bgz_state_update", debug_odo_disable_bgz_state_update=True),
    CaseSpec(
        "odo_nhc_disable_bgz_state_update",
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
    ),
]

SHORT_CASES = [
    CaseSpec("baseline"),
    CaseSpec("nhc_disable_bgz_state_update", debug_nhc_disable_bgz_state_update=True),
    CaseSpec(
        "odo_nhc_disable_bgz_state_update",
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
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


def build_case_config(
    base_cfg: dict[str, Any], case_dir: Path, spec: CaseSpec, *, enable_mechanism: bool
) -> dict[str, Any]:
    cfg = json.loads(json.dumps(base_cfg))
    fusion = cfg["fusion"]
    constraints = fusion.setdefault("constraints", {})
    ablation = fusion.setdefault("ablation", {})

    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)

    constraints["enable_consistency_log"] = True
    constraints["enable_mechanism_log"] = bool(enable_mechanism)
    constraints["mechanism_log_stride"] = 1
    constraints["debug_odo_disable_bgz_jacobian"] = False
    constraints["debug_odo_disable_bgz_state_update"] = bool(
        spec.debug_odo_disable_bgz_state_update
    )
    constraints["debug_nhc_disable_bgz_state_update"] = bool(
        spec.debug_nhc_disable_bgz_state_update
    )
    constraints["debug_run_odo_before_nhc"] = bool(spec.debug_run_odo_before_nhc)
    if spec.debug_seed_mount_yaw_bgz_cov_before_first_nhc is not None:
        constraints["debug_seed_mount_yaw_bgz_cov_before_first_nhc"] = float(
            spec.debug_seed_mount_yaw_bgz_cov_before_first_nhc
        )
    if spec.debug_seed_bg_z_before_first_nhc is not None:
        constraints["debug_seed_bg_z_before_first_nhc"] = float(
            spec.debug_seed_bg_z_before_first_nhc
        )
    if spec.debug_seed_bg_z_att_cov_before_first_nhc is not None:
        constraints["debug_seed_bg_z_att_cov_before_first_nhc"] = [
            float(value) for value in spec.debug_seed_bg_z_att_cov_before_first_nhc
        ]
    if spec.init_mounting_yaw0_deg is not None:
        cfg.setdefault("fusion", {}).setdefault("init", {})["mounting_yaw0"] = float(
            spec.init_mounting_yaw0_deg
        )

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


def empty_mechanism_metrics(prefix: str) -> dict[str, float]:
    cols = [
        "count",
        "sum_dx_bg_z",
        "abs_sum_dx_bg_z",
        "mean_k_row_bg_z_norm",
        "first_prior_corr_att_z_bg_z",
        "first_post_corr_att_z_bg_z",
        "first_prior_corr_mount_yaw_bg_z",
        "first_post_corr_mount_yaw_bg_z",
        "mean_prior_corr_att_z_bg_z",
        "mean_post_corr_att_z_bg_z",
        "mean_prior_corr_mount_yaw_bg_z",
        "mean_post_corr_mount_yaw_bg_z",
    ]
    return {f"{prefix}_{col}": float("nan") for col in cols}


def summarize_mechanism(path: Path) -> dict[str, float]:
    df = pd.read_csv(path)
    out: dict[str, float] = {"mechanism_rows": float(len(df))}
    for tag in ("ODO", "NHC"):
        tag_df = df[df["tag"] == tag].reset_index(drop=True)
        prefix = tag.lower()
        if tag_df.empty:
            out.update(empty_mechanism_metrics(prefix))
            continue

        first = tag_df.iloc[0]
        out[f"{prefix}_count"] = float(len(tag_df))
        out[f"{prefix}_sum_dx_bg_z"] = float(tag_df["dx_bg_z"].sum())
        out[f"{prefix}_abs_sum_dx_bg_z"] = float(tag_df["dx_bg_z"].abs().sum())
        out[f"{prefix}_mean_k_row_bg_z_norm"] = float(tag_df["k_row_bg_z_norm"].mean())
        out[f"{prefix}_first_prior_corr_att_z_bg_z"] = float(
            first["prior_corr_att_z_bg_z"]
        )
        out[f"{prefix}_first_post_corr_att_z_bg_z"] = float(
            first["post_corr_att_z_bg_z"]
        )
        out[f"{prefix}_first_prior_corr_mount_yaw_bg_z"] = float(
            first["prior_corr_mount_yaw_bg_z"]
        )
        out[f"{prefix}_first_post_corr_mount_yaw_bg_z"] = float(
            first["post_corr_mount_yaw_bg_z"]
        )
        out[f"{prefix}_mean_prior_corr_att_z_bg_z"] = float(
            tag_df["prior_corr_att_z_bg_z"].mean()
        )
        out[f"{prefix}_mean_post_corr_att_z_bg_z"] = float(
            tag_df["post_corr_att_z_bg_z"].mean()
        )
        out[f"{prefix}_mean_prior_corr_mount_yaw_bg_z"] = float(
            tag_df["prior_corr_mount_yaw_bg_z"].mean()
        )
        out[f"{prefix}_mean_post_corr_mount_yaw_bg_z"] = float(
            tag_df["post_corr_mount_yaw_bg_z"].mean()
        )
    return out


def format_float(value: Any) -> str:
    try:
        if isinstance(value, str):
            return value
        return f"{float(value):.6f}"
    except Exception:
        return "NA"


def render_table(df: pd.DataFrame, cols: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for _, row in df.iterrows():
        values = [format_float(row[col]) for col in cols]
        lines.append("| " + " | ".join(values) + " |")
    return lines


def render_summary(full_df: pd.DataFrame, short_df: pd.DataFrame) -> str:
    full_cols = [
        "case_id",
        "overall_rmse_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "yaw_final_total_deg",
        "bg_z_final_degh",
    ]
    short_cols = [
        "case_id",
        "yaw_final_total_deg",
        "bg_z_final_degh",
        "odo_sum_dx_bg_z",
        "nhc_sum_dx_bg_z",
        "odo_first_post_corr_att_z_bg_z",
        "nhc_first_post_corr_att_z_bg_z",
        "odo_first_post_corr_mount_yaw_bg_z",
        "nhc_first_post_corr_mount_yaw_bg_z",
    ]
    lines = [
        "# Mounting Yaw NHC/bg_z Compare",
        "",
        "## Full-window",
        "",
        *render_table(full_df, full_cols),
        "",
        "## Short-window mechanism",
        "",
        *render_table(short_df, short_cols),
        "",
    ]
    return "\n".join(lines)


def run_full_window(exe_path: Path, base_cfg_path: Path, outdir: Path) -> pd.DataFrame:
    base_cfg = load_yaml(REPO_ROOT / base_cfg_path)
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    rows: list[dict[str, Any]] = []
    for spec in FULL_CASES:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        cfg = build_case_config(base_cfg, case_dir, spec, enable_mechanism=False)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        stdout_text = run_solver(exe_path, cfg_path)
        stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
        stdout_path.write_text(stdout_text, encoding="utf-8")

        sol_path = case_dir / f"SOL_{spec.case_id}.txt"
        state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
        if not sol_path.exists() or not state_series_path.exists():
            raise RuntimeError(f"missing full-window outputs for {spec.case_id}")

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
            "debug_odo_disable_bgz_state_update": spec.debug_odo_disable_bgz_state_update,
            "debug_nhc_disable_bgz_state_update": spec.debug_nhc_disable_bgz_state_update,
            "debug_run_odo_before_nhc": spec.debug_run_odo_before_nhc,
            "odo_accept_ratio": consistency.get("ODO", {}).get("accept_ratio"),
            "odo_nis_mean": consistency.get("ODO", {}).get("nis_mean"),
            "nhc_accept_ratio": consistency.get("NHC", {}).get("accept_ratio"),
            "nhc_nis_mean": consistency.get("NHC", {}).get("nis_mean"),
        }
        row.update(nav_metrics)
        row.update(state_metrics)
        rows.append(row)

    return pd.DataFrame(rows)


def run_short_window(exe_path: Path, base_cfg_path: Path, outdir: Path) -> pd.DataFrame:
    base_cfg = load_yaml(REPO_ROOT / base_cfg_path)
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    rows: list[dict[str, Any]] = []
    for spec in SHORT_CASES:
        case_dir = cases_dir / spec.case_id
        ensure_dir(case_dir)
        cfg = build_case_config(base_cfg, case_dir, spec, enable_mechanism=True)
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        stdout_text = run_solver(exe_path, cfg_path)
        stdout_path = case_dir / f"solver_stdout_{spec.case_id}.txt"
        stdout_path.write_text(stdout_text, encoding="utf-8")

        sol_path = case_dir / f"SOL_{spec.case_id}.txt"
        state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
        mechanism_path = case_dir / f"SOL_{spec.case_id}_mechanism.csv"
        if not sol_path.exists() or not state_series_path.exists() or not mechanism_path.exists():
            raise RuntimeError(f"missing short-window outputs for {spec.case_id}")

        state_metrics = summarize_state_series(state_series_path)
        mechanism_metrics = summarize_mechanism(mechanism_path)

        row: dict[str, Any] = {
            "case_id": spec.case_id,
            "config_path": rel_from_root(cfg_path, REPO_ROOT),
            "sol_path": rel_from_root(sol_path, REPO_ROOT),
            "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
            "mechanism_path": rel_from_root(mechanism_path, REPO_ROOT),
            "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
            "sol_mtime": mtime_text(sol_path),
            "state_series_mtime": mtime_text(state_series_path),
            "mechanism_mtime": mtime_text(mechanism_path),
            "stdout_mtime": mtime_text(stdout_path),
            "debug_odo_disable_bgz_state_update": spec.debug_odo_disable_bgz_state_update,
            "debug_nhc_disable_bgz_state_update": spec.debug_nhc_disable_bgz_state_update,
            "debug_run_odo_before_nhc": spec.debug_run_odo_before_nhc,
        }
        row.update(state_metrics)
        row.update(mechanism_metrics)
        rows.append(row)

    return pd.DataFrame(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare NHC/ODO bg_z state-update masks on full and short windows."
    )
    parser.add_argument("--base-full-config", type=Path, default=BASE_FULL_CONFIG)
    parser.add_argument("--base-short-config", type=Path, default=BASE_SHORT_CONFIG)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    exe_path = REPO_ROOT / args.exe
    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)

    full_df = run_full_window(exe_path, args.base_full_config, outdir / "full_window")
    short_df = run_short_window(exe_path, args.base_short_config, outdir / "short_window")

    full_metrics_path = outdir / "full_window_metrics.csv"
    short_metrics_path = outdir / "short_window_metrics.csv"
    full_df.to_csv(full_metrics_path, index=False)
    short_df.to_csv(short_metrics_path, index=False)

    summary_path = outdir / "summary.md"
    summary_path.write_text(render_summary(full_df, short_df), encoding="utf-8")

    manifest = {
        "full_window_metrics": rel_from_root(full_metrics_path, REPO_ROOT),
        "short_window_metrics": rel_from_root(short_metrics_path, REPO_ROOT),
        "summary": rel_from_root(summary_path, REPO_ROOT),
    }
    manifest_path = outdir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[done] full metrics: {full_metrics_path}")
    print(f"[done] short metrics: {short_metrics_path}")
    print(f"[done] summary: {summary_path}")
    print(f"[done] manifest: {manifest_path}")


if __name__ == "__main__":
    main()
