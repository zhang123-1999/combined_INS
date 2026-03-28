from __future__ import annotations

import argparse
import copy
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import pandas as pd
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.run_data2_state_sanity_matrix import evaluate_navigation_metrics
from scripts.analysis.run_mounting_yaw_prior_sweep import enrich_metrics
from scripts.analysis.run_mounting_yaw_sigma_sweep import (
    compute_case_metrics,
    ensure_dir,
    mtime_text,
    rel,
    render_table,
    run_solver,
    save_yaml,
)


EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-yaw-nonanchor-check-r1"
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
ANCHOR_CONFIG_DEFAULT = Path("output/debug_mounting_yaw_prior_sweep_20260322/config_working_mounting_yaw_largeP_smallQ.yaml")
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_nonanchor_check_20260322")


def build_nonanchor_config(anchor_cfg: dict[str, Any], case_dir: Path) -> tuple[dict[str, Any], dict[str, Path]]:
    cfg = copy.deepcopy(anchor_cfg)
    fusion = cfg["fusion"]
    init_cfg = fusion.setdefault("init", {})
    case_name = "no_anchor_largeP_smallQ"

    init_cfg["runtime_truth_anchor_pva"] = False

    sol_path = case_dir / f"SOL_{case_name}.txt"
    state_series_path = case_dir / f"state_series_{case_name}.csv"
    cfg_path = case_dir / f"config_{case_name}.yaml"
    stdout_path = case_dir / f"solver_stdout_{case_name}.txt"
    diag_path = case_dir / f"DIAG_{case_name}.txt"

    fusion["output_path"] = rel(sol_path)
    fusion["state_series_output_path"] = rel(state_series_path)

    return cfg, {
        "cfg_path": cfg_path,
        "sol_path": sol_path,
        "state_series_path": state_series_path,
        "stdout_path": stdout_path,
        "diag_path": diag_path,
    }


def compute_full_metrics(
    *,
    case_name: str,
    cfg_path: Path,
    cfg: dict[str, Any],
    sol_path: Path,
    state_series_path: Path,
    diag_path: Path,
    stdout_text: str,
    pos_path: Path,
) -> dict[str, Any]:
    state_metrics, _, _ = compute_case_metrics(
        case_name=case_name,
        sigma_mounting_yaw=float(cfg["fusion"]["noise"]["sigma_mounting_yaw"]),
        cfg=cfg,
        state_series_path=state_series_path,
        diag_path=diag_path,
        pos_path=pos_path,
        stdout_text=stdout_text,
    )
    state_metrics = enrich_metrics(
        state_metrics,
        target_total_yaw_deg=float(cfg["fusion"]["constraints"]["imu_mounting_angle"][2]),
        std_mounting_yaw_deg=float(cfg["fusion"]["init"]["std_mounting_yaw"]),
    )
    nav_metrics, _ = evaluate_navigation_metrics(cfg_path, sol_path)
    merged = dict(state_metrics)
    merged.update(nav_metrics)
    return merged


def summarize_rows(df: pd.DataFrame) -> pd.DataFrame:
    cols = [
        "case_name",
        "runtime_truth_anchor_pva",
        "steady_center_deg",
        "steady_center_error_deg",
        "final_total_mounting_yaw_deg",
        "final_error_deg",
        "overall_std_deg",
        "after_200_peak_to_peak_deg",
        "turn_neg_minus_turn_pos_deg",
        "overall_rmse_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "mean_outage_rmse_3d_m",
        "max_outage_final_err_3d_m",
        "nhc_accept_ratio",
        "odo_accept_ratio",
    ]
    return df[cols].copy()


def write_summary(
    output_dir: Path,
    *,
    exp_id: str,
    anchor_config_path: Path,
    nonanchor_config_path: Path,
    exe_path: Path,
    summary_df: pd.DataFrame,
) -> None:
    anchor_row = summary_df.loc[summary_df["runtime_truth_anchor_pva"] == True].iloc[0]
    noanchor_row = summary_df.loc[summary_df["runtime_truth_anchor_pva"] == False].iloc[0]

    lines = [
        "# mounting_yaw non-anchor check",
        "",
        f"Date: `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{exp_id}`",
        f"- anchor reference config: `{rel(anchor_config_path)}`",
        f"- non-anchor config: `{rel(nonanchor_config_path)}`",
        f"- solver: `{rel(exe_path)}`",
        "",
        "## Comparison",
        "",
    ]
    lines.extend(render_table(summary_df, list(summary_df.columns)))
    lines.extend(
        [
            "",
            "## Assessment",
            "",
            (
                f"- `mounting_yaw` steady center: anchor=`{anchor_row['steady_center_deg']:.6f} deg`, "
                f"no-anchor=`{noanchor_row['steady_center_deg']:.6f} deg`, "
                f"delta=`{noanchor_row['steady_center_deg'] - anchor_row['steady_center_deg']:.6f} deg`."
            ),
            (
                f"- steady fluctuation (`after_200 peak-to-peak`): anchor=`{anchor_row['after_200_peak_to_peak_deg']:.6f} deg`, "
                f"no-anchor=`{noanchor_row['after_200_peak_to_peak_deg']:.6f} deg`."
            ),
            (
                f"- outage navigation: anchor mean/max=`{anchor_row['mean_outage_rmse_3d_m']:.6f}/{anchor_row['max_outage_final_err_3d_m']:.6f} m`, "
                f"no-anchor mean/max=`{noanchor_row['mean_outage_rmse_3d_m']:.6f}/{noanchor_row['max_outage_final_err_3d_m']:.6f} m`."
            ),
            (
                f"- consistency: anchor `ODO/NHC accept={anchor_row['odo_accept_ratio']:.6f}/{anchor_row['nhc_accept_ratio']:.6f}`, "
                f"no-anchor `ODO/NHC accept={noanchor_row['odo_accept_ratio']:.6f}/{noanchor_row['nhc_accept_ratio']:.6f}`."
            ),
            "",
            "## Freshness",
            "",
            f"- anchor config mtime: `{mtime_text(anchor_config_path)}`",
            f"- non-anchor config mtime: `{mtime_text(nonanchor_config_path)}`",
            f"- solver mtime: `{mtime_text(exe_path)}`",
            f"- metrics csv: `{rel(output_dir / 'metrics.csv')}`",
            f"- manifest: `{rel(output_dir / 'manifest.json')}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check the selected large-P small-Q mounting yaw config under runtime_truth_anchor_pva=false.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--anchor-config", type=Path, default=ANCHOR_CONFIG_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    exe_path = (REPO_ROOT / args.exe).resolve()
    anchor_config_path = (REPO_ROOT / args.anchor_config).resolve()
    pos_path = (REPO_ROOT / args.pos).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()

    if not exe_path.exists():
        raise FileNotFoundError(f"missing solver executable: {exe_path}")
    if not anchor_config_path.exists():
        raise FileNotFoundError(f"missing anchor config: {anchor_config_path}")
    if not pos_path.exists():
        raise FileNotFoundError(f"missing POS file: {pos_path}")

    ensure_dir(output_dir)
    case_root = output_dir / "cases"
    ensure_dir(case_root)

    anchor_cfg = yaml.safe_load(anchor_config_path.read_text(encoding="utf-8"))
    anchor_sol_path = (REPO_ROOT / anchor_cfg["fusion"]["output_path"]).resolve()
    anchor_state_series_path = (REPO_ROOT / anchor_cfg["fusion"]["state_series_output_path"]).resolve()
    anchor_case_dir = anchor_sol_path.parent
    anchor_stdout_path = anchor_case_dir / f"solver_stdout_{anchor_case_dir.name}.txt"
    anchor_diag_path = anchor_case_dir / f"DIAG_{anchor_case_dir.name}.txt"
    if not anchor_stdout_path.exists():
        raise FileNotFoundError(f"missing anchor stdout: {anchor_stdout_path}")
    if not anchor_diag_path.exists():
        raise FileNotFoundError(f"missing anchor diag: {anchor_diag_path}")

    anchor_stdout_text = anchor_stdout_path.read_text(encoding="utf-8", errors="ignore")
    anchor_metrics = compute_full_metrics(
        case_name="anchor_largeP_smallQ",
        cfg_path=anchor_config_path,
        cfg=anchor_cfg,
        sol_path=anchor_sol_path,
        state_series_path=anchor_state_series_path,
        diag_path=anchor_diag_path,
        stdout_text=anchor_stdout_text,
        pos_path=pos_path,
    )
    anchor_metrics["runtime_truth_anchor_pva"] = True

    noanchor_case_dir = case_root / "no_anchor_largeP_smallQ"
    ensure_dir(noanchor_case_dir)
    noanchor_cfg, noanchor_paths = build_nonanchor_config(anchor_cfg, noanchor_case_dir)
    save_yaml(noanchor_cfg, noanchor_paths["cfg_path"])

    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    noanchor_stdout_text = run_solver(exe_path, noanchor_paths["cfg_path"])
    noanchor_paths["stdout_path"].write_text(noanchor_stdout_text, encoding="utf-8")
    if not noanchor_paths["sol_path"].exists():
        raise RuntimeError(f"missing no-anchor solution: {noanchor_paths['sol_path']}")
    if not noanchor_paths["state_series_path"].exists():
        raise RuntimeError(f"missing no-anchor state series: {noanchor_paths['state_series_path']}")
    if not root_diag.exists():
        raise RuntimeError("missing DIAG.txt after no-anchor run")
    shutil.copy2(root_diag, noanchor_paths["diag_path"])

    noanchor_metrics = compute_full_metrics(
        case_name="no_anchor_largeP_smallQ",
        cfg_path=noanchor_paths["cfg_path"],
        cfg=noanchor_cfg,
        sol_path=noanchor_paths["sol_path"],
        state_series_path=noanchor_paths["state_series_path"],
        diag_path=noanchor_paths["diag_path"],
        stdout_text=noanchor_stdout_text,
        pos_path=pos_path,
    )
    noanchor_metrics["runtime_truth_anchor_pva"] = False

    metrics_df = pd.DataFrame([anchor_metrics, noanchor_metrics])
    metrics_df.to_csv(output_dir / "metrics.csv", index=False, encoding="utf-8-sig")

    summary_df = summarize_rows(metrics_df)
    write_summary(
        output_dir,
        exp_id=args.exp_id,
        anchor_config_path=anchor_config_path,
        nonanchor_config_path=noanchor_paths["cfg_path"],
        exe_path=exe_path,
        summary_df=summary_df,
    )

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "anchor_config": rel(anchor_config_path),
        "nonanchor_config": rel(noanchor_paths["cfg_path"]),
        "solver_exe": rel(exe_path),
        "pos_path": rel(pos_path),
        "output_dir": rel(output_dir),
        "metrics_csv": rel(output_dir / "metrics.csv"),
        "summary_md": rel(output_dir / "summary.md"),
        "reference_anchor_artifacts": {
            "sol_path": rel(anchor_sol_path),
            "state_series_path": rel(anchor_state_series_path),
            "diag_path": rel(anchor_diag_path),
            "stdout_path": rel(anchor_stdout_path),
        },
        "nonanchor_artifacts": {
            "sol_path": rel(noanchor_paths["sol_path"]),
            "state_series_path": rel(noanchor_paths["state_series_path"]),
            "diag_path": rel(noanchor_paths["diag_path"]),
            "stdout_path": rel(noanchor_paths["stdout_path"]),
        },
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel(output_dir))


if __name__ == "__main__":
    main()
