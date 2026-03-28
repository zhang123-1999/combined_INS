from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.run_mounting_yaw_sigma_sweep import (
    compute_case_metrics,
    ensure_dir,
    mtime_text,
    rel,
    render_table,
    run_solver,
    save_yaml,
)


EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-yaw-prior-sweep-r1"
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
BASE_CONFIG_DEFAULT = Path(
    "output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r3_axis_hard_freeze/"
    "artifacts/cases/release_mounting_yaw/config_release_mounting_yaw.yaml"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_prior_sweep_20260322")
TARGET_TOTAL_YAW_DEG_DEFAULT = 0.84
STD_LIST_DEFAULT = [0.3, 1.0, 3.0, 5.0]
SIGMA_LIST_DEFAULT = [1.0e-6, 0.0]
P0_INDEX_MOUNTING_YAW = 24


def parse_float_list(text: str) -> list[float]:
    values: list[float] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    if not values:
        raise argparse.ArgumentTypeError("list must not be empty")
    return values


def case_name(std_mounting_yaw_deg: float, sigma_mounting_yaw: float) -> str:
    std_text = f"{std_mounting_yaw_deg:.1f}".replace(".", "p")
    if sigma_mounting_yaw == 0.0:
        sigma_text = "0"
    else:
        sigma_text = f"{sigma_mounting_yaw:.0e}".replace("+", "")
    return f"std_{std_text}deg_sigma_{sigma_text}"


def build_case_config(
    base_cfg: dict[str, Any],
    *,
    target_total_yaw_deg: float,
    std_mounting_yaw_deg: float,
    sigma_mounting_yaw: float,
    case_dir: Path,
) -> tuple[dict[str, Any], dict[str, Path]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg["fusion"]
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    init_cfg = fusion.setdefault("init", {})
    case_id = case_name(std_mounting_yaw_deg, sigma_mounting_yaw)

    constraints_mount = list(constraints_cfg.get("imu_mounting_angle", [0.0, 0.0, 0.0]))
    constraints_mount[2] = float(target_total_yaw_deg)
    constraints_cfg["imu_mounting_angle"] = constraints_mount

    init_cfg["mounting_yaw0"] = 0.0
    init_cfg["std_mounting_yaw"] = float(std_mounting_yaw_deg)
    p0_diag = [float(x) for x in init_cfg["P0_diag"]]
    p0_diag[P0_INDEX_MOUNTING_YAW] = float(np.deg2rad(std_mounting_yaw_deg) ** 2)
    init_cfg["P0_diag"] = p0_diag

    noise_cfg["sigma_mounting_yaw"] = float(sigma_mounting_yaw)

    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    cfg_path = case_dir / f"config_{case_id}.yaml"
    stdout_path = case_dir / f"solver_stdout_{case_id}.txt"
    diag_path = case_dir / f"DIAG_{case_id}.txt"
    metrics_json_path = case_dir / f"metrics_{case_id}.json"

    fusion["output_path"] = rel(sol_path)
    fusion["state_series_output_path"] = rel(state_series_path)

    return cfg, {
        "cfg_path": cfg_path,
        "sol_path": sol_path,
        "state_series_path": state_series_path,
        "stdout_path": stdout_path,
        "diag_path": diag_path,
        "metrics_json_path": metrics_json_path,
    }


def enrich_metrics(
    metrics: dict[str, Any],
    *,
    target_total_yaw_deg: float,
    std_mounting_yaw_deg: float,
) -> dict[str, Any]:
    final_total = float(metrics["final_total_mounting_yaw_deg"])
    after_200_mid = 0.5 * (float(metrics["after_200_p05_deg"]) + float(metrics["after_200_p95_deg"]))
    final_error = final_total - target_total_yaw_deg
    steady_center_error = after_200_mid - target_total_yaw_deg
    combined_score = abs(steady_center_error) + float(metrics["after_200_peak_to_peak_deg"])
    enriched = dict(metrics)
    enriched["target_total_yaw_deg"] = float(target_total_yaw_deg)
    enriched["std_mounting_yaw_deg"] = float(std_mounting_yaw_deg)
    enriched["final_error_deg"] = float(final_error)
    enriched["steady_center_deg"] = float(after_200_mid)
    enriched["steady_center_error_deg"] = float(steady_center_error)
    enriched["combined_score_deg"] = float(combined_score)
    return enriched


def write_summary(
    output_dir: Path,
    *,
    exp_id: str,
    base_config_path: Path,
    exe_path: Path,
    target_total_yaw_deg: float,
    metrics_df: pd.DataFrame,
    best_case: pd.Series,
) -> None:
    lines = [
        "# mounting_yaw prior sweep",
        "",
        f"Date: `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{exp_id}`",
        f"- target total yaw truth: `{target_total_yaw_deg:.6f} deg`",
        f"- base config: `{rel(base_config_path)}`",
        f"- solver: `{rel(exe_path)}`",
        f"- output dir: `{rel(output_dir)}`",
        "",
        "## Sweep metrics",
        "",
    ]
    table_columns = [
        "case_name",
        "std_mounting_yaw_deg",
        "sigma_mounting_yaw_rad_sqrtHz",
        "steady_center_deg",
        "steady_center_error_deg",
        "final_total_mounting_yaw_deg",
        "final_error_deg",
        "overall_std_deg",
        "after_200_peak_to_peak_deg",
        "turn_neg_minus_turn_pos_deg",
        "combined_score_deg",
    ]
    lines.extend(render_table(metrics_df[table_columns], table_columns))
    lines.extend(
        [
            "",
            "## Recommended case",
            "",
            (
                f"- `{best_case['case_name']}`: "
                f"`std_mounting_yaw={best_case['std_mounting_yaw_deg']:.6f} deg`, "
                f"`sigma_mounting_yaw={best_case['sigma_mounting_yaw_rad_sqrtHz']:.6e} rad/sqrtHz`, "
                f"`steady_center={best_case['steady_center_deg']:.6f} deg`, "
                f"`steady_center_error={best_case['steady_center_error_deg']:.6f} deg`, "
                f"`after_200_peak_to_peak={best_case['after_200_peak_to_peak_deg']:.6f} deg`, "
                f"`overall_std={best_case['overall_std_deg']:.6f} deg`, "
                f"`final_total_yaw={best_case['final_total_mounting_yaw_deg']:.6f} deg`."
            ),
            "",
            "## Assessment",
            "",
            "- 本轮目标是找一个适合后续研究的工作配置：把 `0.84 deg` 当 nominal truth，用较大初始不确定度保证可调整，再用较小过程噪声压稳态波动。",
            "- `combined_score_deg = |steady_center_error| + after_200_peak_to_peak` 只用于在“误差小”和“波动小”之间做简单排序；最终解释仍以原始指标为准。",
            "",
            "## Freshness",
            "",
            f"- base config mtime: `{mtime_text(base_config_path)}`",
            f"- solver mtime: `{mtime_text(exe_path)}`",
            f"- metrics csv: `{rel(output_dir / 'metrics.csv')}`",
            f"- summary md: `{rel(output_dir / 'summary.md')}`",
            f"- manifest: `{rel(output_dir / 'manifest.json')}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep large yaw initial std + small process noise around nominal total yaw 0.84 deg.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--target-total-yaw-deg", type=float, default=TARGET_TOTAL_YAW_DEG_DEFAULT)
    parser.add_argument("--std-list", type=parse_float_list, default=STD_LIST_DEFAULT, help="deg")
    parser.add_argument("--sigma-list", type=parse_float_list, default=SIGMA_LIST_DEFAULT, help="rad/sqrtHz")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    exe_path = (REPO_ROOT / args.exe).resolve()
    base_config_path = (REPO_ROOT / args.base_config).resolve()
    pos_path = (REPO_ROOT / args.pos).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()

    if not exe_path.exists():
        raise FileNotFoundError(f"missing solver executable: {exe_path}")
    if not base_config_path.exists():
        raise FileNotFoundError(f"missing base config: {base_config_path}")
    if not pos_path.exists():
        raise FileNotFoundError(f"missing POS file: {pos_path}")

    ensure_dir(output_dir)
    base_cfg = yaml.safe_load(base_config_path.read_text(encoding="utf-8"))
    root_diag = REPO_ROOT / "DIAG.txt"

    metrics_rows: list[dict[str, Any]] = []
    case_artifacts: list[dict[str, str]] = []

    for std_mounting_yaw_deg in args.std_list:
        for sigma_mounting_yaw in args.sigma_list:
            current_case_name = case_name(std_mounting_yaw_deg, sigma_mounting_yaw)
            case_dir = output_dir / "cases" / current_case_name
            ensure_dir(case_dir)
            cfg, paths = build_case_config(
                base_cfg,
                target_total_yaw_deg=float(args.target_total_yaw_deg),
                std_mounting_yaw_deg=float(std_mounting_yaw_deg),
                sigma_mounting_yaw=float(sigma_mounting_yaw),
                case_dir=case_dir,
            )
            save_yaml(cfg, paths["cfg_path"])

            if root_diag.exists():
                root_diag.unlink()
            stdout_text = run_solver(exe_path, paths["cfg_path"])
            paths["stdout_path"].write_text(stdout_text, encoding="utf-8")

            if not paths["sol_path"].exists():
                raise RuntimeError(f"missing solution output for {current_case_name}: {paths['sol_path']}")
            if not paths["state_series_path"].exists():
                raise RuntimeError(f"missing state series output for {current_case_name}: {paths['state_series_path']}")
            if not root_diag.exists():
                raise RuntimeError(f"missing DIAG.txt after case {current_case_name}")
            shutil.copy2(root_diag, paths["diag_path"])

            base_metrics, _, _ = compute_case_metrics(
                case_name=current_case_name,
                sigma_mounting_yaw=float(sigma_mounting_yaw),
                cfg=cfg,
                state_series_path=paths["state_series_path"],
                diag_path=paths["diag_path"],
                pos_path=pos_path,
                stdout_text=stdout_text,
            )
            metrics = enrich_metrics(
                base_metrics,
                target_total_yaw_deg=float(args.target_total_yaw_deg),
                std_mounting_yaw_deg=float(std_mounting_yaw_deg),
            )
            paths["metrics_json_path"].write_text(json.dumps(metrics, indent=2), encoding="utf-8")
            metrics_rows.append(metrics)
            case_artifacts.append(
                {
                    "case_name": current_case_name,
                    "config_path": rel(paths["cfg_path"]),
                    "sol_path": rel(paths["sol_path"]),
                    "state_series_path": rel(paths["state_series_path"]),
                    "diag_path": rel(paths["diag_path"]),
                    "stdout_path": rel(paths["stdout_path"]),
                    "metrics_json_path": rel(paths["metrics_json_path"]),
                }
            )

    metrics_df = pd.DataFrame(metrics_rows).sort_values(
        by=["combined_score_deg", "steady_center_error_deg", "after_200_peak_to_peak_deg"],
        key=lambda s: np.abs(s) if s.name == "steady_center_error_deg" else s,
    ).reset_index(drop=True)
    metrics_df.to_csv(output_dir / "metrics.csv", index=False, encoding="utf-8-sig")

    best_case = metrics_df.iloc[0]
    recommended_case_dir = output_dir / "cases" / str(best_case["case_name"])
    recommended_cfg_path = recommended_case_dir / f"config_{best_case['case_name']}.yaml"
    recommended_alias_path = output_dir / "config_recommended_mounting_yaw_prior.yaml"
    shutil.copy2(recommended_cfg_path, recommended_alias_path)

    write_summary(
        output_dir,
        exp_id=args.exp_id,
        base_config_path=base_config_path,
        exe_path=exe_path,
        target_total_yaw_deg=float(args.target_total_yaw_deg),
        metrics_df=metrics_df,
        best_case=best_case,
    )

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "base_config": rel(base_config_path),
        "solver_exe": rel(exe_path),
        "pos_path": rel(pos_path),
        "output_dir": rel(output_dir),
        "target_total_yaw_deg": float(args.target_total_yaw_deg),
        "std_list_deg": [float(x) for x in args.std_list],
        "sigma_list_rad_sqrtHz": [float(x) for x in args.sigma_list],
        "metrics_csv": rel(output_dir / "metrics.csv"),
        "summary_md": rel(output_dir / "summary.md"),
        "recommended_case_name": str(best_case["case_name"]),
        "recommended_config": rel(recommended_alias_path),
        "case_artifacts": case_artifacts,
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel(output_dir))


if __name__ == "__main__":
    main()
