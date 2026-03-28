import argparse
import copy
import datetime as dt
import json
import math
import shutil
import sys
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

from scripts.analysis.odo_nhc_update_sweep import load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import (
    imu_truth_model_internal,
    parse_data2_readme_imu_params,
)
from scripts.analysis.run_data2_ins_gnss_state_sanity import evaluate_navigation_metrics
from scripts.analysis.run_data2_state_sanity_matrix import (
    STATE_META,
    base_p0_diag_from_config,
    build_truth_reference,
    default_ablation_flags,
    format_metric,
    get_group_vector_internal,
    json_safe,
    reset_directory,
    run_command,
)


EXP_ID_DEFAULT = "EXP-20260320-data2-ins-gnss-bias-zero-init-readme-gm-r2-pos320-mainline"
OUTPUT_DIR_DEFAULT = Path("output/data2_ins_gnss_bias_zero_init_readme_gm_pos320_mainline")
BASE_CONFIG_DEFAULT = Path("config_data2_baseline_eskf.yaml")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
README_PATH_DEFAULT = Path("dataset/data2/README.md")
IMU_MODEL_DEFAULT = "POS-320"
BIAS_P0_SIGMA_SCALE_DEFAULT = 6.0
CASE_ID = "zero_init_readme_gm_large_p0"
BIAS_STATE_ORDER = ["ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"]


def anchored_dynamic_std(sigma_internal: np.ndarray) -> np.ndarray:
    return np.asarray(0.1 * sigma_internal, dtype=float)


def system_markov_sigma_note(readme_params: dict[str, Any]) -> str:
    return (
        f"{readme_params['source']} {readme_params['imu_model']}: "
        f"ba sigma={float(readme_params['sigma_ba_mgal']):g} mGal, "
        f"bg sigma={float(readme_params['sigma_bg_degh']):g} deg/h, "
        f"corr_time={float(readme_params['corr_time_hr']):g} h"
    )


def large_zero_init_std(sigma_internal: np.ndarray, sigma_scale: float) -> np.ndarray:
    return np.asarray(float(sigma_scale) * sigma_internal, dtype=float)


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    readme_params: dict[str, Any],
    case_dir: Path,
    bias_p0_sigma_scale: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    p0_diag = list(base_p0_diag_from_config(base_cfg))

    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
    stdout_path = case_dir / f"{CASE_ID}.stdout.txt"
    diag_path = case_dir / f"DIAG_{CASE_ID}.txt"

    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["gnss_schedule"] = {"enabled": False}

    ablation_cfg = default_ablation_flags()
    ablation_cfg["disable_mounting"] = True
    ablation_cfg["disable_odo_lever_arm"] = True
    ablation_cfg["disable_odo_scale"] = True
    ablation_cfg["disable_gnss_lever_z"] = False
    fusion["ablation"] = ablation_cfg
    fusion["post_gnss_ablation"] = {"enabled": False, **default_ablation_flags()}

    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_diagnostics"] = False
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = False
    init_cfg["runtime_truth_anchor_gnss_only"] = False
    init_cfg["runtime_truth_anchor_position"] = False
    init_cfg["runtime_truth_anchor_velocity"] = False
    init_cfg["runtime_truth_anchor_attitude"] = False
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    sg_ref = get_group_vector_internal(truth_reference, "sg")
    sa_ref = get_group_vector_internal(truth_reference, "sa")
    gnss_lever_ref = get_group_vector_internal(truth_reference, "gnss_lever")
    readme_internal = imu_truth_model_internal(readme_params)
    ba_sigma = np.full(3, float(readme_internal["sigma_ba"]), dtype=float)
    bg_sigma = np.full(3, float(readme_internal["sigma_bg"]), dtype=float)
    sg_sigma = np.full(3, float(readme_internal["sigma_sg"]), dtype=float)
    sa_sigma = np.full(3, float(readme_internal["sigma_sa"]), dtype=float)

    ba_init = np.zeros(3, dtype=float)
    bg_init = np.zeros(3, dtype=float)
    ba_std = large_zero_init_std(ba_sigma, bias_p0_sigma_scale)
    bg_std = large_zero_init_std(bg_sigma, bias_p0_sigma_scale)
    sg_std = anchored_dynamic_std(sg_sigma)
    sa_std = anchored_dynamic_std(sa_sigma)
    gnss_lever_std = np.full(3, 0.01, dtype=float)
    gnss_lever_noise = np.full(3, 1.0e-6, dtype=float)

    init_cfg["ba0"] = [float(x) for x in ba_init]
    init_cfg["bg0"] = [float(x) for x in bg_init]
    init_cfg["sg0"] = [float(x) for x in sg_ref]
    init_cfg["sa0"] = [float(x) for x in sa_ref]
    init_cfg["gnss_lever_arm0"] = [float(x) for x in gnss_lever_ref]

    p0_diag[9:12] = [float(x * x) for x in ba_std]
    p0_diag[12:15] = [float(x * x) for x in bg_std]
    p0_diag[15:18] = [float(x * x) for x in sg_std]
    p0_diag[18:21] = [float(x * x) for x in sa_std]
    p0_diag[28:31] = [float(x * x) for x in gnss_lever_std]
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    noise_cfg["sigma_ba"] = float(np.max(ba_sigma))
    noise_cfg["sigma_bg"] = float(np.max(bg_sigma))
    noise_cfg["sigma_sg"] = float(np.max(sg_sigma))
    noise_cfg["sigma_sa"] = float(np.max(sa_sigma))
    noise_cfg["sigma_ba_vec"] = [float(x) for x in ba_sigma]
    noise_cfg["sigma_bg_vec"] = [float(x) for x in bg_sigma]
    noise_cfg["sigma_sg_vec"] = [float(x) for x in sg_sigma]
    noise_cfg["sigma_sa_vec"] = [float(x) for x in sa_sigma]
    noise_cfg["sigma_gnss_lever_arm"] = float(np.max(gnss_lever_noise))
    noise_cfg["sigma_gnss_lever_arm_vec"] = [float(x) for x in gnss_lever_noise]
    noise_cfg["markov_corr_time"] = float(readme_internal["corr_time_s"])

    metadata = {
        "case_id": CASE_ID,
        "description": "ba/bg zero init + README GM process noise + large prior std",
        "bias_init_mode": "zero",
        "bias_p0_sigma_scale": float(bias_p0_sigma_scale),
        "diag_output_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "ba_init_internal": [float(x) for x in ba_init],
        "bg_init_internal": [float(x) for x in bg_init],
        "ba_std_internal": [float(x) for x in ba_std],
        "bg_std_internal": [float(x) for x in bg_std],
        "ba_sigma_internal": [float(x) for x in ba_sigma],
        "bg_sigma_internal": [float(x) for x in bg_sigma],
        "sg_truth_internal": [float(x) for x in sg_ref],
        "sa_truth_internal": [float(x) for x in sa_ref],
        "gnss_lever_truth_internal_m": [float(x) for x in gnss_lever_ref],
        "imu_model": readme_params["imu_model"],
        "readme_path": readme_params["source"],
        "process_noise_reference": system_markov_sigma_note(readme_params),
    }
    return cfg, metadata


def run_case(cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{CASE_ID}.txt"
    state_series_path = case_dir / f"state_series_{CASE_ID}.csv"
    stdout_path = case_dir / f"{CASE_ID}.stdout.txt"
    diag_path = case_dir / f"DIAG_{CASE_ID}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output: {state_series_path}")
    if root_diag.exists():
        shutil.copy2(root_diag, diag_path)
    row: dict[str, Any] = {
        "case_id": CASE_ID,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT) if diag_path.exists() else "",
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "config_mtime": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(evaluate_navigation_metrics(cfg_path, sol_path))
    return row


def evaluate_bias_states(case_row: dict[str, Any]) -> pd.DataFrame:
    state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    usecols = ["timestamp"] + [STATE_META[state_name]["csv"] for state_name in BIAS_STATE_ORDER]
    state_df = pd.read_csv(state_path, usecols=usecols)
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    elapsed_h = (timestamps - float(timestamps[0])) / 3600.0

    rows: list[dict[str, Any]] = []
    for state_name in BIAS_STATE_ORDER:
        column = STATE_META[state_name]["csv"]
        values = state_df[column].to_numpy(dtype=float)
        slope_per_h = math.nan
        if values.size >= 2 and np.unique(elapsed_h).size >= 2:
            slope_per_h = float(np.polyfit(elapsed_h, values, 1)[0])
        rows.append(
            {
                "case_id": CASE_ID,
                "state_name": state_name,
                "unit": STATE_META[state_name]["unit"],
                "start_value": float(values[0]),
                "end_value": float(values[-1]),
                "mean_value": float(np.mean(values)),
                "std_value": float(np.std(values, ddof=0)),
                "min_value": float(np.min(values)),
                "max_value": float(np.max(values)),
                "range_value": float(np.max(values) - np.min(values)),
                "head_to_tail_delta": float(values[-1] - values[0]),
                "trend_slope_per_h": slope_per_h,
            }
        )
    return pd.DataFrame(rows)


def downsample_for_plot(x: np.ndarray, y: np.ndarray, max_points: int = 4000) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= max_points:
        return x, y
    idx = np.linspace(0, x.size - 1, max_points, dtype=int)
    return x[idx], y[idx]


def plot_estimated_bias(case_row: dict[str, Any], output_path: Path, title_note: str) -> None:
    state_df = pd.read_csv((REPO_ROOT / case_row["state_series_path"]).resolve())
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    t0 = float(timestamps[0])
    elapsed_min = (timestamps - t0) / 60.0

    for idx, state_name in enumerate(BIAS_STATE_ORDER):
        ax = axes[idx]
        column = STATE_META[state_name]["csv"]
        values = state_df[column].to_numpy(dtype=float)
        plot_x, plot_y = downsample_for_plot(elapsed_min, values)
        ax.plot(plot_x, plot_y, linewidth=1.15, color="#1f77b4")
        ax.axhline(0.0, linestyle="--", linewidth=0.85, color="black", alpha=0.55)
        ax.set_title(state_name)
        ax.set_ylabel(STATE_META[state_name]["unit"])
        ax.grid(alpha=0.25)

    axes[-2].set_xlabel("elapsed time [min]")
    axes[-1].set_xlabel("elapsed time [min]")
    fig.suptitle(title_note)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_summary(
    output_path: Path,
    case_row: dict[str, Any],
    bias_df: pd.DataFrame,
    readme_params: dict[str, Any],
    bias_p0_sigma_scale: float,
    manifest: dict[str, Any],
) -> None:
    lines: list[str] = [
        "# data2 pure INS/GNSS bias zero-init README-GM plot summary",
        "",
        "## 1. 实验设置",
        "- pipeline: `pure INS/GNSS`, 显式关闭 `ODO/NHC`。",
        "- fixed-truth states: `gnss_lever(28:30)` 固定真值；`sg(15:17), sa(18:20)` 继续使用真值建模。",
        "- probed states: `ba(9:11), bg(12:14)` 从 `0` 初值开始估计。",
        (
            f"- README-based GM process model: `{system_markov_sigma_note(readme_params)}`；"
            "本次只用 README 参数设置 `Q/T`，不再用 bias truth 参与 `ba/bg` 初值或图像对比。"
        ),
        (
            f"- large prior std: `P0 std = {bias_p0_sigma_scale:g} x sigma_readme`，"
            f"即 `ba std = {bias_p0_sigma_scale * float(readme_params['sigma_ba_mgal']):g} mGal`，"
            f"`bg std = {bias_p0_sigma_scale * float(readme_params['sigma_bg_degh']):g} deg/h`。"
        ),
        "- plotting policy: 仅绘制该单方案的估计曲线与零线，不与其他方案做对比。",
        "",
        "## 2. Navigation",
        (
            f"- `{case_row['case_id']}`: `nav_rmse_3d_m={format_metric(float(case_row['nav_rmse_3d_m']))}`, "
            f"`nav_final_err_3d_m={format_metric(float(case_row['nav_final_err_3d_m']))}`。"
        ),
        "",
        "## 3. Bias state series snapshot",
    ]

    for _, row in bias_df.iterrows():
        lines.append(
            f"- `{row['state_name']}`: "
            f"`start={format_metric(float(row['start_value']))} {row['unit']}`, "
            f"`end={format_metric(float(row['end_value']))} {row['unit']}`, "
            f"`range={format_metric(float(row['range_value']))} {row['unit']}`, "
            f"`slope={format_metric(float(row['trend_slope_per_h']))} {row['unit']}/h`。"
        )

    lines.extend(
        [
            "",
            "## 4. Artifacts",
            f"- summary: `{manifest['summary_path']}`",
            f"- case metrics: `{manifest['case_metrics_csv']}`",
            f"- bias state metrics: `{manifest['bias_state_metrics_csv']}`",
            f"- config: `{manifest['config_path']}`",
            f"- plot: `{manifest['estimated_plot']}`",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a single pure INS/GNSS bias probe with README-based GM noise and zero-init large prior."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--readme-path", type=Path, default=README_PATH_DEFAULT)
    parser.add_argument("--imu-model", type=str, default=IMU_MODEL_DEFAULT)
    parser.add_argument("--bias-p0-sigma-scale", type=float, default=BIAS_P0_SIGMA_SCALE_DEFAULT)
    parser.add_argument("--exp-id", type=str, default=EXP_ID_DEFAULT)
    args = parser.parse_args()

    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    readme_params = parse_data2_readme_imu_params(args.readme_path, args.imu_model)
    reset_directory(args.output_dir)

    case_dir = args.output_dir / "artifacts" / "cases" / CASE_ID
    case_dir.mkdir(parents=True, exist_ok=True)

    cfg, metadata = build_case_config(
        base_cfg=base_cfg,
        truth_reference=truth_reference,
        readme_params=readme_params,
        case_dir=case_dir,
        bias_p0_sigma_scale=float(args.bias_p0_sigma_scale),
    )
    cfg_path = case_dir / f"config_{CASE_ID}.yaml"
    save_yaml(cfg, cfg_path)

    case_row = run_case(cfg_path, case_dir, args.exe)
    case_row.update(metadata)
    bias_df = evaluate_bias_states(case_row)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    bias_metrics_path = args.output_dir / "bias_state_metrics.csv"
    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"
    plot_path = args.output_dir / "plots" / "bias_estimated_only.png"

    pd.DataFrame([case_row]).to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    bias_df.to_csv(bias_metrics_path, index=False, encoding="utf-8-sig")

    plot_estimated_bias(
        case_row=case_row,
        output_path=plot_path,
        title_note=(
            f"bias estimate only | zero-init | README GM ({args.imu_model}) | "
            f"P0={float(args.bias_p0_sigma_scale):g}sigma"
        ),
    )

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config.resolve(), REPO_ROOT),
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "exe_path": rel_from_root(args.exe.resolve(), REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "bias_state_metrics_csv": rel_from_root(bias_metrics_path, REPO_ROOT),
        "estimated_plot": rel_from_root(plot_path, REPO_ROOT),
        "summary_path": rel_from_root(summary_path, REPO_ROOT),
        "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        "readme_imu_params": readme_params,
        "bias_p0_sigma_scale": float(args.bias_p0_sigma_scale),
        "system_bias_q_note": system_markov_sigma_note(readme_params),
        "case": json_safe(case_row),
    }
    write_summary(
        output_path=summary_path,
        case_row=case_row,
        bias_df=bias_df,
        readme_params=readme_params,
        bias_p0_sigma_scale=float(args.bias_p0_sigma_scale),
        manifest=manifest,
    )
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"[done] wrote summary: {summary_path}")
    print(f"[done] wrote case metrics: {case_metrics_path}")
    print(f"[done] wrote bias state metrics: {bias_metrics_path}")
    print(f"[done] wrote plot: {plot_path}")


if __name__ == "__main__":
    main()
