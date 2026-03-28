from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml  # noqa: E402
from scripts.analysis.run_data2_baseline_full_ins_gnss import (  # noqa: E402
    BASE_CONFIG_DEFAULT,
    SOLVER_DEFAULT,
    build_run_config as build_full_ins_gnss_config,
    normalize_repo_path,
    run_case as run_full_ins_gnss_case,
)
from scripts.analysis.run_data2_fullwindow_attitude_bias_coupling import (  # noqa: E402
    ALL_STATE_SPECS,
    GROUP_SPECS,
    GroupSpec,
    KEY_COUPLING_STATES,
    build_state_frame,
    compute_case_metrics,
    format_metric,
    render_table,
)
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import mtime_text  # noqa: E402
from scripts.analysis.run_data2_staged_g5_no_imu_scale import (  # noqa: E402
    PVA_ERROR_GROUP_SPECS,
    compute_phase_metrics,
    plot_state_grid,
)
from scripts.analysis.run_data2_state_sanity_matrix import (  # noqa: E402
    build_truth_reference,
    json_safe,
    reset_directory,
)
from scripts.analysis.run_nhc_state_convergence_research import (  # noqa: E402
    build_truth_interp,
    load_pos_dataframe,
    merge_case_outputs,
)


EXP_ID_DEFAULT = "EXP-20260327-data2-full-ins-gnss-bias-scale-process-sweep-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_full_ins_gnss_bias_scale_process_sweep")
Q_SCALES_DEFAULT = (1.0, 2.0, 4.0, 8.0)
CASE_ID_PREFIX = "data2_full_ins_gnss_bias_scale_q"
BIAS_SCALE_GROUP_IDS = ("ba", "bg", "sg", "sa")
PVA_GROUP_IDS = ("position", "velocity", "attitude")
BIAS_SCALE_TRUTH_KEYS_TO_HIDE = {
    "ba_x_mgal",
    "ba_y_mgal",
    "ba_z_mgal",
    "bg_x_degh",
    "bg_y_degh",
    "bg_z_degh",
    "sg_x_ppm",
    "sg_y_ppm",
    "sg_z_ppm",
    "sa_x_ppm",
    "sa_y_ppm",
    "sa_z_ppm",
}


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    label: str
    color: str
    process_noise_scale_rel: float


@dataclass(frozen=True)
class PlotCase:
    case_id: str
    label: str
    color: str


@dataclass(frozen=True)
class BiasScalePlotConfig:
    overview_states: tuple[Any, ...]
    group_specs: tuple[GroupSpec, ...]
    truth_keys_to_hide: set[str]


def format_scale_slug(scale: float) -> str:
    scale = float(scale)
    if scale <= 0.0:
        raise ValueError("process-noise scale must be positive")
    if float(scale).is_integer():
        return f"{int(scale)}x"
    return f"{scale:g}".replace(".", "p") + "x"


def build_case_specs(q_scales: Sequence[float] | None = None) -> list[CaseSpec]:
    q_scales = Q_SCALES_DEFAULT if q_scales is None else tuple(float(value) for value in q_scales)
    colors = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#8c564b", "#17becf")
    specs: list[CaseSpec] = []
    seen: set[str] = set()
    for idx, scale in enumerate(q_scales):
        slug = format_scale_slug(scale)
        case_id = f"{CASE_ID_PREFIX}{slug}"
        if case_id in seen:
            raise ValueError(f"duplicate process-noise case slug: {case_id}")
        specs.append(
            CaseSpec(
                case_id=case_id,
                label=f"full INS/GNSS bias-scale Q {slug}",
                color=colors[idx % len(colors)],
                process_noise_scale_rel=float(scale),
            )
        )
        seen.add(case_id)
    return specs


def build_case_config(
    base_cfg: dict[str, Any],
    output_dir: Path,
    spec: CaseSpec,
    fix_gnss_lever_truth: bool = False,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg, metadata = build_full_ins_gnss_config(
        base_cfg,
        output_dir,
        case_id=spec.case_id,
        case_label=spec.label,
        process_noise_scale_rel=spec.process_noise_scale_rel,
        fix_gnss_lever_truth=fix_gnss_lever_truth,
    )
    ablation = cfg.setdefault("fusion", {}).setdefault("ablation", {})
    ablation["disable_gyro_scale"] = False
    ablation["disable_accel_scale"] = False
    metadata["disable_gyro_scale"] = False
    metadata["disable_accel_scale"] = False
    return cfg, metadata


def build_bias_scale_plot_config() -> BiasScalePlotConfig:
    group_specs = tuple(group for group in GROUP_SPECS if group.group_id in BIAS_SCALE_GROUP_IDS)
    overview_states = tuple(state for group in group_specs for state in group.states)
    return BiasScalePlotConfig(
        overview_states=overview_states,
        group_specs=group_specs,
        truth_keys_to_hide=set(BIAS_SCALE_TRUTH_KEYS_TO_HIDE),
    )


def build_all_state_plot_modes() -> dict[str, str]:
    plot_modes: dict[str, str] = {}
    for group in PVA_ERROR_GROUP_SPECS:
        for state in group.states:
            plot_modes[state.key] = "error"
    return plot_modes


def compute_series_oscillation_metrics(
    values: Sequence[float],
    significant_step_rel: float = 0.02,
    denominator_span_floor_rel: float = 0.05,
) -> dict[str, float]:
    series = np.asarray(values, dtype=float)
    if series.ndim != 1:
        raise ValueError("values must be a 1-D sequence")
    if series.size == 0:
        raise ValueError("values must contain at least one sample")
    if series.size == 1:
        return {
            "samples": 1.0,
            "start_value": float(series[0]),
            "end_value": float(series[0]),
            "net_change": 0.0,
            "net_change_abs": 0.0,
            "span": 0.0,
            "total_variation": 0.0,
            "significant_step_threshold": 0.0,
            "significant_step_count": 0.0,
            "sign_flip_count": 0.0,
            "oscillation_index": 0.0,
            "max_abs_value": float(np.max(np.abs(series))),
        }

    diffs = np.diff(series)
    total_variation = float(np.sum(np.abs(diffs)))
    net_change = float(series[-1] - series[0])
    net_change_abs = abs(net_change)
    span = float(np.max(series) - np.min(series))
    significant_step_threshold = float(max(span * significant_step_rel, 1.0e-12))
    significant_diffs = diffs[np.abs(diffs) > significant_step_threshold]
    sign_flip_count = 0
    if significant_diffs.size >= 2:
        signs = np.sign(significant_diffs)
        sign_flip_count = int(np.count_nonzero(signs[1:] * signs[:-1] < 0.0))
    denominator = max(net_change_abs, span * denominator_span_floor_rel, 1.0e-12)
    return {
        "samples": float(series.size),
        "start_value": float(series[0]),
        "end_value": float(series[-1]),
        "net_change": net_change,
        "net_change_abs": net_change_abs,
        "span": span,
        "total_variation": total_variation,
        "significant_step_threshold": significant_step_threshold,
        "significant_step_count": float(significant_diffs.size),
        "sign_flip_count": float(sign_flip_count),
        "oscillation_index": float(total_variation / denominator) if total_variation > 0.0 else 0.0,
        "max_abs_value": float(np.max(np.abs(series))),
    }


def state_error_series(state_frame: pd.DataFrame, state_spec: Any) -> np.ndarray:
    return (
        state_frame[state_spec.key].to_numpy(dtype=float)
        - state_frame[state_spec.truth_column].to_numpy(dtype=float)
    )


def compute_bias_scale_oscillation_rows(
    state_frame: pd.DataFrame,
    spec: CaseSpec,
) -> pd.DataFrame:
    plot_config = build_bias_scale_plot_config()
    rows: list[dict[str, Any]] = []
    for group in plot_config.group_specs:
        for state_spec in group.states:
            error_series = state_error_series(state_frame, state_spec)
            metrics = compute_series_oscillation_metrics(error_series)
            rows.append(
                {
                    "case_id": spec.case_id,
                    "case_label": spec.label,
                    "process_noise_scale_rel": spec.process_noise_scale_rel,
                    "state_group": group.group_id,
                    "state_key": state_spec.key,
                    "state_label": state_spec.label,
                    "unit": state_spec.unit,
                    "rmse_error": float(np.sqrt(np.mean(error_series * error_series))),
                    **metrics,
                }
            )
    return pd.DataFrame(rows)


def summarize_case_oscillation(oscillation_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for case_id, case_df in oscillation_df.groupby("case_id", sort=False):
        dominant_row = case_df.sort_values(
            ["oscillation_index", "sign_flip_count", "total_variation"],
            ascending=[False, False, False],
        ).iloc[0]
        rows.append(
            {
                "case_id": case_id,
                "case_label": str(case_df["case_label"].iloc[0]),
                "process_noise_scale_rel": float(case_df["process_noise_scale_rel"].iloc[0]),
                "states_evaluated": int(case_df.shape[0]),
                "mean_oscillation_index": float(case_df["oscillation_index"].mean()),
                "median_oscillation_index": float(case_df["oscillation_index"].median()),
                "max_oscillation_index": float(case_df["oscillation_index"].max()),
                "mean_sign_flip_count": float(case_df["sign_flip_count"].mean()),
                "max_sign_flip_count": float(case_df["sign_flip_count"].max()),
                "max_state_rmse_error": float(case_df["rmse_error"].max()),
                "dominant_state_key": str(dominant_row["state_key"]),
                "dominant_state_group": str(dominant_row["state_group"]),
            }
        )
    return pd.DataFrame(rows).sort_values("process_noise_scale_rel").reset_index(drop=True)


def plot_oscillation_summary(summary_df: pd.DataFrame, output_path: Path) -> None:
    if summary_df.empty:
        return
    labels = [format_scale_slug(scale) for scale in summary_df["process_noise_scale_rel"].tolist()]
    x = np.arange(len(labels), dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].bar(x, summary_df["mean_oscillation_index"].to_numpy(dtype=float), color="#4c78a8")
    axes[0].set_ylabel("mean osc idx")
    axes[0].grid(alpha=0.2)

    axes[1].bar(x, summary_df["max_oscillation_index"].to_numpy(dtype=float), color="#f58518")
    axes[1].set_ylabel("max osc idx")
    axes[1].grid(alpha=0.2)

    axes[2].bar(x, summary_df["max_sign_flip_count"].to_numpy(dtype=float), color="#54a24b")
    axes[2].set_ylabel("max sign flips")
    axes[2].set_xlabel("bias/scale process noise scale")
    axes[2].grid(alpha=0.2)
    axes[2].set_xticks(x, labels)

    fig.suptitle("Full INS/GNSS bias-scale process-noise oscillation summary", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    oscillation_summary_df: pd.DataFrame,
    plot_paths: dict[str, str],
) -> None:
    case_rows = [
        [
            str(row["case_id"]),
            format_metric(row["process_noise_scale_rel"]),
            format_metric(row.get("overall_rmse_3d_m_aux")),
            format_metric(row.get("overall_p95_3d_m_aux")),
            format_metric(row.get("overall_final_err_3d_m_aux")),
            format_metric(row.get("yaw_err_max_abs_deg")),
            format_metric(row.get("bg_z_degh_err_max_abs")),
        ]
        for _, row in case_metrics_df.iterrows()
    ]
    osc_rows = [
        [
            str(row["case_id"]),
            format_metric(row["process_noise_scale_rel"]),
            format_metric(row["mean_oscillation_index"]),
            format_metric(row["max_oscillation_index"]),
            format_metric(row["max_sign_flip_count"]),
            str(row["dominant_state_key"]),
        ]
        for _, row in oscillation_summary_df.iterrows()
    ]

    lines = [
        "# data2 full INS/GNSS bias-scale process-noise sweep",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- q_scales: `{manifest['q_scales']}`",
        f"- fix_gnss_lever_truth: `{manifest['fix_gnss_lever_truth']}`",
        f"- full_window: `{manifest['full_window']}`",
        f"- generated_at: `{manifest['generated_at']}`",
        "",
        "## Case Metrics",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "q_scale",
                "rmse3d_m",
                "p95_3d_m",
                "final_3d_m",
                "yaw_abs_max_deg",
                "bg_z_abs_max_degh",
            ],
            case_rows,
        )
    )
    lines.extend(["", "## Oscillation Summary"])
    lines.extend(
        render_table(
            [
                "case_id",
                "q_scale",
                "mean_osc_idx",
                "max_osc_idx",
                "max_sign_flips",
                "dominant_state",
            ],
            osc_rows,
        )
    )
    if plot_paths:
        lines.extend(["", "## Plot Outputs"])
        for key, path in plot_paths.items():
            lines.append(f"- `{key}`: `{path}`")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a full INS/GNSS sweep that inflates ba/bg/sg/sa process noise to check "
            "whether bias and scale-factor states become oscillatory."
        )
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--q-scales", nargs="*", type=float, default=list(Q_SCALES_DEFAULT))
    parser.add_argument("--fix-gnss-lever-truth", action="store_true")
    args = parser.parse_args()
    args.base_config = normalize_repo_path(args.base_config)
    args.exe = normalize_repo_path(args.exe)
    args.output_dir = normalize_repo_path(args.output_dir)
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")

    case_specs = build_case_specs(args.q_scales)
    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_df = load_pos_dataframe((REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve())

    plot_cases: list[PlotCase] = []
    case_frames: dict[str, pd.DataFrame] = {}
    case_rows: list[dict[str, Any]] = []
    phase_metric_frames: list[pd.DataFrame] = []
    oscillation_frames: list[pd.DataFrame] = []
    case_freshness: dict[str, dict[str, Any]] = {}

    for spec in case_specs:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)

        cfg, metadata = build_case_config(
            base_cfg,
            args.output_dir,
            spec,
            fix_gnss_lever_truth=args.fix_gnss_lever_truth,
        )
        cfg_path = case_dir / f"config_{spec.case_id}.yaml"
        save_yaml(cfg, cfg_path)

        case_row = run_full_ins_gnss_case(
            cfg_path,
            args.output_dir,
            case_dir,
            args.exe,
            spec.case_id,
            spec.label,
        )
        sol_path = args.output_dir / f"SOL_{spec.case_id}.txt"
        state_series_path = args.output_dir / f"state_series_{spec.case_id}.csv"
        merged_df = merge_case_outputs(sol_path, state_series_path)

        truth_interp_df = build_truth_interp(merged_df["timestamp"].to_numpy(dtype=float), truth_df)
        state_frame = build_state_frame(merged_df, truth_interp_df, truth_reference)
        all_states_path = case_dir / f"all_states_{spec.case_id}.csv"
        state_frame.to_csv(all_states_path, index=False, encoding="utf-8-sig")

        metrics_row = compute_case_metrics(case_row, state_frame)
        metrics_row["process_noise_scale_rel"] = spec.process_noise_scale_rel
        metrics_row["config_path"] = rel_from_root(cfg_path, REPO_ROOT)
        metrics_row["config_mtime"] = mtime_text(cfg_path)
        metrics_row["all_states_path"] = rel_from_root(all_states_path, REPO_ROOT)
        metrics_row["all_states_mtime"] = mtime_text(all_states_path)
        case_rows.append(metrics_row)

        phase_metrics_df = compute_phase_metrics(
            state_frame,
            spec.case_id,
            [("full_window_ins_gnss", *metadata["full_window"])],
            [],
        )
        phase_metric_frames.append(phase_metrics_df)

        oscillation_df = compute_bias_scale_oscillation_rows(state_frame, spec)
        oscillation_frames.append(oscillation_df)

        case_freshness[spec.case_id] = {
            "case_config_mtime": mtime_text(cfg_path),
            "sol_mtime": mtime_text(sol_path),
            "state_series_mtime": mtime_text(state_series_path),
            "all_states_mtime": mtime_text(all_states_path),
        }
        case_frames[spec.case_id] = state_frame
        plot_cases.append(PlotCase(case_id=spec.case_id, label=spec.label, color=spec.color))

    case_metrics_df = pd.DataFrame(case_rows).sort_values("process_noise_scale_rel").reset_index(drop=True)
    phase_metrics_df = pd.concat(phase_metric_frames, ignore_index=True)
    oscillation_metrics_df = pd.concat(oscillation_frames, ignore_index=True)
    oscillation_summary_df = summarize_case_oscillation(oscillation_metrics_df)

    case_metrics_df = case_metrics_df.merge(
        oscillation_summary_df[
            [
                "case_id",
                "mean_oscillation_index",
                "max_oscillation_index",
                "max_sign_flip_count",
                "dominant_state_key",
            ]
        ],
        on="case_id",
        how="left",
    )

    case_metrics_path = args.output_dir / "case_metrics.csv"
    phase_metrics_path = args.output_dir / "phase_metrics.csv"
    oscillation_metrics_path = args.output_dir / "oscillation_metrics.csv"
    oscillation_summary_path = args.output_dir / "oscillation_summary.csv"
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    phase_metrics_df.to_csv(phase_metrics_path, index=False, encoding="utf-8-sig")
    oscillation_metrics_df.to_csv(oscillation_metrics_path, index=False, encoding="utf-8-sig")
    oscillation_summary_df.to_csv(oscillation_summary_path, index=False, encoding="utf-8-sig")

    plot_config = build_bias_scale_plot_config()
    plot_paths: dict[str, str] = {}
    full_window = [
        float(base_cfg["fusion"]["starttime"]),
        float(base_cfg["fusion"]["finaltime"]),
    ]
    boundary_time = full_window[1]

    all_states_path = args.plot_dir / "all_states_overview.png"
    plot_state_grid(
        case_frames,
        plot_cases,
        ALL_STATE_SPECS,
        all_states_path,
        "Full INS/GNSS all-state overview under bias-scale Q sweep",
        boundary_time,
        boundary_time,
        [],
        plot_modes_by_key=build_all_state_plot_modes(),
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["all_states_overview"] = rel_from_root(all_states_path, REPO_ROOT)

    key_states_path = args.plot_dir / "key_coupling_states.png"
    plot_state_grid(
        case_frames,
        plot_cases,
        KEY_COUPLING_STATES,
        key_states_path,
        "Full INS/GNSS key coupling states under bias-scale Q sweep",
        boundary_time,
        boundary_time,
        [],
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["key_coupling_states"] = rel_from_root(key_states_path, REPO_ROOT)

    bias_scale_overview_path = args.plot_dir / "bias_scale_overview.png"
    plot_state_grid(
        case_frames,
        plot_cases,
        plot_config.overview_states,
        bias_scale_overview_path,
        "Bias and scale-factor state evolution under process-noise sweep",
        boundary_time,
        boundary_time,
        [],
        truth_keys_to_hide=plot_config.truth_keys_to_hide,
    )
    plot_paths["bias_scale_overview"] = rel_from_root(bias_scale_overview_path, REPO_ROOT)

    for group_spec in plot_config.group_specs:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        plot_state_grid(
            case_frames,
            plot_cases,
            group_spec.states,
            group_path,
            f"{group_spec.title} under bias-scale process-noise sweep",
            boundary_time,
            boundary_time,
            [],
            truth_keys_to_hide=plot_config.truth_keys_to_hide,
        )
        plot_paths[group_spec.group_id] = rel_from_root(group_path, REPO_ROOT)

    for group_spec in PVA_ERROR_GROUP_SPECS:
        group_path = args.plot_dir / f"{group_spec.group_id}.png"
        plot_state_grid(
            case_frames,
            plot_cases,
            group_spec.states,
            group_path,
            f"{group_spec.title} under bias-scale process-noise sweep",
            boundary_time,
            boundary_time,
            [],
            plot_mode="error",
            truth_keys_to_hide=plot_config.truth_keys_to_hide,
        )
        plot_paths[group_spec.group_id] = rel_from_root(group_path, REPO_ROOT)

    oscillation_plot_path = args.plot_dir / "oscillation_summary.png"
    plot_oscillation_summary(oscillation_summary_df, oscillation_plot_path)
    plot_paths["oscillation_summary"] = rel_from_root(oscillation_plot_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    manifest = {
        "exp_id": args.exp_id,
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "solver": rel_from_root(args.exe, REPO_ROOT),
        "q_scales": [float(spec.process_noise_scale_rel) for spec in case_specs],
        "fix_gnss_lever_truth": bool(args.fix_gnss_lever_truth),
        "cases": [spec.case_id for spec in case_specs],
        "full_window": full_window,
        "truth_reference_path": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_path": rel_from_root(case_metrics_path, REPO_ROOT),
        "phase_metrics_path": rel_from_root(phase_metrics_path, REPO_ROOT),
        "oscillation_metrics_path": rel_from_root(oscillation_metrics_path, REPO_ROOT),
        "oscillation_summary_path": rel_from_root(oscillation_summary_path, REPO_ROOT),
        "plot_paths": plot_paths,
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_mtime": mtime_text(args.exe),
            "truth_reference_mtime": mtime_text(truth_reference_path),
            "case_metrics_mtime": mtime_text(case_metrics_path),
            "phase_metrics_mtime": mtime_text(phase_metrics_path),
            "oscillation_metrics_mtime": mtime_text(oscillation_metrics_path),
            "oscillation_summary_mtime": mtime_text(oscillation_summary_path),
            "cases": case_freshness,
        },
    }
    write_summary(summary_path, manifest, case_metrics_df, oscillation_summary_df, plot_paths)
    manifest["freshness"]["summary_md_mtime"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
