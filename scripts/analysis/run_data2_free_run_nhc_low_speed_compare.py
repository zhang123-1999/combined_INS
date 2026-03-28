from __future__ import annotations

import argparse
import datetime as dt
import json
import math
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

import scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare as baseline_compare  # noqa: E402
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import json_safe, reset_directory  # noqa: E402


EXP_ID = "EXP-20260323-data2-free-run-nhc-low-speed-compare-r1"
DEFAULT_OUTPUT_DIR = Path("output/data2_free_run_nhc_low_speed_compare_r1_20260323")
DEFAULT_BASE_CONFIG = Path("config_data2_baseline_eskf.yaml")
DEFAULT_LOW_SPEED_THRESHOLD_MPS = 5.0
DEFAULT_ZOOM_START = 528450.0
DEFAULT_ZOOM_END = 528580.0


def mtime_text(path: Path) -> str:
    return dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")


def format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            return "NA"
        return f"{float(value):.6f}"
    return str(value)


def render_table(columns: list[str], rows: list[list[str]]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return lines


def build_case_config(
    base_cfg: dict[str, Any],
    case_dir: Path,
    spec: baseline_compare.CaseSpec,
    nhc_disable_below_forward_speed: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg, overrides = baseline_compare.build_case_config(base_cfg, case_dir, spec)
    constraints_cfg = cfg.setdefault("fusion", {}).setdefault("constraints", {})
    constraints_cfg["nhc_disable_below_forward_speed"] = float(nhc_disable_below_forward_speed)
    overrides["fusion.constraints.nhc_disable_below_forward_speed"] = float(
        nhc_disable_below_forward_speed
    )
    return cfg, overrides


def write_case_config(
    base_cfg: dict[str, Any],
    case_dir: Path,
    spec: baseline_compare.CaseSpec,
    nhc_disable_below_forward_speed: float,
) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(
        base_cfg,
        case_dir,
        spec,
        nhc_disable_below_forward_speed=nhc_disable_below_forward_speed,
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides


def build_final_state_comparison(
    case_frames: dict[str, pd.DataFrame], case_specs: tuple[baseline_compare.CaseSpec, ...]
) -> pd.DataFrame:
    base_case = case_specs[0].case_id
    compare_case = case_specs[1].case_id
    rows: list[dict[str, Any]] = []
    for state_spec in baseline_compare.ALL_STATE_SPECS:
        base_val = float(case_frames[base_case][state_spec.key].iloc[-1])
        compare_val = float(case_frames[compare_case][state_spec.key].iloc[-1])
        values = [base_val, compare_val]
        rows.append(
            {
                "state_name": state_spec.key,
                "label": state_spec.label,
                "unit": state_spec.unit,
                f"{base_case}_final": base_val,
                f"{compare_case}_final": compare_val,
                "low_speed_off_minus_normal": compare_val - base_val,
                "max_span_across_cases": max(values) - min(values),
            }
        )
    return pd.DataFrame(rows)


def plot_all_states_overview_window(
    case_frames: dict[str, pd.DataFrame],
    case_specs: tuple[baseline_compare.CaseSpec, ...],
    output_path: Path,
    start_time: float,
    end_time: float,
) -> None:
    n_cols = 3
    n_rows = math.ceil(len(baseline_compare.ALL_STATE_SPECS) / n_cols)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(16, 2.6 * n_rows), sharex=False)
    axes_flat = np.atleast_1d(axes).reshape(-1)
    for idx, state_spec in enumerate(baseline_compare.ALL_STATE_SPECS):
        ax = axes_flat[idx]
        for case_spec in case_specs:
            df = case_frames[case_spec.case_id]
            mask = (df["timestamp"] >= start_time) & (df["timestamp"] <= end_time)
            df_window = df.loc[mask]
            if df_window.empty:
                continue
            t = df_window["timestamp"].to_numpy(dtype=float)
            v = df_window[state_spec.key].to_numpy(dtype=float)
            t_plot, v_plot = baseline_compare.downsample_for_plot(t, v)
            ax.plot(t_plot, v_plot, linewidth=0.9, color=case_spec.color, label=case_spec.label)
        ax.set_title(state_spec.label, fontsize=9)
        ax.set_ylabel(state_spec.unit, fontsize=8)
        ax.grid(alpha=0.2)
        ax.tick_params(axis="both", labelsize=7)
        if idx == 0:
            ax.legend(loc="best", fontsize=8)
    for idx in range(len(baseline_compare.ALL_STATE_SPECS), len(axes_flat)):
        axes_flat[idx].axis("off")
    fig.suptitle(f"All 31 states overview [{start_time:.0f}, {end_time:.0f}] s", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def write_summary(
    output_path: Path,
    manifest: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    final_state_df: pd.DataFrame,
    case_specs: tuple[baseline_compare.CaseSpec, ...],
    low_speed_threshold_mps: float,
) -> None:
    metric_columns = [
        "case_label",
        "overall_rmse_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "odo_accept_ratio",
        "nhc_accept_ratio",
        "gnss_accept_ratio",
    ]
    metric_rows: list[list[str]] = []
    for _, row in case_metrics_df.iterrows():
        metric_rows.append(
            [
                str(row.get("case_label", "")),
                format_metric(row.get("overall_rmse_3d_m_aux")),
                format_metric(row.get("overall_final_err_3d_m_aux")),
                format_metric(row.get("odo_accept_ratio")),
                format_metric(row.get("nhc_accept_ratio")),
                format_metric(
                    baseline_compare.metric_value(row, "gnss_accept_ratio", "gnss_pos_accept_ratio")
                ),
            ]
        )

    top_delta = final_state_df.sort_values(by="max_span_across_cases", ascending=False).head(8)
    delta_rows = [
        [
            str(row["state_name"]),
            str(row["unit"]),
            format_metric(row[f"{case_specs[0].case_id}_final"]),
            format_metric(row[f"{case_specs[1].case_id}_final"]),
            format_metric(row["low_speed_off_minus_normal"]),
            format_metric(row["max_span_across_cases"]),
        ]
        for _, row in top_delta.iterrows()
    ]

    lines: list[str] = [
        "# data2 free-run NHC low-speed disable comparison",
        "",
        "## 实验设计",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base config: `{manifest['base_config']}`",
        "- 两组都采用 `ESKF free-run`，保持 `INS/GNSS/ODO/NHC`、同一套噪声与 zero-init 口径。",
        "- 两组都沿用当前 `r2` 口径：`non-PVA zero-init`、`odo_scale=1e-6`、安装角过程噪声 `0.1x baseline`。",
        f"- 组一：`normal NHC`，`nhc_disable_below_forward_speed=0.0 m/s`。",
        f"- 组二：`NHC off below {low_speed_threshold_mps:.1f} m/s`，其余配置完全相同。",
        "",
        "## 导航与一致性指标",
    ]
    lines.extend(render_table(metric_columns, metric_rows))
    lines.extend(
        [
            "",
            "## 末态差异最大的状态",
        ]
    )
    lines.extend(
        render_table(
            [
                "state_name",
                "unit",
                f"{case_specs[0].case_id}_final",
                f"{case_specs[1].case_id}_final",
                "low_speed_off_minus_normal",
                "max_span_across_cases",
            ],
            delta_rows,
        )
    )
    lines.extend(
        [
            "",
            "## 产物",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- final_state_comparison: `{manifest['final_state_comparison_csv']}`",
            f"- plots_dir: `{manifest['plots_dir']}`",
            f"- manifest: `{manifest['manifest_path']}`",
        ]
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare normal free-run NHC against a low-speed-disabled NHC free-run case and plot all 31 states."
    )
    parser.add_argument("--base-config", type=Path, default=DEFAULT_BASE_CONFIG)
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--exp-id", default=EXP_ID)
    parser.add_argument(
        "--nhc-disable-below-forward-speed",
        type=float,
        default=DEFAULT_LOW_SPEED_THRESHOLD_MPS,
        help="Disable NHC whenever |v_b.x| is below this threshold in m/s; <=0 disables this probe.",
    )
    parser.add_argument("--zoom-start", type=float, default=DEFAULT_ZOOM_START)
    parser.add_argument("--zoom-end", type=float, default=DEFAULT_ZOOM_END)
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
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

    case_specs: tuple[baseline_compare.CaseSpec, ...] = (
        baseline_compare.CaseSpec(
            "group1_normal_nhc",
            "group1: normal NHC",
            "#4c78a8",
            "ESKF",
            False,
            False,
        ),
        baseline_compare.CaseSpec(
            "group2_low_speed_nhc_off",
            f"group2: NHC off below {args.nhc_disable_below_forward_speed:.1f} m/s",
            "#e45756",
            "ESKF",
            False,
            False,
        ),
    )
    baseline_compare.CASE_SPECS = case_specs

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    thresholds = {
        case_specs[0].case_id: 0.0,
        case_specs[1].case_id: float(args.nhc_disable_below_forward_speed),
    }

    case_rows: list[dict[str, Any]] = []
    case_frames: dict[str, pd.DataFrame] = {}
    case_config_paths: dict[str, str] = {}
    case_unified_paths: dict[str, str] = {}
    case_overrides: dict[str, dict[str, Any]] = {}

    for spec in case_specs:
        case_dir = args.case_root / spec.case_id
        ensure_dir(case_dir)
        cfg_path, overrides = write_case_config(base_cfg, case_dir, spec, thresholds[spec.case_id])
        case_config_paths[spec.case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_overrides[spec.case_id] = overrides
        case_row = baseline_compare.run_case(case_dir, cfg_path, args.exe, spec)
        case_row["nhc_disable_below_forward_speed"] = thresholds[spec.case_id]
        unified_df = baseline_compare.build_unified_state_df(
            (REPO_ROOT / case_row["sol_path"]).resolve(),
            (REPO_ROOT / case_row["state_series_path"]).resolve(),
        )
        unified_path = baseline_compare.save_unified_state_df(case_dir, spec, unified_df)
        case_row["all_states_path"] = rel_from_root(unified_path, REPO_ROOT)
        case_row["all_states_mtime"] = mtime_text(unified_path)
        case_frames[spec.case_id] = unified_df
        case_unified_paths[spec.case_id] = case_row["all_states_path"]
        case_rows.append(case_row)

    case_metrics_df = pd.DataFrame(case_rows)
    case_metrics_df = case_metrics_df.set_index("case_id").loc[[spec.case_id for spec in case_specs]].reset_index()
    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    final_state_df = build_final_state_comparison(case_frames, case_specs)
    final_state_comparison_path = args.output_dir / "final_state_comparison.csv"
    final_state_df.to_csv(final_state_comparison_path, index=False, encoding="utf-8-sig")

    plot_paths: dict[str, str] = {}
    for group_spec in baseline_compare.GROUP_SPECS:
        plot_path = args.plot_dir / f"{group_spec.group_id}_compare.png"
        baseline_compare.plot_group(case_frames, group_spec, plot_path)
        plot_paths[group_spec.group_id] = rel_from_root(plot_path, REPO_ROOT)
    overview_path = args.plot_dir / "all_states_overview.png"
    baseline_compare.plot_all_states_overview(case_frames, overview_path)
    plot_paths["all_states_overview"] = rel_from_root(overview_path, REPO_ROOT)

    zoom_overview_path = args.plot_dir / "all_states_overview_jump_window.png"
    plot_all_states_overview_window(
        case_frames,
        case_specs,
        zoom_overview_path,
        start_time=float(args.zoom_start),
        end_time=float(args.zoom_end),
    )
    plot_paths["all_states_overview_jump_window"] = rel_from_root(zoom_overview_path, REPO_ROOT)

    summary_path = args.output_dir / "summary.md"
    manifest_path = args.output_dir / "manifest.json"
    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "final_state_comparison_csv": rel_from_root(final_state_comparison_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "case_config_paths": case_config_paths,
        "case_all_states_paths": case_unified_paths,
        "case_overrides": case_overrides,
        "plots": plot_paths,
        "assumptions": [
            "Both cases reuse the current data2 ESKF free-run baseline setup with non-PVA zero-init and 0.1x mounting process noise.",
            f"The mitigation probe disables NHC whenever |v_b.x| < {float(args.nhc_disable_below_forward_speed):.1f} m/s.",
            "ODO remains enabled in both cases so the experiment isolates low-speed NHC gating rather than removing all wheel-related constraints.",
            f"A zoomed 31-state overview is generated for [{float(args.zoom_start):.0f}, {float(args.zoom_end):.0f}] s to inspect the 528500+ jump window.",
        ],
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_exe_mtime": mtime_text(args.exe),
            "case_metrics_csv": mtime_text(case_metrics_path),
            "final_state_comparison_csv": mtime_text(final_state_comparison_path),
            "plots_dir": mtime_text(overview_path),
            "zoom_plot": mtime_text(zoom_overview_path),
        },
    }
    manifest["manifest_path"] = rel_from_root(manifest_path, REPO_ROOT)

    write_summary(
        summary_path,
        manifest,
        case_metrics_df,
        final_state_df,
        case_specs,
        low_speed_threshold_mps=float(args.nhc_disable_below_forward_speed),
    )
    manifest["freshness"]["summary_md"] = mtime_text(summary_path)
    manifest_path.write_text(
        json.dumps(json_safe(manifest), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
