import argparse
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_baseline_ins_gnss_outage_no_odo_nhc import (
    BASE_CONFIG_DEFAULT,
    CASE_ID_DEFAULT,
    EXP_ID_DEFAULT as BASE_EXP_ID_DEFAULT,
    README_PATH_DEFAULT,
    SOLVER_DEFAULT,
    build_run_config as build_base_run_config,
    case_label_for_mode,
    run_case as run_base_case,
)
from scripts.analysis.run_data2_ins_gnss_lever_noise_coupling_sweep import (
    parse_scale_list,
    read_rtk_file,
    scale_slug,
)
from scripts.analysis.run_data2_staged_g5_no_imu_scale import format_metric, render_table
from scripts.analysis.run_data2_state_sanity_matrix import json_safe
from scripts.analysis.run_data2_turn_window_shared_correction_probe import parse_matrix_field
from scripts.analysis.run_data2_ins_gnss_odo_nhc_pva_anchor_compare import mtime_text


DEFAULT_GNSS_PATH = Path("dataset/data2/rtk.txt")
DEFAULT_R_SCALES = "1,2,4,8,16"
DEFAULT_R_XY_SCALES = "1,2,4,8"
DEFAULT_R_Z_SCALES = "0.125,0.25,0.5,1"
DEFAULT_TARGET_WINDOW = "phase2"
SOURCE_RUNNER_PATH = Path("scripts/analysis/run_data2_baseline_ins_gnss_outage_no_odo_nhc.py")
WINDOW_SPECS = (
    ("all", None, None),
    ("phase1", "phase1_window", None),
    ("phase2", "phase2_window", None),
    ("phase12", "phase1_window", "phase2_window"),
    ("phase3", "phase3_window", None),
)


@dataclass(frozen=True)
class RScaleSpec:
    r_xy_scale: float
    r_z_scale: float

    @property
    def slug(self) -> str:
        return f"xy{scale_slug(self.r_xy_scale)}_z{scale_slug(self.r_z_scale)}"

    @property
    def is_isotropic(self) -> bool:
        return math.isclose(self.r_xy_scale, self.r_z_scale, rel_tol=0.0, abs_tol=1.0e-12)


def normalize_repo_path(path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def repo_or_abs_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return rel_from_root(resolved, REPO_ROOT)
    except ValueError:
        return str(resolved)


def default_exp_id() -> str:
    return f"EXP-{dt.datetime.now():%Y%m%d}-data2-ins-gnss-gnss-noise-xy-z-r1"


def default_output_dir() -> Path:
    return Path("output/d2_ins_gnss_gnss_noise_xy_z_r1")


def isotropic_r_scale(r_xy_scale: float, r_z_scale: float) -> float:
    if math.isclose(r_xy_scale, r_z_scale, rel_tol=0.0, abs_tol=1.0e-12):
        return float(r_xy_scale)
    return math.nan


def build_r_scale_specs(
    r_scales: list[float] | None,
    r_xy_scales: list[float] | None,
    r_z_scales: list[float] | None,
) -> list[RScaleSpec]:
    if r_xy_scales is None and r_z_scales is None:
        if not r_scales:
            raise ValueError("r_scales must not be empty when anisotropic scales are not provided")
        return [RScaleSpec(float(scale), float(scale)) for scale in r_scales]

    xy_values = [1.0] if r_xy_scales is None else [float(scale) for scale in r_xy_scales]
    z_values = [1.0] if r_z_scales is None else [float(scale) for scale in r_z_scales]
    return [RScaleSpec(r_xy_scale, r_z_scale) for r_xy_scale in xy_values for r_z_scale in z_values]


def write_axis_scaled_gnss_variant(
    src_path: Path,
    dst_path: Path,
    r_xy_scale: float,
    r_z_scale: float,
) -> dict[str, Any]:
    source_df = read_rtk_file(src_path)
    df = source_df.copy()
    df["sigma_n"] = df["sigma_n"].astype(float) * float(r_xy_scale)
    df["sigma_e"] = df["sigma_e"].astype(float) * float(r_xy_scale)
    df["sigma_d"] = df["sigma_d"].astype(float) * float(r_z_scale)
    ensure_dir(dst_path.parent)
    np.savetxt(dst_path, df.to_numpy(dtype=float), fmt=["%.2f", "%.10f", "%.10f", "%.3f", "%.6f", "%.6f", "%.6f"])
    return {
        "gnss_variant_path": repo_or_abs_path(dst_path),
        "r_xy_scale": float(r_xy_scale),
        "r_z_scale": float(r_z_scale),
        "r_scale": isotropic_r_scale(r_xy_scale, r_z_scale),
        "rows": int(df.shape[0]),
        "sigma_median_source": {
            "sigma_n": float(source_df["sigma_n"].median()),
            "sigma_e": float(source_df["sigma_e"].median()),
            "sigma_d": float(source_df["sigma_d"].median()),
        },
        "sigma_median_scaled": {
            "sigma_n": float(df["sigma_n"].median()),
            "sigma_e": float(df["sigma_e"].median()),
            "sigma_d": float(df["sigma_d"].median()),
        },
    }


def build_case_config(
    base_cfg: dict[str, Any],
    output_dir: Path,
    gnss_path: Path,
    case_id: str,
    case_label: str,
    readme_path: Path = README_PATH_DEFAULT,
    fix_gnss_lever_truth: bool = False,
    sigma_bg_degh_override: float | None = None,
    sigma_ba_mgal_override: float | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg, metadata = build_base_run_config(
        base_cfg,
        output_dir,
        readme_path=readme_path,
        fix_gnss_lever_truth=fix_gnss_lever_truth,
        sigma_bg_degh_override=sigma_bg_degh_override,
        sigma_ba_mgal_override=sigma_ba_mgal_override,
    )
    output_dir_abs = normalize_repo_path(output_dir)
    case_dir = output_dir_abs / "artifacts" / "cases" / case_id
    fusion = cfg.setdefault("fusion", {})
    fusion["gnss_path"] = rel_from_root(normalize_repo_path(gnss_path), REPO_ROOT)
    fusion["output_path"] = rel_from_root(output_dir_abs / f"SOL_{case_id}.txt", REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(
        output_dir_abs / f"state_series_{case_id}.csv",
        REPO_ROOT,
    )
    fusion["gnss_update_debug_output_path"] = rel_from_root(
        case_dir / f"gnss_updates_{case_id}.csv",
        REPO_ROOT,
    )
    metadata["case_id"] = case_id
    metadata["case_label"] = case_label
    metadata["gnss_path"] = rel_from_root(normalize_repo_path(gnss_path), REPO_ROOT)
    return cfg, metadata


def compute_window_consistency_metrics(
    frame: pd.DataFrame,
    window_name: str,
    start_time: float | None,
    end_time: float | None,
) -> dict[str, Any]:
    subset = frame.loc[frame["tag"].astype(str).str.contains("GNSS_POS", regex=False, na=False)].copy()
    if start_time is not None:
        subset = subset.loc[subset["gnss_t"] >= float(start_time) - 1.0e-9]
    if end_time is not None:
        subset = subset.loc[subset["gnss_t"] <= float(end_time) + 1.0e-9]
    subset = subset.reset_index(drop=True)

    row: dict[str, Any] = {
        "window_name": window_name,
        "start_time": float(start_time) if start_time is not None else math.nan,
        "end_time": float(end_time) if end_time is not None else math.nan,
        "accepted_updates": int(len(subset)),
    }
    if subset.empty:
        for axis in ("x", "y", "z"):
            row[f"actual_std_{axis}_m"] = math.nan
            row[f"predicted_std_{axis}_m"] = math.nan
            row[f"ratio_{axis}"] = math.nan
            row[f"normalized_std_{axis}"] = math.nan
        return row

    s_diagonals = np.full((len(subset), 3), math.nan, dtype=float)
    for idx, value in enumerate(subset["s_mat"].tolist()):
        s_mat = parse_matrix_field(value, 3, 3)
        if np.isfinite(s_mat).all():
            s_diagonals[idx, :] = np.diag(s_mat)

    for axis_idx, axis in enumerate(("x", "y", "z")):
        innovation = subset[f"y_{axis}"].to_numpy(dtype=float)
        row[f"actual_std_{axis}_m"] = float(np.std(innovation, ddof=0))

        predicted_var = s_diagonals[:, axis_idx].astype(float)
        predicted_mask = np.isfinite(predicted_var) & (predicted_var >= 0.0)
        if np.any(predicted_mask):
            predicted_std = math.sqrt(float(np.mean(predicted_var[predicted_mask])))
            row[f"predicted_std_{axis}_m"] = predicted_std
            row[f"ratio_{axis}"] = (
                float(row[f"actual_std_{axis}_m"] / predicted_std)
                if predicted_std > 0.0
                else math.nan
            )
        else:
            row[f"predicted_std_{axis}_m"] = math.nan
            row[f"ratio_{axis}"] = math.nan

        norm_mask = np.isfinite(predicted_var) & (predicted_var > 0.0)
        if np.any(norm_mask):
            normalized = innovation[norm_mask] / np.sqrt(predicted_var[norm_mask])
            row[f"normalized_std_{axis}"] = float(np.std(normalized, ddof=0))
        else:
            row[f"normalized_std_{axis}"] = math.nan
    return row


def collect_window_metrics(
    case_id: str,
    r_xy_scale: float,
    r_z_scale: float,
    metadata: dict[str, Any],
    gnss_updates_path: Path,
) -> pd.DataFrame:
    frame = pd.read_csv(gnss_updates_path)
    rows: list[dict[str, Any]] = []
    for window_name, start_key, end_key in WINDOW_SPECS:
        start_time: float | None
        end_time: float | None
        if start_key is None and end_key is None:
            start_time = None
            end_time = None
        elif end_key is None:
            start_time, end_time = metadata[start_key]
        else:
            start_time = metadata[start_key][0]
            end_time = metadata[end_key][1]
        row = compute_window_consistency_metrics(frame, window_name, start_time, end_time)
        row["case_id"] = case_id
        row["r_xy_scale"] = float(r_xy_scale)
        row["r_z_scale"] = float(r_z_scale)
        row["r_scale"] = isotropic_r_scale(r_xy_scale, r_z_scale)
        row["gnss_updates_path"] = rel_from_root(gnss_updates_path, REPO_ROOT)
        row["gnss_updates_mtime"] = mtime_text(gnss_updates_path)
        rows.append(row)
    return pd.DataFrame(rows)


def select_recommended_r_scale(summary_df: pd.DataFrame, target_window: str = DEFAULT_TARGET_WINDOW) -> dict[str, Any]:
    subset = summary_df.loc[
        (summary_df["window_name"] == target_window) & (summary_df["accepted_updates"] > 0)
    ].copy()
    if subset.empty:
        raise RuntimeError(f"no GNSS consistency rows found for target window {target_window}")

    def score_row(row: pd.Series) -> float:
        ratios = np.array([row["ratio_x"], row["ratio_y"], row["ratio_z"]], dtype=float)
        mask = np.isfinite(ratios)
        if not np.any(mask):
            return math.inf
        return float(np.mean(np.abs(ratios[mask] - 1.0)))

    subset["consistency_score"] = subset.apply(score_row, axis=1)
    sort_columns = ["consistency_score"]
    for key in ("r_xy_scale", "r_z_scale", "r_scale", "case_id"):
        if key in subset.columns and key not in sort_columns:
            sort_columns.append(key)
    subset = subset.sort_values(by=sort_columns).reset_index(drop=True)
    return subset.iloc[0].to_dict()


def assess_r_inflation_need(row: dict[str, Any] | pd.Series) -> str:
    ratios = np.array([row["ratio_x"], row["ratio_y"], row["ratio_z"]], dtype=float)
    mask = np.isfinite(ratios)
    if not np.any(mask):
        return "unknown"
    ratios = ratios[mask]
    if np.all(ratios >= 1.5):
        return "too_small"
    if np.all(ratios <= 1.0):
        return "not_too_small"
    return "mixed"


def flatten_target_window_rows(summary_df: pd.DataFrame, target_window: str) -> pd.DataFrame:
    subset = summary_df.loc[summary_df["window_name"] == target_window].copy()
    if subset.empty:
        return subset
    subset["consistency_score"] = subset.apply(
        lambda row: float(
            np.mean(
                np.abs(
                    np.array([row["ratio_x"], row["ratio_y"], row["ratio_z"]], dtype=float) - 1.0
                )
            )
        ),
        axis=1,
    )
    sort_columns = [col for col in ("r_xy_scale", "r_z_scale", "r_scale", "case_id") if col in subset.columns]
    if sort_columns:
        subset = subset.sort_values(by=sort_columns)
    return subset.reset_index(drop=True)


def select_base_row(target_rows: pd.DataFrame) -> dict[str, Any]:
    masks: list[pd.Series] = []
    if {"r_xy_scale", "r_z_scale"}.issubset(target_rows.columns):
        masks.append(
            np.isclose(target_rows["r_xy_scale"].astype(float), 1.0)
            & np.isclose(target_rows["r_z_scale"].astype(float), 1.0)
        )
    if "r_scale" in target_rows.columns:
        masks.append(np.isclose(target_rows["r_scale"].fillna(math.nan).astype(float), 1.0))
    for mask in masks:
        subset = target_rows.loc[mask]
        if not subset.empty:
            return subset.iloc[0].to_dict()
    return target_rows.iloc[0].to_dict()


def build_summary_lines(
    manifest: dict[str, Any],
    target_rows: pd.DataFrame,
    recommended: dict[str, Any],
) -> list[str]:
    base_row = select_base_row(target_rows)
    base_assessment = assess_r_inflation_need(base_row)
    table_rows: list[list[str]] = []
    sort_columns = [col for col in ("r_xy_scale", "r_z_scale", "r_scale", "case_id") if col in target_rows.columns]
    ordered_rows = target_rows.sort_values(by=sort_columns) if sort_columns else target_rows
    for _, row in ordered_rows.iterrows():
        table_rows.append(
            [
                str(row["case_id"]),
                format_metric(float(row.get("r_xy_scale", row.get("r_scale", math.nan)))),
                format_metric(float(row.get("r_z_scale", row.get("r_scale", math.nan)))),
                str(int(row["accepted_updates"])),
                format_metric(float(row["actual_std_x_m"])),
                format_metric(float(row["predicted_std_x_m"])),
                format_metric(float(row["ratio_x"])),
                format_metric(float(row["actual_std_y_m"])),
                format_metric(float(row["predicted_std_y_m"])),
                format_metric(float(row["ratio_y"])),
                format_metric(float(row["actual_std_z_m"])),
                format_metric(float(row["predicted_std_z_m"])),
                format_metric(float(row["ratio_z"])),
                format_metric(float(row["consistency_score"])),
                format_metric(float(row.get("overall_rmse_3d_m_aux", math.nan))),
            ]
        )

    lines = [
        "# data2 INS/GNSS GNSS noise consistency summary",
        "",
        f"- exp_id: `{manifest['exp_id']}`",
        f"- base_config: `{manifest['base_config']}`",
        f"- source_runner: `{manifest['source_runner']}`",
        f"- output_dir: `{manifest['output_dir']}`",
        f"- target_window: `{manifest['target_window']}`",
        f"- generated_at: `{manifest['generated_at']}`",
        "- experiment contract: `POS-320` bias/scale-factor settings come from the corrected `INS/GNSS` outage runner; GNSS `R` is adjusted by scaling `dataset/data2/rtk.txt` columns `sigma_n/e` and `sigma_d`.",
        "",
        "## Target Window Comparison",
    ]
    lines.extend(
        render_table(
            [
                "case_id",
                "r_xy_scale",
                "r_z_scale",
                "updates",
                "act_x",
                "pred_x",
                "ratio_x",
                "act_y",
                "pred_y",
                "ratio_y",
                "act_z",
                "pred_z",
                "ratio_z",
                "score",
                "rmse3d_m",
            ],
            table_rows,
        )
    )
    lines.extend(
        [
            "",
            "## Recommendation",
            (
                f"- base_r1_assessment: `{base_assessment}` with "
                f"`x={format_metric(float(base_row['ratio_x']))}`, "
                f"`y={format_metric(float(base_row['ratio_y']))}`, "
                f"`z={format_metric(float(base_row['ratio_z']))}`"
            ),
            (
                f"- recommended_r_scales: `xy={format_metric(float(recommended.get('r_xy_scale', recommended.get('r_scale', math.nan))))}`, "
                f"`z={format_metric(float(recommended.get('r_z_scale', recommended.get('r_scale', math.nan))))}` "
                f"(case=`{recommended['case_id']}`, target_window=`{manifest['target_window']}`, "
                f"consistency_score=`{format_metric(float(recommended['consistency_score']))}`)"
            ),
            (
                f"- recommended ratios: `x={format_metric(float(recommended['ratio_x']))}`, "
                f"`y={format_metric(float(recommended['ratio_y']))}`, "
                f"`z={format_metric(float(recommended['ratio_z']))}`"
            ),
            (
                f"- recommended actual/predicted std: "
                f"`x={format_metric(float(recommended['actual_std_x_m']))}/{format_metric(float(recommended['predicted_std_x_m']))} m`, "
                f"`y={format_metric(float(recommended['actual_std_y_m']))}/{format_metric(float(recommended['predicted_std_y_m']))} m`, "
                f"`z={format_metric(float(recommended['actual_std_z_m']))}/{format_metric(float(recommended['predicted_std_z_m']))} m`"
            ),
        ]
    )
    if base_assessment == "too_small":
        lines.append(
            "- conclusion: base `R=1x` is too small in the target window; isotropic `R` inflation is justified and the recommended scale is the closest tested match."
        )
    elif base_assessment == "not_too_small":
        lines.append(
            "- conclusion: base `R=1x` is not too small in the target window because accepted innovation spread is already at or below `sqrt(HPH^T+R)` on all axes."
        )
    else:
        lines.append(
            "- conclusion: evidence is mixed by axis in the target window; isotropic `R` inflation can improve some axes but does not support a blanket claim that GNSS `R` is globally too small."
        )
    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check GNSS innovation consistency under the POS-320 INS/GNSS bias-scale contract by scanning isotropic or split GNSS R scales."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--exe", type=Path, default=SOLVER_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=default_output_dir())
    parser.add_argument("--gnss-path", type=Path, default=DEFAULT_GNSS_PATH)
    parser.add_argument("--r-scales", default=DEFAULT_R_SCALES)
    parser.add_argument("--r-xy-scales", default=None)
    parser.add_argument("--r-z-scales", default=None)
    parser.add_argument("--exp-id", default=default_exp_id())
    parser.add_argument("--target-window", default=DEFAULT_TARGET_WINDOW)
    parser.add_argument("--fix-gnss-lever-truth", action="store_true")
    parser.add_argument("--sigma-bg-degh-override", type=float, default=None)
    parser.add_argument("--sigma-ba-mgal-override", type=float, default=None)
    args = parser.parse_args()
    args.base_config = normalize_repo_path(args.base_config)
    args.exe = normalize_repo_path(args.exe)
    args.output_dir = normalize_repo_path(args.output_dir)
    args.gnss_path = normalize_repo_path(args.gnss_path)
    args.r_scales = parse_scale_list(args.r_scales) if args.r_scales else None
    args.r_xy_scales = parse_scale_list(args.r_xy_scales) if args.r_xy_scales else None
    args.r_z_scales = parse_scale_list(args.r_z_scales) if args.r_z_scales else None
    if args.r_xy_scales is not None or args.r_z_scales is not None:
        args.r_scales = None
    args.r_scale_specs = build_r_scale_specs(args.r_scales, args.r_xy_scales, args.r_z_scales)
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.gnss_variant_dir = args.artifacts_dir / "gnss_variants"
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver executable: {args.exe}")
    if not args.gnss_path.exists():
        raise FileNotFoundError(f"missing GNSS file: {args.gnss_path}")
    if args.target_window not in {item[0] for item in WINDOW_SPECS}:
        raise ValueError(f"unsupported target window: {args.target_window}")

    ensure_dir(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.gnss_variant_dir)

    base_cfg = load_yaml(args.base_config)

    gnss_variant_rows: list[dict[str, Any]] = []
    gnss_paths: dict[str, Path] = {}
    for spec in args.r_scale_specs:
        variant_path = args.gnss_variant_dir / f"rtk_sigma_{spec.slug}.txt"
        variant_row = write_axis_scaled_gnss_variant(
            args.gnss_path,
            variant_path,
            r_xy_scale=spec.r_xy_scale,
            r_z_scale=spec.r_z_scale,
        )
        gnss_variant_rows.append(variant_row)
        gnss_paths[spec.slug] = variant_path

    case_rows: list[dict[str, Any]] = []
    summary_frames: list[pd.DataFrame] = []

    base_case_label = case_label_for_mode(args.fix_gnss_lever_truth)
    base_case_prefix = "d2ig_fixlev_noise" if args.fix_gnss_lever_truth else "d2ig_noise"
    for idx, spec in enumerate(args.r_scale_specs, start=1):
        case_id = f"{base_case_prefix}_{spec.slug}"
        case_label = f"{base_case_label} GNSS noise xy={spec.r_xy_scale:g}x z={spec.r_z_scale:g}x"
        case_dir = args.case_root / case_id
        ensure_dir(case_dir)

        cfg, metadata = build_case_config(
            base_cfg=base_cfg,
            output_dir=args.output_dir,
            gnss_path=gnss_paths[spec.slug],
            case_id=case_id,
            case_label=case_label,
            fix_gnss_lever_truth=args.fix_gnss_lever_truth,
            sigma_bg_degh_override=args.sigma_bg_degh_override,
            sigma_ba_mgal_override=args.sigma_ba_mgal_override,
        )
        cfg_path = case_dir / f"config_{case_id}.yaml"
        save_yaml(cfg, cfg_path)

        case_row = run_base_case(cfg_path, args.output_dir, case_dir, args.exe, case_id, case_label)
        case_row["r_xy_scale"] = float(spec.r_xy_scale)
        case_row["r_z_scale"] = float(spec.r_z_scale)
        case_row["r_scale"] = isotropic_r_scale(spec.r_xy_scale, spec.r_z_scale)
        case_row["case_config"] = rel_from_root(cfg_path, REPO_ROOT)
        gnss_updates_path = case_dir / f"gnss_updates_{case_id}.csv"
        if not gnss_updates_path.exists():
            raise RuntimeError(f"missing GNSS update debug output for {case_id}: {gnss_updates_path}")
        case_row["gnss_updates_path"] = rel_from_root(gnss_updates_path, REPO_ROOT)
        case_row["gnss_updates_mtime"] = mtime_text(gnss_updates_path)
        case_rows.append(case_row)

        window_df = collect_window_metrics(case_id, spec.r_xy_scale, spec.r_z_scale, metadata, gnss_updates_path)
        summary_frames.append(window_df)
        print(f"[{idx}/{len(args.r_scale_specs)}] completed {case_id}")

    case_metrics_df = (
        pd.DataFrame(case_rows).sort_values(by=["r_xy_scale", "r_z_scale", "case_id"]).reset_index(drop=True)
    )
    summary_df = pd.concat(summary_frames, ignore_index=True)
    summary_df = summary_df.merge(
        case_metrics_df[
            ["case_id", "r_xy_scale", "r_z_scale", "overall_rmse_3d_m_aux", "stdout_path", "config_path"]
        ],
        on=["case_id", "r_xy_scale", "r_z_scale"],
        how="left",
    )
    target_rows = flatten_target_window_rows(summary_df, args.target_window)
    recommended = select_recommended_r_scale(summary_df, args.target_window)

    case_metrics_path = args.output_dir / "case_metrics.csv"
    window_metrics_path = args.output_dir / "window_metrics.csv"
    gnss_variant_metrics_path = args.output_dir / "gnss_variant_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    summary_df.to_csv(window_metrics_path, index=False, encoding="utf-8-sig")
    pd.DataFrame(gnss_variant_rows).to_csv(gnss_variant_metrics_path, index=False, encoding="utf-8-sig")

    summary_path = args.output_dir / "summary.md"
    manifest = {
        "exp_id": args.exp_id,
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "source_runner": rel_from_root(SOURCE_RUNNER_PATH, REPO_ROOT),
        "base_runner_exp_id_reference": BASE_EXP_ID_DEFAULT,
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "solver": rel_from_root(args.exe, REPO_ROOT),
        "target_window": args.target_window,
        "gnss_source_path": rel_from_root(args.gnss_path, REPO_ROOT),
        "r_scales": [float(x) for x in args.r_scales] if args.r_scales is not None else None,
        "r_xy_scales": [float(x) for x in args.r_xy_scales] if args.r_xy_scales is not None else None,
        "r_z_scales": [float(x) for x in args.r_z_scales] if args.r_z_scales is not None else None,
        "r_scale_specs": [
            {"r_xy_scale": float(spec.r_xy_scale), "r_z_scale": float(spec.r_z_scale), "slug": spec.slug}
            for spec in args.r_scale_specs
        ],
        "fix_gnss_lever_truth": bool(args.fix_gnss_lever_truth),
        "sigma_bg_degh_override": args.sigma_bg_degh_override,
        "sigma_ba_mgal_override": args.sigma_ba_mgal_override,
        "case_metrics_path": rel_from_root(case_metrics_path, REPO_ROOT),
        "window_metrics_path": rel_from_root(window_metrics_path, REPO_ROOT),
        "gnss_variant_metrics_path": rel_from_root(gnss_variant_metrics_path, REPO_ROOT),
        "recommended": recommended,
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "freshness": {
            "base_config_mtime": mtime_text(args.base_config),
            "solver_mtime": mtime_text(args.exe),
            "case_metrics_mtime": mtime_text(case_metrics_path),
            "window_metrics_mtime": mtime_text(window_metrics_path),
            "gnss_variant_metrics_mtime": mtime_text(gnss_variant_metrics_path),
        },
    }
    summary_path.write_text("\n".join(build_summary_lines(manifest, target_rows, recommended)) + "\n", encoding="utf-8")
    manifest["freshness"]["summary_md_mtime"] = mtime_text(summary_path)
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
