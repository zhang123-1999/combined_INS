import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import (  # noqa: E402
    ensure_dir,
    load_yaml,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import (  # noqa: E402
    BASE_SHORT_CONFIG,
    EXE_DEFAULT,
    CaseSpec,
    build_case_config,
    mtime_text,
    run_solver,
)


OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_first_update_geometry_r1_20260323")

CASES = [
    CaseSpec("baseline"),
    CaseSpec("nhc_disable_bgz_state_update", debug_nhc_disable_bgz_state_update=True),
    CaseSpec(
        "odo_nhc_disable_bgz_state_update",
        debug_odo_disable_bgz_state_update=True,
        debug_nhc_disable_bgz_state_update=True,
    ),
    CaseSpec("odo_before_nhc", debug_run_odo_before_nhc=True),
]

FIRST_ROW_COLUMNS = [
    "seq_idx",
    "tag",
    "t_meas",
    "y_norm",
    "nis",
    "s_trace",
    "dx_att_z",
    "dx_bg_z",
    "dx_mount_yaw",
    "k_row_att_z_norm",
    "k_row_bg_z_norm",
    "k_row_mount_yaw_norm",
    "k_att_z_vec",
    "k_bg_z_vec",
    "k_mount_yaw_vec",
    "h_col_att_z_norm",
    "h_col_bg_z_norm",
    "h_col_mount_yaw_norm",
    "info_att_z",
    "info_bg_z",
    "info_mount_yaw",
    "num_att_z_vec",
    "num_bg_z_vec",
    "num_mount_yaw_vec",
    "num_bg_z_from_vel_vec",
    "num_bg_z_from_att_vec",
    "num_bg_z_from_bg_vec",
    "num_bg_z_from_sg_vec",
    "num_bg_z_from_mount_vec",
    "num_bg_z_from_lever_vec",
    "p_row_bg_z_att_vec",
    "p_row_bg_z_bg_vec",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
    "h_bg_x_vec",
    "h_bg_y_vec",
    "h_bg_z_vec",
    "num_bg_z_from_att_x_vec",
    "num_bg_z_from_att_y_vec",
    "num_bg_z_from_att_z_vec",
    "num_bg_z_from_bg_x_vec",
    "num_bg_z_from_bg_y_vec",
    "num_bg_z_from_bg_z_vec",
    "prior_corr_att_z_bg_z",
    "post_corr_att_z_bg_z",
    "prior_corr_mount_yaw_bg_z",
    "post_corr_mount_yaw_bg_z",
    "delta_cov_att_z_bg_z_left",
    "delta_cov_att_z_bg_z_right",
    "delta_cov_att_z_bg_z_gain",
    "delta_cov_att_z_bg_z_total",
    "delta_cov_mount_yaw_bg_z_left",
    "delta_cov_mount_yaw_bg_z_right",
    "delta_cov_mount_yaw_bg_z_gain",
    "delta_cov_mount_yaw_bg_z_total",
]


def prepare_case(
    base_cfg: dict[str, Any], case_dir: Path, exe_path: Path, spec: CaseSpec
) -> dict[str, Any]:
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
        raise RuntimeError(f"missing outputs for {spec.case_id}")

    return {
        "case_id": spec.case_id,
        "config_path": cfg_path,
        "sol_path": sol_path,
        "state_series_path": state_series_path,
        "mechanism_path": mechanism_path,
        "stdout_path": stdout_path,
        "stdout_text": stdout_text,
    }


def load_first_rows(case_id: str, mechanism_path: Path, num_rows: int) -> pd.DataFrame:
    df = pd.read_csv(mechanism_path).copy()
    df.insert(0, "seq_idx", range(1, len(df) + 1))
    df.insert(0, "case_id", case_id)
    return df.loc[:, ["case_id", *FIRST_ROW_COLUMNS]].head(num_rows)


def build_tag_first_summary(case_id: str, mechanism_path: Path) -> dict[str, Any]:
    df = pd.read_csv(mechanism_path).copy()
    df.insert(0, "seq_idx", range(1, len(df) + 1))

    summary: dict[str, Any] = {
        "case_id": case_id,
        "first_four_tags": ">".join(df["tag"].head(4).tolist()),
        "first_four_seq": " | ".join(
            f"{int(row.seq_idx)}:{row.tag}" for row in df.head(4).itertuples()
        ),
    }

    for tag in ("NHC", "ODO"):
        tag_df = df[df["tag"] == tag]
        if tag_df.empty:
            for col in (
                "seq_idx",
                "t_meas",
                "y_norm",
                "nis",
                "s_trace",
                "dx_att_z",
                "dx_bg_z",
                "dx_mount_yaw",
                "k_row_bg_z_norm",
                "k_bg_z_vec",
                "h_col_bg_z_norm",
                "h_col_mount_yaw_norm",
                "info_bg_z",
                "info_mount_yaw",
                "num_bg_z_vec",
                "num_mount_yaw_vec",
                "num_bg_z_from_vel_vec",
                "num_bg_z_from_att_vec",
                "num_bg_z_from_bg_vec",
                "num_bg_z_from_sg_vec",
                "num_bg_z_from_mount_vec",
                "num_bg_z_from_lever_vec",
                "p_row_bg_z_att_vec",
                "p_row_bg_z_bg_vec",
                "h_att_x_vec",
                "h_att_y_vec",
                "h_att_z_vec",
                "h_bg_x_vec",
                "h_bg_y_vec",
                "h_bg_z_vec",
                "num_bg_z_from_att_x_vec",
                "num_bg_z_from_att_y_vec",
                "num_bg_z_from_att_z_vec",
                "num_bg_z_from_bg_x_vec",
                "num_bg_z_from_bg_y_vec",
                "num_bg_z_from_bg_z_vec",
                "prior_corr_att_z_bg_z",
                "post_corr_att_z_bg_z",
                "prior_corr_mount_yaw_bg_z",
                "post_corr_mount_yaw_bg_z",
                "delta_cov_att_z_bg_z_left",
                "delta_cov_att_z_bg_z_right",
                "delta_cov_att_z_bg_z_gain",
                "delta_cov_att_z_bg_z_total",
                "delta_cov_mount_yaw_bg_z_left",
                "delta_cov_mount_yaw_bg_z_right",
                "delta_cov_mount_yaw_bg_z_gain",
                "delta_cov_mount_yaw_bg_z_total",
            ):
                summary[f"{tag.lower()}_{col}"] = float("nan")
            continue

        first = tag_df.iloc[0]
        for col in (
            "seq_idx",
            "t_meas",
            "y_norm",
            "nis",
            "s_trace",
            "dx_att_z",
            "dx_bg_z",
            "dx_mount_yaw",
            "k_row_bg_z_norm",
            "k_bg_z_vec",
            "h_col_bg_z_norm",
            "h_col_mount_yaw_norm",
            "info_bg_z",
            "info_mount_yaw",
            "num_bg_z_vec",
            "num_mount_yaw_vec",
            "num_bg_z_from_vel_vec",
            "num_bg_z_from_att_vec",
            "num_bg_z_from_bg_vec",
            "num_bg_z_from_sg_vec",
            "num_bg_z_from_mount_vec",
            "num_bg_z_from_lever_vec",
            "p_row_bg_z_att_vec",
            "p_row_bg_z_bg_vec",
            "h_att_x_vec",
            "h_att_y_vec",
            "h_att_z_vec",
            "h_bg_x_vec",
            "h_bg_y_vec",
            "h_bg_z_vec",
            "num_bg_z_from_att_x_vec",
            "num_bg_z_from_att_y_vec",
            "num_bg_z_from_att_z_vec",
            "num_bg_z_from_bg_x_vec",
            "num_bg_z_from_bg_y_vec",
            "num_bg_z_from_bg_z_vec",
            "prior_corr_att_z_bg_z",
            "post_corr_att_z_bg_z",
            "prior_corr_mount_yaw_bg_z",
            "post_corr_mount_yaw_bg_z",
            "delta_cov_att_z_bg_z_left",
            "delta_cov_att_z_bg_z_right",
            "delta_cov_att_z_bg_z_gain",
            "delta_cov_att_z_bg_z_total",
            "delta_cov_mount_yaw_bg_z_left",
            "delta_cov_mount_yaw_bg_z_right",
            "delta_cov_mount_yaw_bg_z_gain",
            "delta_cov_mount_yaw_bg_z_total",
        ):
            summary[f"{tag.lower()}_{col}"] = first[col]
    if pd.notna(summary["nhc_seq_idx"]) and pd.notna(summary["odo_seq_idx"]):
        summary["odo_minus_nhc_seq"] = float(summary["odo_seq_idx"] - summary["nhc_seq_idx"])
        summary["odo_first_mount_info_vs_nhc"] = float(
            summary["odo_info_mount_yaw"] - summary["nhc_info_mount_yaw"]
        )
    else:
        summary["odo_minus_nhc_seq"] = float("nan")
        summary["odo_first_mount_info_vs_nhc"] = float("nan")
    return summary


def render_table(df: pd.DataFrame, cols: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for _, row in df.iterrows():
        values: list[str] = []
        for col in cols:
            value = row[col]
            if isinstance(value, str):
                values.append(value)
            else:
                try:
                    values.append(f"{float(value):.6f}")
                except Exception:
                    values.append("NA")
        lines.append("| " + " | ".join(values) + " |")
    return lines


def render_summary(tag_first_df: pd.DataFrame, first_rows_df: pd.DataFrame, num_rows: int) -> str:
    overview_cols = [
        "case_id",
        "first_four_tags",
        "nhc_seq_idx",
        "nhc_dx_mount_yaw",
        "nhc_dx_bg_z",
        "nhc_k_bg_z_vec",
        "nhc_num_bg_z_vec",
        "nhc_num_bg_z_from_vel_vec",
        "nhc_num_bg_z_from_att_vec",
        "nhc_num_bg_z_from_bg_vec",
        "nhc_num_bg_z_from_sg_vec",
        "nhc_num_bg_z_from_mount_vec",
        "nhc_num_bg_z_from_lever_vec",
        "nhc_post_corr_att_z_bg_z",
        "nhc_post_corr_mount_yaw_bg_z",
        "nhc_delta_cov_mount_yaw_bg_z_left",
        "nhc_delta_cov_mount_yaw_bg_z_right",
        "nhc_delta_cov_mount_yaw_bg_z_gain",
        "nhc_delta_cov_mount_yaw_bg_z_total",
        "odo_seq_idx",
        "odo_dx_mount_yaw",
        "odo_dx_bg_z",
        "odo_h_col_mount_yaw_norm",
        "odo_info_mount_yaw",
    ]
    detail_cols = [
        "case_id",
        "seq_idx",
        "tag",
        "y_norm",
        "nis",
        "s_trace",
        "dx_mount_yaw",
        "dx_bg_z",
        "k_bg_z_vec",
        "num_bg_z_vec",
        "num_mount_yaw_vec",
        "num_bg_z_from_vel_vec",
        "num_bg_z_from_att_vec",
        "num_bg_z_from_bg_vec",
        "num_bg_z_from_sg_vec",
        "num_bg_z_from_mount_vec",
        "num_bg_z_from_lever_vec",
        "h_col_bg_z_norm",
        "h_col_mount_yaw_norm",
        "k_row_bg_z_norm",
        "info_bg_z",
        "info_mount_yaw",
        "post_corr_att_z_bg_z",
        "post_corr_mount_yaw_bg_z",
        "delta_cov_mount_yaw_bg_z_left",
        "delta_cov_mount_yaw_bg_z_right",
        "delta_cov_mount_yaw_bg_z_gain",
        "delta_cov_mount_yaw_bg_z_total",
    ]
    lines = [
        "# Mounting Yaw First-Update Geometry Compare",
        "",
        "比较 short-window 上 4 个关键 case 的首批 accepted `NHC/ODO` 更新，定位哪一段先把系统带进错误几何。",
        "",
        "## Tag-First Summary",
        "",
        *render_table(tag_first_df, overview_cols),
        "",
        f"## First {num_rows} Accepted Updates",
        "",
        *render_table(first_rows_df, detail_cols),
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare first accepted short-window NHC/ODO updates across geometry cases."
    )
    parser.add_argument("--base-short-config", type=Path, default=BASE_SHORT_CONFIG)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--num-rows", type=int, default=8)
    args = parser.parse_args()

    exe_path = REPO_ROOT / args.exe
    base_cfg = load_yaml(REPO_ROOT / args.base_short_config)
    outdir = REPO_ROOT / args.output_dir
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    first_rows_frames: list[pd.DataFrame] = []
    tag_first_rows: list[dict[str, Any]] = []
    manifest: list[dict[str, Any]] = []

    for spec in CASES:
        case_dir = cases_dir / spec.case_id
        prepared = prepare_case(base_cfg, case_dir, exe_path, spec)
        first_rows_frames.append(
            load_first_rows(spec.case_id, prepared["mechanism_path"], args.num_rows)
        )
        tag_first_rows.append(build_tag_first_summary(spec.case_id, prepared["mechanism_path"]))
        manifest.append(
            {
                "case_id": spec.case_id,
                "config_path": rel_from_root(prepared["config_path"], REPO_ROOT),
                "sol_path": rel_from_root(prepared["sol_path"], REPO_ROOT),
                "state_series_path": rel_from_root(prepared["state_series_path"], REPO_ROOT),
                "mechanism_path": rel_from_root(prepared["mechanism_path"], REPO_ROOT),
                "stdout_path": rel_from_root(prepared["stdout_path"], REPO_ROOT),
                "mechanism_mtime": mtime_text(prepared["mechanism_path"]),
            }
        )

    first_rows_df = pd.concat(first_rows_frames, ignore_index=True)
    tag_first_df = pd.DataFrame(tag_first_rows)

    first_rows_path = outdir / "first_rows_all_cases.csv"
    tag_first_path = outdir / "tag_first_summary.csv"
    summary_path = outdir / "summary.md"
    manifest_path = outdir / "manifest.json"

    first_rows_df.to_csv(first_rows_path, index=False)
    tag_first_df.to_csv(tag_first_path, index=False)
    summary_path.write_text(
        render_summary(tag_first_df, first_rows_df, args.num_rows), encoding="utf-8"
    )
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[done] first rows: {first_rows_path}")
    print(f"[done] tag-first summary: {tag_first_path}")
    print(f"[done] summary: {summary_path}")
    print(f"[done] manifest: {manifest_path}")


if __name__ == "__main__":
    main()
