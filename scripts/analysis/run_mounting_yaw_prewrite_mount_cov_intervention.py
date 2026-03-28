import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

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
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import (  # noqa: E402
    BASE_SHORT_CONFIG,
    EXE_DEFAULT,
    CaseSpec,
    build_case_config,
    mtime_text,
    run_solver,
    summarize_state_series,
)


OUTPUT_DIR_DEFAULT = Path(
    "output/debug_mounting_yaw_prewrite_mount_cov_intervention_r1_20260323"
)
SEED_COV_MATCH = -8.38405823453e-09
SEED_MOUNT_YAW0_MATCH_DEG = 0.112668941848824

CASES = [
    CaseSpec("baseline"),
    CaseSpec(
        "seed_mount_cov_match",
        debug_seed_mount_yaw_bgz_cov_before_first_nhc=SEED_COV_MATCH,
    ),
    CaseSpec(
        "seed_mount_cov_yaw0_match",
        debug_seed_mount_yaw_bgz_cov_before_first_nhc=SEED_COV_MATCH,
        init_mounting_yaw0_deg=SEED_MOUNT_YAW0_MATCH_DEG,
    ),
    CaseSpec("odo_before_nhc", debug_run_odo_before_nhc=True),
]


def parse_vec(text: str) -> list[float]:
    stripped = str(text).strip()
    if stripped == "[]" or not stripped:
        return []
    if stripped[0] == "[" and stripped[-1] == "]":
        stripped = stripped[1:-1]
    if not stripped:
        return []
    return [float(part) for part in stripped.split(";") if part]


def dot(lhs: list[float], rhs: list[float]) -> float:
    return sum(a * b for a, b in zip(lhs, rhs))


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


def extract_case_metrics(prepared: dict[str, Any], spec: CaseSpec) -> dict[str, Any]:
    mechanism_df = pd.read_csv(prepared["mechanism_path"]).copy()
    mechanism_df.insert(0, "seq_idx", range(1, len(mechanism_df) + 1))
    first_nhc = mechanism_df[mechanism_df["tag"] == "NHC"].iloc[0]
    post_nhc_odo = mechanism_df[
        (mechanism_df["tag"] == "ODO") & (mechanism_df["seq_idx"] > int(first_nhc["seq_idx"]))
    ].iloc[0]

    state_summary = summarize_state_series(prepared["state_series_path"])
    consistency = parse_consistency_summary(prepared["stdout_text"])
    nhc_consistency = consistency.get("NHC", {})
    odo_consistency = consistency.get("ODO", {})

    k_mount = parse_vec(first_nhc["k_mount_yaw_vec"])
    num_bg = parse_vec(first_nhc["num_bg_z_vec"])
    mount_prior_vec = parse_vec(first_nhc["p_row_bg_z_mount_vec"])
    mount_term_vec = parse_vec(first_nhc["num_bg_z_from_mount_vec"])
    mount_yaw_term_vec = parse_vec(first_nhc["num_bg_z_from_mount_yaw_vec"])
    h_mount_yaw = parse_vec(first_nhc["h_mount_yaw_vec"])
    q_total = dot(k_mount, num_bg)

    return {
        "case_id": spec.case_id,
        "seed_cov": (
            spec.debug_seed_mount_yaw_bgz_cov_before_first_nhc
            if spec.debug_seed_mount_yaw_bgz_cov_before_first_nhc is not None
            else float("nan")
        ),
        "init_mounting_yaw0_deg": (
            spec.init_mounting_yaw0_deg
            if spec.init_mounting_yaw0_deg is not None
            else float("nan")
        ),
        "debug_run_odo_before_nhc": spec.debug_run_odo_before_nhc,
        "config_path": rel_from_root(prepared["config_path"], REPO_ROOT),
        "sol_path": rel_from_root(prepared["sol_path"], REPO_ROOT),
        "state_series_path": rel_from_root(prepared["state_series_path"], REPO_ROOT),
        "mechanism_path": rel_from_root(prepared["mechanism_path"], REPO_ROOT),
        "stdout_path": rel_from_root(prepared["stdout_path"], REPO_ROOT),
        "config_mtime": mtime_text(prepared["config_path"]),
        "sol_mtime": mtime_text(prepared["sol_path"]),
        "state_series_mtime": mtime_text(prepared["state_series_path"]),
        "mechanism_mtime": mtime_text(prepared["mechanism_path"]),
        "nhc_seq_idx": int(first_nhc["seq_idx"]),
        "nhc_t_meas": float(first_nhc["t_meas"]),
        "nhc_prior_cov_mount_yaw_bg_z": float(first_nhc["prior_cov_mount_yaw_bg_z"]),
        "nhc_prior_corr_mount_yaw_bg_z": float(first_nhc["prior_corr_mount_yaw_bg_z"]),
        "nhc_num_bg_z_vec": num_bg,
        "nhc_num_bg_z_from_mount_vec": mount_term_vec,
        "nhc_num_bg_z_from_mount_yaw_vec": mount_yaw_term_vec,
        "nhc_p_row_bg_z_mount_vec": mount_prior_vec,
        "nhc_h_mount_yaw_vec": h_mount_yaw,
        "nhc_k_mount_yaw_vec": k_mount,
        "nhc_q_total": q_total,
        "nhc_delta_cov_mount_yaw_bg_z_total": float(first_nhc["delta_cov_mount_yaw_bg_z_total"]),
        "nhc_dx_mount_yaw": float(first_nhc["dx_mount_yaw"]),
        "nhc_dx_bg_z": float(first_nhc["dx_bg_z"]),
        "post_nhc_odo_seq_idx": int(post_nhc_odo["seq_idx"]),
        "post_nhc_odo_t_meas": float(post_nhc_odo["t_meas"]),
        "post_nhc_odo_h_col_mount_yaw_norm": float(post_nhc_odo["h_col_mount_yaw_norm"]),
        "post_nhc_odo_info_mount_yaw": float(post_nhc_odo["info_mount_yaw"]),
        "post_nhc_odo_dx_mount_yaw": float(post_nhc_odo["dx_mount_yaw"]),
        "post_nhc_odo_dx_bg_z": float(post_nhc_odo["dx_bg_z"]),
        "odo_accept_ratio": float(odo_consistency.get("accept_ratio", float("nan"))),
        "nhc_accept_ratio": float(nhc_consistency.get("accept_ratio", float("nan"))),
        **state_summary,
    }


def fmt(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "NA"
    if isinstance(value, list):
        return "[" + "; ".join(f"{float(v):.12g}" for v in value) + "]"
    try:
        v = float(value)
    except Exception:
        return str(value)
    if not math.isfinite(v):
        return "NA"
    return f"{v:.12g}"


def render_table(rows: list[dict[str, Any]], columns: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row[col]) for col in columns) + " |")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare a controlled P(bg_z,mount_yaw) prewrite against odo_before_nhc."
    )
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    args = parser.parse_args()

    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)

    base_cfg = load_yaml(REPO_ROOT / BASE_SHORT_CONFIG)
    exe_path = REPO_ROOT / args.exe
    cases_dir = outdir / "cases"
    ensure_dir(cases_dir)

    results: list[dict[str, Any]] = []
    for spec in CASES:
        case_dir = cases_dir / spec.case_id
        prepared = prepare_case(base_cfg, case_dir, exe_path, spec)
        results.append(extract_case_metrics(prepared, spec))

    baseline = next(row for row in results if row["case_id"] == "baseline")
    seeded = next(row for row in results if row["case_id"] == "seed_mount_cov_match")
    seeded_with_yaw0 = next(
        row for row in results if row["case_id"] == "seed_mount_cov_yaw0_match"
    )
    odo_before = next(row for row in results if row["case_id"] == "odo_before_nhc")

    report = {
        "base_config": rel_from_root(REPO_ROOT / BASE_SHORT_CONFIG, REPO_ROOT),
        "exe": rel_from_root(exe_path, REPO_ROOT),
        "seed_cov_match": SEED_COV_MATCH,
        "seed_mount_yaw0_match_deg": SEED_MOUNT_YAW0_MATCH_DEG,
        "cases": results,
        "comparisons": {
            "seed_matches_target_prior_cov": seeded["nhc_prior_cov_mount_yaw_bg_z"] - SEED_COV_MATCH,
            "seed_vs_odo_before_q_gap": seeded["nhc_q_total"] - odo_before["nhc_q_total"],
            "seed_vs_odo_before_mount_term_gap": [
                a - b
                for a, b in zip(
                    seeded["nhc_num_bg_z_from_mount_yaw_vec"],
                    odo_before["nhc_num_bg_z_from_mount_yaw_vec"],
                )
            ],
            "seed_vs_odo_before_post_nhc_odo_info_gap": (
                seeded["post_nhc_odo_info_mount_yaw"] - odo_before["post_nhc_odo_info_mount_yaw"]
            ),
            "seed_vs_baseline_q_ratio": (
                seeded["nhc_q_total"] / baseline["nhc_q_total"]
                if baseline["nhc_q_total"] != 0.0
                else float("nan")
            ),
            "seed_yaw0_vs_odo_before_post_nhc_odo_info_gap": (
                seeded_with_yaw0["post_nhc_odo_info_mount_yaw"]
                - odo_before["post_nhc_odo_info_mount_yaw"]
            ),
            "seed_yaw0_vs_odo_before_post_nhc_odo_h_gap": (
                seeded_with_yaw0["post_nhc_odo_h_col_mount_yaw_norm"]
                - odo_before["post_nhc_odo_h_col_mount_yaw_norm"]
            ),
        },
    }

    summary_lines = [
        "# Mount Covariance Prewrite Intervention",
        "",
        "目标：检验仅在首个 `NHC` 前一次性写入 `P(bg_z,mount_yaw)`，是否足以复制 `odo_before_nhc` 的首拍缓解效果。",
        "",
        "## First NHC",
        "",
        *render_table(
            results,
            [
                "case_id",
                "seed_cov",
                "init_mounting_yaw0_deg",
                "nhc_prior_cov_mount_yaw_bg_z",
                "nhc_num_bg_z_from_mount_yaw_vec",
                "nhc_q_total",
                "nhc_delta_cov_mount_yaw_bg_z_total",
                "nhc_dx_mount_yaw",
                "nhc_dx_bg_z",
            ],
        ),
        "",
        "## First ODO After First NHC",
        "",
        *render_table(
            results,
            [
                "case_id",
                "post_nhc_odo_seq_idx",
                "post_nhc_odo_h_col_mount_yaw_norm",
                "post_nhc_odo_info_mount_yaw",
                "post_nhc_odo_dx_mount_yaw",
                "post_nhc_odo_dx_bg_z",
            ],
        ),
        "",
        "## Short-window End State",
        "",
        *render_table(
            results,
            [
                "case_id",
                "odo_accept_ratio",
                "nhc_accept_ratio",
                "yaw_final_total_deg",
                "bg_z_final_degh",
                "yaw_after_200_peak_to_peak_deg",
            ],
        ),
        "",
        "## Key Checks",
        "",
        f"- seeded case target prior gap = `{fmt(report['comparisons']['seed_matches_target_prior_cov'])}`.",
        f"- seeded vs `odo_before_nhc` first-NHC `q_total` gap = `{fmt(report['comparisons']['seed_vs_odo_before_q_gap'])}`.",
        f"- seeded vs baseline `q_total` ratio = `{fmt(report['comparisons']['seed_vs_baseline_q_ratio'])}`.",
        f"- seeded vs `odo_before_nhc` first post-NHC ODO `info_mount_yaw` gap = `{fmt(report['comparisons']['seed_vs_odo_before_post_nhc_odo_info_gap'])}`.",
        f"- seeded+`yaw0_match` vs `odo_before_nhc` first post-NHC ODO `info_mount_yaw` gap = `{fmt(report['comparisons']['seed_yaw0_vs_odo_before_post_nhc_odo_info_gap'])}`.",
        f"- seeded+`yaw0_match` vs `odo_before_nhc` first post-NHC ODO `h_col_mount_yaw_norm` gap = `{fmt(report['comparisons']['seed_yaw0_vs_odo_before_post_nhc_odo_h_gap'])}`.",
    ]
    summary_text = "\n".join(summary_lines) + "\n"

    (outdir / "summary.md").write_text(summary_text, encoding="utf-8")
    (outdir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    print(f"[done] summary: {outdir / 'summary.md'}")
    print(f"[done] report: {outdir / 'report.json'}")


if __name__ == "__main__":
    main()
