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
    "output/debug_mounting_yaw_prewrite_remaining_gap_probe_r1_20260323"
)
SEED_COV_MATCH = -8.38405823453e-09
SEED_MOUNT_YAW0_MATCH_DEG = 0.112668941848824
SEED_BG_Z_MATCH_RAD_S = 4.393e-06
SEED_BG_Z_ATT_COV_MATCH = (
    4.42366798677e-10,
    1.58848537656e-11,
    -2.23371711006e-09,
)


CASES = [
    CaseSpec("baseline"),
    CaseSpec(
        "seed_cov_yaw0",
        debug_seed_mount_yaw_bgz_cov_before_first_nhc=SEED_COV_MATCH,
        init_mounting_yaw0_deg=SEED_MOUNT_YAW0_MATCH_DEG,
    ),
    CaseSpec(
        "seed_cov_yaw0_bgz",
        debug_seed_mount_yaw_bgz_cov_before_first_nhc=SEED_COV_MATCH,
        debug_seed_bg_z_before_first_nhc=SEED_BG_Z_MATCH_RAD_S,
        init_mounting_yaw0_deg=SEED_MOUNT_YAW0_MATCH_DEG,
    ),
    CaseSpec(
        "seed_cov_yaw0_attcov",
        debug_seed_mount_yaw_bgz_cov_before_first_nhc=SEED_COV_MATCH,
        debug_seed_bg_z_att_cov_before_first_nhc=SEED_BG_Z_ATT_COV_MATCH,
        init_mounting_yaw0_deg=SEED_MOUNT_YAW0_MATCH_DEG,
    ),
    CaseSpec(
        "seed_cov_yaw0_bgz_attcov",
        debug_seed_mount_yaw_bgz_cov_before_first_nhc=SEED_COV_MATCH,
        debug_seed_bg_z_before_first_nhc=SEED_BG_Z_MATCH_RAD_S,
        debug_seed_bg_z_att_cov_before_first_nhc=SEED_BG_Z_ATT_COV_MATCH,
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

    return {
        "case_id": spec.case_id,
        "seed_cov": (
            spec.debug_seed_mount_yaw_bgz_cov_before_first_nhc
            if spec.debug_seed_mount_yaw_bgz_cov_before_first_nhc is not None
            else float("nan")
        ),
        "seed_bg_z": (
            spec.debug_seed_bg_z_before_first_nhc
            if spec.debug_seed_bg_z_before_first_nhc is not None
            else float("nan")
        ),
        "seed_bg_z_att_cov": (
            list(spec.debug_seed_bg_z_att_cov_before_first_nhc)
            if spec.debug_seed_bg_z_att_cov_before_first_nhc is not None
            else []
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
        "nhc_bg_z_before": float(first_nhc["bg_z_before"]),
        "nhc_mount_yaw_before": float(first_nhc["mount_yaw_before"]),
        "nhc_prior_cov_att_z_bg_z": float(first_nhc["prior_cov_att_z_bg_z"]),
        "nhc_prior_cov_mount_yaw_bg_z": float(first_nhc["prior_cov_mount_yaw_bg_z"]),
        "nhc_p_row_bg_z_att_vec": parse_vec(first_nhc["p_row_bg_z_att_vec"]),
        "nhc_p_row_bg_z_mount_vec": parse_vec(first_nhc["p_row_bg_z_mount_vec"]),
        "nhc_h_att_z_vec": parse_vec(first_nhc["h_att_z_vec"]),
        "nhc_h_mount_yaw_vec": parse_vec(first_nhc["h_mount_yaw_vec"]),
        "nhc_num_bg_z_vec": num_bg,
        "nhc_num_bg_z_from_att_vec": parse_vec(first_nhc["num_bg_z_from_att_vec"]),
        "nhc_num_bg_z_from_mount_yaw_vec": parse_vec(first_nhc["num_bg_z_from_mount_yaw_vec"]),
        "nhc_k_mount_yaw_vec": k_mount,
        "nhc_q_total": dot(k_mount, num_bg),
        "nhc_dx_mount_yaw": float(first_nhc["dx_mount_yaw"]),
        "nhc_dx_bg_z": float(first_nhc["dx_bg_z"]),
        "nhc_info_mount_yaw": float(first_nhc["info_mount_yaw"]),
        "nhc_h_col_mount_yaw_norm": float(first_nhc["h_col_mount_yaw_norm"]),
        "post_nhc_odo_seq_idx": int(post_nhc_odo["seq_idx"]),
        "post_nhc_odo_t_meas": float(post_nhc_odo["t_meas"]),
        "post_nhc_odo_bg_z_before": float(post_nhc_odo["bg_z_before"]),
        "post_nhc_odo_mount_yaw_before": float(post_nhc_odo["mount_yaw_before"]),
        "post_nhc_odo_h_att_z_vec": parse_vec(post_nhc_odo["h_att_z_vec"]),
        "post_nhc_odo_h_mount_yaw_vec": parse_vec(post_nhc_odo["h_mount_yaw_vec"]),
        "post_nhc_odo_p_row_bg_z_att_vec": parse_vec(post_nhc_odo["p_row_bg_z_att_vec"]),
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


def compare_to_target(results: list[dict[str, Any]], case_id: str) -> dict[str, Any]:
    target = next(row for row in results if row["case_id"] == "odo_before_nhc")
    row = next(item for item in results if item["case_id"] == case_id)
    return {
        "case_id": case_id,
        "first_nhc_q_gap": row["nhc_q_total"] - target["nhc_q_total"],
        "first_nhc_info_mount_gap": row["nhc_info_mount_yaw"] - target["nhc_info_mount_yaw"],
        "first_nhc_h_mount_gap": row["nhc_h_col_mount_yaw_norm"]
        - target["nhc_h_col_mount_yaw_norm"],
        "first_post_odo_info_gap": row["post_nhc_odo_info_mount_yaw"]
        - target["post_nhc_odo_info_mount_yaw"],
        "first_post_odo_h_gap": row["post_nhc_odo_h_col_mount_yaw_norm"]
        - target["post_nhc_odo_h_col_mount_yaw_norm"],
        "short_window_yaw_final_gap_deg": row["yaw_final_total_deg"] - target["yaw_final_total_deg"],
        "short_window_bg_z_final_gap_degh": row["bg_z_final_degh"] - target["bg_z_final_degh"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Probe the remaining gap between seed_cov+yaw0 and odo_before_nhc."
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

    comparisons = [
        compare_to_target(results, "seed_cov_yaw0"),
        compare_to_target(results, "seed_cov_yaw0_bgz"),
        compare_to_target(results, "seed_cov_yaw0_attcov"),
        compare_to_target(results, "seed_cov_yaw0_bgz_attcov"),
    ]

    report = {
        "base_config": rel_from_root(REPO_ROOT / BASE_SHORT_CONFIG, REPO_ROOT),
        "exe": rel_from_root(exe_path, REPO_ROOT),
        "seed_cov_match": SEED_COV_MATCH,
        "seed_mount_yaw0_match_deg": SEED_MOUNT_YAW0_MATCH_DEG,
        "seed_bg_z_match_rad_s": SEED_BG_Z_MATCH_RAD_S,
        "seed_bg_z_att_cov_match": list(SEED_BG_Z_ATT_COV_MATCH),
        "cases": results,
        "comparisons_vs_odo_before_nhc": comparisons,
    }

    summary_lines = [
        "# Remaining Gap Probe",
        "",
        "Goal: test whether the residual gap from `seed_cov+yaw0` to `odo_before_nhc` is explained by",
        "`bg_z` nominal, `P(bg_z,att_*)`, or both when injected at the same pre-first-NHC hook.",
        "",
        "## First NHC",
        "",
        *render_table(
            results,
            [
                "case_id",
                "seed_bg_z",
                "seed_bg_z_att_cov",
                "nhc_bg_z_before",
                "nhc_p_row_bg_z_att_vec",
                "nhc_h_mount_yaw_vec",
                "nhc_q_total",
                "nhc_info_mount_yaw",
                "nhc_dx_mount_yaw",
            ],
        ),
        "",
        "## First ODO After First NHC",
        "",
        *render_table(
            results,
            [
                "case_id",
                "post_nhc_odo_h_att_z_vec",
                "post_nhc_odo_h_mount_yaw_vec",
                "post_nhc_odo_info_mount_yaw",
                "post_nhc_odo_h_col_mount_yaw_norm",
                "post_nhc_odo_dx_mount_yaw",
                "post_nhc_odo_dx_bg_z",
            ],
        ),
        "",
        "## Gap To odo_before_nhc",
        "",
        *render_table(
            comparisons,
            [
                "case_id",
                "first_nhc_q_gap",
                "first_nhc_info_mount_gap",
                "first_nhc_h_mount_gap",
                "first_post_odo_info_gap",
                "first_post_odo_h_gap",
                "short_window_yaw_final_gap_deg",
                "short_window_bg_z_final_gap_degh",
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
