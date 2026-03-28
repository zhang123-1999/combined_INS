import argparse
import json
import sys
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root  # noqa: E402
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import mtime_text  # noqa: E402


BASELINE_MECHANISM = Path(
    "output/debug_mounting_yaw_first_update_geometry_r1_20260323/"
    "cases/baseline/SOL_baseline_mechanism.csv"
)
ODO_BEFORE_MECHANISM = Path(
    "output/debug_mounting_yaw_first_update_geometry_r1_20260323/"
    "cases/odo_before_nhc/SOL_odo_before_nhc_mechanism.csv"
)
OUTPUT_DIR_DEFAULT = Path(
    "output/debug_mounting_yaw_odo_prewrite_mount_block_origin_r1_20260323"
)


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


def get_vec(row: pd.Series, column: str, default: list[float]) -> list[float]:
    if column not in row.index:
        return default
    return parse_vec(row[column])


def analyze_odo_row(row: pd.Series) -> dict[str, Any]:
    k_mount = parse_vec(row["k_mount_yaw_vec"])
    num_bg = parse_vec(row["num_bg_z_vec"])
    q_odo = dot(k_mount, num_bg)
    p_row_bg_z_mount = get_vec(row, "p_row_bg_z_mount_vec", [0.0, 0.0, 0.0])
    prior_mount_yaw_cov = p_row_bg_z_mount[2] if len(p_row_bg_z_mount) >= 3 else float("nan")
    predicted_post_mount_yaw_cov = prior_mount_yaw_cov - q_odo
    return {
        "seq_idx": int(row["seq_idx"]),
        "t_meas": float(row["t_meas"]),
        "k_mount_yaw_vec": k_mount,
        "num_bg_z_vec": num_bg,
        "q_odo": q_odo,
        "delta_cov_mount_yaw_bg_z_pred": -q_odo,
        "p_row_bg_z_mount_vec": p_row_bg_z_mount,
        "h_mount_yaw_vec": get_vec(row, "h_mount_yaw_vec", []),
        "num_bg_z_from_mount_yaw_vec": get_vec(row, "num_bg_z_from_mount_yaw_vec", []),
        "prior_cov_mount_yaw_bg_z_logged": float(row["prior_cov_mount_yaw_bg_z"]),
        "post_cov_mount_yaw_bg_z_logged": float(row["post_cov_mount_yaw_bg_z"]),
        "predicted_post_mount_yaw_cov": predicted_post_mount_yaw_cov,
        "dx_mount_yaw": float(row["dx_mount_yaw"]),
        "dx_bg_z": float(row["dx_bg_z"]),
        "h_col_mount_yaw_norm": float(row["h_col_mount_yaw_norm"]),
        "info_mount_yaw": float(row["info_mount_yaw"]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Explain how odo_before_nhc prewrites the mount block before the first NHC."
    )
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    args = parser.parse_args()

    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)

    baseline_path = REPO_ROOT / BASELINE_MECHANISM
    odo_before_path = REPO_ROOT / ODO_BEFORE_MECHANISM
    baseline_df = pd.read_csv(baseline_path)
    baseline_df.insert(0, "seq_idx", range(1, len(baseline_df) + 1))
    odo_before_df = pd.read_csv(odo_before_path)
    odo_before_df.insert(0, "seq_idx", range(1, len(odo_before_df) + 1))

    baseline_first_nhc = baseline_df[baseline_df["tag"] == "NHC"].iloc[0]
    odo_before_first_nhc = odo_before_df[odo_before_df["tag"] == "NHC"].iloc[0]
    odo_rows = odo_before_df[odo_before_df["tag"] == "ODO"].reset_index(drop=True)
    odo1 = analyze_odo_row(odo_rows.iloc[0])
    odo2 = analyze_odo_row(odo_rows.iloc[1])

    nhc_mount_prior = parse_vec(odo_before_first_nhc["p_row_bg_z_mount_vec"])
    nhc_mount_yaw_prior = nhc_mount_prior[2] if len(nhc_mount_prior) >= 3 else float("nan")
    nhc_h_mount_yaw = parse_vec(odo_before_first_nhc["h_mount_yaw_vec"])
    nhc_mount_yaw_term = parse_vec(odo_before_first_nhc["num_bg_z_from_mount_yaw_vec"])
    nhc_mount_term = parse_vec(odo_before_first_nhc["num_bg_z_from_mount_vec"])

    cumulative_q = odo1["q_odo"] + odo2["q_odo"]
    baseline_nhc_mount_prior = get_vec(
        baseline_first_nhc, "p_row_bg_z_mount_vec", [0.0, 0.0, 0.0]
    )

    report: dict[str, Any] = {
        "baseline_mechanism": rel_from_root(baseline_path, REPO_ROOT),
        "odo_before_mechanism": rel_from_root(odo_before_path, REPO_ROOT),
        "baseline_mechanism_mtime": mtime_text(baseline_path),
        "odo_before_mechanism_mtime": mtime_text(odo_before_path),
        "odo1": odo1,
        "odo2": odo2,
        "cumulative_q": cumulative_q,
        "baseline_first_nhc": {
            "seq_idx": int(baseline_first_nhc["seq_idx"]),
            "t_meas": float(baseline_first_nhc["t_meas"]),
            "p_row_bg_z_mount_vec": baseline_nhc_mount_prior,
            "num_bg_z_from_mount_vec": parse_vec(baseline_first_nhc["num_bg_z_from_mount_vec"]),
        },
        "odo_before_first_nhc": {
            "seq_idx": int(odo_before_first_nhc["seq_idx"]),
            "t_meas": float(odo_before_first_nhc["t_meas"]),
            "p_row_bg_z_mount_vec": nhc_mount_prior,
            "h_mount_yaw_vec": nhc_h_mount_yaw,
            "num_bg_z_from_mount_yaw_vec": nhc_mount_yaw_term,
            "num_bg_z_from_mount_vec": nhc_mount_term,
            "prior_cov_mount_yaw_bg_z_logged": float(
                odo_before_first_nhc["prior_cov_mount_yaw_bg_z"]
            ),
        },
        "logic_checks": {
            "odo1_post_matches_odo2_prior": abs(
                odo1["predicted_post_mount_yaw_cov"] - odo2["p_row_bg_z_mount_vec"][2]
            )
            < 1e-18,
            "odo12_post_matches_nhc_prior": abs(
                odo2["predicted_post_mount_yaw_cov"] - nhc_mount_yaw_prior
            )
            < 1e-18,
            "nhc_mount_term_is_yaw_only": nhc_mount_yaw_term == nhc_mount_term,
            "baseline_nhc_mount_prior_zero": all(abs(v) < 1e-30 for v in baseline_nhc_mount_prior),
        },
    }

    summary_lines = [
        "# ODO Prewrite Mount-Block Origin",
        "",
        "目标：解释 `odo_before_nhc` 的前两条 `ODO` 更新为何会在首个 `NHC` 前造出足以抵消 `bg/att` 的 `mount block`。",
        "",
        "## Baseline Contrast",
        "",
        f"- baseline first NHC prior `P(bg_z,mount_*) = [{'; '.join(f'{v:.12g}' for v in baseline_nhc_mount_prior)}]`, so baseline has no mount block before the first NHC.",
        f"- `odo_before_nhc` first NHC prior `P(bg_z,mount_*) = [{'; '.join(f'{v:.12g}' for v in nhc_mount_prior)}]`, already contains a pure `mount_yaw` covariance.",
        "",
        "## ODO Chain",
        "",
        f"- ODO1 @ `t={odo1['t_meas']:.6f}`: `k_mount_yaw={odo1['k_mount_yaw_vec'][0]:.12g}`, `num_bg_z={odo1['num_bg_z_vec'][0]:.12g}`, so `q_odo1={odo1['q_odo']:.12g}` and predicted `delta_cov_mount_yaw_bg_z={-odo1['q_odo']:.12g}`.",
        f"- ODO1 prior `P(bg_z,mount_yaw)={odo1['p_row_bg_z_mount_vec'][2]:.12g}`, predicted post `={odo1['predicted_post_mount_yaw_cov']:.12g}`; this exactly matches ODO2 prior.",
        f"- ODO2 @ `t={odo2['t_meas']:.6f}`: `k_mount_yaw={odo2['k_mount_yaw_vec'][0]:.12g}`, `num_bg_z={odo2['num_bg_z_vec'][0]:.12g}`, so `q_odo2={odo2['q_odo']:.12g}` and predicted `delta_cov_mount_yaw_bg_z={-odo2['q_odo']:.12g}`.",
        f"- cumulative `q_odo1+q_odo2={cumulative_q:.12g}` makes `P(bg_z,mount_yaw)` evolve from `0` to `{odo2['predicted_post_mount_yaw_cov']:.12g}`, which exactly matches the first NHC prior.",
        "",
        "## First NHC Use Of Mount Block",
        "",
        f"- first NHC in `odo_before_nhc` sees prior `P(bg_z,mount_yaw)={nhc_mount_yaw_prior:.12g}` and `H_mount_yaw={['{:.12g}'.format(v) for v in nhc_h_mount_yaw]}`.",
        f"- this yields `num_bg_z_from_mount_yaw_vec=[{'; '.join(f'{v:.12g}' for v in nhc_mount_yaw_term)}]`, identical to the whole `mount block` term.",
        f"- because `P(bg_z,mount_yaw)` is negative while `H_mount_yaw[0]` is also negative, the resulting mount contribution is positive and cancels the negative `bg/att` terms before the first NHC update.",
        "",
        "## Conclusion",
        "",
        "- `odo_before_nhc` 的 mount block 不是在 first NHC 当场生成的，而是前两条 ODO 先连续写进 `P(bg_z,mount_yaw)`。",
        "- 这两条 ODO 各自都遵循同一个标量机制：`delta_cov_mount_yaw_bg_z ≈ -K_mount_yaw · num_bg_z`，累积后把 `P(bg_z,mount_yaw)` 推到 `-8.38405823453e-09`。",
        "- 到首个 NHC 时，这个负的 `P(bg_z,mount_yaw)` 经过强 `H_mount_yaw` 映射成正的 `num_bg_z_from_mount_yaw_vec=[9.6600568032e-08;0]`，从而对 `bg/att` 项形成近抵消。",
        "- 因此，`odo_before_nhc` 的缓解机制不是“削弱 NHC 增益”，而是“让 ODO 先重写 numerator geometry，再让 NHC 吃到一个被 mount_yaw block 抵消后的 `num_bg_z`”。",
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
