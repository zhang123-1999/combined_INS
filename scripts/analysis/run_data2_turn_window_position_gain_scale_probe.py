import argparse
import copy
import datetime as dt
import json
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
from scripts.analysis.run_data2_state_sanity_matrix import (
    build_truth_reference,
    json_safe,
    reset_directory,
    run_command,
)
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (
    MIN_SPEED_M_S,
    MIN_WINDOW_SEPARATION_S,
    POS_PATH_DEFAULT,
    ROLLING_WINDOW_S,
    TOP_K_WINDOWS,
    analyze_case_updates,
    build_diag_window_std_summary,
    load_pos_dataframe,
    select_turn_windows,
)


EXP_ID_DEFAULT = "EXP-20260318-data2-turn-window-position-gain-scale-r1"
SOURCE_CASE_CONFIG_DEFAULT = Path(
    "output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml"
)
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_window_position_gain_scale_probe")
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")


@dataclass(frozen=True)
class VariantSpec:
    case_id: str
    label: str
    order_idx: int
    position_gain_scale: float


def variant_specs() -> list[VariantSpec]:
    return [
        VariantSpec("baseline", "pos_gain_1p0", 0, 1.0),
        VariantSpec("pos_gain_0p5", "pos_gain_0p5", 1, 0.5),
        VariantSpec("pos_gain_0p25", "pos_gain_0p25", 2, 0.25),
        VariantSpec("pos_gain_0p0", "pos_gain_0p0", 3, 0.0),
    ]


def build_variant_config(spec: VariantSpec, source_cfg_path: Path, case_dir: Path) -> tuple[dict[str, Any], Path]:
    cfg = copy.deepcopy(load_yaml(source_cfg_path))
    fusion = cfg.setdefault("fusion", {})
    fusion["gnss_pos_position_gain_scale"] = float(spec.position_gain_scale)
    fusion["output_path"] = rel_from_root(case_dir / f"SOL_{spec.case_id}.txt", REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(case_dir / f"state_series_{spec.case_id}.csv", REPO_ROOT)
    fusion["first_update_debug_output_path"] = rel_from_root(
        case_dir / f"first_update_{spec.case_id}.csv", REPO_ROOT
    )
    fusion["gnss_update_debug_output_path"] = rel_from_root(
        case_dir / f"gnss_updates_{spec.case_id}.csv", REPO_ROOT
    )
    cfg_path = case_dir / f"config_{spec.case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg, cfg_path


def run_variant(spec: VariantSpec, source_cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    cfg, cfg_path = build_variant_config(spec, source_cfg_path, case_dir)
    stdout_path = case_dir / f"{spec.case_id}.stdout.txt"
    diag_path = case_dir / f"DIAG_{spec.case_id}.txt"
    sol_path = case_dir / f"SOL_{spec.case_id}.txt"
    state_series_path = case_dir / f"state_series_{spec.case_id}.csv"
    first_update_path = case_dir / f"first_update_{spec.case_id}.csv"
    gnss_update_path = case_dir / f"gnss_updates_{spec.case_id}.csv"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    for required_path in [sol_path, state_series_path, first_update_path, gnss_update_path]:
        if not required_path.exists():
            raise RuntimeError(f"missing solver artifact for {spec.case_id}: {required_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after {spec.case_id}")
    diag_path.write_text(root_diag.read_text(encoding="utf-8"), encoding="utf-8")
    return {
        "case_id": spec.case_id,
        "label": spec.label,
        "r_scale": float(spec.order_idx),
        "position_gain_scale": float(spec.position_gain_scale),
        "source_config_path": rel_from_root(source_cfg_path, REPO_ROOT),
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "first_update_path": rel_from_root(first_update_path, REPO_ROOT),
        "gnss_update_path": rel_from_root(gnss_update_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "gnss_path": str(cfg["fusion"]["gnss_path"]),
        "config_mtime": dt.datetime.fromtimestamp(cfg_path.stat().st_mtime).isoformat(timespec="seconds"),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "first_update_mtime": dt.datetime.fromtimestamp(first_update_path.stat().st_mtime).isoformat(timespec="seconds"),
        "gnss_update_mtime": dt.datetime.fromtimestamp(gnss_update_path.stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(diag_path.stat().st_mtime).isoformat(timespec="seconds"),
    }


def write_summary(output_path: Path, case_meta_rows: list[dict[str, Any]], case_summary_df: pd.DataFrame) -> None:
    merged = case_summary_df.merge(
        pd.DataFrame(case_meta_rows)[["case_id", "label", "position_gain_scale"]],
        on=["case_id", "label"],
        how="left",
    )
    lines: list[str] = []
    lines.append("# data2 turn-window position gain-scale probe")
    lines.append("")
    lines.append("## Case summary")
    for row in merged.itertuples(index=False):
        lines.append(
            f"- `{row.case_id}` (`pos_gain_scale={row.position_gain_scale:.2f}`): "
            f"`lever_share_x/y/z={row.mean_lever_share_x:.4f}/{row.mean_lever_share_y:.4f}/{row.mean_lever_share_z:.4f}`, "
            f"`pos_share_x/y/z={row.mean_pos_share_x:.4f}/{row.mean_pos_share_y:.4f}/{row.mean_pos_share_z:.4f}`, "
            f"`k_lever_x/y/z={row.mean_k_lever_x_norm:.4f}/{row.mean_k_lever_y_norm:.4f}/{row.mean_k_lever_z_norm:.4f}`, "
            f"`lever_x/y/z_error_improve={row.mean_lever_x_error_improve_m:.6f}/{row.mean_lever_y_error_improve_m:.6f}/{row.mean_lever_z_error_improve_m:.6f}` m."
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe GNSS_POS position-gain scaling in turn windows.")
    parser.add_argument("--source-config", type=Path, default=SOURCE_CASE_CONFIG_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    source_cfg_path = (REPO_ROOT / args.source_config).resolve()
    exe_path = (REPO_ROOT / args.exe).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()

    if not exe_path.exists():
        raise RuntimeError(f"missing executable: {exe_path}")
    if not source_cfg_path.exists():
        raise RuntimeError(f"missing source config: {source_cfg_path}")

    reset_directory(output_dir)
    artifacts_dir = output_dir / "artifacts" / "cases"
    ensure_dir(artifacts_dir)

    base_cfg = load_yaml(source_cfg_path)
    truth_reference = build_truth_reference(base_cfg)
    truth_lever = np.array(
        [
            float(truth_reference["states"]["gnss_lever_x"]["reference_value_internal"]),
            float(truth_reference["states"]["gnss_lever_y"]["reference_value_internal"]),
            float(truth_reference["states"]["gnss_lever_z"]["reference_value_internal"]),
        ],
        dtype=float,
    )

    pos_df = load_pos_dataframe(pos_path)
    windows_df, turn_series_df = select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=ROLLING_WINDOW_S,
        top_k=TOP_K_WINDOWS,
        min_speed_m_s=MIN_SPEED_M_S,
        min_separation_s=MIN_WINDOW_SEPARATION_S,
    )
    windows_path = output_dir / "turn_windows.csv"
    windows_df.to_csv(windows_path, index=False, encoding="utf-8-sig")

    case_rows: list[dict[str, Any]] = []
    all_updates: list[pd.DataFrame] = []
    all_window_summaries: list[pd.DataFrame] = []
    all_case_summaries: list[pd.DataFrame] = []

    for spec in variant_specs():
        case_dir = artifacts_dir / spec.case_id
        ensure_dir(case_dir)
        case_meta = run_variant(spec, source_cfg_path, case_dir, exe_path)
        case_rows.append(case_meta)
        per_update_df, window_summary_df, case_summary_df = analyze_case_updates(
            case_meta=case_meta,
            windows_df=windows_df,
            turn_series_df=turn_series_df,
            truth_lever=truth_lever,
        )
        all_updates.append(per_update_df)
        all_window_summaries.append(window_summary_df)
        all_case_summaries.append(case_summary_df)

    update_summary_df = pd.concat(all_updates, ignore_index=True)
    window_summary_df = pd.concat(all_window_summaries, ignore_index=True)
    case_summary_df = pd.concat(all_case_summaries, ignore_index=True).sort_values(by="r_scale").reset_index(drop=True)
    diag_window_std_df = build_diag_window_std_summary(case_rows, windows_df)

    per_update_path = output_dir / "turn_window_update_breakdown.csv"
    window_summary_path = output_dir / "turn_window_summary.csv"
    case_summary_path = output_dir / "case_update_summary.csv"
    diag_window_std_path = output_dir / "diag_turn_window_std_summary.csv"
    summary_path = output_dir / "summary.md"
    manifest_path = output_dir / "manifest.json"

    update_summary_df.to_csv(per_update_path, index=False, encoding="utf-8-sig")
    window_summary_df.to_csv(window_summary_path, index=False, encoding="utf-8-sig")
    case_summary_df.to_csv(case_summary_path, index=False, encoding="utf-8-sig")
    diag_window_std_df.to_csv(diag_window_std_path, index=False, encoding="utf-8-sig")
    write_summary(summary_path, case_rows, case_summary_df)

    manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_config": rel_from_root(source_cfg_path, REPO_ROOT),
        "exe": rel_from_root(exe_path, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "truth_lever_internal_m": truth_lever.tolist(),
        "variants": [
            {
                "case_id": spec.case_id,
                "label": spec.label,
                "position_gain_scale": float(spec.position_gain_scale),
            }
            for spec in variant_specs()
        ],
        "artifacts": {
            "turn_windows_csv": rel_from_root(windows_path, REPO_ROOT),
            "turn_window_update_breakdown_csv": rel_from_root(per_update_path, REPO_ROOT),
            "turn_window_summary_csv": rel_from_root(window_summary_path, REPO_ROOT),
            "case_update_summary_csv": rel_from_root(case_summary_path, REPO_ROOT),
            "diag_turn_window_std_summary_csv": rel_from_root(diag_window_std_path, REPO_ROOT),
            "summary_md": rel_from_root(summary_path, REPO_ROOT),
        },
        "case_rows": case_rows,
    }
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
