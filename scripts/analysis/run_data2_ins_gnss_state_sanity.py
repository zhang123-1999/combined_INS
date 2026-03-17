from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import (
    ensure_dir,
    interp_truth,
    load_sol,
    load_truth_ecef,
    load_yaml,
    rel_from_root,
    save_yaml,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    STATE_META,
    base_p0_diag_from_config,
    build_truth_reference,
    case_release_state_name,
    default_ablation_flags,
    dynamic_behavior_label,
    family_anchored_process_internal,
    family_target_std_human,
    format_metric,
    get_group_instability_internal,
    get_group_vector_internal,
    json_safe,
    overall_label,
    plot_heatmap,
    plot_state_comparison,
    reset_directory,
    run_command,
    static_behavior_label,
)
from scripts.analysis.run_data2_rtk_outage_eval import compute_global_metrics


STATE_ORDER_INS_GNSS = [
    "ba_x",
    "ba_y",
    "ba_z",
    "bg_x",
    "bg_y",
    "bg_z",
    "sg_x",
    "sg_y",
    "sg_z",
    "sa_x",
    "sa_y",
    "sa_z",
    "gnss_lever_x",
    "gnss_lever_y",
    "gnss_lever_z",
]


def requested_case_ids(raw_cases: list[str] | None) -> list[str]:
    all_cases = ["truth_anchor_all_non_pva"] + [f"release_{state_name}" for state_name in STATE_ORDER_INS_GNSS]
    if not raw_cases:
        return all_cases
    unique: list[str] = []
    seen: set[str] = set()
    for case_id in raw_cases:
        if case_id not in all_cases:
            raise ValueError(f"unsupported case_id: {case_id}")
        if case_id not in seen:
            unique.append(case_id)
            seen.add(case_id)
    if "truth_anchor_all_non_pva" not in seen:
        unique.insert(0, "truth_anchor_all_non_pva")
    return unique


def case_sort_key(case_id: str) -> tuple[int, int]:
    if case_id == "truth_anchor_all_non_pva":
        return (0, -1)
    state_name = case_release_state_name(case_id)
    return (1, STATE_ORDER_INS_GNSS.index(state_name))


def build_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
) -> dict[str, Any]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    init_cfg = fusion.setdefault("init", {})
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    base_noise = base_cfg["fusion"]["noise"]
    base_p0 = base_p0_diag_from_config(base_cfg)
    p0_diag = list(base_p0)

    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    fusion["enable_gnss_velocity"] = False
    fusion["gnss_path"] = "dataset/data2/rtk.txt"
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

    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["use_legacy_mounting_base_logic"] = False
    init_cfg["lever_arm_source"] = "init"
    init_cfg["strict_extrinsic_conflict"] = False

    release_state = case_release_state_name(case_id)

    ba_init = get_group_vector_internal(truth_reference, "ba")
    bg_init = get_group_vector_internal(truth_reference, "bg")
    sg_init = get_group_vector_internal(truth_reference, "sg")
    sa_init = get_group_vector_internal(truth_reference, "sa")
    ba_inst = get_group_instability_internal(truth_reference, "ba")
    bg_inst = get_group_instability_internal(truth_reference, "bg")
    sg_inst = get_group_instability_internal(truth_reference, "sg")
    sa_inst = get_group_instability_internal(truth_reference, "sa")
    ba_std = 0.1 * ba_inst
    bg_std = 0.1 * bg_inst
    sg_std = 0.1 * sg_inst
    sa_std = 0.1 * sa_inst
    ba_noise = 0.1 * ba_inst
    bg_noise = 0.1 * bg_inst
    sg_noise = 0.1 * sg_inst
    sa_noise = 0.1 * sa_inst

    if release_state:
        meta = STATE_META[release_state]
        group = meta["group"]
        axis = int(meta["axis"])
        if group == "ba":
            ba_init[axis] = 0.0
            ba_std[axis] = max(abs(get_group_vector_internal(truth_reference, "ba")[axis]), 3.0 * ba_inst[axis])
            ba_noise[axis] = ba_inst[axis]
        elif group == "bg":
            bg_init[axis] = 0.0
            bg_std[axis] = max(abs(get_group_vector_internal(truth_reference, "bg")[axis]), 3.0 * bg_inst[axis])
            bg_noise[axis] = bg_inst[axis]
        elif group == "sg":
            sg_init[axis] = 0.0
            sg_std[axis] = max(abs(get_group_vector_internal(truth_reference, "sg")[axis]), 3.0 * sg_inst[axis])
            sg_noise[axis] = sg_inst[axis]
        elif group == "sa":
            sa_init[axis] = 0.0
            sa_std[axis] = max(abs(get_group_vector_internal(truth_reference, "sa")[axis]), 3.0 * sa_inst[axis])
            sa_noise[axis] = sa_inst[axis]

    init_cfg["ba0"] = [float(x) for x in ba_init]
    init_cfg["bg0"] = [float(x) for x in bg_init]
    init_cfg["sg0"] = [float(x) for x in sg_init]
    init_cfg["sa0"] = [float(x) for x in sa_init]
    p0_diag[9:12] = [float(x * x) for x in ba_std]
    p0_diag[12:15] = [float(x * x) for x in bg_std]
    p0_diag[15:18] = [float(x * x) for x in sg_std]
    p0_diag[18:21] = [float(x * x) for x in sa_std]

    init_cfg["odo_scale"] = float(truth_reference["states"]["odo_scale"]["reference_value_internal"])
    p0_diag[21] = float(1.0e-6)

    mount_ref = get_group_vector_internal(truth_reference, "mounting")
    mount_init_deg = np.rad2deg(mount_ref)
    init_cfg["mounting_roll0"] = float(mount_init_deg[0])
    init_cfg["mounting_pitch0"] = float(mount_init_deg[1])
    init_cfg["mounting_yaw0"] = float(mount_init_deg[2])
    p0_diag[22] = float(math.radians(0.02) ** 2)
    p0_diag[23] = float(math.radians(0.02) ** 2)
    p0_diag[24] = float(math.radians(0.02) ** 2)

    odo_lever_init = get_group_vector_internal(truth_reference, "odo_lever")
    init_cfg["lever_arm0"] = [float(x) for x in odo_lever_init]
    p0_diag[25:28] = [1.0e-4, 1.0e-4, 1.0e-4]

    gnss_lever_init = get_group_vector_internal(truth_reference, "gnss_lever")
    gnss_lever_std = np.full(3, 0.01, dtype=float)
    gnss_lever_noise = np.full(3, family_anchored_process_internal("gnss_lever"), dtype=float)
    if release_state and STATE_META[release_state]["group"] == "gnss_lever":
        axis = int(STATE_META[release_state]["axis"])
        gnss_lever_init[axis] = 0.0
        gnss_lever_std[axis] = family_target_std_human("gnss_lever", axis)
        gnss_lever_noise[axis] = float(base_noise["sigma_gnss_lever_arm"])
    init_cfg["gnss_lever_arm0"] = [float(x) for x in gnss_lever_init]
    p0_diag[28:31] = [float(x * x) for x in gnss_lever_std]
    init_cfg["P0_diag"] = [float(x) for x in p0_diag]

    noise_cfg["sigma_ba"] = float(np.max(ba_noise))
    noise_cfg["sigma_bg"] = float(np.max(bg_noise))
    noise_cfg["sigma_sg"] = float(np.max(sg_noise))
    noise_cfg["sigma_sa"] = float(np.max(sa_noise))
    noise_cfg["sigma_ba_vec"] = [float(x) for x in ba_noise]
    noise_cfg["sigma_bg_vec"] = [float(x) for x in bg_noise]
    noise_cfg["sigma_sg_vec"] = [float(x) for x in sg_noise]
    noise_cfg["sigma_sa_vec"] = [float(x) for x in sa_noise]
    noise_cfg["sigma_odo_scale"] = float(base_noise["sigma_odo_scale"])
    noise_cfg["sigma_mounting_roll"] = float(family_anchored_process_internal("mounting"))
    noise_cfg["sigma_mounting_pitch"] = float(family_anchored_process_internal("mounting"))
    noise_cfg["sigma_mounting_yaw"] = float(family_anchored_process_internal("mounting"))
    noise_cfg["sigma_lever_arm"] = float(family_anchored_process_internal("odo_lever"))
    noise_cfg["sigma_lever_arm_vec"] = [
        float(family_anchored_process_internal("odo_lever")),
        float(family_anchored_process_internal("odo_lever")),
        float(family_anchored_process_internal("odo_lever")),
    ]
    noise_cfg["sigma_gnss_lever_arm"] = float(np.max(gnss_lever_noise))
    noise_cfg["sigma_gnss_lever_arm_vec"] = [float(x) for x in gnss_lever_noise]
    noise_cfg["markov_corr_time"] = 3600.0
    return cfg


def write_case_config(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    case_id: str,
    case_dir: Path,
) -> Path:
    cfg = build_case_config(base_cfg, truth_reference, case_id, case_dir)
    cfg_path = case_dir / f"config_{case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path


def evaluate_navigation_metrics(cfg_path: Path, sol_path: Path) -> dict[str, Any]:
    cfg = load_yaml(cfg_path)
    fusion = cfg["fusion"]
    truth_path = (REPO_ROOT / fusion["pos_path"]).resolve()
    truth_t, truth_xyz = load_truth_ecef(truth_path)
    sol_t, sol_xyz = load_sol(sol_path)
    truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
    err_xyz = sol_xyz - truth_interp
    row = compute_global_metrics(err_xyz)
    row["nav_rmse_3d_m"] = float(row["overall_rmse_3d_m_aux"])
    row["nav_final_err_3d_m"] = float(row["overall_final_err_3d_m_aux"])
    return row


def run_case(case_id: str, cfg_path: Path, case_dir: Path, exe_path: Path) -> dict[str, Any]:
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.csv"
    stdout_path = case_dir / f"{case_id}.stdout.txt"
    diag_path = case_dir / f"DIAG_{case_id}.txt"
    root_diag = REPO_ROOT / "DIAG.txt"
    if root_diag.exists():
        root_diag.unlink()
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output for {case_id}: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output for {case_id}: {state_series_path}")
    if not root_diag.exists():
        raise RuntimeError(f"missing DIAG.txt after case {case_id}")
    shutil.copy2(root_diag, diag_path)
    nav_metrics = evaluate_navigation_metrics(cfg_path, sol_path)
    row: dict[str, Any] = {
        "case_id": case_id,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "diag_path": rel_from_root(diag_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(diag_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    row.update(nav_metrics)
    return row


def impact_label(case_row: dict[str, Any], control_row: dict[str, Any]) -> tuple[str, dict[str, float]]:
    control_rmse = float(control_row["nav_rmse_3d_m"])
    control_final = float(control_row["nav_final_err_3d_m"])
    case_rmse = float(case_row["nav_rmse_3d_m"])
    case_final = float(case_row["nav_final_err_3d_m"])
    delta_rmse = (case_rmse - control_rmse) / max(control_rmse, 1.0e-9)
    delta_final = (case_final - control_final) / max(control_final, 1.0e-9)
    if delta_rmse <= 0.25 and delta_final <= 0.25:
        label = "normal"
    elif delta_rmse <= 1.0 and delta_final <= 1.0:
        label = "borderline"
    else:
        label = "abnormal"
    return label, {
        "delta_nav_rmse3d": float(delta_rmse),
        "delta_nav_final3d": float(delta_final),
        "control_nav_rmse3d": control_rmse,
        "case_nav_rmse3d": case_rmse,
        "control_nav_final3d": control_final,
        "case_nav_final3d": case_final,
    }


def evaluate_state_case(
    state_name: str,
    case_row: dict[str, Any],
    control_row: dict[str, Any],
    truth_reference: dict[str, Any],
) -> dict[str, Any]:
    state_csv_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
    state_df = pd.read_csv(state_csv_path, usecols=["timestamp", STATE_META[state_name]["csv"]])
    timestamps = state_df["timestamp"].to_numpy(dtype=float)
    values = state_df[STATE_META[state_name]["csv"]].to_numpy(dtype=float)
    if STATE_META[state_name]["dynamic"]:
        behavior, behavior_metrics = dynamic_behavior_label(state_name, truth_reference, timestamps, values)
    else:
        behavior, behavior_metrics = static_behavior_label(state_name, truth_reference, values)
    impact, impact_metrics = impact_label(case_row, control_row)
    overall = overall_label(behavior, impact)
    row: dict[str, Any] = {
        "state_name": state_name,
        "case_id": case_row["case_id"],
        "truth_source": truth_reference["states"][state_name]["source"],
        "behavior_label": behavior,
        "impact_label": impact,
        "overall_label": overall,
    }
    row.update(behavior_metrics)
    row.update(impact_metrics)
    return row


def build_case_metrics_table(case_rows: list[dict[str, Any]]) -> pd.DataFrame:
    columns = [
        "case_id",
        "nav_rmse_3d_m",
        "nav_final_err_3d_m",
        "overall_rmse_x_m_aux",
        "overall_rmse_y_m_aux",
        "overall_rmse_z_m_aux",
        "overall_rmse_3d_m_aux",
        "overall_p95_3d_m_aux",
        "overall_final_err_3d_m_aux",
        "config_path",
        "sol_path",
        "state_series_path",
        "diag_path",
        "stdout_path",
        "sol_mtime",
        "state_series_mtime",
        "diag_mtime",
        "stdout_mtime",
    ]
    return pd.DataFrame([{key: row.get(key) for key in columns} for row in case_rows]).sort_values(
        by="case_id", key=lambda col: [case_sort_key(case_id) for case_id in col]
    )


def write_summary(
    output_path: Path,
    truth_reference: dict[str, Any],
    case_metrics_df: pd.DataFrame,
    judgement_df: pd.DataFrame,
    requested_cases: list[str],
    manifest: dict[str, Any],
) -> None:
    control_row = case_metrics_df.loc[case_metrics_df["case_id"] == "truth_anchor_all_non_pva"].iloc[0]
    abnormal_df = judgement_df[judgement_df["overall_label"] == "abnormal"].copy()
    borderline_df = judgement_df[judgement_df["overall_label"] == "borderline"].copy()
    normal_df = judgement_df[judgement_df["overall_label"] == "normal"].copy()
    gnss_lever_df = judgement_df[judgement_df["state_name"].str.startswith("gnss_lever_")].copy()

    lines: list[str] = [
        "# data2 ESKF INS/GNSS-only state sanity summary",
        "",
        "## 1. 控制组表现",
        (
            "- 控制组 `truth_anchor_all_non_pva` 在固定场景 "
            "`data2 + ESKF + corrected RTK + full GNSS + INS/GNSS-only` 下，"
            f"`nav_rmse_3d_m={format_metric(float(control_row['nav_rmse_3d_m']))}`，"
            f"`nav_final_err_3d_m={format_metric(float(control_row['nav_final_err_3d_m']))}`。"
        ),
        (
            "- 本轮实验强制 `enable_odo=false`、`enable_nhc=false`，同时 "
            "`disable_mounting=true`、`disable_odo_lever_arm=true`、`disable_odo_scale=true`，"
            "只检查 `ba/bg/sg/sa/gnss_lever` 的单状态释放行为。"
        ),
        "",
        "## 2. 最异常的状态",
    ]

    if abnormal_df.empty and borderline_df.empty:
        lines.append("- 当前已运行释放状态中，尚未出现 `abnormal/borderline`。")
    else:
        ranked_df = pd.concat([abnormal_df, borderline_df], ignore_index=True)
        severity_map = {"normal": 0, "borderline": 1, "abnormal": 2}
        ranked_df["severity_score"] = ranked_df["overall_label"].map(severity_map)
        ranked_df = ranked_df.sort_values(
            by=["severity_score", "delta_nav_rmse3d", "delta_nav_final3d"],
            ascending=[False, False, False],
        )
        for _, row in ranked_df.head(8).iterrows():
            lines.append(
                f"- `{row['state_name']}`: overall=`{row['overall_label']}`, "
                f"behavior=`{row['behavior_label']}`, impact=`{row['impact_label']}`, "
                f"`delta_nav_rmse3d={format_metric(float(row['delta_nav_rmse3d']))}`, "
                f"`delta_nav_final3d={format_metric(float(row['delta_nav_final3d']))}`。"
            )

    lines.extend(["", "## 3. GNSS 杆臂专项观察"])
    if gnss_lever_df.empty:
        lines.append("- 当前未运行 `gnss_lever` 相关释放实验。")
    else:
        for _, row in gnss_lever_df.sort_values(by="state_name").iterrows():
            lines.append(
                f"- `{row['state_name']}`: overall=`{row['overall_label']}`, "
                f"behavior=`{row['behavior_label']}`, impact=`{row['impact_label']}`。"
            )

    lines.extend(["", "## 4. 可继续保留在线估计的状态"])
    if normal_df.empty:
        lines.append("- 当前已运行释放状态中，暂无可直接判为 `normal` 的状态。")
    else:
        lines.append("- " + "、".join(f"`{state}`" for state in normal_df["state_name"].tolist()))

    lines.extend(
        [
            "",
            "## Notes",
            f"- requested_cases: {', '.join(f'`{case_id}`' for case_id in requested_cases)}",
            f"- truth_reference: `{manifest['truth_reference_json']}`",
            f"- case_metrics: `{manifest['case_metrics_csv']}`",
            f"- state_judgement: `{manifest['state_judgement_csv']}`",
            (
                f"- GNSS lever truth validation median: "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_truth']['value_m'][0]))}`, "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_truth']['value_m'][1]))}`, "
                f"`{format_metric(float(truth_reference['sources']['gnss_lever_truth']['value_m'][2]))}` m."
            ),
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(
        description="Run data2 ESKF INS/GNSS-only truth-anchor control + single-state release sanity matrix."
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=Path("config_data2_baseline_eskf.yaml"),
        help="Baseline config relative to repo root.",
    )
    parser.add_argument(
        "--exe",
        type=Path,
        default=Path("build/Release/eskf_fusion.exe"),
        help="Solver executable relative to repo root.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/data2_eskf_ins_gnss_state_sanity"),
        help="Experiment output directory relative to repo root.",
    )
    parser.add_argument(
        "--exp-id",
        default=f"EXP-{today}-data2-ins-gnss-state-sanity-r1",
        help="Experiment identifier recorded in manifest.",
    )
    parser.add_argument(
        "--cases",
        nargs="*",
        help="Optional subset of case ids. Control is auto-inserted when omitted.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.artifacts_dir = args.output_dir / "artifacts"
    args.case_root = args.artifacts_dir / "cases"
    args.plot_dir = args.output_dir / "plots"
    args.case_ids = requested_case_ids(args.cases)
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    reset_directory(args.output_dir)
    ensure_dir(args.artifacts_dir)
    ensure_dir(args.case_root)
    ensure_dir(args.plot_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    release_state_rows: list[dict[str, Any]] = []

    for case_id in args.case_ids:
        case_dir = args.case_root / case_id
        ensure_dir(case_dir)
        cfg_path = write_case_config(base_cfg, truth_reference, case_id, case_dir)
        case_config_paths[case_id] = rel_from_root(cfg_path, REPO_ROOT)
        case_rows.append(run_case(case_id=case_id, cfg_path=cfg_path, case_dir=case_dir, exe_path=args.exe))

    control_candidates = [row for row in case_rows if row["case_id"] == "truth_anchor_all_non_pva"]
    if not control_candidates:
        raise RuntimeError("control case truth_anchor_all_non_pva was not executed")
    control_row = control_candidates[0]

    for case_row in sorted(case_rows, key=lambda item: case_sort_key(item["case_id"])):
        if case_row["case_id"] == "truth_anchor_all_non_pva":
            continue
        state_name = case_release_state_name(case_row["case_id"])
        release_state_rows.append(evaluate_state_case(state_name, case_row, control_row, truth_reference))
        control_state_path = (REPO_ROOT / control_row["state_series_path"]).resolve()
        case_state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        plot_state_comparison(
            state_name=state_name,
            control_state_path=control_state_path,
            case_state_path=case_state_path,
            truth_reference=truth_reference,
            output_path=args.plot_dir / f"{state_name}_control_vs_release.png",
        )

    case_metrics_df = build_case_metrics_table(case_rows)
    case_metrics_path = args.output_dir / "case_metrics.csv"
    case_metrics_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")

    judgement_df = pd.DataFrame(release_state_rows)
    if not judgement_df.empty:
        judgement_df = judgement_df.sort_values(by="state_name").reset_index(drop=True)
        plot_heatmap(judgement_df, args.plot_dir / "state_judgement_heatmap.png")
    state_judgement_path = args.output_dir / "state_judgement.csv"
    judgement_df.to_csv(state_judgement_path, index=False, encoding="utf-8-sig")

    freshness: dict[str, str] = {
        "truth_reference_json": dt.datetime.fromtimestamp(truth_reference_path.stat().st_mtime).isoformat(timespec="seconds"),
        "case_metrics_csv": dt.datetime.fromtimestamp(case_metrics_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_judgement_csv": dt.datetime.fromtimestamp(state_judgement_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    if not judgement_df.empty:
        freshness["plots_dir"] = dt.datetime.fromtimestamp(
            (args.plot_dir / "state_judgement_heatmap.png").stat().st_mtime
        ).isoformat(timespec="seconds")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(args.artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(args.plot_dir, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "state_judgement_csv": rel_from_root(state_judgement_path, REPO_ROOT),
        "truth_catalog_source": truth_reference["sources"],
        "requested_case_ids": args.case_ids,
        "case_config_paths": case_config_paths,
        "freshness": freshness,
        "mode": "full_gnss_ins_gnss_only",
        "assumptions": [
            "关闭状态语义为真值锚定 + 小初始/过程噪声，而非 mask freeze。",
            "本轮专门移除 ODO/NHC 相关约束链，强制 disable_mounting / disable_odo_lever_arm / disable_odo_scale。",
            "导航影响用全程 RMSE3D 与全程最终 3D 误差相对控制组的变化衡量。",
        ],
    }

    summary_path = args.output_dir / "summary.md"
    write_summary(
        output_path=summary_path,
        truth_reference=truth_reference,
        case_metrics_df=case_metrics_df,
        judgement_df=judgement_df,
        requested_cases=args.case_ids,
        manifest=manifest,
    )
    freshness["summary_md"] = dt.datetime.fromtimestamp(summary_path.stat().st_mtime).isoformat(timespec="seconds")

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
