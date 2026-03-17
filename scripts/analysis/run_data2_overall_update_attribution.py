from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml


TRACKED_RELEASE_STATES = [
    "release_bg_y",
    "release_bg_z",
    "release_sg_z",
    "release_mounting_pitch",
    "release_mounting_roll",
]


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: json_safe(val) for key, val in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, float):
        if value != value or value in (float("inf"), float("-inf")):
            return None
    return value


def run_command(cmd: list[str], cwd: Path) -> str:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(cmd)}\n{merged}")
    return merged


def format_metric(value: float | None) -> str:
    if value is None:
        return "NA"
    if isinstance(value, float) and (value != value or value in (float("inf"), float("-inf"))):
        return "NA"
    return f"{float(value):.6f}"


def variant_specs() -> list[dict[str, Any]]:
    return [
        {
            "variant_id": "baseline_subset",
            "description": "Official outage baseline subset",
            "mutations": {},
        },
        {
            "variant_id": "standard_reset_gamma",
            "description": "Enable debug-only standard ESKF reset gamma",
            "mutations": {
                ("fusion", "fej", "debug_enable_standard_reset_gamma"): True,
            },
        },
        {
            "variant_id": "nhc_off",
            "description": "Disable NHC updates",
            "mutations": {
                ("fusion", "constraints", "enable_nhc"): False,
            },
        },
        {
            "variant_id": "odo_off",
            "description": "Disable ODO updates",
            "mutations": {
                ("fusion", "constraints", "enable_odo"): False,
            },
        },
        {
            "variant_id": "nhc_20hz",
            "description": "Throttle NHC to 20 Hz",
            "mutations": {
                ("fusion", "constraints", "nhc_min_update_interval"): 0.05,
            },
        },
        {
            "variant_id": "odo_20hz",
            "description": "Throttle ODO to 20 Hz",
            "mutations": {
                ("fusion", "constraints", "odo_min_update_interval"): 0.05,
            },
        },
        {
            "variant_id": "standard_reset_gamma_nhc_20hz",
            "description": "Enable standard reset gamma and throttle NHC to 20 Hz",
            "mutations": {
                ("fusion", "fej", "debug_enable_standard_reset_gamma"): True,
                ("fusion", "constraints", "nhc_min_update_interval"): 0.05,
            },
        },
    ]


def set_nested(cfg: dict[str, Any], path: tuple[str, ...], value: Any) -> None:
    node = cfg
    for key in path[:-1]:
        child = node.get(key)
        if not isinstance(child, dict):
            child = {}
            node[key] = child
        node = child
    node[path[-1]] = value


def build_variant_config(base_cfg: dict[str, Any], spec: dict[str, Any], output_dir: Path) -> Path:
    cfg = copy.deepcopy(base_cfg)
    for path, value in spec["mutations"].items():
        set_nested(cfg, path, value)
    cfg_path = output_dir / f"config_{spec['variant_id']}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path


def summarize_variant(
    variant_id: str,
    description: str,
    case_metrics_path: Path,
    judgement_path: Path,
    baseline_control: dict[str, float] | None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    case_df = pd.read_csv(case_metrics_path)
    judgement_df = pd.read_csv(judgement_path)
    control = case_df.loc[case_df["case_id"] == "truth_anchor_all_non_pva"].iloc[0]

    control_row = {
        "variant_id": variant_id,
        "description": description,
        "control_mean_outage_rmse_3d_m": float(control["mean_outage_rmse_3d_m"]),
        "control_max_outage_final_err_3d_m": float(control["max_outage_final_err_3d_m"]),
        "control_overall_rmse_3d_m_aux": float(control["overall_rmse_3d_m_aux"]),
        "tracked_normal_count": int((judgement_df["overall_label"] == "normal").sum()),
        "tracked_borderline_count": int((judgement_df["overall_label"] == "borderline").sum()),
        "tracked_abnormal_count": int((judgement_df["overall_label"] == "abnormal").sum()),
    }
    if baseline_control is None:
        control_row["delta_control_mean_outage_rmse_3d_ratio"] = 0.0
        control_row["delta_control_max_outage_final_err_3d_ratio"] = 0.0
    else:
        control_row["delta_control_mean_outage_rmse_3d_ratio"] = (
            float(control["mean_outage_rmse_3d_m"]) - baseline_control["mean_outage_rmse_3d_m"]
        ) / baseline_control["mean_outage_rmse_3d_m"]
        control_row["delta_control_max_outage_final_err_3d_ratio"] = (
            float(control["max_outage_final_err_3d_m"]) - baseline_control["max_outage_final_err_3d_m"]
        ) / baseline_control["max_outage_final_err_3d_m"]

    state_rows: list[dict[str, Any]] = []
    for _, row in judgement_df.iterrows():
        state_rows.append(
            {
                "variant_id": variant_id,
                "description": description,
                "state_name": str(row["state_name"]),
                "overall_label": str(row["overall_label"]),
                "behavior_label": str(row["behavior_label"]),
                "impact_label": str(row["impact_label"]),
                "delta_mean_rmse3d": float(row["delta_mean_rmse3d"]),
                "delta_max_final3d": float(row["delta_max_final3d"]),
                "recovery_ratio": float(row["recovery_ratio"]) if "recovery_ratio" in row and pd.notna(row["recovery_ratio"]) else None,
                "outside_ratio": float(row["outside_ratio"]) if "outside_ratio" in row and pd.notna(row["outside_ratio"]) else None,
                "diag_head_std_mr": float(row["diag_head_std_mr"]) if "diag_head_std_mr" in row and pd.notna(row["diag_head_std_mr"]) else None,
                "diag_tail_std_mr": float(row["diag_tail_std_mr"]) if "diag_tail_std_mr" in row and pd.notna(row["diag_tail_std_mr"]) else None,
            }
        )
    return control_row, state_rows


def pick_state_row(state_rows: pd.DataFrame, variant_id: str, state_name: str) -> pd.Series | None:
    rows = state_rows[(state_rows["variant_id"] == variant_id) & (state_rows["state_name"] == state_name)]
    if rows.empty:
        return None
    return rows.iloc[0]


def write_summary(
    output_path: Path,
    overview_df: pd.DataFrame,
    state_df: pd.DataFrame,
    manifest: dict[str, Any],
) -> None:
    baseline_row = overview_df.loc[overview_df["variant_id"] == "baseline_subset"].iloc[0]
    lines = [
        "# data2 overall update attribution summary",
        "",
        "## Scope",
        "- 场景固定为 `data2 + ESKF + corrected RTK + 300s on / 100s off / 150s on`。",
        "- 代表性释放状态固定为 `bg_y`, `bg_z`, `sg_z`, `mounting_pitch`, `mounting_roll`，并统一带上 `truth_anchor_all_non_pva` 控制组。",
        "- 本轮重点比较 `standard reset gamma`、`NHC` 与 `ODO` 的开关/降频对控制组和代表性异常状态的影响。",
        "",
        "## Baseline",
        (
            f"- `baseline_subset` 控制组："
            f"`mean_outage_rmse_3d_m={format_metric(float(baseline_row['control_mean_outage_rmse_3d_m']))}`，"
            f"`max_outage_final_err_3d_m={format_metric(float(baseline_row['control_max_outage_final_err_3d_m']))}`。"
        ),
        (
            f"- 跟踪状态标签计数：normal/borderline/abnormal="
            f"`{int(baseline_row['tracked_normal_count'])}/{int(baseline_row['tracked_borderline_count'])}/{int(baseline_row['tracked_abnormal_count'])}`。"
        ),
        "",
        "## Variant Overview",
    ]

    for _, row in overview_df.iterrows():
        if row["variant_id"] == "baseline_subset":
            continue
        lines.append(
            f"- `{row['variant_id']}`: control delta mean/max="
            f"`{format_metric(float(row['delta_control_mean_outage_rmse_3d_ratio']))}`/"
            f"`{format_metric(float(row['delta_control_max_outage_final_err_3d_ratio']))}`，"
            f"tracked abnormal=`{int(row['tracked_abnormal_count'])}`，"
            f"borderline=`{int(row['tracked_borderline_count'])}`。"
        )

    lines.extend(["", "## Representative States"])
    for state_name in TRACKED_RELEASE_STATES:
        lines.append(f"- `{state_name}`:")
        for variant_id in overview_df["variant_id"].tolist():
            state_row = pick_state_row(state_df, variant_id, state_name.replace("release_", ""))
            if state_row is None:
                continue
            lines.append(
                f"  {variant_id}: overall=`{state_row['overall_label']}`, "
                f"behavior=`{state_row['behavior_label']}`, impact=`{state_row['impact_label']}`, "
                f"delta mean/max=`{format_metric(float(state_row['delta_mean_rmse3d']))}`/"
                f"`{format_metric(float(state_row['delta_max_final3d']))}`"
            )

    lines.extend(["", "## Initial Conclusion"])
    better_reset = overview_df.loc[overview_df["variant_id"] == "standard_reset_gamma"]
    better_nhc = overview_df.loc[overview_df["variant_id"] == "nhc_20hz"]
    odo_off = overview_df.loc[overview_df["variant_id"] == "odo_off"]
    nhc_off = overview_df.loc[overview_df["variant_id"] == "nhc_off"]
    if not better_reset.empty and not better_nhc.empty:
        reset_mean = float(better_reset.iloc[0]["delta_control_mean_outage_rmse_3d_ratio"])
        nhc_mean = float(better_nhc.iloc[0]["delta_control_mean_outage_rmse_3d_ratio"])
        lines.append(
            f"- `standard_reset_gamma` 控制组 delta mean={format_metric(reset_mean)}，"
            f"`nhc_20hz` 控制组 delta mean={format_metric(nhc_mean)}。"
        )
    if not odo_off.empty and not nhc_off.empty:
        lines.append(
            f"- `odo_off` 控制组 delta mean/max="
            f"`{format_metric(float(odo_off.iloc[0]['delta_control_mean_outage_rmse_3d_ratio']))}`/"
            f"`{format_metric(float(odo_off.iloc[0]['delta_control_max_outage_final_err_3d_ratio']))}`；"
            f"`nhc_off` 控制组 delta mean/max="
            f"`{format_metric(float(nhc_off.iloc[0]['delta_control_mean_outage_rmse_3d_ratio']))}`/"
            f"`{format_metric(float(nhc_off.iloc[0]['delta_control_max_outage_final_err_3d_ratio']))}`。"
        )
    lines.append(
        "- 以控制组导航变化 + 代表性释放状态标签变化的组合来判断："
        "若 `nhc_20hz` 明显优于 baseline，而 `odo_20hz/odo_off` 不优，说明主要 shared culprit 更偏向 `NHC` 过强/过密；"
        "若 `standard_reset_gamma` 也同步改善 `bg/sg/mounting`，则说明 reset/covariance 语义也是共同原因。"
    )
    lines.extend(
        [
            "",
            "## Artifacts",
            f"- manifest: `{manifest['manifest_path']}`",
            f"- overview_csv: `{manifest['overview_csv']}`",
            f"- tracked_state_csv: `{manifest['tracked_state_csv']}`",
        ]
    )
    output_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(description="Run data2 update attribution experiments on representative state-sanity cases.")
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    parser.add_argument("--state-sanity-script", type=Path, default=Path("scripts/analysis/run_data2_state_sanity_matrix.py"))
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=Path("output/data2_eskf_update_attribution"))
    parser.add_argument("--exp-id", default=f"EXP-{today}-data2-overall-update-attribution-r1")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_config = (REPO_ROOT / args.base_config).resolve()
    state_sanity_script = (REPO_ROOT / args.state_sanity_script).resolve()
    exe_path = (REPO_ROOT / args.exe).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    artifacts_dir = output_dir / "artifacts"
    variants_dir = output_dir / "variants"
    ensure_dir(output_dir)
    ensure_dir(artifacts_dir)
    ensure_dir(variants_dir)

    if not base_config.exists():
        raise FileNotFoundError(f"missing base config: {base_config}")
    if not state_sanity_script.exists():
        raise FileNotFoundError(f"missing state sanity script: {state_sanity_script}")
    if not exe_path.exists():
        raise FileNotFoundError(f"missing solver executable: {exe_path}")

    base_cfg = load_yaml(base_config)
    overview_rows: list[dict[str, Any]] = []
    state_rows: list[dict[str, Any]] = []
    variant_outputs: dict[str, dict[str, str]] = {}
    baseline_control: dict[str, float] | None = None

    for spec in variant_specs():
        variant_id = spec["variant_id"]
        variant_root = variants_dir / variant_id
        ensure_dir(variant_root)
        cfg_path = build_variant_config(base_cfg, spec, variant_root)
        stdout_path = variant_root / f"{variant_id}.stdout.txt"
        cmd = [
            sys.executable,
            str(state_sanity_script),
            "--base-config",
            str(cfg_path),
            "--exe",
            str(exe_path),
            "--output-dir",
            str(variant_root / "result"),
            "--exp-id",
            f"{args.exp_id}-{variant_id}",
            "--cases",
            *TRACKED_RELEASE_STATES,
        ]
        stdout = run_command(cmd, REPO_ROOT)
        stdout_path.write_text(stdout, encoding="utf-8")

        result_dir = variant_root / "result"
        case_metrics_path = result_dir / "case_metrics.csv"
        judgement_path = result_dir / "state_judgement.csv"
        manifest_path = result_dir / "manifest.json"
        if not case_metrics_path.exists() or not judgement_path.exists():
            raise RuntimeError(f"missing result files for variant: {variant_id}")

        overview_row, state_variant_rows = summarize_variant(
            variant_id=variant_id,
            description=spec["description"],
            case_metrics_path=case_metrics_path,
            judgement_path=judgement_path,
            baseline_control=baseline_control,
        )
        if baseline_control is None:
            baseline_control = {
                "mean_outage_rmse_3d_m": float(overview_row["control_mean_outage_rmse_3d_m"]),
                "max_outage_final_err_3d_m": float(overview_row["control_max_outage_final_err_3d_m"]),
            }
            overview_row["delta_control_mean_outage_rmse_3d_ratio"] = 0.0
            overview_row["delta_control_max_outage_final_err_3d_ratio"] = 0.0
        overview_rows.append(overview_row)
        state_rows.extend(state_variant_rows)
        variant_outputs[variant_id] = {
            "config_path": rel_from_root(cfg_path, REPO_ROOT),
            "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
            "result_dir": rel_from_root(result_dir, REPO_ROOT),
            "manifest_path": rel_from_root(manifest_path, REPO_ROOT),
        }

    overview_df = pd.DataFrame(overview_rows)
    state_df = pd.DataFrame(state_rows)
    overview_csv = output_dir / "variant_overview.csv"
    tracked_state_csv = output_dir / "tracked_state_summary.csv"
    overview_df.to_csv(overview_csv, index=False, encoding="utf-8-sig")
    state_df.to_csv(tracked_state_csv, index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(base_config, REPO_ROOT),
        "state_sanity_script": rel_from_root(state_sanity_script, REPO_ROOT),
        "solver_exe": rel_from_root(exe_path, REPO_ROOT),
        "output_dir": rel_from_root(output_dir, REPO_ROOT),
        "tracked_release_states": TRACKED_RELEASE_STATES,
        "variants": variant_outputs,
    }
    manifest_path = output_dir / "manifest.json"
    manifest["manifest_path"] = rel_from_root(manifest_path, REPO_ROOT)
    manifest["overview_csv"] = rel_from_root(overview_csv, REPO_ROOT)
    manifest["tracked_state_csv"] = rel_from_root(tracked_state_csv, REPO_ROOT)
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    summary_path = output_dir / "summary.md"
    write_summary(summary_path, overview_df, state_df, manifest)
    print(rel_from_root(summary_path, REPO_ROOT))


if __name__ == "__main__":
    main()
