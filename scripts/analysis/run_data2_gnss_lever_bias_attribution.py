from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_ins_gnss_state_sanity import (
    build_case_config as build_ins_gnss_case_config,
    evaluate_navigation_metrics,
    run_case as run_nav_case,
)
from scripts.analysis.run_data2_state_sanity_matrix import (
    DATA2_EXTRINSIC_IMU_DEFAULT,
    STATE_META,
    build_truth_reference,
    compute_body_frame_gnss_lever,
    dynamic_behavior_label,
    format_metric,
    json_safe,
    parse_data2_readme_static_vector,
    reset_directory,
)


TARGET_STATES_STAGE_D = ["bg_y", "bg_z", "sg_y", "sg_z"]
GNSS_AXES = ["x", "y", "z"]
GNSS_AXIS_TO_STATE = {axis: f"gnss_lever_{axis}" for axis in GNSS_AXES}
FIRST_CHANGE_THRESHOLD_M = 1.0e-5
PLATEAU_WINDOW_S = 10.0
FIRST_UPDATE_TIME_TOL_S = 0.02
DX_CONSISTENCY_TOL_M = 1.0e-6


@dataclass(frozen=True)
class CaseSpec:
    run_case_id: str
    template_case_id: str
    model_variant: str
    stage: str
    release_state: str | None
    r_scale: float = 1.0
    p0_scale: float = 1.0
    anchor_variant: str = "truth_anchor"
    capture_first_update: bool = True


def scale_slug(value: float) -> str:
    if math.isclose(value, round(value)):
        return str(int(round(value))).replace("-", "m")
    return str(value).replace(".", "p").replace("-", "m")


def released_axis_name(release_state: str | None) -> str | None:
    if release_state is None:
        return None
    if release_state.startswith("gnss_lever_"):
        return release_state.rsplit("_", 1)[-1]
    return None


def parse_vec_field(raw: str) -> np.ndarray:
    text = str(raw).strip().strip("[]")
    if not text:
        return np.zeros(0, dtype=float)
    return np.array([float(part) for part in text.split(";")], dtype=float)


def parse_readme_nominal_lever(
    readme_path: Path,
    imu_model: str = DATA2_EXTRINSIC_IMU_DEFAULT,
) -> list[float]:
    return parse_data2_readme_static_vector(readme_path, "天线杆臂", imu_model)


def apply_model_variant(cfg: dict[str, Any], model_variant: str) -> None:
    fej = cfg.setdefault("fusion", {}).setdefault("fej", {})
    if model_variant == "eskf":
        fej["enable"] = False
        fej["true_iekf_mode"] = False
        return
    if model_variant != "true_iekf":
        raise ValueError(f"unsupported model_variant: {model_variant}")
    fej["enable"] = True
    fej["true_iekf_mode"] = True
    fej["ri_gnss_pos_use_p_ned_local"] = True
    fej["ri_vel_gyro_noise_mode"] = 1
    fej["ri_inject_pos_inverse"] = True


def anchored_gnss_lever_cov() -> list[float]:
    return [1.0e-4, 1.0e-4, 1.0e-4]


def anchored_gnss_lever_noise() -> list[float]:
    return [1.0e-6, 1.0e-6, 1.0e-6]


def configure_case_paths(cfg: dict[str, Any], run_case_id: str, case_dir: Path, capture_first_update: bool) -> Path:
    fusion = cfg["fusion"]
    sol_path = case_dir / f"SOL_{run_case_id}.txt"
    state_series_path = case_dir / f"state_series_{run_case_id}.csv"
    first_update_path = case_dir / f"first_update_{run_case_id}.csv"
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["first_update_debug_output_path"] = (
        rel_from_root(first_update_path, REPO_ROOT) if capture_first_update else ""
    )
    return first_update_path


def build_config_for_spec(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    spec: CaseSpec,
    case_dir: Path,
    wrong_anchor: list[float] | None,
    gnss_path: Path,
) -> tuple[dict[str, Any], Path]:
    cfg = build_ins_gnss_case_config(base_cfg, truth_reference, spec.template_case_id, case_dir, gnss_path)
    first_update_path = configure_case_paths(cfg, spec.run_case_id, case_dir, spec.capture_first_update)
    apply_model_variant(cfg, spec.model_variant)

    fusion = cfg["fusion"]
    init_cfg = fusion["init"]
    noise_cfg = fusion["noise"]
    constraints_cfg = fusion["constraints"]
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_mechanism_log"] = False

    base_sigma_gnss_pos = float(base_cfg["fusion"]["noise"]["sigma_gnss_pos"])
    noise_cfg["sigma_gnss_pos"] = float(base_sigma_gnss_pos * spec.r_scale)

    axis_name = released_axis_name(spec.release_state)
    if axis_name is not None:
        axis = GNSS_AXES.index(axis_name)
        p0_diag = [float(x) for x in init_cfg["P0_diag"]]
        p0_diag[28 + axis] = float(p0_diag[28 + axis] * (spec.p0_scale ** 2))
        init_cfg["P0_diag"] = p0_diag

    if spec.anchor_variant == "wrong_plateau_anchor":
        if wrong_anchor is None:
            raise RuntimeError("wrong_plateau_anchor requested without wrong_anchor values")
        init_cfg["gnss_lever_arm0"] = [float(x) for x in wrong_anchor]
        p0_diag = [float(x) for x in init_cfg["P0_diag"]]
        p0_diag[28:31] = anchored_gnss_lever_cov()
        init_cfg["P0_diag"] = p0_diag
        noise_cfg["sigma_gnss_lever_arm"] = float(max(anchored_gnss_lever_noise()))
        noise_cfg["sigma_gnss_lever_arm_vec"] = anchored_gnss_lever_noise()
    return cfg, first_update_path


def write_case_config(cfg: dict[str, Any], case_dir: Path, run_case_id: str) -> Path:
    cfg_path = case_dir / f"config_{run_case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path


def extract_plateau_metrics_from_csv(
    case_id: str,
    model_variant: str,
    stage: str,
    state_name: str,
    state_series_path: Path,
    source_kind: str,
) -> dict[str, Any]:
    column = STATE_META[state_name]["csv"]
    df = pd.read_csv(state_series_path, usecols=["timestamp", column])
    timestamps = df["timestamp"].to_numpy(dtype=float)
    values = df[column].to_numpy(dtype=float)
    initial_value = float(values[0])
    delta = np.abs(values - initial_value)
    changed = np.where(delta > FIRST_CHANGE_THRESHOLD_M)[0]
    if changed.size == 0:
        return {
            "source_kind": source_kind,
            "stage": stage,
            "case_id": case_id,
            "model_variant": model_variant,
            "state_name": state_name,
            "first_change_time": math.nan,
            "first_change_value": math.nan,
            "early_plateau_median": math.nan,
            "initial_value": initial_value,
            "plateau_window_s": PLATEAU_WINDOW_S,
            "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        }
    first_idx = int(changed[0])
    first_time = float(timestamps[first_idx])
    plateau_mask = (timestamps >= first_time) & (timestamps <= first_time + PLATEAU_WINDOW_S)
    plateau_values = values[plateau_mask]
    return {
        "source_kind": source_kind,
        "stage": stage,
        "case_id": case_id,
        "model_variant": model_variant,
        "state_name": state_name,
        "first_change_time": first_time,
        "first_change_value": float(values[first_idx]),
        "early_plateau_median": float(np.median(plateau_values)),
        "initial_value": initial_value,
        "plateau_window_s": PLATEAU_WINDOW_S,
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
    }


def collect_existing_stage_a(existing_root: Path) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for axis in GNSS_AXES:
        state_name = GNSS_AXIS_TO_STATE[axis]
        case_dir = existing_root / f"release_{state_name}"
        state_path = case_dir / f"state_series_release_{state_name}.csv"
        if not state_path.exists():
            raise FileNotFoundError(f"missing existing stage-A state series: {state_path}")
        rows.append(
            extract_plateau_metrics_from_csv(
                case_id=f"release_{state_name}",
                model_variant="eskf",
                stage="A_existing",
                state_name=state_name,
                state_series_path=state_path,
                source_kind="existing_ins_gnss_state_sanity",
            )
        )
    return pd.DataFrame(rows)


def parse_first_update_rows(
    raw_path: Path,
    spec: CaseSpec,
    plateau_lookup: dict[tuple[str, str], dict[str, Any]],
) -> list[dict[str, Any]]:
    if not raw_path.exists():
        raise FileNotFoundError(f"missing first update debug output: {raw_path}")
    raw_df = pd.read_csv(raw_path)
    rows: list[dict[str, Any]] = []
    target_axis = released_axis_name(spec.release_state)
    plateau_metrics = plateau_lookup.get((spec.run_case_id, spec.release_state or ""))
    for _, record in raw_df.iterrows():
        axis = str(record["gnss_axis"])
        y_vec = parse_vec_field(record["y_vec"])
        h_vec = parse_vec_field(record["h_gnss_lever_vec"])
        k_vec = parse_vec_field(record["k_gnss_lever_vec"])
        lever_before = float(record["lever_before"])
        lever_after = float(record["lever_after"])
        dx = float(record["dx_gnss_lever"])
        row: dict[str, Any] = {
            "stage": spec.stage,
            "case_id": spec.run_case_id,
            "model_variant": spec.model_variant,
            "anchor_variant": spec.anchor_variant,
            "released_state": spec.release_state or "control",
            "target_axis": target_axis or "",
            "gnss_axis": axis,
            "is_target_axis": bool(target_axis == axis),
            "state_t": float(record["state_t"]),
            "gnss_t": float(record["gnss_t"]),
            "lever_before": lever_before,
            "lever_after": lever_after,
            "dx_gnss_lever": dx,
            "y": str(record["y_vec"]),
            "H_gnss_lever": str(record["h_gnss_lever_vec"]),
            "K_gnss_lever": str(record["k_gnss_lever_vec"]),
            "y_norm": float(np.linalg.norm(y_vec)) if y_vec.size else math.nan,
            "h_norm": float(np.linalg.norm(h_vec)) if h_vec.size else math.nan,
            "k_norm": float(np.linalg.norm(k_vec)) if k_vec.size else math.nan,
            "dx_consistency_error": float(abs((lever_after - lever_before) - dx)),
            "raw_path": rel_from_root(raw_path, REPO_ROOT),
        }
        if plateau_metrics and axis == target_axis:
            row["first_change_time"] = float(plateau_metrics["first_change_time"])
            row["first_change_value"] = float(plateau_metrics["first_change_value"])
            row["early_plateau_median"] = float(plateau_metrics["early_plateau_median"])
            row["first_change_time_error_s"] = float(abs(row["first_change_time"] - row["gnss_t"]))
        else:
            row["first_change_time"] = math.nan
            row["first_change_value"] = math.nan
            row["early_plateau_median"] = math.nan
            row["first_change_time_error_s"] = math.nan
        rows.append(row)
    return rows


def artifact_paths(case_dir: Path, run_case_id: str, capture_first_update: bool) -> dict[str, Path]:
    return {
        "config": case_dir / f"config_{run_case_id}.yaml",
        "sol": case_dir / f"SOL_{run_case_id}.txt",
        "state_series": case_dir / f"state_series_{run_case_id}.csv",
        "stdout": case_dir / f"{run_case_id}.stdout.txt",
        "diag": case_dir / f"DIAG_{run_case_id}.txt",
        "first_update": case_dir / f"first_update_{run_case_id}.csv" if capture_first_update else Path(),
    }


def recover_case_row(spec: CaseSpec, case_dir: Path) -> dict[str, Any]:
    paths = artifact_paths(case_dir, spec.run_case_id, spec.capture_first_update)
    required = [paths["config"], paths["sol"], paths["state_series"], paths["stdout"], paths["diag"]]
    if spec.capture_first_update:
        required.append(paths["first_update"])
    for path in required:
        if not path.exists():
            raise FileNotFoundError(f"missing artifact required for resume: {path}")
    nav_metrics = evaluate_navigation_metrics(paths["config"], paths["sol"])
    row: dict[str, Any] = {
        "case_id": spec.run_case_id,
        "config_path": rel_from_root(paths["config"], REPO_ROOT),
        "sol_path": rel_from_root(paths["sol"], REPO_ROOT),
        "state_series_path": rel_from_root(paths["state_series"], REPO_ROOT),
        "diag_path": rel_from_root(paths["diag"], REPO_ROOT),
        "stdout_path": rel_from_root(paths["stdout"], REPO_ROOT),
        "sol_mtime": dt.datetime.fromtimestamp(paths["sol"].stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(paths["state_series"].stat().st_mtime).isoformat(timespec="seconds"),
        "diag_mtime": dt.datetime.fromtimestamp(paths["diag"].stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(paths["stdout"].stat().st_mtime).isoformat(timespec="seconds"),
        "stage": spec.stage,
        "model_variant": spec.model_variant,
        "release_state": spec.release_state or "control",
        "anchor_variant": spec.anchor_variant,
        "first_update_raw_path": (
            rel_from_root(paths["first_update"], REPO_ROOT) if spec.capture_first_update else ""
        ),
    }
    row.update(nav_metrics)
    return row


def run_specs(
    specs: list[CaseSpec],
    output_root: Path,
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    exe_path: Path,
    wrong_anchor: list[float] | None,
    gnss_path: Path,
    reuse_existing: bool = False,
) -> tuple[list[dict[str, Any]], dict[str, str], list[Path]]:
    case_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    first_update_paths: list[Path] = []
    for idx, spec in enumerate(specs, start=1):
        case_dir = output_root / spec.stage.lower() / spec.run_case_id
        ensure_dir(case_dir)
        paths = artifact_paths(case_dir, spec.run_case_id, spec.capture_first_update)
        existing_complete = all(
            path.exists()
            for path in [paths["config"], paths["sol"], paths["state_series"], paths["stdout"], paths["diag"]]
        )
        if spec.capture_first_update:
            existing_complete = existing_complete and paths["first_update"].exists()
        if reuse_existing and existing_complete:
            print(f"[{idx}/{len(specs)}] reusing {spec.run_case_id}")
            row = recover_case_row(spec, case_dir)
            case_config_paths[spec.run_case_id] = row["config_path"]
            if spec.capture_first_update:
                first_update_paths.append(paths["first_update"])
            case_rows.append(row)
            continue

        print(f"[{idx}/{len(specs)}] running {spec.run_case_id}")
        cfg, first_update_path = build_config_for_spec(
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            spec=spec,
            case_dir=case_dir,
            wrong_anchor=wrong_anchor,
            gnss_path=gnss_path,
        )
        cfg_path = write_case_config(cfg, case_dir, spec.run_case_id)
        case_config_paths[spec.run_case_id] = rel_from_root(cfg_path, REPO_ROOT)
        row = run_nav_case(spec.run_case_id, cfg_path, case_dir, exe_path)
        row["stage"] = spec.stage
        row["model_variant"] = spec.model_variant
        row["release_state"] = spec.release_state or "control"
        row["anchor_variant"] = spec.anchor_variant
        row["first_update_raw_path"] = (
            rel_from_root(first_update_path, REPO_ROOT) if spec.capture_first_update else ""
        )
        if spec.capture_first_update and not first_update_path.exists():
            raise RuntimeError(f"missing first update debug CSV for {spec.run_case_id}: {first_update_path}")
        if spec.capture_first_update:
            first_update_paths.append(first_update_path)
        case_rows.append(row)
    return case_rows, case_config_paths, first_update_paths


def make_stage_b_specs() -> list[CaseSpec]:
    specs = [
        CaseSpec(
            run_case_id="truth_anchor_all_non_pva",
            template_case_id="truth_anchor_all_non_pva",
            model_variant="eskf",
            stage="B",
            release_state=None,
        )
    ]
    for model_variant in ["eskf", "true_iekf"]:
        for axis in GNSS_AXES:
            state_name = GNSS_AXIS_TO_STATE[axis]
            specs.append(
                CaseSpec(
                    run_case_id=f"{model_variant}_release_{state_name}",
                    template_case_id=f"release_{state_name}",
                    model_variant=model_variant,
                    stage="B",
                    release_state=state_name,
                )
            )
    return specs


def make_stage_c_specs() -> list[CaseSpec]:
    specs: list[CaseSpec] = []
    for model_variant in ["eskf", "true_iekf"]:
        for axis in ["y", "z"]:
            state_name = GNSS_AXIS_TO_STATE[axis]
            for r_scale in [0.5, 1.0, 2.0]:
                for p0_scale in [0.25, 1.0, 4.0]:
                    specs.append(
                        CaseSpec(
                            run_case_id=(
                                f"{model_variant}_release_{state_name}_"
                                f"R{scale_slug(r_scale)}_P0{scale_slug(p0_scale)}"
                            ),
                            template_case_id=f"release_{state_name}",
                            model_variant=model_variant,
                            stage="C",
                            release_state=state_name,
                            r_scale=r_scale,
                            p0_scale=p0_scale,
                        )
                    )
    return specs


def make_stage_d_specs() -> list[CaseSpec]:
    specs: list[CaseSpec] = []
    for anchor_variant in ["truth_anchor", "wrong_plateau_anchor"]:
        for state_name in TARGET_STATES_STAGE_D:
            specs.append(
                CaseSpec(
                    run_case_id=f"{anchor_variant}_release_{state_name}",
                    template_case_id=f"release_{state_name}",
                    model_variant="eskf",
                    stage="D",
                    release_state=state_name,
                    anchor_variant=anchor_variant,
                    capture_first_update=False,
                )
            )
    return specs


def stage_b_release_specs(specs: list[CaseSpec]) -> list[CaseSpec]:
    return [spec for spec in specs if spec.release_state and spec.release_state.startswith("gnss_lever_")]


def build_reference_table(
    base_cfg: dict[str, Any],
    truth_reference: dict[str, Any],
    stage_b_target_df: pd.DataFrame,
    gnss_path: Path,
) -> pd.DataFrame:
    readme_nominal = parse_readme_nominal_lever((REPO_ROOT / "dataset/data2/README.md").resolve())
    start_time = float(base_cfg["fusion"]["starttime"])
    final_time = float(base_cfg["fusion"]["finaltime"])
    implied_median = compute_body_frame_gnss_lever(
        gnss_path,
        (REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve(),
        start_time,
        final_time,
    )["value_m"]
    truth_body_median = truth_reference["sources"]["gnss_lever_diagnostic_body_median"]["value_m"]

    rows = [
        {
            "reference_name": "README_nominal",
            "x_m": readme_nominal[0],
            "y_m": readme_nominal[1],
            "z_m": readme_nominal[2],
            "source": truth_reference["sources"]["gnss_lever_truth"]["source"],
        },
        {
            "reference_name": "RTK_POS_body_median",
            "x_m": truth_body_median[0],
            "y_m": truth_body_median[1],
            "z_m": truth_body_median[2],
            "source": truth_reference["sources"]["gnss_lever_diagnostic_body_median"]["source"],
        },
        {
            "reference_name": "truth_pose_plus_gnss_implied_median",
            "x_m": implied_median[0],
            "y_m": implied_median[1],
            "z_m": implied_median[2],
            "source": "truth pose + GNSS position body-frame median recomputation",
        },
    ]

    for model_variant in ["eskf", "true_iekf"]:
        subset = stage_b_target_df[stage_b_target_df["model_variant"] == model_variant].copy()
        if subset.empty:
            continue
        values = {axis: math.nan for axis in GNSS_AXES}
        for axis in GNSS_AXES:
            state_name = GNSS_AXIS_TO_STATE[axis]
            axis_df = subset[subset["released_state"] == state_name]
            if not axis_df.empty:
                values[axis] = float(axis_df.iloc[0]["early_plateau_median"])
        rows.append(
            {
                "reference_name": f"{model_variant}_first_jump_plateau",
                "x_m": values["x"],
                "y_m": values["y"],
                "z_m": values["z"],
                "source": f"stage B {model_variant} early plateau median",
            }
        )
    return pd.DataFrame(rows)


def plot_stage_b_series(
    control_path: Path,
    eskf_case_path: Path,
    true_iekf_case_path: Path,
    state_name: str,
    full_path: Path,
    zoom_path: Path,
    first_change_time: float,
    reference_value: float,
) -> None:
    column = STATE_META[state_name]["csv"]
    control_df = pd.read_csv(control_path, usecols=["timestamp", column])
    eskf_df = pd.read_csv(eskf_case_path, usecols=["timestamp", column])
    true_df = pd.read_csv(true_iekf_case_path, usecols=["timestamp", column])

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(control_df["timestamp"], control_df[column], label="control_eskf", linewidth=1.0)
    ax.plot(eskf_df["timestamp"], eskf_df[column], label="release_eskf", linewidth=1.0)
    ax.plot(true_df["timestamp"], true_df[column], label="release_true_iekf", linewidth=1.0)
    ax.axhline(reference_value, color="#444444", linestyle="--", linewidth=1.0, label="RTK/POS median")
    ax.set_title(f"{state_name} full series")
    ax.set_xlabel("timestamp [s]")
    ax.set_ylabel(STATE_META[state_name]["unit"])
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(full_path, dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(control_df["timestamp"], control_df[column], label="control_eskf", linewidth=1.0)
    ax.plot(eskf_df["timestamp"], eskf_df[column], label="release_eskf", linewidth=1.0)
    ax.plot(true_df["timestamp"], true_df[column], label="release_true_iekf", linewidth=1.0)
    ax.axhline(reference_value, color="#444444", linestyle="--", linewidth=1.0, label="RTK/POS median")
    ax.set_xlim(first_change_time - 5.0, first_change_time + 20.0)
    ax.set_title(f"{state_name} first-jump zoom")
    ax.set_xlabel("timestamp [s]")
    ax.set_ylabel(STATE_META[state_name]["unit"])
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(zoom_path, dpi=160)
    plt.close(fig)


def plot_reference_comparison(reference_df: pd.DataFrame, output_path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = reference_df["reference_name"].tolist()
    x = np.arange(len(labels))
    for ax, axis_name in zip(axes, GNSS_AXES):
        values = reference_df[f"{axis_name}_m"].to_numpy(dtype=float)
        ax.bar(x, values, color="#4c78a8")
        ax.set_ylabel(f"{axis_name} [m]")
        ax.grid(True, axis="y", alpha=0.3)
    axes[0].set_title("GNSS lever references vs filter plateau")
    axes[-1].set_xticks(x, labels, rotation=20, ha="right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def plot_stage_d_injection(stage_d_rows: list[dict[str, Any]], output_path: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False)
    axes = axes.ravel()
    for ax, state_name in zip(axes, TARGET_STATES_STAGE_D):
        subset = [row for row in stage_d_rows if row["release_state"] == state_name]
        for row in subset:
            state_path = (REPO_ROOT / row["state_series_path"]).resolve()
            column = STATE_META[state_name]["csv"]
            df = pd.read_csv(state_path, usecols=["timestamp", column])
            ax.plot(
                df["timestamp"].to_numpy(dtype=float),
                df[column].to_numpy(dtype=float),
                label=row["anchor_variant"],
                linewidth=1.0,
            )
        ax.set_title(state_name)
        ax.set_xlabel("timestamp [s]")
        ax.set_ylabel(STATE_META[state_name]["unit"])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def build_stage_d_summary(
    stage_d_rows: list[dict[str, Any]],
    truth_reference: dict[str, Any],
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    long_rows: list[dict[str, Any]] = []
    for case_row in stage_d_rows:
        state_name = str(case_row["release_state"])
        state_path = (REPO_ROOT / case_row["state_series_path"]).resolve()
        column = STATE_META[state_name]["csv"]
        df = pd.read_csv(state_path, usecols=["timestamp", column])
        timestamps = df["timestamp"].to_numpy(dtype=float)
        values = df[column].to_numpy(dtype=float)
        behavior_label, behavior_metrics = dynamic_behavior_label(state_name, truth_reference, timestamps, values)
        long_row: dict[str, Any] = {
            "case_id": case_row["case_id"],
            "anchor_variant": case_row["anchor_variant"],
            "state_name": state_name,
            "behavior_label": behavior_label,
            "final_value": float(values[-1]),
            "nav_rmse_3d_m": float(case_row["nav_rmse_3d_m"]),
            "nav_final_err_3d_m": float(case_row["nav_final_err_3d_m"]),
            "state_series_path": case_row["state_series_path"],
        }
        long_row.update(behavior_metrics)
        long_rows.append(long_row)

    long_df = pd.DataFrame(long_rows)
    for state_name in TARGET_STATES_STAGE_D:
        truth_row = long_df[(long_df["state_name"] == state_name) & (long_df["anchor_variant"] == "truth_anchor")].iloc[0]
        wrong_row = long_df[(long_df["state_name"] == state_name) & (long_df["anchor_variant"] == "wrong_plateau_anchor")].iloc[0]
        rows.append(
            {
                "state_name": state_name,
                "truth_case_id": truth_row["case_id"],
                "wrong_case_id": wrong_row["case_id"],
                "truth_behavior_label": truth_row["behavior_label"],
                "wrong_behavior_label": wrong_row["behavior_label"],
                "truth_final_value": float(truth_row["final_value"]),
                "wrong_final_value": float(wrong_row["final_value"]),
                "truth_outside_ratio": float(truth_row["outside_ratio"]),
                "wrong_outside_ratio": float(wrong_row["outside_ratio"]),
                "truth_nav_rmse_3d_m": float(truth_row["nav_rmse_3d_m"]),
                "wrong_nav_rmse_3d_m": float(wrong_row["nav_rmse_3d_m"]),
                "delta_nav_rmse_3d_m": float(wrong_row["nav_rmse_3d_m"] - truth_row["nav_rmse_3d_m"]),
                "truth_nav_final_err_3d_m": float(truth_row["nav_final_err_3d_m"]),
                "wrong_nav_final_err_3d_m": float(wrong_row["nav_final_err_3d_m"]),
                "delta_nav_final_err_3d_m": float(wrong_row["nav_final_err_3d_m"] - truth_row["nav_final_err_3d_m"]),
                "support_injection": bool(
                    wrong_row["outside_ratio"] > truth_row["outside_ratio"]
                    or wrong_row["nav_rmse_3d_m"] > truth_row["nav_rmse_3d_m"]
                ),
            }
        )
    return pd.DataFrame(rows)


def stage_b_target_rows(first_update_df: pd.DataFrame) -> pd.DataFrame:
    subset = first_update_df[(first_update_df["stage"] == "B") & (first_update_df["is_target_axis"])].copy()
    return subset.sort_values(by=["model_variant", "gnss_axis"]).reset_index(drop=True)


def summarize_stage_b_reference_fit(stage_b_target_df: pd.DataFrame, reference_df: pd.DataFrame) -> pd.DataFrame:
    readme_row = reference_df[reference_df["reference_name"] == "README_nominal"].iloc[0]
    implied_row = reference_df[reference_df["reference_name"] == "truth_pose_plus_gnss_implied_median"].iloc[0]
    rows: list[dict[str, Any]] = []
    for _, row in stage_b_target_df.iterrows():
        axis = str(row["gnss_axis"])
        plateau = float(row["early_plateau_median"])
        rows.append(
            {
                "model_variant": row["model_variant"],
                "gnss_axis": axis,
                "plateau": plateau,
                "abs_err_to_readme": abs(plateau - float(readme_row[f"{axis}_m"])),
                "abs_err_to_implied": abs(plateau - float(implied_row[f"{axis}_m"])),
            }
        )
    return pd.DataFrame(rows)


def write_summary(
    summary_path: Path,
    stage_b_target_df: pd.DataFrame,
    reference_fit_df: pd.DataFrame,
    reference_df: pd.DataFrame,
    stage_c_target_df: pd.DataFrame,
    cross_state_df: pd.DataFrame,
) -> None:
    consistent_count = int((stage_b_target_df["dx_consistency_error"] <= DX_CONSISTENCY_TOL_M).sum()) if not stage_b_target_df.empty else 0
    aligned_count = int((stage_b_target_df["first_change_time_error_s"] <= FIRST_UPDATE_TIME_TOL_S).sum()) if not stage_b_target_df.empty else 0
    total_count = int(len(stage_b_target_df))

    lines: list[str] = [
        "# data2 GNSS lever bias attribution summary",
        "",
        "## 1. 首更是否决定错误平台",
        (
            f"- 阶段 B 的目标轴样本共 `{total_count}` 条；其中 "
            f"`{consistent_count}/{max(total_count, 1)}` 满足 `lever_after - lever_before ~= dx_gnss_lever`，"
            f"`{aligned_count}/{max(total_count, 1)}` 满足首跳时刻与首个接受的 `GNSS_POS` 更新时间一致 "
            f"（容差 `{FIRST_UPDATE_TIME_TOL_S:.3f}s`）。"
        ),
    ]
    for _, row in stage_b_target_df.iterrows():
        lines.append(
            f"- `{row['case_id']}`: `gnss_t={format_metric(float(row['gnss_t']))}`，"
            f"`first_change_t={format_metric(float(row['first_change_time']))}`，"
            f"`plateau={format_metric(float(row['early_plateau_median']))}` m，"
            f"`dx_err={format_metric(float(row['dx_consistency_error']))}` m。"
        )

    lines.extend(["", "## 2. ESKF 与 true_iekf 是否给出不同平台值"])
    for axis in GNSS_AXES:
        eskf_subset = stage_b_target_df[(stage_b_target_df["model_variant"] == "eskf") & (stage_b_target_df["gnss_axis"] == axis)]
        true_subset = stage_b_target_df[(stage_b_target_df["model_variant"] == "true_iekf") & (stage_b_target_df["gnss_axis"] == axis)]
        if eskf_subset.empty or true_subset.empty:
            continue
        eskf_row = eskf_subset.iloc[0]
        true_row = true_subset.iloc[0]
        lines.append(
            f"- `gnss_lever_{axis}`: ESKF plateau=`{format_metric(float(eskf_row['early_plateau_median']))}` m，"
            f"true_iekf plateau=`{format_metric(float(true_row['early_plateau_median']))}` m，"
            f"差值=`{format_metric(abs(float(eskf_row['early_plateau_median']) - float(true_row['early_plateau_median'])))} m`。"
        )

    lines.extend(["", "## 3. 平台值更接近哪一种参考口径"])
    for _, row in reference_fit_df.sort_values(by=["model_variant", "gnss_axis"]).iterrows():
        closer = "implied_dataset_lever" if row["abs_err_to_implied"] < row["abs_err_to_readme"] else "README_nominal"
        lines.append(
            f"- `{row['model_variant']} / gnss_lever_{row['gnss_axis']}`: "
            f"`|plateau-readme|={format_metric(float(row['abs_err_to_readme']))}` m，"
            f"`|plateau-implied|={format_metric(float(row['abs_err_to_implied']))}` m，"
            f"更接近 `{closer}`。"
        )

    lines.extend(["", "## 4. 错误杆臂是否足以注入 bg/sg"])
    if cross_state_df.empty:
        lines.append("- 本次运行未包含阶段 D，因此尚无 `bg/sg` 注入对照结果。")
    else:
        for _, row in cross_state_df.sort_values(by="state_name").iterrows():
            lines.append(
                f"- `{row['state_name']}`: truth_anchor `{row['truth_behavior_label']}` -> wrong_plateau_anchor "
                f"`{row['wrong_behavior_label']}`，"
                f"`delta_nav_rmse_3d_m={format_metric(float(row['delta_nav_rmse_3d_m']))}`，"
                f"`delta_nav_final_err_3d_m={format_metric(float(row['delta_nav_final_err_3d_m']))}`，"
                f"`support_injection={int(bool(row['support_injection']))}`。"
            )

    lines.extend(["", "## 5. 下一步优先方向"])
    if not stage_c_target_df.empty:
        lines.append(
            "- 灵敏度扫中优先检查 `plateau` 与 `k_norm` 的同向缩放、以及平台方向在 `R/P0` 改动下是否保持一致；"
            "若方向稳定而仅幅值随增益变，则优先怀疑系统偏残差/参考口径；若 true_iekf 明显消弱平台，则再回到 `GNSS_POS` correction/reset 语义。"
        )
    lines.append(
        f"- 参考对照表已写入 `{rel_from_root(summary_path.parent / 'implied_lever_reference.csv', REPO_ROOT)}`，"
        "应先核对 README nominal 与数据集隐含杆臂是否一致，再决定是否进入 solver 数学修复。"
    )
    summary_path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    today = dt.datetime.now().strftime("%Y%m%d")
    parser = argparse.ArgumentParser(description="Run data2 GNSS lever wrong-plateau attribution experiment.")
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=Path("output/data2_gnss_lever_bias_attribution"))
    parser.add_argument(
        "--gnss-path",
        type=Path,
        default=Path("dataset/data2/rtk.txt"),
        help="GNSS file used by the pure INS/GNSS attribution rerun.",
    )
    parser.add_argument("--exp-id", default=f"EXP-{today}-data2-gnss-lever-bias-attribution-r1")
    parser.add_argument(
        "--stages",
        nargs="*",
        default=["A", "B", "C", "D"],
        help="Subset of stages to run. Valid: A B C D",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Reuse completed case artifacts in the existing output directory and only run missing cases.",
    )
    args = parser.parse_args()
    args.gnss_path = (REPO_ROOT / args.gnss_path).resolve()
    return args


def main() -> None:
    args = parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing solver exe: {args.exe}")
    if not args.gnss_path.exists():
        raise FileNotFoundError(f"missing GNSS file: {args.gnss_path}")

    stages = {stage.upper() for stage in args.stages}
    if not stages.issubset({"A", "B", "C", "D"}):
        raise ValueError(f"unsupported stages: {args.stages}")

    if args.resume and args.output_dir.exists():
        ensure_dir(args.output_dir)
    else:
        reset_directory(args.output_dir)
    plots_dir = args.output_dir / "plots"
    artifacts_dir = args.output_dir / "artifacts"
    ensure_dir(plots_dir)
    ensure_dir(artifacts_dir)

    base_cfg = load_yaml(args.base_config)
    truth_reference = build_truth_reference(base_cfg)
    truth_reference_path = args.output_dir / "truth_reference.json"
    truth_reference_path.write_text(
        json.dumps(json_safe(truth_reference), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    plateau_frames: list[pd.DataFrame] = []
    manifest_case_configs: dict[str, str] = {}
    all_case_rows: list[dict[str, Any]] = []
    first_update_rows: list[dict[str, Any]] = []
    wrong_anchor: list[float] | None = None

    if "A" in stages:
        existing_root = (REPO_ROOT / "output/data2_eskf_ins_gnss_state_sanity/artifacts/cases").resolve()
        plateau_frames.append(collect_existing_stage_a(existing_root))

    stage_b_specs: list[CaseSpec] = []
    if "B" in stages:
        stage_b_specs = make_stage_b_specs()
        stage_b_rows, stage_b_configs, _ = run_specs(
            specs=stage_b_specs,
            output_root=artifacts_dir,
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            exe_path=args.exe,
            wrong_anchor=None,
            gnss_path=args.gnss_path,
            reuse_existing=args.resume,
        )
        manifest_case_configs.update(stage_b_configs)
        all_case_rows.extend(stage_b_rows)
        stage_b_entries = []
        for spec in stage_b_release_specs(stage_b_specs):
            state_path = (REPO_ROOT / next(row for row in stage_b_rows if row["case_id"] == spec.run_case_id)["state_series_path"]).resolve()
            stage_b_entries.append(
                extract_plateau_metrics_from_csv(
                    case_id=spec.run_case_id,
                    model_variant=spec.model_variant,
                    stage="B",
                    state_name=spec.release_state or "",
                    state_series_path=state_path,
                    source_kind="stage_b_core_matrix",
                )
            )
        stage_b_plateau_df = pd.DataFrame(stage_b_entries)
        plateau_frames.append(stage_b_plateau_df)
        plateau_lookup = {(row["case_id"], row["state_name"]): row for row in stage_b_plateau_df.to_dict("records")}
        for spec in stage_b_specs:
            row = next(item for item in stage_b_rows if item["case_id"] == spec.run_case_id)
            raw_path = (REPO_ROOT / row["first_update_raw_path"]).resolve()
            first_update_rows.extend(parse_first_update_rows(raw_path, spec, plateau_lookup))

        wrong_anchor = [
            float(truth_reference["states"]["gnss_lever_x"]["reference_value_internal"]),
            float(
                stage_b_plateau_df.loc[
                    (stage_b_plateau_df["case_id"] == "eskf_release_gnss_lever_y")
                    & (stage_b_plateau_df["state_name"] == "gnss_lever_y"),
                    "early_plateau_median",
                ].iloc[0]
            ),
            float(
                stage_b_plateau_df.loc[
                    (stage_b_plateau_df["case_id"] == "eskf_release_gnss_lever_z")
                    & (stage_b_plateau_df["state_name"] == "gnss_lever_z"),
                    "early_plateau_median",
                ].iloc[0]
            ),
        ]

    if "C" in stages:
        if wrong_anchor is None:
            raise RuntimeError("stage C requires stage B in the same run")
        stage_c_specs = make_stage_c_specs()
        stage_c_rows, stage_c_configs, _ = run_specs(
            specs=stage_c_specs,
            output_root=artifacts_dir,
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            exe_path=args.exe,
            wrong_anchor=None,
            gnss_path=args.gnss_path,
            reuse_existing=args.resume,
        )
        manifest_case_configs.update(stage_c_configs)
        all_case_rows.extend(stage_c_rows)
        stage_c_entries = []
        for spec in stage_c_specs:
            state_path = (REPO_ROOT / next(row for row in stage_c_rows if row["case_id"] == spec.run_case_id)["state_series_path"]).resolve()
            stage_c_entries.append(
                extract_plateau_metrics_from_csv(
                    case_id=spec.run_case_id,
                    model_variant=spec.model_variant,
                    stage="C",
                    state_name=spec.release_state or "",
                    state_series_path=state_path,
                    source_kind="stage_c_sensitivity",
                )
            )
        stage_c_plateau_df = pd.DataFrame(stage_c_entries)
        plateau_frames.append(stage_c_plateau_df)
        plateau_lookup = {(row["case_id"], row["state_name"]): row for row in stage_c_plateau_df.to_dict("records")}
        for spec in stage_c_specs:
            row = next(item for item in stage_c_rows if item["case_id"] == spec.run_case_id)
            raw_path = (REPO_ROOT / row["first_update_raw_path"]).resolve()
            first_update_rows.extend(parse_first_update_rows(raw_path, spec, plateau_lookup))

    stage_d_rows: list[dict[str, Any]] = []
    cross_state_df = pd.DataFrame()
    if "D" in stages:
        if wrong_anchor is None:
            raise RuntimeError("stage D requires stage B in the same run")
        stage_d_specs = make_stage_d_specs()
        stage_d_rows, stage_d_configs, _ = run_specs(
            specs=stage_d_specs,
            output_root=artifacts_dir,
            base_cfg=base_cfg,
            truth_reference=truth_reference,
            exe_path=args.exe,
            wrong_anchor=wrong_anchor,
            gnss_path=args.gnss_path,
            reuse_existing=args.resume,
        )
        manifest_case_configs.update(stage_d_configs)
        all_case_rows.extend(stage_d_rows)
        cross_state_df = build_stage_d_summary(stage_d_rows, truth_reference)

    plateau_df = pd.concat(plateau_frames, ignore_index=True) if plateau_frames else pd.DataFrame()
    plateau_metrics_path = args.output_dir / "plateau_metrics.csv"
    plateau_df.to_csv(plateau_metrics_path, index=False, encoding="utf-8-sig")

    first_update_df = pd.DataFrame(first_update_rows)
    first_update_path = args.output_dir / "first_update_debug.csv"
    first_update_df.to_csv(first_update_path, index=False, encoding="utf-8-sig")

    stage_b_target_df = stage_b_target_rows(first_update_df) if not first_update_df.empty else pd.DataFrame()
    reference_df = build_reference_table(base_cfg, truth_reference, stage_b_target_df, args.gnss_path)
    implied_reference_path = args.output_dir / "implied_lever_reference.csv"
    reference_df.to_csv(implied_reference_path, index=False, encoding="utf-8-sig")

    reference_fit_df = summarize_stage_b_reference_fit(stage_b_target_df, reference_df) if not stage_b_target_df.empty else pd.DataFrame()
    cross_state_path = args.output_dir / "cross_state_injection.csv"
    cross_state_df.to_csv(cross_state_path, index=False, encoding="utf-8-sig")

    if not stage_b_target_df.empty:
        control_case = next(row for row in all_case_rows if row["case_id"] == "truth_anchor_all_non_pva")
        control_path = (REPO_ROOT / control_case["state_series_path"]).resolve()
        for axis in GNSS_AXES:
            state_name = GNSS_AXIS_TO_STATE[axis]
            eskf_case = next(row for row in all_case_rows if row["case_id"] == f"eskf_release_{state_name}")
            true_case = next(row for row in all_case_rows if row["case_id"] == f"true_iekf_release_{state_name}")
            first_time = float(
                stage_b_target_df.loc[
                    (stage_b_target_df["model_variant"] == "eskf") & (stage_b_target_df["gnss_axis"] == axis),
                    "first_change_time",
                ].iloc[0]
            )
            plot_stage_b_series(
                control_path=control_path,
                eskf_case_path=(REPO_ROOT / eskf_case["state_series_path"]).resolve(),
                true_iekf_case_path=(REPO_ROOT / true_case["state_series_path"]).resolve(),
                state_name=state_name,
                full_path=plots_dir / f"{state_name}_full.png",
                zoom_path=plots_dir / f"{state_name}_first_jump_zoom.png",
                first_change_time=first_time,
                reference_value=float(truth_reference["states"][state_name]["reference_value"]),
            )
        plot_reference_comparison(reference_df, plots_dir / "reference_comparison.png")

    if stage_d_rows:
        plot_stage_d_injection(stage_d_rows, plots_dir / "cross_state_injection.png")

    summary_path = args.output_dir / "summary.md"
    write_summary(
        summary_path=summary_path,
        stage_b_target_df=stage_b_target_df,
        reference_fit_df=reference_fit_df,
        reference_df=reference_df,
        stage_c_target_df=first_update_df[first_update_df["stage"] == "C"].copy() if not first_update_df.empty else pd.DataFrame(),
        cross_state_df=cross_state_df,
    )

    freshness = {}
    for path in [
        truth_reference_path,
        plateau_metrics_path,
        first_update_path,
        implied_reference_path,
        cross_state_path,
        summary_path,
    ]:
        freshness[path.name] = dt.datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(artifacts_dir, REPO_ROOT),
        "plots_dir": rel_from_root(plots_dir, REPO_ROOT),
        "requested_stages": sorted(stages),
        "schedule": f"full GNSS ({rel_from_root(args.gnss_path, REPO_ROOT)}), use_truth_pva=true, ODO/NHC disabled, mounting/odo_lever/odo_scale disabled",
        "gnss_path": rel_from_root(args.gnss_path, REPO_ROOT),
        "truth_reference_json": rel_from_root(truth_reference_path, REPO_ROOT),
        "plateau_metrics_csv": rel_from_root(plateau_metrics_path, REPO_ROOT),
        "first_update_debug_csv": rel_from_root(first_update_path, REPO_ROOT),
        "implied_lever_reference_csv": rel_from_root(implied_reference_path, REPO_ROOT),
        "cross_state_injection_csv": rel_from_root(cross_state_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "truth_catalog_source": truth_reference["sources"],
        "wrong_plateau_anchor_m": wrong_anchor,
        "case_config_paths": manifest_case_configs,
        "freshness": freshness,
        "first_change_threshold_m": FIRST_CHANGE_THRESHOLD_M,
        "plateau_window_s": PLATEAU_WINDOW_S,
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel_from_root(manifest_path, REPO_ROOT))


if __name__ == "__main__":
    main()
