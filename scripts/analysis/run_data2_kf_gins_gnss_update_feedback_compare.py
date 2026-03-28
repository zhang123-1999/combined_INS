from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_baseline_full_ins_gnss import (
    BASE_CONFIG_DEFAULT,
    OUTPUT_DIR_DEFAULT as CURRENT_BASELINE_OUTPUT_DIR_DEFAULT,
    SOLVER_DEFAULT as CURRENT_SOLVER_DEFAULT,
    build_run_config as build_full_window_current_config,
    normalize_repo_path,
)
from scripts.analysis.run_data2_kf_gins_outage_compare import (
    FALLBACK_ROOT_DEFAULT,
    KF_GINS_EXE_DEFAULT,
    build_kf_gins_config,
    copy_file,
    path_is_ascii_safe,
    resolve_runtime_root,
    run_kf_gins,
)
from scripts.analysis.run_data2_state_sanity_matrix import json_safe, run_command
from scripts.analysis.run_data2_turn_window_shared_correction_probe import load_effective_gnss_pos_update_df


EXP_ID_DEFAULT = "EXP-20260328-data2-kf-gins-gnss-update-feedback-compare-r1"
OUTPUT_DIR_DEFAULT = Path("output/data2_kf_gins_gnss_update_feedback_compare_r1")
COMPARE_LIMIT_DEFAULT = 500
CURRENT_CASE_ID = "current_solver"
KF_CASE_ID = "kf_gins"

BLOCK_COMPONENTS = {
    "innovation": ["innovation_x", "innovation_y", "innovation_z"],
    "pos": ["dx_pos_x", "dx_pos_y", "dx_pos_z"],
    "vel": ["dx_vel_x", "dx_vel_y", "dx_vel_z"],
    "att": ["dx_att_x", "dx_att_y", "dx_att_z"],
    "ba": ["dx_ba_x", "dx_ba_y", "dx_ba_z"],
    "bg": ["dx_bg_x", "dx_bg_y", "dx_bg_z"],
    "sg": ["dx_sg_x", "dx_sg_y", "dx_sg_z"],
    "sa": ["dx_sa_x", "dx_sa_y", "dx_sa_z"],
}

SHARED_COMPONENT_COLUMNS = [name for block in BLOCK_COMPONENTS.values() for name in block]

BA_X_DECOMP_SIGNED_BLOCKS = {
    "num_ba_x_from_pos": ["num_ba_x_from_pos_x", "num_ba_x_from_pos_y", "num_ba_x_from_pos_z"],
    "num_ba_x_from_att": ["num_ba_x_from_att_x", "num_ba_x_from_att_y", "num_ba_x_from_att_z"],
    "num_ba_x_from_gnss_lever": [
        "num_ba_x_from_gnss_lever_x",
        "num_ba_x_from_gnss_lever_y",
        "num_ba_x_from_gnss_lever_z",
    ],
    "num_ba_x_total": ["num_ba_x_total_x", "num_ba_x_total_y", "num_ba_x_total_z"],
    "k_ba_x": ["k_ba_x_x", "k_ba_x_y", "k_ba_x_z"],
    "dx_ba_x_from_meas": [
        "dx_ba_x_from_meas_x",
        "dx_ba_x_from_meas_y",
        "dx_ba_x_from_meas_z",
    ],
    "dx_ba_x": ["dx_ba_x"],
}

BA_X_DECOMP_INVARIANT_BLOCKS = {
    "s_mat": [
        "s_00",
        "s_01",
        "s_02",
        "s_10",
        "s_11",
        "s_12",
        "s_20",
        "s_21",
        "s_22",
    ],
}

S_DECOMP_BLOCKS = {
    "s_from_pos_pos": [
        "s_from_pos_pos_00",
        "s_from_pos_pos_01",
        "s_from_pos_pos_02",
        "s_from_pos_pos_10",
        "s_from_pos_pos_11",
        "s_from_pos_pos_12",
        "s_from_pos_pos_20",
        "s_from_pos_pos_21",
        "s_from_pos_pos_22",
    ],
    "s_from_pos_att_cross": [
        "s_from_pos_att_cross_00",
        "s_from_pos_att_cross_01",
        "s_from_pos_att_cross_02",
        "s_from_pos_att_cross_10",
        "s_from_pos_att_cross_11",
        "s_from_pos_att_cross_12",
        "s_from_pos_att_cross_20",
        "s_from_pos_att_cross_21",
        "s_from_pos_att_cross_22",
    ],
    "s_from_att_att": [
        "s_from_att_att_00",
        "s_from_att_att_01",
        "s_from_att_att_02",
        "s_from_att_att_10",
        "s_from_att_att_11",
        "s_from_att_att_12",
        "s_from_att_att_20",
        "s_from_att_att_21",
        "s_from_att_att_22",
    ],
    "s_from_pos_gnss_lever_cross": [
        "s_from_pos_gnss_lever_cross_00",
        "s_from_pos_gnss_lever_cross_01",
        "s_from_pos_gnss_lever_cross_02",
        "s_from_pos_gnss_lever_cross_10",
        "s_from_pos_gnss_lever_cross_11",
        "s_from_pos_gnss_lever_cross_12",
        "s_from_pos_gnss_lever_cross_20",
        "s_from_pos_gnss_lever_cross_21",
        "s_from_pos_gnss_lever_cross_22",
    ],
    "s_from_att_gnss_lever_cross": [
        "s_from_att_gnss_lever_cross_00",
        "s_from_att_gnss_lever_cross_01",
        "s_from_att_gnss_lever_cross_02",
        "s_from_att_gnss_lever_cross_10",
        "s_from_att_gnss_lever_cross_11",
        "s_from_att_gnss_lever_cross_12",
        "s_from_att_gnss_lever_cross_20",
        "s_from_att_gnss_lever_cross_21",
        "s_from_att_gnss_lever_cross_22",
    ],
    "s_from_gnss_lever_gnss_lever": [
        "s_from_gnss_lever_gnss_lever_00",
        "s_from_gnss_lever_gnss_lever_01",
        "s_from_gnss_lever_gnss_lever_02",
        "s_from_gnss_lever_gnss_lever_10",
        "s_from_gnss_lever_gnss_lever_11",
        "s_from_gnss_lever_gnss_lever_12",
        "s_from_gnss_lever_gnss_lever_20",
        "s_from_gnss_lever_gnss_lever_21",
        "s_from_gnss_lever_gnss_lever_22",
    ],
    "r_mat": [
        "r_00",
        "r_01",
        "r_02",
        "r_10",
        "r_11",
        "r_12",
        "r_20",
        "r_21",
        "r_22",
    ],
    "s_reconstructed": [
        "s_reconstructed_00",
        "s_reconstructed_01",
        "s_reconstructed_02",
        "s_reconstructed_10",
        "s_reconstructed_11",
        "s_reconstructed_12",
        "s_reconstructed_20",
        "s_reconstructed_21",
        "s_reconstructed_22",
    ],
    "s_mat": [
        "s_00",
        "s_01",
        "s_02",
        "s_10",
        "s_11",
        "s_12",
        "s_20",
        "s_21",
        "s_22",
    ],
}

BA_X_DECOMP_REQUIRED_CURRENT = [
    "gnss_t",
    "y_x",
    "y_y",
    "y_z",
    "dx_ba_x",
    "prior_cov_ba_pos_mat",
    "prior_cov_ba_att_mat",
    "prior_cov_ba_gnss_lever_mat",
    "s_mat",
    "h_pos_x_vec",
    "h_pos_y_vec",
    "h_pos_z_vec",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
    "h_gnss_lever_x_vec",
    "h_gnss_lever_y_vec",
    "h_gnss_lever_z_vec",
    "k_ba_x_vec",
]

BA_X_DECOMP_REQUIRED_KF = [
    "gnss_t",
    "y_x",
    "y_y",
    "y_z",
    "dx_ba_x",
    "prior_cov_ba_pos_mat",
    "prior_cov_ba_att_mat",
    "s_mat",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
    "k_ba_x_vec",
]

S_DECOMP_REQUIRED_CURRENT = [
    "gnss_t",
    "prior_cov_pos_mat",
    "prior_cov_att_mat",
    "prior_cov_pos_att_mat",
    "prior_cov_pos_gnss_lever_mat",
    "prior_cov_att_gnss_lever_mat",
    "prior_cov_gnss_lever_mat",
    "r_mat",
    "s_mat",
    "h_pos_x_vec",
    "h_pos_y_vec",
    "h_pos_z_vec",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
    "h_gnss_lever_x_vec",
    "h_gnss_lever_y_vec",
    "h_gnss_lever_z_vec",
]

S_DECOMP_REQUIRED_KF = [
    "gnss_t",
    "prior_cov_pos_mat",
    "prior_cov_att_mat",
    "prior_cov_pos_att_mat",
    "r_mat",
    "s_mat",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
]

POST_COV_REQUIRED_CURRENT = [
    "gnss_t",
    "prior_cov_pos_mat",
    "prior_cov_att_mat",
    "prior_cov_pos_att_mat",
    "prior_cov_pos_gnss_lever_mat",
    "prior_cov_att_gnss_lever_mat",
    "prior_cov_gnss_lever_mat",
    "r_mat",
    "h_pos_x_vec",
    "h_pos_y_vec",
    "h_pos_z_vec",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
    "h_gnss_lever_x_vec",
    "h_gnss_lever_y_vec",
    "h_gnss_lever_z_vec",
    "k_pos_x_vec",
    "k_pos_y_vec",
    "k_pos_z_vec",
    "k_att_x_vec",
    "k_att_y_vec",
    "k_att_z_vec",
]

POST_COV_REQUIRED_KF = [
    "gnss_t",
    "prior_cov_pos_mat",
    "prior_cov_att_mat",
    "prior_cov_pos_att_mat",
    "r_mat",
    "h_att_x_vec",
    "h_att_y_vec",
    "h_att_z_vec",
    "k_pos_x_vec",
    "k_pos_y_vec",
    "k_pos_z_vec",
    "k_att_x_vec",
    "k_att_y_vec",
    "k_att_z_vec",
]

PREDICT_DEBUG_REQUIRED_CURRENT = [
    "tag",
    "t_prev",
    "t_curr",
    "dt",
    "p_before_common_mat",
    "phi_common_mat",
    "qd_common_mat",
    "phi_p_common_mat",
    "p_after_raw_common_mat",
    "p_after_final_common_mat",
]

PREDICT_DEBUG_REQUIRED_KF = [
    "tag",
    "t_prev",
    "t_curr",
    "dt",
    "p_before_common_mat",
    "phi_common_mat",
    "qd_common_mat",
    "phi_p_common_mat",
    "p_after_common_mat",
]

PREDICT_BLOCK_SPECS: dict[str, tuple[slice, slice]] = {
    "cov_pos": (slice(0, 3), slice(0, 3)),
    "cov_pos_vel": (slice(0, 3), slice(3, 6)),
    "cov_pos_att": (slice(0, 3), slice(6, 9)),
    "cov_vel_vel": (slice(3, 6), slice(3, 6)),
    "cov_vel_att": (slice(3, 6), slice(6, 9)),
    "cov_att": (slice(6, 9), slice(6, 9)),
}


def _component_norm(frame: pd.DataFrame, columns: list[str]) -> np.ndarray:
    values = frame.loc[:, columns].to_numpy(dtype=float)
    return np.linalg.norm(values, axis=1)


def _ensure_required_columns(frame: pd.DataFrame, required: list[str], context: str) -> None:
    missing = [name for name in required if name not in frame.columns]
    if missing:
        raise KeyError(f"{context} missing required columns: {missing}")


def _parse_bracket_numbers(value: Any, context: str) -> np.ndarray:
    text = str(value).strip()
    if not text.startswith("[") or not text.endswith("]"):
        raise ValueError(f"{context} must use bracket format, got: {value!r}")
    body = text[1:-1].strip()
    if not body:
        return np.array([], dtype=float)
    return np.array([float(part) for part in body.split(";")], dtype=float)


def _parse_vector_field(value: Any, size: int, context: str) -> np.ndarray:
    vec = _parse_bracket_numbers(value, context)
    if vec.size != size:
        raise ValueError(f"{context} expected {size} entries, got {vec.size}")
    return vec


def _parse_matrix_field(value: Any, rows: int, cols: int, context: str) -> np.ndarray:
    values = _parse_bracket_numbers(value, context)
    expected = rows * cols
    if values.size != expected:
        raise ValueError(f"{context} expected {expected} entries, got {values.size}")
    return values.reshape(rows, cols)


def _flatten_vector(prefix: str, values: np.ndarray) -> dict[str, float]:
    return {f"{prefix}_{axis}": float(values[idx]) for idx, axis in enumerate(("x", "y", "z"))}


def _flatten_matrix(prefix: str, values: np.ndarray) -> dict[str, float]:
    return {f"{prefix}_{row}{col}": float(values[row, col]) for row in range(values.shape[0]) for col in range(values.shape[1])}


def _build_column_block(row: pd.Series, names: list[str], context: str) -> np.ndarray:
    cols = [_parse_vector_field(row[name], 3, f"{context}.{name}") for name in names]
    return np.column_stack(cols)


def _build_row_block(row: pd.Series, names: list[str], context: str) -> np.ndarray:
    rows = [_parse_vector_field(row[name], 3, f"{context}.{name}") for name in names]
    return np.vstack(rows)


def _safe_reconstruct_gain(numerator: np.ndarray, s_mat: np.ndarray) -> np.ndarray:
    return np.linalg.solve(s_mat.T, numerator)


def build_current_compare_config(
    base_cfg: dict[str, Any],
    output_dir: Path,
    compare_limit_updates: int = COMPARE_LIMIT_DEFAULT,
    predict_debug_path: Path | None = None,
    predict_window: tuple[float, float] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg, metadata = build_full_window_current_config(
        base_cfg,
        output_dir,
        case_id="data2_full_window_fixed_gnss_lever_truth_current_solver",
        case_label="data2 full-window INS/GNSS fixed true GNSS lever current solver",
        fix_gnss_lever_truth=True,
    )
    output_dir_abs = normalize_repo_path(output_dir)
    fusion = cfg.setdefault("fusion", {})
    debug_path = output_dir_abs / "artifacts" / CURRENT_CASE_ID / "gnss_updates_current_solver.csv"
    fusion["gnss_update_debug_output_path"] = rel_from_root(debug_path, REPO_ROOT)
    if predict_debug_path is not None:
        fusion["predict_debug_output_path"] = rel_from_root(predict_debug_path, REPO_ROOT)
    else:
        fusion.pop("predict_debug_output_path", None)
    if predict_window is not None:
        fusion["predict_debug_start_time"] = float(predict_window[0])
        fusion["predict_debug_end_time"] = float(predict_window[1])
    else:
        fusion.pop("predict_debug_start_time", None)
        fusion.pop("predict_debug_end_time", None)
    metadata["compare_limit_updates"] = int(compare_limit_updates)
    metadata["current_gnss_update_debug_output_path"] = fusion["gnss_update_debug_output_path"]
    if predict_debug_path is not None:
        metadata["current_predict_debug_output_path"] = fusion["predict_debug_output_path"]
    if predict_window is not None:
        metadata["predict_window"] = [float(predict_window[0]), float(predict_window[1])]
    return cfg, metadata


def build_kf_gins_feedback_config(
    source_cfg: dict[str, Any],
    imupath: Path,
    gnsspath: Path,
    outputpath: Path,
    gnss_update_debug_path: Path | None = None,
    predict_debug_path: Path | None = None,
    predict_window: tuple[float, float] | None = None,
) -> dict[str, Any]:
    cfg = build_kf_gins_config(source_cfg, imupath=imupath, gnsspath=gnsspath, outputpath=outputpath)
    if gnss_update_debug_path is not None:
        cfg["gnss_update_debug_path"] = str(gnss_update_debug_path)
    if predict_debug_path is not None:
        cfg["predict_debug_path"] = str(predict_debug_path)
    if predict_window is not None:
        cfg["predict_debug_start_time"] = float(predict_window[0])
        cfg["predict_debug_end_time"] = float(predict_window[1])
    return cfg


def _load_csv_or_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    if isinstance(raw, pd.DataFrame):
        return raw.copy()
    return pd.read_csv(raw)


def _normalize_shared_gnss_update_df(raw_df: pd.DataFrame, context: str) -> pd.DataFrame:
    required = ["gnss_t", *SHARED_COMPONENT_COLUMNS]
    _ensure_required_columns(raw_df, required, context)

    normalized = raw_df.copy().reset_index(drop=True)
    if "update_index" not in normalized.columns:
        normalized["update_index"] = np.arange(len(normalized), dtype=int)

    for block_name, columns in BLOCK_COMPONENTS.items():
        normalized[f"{block_name}_norm" if block_name == "innovation" else f"{columns[0].rsplit('_', 1)[0]}_norm"] = _component_norm(
            normalized, columns
        )
    return normalized


def normalize_current_gnss_update_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    frame = _load_csv_or_df(raw)
    if not isinstance(raw, pd.DataFrame):
        frame = load_effective_gnss_pos_update_df(Path(raw))
    rename_map = {
        "y_x": "innovation_x",
        "y_y": "innovation_y",
        "y_z": "innovation_z",
    }
    normalized = frame.rename(columns=rename_map)
    return _normalize_shared_gnss_update_df(normalized, "current_gnss_update")


def normalize_kf_gins_gnss_update_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    frame = _load_csv_or_df(raw)
    rename_map = {
        "y_x": "innovation_x",
        "y_y": "innovation_y",
        "y_z": "innovation_z",
    }
    normalized = frame.rename(columns=rename_map)
    return _normalize_shared_gnss_update_df(normalized, "kf_gins_gnss_update")


def _build_ba_x_decomposition_df(raw: pd.DataFrame | Path, context: str) -> pd.DataFrame:
    frame = _load_csv_or_df(raw)
    if context == "current" and not isinstance(raw, pd.DataFrame):
        frame = load_effective_gnss_pos_update_df(Path(raw))
    frame = frame.copy().reset_index(drop=True)
    if "update_index" not in frame.columns:
        frame["update_index"] = np.arange(len(frame), dtype=int)

    if context == "current":
        _ensure_required_columns(frame, BA_X_DECOMP_REQUIRED_CURRENT, context)
    elif context == "kf":
        _ensure_required_columns(frame, BA_X_DECOMP_REQUIRED_KF, context)
    else:
        raise ValueError(f"unsupported context: {context}")

    rows: list[dict[str, Any]] = []
    for _, row in frame.iterrows():
        innovation = np.array([float(row["y_x"]), float(row["y_y"]), float(row["y_z"])], dtype=float)
        p_ba_pos = _parse_matrix_field(row["prior_cov_ba_pos_mat"], 3, 3, f"{context}.prior_cov_ba_pos_mat")
        p_ba_att = _parse_matrix_field(row["prior_cov_ba_att_mat"], 3, 3, f"{context}.prior_cov_ba_att_mat")
        if context == "current":
            p_ba_gnss_lever = _parse_matrix_field(
                row["prior_cov_ba_gnss_lever_mat"],
                3,
                3,
                f"{context}.prior_cov_ba_gnss_lever_mat",
            )
            h_pos = _build_column_block(row, ["h_pos_x_vec", "h_pos_y_vec", "h_pos_z_vec"], context)
            h_gnss_lever = _build_column_block(
                row,
                ["h_gnss_lever_x_vec", "h_gnss_lever_y_vec", "h_gnss_lever_z_vec"],
                context,
            )
        else:
            p_ba_gnss_lever = np.zeros((3, 3), dtype=float)
            h_pos = np.eye(3, dtype=float)
            h_gnss_lever = np.zeros((3, 3), dtype=float)
        h_att = _build_column_block(row, ["h_att_x_vec", "h_att_y_vec", "h_att_z_vec"], context)
        s_mat = _parse_matrix_field(row["s_mat"], 3, 3, f"{context}.s_mat")
        k_ba_x = _parse_vector_field(row["k_ba_x_vec"], 3, f"{context}.k_ba_x_vec")

        p_row_pos = p_ba_pos[0, :]
        p_row_att = p_ba_att[0, :]
        p_row_gnss_lever = p_ba_gnss_lever[0, :]

        num_from_pos = h_pos @ p_row_pos
        num_from_att = h_att @ p_row_att
        num_from_gnss_lever = h_gnss_lever @ p_row_gnss_lever
        num_total = num_from_pos + num_from_att + num_from_gnss_lever
        k_ba_x_reconstructed = _safe_reconstruct_gain(num_total, s_mat)
        dx_ba_x_from_meas = k_ba_x * innovation

        record: dict[str, Any] = {
            "update_index": int(row["update_index"]),
            "gnss_t": float(row["gnss_t"]),
            "dx_ba_x": float(row["dx_ba_x"]),
            "k_ba_x_reconstruction_error_norm": float(np.linalg.norm(k_ba_x_reconstructed - k_ba_x)),
        }
        record.update(_flatten_vector("innovation", innovation))
        record.update(_flatten_vector("num_ba_x_from_pos", num_from_pos))
        record.update(_flatten_vector("num_ba_x_from_att", num_from_att))
        record.update(_flatten_vector("num_ba_x_from_gnss_lever", num_from_gnss_lever))
        record.update(_flatten_vector("num_ba_x_total", num_total))
        record.update(_flatten_vector("k_ba_x", k_ba_x))
        record.update(_flatten_vector("k_ba_x_reconstructed", k_ba_x_reconstructed))
        record.update(_flatten_vector("dx_ba_x_from_meas", dx_ba_x_from_meas))
        record.update(_flatten_matrix("s", s_mat))
        rows.append(record)
    return pd.DataFrame(rows)


def build_current_ba_x_decomposition_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_ba_x_decomposition_df(raw, "current")


def build_kf_gins_ba_x_decomposition_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_ba_x_decomposition_df(raw, "kf")


def _build_s_decomposition_df(raw: pd.DataFrame | Path, context: str) -> pd.DataFrame:
    frame = _load_csv_or_df(raw)
    if context == "current" and not isinstance(raw, pd.DataFrame):
        frame = load_effective_gnss_pos_update_df(Path(raw))
    frame = frame.copy().reset_index(drop=True)
    if "update_index" not in frame.columns:
        frame["update_index"] = np.arange(len(frame), dtype=int)

    if context == "current":
        _ensure_required_columns(frame, S_DECOMP_REQUIRED_CURRENT, context)
    elif context == "kf":
        _ensure_required_columns(frame, S_DECOMP_REQUIRED_KF, context)
    else:
        raise ValueError(f"unsupported context: {context}")

    rows: list[dict[str, Any]] = []
    for _, row in frame.iterrows():
        p_pos = _parse_matrix_field(row["prior_cov_pos_mat"], 3, 3, f"{context}.prior_cov_pos_mat")
        p_att = _parse_matrix_field(row["prior_cov_att_mat"], 3, 3, f"{context}.prior_cov_att_mat")
        p_pos_att = _parse_matrix_field(row["prior_cov_pos_att_mat"], 3, 3, f"{context}.prior_cov_pos_att_mat")
        r_mat = _parse_matrix_field(row["r_mat"], 3, 3, f"{context}.r_mat")
        s_mat = _parse_matrix_field(row["s_mat"], 3, 3, f"{context}.s_mat")

        if context == "current":
            p_pos_gnss_lever = _parse_matrix_field(
                row["prior_cov_pos_gnss_lever_mat"], 3, 3, f"{context}.prior_cov_pos_gnss_lever_mat"
            )
            p_att_gnss_lever = _parse_matrix_field(
                row["prior_cov_att_gnss_lever_mat"], 3, 3, f"{context}.prior_cov_att_gnss_lever_mat"
            )
            p_gnss_lever = _parse_matrix_field(
                row["prior_cov_gnss_lever_mat"], 3, 3, f"{context}.prior_cov_gnss_lever_mat"
            )
            h_pos = _build_column_block(row, ["h_pos_x_vec", "h_pos_y_vec", "h_pos_z_vec"], context)
            h_gnss_lever = _build_column_block(
                row,
                ["h_gnss_lever_x_vec", "h_gnss_lever_y_vec", "h_gnss_lever_z_vec"],
                context,
            )
        else:
            p_pos_gnss_lever = np.zeros((3, 3), dtype=float)
            p_att_gnss_lever = np.zeros((3, 3), dtype=float)
            p_gnss_lever = np.zeros((3, 3), dtype=float)
            h_pos = np.eye(3, dtype=float)
            h_gnss_lever = np.zeros((3, 3), dtype=float)
        h_att = _build_column_block(row, ["h_att_x_vec", "h_att_y_vec", "h_att_z_vec"], context)

        s_from_pos_pos = h_pos @ p_pos @ h_pos.T
        s_from_pos_att_cross = h_pos @ p_pos_att @ h_att.T + h_att @ p_pos_att.T @ h_pos.T
        s_from_att_att = h_att @ p_att @ h_att.T
        s_from_pos_gnss_lever_cross = (
            h_pos @ p_pos_gnss_lever @ h_gnss_lever.T + h_gnss_lever @ p_pos_gnss_lever.T @ h_pos.T
        )
        s_from_att_gnss_lever_cross = (
            h_att @ p_att_gnss_lever @ h_gnss_lever.T + h_gnss_lever @ p_att_gnss_lever.T @ h_att.T
        )
        s_from_gnss_lever_gnss_lever = h_gnss_lever @ p_gnss_lever @ h_gnss_lever.T
        s_reconstructed = (
            s_from_pos_pos
            + s_from_pos_att_cross
            + s_from_att_att
            + s_from_pos_gnss_lever_cross
            + s_from_att_gnss_lever_cross
            + s_from_gnss_lever_gnss_lever
            + r_mat
        )

        record: dict[str, Any] = {
            "update_index": int(row["update_index"]),
            "gnss_t": float(row["gnss_t"]),
            "s_reconstruction_error_norm": float(np.linalg.norm(s_reconstructed - s_mat)),
        }
        record.update(_flatten_matrix("s_from_pos_pos", s_from_pos_pos))
        record.update(_flatten_matrix("s_from_pos_att_cross", s_from_pos_att_cross))
        record.update(_flatten_matrix("s_from_att_att", s_from_att_att))
        record.update(_flatten_matrix("s_from_pos_gnss_lever_cross", s_from_pos_gnss_lever_cross))
        record.update(_flatten_matrix("s_from_att_gnss_lever_cross", s_from_att_gnss_lever_cross))
        record.update(_flatten_matrix("s_from_gnss_lever_gnss_lever", s_from_gnss_lever_gnss_lever))
        record.update(_flatten_matrix("r", r_mat))
        record.update(_flatten_matrix("s_reconstructed", s_reconstructed))
        record.update(_flatten_matrix("s", s_mat))
        rows.append(record)
    return pd.DataFrame(rows)


def build_current_s_decomposition_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_s_decomposition_df(raw, "current")


def build_kf_gins_s_decomposition_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_s_decomposition_df(raw, "kf")


def _build_posterior_covariance_df(raw: pd.DataFrame | Path, context: str) -> pd.DataFrame:
    frame = _load_csv_or_df(raw)
    if context == "current" and not isinstance(raw, pd.DataFrame):
        frame = load_effective_gnss_pos_update_df(Path(raw))
    frame = frame.copy().reset_index(drop=True)
    if "update_index" not in frame.columns:
        frame["update_index"] = np.arange(len(frame), dtype=int)

    if context == "current":
        _ensure_required_columns(frame, POST_COV_REQUIRED_CURRENT, context)
    elif context == "kf":
        _ensure_required_columns(frame, POST_COV_REQUIRED_KF, context)
    else:
        raise ValueError(f"unsupported context: {context}")

    rows: list[dict[str, Any]] = []
    for _, row in frame.iterrows():
        p_pos = _parse_matrix_field(row["prior_cov_pos_mat"], 3, 3, f"{context}.prior_cov_pos_mat")
        p_att = _parse_matrix_field(row["prior_cov_att_mat"], 3, 3, f"{context}.prior_cov_att_mat")
        p_pos_att = _parse_matrix_field(row["prior_cov_pos_att_mat"], 3, 3, f"{context}.prior_cov_pos_att_mat")
        r_mat = _parse_matrix_field(row["r_mat"], 3, 3, f"{context}.r_mat")

        if context == "current":
            p_pos_gnss_lever = _parse_matrix_field(
                row["prior_cov_pos_gnss_lever_mat"], 3, 3, f"{context}.prior_cov_pos_gnss_lever_mat"
            )
            p_att_gnss_lever = _parse_matrix_field(
                row["prior_cov_att_gnss_lever_mat"], 3, 3, f"{context}.prior_cov_att_gnss_lever_mat"
            )
            p_gnss_lever = _parse_matrix_field(
                row["prior_cov_gnss_lever_mat"], 3, 3, f"{context}.prior_cov_gnss_lever_mat"
            )
            h_pos = _build_column_block(row, ["h_pos_x_vec", "h_pos_y_vec", "h_pos_z_vec"], context)
            h_gnss_lever = _build_column_block(
                row,
                ["h_gnss_lever_x_vec", "h_gnss_lever_y_vec", "h_gnss_lever_z_vec"],
                context,
            )
        else:
            p_pos_gnss_lever = np.zeros((3, 3), dtype=float)
            p_att_gnss_lever = np.zeros((3, 3), dtype=float)
            p_gnss_lever = np.zeros((3, 3), dtype=float)
            h_pos = np.eye(3, dtype=float)
            h_gnss_lever = np.zeros((3, 3), dtype=float)
        h_att = _build_column_block(row, ["h_att_x_vec", "h_att_y_vec", "h_att_z_vec"], context)
        k_pos = _build_row_block(row, ["k_pos_x_vec", "k_pos_y_vec", "k_pos_z_vec"], context)
        k_att = _build_row_block(row, ["k_att_x_vec", "k_att_y_vec", "k_att_z_vec"], context)

        h_reduced = np.hstack([h_pos, h_att, h_gnss_lever])
        p_reduced = np.block(
            [
                [p_pos, p_pos_att, p_pos_gnss_lever],
                [p_pos_att.T, p_att, p_att_gnss_lever],
                [p_pos_gnss_lever.T, p_att_gnss_lever.T, p_gnss_lever],
            ]
        )
        e_pos = np.hstack([np.eye(3, dtype=float), np.zeros((3, 6), dtype=float)])
        e_att = np.hstack(
            [np.zeros((3, 3), dtype=float), np.eye(3, dtype=float), np.zeros((3, 3), dtype=float)]
        )
        a_pos = e_pos - k_pos @ h_reduced
        a_att = e_att - k_att @ h_reduced

        post_cov_pos = a_pos @ p_reduced @ a_pos.T + k_pos @ r_mat @ k_pos.T
        post_cov_att = a_att @ p_reduced @ a_att.T + k_att @ r_mat @ k_att.T
        post_cov_pos_att = a_pos @ p_reduced @ a_att.T + k_pos @ r_mat @ k_att.T

        e_pos_att = np.vstack([e_pos, e_att])
        k_pos_att = np.vstack([k_pos, k_att])
        a_pos_att = e_pos_att - k_pos_att @ h_reduced
        post_cov_pos_att_block = a_pos_att @ p_reduced @ a_pos_att.T + k_pos_att @ r_mat @ k_pos_att.T
        post_cov_pos_att_simple = e_pos_att @ p_reduced @ e_pos_att.T - k_pos_att @ h_reduced @ p_reduced @ e_pos_att.T

        record: dict[str, Any] = {
            "update_index": int(row["update_index"]),
            "gnss_t": float(row["gnss_t"]),
            "post_cov_reconstruction_error_norm": float(
                np.linalg.norm(post_cov_pos_att_block - post_cov_pos_att_simple)
            ),
        }
        record.update(_flatten_matrix("prior_cov_pos", p_pos))
        record.update(_flatten_matrix("prior_cov_att", p_att))
        record.update(_flatten_matrix("prior_cov_pos_att", p_pos_att))
        record.update(_flatten_matrix("post_cov_pos", post_cov_pos))
        record.update(_flatten_matrix("post_cov_att", post_cov_att))
        record.update(_flatten_matrix("post_cov_pos_att", post_cov_pos_att))
        rows.append(record)
    return pd.DataFrame(rows)


def build_current_posterior_covariance_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_posterior_covariance_df(raw, "current")


def build_kf_gins_posterior_covariance_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_posterior_covariance_df(raw, "kf")


def _sorted_prefix_columns(frame: pd.DataFrame, prefix: str) -> list[str]:
    return sorted(
        [
            name
            for name in frame.columns
            if name.startswith(prefix) and len(name) == len(prefix) + 2 and name[-2:].isdigit()
        ]
    )


def build_first_step_transition_focus_table(
    current_prior_df: pd.DataFrame,
    current_post_df: pd.DataFrame,
    kf_prior_df: pd.DataFrame,
    kf_post_df: pd.DataFrame,
) -> pd.DataFrame:
    shared_post_updates = sorted(set(current_post_df["update_index"]).intersection(kf_post_df["update_index"]))
    shared_prior_updates = set(current_prior_df["update_index"]).intersection(kf_prior_df["update_index"])

    focus_pair: tuple[int, int] | None = None
    for update_index in shared_post_updates:
        next_update = int(update_index) + 1
        if next_update in shared_prior_updates:
            focus_pair = (int(update_index), next_update)
            break
    if focus_pair is None:
        return pd.DataFrame()

    from_update, to_update = focus_pair
    current_post_row = current_post_df.loc[current_post_df["update_index"] == from_update].iloc[0]
    kf_post_row = kf_post_df.loc[kf_post_df["update_index"] == from_update].iloc[0]
    current_prior_row = current_prior_df.loc[current_prior_df["update_index"] == to_update].iloc[0]
    kf_prior_row = kf_prior_df.loc[kf_prior_df["update_index"] == to_update].iloc[0]

    block_specs = {
        "cov_pos": ("prior_cov_pos_", "post_cov_pos_"),
        "cov_pos_att": ("prior_cov_pos_att_", "post_cov_pos_att_"),
        "cov_att": ("prior_cov_att_", "post_cov_att_"),
    }

    record: dict[str, Any] = {
        "from_update_index": from_update,
        "to_update_index": to_update,
        "current_post_gnss_t": float(current_post_row["gnss_t"]),
        "current_prior_gnss_t": float(current_prior_row["gnss_t"]),
        "kf_post_gnss_t": float(kf_post_row["gnss_t"]),
        "kf_prior_gnss_t": float(kf_prior_row["gnss_t"]),
        "current_predict_dt_s": float(current_prior_row["gnss_t"] - current_post_row["gnss_t"]),
        "kf_predict_dt_s": float(kf_prior_row["gnss_t"] - kf_post_row["gnss_t"]),
    }

    for block_name, (prior_prefix, post_prefix) in block_specs.items():
        current_prior_cols = _sorted_prefix_columns(current_prior_df, prior_prefix)
        current_post_cols = _sorted_prefix_columns(current_post_df, post_prefix)
        kf_prior_cols = _sorted_prefix_columns(kf_prior_df, prior_prefix)
        kf_post_cols = _sorted_prefix_columns(kf_post_df, post_prefix)
        if not current_prior_cols or not current_post_cols or not kf_prior_cols or not kf_post_cols:
            raise KeyError(f"missing transition columns for block {block_name}")
        if not (
            len(current_prior_cols)
            == len(current_post_cols)
            == len(kf_prior_cols)
            == len(kf_post_cols)
        ):
            raise ValueError(f"inconsistent transition block dimensions for {block_name}")

        current_prior_values = current_prior_row.loc[current_prior_cols].to_numpy(dtype=float)
        current_post_values = current_post_row.loc[current_post_cols].to_numpy(dtype=float)
        kf_prior_values = kf_prior_row.loc[kf_prior_cols].to_numpy(dtype=float)
        kf_post_values = kf_post_row.loc[kf_post_cols].to_numpy(dtype=float)

        current_predict_delta = current_prior_values - current_post_values
        kf_predict_delta = kf_prior_values - kf_post_values

        record[f"diff_post_{block_name}_norm"] = float(np.linalg.norm(current_post_values - kf_post_values))
        record[f"diff_prior_{block_name}_norm"] = float(np.linalg.norm(current_prior_values - kf_prior_values))
        record[f"current_predict_delta_{block_name}_norm"] = float(np.linalg.norm(current_predict_delta))
        record[f"kf_predict_delta_{block_name}_norm"] = float(np.linalg.norm(kf_predict_delta))
        record[f"diff_predict_delta_{block_name}_norm"] = float(
            np.linalg.norm(current_predict_delta - kf_predict_delta)
        )

    return pd.DataFrame([record])


def infer_first_predict_window_from_updates(current_df: pd.DataFrame) -> tuple[float, float]:
    ordered = current_df.sort_values(by="update_index").reset_index(drop=True)
    if len(ordered) < 2:
        raise ValueError("need at least two GNSS updates to infer first predict window")
    return float(ordered.iloc[0]["gnss_t"]), float(ordered.iloc[1]["gnss_t"])


def _extract_block(mat: np.ndarray, spec: tuple[slice, slice]) -> np.ndarray:
    row_slice, col_slice = spec
    return mat[row_slice, col_slice]


def _build_predict_debug_df(raw: pd.DataFrame | Path, context: str) -> pd.DataFrame:
    frame = _load_csv_or_df(raw).copy().reset_index(drop=True)
    if "step_index" not in frame.columns:
        frame["step_index"] = np.arange(len(frame), dtype=int)
    if context == "current":
        _ensure_required_columns(frame, PREDICT_DEBUG_REQUIRED_CURRENT, context)
        after_raw_col = "p_after_raw_common_mat"
        after_final_col = "p_after_final_common_mat"
    elif context == "kf":
        _ensure_required_columns(frame, PREDICT_DEBUG_REQUIRED_KF, context)
        after_raw_col = "p_after_common_mat"
        after_final_col = "p_after_common_mat"
    else:
        raise ValueError(f"unsupported context: {context}")

    rows: list[dict[str, Any]] = []
    for _, row in frame.iterrows():
        p_before = _parse_matrix_field(row["p_before_common_mat"], 21, 21, f"{context}.p_before_common_mat")
        phi = _parse_matrix_field(row["phi_common_mat"], 21, 21, f"{context}.phi_common_mat")
        qd = _parse_matrix_field(row["qd_common_mat"], 21, 21, f"{context}.qd_common_mat")
        phi_p = _parse_matrix_field(row["phi_p_common_mat"], 21, 21, f"{context}.phi_p_common_mat")
        p_after_raw = _parse_matrix_field(row[after_raw_col], 21, 21, f"{context}.{after_raw_col}")
        p_after_final = _parse_matrix_field(row[after_final_col], 21, 21, f"{context}.{after_final_col}")

        record: dict[str, Any] = {
            "step_index": int(row["step_index"]),
            "tag": str(row["tag"]),
            "t_prev": float(row["t_prev"]),
            "t_curr": float(row["t_curr"]),
            "dt": float(row["dt"]),
            "raw_reconstruction_error_norm": float(np.linalg.norm(phi_p + qd - p_after_raw)),
            "final_minus_raw_norm": float(np.linalg.norm(p_after_final - p_after_raw)),
        }
        term_mats = {
            "p_before": p_before,
            "phi": phi,
            "phi_p": phi_p,
            "qd": qd,
            "p_after_raw": p_after_raw,
            "p_after_final": p_after_final,
        }
        for term_name, mat in term_mats.items():
            for block_name, spec in PREDICT_BLOCK_SPECS.items():
                block = _extract_block(mat, spec)
                record.update(_flatten_matrix(f"{term_name}_{block_name}", block))
                record[f"{term_name}_{block_name}_norm"] = float(np.linalg.norm(block))
        rows.append(record)
    return pd.DataFrame(rows)


def build_current_predict_debug_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_predict_debug_df(raw, "current")


def build_kf_predict_debug_df(raw: pd.DataFrame | Path) -> pd.DataFrame:
    return _build_predict_debug_df(raw, "kf")


def align_predict_steps(
    current_df: pd.DataFrame,
    kf_df: pd.DataFrame,
) -> pd.DataFrame:
    current = current_df.copy().reset_index(drop=True)
    kf = kf_df.copy().reset_index(drop=True)
    current["t_prev_key"] = current["t_prev"].round(6)
    current["t_curr_key"] = current["t_curr"].round(6)
    kf["t_prev_key"] = kf["t_prev"].round(6)
    kf["t_curr_key"] = kf["t_curr"].round(6)

    passthrough = [
        "step_index",
        "tag",
        "t_prev",
        "t_curr",
        "dt",
        "raw_reconstruction_error_norm",
        "final_minus_raw_norm",
    ]
    compare_columns = [
        column
        for column in current.columns
        if any(
            column.startswith(prefix)
            for prefix in ("p_before_", "phi_", "phi_p_", "qd_", "p_after_raw_", "p_after_final_")
        )
    ]

    current = current.loc[:, ["t_prev_key", "t_curr_key", *passthrough, *compare_columns]].rename(
        columns={name: f"current_{name}" for name in passthrough + compare_columns}
    )
    kf = kf.loc[:, ["t_prev_key", "t_curr_key", *passthrough, *compare_columns]].rename(
        columns={name: f"kf_{name}" for name in passthrough + compare_columns}
    )
    merged = current.merge(kf, on=["t_prev_key", "t_curr_key"], how="inner").sort_values(
        by=["t_prev_key", "t_curr_key"]
    ).reset_index(drop=True)
    merged["t_prev_diff_s"] = merged["current_t_prev"] - merged["kf_t_prev"]
    merged["t_curr_diff_s"] = merged["current_t_curr"] - merged["kf_t_curr"]
    merged["dt_diff_s"] = merged["current_dt"] - merged["kf_dt"]

    for column in compare_columns:
        merged[f"diff_{column}"] = merged[f"current_{column}"] - merged[f"kf_{column}"]
    return merged


def summarize_predict_step_deltas(aligned_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    term_names = ["p_before", "phi_p", "qd", "p_after_raw", "p_after_final"]
    for term_name in term_names:
        for block_name, spec in PREDICT_BLOCK_SPECS.items():
            diff_cols = [
                f"diff_{term_name}_{block_name}_{row}{col}"
                for row in range(spec[0].stop - spec[0].start)
                for col in range(spec[1].stop - spec[1].start)
            ]
            _ensure_required_columns(aligned_df, diff_cols, f"predict_aligned[{term_name}.{block_name}]")
            values = aligned_df.loc[:, diff_cols].to_numpy(dtype=float)
            rows.append(
                {
                    "term": term_name,
                    "block": block_name,
                    "samples": int(values.shape[0]),
                    "mean_abs_component_diff": float(np.mean(np.abs(values))),
                    "max_abs_component_diff": float(np.max(np.abs(values))),
                    "mean_vector_diff_norm": float(np.mean(np.linalg.norm(values, axis=1))),
                    "max_vector_diff_norm": float(np.max(np.linalg.norm(values, axis=1))),
                }
            )
    return pd.DataFrame(rows)


def build_predict_step_focus_table(aligned_df: pd.DataFrame, top_k: int = 8) -> pd.DataFrame:
    if aligned_df.empty:
        return pd.DataFrame()
    order = aligned_df.sort_values(
        by=["diff_p_after_final_cov_pos_norm", "diff_phi_p_cov_pos_norm", "diff_qd_cov_pos_norm"],
        ascending=[False, False, False],
    )
    cols = [
        "current_tag",
        "current_step_index",
        "current_t_prev",
        "current_t_curr",
        "current_dt",
        "diff_p_before_cov_pos_norm",
        "diff_phi_p_cov_pos_norm",
        "diff_qd_cov_pos_norm",
        "diff_p_after_raw_cov_pos_norm",
        "diff_p_after_final_cov_pos_norm",
        "diff_phi_p_cov_pos_att_norm",
        "diff_qd_cov_pos_att_norm",
        "diff_p_after_final_cov_pos_att_norm",
        "diff_phi_p_cov_pos_vel_norm",
        "diff_p_after_final_cov_pos_vel_norm",
        "diff_phi_p_cov_vel_vel_norm",
        "diff_p_after_final_cov_vel_vel_norm",
        "current_raw_reconstruction_error_norm",
        "kf_raw_reconstruction_error_norm",
        "current_final_minus_raw_norm",
        "kf_final_minus_raw_norm",
    ]
    return order.loc[:, cols].head(top_k).reset_index(drop=True)


def align_first_n_updates(current_df: pd.DataFrame, kf_df: pd.DataFrame, limit: int = COMPARE_LIMIT_DEFAULT) -> pd.DataFrame:
    current = current_df.sort_values(by="update_index").reset_index(drop=True).copy()
    kf = kf_df.sort_values(by="update_index").reset_index(drop=True).copy()
    if limit <= 0:
        raise ValueError("limit must be positive")

    for column in ["gnss_t", *SHARED_COMPONENT_COLUMNS]:
        if column not in current.columns:
            current[column] = np.nan
        if column not in kf.columns:
            kf[column] = np.nan

    keep = min(int(limit), len(current), len(kf))
    current = current.iloc[:keep].copy()
    kf = kf.iloc[:keep].copy()

    current_cols = ["update_index", "gnss_t", *SHARED_COMPONENT_COLUMNS]
    kf_cols = ["update_index", "gnss_t", *SHARED_COMPONENT_COLUMNS]
    current = current.loc[:, current_cols].rename(columns={name: f"current_{name}" for name in current_cols if name != "update_index"})
    kf = kf.loc[:, kf_cols].rename(columns={name: f"kf_{name}" for name in kf_cols if name != "update_index"})

    merged = current.merge(kf, on="update_index", how="inner").sort_values(by="update_index").reset_index(drop=True)
    merged["gnss_t_diff_s"] = merged["current_gnss_t"] - merged["kf_gnss_t"]
    for name in SHARED_COMPONENT_COLUMNS:
        merged[f"diff_{name}"] = merged[f"current_{name}"] - merged[f"kf_{name}"]
        merged[f"sign_aligned_diff_{name}"] = merged[f"current_{name}"] + merged[f"kf_{name}"]
    return merged


def align_ba_x_decomposition(
    current_df: pd.DataFrame,
    kf_df: pd.DataFrame,
    limit: int = COMPARE_LIMIT_DEFAULT,
) -> pd.DataFrame:
    if limit <= 0:
        raise ValueError("limit must be positive")

    current = current_df.sort_values(by="update_index").reset_index(drop=True).copy()
    kf = kf_df.sort_values(by="update_index").reset_index(drop=True).copy()
    keep = min(int(limit), len(current), len(kf))
    current = current.iloc[:keep].copy()
    kf = kf.iloc[:keep].copy()

    signed_columns = [name for names in BA_X_DECOMP_SIGNED_BLOCKS.values() for name in names]
    invariant_columns = [name for names in BA_X_DECOMP_INVARIANT_BLOCKS.values() for name in names]
    passthrough_columns = ["update_index", "gnss_t", "k_ba_x_reconstruction_error_norm"]

    current_cols = passthrough_columns + signed_columns + invariant_columns
    kf_cols = passthrough_columns + signed_columns + invariant_columns
    current = current.loc[:, current_cols].rename(columns={name: f"current_{name}" for name in current_cols if name != "update_index"})
    kf = kf.loc[:, kf_cols].rename(columns={name: f"kf_{name}" for name in kf_cols if name != "update_index"})

    merged = current.merge(kf, on="update_index", how="inner").sort_values(by="update_index").reset_index(drop=True)
    merged["gnss_t_diff_s"] = merged["current_gnss_t"] - merged["kf_gnss_t"]
    for name in signed_columns:
        merged[f"diff_{name}"] = merged[f"current_{name}"] - merged[f"kf_{name}"]
        merged[f"sign_aligned_diff_{name}"] = merged[f"current_{name}"] + merged[f"kf_{name}"]
    for name in invariant_columns:
        merged[f"diff_{name}"] = merged[f"current_{name}"] - merged[f"kf_{name}"]
    return merged


def align_s_decomposition(
    current_df: pd.DataFrame,
    kf_df: pd.DataFrame,
    limit: int = COMPARE_LIMIT_DEFAULT,
) -> pd.DataFrame:
    if limit <= 0:
        raise ValueError("limit must be positive")

    current = current_df.sort_values(by="update_index").reset_index(drop=True).copy()
    kf = kf_df.sort_values(by="update_index").reset_index(drop=True).copy()
    keep = min(int(limit), len(current), len(kf))
    current = current.iloc[:keep].copy()
    kf = kf.iloc[:keep].copy()

    block_columns = [name for names in S_DECOMP_BLOCKS.values() for name in names]
    passthrough_columns = ["update_index", "gnss_t", "s_reconstruction_error_norm"]

    current_cols = passthrough_columns + block_columns
    kf_cols = passthrough_columns + block_columns
    current = current.loc[:, current_cols].rename(columns={name: f"current_{name}" for name in current_cols if name != "update_index"})
    kf = kf.loc[:, kf_cols].rename(columns={name: f"kf_{name}" for name in kf_cols if name != "update_index"})

    merged = current.merge(kf, on="update_index", how="inner").sort_values(by="update_index").reset_index(drop=True)
    merged["gnss_t_diff_s"] = merged["current_gnss_t"] - merged["kf_gnss_t"]
    for name in block_columns:
        merged[f"diff_{name}"] = merged[f"current_{name}"] - merged[f"kf_{name}"]
    return merged


def summarize_named_block_deltas(
    aligned_df: pd.DataFrame,
    block_components: dict[str, list[str]],
    diff_prefix: str,
) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for block_name, columns in block_components.items():
        diff_cols = [f"{diff_prefix}{name}" for name in columns]
        _ensure_required_columns(aligned_df, diff_cols, f"aligned_df[{block_name}]")
        values = aligned_df.loc[:, diff_cols].to_numpy(dtype=float)
        rows.append(
            {
                "block": block_name,
                "samples": int(values.shape[0]),
                "mean_abs_component_diff": float(np.mean(np.abs(values))),
                "max_abs_component_diff": float(np.max(np.abs(values))),
                "mean_vector_diff_norm": float(np.mean(np.linalg.norm(values, axis=1))),
                "max_vector_diff_norm": float(np.max(np.linalg.norm(values, axis=1))),
            }
        )
    return pd.DataFrame(rows)


def build_ba_x_decomposition_summary(aligned_df: pd.DataFrame) -> pd.DataFrame:
    signed_summary = summarize_named_block_deltas(aligned_df, BA_X_DECOMP_SIGNED_BLOCKS, "sign_aligned_diff_")
    signed_summary.insert(1, "comparison_mode", "sign_aligned")

    invariant_summary = summarize_named_block_deltas(aligned_df, BA_X_DECOMP_INVARIANT_BLOCKS, "diff_")
    invariant_summary.insert(1, "comparison_mode", "plain_diff")

    return pd.concat([signed_summary, invariant_summary], ignore_index=True)


def build_s_decomposition_summary(aligned_df: pd.DataFrame) -> pd.DataFrame:
    summary = summarize_named_block_deltas(aligned_df, S_DECOMP_BLOCKS, "diff_")
    summary.insert(1, "comparison_mode", "plain_diff")
    return summary


def build_ba_x_decomposition_focus_table(aligned_df: pd.DataFrame) -> pd.DataFrame:
    focus_updates = [1, 47, 48, 49, 50]
    rows: list[dict[str, Any]] = []
    for update_index in focus_updates:
        subset = aligned_df.loc[aligned_df["update_index"] == update_index]
        if subset.empty:
            continue
        row = subset.iloc[0]

        def block_norm(names: list[str], prefix: str) -> float:
            values = row.loc[[f"{prefix}{name}" for name in names]].to_numpy(dtype=float)
            return float(np.linalg.norm(values))

        rows.append(
            {
                "update_index": int(update_index),
                "gnss_t": float(row["current_gnss_t"]),
                "sign_aligned_diff_num_ba_x_from_pos_norm": block_norm(
                    BA_X_DECOMP_SIGNED_BLOCKS["num_ba_x_from_pos"], "sign_aligned_diff_"
                ),
                "sign_aligned_diff_num_ba_x_from_att_norm": block_norm(
                    BA_X_DECOMP_SIGNED_BLOCKS["num_ba_x_from_att"], "sign_aligned_diff_"
                ),
                "sign_aligned_diff_num_ba_x_from_gnss_lever_norm": block_norm(
                    BA_X_DECOMP_SIGNED_BLOCKS["num_ba_x_from_gnss_lever"], "sign_aligned_diff_"
                ),
                "sign_aligned_diff_num_ba_x_total_norm": block_norm(
                    BA_X_DECOMP_SIGNED_BLOCKS["num_ba_x_total"], "sign_aligned_diff_"
                ),
                "diff_s_mat_norm": block_norm(BA_X_DECOMP_INVARIANT_BLOCKS["s_mat"], "diff_"),
                "sign_aligned_diff_k_ba_x_norm": block_norm(
                    BA_X_DECOMP_SIGNED_BLOCKS["k_ba_x"], "sign_aligned_diff_"
                ),
                "sign_aligned_diff_dx_ba_x_abs": abs(float(row["sign_aligned_diff_dx_ba_x"])),
                "current_k_ba_x_reconstruction_error_norm": float(row["current_k_ba_x_reconstruction_error_norm"]),
                "kf_k_ba_x_reconstruction_error_norm": float(row["kf_k_ba_x_reconstruction_error_norm"]),
            }
        )
    return pd.DataFrame(rows)


def build_s_decomposition_focus_table(aligned_df: pd.DataFrame) -> pd.DataFrame:
    focus_updates = [1, 47, 48, 49, 50]
    rows: list[dict[str, Any]] = []
    for update_index in focus_updates:
        subset = aligned_df.loc[aligned_df["update_index"] == update_index]
        if subset.empty:
            continue
        row = subset.iloc[0]

        def block_norm(names: list[str]) -> float:
            values = row.loc[[f"diff_{name}" for name in names]].to_numpy(dtype=float)
            return float(np.linalg.norm(values))

        rows.append(
            {
                "update_index": int(update_index),
                "gnss_t": float(row["current_gnss_t"]),
                "diff_s_from_pos_pos_norm": block_norm(S_DECOMP_BLOCKS["s_from_pos_pos"]),
                "diff_s_from_pos_att_cross_norm": block_norm(S_DECOMP_BLOCKS["s_from_pos_att_cross"]),
                "diff_s_from_att_att_norm": block_norm(S_DECOMP_BLOCKS["s_from_att_att"]),
                "diff_s_from_pos_gnss_lever_cross_norm": block_norm(S_DECOMP_BLOCKS["s_from_pos_gnss_lever_cross"]),
                "diff_s_from_att_gnss_lever_cross_norm": block_norm(S_DECOMP_BLOCKS["s_from_att_gnss_lever_cross"]),
                "diff_s_from_gnss_lever_gnss_lever_norm": block_norm(S_DECOMP_BLOCKS["s_from_gnss_lever_gnss_lever"]),
                "diff_r_mat_norm": block_norm(S_DECOMP_BLOCKS["r_mat"]),
                "diff_s_mat_norm": block_norm(S_DECOMP_BLOCKS["s_mat"]),
                "current_s_reconstruction_error_norm": float(row["current_s_reconstruction_error_norm"]),
                "kf_s_reconstruction_error_norm": float(row["kf_s_reconstruction_error_norm"]),
            }
        )
    return pd.DataFrame(rows)


def summarize_feedback_deltas(aligned_df: pd.DataFrame, diff_prefix: str = "diff_") -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for block_name, columns in BLOCK_COMPONENTS.items():
        diff_cols = [f"{diff_prefix}{name}" for name in columns]
        _ensure_required_columns(aligned_df, diff_cols, f"aligned_df[{block_name}]")
        values = aligned_df.loc[:, diff_cols].to_numpy(dtype=float)
        rows.append(
            {
                "block": block_name,
                "samples": int(values.shape[0]),
                "mean_abs_component_diff": float(np.mean(np.abs(values))),
                "max_abs_component_diff": float(np.max(np.abs(values))),
                "mean_vector_diff_norm": float(np.mean(np.linalg.norm(values, axis=1))),
                "max_vector_diff_norm": float(np.max(np.linalg.norm(values, axis=1))),
            }
        )
    return pd.DataFrame(rows)


def render_summary_markdown(
    raw_summary_df: pd.DataFrame,
    sign_aligned_summary_df: pd.DataFrame,
    aligned_df: pd.DataFrame,
    ba_x_summary_df: pd.DataFrame,
    ba_x_focus_df: pd.DataFrame,
    s_summary_df: pd.DataFrame | None = None,
    s_focus_df: pd.DataFrame | None = None,
    first_step_focus_df: pd.DataFrame | None = None,
    predict_summary_df: pd.DataFrame | None = None,
    predict_focus_df: pd.DataFrame | None = None,
) -> str:
    if s_summary_df is None:
        s_summary_df = pd.DataFrame(columns=["block", "comparison_mode", "samples", "mean_abs_component_diff", "max_abs_component_diff", "mean_vector_diff_norm", "max_vector_diff_norm"])
    if s_focus_df is None:
        s_focus_df = pd.DataFrame(
            columns=[
                "update_index",
                "gnss_t",
                "diff_s_from_pos_pos_norm",
                "diff_s_from_pos_att_cross_norm",
                "diff_s_from_att_att_norm",
                "diff_s_from_pos_gnss_lever_cross_norm",
                "diff_s_from_att_gnss_lever_cross_norm",
                "diff_s_from_gnss_lever_gnss_lever_norm",
                "diff_r_mat_norm",
                "diff_s_mat_norm",
                "current_s_reconstruction_error_norm",
                "kf_s_reconstruction_error_norm",
            ]
        )
    if first_step_focus_df is None:
        first_step_focus_df = pd.DataFrame(
            columns=[
                "from_update_index",
                "to_update_index",
                "diff_post_cov_pos_norm",
                "diff_post_cov_pos_att_norm",
                "diff_post_cov_att_norm",
                "diff_prior_cov_pos_norm",
                "diff_prior_cov_pos_att_norm",
                "diff_prior_cov_att_norm",
                "current_predict_delta_cov_pos_norm",
                "kf_predict_delta_cov_pos_norm",
                "diff_predict_delta_cov_pos_norm",
                "current_predict_delta_cov_pos_att_norm",
                "kf_predict_delta_cov_pos_att_norm",
                "diff_predict_delta_cov_pos_att_norm",
                "current_predict_delta_cov_att_norm",
                "kf_predict_delta_cov_att_norm",
                "diff_predict_delta_cov_att_norm",
            ]
        )
    if predict_summary_df is None:
        predict_summary_df = pd.DataFrame(
            columns=[
                "term",
                "block",
                "samples",
                "mean_abs_component_diff",
                "max_abs_component_diff",
                "mean_vector_diff_norm",
                "max_vector_diff_norm",
            ]
        )
    if predict_focus_df is None:
        predict_focus_df = pd.DataFrame(
            columns=[
                "current_tag",
                "current_step_index",
                "current_t_prev",
                "current_t_curr",
                "current_dt",
                "diff_p_before_cov_pos_norm",
                "diff_phi_p_cov_pos_norm",
                "diff_qd_cov_pos_norm",
                "diff_p_after_raw_cov_pos_norm",
                "diff_p_after_final_cov_pos_norm",
                "diff_phi_p_cov_pos_att_norm",
                "diff_qd_cov_pos_att_norm",
                "diff_p_after_final_cov_pos_att_norm",
                "diff_phi_p_cov_pos_vel_norm",
                "diff_p_after_final_cov_pos_vel_norm",
                "diff_phi_p_cov_vel_vel_norm",
                "diff_p_after_final_cov_vel_vel_norm",
                "current_raw_reconstruction_error_norm",
                "kf_raw_reconstruction_error_norm",
                "current_final_minus_raw_norm",
                "kf_final_minus_raw_norm",
            ]
        )
    lines = [
        "# GNSS Update Feedback Compare",
        "",
        f"- generated_at: `{dt.datetime.now().isoformat(timespec='seconds')}`",
        f"- compared_updates: `{len(aligned_df)}`",
        f"- mean_abs_time_delta_s: `{aligned_df['gnss_t_diff_s'].abs().mean():.9f}`",
        "",
        "## Raw Current Minus KF-GINS",
        "",
        "| block | samples | mean_abs_component_diff | max_abs_component_diff | mean_vector_diff_norm | max_vector_diff_norm |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for _, row in raw_summary_df.iterrows():
        lines.append(
            "| {block} | {samples} | {mean_abs_component_diff:.9g} | {max_abs_component_diff:.9g} | {mean_vector_diff_norm:.9g} | {max_vector_diff_norm:.9g} |".format(
                **row.to_dict()
            )
        )
    lines.extend(
        [
            "",
            "## Sign-Aligned Current Plus KF-GINS",
            "",
            "The second table removes the dominant sign-convention flip in `innovation/dx` and is more suitable for locating true chain differences.",
            "",
            "| block | samples | mean_abs_component_diff | max_abs_component_diff | mean_vector_diff_norm | max_vector_diff_norm |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in sign_aligned_summary_df.iterrows():
        lines.append(
            "| {block} | {samples} | {mean_abs_component_diff:.9g} | {max_abs_component_diff:.9g} | {mean_vector_diff_norm:.9g} | {max_vector_diff_norm:.9g} |".format(
                **row.to_dict()
            )
        )
    lines.extend(
        [
            "",
            "## Sign-Aware PH^T Numerator / S / K_ba_x Decomposition",
            "",
            "Signed blocks use `current + KF-GINS` to absorb the dominant sign-convention flip. `s_mat` stays on plain `current - KF-GINS` because it is sign-invariant.",
            "",
            "| block | comparison_mode | samples | mean_abs_component_diff | max_abs_component_diff | mean_vector_diff_norm | max_vector_diff_norm |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in ba_x_summary_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {block} | {comparison_mode} | {samples} | {mean_abs_component_diff:.9g} | {max_abs_component_diff:.9g} | {mean_vector_diff_norm:.9g} | {max_vector_diff_norm:.9g} |".format(
                block=row_dict["block"],
                comparison_mode=row_dict.get("comparison_mode", ""),
                samples=row_dict["samples"],
                mean_abs_component_diff=row_dict["mean_abs_component_diff"],
                max_abs_component_diff=row_dict["max_abs_component_diff"],
                mean_vector_diff_norm=row_dict["mean_vector_diff_norm"],
                max_vector_diff_norm=row_dict["max_vector_diff_norm"],
            )
        )
    lines.extend(
        [
            "",
            "### Focus Updates",
            "",
            "These rows isolate the first large divergence and the `528124~528127 s` window.",
            "",
            "| update_index | gnss_t | sign_aligned_diff_num_ba_x_from_pos_norm | sign_aligned_diff_num_ba_x_from_att_norm | sign_aligned_diff_num_ba_x_from_gnss_lever_norm | sign_aligned_diff_num_ba_x_total_norm | diff_s_mat_norm | sign_aligned_diff_k_ba_x_norm | sign_aligned_diff_dx_ba_x_abs | current_k_ba_x_reconstruction_error_norm | kf_k_ba_x_reconstruction_error_norm |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in ba_x_focus_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {update_index} | {gnss_t:.9f} | {sign_aligned_diff_num_ba_x_from_pos_norm:.9g} | {sign_aligned_diff_num_ba_x_from_att_norm:.9g} | {sign_aligned_diff_num_ba_x_from_gnss_lever_norm:.9g} | {sign_aligned_diff_num_ba_x_total_norm:.9g} | {diff_s_mat_norm:.9g} | {sign_aligned_diff_k_ba_x_norm:.9g} | {sign_aligned_diff_dx_ba_x_abs:.9g} | {current_k_ba_x_reconstruction_error_norm:.9g} | {kf_k_ba_x_reconstruction_error_norm:.9g} |".format(
                update_index=int(row_dict["update_index"]),
                gnss_t=float(row_dict["gnss_t"]),
                sign_aligned_diff_num_ba_x_from_pos_norm=float(row_dict.get("sign_aligned_diff_num_ba_x_from_pos_norm", float("nan"))),
                sign_aligned_diff_num_ba_x_from_att_norm=float(row_dict.get("sign_aligned_diff_num_ba_x_from_att_norm", float("nan"))),
                sign_aligned_diff_num_ba_x_from_gnss_lever_norm=float(row_dict.get("sign_aligned_diff_num_ba_x_from_gnss_lever_norm", float("nan"))),
                sign_aligned_diff_num_ba_x_total_norm=float(row_dict.get("sign_aligned_diff_num_ba_x_total_norm", float("nan"))),
                diff_s_mat_norm=float(row_dict.get("diff_s_mat_norm", float("nan"))),
                sign_aligned_diff_k_ba_x_norm=float(row_dict.get("sign_aligned_diff_k_ba_x_norm", float("nan"))),
                sign_aligned_diff_dx_ba_x_abs=float(row_dict.get("sign_aligned_diff_dx_ba_x_abs", float("nan"))),
                current_k_ba_x_reconstruction_error_norm=float(
                    row_dict.get("current_k_ba_x_reconstruction_error_norm", float("nan"))
                ),
                kf_k_ba_x_reconstruction_error_norm=float(
                    row_dict.get("kf_k_ba_x_reconstruction_error_norm", float("nan"))
                ),
            )
        )
    lines.extend(
        [
            "",
            "## S = HPH^T + R Decomposition",
            "",
            "All rows below use plain `current - KF-GINS` because `S` and its sub-terms are sign-invariant.",
            "",
            "| block | comparison_mode | samples | mean_abs_component_diff | max_abs_component_diff | mean_vector_diff_norm | max_vector_diff_norm |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in s_summary_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {block} | {comparison_mode} | {samples} | {mean_abs_component_diff:.9g} | {max_abs_component_diff:.9g} | {mean_vector_diff_norm:.9g} | {max_vector_diff_norm:.9g} |".format(
                block=row_dict["block"],
                comparison_mode=row_dict.get("comparison_mode", ""),
                samples=row_dict["samples"],
                mean_abs_component_diff=row_dict["mean_abs_component_diff"],
                max_abs_component_diff=row_dict["max_abs_component_diff"],
                mean_vector_diff_norm=row_dict["mean_vector_diff_norm"],
                max_vector_diff_norm=row_dict["max_vector_diff_norm"],
            )
        )
    lines.extend(
        [
            "",
            "### S Focus Updates",
            "",
            "| update_index | gnss_t | diff_s_from_pos_pos_norm | diff_s_from_pos_att_cross_norm | diff_s_from_att_att_norm | diff_s_from_pos_gnss_lever_cross_norm | diff_s_from_att_gnss_lever_cross_norm | diff_s_from_gnss_lever_gnss_lever_norm | diff_r_mat_norm | diff_s_mat_norm | current_s_reconstruction_error_norm | kf_s_reconstruction_error_norm |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in s_focus_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {update_index} | {gnss_t:.9f} | {diff_s_from_pos_pos_norm:.9g} | {diff_s_from_pos_att_cross_norm:.9g} | {diff_s_from_att_att_norm:.9g} | {diff_s_from_pos_gnss_lever_cross_norm:.9g} | {diff_s_from_att_gnss_lever_cross_norm:.9g} | {diff_s_from_gnss_lever_gnss_lever_norm:.9g} | {diff_r_mat_norm:.9g} | {diff_s_mat_norm:.9g} | {current_s_reconstruction_error_norm:.9g} | {kf_s_reconstruction_error_norm:.9g} |".format(
                update_index=int(row_dict["update_index"]),
                gnss_t=float(row_dict["gnss_t"]),
                diff_s_from_pos_pos_norm=float(row_dict.get("diff_s_from_pos_pos_norm", float("nan"))),
                diff_s_from_pos_att_cross_norm=float(row_dict.get("diff_s_from_pos_att_cross_norm", float("nan"))),
                diff_s_from_att_att_norm=float(row_dict.get("diff_s_from_att_att_norm", float("nan"))),
                diff_s_from_pos_gnss_lever_cross_norm=float(
                    row_dict.get("diff_s_from_pos_gnss_lever_cross_norm", float("nan"))
                ),
                diff_s_from_att_gnss_lever_cross_norm=float(
                    row_dict.get("diff_s_from_att_gnss_lever_cross_norm", float("nan"))
                ),
                diff_s_from_gnss_lever_gnss_lever_norm=float(
                    row_dict.get("diff_s_from_gnss_lever_gnss_lever_norm", float("nan"))
                ),
                diff_r_mat_norm=float(row_dict.get("diff_r_mat_norm", float("nan"))),
                diff_s_mat_norm=float(row_dict.get("diff_s_mat_norm", float("nan"))),
                current_s_reconstruction_error_norm=float(
                    row_dict.get("current_s_reconstruction_error_norm", float("nan"))
                ),
                kf_s_reconstruction_error_norm=float(row_dict.get("kf_s_reconstruction_error_norm", float("nan"))),
            )
        )
    lines.extend(
        [
            "",
            "## First-Step Posterior vs Predict Delta",
            "",
            "This isolates whether the first large `update1` gap already exists in `update0` posterior covariance or is created by the predict from `update0 -> update1`.",
            "",
            "| from_update_index | to_update_index | diff_post_cov_pos_norm | diff_post_cov_pos_att_norm | diff_post_cov_att_norm | diff_prior_cov_pos_norm | diff_prior_cov_pos_att_norm | diff_prior_cov_att_norm | current_predict_delta_cov_pos_norm | kf_predict_delta_cov_pos_norm | diff_predict_delta_cov_pos_norm | current_predict_delta_cov_pos_att_norm | kf_predict_delta_cov_pos_att_norm | diff_predict_delta_cov_pos_att_norm | current_predict_delta_cov_att_norm | kf_predict_delta_cov_att_norm | diff_predict_delta_cov_att_norm |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in first_step_focus_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {from_update_index} | {to_update_index} | {diff_post_cov_pos_norm:.9g} | {diff_post_cov_pos_att_norm:.9g} | {diff_post_cov_att_norm:.9g} | {diff_prior_cov_pos_norm:.9g} | {diff_prior_cov_pos_att_norm:.9g} | {diff_prior_cov_att_norm:.9g} | {current_predict_delta_cov_pos_norm:.9g} | {kf_predict_delta_cov_pos_norm:.9g} | {diff_predict_delta_cov_pos_norm:.9g} | {current_predict_delta_cov_pos_att_norm:.9g} | {kf_predict_delta_cov_pos_att_norm:.9g} | {diff_predict_delta_cov_pos_att_norm:.9g} | {current_predict_delta_cov_att_norm:.9g} | {kf_predict_delta_cov_att_norm:.9g} | {diff_predict_delta_cov_att_norm:.9g} |".format(
                from_update_index=int(row_dict["from_update_index"]),
                to_update_index=int(row_dict["to_update_index"]),
                diff_post_cov_pos_norm=float(row_dict.get("diff_post_cov_pos_norm", float("nan"))),
                diff_post_cov_pos_att_norm=float(row_dict.get("diff_post_cov_pos_att_norm", float("nan"))),
                diff_post_cov_att_norm=float(row_dict.get("diff_post_cov_att_norm", float("nan"))),
                diff_prior_cov_pos_norm=float(row_dict.get("diff_prior_cov_pos_norm", float("nan"))),
                diff_prior_cov_pos_att_norm=float(row_dict.get("diff_prior_cov_pos_att_norm", float("nan"))),
                diff_prior_cov_att_norm=float(row_dict.get("diff_prior_cov_att_norm", float("nan"))),
                current_predict_delta_cov_pos_norm=float(
                    row_dict.get("current_predict_delta_cov_pos_norm", float("nan"))
                ),
                kf_predict_delta_cov_pos_norm=float(row_dict.get("kf_predict_delta_cov_pos_norm", float("nan"))),
                diff_predict_delta_cov_pos_norm=float(
                    row_dict.get("diff_predict_delta_cov_pos_norm", float("nan"))
                ),
                current_predict_delta_cov_pos_att_norm=float(
                    row_dict.get("current_predict_delta_cov_pos_att_norm", float("nan"))
                ),
                kf_predict_delta_cov_pos_att_norm=float(
                    row_dict.get("kf_predict_delta_cov_pos_att_norm", float("nan"))
                ),
                diff_predict_delta_cov_pos_att_norm=float(
                    row_dict.get("diff_predict_delta_cov_pos_att_norm", float("nan"))
                ),
                current_predict_delta_cov_att_norm=float(
                    row_dict.get("current_predict_delta_cov_att_norm", float("nan"))
                ),
                kf_predict_delta_cov_att_norm=float(row_dict.get("kf_predict_delta_cov_att_norm", float("nan"))),
                diff_predict_delta_cov_att_norm=float(
                    row_dict.get("diff_predict_delta_cov_att_norm", float("nan"))
                ),
            )
        )
    lines.extend(
        [
            "",
            "## Predict Decomposition Inside First GNSS Window",
            "",
            "These rows use the predict-debug window inferred from the first two effective GNSS updates and split each predict step into `Phi P Phi^T` and `Qd`.",
            "",
            "| term | block | samples | mean_abs_component_diff | max_abs_component_diff | mean_vector_diff_norm | max_vector_diff_norm |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in predict_summary_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {term} | {block} | {samples} | {mean_abs_component_diff:.9g} | {max_abs_component_diff:.9g} | {mean_vector_diff_norm:.9g} | {max_vector_diff_norm:.9g} |".format(
                term=row_dict["term"],
                block=row_dict["block"],
                samples=row_dict["samples"],
                mean_abs_component_diff=row_dict["mean_abs_component_diff"],
                max_abs_component_diff=row_dict["max_abs_component_diff"],
                mean_vector_diff_norm=row_dict["mean_vector_diff_norm"],
                max_vector_diff_norm=row_dict["max_vector_diff_norm"],
            )
        )
    lines.extend(
        [
            "",
            "### Top Predict Steps By `cov_pos` Gap",
            "",
            "| tag | step_index | t_prev | t_curr | dt | diff_p_before_cov_pos_norm | diff_phi_p_cov_pos_norm | diff_qd_cov_pos_norm | diff_p_after_raw_cov_pos_norm | diff_p_after_final_cov_pos_norm | diff_phi_p_cov_pos_att_norm | diff_qd_cov_pos_att_norm | diff_p_after_final_cov_pos_att_norm | diff_phi_p_cov_pos_vel_norm | diff_p_after_final_cov_pos_vel_norm | diff_phi_p_cov_vel_vel_norm | diff_p_after_final_cov_vel_vel_norm | current_raw_reconstruction_error_norm | kf_raw_reconstruction_error_norm | current_final_minus_raw_norm | kf_final_minus_raw_norm |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in predict_focus_df.iterrows():
        row_dict = row.to_dict()
        lines.append(
            "| {current_tag} | {current_step_index} | {current_t_prev:.9f} | {current_t_curr:.9f} | {current_dt:.9g} | {diff_p_before_cov_pos_norm:.9g} | {diff_phi_p_cov_pos_norm:.9g} | {diff_qd_cov_pos_norm:.9g} | {diff_p_after_raw_cov_pos_norm:.9g} | {diff_p_after_final_cov_pos_norm:.9g} | {diff_phi_p_cov_pos_att_norm:.9g} | {diff_qd_cov_pos_att_norm:.9g} | {diff_p_after_final_cov_pos_att_norm:.9g} | {diff_phi_p_cov_pos_vel_norm:.9g} | {diff_p_after_final_cov_pos_vel_norm:.9g} | {diff_phi_p_cov_vel_vel_norm:.9g} | {diff_p_after_final_cov_vel_vel_norm:.9g} | {current_raw_reconstruction_error_norm:.9g} | {kf_raw_reconstruction_error_norm:.9g} | {current_final_minus_raw_norm:.9g} | {kf_final_minus_raw_norm:.9g} |".format(
                **row_dict
            )
        )
    return "\n".join(lines) + "\n"


def run_current_solver(exe_path: Path, config_path: Path, stdout_path: Path) -> str:
    stdout_text = run_command([str(exe_path.resolve()), "--config", str(config_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    return stdout_text


def build_kf_runtime_paths(
    runtime_mode: str,
    runtime_root: Path,
    output_dir: Path,
    kf_exe: Path,
    source_cfg: dict[str, Any],
    repo_gnss_path: Path,
    predict_window: tuple[float, float] | None = None,
) -> dict[str, Path]:
    repo_output_dir = output_dir / KF_CASE_ID
    config_path = repo_output_dir / "config_kf_gins_gnss_feedback_compare.yaml"
    stdout_log_path = repo_output_dir / "kf_gins_stdout.txt"
    debug_output_path = repo_output_dir / "gnss_updates_kf_gins.csv"
    predict_output_path = repo_output_dir / "predict_steps_kf_gins.csv"
    ensure_dir(repo_output_dir)

    imu_src = (REPO_ROOT / source_cfg["fusion"]["imu_path"]).resolve()
    if runtime_mode == "preferred_root":
        cfg = build_kf_gins_feedback_config(
            source_cfg,
            imupath=imu_src,
            gnsspath=repo_gnss_path.resolve(),
            outputpath=repo_output_dir.resolve(),
            gnss_update_debug_path=debug_output_path.resolve(),
            predict_debug_path=predict_output_path.resolve(),
            predict_window=predict_window,
        )
        save_yaml(cfg, config_path)
        return {
            "exe_path": kf_exe.resolve(),
            "config_path": config_path.resolve(),
            "stdout_log_path": stdout_log_path.resolve(),
            "runtime_output_dir": repo_output_dir.resolve(),
            "debug_output_path": debug_output_path.resolve(),
            "predict_output_path": predict_output_path.resolve(),
        }

    run_root = (runtime_root / output_dir.name / KF_CASE_ID).resolve()
    if run_root.exists():
        shutil.rmtree(run_root)
    bin_dir = run_root / "bin"
    inputs_dir = run_root / "inputs"
    output_subdir = run_root / "output"
    config_dir = run_root / "config"
    logs_dir = run_root / "logs"
    for path in (bin_dir, inputs_dir, output_subdir, config_dir, logs_dir):
        ensure_dir(path)

    exe_dst = bin_dir / kf_exe.name
    imu_dst = inputs_dir / imu_src.name
    gnss_dst = inputs_dir / repo_gnss_path.name
    config_dst = config_dir / config_path.name
    stdout_dst = logs_dir / stdout_log_path.name
    debug_dst = output_subdir / debug_output_path.name
    predict_dst = output_subdir / predict_output_path.name

    copy_file(kf_exe.resolve(), exe_dst)
    copy_file(imu_src, imu_dst)
    copy_file(repo_gnss_path.resolve(), gnss_dst)

    cfg = build_kf_gins_feedback_config(
        source_cfg,
        imupath=imu_dst,
        gnsspath=gnss_dst,
        outputpath=output_subdir,
        gnss_update_debug_path=debug_dst,
        predict_debug_path=predict_dst,
        predict_window=predict_window,
    )
    save_yaml(cfg, config_dst)
    return {
        "exe_path": exe_dst.resolve(),
        "config_path": config_dst.resolve(),
        "stdout_log_path": stdout_dst.resolve(),
        "runtime_output_dir": output_subdir.resolve(),
        "debug_output_path": debug_dst.resolve(),
        "predict_output_path": predict_dst.resolve(),
        "repo_debug_output_path": debug_output_path.resolve(),
        "repo_predict_output_path": predict_output_path.resolve(),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare current solver vs KF-GINS GNSS update feedback over the first N GNSS update epochs."
    )
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--current-exe", type=Path, default=CURRENT_SOLVER_DEFAULT)
    parser.add_argument("--kf-exe", type=Path, default=KF_GINS_EXE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--compare-limit", type=int, default=COMPARE_LIMIT_DEFAULT)
    parser.add_argument("--runtime-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--fallback-runtime-root", type=Path, default=FALLBACK_ROOT_DEFAULT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_cfg = load_yaml(normalize_repo_path(args.base_config))
    output_dir = normalize_repo_path(args.output_dir)
    ensure_dir(output_dir)

    current_cfg, metadata = build_current_compare_config(
        base_cfg,
        output_dir,
        compare_limit_updates=args.compare_limit,
    )
    current_case_dir = output_dir / CURRENT_CASE_ID
    ensure_dir(current_case_dir)
    current_cfg_path = current_case_dir / "config_current_solver.yaml"
    current_stdout_path = current_case_dir / "current_solver_stdout.txt"
    current_debug_path = REPO_ROOT / current_cfg["fusion"]["gnss_update_debug_output_path"]
    save_yaml(current_cfg, current_cfg_path)
    run_current_solver(normalize_repo_path(args.current_exe), current_cfg_path, current_stdout_path)
    if not current_debug_path.exists():
        raise FileNotFoundError(f"missing current solver GNSS debug output: {current_debug_path}")
    current_norm = normalize_current_gnss_update_df(current_debug_path)
    predict_window = infer_first_predict_window_from_updates(current_norm)
    current_predict_debug_repo_path = output_dir / CURRENT_CASE_ID / "predict_steps_current_solver.csv"
    current_cfg, metadata = build_current_compare_config(
        base_cfg,
        output_dir,
        compare_limit_updates=args.compare_limit,
        predict_debug_path=current_predict_debug_repo_path,
        predict_window=predict_window,
    )
    current_debug_path = REPO_ROOT / current_cfg["fusion"]["gnss_update_debug_output_path"]
    current_predict_debug_path = REPO_ROOT / current_cfg["fusion"]["predict_debug_output_path"]
    save_yaml(current_cfg, current_cfg_path)
    run_current_solver(normalize_repo_path(args.current_exe), current_cfg_path, current_stdout_path)
    if not current_debug_path.exists():
        raise FileNotFoundError(f"missing current solver GNSS debug output after predict rerun: {current_debug_path}")
    if not current_predict_debug_path.exists():
        raise FileNotFoundError(f"missing current solver predict debug output: {current_predict_debug_path}")

    preferred_root = normalize_repo_path(args.runtime_root)
    fallback_root = normalize_repo_path(args.fallback_runtime_root)
    runtime_root, runtime_mode = resolve_runtime_root(preferred_root, fallback_root)
    repo_gnss_path = (REPO_ROOT / current_cfg["fusion"]["gnss_path"]).resolve()
    kf_paths = build_kf_runtime_paths(
        runtime_mode=runtime_mode,
        runtime_root=runtime_root,
        output_dir=output_dir,
        kf_exe=normalize_repo_path(args.kf_exe),
        source_cfg=current_cfg,
        repo_gnss_path=repo_gnss_path,
        predict_window=predict_window,
    )
    run_kf_gins(kf_paths["exe_path"], kf_paths["config_path"], kf_paths["stdout_log_path"])

    kf_debug_path = kf_paths["debug_output_path"]
    kf_predict_debug_path = kf_paths["predict_output_path"]
    if runtime_mode != "preferred_root":
        repo_kf_debug_path = kf_paths["repo_debug_output_path"]
        copy_file(kf_debug_path, repo_kf_debug_path)
        kf_debug_path = repo_kf_debug_path
        repo_kf_predict_debug_path = kf_paths["repo_predict_output_path"]
        copy_file(kf_predict_debug_path, repo_kf_predict_debug_path)
        kf_predict_debug_path = repo_kf_predict_debug_path
    if not kf_debug_path.exists():
        raise FileNotFoundError(f"missing KF-GINS GNSS debug output: {kf_debug_path}")
    if not kf_predict_debug_path.exists():
        raise FileNotFoundError(f"missing KF-GINS predict debug output: {kf_predict_debug_path}")

    current_norm = normalize_current_gnss_update_df(current_debug_path)
    kf_norm = normalize_kf_gins_gnss_update_df(kf_debug_path)
    aligned = align_first_n_updates(current_norm, kf_norm, limit=args.compare_limit)
    summary = summarize_feedback_deltas(aligned)
    sign_aligned_summary = summarize_feedback_deltas(aligned, diff_prefix="sign_aligned_diff_")
    current_ba_x_decomposition = build_current_ba_x_decomposition_df(current_debug_path)
    kf_ba_x_decomposition = build_kf_gins_ba_x_decomposition_df(kf_debug_path)
    ba_x_aligned = align_ba_x_decomposition(
        current_ba_x_decomposition,
        kf_ba_x_decomposition,
        limit=args.compare_limit,
    )
    ba_x_summary = build_ba_x_decomposition_summary(ba_x_aligned)
    ba_x_focus = build_ba_x_decomposition_focus_table(ba_x_aligned)
    current_s_decomposition = build_current_s_decomposition_df(current_debug_path)
    kf_s_decomposition = build_kf_gins_s_decomposition_df(kf_debug_path)
    s_aligned = align_s_decomposition(current_s_decomposition, kf_s_decomposition, limit=args.compare_limit)
    s_summary = build_s_decomposition_summary(s_aligned)
    s_focus = build_s_decomposition_focus_table(s_aligned)
    current_posterior_covariance = build_current_posterior_covariance_df(current_debug_path)
    kf_posterior_covariance = build_kf_gins_posterior_covariance_df(kf_debug_path)
    current_posterior_covariance_limited = current_posterior_covariance.iloc[: args.compare_limit].copy()
    kf_posterior_covariance_limited = kf_posterior_covariance.iloc[: args.compare_limit].copy()
    first_step_transition_focus = build_first_step_transition_focus_table(
        current_posterior_covariance_limited,
        current_posterior_covariance_limited,
        kf_posterior_covariance_limited,
        kf_posterior_covariance_limited,
    )
    current_predict_debug = build_current_predict_debug_df(current_predict_debug_path)
    kf_predict_debug = build_kf_predict_debug_df(kf_predict_debug_path)
    predict_aligned = align_predict_steps(current_predict_debug, kf_predict_debug)
    if predict_aligned.empty:
        raise RuntimeError("predict debug alignment is empty")
    predict_summary = summarize_predict_step_deltas(predict_aligned)
    predict_focus = build_predict_step_focus_table(predict_aligned)

    compare_csv_path = output_dir / "gnss_update_compare_first500.csv"
    summary_csv_path = output_dir / "gnss_update_block_summary_first500.csv"
    sign_aligned_summary_csv_path = output_dir / "gnss_update_block_summary_first500_sign_aligned.csv"
    ba_x_decomposition_csv_path = output_dir / "gnss_update_ba_x_decomposition_first500.csv"
    ba_x_decomposition_summary_csv_path = output_dir / "gnss_update_ba_x_decomposition_summary_first500.csv"
    ba_x_decomposition_focus_csv_path = output_dir / "gnss_update_ba_x_decomposition_focus_updates.csv"
    s_decomposition_csv_path = output_dir / "gnss_update_s_decomposition_first500.csv"
    s_decomposition_summary_csv_path = output_dir / "gnss_update_s_decomposition_summary_first500.csv"
    s_decomposition_focus_csv_path = output_dir / "gnss_update_s_decomposition_focus_updates.csv"
    current_posterior_covariance_csv_path = output_dir / "gnss_update_current_posterior_covariance_first500.csv"
    kf_posterior_covariance_csv_path = output_dir / "gnss_update_kf_posterior_covariance_first500.csv"
    first_step_transition_focus_csv_path = output_dir / "gnss_update_first_step_transition_focus.csv"
    predict_compare_csv_path = output_dir / "gnss_update_first_window_predict_compare.csv"
    predict_summary_csv_path = output_dir / "gnss_update_first_window_predict_summary.csv"
    predict_focus_csv_path = output_dir / "gnss_update_first_window_predict_focus.csv"
    summary_md_path = output_dir / "summary.md"
    manifest_path = output_dir / "manifest.json"

    aligned.to_csv(compare_csv_path, index=False)
    summary.to_csv(summary_csv_path, index=False)
    sign_aligned_summary.to_csv(sign_aligned_summary_csv_path, index=False)
    ba_x_aligned.to_csv(ba_x_decomposition_csv_path, index=False)
    ba_x_summary.to_csv(ba_x_decomposition_summary_csv_path, index=False)
    ba_x_focus.to_csv(ba_x_decomposition_focus_csv_path, index=False)
    s_aligned.to_csv(s_decomposition_csv_path, index=False)
    s_summary.to_csv(s_decomposition_summary_csv_path, index=False)
    s_focus.to_csv(s_decomposition_focus_csv_path, index=False)
    current_posterior_covariance_limited.to_csv(current_posterior_covariance_csv_path, index=False)
    kf_posterior_covariance_limited.to_csv(kf_posterior_covariance_csv_path, index=False)
    first_step_transition_focus.to_csv(first_step_transition_focus_csv_path, index=False)
    predict_aligned.to_csv(predict_compare_csv_path, index=False)
    predict_summary.to_csv(predict_summary_csv_path, index=False)
    predict_focus.to_csv(predict_focus_csv_path, index=False)
    summary_md_path.write_text(
        render_summary_markdown(
            summary,
            sign_aligned_summary,
            aligned,
            ba_x_summary,
            ba_x_focus,
            s_summary,
            s_focus,
            first_step_transition_focus,
            predict_summary,
            predict_focus,
        ),
        encoding="utf-8",
    )

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "compare_limit_updates": int(args.compare_limit),
        "runtime_mode": runtime_mode,
        "runtime_root": str(runtime_root),
        "current_config_path": rel_from_root(current_cfg_path, REPO_ROOT),
        "current_stdout_path": rel_from_root(current_stdout_path, REPO_ROOT),
        "current_gnss_update_debug_path": rel_from_root(current_debug_path, REPO_ROOT),
        "current_predict_debug_path": rel_from_root(current_predict_debug_path, REPO_ROOT),
        "kf_config_path": rel_from_root(kf_paths["config_path"], REPO_ROOT)
        if kf_paths["config_path"].is_relative_to(REPO_ROOT)
        else str(kf_paths["config_path"]),
        "kf_stdout_path": rel_from_root(kf_paths["stdout_log_path"], REPO_ROOT)
        if kf_paths["stdout_log_path"].is_relative_to(REPO_ROOT)
        else str(kf_paths["stdout_log_path"]),
        "kf_gnss_update_debug_path": rel_from_root(kf_debug_path, REPO_ROOT),
        "kf_predict_debug_path": rel_from_root(kf_predict_debug_path, REPO_ROOT),
        "compare_csv_path": rel_from_root(compare_csv_path, REPO_ROOT),
        "summary_csv_path": rel_from_root(summary_csv_path, REPO_ROOT),
        "sign_aligned_summary_csv_path": rel_from_root(sign_aligned_summary_csv_path, REPO_ROOT),
        "ba_x_decomposition_csv_path": rel_from_root(ba_x_decomposition_csv_path, REPO_ROOT),
        "ba_x_decomposition_summary_csv_path": rel_from_root(ba_x_decomposition_summary_csv_path, REPO_ROOT),
        "ba_x_decomposition_focus_csv_path": rel_from_root(ba_x_decomposition_focus_csv_path, REPO_ROOT),
        "s_decomposition_csv_path": rel_from_root(s_decomposition_csv_path, REPO_ROOT),
        "s_decomposition_summary_csv_path": rel_from_root(s_decomposition_summary_csv_path, REPO_ROOT),
        "s_decomposition_focus_csv_path": rel_from_root(s_decomposition_focus_csv_path, REPO_ROOT),
        "current_posterior_covariance_csv_path": rel_from_root(current_posterior_covariance_csv_path, REPO_ROOT),
        "kf_posterior_covariance_csv_path": rel_from_root(kf_posterior_covariance_csv_path, REPO_ROOT),
        "first_step_transition_focus_csv_path": rel_from_root(first_step_transition_focus_csv_path, REPO_ROOT),
        "predict_window": [float(predict_window[0]), float(predict_window[1])],
        "predict_compare_csv_path": rel_from_root(predict_compare_csv_path, REPO_ROOT),
        "predict_summary_csv_path": rel_from_root(predict_summary_csv_path, REPO_ROOT),
        "predict_focus_csv_path": rel_from_root(predict_focus_csv_path, REPO_ROOT),
        "summary_md_path": rel_from_root(summary_md_path, REPO_ROOT),
        "metadata": json_safe(metadata),
    }
    manifest_path.write_text(json.dumps(json_safe(manifest), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
