from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FIRST_UPDATE = REPO_ROOT / "output/data2_gnss_lever_bias_attribution/artifacts/b/truth_anchor_all_non_pva/first_update_truth_anchor_all_non_pva.csv"
DEFAULT_POS = REPO_ROOT / "dataset/data2_converted/POS_converted.txt"
DEFAULT_RTK = REPO_ROOT / "dataset/data2/rtk.txt"
DEFAULT_REFERENCE = REPO_ROOT / "output/data2_gnss_lever_bias_attribution/implied_lever_reference.csv"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "output/data2_gnss_lever_bias_attribution"


def parse_vec_field(raw: Any) -> np.ndarray:
    text = str(raw).strip().strip("[]")
    if not text:
        return np.zeros(0, dtype=float)
    return np.array([float(part) for part in text.split(";")], dtype=float)


def llh_to_ecef(lat_deg: float, lon_deg: float, h_m: float) -> np.ndarray:
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * cos_lon
    y = (n + h_m) * cos_lat * sin_lon
    z = (n * (1.0 - e2) + h_m) * sin_lat
    return np.array([x, y, z], dtype=float)


def rot_ned_to_ecef(lat_rad: float, lon_rad: float) -> np.ndarray:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    return np.array(
        [
            [-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
            [-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon],
            [cos_lat, 0.0, -sin_lat],
        ],
        dtype=float,
    )


def quat_to_rot(q_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(q_wxyz, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def euler_to_rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def interpolate_pose(pos_df: pd.DataFrame, t_query: float) -> dict[str, float]:
    ts = pos_df["t"].to_numpy(dtype=float)
    cols = ["lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"]
    return {col: float(np.interp(t_query, ts, pos_df[col].to_numpy(dtype=float))) for col in cols}


def select_nearest_row(df: pd.DataFrame, time_col: str, t_query: float, tol_s: float = 1.0e-6) -> pd.Series:
    times = df[time_col].to_numpy(dtype=float)
    idx = int(np.argmin(np.abs(times - t_query)))
    if abs(float(times[idx]) - t_query) > tol_s:
        raise RuntimeError(f"failed to locate row at t={t_query:.9f} within tolerance {tol_s}")
    return df.iloc[idx]


def load_reference_vector(reference_df: pd.DataFrame, name: str) -> np.ndarray:
    row = reference_df.loc[reference_df["reference_name"] == name]
    if row.empty:
        raise RuntimeError(f"missing reference vector: {name}")
    return row.iloc[0][["x_m", "y_m", "z_m"]].to_numpy(dtype=float)


def project_ratio(component: np.ndarray, total: np.ndarray) -> float:
    denom = float(total.dot(total))
    if denom <= 0.0:
        return math.nan
    return float(component.dot(total) / denom)


def component_row(name: str, vec: np.ndarray, total: np.ndarray, note: str) -> dict[str, Any]:
    return {
        "component": name,
        "x_north_m": float(vec[0]),
        "y_east_m": float(vec[1]),
        "z_down_m": float(vec[2]),
        "norm_m": float(np.linalg.norm(vec)),
        "projection_ratio_to_total": project_ratio(vec, total),
        "note": note,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze the source of the first GNSS_POS residual.")
    parser.add_argument("--first-update-csv", type=Path, default=DEFAULT_FIRST_UPDATE)
    parser.add_argument("--pos-path", type=Path, default=DEFAULT_POS)
    parser.add_argument("--rtk-path", type=Path, default=DEFAULT_RTK)
    parser.add_argument("--reference-csv", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    first_update_df = pd.read_csv(args.first_update_csv)
    pos_df = pd.read_csv(
        args.pos_path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
    )
    rtk_df = pd.read_csv(
        args.rtk_path,
        sep=r"\s+",
        header=None,
        usecols=list(range(7)),
        names=["t", "lat", "lon", "h", "std_n", "std_e", "std_d"],
    )
    reference_df = pd.read_csv(args.reference_csv)

    if first_update_df.empty:
        raise RuntimeError("first update csv is empty")
    if first_update_df["gnss_axis"].tolist()[:3] != ["x", "y", "z"]:
        raise RuntimeError("unexpected first-update axis order; expected x/y/z rows")

    row = first_update_df.iloc[0]
    gnss_t = float(row["gnss_t"])
    state_t = float(row["state_t"])
    y_total = parse_vec_field(row["y_vec"])
    if y_total.size != 3:
        raise RuntimeError("first update residual must be 3D")

    lever_before = np.array(
        [
            float(first_update_df.iloc[0]["lever_before"]),
            float(first_update_df.iloc[1]["lever_before"]),
            float(first_update_df.iloc[2]["lever_before"]),
        ],
        dtype=float,
    )
    state_p_before = parse_vec_field(row["state_p_before_ecef"])
    state_v_before = parse_vec_field(row["state_v_before_ecef"])
    state_q_before = parse_vec_field(row["state_q_before_wxyz"])

    truth_g = interpolate_pose(pos_df, gnss_t)
    truth_s = interpolate_pose(pos_df, state_t)
    r_ne_state = rot_ned_to_ecef(math.radians(truth_s["lat"]), math.radians(truth_s["lon"]))

    implied_lever = load_reference_vector(reference_df, "RTK_POS_body_median")
    readme_lever = load_reference_vector(reference_df, "README_nominal")

    rtk_pose = select_nearest_row(rtk_df, "t", gnss_t)
    z_gnss_ecef = llh_to_ecef(float(rtk_pose["lat"]), float(rtk_pose["lon"]), float(rtk_pose["h"]))

    truth_p_g = llh_to_ecef(truth_g["lat"], truth_g["lon"], truth_g["h"])
    truth_p_s = llh_to_ecef(truth_s["lat"], truth_s["lon"], truth_s["h"])
    r_ne_g = rot_ned_to_ecef(math.radians(truth_g["lat"]), math.radians(truth_g["lon"]))
    c_be_truth_g = r_ne_g @ euler_to_rotation(truth_g["roll"], truth_g["pitch"], truth_g["yaw"])
    c_be_truth_s = r_ne_state @ euler_to_rotation(truth_s["roll"], truth_s["pitch"], truth_s["yaw"])
    truth_ant_g = truth_p_g + c_be_truth_g @ implied_lever
    truth_ant_s = truth_p_s + c_be_truth_s @ implied_lever
    truth_ant_s_readme = truth_p_s + c_be_truth_s @ readme_lever

    c_be_filter = quat_to_rot(state_q_before)
    filter_ant_before = state_p_before + c_be_filter @ lever_before
    z_reconstructed = filter_ant_before + r_ne_state @ y_total

    residual_meas = r_ne_state.T @ (z_gnss_ecef - truth_ant_g)
    residual_dt = r_ne_state.T @ (truth_ant_g - truth_ant_s)
    residual_prop = r_ne_state.T @ (truth_ant_s - filter_ant_before)
    residual_sum = residual_meas + residual_dt + residual_prop
    residual_without_dt = residual_meas + residual_prop
    residual_readme_vs_implied = r_ne_state.T @ (truth_ant_s_readme - truth_ant_s)

    truth_v_s = r_ne_state @ np.array([truth_s["vn"], truth_s["ve"], truth_s["vd"]], dtype=float)
    state_pos_error = state_p_before - truth_p_s
    state_vel_error = state_v_before - truth_v_s
    c_delta = c_be_truth_s.T @ c_be_filter
    att_error_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, float((np.trace(c_delta) - 1.0) * 0.5))))
    )

    decomposition_rows = [
        component_row("logged_total_residual", y_total, y_total, "Logged GNSS_POS first-update residual."),
        component_row(
            "measurement_minus_truth_at_gnss",
            residual_meas,
            y_total,
            "GNSS measurement minus truth antenna position at gnss_t.",
        ),
        component_row(
            "time_alignment_gnss_to_state",
            residual_dt,
            y_total,
            "Truth antenna motion from gnss_t to state_t; this term exists because the 7-col GNSS file has no velocity for extrapolation.",
        ),
        component_row(
            "filter_prediction_error_at_state_before_update",
            residual_prop,
            y_total,
            "Pre-update filter antenna prediction error at state_t.",
        ),
        component_row(
            "sum_of_components",
            residual_sum,
            y_total,
            "Should match logged_total_residual up to numerical tolerance.",
        ),
        component_row(
            "residual_without_time_alignment",
            residual_without_dt,
            y_total,
            "Residual that would remain if GNSS position were aligned from gnss_t to state_t.",
        ),
        component_row(
            "readme_vs_implied_lever_reference",
            residual_readme_vs_implied,
            y_total,
            "Reference mismatch induced by using README nominal lever instead of the RTK/POS implied lever.",
        ),
    ]

    decomposition_path = output_dir / "first_update_residual_decomposition.csv"
    pd.DataFrame(decomposition_rows).to_csv(decomposition_path, index=False)

    context = {
        "gnss_t": gnss_t,
        "state_t": state_t,
        "dt_align_s": state_t - gnss_t,
        "gnss_column_count": 7,
        "z_reconstruction_error_m": float(np.linalg.norm(z_reconstructed - z_gnss_ecef)),
        "sum_consistency_error_m": float(np.linalg.norm(residual_sum - y_total)),
        "truth_antenna_motion_norm_m": float(np.linalg.norm(truth_ant_s - truth_ant_g)),
        "state_pos_error_norm_m": float(np.linalg.norm(state_pos_error)),
        "state_vel_error_norm_mps": float(np.linalg.norm(state_vel_error)),
        "state_att_error_deg": att_error_deg,
        "first_update_csv": str(args.first_update_csv.relative_to(REPO_ROOT)),
        "pos_path": str(args.pos_path.relative_to(REPO_ROOT)),
        "rtk_path": str(args.rtk_path.relative_to(REPO_ROOT)),
        "reference_csv": str(args.reference_csv.relative_to(REPO_ROOT)),
    }
    context_path = output_dir / "first_update_residual_context.json"
    context_path.write_text(json.dumps(context, indent=2, ensure_ascii=False), encoding="utf-8")

    summary_lines = [
        "# GNSS_POS first-update residual decomposition",
        "",
        "## Key result",
        f"- Logged control residual: `{np.linalg.norm(y_total):.6f} m` at `gnss_t={gnss_t:.6f}` and `state_t={state_t:.6f}`.",
        f"- `dt_align = {state_t - gnss_t:.6f} s`; current GNSS file has `7` columns, so this path has no velocity-based position extrapolation.",
        f"- Time-alignment term: `{np.linalg.norm(residual_dt):.6f} m`, projection ratio to total = `{project_ratio(residual_dt, y_total):.3f}`.",
        f"- Measurement-vs-truth term at `gnss_t`: `{np.linalg.norm(residual_meas):.6f} m`, projection ratio = `{project_ratio(residual_meas, y_total):.3f}`.",
        f"- Pre-update filter prediction error at `state_t`: `{np.linalg.norm(residual_prop):.6f} m`, projection ratio = `{project_ratio(residual_prop, y_total):.3f}`.",
        f"- Residual after removing only the time-alignment term would drop to `{np.linalg.norm(residual_without_dt):.6f} m`.",
        f"- README nominal vs RTK/POS implied lever only contributes `{np.linalg.norm(residual_readme_vs_implied):.6f} m`.",
        "",
        "## Consistency checks",
        f"- Reconstructed GNSS ECEF from logged `y/state_before` matches RTK measurement within `{np.linalg.norm(z_reconstructed - z_gnss_ecef):.6e} m`.",
        f"- Sum of components matches logged residual within `{np.linalg.norm(residual_sum - y_total):.6e} m`.",
        f"- Pre-update state error norms at `state_t`: position `{np.linalg.norm(state_pos_error):.6f} m`, velocity `{np.linalg.norm(state_vel_error):.6f} m/s`, attitude `{att_error_deg:.6f} deg`.",
        "",
        "## Interpretation",
        "- The control residual is not mainly caused by README-vs-dataset lever mismatch.",
        "- The dominant source is the uncompensated `gnss_t -> state_t` motion between the first GNSS epoch and the IMU state timestamp.",
        "- The initialization-only truth PVA startup drift exists, but it is a secondary millimeter-level contributor here.",
    ]
    summary_path = output_dir / "first_update_residual_summary.md"
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    print(f"[OK] wrote {decomposition_path.relative_to(REPO_ROOT)}")
    print(f"[OK] wrote {context_path.relative_to(REPO_ROOT)}")
    print(f"[OK] wrote {summary_path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
