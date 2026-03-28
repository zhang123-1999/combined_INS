from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ID_DEFAULT = "EXP-20260322-data2-nhc-lateral-source-r1"
CONFIG_DEFAULT = Path(
    "output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r3_axis_hard_freeze/"
    "artifacts/cases/release_mounting_yaw/config_release_mounting_yaw.yaml"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
IMU_DEFAULT = Path("dataset/data2_converted/IMU_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_nhc_lateral_source_20260322")
YAW_CANDIDATES_DEFAULT = [1.37, 0.834466, 0.875]
MIN_SPEED_M_S = 3.0
TURN_YAW_RATE_DEG_S = 8.0
STRAIGHT_YAW_RATE_DEG_S = 3.0
STRAIGHT_FAST_MIN_SPEED_M_S = 5.0

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
OMEGA_IE = 7.292115e-5


@dataclass(frozen=True)
class MountingReference:
    total_roll_deg: float
    total_pitch_deg: float
    config_total_yaw_deg: float
    lever_arm_m: np.ndarray
    bg0_rad_s: np.ndarray
    sg0: np.ndarray
    sigma_nhc_y: float
    sigma_nhc_z: float
    gate_prob: float


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT)).replace("\\", "/")
    except ValueError:
        return str(path).replace("\\", "/")


def mtime_text(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")


def load_pos_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )


def load_imu_dataframe(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "dtheta_x", "dtheta_y", "dtheta_z", "dv_x", "dv_y", "dv_z"],
        engine="python",
    )


def wrap_deg(angle_deg: np.ndarray) -> np.ndarray:
    return (angle_deg + 180.0) % 360.0 - 180.0


def euler_to_rot_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def compute_earth_radius(lat_rad: float) -> tuple[float, float]:
    sin_lat = math.sin(lat_rad)
    sin2 = sin_lat * sin_lat
    rm = WGS84_A * (1.0 - WGS84_E2) / ((1.0 - WGS84_E2 * sin2) ** 1.5)
    rn = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin2)
    return rm, rn


def omega_ie_ned(lat_rad: float) -> np.ndarray:
    return np.array(
        [OMEGA_IE * math.cos(lat_rad), 0.0, -OMEGA_IE * math.sin(lat_rad)], dtype=float
    )


def omega_en_ned(v_ned: np.ndarray, lat_rad: float, h_m: float) -> np.ndarray:
    rm, rn = compute_earth_radius(lat_rad)
    v_n, v_e, _ = v_ned
    return np.array(
        [v_e / (rn + h_m), -v_n / (rm + h_m), -v_e * math.tan(lat_rad) / (rn + h_m)],
        dtype=float,
    )


def resolve_mounting_reference(fusion_cfg: dict) -> MountingReference:
    constraints = fusion_cfg["constraints"]
    init_cfg = fusion_cfg["init"]
    mounting_base = np.asarray(constraints["imu_mounting_angle"], dtype=float)
    use_legacy = bool(init_cfg.get("use_legacy_mounting_base_logic", True))

    init_roll = float(init_cfg.get("mounting_roll0", 0.0))
    init_pitch = float(init_cfg.get("mounting_pitch0", 0.0))
    init_yaw = float(init_cfg.get("mounting_yaw0", 0.0))
    pitch_base = mounting_base[1]
    yaw_base = mounting_base[2]
    if use_legacy:
        if abs(init_pitch) > 1e-12:
            pitch_base = 0.0
        if abs(init_yaw) > 1e-12:
            yaw_base = 0.0

    return MountingReference(
        total_roll_deg=float(mounting_base[0] + init_roll),
        total_pitch_deg=float(pitch_base + init_pitch),
        config_total_yaw_deg=float(yaw_base + init_yaw),
        lever_arm_m=np.asarray(init_cfg["lever_arm0"], dtype=float),
        bg0_rad_s=np.asarray(init_cfg["bg0"], dtype=float),
        sg0=np.asarray(init_cfg["sg0"], dtype=float),
        sigma_nhc_y=float(constraints["sigma_nhc_y"]),
        sigma_nhc_z=float(constraints["sigma_nhc_z"]),
        gate_prob=float(constraints.get("nhc_nis_gate_prob", 0.995)),
    )


def classify_motion(speed_m_s: np.ndarray, yaw_rate_deg_s: np.ndarray) -> np.ndarray:
    labels = np.full(speed_m_s.shape, "transition", dtype=object)
    labels[speed_m_s < MIN_SPEED_M_S] = "low_speed"
    labels[(speed_m_s >= MIN_SPEED_M_S) & (np.abs(yaw_rate_deg_s) <= STRAIGHT_YAW_RATE_DEG_S)] = "straight"
    labels[(speed_m_s >= MIN_SPEED_M_S) & (yaw_rate_deg_s >= TURN_YAW_RATE_DEG_S)] = "turn_pos"
    labels[(speed_m_s >= MIN_SPEED_M_S) & (yaw_rate_deg_s <= -TURN_YAW_RATE_DEG_S)] = "turn_neg"
    return labels


def interp_series(query_t: np.ndarray, src_t: np.ndarray, src: np.ndarray, unwrap: bool = False) -> np.ndarray:
    values = src
    if unwrap:
        values = np.unwrap(np.deg2rad(values))
    out = np.interp(query_t, src_t, values)
    return out


def gate_from_prob(prob: float) -> float:
    # chi2(df=2) has closed form CDF = 1 - exp(-x/2)
    return float(-2.0 * math.log(max(1e-12, 1.0 - prob)))


def summarize_scope(
    scope: str,
    yaw_deg: float,
    mask: np.ndarray,
    lat_total: np.ndarray,
    lat_trans: np.ndarray,
    lat_rot: np.ndarray,
    vert_total: np.ndarray,
    speed_m_s: np.ndarray,
    yaw_rate_deg_s: np.ndarray,
    sigma_y: float,
    sigma_z: float,
    gate: float,
) -> dict[str, float | int | str]:
    if not np.any(mask):
        return {}
    lat = lat_total[mask]
    vert = vert_total[mask]
    nis = (lat / sigma_y) ** 2 + (vert / sigma_z) ** 2
    return {
        "scope": scope,
        "yaw_deg": float(yaw_deg),
        "n": int(mask.sum()),
        "count_ratio": float(mask.mean()),
        "lat_mean_mps": float(np.mean(lat)),
        "lat_rmse_mps": float(np.sqrt(np.mean(lat * lat))),
        "lat_std_mps": float(np.std(lat)),
        "lat_trans_mean_mps": float(np.mean(lat_trans[mask])),
        "lat_rot_mean_mps": float(np.mean(lat_rot[mask])),
        "vert_rmse_mps": float(np.sqrt(np.mean(vert * vert))),
        "speed_mean_mps": float(np.mean(speed_m_s[mask])),
        "yaw_rate_mean_deg_s": float(np.mean(yaw_rate_deg_s[mask])),
        "nis_mean": float(np.mean(nis)),
        "accept_ratio": float(np.mean(nis <= gate)),
    }


def render_markdown_table(df: pd.DataFrame, columns: Iterable[str], rename: dict[str, str]) -> list[str]:
    cols = list(columns)
    headers = [rename.get(col, col) for col in cols]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for _, row in df.iterrows():
        vals: list[str] = []
        for col in cols:
            value = row[col]
            if isinstance(value, str):
                vals.append(value)
            elif isinstance(value, (int, np.integer)):
                vals.append(str(int(value)))
            else:
                vals.append(f"{float(value):.6f}")
        lines.append("| " + " | ".join(vals) + " |")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze the physical source of NHC lateral residuals.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--config", type=Path, default=CONFIG_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--imu", type=Path, default=IMU_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--yaw-candidates", type=float, nargs="+", default=YAW_CANDIDATES_DEFAULT)
    args = parser.parse_args()

    config_path = (REPO_ROOT / args.config).resolve()
    pos_path = (REPO_ROOT / args.pos).resolve()
    imu_path = (REPO_ROOT / args.imu).resolve()
    out_dir = (REPO_ROOT / args.output_dir).resolve()
    ensure_dir(out_dir)

    raw_cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    fusion_cfg = raw_cfg["fusion"]
    mounting_ref = resolve_mounting_reference(fusion_cfg)

    pos_df = load_pos_dataframe(pos_path).sort_values(by="t").reset_index(drop=True)
    imu_df = load_imu_dataframe(imu_path).sort_values(by="t").reset_index(drop=True)

    start_t = max(float(pos_df["t"].iloc[0]), float(imu_df["t"].iloc[0]))
    end_t = min(float(pos_df["t"].iloc[-1]), float(imu_df["t"].iloc[-1]))
    imu_df = imu_df[(imu_df["t"] >= start_t) & (imu_df["t"] <= end_t)].reset_index(drop=True)
    t = imu_df["t"].to_numpy(dtype=float)

    pos_t = pos_df["t"].to_numpy(dtype=float)
    lat_rad = np.deg2rad(interp_series(t, pos_t, pos_df["lat"].to_numpy(dtype=float)))
    lon_rad = np.deg2rad(interp_series(t, pos_t, pos_df["lon"].to_numpy(dtype=float)))
    h_m = interp_series(t, pos_t, pos_df["h"].to_numpy(dtype=float))
    vn = interp_series(t, pos_t, pos_df["vn"].to_numpy(dtype=float))
    ve = interp_series(t, pos_t, pos_df["ve"].to_numpy(dtype=float))
    vd = interp_series(t, pos_t, pos_df["vd"].to_numpy(dtype=float))
    roll_rad = interp_series(t, pos_t, pos_df["roll"].to_numpy(dtype=float), unwrap=True)
    pitch_rad = interp_series(t, pos_t, pos_df["pitch"].to_numpy(dtype=float), unwrap=True)
    yaw_rad = interp_series(t, pos_t, pos_df["yaw"].to_numpy(dtype=float), unwrap=True)
    yaw_rate_rad_s = np.interp(
        t,
        pos_t,
        np.gradient(np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float))), pos_t),
    )

    dt = np.diff(t, prepend=t[0])
    if dt.size > 1:
        dt[0] = dt[1]
    dt = np.clip(dt, 1e-6, None)
    omega_ib_b_raw = imu_df[["dtheta_x", "dtheta_y", "dtheta_z"]].to_numpy(dtype=float) / dt[:, None]

    v_b = np.zeros((t.size, 3), dtype=float)
    v_rot = np.zeros((t.size, 3), dtype=float)
    v_wheel_b = np.zeros((t.size, 3), dtype=float)
    for idx in range(t.size):
        c_bn = euler_to_rot_zyx(float(roll_rad[idx]), float(pitch_rad[idx]), float(yaw_rad[idx]))
        v_ned = np.array([vn[idx], ve[idx], vd[idx]], dtype=float)
        v_body = c_bn.T @ v_ned
        omega_in_b = c_bn.T @ (
            omega_ie_ned(float(lat_rad[idx])) + omega_en_ned(v_ned, float(lat_rad[idx]), float(h_m[idx]))
        )
        omega_ib_corr = (1.0 - mounting_ref.sg0) * (omega_ib_b_raw[idx] - mounting_ref.bg0_rad_s)
        omega_nb_b = omega_ib_corr - omega_in_b
        v_lever = np.cross(omega_nb_b, mounting_ref.lever_arm_m)
        v_b[idx] = v_body
        v_rot[idx] = v_lever
        v_wheel_b[idx] = v_body + v_lever

    speed_m_s = np.hypot(vn, ve)
    yaw_rate_deg_s = np.rad2deg(yaw_rate_rad_s)
    motion_label = classify_motion(speed_m_s, yaw_rate_deg_s)

    course_deg = np.rad2deg(np.arctan2(ve, vn))
    heading_deg = np.rad2deg(yaw_rad)
    heading_minus_course_deg = wrap_deg(heading_deg - course_deg)
    body_beta_deg = np.rad2deg(np.arctan2(v_b[:, 1], np.maximum(1e-9, v_b[:, 0])))
    wheel_beta_deg = np.rad2deg(np.arctan2(v_wheel_b[:, 1], np.maximum(1e-9, v_wheel_b[:, 0])))

    gate = gate_from_prob(mounting_ref.gate_prob)
    summary_rows: list[dict[str, float | int | str]] = []
    per_sample = pd.DataFrame(
        {
            "t": t,
            "speed_m_s": speed_m_s,
            "yaw_rate_deg_s": yaw_rate_deg_s,
            "motion_label": motion_label,
            "heading_minus_course_deg": heading_minus_course_deg,
            "body_beta_deg": body_beta_deg,
            "wheel_beta_deg": wheel_beta_deg,
            "body_lat_mps": v_b[:, 1],
            "lever_lat_mps": v_rot[:, 1],
            "wheel_lat_body_mps": v_wheel_b[:, 1],
            "wheel_fwd_body_mps": v_wheel_b[:, 0],
        }
    )

    yaw_candidates = [float(v) for v in args.yaw_candidates]
    truth_yaw_deg = yaw_candidates[0]
    basin_yaw_deg = yaw_candidates[1] if len(yaw_candidates) > 1 else yaw_candidates[0]

    truth_lat_total = None
    basin_lat_total = None
    truth_lat_rot = None
    for yaw_deg in yaw_candidates:
        c_b_v = euler_to_rot_zyx(
            math.radians(mounting_ref.total_roll_deg),
            math.radians(mounting_ref.total_pitch_deg),
            math.radians(yaw_deg),
        ).T
        v_vehicle = (c_b_v @ v_wheel_b.T).T
        v_vehicle_trans = (c_b_v @ v_b.T).T
        v_vehicle_rot = (c_b_v @ v_rot.T).T
        lat_total = v_vehicle[:, 1]
        lat_trans = v_vehicle_trans[:, 1]
        lat_rot = v_vehicle_rot[:, 1]
        vert_total = v_vehicle[:, 2]
        fwd_total = v_vehicle[:, 0]

        slug = f"{yaw_deg:.6f}".replace("-", "neg").replace(".", "p")
        per_sample[f"lat_total_yaw_{slug}_mps"] = lat_total
        per_sample[f"lat_trans_yaw_{slug}_mps"] = lat_trans
        per_sample[f"lat_rot_yaw_{slug}_mps"] = lat_rot
        per_sample[f"fwd_yaw_{slug}_mps"] = fwd_total
        per_sample[f"vert_yaw_{slug}_mps"] = vert_total

        summary_rows.append(
            summarize_scope(
                "overall",
                yaw_deg,
                np.ones_like(lat_total, dtype=bool),
                lat_total,
                lat_trans,
                lat_rot,
                vert_total,
                speed_m_s,
                yaw_rate_deg_s,
                mounting_ref.sigma_nhc_y,
                mounting_ref.sigma_nhc_z,
                gate,
            )
        )
        for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]:
            summary_rows.append(
                summarize_scope(
                    label,
                    yaw_deg,
                    motion_label == label,
                    lat_total,
                    lat_trans,
                    lat_rot,
                    vert_total,
                    speed_m_s,
                    yaw_rate_deg_s,
                    mounting_ref.sigma_nhc_y,
                    mounting_ref.sigma_nhc_z,
                    gate,
                )
            )
        straight_fast = (motion_label == "straight") & (speed_m_s >= STRAIGHT_FAST_MIN_SPEED_M_S)
        summary_rows.append(
            summarize_scope(
                "straight_fast",
                yaw_deg,
                straight_fast,
                lat_total,
                lat_trans,
                lat_rot,
                vert_total,
                speed_m_s,
                yaw_rate_deg_s,
                mounting_ref.sigma_nhc_y,
                mounting_ref.sigma_nhc_z,
                gate,
            )
        )

        if abs(yaw_deg - truth_yaw_deg) < 1e-9:
            truth_lat_total = lat_total
            truth_lat_rot = lat_rot
        if abs(yaw_deg - basin_yaw_deg) < 1e-9:
            basin_lat_total = lat_total

    summary_df = pd.DataFrame([row for row in summary_rows if row]).sort_values(by=["yaw_deg", "scope"]).reset_index(
        drop=True
    )
    summary_df.to_csv(out_dir / "candidate_motion_breakdown.csv", index=False)
    per_sample.to_csv(out_dir / "per_sample_lateral_profile.csv", index=False)

    if truth_lat_total is None or truth_lat_rot is None or basin_lat_total is None:
        raise RuntimeError("missing truth/basin candidate in yaw list")

    truth_overall = summary_df[(summary_df["yaw_deg"] == truth_yaw_deg) & (summary_df["scope"] == "overall")].iloc[0]
    truth_straight = summary_df[(summary_df["yaw_deg"] == truth_yaw_deg) & (summary_df["scope"] == "straight")].iloc[0]
    truth_straight_fast = summary_df[
        (summary_df["yaw_deg"] == truth_yaw_deg) & (summary_df["scope"] == "straight_fast")
    ].iloc[0]
    basin_straight = summary_df[(summary_df["yaw_deg"] == basin_yaw_deg) & (summary_df["scope"] == "straight")].iloc[0]
    basin_turn_pos = summary_df[(summary_df["yaw_deg"] == basin_yaw_deg) & (summary_df["scope"] == "turn_pos")].iloc[0]
    basin_turn_neg = summary_df[(summary_df["yaw_deg"] == basin_yaw_deg) & (summary_df["scope"] == "turn_neg")].iloc[0]

    straight_fast_mask = (motion_label == "straight") & (speed_m_s >= STRAIGHT_FAST_MIN_SPEED_M_S)
    regression_x = per_sample.loc[straight_fast_mask, f"fwd_yaw_{truth_yaw_deg:.6f}".replace("-", "neg").replace(".", "p") + "_mps"].to_numpy(dtype=float)
    regression_y = truth_lat_total[straight_fast_mask]
    a = np.column_stack([regression_x, np.ones(regression_x.size)])
    coef, *_ = np.linalg.lstsq(a, regression_y, rcond=None)
    slope, intercept = coef
    pred = a @ coef
    ss_res = float(np.sum((regression_y - pred) ** 2))
    ss_tot = float(np.sum((regression_y - np.mean(regression_y)) ** 2))
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 0.0 else float("nan")

    straight_share = float(
        np.abs(np.sum(truth_lat_total[motion_label == "straight"])) / max(1e-12, np.abs(np.sum(truth_lat_total)))
    )
    heading_course_quantiles = np.percentile(heading_minus_course_deg[straight_fast_mask], [5.0, 50.0, 95.0])
    reference_summary = {
        "exp_id": args.exp_id,
        "time_window_s": [start_t, end_t],
        "n_samples": int(t.size),
        "truth_yaw_deg": truth_yaw_deg,
        "basin_yaw_deg": basin_yaw_deg,
        "yaw_gap_deg": float(truth_yaw_deg - basin_yaw_deg),
        "total_roll_deg": mounting_ref.total_roll_deg,
        "total_pitch_deg": mounting_ref.total_pitch_deg,
        "config_total_yaw_deg": mounting_ref.config_total_yaw_deg,
        "straight_count_ratio": float(np.mean(motion_label == "straight")),
        "straight_share_of_truth_lateral_sum": straight_share,
        "truth_overall_lat_mean_mps": float(truth_overall["lat_mean_mps"]),
        "truth_straight_lat_mean_mps": float(truth_straight["lat_mean_mps"]),
        "basin_straight_lat_mean_mps": float(basin_straight["lat_mean_mps"]),
        "straight_body_lat_mean_mps": float(np.mean(v_b[straight_fast_mask, 1])),
        "straight_lever_lat_mean_mps": float(np.mean(v_rot[straight_fast_mask, 1])),
        "straight_heading_minus_course_mean_deg": float(np.mean(heading_minus_course_deg[straight_fast_mask])),
        "straight_heading_minus_course_std_deg": float(np.std(heading_minus_course_deg[straight_fast_mask])),
        "straight_heading_minus_course_p5_deg": float(heading_course_quantiles[0]),
        "straight_heading_minus_course_p50_deg": float(heading_course_quantiles[1]),
        "straight_heading_minus_course_p95_deg": float(heading_course_quantiles[2]),
        "straight_wheel_beta_mean_deg": float(np.mean(wheel_beta_deg[straight_fast_mask])),
        "straight_wheel_beta_std_deg": float(np.std(wheel_beta_deg[straight_fast_mask])),
        "straight_body_beta_mean_deg": float(np.mean(body_beta_deg[straight_fast_mask])),
        "straight_body_beta_std_deg": float(np.std(body_beta_deg[straight_fast_mask])),
        "truth_regression_slope_lat_per_fwd": float(slope),
        "truth_regression_intercept_mps": float(intercept),
        "truth_regression_r2": r2,
        "truth_equivalent_angle_deg": float(-math.degrees(math.asin(max(-1.0, min(1.0, slope))))),
        "basin_turn_pos_lat_mean_mps": float(basin_turn_pos["lat_mean_mps"]),
        "basin_turn_neg_lat_mean_mps": float(basin_turn_neg["lat_mean_mps"]),
    }
    (out_dir / "reference_summary.json").write_text(json.dumps(reference_summary, indent=2), encoding="utf-8")

    motion_ref = pd.DataFrame(
        {
            "motion_label": ["straight", "turn_pos", "turn_neg", "transition", "low_speed"],
            "count_ratio": [float(np.mean(motion_label == label)) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "wheel_beta_mean_deg": [float(np.mean(wheel_beta_deg[motion_label == label])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "wheel_beta_std_deg": [float(np.std(wheel_beta_deg[motion_label == label])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "body_lat_mean_mps": [float(np.mean(v_b[motion_label == label, 1])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "lever_lat_mean_mps": [float(np.mean(v_rot[motion_label == label, 1])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "truth_lat_mean_mps": [float(np.mean(truth_lat_total[motion_label == label])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "basin_lat_mean_mps": [float(np.mean(basin_lat_total[motion_label == label])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "speed_mean_mps": [float(np.mean(speed_m_s[motion_label == label])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
            "yaw_rate_mean_deg_s": [float(np.mean(yaw_rate_deg_s[motion_label == label])) for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]],
        }
    )
    motion_ref.to_csv(out_dir / "motion_reference_breakdown.csv", index=False)

    candidate_overall_df = summary_df[summary_df["scope"] == "overall"].copy()
    candidate_focus_df = summary_df[summary_df["scope"].isin(["straight", "straight_fast", "turn_pos", "turn_neg"])].copy()
    motion_table_df = motion_ref.copy()

    md_lines: list[str] = [
        "# NHC lateral source analysis",
        "",
        f"Date: `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{args.exp_id}`",
        f"- config: `{rel(config_path)}`",
        f"- POS: `{rel(pos_path)}`",
        f"- IMU: `{rel(imu_path)}`",
        f"- output dir: `{rel(out_dir)}`",
        f"- overlap window: `[{start_t:.3f}, {end_t:.3f}] s`, `n={t.size}`",
        f"- mounting reference from config: `roll={mounting_ref.total_roll_deg:.3f} deg`, `pitch={mounting_ref.total_pitch_deg:.3f} deg`, `config_total_yaw={mounting_ref.config_total_yaw_deg:.3f} deg`",
        f"- lever arm: `{mounting_ref.lever_arm_m.tolist()}` m",
        "",
        "## Key findings",
        "",
        f"- Under truth total yaw `{truth_yaw_deg:.6f} deg`, overall `lat_mean={truth_overall['lat_mean_mps']:.6f} m/s`; `straight` alone contributes `{straight_share * 100.0:.2f}%` of the total signed lateral sum, so the bias is not turn-only.",
        f"- On `straight_fast` (`speed >= {STRAIGHT_FAST_MIN_SPEED_M_S:.1f} m/s`, `|yaw_rate| <= {STRAIGHT_YAW_RATE_DEG_S:.1f} deg/s`), `heading-course = {reference_summary['straight_heading_minus_course_mean_deg']:.6f} ± {reference_summary['straight_heading_minus_course_std_deg']:.6f} deg`, and wheel-point direction in body frame is `beta_wheel = {reference_summary['straight_wheel_beta_mean_deg']:.6f} ± {reference_summary['straight_wheel_beta_std_deg']:.6f} deg`.",
        f"- That straight-line wheel direction is essentially the same as the NHC basin yaw `{basin_yaw_deg:.6f} deg`, and differs from truth yaw by `{reference_summary['yaw_gap_deg']:.6f} deg`; truth-case linear fit gives equivalent angle `{reference_summary['truth_equivalent_angle_deg']:.6f} deg` with `R^2={reference_summary['truth_regression_r2']:.6f}`.",
        f"- The lever-arm rotation term is negligible on straight segments: `body_lat_mean={reference_summary['straight_body_lat_mean_mps']:.6f} m/s` vs `lever_lat_mean={reference_summary['straight_lever_lat_mean_mps']:.6f} m/s`. The dominant bias source is the translational `v_b.y`, not `(omega x lever).y`.",
        f"- Turning still shows secondary asymmetric residual after applying basin yaw: `turn_pos={reference_summary['basin_turn_pos_lat_mean_mps']:.6f} m/s`, `turn_neg={reference_summary['basin_turn_neg_lat_mean_mps']:.6f} m/s`. So side-slip-like behavior exists, but it is a second-order effect on top of the straight-line reference offset.",
        "",
        "## Candidate Overall Summary",
        "",
    ]
    md_lines.extend(
        render_markdown_table(
            candidate_overall_df,
            ["yaw_deg", "lat_mean_mps", "lat_rmse_mps", "lat_trans_mean_mps", "lat_rot_mean_mps", "accept_ratio", "nis_mean"],
            {
                "yaw_deg": "yaw_deg",
                "lat_mean_mps": "lat_mean_mps",
                "lat_rmse_mps": "lat_rmse_mps",
                "lat_trans_mean_mps": "lat_trans_mean_mps",
                "lat_rot_mean_mps": "lat_rot_mean_mps",
                "accept_ratio": "accept_ratio",
                "nis_mean": "nis_mean",
            },
        )
    )
    md_lines.extend(
        [
            "",
            "## Candidate Motion Breakdown",
            "",
        ]
    )
    md_lines.extend(
        render_markdown_table(
            candidate_focus_df,
            ["scope", "yaw_deg", "count_ratio", "lat_mean_mps", "lat_rmse_mps", "lat_trans_mean_mps", "lat_rot_mean_mps"],
            {
                "scope": "scope",
                "yaw_deg": "yaw_deg",
                "count_ratio": "count_ratio",
                "lat_mean_mps": "lat_mean_mps",
                "lat_rmse_mps": "lat_rmse_mps",
                "lat_trans_mean_mps": "lat_trans_mean_mps",
                "lat_rot_mean_mps": "lat_rot_mean_mps",
            },
        )
    )
    md_lines.extend(
        [
            "",
            "## Motion Reference Breakdown",
            "",
        ]
    )
    md_lines.extend(
        render_markdown_table(
            motion_table_df,
            ["motion_label", "count_ratio", "wheel_beta_mean_deg", "wheel_beta_std_deg", "body_lat_mean_mps", "lever_lat_mean_mps", "truth_lat_mean_mps", "basin_lat_mean_mps"],
            {
                "motion_label": "motion_label",
                "count_ratio": "count_ratio",
                "wheel_beta_mean_deg": "wheel_beta_mean_deg",
                "wheel_beta_std_deg": "wheel_beta_std_deg",
                "body_lat_mean_mps": "body_lat_mean_mps",
                "lever_lat_mean_mps": "lever_lat_mean_mps",
                "truth_lat_mean_mps": "truth_lat_mean_mps",
                "basin_lat_mean_mps": "basin_lat_mean_mps",
            },
        )
    )
    md_lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- The current NHC basin is explained primarily by a nearly constant straight-line heading/reference offset already present in `POS + IMU`, not by lever-arm rotation or a residual code bug.",
            "- Because the straight-line wheel direction implied by data is about `0.84 deg`, the solver naturally drives total mounting yaw toward `0.83~0.88 deg`.",
            "- The external truth value `1.37 deg` is therefore inconsistent with the motion axis implied by current data. The remaining choice is between `POS heading reference / mounting truth definition mismatch` and a deliberately modeled constant slip offset.",
            "- Turning asymmetry is real, but it remains after the straight bias is removed; it should be treated as a secondary slip-aware modeling issue, not the primary source of the stable yaw basin.",
            "",
            "## Freshness",
            "",
            f"- config mtime: `{mtime_text(config_path)}`",
            f"- POS mtime: `{mtime_text(pos_path)}`",
            f"- IMU mtime: `{mtime_text(imu_path)}`",
            f"- generated artifacts: `{rel(out_dir / 'summary.md')}`, `{rel(out_dir / 'reference_summary.json')}`, `{rel(out_dir / 'candidate_motion_breakdown.csv')}`, `{rel(out_dir / 'motion_reference_breakdown.csv')}`, `{rel(out_dir / 'per_sample_lateral_profile.csv')}`",
        ]
    )
    (out_dir / "summary.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
