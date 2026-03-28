#!/usr/bin/env python3
"""
Decompose shared-chain nominal velocity mismatch into contributors:
  dv_g  = gravity contribution per step
  dv_c  = Coriolis contribution per step (ECEF: -2*wie×v; NED: -(2*wie+wen)×v)
  dv_sf = inferred specific-force contribution (dv_total - dv_g - dv_c)

For the current solver (ECEF frame), dv_c excludes the transport-rate wen term;
for KF-GINS (NED frame), dv_c includes it. The transport-rate difference is
~0.03 mm/s over 199 steps and appears as a tiny bias in delta_dv_sf.

Also compares attitude (roll/pitch/yaw) at the START of each window to test
whether a sustained attitude difference explains the specific-force mismatch.

Outputs
-------
  contributor_breakdown_w2.csv   -- per-step contributor columns for w2
  contributor_breakdown_w1.csv   -- per-step contributor columns for w1
  contributor_summary.csv        -- window-level cumulative sums and fractions
  attitude_at_window_start.csv   -- attitude comparison at w2 and w1 start
  summary.md
  manifest.json
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root

EXP_ID_DEFAULT = "EXP-20260328-data2-kf-gins-outage-contributor-breakdown-r1"
OUTPUT_DIR_DEFAULT = REPO_ROOT / "output" / "d2_kf_outage_contrib_r1"

CURRENT_SOL_DEFAULT = (
    REPO_ROOT / "output" / "d2_outage_fix_dbg_r4"
    / "SOL_data2_baseline_ins_gnss_outage_no_odo_nhc_gnss_lever_truth_fixed.txt"
)
KF_NAV_DEFAULT = (
    REPO_ROOT / "output" / "d2_kf_outage_dbg_r2"
    / "runtime_output" / "KF_GINS_Navresult.nav"
)
TRUTH_DEFAULT = REPO_ROOT / "dataset" / "data2_converted" / "POS_converted.txt"

WINDOW_SPECS: dict[str, tuple[int, int]] = {
    "w2": (528270, 528280),
    "w1": (528180, 528210),
}

# WGS84 constants
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
OMEGA_EARTH = 7.292115e-5


# ---------------------------------------------------------------------------
# Geodesy helpers
# ---------------------------------------------------------------------------

def ecef_to_llh(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Return (lat_rad, lon_rad, h_m) from ECEF (m)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        sl = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
        lat = math.atan2(z + WGS84_E2 * n * sl, p)
    sl = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    h = p / max(1e-12, math.cos(lat)) - n
    return lat, lon, h


def meridian_prime_vertical_radii(lat_rad: float) -> tuple[float, float]:
    """Return (R_M, R_N) in metres."""
    sin_lat = math.sin(lat_rad)
    denom = 1.0 - WGS84_E2 * sin_lat * sin_lat
    R_M = WGS84_A * (1.0 - WGS84_E2) / denom ** 1.5
    R_N = WGS84_A / math.sqrt(denom)
    return R_M, R_N


def rot_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    """R_{e->n}: transforms ECEF column-vector to NED column-vector."""
    sl, cl = math.sin(lat_rad), math.cos(lat_rad)
    so, co = math.sin(lon_rad), math.cos(lon_rad)
    return np.array(
        [[-sl * co, -sl * so, cl],
         [-so,       co,      0.0],
         [-cl * co, -cl * so, -sl]],
        dtype=float,
    )


def gravity_ned(lat_rad: float, h_m: float) -> np.ndarray:
    """
    Normal gravity in NED frame = [0, 0, g(lat,h)].

    Somigliana + simple height correction — matches GravityEcef() in C++.
    """
    sin_lat = math.sin(lat_rad)
    g0 = 9.7803253359 * (1.0 + 0.00193185265241 * sin_lat ** 2) / math.sqrt(
        1.0 - WGS84_E2 * sin_lat ** 2
    )
    g = g0 - 3.086e-6 * h_m
    return np.array([0.0, 0.0, g], dtype=float)


def wie_ned(lat_rad: float) -> np.ndarray:
    """Earth-rotation vector in NED frame."""
    return np.array(
        [OMEGA_EARTH * math.cos(lat_rad), 0.0, -OMEGA_EARTH * math.sin(lat_rad)],
        dtype=float,
    )


def wen_ned(vn: float, ve: float, lat_rad: float, h_m: float, R_M: float, R_N: float) -> np.ndarray:
    """Transport-rate vector in NED frame."""
    return np.array(
        [ve / (R_N + h_m),
         -vn / (R_M + h_m),
         -ve * math.tan(lat_rad) / (R_N + h_m)],
        dtype=float,
    )


def coriolis_current_ned(lat_rad: float, v_ned: np.ndarray) -> np.ndarray:
    """Coriolis for current solver (ECEF → NED): -2 * wie_n × v_ned."""
    return -2.0 * np.cross(wie_ned(lat_rad), v_ned)


def coriolis_kf_ned(lat_rad: float, h_m: float, v_ned: np.ndarray) -> np.ndarray:
    """Coriolis for KF-GINS (NED): -(2*wie_n + wen_n) × v_ned."""
    R_M, R_N = meridian_prime_vertical_radii(lat_rad)
    wie = wie_ned(lat_rad)
    wen = wen_ned(v_ned[0], v_ned[1], lat_rad, h_m, R_M, R_N)
    return -np.cross(2.0 * wie + wen, v_ned)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_truth(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+", header=None).iloc[:, :10].copy()
    frame.columns = ["timestamp", "lat_deg", "lon_deg", "h_m",
                     "vn_mps", "ve_mps", "vd_mps",
                     "roll_deg", "pitch_deg", "yaw_deg"]
    return frame


def load_current_sol(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+")
    # SOL column order from evaluation.cpp:
    # timestamp fused_x fused_y fused_z fused_vx fused_vy fused_vz
    # fused_roll fused_pitch fused_yaw  mounting_pitch mounting_yaw odo_scale
    # sg_x sg_y sg_z  sa_x sa_y sa_z
    # ba_x ba_y ba_z  bg_x bg_y bg_z
    # lever_x lever_y lever_z  gnss_lever_x gnss_lever_y gnss_lever_z
    expected = [
        "timestamp", "fused_x", "fused_y", "fused_z",
        "fused_vx", "fused_vy", "fused_vz",
        "fused_roll", "fused_pitch", "fused_yaw",
        "mounting_pitch", "mounting_yaw", "odo_scale",
        "sg_x", "sg_y", "sg_z", "sa_x", "sa_y", "sa_z",
        "ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z",
        "lever_x", "lever_y", "lever_z",
        "gnss_lever_x", "gnss_lever_y", "gnss_lever_z",
    ]
    if len(frame.columns) == len(expected):
        frame.columns = expected
    return frame


def load_kf_nav(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path, sep=r"\s+", header=None).iloc[:, :11].copy()
    frame.columns = [
        "week", "timestamp",
        "lat_deg", "lon_deg", "h_m",
        "vn_mps", "ve_mps", "vd_mps",
        "roll_deg", "pitch_deg", "yaw_deg",
    ]
    return frame


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def interp_truth(truth: pd.DataFrame, t: float, col: str) -> float:
    return float(np.interp([t], truth["timestamp"].to_numpy(float), truth[col].to_numpy(float))[0])


def along_cross_basis(truth: pd.DataFrame, start_t: float, end_t: float) -> tuple[np.ndarray, np.ndarray]:
    mid_t = 0.5 * (start_t + end_t)
    along = np.array([
        interp_truth(truth, mid_t, "vn_mps"),
        interp_truth(truth, mid_t, "ve_mps"),
    ], dtype=float)
    norm = float(np.linalg.norm(along))
    if norm < 1e-6:
        raise ValueError(f"near-zero truth velocity at t={mid_t}")
    along /= norm
    cross = np.array([-along[1], along[0]], dtype=float)
    return along, cross


def decompose(v_ned: np.ndarray, along_2d: np.ndarray, cross_2d: np.ndarray) -> dict[str, float]:
    return {
        "along": float(v_ned[0] * along_2d[0] + v_ned[1] * along_2d[1]),
        "cross": float(v_ned[0] * cross_2d[0] + v_ned[1] * cross_2d[1]),
        "down":  float(v_ned[2]),
        "norm":  float(np.linalg.norm(v_ned)),
    }


# ---------------------------------------------------------------------------
# Per-step contributor computation
# ---------------------------------------------------------------------------

def contributors_current(
    p_ecef_prev: np.ndarray,
    v_ecef_prev: np.ndarray,
    v_ecef_curr: np.ndarray,
    dt: float,
) -> dict[str, np.ndarray]:
    """
    Decompose dv into gravity / Coriolis / specific-force for the current
    solver (ECEF state).  All outputs are in NED at the previous position.
    """
    lat, lon, h = ecef_to_llh(*p_ecef_prev)
    R_en = rot_ecef_to_ned(lat, lon)

    dv_ecef = v_ecef_curr - v_ecef_prev
    dv_ned  = R_en @ dv_ecef

    v_ned_prev = R_en @ v_ecef_prev
    dv_g_ned = gravity_ned(lat, h) * dt
    dv_c_ned = coriolis_current_ned(lat, v_ned_prev) * dt
    dv_sf_ned = dv_ned - dv_g_ned - dv_c_ned

    return {"dv_total": dv_ned, "dv_g": dv_g_ned, "dv_c": dv_c_ned, "dv_sf": dv_sf_ned}


def contributors_kf(
    lat_prev_deg: float,
    lon_prev_deg: float,
    h_prev_m: float,
    v_ned_prev: np.ndarray,
    v_ned_curr: np.ndarray,
    dt: float,
) -> dict[str, np.ndarray]:
    """
    Decompose dv for KF-GINS (NED state).
    Note: dv_c includes the transport-rate term wen_n.
    """
    lat = math.radians(lat_prev_deg)
    h   = h_prev_m

    dv_ned = v_ned_curr - v_ned_prev
    dv_g_ned = gravity_ned(lat, h) * dt
    dv_c_ned = coriolis_kf_ned(lat, h, v_ned_prev) * dt
    dv_sf_ned = dv_ned - dv_g_ned - dv_c_ned

    return {"dv_total": dv_ned, "dv_g": dv_g_ned, "dv_c": dv_c_ned, "dv_sf": dv_sf_ned}


# ---------------------------------------------------------------------------
# Window analysis
# ---------------------------------------------------------------------------

def analyse_window(
    window_name: str,
    t_start: int,
    t_end: int,
    current_sol: pd.DataFrame,
    kf_nav: pd.DataFrame,
    truth: pd.DataFrame,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    For all shared IMU steps in the window, compute contributor breakdown.
    Returns (per_step_df, window_summary_df).
    """
    # Slice window (slightly wider to get boundary rows for differences)
    tol = 0.5
    cur_win = current_sol[
        (current_sol["timestamp"] >= t_start - tol) &
        (current_sol["timestamp"] <= t_end + tol)
    ].copy().reset_index(drop=True)
    kf_win = kf_nav[
        (kf_nav["timestamp"] >= t_start - tol) &
        (kf_nav["timestamp"] <= t_end + tol)
    ].copy().reset_index(drop=True)

    # Along/cross basis at window midpoint
    along_2d, cross_2d = along_cross_basis(truth, t_start, t_end)

    # Match shared rows by timestamp (tolerance 0.5 ms)
    MATCH_TOL = 5e-4
    rows: list[dict[str, Any]] = []

    cur_ts = cur_win["timestamp"].to_numpy(float)
    kf_ts  = kf_win["timestamp"].to_numpy(float)

    cur_idx = 0
    kf_idx  = 0

    while cur_idx + 1 < len(cur_win) and kf_idx + 1 < len(kf_win):
        tc_prev = cur_ts[cur_idx]
        tc_curr = cur_ts[cur_idx + 1]
        tk_prev = kf_ts[kf_idx]
        tk_curr = kf_ts[kf_idx + 1]

        # Only process steps whose END timestamp both solvers share
        if abs(tc_curr - tk_curr) > MATCH_TOL:
            # advance the lagging solver
            if tc_curr < tk_curr:
                cur_idx += 1
            else:
                kf_idx += 1
            continue

        # Require step to be strictly inside the window
        t_mid = 0.5 * (tc_curr + tk_curr)
        if t_mid < t_start or t_mid > t_end:
            cur_idx += 1
            kf_idx  += 1
            continue

        dt_step = tc_curr - tc_prev

        # ---- current solver contributors ----
        p_ecef_prev = cur_win.loc[cur_idx, ["fused_x", "fused_y", "fused_z"]].to_numpy(float)
        v_ecef_prev = cur_win.loc[cur_idx, ["fused_vx", "fused_vy", "fused_vz"]].to_numpy(float)
        v_ecef_curr = cur_win.loc[cur_idx + 1, ["fused_vx", "fused_vy", "fused_vz"]].to_numpy(float)
        c_cur = contributors_current(p_ecef_prev, v_ecef_prev, v_ecef_curr, dt_step)

        # ---- KF-GINS contributors ----
        lat_prev  = float(kf_win.loc[kf_idx, "lat_deg"])
        lon_prev  = float(kf_win.loc[kf_idx, "lon_deg"])
        h_prev    = float(kf_win.loc[kf_idx, "h_m"])
        vn_prev   = float(kf_win.loc[kf_idx, "vn_mps"])
        ve_prev   = float(kf_win.loc[kf_idx, "ve_mps"])
        vd_prev   = float(kf_win.loc[kf_idx, "vd_mps"])
        vn_curr   = float(kf_win.loc[kf_idx + 1, "vn_mps"])
        ve_curr   = float(kf_win.loc[kf_idx + 1, "ve_mps"])
        vd_curr   = float(kf_win.loc[kf_idx + 1, "vd_mps"])

        v_ned_prev = np.array([vn_prev, ve_prev, vd_prev], dtype=float)
        v_ned_curr = np.array([vn_curr, ve_curr, vd_curr], dtype=float)
        c_kf = contributors_kf(lat_prev, lon_prev, h_prev, v_ned_prev, v_ned_curr, dt_step)

        # ---- delta contributors ----
        delta: dict[str, np.ndarray] = {}
        for key in ("dv_total", "dv_g", "dv_c", "dv_sf"):
            delta[key] = c_cur[key] - c_kf[key]

        def flat(prefix: str, v: np.ndarray) -> dict[str, float]:
            d = decompose(v, along_2d, cross_2d)
            return {f"{prefix}_{k}": v for k, v in d.items()}

        row: dict[str, Any] = {
            "window": window_name,
            "t_curr": float(tc_curr),
            "dt":     float(dt_step),
        }
        for key in ("dv_total", "dv_g", "dv_c", "dv_sf"):
            row.update(flat(f"cur_{key}", c_cur[key]))
            row.update(flat(f"kf_{key}",  c_kf[key]))
            row.update(flat(f"delta_{key}", delta[key]))

        rows.append(row)
        cur_idx += 1
        kf_idx  += 1

    step_df = pd.DataFrame(rows)
    if step_df.empty:
        return step_df, pd.DataFrame()

    # ---- window-level cumulative sums ----
    cum_rows: list[dict[str, Any]] = []
    for key in ("dv_total", "dv_g", "dv_c", "dv_sf"):
        for pfx in ("cur", "kf", "delta"):
            col_base = f"{pfx}_{key}"
            cum = np.zeros(3, dtype=float)
            for _, r in step_df.iterrows():
                cum[0] += r[f"{col_base}_along"]
                cum[1] += r[f"{col_base}_cross"]
                cum[2] += r[f"{col_base}_down"]
            cum_rows.append({
                "window": window_name,
                "solver": pfx,
                "contributor": key,
                "cum_along": float(cum[0]),
                "cum_cross": float(cum[1]),
                "cum_down":  float(cum[2]),
                "cum_norm":  float(np.linalg.norm(cum)),
                "n_steps": len(step_df),
            })

    summary_df = pd.DataFrame(cum_rows)
    return step_df, summary_df


# ---------------------------------------------------------------------------
# Attitude comparison at window start
# ---------------------------------------------------------------------------

def attitude_at_window_start(
    window_name: str,
    t_start: int,
    current_sol: pd.DataFrame,
    kf_nav: pd.DataFrame,
) -> dict[str, Any]:
    """Extract roll/pitch/yaw for both solvers at the first row >= t_start."""
    tol = 0.1
    cur_row = current_sol[current_sol["timestamp"] >= t_start - tol].iloc[0]
    kf_row  = kf_nav[kf_nav["timestamp"] >= t_start - tol].iloc[0]

    def _att(row: pd.Series, prefix: str) -> dict[str, float]:
        return {
            f"{prefix}_t":     float(row["timestamp"]),
            f"{prefix}_roll":  float(row["roll_deg"]),
            f"{prefix}_pitch": float(row["pitch_deg"]),
            f"{prefix}_yaw":   float(row["yaw_deg"]),
        }

    cur_att = _att(cur_row, "cur")
    kf_att  = _att(kf_row,  "kf")

    d_roll  = cur_att["cur_roll"]  - kf_att["kf_roll"]
    d_pitch = cur_att["cur_pitch"] - kf_att["kf_pitch"]
    d_yaw   = cur_att["cur_yaw"]   - kf_att["kf_yaw"]

    # Wrap to [-180, 180]
    def wrap(x: float) -> float:
        return (x + 180.0) % 360.0 - 180.0

    row_out = {"window": window_name}
    row_out.update(cur_att)
    row_out.update(kf_att)
    row_out["d_roll_deg"]  = wrap(d_roll)
    row_out["d_pitch_deg"] = wrap(d_pitch)
    row_out["d_yaw_deg"]   = wrap(d_yaw)
    row_out["d_att_norm_deg"] = math.sqrt(
        wrap(d_roll) ** 2 + wrap(d_pitch) ** 2 + wrap(d_yaw) ** 2
    )
    # Expected dv_sf per step from attitude mismatch:
    # |d_att × f^n_step| ≈ |d_att| * g * dt  (where dt~0.01s, g~9.8)
    d_att_rad = math.radians(row_out["d_att_norm_deg"])
    typical_specific_force_step = 9.8 * 0.01  # [m/s] per step
    row_out["expected_dv_sf_per_step_m_s"] = d_att_rad * typical_specific_force_step
    row_out["expected_cum_dv_sf_199steps_m_s"] = row_out["expected_dv_sf_per_step_m_s"] * 199

    # Also read bias if available in SOL
    for col in ("ba_x", "ba_y", "ba_z", "bg_x", "bg_y", "bg_z"):
        if col in cur_row.index:
            row_out[f"cur_{col}"] = float(cur_row[col])

    return row_out


# ---------------------------------------------------------------------------
# Summary text
# ---------------------------------------------------------------------------

def build_summary(
    exp_id: str,
    output_dir: Path,
    step_dfs: dict[str, pd.DataFrame],
    win_summaries: dict[str, pd.DataFrame],
    att_rows: list[dict[str, Any]],
    inputs: dict[str, Path],
) -> list[str]:
    lines = [
        "# GNSS outage nominal-dv contributor breakdown",
        "",
        f"- exp_id: `{exp_id}`",
        "- goal: decompose shared-chain dv mismatch into gravity / Coriolis / specific-force",
        "- key question: does dv_sf dominate delta_dv_total? (confirms attitude-driven mismatch)",
        "",
        "## Inputs",
    ]
    for k, v in inputs.items():
        lines.append(f"  - {k}: `{rel_from_root(v, REPO_ROOT)}`")
    lines.append("")

    lines.append("## Attitude difference at window start")
    for arow in att_rows:
        win = arow["window"]
        lines += [
            f"### {win}",
            f"  - cur t={arow['cur_t']:.6f}  roll/pitch/yaw = {arow['cur_roll']:.4f}/{arow['cur_pitch']:.4f}/{arow['cur_yaw']:.4f} deg",
            f"  - kf  t={arow['kf_t']:.6f}  roll/pitch/yaw = {arow['kf_roll']:.4f}/{arow['kf_pitch']:.4f}/{arow['kf_yaw']:.4f} deg",
            f"  - delta roll/pitch/yaw = {arow['d_roll_deg']:.4f}/{arow['d_pitch_deg']:.4f}/{arow['d_yaw_deg']:.4f} deg",
            f"  - delta att norm = {arow['d_att_norm_deg']:.4f} deg = {math.radians(arow['d_att_norm_deg']) * 1000:.4f} mrad",
            f"  - expected dv_sf per step (simple estimate) = {arow['expected_dv_sf_per_step_m_s'] * 1e6:.1f} µm/s",
            f"  - expected cum dv_sf over 199 steps = {arow['expected_cum_dv_sf_199steps_m_s'] * 1000:.3f} mm/s",
        ]
        if "cur_ba_x" in arow:
            lines.append(
                f"  - cur ba = ({arow['cur_ba_x']:.6f}, {arow['cur_ba_y']:.6f}, {arow['cur_ba_z']:.6f}) m/s²"
                f"   bg = ({arow['cur_bg_x']:.6f}, {arow['cur_bg_y']:.6f}, {arow['cur_bg_z']:.6f}) rad/s"
            )
        lines.append("")

    lines.append("## Cumulative contributor breakdown")
    for win_name, ws in win_summaries.items():
        lines.append(f"### {win_name}")
        delta_ws = ws[ws["solver"] == "delta"]
        total_norm = float(delta_ws.loc[delta_ws["contributor"] == "dv_total", "cum_norm"].iloc[0]) or 1e-12
        for _, r in delta_ws.iterrows():
            frac = float(r["cum_norm"]) / total_norm
            lines.append(
                f"  - delta {r['contributor']:10s}: along={r['cum_along']:+.6f}  "
                f"cross={r['cum_cross']:+.6f}  down={r['cum_down']:+.6f}  "
                f"norm={r['cum_norm']:.6f}  fraction_of_total={frac:.4f}"
            )
        lines.append(f"  (n_steps = {int(delta_ws['n_steps'].iloc[0])})")
        lines.append("")

    lines += [
        "## Interpretation",
        "",
        "If dv_sf dominates (fraction > 0.9) and dv_g + dv_c fractions are < 0.1:",
        "  → attitude-driven specific-force projection is the dominant per-step error source.",
        "  → The attitude difference at the window start (see above) should predict the",
        "    observed dv_sf magnitude: |d_att_rad| * g * dt * 199 ≈ cum_delta_dv_sf_norm.",
        "",
        "If the predicted magnitude matches the observed magnitude and the direction",
        "(along/cross pattern) is consistent across w1 and w2, this confirms:",
        "  → HYP-61 + HYP-62 share the same root cause: GNSS update timing contract",
        "     (exact-time vs near-prev-IMU) accumulates a persistent attitude offset,",
        "     which then manifests as systematic specific-force projection error during",
        "     the shared predict chain — identical IMU data, different attitude basis.",
    ]
    return lines


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Outage dv contributor breakdown")
    parser.add_argument("--exp-id",      default=EXP_ID_DEFAULT)
    parser.add_argument("--output-dir",  default=str(OUTPUT_DIR_DEFAULT), type=Path)
    parser.add_argument("--sol",         default=str(CURRENT_SOL_DEFAULT), type=Path)
    parser.add_argument("--kf-nav",      default=str(KF_NAV_DEFAULT),     type=Path)
    parser.add_argument("--truth",       default=str(TRUTH_DEFAULT),      type=Path)
    args = parser.parse_args()

    output_dir: Path = args.output_dir
    ensure_dir(output_dir)

    inputs = {
        "current_sol": args.sol,
        "kf_nav":      args.kf_nav,
        "truth":       args.truth,
    }
    for k, v in inputs.items():
        if not v.exists():
            print(f"ERROR: input not found: {v}", file=sys.stderr)
            return 1

    print(f"Loading current SOL: {args.sol}")
    current_sol = load_current_sol(args.sol)
    print(f"Loading KF-GINS nav: {args.kf_nav}")
    kf_nav = load_kf_nav(args.kf_nav)
    print(f"Loading truth: {args.truth}")
    truth = load_truth(args.truth)

    # Convert current ECEF velocity to NED (only used for attitude-at-start extraction)
    # The per-step computation works in ECEF directly.

    step_dfs: dict[str, pd.DataFrame] = {}
    win_summaries: dict[str, pd.DataFrame] = {}
    att_rows: list[dict[str, Any]] = []

    for win_name, (t_start, t_end) in WINDOW_SPECS.items():
        print(f"Processing window {win_name} [{t_start}, {t_end}] ...")
        step_df, summary_df = analyse_window(
            win_name, t_start, t_end, current_sol, kf_nav, truth
        )
        step_dfs[win_name]    = step_df
        win_summaries[win_name] = summary_df

        att_row = attitude_at_window_start(win_name, t_start, current_sol, kf_nav)
        att_rows.append(att_row)

        if not step_df.empty:
            step_path = output_dir / f"contributor_breakdown_{win_name}.csv"
            step_df.to_csv(step_path, index=False)
            print(f"  Wrote {step_path.name} ({len(step_df)} rows)")

        if not summary_df.empty:
            print(f"  {win_name} step count: {summary_df['n_steps'].iloc[0]}")
            delta_total = summary_df[
                (summary_df["solver"] == "delta") & (summary_df["contributor"] == "dv_total")
            ]
            delta_sf = summary_df[
                (summary_df["solver"] == "delta") & (summary_df["contributor"] == "dv_sf")
            ]
            if not delta_total.empty and not delta_sf.empty:
                total_norm = float(delta_total["cum_norm"].iloc[0])
                sf_norm    = float(delta_sf["cum_norm"].iloc[0])
                print(f"  delta_dv_total norm: {total_norm:.6f} m/s")
                print(f"  delta_dv_sf    norm: {sf_norm:.6f} m/s  fraction: {sf_norm / max(total_norm, 1e-12):.4f}")

    # Save combined summary
    all_summaries = pd.concat(list(win_summaries.values()), ignore_index=True) if win_summaries else pd.DataFrame()
    if not all_summaries.empty:
        all_summaries.to_csv(output_dir / "contributor_summary.csv", index=False)

    att_df = pd.DataFrame(att_rows)
    att_df.to_csv(output_dir / "attitude_at_window_start.csv", index=False)

    summary_lines = build_summary(args.exp_id, output_dir, step_dfs, win_summaries, att_rows, inputs)
    summary_path = output_dir / "summary.md"
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    print(f"Wrote summary: {summary_path}")

    manifest = {
        "exp_id":    args.exp_id,
        "timestamp": dt.datetime.now().isoformat(),
        "inputs":    {k: str(v) for k, v in inputs.items()},
        "outputs": {
            "contributor_breakdown_w2": str(output_dir / "contributor_breakdown_w2.csv"),
            "contributor_breakdown_w1": str(output_dir / "contributor_breakdown_w1.csv"),
            "contributor_summary":      str(output_dir / "contributor_summary.csv"),
            "attitude_at_window_start": str(output_dir / "attitude_at_window_start.csv"),
            "summary":                  str(summary_path),
        },
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
