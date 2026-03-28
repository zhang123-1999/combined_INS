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

from scripts.analysis.analyze_turn_vs_straight_xy_decoupling import assign_window_id  # noqa: E402
from scripts.analysis.analyze_turn_window_xy_term_source import load_case_ids, load_case_meta  # noqa: E402
from scripts.analysis.odo_nhc_update_sweep import ensure_dir, rel_from_root  # noqa: E402
from scripts.analysis.run_data2_state_sanity_matrix import json_safe  # noqa: E402
from scripts.analysis.run_data2_turn_window_shared_correction_probe import (  # noqa: E402
    POS_PATH_DEFAULT,
    load_effective_gnss_pos_update_df,
    load_pos_dataframe,
    select_turn_windows,
)


EXP_ID_DEFAULT = "EXP-20260319-data2-turn-direction-proxy-r1"
SOURCE_PROBE_DEFAULT = Path("output/data2_turn_window_lgx_from_y_gain_scale_probe")
OUTPUT_DIR_DEFAULT = Path("output/data2_turn_direction_proxy")
PROXY_RATE_EPS = 1.0e-9
CURVATURE_EPS = 1.0e-9

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare offline POS yaw-rate turn direction against runtime-available proxies "
            "at GNSS update times."
        )
    )
    parser.add_argument("--source-probe", type=Path, default=SOURCE_PROBE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--pos-path", type=Path, default=POS_PATH_DEFAULT)
    return parser


def nonzero_sign(value: float, eps: float) -> float:
    if not np.isfinite(value) or abs(value) <= eps:
        return math.nan
    return 1.0 if value > 0.0 else -1.0


def ecef_to_lla(
    x: np.ndarray, y: np.ndarray, z: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lon = np.arctan2(y, x)
    p = np.sqrt(x * x + y * y)
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    for _ in range(5):
        sin_lat = np.sin(lat)
        n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = np.arctan2(z + WGS84_E2 * n * sin_lat, p)
    sin_lat = np.sin(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / np.cos(lat) - n
    return np.rad2deg(lat), np.rad2deg(lon), alt


def ecef_vel_to_ned(
    vx: np.ndarray, vy: np.ndarray, vz: np.ndarray, lat_deg: np.ndarray, lon_deg: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    vn = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz
    ve = -sin_lon * vx + cos_lon * vy
    vd = -cos_lat * cos_lon * vx - cos_lat * sin_lon * vy - sin_lat * vz
    return vn, ve, vd


def load_turn_windows(source_probe: Path, pos_path: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    turn_windows_path = source_probe / "turn_windows.csv"
    if turn_windows_path.exists():
        windows_df = pd.read_csv(turn_windows_path)
        pos_df = load_pos_dataframe(pos_path)
        _, series_df = select_turn_windows(
            pos_df=pos_df,
            rolling_window_s=5.0,
            top_k=max(5, len(windows_df)),
            min_speed_m_s=3.0,
            min_separation_s=30.0,
        )
        return windows_df, series_df
    pos_df = load_pos_dataframe(pos_path)
    return select_turn_windows(
        pos_df=pos_df,
        rolling_window_s=5.0,
        top_k=5,
        min_speed_m_s=3.0,
        min_separation_s=30.0,
    )


def load_diag_dataframe(path: Path, reference_abs_t: float) -> pd.DataFrame:
    diag_df = pd.read_csv(path, sep=r"\s+", engine="python")
    if diag_df.empty or "t" not in diag_df.columns:
        raise RuntimeError(f"invalid DIAG log: {path}")
    diag_t0 = reference_abs_t - float(diag_df["t"].iloc[0])
    out = diag_df.copy()
    out["abs_t"] = out["t"].to_numpy(dtype=float) + diag_t0
    return out


def load_sol_proxy_df(path: Path) -> pd.DataFrame:
    sol_df = pd.read_csv(path, sep=r"\s+", engine="python")
    if sol_df.empty or "timestamp" not in sol_df.columns:
        raise RuntimeError(f"invalid SOL log: {path}")
    out = sol_df.copy()
    t = out["timestamp"].to_numpy(dtype=float)
    yaw_deg = out["fused_yaw"].to_numpy(dtype=float)
    yaw_rate_deg_s = np.rad2deg(np.gradient(np.unwrap(np.deg2rad(yaw_deg)), t))
    lat_deg, lon_deg, _ = ecef_to_lla(
        out["fused_x"].to_numpy(dtype=float),
        out["fused_y"].to_numpy(dtype=float),
        out["fused_z"].to_numpy(dtype=float),
    )
    vn, ve, _ = ecef_vel_to_ned(
        out["fused_vx"].to_numpy(dtype=float),
        out["fused_vy"].to_numpy(dtype=float),
        out["fused_vz"].to_numpy(dtype=float),
        lat_deg,
        lon_deg,
    )
    an = np.gradient(vn, t)
    ae = np.gradient(ve, t)
    curvature_cross = vn * ae - ve * an
    out["sol_yaw_rate_deg_s"] = yaw_rate_deg_s
    out["curvature_cross_ne"] = curvature_cross
    return out


def interp_to_queries(series_t: np.ndarray, series_v: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    if len(series_t) == 0:
        return np.full(len(query_t), math.nan, dtype=float)
    return np.interp(query_t, series_t, series_v)


def build_per_update_df(source_probe: Path, pos_path: Path) -> pd.DataFrame:
    turn_windows_df, turn_series_df = load_turn_windows(source_probe, pos_path)
    pos_interp_t = turn_series_df["t"].to_numpy(dtype=float)
    pos_yaw_rate = turn_series_df["yaw_rate_deg_s"].to_numpy(dtype=float)

    case_meta_map = load_case_meta(source_probe)
    case_ids = load_case_ids(source_probe)
    rows: list[dict[str, Any]] = []

    for case_id in case_ids:
        case_meta = case_meta_map.get(case_id, {})
        case_dir = source_probe / "artifacts" / "cases" / case_id
        gnss_path = case_dir / f"gnss_updates_{case_id}.csv"
        diag_path = case_dir / f"DIAG_{case_id}.txt"
        sol_path = case_dir / f"SOL_{case_id}.txt"
        if not gnss_path.exists() or not diag_path.exists() or not sol_path.exists():
            raise RuntimeError(f"missing required case artifact under {case_dir}")

        raw_df = load_effective_gnss_pos_update_df(gnss_path)
        raw_df = raw_df.loc[raw_df["tag"] == "GNSS_POS"].copy().sort_values(by="gnss_t").reset_index(drop=True)
        raw_df["turn_window_id"] = assign_window_id(raw_df["gnss_t"], turn_windows_df, "window_id")
        raw_df = raw_df.loc[raw_df["turn_window_id"].notna()].copy().reset_index(drop=True)
        if raw_df.empty:
            continue

        sol_df = load_sol_proxy_df(sol_path)
        diag_df = load_diag_dataframe(diag_path, reference_abs_t=float(sol_df["timestamp"].iloc[0]))
        query_t = raw_df["state_t"].to_numpy(dtype=float)

        pos_rate_interp = interp_to_queries(pos_interp_t, pos_yaw_rate, raw_df["gnss_t"].to_numpy(dtype=float))
        imu_omega_z = interp_to_queries(
            diag_df["abs_t"].to_numpy(dtype=float),
            diag_df["omega_z"].to_numpy(dtype=float),
            query_t,
        )
        sol_yaw_rate_interp = interp_to_queries(
            sol_df["timestamp"].to_numpy(dtype=float),
            sol_df["sol_yaw_rate_deg_s"].to_numpy(dtype=float),
            query_t,
        )
        curvature_interp = interp_to_queries(
            sol_df["timestamp"].to_numpy(dtype=float),
            sol_df["curvature_cross_ne"].to_numpy(dtype=float),
            query_t,
        )

        for idx, row in raw_df.iterrows():
            rows.append(
                {
                    "case_id": case_id,
                    "label": str(case_meta.get("label", case_id)),
                    "r_scale": float(case_meta.get("r_scale", math.nan)),
                    "turn_window_id": str(row["turn_window_id"]),
                    "gnss_t": float(row["gnss_t"]),
                    "state_t": float(row["state_t"]),
                    "pos_yaw_rate_deg_s": float(pos_rate_interp[idx]),
                    "pos_turn_sign": nonzero_sign(float(pos_rate_interp[idx]), PROXY_RATE_EPS),
                    "imu_omega_z_rad_s": float(imu_omega_z[idx]),
                    "imu_omega_z_deg_s": float(np.rad2deg(imu_omega_z[idx])),
                    "imu_turn_sign": nonzero_sign(float(imu_omega_z[idx]), PROXY_RATE_EPS),
                    "sol_yaw_rate_deg_s": float(sol_yaw_rate_interp[idx]),
                    "sol_yaw_turn_sign": nonzero_sign(float(sol_yaw_rate_interp[idx]), PROXY_RATE_EPS),
                    "sol_curvature_cross_ne": float(curvature_interp[idx]),
                    "sol_curvature_turn_sign": nonzero_sign(float(curvature_interp[idx]), CURVATURE_EPS),
                }
            )

    return pd.DataFrame(rows).sort_values(by=["r_scale", "gnss_t"]).reset_index(drop=True)


def summarize_proxy(per_update_df: pd.DataFrame, proxy_col: str) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    group_cols = ["case_id", "label", "r_scale"]
    for key, group in per_update_df.groupby(group_cols, sort=False):
        case_id, label, r_scale = key
        pos_sign = group["pos_turn_sign"].to_numpy(dtype=float)
        proxy_sign = group[proxy_col].to_numpy(dtype=float)
        valid = np.isfinite(pos_sign) & np.isfinite(proxy_sign)
        if not np.any(valid):
            continue
        pos_valid = pos_sign[valid]
        proxy_valid = proxy_sign[valid]
        direct = float(np.mean(proxy_valid == pos_valid))
        inverted = float(np.mean(proxy_valid == -pos_valid))
        best_alignment = "same" if direct >= inverted else "flip"
        best_rate = max(direct, inverted)
        positive_turn_proxy_mean = float(np.nanmean(proxy_valid[pos_valid > 0.0])) if np.any(pos_valid > 0.0) else math.nan
        negative_turn_proxy_mean = float(np.nanmean(proxy_valid[pos_valid < 0.0])) if np.any(pos_valid < 0.0) else math.nan
        rows.append(
            {
                "case_id": str(case_id),
                "label": str(label),
                "r_scale": float(r_scale),
                "proxy": proxy_col,
                "updates": int(np.sum(valid)),
                "direct_match_rate": direct,
                "inverted_match_rate": inverted,
                "best_alignment": best_alignment,
                "best_match_rate": best_rate,
                "proxy_positive_rate": float(np.mean(proxy_valid > 0.0)),
                "proxy_negative_rate": float(np.mean(proxy_valid < 0.0)),
                "positive_turn_proxy_sign_mean": positive_turn_proxy_mean,
                "negative_turn_proxy_sign_mean": negative_turn_proxy_mean,
            }
        )
    return pd.DataFrame(rows).sort_values(by=["proxy", "r_scale"]).reset_index(drop=True)


def summarize_overall(proxy_summary_df: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for proxy, group in proxy_summary_df.groupby("proxy", sort=False):
        weights = group["updates"].to_numpy(dtype=float)
        best_rate = group["best_match_rate"].to_numpy(dtype=float)
        direct_rate = group["direct_match_rate"].to_numpy(dtype=float)
        inverted_rate = group["inverted_match_rate"].to_numpy(dtype=float)
        weight_sum = float(np.sum(weights))
        if weight_sum <= 0.0:
            continue
        rows.append(
            {
                "proxy": str(proxy),
                "cases": int(len(group)),
                "weighted_direct_match_rate": float(np.sum(direct_rate * weights) / weight_sum),
                "weighted_inverted_match_rate": float(np.sum(inverted_rate * weights) / weight_sum),
                "weighted_best_match_rate": float(np.sum(best_rate * weights) / weight_sum),
                "recommended_alignment": "same"
                if float(np.sum(direct_rate * weights)) >= float(np.sum(inverted_rate * weights))
                else "flip",
            }
        )
    return pd.DataFrame(rows).sort_values(by="weighted_best_match_rate", ascending=False).reset_index(drop=True)


def write_summary(
    output_path: Path,
    source_probe: Path,
    overall_df: pd.DataFrame,
    case_proxy_df: pd.DataFrame,
) -> None:
    lines: list[str] = []
    lines.append("# turn-direction proxy analysis")
    lines.append("")
    if overall_df.empty:
        lines.append("- No valid proxy comparisons were produced.")
        output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return

    best = overall_df.iloc[0]
    lines.append("## Core judgement")
    lines.append(
        f"- Best runtime proxy on `{source_probe.name}` turn-window GNSS updates is "
        f"`{best['proxy']}` with weighted best-match rate `{best['weighted_best_match_rate']:.3f}` "
        f"against offline `POS yaw_rate` sign."
    )
    lines.append(
        f"- Recommended POS-sign mapping for `{best['proxy']}` is `{best['recommended_alignment']}` "
        f"(that is, POS sign {'=' if best['recommended_alignment'] == 'same' else '=-'} proxy sign)."
    )
    lines.append("")
    lines.append("## Overall ranking")
    for row in overall_df.itertuples(index=False):
        lines.append(
            f"- `{row.proxy}`: weighted `direct={row.weighted_direct_match_rate:.3f}`, "
            f"`inverted={row.weighted_inverted_match_rate:.3f}`, "
            f"`best={row.weighted_best_match_rate:.3f}`, recommended alignment=`{row.recommended_alignment}`."
        )
    lines.append("")
    lines.append("## Case details")
    for row in case_proxy_df.itertuples(index=False):
        lines.append(
            f"- `{row.label}` / `{row.proxy}`: `best_match={row.best_match_rate:.3f}`, "
            f"`direct={row.direct_match_rate:.3f}`, `inverted={row.inverted_match_rate:.3f}`, "
            f"recommended alignment=`{row.best_alignment}`, updates=`{row.updates}`."
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = build_parser().parse_args()
    source_probe = (REPO_ROOT / args.source_probe).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()
    pos_path = (REPO_ROOT / args.pos_path).resolve()

    ensure_dir(output_dir)

    per_update_df = build_per_update_df(source_probe, pos_path)
    if per_update_df.empty:
        raise RuntimeError(f"no turn-window GNSS updates found under {source_probe}")

    proxy_columns = [
        "imu_turn_sign",
        "sol_yaw_turn_sign",
        "sol_curvature_turn_sign",
    ]
    case_proxy_frames = [summarize_proxy(per_update_df, proxy_col) for proxy_col in proxy_columns]
    case_proxy_df = pd.concat(case_proxy_frames, ignore_index=True)
    overall_df = summarize_overall(case_proxy_df)

    per_update_path = output_dir / "proxy_per_update.csv"
    case_proxy_path = output_dir / "proxy_case_summary.csv"
    overall_path = output_dir / "proxy_overall_summary.csv"
    summary_path = output_dir / "summary.md"
    manifest_path = output_dir / "manifest.json"

    per_update_df.to_csv(per_update_path, index=False)
    case_proxy_df.to_csv(case_proxy_path, index=False)
    overall_df.to_csv(overall_path, index=False)
    write_summary(summary_path, source_probe, overall_df, case_proxy_df)

    manifest = {
        "exp_id": EXP_ID_DEFAULT,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "source_probe": rel_from_root(source_probe, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "artifacts": {
            "proxy_per_update_csv": rel_from_root(per_update_path, REPO_ROOT),
            "proxy_case_summary_csv": rel_from_root(case_proxy_path, REPO_ROOT),
            "proxy_overall_summary_csv": rel_from_root(overall_path, REPO_ROOT),
            "summary_md": rel_from_root(summary_path, REPO_ROOT),
        },
        "overall_rows": json_safe(overall_df.to_dict(orient="records")),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
