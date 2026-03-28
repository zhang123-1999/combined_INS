from __future__ import annotations

import argparse
import copy
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

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, llh_deg_to_ecef, load_yaml, rel_from_root, save_yaml
from scripts.analysis.run_data2_state_sanity_matrix import json_safe, reset_directory, run_command


CASE_SPECS = {
    "imulog": {
        "label": "IMULog",
        "imu_path": Path("dataset/data2_converted/IMU_converted.txt"),
        "source_note": "default converted IMULog stream used by config_data2_baseline_eskf.yaml",
    },
    "hl": {
        "label": "HL",
        "imu_path": Path("output/data2_imu_source_audit/artifacts/converted/HL20181229023353_0_IMU_converted.txt"),
        "source_note": "HL raw IMU converted to solver text format by run_data2_imu_source_audit.py --emit-converted",
    },
}
HORIZONS_S = [1.0, 5.0, 10.0, 30.0, 60.0, 120.0, 300.0, 600.0, 1200.0]
OUTPUT_DIR_DEFAULT = Path("output/data2_pure_ins_imu_compare")
EXP_ID_DEFAULT = "EXP-20260320-data2-pure-ins-imu-compare-r1"


def rot_ned_to_ecef(lat_rad: float, lon_rad: float) -> np.ndarray:
    sl = math.sin(lat_rad)
    cl = math.cos(lat_rad)
    so = math.sin(lon_rad)
    co = math.cos(lon_rad)
    return np.array(
        [
            [-sl * co, -so, -cl * co],
            [-sl * so, co, -cl * so],
            [cl, 0.0, -sl],
        ],
        dtype=float,
    )


def wrap_deg(angle_deg: np.ndarray) -> np.ndarray:
    return (angle_deg + 180.0) % 360.0 - 180.0


def load_truth_full(path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(path, sep=r"\s+")
    if not {"timestamp", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"}.issubset(df.columns):
        raw = pd.read_csv(path, sep=r"\s+", header=None)
        if raw.shape[1] < 10:
            raise RuntimeError(f"truth file columns < 10: {path}")
        raw.columns = ["timestamp", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"]
        df = raw

    t = df["timestamp"].to_numpy(dtype=float)
    lat_deg = df["lat"].to_numpy(dtype=float)
    lon_deg = df["lon"].to_numpy(dtype=float)
    h = df["h"].to_numpy(dtype=float)
    vn = df["vn"].to_numpy(dtype=float)
    ve = df["ve"].to_numpy(dtype=float)
    vd = df["vd"].to_numpy(dtype=float)
    pos_ecef = llh_deg_to_ecef(lat_deg, lon_deg, h)

    vel_ecef = np.zeros((t.size, 3), dtype=float)
    for i in range(t.size):
        r_ne = rot_ned_to_ecef(math.radians(lat_deg[i]), math.radians(lon_deg[i]))
        vel_ecef[i] = r_ne @ np.array([vn[i], ve[i], vd[i]], dtype=float)

    rpy_deg = df[["roll", "pitch", "yaw"]].to_numpy(dtype=float)
    return {
        "timestamp": t,
        "pos_ecef": pos_ecef,
        "vel_ecef": vel_ecef,
        "rpy_deg": rpy_deg,
    }


def load_solution_series(path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(path, sep=r"\s+")
    required = {
        "timestamp",
        "fused_x",
        "fused_y",
        "fused_z",
        "fused_vx",
        "fused_vy",
        "fused_vz",
        "fused_roll",
        "fused_pitch",
        "fused_yaw",
    }
    missing = required.difference(df.columns)
    if missing:
        raise RuntimeError(f"missing fused columns in solution file {path}: {sorted(missing)}")
    return {
        "timestamp": df["timestamp"].to_numpy(dtype=float),
        "pos_ecef": df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float),
        "vel_ecef": df[["fused_vx", "fused_vy", "fused_vz"]].to_numpy(dtype=float),
        "rpy_deg": df[["fused_roll", "fused_pitch", "fused_yaw"]].to_numpy(dtype=float),
    }


def interp_columns(t_query: np.ndarray, t_src: np.ndarray, cols: np.ndarray) -> np.ndarray:
    out = np.zeros((t_query.size, cols.shape[1]), dtype=float)
    for i in range(cols.shape[1]):
        out[:, i] = np.interp(t_query, t_src, cols[:, i])
    return out


def interp_angles_deg(t_query: np.ndarray, t_src: np.ndarray, cols_deg: np.ndarray) -> np.ndarray:
    out = np.zeros((t_query.size, cols_deg.shape[1]), dtype=float)
    for i in range(cols_deg.shape[1]):
        rad = np.unwrap(np.deg2rad(cols_deg[:, i]))
        out[:, i] = np.rad2deg(np.interp(t_query, t_src, rad))
    return out


def build_case_config(
    base_cfg: dict[str, Any],
    case_id: str,
    imu_path: Path,
    case_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg.setdefault("fusion", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    init_cfg = fusion.setdefault("init", {})

    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.txt"

    fusion["imu_path"] = rel_from_root(imu_path, REPO_ROOT)
    fusion["gnss_path"] = ""
    fusion["uwb_path"] = ""
    fusion["output_path"] = rel_from_root(sol_path, REPO_ROOT)
    fusion["state_series_output_path"] = rel_from_root(state_series_path, REPO_ROOT)
    fusion["first_update_debug_output_path"] = ""
    fusion["gnss_update_debug_output_path"] = ""
    fusion["enable_gnss_velocity"] = False
    fusion["gnss_schedule"] = {"enabled": False}
    fusion["post_gnss_ablation"] = {"enabled": False}

    constraints_cfg["enable_odo"] = False
    constraints_cfg["enable_nhc"] = False
    constraints_cfg["enable_zupt"] = False
    constraints_cfg["enable_diagnostics"] = False
    constraints_cfg["enable_consistency_log"] = False
    constraints_cfg["enable_mechanism_log"] = False

    init_cfg["use_truth_pva"] = True
    init_cfg["runtime_truth_anchor_pva"] = False
    init_cfg["runtime_truth_anchor_gnss_only"] = False
    init_cfg["runtime_truth_anchor_position"] = False
    init_cfg["runtime_truth_anchor_velocity"] = False
    init_cfg["runtime_truth_anchor_attitude"] = False

    overrides = {
        "fusion.imu_path": rel_from_root(imu_path, REPO_ROOT),
        "fusion.gnss_path": "",
        "fusion.uwb_path": "",
        "fusion.enable_gnss_velocity": False,
        "fusion.constraints.enable_odo": False,
        "fusion.constraints.enable_nhc": False,
        "fusion.constraints.enable_zupt": False,
        "fusion.init.use_truth_pva": True,
        "fusion.init.runtime_truth_anchor_pva": False,
    }
    return cfg, overrides


def write_case_config(base_cfg: dict[str, Any], case_id: str, imu_path: Path, case_dir: Path) -> tuple[Path, dict[str, Any]]:
    cfg, overrides = build_case_config(base_cfg, case_id, imu_path, case_dir)
    cfg_path = case_dir / f"config_{case_id}.yaml"
    save_yaml(cfg, cfg_path)
    return cfg_path, overrides


def compute_window_metrics(t: np.ndarray, pos_err: np.ndarray, vel_err: np.ndarray, rpy_err_deg: np.ndarray) -> list[dict[str, Any]]:
    elapsed = t - float(t[0])
    rows: list[dict[str, Any]] = []
    windows: list[tuple[str, np.ndarray]] = []
    for horizon_s in HORIZONS_S:
        mask = elapsed <= horizon_s + 1.0e-9
        if int(np.count_nonzero(mask)) >= 2:
            windows.append((f"{int(horizon_s)}s", mask))
    windows.append(("full", np.ones_like(elapsed, dtype=bool)))

    for label, mask in windows:
        p = pos_err[mask]
        v = vel_err[mask]
        a = rpy_err_deg[mask]
        p3 = np.linalg.norm(p, axis=1)
        v3 = np.linalg.norm(v, axis=1)
        a3 = np.linalg.norm(a, axis=1)
        rows.append(
            {
                "window": label,
                "samples": int(p.shape[0]),
                "duration_s": float(elapsed[mask][-1]),
                "pos_rmse_x_m": float(np.sqrt(np.mean(p[:, 0] ** 2))),
                "pos_rmse_y_m": float(np.sqrt(np.mean(p[:, 1] ** 2))),
                "pos_rmse_z_m": float(np.sqrt(np.mean(p[:, 2] ** 2))),
                "pos_rmse_3d_m": float(np.sqrt(np.mean(p3 ** 2))),
                "pos_final_3d_m": float(p3[-1]),
                "vel_rmse_3d_mps": float(np.sqrt(np.mean(v3 ** 2))),
                "vel_final_3d_mps": float(v3[-1]),
                "att_rmse_3d_deg": float(np.sqrt(np.mean(a3 ** 2))),
                "roll_rmse_deg": float(np.sqrt(np.mean(a[:, 0] ** 2))),
                "pitch_rmse_deg": float(np.sqrt(np.mean(a[:, 1] ** 2))),
                "yaw_rmse_deg": float(np.sqrt(np.mean(a[:, 2] ** 2))),
                "yaw_final_abs_deg": float(abs(a[-1, 2])),
            }
        )
    return rows


def run_case(
    case_id: str,
    case_label: str,
    source_note: str,
    cfg_path: Path,
    case_dir: Path,
    exe_path: Path,
    truth: dict[str, np.ndarray],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    sol_path = case_dir / f"SOL_{case_id}.txt"
    state_series_path = case_dir / f"state_series_{case_id}.txt"
    stdout_path = case_dir / f"{case_id}.stdout.txt"

    stdout_text = run_command([str(exe_path.resolve()), "--config", str(cfg_path.resolve())], REPO_ROOT)
    stdout_path.write_text(stdout_text, encoding="utf-8")
    if not sol_path.exists():
        raise RuntimeError(f"missing solver output: {sol_path}")
    if not state_series_path.exists():
        raise RuntimeError(f"missing state series output: {state_series_path}")

    est = load_solution_series(sol_path)
    truth_pos = interp_columns(est["timestamp"], truth["timestamp"], truth["pos_ecef"])
    truth_vel = interp_columns(est["timestamp"], truth["timestamp"], truth["vel_ecef"])
    truth_rpy = interp_angles_deg(est["timestamp"], truth["timestamp"], truth["rpy_deg"])

    pos_err = est["pos_ecef"] - truth_pos
    vel_err = est["vel_ecef"] - truth_vel
    rpy_err = wrap_deg(est["rpy_deg"] - truth_rpy)
    window_rows = compute_window_metrics(est["timestamp"], pos_err, vel_err, rpy_err)

    full_row = next(row for row in window_rows if row["window"] == "full")
    case_row = {
        "case_id": case_id,
        "label": case_label,
        "source_note": source_note,
        "config_path": rel_from_root(cfg_path, REPO_ROOT),
        "sol_path": rel_from_root(sol_path, REPO_ROOT),
        "state_series_path": rel_from_root(state_series_path, REPO_ROOT),
        "stdout_path": rel_from_root(stdout_path, REPO_ROOT),
        "sol_mtime": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
        "state_series_mtime": dt.datetime.fromtimestamp(state_series_path.stat().st_mtime).isoformat(timespec="seconds"),
        "stdout_mtime": dt.datetime.fromtimestamp(stdout_path.stat().st_mtime).isoformat(timespec="seconds"),
    }
    case_row.update(full_row)
    return case_row, window_rows


def build_summary(case_df: pd.DataFrame, window_df: pd.DataFrame) -> str:
    lines = [
        "# data2 pure INS IMU source compare",
        "",
        "- pipeline: pure `INS` only, explicit `enable_odo=false`, `enable_nhc=false`, `enable_zupt=false`, `gnss_path=''`, `uwb_path=''`.",
        "- initialization: `use_truth_pva=true`, `runtime_truth_anchor_pva=false`; both cases use the same initial state and same solver settings except `fusion.imu_path`.",
        "- truth reference: `dataset/data2_converted/POS_converted.txt`.",
        "- note: position error may still contain POS-vs-T3 reference-point effects; velocity and attitude are the cleaner IMU-quality indicators.",
        "",
        "## Full-window comparison",
    ]
    cols = [
        "label",
        "pos_rmse_3d_m",
        "pos_final_3d_m",
        "vel_rmse_3d_mps",
        "vel_final_3d_mps",
        "att_rmse_3d_deg",
        "yaw_rmse_deg",
        "yaw_final_abs_deg",
    ]
    for row in case_df[cols].sort_values("pos_rmse_3d_m").to_dict("records"):
        lines.append(
            f"- `{row['label']}`: "
            f"`pos_rmse_3d={row['pos_rmse_3d_m']:.6f} m`, "
            f"`pos_final_3d={row['pos_final_3d_m']:.6f} m`, "
            f"`vel_rmse_3d={row['vel_rmse_3d_mps']:.6f} m/s`, "
            f"`att_rmse_3d={row['att_rmse_3d_deg']:.6f} deg`, "
            f"`yaw_rmse={row['yaw_rmse_deg']:.6f} deg`."
        )

    lines += ["", "## Horizon slices"]
    for window in ["1s", "5s", "10s", "30s", "60s", "120s", "300s", "600s", "1200s", "full"]:
        wdf = window_df[window_df["window"] == window]
        if wdf.empty:
            continue
        lines.append(f"- `{window}`:")
        for row in wdf.sort_values("pos_rmse_3d_m").to_dict("records"):
            lines.append(
                f"  `{row['label']}` -> "
                f"`pos_rmse_3d={row['pos_rmse_3d_m']:.6f} m`, "
                f"`vel_rmse_3d={row['vel_rmse_3d_mps']:.6f} m/s`, "
                f"`yaw_rmse={row['yaw_rmse_deg']:.6f} deg`, "
                f"`yaw_final_abs={row['yaw_final_abs_deg']:.6f} deg`."
            )

    better_counts: dict[str, int] = {}
    for metric in ["pos_rmse_3d_m", "vel_rmse_3d_mps", "yaw_rmse_deg"]:
        subset = window_df[window_df["window"].isin(["1s", "5s", "10s", "30s", "60s", "120s", "300s", "600s", "1200s", "full"])]
        winners = subset.sort_values(["window", metric]).groupby("window", as_index=False).first()
        better_counts[metric] = int((winners["label"] == "IMULog").sum())

    lines += [
        "",
        "## Takeaway",
        f"- `IMULog` wins `{better_counts['pos_rmse_3d_m']}` horizon slices on `pos_rmse_3d_m`, "
        f"`{better_counts['vel_rmse_3d_mps']}` on `vel_rmse_3d_mps`, "
        f"`{better_counts['yaw_rmse_deg']}` on `yaw_rmse_deg`."
        if "IMULog" in case_df["label"].values
        else "- comparison rows missing.",
    ]
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare pure INS accuracy between IMULog and HL IMU sources on data2.")
    parser.add_argument("--base-config", type=Path, default=Path("config_data2_baseline_eskf.yaml"))
    parser.add_argument("--exe", type=Path, default=Path("build/Release/eskf_fusion.exe"))
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument(
        "--cases",
        default="imulog,hl",
        help="Comma-separated case ids from {imulog,hl}.",
    )
    args = parser.parse_args()
    args.base_config = (REPO_ROOT / args.base_config).resolve()
    args.exe = (REPO_ROOT / args.exe).resolve()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    return args


def main() -> None:
    args = parse_args()
    if not args.base_config.exists():
        raise FileNotFoundError(f"missing base config: {args.base_config}")
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    case_ids = [token.strip() for token in args.cases.split(",") if token.strip()]
    if not case_ids:
        raise RuntimeError("no cases selected")
    invalid = [case_id for case_id in case_ids if case_id not in CASE_SPECS]
    if invalid:
        raise RuntimeError(f"unsupported cases: {invalid}")

    reset_directory(args.output_dir)
    artifacts_dir = args.output_dir / "artifacts"
    cases_dir = artifacts_dir / "cases"
    ensure_dir(cases_dir)

    base_cfg = load_yaml(args.base_config)
    truth_path = (REPO_ROOT / base_cfg["fusion"]["pos_path"]).resolve()
    truth = load_truth_full(truth_path)

    case_rows: list[dict[str, Any]] = []
    window_rows: list[dict[str, Any]] = []
    case_config_paths: dict[str, str] = {}
    exact_overrides: dict[str, dict[str, Any]] = {}

    for case_id in case_ids:
        spec = CASE_SPECS[case_id]
        imu_path = (REPO_ROOT / spec["imu_path"]).resolve()
        if not imu_path.exists():
            raise FileNotFoundError(f"missing imu path for {case_id}: {imu_path}")
        case_dir = cases_dir / case_id
        ensure_dir(case_dir)
        cfg_path, overrides = write_case_config(base_cfg, case_id, imu_path, case_dir)
        case_config_paths[case_id] = rel_from_root(cfg_path, REPO_ROOT)
        exact_overrides[case_id] = overrides
        case_row, per_window = run_case(case_id, spec["label"], spec["source_note"], cfg_path, case_dir, args.exe, truth)
        case_rows.append(case_row)
        for row in per_window:
            row["case_id"] = case_id
            row["label"] = spec["label"]
            window_rows.append(row)

    case_df = pd.DataFrame(case_rows)
    window_df = pd.DataFrame(window_rows)
    case_metrics_path = args.output_dir / "case_metrics.csv"
    horizon_metrics_path = args.output_dir / "horizon_metrics.csv"
    case_df.to_csv(case_metrics_path, index=False, encoding="utf-8-sig")
    window_df.to_csv(horizon_metrics_path, index=False, encoding="utf-8-sig")

    summary_path = args.output_dir / "summary.md"
    summary_path.write_text(build_summary(case_df, window_df), encoding="utf-8")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "base_config": rel_from_root(args.base_config, REPO_ROOT),
        "solver_exe": rel_from_root(args.exe, REPO_ROOT),
        "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
        "artifacts_dir": rel_from_root(artifacts_dir, REPO_ROOT),
        "truth_path": rel_from_root(truth_path, REPO_ROOT),
        "case_metrics_csv": rel_from_root(case_metrics_path, REPO_ROOT),
        "horizon_metrics_csv": rel_from_root(horizon_metrics_path, REPO_ROOT),
        "summary_md": rel_from_root(summary_path, REPO_ROOT),
        "selected_cases": case_ids,
        "case_specs": {
            case_id: {
                "label": CASE_SPECS[case_id]["label"],
                "imu_path": rel_from_root((REPO_ROOT / CASE_SPECS[case_id]["imu_path"]).resolve(), REPO_ROOT),
                "source_note": CASE_SPECS[case_id]["source_note"],
                "config_path": case_config_paths[case_id],
                "exact_overrides": exact_overrides[case_id],
            }
            for case_id in case_ids
        },
    }
    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(json_safe(manifest), indent=2, ensure_ascii=False), encoding="utf-8")

    print(
        json.dumps(
            {
                "exp_id": args.exp_id,
                "output_dir": rel_from_root(args.output_dir, REPO_ROOT),
                "summary": rel_from_root(summary_path, REPO_ROOT),
                "case_metrics": rel_from_root(case_metrics_path, REPO_ROOT),
                "horizon_metrics": rel_from_root(horizon_metrics_path, REPO_ROOT),
                "manifest": rel_from_root(manifest_path, REPO_ROOT),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
