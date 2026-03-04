#!/usr/bin/env python3
"""Run state-ablation experiments and generate comparison plots."""

from __future__ import annotations

import argparse
import copy
import math
import re
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F


def llh_deg_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, h_m: np.ndarray) -> np.ndarray:
  lat = np.deg2rad(lat_deg)
  lon = np.deg2rad(lon_deg)
  sin_lat = np.sin(lat)
  cos_lat = np.cos(lat)
  sin_lon = np.sin(lon)
  cos_lon = np.cos(lon)
  N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
  x = (N + h_m) * cos_lat * cos_lon
  y = (N + h_m) * cos_lat * sin_lon
  z = (N * (1.0 - WGS84_E2) + h_m) * sin_lat
  return np.column_stack((x, y, z))


def load_truth_ecef(pos_path: Path) -> tuple[np.ndarray, np.ndarray]:
  df = pd.read_csv(pos_path, sep=r"\s+")
  if {"timestamp", "lat", "lon", "h"}.issubset(df.columns):
    ecef = llh_deg_to_ecef(df["lat"].to_numpy(), df["lon"].to_numpy(), df["h"].to_numpy())
    return df["timestamp"].to_numpy(), ecef
  if {"timestamp", "x", "y", "z"}.issubset(df.columns):
    ecef = df[["x", "y", "z"]].to_numpy()
    return df["timestamp"].to_numpy(), ecef
  # Fallback: assume timestamp + xyz in columns [0,1,2,3]
  t = df.iloc[:, 0].to_numpy()
  xyz = df.iloc[:, 1:4].to_numpy()
  if np.max(np.abs(xyz[:, :2])) < 200.0:
    xyz = llh_deg_to_ecef(xyz[:, 0], xyz[:, 1], xyz[:, 2])
  return t, xyz


def interp_truth(sol_t: np.ndarray, truth_t: np.ndarray, truth_xyz: np.ndarray) -> np.ndarray:
  interp_xyz = np.zeros((sol_t.size, 3), dtype=float)
  for i in range(3):
    interp_xyz[:, i] = np.interp(sol_t, truth_t, truth_xyz[:, i])
  return interp_xyz


def load_sol(sol_path: Path) -> tuple[np.ndarray, np.ndarray]:
  try:
    df = pd.read_csv(sol_path, sep=r"\s+")
    expected = {"timestamp", "fused_x", "fused_y", "fused_z"}
    if expected.issubset(df.columns):
      return df["timestamp"].to_numpy(), df[["fused_x", "fused_y", "fused_z"]].to_numpy()

    # Header-less files can be parsed with the first data row as header by default.
    # Reload with header=None to keep all numeric rows.
    raw = pd.read_csv(sol_path, sep=r"\s+", header=None)
    if raw.shape[1] < 4:
      raise RuntimeError(f"expected >=4 columns, got {raw.shape[1]}")

    print(f"[WARN] load_sol: expected columns not found, fallback to [0,1,2,3]: {sol_path}")
    arr = raw.iloc[:, :4].to_numpy(dtype=float)
    return arr[:, 0], arr[:, 1:4]
  except Exception as exc:
    raise RuntimeError(f"load_sol failed for {sol_path}: {exc}") from exc


def run_cmd(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
  return subprocess.run(
      cmd,
      cwd=str(cwd),
      capture_output=True,
      text=True,
      encoding="utf-8",
      errors="ignore",
      check=False,
  )


def parse_rmse(stdout: str) -> tuple[float, float, float] | None:
  m = re.search(r"RMSE \(融合\) \[m\]:\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", stdout)
  if not m:
    return None
  return float(m.group(1)), float(m.group(2)), float(m.group(3))


def main() -> int:
  parser = argparse.ArgumentParser(description="Run 31D state ablation experiments.")
  parser.add_argument("--config", default="config_data4_gnss_std.yaml", help="Base fusion config path")
  parser.add_argument("--exe", default="build/Release/eskf_fusion.exe", help="eskf_fusion executable path")
  args = parser.parse_args()

  root = Path(__file__).resolve().parents[2]
  base_config_path = (root / args.config).resolve()
  exe_path = (root / args.exe).resolve()
  if not base_config_path.exists():
    raise FileNotFoundError(f"Base config not found: {base_config_path}")
  if not exe_path.exists():
    raise FileNotFoundError(f"Executable not found: {exe_path}")

  with base_config_path.open("r", encoding="utf-8") as f:
    base_cfg = yaml.safe_load(f)

  fusion_cfg = base_cfg.get("fusion", {})
  pos_path = (root / fusion_cfg.get("pos_path", "")).resolve()
  if not pos_path.exists():
    raise FileNotFoundError(f"Truth/POS file not found: {pos_path}")

  out_root = root / "output" / "ablation"
  cfg_out_dir = out_root / "configs"
  sol_out_dir = out_root / "results"
  cmp_out_dir = out_root / "comparison"
  cfg_out_dir.mkdir(parents=True, exist_ok=True)
  sol_out_dir.mkdir(parents=True, exist_ok=True)
  cmp_out_dir.mkdir(parents=True, exist_ok=True)

  cases = [
      ("full_31d", {}),
      ("no_gnss_lever_arm", {"disable_gnss_lever_arm": True}),
      ("no_odo_lever_arm", {"disable_odo_lever_arm": True}),
      ("no_odo_scale", {"disable_odo_scale": True}),
      ("no_gyro_scale", {"disable_gyro_scale": True}),
      ("no_accel_scale", {"disable_accel_scale": True}),
      ("no_mounting", {"disable_mounting": True}),
  ]
  keys = [
      "disable_gnss_lever_arm",
      "disable_odo_lever_arm",
      "disable_odo_scale",
      "disable_gyro_scale",
      "disable_accel_scale",
      "disable_mounting",
  ]

  truth_t, truth_xyz = load_truth_ecef(pos_path)
  summary_rows: list[dict[str, float | str]] = []
  curve_store: dict[str, tuple[np.ndarray, np.ndarray]] = {}

  for case_name, flags in cases:
    cfg = copy.deepcopy(base_cfg)
    cfg.setdefault("fusion", {})
    cfg["fusion"]["output_path"] = str((sol_out_dir / f"SOL_{case_name}.txt").as_posix())
    cfg["fusion"]["ablation"] = {k: bool(flags.get(k, False)) for k in keys}

    cfg_path = cfg_out_dir / f"config_{case_name}.yaml"
    with cfg_path.open("w", encoding="utf-8") as f:
      yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)

    print(f"[RUN] {case_name}")
    proc = run_cmd([str(exe_path), "--config", str(cfg_path)], root)
    if proc.returncode != 0:
      print(proc.stdout)
      print(proc.stderr)
      raise RuntimeError(f"Experiment failed: {case_name}")
    rmse_txt = parse_rmse(proc.stdout)
    print(proc.stdout.strip().splitlines()[-2] if proc.stdout.strip().splitlines() else "")

    sol_path = sol_out_dir / f"SOL_{case_name}.txt"
    if not sol_path.exists():
      raise FileNotFoundError(f"SOL output not found: {sol_path}")

    # Generate per-case figures using existing plotting script.
    plot_proc = run_cmd([sys.executable, str(root / "plot_navresult.py"), str(sol_path)], root)
    if plot_proc.returncode != 0:
      print(plot_proc.stdout)
      print(plot_proc.stderr)
      raise RuntimeError(f"Plot generation failed: {case_name}")

    sol_t, sol_xyz = load_sol(sol_path)
    truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
    err = sol_xyz - truth_interp
    err_norm = np.linalg.norm(err, axis=1)
    rmse_xyz = np.sqrt(np.mean(err * err, axis=0))
    rmse_3d = float(np.sqrt(np.mean(err_norm * err_norm)))
    final_3d = float(err_norm[-1]) if err_norm.size else math.nan
    curve_store[case_name] = (sol_t - sol_t[0], err_norm)

    row = {
        "case": case_name,
        "rmse_x_m": float(rmse_xyz[0]),
        "rmse_y_m": float(rmse_xyz[1]),
        "rmse_z_m": float(rmse_xyz[2]),
        "rmse_3d_m": rmse_3d,
        "final_err_3d_m": final_3d,
    }
    if rmse_txt is not None:
      row["rmse_stdout_x_m"] = rmse_txt[0]
      row["rmse_stdout_y_m"] = rmse_txt[1]
      row["rmse_stdout_z_m"] = rmse_txt[2]
    summary_rows.append(row)

  summary_df = pd.DataFrame(summary_rows)
  summary_df.sort_values("rmse_3d_m", inplace=True)
  summary_csv = cmp_out_dir / "ablation_summary.csv"
  summary_df.to_csv(summary_csv, index=False, encoding="utf-8-sig")

  # Plot 1: 3D position error norm over time.
  plt.figure(figsize=(14, 7))
  for case_name, (tt, ee) in curve_store.items():
    plt.plot(tt, ee, linewidth=0.9, label=case_name)
  plt.xlabel("Time [s]")
  plt.ylabel("3D Position Error [m]")
  plt.title("State Ablation Comparison: 3D Position Error")
  plt.grid(alpha=0.3)
  plt.legend(fontsize=8, ncol=2)
  plt.tight_layout()
  plt.savefig(cmp_out_dir / "ablation_error_norm.png", dpi=150)
  plt.close()

  # Plot 2: RMSE_3D bar chart.
  plot_df = summary_df.sort_values("rmse_3d_m", ascending=False)
  plt.figure(figsize=(12, 6))
  bars = plt.bar(plot_df["case"], plot_df["rmse_3d_m"], color="#4c72b0")
  for b in bars:
    h = b.get_height()
    plt.text(b.get_x() + b.get_width() * 0.5, h, f"{h:.3f}", ha="center", va="bottom", fontsize=8)
  plt.xticks(rotation=30, ha="right")
  plt.ylabel("RMSE 3D [m]")
  plt.title("State Ablation Comparison: RMSE 3D")
  plt.grid(axis="y", alpha=0.3)
  plt.tight_layout()
  plt.savefig(cmp_out_dir / "ablation_rmse_bar.png", dpi=150)
  plt.close()

  print("\n[Done] Saved:")
  print(f"  - {summary_csv}")
  print(f"  - {cmp_out_dir / 'ablation_error_norm.png'}")
  print(f"  - {cmp_out_dir / 'ablation_rmse_bar.png'}")
  print(f"  - per-case SOL files: {sol_out_dir}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
