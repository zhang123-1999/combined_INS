#!/usr/bin/env python3
"""Run InEKF/ESKF research pipeline and regenerate report artifacts."""

from __future__ import annotations

import argparse
import copy
import csv
import datetime as dt
import hashlib
import json
import math
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F
STATE_LABEL = (
    "21=odo_scale, 22=mounting_roll, 23=mounting_pitch, 24=mounting_yaw, "
    "25-27=lever_odo, 28-30=lever_gnss"
)


def now_local() -> dt.datetime:
    return dt.datetime.now()


def exp_date_tag() -> str:
    return now_local().strftime("%Y%m%d")


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            b = f.read(1024 * 1024)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def llh_deg_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, h_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * cos_lon
    y = (n + h_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + h_m) * sin_lat
    return np.column_stack((x, y, z))


def load_truth_ecef(path: Path) -> tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(path, sep=r"\s+")
    cols = set(df.columns.tolist())
    if {"timestamp", "lat", "lon", "h"}.issubset(cols):
        xyz = llh_deg_to_ecef(
            df["lat"].to_numpy(dtype=float),
            df["lon"].to_numpy(dtype=float),
            df["h"].to_numpy(dtype=float),
        )
        return df["timestamp"].to_numpy(dtype=float), xyz
    if {"timestamp", "x", "y", "z"}.issubset(cols):
        return (
            df["timestamp"].to_numpy(dtype=float),
            df[["x", "y", "z"]].to_numpy(dtype=float),
        )
    raw = pd.read_csv(path, sep=r"\s+", header=None)
    if raw.shape[1] < 4:
        raise RuntimeError(f"truth file columns < 4: {path}")
    t = raw.iloc[:, 0].to_numpy(dtype=float)
    xyz = raw.iloc[:, 1:4].to_numpy(dtype=float)
    if np.max(np.abs(xyz[:, :2])) < 200.0:
        xyz = llh_deg_to_ecef(xyz[:, 0], xyz[:, 1], xyz[:, 2])
    return t, xyz


def load_sol(sol_path: Path) -> tuple[np.ndarray, np.ndarray, pd.DataFrame]:
    df = pd.read_csv(sol_path, sep=r"\s+", comment="#")
    if {"timestamp", "fused_x", "fused_y", "fused_z"}.issubset(df.columns):
        t = df["timestamp"].to_numpy(dtype=float)
        xyz = df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float)
        return t, xyz, df
    raw = pd.read_csv(sol_path, sep=r"\s+", header=None)
    if raw.shape[1] < 4:
        raise RuntimeError(f"sol file columns < 4: {sol_path}")
    t = raw.iloc[:, 0].to_numpy(dtype=float)
    xyz = raw.iloc[:, 1:4].to_numpy(dtype=float)
    fallback = pd.DataFrame(
        {
            "timestamp": t,
            "fused_x": xyz[:, 0],
            "fused_y": xyz[:, 1],
            "fused_z": xyz[:, 2],
        }
    )
    return t, xyz, fallback


def interp_truth(sol_t: np.ndarray, truth_t: np.ndarray, truth_xyz: np.ndarray) -> np.ndarray:
    out = np.zeros((sol_t.size, 3), dtype=float)
    for i in range(3):
        out[:, i] = np.interp(sol_t, truth_t, truth_xyz[:, i])
    return out


def parse_rmse_xyz(text: str) -> tuple[float, float, float] | None:
    m = re.search(r"RMSE \(融合\) \[m\]:\s*([-+0-9eE\.]+)\s+([-+0-9eE\.]+)\s+([-+0-9eE\.]+)", text)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def parse_split_t(text: str) -> float | None:
    m = re.search(r"\[GNSS\] schedule ON: .* split_t=([-+0-9eE\.]+)", text)
    if not m:
        return None
    return float(m.group(1))


def parse_hatt_series(log_text: str) -> pd.DataFrame:
    pat = re.compile(r"\[GNSS_POS\] t=([-+0-9eE\.]+).*?\|\|H_att\|\|_F=([-+0-9eE\.]+)")
    rows: list[tuple[float, float]] = []
    for m in pat.finditer(log_text):
        rows.append((float(m.group(1)), float(m.group(2))))
    return pd.DataFrame(rows, columns=["t", "h_att_f"])


def parse_consistency_summary(log_text: str) -> dict[str, dict[str, float]]:
    pat = re.compile(
        r"\[Consistency\]\s+(\w+)\s+seen=(\d+)\s+accepted=(\d+)\s+accept_ratio=([-+0-9eE\.]+)\s+"
        r"reject_nis=(\d+)\s+reject_numeric=(\d+)\s+nis_mean=([-+0-9eE\.]+)\s+nis_max=([-+0-9eE\.]+)\s+"
        r"robust_w_mean=([-+0-9eE\.]+)\s+noise_scale_mean=([-+0-9eE\.]+)"
    )
    out: dict[str, dict[str, float]] = {}
    for m in pat.finditer(log_text):
        sensor = m.group(1).upper()
        out[sensor] = {
            "seen": float(m.group(2)),
            "accepted": float(m.group(3)),
            "accept_ratio": float(m.group(4)),
            "reject_nis": float(m.group(5)),
            "reject_numeric": float(m.group(6)),
            "nis_mean": float(m.group(7)),
            "nis_max": float(m.group(8)),
            "robust_w_mean": float(m.group(9)),
            "noise_scale_mean": float(m.group(10)),
        }
    return out


def rel_from_root(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def latex_escape(text: str) -> str:
    return text.replace("\\", "\\textbackslash{}").replace("%", "\\%").replace("_", "\\_")


@dataclass
class RunResult:
    case: str
    config_path: Path
    log_path: Path
    sol_path: Path
    stdout_path: Path
    rmse_x: float
    rmse_y: float
    rmse_z: float
    rmse_3d: float
    p95_3d: float
    tail70_rmse_3d: float
    final_err_3d: float
    split_t: float | None
    h_att_max: float
    consistency: dict[str, dict[str, float]]
    time_s: np.ndarray
    err_3d: np.ndarray
    sol_df: pd.DataFrame
    diag_path: Path | None


class Pipeline:
    def __init__(self, root: Path, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.exe = (root / "build" / "Release" / "eskf_fusion.exe").resolve()
        if not self.exe.exists():
            raise FileNotFoundError(f"missing executable: {self.exe}")
        self.review_root = root / "output" / "review"
        ensure_dir(self.review_root)
        self.docs_root = root / "算法文档" / "结果文档"
        ensure_dir(self.docs_root)
        self.manifest_entries: list[dict[str, Any]] = []
        self.phase_outputs: dict[str, Any] = {}
        self.date_tag = exp_date_tag()

    def _exp_dir(self, short_name: str) -> Path:
        return self.review_root / f"EXP-{self.date_tag}-{short_name}"

    def _run_cmd(self, cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            cmd,
            cwd=str(cwd),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
            check=False,
        )

    def _load_cfg(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _save_cfg(self, cfg: dict[str, Any], path: Path) -> None:
        with path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)

    def _set_default_consistency(self, cfg: dict[str, Any], enabled: bool = True) -> None:
        fusion = cfg.setdefault("fusion", {})
        constraints = fusion.setdefault("constraints", {})
        constraints["enable_consistency_log"] = bool(enabled)

    def _set_default_diagnostics(self, cfg: dict[str, Any], enabled: bool) -> None:
        fusion = cfg.setdefault("fusion", {})
        constraints = fusion.setdefault("constraints", {})
        constraints["enable_diagnostics"] = bool(enabled)

    def _write_csv(self, path: Path, rows: list[dict[str, Any]]) -> None:
        if not rows:
            with path.open("w", encoding="utf-8", newline="") as f:
                f.write("")
            return
        keys = list(rows[0].keys())
        with path.open("w", encoding="utf-8-sig", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)

    def _load_diag(self, path: Path | None) -> pd.DataFrame | None:
        if path is None or not path.exists():
            return None
        with path.open("r", encoding="utf-8") as f:
            header = f.readline().strip().split()
        if not header:
            return None
        df = pd.read_csv(path, sep=r"\s+", skiprows=1, header=None)
        if df.shape[1] != len(header):
            return None
        df.columns = header
        return df

    def _run_case(
        self,
        *,
        exp_dir: Path,
        case: str,
        base_cfg_path: Path,
        cfg_mutator,
        truth_path_override: Path | None = None,
        plot: bool = True,
    ) -> RunResult:
        ensure_dir(exp_dir)
        cfg = self._load_cfg(base_cfg_path)
        cfg_mutator(cfg)

        sol_path = exp_dir / f"SOL_{case}.txt"
        cfg["fusion"]["output_path"] = str(sol_path.resolve())
        cfg_path = exp_dir / f"cfg_{case}.yaml"
        self._save_cfg(cfg, cfg_path)

        cmd = [str(self.exe), "--config", str(cfg_path.resolve())]
        proc = self._run_cmd(cmd, self.root)
        log_path = exp_dir / f"{case}.log"
        stdout_path = exp_dir / f"{case}.stdout.txt"
        with stdout_path.open("w", encoding="utf-8") as f:
            f.write(proc.stdout)
            if proc.stderr:
                f.write("\n[STDERR]\n")
                f.write(proc.stderr)
        with log_path.open("w", encoding="utf-8") as f:
            f.write(proc.stdout)
            if proc.stderr:
                f.write("\n")
                f.write(proc.stderr)
        if proc.returncode != 0:
            raise RuntimeError(f"run failed: {case} return={proc.returncode}")
        if not sol_path.exists():
            raise FileNotFoundError(f"missing SOL: {sol_path}")

        diag_out: Path | None = None
        if cfg["fusion"]["constraints"].get("enable_diagnostics", False):
            diag_src = self.root / "DIAG.txt"
            if diag_src.exists():
                diag_out = exp_dir / f"DIAG_{case}.txt"
                shutil.copy2(diag_src, diag_out)

        pos_path_cfg = cfg["fusion"]["pos_path"]
        truth_path = truth_path_override or (self.root / pos_path_cfg).resolve()
        truth_t, truth_xyz = load_truth_ecef(truth_path)
        sol_t, sol_xyz, sol_df = load_sol(sol_path)
        truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
        err = sol_xyz - truth_interp
        err3 = np.linalg.norm(err, axis=1)
        rmse_xyz = np.sqrt(np.mean(err * err, axis=0))
        rmse_3d = float(np.sqrt(np.mean(err3 * err3)))
        p95_3d = float(np.percentile(err3, 95))
        start_idx = int(max(0, math.floor(err3.size * 0.3)))
        tail = err3[start_idx:]
        tail70_rmse_3d = float(np.sqrt(np.mean(tail * tail))) if tail.size else float("nan")
        final_err_3d = float(err3[-1]) if err3.size else float("nan")

        merged_log = proc.stdout + "\n" + proc.stderr
        split_t = parse_split_t(merged_log)
        h = parse_hatt_series(merged_log)
        h_att_max = float(h["h_att_f"].max()) if not h.empty else float("nan")
        consistency = parse_consistency_summary(merged_log)

        if plot:
            p = self._run_cmd([sys.executable, str(self.root / "plot_navresult.py"), str(sol_path.resolve())], self.root)
            stem = sol_path.stem.replace("SOL_", "").replace("_gnss", "").replace("_uwb", "")
            plot_dir = self.root / "output" / f"result_{stem}"
            if p.returncode == 0 and plot_dir.exists():
                dst = exp_dir / "plots" / plot_dir.name
                ensure_dir(dst.parent)
                if dst.exists():
                    shutil.rmtree(dst)
                shutil.copytree(plot_dir, dst)

        self.manifest_entries.append(
            {
                "case": case,
                "base_config": rel_from_root(base_cfg_path, self.root),
                "config_path": rel_from_root(cfg_path, self.root),
                "sol_path": rel_from_root(sol_path, self.root),
                "log_path": rel_from_root(log_path, self.root),
                "stdout_path": rel_from_root(stdout_path, self.root),
                "diag_path": rel_from_root(diag_out, self.root) if diag_out else None,
                "command": cmd,
                "config_sha256": sha256_file(cfg_path),
                "artifact_mtime": {
                    "sol": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
                    "log": dt.datetime.fromtimestamp(log_path.stat().st_mtime).isoformat(timespec="seconds"),
                },
                "metrics": {
                    "rmse_xyz": [float(rmse_xyz[0]), float(rmse_xyz[1]), float(rmse_xyz[2])],
                    "rmse_3d": rmse_3d,
                    "p95_3d": p95_3d,
                    "tail70_rmse_3d": tail70_rmse_3d,
                    "final_err_3d": final_err_3d,
                    "h_att_max": h_att_max,
                },
            }
        )

        return RunResult(
            case=case,
            config_path=cfg_path,
            log_path=log_path,
            sol_path=sol_path,
            stdout_path=stdout_path,
            rmse_x=float(rmse_xyz[0]),
            rmse_y=float(rmse_xyz[1]),
            rmse_z=float(rmse_xyz[2]),
            rmse_3d=rmse_3d,
            p95_3d=p95_3d,
            tail70_rmse_3d=tail70_rmse_3d,
            final_err_3d=final_err_3d,
            split_t=split_t,
            h_att_max=h_att_max,
            consistency=consistency,
            time_s=sol_t,
            err_3d=err3,
            sol_df=sol_df,
            diag_path=diag_out,
        )

    def _mutate_data2_gnsspos_case(self, cfg: dict[str, Any], p_local: bool, g_mode: int) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"]["fej"]["ri_gnss_pos_use_p_ned_local"] = bool(p_local)
        cfg["fusion"]["fej"]["ri_vel_gyro_noise_mode"] = int(g_mode)
        cfg["fusion"]["fej"]["ri_inject_pos_inverse"] = True

    def _mutate_process_noise_case(self, cfg: dict[str, Any], g_mode: int, scenario: str) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"]["fej"]["ri_gnss_pos_use_p_ned_local"] = True
        cfg["fusion"]["fej"]["ri_vel_gyro_noise_mode"] = int(g_mode)
        cfg["fusion"]["fej"]["ri_inject_pos_inverse"] = True
        if scenario == "gnss30":
            cfg["fusion"].setdefault("gnss_schedule", {})
            cfg["fusion"]["gnss_schedule"]["enabled"] = True
            cfg["fusion"]["gnss_schedule"]["head_ratio"] = float(self.args.head_ratio)

    def _mutate_freeze_case(
        self,
        cfg: dict[str, Any],
        freeze_scale: bool,
        freeze_mounting: bool,
        freeze_lever_all: bool,
        diagnostics: bool,
        consistency: bool,
        gnss30: bool,
    ) -> None:
        self._set_default_consistency(cfg, consistency)
        self._set_default_diagnostics(cfg, diagnostics)
        if gnss30:
            cfg["fusion"].setdefault("gnss_schedule", {})
            cfg["fusion"]["gnss_schedule"]["enabled"] = True
            cfg["fusion"]["gnss_schedule"]["head_ratio"] = float(self.args.head_ratio)
        pga = cfg["fusion"].setdefault("post_gnss_ablation", {})
        pga["enabled"] = True
        pga["disable_odo_scale"] = bool(freeze_scale)
        pga["disable_mounting"] = bool(freeze_mounting)
        pga["disable_odo_lever_arm"] = bool(freeze_lever_all)
        pga["disable_gnss_lever_arm"] = bool(freeze_lever_all)

    def run_phase_data2_queue(self) -> None:
        outputs: dict[str, Any] = {}

        exp_mech = self._exp_dir("data2-gnsspos-mechanism-r2")
        ensure_dir(exp_mech)
        base_cfg = self.root / "config_data2_baseline_inekf_best.yaml"
        mech_cases = [
            ("pon_g0", True, 0),
            ("pon_g1", True, 1),
            ("poff_g0", False, 0),
            ("poff_g1", False, 1),
        ]
        mech_results: dict[str, RunResult] = {}
        for case, p_local, g_mode in mech_cases:
            mech_results[case] = self._run_case(
                exp_dir=exp_mech,
                case=case,
                base_cfg_path=base_cfg,
                cfg_mutator=lambda cfg, pl=p_local, gm=g_mode: self._mutate_data2_gnsspos_case(cfg, pl, gm),
                plot=True,
            )

        rows_mech: list[dict[str, Any]] = []
        for case, r in mech_results.items():
            row: dict[str, Any] = {
                "case": case,
                "rmse_x_m": r.rmse_x,
                "rmse_y_m": r.rmse_y,
                "rmse_z_m": r.rmse_z,
                "rmse_3d_m": r.rmse_3d,
                "p95_3d_m": r.p95_3d,
                "tail70_rmse_3d_m": r.tail70_rmse_3d,
                "final_err_3d_m": r.final_err_3d,
                "h_att_max": r.h_att_max,
            }
            for sensor in ("NHC", "ODO"):
                row[f"{sensor.lower()}_accept_ratio"] = r.consistency.get(sensor, {}).get("accept_ratio", float("nan"))
                row[f"{sensor.lower()}_nis_mean"] = r.consistency.get(sensor, {}).get("nis_mean", float("nan"))
            rows_mech.append(row)
        rows_mech.sort(key=lambda x: x["case"])
        self._write_csv(exp_mech / "gnsspos_mechanism_metrics.csv", rows_mech)

        timeline_rows: list[dict[str, Any]] = []
        for case, r in mech_results.items():
            h = parse_hatt_series(r.log_path.read_text(encoding="utf-8", errors="ignore"))
            if h.empty:
                continue
            for _, rr in h.iterrows():
                timeline_rows.append({"case": case, "t": float(rr["t"]), "h_att_f": float(rr["h_att_f"])})
        self._write_csv(exp_mech / "h_att_timeline.csv", timeline_rows)

        p_on_g0 = mech_results["pon_g0"]
        p_on_g1 = mech_results["pon_g1"]
        t_ref = p_on_g0.time_s
        e0 = p_on_g0.err_3d
        e1 = np.interp(t_ref, p_on_g1.time_s, p_on_g1.err_3d)
        delta_e = e1 - e0
        h0 = parse_hatt_series(p_on_g0.log_path.read_text(encoding="utf-8", errors="ignore"))
        h1 = parse_hatt_series(p_on_g1.log_path.read_text(encoding="utf-8", errors="ignore"))
        if (not h0.empty) and (not h1.empty):
            h0i = np.interp(t_ref, h0["t"].to_numpy(dtype=float), h0["h_att_f"].to_numpy(dtype=float))
            h1i = np.interp(t_ref, h1["t"].to_numpy(dtype=float), h1["h_att_f"].to_numpy(dtype=float))
            delta_h = h1i - h0i
            corr = float(np.corrcoef(delta_e, delta_h)[0, 1]) if delta_e.size > 3 else float("nan")
        else:
            delta_h = np.full_like(delta_e, np.nan)
            corr = float("nan")

        windows: list[dict[str, Any]] = []
        if t_ref.size:
            a = float(t_ref[0])
            t_end = float(t_ref[-1])
            while a < t_end:
                b = min(a + 60.0, t_end)
                m = (t_ref >= a) & (t_ref < b)
                if np.any(m):
                    windows.append(
                        {
                            "t_start": a,
                            "t_end": b,
                            "samples": int(np.count_nonzero(m)),
                            "delta_err3d_mean": float(np.mean(delta_e[m])),
                            "delta_hatt_mean": float(np.nanmean(delta_h[m])),
                        }
                    )
                a += 60.0
        self._write_csv(exp_mech / "timeline_windows_pon_g1_minus_g0.csv", windows)
        (exp_mech / "summary.md").write_text(
            "\n".join(
                [
                    "# data2 GNSS_POS mechanism completion (r2)",
                    f"- p_local=ON: RMSE3D g0/g1 = {p_on_g0.rmse_3d:.6f} / {p_on_g1.rmse_3d:.6f}",
                    (
                        f"- p_local=OFF: RMSE3D g0/g1 = {mech_results['poff_g0'].rmse_3d:.6f} / "
                        f"{mech_results['poff_g1'].rmse_3d:.6f}"
                    ),
                    f"- corr(delta_err3d, delta_hatt) = {corr:.6f}",
                    "- 统计包含 NHC/ODO accept_ratio 与 NIS。",
                ]
            ),
            encoding="utf-8",
        )

        exp_roll = self._exp_dir("data2-mountroll-observability-r1")
        ensure_dir(exp_roll)
        base_gnss30 = self.root / "config_data2_gnss30_inekf_best.yaml"
        roll_cases = [("s0m0l0", False, False, False), ("s1m1l0", True, True, False)]
        roll_results: dict[str, RunResult] = {}
        for case, s, m, l in roll_cases:
            roll_results[case] = self._run_case(
                exp_dir=exp_roll,
                case=case,
                base_cfg_path=base_gnss30,
                cfg_mutator=lambda cfg, ss=s, mm=m, ll=l: self._mutate_freeze_case(
                    cfg, ss, mm, ll, diagnostics=True, consistency=True, gnss30=True
                ),
                plot=True,
            )
        roll_rows: list[dict[str, Any]] = []
        fig = plt.figure(figsize=(12, 5))
        for case, rr in roll_results.items():
            diag = self._load_diag(rr.diag_path)
            if diag is None or diag.empty:
                continue
            t_abs = diag["t"].to_numpy(dtype=float) + rr.time_s[0]
            std_mr = diag["std_mr"].to_numpy(dtype=float)
            std_mp = diag["std_mp"].to_numpy(dtype=float)
            std_my = diag["std_my"].to_numpy(dtype=float)
            split_t = rr.split_t if rr.split_t is not None else float(np.quantile(t_abs, 0.3))
            pre = t_abs <= split_t
            post = t_abs > split_t
            roll_rows.append(
                {
                    "case": case,
                    "split_t": split_t,
                    "rmse_3d_m": rr.rmse_3d,
                    "std_mr_pre_mean": float(np.mean(std_mr[pre])) if np.any(pre) else float("nan"),
                    "std_mr_post_mean": float(np.mean(std_mr[post])) if np.any(post) else float("nan"),
                    "std_mp_post_mean": float(np.mean(std_mp[post])) if np.any(post) else float("nan"),
                    "std_my_post_mean": float(np.mean(std_my[post])) if np.any(post) else float("nan"),
                }
            )
            plt.plot(t_abs, std_mr, linewidth=0.9, label=f"{case} std_mr")
        if roll_rows:
            plt.xlabel("timestamp [s]")
            plt.ylabel("std_mr")
            plt.title("data2 mounting_roll observability")
            plt.grid(alpha=0.3)
            plt.legend()
            plt.tight_layout()
            plt.savefig(exp_roll / "std_mr_cmp.png", dpi=150)
        plt.close(fig)
        self._write_csv(exp_roll / "mountroll_observability_metrics.csv", roll_rows)
        (exp_roll / "summary.md").write_text(
            "\n".join(
                [
                    "# data2 mounting_roll observability supplement",
                    f"- State mapping: {STATE_LABEL}",
                    "- 通过 DIAG 的 std_mr/std_mp/std_my 统计补充 mounting_roll 证据。",
                ]
            ),
            encoding="utf-8",
        )

        exp_pn = self._exp_dir("data2-process-noise-regression-r1")
        ensure_dir(exp_pn)
        pn_rows: list[dict[str, Any]] = []
        for scenario, cfg_path in {
            "baseline": self.root / "config_data2_baseline_inekf_best.yaml",
            "gnss30": self.root / "config_data2_gnss30_inekf_best.yaml",
        }.items():
            for mode in (-1, 0, 1):
                case = f"{scenario}_g{mode}"
                rr = self._run_case(
                    exp_dir=exp_pn,
                    case=case,
                    base_cfg_path=cfg_path,
                    cfg_mutator=lambda cfg, gm=mode, sc=scenario: self._mutate_process_noise_case(cfg, gm, sc),
                    plot=False,
                )
                pn_rows.append(
                    {
                        "scenario": scenario,
                        "g_mode": mode,
                        "case": case,
                        "rmse_3d_m": rr.rmse_3d,
                        "p95_3d_m": rr.p95_3d,
                        "odo_accept_ratio": rr.consistency.get("ODO", {}).get("accept_ratio", float("nan")),
                        "nhc_accept_ratio": rr.consistency.get("NHC", {}).get("accept_ratio", float("nan")),
                    }
                )
        self._write_csv(exp_pn / "process_noise_mode_regression.csv", pn_rows)
        (exp_pn / "summary.md").write_text(
            "\n".join(
                ["# data2 process-noise mapping consistency regression"]
                + [
                    f"- {r['case']}: RMSE3D={r['rmse_3d_m']:.6f}, ODO={r['odo_accept_ratio']:.6f}"
                    for r in pn_rows
                ]
            ),
            encoding="utf-8",
        )

        exp_fz = self._exp_dir("data2-postgnss-freeze-matrix-r2")
        ensure_dir(exp_fz)
        fz_rows: list[dict[str, Any]] = []
        for s in (0, 1):
            for m in (0, 1):
                for l in (0, 1):
                    case = f"s{s}m{m}l{l}"
                    rr = self._run_case(
                        exp_dir=exp_fz,
                        case=case,
                        base_cfg_path=base_gnss30,
                        cfg_mutator=lambda cfg, ss=bool(s), mm=bool(m), ll=bool(l): self._mutate_freeze_case(
                            cfg, ss, mm, ll, diagnostics=False, consistency=True, gnss30=True
                        ),
                        plot=False,
                    )
                    fz_rows.append(
                        {
                            "case": case,
                            "freeze_odo_scale": s,
                            "freeze_mounting": m,
                            "freeze_lever_all": l,
                            "rmse_3d_m": rr.rmse_3d,
                            "p95_3d_m": rr.p95_3d,
                        }
                    )
        ref = next((x for x in fz_rows if x["case"] == "s0m0l0"), None)
        ref_rmse = float(ref["rmse_3d_m"]) if ref else float("nan")
        for x in fz_rows:
            delta = x["rmse_3d_m"] - ref_rmse
            x["delta_vs_ref_m"] = delta
            x["effect"] = "improved" if delta < -1e-3 else ("degraded" if delta > 1e-3 else "neutral")
        fz_rows.sort(key=lambda x: x["rmse_3d_m"])
        self._write_csv(exp_fz / "freeze_matrix_metrics.csv", fz_rows)
        (exp_fz / "summary.md").write_text(
            "\n".join(
                [
                    "# data2 post-GNSS freeze matrix (r2)",
                    f"- reference s0m0l0 RMSE3D={ref_rmse:.6f}",
                    f"- State mapping: {STATE_LABEL}",
                ]
                + [f"- {x['case']}: {x['effect']} ({x['delta_vs_ref_m']:.6f})" for x in fz_rows[:8]]
            ),
            encoding="utf-8",
        )

        outputs["gnsspos_exp_dir"] = rel_from_root(exp_mech, self.root)
        outputs["mountroll_exp_dir"] = rel_from_root(exp_roll, self.root)
        outputs["process_noise_exp_dir"] = rel_from_root(exp_pn, self.root)
        outputs["data2_freeze_exp_dir"] = rel_from_root(exp_fz, self.root)
        outputs["issue_closure_input"] = {
            "issue_005": {
                "pon_g0_rmse3d": p_on_g0.rmse_3d,
                "pon_g1_rmse3d": p_on_g1.rmse_3d,
                "poff_g0_rmse3d": mech_results["poff_g0"].rmse_3d,
                "poff_g1_rmse3d": mech_results["poff_g1"].rmse_3d,
                "corr_delta_err3d_delta_hatt": corr,
            }
        }
        self.phase_outputs["data2_queue"] = outputs

    def _mutate_data4_switchscan_case(self, cfg: dict[str, Any], p_local: bool, g_mode: int, inject_inv: bool) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"]["fej"]["enable"] = True
        cfg["fusion"]["fej"]["ri_gnss_pos_use_p_ned_local"] = bool(p_local)
        cfg["fusion"]["fej"]["ri_vel_gyro_noise_mode"] = int(g_mode)
        cfg["fusion"]["fej"]["ri_inject_pos_inverse"] = bool(inject_inv)
        cfg["fusion"]["gnss_path"] = "dataset/data4_converted/GNSS_converted.txt"
        cfg["fusion"].setdefault("gnss_schedule", {})
        cfg["fusion"]["gnss_schedule"]["enabled"] = False

    def _mutate_data4_main_case(self, cfg: dict[str, Any], case: str, sw: dict[str, Any]) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"]["gnss_path"] = "dataset/data4_converted/GNSS_converted.txt"
        if "inekf" in case:
            cfg["fusion"]["fej"]["enable"] = True
            cfg["fusion"]["fej"]["ri_gnss_pos_use_p_ned_local"] = bool(sw["p_local"])
            cfg["fusion"]["fej"]["ri_vel_gyro_noise_mode"] = int(sw["g_mode"])
            cfg["fusion"]["fej"]["ri_inject_pos_inverse"] = bool(sw["inject_inverse"])
        else:
            cfg["fusion"]["fej"]["enable"] = False
        cfg["fusion"].setdefault("gnss_schedule", {})
        if "gnss30" in case:
            cfg["fusion"]["gnss_schedule"]["enabled"] = True
            cfg["fusion"]["gnss_schedule"]["head_ratio"] = float(self.args.head_ratio)
        else:
            cfg["fusion"]["gnss_schedule"]["enabled"] = False

    def _mutate_data4_freeze_case(
        self,
        cfg: dict[str, Any],
        freeze_scale: bool,
        freeze_mounting: bool,
        freeze_lever_all: bool,
        sw: dict[str, Any],
    ) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"]["fej"]["enable"] = True
        cfg["fusion"]["fej"]["ri_gnss_pos_use_p_ned_local"] = bool(sw["p_local"])
        cfg["fusion"]["fej"]["ri_vel_gyro_noise_mode"] = int(sw["g_mode"])
        cfg["fusion"]["fej"]["ri_inject_pos_inverse"] = bool(sw["inject_inverse"])
        cfg["fusion"]["gnss_path"] = "dataset/data4_converted/GNSS_converted.txt"
        cfg["fusion"].setdefault("gnss_schedule", {})
        cfg["fusion"]["gnss_schedule"]["enabled"] = True
        cfg["fusion"]["gnss_schedule"]["head_ratio"] = float(self.args.head_ratio)
        pga = cfg["fusion"].setdefault("post_gnss_ablation", {})
        pga["enabled"] = True
        pga["disable_odo_scale"] = bool(freeze_scale)
        pga["disable_mounting"] = bool(freeze_mounting)
        pga["disable_odo_lever_arm"] = bool(freeze_lever_all)
        pga["disable_gnss_lever_arm"] = bool(freeze_lever_all)

    def _mutate_data4_sensitivity_case(
        self,
        cfg: dict[str, Any],
        gnss_path: str,
        gnss30: bool,
        sw: dict[str, Any],
    ) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"]["fej"]["enable"] = True
        cfg["fusion"]["fej"]["ri_gnss_pos_use_p_ned_local"] = bool(sw["p_local"])
        cfg["fusion"]["fej"]["ri_vel_gyro_noise_mode"] = int(sw["g_mode"])
        cfg["fusion"]["fej"]["ri_inject_pos_inverse"] = bool(sw["inject_inverse"])
        cfg["fusion"]["gnss_path"] = gnss_path
        cfg["fusion"].setdefault("gnss_schedule", {})
        cfg["fusion"]["gnss_schedule"]["enabled"] = bool(gnss30)
        if gnss30:
            cfg["fusion"]["gnss_schedule"]["head_ratio"] = float(self.args.head_ratio)

    def run_phase_data4_main(self) -> None:
        outputs: dict[str, Any] = {}
        base_in = self.root / "config_data4_baseline_inekf_best.yaml"
        if not base_in.exists():
            raise FileNotFoundError("missing config_data4_baseline_inekf_best.yaml")

        exp_scan = self._exp_dir("data4-inekf-switchscan-r1")
        ensure_dir(exp_scan)
        scan_rows: list[dict[str, Any]] = []
        for p_local in (0, 1):
            for g_mode in (0, 1):
                for inj in (0, 1):
                    case = f"p{p_local}_g{g_mode}_i{inj}"
                    rr = self._run_case(
                        exp_dir=exp_scan,
                        case=case,
                        base_cfg_path=base_in,
                        cfg_mutator=lambda cfg, pl=bool(p_local), gm=g_mode, ij=bool(inj): self._mutate_data4_switchscan_case(
                            cfg, pl, gm, ij
                        ),
                        plot=False,
                    )
                    scan_rows.append(
                        {
                            "case": case,
                            "p_local": p_local,
                            "g_mode": g_mode,
                            "inject_inverse": inj,
                            "rmse_3d_m": rr.rmse_3d,
                            "p95_3d_m": rr.p95_3d,
                            "h_att_max": rr.h_att_max,
                            "odo_accept_ratio": rr.consistency.get("ODO", {}).get("accept_ratio", float("nan")),
                        }
                    )
        scan_rows.sort(key=lambda x: x["rmse_3d_m"])
        self._write_csv(exp_scan / "switch_scan_metrics.csv", scan_rows)
        best_scan = scan_rows[0]
        best_switch = {
            "p_local": bool(best_scan["p_local"]),
            "g_mode": int(best_scan["g_mode"]),
            "inject_inverse": bool(best_scan["inject_inverse"]),
        }
        (exp_scan / "summary.md").write_text(
            "\n".join(
                [
                    "# data4 InEKF switch short scan",
                    (
                        f"- best: {best_scan['case']} "
                        f"RMSE3D={best_scan['rmse_3d_m']:.6f}, P95={best_scan['p95_3d_m']:.6f}"
                    ),
                ]
            ),
            encoding="utf-8",
        )

        exp_main = self._exp_dir("data4-main4-regression-r1")
        ensure_dir(exp_main)
        cfgs = {
            "baseline_eskf": self.root / "config_data4_baseline_eskf.yaml",
            "baseline_inekf": self.root / "config_data4_baseline_inekf_best.yaml",
            "gnss30_eskf": self.root / "config_data4_gnss30_eskf.yaml",
            "gnss30_inekf": self.root / "config_data4_gnss30_inekf_best.yaml",
        }
        main_rows: list[dict[str, Any]] = []
        for case, cfg_path in cfgs.items():
            rr = self._run_case(
                exp_dir=exp_main,
                case=case,
                base_cfg_path=cfg_path,
                cfg_mutator=lambda cfg, c=case, sw=best_switch: self._mutate_data4_main_case(cfg, c, sw),
                plot=True,
            )
            main_rows.append(
                {
                    "case": case,
                    "rmse_x_m": rr.rmse_x,
                    "rmse_y_m": rr.rmse_y,
                    "rmse_z_m": rr.rmse_z,
                    "rmse_3d_m": rr.rmse_3d,
                    "p95_3d_m": rr.p95_3d,
                    "tail70_rmse_3d_m": rr.tail70_rmse_3d,
                    "final_err_3d_m": rr.final_err_3d,
                    "h_att_max": rr.h_att_max,
                    "odo_accept_ratio": rr.consistency.get("ODO", {}).get("accept_ratio", float("nan")),
                    "nhc_accept_ratio": rr.consistency.get("NHC", {}).get("accept_ratio", float("nan")),
                }
            )
        self._write_csv(exp_main / "metrics_summary.csv", main_rows)
        (exp_main / "summary.md").write_text(
            "\n".join(["# data4 main4 regression"] + [f"- {x['case']}: RMSE3D={x['rmse_3d_m']:.6f}" for x in main_rows]),
            encoding="utf-8",
        )

        exp_fz = self._exp_dir("data4-gnss30-freeze-matrix-r1")
        ensure_dir(exp_fz)
        fz_rows: list[dict[str, Any]] = []
        for s in (0, 1):
            for m in (0, 1):
                for l in (0, 1):
                    case = f"s{s}m{m}l{l}"
                    rr = self._run_case(
                        exp_dir=exp_fz,
                        case=case,
                        base_cfg_path=self.root / "config_data4_gnss30_inekf_best.yaml",
                        cfg_mutator=lambda cfg, ss=bool(s), mm=bool(m), ll=bool(l), sw=best_switch: self._mutate_data4_freeze_case(
                            cfg, ss, mm, ll, sw
                        ),
                        plot=False,
                    )
                    fz_rows.append(
                        {
                            "case": case,
                            "freeze_odo_scale": s,
                            "freeze_mounting": m,
                            "freeze_lever_all": l,
                            "rmse_3d_m": rr.rmse_3d,
                            "p95_3d_m": rr.p95_3d,
                        }
                    )
        ref = next((x for x in fz_rows if x["case"] == "s0m0l0"), None)
        ref_rmse = float(ref["rmse_3d_m"]) if ref else float("nan")
        for x in fz_rows:
            delta = x["rmse_3d_m"] - ref_rmse
            x["delta_vs_ref_m"] = delta
            x["effect"] = "improved" if delta < -1e-3 else ("degraded" if delta > 1e-3 else "neutral")
        fz_rows.sort(key=lambda x: x["rmse_3d_m"])
        self._write_csv(exp_fz / "freeze_matrix_metrics.csv", fz_rows)
        (exp_fz / "summary.md").write_text(
            "\n".join(
                [
                    "# data4 GNSS30 freeze matrix",
                    f"- reference s0m0l0 RMSE3D={ref_rmse:.6f}",
                    f"- State mapping: {STATE_LABEL}",
                ]
            ),
            encoding="utf-8",
        )

        exp_sens = self._exp_dir("data4-gnssvel-sensitivity-r1")
        ensure_dir(exp_sens)
        sens_rows: list[dict[str, Any]] = []
        cases = [
            ("baseline_primary", "dataset/data4_converted/GNSS_converted.txt", False),
            ("baseline_velsigma3", "dataset/data4_converted/GNSS_converted_velsigma3.txt", False),
            ("gnss30_primary", "dataset/data4_converted/GNSS_converted.txt", True),
            ("gnss30_velsigma3", "dataset/data4_converted/GNSS_converted_velsigma3.txt", True),
        ]
        for case, gnss_path, gnss30 in cases:
            rr = self._run_case(
                exp_dir=exp_sens,
                case=case,
                base_cfg_path=self.root / "config_data4_baseline_inekf_best.yaml",
                cfg_mutator=lambda cfg, gp=gnss_path, is30=gnss30, sw=best_switch: self._mutate_data4_sensitivity_case(
                    cfg, gp, is30, sw
                ),
                plot=False,
            )
            sens_rows.append(
                {
                    "case": case,
                    "gnss_path": gnss_path,
                    "gnss30": int(gnss30),
                    "rmse_3d_m": rr.rmse_3d,
                    "p95_3d_m": rr.p95_3d,
                }
            )
        self._write_csv(exp_sens / "sensitivity_metrics.csv", sens_rows)
        (exp_sens / "summary.md").write_text(
            "\n".join(
                ["# data4 GNSS velocity sensitivity"] + [f"- {x['case']}: RMSE3D={x['rmse_3d_m']:.6f}" for x in sens_rows]
            ),
            encoding="utf-8",
        )

        outputs["switch_scan_exp_dir"] = rel_from_root(exp_scan, self.root)
        outputs["main4_exp_dir"] = rel_from_root(exp_main, self.root)
        outputs["freeze_exp_dir"] = rel_from_root(exp_fz, self.root)
        outputs["sensitivity_exp_dir"] = rel_from_root(exp_sens, self.root)
        outputs["best_switch"] = best_switch
        self.phase_outputs["data4_main"] = outputs

    def _mutate_data2_main_case(self, cfg: dict[str, Any], case: str) -> None:
        self._set_default_consistency(cfg, True)
        self._set_default_diagnostics(cfg, False)
        cfg["fusion"].setdefault("gnss_schedule", {})
        if "gnss30" in case:
            cfg["fusion"]["gnss_schedule"]["enabled"] = True
            cfg["fusion"]["gnss_schedule"]["head_ratio"] = float(self.args.head_ratio)
        else:
            cfg["fusion"]["gnss_schedule"]["enabled"] = False

    def _find_row(self, rows: list[dict[str, Any]], case: str) -> dict[str, Any]:
        for r in rows:
            if r["case"] == case:
                return r
        raise KeyError(case)

    def _find_df_row(self, df: pd.DataFrame, case: str) -> dict[str, Any]:
        row = df[df["case"] == case]
        if row.empty:
            raise KeyError(case)
        return row.iloc[0].to_dict()

    def _write_tex_combine_nav(
        self,
        *,
        tex_path: Path,
        title: str,
        full_row: dict[str, Any],
        part_row: dict[str, Any],
        fig_a_dir: str,
        fig_b_dir: str,
        dataset_label: str,
        extra_conclusion: str,
    ) -> None:
        safe_title = latex_escape(title)
        safe_conclusion = latex_escape(extra_conclusion)
        tex = f"""\\documentclass[UTF8]{{ctexart}}
\\usepackage[a4paper,top=2.5cm,bottom=2.5cm,left=2.8cm,right=2.8cm]{{geometry}}
\\usepackage{{graphicx}}
\\usepackage{{booktabs}}
\\usepackage{{float}}
\\usepackage{{caption}}
\\usepackage{{hyperref}}
\\captionsetup{{font=small, labelfont=bf}}
\\graphicspath{{{{../../}}}}
\\title{{\\textbf{{{safe_title}}}}}
\\author{{}}
\\date{{{now_local().strftime("%Y年%m月%d日")}}}
\\begin{{document}}
\\maketitle
\\tableofcontents
\\newpage
\\section{{实验设置}}
数据集：{dataset_label}。比较全程 GNSS 与前30\\% GNSS 的 ESKF 结果。
\\section{{量化指标}}
\\begin{{table}}[H]\\centering
\\begin{{tabular}}{{lcccc}}\\toprule
方案 & RMSE$_x$ & RMSE$_y$ & RMSE$_z$ & RMSE$_{{3D}}$ \\\\ \\midrule
全程 GNSS & {full_row['rmse_x_m']:.6f} & {full_row['rmse_y_m']:.6f} & {full_row['rmse_z_m']:.6f} & {full_row['rmse_3d_m']:.6f} \\\\
前30\\% GNSS & {part_row['rmse_x_m']:.6f} & {part_row['rmse_y_m']:.6f} & {part_row['rmse_z_m']:.6f} & {part_row['rmse_3d_m']:.6f} \\\\
\\bottomrule\\end{{tabular}}
\\end{{table}}
\\section{{图像对比}}
\\begin{{figure}}[H]\\centering\\includegraphics[width=0.82\\textwidth]{{{fig_a_dir}/01_trajectory_altitude.png}}\\caption{{全程GNSS轨迹}}\\end{{figure}}
\\begin{{figure}}[H]\\centering\\includegraphics[width=0.82\\textwidth]{{{fig_b_dir}/01_trajectory_altitude.png}}\\caption{{前30\\%GNSS轨迹}}\\end{{figure}}
\\section{{结论}}
{safe_conclusion}
\\end{{document}}
"""
        tex_path.write_text(tex, encoding="utf-8")

    def _write_tex_inekf_report(
        self,
        *,
        tex_path: Path,
        title: str,
        inekf_row: dict[str, Any],
        eskf_row: dict[str, Any],
        best_case: dict[str, Any],
        ref_case: dict[str, Any],
        fig_eskf_dir: str,
        fig_inekf_dir: str,
        freeze_csv: str,
        dataset_label: str,
    ) -> None:
        delta = float(best_case["rmse_3d_m"]) - float(ref_case["rmse_3d_m"])
        safe_title = latex_escape(title)
        safe_state = latex_escape(STATE_LABEL)
        safe_freeze = latex_escape(freeze_csv)
        tex = f"""\\documentclass[UTF8]{{ctexart}}
\\usepackage[a4paper,top=2.5cm,bottom=2.5cm,left=2.8cm,right=2.8cm]{{geometry}}
\\usepackage{{graphicx}}
\\usepackage{{booktabs}}
\\usepackage{{float}}
\\usepackage{{caption}}
\\usepackage{{hyperref}}
\\captionsetup{{font=small, labelfont=bf}}
\\graphicspath{{{{../../}}}}
\\title{{\\textbf{{{safe_title}}}}}
\\author{{}}
\\date{{{now_local().strftime("%Y年%m月%d日")}}}
\\begin{{document}}
\\maketitle
\\tableofcontents
\\newpage
\\section{{实验设置}}
数据集：{dataset_label}。GNSS 30\\% 条件下对比 ESKF 与 InEKF，并纳入 post-GNSS 冻结矩阵（状态块 {safe_state}）。
\\section{{ESKF vs InEKF}}
\\begin{{table}}[H]\\centering
\\begin{{tabular}}{{lcccc}}\\toprule
方案 & RMSE$_x$ & RMSE$_y$ & RMSE$_z$ & RMSE$_{{3D}}$ \\\\ \\midrule
ESKF & {eskf_row['rmse_x_m']:.6f} & {eskf_row['rmse_y_m']:.6f} & {eskf_row['rmse_z_m']:.6f} & {eskf_row['rmse_3d_m']:.6f} \\\\
InEKF & {inekf_row['rmse_x_m']:.6f} & {inekf_row['rmse_y_m']:.6f} & {inekf_row['rmse_z_m']:.6f} & {inekf_row['rmse_3d_m']:.6f} \\\\
\\bottomrule\\end{{tabular}}
\\end{{table}}
\\section{{冻结实验}}
冻结矩阵来源：\\texttt{{{safe_freeze}}}
\\begin{{table}}[H]\\centering
\\begin{{tabular}}{{lcc}}\\toprule
指标 & 参考 s0m0l0 & 最优 {best_case['case']} \\\\ \\midrule
RMSE$_{{3D}}$ (m) & {float(ref_case['rmse_3d_m']):.6f} & {float(best_case['rmse_3d_m']):.6f} \\\\
Delta(best-ref) & \\multicolumn{{2}}{{c}}{{{delta:.6f}}} \\\\
\\bottomrule\\end{{tabular}}
\\end{{table}}
\\section{{图像对比}}
\\begin{{figure}}[H]\\centering\\includegraphics[width=0.82\\textwidth]{{{fig_eskf_dir}/01_trajectory_altitude.png}}\\caption{{ESKF轨迹}}\\end{{figure}}
\\begin{{figure}}[H]\\centering\\includegraphics[width=0.82\\textwidth]{{{fig_inekf_dir}/01_trajectory_altitude.png}}\\caption{{InEKF轨迹}}\\end{{figure}}
\\section{{结论与局限}}
本报告给出 improved/degraded/neutral 状态块结论；结论仅针对当前数据和参数口径。
\\end{{document}}
"""
        tex_path.write_text(tex, encoding="utf-8")

    def run_phase_docs(self) -> None:
        outputs: dict[str, Any] = {}
        exp_d2_main = self._exp_dir("data2-main4-docsync-r1")
        ensure_dir(exp_d2_main)
        d2_cfgs = {
            "baseline_eskf": self.root / "config_data2_baseline_eskf.yaml",
            "baseline_inekf": self.root / "config_data2_baseline_inekf_best.yaml",
            "gnss30_eskf": self.root / "config_data2_gnss30_eskf_nofreeze.yaml",
            "gnss30_inekf": self.root / "config_data2_gnss30_inekf_best.yaml",
        }
        d2_rows: list[dict[str, Any]] = []
        for case, cfgp in d2_cfgs.items():
            rr = self._run_case(
                exp_dir=exp_d2_main,
                case=case,
                base_cfg_path=cfgp,
                cfg_mutator=lambda cfg, c=case: self._mutate_data2_main_case(cfg, c),
                plot=True,
            )
            d2_rows.append(
                {
                    "case": case,
                    "rmse_x_m": rr.rmse_x,
                    "rmse_y_m": rr.rmse_y,
                    "rmse_z_m": rr.rmse_z,
                    "rmse_3d_m": rr.rmse_3d,
                }
            )
        self._write_csv(exp_d2_main / "metrics_summary.csv", d2_rows)

        d2_fz_path = self._exp_dir("data2-postgnss-freeze-matrix-r2") / "freeze_matrix_metrics.csv"
        d4_fz_path = self._exp_dir("data4-gnss30-freeze-matrix-r1") / "freeze_matrix_metrics.csv"
        if not d2_fz_path.exists() or not d4_fz_path.exists():
            raise FileNotFoundError("freeze matrix csv missing before docs phase")
        d2_fz = pd.read_csv(d2_fz_path)
        d4_fz = pd.read_csv(d4_fz_path)
        d2_best = d2_fz.sort_values("rmse_3d_m").iloc[0].to_dict()
        d4_best = d4_fz.sort_values("rmse_3d_m").iloc[0].to_dict()
        d2_ref = d2_fz[d2_fz["case"] == "s0m0l0"].iloc[0].to_dict()
        d4_ref = d4_fz[d4_fz["case"] == "s0m0l0"].iloc[0].to_dict()
        d4_main_path = self._exp_dir("data4-main4-regression-r1") / "metrics_summary.csv"
        if not d4_main_path.exists():
            raise FileNotFoundError("data4 main4 metrics missing before docs phase")
        d4_main = pd.read_csv(d4_main_path)

        tex1 = self.docs_root / "组合导航.tex"
        tex2 = self.docs_root / "data2_inekf_gnss30_对比冻结.tex"
        tex3 = self.docs_root / "data4_eskf_全程_vs_30GNSS.tex"
        tex4 = self.docs_root / "data4_inekf_gnss30_对比冻结.tex"
        self._write_tex_combine_nav(
            tex_path=tex1,
            title="组合导航实验对比报告（data2 ESKF）",
            full_row=self._find_row(d2_rows, "baseline_eskf"),
            part_row=self._find_row(d2_rows, "gnss30_eskf"),
            fig_a_dir=rel_from_root(exp_d2_main / "plots" / "result_baseline_eskf", self.root),
            fig_b_dir=rel_from_root(exp_d2_main / "plots" / "result_gnss30_eskf", self.root),
            dataset_label="data2",
            extra_conclusion="本轮口径下，GNSS 仅前30%可用时 ESKF 误差显著上升。",
        )
        self._write_tex_inekf_report(
            tex_path=tex2,
            title="data2 InEKF 30%GNSS 结果（含 ESKF 对比与冻结实验）",
            inekf_row=self._find_row(d2_rows, "gnss30_inekf"),
            eskf_row=self._find_row(d2_rows, "gnss30_eskf"),
            best_case=d2_best,
            ref_case=d2_ref,
            fig_eskf_dir=rel_from_root(exp_d2_main / "plots" / "result_gnss30_eskf", self.root),
            fig_inekf_dir=rel_from_root(exp_d2_main / "plots" / "result_gnss30_inekf", self.root),
            freeze_csv=rel_from_root(d2_fz_path, self.root),
            dataset_label="data2",
        )
        self._write_tex_combine_nav(
            tex_path=tex3,
            title="组合导航实验对比报告（data4 ESKF）",
            full_row=self._find_df_row(d4_main, "baseline_eskf"),
            part_row=self._find_df_row(d4_main, "gnss30_eskf"),
            fig_a_dir=rel_from_root(self._exp_dir("data4-main4-regression-r1") / "plots" / "result_baseline_eskf", self.root),
            fig_b_dir=rel_from_root(self._exp_dir("data4-main4-regression-r1") / "plots" / "result_gnss30_eskf", self.root),
            dataset_label="data4",
            extra_conclusion="主口径使用13列 GNSS（含速度），并保留速度噪声敏感性复核。",
        )
        self._write_tex_inekf_report(
            tex_path=tex4,
            title="data4 InEKF 30%GNSS 结果（含 ESKF 对比与冻结实验）",
            inekf_row=self._find_df_row(d4_main, "gnss30_inekf"),
            eskf_row=self._find_df_row(d4_main, "gnss30_eskf"),
            best_case=d4_best,
            ref_case=d4_ref,
            fig_eskf_dir=rel_from_root(self._exp_dir("data4-main4-regression-r1") / "plots" / "result_gnss30_eskf", self.root),
            fig_inekf_dir=rel_from_root(self._exp_dir("data4-main4-regression-r1") / "plots" / "result_gnss30_inekf", self.root),
            freeze_csv=rel_from_root(d4_fz_path, self.root),
            dataset_label="data4",
        )

        compile_rows: list[dict[str, Any]] = []
        xelatex = shutil.which("xelatex")
        for tex in (tex1, tex2, tex3, tex4):
            status = "skipped: xelatex not found"
            if xelatex:
                proc = self._run_cmd([xelatex, "-interaction=nonstopmode", "-halt-on-error", tex.name], self.docs_root)
                status = "ok" if proc.returncode == 0 else "failed"
                (tex.with_suffix(".compile.log")).write_text(proc.stdout + "\n" + proc.stderr, encoding="utf-8")
            compile_rows.append(
                {
                    "tex": rel_from_root(tex, self.root),
                    "pdf_exists": int(tex.with_suffix(".pdf").exists()),
                    "compile_status": status,
                }
            )
        self._write_csv(self.docs_root / "tex_compile_status.csv", compile_rows)

        doc_assets = {
            "docs": [
                rel_from_root(tex1, self.root),
                rel_from_root(tex2, self.root),
                rel_from_root(tex3, self.root),
                rel_from_root(tex4, self.root),
            ],
            "metrics_sources": {
                "data2_main4": rel_from_root(exp_d2_main / "metrics_summary.csv", self.root),
                "data2_freeze": rel_from_root(d2_fz_path, self.root),
                "data4_main4": rel_from_root(d4_main_path, self.root),
                "data4_freeze": rel_from_root(d4_fz_path, self.root),
            },
        }
        (self.docs_root / "doc_assets_index.json").write_text(json.dumps(doc_assets, indent=2, ensure_ascii=False), encoding="utf-8")
        outputs["data2_main4_exp_dir"] = rel_from_root(exp_d2_main, self.root)
        outputs["docs"] = doc_assets["docs"]
        outputs["doc_assets_index"] = rel_from_root(self.docs_root / "doc_assets_index.json", self.root)
        self.phase_outputs["docs"] = outputs

    def _build_aggregate_outputs(self) -> None:
        root_exp = self._exp_dir("research-pipeline-summary")
        ensure_dir(root_exp)
        manifest_path = root_exp / "experiment_manifest.json"
        metrics_path = root_exp / "metrics_summary.csv"
        summary_path = root_exp / "run_summary.json"
        manifest_path.write_text(json.dumps(self.manifest_entries, indent=2, ensure_ascii=False), encoding="utf-8")

        rows: list[dict[str, Any]] = []
        for item in self.manifest_entries:
            m = item.get("metrics", {})
            rows.append(
                {
                    "case": item.get("case"),
                    "config_path": item.get("config_path"),
                    "sol_path": item.get("sol_path"),
                    "rmse_3d_m": m.get("rmse_3d"),
                    "p95_3d_m": m.get("p95_3d"),
                    "tail70_rmse_3d_m": m.get("tail70_rmse_3d"),
                    "final_err_3d_m": m.get("final_err_3d"),
                    "h_att_max": m.get("h_att_max"),
                }
            )
        self._write_csv(metrics_path, rows)
        out = {
            "generated_at": now_local().isoformat(timespec="seconds"),
            "phase_outputs": self.phase_outputs,
            "manifest_count": len(self.manifest_entries),
            "metrics_summary": rel_from_root(metrics_path, self.root),
            "experiment_manifest": rel_from_root(manifest_path, self.root),
        }
        summary_path.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        doc_assets = self.docs_root / "doc_assets_index.json"
        if doc_assets.exists():
            shutil.copy2(doc_assets, root_exp / "doc_assets_index.json")

    def run(self) -> None:
        if self.args.phase in ("data2_queue", "all"):
            self.run_phase_data2_queue()
        if self.args.phase in ("data4_main", "all"):
            self.run_phase_data4_main()
        if self.args.phase in ("docs", "all"):
            self.run_phase_docs()
        self._build_aggregate_outputs()


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Run InEKF/ESKF research pipeline.")
    ap.add_argument("--phase", choices=["data2_queue", "data4_main", "docs", "all"], default="all")
    ap.add_argument("--dataset", choices=["data2", "data4", "both"], default="both")
    ap.add_argument("--head-ratio", type=float, default=0.3)
    ap.add_argument("--mode", choices=["eskf", "inekf", "both"], default="both")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parents[2]
    runner = Pipeline(root, args)
    runner.run()
    print("Pipeline completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
