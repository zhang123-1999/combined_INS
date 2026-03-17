#!/usr/bin/env python3
"""Sweep ODO/NHC min-update-interval settings and summarize navigation impact."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import re
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


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def rel_from_root(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_yaml(cfg: dict[str, Any], path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


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
        return df["timestamp"].to_numpy(dtype=float), df[["x", "y", "z"]].to_numpy(dtype=float)
    raw = pd.read_csv(path, sep=r"\s+", header=None)
    if raw.shape[1] < 4:
        raise RuntimeError(f"truth file columns < 4: {path}")
    t = raw.iloc[:, 0].to_numpy(dtype=float)
    xyz = raw.iloc[:, 1:4].to_numpy(dtype=float)
    if np.max(np.abs(xyz[:, :2])) < 200.0:
        xyz = llh_deg_to_ecef(xyz[:, 0], xyz[:, 1], xyz[:, 2])
    return t, xyz


def load_sol(path: Path) -> tuple[np.ndarray, np.ndarray]:
    df = pd.read_csv(path, sep=r"\s+", comment="#")
    if {"timestamp", "fused_x", "fused_y", "fused_z"}.issubset(df.columns):
        return (
            df["timestamp"].to_numpy(dtype=float),
            df[["fused_x", "fused_y", "fused_z"]].to_numpy(dtype=float),
        )
    raw = pd.read_csv(path, sep=r"\s+", header=None)
    if raw.shape[1] < 4:
        raise RuntimeError(f"sol file columns < 4: {path}")
    return raw.iloc[:, 0].to_numpy(dtype=float), raw.iloc[:, 1:4].to_numpy(dtype=float)


def interp_truth(sol_t: np.ndarray, truth_t: np.ndarray, truth_xyz: np.ndarray) -> np.ndarray:
    out = np.zeros((sol_t.size, 3), dtype=float)
    for i in range(3):
        out[:, i] = np.interp(sol_t, truth_t, truth_xyz[:, i])
    return out


def parse_consistency_summary(text: str) -> dict[str, dict[str, float]]:
    pat = re.compile(
        r"\[Consistency\]\s+(\w+)\s+seen=(\d+)\s+accepted=(\d+)\s+accept_ratio=([-+0-9eE\.]+)\s+"
        r"reject_nis=(\d+)\s+reject_numeric=(\d+)\s+nis_mean=([-+0-9eE\.]+)\s+nis_max=([-+0-9eE\.]+)\s+"
        r"robust_w_mean=([-+0-9eE\.]+)\s+noise_scale_mean=([-+0-9eE\.]+)"
    )
    out: dict[str, dict[str, float]] = {}
    for m in pat.finditer(text):
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


def parse_split_t(text: str) -> float | None:
    m = re.search(r"\[GNSS\] schedule ON: .* split_t=([-+0-9eE\.]+)", text)
    return float(m.group(1)) if m else None


def hz_label(interval_s: float) -> str:
    if interval_s <= 0.0:
        return "raw"
    hz = 1.0 / interval_s
    if abs(hz - round(hz)) < 1e-9:
        return f"{int(round(hz))}hz"
    return f"{hz:.3f}hz"


def parse_interval_list(text: str) -> list[float]:
    vals: list[float] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        vals.append(float(token))
    if not vals:
        raise argparse.ArgumentTypeError("interval list must not be empty")
    return vals


def parse_optional_interval_list(text: str | None) -> list[float]:
    if text is None:
        return []
    return parse_interval_list(text)


@dataclass(frozen=True)
class Scenario:
    name: str
    base_config: Path


class SweepRunner:
    def __init__(self, root: Path, exe: Path, outdir: Path) -> None:
        self.root = root
        self.exe = exe
        self.outdir = outdir
        ensure_dir(self.outdir)

    def _run_case(
        self,
        scenario: Scenario,
        group: str,
        odo_interval_s: float,
        nhc_interval_s: float,
    ) -> dict[str, Any]:
        scenario_dir = self.outdir / scenario.name
        ensure_dir(scenario_dir)

        case = f"{group}_odo_{hz_label(odo_interval_s)}_nhc_{hz_label(nhc_interval_s)}"
        cfg = load_yaml(scenario.base_config)
        fusion = cfg.setdefault("fusion", {})
        constraints = fusion.setdefault("constraints", {})
        constraints["odo_min_update_interval"] = float(odo_interval_s)
        constraints["nhc_min_update_interval"] = float(nhc_interval_s)
        constraints["enable_consistency_log"] = True
        constraints["enable_diagnostics"] = False

        sol_path = scenario_dir / f"SOL_{case}.txt"
        cfg_path = scenario_dir / f"cfg_{case}.yaml"
        log_path = scenario_dir / f"{case}.log"
        stdout_path = scenario_dir / f"{case}.stdout.txt"
        fusion["output_path"] = str(sol_path.resolve())
        save_yaml(cfg, cfg_path)

        cmd = [str(self.exe), "--config", str(cfg_path.resolve())]
        proc = subprocess.run(
            cmd,
            cwd=str(self.root),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
            check=False,
        )
        merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
        stdout_path.write_text(merged, encoding="utf-8")
        log_path.write_text(merged, encoding="utf-8")
        if proc.returncode != 0:
            raise RuntimeError(f"run failed for {case}: returncode={proc.returncode}")
        if not sol_path.exists():
            raise FileNotFoundError(f"missing solution file: {sol_path}")

        truth_path = (self.root / fusion["pos_path"]).resolve()
        truth_t, truth_xyz = load_truth_ecef(truth_path)
        sol_t, sol_xyz = load_sol(sol_path)
        truth_interp = interp_truth(sol_t, truth_t, truth_xyz)
        err = sol_xyz - truth_interp
        err3 = np.linalg.norm(err, axis=1)
        rmse_xyz = np.sqrt(np.mean(err * err, axis=0))
        nav_duration_s = float(sol_t[-1] - sol_t[0]) if sol_t.size > 1 else 0.0
        tail_start = int(max(0, math.floor(err3.size * 0.3)))
        tail = err3[tail_start:]
        consistency = parse_consistency_summary(merged)
        split_t = parse_split_t(merged)

        row: dict[str, Any] = {
            "scenario": scenario.name,
            "group": group,
            "case": case,
            "base_config": rel_from_root(scenario.base_config, self.root),
            "config_path": rel_from_root(cfg_path, self.root),
            "sol_path": rel_from_root(sol_path, self.root),
            "log_path": rel_from_root(log_path, self.root),
            "stdout_path": rel_from_root(stdout_path, self.root),
            "odo_min_update_interval_s": float(odo_interval_s),
            "nhc_min_update_interval_s": float(nhc_interval_s),
            "odo_label": hz_label(odo_interval_s),
            "nhc_label": hz_label(nhc_interval_s),
            "odo_nominal_hz": float("nan") if odo_interval_s <= 0.0 else 1.0 / odo_interval_s,
            "nhc_nominal_hz": float("nan") if nhc_interval_s <= 0.0 else 1.0 / nhc_interval_s,
            "rmse_x_m": float(rmse_xyz[0]),
            "rmse_y_m": float(rmse_xyz[1]),
            "rmse_z_m": float(rmse_xyz[2]),
            "rmse_3d_m": float(np.sqrt(np.mean(err3 * err3))),
            "p95_3d_m": float(np.percentile(err3, 95)),
            "tail70_rmse_3d_m": float(np.sqrt(np.mean(tail * tail))) if tail.size else float("nan"),
            "final_err_3d_m": float(err3[-1]) if err3.size else float("nan"),
            "nav_duration_s": nav_duration_s,
            "split_t": split_t if split_t is not None else float("nan"),
            "artifact_mtime_sol": dt.datetime.fromtimestamp(sol_path.stat().st_mtime).isoformat(timespec="seconds"),
            "artifact_mtime_log": dt.datetime.fromtimestamp(log_path.stat().st_mtime).isoformat(timespec="seconds"),
        }

        for sensor in ("ODO", "NHC"):
            stats = consistency.get(sensor, {})
            seen = float(stats.get("seen", float("nan")))
            accepted = float(stats.get("accepted", float("nan")))
            row[f"{sensor.lower()}_seen"] = seen
            row[f"{sensor.lower()}_accepted"] = accepted
            row[f"{sensor.lower()}_accept_ratio"] = float(stats.get("accept_ratio", float("nan")))
            row[f"{sensor.lower()}_reject_nis"] = float(stats.get("reject_nis", float("nan")))
            row[f"{sensor.lower()}_reject_numeric"] = float(stats.get("reject_numeric", float("nan")))
            row[f"{sensor.lower()}_nis_mean"] = float(stats.get("nis_mean", float("nan")))
            row[f"{sensor.lower()}_nis_max"] = float(stats.get("nis_max", float("nan")))
            row[f"{sensor.lower()}_accepted_hz"] = (
                accepted / nav_duration_s if nav_duration_s > 0.0 and math.isfinite(accepted) else float("nan")
            )
            row[f"{sensor.lower()}_seen_hz"] = (
                seen / nav_duration_s if nav_duration_s > 0.0 and math.isfinite(seen) else float("nan")
            )
        return row

    def run_fixed_odo_nhc_sweep(
        self,
        scenario: Scenario,
        *,
        fixed_odo_interval_s: float,
        nhc_intervals_s: list[float],
    ) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for nhc_interval_s in nhc_intervals_s:
            rows.append(
                self._run_case(
                    scenario,
                    "fixed_odo_nhc_sweep",
                    fixed_odo_interval_s,
                    nhc_interval_s,
                )
            )
        return rows

    def run_fixed_nhc_odo_sweep(
        self,
        scenario: Scenario,
        *,
        fixed_nhc_interval_s: float,
        odo_intervals_s: list[float],
    ) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for odo_interval_s in odo_intervals_s:
            rows.append(
                self._run_case(
                    scenario,
                    "fixed_nhc_odo_sweep",
                    odo_interval_s,
                    fixed_nhc_interval_s,
                )
            )
        return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(rows[0].keys())
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    df = pd.DataFrame(rows)
    lines: list[str] = ["# ODO/NHC update-interval sweep", ""]
    for scenario in sorted(df["scenario"].unique()):
        sdf = df[df["scenario"] == scenario]
        lines.append(f"## {scenario}")
        matched = sdf[sdf["group"] == "matched"].sort_values("odo_min_update_interval_s")
        if not matched.empty:
            raw = matched.iloc[0].to_dict()
            best = matched.sort_values("rmse_3d_m").iloc[0].to_dict()
            lines.append(
                f"- matched best: `{best['case']}` RMSE3D={best['rmse_3d_m']:.6f} m, "
                f"tail70={best['tail70_rmse_3d_m']:.6f} m"
            )
            lines.append(
                f"- matched raw ref: `{raw['case']}` RMSE3D={raw['rmse_3d_m']:.6f} m, "
                f"ODO/NHC accepted Hz={raw['odo_accepted_hz']:.3f}/{raw['nhc_accepted_hz']:.3f}"
            )
        matrix = sdf[sdf["group"] == "matrix"]
        if not matrix.empty:
            best_m = matrix.sort_values("rmse_3d_m").iloc[0].to_dict()
            worst_m = matrix.sort_values("rmse_3d_m", ascending=False).iloc[0].to_dict()
            lines.append(
                f"- matrix best: `{best_m['case']}` RMSE3D={best_m['rmse_3d_m']:.6f} m "
                f"(ODO={best_m['odo_label']}, NHC={best_m['nhc_label']})"
            )
            lines.append(
                f"- matrix worst: `{worst_m['case']}` RMSE3D={worst_m['rmse_3d_m']:.6f} m "
                f"(ODO={worst_m['odo_label']}, NHC={worst_m['nhc_label']})"
            )
        fixed_odo = sdf[sdf["group"] == "fixed_odo_nhc_sweep"].sort_values("nhc_min_update_interval_s")
        if not fixed_odo.empty:
            best_fodo = fixed_odo.sort_values("rmse_3d_m").iloc[0].to_dict()
            last_fodo = fixed_odo.iloc[-1].to_dict()
            lines.append(
                f"- fixed ODO={fixed_odo.iloc[0]['odo_label']}: best `{best_fodo['case']}` "
                f"RMSE3D={best_fodo['rmse_3d_m']:.6f} m"
            )
            lines.append(
                f"- fixed ODO trend tail: lowest-NHC `{last_fodo['case']}` "
                f"RMSE3D={last_fodo['rmse_3d_m']:.6f} m"
            )
        fixed_nhc = sdf[sdf["group"] == "fixed_nhc_odo_sweep"].sort_values("odo_min_update_interval_s")
        if not fixed_nhc.empty:
            best_fnhc = fixed_nhc.sort_values("rmse_3d_m").iloc[0].to_dict()
            last_fnhc = fixed_nhc.iloc[-1].to_dict()
            lines.append(
                f"- fixed NHC={fixed_nhc.iloc[0]['nhc_label']}: best `{best_fnhc['case']}` "
                f"RMSE3D={best_fnhc['rmse_3d_m']:.6f} m"
            )
            lines.append(
                f"- fixed NHC trend tail: lowest-ODO `{last_fnhc['case']}` "
                f"RMSE3D={last_fnhc['rmse_3d_m']:.6f} m"
            )
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_outputs(outdir: Path, rows: list[dict[str, Any]]) -> None:
    df = pd.DataFrame(rows)
    plots_dir = outdir / "plots"
    ensure_dir(plots_dir)

    for scenario in sorted(df["scenario"].unique()):
        sdf = df[df["scenario"] == scenario]
        matched = sdf[sdf["group"] == "matched"].copy()
        if not matched.empty:
            matched["x_hz"] = matched["odo_min_update_interval_s"].apply(lambda x: np.nan if x <= 0.0 else 1.0 / x)
            matched["x_plot"] = matched["x_hz"].fillna(220.0)
            fig, ax1 = plt.subplots(figsize=(7.5, 4.5))
            ax1.plot(matched["x_plot"], matched["rmse_3d_m"], marker="o", label="RMSE3D [m]")
            ax1.set_xscale("log")
            ax1.set_xlabel("Matched ODO/NHC nominal rate [Hz] (raw shown at 220)")
            ax1.set_ylabel("RMSE3D [m]")
            ax1.grid(True, ls="--", alpha=0.3)
            ax2 = ax1.twinx()
            ax2.plot(
                matched["x_plot"],
                matched["odo_accepted_hz"],
                marker="s",
                color="#d95f02",
                label="ODO accepted [Hz]",
            )
            ax2.plot(
                matched["x_plot"],
                matched["nhc_accepted_hz"],
                marker="^",
                color="#1b9e77",
                label="NHC accepted [Hz]",
            )
            ax2.set_ylabel("Accepted update rate [Hz]")
            handles1, labels1 = ax1.get_legend_handles_labels()
            handles2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(handles1 + handles2, labels1 + labels2, loc="best")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{scenario}_matched_sweep.png", dpi=180)
            plt.close(fig)

        matrix = sdf[sdf["group"] == "matrix"].copy()
        if not matrix.empty:
            pivot = matrix.pivot(index="nhc_label", columns="odo_label", values="rmse_3d_m")
            fig, ax = plt.subplots(figsize=(6.2, 4.8))
            im = ax.imshow(pivot.to_numpy(dtype=float), cmap="viridis")
            ax.set_xticks(range(len(pivot.columns)))
            ax.set_xticklabels(pivot.columns)
            ax.set_yticks(range(len(pivot.index)))
            ax.set_yticklabels(pivot.index)
            ax.set_xlabel("ODO nominal rate")
            ax.set_ylabel("NHC nominal rate")
            ax.set_title(f"{scenario} matrix RMSE3D [m]")
            for i in range(pivot.shape[0]):
                for j in range(pivot.shape[1]):
                    ax.text(j, i, f"{pivot.iloc[i, j]:.2f}", ha="center", va="center", color="white", fontsize=8)
            fig.colorbar(im, ax=ax, label="RMSE3D [m]")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{scenario}_matrix_rmse3d.png", dpi=180)
            plt.close(fig)

        fixed_odo = sdf[sdf["group"] == "fixed_odo_nhc_sweep"].copy()
        if not fixed_odo.empty:
            fixed_odo["x_hz"] = fixed_odo["nhc_min_update_interval_s"].apply(lambda x: np.nan if x <= 0.0 else 1.0 / x)
            fixed_odo["x_plot"] = fixed_odo["x_hz"].fillna(220.0)
            fig, ax1 = plt.subplots(figsize=(7.5, 4.5))
            ax1.plot(fixed_odo["x_plot"], fixed_odo["rmse_3d_m"], marker="o", label="RMSE3D [m]")
            ax1.set_xscale("log")
            ax1.set_xlabel(f"NHC nominal rate [Hz], ODO fixed at {fixed_odo.iloc[0]['odo_label']} (raw shown at 220)")
            ax1.set_ylabel("RMSE3D [m]")
            ax1.grid(True, ls="--", alpha=0.3)
            ax2 = ax1.twinx()
            ax2.plot(
                fixed_odo["x_plot"],
                fixed_odo["odo_accepted_hz"],
                marker="s",
                color="#d95f02",
                label="ODO accepted [Hz]",
            )
            ax2.plot(
                fixed_odo["x_plot"],
                fixed_odo["nhc_accepted_hz"],
                marker="^",
                color="#1b9e77",
                label="NHC accepted [Hz]",
            )
            ax2.set_ylabel("Accepted update rate [Hz]")
            handles1, labels1 = ax1.get_legend_handles_labels()
            handles2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(handles1 + handles2, labels1 + labels2, loc="best")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{scenario}_fixed_odo_nhc_sweep.png", dpi=180)
            plt.close(fig)

        fixed_nhc = sdf[sdf["group"] == "fixed_nhc_odo_sweep"].copy()
        if not fixed_nhc.empty:
            fixed_nhc["x_hz"] = fixed_nhc["odo_min_update_interval_s"].apply(lambda x: np.nan if x <= 0.0 else 1.0 / x)
            fixed_nhc["x_plot"] = fixed_nhc["x_hz"].fillna(220.0)
            fig, ax1 = plt.subplots(figsize=(7.5, 4.5))
            ax1.plot(fixed_nhc["x_plot"], fixed_nhc["rmse_3d_m"], marker="o", label="RMSE3D [m]")
            ax1.set_xscale("log")
            ax1.set_xlabel(f"ODO nominal rate [Hz], NHC fixed at {fixed_nhc.iloc[0]['nhc_label']} (raw shown at 220)")
            ax1.set_ylabel("RMSE3D [m]")
            ax1.grid(True, ls="--", alpha=0.3)
            ax2 = ax1.twinx()
            ax2.plot(
                fixed_nhc["x_plot"],
                fixed_nhc["odo_accepted_hz"],
                marker="s",
                color="#d95f02",
                label="ODO accepted [Hz]",
            )
            ax2.plot(
                fixed_nhc["x_plot"],
                fixed_nhc["nhc_accepted_hz"],
                marker="^",
                color="#1b9e77",
                label="NHC accepted [Hz]",
            )
            ax2.set_ylabel("Accepted update rate [Hz]")
            handles1, labels1 = ax1.get_legend_handles_labels()
            handles2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(handles1 + handles2, labels1 + labels2, loc="best")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{scenario}_fixed_nhc_odo_sweep.png", dpi=180)
            plt.close(fig)


def build_scenarios(root: Path, scenario_args: list[str]) -> list[Scenario]:
    scenarios: list[Scenario] = []
    for item in scenario_args:
        if "=" not in item:
            raise argparse.ArgumentTypeError(f"scenario must be name=path, got: {item}")
        name, rel_path = item.split("=", 1)
        path = (root / rel_path).resolve()
        if not path.exists():
            raise FileNotFoundError(f"missing scenario config: {path}")
        scenarios.append(Scenario(name=name.strip(), base_config=path))
    if not scenarios:
        raise argparse.ArgumentTypeError("at least one --scenario is required")
    return scenarios


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser(description="Sweep ODO/NHC update intervals.")
    ap.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="Scenario in the form name=relative/path/to/config.yaml",
    )
    ap.add_argument(
        "--matched-intervals",
        type=parse_interval_list,
        default=parse_interval_list("0.0,0.01,0.02,0.05,0.10"),
        help="Comma-separated matched ODO/NHC min-update intervals in seconds.",
    )
    ap.add_argument(
        "--matrix-intervals",
        type=parse_interval_list,
        default=parse_interval_list("0.0,0.02,0.05"),
        help="Comma-separated ODO/NHC matrix min-update intervals in seconds.",
    )
    ap.add_argument(
        "--skip-matrix",
        action="store_true",
        help="Only run matched sweep, skip ODO/NHC split matrix.",
    )
    ap.add_argument(
        "--fixed-odo-interval",
        type=float,
        default=None,
        help="If set, keep ODO at this interval and sweep NHC using --nhc-sweep-intervals.",
    )
    ap.add_argument(
        "--nhc-sweep-intervals",
        type=parse_optional_interval_list,
        default=[],
        help="Comma-separated NHC sweep intervals used with --fixed-odo-interval.",
    )
    ap.add_argument(
        "--fixed-nhc-interval",
        type=float,
        default=None,
        help="If set, keep NHC at this interval and sweep ODO using --odo-sweep-intervals.",
    )
    ap.add_argument(
        "--odo-sweep-intervals",
        type=parse_optional_interval_list,
        default=[],
        help="Comma-separated ODO sweep intervals used with --fixed-nhc-interval.",
    )
    ap.add_argument(
        "--outdir",
        type=str,
        default=f"output/review/EXP-{dt.datetime.now().strftime('%Y%m%d')}-odo-nhc-rate-sweep-r1",
        help="Output directory relative to repo root.",
    )
    ap.add_argument(
        "--exe",
        type=str,
        default="build/Release/eskf_fusion.exe",
        help="Solver executable relative to repo root.",
    )
    args = ap.parse_args()
    args.root = root
    args.outdir = (root / args.outdir).resolve()
    args.exe = (root / args.exe).resolve()
    args.scenarios = build_scenarios(root, args.scenario)
    return args


def main() -> int:
    args = parse_args()
    if not args.exe.exists():
        raise FileNotFoundError(f"missing executable: {args.exe}")

    runner = SweepRunner(args.root, args.exe, args.outdir)
    rows: list[dict[str, Any]] = []

    for scenario in args.scenarios:
        for interval in args.matched_intervals:
            rows.append(runner._run_case(scenario, "matched", interval, interval))
        if not args.skip_matrix:
            for odo_interval in args.matrix_intervals:
                for nhc_interval in args.matrix_intervals:
                    rows.append(runner._run_case(scenario, "matrix", odo_interval, nhc_interval))
        if args.fixed_odo_interval is not None and args.nhc_sweep_intervals:
            rows.extend(
                runner.run_fixed_odo_nhc_sweep(
                    scenario,
                    fixed_odo_interval_s=float(args.fixed_odo_interval),
                    nhc_intervals_s=args.nhc_sweep_intervals,
                )
            )
        if args.fixed_nhc_interval is not None and args.odo_sweep_intervals:
            rows.extend(
                runner.run_fixed_nhc_odo_sweep(
                    scenario,
                    fixed_nhc_interval_s=float(args.fixed_nhc_interval),
                    odo_intervals_s=args.odo_sweep_intervals,
                )
            )

    rows.sort(key=lambda x: (x["scenario"], x["group"], x["odo_min_update_interval_s"], x["nhc_min_update_interval_s"]))
    write_csv(args.outdir / "metrics.csv", rows)
    write_summary(args.outdir / "summary.md", rows)
    plot_outputs(args.outdir, rows)
    manifest = {
        "generated_at": dt.datetime.now().isoformat(timespec="seconds"),
        "scenarios": [
            {"name": s.name, "base_config": rel_from_root(s.base_config, args.root)} for s in args.scenarios
        ],
        "matched_intervals_s": args.matched_intervals,
        "matrix_intervals_s": args.matrix_intervals,
        "skip_matrix": bool(args.skip_matrix),
        "metrics_csv": rel_from_root(args.outdir / "metrics.csv", args.root),
        "summary_md": rel_from_root(args.outdir / "summary.md", args.root),
        "plots_dir": rel_from_root(args.outdir / "plots", args.root),
    }
    (args.outdir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[done] wrote {rel_from_root(args.outdir, args.root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
