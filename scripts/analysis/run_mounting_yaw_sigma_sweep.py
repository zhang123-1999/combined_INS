from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import parse_consistency_summary


EXP_ID_DEFAULT = "EXP-20260322-data2-mounting-yaw-sigma-sweep-r1"
EXE_DEFAULT = Path("build/Release/eskf_fusion.exe")
BASE_CONFIG_DEFAULT = Path(
    "output/data2_ins_gnss_odo_nhc_mounting_axis_probe_r3_axis_hard_freeze/"
    "artifacts/cases/release_mounting_yaw/config_release_mounting_yaw.yaml"
)
POS_DEFAULT = Path("dataset/data2_converted/POS_converted.txt")
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_sigma_sweep_20260322")
SIGMA_LIST_DEFAULT = [1.0e-4, 1.0e-5, 1.0e-6, 0.0]


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def rel(path: Path) -> str:
    return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()


def mtime_text(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")


def save_yaml(cfg: dict[str, Any], path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


def parse_sigma_list(text: str) -> list[float]:
    values: list[float] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    if not values:
        raise argparse.ArgumentTypeError("sigma list must not be empty")
    return values


def classify_motion(speed_m_s: np.ndarray, yaw_rate_deg_s: np.ndarray) -> np.ndarray:
    labels = np.full(speed_m_s.shape, "transition", dtype=object)
    labels[speed_m_s < 3.0] = "low_speed"
    labels[(speed_m_s >= 3.0) & (np.abs(yaw_rate_deg_s) <= 3.0)] = "straight"
    labels[(speed_m_s >= 3.0) & (yaw_rate_deg_s >= 8.0)] = "turn_pos"
    labels[(speed_m_s >= 3.0) & (yaw_rate_deg_s <= -8.0)] = "turn_neg"
    return labels


def compute_gnss_schedule_mask(state_t: np.ndarray, fusion_cfg: dict[str, Any]) -> np.ndarray:
    schedule = fusion_cfg.get("gnss_schedule", {})
    initial_on = float(schedule.get("initial_on_duration", 300.0))
    off_duration = float(schedule.get("off_duration", 100.0))
    on_duration = float(schedule.get("on_duration", 150.0))
    cycle_start = str(schedule.get("cycle_start", "off"))
    t_rel = state_t - state_t[0]
    mask = np.ones_like(state_t, dtype=bool)
    if cycle_start != "off":
        return mask
    mask = t_rel < initial_on
    remaining = ~mask
    cycle = off_duration + on_duration
    phase = np.mod(t_rel[remaining] - initial_on, cycle)
    mask[remaining] = phase >= off_duration
    return mask


def summarize_series(values: np.ndarray) -> dict[str, float]:
    return {
        "mean_deg": float(np.mean(values)),
        "std_deg": float(np.std(values)),
        "min_deg": float(np.min(values)),
        "max_deg": float(np.max(values)),
        "p05_deg": float(np.percentile(values, 5.0)),
        "p50_deg": float(np.percentile(values, 50.0)),
        "p95_deg": float(np.percentile(values, 95.0)),
        "peak_to_peak_deg": float(np.ptp(values)),
    }


def sigma_case_name(sigma: float) -> str:
    if sigma == 0.0:
        return "sigma_0"
    text = f"{sigma:.0e}".replace("+", "")
    return f"sigma_{text}"


def render_table(df: pd.DataFrame, columns: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for _, row in df.iterrows():
        vals: list[str] = []
        for col in columns:
            value = row[col]
            if isinstance(value, str):
                vals.append(value)
            elif isinstance(value, (int, np.integer)):
                vals.append(str(int(value)))
            else:
                vals.append(f"{float(value):.6f}")
        lines.append("| " + " | ".join(vals) + " |")
    return lines


def build_case_config(base_cfg: dict[str, Any], sigma_mounting_yaw: float, case_dir: Path) -> tuple[dict[str, Any], dict[str, Path]]:
    cfg = copy.deepcopy(base_cfg)
    fusion = cfg["fusion"]
    noise_cfg = fusion.setdefault("noise", {})
    constraints_cfg = fusion.setdefault("constraints", {})
    case_name = sigma_case_name(sigma_mounting_yaw)

    sol_path = case_dir / f"SOL_{case_name}.txt"
    state_series_path = case_dir / f"state_series_{case_name}.csv"
    cfg_path = case_dir / f"config_{case_name}.yaml"
    stdout_path = case_dir / f"solver_stdout_{case_name}.txt"
    diag_path = case_dir / f"DIAG_{case_name}.txt"
    metrics_json_path = case_dir / f"metrics_{case_name}.json"
    motion_csv_path = case_dir / f"motion_breakdown_{case_name}.csv"
    schedule_csv_path = case_dir / f"schedule_breakdown_{case_name}.csv"

    noise_cfg["sigma_mounting_yaw"] = float(sigma_mounting_yaw)
    constraints_cfg["enable_diagnostics"] = True
    constraints_cfg["enable_consistency_log"] = True
    fusion["output_path"] = rel(sol_path)
    fusion["state_series_output_path"] = rel(state_series_path)

    return cfg, {
        "cfg_path": cfg_path,
        "sol_path": sol_path,
        "state_series_path": state_series_path,
        "stdout_path": stdout_path,
        "diag_path": diag_path,
        "metrics_json_path": metrics_json_path,
        "motion_csv_path": motion_csv_path,
        "schedule_csv_path": schedule_csv_path,
    }


def run_solver(exe_path: Path, cfg_path: Path) -> str:
    proc = subprocess.run(
        [str(exe_path.resolve()), "--config", str(cfg_path.resolve())],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        check=False,
    )
    merged = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    if proc.returncode != 0:
        raise RuntimeError(f"solver failed for {cfg_path.name}: returncode={proc.returncode}")
    return merged


def compute_case_metrics(
    *,
    case_name: str,
    sigma_mounting_yaw: float,
    cfg: dict[str, Any],
    state_series_path: Path,
    diag_path: Path,
    pos_path: Path,
    stdout_text: str,
) -> tuple[dict[str, Any], pd.DataFrame, pd.DataFrame]:
    state_df = pd.read_csv(state_series_path)
    diag_df = pd.read_csv(diag_path, sep=r"\s+")
    pos_df = pd.read_csv(
        pos_path,
        sep=r"\s+",
        header=None,
        names=["t", "lat", "lon", "h", "vn", "ve", "vd", "roll", "pitch", "yaw"],
        engine="python",
    )

    state_t = state_df["timestamp"].to_numpy(dtype=float)
    yaw_deg = state_df["total_mounting_yaw_deg"].to_numpy(dtype=float)
    pos_t = pos_df["t"].to_numpy(dtype=float)
    yaw_truth = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    yaw_rate_deg_s = np.interp(state_t, pos_t, np.rad2deg(np.gradient(yaw_truth, pos_t)))
    speed_m_s = np.interp(
        state_t,
        pos_t,
        np.hypot(pos_df["vn"].to_numpy(dtype=float), pos_df["ve"].to_numpy(dtype=float)),
    )
    motion_label = classify_motion(speed_m_s, yaw_rate_deg_s)
    gnss_on = compute_gnss_schedule_mask(state_t, cfg["fusion"])

    overall = summarize_series(yaw_deg)
    after_200_mask = (state_t - state_t[0]) >= 200.0
    if not np.any(after_200_mask):
        after_200_mask = np.ones_like(state_t, dtype=bool)
    after_200 = summarize_series(yaw_deg[after_200_mask])

    motion_rows: list[dict[str, Any]] = []
    for label in ["straight", "turn_pos", "turn_neg", "transition", "low_speed"]:
        mask = motion_label == label
        if not np.any(mask):
            continue
        stats = summarize_series(yaw_deg[mask])
        motion_rows.append(
            {
                "case_name": case_name,
                "motion_label": label,
                "n": int(mask.sum()),
                "count_ratio": float(mask.mean()),
                "mean_yaw_deg": stats["mean_deg"],
                "std_yaw_deg": stats["std_deg"],
                "p05_deg": stats["p05_deg"],
                "p95_deg": stats["p95_deg"],
                "mean_speed_mps": float(np.mean(speed_m_s[mask])),
                "mean_yaw_rate_deg_s": float(np.mean(yaw_rate_deg_s[mask])),
            }
        )
    motion_df = pd.DataFrame(motion_rows)

    schedule_rows: list[dict[str, Any]] = []
    for label, mask in [("gnss_on", gnss_on), ("gnss_off", ~gnss_on)]:
        stats = summarize_series(yaw_deg[mask])
        schedule_rows.append(
            {
                "case_name": case_name,
                "schedule_label": label,
                "n": int(mask.sum()),
                "count_ratio": float(mask.mean()),
                "mean_yaw_deg": stats["mean_deg"],
                "std_yaw_deg": stats["std_deg"],
                "p05_deg": stats["p05_deg"],
                "p95_deg": stats["p95_deg"],
            }
        )
    schedule_df = pd.DataFrame(schedule_rows)

    std_my_deg = diag_df["std_my"].to_numpy(dtype=float) * 180.0 / math.pi
    consistency = parse_consistency_summary(stdout_text)
    odo_stats = consistency.get("ODO", {})
    nhc_stats = consistency.get("NHC", {})

    def motion_mean(label: str) -> float:
        row = motion_df.loc[motion_df["motion_label"] == label, "mean_yaw_deg"]
        return float(row.iloc[0]) if not row.empty else float("nan")

    def schedule_mean(label: str) -> float:
        row = schedule_df.loc[schedule_df["schedule_label"] == label, "mean_yaw_deg"]
        return float(row.iloc[0]) if not row.empty else float("nan")

    dataset_duration = float(state_t[-1] - state_t[0]) if state_t.size > 1 else 0.0
    metrics = {
        "case_name": case_name,
        "sigma_mounting_yaw_rad_sqrtHz": float(sigma_mounting_yaw),
        "sigma_mounting_yaw_deg_sqrtHz": float(sigma_mounting_yaw * 180.0 / math.pi),
        "rw_1sigma_100s_deg": float(sigma_mounting_yaw * math.sqrt(100.0) * 180.0 / math.pi),
        "rw_1sigma_fullrun_deg": float(sigma_mounting_yaw * math.sqrt(max(dataset_duration, 0.0)) * 180.0 / math.pi),
        "dataset_duration_s": dataset_duration,
        "overall_mean_deg": overall["mean_deg"],
        "overall_std_deg": overall["std_deg"],
        "overall_min_deg": overall["min_deg"],
        "overall_max_deg": overall["max_deg"],
        "overall_peak_to_peak_deg": overall["peak_to_peak_deg"],
        "after_200_p05_deg": after_200["p05_deg"],
        "after_200_p95_deg": after_200["p95_deg"],
        "after_200_peak_to_peak_deg": after_200["peak_to_peak_deg"],
        "final_total_mounting_yaw_deg": float(yaw_deg[-1]),
        "straight_mean_deg": motion_mean("straight"),
        "straight_std_deg": float(motion_df.loc[motion_df["motion_label"] == "straight", "std_yaw_deg"].iloc[0])
        if "straight" in motion_df["motion_label"].values
        else float("nan"),
        "turn_pos_mean_deg": motion_mean("turn_pos"),
        "turn_neg_mean_deg": motion_mean("turn_neg"),
        "turn_neg_minus_turn_pos_deg": motion_mean("turn_neg") - motion_mean("turn_pos"),
        "low_speed_std_deg": float(motion_df.loc[motion_df["motion_label"] == "low_speed", "std_yaw_deg"].iloc[0])
        if "low_speed" in motion_df["motion_label"].values
        else float("nan"),
        "gnss_on_mean_deg": schedule_mean("gnss_on"),
        "gnss_off_mean_deg": schedule_mean("gnss_off"),
        "gnss_on_minus_off_deg": schedule_mean("gnss_on") - schedule_mean("gnss_off"),
        "diag_std_my_median_deg": float(np.median(std_my_deg)),
        "diag_std_my_p95_deg": float(np.percentile(std_my_deg, 95.0)),
        "diag_std_my_final_deg": float(std_my_deg[-1]),
        "odo_accept_ratio": float(odo_stats.get("accept_ratio", float("nan"))),
        "nhc_accept_ratio": float(nhc_stats.get("accept_ratio", float("nan"))),
        "odo_nis_mean": float(odo_stats.get("nis_mean", float("nan"))),
        "nhc_nis_mean": float(nhc_stats.get("nis_mean", float("nan"))),
    }
    return metrics, motion_df, schedule_df


def write_summary(
    output_dir: Path,
    exp_id: str,
    base_config_path: Path,
    exe_path: Path,
    metrics_df: pd.DataFrame,
    case_artifacts: list[dict[str, str]],
) -> None:
    metrics_df = metrics_df.sort_values(by="sigma_mounting_yaw_rad_sqrtHz", ascending=False).reset_index(drop=True)
    base_row = metrics_df.iloc[0]
    tightest_row = metrics_df.iloc[-1]

    lines = [
        "# mounting_yaw sigma sweep",
        "",
        f"Date: `{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}`",
        "",
        "## Inputs",
        "",
        f"- experiment id: `{exp_id}`",
        f"- base config: `{rel(base_config_path)}`",
        f"- solver: `{rel(exe_path)}`",
        f"- output dir: `{rel(output_dir)}`",
        "",
        "## Sweep metrics",
        "",
    ]
    lines.extend(
        render_table(
            metrics_df[
                [
                    "case_name",
                    "sigma_mounting_yaw_rad_sqrtHz",
                    "rw_1sigma_100s_deg",
                    "overall_std_deg",
                    "overall_peak_to_peak_deg",
                    "after_200_p05_deg",
                    "after_200_p95_deg",
                    "straight_mean_deg",
                    "turn_pos_mean_deg",
                    "turn_neg_mean_deg",
                    "turn_neg_minus_turn_pos_deg",
                    "gnss_on_minus_off_deg",
                    "final_total_mounting_yaw_deg",
                    "nhc_accept_ratio",
                ]
            ],
            [
                "case_name",
                "sigma_mounting_yaw_rad_sqrtHz",
                "rw_1sigma_100s_deg",
                "overall_std_deg",
                "overall_peak_to_peak_deg",
                "after_200_p05_deg",
                "after_200_p95_deg",
                "straight_mean_deg",
                "turn_pos_mean_deg",
                "turn_neg_mean_deg",
                "turn_neg_minus_turn_pos_deg",
                "gnss_on_minus_off_deg",
                "final_total_mounting_yaw_deg",
                "nhc_accept_ratio",
            ],
        )
    )
    lines.extend(
        [
            "",
            "## Assessment",
            "",
            (
                f"- Baseline `{base_row['case_name']}` has `std={base_row['overall_std_deg']:.6f} deg`, "
                f"`peak-to-peak={base_row['overall_peak_to_peak_deg']:.6f} deg`, "
                f"`turn shift={base_row['turn_neg_minus_turn_pos_deg']:.6f} deg`."
            ),
            (
                f"- Tightest-noise `{tightest_row['case_name']}` has `std={tightest_row['overall_std_deg']:.6f} deg`, "
                f"`peak-to-peak={tightest_row['overall_peak_to_peak_deg']:.6f} deg`, "
                f"`turn shift={tightest_row['turn_neg_minus_turn_pos_deg']:.6f} deg`."
            ),
            (
                f"- `GNSS on/off` mean shift stays small across the sweep: "
                f"`min/max={metrics_df['gnss_on_minus_off_deg'].min():.6f}/{metrics_df['gnss_on_minus_off_deg'].max():.6f} deg`."
            ),
            (
                f"- `NHC accept_ratio` remains high across the sweep: "
                f"`min/max={metrics_df['nhc_accept_ratio'].min():.6f}/{metrics_df['nhc_accept_ratio'].max():.6f}`."
            ),
            "",
            "## Artifacts",
            "",
        ]
    )
    for item in case_artifacts:
        lines.append(
            (
                f"- `{item['case_name']}`: "
                f"`{item['config_path']}`, `{item['sol_path']}`, `{item['state_series_path']}`, "
                f"`{item['diag_path']}`, `{item['stdout_path']}`, `{item['metrics_json_path']}`."
            )
        )
    lines.extend(
        [
            "",
            "## Freshness",
            "",
            f"- base config mtime: `{mtime_text(base_config_path)}`",
            f"- solver mtime: `{mtime_text(exe_path)}`",
            f"- metrics csv: `{rel(output_dir / 'metrics.csv')}`",
            f"- manifest: `{rel(output_dir / 'manifest.json')}`",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep sigma_mounting_yaw using the fresh release_mounting_yaw case.")
    parser.add_argument("--exp-id", default=EXP_ID_DEFAULT)
    parser.add_argument("--exe", type=Path, default=EXE_DEFAULT)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--pos", type=Path, default=POS_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument(
        "--sigmas",
        type=parse_sigma_list,
        default=SIGMA_LIST_DEFAULT,
        help="Comma-separated sigma_mounting_yaw list in rad/sqrt(Hz).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    exe_path = (REPO_ROOT / args.exe).resolve()
    base_config_path = (REPO_ROOT / args.base_config).resolve()
    pos_path = (REPO_ROOT / args.pos).resolve()
    output_dir = (REPO_ROOT / args.output_dir).resolve()

    if not exe_path.exists():
        raise FileNotFoundError(f"missing solver executable: {exe_path}")
    if not base_config_path.exists():
        raise FileNotFoundError(f"missing base config: {base_config_path}")
    if not pos_path.exists():
        raise FileNotFoundError(f"missing POS file: {pos_path}")

    ensure_dir(output_dir)
    base_cfg = yaml.safe_load(base_config_path.read_text(encoding="utf-8"))

    case_metrics_rows: list[dict[str, Any]] = []
    case_artifacts: list[dict[str, str]] = []

    root_diag = REPO_ROOT / "DIAG.txt"
    for sigma in args.sigmas:
        case_name = sigma_case_name(float(sigma))
        case_dir = output_dir / "cases" / case_name
        ensure_dir(case_dir)
        cfg, paths = build_case_config(base_cfg, float(sigma), case_dir)
        save_yaml(cfg, paths["cfg_path"])

        if root_diag.exists():
            root_diag.unlink()
        stdout_text = run_solver(exe_path, paths["cfg_path"])
        paths["stdout_path"].write_text(stdout_text, encoding="utf-8")

        if not paths["sol_path"].exists():
            raise RuntimeError(f"missing solution output for {case_name}: {paths['sol_path']}")
        if not paths["state_series_path"].exists():
            raise RuntimeError(f"missing state series output for {case_name}: {paths['state_series_path']}")
        if not root_diag.exists():
            raise RuntimeError(f"missing DIAG.txt after case {case_name}")
        shutil.copy2(root_diag, paths["diag_path"])

        metrics, motion_df, schedule_df = compute_case_metrics(
            case_name=case_name,
            sigma_mounting_yaw=float(sigma),
            cfg=cfg,
            state_series_path=paths["state_series_path"],
            diag_path=paths["diag_path"],
            pos_path=pos_path,
            stdout_text=stdout_text,
        )
        motion_df.to_csv(paths["motion_csv_path"], index=False)
        schedule_df.to_csv(paths["schedule_csv_path"], index=False)
        paths["metrics_json_path"].write_text(json.dumps(metrics, indent=2), encoding="utf-8")

        case_metrics_rows.append(metrics)
        case_artifacts.append(
            {
                "case_name": case_name,
                "config_path": rel(paths["cfg_path"]),
                "sol_path": rel(paths["sol_path"]),
                "state_series_path": rel(paths["state_series_path"]),
                "diag_path": rel(paths["diag_path"]),
                "stdout_path": rel(paths["stdout_path"]),
                "metrics_json_path": rel(paths["metrics_json_path"]),
            }
        )

    metrics_df = pd.DataFrame(case_metrics_rows).sort_values(
        by="sigma_mounting_yaw_rad_sqrtHz", ascending=False
    ).reset_index(drop=True)
    metrics_df.to_csv(output_dir / "metrics.csv", index=False, encoding="utf-8-sig")

    manifest = {
        "exp_id": args.exp_id,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "base_config": rel(base_config_path),
        "solver_exe": rel(exe_path),
        "pos_path": rel(pos_path),
        "output_dir": rel(output_dir),
        "sigmas_rad_sqrtHz": [float(x) for x in args.sigmas],
        "metrics_csv": rel(output_dir / "metrics.csv"),
        "summary_md": rel(output_dir / "summary.md"),
        "case_artifacts": case_artifacts,
    }

    write_summary(output_dir, args.exp_id, base_config_path, exe_path, metrics_df, case_artifacts)
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    print(rel(output_dir))


if __name__ == "__main__":
    main()
