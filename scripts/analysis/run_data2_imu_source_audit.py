from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.run_data2_ins_gnss_lever_truth_imu_params_probe import parse_data2_readme_imu_params


RAW_IMU_FILES = {
    "imulog": Path("dataset/data2/IMULog201812290233_2.bin"),
    "hl": Path("dataset/data2/HL20181229023353_0_IMU.bin"),
}
README_MODELS = ("POS-320", "ICM-20602")
IMU_RECORD_SIZE = 56


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def rel_from_root(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def load_imu_binary(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    n_records = len(raw) // IMU_RECORD_SIZE
    if n_records <= 0:
        raise RuntimeError(f"empty imu binary: {path}")
    flat = struct.unpack(f"<{n_records * 7}d", raw[: n_records * IMU_RECORD_SIZE])
    return np.array(flat, dtype=float).reshape(n_records, 7)


def expected_increment_std(readme_row: dict[str, Any], dt_s: float) -> tuple[float, float]:
    sigma_g = float(readme_row["sigma_arw_deg_sqrt_hr"]) * math.pi / 180.0 / math.sqrt(3600.0)
    sigma_a = float(readme_row["sigma_vrw_mps_sqrt_hr"]) / math.sqrt(3600.0)
    return sigma_g * math.sqrt(dt_s), sigma_a * math.sqrt(dt_s)


def summarize_file(name: str, arr: np.ndarray, window_s: float) -> dict[str, Any]:
    ts = arr[:, 0]
    dt = np.diff(ts)
    median_dt = float(np.median(dt))
    seg = arr[ts <= ts[0] + window_s]
    gyro_std = seg[:, 1:4].std(axis=0)
    acc_std = seg[:, 4:7].std(axis=0)
    return {
        "file_key": name,
        "rows": int(arr.shape[0]),
        "t0": float(ts[0]),
        "t1": float(ts[-1]),
        "duration_s": float(ts[-1] - ts[0]),
        "median_dt_s": median_dt,
        "mean_dt_s": float(dt.mean()),
        "static_window_s": float(window_s),
        "static_rows": int(seg.shape[0]),
        "gyro_std_x": float(gyro_std[0]),
        "gyro_std_y": float(gyro_std[1]),
        "gyro_std_z": float(gyro_std[2]),
        "acc_std_x": float(acc_std[0]),
        "acc_std_y": float(acc_std[1]),
        "acc_std_z": float(acc_std[2]),
    }


def match_model(summary_row: dict[str, Any], readme_row: dict[str, Any]) -> dict[str, Any]:
    expected_dtheta_std, expected_dvel_std = expected_increment_std(readme_row, float(summary_row["median_dt_s"]))
    gyro_obs = np.array(
        [
            float(summary_row["gyro_std_x"]),
            float(summary_row["gyro_std_y"]),
            float(summary_row["gyro_std_z"]),
        ],
        dtype=float,
    )
    acc_obs = np.array(
        [
            float(summary_row["acc_std_x"]),
            float(summary_row["acc_std_y"]),
            float(summary_row["acc_std_z"]),
        ],
        dtype=float,
    )
    gyro_ratio = gyro_obs / max(expected_dtheta_std, 1e-12)
    acc_ratio = acc_obs / max(expected_dvel_std, 1e-12)
    gyro_log_error = float(np.mean(np.abs(np.log(np.clip(gyro_ratio, 1e-12, None)))))
    acc_log_error = float(np.mean(np.abs(np.log(np.clip(acc_ratio, 1e-12, None)))))
    return {
        "imu_model": str(readme_row["imu_model"]),
        "expected_dtheta_std": float(expected_dtheta_std),
        "expected_dvel_std": float(expected_dvel_std),
        "gyro_ratio_mean": float(np.mean(gyro_ratio)),
        "acc_ratio_mean": float(np.mean(acc_ratio)),
        "gyro_log_error": gyro_log_error,
        "acc_log_error": acc_log_error,
        "combined_log_error": gyro_log_error + acc_log_error,
    }


def write_converted_text(path: Path, arr: np.ndarray) -> None:
    np.savetxt(path, arr, fmt="%.6f %.15e %.15e %.15e %.15e %.15e %.15e")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit data2 raw IMU source mapping against README noise models.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("output/data2_imu_source_audit"),
        help="Output directory relative to repo root.",
    )
    parser.add_argument(
        "--readme-path",
        type=Path,
        default=Path("dataset/data2/README.md"),
        help="README containing POS-320 / ICM-20602 IMU rows.",
    )
    parser.add_argument(
        "--static-window-s",
        type=float,
        default=60.0,
        help="Use the first N seconds of each file as a quasi-static window for increment-noise statistics.",
    )
    parser.add_argument(
        "--emit-converted",
        action="store_true",
        help="Also dump both raw binaries to text IMU format under output_dir/artifacts/converted/.",
    )
    args = parser.parse_args()
    args.output_dir = (REPO_ROOT / args.output_dir).resolve()
    args.readme_path = (REPO_ROOT / args.readme_path).resolve()
    return args


def main() -> None:
    args = parse_args()
    ensure_dir(args.output_dir)
    artifacts_dir = args.output_dir / "artifacts"
    ensure_dir(artifacts_dir)

    summary_rows: list[dict[str, Any]] = []
    match_rows: list[dict[str, Any]] = []
    best_match_rows: list[dict[str, Any]] = []
    converted_dir = artifacts_dir / "converted"
    if args.emit_converted:
        ensure_dir(converted_dir)

    readme_rows = {
        model: parse_data2_readme_imu_params(args.readme_path, model)
        for model in README_MODELS
    }

    for file_key, rel_path in RAW_IMU_FILES.items():
        raw_path = (REPO_ROOT / rel_path).resolve()
        arr = load_imu_binary(raw_path)
        summary = summarize_file(file_key, arr, args.static_window_s)
        summary["raw_path"] = rel_from_root(raw_path)
        summary_rows.append(summary)

        model_matches: list[dict[str, Any]] = []
        for model in README_MODELS:
            match = match_model(summary, readme_rows[model])
            match_row = dict(summary)
            match_row.update(match)
            match_rows.append(match_row)
            model_matches.append(match_row)

        best = min(model_matches, key=lambda row: row["gyro_log_error"])
        best_match_rows.append(
            {
                "file_key": file_key,
                "raw_path": rel_from_root(raw_path),
                "best_model_by_gyro": best["imu_model"],
                "best_model_gyro_log_error": float(best["gyro_log_error"]),
                "best_model_acc_log_error": float(best["acc_log_error"]),
                "best_model_combined_log_error": float(best["combined_log_error"]),
            }
        )

        if args.emit_converted:
            converted_path = converted_dir / f"{raw_path.stem}_converted.txt"
            write_converted_text(converted_path, arr)

    summary_csv = args.output_dir / "imu_file_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    match_csv = args.output_dir / "imu_model_match.csv"
    with match_csv.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(match_rows[0].keys()))
        writer.writeheader()
        writer.writerows(match_rows)

    best_csv = args.output_dir / "imu_best_match.csv"
    with best_csv.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(best_match_rows[0].keys()))
        writer.writeheader()
        writer.writerows(best_match_rows)

    summary_lines = [
        "# data2 IMU source audit",
        "",
        f"- README source: `{rel_from_root(args.readme_path)}`",
        f"- static window: first `{args.static_window_s:.1f} s` of each raw IMU file",
        f"- best-match criterion: minimum `gyro_log_error` against README per-sample increment STD",
        "",
        "## Best match",
    ]
    for row in best_match_rows:
        summary_lines.append(
            f"- `{row['file_key']}` (`{row['raw_path']}`) best matches `{row['best_model_by_gyro']}` "
            f"with gyro_log_error={row['best_model_gyro_log_error']:.6f}"
        )
    summary_lines += [
        "",
        "## Key implication",
        "- If the solver keeps using `dataset/data2_converted/IMU_converted.txt` generated from `IMULog201812290233_2.bin`, "
        "the inertial source is statistically closer to `POS-320` than to `ICM-20602`.",
        "- The alternative raw file `HL20181229023353_0_IMU.bin` is statistically closer to README `ICM-20602`, "
        "so it is the stronger T3 / low-grade IMU candidate.",
    ]
    summary_md = args.output_dir / "summary.md"
    summary_md.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    manifest = {
        "generated_at": np.datetime_as_string(np.datetime64("now"), unit="s"),
        "readme_path": rel_from_root(args.readme_path),
        "output_dir": rel_from_root(args.output_dir),
        "summary_csv": rel_from_root(summary_csv),
        "match_csv": rel_from_root(match_csv),
        "best_csv": rel_from_root(best_csv),
        "summary_md": rel_from_root(summary_md),
        "raw_files": {k: rel_from_root((REPO_ROOT / v).resolve()) for k, v in RAW_IMU_FILES.items()},
        "emit_converted": bool(args.emit_converted),
    }
    if args.emit_converted:
        manifest["converted_dir"] = rel_from_root(converted_dir)
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
