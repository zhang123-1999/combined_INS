from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.analysis.odo_nhc_update_sweep import ensure_dir, load_yaml, rel_from_root  # noqa: E402
from scripts.analysis.run_mounting_yaw_first_predict_bgz_attz_source import (  # noqa: E402
    MECHANISM_DEFAULT,
    find_exact_time_index,
    find_row_at_or_after,
    parse_vec,
    read_imu,
    read_pos,
)
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import mtime_text  # noqa: E402


BASE_CONFIG_DEFAULT = Path(
    "output/debug_mounting_yaw_first_update_geometry_r1_20260323/"
    "cases/baseline/config_baseline.yaml"
)
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_p0_seed_sweep_r1_20260323")
SCALES_DEFAULT = (0.1, 1.0, 10.0)


def euler_to_rotation(roll_rad: float, pitch_rad: float, yaw_rad: float) -> list[list[float]]:
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def resolve_sigma_vec(cfg_noise: dict[str, Any], vec_key: str, scalar_key: str) -> list[float]:
    values = cfg_noise.get(vec_key)
    if isinstance(values, list) and len(values) == 3 and all(float(v) >= 0.0 for v in values):
        return [float(v) for v in values]
    scalar = float(cfg_noise[scalar_key])
    return [scalar, scalar, scalar]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sweep P0(bg_z,bg_z) and quantify first-predict covariance seeding."
    )
    parser.add_argument("--config", type=Path, default=BASE_CONFIG_DEFAULT)
    parser.add_argument("--mechanism", type=Path, default=MECHANISM_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
    parser.add_argument("--scales", type=float, nargs="+", default=list(SCALES_DEFAULT))
    args = parser.parse_args()

    config_path = REPO_ROOT / args.config
    mechanism_path = REPO_ROOT / args.mechanism
    outdir = REPO_ROOT / args.output_dir
    ensure_dir(outdir)

    cfg = load_yaml(config_path)
    fusion_cfg = cfg["fusion"]
    init_cfg = fusion_cfg["init"]
    noise_cfg = fusion_cfg["noise"]

    mechanism_df = pd.read_csv(mechanism_path)
    if mechanism_df.empty:
        raise RuntimeError("mechanism csv is empty")
    first_row = mechanism_df.iloc[0]
    t_meas = float(first_row["t_meas"])

    imu_path = REPO_ROOT / fusion_cfg["imu_path"]
    pos_path = REPO_ROOT / fusion_cfg["pos_path"]
    imu_df = read_imu(imu_path)
    pos_df = read_pos(pos_path)

    imu_curr_idx = find_exact_time_index(imu_df, t_meas)
    if imu_curr_idx <= 0:
        raise RuntimeError("first mechanism row maps to first IMU row; missing previous IMU for predict")
    imu_prev = imu_df.iloc[imu_curr_idx - 1]
    imu_curr = imu_df.iloc[imu_curr_idx]
    dt = float(imu_curr["t"] - imu_prev["t"])

    init_truth = find_row_at_or_after(pos_df, float(fusion_cfg["starttime"]))
    roll_rad = math.radians(float(init_truth["roll_deg"]))
    pitch_rad = math.radians(float(init_truth["pitch_deg"]))
    yaw_rad = math.radians(float(init_truth["yaw_deg"]))
    c_bn = euler_to_rotation(roll_rad, pitch_rad, yaw_rad)
    c_bn_col_bgz = [c_bn[0][2], c_bn[1][2], c_bn[2][2]]
    phi_att_from_bg_col_z = [-value * dt for value in c_bn_col_bgz]

    p0_diag = [float(v) for v in init_cfg["P0_diag"]]
    p0_bgz_base = p0_diag[14]
    markov_corr_time = float(noise_cfg["markov_corr_time"])
    phi_bg_diag = 1.0
    if markov_corr_time > 0.0:
        phi_bg_diag -= dt / markov_corr_time

    bg_sigma_vec = resolve_sigma_vec(noise_cfg, "sigma_bg_vec", "sigma_bg")
    if markov_corr_time > 0.0:
        bg_w_vec = [sigma * math.sqrt(2.0 / markov_corr_time) for sigma in bg_sigma_vec]
    else:
        bg_w_vec = bg_sigma_vec
    q_bgz_bgz_cont = bg_w_vec[2] * bg_w_vec[2]
    q_bgz_att = [
        0.5 * phi_bg_diag * q_bgz_bgz_cont * phi_coeff * dt
        for phi_coeff in phi_att_from_bg_col_z
    ]
    q_bgz_bgz = 0.5 * (phi_bg_diag * q_bgz_bgz_cont * phi_bg_diag + q_bgz_bgz_cont) * dt

    h_bg_z_vec = parse_vec(first_row["h_bg_z_vec"])
    if len(h_bg_z_vec) == 0:
        raise RuntimeError("missing h_bg_z_vec in mechanism row")

    rows: list[dict[str, Any]] = []
    for scale in args.scales:
        p0_bgz = float(scale) * p0_bgz_base
        p1_bgz_bgz = (phi_bg_diag * phi_bg_diag) * p0_bgz + q_bgz_bgz
        delta_p_bgz_bgz = p1_bgz_bgz - p0_bgz

        phi_p0phi_bgz_att = [phi_bg_diag * p0_bgz * value for value in phi_att_from_bg_col_z]
        p1_bgz_att = [phi_term + q_term for phi_term, q_term in zip(phi_p0phi_bgz_att, q_bgz_att)]
        delta_p_bgz_att = p1_bgz_att[2]
        delta_p_bgz_mount_yaw = 0.0

        projected_first_nhc_num_bg_z_from_bg = float(p1_bgz_bgz * h_bg_z_vec[0])
        row = {
            "scale": float(scale),
            "p0_bgz_bgz": p0_bgz,
            "p1_bgz_bgz": p1_bgz_bgz,
            "delta_p_bgz_bgz": delta_p_bgz_bgz,
            "phi_p0phi_bgz_att_z": float(phi_p0phi_bgz_att[2]),
            "q_bgz_att_z": float(q_bgz_att[2]),
            "p1_bgz_att_z": float(p1_bgz_att[2]),
            "delta_p_bgz_att_z": float(delta_p_bgz_att),
            "delta_p_bgz_mount_yaw": float(delta_p_bgz_mount_yaw),
            "projected_first_nhc_num_bg_z_from_bg_component_0": projected_first_nhc_num_bg_z_from_bg,
        }
        rows.append(row)

    df = pd.DataFrame(rows)
    metrics_path = outdir / "metrics.csv"
    df.to_csv(metrics_path, index=False)

    payload = {
        "config_path": rel_from_root(config_path, REPO_ROOT),
        "mechanism_path": rel_from_root(mechanism_path, REPO_ROOT),
        "imu_path": rel_from_root(imu_path, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "config_mtime": mtime_text(config_path),
        "mechanism_mtime": mtime_text(mechanism_path),
        "imu_mtime": mtime_text(imu_path),
        "pos_mtime": mtime_text(pos_path),
        "dt": dt,
        "phi_bg_diag": phi_bg_diag,
        "phi_att_from_bg_col_z": phi_att_from_bg_col_z,
        "q_bgz_att": q_bgz_att,
        "q_bgz_bgz": q_bgz_bgz,
        "results": rows,
    }
    json_path = outdir / "metrics.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    baseline = df.loc[df["scale"] == 1.0].iloc[0]
    lines = [
        "# P0(bg_z,bg_z) Seed Sweep",
        "",
        "目标：只分析首个 predict，把 `P0(bg_z,bg_z)` 分别放大为 `0.1x / 1x / 10x`，",
        "检查 `ΔP(bg_z,att_z)` 是否近似线性放大，以及 `ΔP(bg_z,mounting_yaw)` 是否依然近零。",
        "",
        "## Inputs",
        f"- config: `{rel_from_root(config_path, REPO_ROOT)}`",
        f"- mechanism: `{rel_from_root(mechanism_path, REPO_ROOT)}`",
        f"- dt: `{dt:.9f}` s",
        f"- baseline `P0(bg_z,bg_z)`: `{p0_bgz_base:.12g}`",
        "",
        "## Results",
        "",
        "| scale | P0(bg_z,bg_z) | P1(bg_z,bg_z) | ΔP(bg_z,att_z) | ΔP(bg_z,mounting_yaw) | PhiP0Phi^T(bg_z,att_z) | Q(bg_z,att_z) |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for _, row in df.iterrows():
        lines.append(
            "| "
            + " | ".join(
                [
                    f"{float(row['scale']):.1f}",
                    f"{float(row['p0_bgz_bgz']):.12g}",
                    f"{float(row['p1_bgz_bgz']):.12g}",
                    f"{float(row['delta_p_bgz_att_z']):.12g}",
                    f"{float(row['delta_p_bgz_mount_yaw']):.12g}",
                    f"{float(row['phi_p0phi_bgz_att_z']):.12g}",
                    f"{float(row['q_bgz_att_z']):.12g}",
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Key Reading",
            (
                f"- baseline `ΔP(bg_z,att_z) = {float(baseline['delta_p_bgz_att_z']):.12g}`，"
                f"`PhiP0Phi^T` 项为 `{float(baseline['phi_p0phi_bgz_att_z']):.12g}`，"
                f"`Q` 项仅为 `{float(baseline['q_bgz_att_z']):.12g}`。"
            ),
            "- 若 `scale` 与 `ΔP(bg_z,att_z)` 近似线性，则说明首拍污染主要来自 `P0` 经 `Phi` 搬运，而不是过程噪声 `Q`。",
            "- `ΔP(bg_z,mounting_yaw)` 保持近零，则说明首个 predict 并不直接生成 `bg_z-mounting_yaw` 交叉项；该相关结构需要后续量测更新建立。",
            "",
            "## Artifacts",
            f"- metrics: `{rel_from_root(metrics_path, REPO_ROOT)}`",
            f"- json: `{rel_from_root(json_path, REPO_ROOT)}`",
        ]
    )
    (outdir / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"[done] metrics: {metrics_path}")
    print(f"[done] json: {json_path}")


if __name__ == "__main__":
    main()
