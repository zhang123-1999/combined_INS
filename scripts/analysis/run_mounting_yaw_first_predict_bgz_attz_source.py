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
from scripts.analysis.run_mounting_yaw_nhc_bgz_compare import BASE_SHORT_CONFIG, mtime_text  # noqa: E402


MECHANISM_DEFAULT = Path(
    "output/debug_mounting_yaw_first_update_geometry_r1_20260323/"
    "cases/baseline/SOL_baseline_mechanism.csv"
)
OUTPUT_DIR_DEFAULT = Path("output/debug_mounting_yaw_first_predict_bgz_attz_source_r1_20260323")
INS_MECH_CODE = Path("src/core/ins_mech.cpp")


def parse_vec(text: str) -> list[float]:
    stripped = str(text).strip()
    if stripped == "[]" or not stripped:
        return []
    if stripped[0] == "[" and stripped[-1] == "]":
        stripped = stripped[1:-1]
    if not stripped:
        return []
    return [float(part) for part in stripped.split(";") if part]


def vec_norm(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values))


def read_code_markers(path: Path, markers: list[str]) -> dict[str, bool]:
    text = path.read_text(encoding="utf-8")
    return {marker: (marker in text) for marker in markers}


def read_imu(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=["t", "dtheta_x", "dtheta_y", "dtheta_z", "dvel_x", "dvel_y", "dvel_z"],
    )


def read_pos(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        sep=r"\s+",
        header=None,
        names=[
            "t",
            "lat_deg",
            "lon_deg",
            "h_m",
            "vn",
            "ve",
            "vd",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
        ],
    )


def find_row_at_or_after(df: pd.DataFrame, t0: float) -> pd.Series:
    hits = df[df["t"] >= t0 - 1e-9]
    if hits.empty:
        raise RuntimeError(f"no row found at or after t={t0}")
    return hits.iloc[0]


def find_exact_time_index(df: pd.DataFrame, t_meas: float) -> int:
    hits = df.index[(df["t"] - t_meas).abs() < 1e-9].tolist()
    if not hits:
        raise RuntimeError(f"no row found at t={t_meas}")
    return int(hits[0])


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
    if isinstance(values, list) and len(values) == 3 and all(v >= 0.0 for v in values):
        return [float(v) for v in values]
    scalar = float(cfg_noise[scalar_key])
    return [scalar, scalar, scalar]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Audit whether the first predict generates bg_z-att_z via PhiP0Phi^T or Q."
    )
    parser.add_argument("--config", type=Path, default=BASE_SHORT_CONFIG)
    parser.add_argument("--mechanism", type=Path, default=MECHANISM_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR_DEFAULT)
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
    mechanism_df.insert(0, "seq_idx", range(1, len(mechanism_df) + 1))
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

    p0_diag = [float(v) for v in init_cfg["P0_diag"]]
    p0_bg = p0_diag[12:15]
    p0_bgz = p0_diag[14]
    markov_corr_time = float(noise_cfg["markov_corr_time"])
    phi_bg_diag = 1.0
    if markov_corr_time > 0.0:
      phi_bg_diag -= dt / markov_corr_time

    phi_att_from_bg_col_z = [-value * dt for value in c_bn_col_bgz]
    phi_p0phi_bgz_att = [phi_bg_diag * p0_bgz * value for value in phi_att_from_bg_col_z]

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
    predicted_p_row_bg_z_att = [
        phi_term + q_term for phi_term, q_term in zip(phi_p0phi_bgz_att, q_bgz_att)
    ]

    phi_p0phi_bgz_bgz = (phi_bg_diag * phi_bg_diag) * p0_bgz
    q_bgz_bgz = 0.5 * (phi_bg_diag * q_bgz_bgz_cont * phi_bg_diag + q_bgz_bgz_cont) * dt
    predicted_p_bgz_bgz = phi_p0phi_bgz_bgz + q_bgz_bgz

    mechanism_p_row_bg_z_att = parse_vec(first_row["p_row_bg_z_att_vec"])
    mechanism_p_row_bg_z_bg = parse_vec(first_row["p_row_bg_z_bg_vec"])
    mechanism_att_z = mechanism_p_row_bg_z_att[2]
    predicted_att_z = predicted_p_row_bg_z_att[2]

    ins_mech_markers = read_code_markers(
        REPO_ROOT / INS_MECH_CODE,
        [
            "F.block<3, 3>(StateIdx::kAtt, StateIdx::kBg) = -C_bn;",
            "F.block<3, 3>(StateIdx::kBg, StateIdx::kBg) = neg_invT_I;  // bg",
            "G.block<3, 3>(StateIdx::kBg, 9) = Matrix3d::Identity();   // bg process noise",
            "Qd = 0.5 * (Phi * Qc_cont * Phi.transpose() + Qc_cont) * dt;",
        ],
    )

    report: dict[str, Any] = {
        "config_path": rel_from_root(config_path, REPO_ROOT),
        "mechanism_path": rel_from_root(mechanism_path, REPO_ROOT),
        "imu_path": rel_from_root(imu_path, REPO_ROOT),
        "pos_path": rel_from_root(pos_path, REPO_ROOT),
        "config_mtime": mtime_text(config_path),
        "mechanism_mtime": mtime_text(mechanism_path),
        "imu_mtime": mtime_text(imu_path),
        "pos_mtime": mtime_text(pos_path),
        "first_mechanism_seq": int(first_row["seq_idx"]),
        "first_mechanism_tag": str(first_row["tag"]),
        "first_mechanism_t_meas": t_meas,
        "imu_prev_t": float(imu_prev["t"]),
        "imu_curr_t": float(imu_curr["t"]),
        "dt": dt,
        "init_truth_t": float(init_truth["t"]),
        "init_truth_rpy_deg": [
            float(init_truth["roll_deg"]),
            float(init_truth["pitch_deg"]),
            float(init_truth["yaw_deg"]),
        ],
        "c_bn_col_bgz": c_bn_col_bgz,
        "p0_bg_diag": p0_bg,
        "p0_bgz": p0_bgz,
        "markov_corr_time": markov_corr_time,
        "phi_bgz_bgz": phi_bg_diag,
        "phi_att_from_bg_col_z": phi_att_from_bg_col_z,
        "phi_p0phi_bgz_att": phi_p0phi_bgz_att,
        "q_bgz_att": q_bgz_att,
        "predicted_p_row_bg_z_att": predicted_p_row_bg_z_att,
        "predicted_p_bgz_bgz": predicted_p_bgz_bgz,
        "phi_p0phi_bgz_bgz": phi_p0phi_bgz_bgz,
        "q_bgz_bgz": q_bgz_bgz,
        "mechanism_p_row_bg_z_att": mechanism_p_row_bg_z_att,
        "mechanism_p_row_bg_z_bg": mechanism_p_row_bg_z_bg,
        "mechanism_att_z": mechanism_att_z,
        "predicted_att_z": predicted_att_z,
        "att_z_abs_residual": mechanism_att_z - predicted_att_z,
        "att_z_rel_residual": abs(mechanism_att_z - predicted_att_z) / abs(mechanism_att_z),
        "q_over_total_att_z_ratio": abs(q_bgz_att[2]) / abs(predicted_att_z),
        "ins_mech_markers": ins_mech_markers,
    }
    report["logic_checks"] = {
        "first_update_is_nhc": report["first_mechanism_seq"] == 1
        and report["first_mechanism_tag"] == "NHC",
        "truth_init_matches_prev_imu": abs(report["init_truth_t"] - report["imu_prev_t"]) < 1e-9,
        "ins_mech_markers_present": all(ins_mech_markers.values()),
        "predicted_att_z_matches_mechanism": report["att_z_rel_residual"] < 1e-5,
    }

    summary_lines = [
        "# First Predict bg_z-att_z Source Audit",
        "",
        "目标：把 baseline short-window 中首个关键 `NHC` 前的 `P(bg_z,att_z)` 拆成首个 predict 的 `Phi P0 Phi^T` 与 `Q` 两部分，并定位对应过程模型子块。",
        "",
        "## Inputs",
        "",
        f"- config: `{report['config_path']}`",
        f"- mechanism: `{report['mechanism_path']}`",
        f"- imu: `{report['imu_path']}`",
        f"- pos/truth: `{report['pos_path']}`",
        f"- config_mtime: `{report['config_mtime']}`",
        f"- mechanism_mtime: `{report['mechanism_mtime']}`",
        "",
        "## First Predict Match",
        "",
        f"- first mechanism row: `seq={report['first_mechanism_seq']}`, `tag={report['first_mechanism_tag']}`, `t={report['first_mechanism_t_meas']:.6f}`.",
        f"- matched IMU pair: `t_prev={report['imu_prev_t']:.6f}`, `t_curr={report['imu_curr_t']:.6f}`, `dt={report['dt']:.12f}`.",
        f"- init truth row: `t={report['init_truth_t']:.6f}`, `rpy_deg=[{report['init_truth_rpy_deg'][0]:.6f}; {report['init_truth_rpy_deg'][1]:.6f}; {report['init_truth_rpy_deg'][2]:.6f}]`.",
        f"- `C_bn(:,bg_z)=[{'; '.join(f'{v:.12g}' for v in report['c_bn_col_bgz'])}]`, so `bg_z` mostly projects into `att_z`.",
        "",
        "## Process-Model Route",
        "",
        "- `ins_mech.cpp` gives `F_att,bg = -C_bn`, so `Phi_att, bg_z = -C_bn(:,2) * dt` to first order.",
        "- `ins_mech.cpp` gives `F_bg,bg = -I/T`, so `bg_z` row only keeps its own Markov self-decay before any measurement update.",
        "- `Qd` uses `0.5 * (Phi * Qc_cont * Phi^T + Qc_cont) * dt`, so `bg_z-att_z` process-noise cross term is only second-order in `dt`.",
        "",
        "## Decomposition",
        "",
        f"- `P0(bg_z,bg_z) = {report['p0_bgz']:.12g}`.",
        f"- `Phi(bg_z,bg_z) = {report['phi_bgz_bgz']:.12g}`.",
        f"- `Phi(att_x:att_z, bg_z) = [{'; '.join(f'{v:.12g}' for v in report['phi_att_from_bg_col_z'])}]`.",
        f"- `PhiP0Phi^T -> P(bg_z,att_x:att_z) = [{'; '.join(f'{v:.12g}' for v in report['phi_p0phi_bgz_att'])}]`.",
        f"- `Q -> P(bg_z,att_x:att_z) = [{'; '.join(f'{v:.12g}' for v in report['q_bgz_att'])}]`.",
        f"- predicted `P(bg_z,att_x:att_z) = [{'; '.join(f'{v:.12g}' for v in report['predicted_p_row_bg_z_att'])}]`.",
        f"- mechanism `P(bg_z,att_x:att_z) = [{'; '.join(f'{v:.12g}' for v in report['mechanism_p_row_bg_z_att'])}]`.",
        f"- predicted `P(bg_z,bg_z) = {report['predicted_p_bgz_bgz']:.12g}`, mechanism `P(bg_z,bg_z) = {report['mechanism_p_row_bg_z_bg'][2]:.12g}`.",
        "",
        "## Conclusion",
        "",
        "- 首个关键 `NHC` 前的 `P(bg_z,att_z)` 基本完全由 `Phi P0 Phi^T` 生成，主链就是 `P0(bg_z,bg_z)` 经 `F_att,bg = -C_bn` 映射到姿态行。",
        f"- 对 `att_z` 而言，`PhiP0Phi^T` 项为 `{report['phi_p0phi_bgz_att'][2]:.12g}`，`Q` 项仅 `{report['q_bgz_att'][2]:.12g}`，比例 `{report['q_over_total_att_z_ratio']:.12g}`。",
        f"- predicted vs mechanism residual on `P(bg_z,att_z)` is `{report['att_z_abs_residual']:.12g}` (relative `{report['att_z_rel_residual']:.12g}`).",
        "- 因此，baseline 首个坏 route 的剩余关键解释不是“首个 NHC 因 0 初值先错”，而是“首个 predict 已把 `bg_z` 方差搬运进 `att_z`，首个 NHC 再顺着这个几何放大错误相关结构”。",
    ]
    summary_text = "\n".join(summary_lines) + "\n"

    (outdir / "summary.md").write_text(summary_text, encoding="utf-8")
    (outdir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    print(f"[done] summary: {outdir / 'summary.md'}")
    print(f"[done] report: {outdir / 'report.json'}")


if __name__ == "__main__":
    main()
