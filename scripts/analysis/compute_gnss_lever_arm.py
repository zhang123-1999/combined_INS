"""
compute_gnss_lever_arm.py
通过 GNSS 天线位置与 IMU 真值位置的差值，在体系（body frame）下估计 GNSS 天线杆臂。

算法流程：
  1. 加载 GNSS_converted.txt（13列：timestamp lat lon h ...）
  2. 加载 POS_converted.txt（10列：timestamp lat lon alt vn ve vd roll pitch yaw）
  3. 找到时间重叠区间，将 POS 插值到 GNSS 时间戳
  4. 两者均转为 ECEF
  5. delta_ecef = p_gnss - p_imu
  6. 转到 NED：delta_ned = R_en * delta_ecef  (用 POS 参考点)
  7. 转到 body frame：delta_body = C_bn^T * delta_ned  (用 POS 插值姿态)
  8. 输出统计（均值 / 标准差 / 中位数）

用法：
  python scripts/analysis/compute_gnss_lever_arm.py
  python scripts/analysis/compute_gnss_lever_arm.py \
      --gnss dataset/data4_converted/GNSS_converted.txt \
      --pos  dataset/data4_converted/POS_converted.txt
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_E2 = 1.0 - (WGS84_B / WGS84_A) ** 2


# ---------------------------------------------------------------------------
# 坐标变换工具
# ---------------------------------------------------------------------------

def llh_deg_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, h_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * np.cos(lon)
    y = (n + h_m) * cos_lat * np.sin(lon)
    z = (n * (1.0 - WGS84_E2) + h_m) * sin_lat
    return np.column_stack((x, y, z))


def ecef_delta_to_ned(
    delta_ecef: np.ndarray,
    ref_lat_deg: np.ndarray,
    ref_lon_deg: np.ndarray,
) -> np.ndarray:
    """将 ECEF 向量旋转到 NED（以参考点经纬度为准）。"""
    lat = np.deg2rad(ref_lat_deg)
    lon = np.deg2rad(ref_lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    # R_en (3, 3) 对每个样本不同，用逐元素 einsum 处理
    # N = -sin_lat*cos_lon*X - sin_lat*sin_lon*Y + cos_lat*Z
    # E = -sin_lon*X + cos_lon*Y
    # D = -cos_lat*cos_lon*X - cos_lat*sin_lon*Y - sin_lat*Z
    dn = -sin_lat * cos_lon * delta_ecef[:, 0] - sin_lat * sin_lon * delta_ecef[:, 1] + cos_lat * delta_ecef[:, 2]
    de = -sin_lon * delta_ecef[:, 0] + cos_lon * delta_ecef[:, 1]
    dd = -cos_lat * cos_lon * delta_ecef[:, 0] - cos_lat * sin_lon * delta_ecef[:, 1] - sin_lat * delta_ecef[:, 2]
    return np.column_stack((dn, de, dd))


def euler_to_rotation(roll_rad: np.ndarray, pitch_rad: np.ndarray, yaw_rad: np.ndarray) -> np.ndarray:
    """建立 C_bn（body to nav，即 NED->body 的逆）旋转矩阵，形状 (N,3,3)。"""
    sr, cr = np.sin(roll_rad), np.cos(roll_rad)
    sp, cp = np.sin(pitch_rad), np.cos(pitch_rad)
    sy, cy = np.sin(yaw_rad), np.cos(yaw_rad)
    r11 = cp * cy
    r12 = sr * sp * cy - cr * sy
    r13 = cr * sp * cy + sr * sy
    r21 = cp * sy
    r22 = sr * sp * sy + cr * cy
    r23 = cr * sp * sy - sr * cy
    r31 = -sp
    r32 = sr * cp
    r33 = cr * cp
    return np.stack(
        [np.stack([r11, r12, r13], axis=-1),
         np.stack([r21, r22, r23], axis=-1),
         np.stack([r31, r32, r33], axis=-1)],
        axis=-2,
    )


def ned_to_body(delta_ned: np.ndarray, C_nb: np.ndarray) -> np.ndarray:
    """delta_body = C_nb^T * delta_ned = C_bn * delta_ned。
    C_nb.shape == (N, 3, 3)，delta_ned.shape == (N, 3)。"""
    # C_nb^T = C_bn
    return np.einsum("...ji,...j->...i", C_nb, delta_ned)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def load_gnss(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, sep=r"\s+")
    required = {"timestamp", "lat", "lon", "h"}
    missing = required - set(df.columns)
    if missing:
        raise RuntimeError(f"GNSS 文件缺少列 {missing}: {path}")
    return df


def load_pos(path: Path) -> pd.DataFrame:
    has_header = True
    with path.open() as f:
        first = f.readline().strip()
    if first.split()[0].replace(".", "").replace("-", "").isdigit():
        has_header = False
    if has_header:
        df = pd.read_csv(path, sep=r"\s+")
        df = df.rename(columns={"h": "alt"})
    else:
        df = pd.read_csv(path, sep=r"\s+", header=None)
        df.columns = ["timestamp", "lat", "lon", "alt", "vn", "ve", "vd", "roll", "pitch", "yaw"]
    return df


def interp_pos_to_gnss(gnss_time: np.ndarray, pos_df: pd.DataFrame):
    """将 POS 各列插值到 GNSS 时间戳（仅插值时间重叠部分）。"""
    pos_time = pos_df["timestamp"].to_numpy(dtype=float)
    t_lo, t_hi = max(gnss_time[0], pos_time[0]), min(gnss_time[-1], pos_time[-1])
    mask = (gnss_time >= t_lo) & (gnss_time <= t_hi)
    t = gnss_time[mask]
    out: dict[str, np.ndarray] = {"mask": mask}
    for col in ["lat", "lon", "alt", "roll", "pitch"]:
        out[col] = np.interp(t, pos_time, pos_df[col].to_numpy(dtype=float))
    # yaw 需要 unwrap 插值防止 ±180 跳变
    yaw_rad = np.unwrap(np.deg2rad(pos_df["yaw"].to_numpy(dtype=float)))
    out["yaw"] = np.rad2deg(np.interp(t, pos_time, yaw_rad))
    return t, out


def compute_lever_arm(
    gnss_path: Path,
    pos_path: Path,
    *,
    verbose: bool = True,
    percentile_lo: float = 5.0,
    percentile_hi: float = 95.0,
) -> np.ndarray:
    gnss_df = load_gnss(gnss_path)
    pos_df = load_pos(pos_path)

    gnss_time = gnss_df["timestamp"].to_numpy(dtype=float)
    t_common, pos_interp = interp_pos_to_gnss(gnss_time, pos_df)
    mask = pos_interp["mask"]

    n_samples = int(mask.sum())
    if n_samples < 10:
        raise RuntimeError(f"时间重叠区间样本过少（{n_samples}），无法估计杆臂。")

    # GNSS 天线位置（ECEF）
    gnss_lat = gnss_df.loc[mask, "lat"].to_numpy(dtype=float)
    gnss_lon = gnss_df.loc[mask, "lon"].to_numpy(dtype=float)
    gnss_h   = gnss_df.loc[mask, "h"].to_numpy(dtype=float)
    p_gnss_ecef = llh_deg_to_ecef(gnss_lat, gnss_lon, gnss_h)

    # IMU 真值位置（ECEF）
    p_imu_ecef = llh_deg_to_ecef(
        pos_interp["lat"], pos_interp["lon"], pos_interp["alt"]
    )

    # ECEF 差向量，转 NED（以 IMU 真值经纬为参考方向）
    delta_ecef = p_gnss_ecef - p_imu_ecef
    delta_ned  = ecef_delta_to_ned(delta_ecef, pos_interp["lat"], pos_interp["lon"])

    # 转体系（body frame）
    C_nb = euler_to_rotation(
        np.deg2rad(pos_interp["roll"]),
        np.deg2rad(pos_interp["pitch"]),
        np.deg2rad(pos_interp["yaw"]),
    )
    delta_body = ned_to_body(delta_ned, C_nb)

    # 统计
    mean_b  = np.mean(delta_body, axis=0)
    std_b   = np.std(delta_body, axis=0)
    median_b = np.median(delta_body, axis=0)
    lo_b    = np.percentile(delta_body, percentile_lo, axis=0)
    hi_b    = np.percentile(delta_body, percentile_hi, axis=0)

    if verbose:
        print(f"\n{'='*60}")
        print(f" GNSS 天线杆臂估计结果（体系 body frame，单位 m）")
        print(f"{'='*60}")
        print(f" 样本数量      : {n_samples}")
        print(f" 时间区间      : {t_common[0]:.3f} ~ {t_common[-1]:.3f} s")
        print(f"")
        print(f" {'量':>12}  {'x_b (前向)':>14}  {'y_b (右向)':>14}  {'z_b (下向)':>14}")
        print(f" {'-'*12}  {'-'*14}  {'-'*14}  {'-'*14}")
        print(f" {'均值':>12}  {mean_b[0]:14.4f}  {mean_b[1]:14.4f}  {mean_b[2]:14.4f}")
        print(f" {'中位数':>12}  {median_b[0]:14.4f}  {median_b[1]:14.4f}  {median_b[2]:14.4f}")
        print(f" {'标准差':>12}  {std_b[0]:14.4f}  {std_b[1]:14.4f}  {std_b[2]:14.4f}")
        print(f" {f'P{percentile_lo:.0f}':>12}  {lo_b[0]:14.4f}  {lo_b[1]:14.4f}  {lo_b[2]:14.4f}")
        print(f" {f'P{percentile_hi:.0f}':>12}  {hi_b[0]:14.4f}  {hi_b[1]:14.4f}  {hi_b[2]:14.4f}")
        print(f"")
        print(f" 推荐值（中位数）: gnss_lever_arm0: [{median_b[0]:.4f}, {median_b[1]:.4f}, {median_b[2]:.4f}]")
        print(f"{'='*60}\n")

    return median_b


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="估计 GNSS 天线杆臂（体系 body frame）")
    p.add_argument(
        "--gnss",
        type=Path,
        default=REPO_ROOT / "dataset/data4_converted/GNSS_converted.txt",
        help="GNSS_converted.txt 路径",
    )
    p.add_argument(
        "--pos",
        type=Path,
        default=REPO_ROOT / "dataset/data4_converted/POS_converted.txt",
        help="POS_converted.txt 路径",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    compute_lever_arm(args.gnss, args.pos, verbose=True)


if __name__ == "__main__":
    main()
