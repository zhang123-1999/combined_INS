#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
data4 数据集转换工具
将原始 IMU(bin)/POS(.pos)/NAV(.nav)/ODO(.txt) 转换为融合程序统一输入格式。

原始数据目录结构:
    dataset/data4/
        IMU/
            ADIS-16460/HL20250924042130_ADIS_0_IMU.bin   (中等精度)
            ANS-200R/1.bin                                (高精度)
            ICM-20602/HL20250924042130_ICM_0_IMU.bin      (低精度)
        ref/LC_SM_TXT.nav                                 (真值/参考轨迹)
        rover/202509240423.pos                            (GNSS RTK 定位结果)
        odo/1_vel_odo.txt                                 (里程计速度)

输出格式 (保存到 dataset/data4_converted/):
    IMU_converted_<imu_name>.txt   7列: t dθx dθy dθz dvx dvy dvz (Body FRD 增量)
    POS_converted.txt              10列: t lat lon h vn ve vd roll pitch yaw (LLA deg + NED m/s + RPY deg)
    GNSS_converted.txt             7列: t lat lon h σn σe σd (LLA deg + NED精度 m)
    ODO_converted.txt              2列: t v_forward (m/s)

用法:
    python scripts/tools/convert_data4.py
    python scripts/tools/convert_data4.py --imu ADIS-16460     # 仅转换指定 IMU
    python scripts/tools/convert_data4.py --gnss-q-max 2       # 包含 float 解
"""

import argparse
import os
import struct
import sys
from datetime import datetime

import numpy as np


# ============================================================
# GPS 时间工具
# ============================================================
GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0)


def gpst_str_to_gps_sow(date_str: str, time_str: str) -> float:
    """
    RTKPOST 的 GPST 时间字符串 → GPS 周内秒。
    输入: date_str='2025/09/24', time_str='04:23:00.000'
    GPST 是连续时间，无需闰秒修正。
    """
    dt = datetime.strptime(f"{date_str} {time_str}", "%Y/%m/%d %H:%M:%S.%f")
    total_seconds = (dt - GPS_EPOCH).total_seconds()
    gps_sow = total_seconds % 604800.0
    return gps_sow


# ============================================================
# IMU 二进制文件转换
# ============================================================
IMU_RECORD_SIZE = 56  # 7 × 8 bytes (7 doubles: t, gx, gy, gz, ax, ay, az)

# IMU 文件名 → 二进制文件相对路径
IMU_MAP = {
    "ADIS-16460": "ADIS-16460/HL20250924042130_ADIS_0_IMU.bin",
    "ANS-200R":   "ANS-200R/1.bin",
    "ICM-20602":  "ICM-20602/HL20250924042130_ICM_0_IMU.bin",
}


def convert_imu(bin_path: str, output_path: str) -> int:
    """
    读取 IMU 二进制文件 (7 doubles/record)，直接输出为文本。
    data4 的 IMU 数据已经是增量形式 (dtheta rad, dvel m/s)，无需再乘 dt。
    格式: timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z
    """
    with open(bin_path, "rb") as f:
        raw = f.read()

    n_records = len(raw) // IMU_RECORD_SIZE
    if n_records == 0:
        print(f"  [WARN] IMU 文件为空: {bin_path}")
        return 0

    # 按 little-endian 解析全部数据
    fmt = f"<{n_records * 7}d"
    flat = struct.unpack(fmt, raw[: n_records * IMU_RECORD_SIZE])
    data = np.array(flat).reshape(n_records, 7)

    # 写入文本
    with open(output_path, "w") as f:
        f.write("timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z\n")
        for row in data:
            f.write(
                f"{row[0]:.6f} {row[1]:.15e} {row[2]:.15e} {row[3]:.15e} "
                f"{row[4]:.15e} {row[5]:.15e} {row[6]:.15e}\n"
            )

    print(f"  IMU -> {output_path}  ({n_records} records, dt≈{np.median(np.diff(data[:100,0])):.6f}s)")
    return n_records


# ============================================================
# 真值 .nav 文件转换
# ============================================================
def convert_truth(nav_path: str, output_path: str) -> int:
    """
    LC_SM_TXT.nav 格式 (14列):
      flag  t  lat(deg) lon(deg) h(m)  vn ve vd  roll pitch yaw  extra1 extra2 extra3

    输出 POS_converted.txt (10列):
      t lat(deg) lon(deg) h(m) vn(m/s) ve(m/s) vd(m/s) roll(deg) pitch(deg) yaw(deg)

    保持 LLA+NED 格式，LoadDataset 会自动检测并转换为 ECEF。
    """
    data = np.loadtxt(nav_path)
    n = data.shape[0]

    # 列索引: [0]=flag, [1]=t, [2]=lat, [3]=lon, [4]=h, [5]=vn, [6]=ve, [7]=vd,
    #          [8]=roll, [9]=pitch, [10]=yaw, [11:14]=extras
    timestamps = data[:, 1]
    lat = data[:, 2]
    lon = data[:, 3]
    h = data[:, 4]
    vn = data[:, 5]
    ve = data[:, 6]
    vd = data[:, 7]
    roll = data[:, 8]
    pitch = data[:, 9]
    yaw = data[:, 10]

    output = np.column_stack([timestamps, lat, lon, h, vn, ve, vd, roll, pitch, yaw])

    with open(output_path, "w") as f:
        f.write("timestamp lat lon h vn ve vd roll pitch yaw\n")
        np.savetxt(f, output, fmt="%.15g")

    print(f"  Truth -> {output_path}  ({n} records, t=[{timestamps[0]:.3f}, {timestamps[-1]:.3f}])")
    return n


# ============================================================
# GNSS .pos 文件转换
# ============================================================
def convert_gnss(pos_path: str, output_path: str, q_max: int = 1) -> int:
    """
    RTKPOST .pos 文件 (GPST 格式):
      date  time  lat(deg) lon(deg) h(m)  Q  ns  sdn sde sdu  sdne sdeu sdun  age ratio

    输出 GNSS_converted.txt (13列):
      t(GPS周内秒) lat(deg) lon(deg) h(m) sigma_n(m) sigma_e(m) sigma_d(m) vn(m/s) ve(m/s) vd(m/s) sigma_vn(m/s) sigma_ve(m/s) sigma_vd(m/s)
      坐标系: 标准 NED (北/东/下)，此处已将 RTKPOST 输出的 vu(Up) 取反为 vd(Down)。
      pipeline_fusion 读入后可直接按 NED 使用，无需再做坐标变换。

    q_max: 最大允许的解算质量 (1=fix, 2=float, 3=sbas, ...)
    """
    results = []
    with open(pos_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 10:
                continue
            try:
                q = int(parts[5])
                if q > q_max:
                    continue

                gps_sow = gpst_str_to_gps_sow(parts[0], parts[1])
                lat = float(parts[2])
                lon = float(parts[3])
                h = float(parts[4])
                sdn = float(parts[7])
                sde = float(parts[8])
                sdu = float(parts[9])

                # 提取速度信息 (vn, ve, vu) 及标准差 (sdvn, sdve, sdvu)
                # RTKPOST .pos格式列索引(0-based):
                # [0]date [1]time [2]lat [3]lon [4]h [5]Q [6]ns
                # [7]sdn [8]sde [9]sdu [10]sdne [11]sdeu [12]sdun
                # [13]age [14]ratio [15]vn [16]ve [17]vu
                # [18]sdvn [19]sdve [20]sdvu [21]sdvne [22]sdveu [23]sdvun
                vn   = float(parts[15]) if len(parts) > 15 else 0.0
                ve   = float(parts[16]) if len(parts) > 16 else 0.0
                vu   = float(parts[17]) if len(parts) > 17 else 0.0
                sdvn = float(parts[18]) if len(parts) > 18 else 0.1
                sdve = float(parts[19]) if len(parts) > 19 else 0.1
                sdvu = float(parts[20]) if len(parts) > 20 else 0.1

                # RTKPOST 输出的第三速度分量是 vu (Up，向上为正)
                # 统一转换为 NED 约定: vd = -vu (Down，向下为正)
                vd   = -vu

                results.append([gps_sow, lat, lon, h, sdn, sde, sdu, vn, ve, vd, sdvn, sdve, sdvu])
            except (ValueError, IndexError):
                continue

    if not results:
        print(f"  [WARN] GNSS: 无有效数据 (q_max={q_max})")
        return 0

    arr = np.array(results)

    with open(output_path, "w") as f:
        f.write("timestamp lat lon h sigma_n sigma_e sigma_d vn ve vd sigma_vn sigma_ve sigma_vd\n")
        np.savetxt(f, arr, fmt="%.15g")

    print(f"  GNSS -> {output_path}  ({len(results)} records, q<={q_max}, "
          f"t=[{arr[0,0]:.3f}, {arr[-1,0]:.3f}])")
    return len(results)


# ============================================================
# ODO 文件转换
# ============================================================
def convert_odo(odo_path: str, output_path: str) -> int:
    """
    原始 ODO 格式 (3列): t  v_forward  v_unused(全零)
    输出 ODO_converted.txt (2列): t  v_forward(m/s)
    """
    data = np.loadtxt(odo_path)
    n = data.shape[0]

    output = data[:, :2]  # 只取前两列

    with open(output_path, "w") as f:
        f.write("timestamp v_forward\n")
        np.savetxt(f, output, fmt="%.15g")

    nonzero = np.count_nonzero(np.abs(data[:, 1]) > 0.01)
    print(f"  ODO  -> {output_path}  ({n} records, {nonzero} non-zero, "
          f"v_max={np.abs(data[:,1]).max():.2f} m/s)")
    return n


# ============================================================
# 主函数
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="data4 数据集格式转换")
    parser.add_argument("--data-dir", default="dataset/data4",
                        help="原始数据目录 (default: dataset/data4)")
    parser.add_argument("--output-dir", default="dataset/data4_converted",
                        help="输出目录 (default: dataset/data4_converted)")
    parser.add_argument("--imu", default=None, choices=list(IMU_MAP.keys()),
                        help="仅转换指定 IMU (默认全部)")
    parser.add_argument("--gnss-q-max", type=int, default=1,
                        help="GNSS 最大解算质量 (1=fix only, 2=+float, default=1)")
    args = parser.parse_args()

    # 路径处理 (支持从项目根目录运行)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
    data_dir = os.path.join(project_root, args.data_dir)
    output_dir = os.path.join(project_root, args.output_dir)

    if not os.path.isdir(data_dir):
        print(f"ERROR: 数据目录不存在: {data_dir}")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)
    print(f"数据源: {data_dir}")
    print(f"输出到: {output_dir}")
    print()

    # 1) IMU 转换
    imu_dir = os.path.join(data_dir, "IMU")
    imu_names = [args.imu] if args.imu else list(IMU_MAP.keys())
    for name in imu_names:
        bin_rel = IMU_MAP[name]
        bin_path = os.path.join(imu_dir, bin_rel)
        if os.path.exists(bin_path):
            out_path = os.path.join(output_dir, f"IMU_converted_{name}.txt")
            convert_imu(bin_path, out_path)
        else:
            print(f"  [SKIP] IMU 文件不存在: {bin_path}")

    # 如果只转换了一种 IMU，额外创建一个通用名称的符号/副本
    if args.imu:
        src = os.path.join(output_dir, f"IMU_converted_{args.imu}.txt")
        dst = os.path.join(output_dir, "IMU_converted.txt")
        if os.path.exists(src):
            import shutil
            shutil.copy2(src, dst)
            print(f"  -> 复制为 IMU_converted.txt (当前选择: {args.imu})")

    print()

    # 2) 真值转换
    nav_path = os.path.join(data_dir, "ref", "LC_SM_TXT.nav")
    if os.path.exists(nav_path):
        convert_truth(nav_path, os.path.join(output_dir, "POS_converted.txt"))
    else:
        print(f"  [SKIP] 真值文件不存在: {nav_path}")

    print()

    # 3) GNSS 转换
    pos_path = os.path.join(data_dir, "rover", "202509240423.pos")
    if os.path.exists(pos_path):
        convert_gnss(pos_path, os.path.join(output_dir, "GNSS_converted.txt"),
                     q_max=args.gnss_q_max)
    else:
        print(f"  [SKIP] GNSS 文件不存在: {pos_path}")

    print()

    # 4) ODO 转换
    odo_path = os.path.join(data_dir, "odo", "1_vel_odo.txt")
    if os.path.exists(odo_path):
        convert_odo(odo_path, os.path.join(output_dir, "ODO_converted.txt"))
    else:
        print(f"  [SKIP] ODO 文件不存在: {odo_path}")

    print()
    print("转换完成!")
    print(f"输出文件列表:")
    for fname in sorted(os.listdir(output_dir)):
        fpath = os.path.join(output_dir, fname)
        size_mb = os.path.getsize(fpath) / 1024 / 1024
        print(f"  {fname}  ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
