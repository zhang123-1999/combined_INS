#!/usr/bin/env python3
"""
转换 data2 数据集到统一格式。

修正点:
1) ODO前向速度取原始ODO文件的倒数第二列（最后一列在该数据中恒为0）。
2) GNSS时间戳对齐 data2 真值时基，补偿 18s 偏移。
"""

import numpy as np
import pandas as pd
import struct
import os
import sys

DATA2_GNSS_TIME_OFFSET_SEC = 18.0


def convert_imu_binary(bin_path, output_txt):
    """转换IMU二进制文件到文本格式"""
    print(f"Converting IMU binary: {bin_path}")

    # 读取二进制文件
    with open(bin_path, 'rb') as f:
        data = f.read()

    # 假设数据结构: timestamp(8 bytes double) + gyro(3x8 bytes double) + acc(3x8 bytes double)
    record_size = 8 + 3*8 + 3*8  # 56 bytes per record
    num_records = len(data) // record_size

    results = []
    for i in range(num_records):
        offset = i * record_size
        # 解析数据
        timestamp = struct.unpack('d', data[offset:offset+8])[0]
        gyro_x = struct.unpack('d', data[offset+8:offset+16])[0]
        gyro_y = struct.unpack('d', data[offset+16:offset+24])[0]
        gyro_z = struct.unpack('d', data[offset+24:offset+32])[0]
        acc_x = struct.unpack('d', data[offset+32:offset+40])[0]
        acc_y = struct.unpack('d', data[offset+40:offset+48])[0]
        acc_z = struct.unpack('d', data[offset+48:offset+56])[0]

        # data2 二进制文件中这6个量已是增量(dtheta/dvel)，无需再乘 dt。
        dtheta_x = gyro_x
        dtheta_y = gyro_y
        dtheta_z = gyro_z
        dvel_x = acc_x
        dvel_y = acc_y
        dvel_z = acc_z

        results.append([timestamp, dtheta_x, dtheta_y, dtheta_z, dvel_x, dvel_y, dvel_z])

    # 保存为文本
    np.savetxt(output_txt, np.array(results), fmt='%.6f')
    print(f"IMU converted: {output_txt}, {len(results)} records")


def convert_odo_txt(odo_path, output_txt):
    """转换ODO文本文件"""
    print(f"Converting ODO: {odo_path}")

    # 读取原始ODO数据
    # 格式(9列): timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z odo_speed reserved
    data = np.loadtxt(odo_path)
    if data.ndim != 2 or data.shape[1] < 2:
        raise ValueError(f"Invalid ODO format: {odo_path}")

    # data2 的有效ODO速度在倒数第二列；最后一列为保留字段且恒为0。
    timestamps = data[:, 0]
    forward_speed = data[:, -2]

    output = np.column_stack([timestamps, forward_speed])
    np.savetxt(output_txt, output, fmt='%.6f')
    print(f"ODO converted: {output_txt}, {len(timestamps)} records")


def convert_truth_nav(nav_path, output_txt):
    """转换真值导航文件"""
    print(f"Converting truth: {nav_path}")

    # 读取真值数据
    # 格式: flag timestamp lat lon height vn ve vd roll pitch yaw
    data = np.loadtxt(nav_path)

    # 转换为ECEF坐标 (简化处理，实际需要完整的坐标转换)
    # 这里我们保持LLA格式，后续处理
    timestamps = data[:, 1]
    lat = data[:, 2]
    lon = data[:, 3]
    height = data[:, 4]
    vn = data[:, 5]
    ve = data[:, 6]
    vd = data[:, 7]
    roll = data[:, 8]
    pitch = data[:, 9]
    yaw = data[:, 10]

    # 保存转换后的数据 (包含LLA和ECEF速度、姿态)
    output = np.column_stack([timestamps, lat, lon, height, vn, ve, vd, roll, pitch, yaw])
    np.savetxt(output_txt, output, fmt='%.9f')
    print(f"Truth converted: {output_txt}, {len(timestamps)} records")


def convert_gnss_csv(gnss_path, output_txt, time_offset_sec=0.0):
    """转换GNSS CSV文件"""
    print(f"Converting GNSS: {gnss_path}")

    # 读取GNSS数据
    df = pd.read_csv(gnss_path)

    # 提取时间和位置信息
    # GPS_Week, GPS_Seconds, Latitude, Longitude, Height, Std_North, Std_East, Std_Up
    gps_week = df['GPS_Week'].values
    gps_seconds = df['GPS_Seconds'].values
    lat = df['Latitude'].values
    lon = df['Longitude'].values
    height = df['Height'].values
    std_north = df['Std_North'].values
    std_east = df['Std_East'].values
    std_up = df['Std_Up'].values

    # 使用 GPS_Seconds，并补偿 data2 时基偏移
    timestamps = gps_seconds + float(time_offset_sec)

    # 保存转换后的数据
    output = np.column_stack([timestamps, lat, lon, height, std_north, std_east, std_up])
    np.savetxt(output_txt, output, fmt='%.9f')
    print(
        f"GNSS converted: {output_txt}, {len(timestamps)} records, "
        f"time_offset={time_offset_sec:.3f}s"
    )


def main():
    """主函数"""
    data2_dir = "dataset/data2"
    output_dir = "dataset/data2_converted"

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    # 转换IMU数据
    imu_bin = os.path.join(data2_dir, "IMULog201812290233_2.bin")
    imu_txt = os.path.join(output_dir, "IMU_converted.txt")
    if os.path.exists(imu_bin):
        convert_imu_binary(imu_bin, imu_txt)
    else:
        print(f"IMU binary not found: {imu_bin}")

    # 转换ODO数据
    odo_txt = os.path.join(data2_dir, "IMULog201812290233_2_ODO.txt")
    odo_out = os.path.join(output_dir, "ODO_converted.txt")
    if os.path.exists(odo_txt):
        convert_odo_txt(odo_txt, odo_out)
    else:
        print(f"ODO file not found: {odo_txt}")

    # 转换真值数据
    nav_txt = os.path.join(data2_dir, "LC_SM_TXT.nav")
    nav_out = os.path.join(output_dir, "POS_converted.txt")
    if os.path.exists(nav_txt):
        convert_truth_nav(nav_txt, nav_out)
    else:
        print(f"Truth file not found: {nav_txt}")

    # 转换GNSS数据
    gnss_csv = os.path.join(data2_dir, "GPSC1Log201812290233_2_converted.csv")
    gnss_out = os.path.join(output_dir, "GNSS_converted.txt")
    if os.path.exists(gnss_csv):
        convert_gnss_csv(
            gnss_csv,
            gnss_out,
            time_offset_sec=DATA2_GNSS_TIME_OFFSET_SEC,
        )
    else:
        print(f"GNSS file not found: {gnss_csv}")

    print("\nData conversion completed!")
    print(f"Output directory: {output_dir}")


if __name__ == "__main__":
    main()
