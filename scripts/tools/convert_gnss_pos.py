#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GNSS定位结果文件转换程序
将RTKPOST输出的.pos文件转换为：GPS周, GPS秒, 纬度, 经度, 高度, 北向标准差, 东向标准差, 天向标准差
"""

import os
import re
from datetime import datetime, timezone, timedelta


def parse_gpst_to_gps_week_seconds(gpst_str):
    """
    将GPST时间字符串转换为GPS周和周内秒
    输入格式: 2018/12/29 02:33:57.000
    返回: (gps_week, gps_seconds)
    """
    # 解析时间字符串
    dt = datetime.strptime(gpst_str.strip(), "%Y/%m/%d %H:%M:%S.%f")

    # 设置时区为UTC (GPST与UTC时间相差18秒左右，但这里近似使用UTC)
    # GPST = UTC + 闰秒数 (当前闰秒数为18秒)
    # 为了精确，我们需要计算GPS周和秒

    # GPS起始时间: 1980年1月6日 00:00:00 UTC
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

    # 将解析的时间转换为UTC时间 (假设输入是GPST)
    # GPST比UTC快18秒 (截至2024年，闰秒数为18)
    leap_seconds = 18
    utc_dt = dt - timedelta(seconds=leap_seconds)

    # 计算从GPS epoch到当前时间的秒数
    time_diff = utc_dt.replace(tzinfo=timezone.utc) - gps_epoch
    total_seconds = time_diff.total_seconds()

    # 计算GPS周和周内秒
    gps_week = int(total_seconds // 604800)  # 604800 = 7 * 24 * 3600
    gps_seconds = total_seconds % 604800

    return gps_week, gps_seconds


def convert_pos_file(input_file, output_file):
    """
    将.pos文件转换为指定格式
    输出格式: GPS周, GPS秒, 纬度, 经度, 高度, 北向标准差, 东向标准差, 天向标准差
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    results = []

    for line in lines:
        line = line.strip()

        # 跳过空行和注释行
        if not line or line.startswith('%'):
            continue

        # 分割数据行
        parts = line.split()

        if len(parts) < 10:
            continue

        try:
            # 解析时间 (第1-2列: 日期和时间)
            date_str = parts[0]
            time_str = parts[1]
            gpst_str = f"{date_str} {time_str}"

            gps_week, gps_seconds = parse_gpst_to_gps_week_seconds(gpst_str)

            # 解析位置 (第2-4列: 纬度, 经度, 高度)
            latitude = float(parts[2])
            longitude = float(parts[3])
            height = float(parts[4])

            # 解析标准差 (第7-9列: sdn, sde, sdu)
            sdn = float(parts[7])  # 北向标准差
            sde = float(parts[8])  # 东向标准差
            sdu = float(parts[9])  # 天向标准差

            results.append({
                'gps_week': gps_week,
                'gps_seconds': gps_seconds,
                'latitude': latitude,
                'longitude': longitude,
                'height': height,
                'sdn': sdn,
                'sde': sde,
                'sdu': sdu
            })

        except (ValueError, IndexError) as e:
            print(f"解析行出错: {line}")
            print(f"错误: {e}")
            continue

    # 写入输出文件
    with open(output_file, 'w', encoding='utf-8') as f:
        # 写入表头
        f.write("GPS_Week,GPS_Seconds,Latitude,Longitude,Height,Std_North,Std_East,Std_Up\n")

        # 写入数据
        for r in results:
            f.write(f"{r['gps_week']},{r['gps_seconds']:.3f},{r['latitude']},{r['longitude']},{r['height']},{r['sdn']},{r['sde']},{r['sdu']}\n")

    print(f"转换完成!")
    print(f"输入文件: {input_file}")
    print(f"输出文件: {output_file}")
    print(f"总记录数: {len(results)}")


def main():
    # 输入文件路径
    input_file = r"D:\因子图算法科研训练\UWB\dataset\data2\rover\GPSC1Log201812290233_2.pos"

    # 输出文件路径 (data2目录下)
    output_file = r"D:\因子图算法科研训练\UWB\dataset\data2\GPSC1Log201812290233_2_converted.csv"

    # 执行转换
    convert_pos_file(input_file, output_file)


if __name__ == "__main__":
    main()
