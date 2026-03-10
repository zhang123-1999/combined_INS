#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESKF融合导航结果可视化分析脚本

支持两种结果文件格式：
  1. 新格式 (SOL_*.txt): ECEF坐标，28/31列
     timestamp fused_x fused_y fused_z fused_vx fused_vy fused_vz
     fused_roll fused_pitch fused_yaw mounting_pitch mounting_yaw odo_scale
     sg_x sg_y sg_z sa_x sa_y sa_z ba_x ba_y ba_z bg_x bg_y bg_z
     lever_x lever_y lever_z [gnss_lever_x gnss_lever_y gnss_lever_z]
  2. 旧格式 (KF-GINS): 经纬高坐标，51列（含std列）
"""

import os
import glob
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互后端，直接保存不弹窗
import matplotlib.pyplot as plt
import sys
import yaml

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ========== ECEF <-> LLA 转换 ==========
# WGS-84 椭球参数
WGS84_A = 6378137.0            # 长半轴 [m]
WGS84_F = 1.0 / 298.257223563  # 扁率
WGS84_B = WGS84_A * (1 - WGS84_F)
WGS84_E2 = 1 - (WGS84_B / WGS84_A) ** 2  # 第一偏心率平方


def ecef_to_lla(x, y, z):
    """ECEF坐标转经纬高 (Bowring迭代法)"""
    lon = np.arctan2(y, x)
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - WGS84_E2))  # 初始估计
    for _ in range(5):
        sin_lat = np.sin(lat)
        N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)
        lat = np.arctan2(z + WGS84_E2 * N * sin_lat, p)
    sin_lat = np.sin(lat)
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat**2)
    alt = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), alt


def ecef_vel_to_ned(vx, vy, vz, lat_rad, lon_rad):
    """ECEF速度转NED速度"""
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)
    vn = -sin_lat * cos_lon * vx - sin_lat * sin_lon * vy + cos_lat * vz
    ve = -sin_lon * vx + cos_lon * vy
    vd = -cos_lat * cos_lon * vx - cos_lat * sin_lon * vy - sin_lat * vz
    return vn, ve, vd


def euler_to_rotation(roll_rad, pitch_rad, yaw_rad):
    """欧拉角(Body -> Nav, ZYX)转旋转矩阵，支持标量或等长数组。"""
    roll_rad = np.asarray(roll_rad)
    pitch_rad = np.asarray(pitch_rad)
    yaw_rad = np.asarray(yaw_rad)

    sr = np.sin(roll_rad)
    cr = np.cos(roll_rad)
    sp = np.sin(pitch_rad)
    cp = np.cos(pitch_rad)
    sy = np.sin(yaw_rad)
    cy = np.cos(yaw_rad)

    r11 = cp * cy
    r12 = sr * sp * cy - cr * sy
    r13 = cr * sp * cy + sr * sy
    r21 = cp * sy
    r22 = sr * sp * sy + cr * cy
    r23 = cr * sp * sy - sr * cy
    r31 = -sp
    r32 = sr * cp
    r33 = cr * cp

    return np.stack([
        np.stack([r11, r12, r13], axis=-1),
        np.stack([r21, r22, r23], axis=-1),
        np.stack([r31, r32, r33], axis=-1),
    ], axis=-2)


def body_from_nav_velocity(vn, ve, vd, roll_rad, pitch_rad, yaw_rad):
    """将 NED 速度转换到机体系速度 v_b = C_bn^T * v_n。"""
    C_bn = euler_to_rotation(roll_rad, pitch_rad, yaw_rad)
    v_n = np.stack([vn, ve, vd], axis=-1)
    return np.einsum('...ji,...j->...i', C_bn, v_n)


def rotate_body_to_nav(vx_body, vy_body, vz_body, roll_rad, pitch_rad, yaw_rad):
    """将 body 系向量旋转到对应 nav 系，v_nav = C_bn * v_body。"""
    C_bn = euler_to_rotation(roll_rad, pitch_rad, yaw_rad)
    v_b = np.stack([vx_body, vy_body, vz_body], axis=-1)
    return np.einsum('...ij,...j->...i', C_bn, v_b)


# ========== 数据加载 ==========

# 新格式列名
NEW_FORMAT_COLS_28 = [
    'timestamp',
    'fused_x', 'fused_y', 'fused_z',
    'fused_vx', 'fused_vy', 'fused_vz',
    'fused_roll', 'fused_pitch', 'fused_yaw',
    'mounting_pitch', 'mounting_yaw', 'odo_scale',
    'sg_x', 'sg_y', 'sg_z',
    'sa_x', 'sa_y', 'sa_z',
    'ba_x', 'ba_y', 'ba_z',
    'bg_x', 'bg_y', 'bg_z',
    'lever_x', 'lever_y', 'lever_z',
]

NEW_FORMAT_COLS_31 = NEW_FORMAT_COLS_28 + [
    'gnss_lever_x', 'gnss_lever_y', 'gnss_lever_z',
]


def detect_format(filepath):
    """根据表头或列数自动检测文件格式，返回 'new' 或 'old'"""
    with open(filepath, 'r') as f:
        first_line = f.readline().strip()
    if first_line.startswith('timestamp') or first_line.startswith('fused'):
        return 'new'
    if first_line.startswith('#'):
        return 'old'
    # 按列数猜测
    parts = first_line.split()
    if len(parts) in (28, 31):
        return 'new'
    return 'old'


def load_data(filepath):
    """加载导航结果数据，自动适配新旧格式"""
    fmt = detect_format(filepath)
    print(f"检测到文件格式: {'ESKF融合结果 (ECEF 28/31列)' if fmt == 'new' else 'KF-GINS旧格式 (LLA 51列)'}")

    if fmt == 'new':
        return _load_new_format(filepath)
    else:
        return _load_old_format(filepath)


def _load_new_format(filepath):
    """加载新格式 SOL 文件"""
    # 判断第一行是否为表头
    with open(filepath, 'r') as f:
        first_line = f.readline().strip()
    has_header = not first_line[0].lstrip('-').replace('.', '', 1).isdigit()

    data = pd.read_csv(filepath, sep=r'\s+',
                       header=0 if has_header else None,
                       comment='#')
    if not has_header:
        if data.shape[1] == len(NEW_FORMAT_COLS_31):
            data.columns = NEW_FORMAT_COLS_31
        elif data.shape[1] == len(NEW_FORMAT_COLS_28):
            data.columns = NEW_FORMAT_COLS_28
        else:
            print("警告: 新格式列数既不是28也不是31，使用默认索引")

    # 派生时间列（相对于起始时刻的秒数）
    data['time_sec'] = data['timestamp'] - data['timestamp'].iloc[0]

    # ECEF -> LLA
    lat, lon, alt = ecef_to_lla(data['fused_x'].values,
                                data['fused_y'].values,
                                data['fused_z'].values)
    data['lat'] = lat
    data['lon'] = lon
    data['alt'] = alt

    # ECEF速度 -> NED速度
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    vn, ve, vd = ecef_vel_to_ned(data['fused_vx'].values,
                                 data['fused_vy'].values,
                                 data['fused_vz'].values,
                                 lat_rad, lon_rad)
    data['vn'] = vn
    data['ve'] = ve
    data['vd'] = vd

    data['_format'] = 'new'
    return data


def _load_old_format(filepath):
    """加载旧格式 KF-GINS 文件"""
    with open(filepath, 'r') as f:
        header_line = f.readline().strip()
        columns = header_line[1:].split() if header_line.startswith('#') else None

    data = pd.read_csv(filepath, comment='#', sep=r'\s+', header=None)
    if columns and len(columns) == data.shape[1]:
        data.columns = columns
    else:
        default_cols = [
            'week', 'sec', 'lat', 'lon', 'alt',
            'vn', 've', 'vd', 'roll', 'pitch', 'yaw',
            'gb_x', 'gb_y', 'gb_z', 'ab_x', 'ab_y', 'ab_z',
            'sg_x', 'sg_y', 'sg_z', 'sa_x', 'sa_y', 'sa_z',
            'IG_rx', 'IG_ry', 'IG_rz',
            'std_lat', 'std_lon', 'std_alt',
            'std_vn', 'std_ve', 'std_vd',
            'std_roll', 'std_pitch', 'std_yaw',
            'std_gb_x', 'std_gb_y', 'std_gb_z',
            'std_ab_x', 'std_ab_y', 'std_ab_z',
            'std_sg_x', 'std_sg_y', 'std_sg_z',
            'std_sa_x', 'std_sa_y', 'std_sa_z',
            'std_IG_rx', 'std_IG_ry', 'std_IG_rz',
        ]
        if len(default_cols) == data.shape[1]:
            data.columns = default_cols

    data['time_sec'] = data['sec'] - data['sec'].iloc[0] if 'sec' in data.columns else np.arange(len(data))
    data['_format'] = 'old'
    return data


# ========== 真值加载 ==========

REF_STYLE = dict(color='gray', linewidth=0.6, alpha=0.7, label='真值', zorder=0)


def find_ref_file(sol_path):
    """根据SOL文件名自动搜索对应的真值文件

    搜索策略:
      1. 配置文件中指定的 pos_path
      2. dataset/data*_converted/POS_converted*.txt
      3. dataset/data*/ref/LC_SM_TXT.nav
    """
    sol_base = os.path.basename(sol_path).lower()
    # 从SOL文件名提取 data* 标识 (e.g. SOL_data4_gnss.txt → data4)
    dataset_id = None
    for part in sol_base.replace('.txt', '').split('_'):
        if part.startswith('data'):
            dataset_id = part
            break

    if dataset_id is None:
        return None

    # 候选路径 (相对于工作目录)
    candidates = [
        f'dataset/{dataset_id}_converted/POS_converted.txt',
        f'dataset/{dataset_id}_converted/POS_converted_*.txt',
        f'dataset/{dataset_id}/ref/LC_SM_TXT.nav',
    ]
    for pattern in candidates:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None


def find_config_file(sol_path):
    """根据 SOL 文件自动寻找对应配置文件。"""
    sol_name = os.path.basename(sol_path)
    stem = os.path.splitext(sol_name)[0]

    if stem.startswith('SOL_'):
        candidate = f'config_{stem[4:]}.yaml'
        if os.path.isfile(candidate):
            return candidate

    for candidate in sorted(glob.glob('config*.yaml')):
        try:
            with open(candidate, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            fusion_output = ((cfg.get('fusion') or {}).get('output_path')) or ''
            generator_output = ((cfg.get('generator') or {}).get('output_path')) or ''
            if os.path.basename(str(fusion_output)) == sol_name:
                return candidate
            if os.path.basename(str(generator_output)) == sol_name:
                return candidate
        except Exception:
            continue
    return None


def load_mounting_base_rpy_deg(config_path):
    """复现 pipeline 中 mounting_base_rpy 的逻辑，返回 [roll, pitch, yaw] (deg)。"""
    if not config_path or not os.path.isfile(config_path):
        return np.zeros(3)

    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    fusion_cfg = cfg.get('fusion') or {}
    constraints_cfg = fusion_cfg.get('constraints') or {}
    init_cfg = fusion_cfg.get('init') or {}

    cfg_mounting = np.array(constraints_cfg.get('imu_mounting_angle', [0.0, 0.0, 0.0]), dtype=float)
    if cfg_mounting.size != 3:
        cfg_mounting = np.zeros(3)

    mounting_base = cfg_mounting.copy()
    use_legacy = bool(init_cfg.get('use_legacy_mounting_base_logic', True))
    init_pitch = float(init_cfg.get('mounting_pitch0', 0.0) or 0.0)
    init_yaw = float(init_cfg.get('mounting_yaw0', 0.0) or 0.0)
    eps = 1e-12
    if use_legacy:
        if abs(init_pitch) > eps:
            mounting_base[1] = 0.0
        if abs(init_yaw) > eps:
            mounting_base[2] = 0.0
    return mounting_base


def maybe_add_vehicle_velocity(data, config_path=None):
    """为新格式数据添加车体 v 系速度列。"""
    if data['_format'].iloc[0] != 'new':
        return None

    required_cols = ['vn', 've', 'vd', 'fused_roll', 'fused_pitch', 'fused_yaw',
                     'mounting_pitch', 'mounting_yaw']
    if not all(col in data.columns for col in required_cols):
        return None

    mounting_base_deg = load_mounting_base_rpy_deg(config_path)
    body_vel = body_from_nav_velocity(
        data['vn'].to_numpy(),
        data['ve'].to_numpy(),
        data['vd'].to_numpy(),
        np.radians(data['fused_roll'].to_numpy()),
        np.radians(data['fused_pitch'].to_numpy()),
        np.radians(data['fused_yaw'].to_numpy()),
    )

    mount_roll_deg = np.full(len(data), mounting_base_deg[0])
    mount_pitch_deg = mounting_base_deg[1] + data['mounting_pitch'].to_numpy()
    mount_yaw_deg = mounting_base_deg[2] + data['mounting_yaw'].to_numpy()
    veh_vel = rotate_body_to_nav(
        body_vel[:, 0], body_vel[:, 1], body_vel[:, 2],
        np.radians(mount_roll_deg),
        np.radians(mount_pitch_deg),
        np.radians(mount_yaw_deg),
    )

    data['vv_x'] = veh_vel[:, 0]
    data['vv_y'] = veh_vel[:, 1]
    data['vv_z'] = veh_vel[:, 2]
    return {
        'config_path': config_path,
        'mounting_base_deg': mounting_base_deg,
    }


def load_ref(filepath):
    """加载真值文件，返回统一格式的DataFrame

    支持两种格式:
      - POS_converted.txt (10列, 有表头): timestamp lat lon h vn ve vd roll pitch yaw
      - LC_SM_TXT.nav     (14列, 无表头): flag timestamp lat lon alt vn ve vd roll pitch yaw ...
    """
    with open(filepath, 'r') as f:
        first_line = f.readline().strip()

    has_header = first_line.startswith('timestamp') or first_line.startswith('lat')

    if has_header:
        ref = pd.read_csv(filepath, sep=r'\s+', header=0)
        # 统一列名
        rename_map = {'h': 'alt'}
        ref.rename(columns=rename_map, inplace=True)
    else:
        ref = pd.read_csv(filepath, sep=r'\s+', header=None)
        ncol = ref.shape[1]
        if ncol == 10:
            # POS_converted 无表头版本: timestamp lat lon alt vn ve vd roll pitch yaw
            ref.columns = ['timestamp', 'lat', 'lon', 'alt',
                           'vn', 've', 'vd', 'roll', 'pitch', 'yaw']
        elif ncol >= 11:
            cols = ['flag', 'timestamp', 'lat', 'lon', 'alt',
                    'vn', 've', 'vd', 'roll', 'pitch', 'yaw']
            if ncol > 11:
                cols += [f'extra_{i}' for i in range(ncol - 11)]
            ref.columns = cols

    # 派生时间
    if 'timestamp' in ref.columns:
        ref['time_sec'] = ref['timestamp'] - ref['timestamp'].iloc[0]
    return ref


# ========== 通用辅助 ==========

def get_time(data):
    """获取以秒为单位的相对时间"""
    return data['time_sec']


def create_figure(title, figsize=(16, 10)):
    fig = plt.figure(figsize=figsize)
    fig.suptitle(title, fontsize=14, fontweight='bold')
    return fig


# ========== 绘图函数 ==========

def plot_position(data, axes, ref=None):
    """绘制位置 (经纬度)"""
    ax1, ax2 = axes
    time = get_time(data)
    if ref is not None:
        rt = get_time(ref)
        ax1.plot(rt, ref['lat'], **REF_STYLE)
        ax2.plot(rt, ref['lon'], **{**REF_STYLE, 'label': ''})
    ax1.plot(time, data['lat'], 'b-', linewidth=0.8, label='融合')
    ax1.set_ylabel('纬度 [deg]')
    ax1.set_title('Latitude')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(time, data['lon'], 'r-', linewidth=0.8)
    ax2.set_ylabel('经度 [deg]')
    ax2.set_title('Longitude')
    ax2.grid(True, alpha=0.3)


def plot_altitude(data, ax, ref=None):
    """绘制高度"""
    time = get_time(data)
    if ref is not None:
        ax.plot(get_time(ref), ref['alt'], **REF_STYLE)
    ax.plot(time, data['alt'], 'g-', linewidth=0.8, label='融合')
    if ref is not None:
        ax.legend(fontsize=8)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('高度 [m]')
    ax.set_title('Altitude')
    ax.grid(True, alpha=0.3)


def plot_trajectory_2d(data, ax, ref=None):
    """绘制二维轨迹 (经度-纬度)"""
    if ref is not None:
        ax.plot(ref['lon'], ref['lat'], **REF_STYLE)
    ax.plot(data['lon'], data['lat'], 'b-', linewidth=0.6, label='融合')
    ax.plot(data['lon'].iloc[0], data['lat'].iloc[0], 'go', markersize=8, label='起点')
    ax.plot(data['lon'].iloc[-1], data['lat'].iloc[-1], 'r^', markersize=8, label='终点')
    ax.set_xlabel('经度 [deg]')
    ax.set_ylabel('纬度 [deg]')
    ax.set_title('2D Trajectory')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')


def plot_velocity(data, axes, ref=None):
    """绘制NED速度"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    for ax, key, color, label in zip(
        [ax1, ax2, ax3],
        ['vn', 've', 'vd'],
        ['b', 'r', 'g'],
        ['北向速度 Vn', '东向速度 Ve', '天向速度 Vd'],
    ):
        if ref is not None and key in ref.columns:
            ax.plot(get_time(ref), ref[key], **REF_STYLE)
        ax.plot(time, data[key], f'{color}-', linewidth=0.8, label='融合')
        ax.set_ylabel(f'{label} [m/s]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
        if ref is not None:
            ax.legend(fontsize=8)
    ax3.set_xlabel('时间 [s]')


def plot_velocity_vehicle_v(data, axes):
    """绘制车体 v 系速度。"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    for ax, key, color, label in zip(
        [ax1, ax2, ax3],
        ['vv_x', 'vv_y', 'vv_z'],
        ['b', 'r', 'g'],
        ['v系前向速度 $v_x^v$', 'v系横向速度 $v_y^v$', 'v系垂向速度 $v_z^v$'],
    ):
        ax.plot(time, data[key], f'{color}-', linewidth=0.8)
        if key in ('vv_y', 'vv_z'):
            ax.axhline(0.0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
        ax.set_ylabel(f'{label} [m/s]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_attitude(data, axes, ref=None):
    """绘制姿态角"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    fmt = data['_format'].iloc[0]
    if fmt == 'new':
        roll_col, pitch_col, yaw_col = 'fused_roll', 'fused_pitch', 'fused_yaw'
    else:
        roll_col, pitch_col, yaw_col = 'roll', 'pitch', 'yaw'

    ref_cols = ['roll', 'pitch', 'yaw']
    for ax, col, ref_key, color, label in zip(
        [ax1, ax2, ax3],
        [roll_col, pitch_col, yaw_col],
        ref_cols,
        ['b', 'r', 'g'],
        ['横滚角 Roll', '俯仰角 Pitch', '航向角 Yaw'],
    ):
        if ref is not None and ref_key in ref.columns:
            ref_vals = ref[ref_key].values.copy()
            # 处理yaw的360°跳变: 将真值yaw统一到与融合值同一范围
            if ref_key == 'yaw':
                fused_mean = data[col].mean()
                ref_vals = ref_vals - 360 * np.round((ref_vals - fused_mean) / 360)
            ax.plot(get_time(ref), ref_vals, **REF_STYLE)
        ax.plot(time, data[col], f'{color}-', linewidth=0.8, label='融合')
        ax.set_ylabel(f'{label} [deg]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
        if ref is not None:
            ax.legend(fontsize=8)
    ax3.set_xlabel('时间 [s]')


def plot_gyro_bias(data, axes):
    """绘制陀螺仪零偏"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    fmt = data['_format'].iloc[0]
    prefix = 'bg' if fmt == 'new' else 'gb'
    # 新格式输出的 bg_* 单位是 rad/s，转换到 deg/h 需乘 180/pi*3600。
    # 旧格式历史字段 gb_* 也是按 rad/s 存储，保持同样换算。
    radps_to_degh = 180.0 / np.pi * 3600.0

    for ax, suffix, color, label in zip(
        [ax1, ax2, ax3], ['x', 'y', 'z'], ['b', 'r', 'g'],
        ['X 陀螺零偏', 'Y 陀螺零偏', 'Z 陀螺零偏'],
    ):
        col = f'{prefix}_{suffix}'
        ax.plot(time, data[col] * radps_to_degh, f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [deg/h]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_accel_bias(data, axes):
    """绘制加速度计零偏"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    fmt = data['_format'].iloc[0]
    prefix = 'ba' if fmt == 'new' else 'ab'
    # 1 mGal = 1e-5 m/s^2，因此 m/s^2 -> mGal 需乘 1e5。
    ms2_to_mgal = 1e5

    for ax, suffix, color, label in zip(
        [ax1, ax2, ax3], ['x', 'y', 'z'], ['b', 'r', 'g'],
        ['X 加计零偏', 'Y 加计零偏', 'Z 加计零偏'],
    ):
        col = f'{prefix}_{suffix}'
        ax.plot(time, data[col] * ms2_to_mgal, f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [mGal]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_scale_factor_gyro(data, axes):
    """绘制陀螺仪比例因子"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    # 新格式 SOL 文件中的 sg_* 已是 ppm；
    # 旧格式中的 sg_* 保持历史约定（无量纲），绘图时换算到 ppm。
    scale = 1.0 if data['_format'].iloc[0] == 'new' else 1e6
    for ax, suffix, color, label in zip(
        [ax1, ax2, ax3], ['x', 'y', 'z'], ['b', 'r', 'g'],
        ['X 陀螺比例因子', 'Y 陀螺比例因子', 'Z 陀螺比例因子'],
    ):
        ax.plot(time, data[f'sg_{suffix}'] * scale, f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [ppm]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_scale_factor_accel(data, axes):
    """绘制加速度计比例因子"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    # 新格式 SOL 文件中的 sa_* 已是 ppm；
    # 旧格式中的 sa_* 保持历史约定（无量纲），绘图时换算到 ppm。
    scale = 1.0 if data['_format'].iloc[0] == 'new' else 1e6
    for ax, suffix, color, label in zip(
        [ax1, ax2, ax3], ['x', 'y', 'z'], ['b', 'r', 'g'],
        ['X 加计比例因子', 'Y 加计比例因子', 'Z 加计比例因子'],
    ):
        ax.plot(time, data[f'sa_{suffix}'] * scale, f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [ppm]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_mount_angles(data, axes):
    """绘制安装角"""
    fmt = data['_format'].iloc[0]
    time = get_time(data)

    if fmt == 'new':
        ax1, ax2 = axes[0], axes[1]
        ax1.plot(time, data['mounting_pitch'], 'r-', linewidth=0.8)
        ax1.set_ylabel('安装俯仰角 [deg]')
        ax1.set_title('Mounting Pitch')
        ax1.grid(True, alpha=0.3)

        ax2.plot(time, data['mounting_yaw'], 'g-', linewidth=0.8)
        ax2.set_ylabel('安装航向角 [deg]')
        ax2.set_title('Mounting Yaw')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlabel('时间 [s]')
    else:
        ax1, ax2, ax3 = axes
        ax1.plot(time, data['IG_rx'] * 3600, 'b-', linewidth=0.8)
        ax1.set_ylabel('Roll安装角 [arcsec]')
        ax1.set_title('IG_rx (不可观)')
        ax1.grid(True, alpha=0.3)

        ax2.plot(time, data['IG_ry'] * 3600, 'r-', linewidth=0.8)
        ax2.set_ylabel('Pitch安装角 [arcsec]')
        ax2.set_title('IG_ry')
        ax2.grid(True, alpha=0.3)

        ax3.plot(time, data['IG_rz'] * 3600, 'g-', linewidth=0.8)
        ax3.set_ylabel('Yaw安装角 [arcsec]')
        ax3.set_title('IG_rz')
        ax3.grid(True, alpha=0.3)
        ax3.set_xlabel('时间 [s]')


def plot_odo_scale(data, ax):
    """绘制里程计比例因子"""
    time = get_time(data)
    ax.plot(time, data['odo_scale'], 'b-', linewidth=0.8)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('里程计比例因子')
    ax.set_title('Odometer Scale Factor')
    ax.grid(True, alpha=0.3)


def plot_lever_arm(data, axes):
    """绘制杆臂"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    for ax, suffix, color, label in zip(
        [ax1, ax2, ax3], ['x', 'y', 'z'], ['b', 'r', 'g'],
        ['X 杆臂', 'Y 杆臂', 'Z 杆臂'],
    ):
        ax.plot(time, data[f'lever_{suffix}'], f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [m]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_gnss_lever_arm(data, axes):
    """绘制GNSS杆臂（若存在）"""
    if not all(c in data.columns for c in ['gnss_lever_x', 'gnss_lever_y', 'gnss_lever_z']):
        return
    ax1, ax2, ax3 = axes
    time = get_time(data)
    for ax, col, color, label in zip(
        [ax1, ax2, ax3],
        ['gnss_lever_x', 'gnss_lever_y', 'gnss_lever_z'],
        ['b', 'r', 'g'],
        ['GNSS X 杆臂', 'GNSS Y 杆臂', 'GNSS Z 杆臂'],
    ):
        ax.plot(time, data[col], f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [m]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


def plot_ecef_position(data, axes):
    """绘制ECEF原始坐标"""
    ax1, ax2, ax3 = axes
    time = get_time(data)
    for ax, col, color, label in zip(
        [ax1, ax2, ax3],
        ['fused_x', 'fused_y', 'fused_z'],
        ['b', 'r', 'g'],
        ['ECEF X', 'ECEF Y', 'ECEF Z'],
    ):
        ax.plot(time, data[col], f'{color}-', linewidth=0.8)
        ax.set_ylabel(f'{label} [m]')
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
    ax3.set_xlabel('时间 [s]')


# ========== 主函数 ==========

def main():
    if len(sys.argv) not in (2, 3):
        print("用法: python plot_navresult.py <结果文件> [配置文件]")
        print("示例: python plot_navresult.py SOL_data4_gnss.txt")
        print("示例: python plot_navresult.py SOL_data4_gnss30_true_iekf.txt config_data4_gnss30_true_iekf.yaml")
        sys.exit(1)

    filepath = sys.argv[1]
    data = load_data(filepath)
    if data is None:
        print("数据加载失败")
        sys.exit(1)

    fmt = data['_format'].iloc[0]
    print(f"已加载 {len(data)} 条记录, 共 {data.shape[1]} 列")
    print(f"列名: {[c for c in data.columns if c != '_format']}")

    # 确定输出目录：从文件名推断 (SOL_data4_gnss.txt -> result_data4)
    basename = os.path.splitext(os.path.basename(filepath))[0]  # e.g. SOL_data4_gnss
    # 尝试提取 data* 部分
    parts = basename.replace('SOL_', '').replace('_gnss', '').replace('_uwb', '')
    out_dir = os.path.join('output', f'result_{parts}') if parts else os.path.join('output', 'result')
    os.makedirs(out_dir, exist_ok=True)
    print(f"图像将保存到: {os.path.abspath(out_dir)}")

    figure_names = []  # (fig, filename) pairs
    figures = []

    # ---- 加载真值 ----
    ref = None
    ref_path = find_ref_file(filepath)
    config_path = sys.argv[2] if len(sys.argv) == 3 else find_config_file(filepath)
    vehicle_vel_meta = maybe_add_vehicle_velocity(data, config_path)
    if vehicle_vel_meta is not None:
        base_roll, base_pitch, base_yaw = vehicle_vel_meta['mounting_base_deg']
        print(f"v系速度计算配置: {vehicle_vel_meta['config_path']}")
        print(f"v系 base mounting rpy [deg]: roll={base_roll:.6f}, pitch={base_pitch:.6f}, yaw={base_yaw:.6f}")
    else:
        print("未生成 v系速度列（可能是旧格式结果或未找到所需安装角信息）")

    if ref_path and os.path.isfile(ref_path):
        ref = load_ref(ref_path)
        print(f"已加载真值: {ref_path} ({len(ref)} 条记录)")
    else:
        print("未找到对应真值文件，仅绘制融合结果")

    # ---- 第1页: 轨迹 + 高度 ----
    fig1 = create_figure('导航结果 — 轨迹与高度')
    ax_traj = fig1.add_subplot(2, 2, (1, 2))
    plot_trajectory_2d(data, ax_traj, ref)
    ax_alt = fig1.add_subplot(2, 2, 3)
    plot_altitude(data, ax_alt, ref)
    if fmt == 'new':
        ax_ecef = fig1.add_subplot(2, 2, 4)
        # 绘制3D轨迹的距离
        dx = data['fused_x'] - data['fused_x'].iloc[0]
        dy = data['fused_y'] - data['fused_y'].iloc[0]
        dz = data['fused_z'] - data['fused_z'].iloc[0]
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        ax_ecef.plot(get_time(data), dist, 'm-', linewidth=0.8)
        ax_ecef.set_xlabel('时间 [s]')
        ax_ecef.set_ylabel('距起点距离 [m]')
        ax_ecef.set_title('Distance from Start')
        ax_ecef.grid(True, alpha=0.3)
    figures.append(fig1)
    figure_names.append((fig1, '01_trajectory_altitude.png'))

    # ---- 第2页: 速度 ----
    fig2 = create_figure('导航结果 — 速度 (NED)')
    axes2 = [fig2.add_subplot(3, 1, i) for i in range(1, 4)]
    plot_velocity(data, axes2, ref)
    figures.append(fig2)
    figure_names.append((fig2, '02_velocity_NED.png'))

    # ---- 第2b页: 车体 v 系速度 ----
    if vehicle_vel_meta is not None:
        fig2b = create_figure('导航结果 — 速度 (车体 v 系)')
        axes2b = [fig2b.add_subplot(3, 1, i) for i in range(1, 4)]
        plot_velocity_vehicle_v(data, axes2b)
        figures.append(fig2b)
        figure_names.append((fig2b, '02b_velocity_vehicle_v.png'))

    # ---- 第3页: 姿态角 ----
    fig3 = create_figure('导航结果 — 姿态角')
    axes3 = [fig3.add_subplot(3, 1, i) for i in range(1, 4)]
    plot_attitude(data, axes3, ref)
    figures.append(fig3)
    figure_names.append((fig3, '03_attitude.png'))

    # ---- 第4页: 陀螺仪零偏 ----
    fig4 = create_figure('导航结果 — 陀螺仪零偏')
    axes4 = [fig4.add_subplot(3, 1, i) for i in range(1, 4)]
    plot_gyro_bias(data, axes4)
    figures.append(fig4)
    figure_names.append((fig4, '04_gyro_bias.png'))

    # ---- 第5页: 加速度计零偏 ----
    fig5 = create_figure('导航结果 — 加速度计零偏')
    axes5 = [fig5.add_subplot(3, 1, i) for i in range(1, 4)]
    plot_accel_bias(data, axes5)
    figures.append(fig5)
    figure_names.append((fig5, '05_accel_bias.png'))

    # ---- 第6页: 陀螺仪比例因子 ----
    fig6 = create_figure('导航结果 — 陀螺仪比例因子')
    axes6 = [fig6.add_subplot(3, 1, i) for i in range(1, 4)]
    plot_scale_factor_gyro(data, axes6)
    figures.append(fig6)
    figure_names.append((fig6, '06_gyro_scale_factor.png'))

    # ---- 第7页: 加速度计比例因子 ----
    fig7 = create_figure('导航结果 — 加速度计比例因子')
    axes7 = [fig7.add_subplot(3, 1, i) for i in range(1, 4)]
    plot_scale_factor_accel(data, axes7)
    figures.append(fig7)
    figure_names.append((fig7, '07_accel_scale_factor.png'))

    # ---- 第8页: 安装角 ----
    if fmt == 'new':
        fig8 = create_figure('导航结果 — 安装角 & 里程计比例因子')
        ax_mp = fig8.add_subplot(3, 1, 1)
        ax_my = fig8.add_subplot(3, 1, 2)
        ax_odo = fig8.add_subplot(3, 1, 3)
        plot_mount_angles(data, [ax_mp, ax_my])
        plot_odo_scale(data, ax_odo)
    else:
        fig8 = create_figure('导航结果 — 安装角')
        axes8 = [fig8.add_subplot(3, 1, i) for i in range(1, 4)]
        plot_mount_angles(data, axes8)
    figures.append(fig8)
    figure_names.append((fig8, '08_mount_angles.png'))

    # ---- 第9页: 杆臂 (仅新格式) ----
    if fmt == 'new':
        fig9 = create_figure('导航结果 — 杆臂估计')
        axes9 = [fig9.add_subplot(3, 1, i) for i in range(1, 4)]
        plot_lever_arm(data, axes9)
        figures.append(fig9)
        figure_names.append((fig9, '09_lever_arm.png'))

    # ---- 第10页: GNSS杆臂 (仅31列新格式) ----
    if fmt == 'new':
        if all(c in data.columns for c in ['gnss_lever_x', 'gnss_lever_y', 'gnss_lever_z']):
            fig10 = create_figure('导航结果 — GNSS 杆臂估计')
            axes10 = [fig10.add_subplot(3, 1, i) for i in range(1, 4)]
            plot_gnss_lever_arm(data, axes10)
            figures.append(fig10)
            figure_names.append((fig10, '10_gnss_lever_arm.png'))

    # ---- 第11页: ECEF原始坐标 (仅新格式) ----
    if fmt == 'new':
        fig11 = create_figure('导航结果 — ECEF 位置')
        axes11 = [fig11.add_subplot(3, 1, i) for i in range(1, 4)]
        plot_ecef_position(data, axes11)
        figures.append(fig11)
        figure_names.append((fig11, '11_ecef_position.png'))

    # ---- 旧格式专属: 标准差页面 ----
    if fmt == 'old' and 'std_lat' in data.columns:
        # 位置标准差
        fig_pstd = create_figure('导航结果 — 位置不确定度 (3σ)')
        axes_pstd = [fig_pstd.add_subplot(3, 1, i) for i in range(1, 4)]
        time = get_time(data)
        for ax, col, color, lbl in zip(
            axes_pstd, ['std_lat', 'std_lon', 'std_alt'], ['b', 'r', 'g'],
            ['北向位置 3σ [m]', '东向位置 3σ [m]', '高度 3σ [m]'],
        ):
            ax.semilogy(time, data[col] * 3, f'{color}-', linewidth=0.8)
            ax.set_ylabel(lbl)
            ax.grid(True, alpha=0.3)
        axes_pstd[-1].set_xlabel('时间 [s]')
        figures.append(fig_pstd)
        figure_names.append((fig_pstd, '09_position_std.png'))

    if fmt == 'old' and 'std_vn' in data.columns:
        fig_vstd = create_figure('导航结果 — 速度不确定度 (3σ)')
        axes_vstd = [fig_vstd.add_subplot(3, 1, i) for i in range(1, 4)]
        time = get_time(data)
        for ax, col, color, lbl in zip(
            axes_vstd, ['std_vn', 'std_ve', 'std_vd'], ['b', 'r', 'g'],
            ['Vn 3σ [m/s]', 'Ve 3σ [m/s]', 'Vd 3σ [m/s]'],
        ):
            ax.semilogy(time, data[col] * 3, f'{color}-', linewidth=0.8)
            ax.set_ylabel(lbl)
            ax.grid(True, alpha=0.3)
        axes_vstd[-1].set_xlabel('时间 [s]')
        figures.append(fig_vstd)
        figure_names.append((fig_vstd, '10_velocity_std.png'))

    if fmt == 'old' and 'std_roll' in data.columns:
        fig_astd = create_figure('导航结果 — 姿态不确定度 (3σ)')
        axes_astd = [fig_astd.add_subplot(3, 1, i) for i in range(1, 4)]
        time = get_time(data)
        for ax, col, color, lbl in zip(
            axes_astd, ['std_roll', 'std_pitch', 'std_yaw'], ['b', 'r', 'g'],
            ['Roll 3σ [deg]', 'Pitch 3σ [deg]', 'Yaw 3σ [deg]'],
        ):
            ax.semilogy(time, np.degrees(data[col]) * 3, f'{color}-', linewidth=0.8)
            ax.set_ylabel(lbl)
            ax.grid(True, alpha=0.3)
        axes_astd[-1].set_xlabel('时间 [s]')
        figures.append(fig_astd)
        figure_names.append((fig_astd, '11_attitude_std.png'))

    # 调整布局并保存
    for fig, fname in figure_names:
        fig.tight_layout(rect=[0, 0.02, 1, 0.95])
        save_path = os.path.join(out_dir, fname)
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"  已保存: {save_path}")

    print(f"\n共保存 {len(figure_names)} 张图像到 {os.path.abspath(out_dir)}")


if __name__ == '__main__':
    main()
