"""分析车体系下 down 方向速度波动的来源"""
import numpy as np
import json

sol = np.loadtxt('SOL.txt', skiprows=1)
sol = sol[~np.isnan(sol).any(axis=1)]
t = sol[:, 0]
v_ecef = sol[:, 4:7]
p_ecef = sol[:, 1:4]
euler_deg = sol[:, 7:10]  # roll, pitch, yaw (deg)
t0 = t[0]

# 参考数据
ref = np.loadtxt('POS_converted.txt', skiprows=1)
t_ref = ref[:, 0]

def ecef_to_lla(p):
    x, y, z = p
    kA, kE2 = 6378137.0, 6.69437999014e-3
    lon = np.arctan2(y, x)
    p_dist = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p_dist * (1.0 - kE2))
    for _ in range(5):
        sl = np.sin(lat)
        N = kA / np.sqrt(1.0 - kE2 * sl * sl)
        lat = np.arctan2(z + kE2 * N * sl, p_dist)
    return lat, lon

def calc_Rne(lat, lon):
    sl, cl = np.sin(lat), np.cos(lat)
    sn, cn = np.sin(lon), np.cos(lon)
    return np.array([[-sl*cn, -sn, -cl*cn],
                     [-sl*sn,  cn, -cl*sn],
                     [ cl,     0,  -sl]])

def body_vel(v_ecef, p_ecef, eul_deg):
    """返回车体系（body frame）的速度 [forward, right, down]"""
    lat, lon = ecef_to_lla(p_ecef)
    R_ne = calc_Rne(lat, lon)
    v_ned = R_ne.T @ v_ecef
    r, p, y = np.deg2rad(eul_deg)
    # Euler to DCM: R_b^n
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rnb = np.array([
        [cy*cp, sy*cp, -sp],
        [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
        [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
    ])
    v_b = Rnb @ v_ned  # [forward, right, down]
    return v_b

# 安装角
mount_pitch_deg = sol[:, 10]
mount_yaw_deg = sol[:, 11]
fixed_roll_deg = 0.0

# C_b_v 矩阵 (从 body 到 vehicle)
def calc_C_b_v(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, sy*cp, -sp],
        [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
        [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
    ])

# 采样分析 (每200个点采一个，约200Hz下每秒一个)
step = 200
idx = np.arange(0, len(sol), step)

v_body_fwd = []
v_body_right = []
v_body_down = []
v_ned_d = []
pitch_list = []
v_veh_down = []

for i in idx:
    v_b = body_vel(v_ecef[i], p_ecef[i], euler_deg[i])
    v_body_fwd.append(v_b[0])
    v_body_right.append(v_b[1])
    v_body_down.append(v_b[2])
    
    # NED 速度
    lat, lon = ecef_to_lla(p_ecef[i])
    R_ne = calc_Rne(lat, lon)
    v_ned = R_ne.T @ v_ecef[i]
    v_ned_d.append(v_ned[2])
    
    # Vehicle frame velocity
    C_bv = calc_C_b_v(fixed_roll_deg, mount_pitch_deg[i], mount_yaw_deg[i])
    r, p, y = np.deg2rad(euler_deg[i])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rnb = np.array([
        [cy*cp, sy*cp, -sp],
        [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
        [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
    ])
    v_b_full = Rnb @ v_ned
    v_v = C_bv @ v_b_full
    v_veh_down.append(v_v[2])
    pitch_list.append(euler_deg[i, 1])

v_body_fwd = np.array(v_body_fwd)
v_body_right = np.array(v_body_right)
v_body_down = np.array(v_body_down)
v_ned_d = np.array(v_ned_d)
v_veh_down = np.array(v_veh_down)
pitch_list = np.array(pitch_list)
t_s = t[idx] - t0

results = {
    'v_body_down': {
        'mean': round(float(v_body_down.mean()), 4),
        'std': round(float(v_body_down.std()), 4),
        'min': round(float(v_body_down.min()), 4),
        'max': round(float(v_body_down.max()), 4),
        'abs_mean': round(float(np.abs(v_body_down).mean()), 4),
    },
    'v_body_right': {
        'mean': round(float(v_body_right.mean()), 4),
        'std': round(float(v_body_right.std()), 4),
        'min': round(float(v_body_right.min()), 4),
        'max': round(float(v_body_right.max()), 4),
    },
    'v_veh_down': {
        'mean': round(float(v_veh_down.mean()), 4),
        'std': round(float(v_veh_down.std()), 4),
        'min': round(float(v_veh_down.min()), 4),
        'max': round(float(v_veh_down.max()), 4),
    },
    'v_ned_down': {
        'mean': round(float(v_ned_d.mean()), 4),
        'std': round(float(v_ned_d.std()), 4),
        'min': round(float(v_ned_d.min()), 4),
        'max': round(float(v_ned_d.max()), 4),
    },
    'pitch_deg': {
        'mean': round(float(pitch_list.mean()), 4),
        'std': round(float(pitch_list.std()), 4),
        'min': round(float(pitch_list.min()), 4),
        'max': round(float(pitch_list.max()), 4),
    },
    # down 速度和前向速度的相关性
    'corr_down_fwd': round(float(np.corrcoef(v_body_down, v_body_fwd)[0, 1]), 4),
    'corr_down_pitch': round(float(np.corrcoef(v_body_down, pitch_list)[0, 1]), 4),
    # NHC约束的 v_veh 和 v_body 的关系
    'note': 'v_body_down = body frame down vel, v_veh_down = vehicle frame down vel (NHC constrains this)',
    # 运动时 down 速度
    'moving_v_body_down': {
        'mask': 'v_fwd > 3 m/s',
        'mean': round(float(v_body_down[v_body_fwd > 3].mean()), 4),
        'std': round(float(v_body_down[v_body_fwd > 3].std()), 4),
    },
    'moving_v_veh_down': {
        'mask': 'v_fwd > 3 m/s',
        'mean': round(float(v_veh_down[v_body_fwd > 3].mean()), 4),
        'std': round(float(v_veh_down[v_body_fwd > 3].std()), 4),
    },
}

with open('down_vel_analysis.json', 'w') as f:
    json.dump(results, f, indent=2)

print("done")
