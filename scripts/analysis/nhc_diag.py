"""诊断 NHC 约束的实际效果：对比 NHC 更新前后的速度变化"""
import numpy as np
import json

# 核心问题：NHC 应该强约束 right/down，但没有。
# 可能原因：
# 1. Kalman 增益 K 太小（P 太小 或 R 太大）
# 2. NHC 施加在 vehicle frame，但分析看的是 body frame
# 3. 姿态误差让 NHC 的修正方向不对

# 分析方法：检查 v_veh (NHC 直接约束的量) vs v_body (我们观察的量)

sol = np.loadtxt('SOL.txt', skiprows=1)
sol = sol[~np.isnan(sol).any(axis=1)]
t = sol[:, 0] - sol[0, 0]
v_ecef = sol[:, 4:7]
p_ecef = sol[:, 1:4]
euler_deg = sol[:, 7:10]
mp_deg = sol[:, 10]
my_deg = sol[:, 11]

def ecef_to_lla(p):
    x, y, z = p
    kA, kE2 = 6378137.0, 6.69437999014e-3
    lon = np.arctan2(y, x)
    pd = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, pd * (1.0 - kE2))
    for _ in range(5):
        sl = np.sin(lat)
        N = kA / np.sqrt(1.0 - kE2 * sl * sl)
        lat = np.arctan2(z + kE2 * N * sl, pd)
    return lat, lon

def calc_Rne(lat, lon):
    sl, cl = np.sin(lat), np.cos(lat)
    sn, cn = np.sin(lon), np.cos(lon)
    return np.array([[-sl*cn, -sn, -cl*cn], [-sl*sn, cn, -cl*sn], [cl, 0, -sl]])

def euler_to_dcm(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, sy*cp, -sp],
        [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
        [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
    ])

step = 200
idx = np.arange(0, len(sol), step)

v_veh_y = []
v_veh_z = []
v_body_y = []
v_body_z = []
v_fwd = []

for i in idx:
    lat, lon = ecef_to_lla(p_ecef[i])
    R_ne = calc_Rne(lat, lon)
    v_ned = R_ne.T @ v_ecef[i]
    
    # attitude DCM (NED -> body)
    r, p, y = np.deg2rad(euler_deg[i])
    R_nb = euler_to_dcm(r, p, y)  # body = R_nb * ned
    v_b = R_nb @ v_ned
    
    # mounting DCM (body -> vehicle)
    C_bv = euler_to_dcm(0, np.deg2rad(mp_deg[i]), np.deg2rad(my_deg[i]))
    v_v = C_bv @ v_b
    
    v_body_y.append(v_b[1])
    v_body_z.append(v_b[2])
    v_veh_y.append(v_v[1])
    v_veh_z.append(v_v[2])
    v_fwd.append(v_b[0])

v_body_y = np.array(v_body_y)
v_body_z = np.array(v_body_z)
v_veh_y = np.array(v_veh_y)
v_veh_z = np.array(v_veh_z)
v_fwd = np.array(v_fwd)

# 运动时 (v_fwd > 3)
mov = v_fwd > 3.0

results = {
    'all_epochs': {
        'v_body_y': {'mean': round(float(v_body_y.mean()), 4), 'std': round(float(v_body_y.std()), 4)},
        'v_body_z': {'mean': round(float(v_body_z.mean()), 4), 'std': round(float(v_body_z.std()), 4)},
        'v_veh_y':  {'mean': round(float(v_veh_y.mean()), 4),  'std': round(float(v_veh_y.std()), 4)},
        'v_veh_z':  {'mean': round(float(v_veh_z.mean()), 4),  'std': round(float(v_veh_z.std()), 4)},
    },
    'moving_only': {
        'v_body_y': {'mean': round(float(v_body_y[mov].mean()), 4), 'std': round(float(v_body_y[mov].std()), 4)},
        'v_body_z': {'mean': round(float(v_body_z[mov].mean()), 4), 'std': round(float(v_body_z[mov].std()), 4)},
        'v_veh_y':  {'mean': round(float(v_veh_y[mov].mean()), 4),  'std': round(float(v_veh_y[mov].std()), 4)},
        'v_veh_z':  {'mean': round(float(v_veh_z[mov].mean()), 4),  'std': round(float(v_veh_z[mov].std()), 4)},
    },
    'mounting_angles_deg': {
        'pitch_mean': round(float(mp_deg.mean()), 4),
        'yaw_mean': round(float(my_deg.mean()), 4),
        'yaw_init': round(float(my_deg[0]), 4),
        'yaw_final': round(float(my_deg[-1]), 4),
    },
    # body vs vehicle 差异 = 安装角旋转的效果
    'body_to_veh_diff_y': round(float((v_veh_y - v_body_y).mean()), 4),
    'body_to_veh_diff_z': round(float((v_veh_z - v_body_z).mean()), 4),
}

with open('nhc_diag.json', 'w') as f:
    json.dump(results, f, indent=2)
print("done")
