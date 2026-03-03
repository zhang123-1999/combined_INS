import numpy as np
import json

d = np.loadtxt('SOL.txt', skiprows=1)
d = d[~np.isnan(d).any(axis=1)]

r = {}
# 基本维度
r['rows'] = int(d.shape[0])
r['cols'] = int(d.shape[1])

# 各参数的初值和终值
r['odo_scale'] = {'init': round(float(d[0, 12]), 6), 'final': round(float(d[-1, 12]), 6)}
r['mount_pitch_deg'] = {'init': round(float(d[0, 10]), 4), 'final': round(float(d[-1, 10]), 4)}
r['mount_yaw_deg'] = {'init': round(float(d[0, 11]), 4), 'final': round(float(d[-1, 11]), 4)}

# 比例因子 (ppm)
for name, cols in [('sg', [13,14,15]), ('sa', [16,17,18])]:
    r[name + '_ppm'] = {
        'init': [round(float(d[0, c]), 1) for c in cols],
        'final': [round(float(d[-1, c]), 1) for c in cols],
    }

# 分时间段看 odo_scale 收敛
t = d[:, 0]
t0 = t[0]
for nm, s, e in [('t=60s', 60,60), ('t=300s', 300,300), ('t=600s', 600,600), 
                  ('t=1800s', 1800,1800), ('t=3600s', 3599,3599)]:
    idx = np.argmin(np.abs(t - (t0 + s)))
    r[f'odo_scale_{nm}'] = round(float(d[idx, 12]), 4)

# RMSE 信息 (从 ECEF 位置与真值比较)
ref = np.loadtxt('POS_converted.txt', skiprows=1)
t_ref, p_ref = ref[:, 0], ref[:, 1:4]
# 在 sol 时间点插值真值
from scipy.interpolate import interp1d
interp_x = interp1d(t_ref, p_ref[:, 0], fill_value='extrapolate')
interp_y = interp1d(t_ref, p_ref[:, 1], fill_value='extrapolate')
interp_z = interp1d(t_ref, p_ref[:, 2], fill_value='extrapolate')

# 每 1000 个点采样
idx_sample = np.arange(0, len(d), 1000)
p_sol = d[idx_sample, 1:4]
t_sol = d[idx_sample, 0]
p_truth = np.column_stack([interp_x(t_sol), interp_y(t_sol), interp_z(t_sol)])
err = p_sol - p_truth
rmse_xyz = np.sqrt(np.mean(err**2, axis=0))
r['rmse_m'] = {'x': round(float(rmse_xyz[0]), 1), 'y': round(float(rmse_xyz[1]), 1), 'z': round(float(rmse_xyz[2]), 1)}
r['rmse_3d'] = round(float(np.sqrt(np.sum(rmse_xyz**2))), 1)

# 位置末端误差
p_end_sol = d[-1, 1:4]
p_end_ref = np.array([interp_x(t[-1]), interp_y(t[-1]), interp_z(t[-1])])
end_err = np.linalg.norm(p_end_sol - p_end_ref)
r['end_pos_error_m'] = round(float(end_err), 1)

with open('analysis_result.json', 'w') as f:
    json.dump(r, f, indent=2)
print("done")
