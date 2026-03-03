import numpy as np
import json

sol = np.loadtxt('SOL.txt', skiprows=1)
sol = sol[~np.isnan(sol).any(axis=1)]
t_sol, v_sol, p_sol = sol[:, 0], sol[:, 4:7], sol[:, 1:4]
euler, odo_sc = sol[:, 7:10], sol[:, 12]

odo = np.loadtxt('iODO.txt')
t_odo, v_odo = odo[:, 0], odo[:, 1]

ref = np.loadtxt('POS_converted.txt', skiprows=1)
t_ref = ref[:, 0]
v_ref_speed = np.sqrt(np.sum(ref[:, 4:7]**2, axis=1))

def body_fwd_speed(v_ecef, p_ecef, eul_deg):
    x, y, z = p_ecef
    kA, kE2 = 6378137.0, 6.69437999014e-3
    p = np.sqrt(x*x + y*y)
    lon = np.arctan2(y, x)
    lat = np.arctan2(z, p * (1.0 - kE2))
    for _ in range(5):
        sl = np.sin(lat)
        N = kA / np.sqrt(1.0 - kE2 * sl * sl)
        lat = np.arctan2(z + kE2 * N * sl, p)
    sl, cl = np.sin(lat), np.cos(lat)
    sn, cn = np.sin(lon), np.cos(lon)
    R_ne = np.array([[-sl*cn, -sn, -cl*cn],
                     [-sl*sn,  cn, -cl*sn],
                     [ cl,     0,  -sl]])
    v_ned = R_ne.T @ v_ecef
    r, pp, yy = np.deg2rad(eul_deg)
    return np.cos(yy)*np.cos(pp)*v_ned[0] + np.sin(yy)*np.cos(pp)*v_ned[1] - np.sin(pp)*v_ned[2]

t0 = t_sol[0]
results = {}
for nm, s, e in [('0-60s', 0, 60), ('60-300s', 60, 300), ('300-600s', 300, 600),
                  ('600-1800s', 600, 1800), ('1800-3600s', 1800, 3600)]:
    mo = (t_odo >= t0 + s) & (t_odo <= t0 + e) & (v_odo > 3.0)
    if mo.sum() < 5:
        continue
    to = t_odo[mo]
    vo = v_odo[mo]
    vf_list = []
    for t in to:
        idx = np.argmin(np.abs(t_sol - t))
        vf_list.append(body_fwd_speed(v_sol[idx], p_sol[idx], euler[idx]))
    vf = np.array(vf_list)
    vr = np.interp(to, t_ref, v_ref_speed)
    os_ = np.interp(to, t_sol, odo_sc).mean()
    results[nm] = {
        'INS_fwd': round(float(vf.mean()), 1),
        'REF_spd': round(float(vr.mean()), 1),
        'ODO': round(float(vo.mean()), 1),
        'INS_ODO': round(float(vf.mean()/vo.mean()), 3),
        'REF_ODO': round(float(vr.mean()/vo.mean()), 3),
        'odo_scale': round(float(os_), 3),
    }

with open('check_odo_result2.json', 'w') as f:
    json.dump(results, f, indent=2)
print("done")
