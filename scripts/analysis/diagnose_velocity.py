"""简化诊断脚本：快速比较INS与真值速度比值"""
import numpy as np

sol = np.loadtxt('SOL.txt', skiprows=1)
sol = sol[~np.isnan(sol).any(axis=1)]
t_sol, v_sol = sol[:, 0], sol[:, 4:7]
odo_scale = sol[:, 12]

odo = np.loadtxt('iODO.txt')
t_odo, v_odo = odo[:, 0], odo[:, 1]

ref = np.loadtxt('POS_converted.txt', skiprows=1)
t_ref, v_ref = ref[:, 0], ref[:, 4:7]

# 计算INS和真值的速度大小
v_ins_mag = np.sqrt(np.sum(v_sol**2, axis=1))
v_ref_mag = np.sqrt(np.sum(v_ref**2, axis=1))

# 时间同步和统计
t0 = t_sol[0]
results = {}
for nm, s, e in [('0-300s', 0, 300), ('300-1000s', 300, 1000), 
                  ('1000-2000s', 1000, 2000), ('2000-3600s', 2000, 3600)]:
    ms = (t_sol >= t0 + s) & (t_sol <= t0 + e)
    mo = (t_odo >= t0 + s) & (t_odo <= t0 + e) & (v_odo > 3.0)
    
    if ms.sum() < 10 or mo.sum() < 10:
        continue
        
    # INS速度大小平均
    ins_avg = v_ins_mag[ms].mean()
    
    # 真值速度插值后平均
    ref_interp = np.interp(t_sol[ms], t_ref, v_ref_mag)
    ref_avg = ref_interp.mean()
    
    # ODO平均  
    odo_avg = v_odo[mo].mean()
    
    # ODO scale平均
    scale_avg = odo_scale[ms].mean()
    
    results[nm] = {
        'INS_speed': round(ins_avg, 2),
        'REF_speed': round(ref_avg, 2),
        'ODO_speed': round(odo_avg, 2),
        'INS/REF': round(ins_avg/ref_avg, 4),
        'REF/ODO': round(ref_avg/odo_avg, 4),
        'INS/ODO': round(ins_avg/odo_avg, 4),
        'odo_scale': round(scale_avg, 4),
    }

import json
print(json.dumps(results, indent=2))
