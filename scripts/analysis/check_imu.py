import numpy as np
import json

imu = np.loadtxt('IMU_converted.txt', skiprows=1)
dt = float(np.median(np.diff(imu[:, 0])))

result = {}
result['dt'] = dt
result['n_samples'] = len(imu)

names = ['dtheta_x','dtheta_y','dtheta_z','dvel_x','dvel_y','dvel_z']
for j, name in enumerate(names):
    result[f'mean_{name}'] = float(imu[:, j+1].mean())
    result[f'accel_{name}'] = float(imu[:, j+1].mean() / dt)

static_mask = imu[:, 0] < imu[0, 0] + 10
result['static_n'] = int(static_mask.sum())
for j, name in enumerate(names):
    result[f'static_mean_{name}'] = float(imu[static_mask, j+1].mean())
    result[f'static_accel_{name}'] = float(imu[static_mask, j+1].mean() / dt)

with open('check_imu_result.json', 'w') as f:
    json.dump(result, f, indent=2)

print("Done")
