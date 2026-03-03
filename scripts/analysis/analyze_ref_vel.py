
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d
import os

def euler_to_rot(r, p, y):
    # Rad to Matrix (Body to Nav, ZYX sequence typical for NED)
    # C_b^n = Rz(y) Ry(p) Rx(r)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    return Rz @ Ry @ Rx

def main():
    # 1. Parameters
    l_gnss_b = np.array([0.505, -0.145, -1.105]) # GNSS lever arm in Body frame
    mounting_deg = np.array([0.0, -0.532, 1.38]) # IMU to Vehicle mounting angles
    
    # Construct C_b_v (Rotation from IMU Body to Vehicle)
    # "对应矩阵为C_b^v" - Assuming typical Euler sequence ZYX
    rm_r, rm_p, rm_y = np.radians(mounting_deg)
    C_b_v = euler_to_rot(rm_r, rm_p, rm_y) 
    
    print(f"Lever Arm (GNSS): {l_gnss_b}")
    print(f"Mounting Angles (deg): {mounting_deg}")
    print(f"C_b_v:\n{C_b_v}")

    # 2. Load REF_NAV (Ground Truth)
    # Filename: REF_NAV.txt
    # Format: ID Time Lat Lon H Vn Ve Vd Roll Pitch Yaw
    print("Loading REF_NAV.txt...")
    cols = ['id', 't', 'lat', 'lon', 'h', 'vn', 've', 'vd', 'roll', 'pitch', 'yaw']
    try:
        ref_df = pd.read_csv('REF_NAV.txt', delim_whitespace=True, names=cols, header=None)
    except Exception as e:
        print(f"Error loading REF_NAV.txt: {e}")
        return

    # Filter time range [527000, 530626]
    ref_df = ref_df[(ref_df['t'] >= 527000) & (ref_df['t'] <= 530626)].copy()
    print(f"Filtered REF_NAV data: {len(ref_df)} rows")

    if len(ref_df) == 0:
        print("No REF_NAV data in range.")
        return

    # 3. Load IMU for angular velocity (omega)
    # Filename: IMU_converted.txt
    # Format: timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z
    print("Loading IMU_converted.txt...")
    try:
        imu_df = pd.read_csv('IMU_converted.txt', delim_whitespace=True)
    except Exception as e:
        print(f"Error loading IMU_converted.txt: {e}")
        return

    # Calculate omega = dtheta / dt
    t_imu = imu_df['timestamp'].values
    dtheta = imu_df[['dtheta_x', 'dtheta_y', 'dtheta_z']].values
    
    # Calculate dt (assuming roughly constant or diff)
    dt = np.diff(t_imu, prepend=t_imu[0]-0.005)
    # Filter valid dt
    dt = np.where(dt > 1e-6, dt, 0.005) 
    
    omega = dtheta / dt[:, None] # rad/s
    
    # Interpolate omega to REF_NAV timestamps
    print("Interpolating omega...")
    f_interp = interp1d(t_imu, omega, axis=0, bounds_error=False, fill_value="extrapolate")
    omega_ref = f_interp(ref_df['t'].values)

    # 4. Compute Velocities
    v_n_gnss_list = []
    v_v_uncorrected_list = [] # Without lever arm compensation
    v_v_corrected_list = []   # With lever arm compensation
    
    print("Computing frames...")
    
    t_ref = ref_df['t'].values
    vn_gnss_all = ref_df[['vn', 've', 'vd']].values
    att_deg = ref_df[['roll', 'pitch', 'yaw']].values
    
    for i in range(len(ref_df)):
        # 4a. Ground Truth Nav Velocity
        v_n_gnss = vn_gnss_all[i]
        
        # 4b. Rotation Matrix C_b_n
        r, p, y = np.radians(att_deg[i])
        C_b_n = euler_to_rot(r, p, y)
        
        # 4c. Lever Arm Correction
        # v_gnss = v_imu + C_b^n (omega x l)
        # v_imu_n = v_gnss_n - C_b^n (omega x l)
        w_b = omega_ref[i]
        wx_l = np.cross(w_b, l_gnss_b)
        v_lever_n = C_b_n @ wx_l
        
        v_n_imu = v_n_gnss - v_lever_n
        
        # 4d. Project to Body and Vehicle
        v_b_imu = C_b_n.T @ v_n_imu  # Nav to Body
        v_v_imu = C_b_v @ v_b_imu    # Body to Vehicle (final result)
        
        # Uncorrected check (what if we ignored lever arm?)
        v_b_unc = C_b_n.T @ v_n_gnss
        v_v_unc = C_b_v @ v_b_unc
        
        v_v_corrected_list.append(v_v_imu)
        v_v_uncorrected_list.append(v_v_unc)
        
    v_v_corr = np.array(v_v_corrected_list)
    v_v_unc = np.array(v_v_uncorrected_list)
    
    # 5. Plotting
    print("Plotting...")
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ['Forward (x)', 'Right (y)', 'Down (z)']
    colors = ['tab:blue', 'tab:orange', 'tab:green']
    
    for i in range(3):
        axes[i].plot(t_ref, v_v_corr[:, i], label='Corrected (Lever Arm + Mounting)', color=colors[i], linewidth=1)
        axes[i].plot(t_ref, v_v_unc[:, i], label='Uncorrected (Raw GNSS rotated)', color='gray', linestyle='--', alpha=0.5, linewidth=0.8)
        axes[i].set_ylabel(f'Vel {labels[i]} [m/s]')
        axes[i].set_title(f"Truth Vehicle Velocity - {labels[i]}")
        axes[i].grid(True, linestyle='--', alpha=0.7)
        axes[i].legend(loc='upper right')
        
        # Calculate stats
        mean_v = np.mean(v_v_corr[:, i])
        std_v = np.std(v_v_corr[:, i])
        axes[i].text(0.02, 0.9, f"Mean: {mean_v:.3f}\nStd: {std_v:.3f}", transform=axes[i].transAxes, bbox=dict(facecolor='white', alpha=0.7))

    axes[2].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.savefig('ref_body_vel_analysis.png', dpi=300)
    print("Saved ref_body_vel_analysis.png")
    
    # 6. Detailed Down Velocity Analysis
    plt.figure(figsize=(12, 6))
    plt.plot(t_ref, v_v_corr[:, 2], label='Down Vel (Corrected)', color='green', linewidth=1)
    # Add rolling mean/std to see fluctuations clearer
    # window = 100 # 0.5s approx if 200Hz? No ref is 100Hz?
    # pd_v_down = pd.Series(v_v_corr[:, 2])
    # plt.plot(t_ref, pd_v_down.rolling(50).mean(), label='Rolling Mean (0.5s)', color='black', linewidth=0.8)
    
    plt.title("Truth Vehicle Down Velocity Analysis")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend()
    plt.savefig('ref_down_vel_detail.png', dpi=300)
    print("Saved ref_down_vel_detail.png")

    # 7. Print stats JSON for agent to read
    import json
    stats = {
        "x_mean": float(np.mean(v_v_corr[:, 0])),
        "x_std": float(np.std(v_v_corr[:, 0])),
        "y_mean": float(np.mean(v_v_corr[:, 1])),
        "y_std": float(np.std(v_v_corr[:, 1])),
        "z_mean": float(np.mean(v_v_corr[:, 2])),
        "z_std": float(np.std(v_v_corr[:, 2])),
        "y_unc_std": float(np.std(v_v_unc[:, 1])),
        "z_unc_std": float(np.std(v_v_unc[:, 2]))
    }
    with open('ref_body_vel.json', 'w') as f:
        json.dump(stats, f, indent=2)
    print("Stats saved to ref_body_vel.json")

if __name__ == "__main__":
    main()
