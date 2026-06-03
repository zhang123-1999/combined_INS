# Technical Notes

This directory keeps concise supporting notes for the navigation project. Long
development logs, external reference projects, and one-off debugging records
were intentionally removed from the display branch.

## Core Pipeline

1. `apps/eskf_fusion_main.cpp` parses `--config <yaml>`.
2. `src/app/config.cpp` loads fusion options, runtime phases, schedules, and
   filter switches.
3. `src/app/dataset_loader.cpp` loads IMU, reference, GNSS, ODO, and optional
   UWB data.
4. `src/app/initialization.cpp` builds the nominal state and covariance.
5. `src/app/fusion_runtime.cpp` drives IMU propagation and scheduled
   measurement updates.
6. `src/navigation/` implements standard ESKF and InEKF process/measurement
   semantics.
7. `src/app/evaluation.cpp` writes the 31-column navigation solution.

## Input Summary

- IMU: `timestamp dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z`
- GNSS position: `timestamp pos(3) std(3)`
- GNSS position and velocity: `timestamp pos(3) std(3) vel(3) vel_std(3)`
- ODO: `timestamp v_forward`
- UWB: `timestamp dist_1 dist_2 ... dist_N`

Positions can be supplied as LLA or ECEF where supported by the loader. The
runtime converts data internally to the coordinate representation required by
the filter.

## Main Experiment Configs

- `configs/data2_ins_gnss_odo_nhc_eskf_large_initial_error.yaml`
- `configs/data2_ins_gnss_odo_nhc_inekf_large_initial_error.yaml`

Both configs use the same INS/GNSS/ODO/NHC measurement setup and GNSS outage
schedule. The only intended comparison switch is `fusion.inekf.enable`.
