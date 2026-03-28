# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-26`

## Project Snapshot

- long_term_objective: 比较不同组合导航算法，并研究不同传感器调度/约束下各状态块的可观性。
- current_phase_objective: 在 `strict weak-excitation gate + phase2/phase3 freeze GNSS lever + fusion.init.odo_scale=1.0` 的 staged current mainline 上，`EXP-20260325-data2-nhc-admission-alignment-r1`、`EXP-20260325-data2-odo-reference-semantics-audit-r1` 与 `EXP-20260325-data2-raw-odo-truth-speed-audit-r1` 已分别排除 `NHC admission mismatch`、当前 `ODO residual / v_vehicle` 语义错误与“原始 ODO 实为 pulse count”三条解释线；此前 full-window `EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1` 的 `G5` 已证明去掉 `sg/sa(15-20)` 只能缓和、不能切断姿态-零偏耦合。当前三段式研究继续沿用 staged `G5` 的 `90 s / 90 s` phase3 周期断星。`EXP-20260325-data2-staged-g5-no-imu-scale-r1` 曾确认 `4x/5x/6x` 下 `phase2 RMSE3D` 几乎不变，而 `phase3 RMSE3D` 随 `ODO/NHC` 降权单调改善到 `3.178389 m`。随后 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2` 又表明 total-zero-init mounting 到 `phase2_end` 也只学到 `yaw≈0.896198 deg`，问题不在 mounting `Q` 倍率。latest fresh 的 `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 在原 `4x/5x/6x` 基础上新增了 phase3 `INS/GNSS outage` 对照，即 phase1/phase2 保持 `6x` 基线一致、phase3 显式关闭 `ODO/NHC`。结果显示纯对照组并没有把 90 s 漂移拉回 `~2 m`，反而出现 `phase3 RMSE3D=31.657637 m`，首个 `gnss_off_01` 就达到 `RMSE3D=46.206528 m`、`final_err_3d=112.485737 m`；而同一 `6x` 基线保留 `ODO/NHC` 时对应值只有 `phase3 RMSE3D=3.178389 m`、`gnss_off_01 RMSE3D=5.220606 m`。本轮 `EXP-20260326-data2-staged-g5-6x-interactive-report-r1` 已在此 fresh artifact 之上完成第二次网页重构：单页 Plotly 离线 HTML 改为紧凑的 LaTeX/IEEE 风格，非 PVA 部分改成 `22` 维（移除 `mount_state_*` 中间安装角状态项），`ba/bg` 不再叠加真值线，`ODO/NHC` 校准项统一改成误差曲线，便于后续机理分析与汇报取图。
- last_completed_session: `20260326-0125-data2-staged-g5-6x-report-restyle`
- last_fresh_solver_session: `20260326-0012-data2-staged-g5-phase3-ins-only-control`
- recent_retained_sessions: `20260326-0125-data2-staged-g5-6x-report-restyle`、`20260326-0105-data2-staged-g5-6x-interactive-report`、`20260326-0012-data2-staged-g5-phase3-ins-only-control`、`20260325-2340-data2-staged-g5-total-zero-init-fix`、`20260325-2318-data2-staged-g5-zero-init-vs-total-output-clarification`
- current_experiment_context: `EXP-20260326-data2-staged-g5-6x-interactive-report-r1`、`EXP-20260325-data2-staged-g5-no-imu-scale-r2`、`EXP-20260325-data2-staged-g5-no-imu-scale-r1`、`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2`、`EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1`、`EXP-20260325-data2-bgz-route-decomposition-r1`、`EXP-20260325-data2-bgz-jacobian-fd-audit-r1`、`EXP-20260325-data2-nhc-admission-alignment-r1`、`EXP-20260325-data2-odo-reference-semantics-audit-r1`、`EXP-20260325-data2-raw-odo-truth-speed-audit-r1`、`EXP-20260325-data2-bgz-state-competition-r1`、`EXP-20260325-data2-bgz-mitigation-validation-r1`、`EXP-20260325-data2-bgz-mitigation-validation-r2`、`EXP-20260325-data2-bgz-mitigation-validation-r3`
- open_hypotheses: `HYP-41`、`HYP-43`、`HYP-47`、`HYP-48`、`HYP-49`、`HYP-50`、`HYP-52`、`HYP-53`、`HYP-54`
- active_archive_docs: `docs/project_history/walkthrough_preservation_snapshot_20260313.md`、`docs/project_history/walkthrough_archive_registry_20260313.md`、`docs/project_history/walkthrough_archive_sessions_20260313.md`、`docs/project_history/walkthrough_archive_sessions_20260318.md`、`docs/project_history/walkthrough_archive_sessions_20260320.md`、`docs/project_history/artifact_relocation_index.md`
- pending_next_actions: 见文末 `## Next Actions`
- compaction_note: 本次已按清理要求压缩主档，移除 `phase2_gnss_transition_guard / fixed5s_position_only` 这条已废弃分支的详表与详记；旧历史只保留必要结论与 archive 追溯入口。

### Current Baseline Facts

- state_dimension: `kStateDim = 31`
- filter_modes: 标准 ESKF=`fusion.fej.enable:false`；InEKF switch=`fusion.fej.enable:true`
- official_baseline_config: `config_data2_baseline_eskf.yaml`
- official_output_dir: `output/data2_eskf_baseline/`
- official_gnss_source: `dataset/data2/rtk.txt`（7 列，仅位置和位置标准差）
- official_constraints: `fusion.enable_gnss_velocity=false`、`fusion.ablation.disable_mounting_roll=true`、`fusion.ablation.disable_gnss_lever_z=true`
- runtime_anchor_switch: `fusion.init.runtime_truth_anchor_pva=false`
- current_bias_scale_init_std_human:
  - `ba=[2250,225,112.5] mGal`
  - `bg=[150,112.5,150] deg/h`
  - `sg=[1500,2250,6000] ppm`
  - `sa=[3900,3150,1350] ppm`
- freshness_caveat: 上述 prior 已同步到 config，但 `output/data2_eskf_baseline/` 尚未按当前 prior fresh 重跑。

### State Blocks

- `p(3), v(3), att(3), ba(3), bg(3), sg(3), sa(3), odo_scale(1), mounting(3), lever_odo(3), lever_gnss(3)`

### Pipeline Order

1. Predict (IMU)
2. ZUPT
3. Gravity alignment diagnostics
4. NHC
5. ODO
6. UWB
7. GNSS（可带 `fusion.gnss_schedule.*` 与 `fusion.post_gnss_ablation.*`）
8. Diagnostics + result recording

## Experiment Registry

| exp_id | date | purpose | config | key artifacts | key metrics | status | freshness |
|---|---|---|---|---|---|---|---|
| `EXP-20260324-data2-staged-truth-ablation-r1` | `2026-03-24` | 在当前 bug-fixed staged baseline 上比较 `control`、`fix_mounting_truth`、`fix_odo_lever_truth`，判断主崩坏是否来自 mounting 还是 `ODO lever` 语义 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; output=`output/data2_staged_truth_ablation_r1_20260324/`; shared_controls=`strict weak gate`, `phase2/phase3 freeze GNSS lever`, `init.odo_scale=1.0` | `output/data2_staged_truth_ablation_r1_20260324/summary.md`, `output/data2_staged_truth_ablation_r1_20260324/case_metrics.csv`, `output/data2_staged_truth_ablation_r1_20260324/phase_metrics.csv`, `output/data2_staged_truth_ablation_r1_20260324/plots/key_states_jump_zoom.png` | control `phase2 RMSE3D=0.102727 m`, `first_divergence_start_t=528867.106`; `fix_mounting_truth` 近乎中性；`fix_odo_lever_truth` 提前在 `528314.000008` 发散，强烈支持 `ODO lever` 语义失配 | `pass(mainline_control_retained_mounting_fix_neutral_odo_lever_truth_fix_worse)` | `summary/csv/plots @ 2026-03-24 17:08 local` |
| `EXP-20260325-data2-staged-truth-ablation-r2` | `2026-03-25` | 在 `r1` 基础上新增 `fix_mounting_and_odo_lever_truth`，直接验证“安装角真值 + ODO 杆臂真值同时固定”是否能改善 second-stage 表现 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; output=`output/data2_staged_truth_ablation_r2_20260325/`; shared_controls=`strict weak gate`, `phase2/phase3 freeze GNSS lever`, `init.odo_scale=1.0`; new_case=`fix_mounting_and_odo_lever_truth` | `output/data2_staged_truth_ablation_r2_20260325/summary.md`, `output/data2_staged_truth_ablation_r2_20260325/case_metrics.csv`, `output/data2_staged_truth_ablation_r2_20260325/phase_metrics.csv`, `output/data2_staged_truth_ablation_r2_20260325/plots/nav_errors_compare.png`, `output/data2_staged_truth_ablation_r2_20260325/plots/key_states_compare.png`, `output/data2_staged_truth_ablation_r2_20260325/plots/key_states_jump_zoom.png`, `output/data2_staged_truth_ablation_r2_20260325/plots/velocity_vehicle_compare.png` | combined case `phase2 RMSE3D=0.109494 m`, `phase3 RMSE3D=96.781242 m`, `first_divergence_start_t=528315.655012`, `odo_accept_ratio=0.946696`, `nhc_accept_ratio=0.694537`; 比 `fix_odo_lever_truth` 只略微晚 `1.655 s` 发散，但整体仍显著差于 control | `pass(combined_mounting_plus_odo_lever_truth_still_fails)` | `summary/csv/plots @ 2026-03-25 00:16 local` |
| `EXP-20260325-data2-constraint-semantics-audit-r1` | `2026-03-25` | 不跑滤波器，仅用 `POS/IMU/ODO` 真值与 staged `control / fix_odo_lever_truth / fix_mounting_and_odo_lever_truth` 配置，审计 raw `ODO residual`、`NHC v_v.y/z` 与 lever-induced Jacobian coupling，区分“真值几何本身更差”还是“滤波更新链更差” | base=`config_data2_baseline_eskf.yaml`; compared_configs=`output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/control_strict_gate_freeze_gnss_lever/config_control_strict_gate_freeze_gnss_lever.yaml`, `output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/fix_odo_lever_truth/config_fix_odo_lever_truth.yaml`, `output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/config_fix_mounting_and_odo_lever_truth.yaml`; output=`output/data2_constraint_semantics_audit_20260325/` | `output/data2_constraint_semantics_audit_20260325/summary.md`, `output/data2_constraint_semantics_audit_20260325/constraint_truth_semantics_summary.csv`, `output/data2_constraint_semantics_audit_20260325/constraint_jacobian_coupling_summary.csv` | phase2 raw audit: control `odo_res_rmse=0.129038 m/s`, `nhc_lat_rmse=0.110929 m/s`; combined truth `odo_res_rmse=0.111925 m/s`, `nhc_lat_rmse=0.020724 m/s`; 但 Jacobian coupling 从 control `odo_H_bgz_abs_mean=0` / `nhc_H_bgz_col_norm_mean=0` 升到 combined truth `1.002805` / `0.185425` | `pass(raw_truth_geometry_not_worse_but_nonzero_lever_activates_bgz_route)` | `summary/csv @ 2026-03-25 13:48~13:50 local` |
| `EXP-20260325-data2-bgz-route-probe-r1` | `2026-03-25` | 在 `fix_mounting_and_odo_lever_truth` case 上仅切断 `ODO/NHC -> bg_z` debug 路径，验证 truth-fixed case 的主坏链是否就是 `bg_z` 写入通路 | base_case=`EXP-20260325-data2-staged-truth-ablation-r2::fix_mounting_and_odo_lever_truth`; solver=`build/Release/eskf_fusion.exe`; derived_config=`output/data2_constraint_semantics_audit_20260325/config_fix_mounting_and_odo_lever_truth_disable_bgz.yaml`; switches=`debug_odo_disable_bgz_jacobian=true`, `debug_odo_disable_bgz_state_update=true`, `debug_nhc_disable_bgz_state_update=true`; output=`output/data2_constraint_semantics_audit_20260325/` | `output/data2_constraint_semantics_audit_20260325/bgz_route_probe_summary.md`, `output/data2_constraint_semantics_audit_20260325/run_fix_mounting_and_odo_lever_truth_disable_bgz.log`, `output/data2_constraint_semantics_audit_20260325/SOL_fix_mounting_and_odo_lever_truth_disable_bgz.txt`, `output/data2_constraint_semantics_audit_20260325/state_series_fix_mounting_and_odo_lever_truth_disable_bgz.csv` | probe case `phase2 RMSE3D=0.089440 m`, `phase3 RMSE3D=3.610158 m`, `phase2 yaw_abs_max=1.129489 deg`, `phase3 bg_z_abs_max=54.749082 deg/h`, `first_divergence_start_t=NA`, `odo_accept_ratio=0.999553`, `nhc_accept_ratio=0.999426` | `pass(disabling_odo_nhc_to_bgz_route_recovers_combined_truth_case)` | `log/sol/state_series/summary @ 2026-03-25 13:52~13:54 local` |
| `EXP-20260325-data2-bgz-route-decomposition-r1` | `2026-03-25` | 在 `control + fix_odo_lever_truth + fix_mounting_and_odo_lever_truth` 上做 route attribution，拆 `ODO bg_z Jacobian`、`ODO bg_z state update`、`NHC bg_z state update` 三条路径，判断谁是 primary 坏链 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; output=`output/data2_bgz_route_decomposition_r1_20260325/`; mechanism_window=`528276.000000~528336.000000`; cases=`control_strict_gate_freeze_gnss_lever`, `fix_odo_lever_truth`, `fix_odo_lever_truth_disable_odo_bgz_jacobian`, `fix_odo_lever_truth_disable_odo_bgz_state_update`, `fix_odo_lever_truth_disable_nhc_bgz_state_update`, `fix_odo_lever_truth_disable_both_bgz_state_update`, `fix_odo_lever_truth_disable_odo_jacobian_and_both_bgz_state_update`, `fix_mounting_and_odo_lever_truth`, `fix_mounting_and_odo_lever_truth_disable_odo_bgz_jacobian`, `fix_mounting_and_odo_lever_truth_disable_odo_bgz_state_update`, `fix_mounting_and_odo_lever_truth_disable_nhc_bgz_state_update`, `fix_mounting_and_odo_lever_truth_disable_both_bgz_state_update`, `fix_mounting_and_odo_lever_truth_disable_odo_jacobian_and_both_bgz_state_update` | `output/data2_bgz_route_decomposition_r1_20260325/summary.md`, `output/data2_bgz_route_decomposition_r1_20260325/case_metrics.csv`, `output/data2_bgz_route_decomposition_r1_20260325/bgz_step_metrics.csv`, `output/data2_bgz_route_decomposition_r1_20260325/manifest.json`, `output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_odo_lever_truth/SOL_fix_odo_lever_truth_mechanism.csv`, `output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/SOL_fix_mounting_and_odo_lever_truth_mechanism.csv` | stress A `phase3 RMSE3D 95.854081 -> 3.390878 m` when `disable_odo_bgz_jacobian`, but `disable_odo_bgz_state_update` stays `96.325394 m`; stress B `96.781242 -> 3.252093 m` when `disable_odo_bgz_jacobian`, but `disable_odo_bgz_state_update` stays `96.945484 m`; `disable_nhc_bgz_state_update` instead worsens to `234.602439 / 240.441111 m` | `pass(primary_bad_path_is_odo_bgz_jacobian_nhc_bgz_state_update_not_primary)` | `summary/csv/mechanism logs @ 2026-03-25 14:51~14:52 local` |
| `EXP-20260325-data2-bgz-jacobian-fd-audit-r1` | `2026-03-25` | 对 `control / fix_odo_lever_truth / fix_mounting_and_odo_lever_truth` 三组 case 在四个关键时刻做 finite-difference 审计，直接核对 `ComputeOdoModel()` / `ComputeNhcModel()` 的 `bg_z/sg_z/att_z` 等 Jacobian 是否与数值差分一致，并拆出 `ODO H_bgz/H_sgz/H_att_z` 分量 | app=`build/Release/odo_nhc_bgz_jacobian_fd.exe`; source=`apps/odo_nhc_bgz_jacobian_fd_main.cpp`; configs=`output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/control_strict_gate_freeze_gnss_lever/config_control_strict_gate_freeze_gnss_lever.yaml`, `output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_odo_lever_truth/config_fix_odo_lever_truth.yaml`, `output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/config_fix_mounting_and_odo_lever_truth.yaml`; timestamps=`528276.004906`, `528314.000008`, `528315.655012`, `528867.106000`; output=`output/data2_bgz_jacobian_fd_audit_r1_20260325/` | `output/data2_bgz_jacobian_fd_audit_r1_20260325/summary.md`, `output/data2_bgz_jacobian_fd_audit_r1_20260325/case_meta.csv`, `output/data2_bgz_jacobian_fd_audit_r1_20260325/fd_vs_analytic.csv`, `output/data2_bgz_jacobian_fd_audit_r1_20260325/sample_summary.csv`, `output/data2_bgz_jacobian_fd_audit_r1_20260325/odo_component_breakdown.csv` | worst ODO sample `rel_fro=8.123593e-08`, `max_abs=5.272630e-07`; worst ODO key-column diffs: `|ΔH_bgz|<=1.168572e-09`, `|ΔH_sgz|<=1.754965e-09`, `|ΔH_att_z|<=3.207355e-08`; stressed cases `H_bgz≈-1.003~-1.006` at phase2 samples while `|H_sgz|<=0.00521`, `|H_att_total_z|<=0.01572`, `|H_att_lever_z|<=7.74e-07` | `pass(analytic_jacobian_matches_fd_primary_bgz_route_is_real_model_sensitivity_not_formula_bug)` | `summary/csv @ 2026-03-25 15:17:51 local` |
| `EXP-20260325-data2-nhc-admission-alignment-r1` | `2026-03-25` | 在 staged current mainline 上正式比较 `RunNhcUpdate()` 的 `v_b`、`v_wheel_b`、`v_v` 三种 admission 口径，验证 admission mismatch 是否能解释 `current mainline vs INS/GNSS` gap | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; code=`include/app/fusion.h`, `src/app/config.cpp`, `src/app/pipeline_fusion.cpp`; script=`scripts/analysis/analyze_bgz_admission_alignment.py`; output=`output/data2_nhc_admission_alignment_r1_20260325/`; variants=`baseline_current_v_b_gate`, `nhc_gate_use_v_wheel_b`, `nhc_gate_use_v_v` | `output/data2_nhc_admission_alignment_r1_20260325/summary.md`, `output/data2_nhc_admission_alignment_r1_20260325/case_metrics.csv`, `output/data2_nhc_admission_alignment_r1_20260325/admission_mismatch_summary.csv`, `output/data2_nhc_admission_alignment_r1_20260325/artifacts/cases/baseline_current_v_b_gate/SOL_baseline_current_v_b_gate_nhc_admission.csv` | 三组 case 完全重合：`phase2 RMSE3D=0.102727 m`, `phase3 RMSE3D=23.838294 m`, `first_divergence_start_t=528867.106000`, `odo_accept_ratio=0.998610`, `nhc_accept_ratio=0.998891`；admission log 还显示 `accept_v_b=accept_v_wheel_b=accept_v_v=1.0`、`current_vs_selected_mismatch_ratio=0` | `pass(admission_mismatch_rejected_for_current_mainline)` | `summary/csv/admission log @ 2026-03-25 17:41 local` |
| `EXP-20260325-data2-odo-reference-semantics-audit-r1` | `2026-03-25` | 复用 `EXP-20260325-data2-nhc-admission-alignment-r1::baseline_current_v_b_gate` 的 fresh current-mainline 输出，直接审计 wheel-speed 对 `v_b.x`、`v_v.x`、`odo_scale*v_v.x` 等前向速度语义的匹配度，判断 `ODO residual / v_vehicle` 语义是否是当前主 gap 的解释 | source_case=`EXP-20260325-data2-nhc-admission-alignment-r1::baseline_current_v_b_gate`; source_config=`output/data2_nhc_admission_alignment_r1_20260325/artifacts/cases/baseline_current_v_b_gate/config_baseline_current_v_b_gate.yaml`; source_sol=`output/data2_nhc_admission_alignment_r1_20260325/artifacts/cases/baseline_current_v_b_gate/SOL_baseline_current_v_b_gate.txt`; source_state_series=`output/data2_nhc_admission_alignment_r1_20260325/artifacts/cases/baseline_current_v_b_gate/state_series_baseline_current_v_b_gate.csv`; script=`scripts/analysis/analyze_odo_reference_semantics.py`; output=`output/data2_odo_reference_semantics_audit_r1_20260325/` | `output/data2_odo_reference_semantics_audit_r1_20260325/summary.md`, `output/data2_odo_reference_semantics_audit_r1_20260325/semantics_metrics.csv`, `output/data2_odo_reference_semantics_audit_r1_20260325/contribution_summary.csv` | `odo_model_est = odo_scale*v_v.x` 在 `overall / phase2 / phase3 / divergence+60s` 全部是最优候选：例如 `phase3 mean|res|=0.031709 m/s`, `rmse=0.047036 m/s`；显著优于 `v_b.x` 的 `0.109409 / 0.126220 m/s` 与 `truth_v_v.x` 的 `0.107336 / 0.121291 m/s`；同时 `phase3 mean|v_v_rot_x|=0.028090 m/s`, `mean odo_scale=0.991441` | `pass(current_odo_model_semantics_match_measurement_best)` | `summary/csv @ 2026-03-25 17:47 local` |
| `EXP-20260325-data2-raw-odo-truth-speed-audit-r1` | `2026-03-25` | 不依赖当前求解器解释链，直接用原始 `IMULog201812290233_2_ODO.txt` 与真值 `LC_SM_TXT.nav` 对账，判断原始 ODO 文件倒数第二列更像“原始脉冲计数”还是“已换算前向速度” | raw_odo=`dataset/data2/IMULog201812290233_2_ODO.txt`; truth=`dataset/data2/LC_SM_TXT.nav`; script=`scripts/analysis/analyze_data2_raw_odo_truth_speed.py`; output=`output/data2_raw_odo_truth_speed_audit_r1_20260325/` | `output/data2_raw_odo_truth_speed_audit_r1_20260325/summary.md`, `output/data2_raw_odo_truth_speed_audit_r1_20260325/metrics.csv`, `output/data2_raw_odo_truth_speed_audit_r1_20260325/samples.csv` | 对真值三维速度模长：`MAE=0.116560 m/s`, `RMSE=0.136404 m/s`, `corr=0.999832`, `odo/ref scale=0.991519`, `bias=-0.096710 m/s`；最后一列恒为 `0`；整数特征比例仅 `~6.4%`，不呈现计数列特征 | `pass(raw_odo_penultimate_column_matches_truth_speed_and_is_not_pulse_count)` | `summary/csv @ 2026-03-25 20:14 local` |
| `EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1` | `2026-03-25` | 在不分阶段的 full-window 口径下，执行五组 `INS/GNSS` / `INS/GNSS/ODO/NHC` 对照，直接比较姿态误差与 bias 异常是否随外参与 `ODO/NHC` 权重、以及 `sg/sa` 比例因子状态是否参与估计而同步变化 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py`; output=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/`; groups=`G1 INS/GNSS truth GNSS lever fixed`, `G2 truth extrinsics fixed`, `G3 truth extrinsics soft(0.01x Q)`, `G4 truth extrinsics soft + 5x ODO/NHC R`, `G5 G4 + disable sg/sa(15-20)` | `output/data2_fullwindow_attitude_bias_coupling_r1_20260325/summary.md`, `output/data2_fullwindow_attitude_bias_coupling_r1_20260325/case_metrics.csv`, `output/data2_fullwindow_attitude_bias_coupling_r1_20260325/manifest.json`, `output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/all_states_overview.png`, `output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/key_coupling_states.png`, `output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale/all_states_group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale.csv` | `G1 RMSE3D=0.043118 m, yaw_max=1.043771 deg`; `G2 RMSE3D=0.545297 m, yaw_max=179.999974 deg, bg_z_err_max=16917.596262 deg/h`; `G3 RMSE3D=7.616816 m, yaw_max=179.991929 deg, bg_z_err_max=24901.080574 deg/h`; `G4 RMSE3D=0.338954 m, yaw_max=111.942713 deg, bg_z_err_max=16389.812875 deg/h`; `G5 RMSE3D=0.301991 m, yaw_max=85.119234 deg, bg_z_err_max=15068.234661 deg/h`, 且 `G4/G5` 的 `ODO/NHC accept_ratio=1.0/1.0` | `pass(g5_ablation_further_helps_but_coupling_remains)` | `summary/csv/plots overwritten @ 2026-03-25 21:01 local` |
| `EXP-20260325-data2-staged-g5-no-imu-scale-r1` | `2026-03-25` | 以 full-window `G5` 为研究对象，构造三段式 `INS/GNSS -> INS/GNSS/ODO/NHC -> phase3 周期断星` 实验，全程禁用 `sg/sa(15-20)` 在线估计，并比较 `ODO/NHC` 量测噪声 `4x/5x/6x` 对 phase2 联合段与 phase3 长 outage 段的影响；同时保留 dedicated `position/velocity/attitude` 误差图 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_staged_g5_no_imu_scale.py`; output=`output/data2_staged_g5_no_imu_scale_r1_20260325/`; phase_windows=`[528076.0,528276.0] / [528276.0,528776.0] / [528776.0,530488.9]`; phase3_periodic_gnss=`on=90 s, off=90 s`; cases=`staged_g5_odo_nhc_noise_4x`, `staged_g5_odo_nhc_noise_5x`, `staged_g5_odo_nhc_noise_6x` | `output/data2_staged_g5_no_imu_scale_r1_20260325/summary.md`, `output/data2_staged_g5_no_imu_scale_r1_20260325/case_metrics.csv`, `output/data2_staged_g5_no_imu_scale_r1_20260325/phase_metrics.csv`, `output/data2_staged_g5_no_imu_scale_r1_20260325/manifest.json`, `output/data2_staged_g5_no_imu_scale_r1_20260325/plots/all_states_overview.png`, `output/data2_staged_g5_no_imu_scale_r1_20260325/plots/key_coupling_states.png`, `output/data2_staged_g5_no_imu_scale_r1_20260325/plots/position.png`, `output/data2_staged_g5_no_imu_scale_r1_20260325/plots/velocity.png`, `output/data2_staged_g5_no_imu_scale_r1_20260325/plots/attitude.png`, `output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv` | `4x: overall RMSE3D=3.648409 m, phase2=0.070390 m, phase3=4.329976 m, yaw_max=5.205972 deg, bg_z_err_max=189.070546 deg/h`; `5x: overall RMSE3D=2.763782 m, phase2=0.070832 m, phase3=3.279972 m, yaw_max=4.754404 deg, bg_z_err_max=138.454914 deg/h`; `6x: overall RMSE3D=2.678202 m, phase2=0.071153 m, phase3=3.178389 m, yaw_max=4.240251 deg, bg_z_err_max=110.196945 deg/h`; `phase2 spread=0.000763 m`; `Δphase3(4x->6x)=-1.151587 m`; 三组 `ODO/NHC accept_ratio=1.0/1.0`; `phase3 outage windows=10` | `pass(phase3_long_outage_drift_still_improves_at_5x_6x_but_enters_diminishing_returns_under_staged_g5)` | `summary/manifest @ 2026-03-25 21:56:00 local; plots @ 21:55:58 local` |
| `EXP-20260325-data2-staged-g5-no-imu-scale-r2` | `2026-03-26` | 在 `r1` 的 staged `G5` `4x/5x/6x` 基础上新增一个 phase3 `INS/GNSS outage` 对照组：phase1/phase2 与 `6x` 基线完全一致，但 phase3 显式关闭 `ODO/NHC`，用于直接验证“当前 phase3 漂移是否主要由 ODO/NHC 自身带坏” | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_staged_g5_no_imu_scale.py`; output=`output/data2_staged_g5_no_imu_scale_r2_20260325/`; phase_windows=`[528076.0,528276.0] / [528276.0,528776.0] / [528776.0,530488.9]`; phase3_periodic_gnss=`on=90 s, off=90 s`; cases=`staged_g5_odo_nhc_noise_4x`, `staged_g5_odo_nhc_noise_5x`, `staged_g5_odo_nhc_noise_6x`, `staged_g5_odo_nhc_noise_6x_phase3_ins_only`; control_semantics=`phase3 enable_odo=false`, `enable_nhc=false` | `output/data2_staged_g5_no_imu_scale_r2_20260325/summary.md`, `output/data2_staged_g5_no_imu_scale_r2_20260325/case_metrics.csv`, `output/data2_staged_g5_no_imu_scale_r2_20260325/phase_metrics.csv`, `output/data2_staged_g5_no_imu_scale_r2_20260325/manifest.json`, `output/data2_staged_g5_no_imu_scale_r2_20260325/plots/position.png`, `output/data2_staged_g5_no_imu_scale_r2_20260325/plots/key_coupling_states.png`, `output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml`, `output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv` | `6x` 基线保持 `phase2 RMSE3D=0.071153 m`, `phase3 RMSE3D=3.178389 m`, `gnss_off_01 RMSE3D=5.220606 m`, `gnss_off_01 final_err_3d=8.296212 m`; 新增 phase3 `INS/GNSS outage` 对照则恶化到 `overall RMSE3D=26.673278 m`, `phase3 RMSE3D=31.657637 m`, `gnss_off_01 RMSE3D=46.206528 m`, `gnss_off_01 final_err_3d=112.485737 m`, `gnss_off_09 final_err_3d=205.608601 m`; 但其 `phase2 RMSE3D` 与 `6x` 完全相同，且 `yaw_err_max_abs` 仅 `1.098057 deg`、`bg_z_err_max_abs=110.041076 deg/h` | `pass(phase3_ins_only_control_shows_odo_nhc_are_helping_strongly_not_hurting_under_current_staged_g5)` | `summary/manifest @ 2026-03-26 00:11:49 local; plots @ 00:11:46 local` |
| `EXP-20260326-data2-staged-g5-6x-interactive-report-r1` | `2026-03-26` | 基于 fresh `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 产物生成并重构单页 Plotly 离线交互网页，统一展示 `6x` 主结果、22 维非 PVA 状态/误差、PVA 误差、`v` 系速度误差、`v` 系航向误差、轨迹，以及 phase3 无 `ODO/NHC` 的 PVA 对照 | source_exp=`EXP-20260325-data2-staged-g5-no-imu-scale-r2`; source_output=`output/data2_staged_g5_no_imu_scale_r2_20260325/`; script=`scripts/analysis/build_data2_staged_g5_6x_interactive_report.py`; test=`tests/test_build_data2_staged_g5_6x_interactive_report.py`; output=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/`; main_case=`staged_g5_odo_nhc_noise_6x`; control_case=`staged_g5_odo_nhc_noise_6x_phase3_ins_only`; vehicle_truth=`pitch=0.36 deg, yaw=0.84 deg` | `output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report.html`, `output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report_manifest.json`, `output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv`, `output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv` | 汇总 `6x phase2 RMSE3D=0.071153 m`, `6x phase3 RMSE3D=3.178389 m`, `6x gnss_off_01 RMSE3D=5.220606 m`, `control phase3 RMSE3D=31.657637 m`, `control gnss_off_01 final_err_3d=112.485737 m`; 页面固定紧凑 `3-column` 布局，`ba/bg` 不再显示 truth，`ODO/NHC` 校准项改为误差曲线，`vehicle_heading` truth yaw 采用 `0.84 deg` | `pass(interactive_report_generated_and_restyled_from_fresh_r2_artifacts)` | `html/manifest @ 2026-03-26 01:25:24 local` |
| `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r1` | `2026-03-25` | 首次实现 staged `G5 6x` 下的 zero-init mounting `Q` sweep，但把大 mounting 协方差仅写在 `t0 P0_diag`，未考虑 `phase1` 冻结时协方差会被清零 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py` (pre-fix); output=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/` | `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/summary.md`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/case_metrics.csv`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/manifest.json`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv` | `mounting_pitch/yaw` 几乎一直贴近 `0`；后续代码审计确认根因是 `phase1` 的 state mask 会把 `P(22:24,:)` 清零，使 `phase2` 开始时的大 `P0` 已失效，因此该批结果不能用来回答“phase2 起点 zero-init mounting + large prior”问题 | `stale_or_conflict(phase1_mask_zeroed_mounting_cov_so_r1_did_not_match_intended_phase2_seed_semantics)` | `superseded by r2 at 2026-03-25 22:37 local` |
| `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2` | `2026-03-25` | 修正 zero-init mounting `Q` sweep 的 phase2 起跑语义后，再按用户确认把实验统一为“总安装角从 0 初值开始估计”：保留 `phase1` 冻结 mounting，但在 `phase2` 开头插入 `0.02 s` 的 mounting covariance seed 窗，通过 runtime covariance floor 在第二阶段开始时重建 `3 deg` mounting 初始协方差；case config 显式设 `constraints.imu_mounting_angle=[0,0,0]` 与 `init.mounting_*0=0`，并继续比较 `0.5x/1x/2x` mounting `Q`；`all_states`/plots 中的 `mounting_*` 统一为 total mounting，原始状态增量单独保留在 `mounting_state_*` | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`; helper=`scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py`; output=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/`; phase_windows=`[528076.0,528276.0] / [528276.0,528776.0] / [528776.0,530488.9]`; phase2_seed=`duration=0.02 s`, `enable_covariance_floor=true`, `p_floor_mounting_deg=3.0`; fixed_controls=`odo_nhc_r=6x vs G5`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `lever_odo/lever_gnss truth init`, `odo_scale=1.0`, `mounting_total_base=[0,0,0]`; cases=`staged_g5_mounting_zero_init_q_0p5x`, `staged_g5_mounting_zero_init_q_1x`, `staged_g5_mounting_zero_init_q_2x` | `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/summary.md`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/case_metrics.csv`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/phase_metrics.csv`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/manifest.json`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/truth_reference.json`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/plots/mounting.png`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/config_staged_g5_mounting_zero_init_q_1x.yaml`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv` | `0.5x/1x/2x` 三组仍几乎重合：`overall RMSE3D=3.399490 m`, `phase2 RMSE3D=0.070300 m`, `phase3 RMSE3D=4.034528 m`, `yaw_max=3.954837 deg`, `bg_z_err_max≈171.0529 deg/h`；`q_1x` 首行现为 `mounting_yaw(total)=0.0 deg`, `mounting_state_yaw=0.0 deg`, `truth_mounting_yaw=1.37 deg`；其 phase2 内 `mounting_pitch(total)∈[0.001762,0.361059] deg`, `mounting_yaw(total)∈[0.060739,0.917667] deg`，且 `phase2_end mounting_yaw(total)=0.896198 deg`；相对 truth-init `staged_g5_odo_nhc_noise_6x`，`Δphase3=+0.856139 m`, `Δbg_z_err_max=+60.855923 deg/h` | `pass(total_zero_init_semantics_are_now_fresh_q_sweep_still_nearly_neutral_and_total_zero_case_is_worse_than_truth_init_6x)` | `summary/manifest @ 2026-03-25 23:39:10 local; q1 all_states @ 23:37:55 local` |
| `EXP-20260325-data2-bgz-state-competition-r1` | `2026-03-25` | 在保持 combined-truth `mounting + lever_odo` 几何不变的前提下，只切 `mounting/lever` 的 freeze/open，判断 residual 是否会从 `bg_z` 明显转移到 `mounting_yaw` 或 `lever_odo_y` | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_bgz_competition_probe.py`; output=`output/data2_bgz_state_competition_r1_20260325/`; shared_controls=`strict weak gate`, `phase2/phase3 freeze GNSS lever`, `init.odo_scale=1.0`, `fej=false`; critical_window=`528276.000000~528336.000000`; cases=`mounting_fixed+lever_fixed`, `mounting_open+lever_fixed`, `mounting_fixed+lever_open`, `mounting_open+lever_open` | `output/data2_bgz_state_competition_r1_20260325/state_competition_summary.md`, `output/data2_bgz_state_competition_r1_20260325/case_metrics.csv`, `output/data2_bgz_state_competition_r1_20260325/manifest.json` | fixed-fixed `phase3 RMSE3D=96.781242 m`; `mounting_open+lever_fixed=97.006751 m` (near-neutral/worse); `mounting_fixed+lever_open=87.329409 m`, `mounting_open+lever_open=87.252899 m` (only modest late relief); first divergence 仍在 `528315.655012~528315.660012`; critical-window `mounting_yaw` peak change only `0.000065 deg`, but lever-open cases 到 phase2 末 `lever_odo_y` net delta `≈0.2795 m` | `pass(mounting_state_competition_negligible_lever_open_secondary_bgz_route_still_primary)` | `summary/csv/manifest @ 2026-03-25 15:48 local` |
| `EXP-20260325-data2-bgz-mitigation-validation-r1` | `2026-03-25` | 用现有 non-debug `enable_bgz_observability_gate` 做第一轮正式验证，比较 `INS/GNSS`、current mainline、global `bg_z gate` 以及 `bg_z gate + covariance forgetting` | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_bgz_mitigation_validation.py --case-set r1`; output=`output/data2_bgz_mitigation_validation_r1_20260325/`; variants=`ins_gnss_only`, `current_mainline_ins_gnss_odo_nhc`, `mitigated_bgz_gate_only`, `mitigated_bgz_gate_and_forgetting` | `output/data2_bgz_mitigation_validation_r1_20260325/summary.md`, `output/data2_bgz_mitigation_validation_r1_20260325/case_metrics.csv`, `output/data2_bgz_mitigation_validation_r1_20260325/manifest.json` | `ins_gnss_only phase3 RMSE3D=7.835156 m`; current mainline `23.838294 m`; global gate `23.906627 m`（更差）；gate+forget `24.323763 m`（显著更差），说明 global gate 会误伤 NHC 补偿且 forgetting 在 current mainline 下有害 | `pass(global_bgz_gate_not_enough_and_gate_plus_forgetting_harmful)` | `summary/csv/manifest @ 2026-03-25 16:31 local` |
| `EXP-20260325-data2-bgz-mitigation-validation-r2` | `2026-03-25` | 在 C++ 中把 `bg_z` gate 拆成 sensor-specific 开关，只对 `ODO` 生效、不再抑制 `NHC`，验证是否比 global gate 更合理 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; code=`include/app/fusion.h`, `src/app/config.cpp`, `src/app/pipeline_fusion.cpp`; script=`scripts/analysis/run_data2_bgz_mitigation_validation.py --case-set odo_only_r2`; output=`output/data2_bgz_mitigation_validation_r2_20260325/`; variants=`ins_gnss_only`, `current_mainline_ins_gnss_odo_nhc`, `mitigated_bgz_gate_odo_only` | `output/data2_bgz_mitigation_validation_r2_20260325/summary.md`, `output/data2_bgz_mitigation_validation_r2_20260325/case_metrics.csv`, `output/data2_bgz_mitigation_validation_r2_20260325/manifest.json` | ODO-only gate `phase3 RMSE3D=23.704855 m`，较 current mainline 仅改善 `0.133439 m`；`phase3 yaw peak 29.954979 -> 29.346651 deg`，但 `phase3 bg_z peak` 反而 `153.518974 -> 159.863803 deg/h` | `pass(odo_only_gate_avoids_global_gate_regression_but_gain_is_small)` | `summary/csv/manifest @ 2026-03-25 16:55 local` |
| `EXP-20260325-data2-bgz-mitigation-validation-r3` | `2026-03-25` | 对 ODO-only gate 再做更强门限验证：只允许 `yaw_rate>=12 deg/s`，并关掉 lateral-acc 通道，检查“方向对了但门限太松”是否成立 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; script=`scripts/analysis/run_data2_bgz_mitigation_validation.py --case-set odo_only_tight_r3`; output=`output/data2_bgz_mitigation_validation_r3_20260325/`; variant=`mitigated_bgz_gate_odo_only_yaw12` | `output/data2_bgz_mitigation_validation_r3_20260325/summary.md`, `output/data2_bgz_mitigation_validation_r3_20260325/case_metrics.csv`, `output/data2_bgz_mitigation_validation_r3_20260325/manifest.json` | tight yaw-only gate `phase3 RMSE3D=23.713238 m`，较 current mainline 仅改善 `0.125056 m`，与 `r2` 基本同级；说明 current mainline 的主 gap 不能靠单一 `ODO bg_z gate` 解决 | `pass(tighter_odo_only_gate_still_insufficient_for_current_mainline)` | `summary/csv/manifest @ 2026-03-25 17:07 local` |
| `EXP-20260324-data2-phase2-entry-intervention-r1` | `2026-03-24` | 对比 `value reseed / partial freeze / relaxed gate` 三类 phase2 入口干预，判断谁能把首条 live accepted `ODO` 拉回 phase2 首拍 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; output=`output/data2_phase2_entry_intervention_r1_20260324/`; probe_window=`528076.0~528340.6` | `output/data2_phase2_entry_intervention_r1_20260324/analysis/summary.md`, `output/data2_phase2_entry_intervention_r1_20260324/analysis/entry_summary.csv`, `output/data2_phase2_entry_intervention_r1_20260324/analysis/odo_reject_summary.csv` | baseline `first live accepted ODO=528300.964971`; `value_reseed_odo_scale_1p0` 提前到 `528276.004906` 且 `entry mean |odo_speed-odo_scale*v_v.x|=0.049381 m/s`; `partial_freeze` 与 `relaxed_gate` 都不能真正解锁 ODO | `pass(odo_scale_nominal_reseed_is_primary_entry_fix_knob)` | `analysis @ 2026-03-24 15:10 local` |
| `EXP-20260324-data2-phase2-early-window-probe-r1` | `2026-03-24` | 审计 `phase2` 入口 `528276~528340.6 s` 的 earliest accepted-update 链，确认 `528277` 跳变到底由谁执行 | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; output=`output/data2_phase2_early_window_probe_r1_20260324/`; entry_window=`528276.004906~528276.999907` | `output/data2_phase2_early_window_probe_r1_20260324/analysis/summary.md`, `output/data2_phase2_early_window_probe_r1_20260324/analysis/burst_summary.csv`, `output/data2_phase2_early_window_probe_r1_20260324/analysis/gnss_update_summary.csv` | 首秒三组 case 都是 `accepted NHC=200 / accepted ODO=0`; `GNSS_POS@528277` 仍执行离散 jump；说明去掉 stale replay / frozen clamp 后，phase-entry mismatch 仍真实存在 | `pass(earliest_visible_jump_chain_real_after_bug_fixes)` | `analysis @ 2026-03-24 14:38 local` |
| `EXP-20260324-data2-phase2-bridge-probe-r1` | `2026-03-24` | 验证 `phase2 bridge / covariance pinning` 是否能推迟 staged failure，并区分 `GNSS lever` 是 earliest source 还是 downstream executor | base=`config_data2_baseline_eskf.yaml`; solver=`build/Release/eskf_fusion.exe`; output=`output/data2_phase2_bridge_probe_r1_20260324/`; probe_window=`528076.0~528560.6` | `output/data2_phase2_bridge_probe_r1_20260324/summary.md`, `output/data2_phase2_bridge_probe_r1_20260324/case_metrics.csv`, `output/data2_phase2_bridge_probe_r1_20260324/threshold_crossings.csv`, `output/data2_phase2_bridge_probe_r1_20260324/jump_window_audit_staged_release_phase2_freeze_gnss_lever.md` | bridge 可把 `bg_z>=10` / `sa_x>=5000` / `gnss_lever_y>=0.5` 首穿显著后移；freeze `GNSS lever` 不改变 earliest divergence，但显著压低 visible jump 幅值 | `pass(phase2_bridge_upstream_gnss_lever_downstream_executor)` | `summary/jump_audit @ 2026-03-24 13:07 local` |

## Known Inconsistencies

| issue_id | detected_on | description | affected files | impact | resolution status |
|---|---|---|---|---|---|
| `ISSUE-20260324-guard-line-retired` | `2026-03-24` | `fixed5s_position_only / phase2_gnss_transition_guard / mounting-fixed fullwindow` 这一整条 phase-transition jump 解决线未带来稳定收益，且不再作为当前主线证据。相关 output、专用脚本与 runtime guard-release 代码已从 active workspace 清理。 | `output/data2_phase2_gnss_transition_guard_r*_20260324/`, `output/data2_fixed5s_position_only_mounting_fixed_fullwindow_r*_20260324/`, `scripts/analysis/*phase2_gnss_transition_guard*`, `scripts/analysis/plot_phase2_guard_fixed5s_all_states.py`, `include/app/fusion.h`, `src/app/config.cpp`, `src/app/pipeline_fusion.cpp` | 不得再引用已清理产物或把 `fixed 5 s position_only` 当当前推荐方案。 | `resolved(retired_and_cleaned_from_active_workspace)` |
| `ISSUE-20260324-baseline-prior-not-fresh` | `2026-03-24` | `config_data2_baseline_eskf.yaml` 已承载新的 bias/scale prior，但 official baseline output 仍未 fresh 对齐当前 prior。 | `config_data2_baseline_eskf.yaml`, `output/data2_eskf_baseline/` | 官方 baseline 数值引用需注明 freshness caveat。 | `open` |
| `ISSUE-20260325-mounting-q-r1-phase2-seed-missing` | `2026-03-25` | 首版 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r1` 仅把大 mounting 协方差写入 `t0 P0_diag[22:24]`，但 `phase1` 冻结 mounting 时引擎会清零对应协方差，导致 `phase2` 开始时 mounting 实际上从零状态、近零协方差起跑，实验语义与“第二阶段开始给大初始噪声”不一致。 | `scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`, `include/app/fusion.h`, `src/app/config.cpp`, `src/app/pipeline_fusion.cpp`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/` | `r1` 结果不能再用于回答 zero-init mounting 的正式结论，后续引用必须切换到 `r2`。 | `resolved_by_r2_phase2_cov_seed_window_and_runtime_covariance_floor_override` |

## Open Hypotheses

| hyp_id | hypothesis | evidence so far | next check | status |
|---|---|---|---|---|
| `HYP-41` | 当前 staged / free-run 可见 jump 的更直接上游问题，不是单独 `NHC` 太强，而是 wheel-constraint 分支里的 `ODO bg_z Jacobian` 在当前可观性条件下把 residual 写进了错误状态通路；NHC 的直接 `bg_z` 写入更像后续补偿/次级分支。 | `EXP-20260325-data2-bgz-route-decomposition-r1` 已显示两组 stressed case 里，单独 `disable_odo_bgz_jacobian` 都能把 `phase3 RMSE3D` 从 `95.854081/96.781242 m` 拉回到 `3.390878/3.252093 m`；`EXP-20260325-data2-bgz-jacobian-fd-audit-r1` 又证明 `ComputeOdoModel()` 的 analytic Jacobian 与 FD 在 sampled cases 上高度一致；`EXP-20260325-data2-bgz-state-competition-r1` 进一步排除外参 freeze/open 为 primary 根因；本轮 `EXP-20260325-data2-bgz-mitigation-validation-r1/r2/r3` 又显示 global gate 比 current mainline 更差，而 ODO-only gate 则比 global gate 明显更合理、但收益仅 `~0.13 m`。这说明 “NHC bg_z` 更像补偿而不是 primary poison source” 这一判断仍成立。 | 优先执行 `EXP-20260325-data2-nhc-admission-alignment-r1` 与更细的 `ODO residual / v_vehicle` semantics 审计，而不是继续堆 `bg_z gate` 变体。 | `open(strengthened_nhc_as_secondary_compensation_global_gate_hurts_odo_only_gate_small_gain)` |
| `HYP-43` | `phase2` 入口当前 primary blocker 是 `odo_scale` nominal/value mismatch；modest gate 或 early `ba/sa` freeze 都不是主解。 | `EXP-20260324-data2-phase2-entry-intervention-r1` 中只有 `value_reseed_odo_scale_1p0` 把首条 live accepted `ODO` 从 `528300.964971` 提前到 `528276.004906`。 | 把 `odo_scale` value reseed 提升到更长 `phase2/full-window` 实验，并与 covariance bridge 组合。 | `validated(refined_to_phase2_odo_scale_nominal_reseed_as_primary_entry_fix_knob)` |
| `HYP-47` | `phase2` 最早的 `528277` 入口跳变不是 accepted `ODO` 先失控，而是 `ODO` 数秒内持续被拒、`NHC` 先写入 `ba_x/sa_x/bg_z`，随后首个 `GNSS_POS` 执行离散 jump。 | `EXP-20260324-data2-phase2-early-window-probe-r1` 已在修掉 stale replay / frozen clamp 后复现该链路。 | 若需要继续做入口机理，只保留原始 joint baseline 观测，不再延伸旧 guard 方案。 | `validated_by_phase2_entry_bug_audit_and_early_window_probe` |
| `HYP-48` | README `ODO lever truth [0.2,-1.0,0.6]` 可能仍存在局部约定差异，但它已不再是当前“truth-fixed case 更差”的最强解释。 | `EXP-20260325-data2-constraint-semantics-audit-r1` 中，phase2 raw truth audit 反而显示 combined truth 比 control 的 `odo_res_rmse` 与 `nhc_lat_rmse` 更小；`EXP-20260325-data2-raw-odo-truth-speed-audit-r1` 又进一步表明原始 ODO 文件倒数第二列本身就与真值速度模长高度一致，因此“把原始脉冲误当真实速度”也不是当前仓内 data2 文件的主要解释。 | 仅保留对 lever truth 坐标约定与外部 README 标注的一致性复核，不再把“truth 语义错误/ODO 原始脉冲未换算”当首要主线。 | `weakened_by_raw_truth_audit_truth_geometry_not_worse` |
| `HYP-49` | 当前 `INS/GNSS/ODO/NHC` 劣于 `INS/GNSS` 的主因，是 nonzero `lever_odo(25-27)` 打开的 `ODO bg_z Jacobian` 上游链，而不是“谁直接写了 `bg_z(14)`”；外参 state competition 最多是 secondary 效应，不是 primary 根因。 | `EXP-20260325-data2-constraint-semantics-audit-r1` 已显示 truth case 的 `odo_H_bgz_abs_mean≈1.00`、`nhc_H_bgz_col_norm_mean≈0.18`；`EXP-20260325-data2-bgz-route-decomposition-r1` 表明单独关掉 `ODO bg_z Jacobian` 即可恢复 stressed cases；`EXP-20260325-data2-bgz-jacobian-fd-audit-r1` 确认 stressed cases 的 `H_bgz≈-1.003~-1.006` 是解析正确且显著主导的列；本轮 `EXP-20260325-data2-bgz-mitigation-validation-r2/r3` 又显示 “只 gate ODO、不 gate NHC” 比 global gate 更合理，但 current mainline 的 `phase3 RMSE3D` 也仅从 `23.838294 m` 降到 `23.704855/23.713238 m`。 | 把该假设从“sole mainline root cause”收缩为“强而真实的一条坏链，但不足以单独解释 current mainline 相对 `INS/GNSS` 的全部 gap”；下一步查 admission / semantics。 | `open(refined_odo_h_bgz_real_but_not_sufficient_alone_for_current_mainline_gap)` |
| `HYP-50` | `ComputeOdoModel()` 中 lever-induced `H_bgz/H_sg/H_att` 至少有一列存在符号、尺度或参考系 Jacobian bug，或者该 Jacobian 在当前弱激励段缺少必要门控，因而被错误地当成“强约束”使用。 | `EXP-20260325-data2-bgz-jacobian-fd-audit-r1` 已排除“解析 Jacobian 抄错/符号错/尺度错”；`EXP-20260325-data2-bgz-mitigation-validation-r1/r2/r3` 说明单一 `ODO-only bg_z gate` 只有 `~0.13 m` 级弱收益；本轮 `EXP-20260325-data2-nhc-admission-alignment-r1` 又排除 `NHC admission mismatch`，`EXP-20260325-data2-odo-reference-semantics-audit-r1` 也显示 `odo_scale*v_v.x` 对 wheel-speed 的匹配反而最好。于是“measurement semantics / admission mismatch 更可能”这一支已被削弱，剩余更像是 current mainline 在 phase3 的 covariance / route activation 使真实的高增益 `ODO H_bgz` 列在错误时机占优。 | 直接对 current mainline `528840~528930 s` 执行 `enable_mechanism_log` 审计，定位 first-divergence 前后首个 `ODO/NHC` 成功更新的 `H_bg_z`、numerator、`P(bg_z,*)` 与 `dx_bg_z`。 | `open(refined_to_current_mainline_covariance_route_activation_audit)` |
| `HYP-51` | `RunNhcUpdate()` 当前基于 `v_b` 的 admission logic 与实际 residual 使用的 `v_wheel_b/v_v` 参考点不一致，可能仍会造成错放行，但它已不再像是当前 primary root cause。 | 代码检查确认 precheck 用 `v_b`、残差模型用 `v_wheel_b -> v_v`；但本轮 `EXP-20260325-data2-nhc-admission-alignment-r1` 在 staged current mainline 上直接比较 `v_b / v_wheel_b / v_v` 三种口径后发现三组 case 的 `phase2/phase3 RMSE3D`、`bg_z peak`、`first_divergence_start_t`、`odo/nhc accept ratio` 完全相同，且 admission log 给出 `accept_v_b=accept_v_wheel_b=accept_v_v=1.0`、`current_vs_selected_mismatch_ratio=0`。这说明当前主线上 admission 阈值根本没有发挥区分作用（`nhc_disable_below_forward_speed=0`, `nhc_max_abs_v=5.0`）。 | 不再继续做 `NHC admission` 口径变体；转向 current mainline 的 `ODO/NHC` 成功更新机理审计。 | `rejected(current_mainline_nhc_admission_mismatch_zero_under_active_thresholds)` |
| `HYP-52` | 在 full-window `INS/GNSS/ODO/NHC` 工况下，姿态异常与零偏异常由同一条 `ODO/NHC` 权重驱动的耦合链路触发；增大 `ODO/NHC` 量测噪声会先显著改善导航误差，而再去掉 `sg/sa(15-20)` 只会进一步缓和该链路，不会自动消除 `bg_z` 大幅偏离。 | `EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1` 显示：`G2 truth 外参固定` 已把 `yaw_err_max_abs` 推到 `179.999974 deg`、`bg_z_err_max_abs` 推到 `16917.596262 deg/h`；`G3 truth 外参小Q释放` 进一步恶化到 `RMSE3D=7.616816 m` 与 `bg_z_err_max_abs=24901.080574 deg/h`；在 `G3` 基础上仅把 `sigma_odo/sigma_nhc_y/sigma_nhc_z` 增大到 `5x` 的 `G4`，`RMSE3D` 回落到 `0.338954 m`；而在 `G4` 基础上再固定 `sg/sa` 的 `G5`，`RMSE3D` 进一步降到 `0.301991 m`、`yaw_err_max_abs` 降到 `85.119234 deg`、`bg_z_err_max_abs` 降到 `15068.234661 deg/h`。这说明“观测权重过强导致姿态-零偏共同失稳”仍是主导项，`sg/sa` 竞争更像 secondary amplifier；同时 `G5` 的 `yaw/bg_z` 仍很大，又表明去掉这 6 个状态并未切断主坏链。 | 以 `G2`、`G4`、`G5` 为主，对 full-window 首个姿态/`bg_z` 明显离轨窗口做 mechanism / acceptance / `NIS` 联审，并对比 `sg/sa` 固定前后是否只改变坏写幅值/协方差而不改变触发顺序。 | `open(strongly_supported_g5_reduces_but_does_not_eliminate_coupling)` |
| `HYP-53` | 在三段式 `G5` 工况下，`ODO/NHC` 权重对 phase2 联合段的影响很小，但会在 phase3 周期断星段显著改变漂移规模；同时，若安装角不再 truth-init，而是依赖 phase2 自行学到正确 mounting，则当前 phase2 对 mounting 的可观性/激励可能并不足以把系统带回 truth-init `6x` 的最好状态。 | `EXP-20260325-data2-staged-g5-no-imu-scale-r1` 的 fresh 结果表明 `4x/5x/6x` 下 `phase2 RMSE3D` 仅 `0.070390 / 0.070832 / 0.071153 m`，spread `0.000763 m`，而 `phase3 RMSE3D` 从 `4.329976` 单调降到 `3.178389 m`，说明 phase3 对 `ODO/NHC` 权重更敏感。按用户要求修正为“总安装角从 0 起跑”的 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2` 又显示：`q_1x` 到 `phase2_end` 也只学到 `mounting_yaw≈0.896198 deg`，且相对 truth-init `6x` 恶化到 `phase3 RMSE3D=4.034528 m`、`bg_z_err_max_abs=171.052868 deg/h`。本轮新增的 `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 更进一步显示：如果只在 phase3 去掉 `ODO/NHC`，保持 phase1/phase2 与 `6x` 完全一致，则 `phase2 RMSE3D` 仍保持 `0.071153 m` 不变，但 `phase3 RMSE3D` 会直接恶化到 `31.657637 m`，首个 `gnss_off_01` 即达到 `RMSE3D=46.206528 m`, `final_err_3d=112.485737 m`。这说明 current staged `G5` 的 phase3 大误差并不是“ODO/NHC 本身在拖后腿”；相反，它们在当前链路下已经显著压住了纯 INS 漂移。剩余主问题因此被分裂成两层：一是纯 INS outage 为何远超按惯导精度预期的 `~2 m`，二是 ODO/NHC 虽然明显有益，但为什么仍只能把 `90 s` 窗口压到 `~5.22 m` 而非预期量级。 | 先对 truth-init `staged_g5_odo_nhc_noise_6x` 与其 phase3 `INS-only` 对照抓取首个 `gnss_off_01 (528866~528956 s)` 的传播/更新日志，拆解 `INS-only` 的位置误差增长率与 `6x` case 的约束贡献；并并行保留 total-zero-init `q_1x` 的 `phase2` 早窗 `dx_mounting_yaw`、`dx_bg_z`、`K` 审计，用于判断当前剩余 gap 中有多少来自 phase2 学习不足。 | `open(strongly_supported_odo_nhc_help_phase3_but_pure_ins_and_remaining_gap_both_need_explanation)` |
| `HYP-54` | 当前 phase3 `INS-only` 远超预期的首要原因，不像是“ECEF 姿态传播漏掉 `omega_en` 或 yaw 符号写反”这类纯机械编排硬错误；更像是 phase2 交给 phase3 的 `roll/pitch + ba/bg` 入口状态已经不够干净，而 phase3 预测里又对 `ba/bg` 施加了 nominal GM 回零，使纯惯导窗口持续失去已学到的 IMU 误差补偿。 | 代码审计显示 [`src/core/ins_mech.cpp`] 与 [`src/core/eskf_engine.cpp`] 的 nominal 传播路径是 `ECEF mechanization + NED error-state`：姿态传播中只减 `omega_ie`，这对 ECEF nav frame 本身是合理的；未发现“少减 `omega_en`”式直接符号 bug。相反，`Predict()` 每步都会调用 `ApplyMarkovNominalPropagation()` 把 `ba/bg/sg/sa` 指数回零；在 fresh `EXP-20260325-data2-staged-g5-no-imu-scale-r2::staged_g5_odo_nhc_noise_6x_phase3_ins_only` 中，首个 outage 期间 `ba_x` 从 `1401.921268 mGal @ 528866.001484` 衰减到 `1367.309516 mGal @ 528955.996726`，`bg_x` 从 `-11.133383 deg/h` 变到 `-10.858513 deg/h`，证明 phase3 纯传播并不是“固定使用 phase2 学到的 bias”。同时同一 fresh phase metrics 里 `gnss_off_01` 的 `yaw_abs_max_deg` 只有 `0.059856 deg`，不支持把首窗 `RMSE3D=46.206528 m` 归咎为 yaw 传播 bug；更值得怀疑的是 roll/pitch / accel-bias 入口误差与 nominal bias 回零叠加。 | 做两类最小 A/B：1. 在 phase3 `INS-only` 控制组中关掉 nominal `ba/bg` 回零，只保留协方差 GM/RW 传播，重跑首个 `gnss_off_01`；2. 补导出首窗的 roll/pitch 误差与 phase3 起点 `ba/bg` truth error，验证是否仅 `~0.1 deg` 级倾斜就足以解释 `90 s` 内几十米漂移。 | `open(code_audit_supports_pre_outage_state_quality_plus_nominal_bias_relaxation_over_pure_yaw_bug)` |

## Session Log

### Historical Session Summary

| span | covered_sessions | condensed takeaway | traceability |
|---|---|---|---|
| `20260304~20260323` | data2/data4、InEKF、mounting、turn-window、bias/scale、输出整理等早期会话 | 旧主线与 supporting history 已归档；当前只保留对 data2 staged 主线仍有直接作用的近期结论。 | `docs/project_history/walkthrough_archive_sessions_20260313.md`, `docs/project_history/walkthrough_archive_sessions_20260318.md`, `docs/project_history/walkthrough_archive_sessions_20260320.md` |
| `20260324-1307-phase2-bridge-probe` | staged bridge / freeze GNSS lever | 证明 `phase2 bridge` 是上游真实机制，`GNSS lever` 更像 downstream jump executor。 | `output/data2_phase2_bridge_probe_r1_20260324/` |
| `20260324-1351-phase2-early-window-burst-audit-r1` | earliest update-chain audit | 证明 `phase2` 首秒是 `NHC=200 / ODO=0`，`GNSS_POS@528277` 才执行 visible jump。 | `output/data2_phase2_early_window_probe_r1_20260324/` |
| `20260324-1509-phase2-entry-intervention-r1` | entry intervention A/B | 把入口 primary knob 收缩到 `odo_scale` nominal/value reseed。 | `output/data2_phase2_entry_intervention_r1_20260324/` |
| `20260324-1708-staged-truth-ablation-r1` | staged truth ablation | mounting 固定近乎中性，`fix_odo_lever_truth` 明显更差，主线转向 `ODO lever` 语义。 | `output/data2_staged_truth_ablation_r1_20260324/` |
| `20260325-0010-staged-truth-ablation-r2-combined` | add combined truth case | `mounting + ODO lever truth` 同时固定后仍显著更差，说明问题不只是 mounting 未固定。 | `output/data2_staged_truth_ablation_r2_20260325/` |
| `20260325-1354-constraint-semantics-bgz-route-audit` | raw semantics audit + bgz route probe | raw truth geometry 不是更差主因；truth-fixed case 的 dominant bad path 是 `ODO/NHC -> bg_z`。 | `output/data2_constraint_semantics_audit_20260325/` |
| `20260325-1406-odo-nhc-bgz-bug-plan` | experiment design | 将 `ODO/NHC -> bg_z` bug 探查拆成 route attribution、Jacobian FD、admission alignment、state competition、mitigation validation 五段实验计划。 | `docs/odo-nhc-bgz-chain-bug-plan.md` |

### session_id: 20260324-2321-rollback-phase2-guard-line

- objective:
  - 废弃 `fixed5s_position_only / phase2_gnss_transition_guard` 这条无效的 phase-transition jump 解决线，回到原始 staged 主线，并清理相关 output、代码和 walkthrough 记录。
- scope:
  - 从 runtime phase 约束中移除 GNSS non-position guard release 相关字段与逻辑；删除 3 个专用分析脚本；删除 9 个相关 output 目录；压缩重写 `walkthrough.md`，只保留当前主线所需证据与假设。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/analyze_data2_phase2_gnss_transition_guard_probe.py` (`deleted`)
  - `scripts/analysis/run_data2_phase2_gnss_transition_guard_probe.py` (`deleted`)
  - `scripts/analysis/plot_phase2_guard_fixed5s_all_states.py` (`deleted`)
  - `walkthrough.md`
- configs:
  - active_baseline=`config_data2_baseline_eskf.yaml`
  - retained_mainline_experiments=`EXP-20260324-data2-staged-truth-ablation-r1`, `EXP-20260324-data2-phase2-entry-intervention-r1`, `EXP-20260324-data2-phase2-early-window-probe-r1`, `EXP-20260324-data2-phase2-bridge-probe-r1`
  - deleted_output_dirs=`output/data2_phase2_gnss_transition_guard_r1_20260324`, `output/data2_phase2_gnss_transition_guard_r2_20260324`, `output/data2_phase2_gnss_transition_guard_r3_20260324`, `output/data2_phase2_gnss_transition_guard_r4_20260324`, `output/data2_phase2_gnss_transition_guard_r5_20260324`, `output/data2_phase2_gnss_transition_guard_r6_20260324`, `output/data2_phase2_gnss_transition_guard_r7_20260324`, `output/data2_fixed5s_position_only_mounting_fixed_fullwindow_r1_20260324`, `output/data2_fixed5s_position_only_mounting_fixed_fullwindow_r2_20260324`
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "fixed5s|phase2_gnss_transition_guard|gnss_pos_release_*|predicted_joint" walkthrough.md scripts src include`
  - `cmake --build build --config Release --target eskf_fusion`
  - `Remove-Item output/data2_phase2_gnss_transition_guard_r*_20260324 -Recurse -Force`
  - `Remove-Item output/data2_fixed5s_position_only_mounting_fixed_fullwindow_r*_20260324 -Recurse -Force`
  - `rg -n "gnss_pos_release_after_first_odo|gnss_pos_release_min_hold_s|gnss_pos_release_after_guarded_gnss_pos_updates|gnss_pos_release_max_predicted_joint|gnss_pos_release_after_consecutive_odo_nhc_pairs" include src scripts walkthrough.md`
  - `Get-ChildItem output -Directory | Where-Object { $_.Name -match "fixed5s|phase2_gnss_transition_guard|mounting_fixed_fullwindow|gnss_transition_guard" }`
- artifacts:
  - retained_only=`output/data2_staged_truth_ablation_r1_20260324/`, `output/data2_phase2_entry_intervention_r1_20260324/`, `output/data2_phase2_early_window_probe_r1_20260324/`, `output/data2_phase2_bridge_probe_r1_20260324/`
  - new_artifacts=`none (cleanup-only session)`
- metrics:
  - deleted_output_dir_count=`9`
  - cleaned_size_mb=`14923.68`
  - build_status=`eskf_fusion Release compile pass`
  - residual_guard_release_refs_in_code=`0`
  - residual_guard_output_dirs=`0`
- observability_notes:
  - no_new_observability_run=`true`
  - retired_branch_statement:
    `fixed5s_position_only`、`count-only guarded GNSS_POS`、`count+threshold`、`predicted-threshold` 与 `mounting-fixed fullwindow` 都未形成更好的主线结果，现统一降为 retired branch。
  - active_state_focus:
    后续主线重新回到 `odo_scale(21)`, `lever_odo(25-27)`, `lever_gnss(28-30)`, `bg/sg/sa(12-20)` 在原始 joint 方案下的残差与可观性解释。
- decision:
  - 不再把 `fixed 5 s position_only` 或任何 guard-release 规则当当前推荐方案。
  - 当前工作基线恢复为“不加方案”的原始 staged 方法；phase-transition jump 仍可作为现象观察，但不继续沿旧 safeguard 线扩展。
  - 代码侧仅保留通用 `gnss_pos_update_mode` 能力，不再保留这条 guard branch 专用 release 逻辑。
- next_step:
  - 以 `EXP-20260324-data2-staged-truth-ablation-r1::control_strict_gate_freeze_gnss_lever` 为基线，直接对比 control 与 `fix_odo_lever_truth` 在 `528276~528320 s`、`528867 s` 两段的 `ODO residual`、lever rotational term、predicted speed 与 `NIS`。

### session_id: 20260324-2338-runtime-phase-noise-logic-audit

- objective:
  - 核查“phase1 结束后固定 `GNSS lever` 并显著降低 bias/scale 相关过程噪声；phase2 结束后除 PVA 外其余状态过程噪声都显著减小，且已固定状态保持固定”这套 staged runtime 逻辑，判断程序是否按该语义实现。
- scope:
  - 仅做代码检查；读取 `src/app/config.cpp`、`src/app/pipeline_fusion.cpp`、`scripts/analysis/run_data2_staged_estimation.py`、`scripts/analysis/run_data2_staged_truth_ablation_probe.py` 与 `config_data2_baseline_eskf.yaml`，不运行新实验、不修改求解器逻辑。
- changed_files:
  - `walkthrough.md`
- configs:
  - inspected_baseline=`config_data2_baseline_eskf.yaml`
  - inspected_runtime_scripts=`scripts/analysis/run_data2_staged_estimation.py`, `scripts/analysis/run_data2_staged_truth_ablation_probe.py`
  - active_mainline_reference=`EXP-20260324-data2-staged-truth-ablation-r1::control_strict_gate_freeze_gnss_lever`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content src\app\pipeline_fusion.cpp`
  - `Get-Content src\app\config.cpp`
  - `Get-Content scripts\analysis\run_data2_staged_estimation.py`
  - `Get-Content scripts\analysis\run_data2_staged_truth_ablation_probe.py`
  - `Get-Content config_data2_baseline_eskf.yaml`
  - `rg -n "runtime_phases|ApplyAblationToNoise|ApplyRuntimeNoiseOverride|build_effective_noise|sync_runtime_controls|scaled_noise_override|freeze_gnss_lever" src scripts config_data2_baseline_eskf.yaml walkthrough.md`
- artifacts:
  - new_artifacts=`none (inspection-only session)`
- metrics:
  - inspection_only=`true`
  - runtime_noise_scale_default=`0.1`
  - phase2_phase3_reduced_noise_keys=`sigma_ba,sigma_bg,sigma_sg,sigma_sa,sigma_gnss_lever_arm`
  - phase2_phase3_not_reduced_by_default=`sigma_odo_scale,sigma_mounting,sigma_lever_arm`
- observability_notes:
  - runtime_framework:
    `runtime_phases` 会被解析并在每个 IMU 时刻重算 `effective_ablation / effective_constraints / effective_noise`；随后调用 `engine.SetNoiseParams(...)` 与 `engine.SetStateMask(...)`，因此 phase 切换本身是按时生效的。
  - state_block_mapping:
    `ba/bg/sg/sa(9-20)` 的 reduced-noise 逻辑存在；`odo_scale(21)`、`mounting(22-24)`、`lever_odo(25-27)`、`lever_gnss(28-30)` 是否冻结/降噪取决于脚本下发的 `runtime_phases`，不是 C++ 端默认全覆盖。
  - phase_policy_gap:
    原始 `run_data2_staged_estimation.py` 的 phase2/phase3 只把 `GNSS lever + IMU bias/scale` 过程噪声降到 `0.1x`，并不会把“除 PVA 外的所有状态量”统一降噪；同时它也不会在 phase2/phase3 固定 `GNSS lever`。
  - active_mainline_gap:
    当前主线 `run_data2_staged_truth_ablation_probe.py` 会在 phase2/phase3 固定 `GNSS lever`，但仍只对 `ba/bg/sg/sa` 这类 bias/scale 噪声做 `0.1x` 缩放；`odo_scale/mounting/lever_odo` 并不会在 phase2/phase3 自动保持 phase1 的冻结。
- decision:
  - C++ 运行时机制实现正常：phase 噪声 override 先应用，随后 ablation 再把被冻结状态的过程噪声清零，因此“已固定状态保持固定”这件事在机制上是成立的。
  - 但当前脚本定义并没有完整实现“phase2 后除 PVA 外所有非 PVA 状态都显著降噪”这套策略；真正被降噪的默认只有 `ba/bg/sg/sa` 与在原始 staged 脚本中的 `GNSS lever`。
  - 若以当前主线 control 为准，则“phase1 结束后固定 `GNSS lever`”是成立的；若以原始 `run_data2_staged_estimation.py` 为准，则这条也不成立，因为它只降 `GNSS lever` 噪声、不做冻结。
- next_step:
  - 若后续要让 staged 逻辑严格符合“phase2 后所有非 PVA 状态统一小过程噪声、且已冻结保持冻结”的语义，应直接修改 `build_runtime_phases()` / `scaled_noise_override()`：把 `sigma_odo_scale`、`sigma_mounting*`、`sigma_lever_arm*` 纳入 phase2/phase3 的 noise override，并明确哪些 ablation 需要跨 phase 持续保留。

### session_id: 20260324-2349-odo-lever-truth-and-nhc-audit

- objective:
  - 按当前主线检查“固定 ODO 杆臂真值是否能改善 phase2 表现”，并确认 NHC 模块是否包含 ODO 杆臂改正。
- scope:
  - 复核 `EXP-20260324-data2-staged-truth-ablation-r1` 的 `fix_odo_lever_truth` 结果；检查 `src/core/measurement_models_uwb.cpp`、`src/app/pipeline_fusion.cpp`、`src/app/initialization.cpp` 中 ODO/NHC 杆臂使用链路；不新增实验、不修改代码。
- changed_files:
  - `walkthrough.md`
- configs:
  - inspected_experiment=`EXP-20260324-data2-staged-truth-ablation-r1`
  - inspected_summary=`output/data2_staged_truth_ablation_r1_20260324/summary.md`
  - active_control_case=`control_strict_gate_freeze_gnss_lever`
  - inspected_truth_fix_case=`fix_odo_lever_truth`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content output\data2_staged_truth_ablation_r1_20260324\summary.md`
  - `Import-Csv output\data2_staged_truth_ablation_r1_20260324\case_metrics.csv`
  - `rg -n "NHC|nhc|lever|odo_lever|lever_arm|wheel|non-holonomic" src include -g "*.cpp" -g "*.h"`
  - `Get-Content src\core\measurement_models_uwb.cpp`
  - `Get-Content src\app\pipeline_fusion.cpp`
  - `Get-Content src\app\initialization.cpp`
  - `Get-Content scripts\analysis\run_data2_staged_truth_ablation_probe.py`
- artifacts:
  - inspected_artifacts=`output/data2_staged_truth_ablation_r1_20260324/summary.md`, `output/data2_staged_truth_ablation_r1_20260324/case_metrics.csv`
  - new_artifacts=`none (inspection-only session)`
- metrics:
  - artifact_generated_at=`2026-03-24T17:08:46`
  - control_phase2_rmse_3d_m=`0.102727`
  - fix_odo_lever_truth_phase2_rmse_3d_m=`0.110831`
  - control_phase3_rmse_3d_m=`23.838294`
  - fix_odo_lever_truth_phase3_rmse_3d_m=`95.854081`
  - control_first_divergence_start_t=`528867.106`
  - fix_odo_lever_truth_first_divergence_start_t=`528314.000008`
  - fix_odo_lever_truth_nhc_accept_ratio=`0.696286`
  - fix_odo_lever_truth_odo_accept_ratio=`0.947010`
- observability_notes:
  - experiment_check:
    `fix_odo_lever_truth` 已在当前主线 fresh 跑过，而且不是改善而是显著变差；phase2 起 `yaw/bg_z/sa_x` 异常明显放大，支持 `ODO lever` 真值语义与模型参考点不一致。
  - nhc_model_chain:
    NHC 残差模型显式使用 `v_wheel_b = v_b + omega_nb_b.cross(lever_arm)`，再经 `C_b^v` 投到车体系，且雅可比对 `lever/bg/sg/mounting/att` 都有对应列；因此 NHC 本体并非“完全没有 ODO 杆臂改正”。
  - nhc_runtime_behavior:
    `RunNhcUpdate()` 调用的就是上述 NHC 模型；但其低速/异常速度预检使用的是 `v_b`，不是 `v_wheel_b/v_v`。此外在 weak-excitation 下，鲁棒包装会把 mounting 与 ODO lever 的雅可比列清零，因此该阶段 NHC 会继续使用当前杆臂参与残差预测，但不会依靠 NHC 去修正杆臂。
  - fixed_truth_path:
    `fix_odo_lever_truth` case 同时把 `constraints.odo_lever_arm` 与 `init.lever_arm0` 设成 truth，并打开 `fusion.ablation.disable_odo_lever_arm=true`；初始化时 `x0.lever_arm` 会从 `init/constraints` 注入状态，因此该 case 确实是在整个会话中把 ODO 杆臂固定成 truth。
- decision:
  - “把 ODO 杆臂固定为真值看看”这一步已经有主线证据，且结果明确为负：固定真值后程序更差，不支持“只是因为 ODO 杆臂估计慢，所以把它固定就会改善”。
  - NHC 模块中确实有 ODO 杆臂改正，问题更像 `ODO/NHC` 所共享的杆臂/参考点/车体系语义与 README truth 不一致，而不是 NHC 完全漏掉杆臂补偿。
  - 下一步应优先核对 `wheel-speed` 与 `NHC` 使用的速度参考点和坐标语义，而不是继续做“固定杆臂真值是否有效”的重复实验。
- next_step:
  - 直接对账 `ComputeNhcModel()` 与 `ComputeOdoModel()` 中 `v_b`、`v_wheel_b`、`v_v` 的物理含义，并核对 README `odo_lever_truth` 是否真的是从 IMU 指向轮速计参考点、且与 FRD / `C_b^v` 约定一致。

### session_id: 20260325-0010-truth-ablation-case-clarify

- objective:
  - 澄清 `EXP-20260324-data2-staged-truth-ablation-r1` 是否已经包含“安装角真值 + ODO 杆臂真值同时固定”的 case。
- scope:
  - 只检查 `run_data2_staged_truth_ablation_probe.py` 的 `CASE_SPECS` 与 truth-fix 配置分支，并对照 `summary.md` 的 case 列表；不新增实验、不改脚本。
- changed_files:
  - `walkthrough.md`
- configs:
  - inspected_script=`scripts/analysis/run_data2_staged_truth_ablation_probe.py`
  - inspected_summary=`output/data2_staged_truth_ablation_r1_20260324/summary.md`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content scripts\analysis\run_data2_staged_truth_ablation_probe.py`
  - `Get-Content output\data2_staged_truth_ablation_r1_20260324\summary.md`
- artifacts:
  - inspected_artifacts=`output/data2_staged_truth_ablation_r1_20260324/summary.md`
  - new_artifacts=`none (inspection-only session)`
- metrics:
  - case_count=`3`
  - case_ids=`control_strict_gate_freeze_gnss_lever,fix_mounting_truth,fix_odo_lever_truth`
- observability_notes:
  - truth_case_definition:
    `CASE_SPECS` 只有三个 case：control、仅 mounting truth、仅 ODO lever truth；没有“同时 fixed mounting + fixed ODO lever”这一 case。
  - branch_logic:
    `fix_mounting_truth` 与 `fix_odo_lever_truth` 是两个独立布尔分支，代码上理论可以同时为真，但当前 `CASE_SPECS` 没有定义这样的组合。
- decision:
  - `EXP-20260324-data2-staged-truth-ablation-r1` 不是“安装角和 ODO 杆臂都设置真值”的实验。
  - 该实验中：
    `fix_mounting_truth` 只固定安装角真值；
    `fix_odo_lever_truth` 只固定 ODO 杆臂真值；
    两者没有组合到同一个 case。
- next_step:
  - 如果要验证“安装角真值 + ODO 杆臂真值同时固定”的效果，需要在 `run_data2_staged_truth_ablation_probe.py` 里新增一个同时设置 `fix_mounting_truth=True` 和 `fix_odo_lever_truth=True` 的 case 后重跑。

### session_id: 20260325-0010-staged-truth-ablation-r2-combined

- objective:
  - 新增并运行“安装角真值 + ODO 杆臂真值同时固定”的组合 case，输出新的 truth-ablation 对比图像与量化指标。
- scope:
  - 修改 `scripts/analysis/run_data2_staged_truth_ablation_probe.py`：新增 `fix_mounting_and_odo_lever_truth` case，并允许 `truth_note` 同时记录两个 truth fix；随后运行新实验 `EXP-20260325-data2-staged-truth-ablation-r2`。
- changed_files:
  - `scripts/analysis/run_data2_staged_truth_ablation_probe.py`
  - `walkthrough.md`
- configs:
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - exp_id=`EXP-20260325-data2-staged-truth-ablation-r2`
  - output_dir=`output/data2_staged_truth_ablation_r2_20260325/`
  - new_case=`fix_mounting_and_odo_lever_truth`
- commands:
  - `Get-Content walkthrough.md`
  - `python -m py_compile scripts\analysis\run_data2_staged_truth_ablation_probe.py`
  - `python scripts\analysis\run_data2_staged_truth_ablation_probe.py --exp-id EXP-20260325-data2-staged-truth-ablation-r2 --output-dir output\data2_staged_truth_ablation_r2_20260325`
  - `Get-Content output\data2_staged_truth_ablation_r2_20260325\summary.md`
  - `Import-Csv output\data2_staged_truth_ablation_r2_20260325\case_metrics.csv`
  - `Get-ChildItem output\data2_staged_truth_ablation_r2_20260325\plots`
- artifacts:
  - summary=`output/data2_staged_truth_ablation_r2_20260325/summary.md`
  - case_metrics=`output/data2_staged_truth_ablation_r2_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_truth_ablation_r2_20260325/phase_metrics.csv`
  - plots=`output/data2_staged_truth_ablation_r2_20260325/plots/nav_errors_compare.png`, `output/data2_staged_truth_ablation_r2_20260325/plots/key_states_compare.png`, `output/data2_staged_truth_ablation_r2_20260325/plots/key_states_jump_zoom.png`, `output/data2_staged_truth_ablation_r2_20260325/plots/velocity_vehicle_compare.png`
  - combined_case_config=`output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/config_fix_mounting_and_odo_lever_truth.yaml`
- metrics:
  - artifact_generated_at=`2026-03-25T00:16:54`
  - combined_phase2_rmse_3d_m=`0.109494`
  - combined_phase3_rmse_3d_m=`96.781242`
  - combined_first_divergence_start_t=`528315.655012`
  - combined_odo_accept_ratio=`0.946696`
  - combined_nhc_accept_ratio=`0.694537`
  - comparison_vs_fix_odo_only:
    `phase2 RMSE3D 0.110831 -> 0.109494`、`first_divergence 528314.000008 -> 528315.655012`、`phase3 RMSE3D 95.854081 -> 96.781242`
- observability_notes:
  - state_block_freeze:
    组合 case 同时冻结 `mounting(22-24)` 与 `lever_odo(25-27)`；`phase2/phase3` 继续冻结 `lever_gnss(28-30)`，并保持 strict weak-excitation gate。
  - result_interpretation:
    即便把 `mounting` 与 `ODO lever` 同时固定为 truth，phase2 后的姿态/`bg_z`/`sa_x` 异常仍然存在，且整体表现与 `fix_odo_lever_truth` 一样显著差于 control。
  - implication:
    问题不支持“只要把 mounting 和 ODO lever 都设成真值就能恢复正常”；更像当前 README truth 与 ODO/NHC 观测模型的速度参考点/坐标语义并不一致。
- decision:
  - “安装角真值 + ODO 杆臂真值同时固定”不是有效修复；该组合只带来极轻微的 phase2 指标变化，但 phase3 仍严重发散。
  - 当前主线继续保持 `control_strict_gate_freeze_gnss_lever` 作为 working baseline，不把 combined truth case 当候选修复方案。
- next_step:
  - 直接对账 combined case 与 control 在 phase2 入口段的 `v_b / v_wheel_b / v_v`、`ODO residual`、`NHC residual` 与 `NIS`，确认究竟是 README truth 语义不对，还是 `C_b^v` / 速度参考点定义有额外失配。

### session_id: 20260325-1354-constraint-semantics-bgz-route-audit

- objective:
  - 审查“为什么 `INS/GNSS/ODO/NHC` 比 `INS/GNSS` 差，且把 `ODO lever / mounting` 固定真值后仍更差”，区分 raw measurement semantics 问题与滤波更新通路问题。
- scope:
  - 读取 `README.md`、`config_data2_baseline_eskf.yaml`、`src/core/measurement_models_uwb.cpp`、`src/app/pipeline_fusion.cpp`；新增一个不跑滤波器的 raw constraint semantics audit；再对 `fix_mounting_and_odo_lever_truth` 做单 case debug probe，仅切断 `ODO/NHC -> bg_z` 路径，不修改源码。
- changed_files:
  - `walkthrough.md`
- configs:
  - base_config=`config_data2_baseline_eskf.yaml`
  - base_config_hash=`2E03B6B4DF74D2A76C460E7903B1FD1C0878B1874CC302D52F9BCB51886DF847`
  - staged_control_config=`output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/control_strict_gate_freeze_gnss_lever/config_control_strict_gate_freeze_gnss_lever.yaml`
  - staged_fix_odo_config=`output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/fix_odo_lever_truth/config_fix_odo_lever_truth.yaml`
  - staged_combined_truth_config=`output/data2_staged_truth_ablation_r2_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/config_fix_mounting_and_odo_lever_truth.yaml`
  - bgz_probe_config=`output/data2_constraint_semantics_audit_20260325/config_fix_mounting_and_odo_lever_truth_disable_bgz.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content README.md`
  - `Get-Content config_data2_baseline_eskf.yaml`
  - `rg -n "ComputeOdoModel|ComputeNhcModel|ZeroExtrinsicSensitivity|IsWeakExcitation|MeetsWeakExcitationThresholds|freeze_extrinsics_when_weak_excitation|v_wheel_b|v_b" src\core\measurement_models_uwb.cpp src\app\pipeline_fusion.cpp`
  - `python - <<PY ... raw truth semantics audit ... PY`
  - `python - <<PY ... combined truth + disable ODO/NHC->bg_z config writer ... PY`
  - `build\Release\eskf_fusion.exe --config output\data2_constraint_semantics_audit_20260325\config_fix_mounting_and_odo_lever_truth_disable_bgz.yaml`
  - `python - <<PY ... phase metrics extractor for bgz probe ... PY`
- artifacts:
  - semantics_summary=`output/data2_constraint_semantics_audit_20260325/summary.md`
  - semantics_csv=`output/data2_constraint_semantics_audit_20260325/constraint_truth_semantics_summary.csv`
  - jacobian_csv=`output/data2_constraint_semantics_audit_20260325/constraint_jacobian_coupling_summary.csv`
  - bgz_probe_summary=`output/data2_constraint_semantics_audit_20260325/bgz_route_probe_summary.md`
  - bgz_probe_log=`output/data2_constraint_semantics_audit_20260325/run_fix_mounting_and_odo_lever_truth_disable_bgz.log`
  - bgz_probe_sol=`output/data2_constraint_semantics_audit_20260325/SOL_fix_mounting_and_odo_lever_truth_disable_bgz.txt`
  - bgz_probe_state_series=`output/data2_constraint_semantics_audit_20260325/state_series_fix_mounting_and_odo_lever_truth_disable_bgz.csv`
  - artifact_mtime=`summary.md @ 2026-03-25 13:48:12 local; constraint_truth_semantics_summary.csv @ 2026-03-25 13:48:12 local; constraint_jacobian_coupling_summary.csv @ 2026-03-25 13:50:51 local; run_fix_mounting_and_odo_lever_truth_disable_bgz.log @ 2026-03-25 13:52:39 local; bgz_route_probe_summary.md @ 2026-03-25 13:54:02 local`
- metrics:
  - raw_truth_phase2_comparison:
    control `odo_res_rmse=0.129038 m/s`, `nhc_lat_rmse=0.110929 m/s`, `odo_accept_ratio_truth_model=0.959300`, `nhc_accept_ratio_truth_model=0.997110`;
    combined truth `odo_res_rmse=0.111925 m/s`, `nhc_lat_rmse=0.020724 m/s`, `odo_accept_ratio_truth_model=0.981270`, `nhc_accept_ratio_truth_model=0.998260`
  - jacobian_coupling_phase2:
    control `odo_H_bgz_abs_mean=0.000000`, `nhc_H_bgz_col_norm_mean=0.000000`;
    fix_odo_truth `1.004527`, `0.175859`;
    combined truth `1.002805`, `0.185425`
  - bgz_probe_recovery:
    combined truth baseline `phase2 RMSE3D=0.109494 m`, `phase3 RMSE3D=96.781242 m`, `phase2 yaw_abs_max=25.369244 deg`, `phase3 bg_z_abs_max=3073.377887 deg/h`, `first_divergence_start_t=528315.655012`, `odo_accept_ratio=0.946696`, `nhc_accept_ratio=0.694537`
    probe `phase2 RMSE3D=0.089440 m`, `phase3 RMSE3D=3.610158 m`, `phase2 yaw_abs_max=1.129489 deg`, `phase3 bg_z_abs_max=54.749082 deg/h`, `first_divergence_start_t=NA`, `odo_accept_ratio=0.999553`, `nhc_accept_ratio=0.999426`
- observability_notes:
  - state_block_mapping:
    当前主坏链集中在 `lever_odo(25-27)` 非零时对 `bg(12-14)`、`sg(15-17)` 与 `att(6-8)` 打开的 Jacobian；control 因 `lever_odo=0`，这条 `bg_z` 通路在模型里天然为零。
  - schedule_effect:
    本轮 staged 口径仍是 `phase1` 关闭 `ODO/NHC`，`phase2/phase3` 使用 `fusion.gnss_schedule.*` 周期开关 GNSS，并冻结 `lever_gnss(28-30)`；因此本轮恢复效果不能归因到 GNSS lever 写回，而是 ODO/NHC 上游链路。
  - weak_excitation_note:
    当前主线 config 明确是 `freeze_extrinsics_when_weak_excitation=false`；strict weak gate 实际上只会整段停用 `ODO/NHC`，一旦 update 被允许，nonzero lever 诱发的 `H_bg/H_sg/H_att` 仍全部生效。
  - runtime_precheck_gap:
    `RunNhcUpdate()` 的低速/侧向预检仍用 `v_b` 而不是 `v_wheel_b/v_v`；这会让 admission logic 与真实 residual 参考点不完全一致，但它已不是本轮最强根因。
- decision:
  - “把 mounting/ODO lever 固定真值后仍更差”并不主要说明 truth geometry 自身错误；raw truth audit 恰好显示 truth geometry 的 `ODO/NHC` 物理一致性比 control 更好。
  - 当前最强主因是：nonzero `lever_odo` 使 `ODO/NHC` 更新把 residual 强写进 `bg_z` 等活跃状态；在 truth-fixed case 中，外参本身又被冻结，residual 无法通过 `mounting/lever` 吸收，最终把 `bg_z`、姿态和尺度类状态推坏。
  - control case 之所以“看起来更稳”，更像是因为 `lever_odo=0` 把这条坏的 `bg_z` 路由从模型里关掉了，而不是因为 control 的几何更物理正确。
- next_step:
  - 先把 `EXP-20260325-data2-bgz-route-probe-r1` 的 debug 结果收敛成正式、非 debug 的 `ODO/NHC -> bg_z` 可观性门控/禁更策略；随后再单独排查剩余 `sg/att` 次级通路，以及把 NHC 预检从 `v_b` 改到 `v_wheel_b/v_v` 的影响。

### session_id: 20260325-1406-odo-nhc-bgz-bug-plan

- objective:
  - 设计一组可执行实验，把 `ODO/NHC -> bg_z` 坏链从“现象解释”推进到“bug 定位”。
- scope:
  - 仅做计划与编排，不运行新实验、不改求解器逻辑；基于当前 fresh 证据输出一份正式计划文件，并把拟执行实验登记到 `walkthrough.md`。
- changed_files:
  - `docs/odo-nhc-bgz-chain-bug-plan.md`
  - `walkthrough.md`
- configs:
  - active_baseline=`config_data2_baseline_eskf.yaml`
  - planning_reference_cases=`EXP-20260325-data2-staged-truth-ablation-r2::control_strict_gate_freeze_gnss_lever`, `EXP-20260325-data2-staged-truth-ablation-r2::fix_mounting_and_odo_lever_truth`
  - planned_experiments=`EXP-20260325-data2-bgz-route-decomposition-r1`, `EXP-20260325-data2-bgz-jacobian-fd-audit-r1`, `EXP-20260325-data2-nhc-admission-alignment-r1`, `EXP-20260325-data2-bgz-state-competition-r1`, `EXP-20260325-data2-bgz-mitigation-validation-r1`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\skills\planner\SKILL.md`
  - `Get-Date -Format "yyyyMMdd-HHmm"`
  - `Get-Date -Format "yyyy-MM-dd HH:mm:ss"`
  - `Get-Item docs\odo-nhc-bgz-chain-bug-plan.md`
  - `Get-Content docs\odo-nhc-bgz-chain-bug-plan.md`
- artifacts:
  - plan_doc=`docs/odo-nhc-bgz-chain-bug-plan.md`
  - artifact_mtime=`docs/odo-nhc-bgz-chain-bug-plan.md @ 2026-03-25 14:08:05 local`
- metrics:
  - plan_only=`true`
  - sprint_count=`5`
  - planned_experiment_count=`5`
  - newly_registered_hypotheses=`HYP-50`, `HYP-51`
- observability_notes:
  - planned_route_attribution:
    先拆 `ODO` 与 `NHC` 分别对 `bg_z(14)` 的 Jacobian 与 state-update 通路，再看 first bad write 来自哪一支。
  - planned_state_blocks:
    重点追踪 `bg_z(14)`、`sg_z(17)`、`att_z(8)`、`mounting_yaw(24)`、`lever_odo_y(26)`，并用 `mounting/lever` freeze-open 对照判断 residual 最终流向。
  - planned_schedule_control:
    所有 planned experiments 默认沿用现有 staged schedule、`strict weak gate`、`phase2/phase3 freeze GNSS lever` 与 `init.odo_scale=1.0`，避免把调度变化混入 bug 归因。
- decision:
  - 不再直接从“解释”跳到“修复”；先按 `route attribution -> Jacobian FD -> admission alignment -> state competition -> mitigation validation` 顺序做收缩。
  - `HYP-50` 与 `HYP-51` 已单独立项，避免把 Jacobian bug 与 admission mismatch 混在同一轮实验里。
- next_step:
  - 第一轮直接执行 `EXP-20260325-data2-bgz-route-decomposition-r1`，因为它能最快回答“到底是 ODO 写坏、NHC 写坏，还是两者叠加写坏”。 

### session_id: 20260325-1452-bgz-route-decomposition-r1

- objective:
  - 执行 `EXP-20260325-data2-bgz-route-decomposition-r1`，把 `fix_odo_lever_truth` 和 `fix_mounting_and_odo_lever_truth` 同时纳入 stressed case，对比 `ODO bg_z Jacobian`、`ODO bg_z state update`、`NHC bg_z state update` 三条路径。
- scope:
  - 为 `mechanism log` 增加时间窗配置，避免全程逐次日志过大；新增 `scripts/analysis/run_data2_bgz_route_decomposition.py`；编译 `eskf_fusion.exe`；运行 13 个 route-decomposition case 并生成 summary/csv；最后用 `--reuse-existing` 复用已跑完的 solver artifacts 进行汇总收尾。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/diagnostics.cpp`
  - `scripts/analysis/run_data2_bgz_route_decomposition.py`
  - `walkthrough.md`
- configs:
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - exp_id=`EXP-20260325-data2-bgz-route-decomposition-r1`
  - output_dir=`output/data2_bgz_route_decomposition_r1_20260325/`
  - shared_controls=`strict weak gate`, `phase2/phase3 freeze GNSS lever`, `fusion.init.odo_scale=1.0`
  - stressed_cases=`fix_odo_lever_truth`, `fix_mounting_and_odo_lever_truth`
  - mechanism_window=`528276.000000~528336.000000`
  - debug_variants=`disable_odo_bgz_jacobian`, `disable_odo_bgz_state_update`, `disable_nhc_bgz_state_update`, `disable_both_bgz_state_update`, `disable_odo_jacobian_and_both_bgz_state_update`
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "enable_mechanism_log|mechanism|Correct\\(|debug_odo_disable_bgz_jacobian|debug_odo_disable_bgz_state_update|debug_nhc_disable_bgz_state_update|delta_bg_z|H_bgz|K_bgz|bg_z" src\app\diagnostics.cpp src\app\pipeline_fusion.cpp include\app\fusion.h src\app\config.cpp scripts\analysis\run_data2_staged_truth_ablation_probe.py`
  - `python -m py_compile scripts\analysis\run_data2_bgz_route_decomposition.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_bgz_route_decomposition.py`
  - `python scripts\analysis\run_data2_bgz_route_decomposition.py --reuse-existing`
  - `Get-Content output\data2_bgz_route_decomposition_r1_20260325\summary.md`
- artifacts:
  - summary=`output/data2_bgz_route_decomposition_r1_20260325/summary.md`
  - case_metrics=`output/data2_bgz_route_decomposition_r1_20260325/case_metrics.csv`
  - bgz_step_metrics=`output/data2_bgz_route_decomposition_r1_20260325/bgz_step_metrics.csv`
  - manifest=`output/data2_bgz_route_decomposition_r1_20260325/manifest.json`
  - mechanism_fix_odo=`output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_odo_lever_truth/SOL_fix_odo_lever_truth_mechanism.csv`
  - mechanism_combined=`output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/SOL_fix_mounting_and_odo_lever_truth_mechanism.csv`
  - artifact_mtime=`summary.md @ 2026-03-25 14:51:36 local; case_metrics.csv @ 2026-03-25 14:51:36 local; bgz_step_metrics.csv @ 2026-03-25 14:51:36 local; manifest.json @ 2026-03-25 14:51:36 local`
- metrics:
  - control_window_baseline:
    `first_bgz_write_tag=NHC`, `first_bgz_write_t=528276.004906`, `first_bgz_write_delta_bgz=-8.377651 deg/h`, `total_cum_abs_delta_bgz=39.234041 deg/h`
  - stress_a_fix_odo_lever_truth:
    baseline `phase3 RMSE3D=95.854081 m`, `first_divergence_start_t=528314.000008`, `odo_accept_ratio=0.947010`, `nhc_accept_ratio=0.696286`, `total_cum_abs_delta_bgz=112.221256 deg/h`
    `disable_odo_bgz_jacobian -> phase3 RMSE3D=3.390878 m`, `first_divergence_start_t=NA`, `odo_accept_ratio=0.999559`, `nhc_accept_ratio=0.999426`
    `disable_odo_bgz_state_update -> phase3 RMSE3D=96.325394 m`
    `disable_nhc_bgz_state_update -> phase3 RMSE3D=234.602439 m`, `first_divergence_start_t=528311.004998`
    `disable_both_bgz_state_update -> phase3 RMSE3D=181.394071 m`
    `disable_odo_jacobian_and_both_bgz_state_update -> phase3 RMSE3D=3.512375 m`, `first_divergence_start_t=NA`
  - stress_b_fix_mounting_and_odo_lever_truth:
    baseline `phase3 RMSE3D=96.781242 m`, `first_divergence_start_t=528315.655012`, `odo_accept_ratio=0.946696`, `nhc_accept_ratio=0.694537`, `total_cum_abs_delta_bgz=105.586542 deg/h`
    `disable_odo_bgz_jacobian -> phase3 RMSE3D=3.252093 m`, `first_divergence_start_t=NA`, `odo_accept_ratio=0.999553`, `nhc_accept_ratio=0.999426`
    `disable_odo_bgz_state_update -> phase3 RMSE3D=96.945484 m`
    `disable_nhc_bgz_state_update -> phase3 RMSE3D=240.441111 m`, `first_divergence_start_t=528312.410001`
    `disable_both_bgz_state_update -> phase3 RMSE3D=177.880903 m`
    `disable_odo_jacobian_and_both_bgz_state_update -> phase3 RMSE3D=3.610158 m`, `first_divergence_start_t=NA`
- observability_notes:
  - state_blocks:
    本轮直接拆的是 `bg_z(14)` 的三条 route：`ODO H_bgz`、`ODO bg_z direct state update`、`NHC bg_z direct state update`；同时仍在冻结 `mounting(22-24)` / `lever_odo(25-27)` / `lever_gnss(28-30)` 的 staged 主线下观测其外溢到 `att/sg` 的效果。
  - schedule_effect:
    所有 case 都保持 `phase1` 关闭 `ODO/NHC`，`phase2/phase3` 开启 strict weak gate 与 GNSS periodic outage；mechanism log 只记录 `phase2` 入口 `528276~528336 s`，保证 first-write attribution 不被长时输出规模污染。
  - route_decomposition:
    两组 stressed case 的 `first_bgz_write` 都仍然是 `NHC@528276.004906`，但单独禁用 `NHC bg_z state update` 会显著更差；因此“first write tag = NHC”不等于“primary bad route = NHC state update”。
  - primary_upstream_path:
    两组 stressed case 里，只要把 `ODO bg_z Jacobian` 清零，`phase3 RMSE3D` 就从 `~96 m` 降到 `~3.3 m` 且不再发散；而单独清掉 `ODO bg_z state update` 几乎无效，说明 primary 问题在 `ODO Jacobian` 上游，不在 `bg_z` 的 direct write mask。
  - secondary_compensatory_path:
    `disable_nhc_bgz_state_update` 与 `disable_both_bgz_state_update` 都会明显恶化，说明 NHC 的 direct `bg_z` 更新更像下游补偿/缓冲，而不是当前主坏源。
  - control_contrast:
    control case 在同一窗口内也有小幅 `NHC/ODO -> bg_z` 写入，但 `h_col_bgz_norm` 量级仅 `NHC≈0.001368`, `ODO≈0.000223`，远低于 stressed case 的 `NHC≈0.176~0.186`, `ODO≈1.004~1.006`；问题核心仍是 truth lever 打开的高增益 Jacobian。
- decision:
  - `EXP-20260325-data2-bgz-route-decomposition-r1` 已经回答了“到底是 ODO 写坏、NHC 写坏，还是两者叠加写坏”：真正起 primary 作用的是 `ODO bg_z Jacobian` 上游链，而不是 `ODO/NHC` 谁在 `bg_z` 上做了 direct state update。
  - 仅以 `update_mask` 禁 `bg_z` 并不能修复；正式修复优先级应从“禁写 `bg_z`”切换为“核对/门控 `ODO H_bgz/H_att/H_sg`”。
  - `NHC` 参考点 admission mismatch 仍值得审计，但已从 primary root cause 降为 secondary issue。
- next_step:
  - 立刻执行 `EXP-20260325-data2-bgz-jacobian-fd-audit-r1`，把 `ODO` 的 `H_bgz/H_att_z/H_sg_z` 放在首优先级做 analytic vs FD；随后再做 `EXP-20260325-data2-nhc-admission-alignment-r1`，确认 NHC admission mismatch 是否只是次级问题。

### session_id: 20260325-1519-odo-jacobian-fd-audit-r1

- objective:
  - 执行 `EXP-20260325-data2-bgz-jacobian-fd-audit-r1`，对 `ComputeOdoModel()` / `ComputeNhcModel()` 在三组 staged case、四个关键时刻的 analytic Jacobian 做 finite-difference 审计，并仔细检查 `ODO H_bgz/H_sgz/H_att_z`。
- scope:
  - 新增一个窄用途 C++ 审计程序 `odo_nhc_bgz_jacobian_fd`，直接复用真实量测模型与当前误差注入约定；编译新 target；运行默认三组 config 与四个目标时刻；输出 `fd_vs_analytic.csv`、`sample_summary.csv`、`odo_component_breakdown.csv` 和 `summary.md`。
- changed_files:
  - `apps/odo_nhc_bgz_jacobian_fd_main.cpp`
  - `CMakeLists.txt`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-bgz-jacobian-fd-audit-r1`
  - solver_like_app=`build/Release/odo_nhc_bgz_jacobian_fd.exe`
  - output_dir=`output/data2_bgz_jacobian_fd_audit_r1_20260325/`
  - audited_configs=`output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/control_strict_gate_freeze_gnss_lever/config_control_strict_gate_freeze_gnss_lever.yaml`, `output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_odo_lever_truth/config_fix_odo_lever_truth.yaml`, `output/data2_bgz_route_decomposition_r1_20260325/artifacts/cases/fix_mounting_and_odo_lever_truth/config_fix_mounting_and_odo_lever_truth.yaml`
  - audited_timestamps=`528276.004906`, `528314.000008`, `528315.655012`, `528867.106000`
  - dataset_time_window=`528076.009368~530488.900000`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content apps\odo_nhc_bgz_jacobian_fd_main.cpp`
  - `Get-Content src\core\measurement_models_uwb.cpp`
  - `Get-Content CMakeLists.txt`
  - `cmake --build build --config Release --target odo_nhc_bgz_jacobian_fd`
  - `build\Release\odo_nhc_bgz_jacobian_fd.exe`
  - `Get-Content output\data2_bgz_jacobian_fd_audit_r1_20260325\summary.md`
  - `Import-Csv output\data2_bgz_jacobian_fd_audit_r1_20260325\sample_summary.csv`
  - `Import-Csv output\data2_bgz_jacobian_fd_audit_r1_20260325\fd_vs_analytic.csv`
  - `Import-Csv output\data2_bgz_jacobian_fd_audit_r1_20260325\odo_component_breakdown.csv`
- artifacts:
  - summary=`output/data2_bgz_jacobian_fd_audit_r1_20260325/summary.md`
  - case_meta=`output/data2_bgz_jacobian_fd_audit_r1_20260325/case_meta.csv`
  - fd_vs_analytic=`output/data2_bgz_jacobian_fd_audit_r1_20260325/fd_vs_analytic.csv`
  - sample_summary=`output/data2_bgz_jacobian_fd_audit_r1_20260325/sample_summary.csv`
  - odo_component_breakdown=`output/data2_bgz_jacobian_fd_audit_r1_20260325/odo_component_breakdown.csv`
  - artifact_mtime=`summary.md @ 2026-03-25 15:17:51 local; fd_vs_analytic.csv @ 2026-03-25 15:17:51 local; sample_summary.csv @ 2026-03-25 15:17:51 local; odo_component_breakdown.csv @ 2026-03-25 15:17:51 local`
  - result_freshness_check=`artifacts regenerated by current executable run; mtimes consistent across summary/csv outputs`
- metrics:
  - worst_odo_sample=`fix_mounting_and_odo_lever_truth @ 528867.106488: rel_fro=8.123593e-08, max_abs=5.272630e-07, worst_state=mount_yaw`
  - worst_nhc_sample=`fix_mounting_and_odo_lever_truth @ 528867.106488: rel_fro=1.279450e-05, max_abs=1.687604e-04, worst_state=mount_yaw`
  - odo_key_column_fd_match:
    `|ΔH_bgz|<=1.168572e-09`, `|ΔH_sgz|<=1.754965e-09`, `|ΔH_att_z|<=3.207355e-08`, `|ΔH_lever_y|<=2.393759e-10`, `|ΔH_mount_pitch|<=1.292014e-08`, `|ΔH_mount_yaw|<=5.272630e-07`
  - stressed_case_component_scale:
    early/transition samples `H_bgz≈-1.002932~-1.006055`, `|H_sgz|<=0.005209`, `|H_att_total_z|<=0.015718`, `|H_att_lever_z|<=7.734071e-07`;
    late sample `H_bgz≈-1.05413`, `H_sgz≈0.205`, `H_att_total_z≈-0.064~-0.067`
- observability_notes:
  - state_block_mapping:
    本轮直接核对的是 `att(6-8)`、`bg(12-14)`、`sg(15-17)`、`mounting(22-24)`、`lever_odo(25-27)` 这些 `ODO/NHC` 相关列；其中 `ODO bg_z(14)` 在 stressed cases 上稳定维持 `~ -1.0` 量级，而 `sg_z(17)` 与 `att_z(8)` 远小一个到三个数量级。
  - schedule_effect:
    被审计状态来自 `EXP-20260325-data2-bgz-route-decomposition-r1` 的 staged 输出，因此仍对应 `phase1` 禁用 `ODO/NHC`、`phase2/phase3` 开启 strict weak gate、GNSS 周期性 outage 且冻结 `lever_gnss(28-30)` 的主线时序。
  - odo_jacobian_interpretation:
    `ComputeOdoModel()` 的 analytic Jacobian 与当前误差注入约定下的 FD 在 sampled cases 上几乎完全一致；`H_bgz` 不是解析式偶然算偏，而是真实由 nonzero `lever_odo` 打开的高增益列。
  - component_breakdown:
    对 stressed cases，`H_att_lever_z` 极小（`1e-7~1e-5`），`H_att_total_z` 也明显小于 `H_bgz`；因此当前 `ODO -> bg_z` 链主要由 direct `H_bgz` 主导，而不是经 `att_z`/lever-coupled attitude 项间接放大。
- decision:
  - `EXP-20260325-data2-bgz-jacobian-fd-audit-r1` 已排除“`ComputeOdoModel()` 的 `H_bgz/H_sgz/H_att_z` 解析 Jacobian 抄错/符号错/尺度错”作为当前主因。
  - 当前更强解释是：nonzero `lever_odo` 打开的 `H_bgz` 本身就很强，且在 staged 主线下缺少正确的可观性门控，或 residual 被错误地竞争到 `bg_z` 而不是外参状态。
  - `NHC` 作为对照也基本与 FD 一致；其少量 `mount_yaw` 数值差分误差量级仍远小于 route-decomposition 中 observed failure scale，不足以解释主崩坏。
- next_step:
  - 优先执行 `EXP-20260325-data2-bgz-state-competition-r1`，在 `mounting/lever` freeze-open 对照下检查 residual 最终流向 `bg_z`、`mounting_yaw` 还是 `lever_odo_y`。
  - 随后执行 `EXP-20260325-data2-nhc-admission-alignment-r1`，确认 `v_b` vs `v_wheel_b/v_v` gate 不一致是否只是 secondary issue。

### session_id: 20260325-1549-bgz-state-competition-r1

- objective:
  - 执行 `EXP-20260325-data2-bgz-state-competition-r1`，在保持 combined-truth `mounting + lever_odo` 几何不变的前提下，只切 `mounting/lever` 的 freeze/open，验证 residual 是否会从 `bg_z` 显著转移到外参状态。
- scope:
  - 新增 `scripts/analysis/run_data2_bgz_competition_probe.py`；生成四组 case config；运行 fresh solver；汇总 `phase2/phase3 RMSE3D`、`first_divergence_start_t`、`odo/nhc acceptance`、critical-window `bg_z/mounting_yaw/lever_odo_y` 峰值变化与 phase2 末净漂移；回填 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_bgz_competition_probe.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-bgz-state-competition-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_bgz_state_competition_r1_20260325/`
  - shared_controls=`strict weak gate`, `phase2/phase3 freeze GNSS lever`, `fusion.init.odo_scale=1.0`, `fusion.fej.enable=false`
  - geometry_policy=`all four cases keep combined-truth mounting base + ODO lever init; open means estimable after phase1, not reset to zero`
  - cases=`mounting_fixed+lever_fixed`, `mounting_open+lever_fixed`, `mounting_fixed+lever_open`, `mounting_open+lever_open`
  - critical_window=`528276.000000~528336.000000`
  - dataset_time_window=`528076.000000~530488.900000`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content docs\odo-nhc-bgz-chain-bug-plan.md`
  - `Get-Content scripts\analysis\run_data2_staged_truth_ablation_probe.py`
  - `Get-Content scripts\analysis\run_data2_bgz_route_decomposition.py`
  - `rg -n "odo_lever_arm|lever_arm0|imu_mounting_angle|disable_odo_lever_arm|disable_mounting" src include -g "*.cpp" -g "*.h"`
  - `python -m py_compile scripts\analysis\run_data2_bgz_competition_probe.py`
  - `python scripts\analysis\run_data2_bgz_competition_probe.py`
  - `Import-Csv output\data2_bgz_state_competition_r1_20260325\case_metrics.csv`
  - `Get-Content output\data2_bgz_state_competition_r1_20260325\state_competition_summary.md`
- artifacts:
  - summary=`output/data2_bgz_state_competition_r1_20260325/state_competition_summary.md`
  - case_metrics=`output/data2_bgz_state_competition_r1_20260325/case_metrics.csv`
  - manifest=`output/data2_bgz_state_competition_r1_20260325/manifest.json`
  - case_configs=`output/data2_bgz_state_competition_r1_20260325/artifacts/cases/*/config_*.yaml`
  - artifact_mtime=`state_competition_summary.md @ 2026-03-25 15:48:14 local; case_metrics.csv @ 2026-03-25 15:48:14 local; manifest.json @ 2026-03-25 15:48:14 local`
  - config_hash_or_mtime=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local; eskf_fusion.exe @ 2026-03-25 14:26:31 local; derived case configs regenerated in current run`
  - result_freshness_check=`all four case configs and summary/csv artifacts regenerated by current solver run; mtimes aligned at 15:48 local`
- metrics:
  - fixed_fixed=`phase2_rmse_3d=0.109494 m`, `phase3_rmse_3d=96.781242 m`, `first_divergence_start_t=528315.655012`, `odo_accept_ratio=0.946696`, `nhc_accept_ratio=0.694537`
  - mounting_open_lever_fixed=`phase3_rmse_3d=97.006751 m`, `first_divergence_start_t=528315.655012`, `critical_mounting_peak_abs_change=0.000065 deg`
  - mounting_fixed_lever_open=`phase3_rmse_3d=87.329409 m`, `first_divergence_start_t=528315.660012`, `phase2_end_net_odo_lever_y_delta=0.279504 m`, `nhc_accept_ratio=0.831223`
  - mounting_open_lever_open=`phase3_rmse_3d=87.252899 m`, `first_divergence_start_t=528315.660012`, `phase2_end_net_odo_lever_y_delta=0.279515 m`, `odo_accept_ratio=0.948766`
  - bgz_competition_core=`critical_window_peak_abs_bg_z_change≈1016.811~1017.188 deg/h across all four cases`; lever-open cases `phase2_bg_z_peak` 甚至略高于 fixed-fixed (`2380.95` vs `2341.80 deg/h`)
- observability_notes:
  - state_block_mapping:
    本轮直接比较 `bg_z(14)`、`mounting(22-24)`、`lever_odo(25-27)` 三块的竞争关系；`mounting_open` 几乎不吸收残差，critical-window `mounting_yaw` 峰值变化仅 `0.000065 deg`，而 `lever_open` 到 phase2 末可吸收 `~0.2795 m` 的 `lever_odo_y` 漂移。
  - schedule_effect:
    仍沿用 staged 主线：phase1 禁用 `ODO/NHC` 并冻结 `odo_scale/mounting/lever_odo`；phase2/phase3 开启 strict weak gate、保留 `lever_gnss(28-30)` 冻结、GNSS 周期性 outage。
  - behavior_by_case:
    `mounting_open+lever_fixed` 基本中性甚至略差；`mounting_fixed+lever_open` 与 `mounting_open+lever_open` 只带来 phase3 末端的有限缓解，未改变 `528315.655~528315.660 s` 的 earliest divergence，也未显著削弱 early `bg_z` 失稳。
  - interpretation:
    外参 state competition 不是 primary root cause；`lever_odo` 打开的强 `ODO H_bgz` 仍会在外参开放时优先把 residual 写向 `bg_z`，lever state 只是在后段形成一个次级吸收池。
- decision:
  - `EXP-20260325-data2-bgz-state-competition-r1` 已基本排除“外参被冻结后 residual 无处可去，所以才被迫灌进 `bg_z`”作为 primary 解释。
  - `mounting` 通道几乎不起作用；`lever_odo` 通道即使打开，也只带来有限晚期缓解，不能阻止 early `bg_z` route。
  - 当前后续优先级应从“继续做 state competition 变体”切换为“实现并验证 non-debug `ODO H_bgz` observability gate / mitigation”，`NHC admission alignment` 保留为次级核对。
- next_step:
  - 先实现一个 non-debug `ODO bg_z` observability gate / route mitigation 候选，并进入 `EXP-20260325-data2-bgz-mitigation-validation-r1`。
  - 再执行 `EXP-20260325-data2-nhc-admission-alignment-r1`，量化 `v_b` vs `v_wheel_b/v_v` admission mismatch 是否仍有可见影响。

### session_id: 20260325-1709-bgz-mitigation-validation-r1-r3

- objective:
  - 执行 `EXP-20260325-data2-bgz-mitigation-validation-r1/r2/r3`，验证 non-debug `bg_z` gate 候选能否把 staged current mainline 从 `INS/GNSS/ODO/NHC` 拉回接近 `INS/GNSS`，并区分 global gate 与 ODO-only gate 的差别。
- scope:
  - 新增 `scripts/analysis/run_data2_bgz_mitigation_validation.py`；先跑 `r1` 的 global gate / gate+forget，对照 `INS/GNSS` 与 current mainline；随后在 C++ 中把 `bg_z` gate 拆成 sensor-specific `ODO/NHC` 开关，再跑 `r2` 的 ODO-only gate 与 `r3` 的 tighter yaw-only ODO gate；回填 `walkthrough.md`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/run_data2_bgz_mitigation_validation.py`
  - `walkthrough.md`
- configs:
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - code_change=`bgz_gate_apply_to_odo`, `bgz_gate_apply_to_nhc` sensor-specific switches added`
  - exp_r1=`EXP-20260325-data2-bgz-mitigation-validation-r1`, output=`output/data2_bgz_mitigation_validation_r1_20260325/`, case_set=`r1`
  - exp_r2=`EXP-20260325-data2-bgz-mitigation-validation-r2`, output=`output/data2_bgz_mitigation_validation_r2_20260325/`, case_set=`odo_only_r2`
  - exp_r3=`EXP-20260325-data2-bgz-mitigation-validation-r3`, output=`output/data2_bgz_mitigation_validation_r3_20260325/`, case_set=`odo_only_tight_r3`
  - shared_controls=`strict weak gate`, `phase2/phase3 freeze GNSS lever`, `fusion.init.odo_scale=1.0`, `fusion.fej.enable=false`
  - current_mainline_case=`control_strict_gate_freeze_gnss_lever` semantics under staged truth builder
  - ins_gnss_reference=`ODO/NHC disabled for all phases + freeze odo_scale/mounting/lever_odo`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content docs\odo-nhc-bgz-chain-bug-plan.md`
  - `rg -n "enable_bgz_observability_gate|bgz_gate" include\app\fusion.h src\app\config.cpp src\app\pipeline_fusion.cpp`
  - `python -m py_compile scripts\analysis\run_data2_bgz_mitigation_validation.py`
  - `python scripts\analysis\run_data2_bgz_mitigation_validation.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_bgz_mitigation_validation.py --case-set odo_only_r2 --exp-id EXP-20260325-data2-bgz-mitigation-validation-r2 --output-dir output\data2_bgz_mitigation_validation_r2_20260325`
  - `python scripts\analysis\run_data2_bgz_mitigation_validation.py --case-set odo_only_tight_r3 --exp-id EXP-20260325-data2-bgz-mitigation-validation-r3 --output-dir output\data2_bgz_mitigation_validation_r3_20260325`
  - `Import-Csv output\data2_bgz_mitigation_validation_r1_20260325\case_metrics.csv`
  - `Import-Csv output\data2_bgz_mitigation_validation_r2_20260325\case_metrics.csv`
  - `Import-Csv output\data2_bgz_mitigation_validation_r3_20260325\case_metrics.csv`
  - `Get-Content output\data2_bgz_mitigation_validation_r1_20260325\summary.md`
  - `Get-Content output\data2_bgz_mitigation_validation_r2_20260325\summary.md`
  - `Get-Content output\data2_bgz_mitigation_validation_r3_20260325\summary.md`
- artifacts:
  - r1_summary=`output/data2_bgz_mitigation_validation_r1_20260325/summary.md`
  - r1_case_metrics=`output/data2_bgz_mitigation_validation_r1_20260325/case_metrics.csv`
  - r1_manifest=`output/data2_bgz_mitigation_validation_r1_20260325/manifest.json`
  - r2_summary=`output/data2_bgz_mitigation_validation_r2_20260325/summary.md`
  - r2_case_metrics=`output/data2_bgz_mitigation_validation_r2_20260325/case_metrics.csv`
  - r2_manifest=`output/data2_bgz_mitigation_validation_r2_20260325/manifest.json`
  - r3_summary=`output/data2_bgz_mitigation_validation_r3_20260325/summary.md`
  - r3_case_metrics=`output/data2_bgz_mitigation_validation_r3_20260325/case_metrics.csv`
  - r3_manifest=`output/data2_bgz_mitigation_validation_r3_20260325/manifest.json`
  - artifact_mtime=`r1 summary/csv/manifest @ 2026-03-25 16:31:10 local; r2 summary/csv/manifest @ 2026-03-25 16:55:05 local; r3 summary/csv/manifest @ 2026-03-25 17:07:44 local`
  - config_hash_or_mtime=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local; eskf_fusion.exe @ 2026-03-25 16:36:56 local`
  - result_freshness_check=`all r1/r2/r3 artifacts regenerated in current session; solver rebuilt before r2/r3 after sensor-specific gate code change`
- metrics:
  - ins_gnss_reference=`phase2_rmse_3d=0.083253 m`, `phase3_rmse_3d=7.835156 m`, `phase3 bg_z_abs_max=2.147807 deg/h`, `phase3 yaw_abs_max=1.062930 deg`
  - current_mainline=`phase2_rmse_3d=0.102727 m`, `phase3_rmse_3d=23.838294 m`, `phase3 bg_z_abs_max=153.518974 deg/h`, `phase3 yaw_abs_max=29.954979 deg`, `first_divergence_start_t=528867.106000`
  - r1_global_gate=`phase3_rmse_3d=23.906627 m`, `phase3 bg_z_abs_max=150.989673 deg/h`, `phase3 yaw_abs_max=30.630798 deg`, `delta_vs_current=-0.068333 m`
  - r1_gate_forgetting=`phase3_rmse_3d=24.323763 m`, `phase3 bg_z_abs_max=565.042778 deg/h`, `phase3 yaw_abs_max=62.119012 deg`, `delta_vs_current=-0.485469 m`
  - r2_odo_only_gate=`phase3_rmse_3d=23.704855 m`, `delta_vs_current=+0.133439 m`, `phase3 yaw_abs_max=29.346651 deg`, `phase3 bg_z_abs_max=159.863803 deg/h`
  - r3_odo_only_yaw12=`phase3_rmse_3d=23.713238 m`, `delta_vs_current=+0.125056 m`, `phase3 yaw_abs_max=29.642954 deg`, `phase3 bg_z_abs_max=160.604629 deg/h`
- observability_notes:
  - state_block_mapping:
    本轮 mitigation 直接作用于 `bg_z(14)` 的 update gain 路由；r2/r3 进一步把 gate 限制为 `ODO` 生效、`NHC` 不受影响，从而测试 `NHC` 是否补偿通路。
  - schedule_effect:
    三轮验证都保持 staged 主线同一时序；`ins_gnss_only` 仅额外关闭 `ODO/NHC` 并冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - sensor_specific_findings:
    global gate 会同时压 `ODO` 与 `NHC` 的 `bg_z` 更新，结果 current mainline 变差；改成 ODO-only 后，current mainline 有微弱正收益（`~0.13 m`），支持 “NHC bg_z 更像补偿而不是 primary poison source”。
  - limitation:
    即使改成 ODO-only 且加严到 yaw-only `12 deg/s`，`phase3 RMSE3D` 仍与 current mainline 同级，远高于 `INS/GNSS` 的 `7.835156 m`，说明单一 `bg_z gate` 不是 current mainline gap 的充分修复。
- decision:
  - `EXP-20260325-data2-bgz-mitigation-validation-r1` 已否定“直接开启 global `bg_z` gate 就能修好 current mainline”。
  - sensor-specific `ODO-only bg_z gate` 比 global gate 合理，但收益过小，不能作为当前正式修复方案单独交付。
  - 当前最应该优先推进的是 `EXP-20260325-data2-nhc-admission-alignment-r1` 与 `ODO residual / v_vehicle` semantics 审计，而不是继续只调 `bg_z gate` 门限。
- next_step:
  - 立即执行 `EXP-20260325-data2-nhc-admission-alignment-r1`，把 `current v_b gate`、`v_wheel_b gate`、`v_v gate` 做成可量化对照。
  - 若 admission alignment 仍解释不了 current mainline gap，再回到 `ODO residual / wheel-speed semantics` 做更直接的参考点语义审计。

### session_id: 20260325-1748-admission-alignment-and-odo-semantics-audit

- objective:
  - 执行 `EXP-20260325-data2-nhc-admission-alignment-r1`，正式验证 `RunNhcUpdate()` 的 `v_b / v_wheel_b / v_v` admission 口径是否会改变 current mainline；若结论为否，再立刻补做 `EXP-20260325-data2-odo-reference-semantics-audit-r1`，判断 `ODO residual / wheel-speed semantics` 是否是当前主 gap 的解释。
- scope:
  - 在 C++ 中新增正式 `NHC admission velocity source` 配置与 per-attempt admission CSV 日志；新增 `scripts/analysis/analyze_bgz_admission_alignment.py` 运行三组 fresh case；随后新增 `scripts/analysis/analyze_odo_reference_semantics.py`，复用 fresh current-mainline 输出做离线 `ODO` 语义审计；最后回填 `walkthrough.md`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/analyze_bgz_admission_alignment.py`
  - `scripts/analysis/analyze_odo_reference_semantics.py`
  - `walkthrough.md`
- configs:
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - solver_mtime=`2026-03-25 17:29:27 local`
  - exp_admission=`EXP-20260325-data2-nhc-admission-alignment-r1`
  - admission_output=`output/data2_nhc_admission_alignment_r1_20260325/`
  - admission_variants=`baseline_current_v_b_gate`, `nhc_gate_use_v_wheel_b`, `nhc_gate_use_v_v`
  - new_runtime_fields=`fusion.constraints.nhc_admission_velocity_source`, `fusion.constraints.enable_nhc_admission_log`
  - exp_odo_semantics=`EXP-20260325-data2-odo-reference-semantics-audit-r1`
  - odo_semantics_output=`output/data2_odo_reference_semantics_audit_r1_20260325/`
  - source_case_for_odo_semantics=`EXP-20260325-data2-nhc-admission-alignment-r1::baseline_current_v_b_gate`
  - dataset_time_window=`528076.009368~530488.900000`
- commands:
  - `Get-Content C:\Users\不存在的骑士\.codex\skills\using-superpowers\SKILL.md`
  - `Get-Content walkthrough.md`
  - `Get-Content docs\odo-nhc-bgz-chain-bug-plan.md`
  - `Get-Content src\app\pipeline_fusion.cpp`
  - `Get-Content src\core\measurement_models_uwb.cpp`
  - `python -m py_compile scripts\analysis\analyze_bgz_admission_alignment.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\analyze_bgz_admission_alignment.py`
  - `python -m py_compile scripts\analysis\analyze_odo_reference_semantics.py`
  - `python scripts\analysis\analyze_odo_reference_semantics.py`
  - `Get-Content output\data2_nhc_admission_alignment_r1_20260325\summary.md`
  - `Get-Content output\data2_odo_reference_semantics_audit_r1_20260325\summary.md`
- artifacts:
  - admission_summary=`output/data2_nhc_admission_alignment_r1_20260325/summary.md`
  - admission_case_metrics=`output/data2_nhc_admission_alignment_r1_20260325/case_metrics.csv`
  - admission_mismatch_summary=`output/data2_nhc_admission_alignment_r1_20260325/admission_mismatch_summary.csv`
  - admission_log_baseline=`output/data2_nhc_admission_alignment_r1_20260325/artifacts/cases/baseline_current_v_b_gate/SOL_baseline_current_v_b_gate_nhc_admission.csv`
  - odo_semantics_summary=`output/data2_odo_reference_semantics_audit_r1_20260325/summary.md`
  - odo_semantics_metrics=`output/data2_odo_reference_semantics_audit_r1_20260325/semantics_metrics.csv`
  - odo_semantics_contribution=`output/data2_odo_reference_semantics_audit_r1_20260325/contribution_summary.csv`
  - artifact_mtime=`admission summary/csv @ 2026-03-25 17:41:30~17:41:31 local; odo semantics summary/csv @ 2026-03-25 17:47:08~17:47:09 local`
  - config_hash_or_mtime=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local; current case config regenerated in current run`
  - result_freshness_check=`admission experiment three cases fresh rerun under rebuilt solver; ODO semantics audit reuses the same fresh baseline_current_v_b_gate outputs with aligned mtimes`
- metrics:
  - admission_r1_all_variants_identical:
    `phase2_rmse_3d=0.102727 m`, `phase3_rmse_3d=23.838294 m`, `first_divergence_start_t=528867.106000`, `phase2_bg_z_abs_max=110.504914 deg/h`, `phase3_bg_z_abs_max=153.518974 deg/h`, `odo_accept_ratio=0.998610`, `nhc_accept_ratio=0.998891`
  - admission_r1_mismatch_core:
    all three variants `selected_precheck_accept_ratio=1.0`, `current_vs_selected_mismatch_ratio=0.0`, `current_accept_selected_reject_ratio=0.0`, `current_reject_selected_accept_ratio=0.0`
  - odo_semantics_best_candidate:
    `odo_model_est = odo_scale*v_v.x` is best in every window; e.g. `phase2 mean|res|=0.026251 m/s, rmse=0.040528 m/s`, `phase3 mean|res|=0.031709 m/s, rmse=0.047036 m/s`, `divergence_plus60s mean|res|=0.030701 m/s, rmse=0.040408 m/s`
  - odo_semantics_compared_candidates:
    `phase3 v_b.x mean|res|=0.109409 m/s, rmse=0.126220 m/s`; `phase3 truth_v_v.x mean|res|=0.107336 m/s, rmse=0.121291 m/s`; `phase3 odo_scale*v_v_trans.x mean|res|=0.050716 m/s, rmse=0.078798 m/s`
  - odo_forward_contribution_scale:
    `phase3 mean|v_v_rot_x|=0.028090 m/s`, `mean|v_v-v_b|_x=0.028451 m/s`, `mean odo_scale=0.991441`, `max |odo_scale-1|=0.009403`
- observability_notes:
  - state_block_mapping:
    本轮直接触及 `bg_z(14)` 的候选解释链路，但 admission 对齐实验显示在 current mainline 上，`NHC` 是否进入更新并不受 `v_b / v_wheel_b / v_v` 口径影响；`mounting(22-24)` 与 `lever_odo(25-27)` 则在 ODO 语义审计里只体现为 `v_v` 与 `v_b` 的小幅前向差异（phase3 均值约 `0.028 m/s`），不足以解释 `23.838294 m` 的主 gap。
  - schedule_effect:
    两轮审计都保持 staged 主线同一时序：phase1 纯 INS/GNSS，phase2 joint，phase3 周期性 GNSS outage 且冻结 `lever_gnss(28-30)`；行为结果对三种 `NHC` admission source 完全中性。
  - sensor_behavior:
    `NHC` admission 在当前配置下实质上是 inactive discriminator：`nhc_disable_below_forward_speed=0.0`、`nhc_max_abs_v=5.0` 使所有尝试在三种速度口径下都被放行；真正的 `NHC` reject 仍来自后续 NIS / 数值层。
  - odo_semantics_interpretation:
    `ODO` 当前 measurement model `odo_scale*v_v.x` 与 wheel-speed 的一致性明显优于 `v_b.x`、`v_v_trans.x` 和 `truth_v_v.x`，说明“wheel-speed 参考点定义错了”不是 current mainline 主因；量测本身更像是正确的，问题在于同一套量测 Jacobian/协方差在 phase3 被如何写入状态。
- decision:
  - `EXP-20260325-data2-nhc-admission-alignment-r1` 已直接否定 `HYP-51`：当前主线上不存在可见的 `NHC admission mismatch`。
  - `EXP-20260325-data2-odo-reference-semantics-audit-r1` 已否定“当前 `ODO residual / v_vehicle` 语义不对”作为 mainline 主 gap 的解释；恰恰是当前 `odo_scale*v_v.x` 残差定义最贴近 wheel-speed。
  - 当前剩余最高优先级应转为 current mainline `phase3` first-divergence 周围的 `ODO/NHC` 成功更新机理审计，重点看 `P(bg_z,*)`、measurement numerator 与 `dx_bg_z` 何时从“合理小修正”切换到“错误主导”。
- next_step:
  - 运行一个 current-mainline `enable_mechanism_log` 定窗实验，窗口先锁定 `528840~528930 s`，直接抓取 first-divergence 前后 `ODO/NHC` 成功更新的 `H_bg_z`、`num_bg_z_*`、`P(bg_z,att/mount/bg)`、`dx_bg_z`。
  - 若 mechanism log 指向 `ODO` 首个坏写，再对照 `ODO-only bg_z gate` case 做同窗比较，确认 current mainline 剩余 gap 是否来自 gate 之外的 covariance carry-over / state competition。

### session_id: 20260325-2014-data2-raw-odo-truth-speed-audit

- objective:
  - 不依赖当前程序的 ODO 读取/残差定义，直接基于原始 `data2` ODO 文件与真值速度对账，回答“`IMULog201812290233_2_ODO.txt` 给出的到底是原始脉冲计数，还是已换算速度”。
- scope:
  - 检查仓内 `README.md` 与历史 README 是否包含轮径/分辨率说明；编写一个独立 Python 脚本，把原始 ODO 倒数第二列与 `LC_SM_TXT.nav` 的真值速度模长做时间对齐和统计比较；产出正式 `summary.md/csv` 并回填 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/analyze_data2_raw_odo_truth_speed.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-raw-odo-truth-speed-audit-r1`
  - raw_odo_path=`dataset/data2/IMULog201812290233_2_ODO.txt`
  - truth_path=`dataset/data2/LC_SM_TXT.nav`
  - output_dir=`output/data2_raw_odo_truth_speed_audit_r1_20260325/`
  - dataset_time_window=`528076.009368~530488.900000`
- commands:
  - `Get-Content walkthrough.md -Head 40`
  - `rg -n "diameter|直径|分辨率|odo|wheel|轮" README.md . -g 'README.md' -g '!build'`
  - `Get-Content README.md`
  - `rg -n "分辨率|轮径|直径|encoder|pulse|脉冲|resolution|diameter|wheel" _upload_combined_INS README.md docs .research -g '*.md' -g '!build'`
  - `python -m py_compile scripts\analysis\analyze_data2_raw_odo_truth_speed.py`
  - `python scripts\analysis\analyze_data2_raw_odo_truth_speed.py`
  - `Get-Content output\data2_raw_odo_truth_speed_audit_r1_20260325\summary.md`
- artifacts:
  - summary=`output/data2_raw_odo_truth_speed_audit_r1_20260325/summary.md`
  - metrics=`output/data2_raw_odo_truth_speed_audit_r1_20260325/metrics.csv`
  - samples=`output/data2_raw_odo_truth_speed_audit_r1_20260325/samples.csv`
  - artifact_mtime=`summary/csv @ 2026-03-25 20:14:36 local`
  - config_hash_or_mtime=`raw ODO txt @ 2025-05-08 14:18:35 local; analysis script regenerated in current session`
  - result_freshness_check=`all artifacts generated fresh in current session from raw ODO + truth files`
- metrics:
  - raw_odo_vs_truth_speed_3d=`MAE=0.116560 m/s`, `RMSE=0.136404 m/s`, `corr=0.999832`, `odo/ref scale=0.991519`, `bias=-0.096710 m/s`
  - raw_odo_vs_truth_speed_h=`MAE=0.115543 m/s`, `RMSE=0.135514 m/s`, `corr=0.999832`, `odo/ref scale=0.991611`, `bias=-0.095554 m/s`
  - raw_odo_sampling=`odo_dt_median=0.005000010 s`, `truth_dt_median=0.005000000 s`
  - raw_odo_column_stats=`min=-0.053812 m/s`, `median=13.064418 m/s`, `max=19.148725 m/s`, `reserved_last_col_unique_count=1`, `near_integer_ratio_eps1e-3=0.064298`
- observability_notes:
  - state_block_mapping:
    本轮不触及滤波状态块估计，也不涉及 `21-30` 状态块或 `bg_z(14)` 更新链；它只回答原始 ODO 数据物理语义。
  - schedule_effect:
    本轮未运行融合器，不存在 GNSS/ODO/NHC 调度窗口差异。
  - sensor_semantics_finding:
    原始 `IMULog201812290233_2_ODO.txt` 倒数第二列与真值速度模长高度一致，且最后一列恒为零，不支持“原始脉冲计数未换算”的解释。
- decision:
  - `EXP-20260325-data2-raw-odo-truth-speed-audit-r1` 已支持一个更基础的结论：仓内 `data2` 原始 ODO 文件倒数第二列本身就应视为已换算前向速度，单位近似 `m/s`。
  - 因此，当前 `INS/GNSS/ODO/NHC` 相对 `INS/GNSS` 的负作用，不能再归因于“把原始 pulse count 当真实速度直接使用”。
- next_step:
  - 若后续还要追 `README` 中轮径/分辨率与仓内数据的关系，需要拿到那份外部 README 的精确路径或内容，进一步对账其 implied conversion factor。
  - 当前主研究链不变，继续执行 current-mainline `phase3` mechanism log，定位 `ODO/NHC -> bg_z` 的 first bad write。

### session_id: 20260325-2042-data2-fullwindow-attitude-bias-coupling

- objective:
  - 在 full-window 口径下把问题收敛到 `INS/GNSS/ODO/NHC`，直接比较姿态误差与零偏异常是否随外参固定/释放和 `ODO/NHC` 量测权重同步变化，并输出 31 维状态全程曲线。
- scope:
  - 新增一条专用实验脚本，派生四组 fresh 配置并运行求解器；生成 `SOL/state_series/all_states`、全状态带真值叠加图、关键耦合状态图、case metrics、manifest 与 summary；同时补一条最小 `pytest` 契约测试并回填 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py`
  - `tests/test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/`
  - case_g1=`INS/GNSS`, `lever_gnss(28-30)` truth fixed, `enable_odo=false`, `enable_nhc=false`
  - case_g2=`INS/GNSS/ODO/NHC`, `mounting(22-24)+lever_odo(25-27)+lever_gnss(28-30)` truth fixed, `odo_scale(21)=1.0` fixed
  - case_g3=`INS/GNSS/ODO/NHC`, truth-init 外参释放，`sigma_mounting* / sigma_lever_arm* / sigma_gnss_lever_arm* = 0.01x`
  - case_g4=`G3 + sigma_odo/sigma_nhc_y/sigma_nhc_z = 5x`
  - schedule_mode=`full_window`, `gnss_schedule.enabled=false`, `runtime_phases=none`, `post_gnss_ablation=none`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\executing-plans\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `git branch --show-current`
  - `git status --short`
  - `pytest --version`
  - `Get-Content scripts\analysis\run_data2_staged_truth_ablation_probe.py`
  - `Get-Content scripts\analysis\run_nhc_state_convergence_research.py`
  - `Get-Content scripts\analysis\run_data2_ins_gnss_odo_nhc_pva_anchor_compare.py`
  - `Get-Content scripts\analysis\run_data2_state_sanity_matrix.py`
  - `Get-Content dataset\data2\README.md`
  - `pytest tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python -m py_compile scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py`
  - `Get-Content output\data2_fullwindow_attitude_bias_coupling_r1_20260325\summary.md`
  - `Import-Csv output\data2_fullwindow_attitude_bias_coupling_r1_20260325\case_metrics.csv | ConvertTo-Json -Depth 3`
- artifacts:
  - summary=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/summary.md`
  - case_metrics=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/case_metrics.csv`
  - manifest=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/manifest.json`
  - truth_reference=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/truth_reference.json`
  - all_states_plot=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/all_states_overview.png`
  - key_coupling_plot=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/key_coupling_states.png`
  - per_group_plots=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/{position,velocity,attitude,ba,bg,sg,sa,odo_scale,mounting,odo_lever,gnss_lever}.png`
  - g1_all_states=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group1_ins_gnss_truth_gnss_lever_fixed/all_states_group1_ins_gnss_truth_gnss_lever_fixed.csv`
  - g2_all_states=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group2_ins_gnss_odo_nhc_truth_extrinsics_fixed/all_states_group2_ins_gnss_odo_nhc_truth_extrinsics_fixed.csv`
  - g3_all_states=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group3_ins_gnss_odo_nhc_truth_extrinsics_soft/all_states_group3_ins_gnss_odo_nhc_truth_extrinsics_soft.csv`
  - g4_all_states=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group4_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise/all_states_group4_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise.csv`
  - artifact_mtime=`g1 config/sol/state/all_states @ 2026-03-25 20:37:04 / 20:37:28 / 20:37:33 / 20:38:04 local; g2 @ 20:38:04 / 20:38:41 / 20:38:46 / 20:39:11; g3 @ 20:39:11 / 20:39:47 / 20:39:53 / 20:40:20; g4 @ 20:40:20 / 20:40:59 / 20:41:05 / 20:41:31; summary/plots @ 20:41:33~20:41:36`
  - config_hash_or_mtime=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local; four derived configs regenerated fresh in current session`
  - result_freshness_check=`all four cases rerun fresh in current session; summary/case_metrics/manifest/plots generated after solver outputs and mtimes are monotonic`
- metrics:
  - g1_ins_gnss_control=`RMSE3D=0.043118 m`, `P95_3D=0.080710 m`, `final_3D=0.005644 m`, `yaw_err_max_abs=1.043771 deg`, `bg_z_err_max_abs=109.689361 deg/h`
  - g2_truth_extrinsics_fixed=`RMSE3D=0.545297 m`, `P95_3D=1.039979 m`, `final_3D=0.645871 m`, `yaw_err_max_abs=179.999974 deg`, `bg_z_err_max_abs=16917.596262 deg/h`, `odo_accept_ratio=0.842856`, `nhc_accept_ratio=0.939826`
  - g3_truth_extrinsics_soft=`RMSE3D=7.616816 m`, `P95_3D=9.567460 m`, `final_3D=9.580329 m`, `yaw_err_max_abs=179.991929 deg`, `bg_z_err_max_abs=24901.080574 deg/h`, `odo_accept_ratio=0.867470`, `nhc_accept_ratio=0.863382`
  - g4_soft_plus_high_meas_noise=`RMSE3D=0.338954 m`, `P95_3D=0.472171 m`, `final_3D=0.605251 m`, `yaw_err_max_abs=111.942713 deg`, `bg_z_err_max_abs=16389.812875 deg/h`, `odo_accept_ratio=1.000000`, `nhc_accept_ratio=1.000000`
  - relative_effects=`G2/G3` 都显著差于 `G1`; `G4` 相比 `G3` 的 `RMSE3D` 改善 `7.277862 m`，说明仅增大 `ODO/NHC` 量测噪声就能明显缓和全局导航发散
- observability_notes:
  - state_block_mapping:
    `G1` 固定 `lever_gnss(28-30)` 且关闭 `ODO/NHC`；`G2` 固定 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`, `lever_gnss(28-30)`；`G3` 释放 `mounting + lever_odo + lever_gnss`，仅保留 `odo_scale(21)` 固定，并把外参过程噪声降到 `0.01x`；`G4` 在 `G3` 基础上只增大 `sigma_odo/sigma_nhc_y/sigma_nhc_z`。
  - schedule_effect:
    四组均为全时段传感器配置，不再分 phase；`GNSS` 全时段开启，`gnss_schedule`、`runtime_phases`、`post_gnss_ablation` 均关闭，因此异常差异主要来自 `ODO/NHC` 量测参与方式与外参状态处理，而不是调度切换。
  - behavior_by_group:
    `G2` 与 `G3` 相比 `G1` 都出现姿态与 `bg_z` 同时失稳，支持“姿态误差与零偏异常耦合”判断；`G4` 在保持同一 truth-init 外参与 `ODO/NHC` 全时段参与的前提下，仅通过增大 `ODO/NHC` 量测噪声就把 `RMSE3D` 明显拉回，说明坏链对量测权重高度敏感。
  - residual_implication:
    `G4` 虽然位置指标显著恢复，但 `yaw/bg_z` 仍很大，说明 noise 放大只是在削弱坏链的显性导航后果，并未从根上移除 `ODO/NHC -> 姿态/bg_z` 的耦合通路。
- decision:
  - full-window 结果不支持“只要把安装角和杆臂带成真值并固定，姿态/零偏问题就会自然消失”；`G2` 已直接否定这一点。
  - 也不支持“外参给小过程噪声释放会比固定真值更稳”；`G3` 反而最差。
  - 当前最强的新证据是：把 `ODO/NHC` 量测噪声增大到 `5x` 后，`G4` 的导航误差显著恢复，说明当前问题更像 `ODO/NHC` 权重过强触发的姿态-零偏耦合，而不是单纯外参未收敛。
- next_step:
  - 以 `G2` 与 `G4` 为主，定位 full-window 中第一个 `yaw/bg_z` 明显离开 `G1` 轨迹的时间窗，并补做 mechanism / `NIS` / accept-ratio 联审，确认是 `ODO`、`NHC` 还是二者顺序共同触发。
  - 对 `G2/G3/G4` 的 `all_states` 曲线计算 first-divergence timestamp，量化 `yaw`, `bg_z`, `ba_x`, `mounting_yaw`, `odo_lever_y` 的先后顺序，避免只看终值。

### session_id: 20260325-2101-data2-fullwindow-attitude-bias-coupling-g5

- objective:
  - 在现有 full-window `G4` 基础上新增 `G5`：保持 truth-init 外参与 `5x ODO/NHC` 量测噪声不变，但不再估计 `sg/sa(15-20)`，验证“去掉 6 个 IMU 比例因子状态”是否还能进一步缓和姿态-零偏耦合。
- scope:
  - 复用已进入 RED 的 `pytest` 契约测试；修改 `scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py` 以支持 `disable_gyro_scale/disable_accel_scale` 与 `G5` case；重跑 full-window 脚本到原输出目录，直接覆盖旧 4 组图、summary、manifest 与 metrics；最后回填 fresh 结论到 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-fullwindow-attitude-bias-coupling-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/`
  - case_g5=`G4 + fusion.ablation.disable_gyro_scale=true + fusion.ablation.disable_accel_scale=true`
  - retained_cases=`G1~G4` regenerated fresh in same session and output directory
  - overwrite_policy=`reuse output/data2_fullwindow_attitude_bias_coupling_r1_20260325 and overwrite prior 4-group plots/artifacts`
- commands:
  - `Get-Content walkthrough.md -TotalCount 260`
  - `rg -n "CaseSpec|CASE_SPECS|disable_gyro_scale|disable_accel_scale|measurement_noise_scale|group5|group4" scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `pytest tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py`
  - `Get-Content output\data2_fullwindow_attitude_bias_coupling_r1_20260325\summary.md -TotalCount 220`
  - `Import-Csv output\data2_fullwindow_attitude_bias_coupling_r1_20260325\case_metrics.csv`
  - `Get-ChildItem output\data2_fullwindow_attitude_bias_coupling_r1_20260325\plots`
- artifacts:
  - summary=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/summary.md`
  - case_metrics=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/case_metrics.csv`
  - manifest=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/manifest.json`
  - all_states_plot=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/all_states_overview.png`
  - key_coupling_plot=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/plots/key_coupling_states.png`
  - g5_config=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale/config_group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale.yaml`
  - g5_sol=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale/SOL_group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale.txt`
  - g5_state_series=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale/state_series_group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale.csv`
  - g5_all_states=`output/data2_fullwindow_attitude_bias_coupling_r1_20260325/artifacts/cases/group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale/all_states_group5_ins_gnss_odo_nhc_truth_extrinsics_soft_high_meas_noise_no_imu_scale.csv`
- metrics:
  - test_status=`pytest 2 passed`
  - g5_metrics=`RMSE3D=0.301991 m`, `P95_3D=0.414670 m`, `final_3D=0.433576 m`, `yaw_err_max_abs=85.119234 deg`, `bg_z_err_max_abs=15068.234661 deg/h`, `odo_accept_ratio=1.000000`, `nhc_accept_ratio=1.000000`
  - g5_vs_g4=`ΔRMSE3D=-0.036963 m`, `ΔP95_3D=-0.057501 m`, `Δfinal_3D=-0.171675 m`, `Δyaw_err_max_abs=-26.823479 deg`, `Δbg_z_err_max_abs=-1321.578214 deg/h`
- artifact_mtime:
  - g5_case=`config @ 2026-03-25 21:00:08 local; sol @ 21:00:46; state_series @ 21:00:52; all_states @ 21:01:18`
  - overwritten_outputs=`case_metrics @ 2026-03-25 21:01:18 local; summary/manifest @ 21:01:23; plots @ 21:01:20~21:01:23`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - derived_configs=`all G1~G5 configs regenerated fresh in current session`
- dataset_time_window:
  - g5_all_states=`528076.014367~530488.895829`
- result_freshness_check:
  - `pytest` passed after adding `G5`
  - `python scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py` regenerated `G1~G5` in timestamp order and overwrote the prior 4-group `summary/case_metrics/manifest/plots`
- observability_notes:
  - state_block_mapping:
    `G5` 延续 `G4` 的 `odo_scale(21)` 固定、`mounting(22-24)+lever_odo(25-27)+lever_gnss(28-30)` truth-init 且小过程噪声释放、`sigma_odo/sigma_nhc_y/sigma_nhc_z = 5x`，并额外冻结 `sg(15-17)` 与 `sa(18-20)`。
  - schedule_effect:
    五组均为 full-window；`gnss_schedule.enabled=false`、`runtime_phases=none`、`post_gnss_ablation=none`，因此差异来自 `ODO/NHC` 权重和状态块参与方式，而不是调度切换。
  - behavior_change:
    相比 `G4`，`G5` 在 `ODO/NHC accept_ratio` 仍为 `1.0/1.0` 的前提下继续改善位置与姿态峰值，但 `yaw/bg_z` 仍远大于 `G1`，说明 `sg/sa` 只是耦合链的次级竞争项，不是 primary trigger。
- decision:
  - “不加加计和陀螺比例因子”这组实验成立且已并入正式 full-window 套件；新 5 组图像已覆盖旧 4 组版本。
  - `G5` 支持一个更具体的判断：去掉 `sg/sa(15-20)` 会进一步减轻姿态-零偏耦合，但不足以消除异常，因此主问题仍应继续锁定在 `ODO/NHC` 更新链的权重/协方差/触发时序，而不是把 `sg/sa` 当成根因。
- next_step:
  - 以 `G2`、`G4`、`G5` 为主，定位 full-window 中第一个 `yaw/bg_z` 明显离开 `G1` 的时间窗，并补做 mechanism / `NIS` / accept-ratio 联审。
  - 对 `G4` 与 `G5` 的同窗结果重点比较 `P(bg_z,sg,sa,att)`、`dx_bg_z` 与 `dx_mounting_yaw`，判断去掉 `sg/sa` 改变的是坏写触发时刻，还是只改变坏写幅值/协方差条件。

### session_id: 20260325-2124-data2-staged-g5-no-imu-scale

- objective:
  - 以 full-window `G5` 为基线，把实验改造成三段式 `INS/GNSS -> INS/GNSS/ODO/NHC -> phase3 周期断星`，全程放弃 `sg/sa(15-20)` 在线估计，并比较 `ODO/NHC` 量测噪声 `1x/2x` 对 phase2 与 phase3 的影响。
- scope:
  - 新增一条专用 staged 脚本和对应最小 `pytest` 契约测试；复用 full-window `G5` 的 truth-init 外参与 31 维状态叠图能力，接入 staged 的 `runtime_phases + gnss_schedule`；fresh 运行两组 `1x/2x` 案例并回填 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_no_imu_scale.py`
  - `tests/test_run_data2_staged_g5_no_imu_scale.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-no-imu-scale-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_no_imu_scale_r1_20260325/`
  - phase_windows=`phase1=[528076.0,528276.0]`, `phase2=[528276.0,528776.0]`, `phase3=[528776.0,530488.9]`
  - phase3_periodic_gnss=`on=60 s`, `off=60 s`
  - shared_g5_controls=`truth-init mounting/lever_odo/lever_gnss`, `odo_scale=1.0`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `extrinsics_q_scale=0.01`
  - case_1x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 5`
  - case_2x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 10`
- commands:
  - `Get-Content walkthrough.md -TotalCount 220`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\executing-plans\SKILL.md -TotalCount 220`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md -TotalCount 220`
  - `rg -n "phase_absolute_times|build_runtime_phases|build_periodic_gnss_windows|GROUP_SPECS|ALL_STATE_SPECS|boundary_lines|CASE_SPECS|disable_gyro_scale|disable_accel_scale" scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py scripts\analysis\run_data2_staged_truth_ablation_probe.py scripts\analysis\run_data2_staged_estimation.py tests -g "*.py"`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `python scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r1_20260325\case_metrics.csv`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r1_20260325\phase_metrics.csv`
- artifacts:
  - summary=`output/data2_staged_g5_no_imu_scale_r1_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/phase_metrics.csv`
  - manifest=`output/data2_staged_g5_no_imu_scale_r1_20260325/manifest.json`
  - truth_reference=`output/data2_staged_g5_no_imu_scale_r1_20260325/truth_reference.json`
  - all_states_plot=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/all_states_overview.png`
  - key_coupling_plot=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/key_coupling_states.png`
  - per_group_plots=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/{position,velocity,attitude,ba,bg,sg,sa,odo_scale,mounting,odo_lever,gnss_lever}.png`
  - case_1x_all_states=`output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_1x/all_states_staged_g5_odo_nhc_noise_1x.csv`
  - case_2x_all_states=`output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_2x/all_states_staged_g5_odo_nhc_noise_2x.csv`
- metrics:
  - test_status=`pytest 4 passed (new staged + existing full-window tests)`
  - noise_1x=`overall RMSE3D=22.185549 m`, `phase2 RMSE3D=0.069783 m`, `phase3 RMSE3D=26.331289 m`, `final_3D=0.100634 m`, `yaw_err_max_abs=21.542040 deg`, `bg_z_err_max_abs=1383.261426 deg/h`, `odo/nhc_accept_ratio=1.0/1.0`
  - noise_2x=`overall RMSE3D=7.443702 m`, `phase2 RMSE3D=0.069365 m`, `phase3 RMSE3D=8.834596 m`, `final_3D=0.084317 m`, `yaw_err_max_abs=8.017227 deg`, `bg_z_err_max_abs=565.541974 deg/h`, `odo/nhc_accept_ratio=1.0/1.0`
  - relative_effects=`Δoverall RMSE3D=-14.741847 m`, `Δphase2=-0.000418 m`, `Δphase3=-17.496694 m`, `Δfinal_3D=-0.016317 m`, `Δyaw_err_max_abs=-13.524813 deg`, `Δbg_z_err_max_abs=-817.719452 deg/h`
- artifact_mtime:
  - case_1x=`config @ 2026-03-25 21:21:12 local; sol @ 21:21:49; state_series @ 21:21:54; all_states @ 21:22:22`
  - case_2x=`config @ 2026-03-25 21:22:22 local; sol @ 21:23:09; state_series @ 21:23:15; all_states @ 21:23:47`
  - summary_outputs=`case_metrics/phase_metrics @ 2026-03-25 21:23:47 local; summary/manifest @ 21:23:54; plots @ 21:23:50~21:23:54`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - derived_configs=`1x/2x staged G5 configs regenerated fresh in current session`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 新增脚本先经 RED->GREEN 契约测试验证，再 fresh 运行两组 solver case
  - `summary/case_metrics/phase_metrics/manifest/plots` 的 mtime 晚于两组 `SOL/state_series/all_states`
  - 回归 `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py` 全绿
- observability_notes:
  - state_block_mapping:
    两组 case 全程固定 `sg(15-17)` 与 `sa(18-20)`；`phase1` 冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`；`phase2` 固定 `lever_gnss(28-30)`；`phase3` 继续固定 `lever_gnss(28-30)`, `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - schedule_effect:
    `phase1` 仅 `INS/GNSS`；`phase2` 打开 `ODO/NHC`；`phase3` 采用 `60 s on / 60 s off` 的周期断星。两组差异主要出现在 `gnss_off` 窗，而非 `phase2` 联合段。
  - behavior_change:
    `1x -> 2x` 时 `phase2 RMSE3D` 近乎不变，但 `phase3` 和各个 `gnss_off_*` 窗的 `RMSE3D / yaw / bg_z` 明显下降，说明在 staged `G5` 下坏链更像是 outage 漂移积累，而不是 phase2 当下就把状态拟合坏。
- decision:
  - 新的三段式 `G5` 套件已经建立并可复现实验；它把“量测权重敏感”进一步定位到了 `phase3` 周期断星阶段。
  - 当前最值得优先追的不是继续扫更多噪声倍率，而是对 `1x/2x` 在首个 `gnss_off` 窗的 mechanism / covariance 分化做定窗审计。
- next_step:
  - 以 `staged_g5_odo_nhc_noise_1x/2x` 为主，抓 `phase2_end=528776 s` 后首个 `gnss_off` 窗 (`528836~528896 s`) 的 mechanism / `NIS` / accept-ratio / covariance 日志。
  - 对比同窗内 `dx_bg_z`、`dx_mounting_yaw`、`dx_odo_scale`、`P(bg_z,att/mount/odo_scale)` 和 `ODO/NHC` 分别贡献，确认 `2x` 改变的是坏写触发时刻还是累积增益。

### session_id: 20260325-2136-data2-staged-g5-noise-3x-pva-error

- objective:
  - 在三段式 `G5` 套件上继续收敛分析口径：把 dedicated `PVA` 图改为误差图，并在 `ODO/NHC` 高量测噪声对照中新增 `3x` case，检查 phase2/phase3 结论是否保持并强化。
- scope:
  - 修改 `run_data2_staged_g5_no_imu_scale.py` 的 case 集与绘图逻辑；补齐对应 `pytest` 契约；fresh 重跑三组 `1x/2x/3x` staged `G5` 实验并做与 full-window 测试的联合回归；更新 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_no_imu_scale.py`
  - `tests/test_run_data2_staged_g5_no_imu_scale.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-no-imu-scale-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_no_imu_scale_r1_20260325/`
  - phase_windows=`phase1=[528076.0,528276.0]`, `phase2=[528276.0,528776.0]`, `phase3=[528776.0,530488.9]`
  - phase3_periodic_gnss=`on=60 s`, `off=60 s`
  - shared_g5_controls=`truth-init mounting/lever_odo/lever_gnss`, `odo_scale=1.0`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `extrinsics_q_scale=0.01`
  - case_1x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 5`
  - case_2x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 10`
  - case_3x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 15`
  - pva_plot_mode=`position/velocity/attitude -> error plot`; `all_states_overview/key_coupling_states -> state view retained`
- commands:
  - `rg -n "CASE_SPECS|PVA_ERROR_GROUP_SPECS|plot_mode|state_error_values|staged_g5_odo_nhc_noise_3x" scripts\analysis\run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `python scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md`
  - `Get-Item output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md, output\data2_staged_g5_no_imu_scale_r1_20260325\manifest.json, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\position.png, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\velocity.png, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\attitude.png`
  - `Get-Item config_data2_baseline_eskf.yaml`
- artifacts:
  - summary=`output/data2_staged_g5_no_imu_scale_r1_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/phase_metrics.csv`
  - manifest=`output/data2_staged_g5_no_imu_scale_r1_20260325/manifest.json`
  - pva_error_plots=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/{position,velocity,attitude}.png`
  - overview_plots=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/all_states_overview.png`, `output/data2_staged_g5_no_imu_scale_r1_20260325/plots/key_coupling_states.png`
  - case_3x_all_states=`output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_3x/all_states_staged_g5_odo_nhc_noise_3x.csv`
- metrics:
  - test_status=`pytest 5 passed (staged_g5 + fullwindow_coupling)`
  - noise_1x=`overall RMSE3D=22.185549 m`, `phase2 RMSE3D=0.069783 m`, `phase3 RMSE3D=26.331289 m`, `final_3D=0.100634 m`, `yaw_err_max_abs=21.542040 deg`, `bg_z_err_max_abs=1383.261426 deg/h`
  - noise_2x=`overall RMSE3D=7.443702 m`, `phase2 RMSE3D=0.069365 m`, `phase3 RMSE3D=8.834596 m`, `final_3D=0.084317 m`, `yaw_err_max_abs=8.017227 deg`, `bg_z_err_max_abs=565.541974 deg/h`
  - noise_3x=`overall RMSE3D=3.375818 m`, `phase2 RMSE3D=0.069825 m`, `phase3 RMSE3D=4.006432 m`, `final_3D=0.078017 m`, `yaw_err_max_abs=3.788099 deg`, `bg_z_err_max_abs=302.861866 deg/h`
  - relative_effects=`phase2 spread=0.000460 m`, `Δoverall RMSE3D(1x->3x)=-18.809731 m`, `Δphase3(1x->3x)=-22.324857 m`, `Δyaw_err_max_abs(1x->3x)=-17.753942 deg`, `Δbg_z_err_max_abs(1x->3x)=-1080.399560 deg/h`
- artifact_mtime:
  - summary_manifest=`2026-03-25 21:34:47 local`
  - pva_error_plots=`position/velocity @ 2026-03-25 21:34:45 local; attitude @ 21:34:46 local`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - derived_configs=`1x/2x/3x staged G5 configs regenerated fresh in current session`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 先通过 `pytest` 固化 `3x case` 与 `PVA_ERROR_GROUP_SPECS` 契约，再 fresh 重跑三组 solver case
  - `summary/manifest` 与 `position/velocity/attitude` 图的 mtime 晚于旧版 `21:23~21:24` 输出，确认原 2 组图像已被覆盖为 3 组新版本
  - 联合回归 `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py` 全绿
- observability_notes:
  - state_block_mapping:
    三组 case 全程固定 `sg(15-17)` 与 `sa(18-20)`；`phase1` 冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`；`phase2` 固定 `lever_gnss(28-30)`；`phase3` 继续固定 `lever_gnss(28-30)`, `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - schedule_effect:
    `phase1` 仅 `INS/GNSS`；`phase2` 打开 `ODO/NHC`；`phase3` 采用 `60 s on / 60 s off` 的周期断星。三组差异仍主要出现在 `gnss_off_*` 窗，而非 `phase2` 联合段。
  - behavior_change:
    `1x -> 2x -> 3x` 时 `phase2 RMSE3D` 基本保持不变，但 `phase3 RMSE3D / yaw / bg_z` 单调改善，说明减弱 `ODO/NHC` 权重主要是在 outage 漂移积累阶段削弱坏链。
  - plotting_interpretability:
    dedicated `PVA` 图改为误差图后，`phase2` 平稳与 `phase3` 分化可以直接从误差时间序列读取，不再需要从 estimate-vs-truth 双线图二次心算。
- decision:
  - `3x` 对照组强化了 staged `G5` 的核心结论：坏链对 `ODO/NHC` 权重高度敏感，但敏感性主要体现在 `phase3` 周期断星段，而不是 `phase2` 联合段。
  - `position/velocity/attitude` 三张图已切换为误差图并覆盖原有 2 组版本；当前无需再回到比例因子在线估计分支。
- next_step:
  - 优先对 `staged_g5_odo_nhc_noise_1x/3x` 做首个 `gnss_off` 窗 (`528836~528896 s`) 的 mechanism / `NIS` / covariance 审计，并用 `2x` 检查单调性是否贯穿 update 级别。
  - 在同窗内对比 `dx_bg_z`、`dx_mounting_yaw`、`dx_odo_scale`、`P(bg_z,att/mount/odo_scale)` 与 `ODO/NHC` 分别贡献，确认 `3x` 改变的是坏写触发时刻还是累积增益。

### session_id: 20260325-2147-data2-staged-g5-noise-4x-longer-phase3

- objective:
  - 舍弃 staged `G5` 旧版 `1x` 对照，新增 `4x` 高量测噪声组，并把 phase3 每个 `GNSS on/off` 窗拉长到旧版的 `1.5x`，检查“phase2 基本不变、phase3 单调改善”的判断在更长 outage 窗下是否仍成立。
- scope:
  - 按 TDD 先改 staged `G5` 契约测试，再修改实验脚本的 case 集与 phase3 周期默认值；fresh 重跑同一输出目录，让 `2x/3x/4x + 90/90` 结果覆盖旧 `1x/2x/3x + 60/60` 版本；更新 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_no_imu_scale.py`
  - `tests/test_run_data2_staged_g5_no_imu_scale.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-no-imu-scale-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_no_imu_scale_r1_20260325/`
  - phase_windows=`phase1=[528076.0,528276.0]`, `phase2=[528276.0,528776.0]`, `phase3=[528776.0,530488.9]`
  - phase3_periodic_gnss=`on=90 s`, `off=90 s`
  - shared_g5_controls=`truth-init mounting/lever_odo/lever_gnss`, `odo_scale=1.0`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `extrinsics_q_scale=0.01`
  - case_2x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 10`
  - case_3x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 15`
  - case_4x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 20`
  - pva_plot_mode=`position/velocity/attitude -> error plot retained`
- commands:
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md -TotalCount 220`
  - `Get-Content walkthrough.md -TotalCount 140`
  - `Get-Content scripts\analysis\run_data2_staged_g5_no_imu_scale.py -TotalCount 260`
  - `Get-Content tests\test_run_data2_staged_g5_no_imu_scale.py -TotalCount 240`
  - `rg -n "1x|2x|3x|PHASE3_GNSS_ON_DEFAULT|PHASE3_GNSS_OFF_DEFAULT|gnss_off_" scripts\analysis\run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py` (`RED`, expected fail on `PHASE3_GNSS_ON_DEFAULT == 90.0`)
  - `rg -n "def build_periodic_gnss_windows|phase3_on_duration|phase3_off_duration" scripts\analysis\run_data2_staged_estimation.py`
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md`
  - `Get-Item output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md, output\data2_staged_g5_no_imu_scale_r1_20260325\manifest.json, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\position.png, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\velocity.png, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\attitude.png`
- artifacts:
  - summary=`output/data2_staged_g5_no_imu_scale_r1_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/phase_metrics.csv`
  - manifest=`output/data2_staged_g5_no_imu_scale_r1_20260325/manifest.json`
  - pva_error_plots=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/{position,velocity,attitude}.png`
  - case_4x_all_states=`output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_4x/all_states_staged_g5_odo_nhc_noise_4x.csv`
- metrics:
  - test_status=`pytest 5 passed (staged_g5 + fullwindow_coupling)`
  - noise_2x=`overall RMSE3D=15.268904 m`, `phase2 RMSE3D=0.069365 m`, `phase3 RMSE3D=18.122129 m`, `final_3D=0.083978 m`, `yaw_err_max_abs=10.619116 deg`, `bg_z_err_max_abs=545.301613 deg/h`
  - noise_3x=`overall RMSE3D=6.635262 m`, `phase2 RMSE3D=0.069825 m`, `phase3 RMSE3D=7.875070 m`, `final_3D=0.072530 m`, `yaw_err_max_abs=5.244213 deg`, `bg_z_err_max_abs=291.912661 deg/h`
  - noise_4x=`overall RMSE3D=3.648409 m`, `phase2 RMSE3D=0.070390 m`, `phase3 RMSE3D=4.329976 m`, `final_3D=0.065548 m`, `yaw_err_max_abs=5.205972 deg`, `bg_z_err_max_abs=189.070546 deg/h`
  - relative_effects=`phase2 spread=0.001025 m`, `Δoverall RMSE3D(2x->4x)=-11.620495 m`, `Δphase3(2x->4x)=-13.792153 m`, `Δyaw_err_max_abs(2x->4x)=-5.413144 deg`, `Δbg_z_err_max_abs(2x->4x)=-356.231067 deg/h`, `phase3_off_window_count=10`
- artifact_mtime:
  - summary_manifest=`2026-03-25 21:47:34 local`
  - pva_error_plots=`2026-03-25 21:47:32 local`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - derived_configs=`2x/3x/4x staged G5 configs regenerated fresh in current session`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 先将 staged `G5` 契约测试改成 `2x/3x/4x + on/off=90s/90s`，并确认 RED，再改实现到 GREEN
  - `summary/manifest/plots` 的 mtime 晚于上一版 `21:34` 输出，确认旧 `1x/2x/3x + 60/60` 结果已被覆盖
  - 联合回归 `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py` 全绿
- observability_notes:
  - state_block_mapping:
    三组 case 全程固定 `sg(15-17)` 与 `sa(18-20)`；`phase1` 冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`；`phase2` 固定 `lever_gnss(28-30)`；`phase3` 继续固定 `lever_gnss(28-30)`, `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - schedule_effect:
    `phase1` 仅 `INS/GNSS`；`phase2` 打开 `ODO/NHC`；`phase3` 改成 `90 s on / 90 s off`，因此窗口数从旧版 `14` 减到 `10`，首个长窗 `gnss_off` 变为 `528866~528956 s`。
  - behavior_change:
    `2x -> 3x -> 4x` 时 `phase2 RMSE3D` 仍基本不动，但 `phase3 RMSE3D` 与 `bg_z` 峰值持续下降；`yaw` 在 `3x -> 4x` 的改善已明显趋缓，提示继续加大噪声可能进入收益递减区。
  - plotting_interpretability:
    dedicated `PVA` 误差图已随新 schedule 和新 case 集一起覆盖，便于直接读取长窗 outage 下的误差累积形态。
- decision:
  - 在更长的 `90/90` outage schedule 下，staged `G5` 的核心结论仍成立：决定性差异仍主要发生在 phase3，而不是 phase2。
  - `1x` 已正式从当前研究口径移除；后续优先围绕 `2x/4x` 做夹逼分析，`3x` 仅作为中间单调性检查点。
- next_step:
  - 优先对 `staged_g5_odo_nhc_noise_2x/4x` 做首个长窗 `gnss_off` (`528866~528956 s`) 的 mechanism / `NIS` / covariance 审计，并用 `3x` 验证 update 级别单调性。
  - 在同窗内对比 `dx_bg_z`、`dx_mounting_yaw`、`dx_odo_scale`、`P(bg_z,att/mount/odo_scale)` 与 `ODO/NHC` 分别贡献，确认 `4x` 改变的是坏写触发时刻还是累积增益。

### session_id: 20260325-2156-data2-staged-g5-noise-6x-sweep

- objective:
  - 在 staged `G5` 的 `90/90` 长窗 schedule 下继续向高量测噪声侧推进，去掉 `2x/3x`，改成 `4x/5x/6x`，检验 `4x` 之后是否仍有收益，以及收益是否开始饱和。
- scope:
  - 按 TDD 先修改 staged `G5` 契约测试为 `4x/5x/6x`，确认 RED；再修改实验脚本 case 集，保留 phase3 `90 s / 90 s` 和 PVA 误差图逻辑；fresh 重跑同一输出目录，覆盖上一版 `2x/3x/4x` 结果；更新 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_no_imu_scale.py`
  - `tests/test_run_data2_staged_g5_no_imu_scale.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-no-imu-scale-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_no_imu_scale_r1_20260325/`
  - phase_windows=`phase1=[528076.0,528276.0]`, `phase2=[528276.0,528776.0]`, `phase3=[528776.0,530488.9]`
  - phase3_periodic_gnss=`on=90 s`, `off=90 s`
  - shared_g5_controls=`truth-init mounting/lever_odo/lever_gnss`, `odo_scale=1.0`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `extrinsics_q_scale=0.01`
  - case_4x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 20`
  - case_5x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 25`
  - case_6x=`sigma_odo/sigma_nhc_y/sigma_nhc_z = baseline * 30`
  - pva_plot_mode=`position/velocity/attitude -> error plot retained`
- commands:
  - `Get-Content walkthrough.md -TotalCount 140`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md -TotalCount 220`
  - `Get-Content scripts\analysis\run_data2_staged_g5_no_imu_scale.py -TotalCount 220`
  - `Get-Content tests\test_run_data2_staged_g5_no_imu_scale.py -TotalCount 220`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py` (`RED`, expected fail on stale `2x/3x/4x` case list)
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md`
  - `Get-Item output\data2_staged_g5_no_imu_scale_r1_20260325\summary.md, output\data2_staged_g5_no_imu_scale_r1_20260325\manifest.json, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\position.png, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\velocity.png, output\data2_staged_g5_no_imu_scale_r1_20260325\plots\attitude.png`
- artifacts:
  - summary=`output/data2_staged_g5_no_imu_scale_r1_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_no_imu_scale_r1_20260325/phase_metrics.csv`
  - manifest=`output/data2_staged_g5_no_imu_scale_r1_20260325/manifest.json`
  - pva_error_plots=`output/data2_staged_g5_no_imu_scale_r1_20260325/plots/{position,velocity,attitude}.png`
  - case_6x_all_states=`output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv`
- metrics:
  - test_status=`pytest 5 passed (staged_g5 + fullwindow_coupling)`
  - noise_4x=`overall RMSE3D=3.648409 m`, `phase2 RMSE3D=0.070390 m`, `phase3 RMSE3D=4.329976 m`, `final_3D=0.065548 m`, `yaw_err_max_abs=5.205972 deg`, `bg_z_err_max_abs=189.070546 deg/h`
  - noise_5x=`overall RMSE3D=2.763782 m`, `phase2 RMSE3D=0.070832 m`, `phase3 RMSE3D=3.279972 m`, `final_3D=0.060014 m`, `yaw_err_max_abs=4.754404 deg`, `bg_z_err_max_abs=138.454914 deg/h`
  - noise_6x=`overall RMSE3D=2.678202 m`, `phase2 RMSE3D=0.071153 m`, `phase3 RMSE3D=3.178389 m`, `final_3D=0.055405 m`, `yaw_err_max_abs=4.240251 deg`, `bg_z_err_max_abs=110.196945 deg/h`
  - relative_effects=`phase2 spread=0.000763 m`, `Δoverall RMSE3D(4x->6x)=-0.970207 m`, `Δphase3(4x->6x)=-1.151587 m`, `Δyaw_err_max_abs(4x->6x)=-0.965721 deg`, `Δbg_z_err_max_abs(4x->6x)=-78.873601 deg/h`, `phase3_off_window_count=10`
- artifact_mtime:
  - summary_manifest=`2026-03-25 21:56:00 local`
  - pva_error_plots=`2026-03-25 21:55:58 local`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - derived_configs=`4x/5x/6x staged G5 configs regenerated fresh in current session`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 先将 staged `G5` 契约测试改成 `4x/5x/6x`，并确认 RED，再改实现到 GREEN
  - `summary/manifest/plots` 的 mtime 晚于上一版 `21:47` 输出，确认旧 `2x/3x/4x + 90/90` 结果已被覆盖
  - 联合回归 `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py` 全绿
- observability_notes:
  - state_block_mapping:
    三组 case 全程固定 `sg(15-17)` 与 `sa(18-20)`；`phase1` 冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`；`phase2` 固定 `lever_gnss(28-30)`；`phase3` 继续固定 `lever_gnss(28-30)`, `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - schedule_effect:
    `phase1` 仅 `INS/GNSS`；`phase2` 打开 `ODO/NHC`；`phase3` 继续使用 `90 s on / 90 s off`，窗口数保持 `10`，首个长窗 `gnss_off` 仍为 `528866~528956 s`。
  - behavior_change:
    `4x -> 5x -> 6x` 时 `phase2 RMSE3D` 仍几乎不动，`phase3 RMSE3D / yaw / bg_z` 继续单调改善；但收益相比 `2x -> 4x` 已明显缩小，说明高噪声侧开始进入边际递减区。
  - plotting_interpretability:
    dedicated `PVA` 误差图已随新 case 集一起覆盖，便于直接读取 `4x/5x/6x` 在长窗 outage 下的误差累积差异。
- decision:
  - `4x` 还没有到极限，继续增大 `ODO/NHC` 量测噪声到 `5x/6x` 仍能改善 phase3 漂移。
  - 但从 `4x -> 6x` 的收益已经明显变小，因此后续应从“继续扫更大倍率”转向“解释为什么高噪声会抑制同一条坏链，以及其极限收益为什么变缓”。
- next_step:
  - 优先对 `staged_g5_odo_nhc_noise_4x/6x` 做首个长窗 `gnss_off` (`528866~528956 s`) 的 mechanism / `NIS` / covariance 审计，并用 `5x` 验证 update 级别单调性。
  - 在同窗内对比 `dx_bg_z`、`dx_mounting_yaw`、`dx_odo_scale`、`P(bg_z,att/mount/odo_scale)` 与 `ODO/NHC` 分别贡献，确认 `6x` 改变的是坏写触发时刻还是累积增益。

### session_id: 20260325-2217-data2-staged-g5-mounting-zero-init-q-sweep

- objective:
  - 以当前 staged `G5 6x` 为基础配置，新增一组独立实验：安装角从 `0 deg` 初值开始估计、给 `3 deg std` 的大初始协方差，并把当前 staged-6x 的 mounting 过程噪声按 `0.5x/1x/2x` 扫频，判断 mounting 初始化与 mounting `Q` 是否仍是当前一阶敏感杠杆。
- scope:
  - 按 TDD 先新增契约测试，再新建独立实验脚本 `run_data2_staged_g5_mounting_zero_init_q_sweep.py`；不覆盖现有 `EXP-20260325-data2-staged-g5-no-imu-scale-r1` 输出，而是生成到新的 output 目录；fresh 重跑并更新 `walkthrough.md`。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `tests/test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r1`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/`
  - phase_windows=`phase1=[528076.0,528276.0]`, `phase2=[528276.0,528776.0]`, `phase3=[528776.0,530488.9]`
  - phase3_periodic_gnss=`on=90 s`, `off=90 s`
  - fixed_controls=`odo_nhc_r=6x vs G5`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `lever_odo/lever_gnss truth init`, `odo_scale=1.0`
  - mounting_init=`mounting_roll0=mounting_pitch0=mounting_yaw0=0.0 deg`, `std_mounting_roll/pitch/yaw=3.0 deg`, `P0_diag[22:24]=0.00274155677808 rad^2`
  - cases=`staged_g5_mounting_zero_init_q_0p5x`, `staged_g5_mounting_zero_init_q_1x`, `staged_g5_mounting_zero_init_q_2x`
  - mounting_q_effective=`base_q * 0.01 * {0.5,1.0,2.0}` for `sigma_mounting`, `sigma_mounting_roll`, `sigma_mounting_pitch`, `sigma_mounting_yaw`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\brainstorming\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `rg -n "P0_diag|mounting_roll0|mounting_pitch0|mounting_yaw0|STD_MOUNTING_DEG|base_p0_diag_from_config" config_data2_baseline_eskf.yaml scripts\analysis src\app include\app tests`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py` (`RED`, expected fail on missing new script)
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\summary.md`
  - `Get-Item output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\summary.md, output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\manifest.json, output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\plots\position.png, output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\plots\velocity.png, output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\plots\attitude.png`
  - `Import-Csv output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\case_metrics.csv | Format-Table case_id,mounting_q_scale_rel_to_staged_6x,overall_rmse_3d_m_aux,phase2_rmse_3d_m,phase3_rmse_3d_m,overall_final_err_3d_m_aux,yaw_err_max_abs_deg,bg_z_degh_err_max_abs,odo_accept_ratio,nhc_accept_ratio`
  - `Import-Csv output\data2_staged_g5_no_imu_scale_r1_20260325\case_metrics.csv | Where-Object {$_.case_id -eq 'staged_g5_odo_nhc_noise_6x'} | Format-Table case_id,overall_rmse_3d_m_aux,phase2_rmse_3d_m,phase3_rmse_3d_m,overall_final_err_3d_m_aux,yaw_err_max_abs_deg,bg_z_degh_err_max_abs,odo_accept_ratio,nhc_accept_ratio`
  - `python -c "import csv; old='output/data2_staged_g5_no_imu_scale_r1_20260325/case_metrics.csv'; new='output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/case_metrics.csv'; old_rows={r['case_id']:r for r in csv.DictReader(open(old, encoding='utf-8-sig'))}; new_rows={r['case_id']:r for r in csv.DictReader(open(new, encoding='utf-8-sig'))}; old=old_rows['staged_g5_odo_nhc_noise_6x']; new=new_rows['staged_g5_mounting_zero_init_q_1x']; keys=['overall_rmse_3d_m_aux','phase2_rmse_3d_m','phase3_rmse_3d_m','overall_final_err_3d_m_aux','yaw_err_max_abs_deg','bg_z_degh_err_max_abs']; print({k: float(new[k])-float(old[k]) for k in keys})"`
  - `python -c "import csv; rows={r['case_id']:r for r in csv.DictReader(open('output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/case_metrics.csv', encoding='utf-8-sig'))}; a=rows['staged_g5_mounting_zero_init_q_0p5x']; b=rows['staged_g5_mounting_zero_init_q_2x']; keys=['overall_rmse_3d_m_aux','phase2_rmse_3d_m','phase3_rmse_3d_m','overall_final_err_3d_m_aux','yaw_err_max_abs_deg','bg_z_degh_err_max_abs']; print({k: float(b[k])-float(a[k]) for k in keys})"`
- artifacts:
  - summary=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/phase_metrics.csv`
  - manifest=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/manifest.json`
  - pva_error_plots=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/plots/{position,velocity,attitude}.png`
  - overview_plots=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/plots/all_states_overview.png`, `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/plots/key_coupling_states.png`
  - case_1x_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
- metrics:
  - test_status=`pytest 8 passed (new mounting-zero-init sweep + staged_g5 + fullwindow_coupling)`
  - q_0p5x=`overall RMSE3D=2.683596 m`, `phase2 RMSE3D=0.071729 m`, `phase3 RMSE3D=3.184788 m`, `final_3D=0.059930 m`, `yaw_err_max_abs=4.277668 deg`, `bg_z_err_max_abs=110.041076 deg/h`
  - q_1x=`overall RMSE3D=2.683586 m`, `phase2 RMSE3D=0.071729 m`, `phase3 RMSE3D=3.184775 m`, `final_3D=0.059929 m`, `yaw_err_max_abs=4.277664 deg`, `bg_z_err_max_abs=110.041076 deg/h`
  - q_2x=`overall RMSE3D=2.683542 m`, `phase2 RMSE3D=0.071729 m`, `phase3 RMSE3D=3.184724 m`, `final_3D=0.059928 m`, `yaw_err_max_abs=4.277648 deg`, `bg_z_err_max_abs=110.041076 deg/h`
  - relative_effects=`Δoverall(0.5x->2x)=-0.000054 m`, `Δphase2(0.5x->2x)=+0.000000047 m`, `Δphase3(0.5x->2x)=-0.000064 m`, `Δyaw_max(0.5x->2x)=-0.000019 deg`, `Δbg_z_err_max(0.5x->2x)=0 deg/h`
  - vs_old_staged_6x_truth_init=`Δoverall(1x-new minus 6x-old)=+0.005384 m`, `Δphase2=+0.000577 m`, `Δphase3=+0.006386 m`, `Δfinal_3D=+0.004525 m`, `Δyaw_max=+0.037413 deg`, `Δbg_z_err_max=-0.155869 deg/h`
- artifact_mtime:
  - summary_manifest=`2026-03-25 22:17:19 local`
  - pva_error_plots=`2026-03-25 22:17:19 local`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - derived_configs=`0.5x/1x/2x mounting-Q sweep configs regenerated fresh in current session`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 先新增 `mounting zero-init q sweep` 契约测试并确认 RED，再实现新脚本到 GREEN
  - 联合回归 `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py` 全绿
  - 新输出位于独立目录 `output/data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325/`，`summary/manifest/plots` 的 mtime 为 `22:17:19 local`，晚于脚本实现与测试时间，确认是 fresh 结果而非旧目录复用
- observability_notes:
  - state_block_mapping:
    三组 case 全程固定 `sg(15-17)` 与 `sa(18-20)`；`phase1` 冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`；`phase2` 释放 `mounting(22-24)` 但固定 `lever_gnss(28-30)`；`phase3` 再次固定 `lever_gnss(28-30)`, `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - schedule_effect:
    `phase1` 仅 `INS/GNSS`；`phase2` 打开 `ODO/NHC` 并成为 mounting 唯一可估计阶段；`phase3` 继续使用 `90 s on / 90 s off` 的周期断星，因此本轮 mounting `Q` sweep 只会通过 phase2 学到的 mounting 状态间接影响 phase3，而不是在 phase3 持续在线作用。
  - behavior_change:
    在 `0.5x/1x/2x` 下，`phase2/phase3/yaw/bg_z` 都几乎完全重合，说明当前 `6x` 量测噪声口径下，安装角零初值、大 `P0` 和 phase2 内部 mounting `Q` 细扫都不是一阶敏感杠杆。
  - comparison_to_old_6x:
    与旧 `staged_g5_odo_nhc_noise_6x` 相比，新 `1x` 仅表现出 `毫米/百分之一度` 量级变化；因此“把 mounting 从 truth-init 改成 zero-init + large P0”不会重写当前主结论。
- decision:
  - staged `G5 6x` 下，mounting 初值从 truth 改为 zero、并给大初始协方差，不会显著改变 phase2/phase3 行为。
  - 在当前三段式设计中继续细扫 mounting `Q` 没有足够信息增益；后续优先改扫其他参数，或直接转入首个长窗 `gnss_off` 的 mechanism / covariance onset 审计。
- next_step:
  - 先停止继续扩展 `mounting_q` 倍率；把 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r1::staged_g5_mounting_zero_init_q_1x` 作为新的 `6x + zero-init mounting` 对照基线。
  - 优先对 `EXP-20260325-data2-staged-g5-no-imu-scale-r1` 的 `4x/6x` 和新基线 `staged_g5_mounting_zero_init_q_1x` 做首个长窗 `gnss_off` (`528866~528956 s`) 机理对账，确认零初值 mounting 只影响 phase2 学习细节，而不改变 phase3 主坏链。

### session_id: 20260325-2238-data2-staged-g5-mounting-phase2-cov-seed-fix

- objective:
  - 复核用户指出的 zero-init mounting 脚本语义问题，确认安装角是否被错误钉在 `0`，并把“大初始噪声应在第二阶段开始时给”的要求实现到实际求解器执行路径。
- scope:
  - 先做 root-cause 排查，再按 TDD 修复；修改 runtime constraint override，使 runtime phase 能临时覆盖 covariance floor；更新 zero-init mounting sweep 脚本为 `r2`，在 phase2 开头插入 `0.02 s` mounting covariance seed 窗；fresh 重跑并更新 `walkthrough.md`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `tests/test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/`
  - phase_windows=`phase1=[528076.0,528276.0]`, `phase2=[528276.0,528776.0]`, `phase3=[528776.0,530488.9]`
  - phase2_seed=`phase2_mounting_cov_seed`, `duration=0.02 s`, `enable_covariance_floor=true`, `p_floor_mounting_deg=3.0`
  - phase3_periodic_gnss=`on=90 s`, `off=90 s`
  - fixed_controls=`odo_nhc_r=6x vs G5`, `disable_gyro_scale=true`, `disable_accel_scale=true`, `lever_odo/lever_gnss truth init`, `odo_scale=1.0`
  - cases=`staged_g5_mounting_zero_init_q_0p5x`, `staged_g5_mounting_zero_init_q_1x`, `staged_g5_mounting_zero_init_q_2x`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\systematic-debugging\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `Import-Csv output\data2_staged_g5_mounting_zero_init_q_sweep_r1_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\all_states_staged_g5_mounting_zero_init_q_1x.csv | Select-Object -First 5/Last 5 ...`
  - `rg -n "imu_mounting_angle|disable_mounting|SetStateMask|ApplyStateMaskToCov|freeze_extrinsics_when_weak_excitation|enable_covariance_floor|p_floor_mounting_deg" config_data2_baseline_eskf.yaml src\app include\app src\core`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py` (`RED`, expected fail on missing phase2 covariance seed contract)
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\summary.md`
  - `Import-Csv output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\all_states_staged_g5_mounting_zero_init_q_1x.csv | Where-Object {[double]$_.timestamp -ge 528276 -and [double]$_.timestamp -le 528776} | Measure-Object mounting_pitch_deg/mounting_yaw_deg -Minimum -Maximum`
- artifacts:
  - summary=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/phase_metrics.csv`
  - manifest=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/manifest.json`
  - pva_error_plots=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/plots/{position,velocity,attitude}.png`
  - case_1x_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
- metrics:
  - test_status=`pytest 8 passed (corrected mounting-zero-init sweep + staged_g5 + fullwindow_coupling)`
  - q_0p5x=`overall RMSE3D=3.061294 m`, `phase2 RMSE3D=0.070415 m`, `phase3 RMSE3D=3.633109 m`, `final_3D=0.055210 m`, `yaw_err_max_abs=3.946730 deg`, `bg_z_err_max_abs=155.279637 deg/h`
  - q_1x=`overall RMSE3D=3.061294 m`, `phase2 RMSE3D=0.070415 m`, `phase3 RMSE3D=3.633109 m`, `final_3D=0.055210 m`, `yaw_err_max_abs=3.946730 deg`, `bg_z_err_max_abs=155.279636 deg/h`
  - q_2x=`overall RMSE3D=3.061294 m`, `phase2 RMSE3D=0.070415 m`, `phase3 RMSE3D=3.633109 m`, `final_3D=0.055210 m`, `yaw_err_max_abs=3.946730 deg`, `bg_z_err_max_abs=155.279632 deg/h`
  - q_neutrality=`Δphase3(0.5x->2x)≈0 m`, `Δbg_z_err_max(0.5x->2x)≈0 deg/h`
  - q1_phase2_mounting_range=`pitch=[0.001923,0.363887] deg`, `yaw=[-0.596078,-0.043200] deg`
  - vs_truth_init_6x=`Δoverall=+0.383092 m`, `Δphase3=+0.454720 m`, `Δyaw_max=-0.293521 deg`, `Δbg_z_err_max=+45.082691 deg/h`
- artifact_mtime:
  - summary_manifest=`2026-03-25 22:37:47 local`
  - pva_error_plots=`2026-03-25 22:37:47 local`
- config_hash_or_mtime:
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
  - solver=`build/Release/eskf_fusion.exe rebuilt fresh in current session`
  - derived_configs=`corrected zero-init mounting q-sweep configs regenerated fresh in current session`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 通过 root-cause 排查确认 `ApplyStateMaskToCov()` 会在 phase1 冻结 mounting 时清零 `P(22:24,:)`，从而解释 `r1` 的语义错误
  - 先将新契约测试改成“必须存在 phase2 covariance seed 窗”，确认 RED，再实现到 GREEN
  - 重新编译 `eskf_fusion` 并 fresh 重跑 `r2`; `summary/manifest/plots` 的 mtime 为 `22:37:47 local`
- observability_notes:
  - state_block_mapping:
    `phase1` 仍冻结 `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`；`phase2_mounting_cov_seed + phase2_main` 打开 `mounting(22-24)` 并固定 `lever_gnss(28-30)`；`phase3` 继续固定 `lever_gnss(28-30)`, `odo_scale(21)`, `mounting(22-24)`, `lever_odo(25-27)`。
  - schedule_effect:
    修复后 mounting 不再在整个 phase2 贴着 `0`；但 phase3 仍是 mounting 冻结后的 outage 漂移测试，所以 mounting `Q` 倍率只通过 phase2 学习结果间接影响 phase3。
  - behavior_change:
    `r2` 证明用户指出的问题成立：旧 `r1` 并没有把“大 mounting 初始噪声”真正给到第二阶段。修复后 mounting 可动，但 `0.5x/1x/2x` 仍几乎重合，说明当前优先问题不是这组 `Q` 倍率。
- decision:
  - 用户对旧脚本的质疑成立：`r1` 语义确实写错了。
  - 修复后的 `r2` 应作为正式 zero-init mounting 结果；`r1` 降为 stale/conflict。
  - 当前后续不应继续纠缠 `mounting_q 0.5/1/2` 这组倍率，而应转向“phase2 mounting 学得是否足够好”与 phase3 主坏链机理。
- next_step:
  - 用 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2::staged_g5_mounting_zero_init_q_1x` 与 truth-init `staged_g5_odo_nhc_noise_6x` 做同窗机制对账，解释为什么 zero-init mounting 在 phase2 能追到接近 truth pitch，却 phase3 仍比 truth-init 更差。
  - 如需继续调参，优先考虑比 `mounting_q 0.5/1/2` 更可能是一阶敏感项的参数或约束，而不是再扩这条 sweep。

### session_id: 20260325-2244-data2-staged-g5-mounting-user-query-validation

- objective:
  - 回答用户对 zero-init mounting 脚本语义的质疑，复核“安装角是否被固定在 0”“是否只是 `Q` 太小”“大初始安装角噪声是否应在第二阶段开始给”这三点。
- scope:
  - 不改代码，只重新核对 `walkthrough`、求解器 masking 逻辑、`r2` 脚本相位配置与 fresh 输出状态序列，确保对用户的解释与当前证据一致。
- changed_files:
  - `walkthrough.md`
- configs:
  - reference_exp=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2`
  - reference_output=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/`
  - reference_script=`scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - reference_solver_code=`src/core/eskf_engine.cpp`, `src/app/pipeline_fusion.cpp`
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "phase2_mounting_cov_seed|p_floor_mounting_deg|mounting_roll0|mounting_pitch0|mounting_yaw0|std_mounting_roll|std_mounting_pitch|std_mounting_yaw|use_legacy_mounting_base_logic|ApplyStateMaskToCov" scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py src\core\eskf_engine.cpp src\app\pipeline_fusion.cpp`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\summary.md`
  - `python -c "import csv; ... print({'pitch_min':...,'pitch_max':...,'yaw_min':...,'yaw_max':...})"`
  - `python -c "import csv; ... print(rows[:5]); print(rows[-5:])"`
- artifacts:
  - summary=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/summary.md`
  - case_1x_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
- metrics:
  - masking_root_cause=`ApplyStateMaskToCov()` 对冻结状态执行 `P_.row(i).setZero(); P_.col(i).setZero();`
  - phase2_seed_contract=`phase2_mounting_cov_seed duration=0.02 s`, `enable_covariance_floor=true`, `p_floor_mounting_deg=3.0`
  - init_semantics=`mounting_roll0/pitch0/yaw0=0`, `std_mounting_roll/pitch/yaw=3.0 deg`, `use_legacy_mounting_base_logic=false`
  - q1_phase2_motion=`mounting_pitch=[0.0019226,0.363886984] deg`, `mounting_yaw=[-0.59607801,-0.043200119] deg`
  - q_neutrality=`0.5x/1x/2x` 的 `overall RMSE3D`、`phase2 RMSE3D`、`phase3 RMSE3D` 与 `bg_z_err_max` 在 `summary.md` 中几乎完全重合
- artifact_mtime:
  - reused_fresh_output=`summary/manifest generated_at=2026-03-25T22:37:47`
- config_hash_or_mtime:
  - script_reference=`scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py @ same-day validated`
- dataset_time_window:
  - validation_window=`phase2 motion check over 528276.0~528776.0`
- result_freshness_check:
  - 本次未重新求解，仅复核上一 session fresh 产物；`r2` 输出时间 `22:37:47 local` 晚于修复实现与编译时间，且与脚本当前 phase2 seed 语义一致。
- observability_notes:
  - state_block_mapping:
    `phase1` 冻结 `mounting(22-24)` 时协方差被直接清零；`phase2_mounting_cov_seed` 重新对 `mounting(22-24)` 注入 covariance floor；`phase3` 再次冻结 `mounting(22-24)`，因此本轮 `Q` sweep 只影响第二阶段学习结果。
  - schedule_effect:
    `q_1x` 的状态序列显示 phase1 末仍为 `mounting_pitch=0`, `mounting_yaw=0`，而 phase2 开始数秒后已移动到 `mounting_pitch≈0.235 deg`, `mounting_yaw≈-0.400 deg`，证明第二阶段 seed 生效且状态未被继续钉死。
  - behavior_change:
    用户指出的“应在第二阶段开始给大初始安装角噪声”已在 `r2` 落地；修复后安装角可动，但 `Q` 倍率仍非一阶敏感项。
- decision:
  - 用户对旧 `r1` 的质疑成立，旧结果不能再作为“zero-init mounting 中性”的证据。
  - 当前正确答案是：不是单纯 `Q` 太小，而是旧实验先把 mounting 协方差清掉了；修复后 mounting 会动，但 `0.5x/1x/2x` 仍几乎等价。
  - 后续分析应转向 `phase2` 学习质量与 phase3 outage 主坏链，而不是继续纠缠这组 mounting `Q` 倍率。
- next_step:
  - 继续执行首个长窗 `gnss_off` 的 truth-init `6x` vs corrected zero-init `r2::q_1x` 机制对账，定位为何 `phase2` 可学习但 `phase3` 仍更差。
  - 若再做 mounting 相关调参，优先考虑第二阶段学习窗口、冻结时机或与 `bg_z/odo_scale` 的竞争关系，而不是单独继续扩 `Q` 倍率。

### session_id: 20260325-2250-data2-staged-g5-mounting-yaw-sign-audit

- objective:
  - 回答用户关于“pitch 接近真值而 yaw 看起来一开始直接估反”的问题，判定这是状态语义误读、总安装角语义错误，还是程序中的 yaw 符号真的写反。
- scope:
  - 不改代码；核查 baseline `constraints.imu_mounting_angle`、truth reference 的 mounting 定义、求解器输出中的 `state yaw` 与 `total yaw`、以及 `mount_yaw` Jacobian 的解析/数值差分一致性。
- changed_files:
  - `walkthrough.md`
- configs:
  - reference_exp_zero_init=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2::staged_g5_mounting_zero_init_q_1x`
  - reference_exp_truth_init=`EXP-20260325-data2-staged-g5-no-imu-scale-r1::staged_g5_odo_nhc_noise_6x`
  - base_config=`config_data2_baseline_eskf.yaml`
  - measurement_model=`src/core/measurement_models_uwb.cpp`
  - truth_reference_builder=`scripts/analysis/run_data2_state_sanity_matrix.py`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\systematic-debugging\SKILL.md`
  - `rg -n "truth_mounting|imu_mounting_angle|C_b_v|mounting_base_rpy|transpose\\(|Stored mounting angle|inverse rotation|total_mounting_yaw_deg" include\app\fusion.h src\app\pipeline_fusion.cpp src\app\evaluation.cpp src\core\measurement_models_uwb.cpp scripts\analysis config_data2_baseline_eskf.yaml`
  - `Get-Content config_data2_baseline_eskf.yaml`
  - `Get-Content dataset\data2\README.md`
  - `Import-Csv output\data2_bgz_jacobian_fd_audit_r1_20260325\fd_vs_analytic.csv | Where-Object {$_.state_name -eq 'mount_yaw'} | Format-Table ...`
  - `python -c "import csv; ... compare zero-init q1 all_states/state_series and truth-init 6x state_series ..."`
- artifacts:
  - zero_init_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
  - zero_init_state_series=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/state_series_staged_g5_mounting_zero_init_q_1x.csv`
  - truth_init_state_series=`output/data2_staged_g5_no_imu_scale_r1_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/state_series_staged_g5_odo_nhc_noise_6x.csv`
  - fd_audit=`output/data2_bgz_jacobian_fd_audit_r1_20260325/fd_vs_analytic.csv`
- metrics:
  - baseline_mounting_base=`constraints.imu_mounting_angle=[0.0,0.0,1.38] deg`
  - readme_total_mounting_truth=`pitch=0.36 deg`, `yaw=1.37 deg`
  - derived_state_truth=`mounting_pitch_state_truth=0.36 deg`, `mounting_yaw_state_truth=-0.01 deg`
  - zero_init_phase2_start=`state_yaw=-0.043200 deg`, `total_yaw=1.336800 deg`
  - zero_init_phase2_early_window=`total_yaw min/max over 528276~528330 s = 0.847749 / 1.336800 deg`
  - zero_init_phase2_full=`total_yaw mean/min/max over 528276~528776 s = 0.917764 / 0.783922 / 1.336800 deg`
  - truth_init_phase2_full=`total_yaw mean/min/max over 528276~528776 s = 1.369999 / 1.369998 / 1.370000 deg`
  - fd_mount_yaw_sample=`t528276.004906 NHC row0 analytic=-16.7580435731, numeric=-16.7580435731; ODO row0 analytic=-0.00416748797812, numeric=-0.00416747525378`
- artifact_mtime:
  - reused_outputs=`all referenced outputs are 2026-03-25 fresh artifacts from prior sessions`
- config_hash_or_mtime:
  - baseline_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
- dataset_time_window:
  - audit_windows=`phase2 early 528276~528330 s`, `phase2 full 528276~528776 s`
- result_freshness_check:
  - 本次未重跑求解器；只复用已记录为 fresh 的 `r2`、truth-init `6x` 与 `fd_audit` 产物，并与当前代码/配置逐项核对。
- observability_notes:
  - state_block_mapping:
    `mounting_yaw(24)` 在当前设计里是相对 `constraints.imu_mounting_angle[2]` 的状态增量，不是总安装角；求解器另存了 `total_mounting_yaw_deg = base + state`。
  - schedule_effect:
    `phase2` 开始时 zero-init case 的 `mounting_yaw_deg` 进入负值，首先意味着“从 base yaw=1.38 deg 往 README 总真值 1.37 deg 的负方向修正”，这个方向本身没有语义错误；但随后 `total_mounting_yaw_deg` 持续跌到 `~0.78~0.89 deg`，说明 update 链把 yaw 拉过头了。
  - behavior_change:
    truth-init `6x` 在同一 phase2 窗内能稳定保持 `total_mounting_yaw≈1.37 deg`；结合 `mount_yaw` Jacobian 的解析/数值差分同号一致，当前更像是 zero-init 条件下的 phase2 竞争/耦合问题，而不是程序把 yaw 正负号写反。
- decision:
  - `mounting_yaw_deg` 为负并不等于“估反了”，因为在当前 baseline 下 yaw 真值状态本来就是 `-0.01 deg`。
  - 现有证据不支持“程序中 yaw 符号写反”这一解释；`measurement_models_uwb.cpp` 的符号约定与 `fd_vs_analytic.csv` 数值差分一致。
  - 真正存在的问题是：zero-init case 的 yaw 更新方向虽然一开始与真值修正方向一致，但幅值迅速过冲，导致 `total_mounting_yaw` 偏离真值，这应归入 phase2 ODO/NHC 与 `bg_z/attitude` 耦合链的机理问题。
- next_step:
  - 在 `528276~528330 s` 窗口提取 `dx_mount_yaw`、`dx_bg_z`、`innov_odo`、`innov_nhc_y`、`K_mount_yaw` 与 `K_bg_z`，确认是谁在把 zero-init yaw 从 `1.3368` 继续拉向 `0.8~0.9 deg`。
  - 后续所有 mounting yaw 图同时看 `mounting_yaw_deg` 与 `total_mounting_yaw_deg`，避免再把“状态增量为负”误判成“总安装角反号”。

### session_id: 20260325-2313-data2-staged-g5-total-mounting-semantics-rerun

- objective:
  - 按用户要求把当前实验对外暴露的 `mounting_yaw` 语义统一为总安装角，并基于新语义 fresh 重跑 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2`。
- scope:
  - 不改滤波器内部状态坐标；仅在 analysis/helper 层把 `all_states` 和 plots 中的 `mounting_*` 列切换到 total mounting，同时保留原始状态增量为 `mounting_state_*`；补测试、回归并 fresh 重跑 `r2`。
- changed_files:
  - `scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py`
  - `tests/test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - rerun_script=`scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - output_dir=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/`
  - semantics_change=`all_states/plots mounting_* => total mounting`, `mounting_state_* => raw solver delta`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\brainstorming\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `Get-Process | Where-Object { $_.ProcessName -match 'python|eskf_fusion|pytest|cmake' }`
  - `rg -n "total_mounting_|mounting_pitch_deg|mounting_yaw_deg|build_state_frame|build_truth_reference" scripts\analysis src\app tests`
  - `Get-Content scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py`
  - `Get-Content scripts\analysis\run_data2_state_sanity_matrix.py`
  - `Get-Content tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `pytest tests\test_run_data2_fullwindow_attitude_bias_coupling.py -k mounting_semantics` (`RED` then `GREEN`)
  - `pytest tests\test_run_data2_fullwindow_attitude_bias_coupling.py tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `python -m py_compile scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `python scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\summary.md`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\all_states_staged_g5_mounting_zero_init_q_1x.csv -Head 3`
  - `python -c "import csv; ... compute phase2 total/state mounting ranges ..."`
- artifacts:
  - summary=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/summary.md`
  - manifest=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/manifest.json`
  - mounting_plot=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/plots/mounting.png`
  - case_1x_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
- metrics:
  - test_status=`pytest 9 passed (fullwindow + staged_g5_mounting + staged_g5_no_imu_scale)`
  - semantics_contract=`mounting_yaw_deg(total)=1.38 deg at phase1 start`, `truth_mounting_yaw_deg=1.37 deg`, `mounting_state_yaw_deg=0 deg at phase1 start`
  - q1_phase2_total_mounting=`pitch=[0.0019226,0.363886984] deg`, `yaw=[0.78392199,1.336799881] deg`
  - q1_phase2_state_mounting=`pitch=[0.0019226,0.363886984] deg`, `yaw=[-0.59607801,-0.043200119] deg`
  - nav_metrics_unchanged=`overall RMSE3D=3.061294 m`, `phase2 RMSE3D=0.070415 m`, `phase3 RMSE3D=3.633109 m`
- artifact_mtime:
  - summary_manifest=`2026-03-25 23:13:04 local`
  - mounting_plot=`2026-03-25 23:13:03 local`
- config_hash_or_mtime:
  - helper_script=`scripts/analysis/run_data2_fullwindow_attitude_bias_coupling.py modified in current session`
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
- dataset_time_window:
  - experiment_window=`528076.0~530488.9`
- result_freshness_check:
  - 先新增 mounting total-semantics 契约测试并确认 RED，再实现到 GREEN。
  - 回归 `pytest` 9 项全绿，`py_compile` 通过。
  - fresh 重跑 `run_data2_staged_g5_mounting_zero_init_q_sweep.py`，`summary/manifest/mounting plot` 时间为 `23:13 local`，晚于当前修改与测试时间。
- observability_notes:
  - state_block_mapping:
    滤波器内部 `mounting(22-24)` 仍是相对 `constraints.imu_mounting_angle` 的状态坐标；本次只改变实验输出语义，不改变内部可观性结构。
  - schedule_effect:
    `phase2` 的 total mounting 曲线现在可直接与 README 总安装角真值对比；原始状态增量仍保留在 `mounting_state_*`，便于后续分析 `bg_z` 耦合。
  - behavior_change:
    `mounting_yaw_deg` 不再需要和 `total_mounting_yaw_deg` 来回换算；用户看到的主图已统一为总安装角。
- decision:
  - 当前最小且可验证的改法是“对外统一为 total mounting、对内保留 delta state”，已满足实验解释需求且不扰动滤波器核心。
  - fresh 重跑表明语义修正不改变导航主指标，只改变 mounting 图像与 `all_states` 的解释方式。
- next_step:
  - 继续按新的 total mounting 语义审计 `phase2` 早窗 `dx_mount_yaw/dx_bg_z/K`，解释为何 total yaw 会从 `1.3368 deg` 继续滑到 `~0.8~0.9 deg`。
  - 如需跨实验做直接对比，再补跑 `EXP-20260325-data2-staged-g5-no-imu-scale-r1`，把其 `all_states/plots` 同步刷新到 total mounting 语义。

### session_id: 20260325-2318-data2-staged-g5-zero-init-vs-total-output-clarification

- objective:
  - 回答用户“为什么 yaw 安装角没有从 0 开始估计”的问题，明确当前实验里“0 初值开始”对应的是内部状态增量还是对外显示的总安装角。
- scope:
  - 不改代码；仅核查 fresh `q_1x` 配置文件、`all_states` 首行以及 helper 中 total-mounting 兼容层，澄清当前行为是否符合原实验设计。
- changed_files:
  - `walkthrough.md`
- configs:
  - reference_case=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2::staged_g5_mounting_zero_init_q_1x`
  - reference_config=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/config_staged_g5_mounting_zero_init_q_1x.yaml`
  - reference_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
- commands:
  - `Get-Content walkthrough.md -Head 40`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\config_staged_g5_mounting_zero_init_q_1x.yaml`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\all_states_staged_g5_mounting_zero_init_q_1x.csv -Head 2`
  - `rg -n "mounting_yaw0|use_legacy_mounting_base_logic|imu_mounting_angle|total_mounting_yaw_deg|mounting_state_yaw_deg" scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py config_data2_baseline_eskf.yaml output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\config_staged_g5_mounting_zero_init_q_1x.yaml`
- artifacts:
  - reference_config=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/config_staged_g5_mounting_zero_init_q_1x.yaml`
  - reference_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
- metrics:
  - internal_zero_init=`init.mounting_yaw0=0.0`, `use_legacy_mounting_base_logic=false`
  - base_mounting_total=`constraints.imu_mounting_angle[2]=1.38 deg`
  - all_states_first_row=`mounting_state_yaw_deg=0.0`, `mounting_yaw_deg(total)=1.38`, `truth_mounting_yaw_deg(total)=1.37`
- artifact_mtime:
  - reused_fresh_case=`all_states from 2026-03-25 23:13 local rerun`
- config_hash_or_mtime:
  - reference_config=`config_staged_g5_mounting_zero_init_q_1x.yaml generated at 2026-03-25 23:12~23:13 local`
- dataset_time_window:
  - clarification_row=`first sample at 528076.014367 s`
- result_freshness_check:
  - 本次未重跑求解器；复用 `23:13 local` 的 fresh `r2` 结果，并与当前 case config/helper 代码逐项核对。
- observability_notes:
  - state_block_mapping:
    当前 zero-init 设计仍然是“内部 `mounting_yaw(24)` 从 `0` 起跑”，只是对外展示层把 `mounting_yaw_deg` 映射成了 `base + state` 的 total mounting。
  - schedule_effect:
    因此 phase1 起点看到 `mounting_yaw_deg=1.38` 不表示“没有从 0 开始估计”，而表示“总安装角 = base 1.38 + state 0”。
  - behavior_change:
    若用户要的是“总安装角本身也从 0 起跑”，那已经不是当前这版 total-output 兼容层，而是需要把 `constraints.imu_mounting_angle` 的 yaw 基准也改成 `0` 再重跑。
- decision:
  - 当前实现满足的是“内部状态增量 0 初值开始”，不满足“总安装角 0 初值开始”。
  - 用户指出这一点是合理的；若后续要求“总安装角也从 0 开始估计”，需要修改实验配置而不只是输出语义。
- next_step:
  - 如果继续按 total-mounting 语义做研究，需要先决定保留“delta=0 起跑”还是改成“total=0 起跑”。
  - 若用户选择后者，下一步应把 zero-init case 的 `constraints.imu_mounting_angle` 对应轴改为 `0`，并同步调整 truth-reference/图表语义后 fresh 重跑。

### session_id: 20260325-2340-data2-staged-g5-total-zero-init-fix

- objective:
  - 按用户确认把 `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2` 从“内部 mounting 增量 0 初值”修正为“总安装角也从 0 初值开始估计”，并 fresh 重跑实验。
- scope:
  - 仅修改 staged G5 zero-init `Q` sweep 的实验脚本、对应回归测试与 `walkthrough.md`；不改滤波器核心状态定义。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `tests/test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/`
  - cases=`staged_g5_mounting_zero_init_q_0p5x`, `staged_g5_mounting_zero_init_q_1x`, `staged_g5_mounting_zero_init_q_2x`
  - new_semantics=`constraints.imu_mounting_angle=[0,0,0]`, `init.mounting_roll0/pitch0/yaw0=0`, `case-level truth_reference`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\skills\using-superpowers\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\systematic-debugging\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `Get-Content scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `Get-Content tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py -k zero_init_q_sweep_plan`
  - `pytest tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `python scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\config_staged_g5_mounting_zero_init_q_1x.yaml`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\artifacts\cases\staged_g5_mounting_zero_init_q_1x\all_states_staged_g5_mounting_zero_init_q_1x.csv -Head 2`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\truth_reference.json`
  - `Get-Content output\data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325\summary.md`
  - `@' ... extract phase2 mounting_pitch/yaw min/max from q1 all_states ... '@ | python -`
- artifacts:
  - summary=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/summary.md`
  - manifest=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/manifest.json`
  - truth_reference=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/truth_reference.json`
  - q1_config=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/config_staged_g5_mounting_zero_init_q_1x.yaml`
  - q1_all_states=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/artifacts/cases/staged_g5_mounting_zero_init_q_1x/all_states_staged_g5_mounting_zero_init_q_1x.csv`
  - mounting_plot=`output/data2_staged_g5_mounting_zero_init_q_sweep_r2_20260325/plots/mounting.png`
- metrics:
  - test_status=`RED->GREEN on zero-init contract test`, `pytest 9 passed`, `py_compile passed`
  - config_semantics=`constraints.imu_mounting_angle=[0,0,0]`, `init.mounting_*0=0`, `truth_reference.sources.constraints_mounting_base_deg=[0,0,0]`
  - q1_first_row=`timestamp=528076.014367 s`, `mounting_yaw(total)=0.0 deg`, `mounting_state_yaw=0.0 deg`, `truth_mounting_yaw=1.37 deg`
  - q1_phase2_total=`mounting_pitch=[0.001761741,0.361059399] deg`, `mounting_yaw=[0.060738648,0.917667460] deg`, `phase2_end_mounting_yaw=0.896198382 deg`
  - q_sweep_metrics=`overall RMSE3D=3.399490 m`, `phase2 RMSE3D=0.070300 m`, `phase3 RMSE3D=4.034528 m`, `yaw_err_max_abs=3.954837 deg`, `bg_z_err_max_abs=171.052868 deg/h`
  - vs_truth_init_6x=`Δphase3=+0.856139 m`, `Δbg_z_err_max=+60.855923 deg/h`
- artifact_mtime:
  - summary_manifest=`2026-03-25 23:39:10 local`
  - q1_all_states=`2026-03-25 23:37:55 local`
- config_hash_or_mtime:
  - experiment_script=`scripts/analysis/run_data2_staged_g5_mounting_zero_init_q_sweep.py modified in current session`
  - regression_test=`tests/test_run_data2_staged_g5_mounting_zero_init_q_sweep.py modified in current session`
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
- dataset_time_window:
  - full_window=`528076.0~530488.9`
  - phase2_window=`528276.0~528776.0`
  - first_row_check=`528076.014367 s`
- result_freshness_check:
  - 先新增 “total mounting zero-init” 契约测试并确认 RED，再最小修改脚本到 GREEN。
  - 回归 `pytest` 9 项全绿，`py_compile` 通过。
  - fresh 重跑 `run_data2_staged_g5_mounting_zero_init_q_sweep.py`，`summary/manifest` 时间为 `23:39:10 local`，晚于当前代码修改与测试时间。
- observability_notes:
  - state_block_mapping:
    本次没有改变滤波器内部 `mounting(22-24)` 的状态定义，只把该实验的 mounting base 改为 `0`；因此在这组 case 中 `mounting_state_*` 与 `mounting_* (total)` 数值相同，真值则变为 `mounting_pitch=0.36 deg`, `mounting_yaw=1.37 deg`。
  - schedule_effect:
    `phase1` 仍冻结 `mounting(22-24)`，`phase2_mounting_cov_seed` 仍在 `528276.0~528276.02 s` 对 `mounting(22-24)` 注入 `3 deg` covariance floor，之后 phase2 联合 `GNSS/ODO/NHC` 开始学习 mounting，phase3 再冻结 `mounting/odo_scale/lever_odo`。
  - behavior_change:
    总安装角从 `0` 起跑后，`q_1x` 到 `phase2_end` 只学到 `mounting_yaw≈0.896 deg`，明显低于真值 `1.37 deg`；同时 `0.5x/1x/2x` 导航指标仍几乎完全重合，说明当前差异不是由 mounting `Q` 倍率主导，而更像是 phase2 可观性不足导致的初始学习质量差异。
- decision:
  - 用户指出的问题成立：此前 `r2` 只满足“delta zero-init”，不满足“total zero-init”。
  - 现已把实验修正为真正的总安装角 0 初值，并完成 fresh 重跑；该 fresh 结果应覆盖旧 `23:13 local` 那版 `r2` 解释。
  - total-zero-init 相比 truth-init `6x` 更差，且比上一版 delta-zero-init `r2` 还更差，进一步强化 `HYP-53`：主问题不是 mounting `Q` 倍率，而是 phase2 mounting 学习质量及其在 phase3 的 downstream 放大。
- next_step:
  - 先对 truth-init `staged_g5_odo_nhc_noise_6x` 与 total-zero-init `staged_g5_mounting_zero_init_q_1x` 做 `phase2` 早窗 `528276~528330 s` 的 `dx_mounting_yaw/dx_bg_z/K` 机理审计。
  - 再对两组 case 的首个长窗 `gnss_off_01 (528866~528956 s)` 抓取 `ODO/NHC` mechanism、`NIS` 与 `P(bg_z,*)`，解释为何 total-zero-init 的 phase3 更差。

### session_id: 20260326-0012-data2-staged-g5-phase3-ins-only-control

- objective:
  - 在 staged `G5` 三段式实验中新增一个 phase3 `INS/GNSS outage` 对照组，即 phase1/phase2 保持 `6x` 基线一致，phase3 显式关闭 `ODO/NHC`，用于判断当前 phase3 大误差是否主要由 `ODO/NHC` 自身带坏。
- scope:
  - 仅修改 `run_data2_staged_g5_no_imu_scale.py`、对应测试与 `walkthrough.md`；不改求解器核心。
- changed_files:
  - `scripts/analysis/run_data2_staged_g5_no_imu_scale.py`
  - `tests/test_run_data2_staged_g5_no_imu_scale.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260325-data2-staged-g5-no-imu-scale-r2`
  - base_config=`config_data2_baseline_eskf.yaml`
  - solver=`build/Release/eskf_fusion.exe`
  - output_dir=`output/data2_staged_g5_no_imu_scale_r2_20260325/`
  - cases=`staged_g5_odo_nhc_noise_4x`, `staged_g5_odo_nhc_noise_5x`, `staged_g5_odo_nhc_noise_6x`, `staged_g5_odo_nhc_noise_6x_phase3_ins_only`
  - control_semantics=`phase1/phase2 same as 6x`, `phase3 enable_odo=false`, `phase3 enable_nhc=false`
- commands:
  - `Get-Content walkthrough.md -Head 80`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\brainstorming\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\writing-plans\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\verification-before-completion\SKILL.md`
  - `Get-Content scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `Get-Content tests\test_run_data2_staged_g5_no_imu_scale.py`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py -k staged_g5_plan`
  - `pytest tests\test_run_data2_staged_g5_no_imu_scale.py tests\test_run_data2_staged_g5_mounting_zero_init_q_sweep.py tests\test_run_data2_fullwindow_attitude_bias_coupling.py`
  - `python -m py_compile scripts\analysis\run_data2_staged_g5_no_imu_scale.py scripts\analysis\run_data2_staged_g5_mounting_zero_init_q_sweep.py scripts\analysis\run_data2_fullwindow_attitude_bias_coupling.py`
  - `python scripts\analysis\run_data2_staged_g5_no_imu_scale.py`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\summary.md`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\case_metrics.csv`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\artifacts\cases\staged_g5_odo_nhc_noise_6x_phase3_ins_only\config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml`
- artifacts:
  - summary=`output/data2_staged_g5_no_imu_scale_r2_20260325/summary.md`
  - manifest=`output/data2_staged_g5_no_imu_scale_r2_20260325/manifest.json`
  - case_metrics=`output/data2_staged_g5_no_imu_scale_r2_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_no_imu_scale_r2_20260325/phase_metrics.csv`
  - position_plot=`output/data2_staged_g5_no_imu_scale_r2_20260325/plots/position.png`
  - key_states_plot=`output/data2_staged_g5_no_imu_scale_r2_20260325/plots/key_coupling_states.png`
  - control_config=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml`
  - control_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv`
- metrics:
  - test_status=`RED->GREEN on phase3-ins-only contract test`, `pytest 9 passed`, `py_compile passed`
  - control_config_contract=`phase3 runtime_phases[2].constraints.enable_odo=false`, `enable_nhc=false`
  - baseline_6x=`phase2 RMSE3D=0.071153 m`, `phase3 RMSE3D=3.178389 m`, `gnss_off_01 RMSE3D=5.220606 m`, `gnss_off_01 final_err_3d=8.296212 m`
  - phase3_ins_only_control=`overall RMSE3D=26.673278 m`, `phase2 RMSE3D=0.071153 m`, `phase3 RMSE3D=31.657637 m`, `gnss_off_01 RMSE3D=46.206528 m`, `gnss_off_01 final_err_3d=112.485737 m`, `gnss_off_09 final_err_3d=205.608601 m`
  - comparison=`Δphase3(INS-only - 6x)=+28.479248 m`, `Δgnss_off_01 RMSE=+40.985922 m`, `Δgnss_off_01 final_err_3d=+104.189526 m`
  - error_character=`phase3 INS-only yaw_err_max_abs=1.098057 deg`, `bg_z_err_max_abs=110.041076 deg/h`
- artifact_mtime:
  - summary_manifest=`2026-03-26 00:11:49 local`
  - plots=`2026-03-26 00:11:46 local`
  - control_all_states=`2026-03-26 00:11:43 local`
- config_hash_or_mtime:
  - experiment_script=`scripts/analysis/run_data2_staged_g5_no_imu_scale.py modified in current session`
  - regression_test=`tests/test_run_data2_staged_g5_no_imu_scale.py modified in current session`
  - base_config=`config_data2_baseline_eskf.yaml @ 2026-03-21 23:38:56 local`
- dataset_time_window:
  - full_window=`528076.0~530488.9`
  - first_phase3_outage=`528866.0~528956.0`
- result_freshness_check:
  - 先补 phase3 INS-only 对照 case 的契约测试并确认 RED，再最小修改脚本到 GREEN。
  - 回归 `pytest` 9 项全绿，`py_compile` 通过。
  - fresh 重跑 `run_data2_staged_g5_no_imu_scale.py`，`summary/manifest` 时间为 `2026-03-26 00:11:49 local`，晚于当前代码修改与测试时间。
- observability_notes:
  - state_block_mapping:
    新增 control case 不改变 phase1/phase2 的 `mounting(22-24)`, `lever_odo(25-27)`, `lever_gnss(28-30)`, `odo_scale(21)` 处理，只在 phase3 关闭 `ODO/NHC` 更新，因此 phase2 学习结果与 `6x` 基线完全对齐。
  - schedule_effect:
    phase3 仍沿用 `90 s on / 90 s off` 的 GNSS 周期窗口；control case 仅在 `gnss_off` 时退化为纯 INS，在 `gnss_on` 时为 INS/GNSS。
  - behavior_change:
    纯对照组的 phase3 漂移比 `6x` 基线大一个数量级，首个 `90 s` outage 就从 `8.296 m` 终点误差恶化到 `112.486 m`，说明当前 `ODO/NHC` 在 phase3 总体上是强约束而不是坏源；剩余 gap 更像是纯 INS 传播本身远超预期，以及 `ODO/NHC` 只能部分补偿。
- decision:
  - 用户要求的 phase3 `INS/GNSS outage` 对照已落地并 fresh 重跑。
  - 现有证据不支持“当前 phase3 大误差主要是 ODO/NHC 自身带坏”；相反，去掉它们后 phase3 明显更糟。
  - 接下来更有价值的问题是：为什么纯 INS `90 s` outage 已经远超按惯导精度预期的 `~2 m`，以及为什么加上 ODO/NHC 后仍只能压到 `~5.22 m` 而不是更低。
- next_step:
  - 先对 `staged_g5_odo_nhc_noise_6x` 与 `staged_g5_odo_nhc_noise_6x_phase3_ins_only` 的首个 `gnss_off_01` 做 propagation vs update 分解，量化纯 INS 的误差增长率与 `ODO/NHC` 的净贡献。
  - 再回到 total-zero-init `q_1x`，继续审计 phase2 学习不足与 phase3 剩余误差之间的关联。

### session_id: 20260326-0038-pure-ins-mech-audit

- objective:
  - 代码审查当前纯惯导传播链，判断 `90 s` 纯 `INS/GNSS outage` 远超预期是否来自机械编排中的不合理近似或语义错误。
- scope:
  - 只做代码与已有 fresh artifact 审查；读取 `src/core/ins_mech.cpp`、`src/core/eskf_engine.cpp`、`src/core/measurement_models_uwb.cpp`、`src/utils/math_utils.cpp`、`src/app/pipeline_fusion.cpp` 以及 `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 的 `summary/csv/state_series`；不改求解器、不新增实验。
- changed_files:
  - `walkthrough.md`
- configs:
  - inspected_experiment=`EXP-20260325-data2-staged-g5-no-imu-scale-r2`
  - inspected_case=`staged_g5_odo_nhc_noise_6x_phase3_ins_only`
  - inspected_solver_path=`build/Release/eskf_fusion.exe (no rebuild/no rerun in this session)`
- commands:
  - `Get-Content walkthrough.md -TotalCount 220`
  - `rg -n "GravityEcef|QuatFromSmallAngle|QuatToRot|EcefToLlh|RotNedToEcef|Skew|omega_ie_b|omega_en|ApplyMarkovNominalPropagation|ComputeErrorStateF|ProcessNoise" src include`
  - `Get-Content src\core\ins_mech.cpp`
  - `Get-Content src\core\eskf_engine.cpp`
  - `Get-Content src\utils\math_utils.cpp`
  - `Get-Content src\core\measurement_models_uwb.cpp`
  - `Get-Content include\core\eskf.h`
  - `Get-Content src\app\pipeline_fusion.cpp`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\summary.md`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\case_metrics.csv`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\phase_metrics.csv`
  - `Select-String -Path output\data2_staged_g5_no_imu_scale_r2_20260325\artifacts\cases\staged_g5_odo_nhc_noise_6x_phase3_ins_only\state_series_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv -Pattern '^528866\.'`
  - `Select-String -Path output\data2_staged_g5_no_imu_scale_r2_20260325\artifacts\cases\staged_g5_odo_nhc_noise_6x_phase3_ins_only\state_series_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv -Pattern '^528955\.'`
- artifacts:
  - summary=`output/data2_staged_g5_no_imu_scale_r2_20260325/summary.md`
  - case_metrics=`output/data2_staged_g5_no_imu_scale_r2_20260325/case_metrics.csv`
  - phase_metrics=`output/data2_staged_g5_no_imu_scale_r2_20260325/phase_metrics.csv`
  - inspected_state_series=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/state_series_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv`
- metrics:
  - code_audit_findings:
    `Predict()` 每步执行 nominal `ba/bg/sg/sa` 指数回零；nominal mechanization 用 coning/sculling + ECEF 姿态/速度传播；而 process/update 侧的 `omega_ib/omega_nb` 主要用单样本一阶角速率。
  - phase3_ins_only_first_window=`RMSE3D=46.206528 m`, `final_err_3d=112.485737 m`, `yaw_abs_max_deg=0.059856`
  - nominal_bias_relaxation_observed=`ba_x: 1401.921268 -> 1367.309516 mGal`, `bg_x: -11.133383 -> -10.858513 deg/h`, window=`528866.001484~528955.996726`
  - phase2_shared_entry=`phase2 RMSE3D stays 0.071153 m for 6x baseline and phase3 INS-only control`
- artifact_mtime:
  - summary_phase_csv=`2026-03-26 00:11:49 local`
  - inspected_state_series=`2026-03-26 00:11:17 local`
- config_hash_or_mtime:
  - control_config=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml @ 2026-03-26 00:10:45 local`
  - source_files=`inspection only; no code edits this session`
- dataset_time_window:
  - first_outage_window=`528866.0~528956.0`
- result_freshness_check:
  - 本轮未重跑求解器；结论仅基于 `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 的 fresh artifact 与当前 mainline 代码审计。
  - 所引用 `summary/case_metrics/phase_metrics/state_series` 时间一致，且与 `20260326-0012` 会话记录匹配。
- observability_notes:
  - state_block_mapping:
    纯传播直接影响 `ba/bg(9-14)` 与 nominal PVA；当前审计未发现 phase3 `INS-only` 首窗是 `mounting(22-24)` 或 `lever_odo(25-27)` 在 phase3 被继续写坏，因为该 control case 在 phase3 已冻结这些状态且关闭 `ODO/NHC`。
  - schedule_effect:
    `phase3` 首窗 `528866~528956 s` 中 `GNSS/ODO/NHC` 都不参与更新，因此该窗口的名义轨迹只受 `IMU mechanization + nominal ba/bg propagation` 支配；误差状态 `F/Q` 只影响协方差，不直接驱动名义轨迹。
  - behavior_change:
    代码层未发现“ECEF 姿态传播漏掉 `omega_en`”这类直接硬错误；更像是 phase2 交出的 roll/pitch/ba/bg 入口质量不足，再叠加 phase3 nominal bias 回零，使纯 INS 漂移被放大。
- decision:
  - 当前最可疑的纯惯导近似不是 yaw 符号或 ECEF/Earth-rate 主公式写反，而是 `Predict()` 对 nominal `ba/bg` 的持续 GM 回零，以及 nominal / process / measurement 三处角速率定义不完全一致。
  - 现有 fresh 指标也不支持把首个 `90 s` 漂移主因归到 yaw：`gnss_off_01 yaw_abs_max_deg=0.059856` 明显过小，更应优先检查 roll/pitch 与 accel-bias 链。
  - 若用户后续要做最小验证实验，优先级应高于继续扩展 `ODO/NHC` 变体：先 A/B `phase3` nominal bias hold vs decay，再补导出首窗 roll/pitch 与 `ba/bg` truth error。
- next_step:
  - 做一个最小 solver A/B：在不改 phase2/phase3 协方差模型的前提下，仅关掉 `ApplyMarkovNominalPropagation()` 对 nominal `ba/bg` 的回零，重跑 `staged_g5_odo_nhc_noise_6x_phase3_ins_only`，比较首个 `gnss_off_01` 漂移是否明显下降。
  - 补一条 analysis 脚本或导出字段，直接给出首个 `gnss_off_01` 的 roll/pitch 误差、phase3 起点 `ba/bg` truth error 与其对 `90 s` 漂移的量级解释。

### session_id: 20260326-0047-nominal-bias-decay-explanation

- objective:
  - 向用户更详细解释当前代码里的 nominal `ba/bg` 回零语义、为什么它在本实验里值得优先怀疑，以及它大致能贡献到什么量级的漂移。
- scope:
  - 仅基于现有代码与 fresh artifact 做解释性分析；不改求解器、不新增实验。
- changed_files:
  - `walkthrough.md`
- configs:
  - inspected_config=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml`
  - markov_corr_time=`3600 s`
- commands:
  - `Get-Content src\core\eskf_engine.cpp`
  - `Get-Content src\core\ins_mech.cpp`
  - `Get-Content output\data2_staged_g5_no_imu_scale_r2_20260325\artifacts\cases\staged_g5_odo_nhc_noise_6x_phase3_ins_only\config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml`
  - `Select-String -Path output\data2_staged_g5_no_imu_scale_r2_20260325\artifacts\cases\staged_g5_odo_nhc_noise_6x_phase3_ins_only\state_series_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv -Pattern '^528866\.|^528955\.'`
  - `powershell numeric check for exp(-dt/T), exp(-90/T), bias/position order-of-magnitude`
- artifacts:
  - referenced_config=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/config_staged_g5_odo_nhc_noise_6x_phase3_ins_only.yaml`
  - referenced_state_series=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/state_series_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv`
- metrics:
  - decay_factor=`per_step@5ms=0.9999986111`, `90s=0.9753099120`, `full_phase3_1712.9s=0.6213842952`
  - ba_x_decay_example=`1401.921268 mGal -> 1367.309516 mGal over first 90s`, `Δ=34.613559 mGal = 3.46135594e-4 m/s^2`
  - inferred_effect_if_isolated=`Δv@90s≈0.03115 m/s`, `Δp@90s≈1.40185 m`
  - bg_x_decay_example=`|-11.133383 -> -10.858513| deg/h`, `equivalent integrated angle over 90s ≈ 0.00687 deg if isolated`
- observability_notes:
  - interpretation:
    `ApplyMarkovNominalPropagation()` 与 `F/Q` 中的 `-1/T`、`sqrt(2/T)` 不是彼此矛盾，而是同一个 zero-mean GM bias 假设的 mean/covariance 两部分。
  - implication_for_this_repo:
    在当前 staged `G5` 上，phase2 学到的 `ba/bg` 数值明显不小，且很可能同时吸收了 residual mounting / attitude / scale mismatch；因此 phase3 再把 nominal bias 往 0 拉，会主动移除这些已学到的补偿。
  - limit_of_claim:
    当前仅能说明 nominal bias decay 是高优先级嫌疑项，而不能单独证明它就是全部 `46.2 m / 112.5 m` 漂移来源；roll/pitch 入口误差仍需并联核查。
- decision:
  - 对用户的进一步解释应明确区分“数学上一致”和“工程上适不适合当前 outage 研究目标”。
  - 当前更合理的表述是：这不是明显的公式 bug，而是一个很强的建模选择，它会在 phase3 纯传播时主动冲掉 phase2 学到的 bias 补偿。
- next_step:
  - 维持上一会话结论不变，优先做 `disable nominal ba/bg decay` 的最小 A/B，再用首窗 roll/pitch truth error 做量级归因。

### session_id: 20260326-0105-data2-staged-g5-6x-interactive-report

- objective:
  - 为当前 fresh `6x` staged `G5` 结果生成单页 Plotly 交互网页，集中展示 25 维非 PVA 状态、PVA 误差、`v` 系速度误差、`v` 系航向误差、轨迹，以及 phase3 无 `ODO/NHC` 的 PVA 对照。
- scope:
  - 只新增结果展示脚本、对应回归测试与网页产物；不改 solver、不重跑滤波实验。
- changed_files:
  - `scripts/analysis/build_data2_staged_g5_6x_interactive_report.py`
  - `tests/test_build_data2_staged_g5_6x_interactive_report.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260326-data2-staged-g5-6x-interactive-report-r1`
  - source_exp=`EXP-20260325-data2-staged-g5-no-imu-scale-r2`
  - input_dir=`output/data2_staged_g5_no_imu_scale_r2_20260325/`
  - output_html=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report.html`
  - output_manifest=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report_manifest.json`
  - main_case=`staged_g5_odo_nhc_noise_6x`
  - control_case=`staged_g5_odo_nhc_noise_6x_phase3_ins_only`
  - vehicle_truth=`roll=0.0 deg`, `pitch=0.36 deg`, `yaw=0.84 deg`
- commands:
  - `Get-Content walkthrough.md -Tail 220`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\executing-plans\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\test-driven-development\SKILL.md`
  - `Get-Content C:\Users\不存在的骑士\.codex\superpowers\skills\verification-before-completion\SKILL.md`
  - `pytest tests\test_build_data2_staged_g5_6x_interactive_report.py -q`
  - `python -m py_compile scripts\analysis\build_data2_staged_g5_6x_interactive_report.py`
  - `python scripts\analysis\build_data2_staged_g5_6x_interactive_report.py`
  - `pytest tests\test_build_data2_staged_g5_6x_interactive_report.py tests\test_run_data2_staged_g5_no_imu_scale.py -q`
  - `Select-String -Path output\data2_staged_g5_no_imu_scale_r2_20260325\interactive\staged_g5_6x_report.html -Pattern "6x 非 PVA 25 维状态变化","v 系航向误差","第三阶段不加 ODO/NHC 的 PVA 误差图","vv_x_err_mps","vehicle heading err" -SimpleMatch`
- artifacts:
  - report_html=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report.html`
  - report_manifest=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report_manifest.json`
  - main_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv`
  - control_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv`
- metrics:
  - test_status=`RED->GREEN on new report contract tests`, `pytest 8 passed`, `py_compile passed`
  - html_size=`12180342 bytes`
  - report_summary=`6x phase2 RMSE3D=0.071153 m`, `6x phase3 RMSE3D=3.178389 m`, `6x gnss_off_01 RMSE3D=5.220606 m`, `control phase3 RMSE3D=31.657637 m`, `control gnss_off_01 final_err_3d=112.485737 m`
  - report_layout=`25 non-PVA state tiles`, `3 main PVA cards`, `v-velocity/v-heading/trajectory row`, `3 phase3-control PVA cards`
- artifact_mtime:
  - report_html=`2026-03-26 01:05:10 local`
  - report_manifest=`2026-03-26 01:05:10 local`
- config_hash_or_mtime:
  - source_case_metrics=`output/data2_staged_g5_no_imu_scale_r2_20260325/case_metrics.csv @ 2026-03-26 00:11:43 local`
  - source_phase_metrics=`output/data2_staged_g5_no_imu_scale_r2_20260325/phase_metrics.csv @ 2026-03-26 00:11:43 local`
  - source_main_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv @ 2026-03-26 00:10:45 local`
  - source_control_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv @ 2026-03-26 00:11:43 local`
- dataset_time_window:
  - full_window=`528076.0~530488.9`
  - phase3_window=`528776.0~530488.9`
  - first_outage_window=`528866.0~528956.0`
- result_freshness_check:
  - 新网页不依赖新 solver run，只消费 `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 的 fresh artifact。
  - `report_html/report_manifest` 生成时间 `01:05:10 local` 晚于本轮脚本与测试完成时间，也晚于引用的 `case_metrics/phase_metrics/all_states` 时间。
  - 页面中已静态确认存在 `6x 非 PVA 25 维状态变化`、`第三阶段不加 ODO/NHC 的 PVA 误差图`、`vv_x_err_mps` 与 `vehicle heading err` 关键 section/trace。
- observability_notes:
  - state_block_mapping:
    页面显式覆盖 `ba/bg/sg/sa(9-20)`、`odo_scale(21)`、`mounting_state(22-24)`、`total mounting(22-24 + base)`、`lever_odo(25-27)`、`lever_gnss(28-30)`，并把 PVA 与非 PVA 状态分开展示。
  - schedule_effect:
    主页面复用原实验的 `phase1 -> phase2 -> phase3` 分界线与 `90 s on / 90 s off` GNSS outage 背景；control 区只保留 `phase3` 时间窗，直接对应 `fusion.runtime_phases[2]` 中 `enable_odo=false`, `enable_nhc=false` 的 case。
  - behavior_change:
    本轮没有改动任何滤波行为，只新增结果可视化；其中 `v` 系相关 truth 口径固定为工作安装角真值 `pitch=0.36 deg`, `yaw=0.84 deg`，避免把 `1.37 deg` 总安装角 truth 混入 vehicle-heading 诊断。
- decision:
  - 用户要求的单页 `6x` Plotly 交互网页已落地并完成离线生成。
  - 当前报告产物已经足以支撑后续对 `6x` 与 phase3 `INS-only` 对照的人工检查、汇报与截图，不需要额外重跑 solver。
- next_step:
  - 继续按主线优先级推进 `phase3 INS-only` 的 `disable nominal ba/bg decay` 最小 A/B。
  - 如需进一步服务汇报，再考虑在当前 HTML 上补 `phase3 zoom` 或 `gnss_off_01` 局部放大卡片，而不是新增一套平行脚本。

### session_id: 20260326-0125-data2-staged-g5-6x-report-restyle

- objective:
  - 按用户反馈重构 `EXP-20260326-data2-staged-g5-6x-interactive-report-r1` 的网页样式与状态图口径：改成紧凑 LaTeX/IEEE 风格，去掉 bias truth 线，并修正 `ODO/NHC` 相关状态“看起来全是固定值”的展示问题。
- scope:
  - 只修改 `scripts/analysis/build_data2_staged_g5_6x_interactive_report.py`、对应回归测试和离线 HTML 产物；不改 solver、不重跑 `EXP-20260325-data2-staged-g5-no-imu-scale-r2`。
- changed_files:
  - `scripts/analysis/build_data2_staged_g5_6x_interactive_report.py`
  - `tests/test_build_data2_staged_g5_6x_interactive_report.py`
  - `walkthrough.md`
- configs:
  - exp_id=`EXP-20260326-data2-staged-g5-6x-interactive-report-r1`
  - source_exp=`EXP-20260325-data2-staged-g5-no-imu-scale-r2`
  - input_dir=`output/data2_staged_g5_no_imu_scale_r2_20260325/`
  - output_html=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report.html`
  - output_manifest=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report_manifest.json`
  - main_case=`staged_g5_odo_nhc_noise_6x`
  - control_case=`staged_g5_odo_nhc_noise_6x_phase3_ins_only`
  - vehicle_truth=`roll=0.0 deg`, `pitch=0.36 deg`, `yaw=0.84 deg`
- commands:
  - `Get-Content walkthrough.md`
  - `python -c "import pandas as pd; ... all_states ... min/max/std/nunique ..."`
  - `pytest tests\test_build_data2_staged_g5_6x_interactive_report.py -q`
  - `python -m py_compile scripts\analysis\build_data2_staged_g5_6x_interactive_report.py`
  - `python scripts\analysis\build_data2_staged_g5_6x_interactive_report.py`
  - `Select-String -Path output\data2_staged_g5_no_imu_scale_r2_20260325\interactive\staged_g5_6x_report.html -Pattern "6x 非 PVA 22 维状态 / 误差","零偏只画估计值；ODO/NHC 相关状态改为误差曲线；中间安装角状态项已移除。","--ieee-blue","box-shadow: none;" -SimpleMatch`
  - `python -c "from pathlib import Path; ... print(text.count('mount_state')) ..."`
- artifacts:
  - report_html=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report.html`
  - report_manifest=`output/data2_staged_g5_no_imu_scale_r2_20260325/interactive/staged_g5_6x_report_manifest.json`
  - main_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv`
  - control_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv`
- metrics:
  - test_status=`RED->GREEN on restyle contract`, `pytest 8 passed`, `py_compile passed`
  - state_layout=`22 non-PVA cards after dropping mount_state_*`, `3 main PVA cards`, `3 vehicle/trajectory cards`, `3 phase3-control PVA cards`
  - style_contract=`compact 3-column layout`, `Times New Roman / Songti serif`, `gap=6 px`, `plot-card box-shadow=none`, `main color=#0B5FA5`, `control color=#8F2D2D`
  - plot_contract=`ba/bg no truth trace`, `ODO/NHC calibration states plotted as estimate-truth error`, `mount_state residual count in HTML=0`
  - source_state_probe=`mounting_state_roll std=0`, `mounting_state_pitch std=5.0469e-06 deg`, `mounting_state_yaw std=5.7764e-07 deg`, `odo_scale_state std=1.4864e-04`, `gnss_lever_x std=1.8184e-03 m`
  - html_size=`10032995 bytes`
- artifact_mtime:
  - report_html=`2026-03-26 01:25:24 local`
  - report_manifest=`2026-03-26 01:25:24 local`
- config_hash_or_mtime:
  - report_script=`scripts/analysis/build_data2_staged_g5_6x_interactive_report.py modified in current session`
  - report_test=`tests/test_build_data2_staged_g5_6x_interactive_report.py modified in current session`
  - source_main_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x/all_states_staged_g5_odo_nhc_noise_6x.csv @ 2026-03-26 00:10:45 local`
  - source_control_all_states=`output/data2_staged_g5_no_imu_scale_r2_20260325/artifacts/cases/staged_g5_odo_nhc_noise_6x_phase3_ins_only/all_states_staged_g5_odo_nhc_noise_6x_phase3_ins_only.csv @ 2026-03-26 00:11:43 local`
- dataset_time_window:
  - full_window=`528076.0~530488.9`
  - phase3_window=`528776.0~530488.9`
  - first_outage_window=`528866.0~528956.0`
- result_freshness_check:
  - 本轮没有新增 solver 运行，只消费 `EXP-20260325-data2-staged-g5-no-imu-scale-r2` 的 fresh `all_states/case_metrics/phase_metrics`。
  - 先用新增回归测试锁定“去掉 `mount_state_*`、bias 不画 truth、ODO/NHC 误差曲线、紧凑 IEEE 风格”四类契约，再最小改脚本到 `pytest 8 passed`。
  - `report_html/report_manifest` 的生成时间 `01:25:24 local` 晚于本轮脚本修改时间；输出 HTML 中关键样式标记存在，且 `mount_state` 计数为 `0`。
- observability_notes:
  - state_block_mapping:
    本轮页面显式覆盖 `ba/bg/sg/sa(9-20)`、`odo_scale(21)`、`total mounting(22-24)`、`lever_odo(25-27)`、`lever_gnss(28-30)`；中间安装角状态项不再单独展示，避免与 total mounting 重复且因近似常值造成误读。
  - schedule_effect:
    phase1/phase2/phase3 分界与 `90 s on / 90 s off` outage 底纹保持不变；control 区仍只保留 `phase3` 时间窗，直接对应 `fusion.runtime_phases[2]` 中 `enable_odo=false`, `enable_nhc=false`。
  - behavior_change:
    本轮只改可视化，不改滤波；`ODO/NHC` 校准项改成误差曲线后，`odo_scale` 与 `gnss_lever` 的小幅变化可以直接读出，bias 图则避免因 truth 线叠加干扰视觉判断。
- decision:
  - 用户反馈的两个核心问题已经修正：页面风格从 dashboard 改为紧凑白底论文风格；`ODO/NHC` 相关状态不再以“绝对值叠真值”的方式误导读者。
  - 当前报表产物已更适合汇报对照，不需要为这次样式修正新增实验号或重跑 solver。
- next_step:
  - 若继续服务汇报，可在这份新网页基础上补 `gnss_off_01` 或 `phase3` 局部 zoom 卡片，而不是另起一套脚本。
  - 主线研究仍回到 `phase3 INS-only` 的 `disable nominal ba/bg decay` 最小 A/B。

## Next Actions

1. 优先做最小 solver A/B：仅关闭 phase3 `Predict()` 中 nominal `ba/bg` 回零，重跑 `EXP-20260325-data2-staged-g5-no-imu-scale-r2::staged_g5_odo_nhc_noise_6x_phase3_ins_only`，检查首个 `gnss_off_01` 的 `RMSE3D/final_err_3d` 是否明显回落。
2. 为 phase3 `INS-only` 首窗补导出 roll/pitch 误差与 phase3 起点 `ba/bg` truth error，验证“几十米漂移主要来自 roll/pitch + accel-bias 入口误差，而不是 yaw bug”这一分支。
3. 用 `EXP-20260326-data2-staged-g5-6x-interactive-report-r1` 的紧凑版单页网页快速人工复核 `6x` 与 phase3 `INS-only` 的同窗误差形态，必要时再决定是否给 `gnss_off_01` 单独补 zoom 卡片。
4. 在完成上述 two-step pure-INS audit 后，再对 truth-init `staged_g5_odo_nhc_noise_6x` 与其 phase3 `INS-only` 对照做 propagation vs update 分解，量化 `ODO/NHC` 的净抑制量。
5. 在确认 `ODO/NHC` 整体有益之后，回到 total-zero-init `EXP-20260325-data2-staged-g5-mounting-zero-init-q-sweep-r2::staged_g5_mounting_zero_init_q_1x`，继续做 `phase2` 早窗 `528276~528330 s` 的 `dx_mounting_yaw`、`dx_bg_z`、`K` 与 residual 审计，解释 mounting 为何只学到 `~0.896 deg`；再判断剩余 `3~5 m` 级误差是否仍共享同一个 `ODO/NHC -> bg_z/attitude` 主通道。
