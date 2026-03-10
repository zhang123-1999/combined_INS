# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-10`

## 项目快照

### 长期目标

- 对比不同组合导航算法方案。
- 研究不同约束、调度与消融条件下状态量的可观性。

### 当前基线与代码事实

- 状态维度: `kStateDim = 31`
- 主要滤波模式:
  - 标准 ESKF（`fusion.fej.enable: false`）
  - InEKF 开关模式（`fusion.fej.enable: true`）
- ESKF 基线配置: `config_data2_baseline_eskf.yaml`
- InEKF 历史对照配置（保留复现实验）: `config_data2_baseline_inekf_ctrl.yaml`, `config_data2_gnss30_inekf.yaml`
- InEKF 新基线配置（当前推荐）:
  - `config_data2_baseline_inekf_best.yaml`
  - `config_data2_gnss30_inekf_best.yaml`
  - 固定开关: `ri_gnss_pos_use_p_ned_local=true`, `ri_vel_gyro_noise_mode=1`, `ri_inject_pos_inverse=true`
- data4 主口径与配置:
  - GNSS 主口径: `dataset/data4_converted/GNSS_converted.txt`（13 列，含速度）
  - data4 4 组主配置: `config_data4_baseline_eskf.yaml`, `config_data4_baseline_inekf_best.yaml`,
    `config_data4_gnss30_eskf.yaml`, `config_data4_gnss30_inekf_best.yaml`
  - data4 InEKF 最优开关（来自 `EXP-20260305-data4-inekf-switchscan-r1`）:
    `p_local=OFF, g=+1, inject=OFF`
- 当前 data2 基线组合: INS + GNSS + ODO + NHC（UWB 关闭）
- 当前 data4 基线组合: INS + GNSS + ODO + NHC（UWB 关闭）

### 31维状态拆分备忘

- `p(3), v(3), att(3), ba(3), bg(3), sg(3), sa(3), odo_scale(1), mounting(3), lever_odo(3), lever_gnss(3)`

### 融合主流程顺序（代码一致）

1. IMU 预测
2. ZUPT
3. 重力对准诊断
4. NHC
5. ODO
6. UWB
7. GNSS（含 schedule 与 post-GNSS ablation）
8. 诊断与结果记录

## 实验索引（Experiment Registry）

| exp_id | 日期 | 目的 | 配置 | 关键产物 | 关键指标 | 状态 | 新鲜度 |
|---|---|---|---|---|---|---|---|
| EXP-20260304-data2-baseline-eskf | 2026-03-04 | data2 标准 ESKF 基线 | `config_data2_baseline_eskf.yaml` | `SOL_data2_baseline_eskf.txt`; `output/logs_runcheck_baseline_eskf.log` | RMSE xyz `0.763591 0.565924 0.770052` | 参考基线 | pass |
| EXP-20260304-data2-baseline-inekf-ctrl | 2026-03-04 | data2 旧 InEKF 对照基线 | `config_data2_baseline_inekf_ctrl.yaml` | `SOL_data2_baseline_inekf_ctrl.txt`; `output/logs_runcheck_baseline_inekf_ctrl.log` | RMSE xyz `0.763 0.566 0.769`（旧） | 历史对照 | stale_or_conflict |
| EXP-20260304-inekf-ri-realization-codeoverhaul3 | 2026-03-04 | RI InEKF 初版完整落地 | 4组配置（baseline/GNSS30 × ESKF/InEKF） | `output/review/20260304-2312-inekf-ri-implementation/*.log` | baseline InEKF RMSE3D `1.692779`; GNSS30 InEKF RMSE3D `118.819261` | 历史关键节点 | pass |
| EXP-20260304-inekf-ri-gvelgyro-fix | 2026-03-04 | 在 RI 中加入 `G(kVel,gyro)=-Skew(v)C_bn` | 同上 | `output/review/20260304-2318-inekf-ri-gvelgyro-fix/*.log` | baseline InEKF RMSE3D `2.078802`; GNSS30 InEKF RMSE3D `165.932710` | 历史关键节点 | pass |
| EXP-20260305-inekf-ab-seq-r1 | 2026-03-05 | 按顺序 A/B：`p_ned_local`→`G(kVel,gyro)`→注入逆变换 | `output/review/20260305-inekf-ab-seq-r1-inekf-ab-sequence/cfg_*.yaml` | `ab_metrics.csv`; `summary.md`; `*.log` | 最优分支: `p_ned_local=ON, g=+1, inject=ON` | 当前定位依据 | pass |
| EXP-20260305-inekf-best4-regression-r1 | 2026-03-05 | 在最优分支执行 4 组标准回归并绘图 | `output/review/20260305-inekf-best4-reg-r1/cfg_*.yaml` | `metrics_summary.csv`; `run_summary.md`; `plots/`; `compare_*` | baseline RMSE3D ESKF/InEKF `1.223242/1.723701`; GNSS30 `127.618202/88.890702` | InEKF 新基线验证 | pass |
| EXP-20260305-inekf-fghi-sign-audit-r1 | 2026-03-05 | 做 `F/G/H/Inject` 全链路符号一致性审查 | N/A（代码审查） | `output/review/20260305-inekf-fghi-sign-audit-r1/inekf_fghi_sign_audit.md` | 形成 `g=+1` 与 `H/Inject` 成对项的一致性证据链 | 理论闭环审查 | pass |
| EXP-20260306-inekf-tex-code-audit-r1 | 2026-03-06 | 逐条核对 InEKF 源码实现与 `可观性分析讨论与InEKF算法.tex` 推导是否一致 | N/A（代码/文档审查） | `output/review/20260306-inekf-tex-code-audit-r1/inekf_tex_code_audit.md` | 9项检查：exact `1`，partial `3`，mismatch/undocumented `5` | 理论-实现一致性审查 | pass |
| EXP-20260306-true-iekf-refactor-r1 | 2026-03-06 | 按“代码贴理论”主线新增 `true_iekf` 分支，并与现有 hybrid RI 实现做 fresh 对比 | `config_data2_baseline_true_iekf.yaml`; `config_data2_gnss30_true_iekf.yaml`; `config_data4_baseline_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `output/review/20260306-true-iekf-refactor-r1/{metrics_summary.csv,metrics_summary.json,summary.md,*.log}`; `.research/20260306-true-iekf-refactor-r1/polished_report.md` | data2 baseline RMSE3D `1.723701→1.228326`; data2 GNSS30 `88.890702→201.158876`; data4 baseline `0.876828→0.913143`; data4 GNSS30 `73.715425→115.499686`; data2 `H_att_max 2354→2.31` | 结构改造已落地，性能结论 mixed | pass |
| EXP-20260306-true-iekf-ablation-r1 | 2026-03-06 | 在 `true_iekf` GNSS30 场景下做 `GNSS_VEL / ODO / NHC` 消融矩阵，定位 sparse-GNSS 退化主因 | `output/review/20260306-true-iekf-ablation-r1/cfg_*.yaml` | `output/review/20260306-true-iekf-ablation-r1/{ablation_metrics.csv,ablation_summary.json,summary.md,cfg_*.log}` | data2: `no_gnss_vel` 无变化，`no_odo/no_nhc` 显著恶化；data4 同趋势 | sparse-GNSS 根因定位 | pass |
| EXP-20260306-true-iekf-phase2-r1 | 2026-03-06 | 进入 phase-2：将 `ODO/NHC` 改为 true-core 直接 Jacobian，并补上 adjoint-style core reset Jacobian 后做 fresh 回归 | `config_data2_baseline_true_iekf.yaml`; `config_data2_gnss30_true_iekf.yaml`; `config_data4_baseline_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `output/review/20260306-true-iekf-phase2-r1/{metrics_summary.csv,metrics_summary.json,summary.md,*.log}`; `算法文档/可观性分析讨论与InEKF算法.pdf` | data2 baseline `1.228326→1.228557`; data2 GNSS30 `201.158876→165.591338`; data4 baseline `0.913143→0.914331`; data4 GNSS30 `115.499686→22.405382`; `GNSS_POS H_att_max` 基本不变 | phase-2 关键收益已验证 | pass |
| EXP-20260306-true-iekf-phase2-ablation-lite | 2026-03-06 | 用 phase-2 true 分支对 `data2/data4 GNSS30` 做精简 `ODO/NHC` 消融，解释为何 `data4` 明显优于 `data2` | `output/review/20260306-true-iekf-phase2-ablation-lite/cfg_*.yaml` | `output/review/20260306-true-iekf-phase2-ablation-lite/{ablation_metrics.csv,summary.md,*.log}` | data2 full/no_odo/no_nhc RMSE3D `165.591338/343.623780/695.077915`; data4 `22.405382/628.760167/717.409977` | data4/data2 差异机理定位 | pass |
| EXP-20260306-phase2b-attribution-r1 | 2026-03-06 | 对 `data2 GNSS30` 残余 gap 做 heading-source attribution：验证 `GNSS_VEL`、`bg_z/sg_z` 与 mounting 漂移谁是主因 | `output/review/20260306-phase2b-attribution/cfg_data4_no_gnss_vel.yaml`; `output/review/20260306-phase2b-postgnss-freeze/cfg_*.yaml` | `output/review/20260306-phase2b-attribution/{heading_source_summary.md,final_findings.md}`; `output/review/20260306-phase2b-postgnss-freeze/summary.md`; `output/review/20260306-phase2b-diagnostics/phase2b_diag_summary.md` | `data4 no_gnss_vel RMSE3D≈19.37m`; data2 post-GNSS `freeze_sg/freeze_mount/freeze_sg_mount` RMSE3D `163.17/145.83/142.99`; `bg_z` 积分角 data2/data4 `22.32/2.16 deg` 对应 yaw growth `28.50/3.10 deg` | heading-source 归因 | pass |
| EXP-20260306-phase2c-bg-freeze-r1 | 2026-03-06 | 在 phase-2 true 分支中新增 `post_gnss_ablation.disable_gyro_bias`，直接验证 `bg` 冻结对 `data2 GNSS30` 的因果作用，并与 `mounting` 组合对照 | `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_mount,freeze_bg,freeze_bg_mount}.yaml` | `output/review/20260306-phase2c-bg-freeze/{metrics_summary.csv,metrics_summary.json,summary.md,SOL_*.txt,*.log}` | data2 full/freeze_mount/freeze_bg/freeze_bg_mount RMSE3D `165.59/145.83/98.82/56.03`; yaw growth `-28.50/-28.66/-9.28/-9.17 deg` | post-GNSS `bg` 因果验证与缓解评估 | pass |
| EXP-20260306-gnss30-stateplots-r1 | 2026-03-06 | 对当前主配置 `data2/data4 GNSS30 true_iekf` 做 fresh 运行，并用 `plot_navresult.py` 导出全状态量变化图像，供人工审图定位问题 | `config_data2_gnss30_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `output/review/20260306-gnss30-stateplots-r1/{data2_run.log,data4_run.log}`; `output/result_data230_true_iekf/*.png`; `output/result_data430_true_iekf/*.png` | data2 RMSE xyz `88.619/80.456/114.429`; data4 RMSE xyz `9.561/14.598/14.053`; 两组各输出 `11` 张全状态图 | fresh 结果可视化 | pass |
| EXP-20260306-consistency-audit-r1 | 2026-03-06 | 对 `ESKF/InEKF × data2/data4 GNSS30` 打开 `enable_consistency_log`，检查 ODO/NHC 是否因程序分支或门控导致接受率显著不同 | `output/review/20260306-consistency-audit-r1/cfg_*.yaml` | `output/review/20260306-consistency-audit-r1/{data2_eskf,data2_true,data4_eskf,data4_true}.log` | data2: NHC `482572/482576`、ODO `482426/482577`(ESKF) vs `482427/482577`(true); data4: NHC `398134/398198`(ESKF) vs `398132/398198`(true), ODO `396812/398198` vs `396816/398198` | 约束接受率审计 | pass |
| EXP-20260306-gnsspos-rframe-fix-r1 | 2026-03-06 | 修正 `true_iekf` 下 `GNSS_POS` body-frame residual 的协方差坐标变换错误，并 fresh 复测 data2/data4 GNSS30 | `config_data2_gnss30_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `output/review/20260306-gnsspos-rframe-fix-r1/{data2_true.log,data4_true.log}` | data2 RMSE3D `165.59→164.25`; data4 `22.41→23.14`，影响存在但不足以解释 data2/data4 矛盾 | GNSS_POS 协方差框架修复 | pass |
| EXP-20260306-jacobian-audit-r1 | 2026-03-07 | Numerical Jacobian audit for post-split turning-rich `ODO/NHC` true_iekf paths on `data2/data4`, checking analytic `H` against filter-consistent finite difference | `config_data2_gnss30_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `apps/jacobian_audit_main.cpp`; `output/review/20260306-jacobian-audit-r1/{summary.md,selected_samples.csv,block_errors.csv,column_errors.csv}` | data2: NHC/ODO worst rel_fro `1.48e-4/1.44e-4`; data4: `4.12e-4/3.16e-4`; except `mount_pitch`, other focus blocks are about `0~6e-6` | Numerical Jacobian audit | pass |
| EXP-20260307-jacobian-audit-r2 | 2026-03-07 | Extend numerical Jacobian audit to `GNSS_VEL` under current `true_iekf` configs; verify whether the remaining contradiction can come from the GNSS velocity measurement model | `config_data2_gnss30_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `apps/jacobian_audit_main.cpp`; `output/review/20260307-jacobian-audit-r2/{summary.md,selected_samples.csv,block_errors.csv,column_errors.csv}` | data4 `GNSS_VEL` focus blocks (`vel/att/bg/sg/gnss_lever`) all show `rel_fro=0` at output precision on 5 turning-rich head-window samples; data2 current config has no GNSS velocity data | GNSS_VEL numerical Jacobian audit | pass |
| EXP-20260305-inekf-doubleoff-interaction-r1 | 2026-03-05 | 交互实验：`p_ned_local=OFF` + `inject_pos_inverse=OFF`（`g=+1`） | `output/review/20260305-inekf-doubleoff-interaction-r1/cfg_doubleoff_*.yaml` | `doubleoff_metrics.csv`; `summary.md`; `doubleoff_*.log`; `SOL_doubleoff_*.txt` | RMSE3D baseline/GNSS30 `615824.703508/304357.873776`（显著发散） | 交互项排错证据 | pass |
| EXP-20260305-issue001-baseline-compare-r1 | 2026-03-05 | 用 InEKF best 口径重生 baseline 对比目录并统一指标 | `output/review/20260305-issue001-baseline-compare-r1/cfg_*.yaml` | `output/compare_data2_baseline_eskf_vs_inekf_ctrl/{metrics.csv,summary.txt,summary.json,*.png}`; `output/review/20260305-issue001-baseline-compare-r1/*.log` | RMSE3D ESKF/InEKF_best `1.223243/1.723701`（与本轮日志一致） | ISSUE-001 处置证据 | pass |
| EXP-20260305-postgnss-freeze-matrix-r1 | 2026-03-05 | 在 InEKF best 分支执行 post-GNSS 冻结矩阵（`odo_scale/mounting/lever_all`） | `output/review/20260305-postgnss-freeze-matrix-r1/cfg_*.yaml` | `freeze_matrix_metrics.csv`; `summary.md`; `s*.log`; `SOL_s*.txt` | 最优 `s1m1l0` RMSE3D `61.048358`；参考 `s0m0l0` 为 `88.890702` | 可观性证据补齐 | pass |
| EXP-20260305-baseline-gmode-sensitivity-r1 | 2026-03-05 | baseline 下定位 `g=0` vs `g=+1` 时段敏感性差异 | `output/review/20260305-baseline-gmode-sensitivity-r1/cfg_gmode_{0,1}.yaml` | `gmode_sensitivity_metrics.csv`; `summary.md`; `gmode_{0,1}.log`; `SOL_gmode_{0,1}.txt` | RMSE3D `g0/g1=1.692779/1.723701`；head/tail 呈互补差异 | HYP-1 细化证据 | pass |
| EXP-20260305-postgnss-headratio-robustness-r1 | 2026-03-05 | 复验 post-GNSS 冻结策略在不同 `head_ratio` 的稳健性 | `output/review/20260305-postgnss-headratio-robustness-r1/cfg_r*_*.yaml` | `headratio_robustness_metrics.csv`; `summary.md`; `r*_*.log`; `SOL_r*_*.txt` | `freeze(scale+mounting)` 在 `0.2/0.3/0.4/0.5` 全部优于 ref（delta `-48.72~-11.80`） | HYP-3 稳健性证据 | pass |
| EXP-20260305-baseline-gmode-gnssvel-mechanism-r1 | 2026-03-05 | 验证 baseline `g=0/1` 差异是否来自 GNSS速度更新 | `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/cfg_g*_vel*.yaml` | `gnssvel_mechanism_metrics.csv`; `summary.md`; `g*_vel*.log`; `SOL_g*_vel*.txt` | `vel_on/off` 下结果完全一致：`g0/g1=1.692779/1.723701` | HYP-1 机制排除证据 | pass |
| EXP-20260305-baseline-gmode-gnsspos-coupling-r1 | 2026-03-05 | 深挖 `GNSS_POS` 相关项在 `g=0/1` 下的时段作用机制，并定义 `ISSUE-005/006` 关闭门槛。 | `output/review/20260305-baseline-gmode-gnsspos-coupling-r1/cfg_p{on,off}_g{0,1}.yaml` | `gnsspos_coupling_metrics.csv`; `timeline_windows_pon_g1_minus_g0.csv`; `summary.md`; `p*_g*.log`; `SOL_p*_g*.txt` | p_local=on: g0/g1 RMSE3D `1.692779/1.723701`; p_local=off: `4.1177455e7/1.0528657e6` | HYP-1 ?????? | pass |
| EXP-20260305-state21-30-drift-r1 | 2026-03-05 | 输出状态块 `21-30` 的 post-GNSS 时序漂移并对齐冻结结论 | `output/review/20260305-postgnss-freeze-matrix-r1/SOL_s*.txt`（分析输入） | `output/review/20260305-state21_30-drift-r1/{state21_30_postgnss_drift.csv,summary.md,*.png}` | `mounting` 冻结分支的 `mounting_pitch/yaw` 漂移约为 0，且对应更低 RMSE3D | 状态块时序证据 | pass |
| EXP-20260310-gnss-lever-fix-r1 | 2026-03-10 | data4 全程GNSS，GNSS杆臂自由估计 vs 固定真值——验证高程误差主因 | `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_{eskf_free,eskf_fixed,true_iekf_free,true_iekf_fixed}.yaml` | `SOL_data4_lever_{eskf_free,eskf_fixed,true_iekf_free,true_iekf_fixed}.txt` | ESKF free/fixed RMSE3D `0.829/0.046`m；true_iekf free/fixed `0.830/0.026`m | 高程误差归因已确认 | pass |
| EXP-20260310-gnss-vel-effect-r1 | 2026-03-10 | data4 全程GNSS，true_iekf含/不含GNSS速度对比 | `output/review/EXP-20260310-gnss-vel-effect-r1/cfg_{with_vel,no_vel}.yaml` | `SOL_data4_vel_{with,no}.txt` | with_vel/no_vel RMSE3D `0.830/0.843`m（差异极小） | GNSS速度在全程GNSS场景贡献极小 | pass |
| EXP-20260310-data4-imu-precision-r1 | 2026-03-10 | data4 三种 IMU 精度对照（GNSS30 × ESKF/true_iekf） | `output/review/EXP-20260310-data4-imu-precision-r1/cfg_data4_gnss30_{eskf,true_iekf}_imu_{adis,ans,icm}.yaml` | `output/review/EXP-20260310-data4-imu-precision-r1/SOL_data4_gnss30_{eskf,true_iekf}_{adis,ans,icm}.txt`; `output/html/representative_navigation_interactive_report.html` | RMSE3D ESKF (ADIS/ANS/ICM) `1277.644/51.374/1136.345`; true_iekf `180.320/23.144/1670.475` | IMU 精度影响量化 | pass |
| EXP-20260310-mounting-median-r1 | 2026-03-10 | data2 全程 GNSS true_iekf，产出安装角中位数基准 | `output/review/EXP-20260310-mounting-median-r1/cfg_data2_baseline_true_iekf_full.yaml` | `output/review/EXP-20260310-mounting-median-r1/SOL_data2_baseline_true_iekf_full.txt` | mounting median pitch/yaw (drop10%) `-0.242531/-2.367232` deg | 参考 | pass |
| EXP-20260310-data2-gnss-outage-cycle-r1 | 2026-03-10 | data2 ESKF 多次 GNSS outage（900s 收敛 + 300/120 周期） | `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/cfg_data2_eskf_gnss_outage_cycle.yaml` | `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/{GNSS_outage_cycle.txt,SOL_data2_eskf_gnss_outage_cycle.txt}` | RMSE xyz `2.406998 2.579152 3.979878` | 基础对准扩展 | pass |
| EXP-20260310-inekf-mechanism-r1 | 2026-03-10 | InEKF 作用机理：data4/data2 GNSS30 + post-GNSS 冻结 | `output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_{full,freeze_bg,freeze_mount}.yaml`; data2 复用 `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_bg,freeze_mount}.yaml` | `output/review/EXP-20260310-inekf-mechanism-r1/{SOL_data4_gnss30_true_*.txt,metrics.csv,summary.md}`; `output/html/representative_navigation_interactive_report.html` | data2 slope `-16.90→-5.29` deg/ks (freeze_bg); data4 slope `-2.11→-0.70` deg/ks | 机理实验 | pass |
| EXP-20260310-interactive-report-html-r4 | 2026-03-10 | 将HTML交互报告扩展为左侧固定侧边栏分页布局，新增第4-6组实验页面 | `scripts/analysis/interactive_nav_report.py`；`scripts/analysis/export_interactive_nav_report_html.py` | `output/html/representative_navigation_interactive_report.html` | 6页侧边栏HTML，第4-6页展示新实验 | 报告扩展完成 | pass |
| EXP-20260305-data2-gnsspos-mechanism-r2 | 2026-03-05 | data2 GNSS_POS 机制补全 | `config_data2_baseline_inekf_best.yaml`（派生 g0/g1 × p_on/off） | `output/review/EXP-20260305-data2-gnsspos-mechanism-r2/{gnsspos_mechanism_metrics.csv,h_att_timeline.csv,summary.md}` | p_on g0/g1 RMSE3D `1.692779/1.723701`; p_off g0/g1 `4.12e7/1.05e6`; corr `-0.091572` | 机制补证 | pass |
| EXP-20260305-data2-mountroll-observability-r1 | 2026-03-05 | mounting_roll 可观性补证 | `config_data2_gnss30_inekf_best.yaml`（冻结矩阵对照） | `output/review/EXP-20260305-data2-mountroll-observability-r1/{mountroll_observability_metrics.csv,std_mr_cmp.png,summary.md}` | `s1m1l0` post std_mr=0; RMSE3D `61.048699` vs ref `88.890428` | 状态块补证 | pass |
| EXP-20260305-data2-process-noise-regression-r1 | 2026-03-05 | RI 过程噪声映射回归 | `config_data2_baseline_inekf_best.yaml`, `config_data2_gnss30_inekf_best.yaml` | `output/review/EXP-20260305-data2-process-noise-regression-r1/process_noise_mode_regression.csv` | baseline g0 best `1.692779`; gnss30 g1 best `88.890428` | 回归证据 | pass |
| EXP-20260305-data2-postgnss-freeze-matrix-r2 | 2026-03-05 | post-GNSS 冻结矩阵重跑 | `config_data2_gnss30_inekf_best.yaml` | `output/review/EXP-20260305-data2-postgnss-freeze-matrix-r2/freeze_matrix_metrics.csv` | best `s1m1l0` RMSE3D `61.048699` (delta `-27.841730`) | 可观性证据补齐 | pass |
| EXP-20260305-data4-inekf-switchscan-r1 | 2026-03-05 | data4 InEKF 开关短扫 | `config_data4_baseline_inekf_best.yaml` | `output/review/EXP-20260305-data4-inekf-switchscan-r1/switch_scan_metrics.csv` | best `p0_g1_i0` RMSE3D `0.876829` | data4 最优开关 | pass |
| EXP-20260305-data4-main4-regression-r1 | 2026-03-05 | data4 baseline/GNSS30 × ESKF/InEKF | data4 4 组配置 | `output/review/EXP-20260305-data4-main4-regression-r1/{metrics_summary.csv,plots/}` | RMSE3D baseline ESKF/InEKF `0.929070/0.876829`; GNSS30 `51.371318/73.714930` | data4 主回归 | pass |
| EXP-20260305-data4-gnss30-freeze-matrix-r1 | 2026-03-05 | data4 GNSS30 冻结矩阵 | `config_data4_gnss30_inekf_best.yaml` | `output/review/EXP-20260305-data4-gnss30-freeze-matrix-r1/freeze_matrix_metrics.csv` | best `s0m1l1` RMSE3D `66.752679` (delta `-6.962252`) | data4 冻结证据 | pass |
| EXP-20260305-data4-gnssvel-sensitivity-r1 | 2026-03-05 | data4 GNSS 速度噪声敏感性 | data4 baseline/gnss30 InEKF | `output/review/EXP-20260305-data4-gnssvel-sensitivity-r1/sensitivity_metrics.csv` | baseline `0.876829` vs `5.178337`; gnss30 `73.714930` vs `26.479406` | 风险提示 | pass |
| EXP-20260305-data2-main4-docsync-r1 | 2026-03-05 | data2 4 组 doc 口径回归 | data2 4 组配置 | `output/review/EXP-20260305-data2-main4-docsync-r1/metrics_summary.csv` | RMSE3D baseline ESKF/InEKF `1.223243/1.723701`; GNSS30 `127.618348/88.890428` | 文档口径 | pass |
| EXP-20260307-vframe-velocity-plot-r1 | 2026-03-07 | 在 `plot_navresult.py` 中新增车体 `v系` 速度图，并对 `data2/data4 GNSS30 true_iekf` fresh 生成绘图产物，便于直观审阅 ODO/NHC 对 `v_y^v`/`v_z^v` 的约束效果 | `config_data2_gnss30_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `output/result_data230_true_iekf/02b_velocity_vehicle_v.png`; `output/result_data430_true_iekf/02b_velocity_vehicle_v.png`; `plot_navresult.py` | data2 post-GNSS `v_y^v/v_z^v` RMS `0.0848/0.0389 m/s`, P95 abs `0.243/0.078 m/s`; data4 post-GNSS `0.0649/0.0377 m/s`, P95 abs `0.179/0.063 m/s` | ODO/NHC 视觉诊断增强 | pass |
| EXP-20260307-reset-floor-r1 | 2026-03-07 | 验证每次 `ApplyTrueInEkfReset` 后追加 `ApplyCovarianceFloor()` 是否改善 `data2 GNSS30 true_iekf` | `output/review/20260307-reset-floor-r1/{cfg_ref.yaml,cfg_predict_floor.yaml,cfg_predict_and_reset_floor.yaml}` | `output/review/20260307-reset-floor-r1/{ref.log,predict_floor.log,predict_and_reset_floor.log}`; `output/review/20260307-reset-floor-r1/SOL_data2_gnss30_true_iekf_*.txt` | RMSE3D `164.245119 -> 163.513035 -> 163.205318`；after-reset floor 相对 predict-floor 额外改善 `-0.307717 m` (`-0.188%`) | reset-floor 边际收益验证 | pass |
| EXP-20260307-update-reset-consistency-r1 | 2026-03-07 | 扩展 `jacobian_audit` 到 GNSS split 互协方差与 update-reset-covariance 一致性审查，对比 `data2/data4 GNSS30 true_iekf` | `config_data2_gnss30_true_iekf.yaml`; `config_data4_gnss30_true_iekf.yaml` | `output/review/20260307-update-reset-consistency-r1/{summary.md,split_covariance.csv,reset_consistency.csv,data2_reset_*.txt,data4_reset_*.txt}` | split 点 `corr(att_z,bg_z)` data2/data4=`-0.020641/-0.084320`，`|P(att_z,bg_z)|` 约差 `8x`；reset consistency `rel_fro=max_abs=0`（两数据集） | reset/covariance 审计 | pass |
| EXP-20260308-result-docs-r1 | 2026-03-08 | 清理 `算法文档/结果文档` 中旧稿并重建 3 份正式结果文档；统一按同类型图像横向对照排版 | `output/review/EXP-20260305-data4-main4-regression-r1/cfg_{baseline_eskf,gnss30_eskf}.yaml`; `config_data4_gnss30_true_iekf.yaml`; `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`; `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_mount,freeze_bg,freeze_bg_mount}.yaml` | `算法文档/结果文档/{ESKF_data4_base_GNSS30,InEKF_ESKF,InEKF_data2_freeze}.{tex,pdf}`; `output/review/EXP-20260307-result-docs-r1/plots/*` | data4 ESKF baseline/GNSS30 RMSE3D `0.929070/51.371318`; data4 GNSS30 ESKF vs true_iekf `51.371318/23.144284`; data2 true full/freeze_bg/freeze_bg_mount `165.591644/98.817682/56.032658`; data4 no_gnss_vel `19.369316` | 结果文档成稿 | pass |
| EXP-20260308-inekf-doc-typeset-r1 | 2026-03-08 | 将 `可观性分析讨论与InEKF算法.tex` 的字体、行距、页边距与标题层级调整为与新结果文档一致的版式风格 | `算法文档/可观性分析讨论与InEKF算法.tex` | `算法文档/可观性分析讨论与InEKF算法.{tex,pdf}` | 编译通过；文档共 `22` 页；页边距调整为 `2.3/2.3/2.2/2.2 cm`，行距 `1.18`，标题/图注/列表间距统一 | 算法文档排版优化 | pass |
| EXP-20260308-inekf-doc-typeset-r2 | 2026-03-08 | 对 `可观性分析讨论与InEKF算法.tex` 做第二轮精修：字号下调到与结果文档一致，并细化标题、目录、图注与框体间距 | `算法文档/可观性分析讨论与InEKF算法.tex` | `算法文档/可观性分析讨论与InEKF算法.{tex,pdf}` | 编译通过；文档共 `17` 页；字号 `10pt`，行距 `1.14`，标题页/目录/框体间距进一步收紧 | 算法文档精修 | pass |
| EXP-20260308-inekf-doc-typeset-r3 | 2026-03-08 | 对 `可观性分析讨论与InEKF算法.tex` 做第三轮局部美化：优化目录样式、修正含数学标题的 PDF 书签、改善表格/公式与彩色框的疏密 | `算法文档/可观性分析讨论与InEKF算法.tex` | `算法文档/可观性分析讨论与InEKF算法.{tex,pdf}`; `算法文档/tmp/pdfs/inekf_doc_review/page-*.png` | 编译通过；文档共 `16` 页；目录标题与 section 字重统一；前两页已渲染 PNG 做版式核查 | 算法文档精修 | pass |
| EXP-20260309-inekf-doc-symbols-r1 | 2026-03-09 | 将 `可观性分析讨论与InEKF算法.tex` 的“符号说明”整理为前置未编号章节，并补充 31 维状态块与量测记号说明 | `算法文档/可观性分析讨论与InEKF算法.tex` | `算法文档/可观性分析讨论与InEKF算法.{tex,pdf,toc}` | 编译通过；文档共 `17` 页；目录新增“符号说明”，`引言` 恢复为第 `1` 节 | 算法文档前置说明完善 | pass |
| EXP-20260309-inekf-doc-compile-check-r1 | 2026-03-09 | 检查 `可观性分析讨论与InEKF算法.tex` 当前是否存在真实编译故障，并区分首轮正常 warning 与实际错误 | `算法文档/可观性分析讨论与InEKF算法.tex` | `算法文档/可观性分析讨论与InEKF算法.{pdf,aux,log,toc,xdv}` | `latexmk` 双轮 `xelatex + xdvipdfmx` 成功；输出 `17` 页 PDF；最终日志无未定义引用、无 VS Code 诊断错误 | 编译稳定性核查 | pass |

| EXP-20260309-interactive-report-notebook-r1 | 2026-03-09 | 为 3 组代表性实验构建 Plotly + Jupyter 交互分析笔记，统一展示轨迹、n/v 系速度误差、v系航向误差、pitch/roll 与高程误差 | `output/review/EXP-20260305-data4-main4-regression-r1/cfg_{baseline_eskf,gnss30_eskf}.yaml`; `config_data4_gnss30_true_iekf.yaml`; `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`; `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_mount,freeze_bg,freeze_bg_mount}.yaml` | `scripts/analysis/interactive_nav_report.py`; `output/jupyter-notebook/representative_navigation_interactive_report.ipynb` | RMSE3D 复核：data4 baseline/GNSS30 ESKF `0.929070/51.371318`；data4 GNSS30 true_iekf `23.144284`；data2 ESKF/full/freeze_mount/freeze_bg/freeze_bg_mount `127.618348/165.591644/145.828424/98.817682/56.032658` | 交互分析工具 | pass |

| EXP-20260310-interactive-report-html-r1 | 2026-03-10 | 修复交互 notebook 中文编码与运行入口，并导出单文件自包含 HTML 报告，便于对外分享 | 同 `EXP-20260309-interactive-report-notebook-r1` 所用 3 组配置 | `scripts/analysis/export_interactive_nav_report_html.py`; `output/html/representative_navigation_interactive_report.html`; `output/jupyter-notebook/representative_navigation_interactive_report.ipynb` | HTML 单文件 `7999646 bytes`；共享时无需额外附带数据目录；notebook 已改为 UTF-8 无 BOM | 单文件交付修复 | pass |
| EXP-20260310-interactive-report-html-r2 | 2026-03-10 | 将单文件 HTML 交互报告重构为 IEEE / Claude 风格的简约版式，并把公式标签改为离线可渲染的上下标记法 | 同 `EXP-20260309-interactive-report-notebook-r1` 所用 3 组配置 | `scripts/analysis/interactive_nav_report.py`; `scripts/analysis/export_interactive_nav_report_html.py`; `output/html/representative_navigation_interactive_report.html` | HTML 单文件 `8035988 bytes`；原始 `$...$` 标签检索为 `0`；图表与统计表均改为 `<sub>/<sup>` 离线符号标记 | 单文件交付美化 | pass |
| EXP-20260310-interactive-report-html-r3 | 2026-03-10 | 将单文件 HTML 继续收敛为“交互式 PDF”风格：固定图像范围、纯白图底、无网格、细亮线、宋体/Times 字体与更紧凑清晰的表格版式 | 同 `EXP-20260309-interactive-report-notebook-r1` 所用 3 组配置 | `scripts/analysis/interactive_nav_report.py`; `scripts/analysis/export_interactive_nav_report_html.py`; `output/html/representative_navigation_interactive_report.html` | HTML 单文件 `8026344 bytes`；图表导出配置关闭 `displayModeBar/scrollZoom/doubleClick`；坐标轴 `fixedrange=true`，注释改为“虚线表示 GNSS 关闭分界线” | 单文件交付精修 | pass |

### 历史实验压缩索引（保留 ID 可追溯）

- `EXP-20260304-data2-gnss30-postfix-gnsslever-eskf`
- `EXP-20260304-data2-gnss30-inekf`
- `EXP-20260304-data2-gnss30-eskf-nofreeze`
- `EXP-20260304-preexp-fix-regression`
- `EXP-20260304-inekf-refactor-minrisk-debug`
- `EXP-20260304-inekf-rewrite-no-fej`
- `EXP-20260304-inekf-ri-fgqd-audit-r2`
- `EXP-20260305-inekf-deep-research-audit-r1`

## 已知不一致项（Known Inconsistencies）

| issue_id | 发现日期 | 描述 | 影响文件 | 影响 | 处理状态 |
|---|---|---|---|---|---|
| ISSUE-001-metric-conflict-baseline-inekf | 2026-03-04 | 历史 `output/compare_data2_baseline_eskf_vs_inekf_ctrl/` 指标口径曾与 runcheck 冲突；已在 2026-03-05 用 InEKF best 口径重生。 | `output/compare_data2_baseline_eskf_vs_inekf_ctrl/*` | 历史结论需以重生产物为准。 | downgraded |
| ISSUE-005-baseline-inekf-ri-regression | 2026-03-04 | baseline InEKF 相对 ESKF 的差异已被机制实验约束：`p_local=on` 差异小；`p_local=off` 出现灾难性发散；与 `GNSS_VEL` 无关。 | `output/review/EXP-20260305-data2-gnsspos-mechanism-r2/*` | 影响解释闭环。 | closed_evidence |
| ISSUE-006-ri-process-noise-mapping-incomplete | 2026-03-04 | RI 过程噪声映射一致性已完成回归：baseline g0 最优，gnss30 g1 最优，无自相矛盾证据。 | `output/review/EXP-20260305-data2-process-noise-regression-r1/*` | 解释 GNSS30 与 baseline 差异。 | closed_evidence |
| ISSUE-007-inekf-tex-code-gap | 2026-03-06 | 代码-理论差距已进一步收敛：phase-2 后，`ODO/NHC` 的 Lie 核心 Jacobian 与 `reset Jacobian` 已贴近 `可观性分析讨论与InEKF算法.tex`；但 `GNSS_VEL` 与 `21-30` 欧氏扩展块仍未完全 invariant/estimate-independent，故仍不能宣称“全31维完全无泄露”。 | `算法文档/可观性分析讨论与InEKF算法.tex`; `include/core/eskf.h`; `src/core/ins_mech.cpp`; `src/core/measurement_models_uwb.cpp`; `src/core/eskf_engine.cpp`; `output/review/20260306-true-iekf-phase2-r1/summary.md` | 影响 true-IEKF 的最终口径与 data2 sparse-GNSS 残余误差。 | partially_resolved_phase2 |
| ISSUE-008-true-gnsspos-r-cov-frame-bug | 2026-03-06 | `true_iekf` 的 `GNSS_POS` 分支在 body-frame residual 下错误地把 NED 各向 sigma 当成 ECEF 协方差再经 `R_ne` 变换；已修正为 `R_body = C_bn^T R_ned C_bn`。 | `src/core/measurement_models_uwb.cpp`; `output/review/20260306-gnsspos-rframe-fix-r1/*` | 会扭曲 GNSS 头 30% 窗口的体坐标量测权重，但 fresh 复测表明它不是 data2/data4 性能反转的主因。 | fixed_with_evidence |

| ISSUE-20260310-utf8-notebook-stdin | 2026-03-10 | 通过 PowerShell 将含中文的 notebook/markdown 内容直接送入 `python -` 或非 UTF-8 安全链路时，会把中文写成 `?`，并可能导致 notebook 内容损坏；后续必须改用 UTF-8 直接文件写入（避免 stdin codepage 路径）。 | `output/jupyter-notebook/*.ipynb`; `walkthrough.md` | 会破坏中文说明、误导结果审阅，并造成 notebook 解析/运行体验变差。 | mitigated_use_utf8_direct_write |

### 已解决项（压缩）

- `ISSUE-002-manual-init-missing-truth-crash`：resolved（2026-03-04）
- `ISSUE-003-gnss-load-nonfatal`：resolved（2026-03-04）
- `ISSUE-004-inekf-fej-semantic-conflict`：resolved（2026-03-04）

## 开放假设（Open Hypotheses）

| hyp_id | 假设 | 当前证据 | 下一步验证 | 状态 |
|---|---|---|---|---|
| HYP-1 | data2 InEKF 与 ESKF 差异来自 `GNSS_POS`-RI 耦合与过程噪声映射，而非 `GNSS_VEL`。 | `p_local=on` 时 g0/g1 RMSE3D `1.692779/1.723701`；`p_local=off` 出现灾难性发散 (`4.12e7/1.05e6`)；corr(delta_err3d, delta_hatt) `-0.091572`。 | 需要进一步给出 `GNSS_POS` Jacobian 的时段因果解释（blocker：相关性较弱）。 | open(blocker: mechanistic_causality_missing) |
| HYP-3 | 外参相关状态可观性对调度与激励敏感。 | 冻结矩阵 + 状态块漂移图 + mounting_roll 证据一致：`s1m1l0` post std_mr=0，RMSE3D `61.048699` vs ref `88.890428`。 | 已闭合（后续仅跨数据集复验）。 | resolved |
| HYP-6 | RI 误差约定下，`G(kVel,gyro)` 更合理实现是 `+Skew(v_ned)C_bn`。 | data2 回归：baseline g0 最优但 gnss30 g1 最优；data4 开关短扫最佳 `p0_g1_i0`。 | 解释数据集差异的 RI 开关依赖关系。 | open(conditionally_supported) |
| HYP-7 | true IEKF 的主要收益先体现在 `GNSS_POS` Lie 核心结构修正上；若不继续重写 ODO/NHC、GNSS_VEL 和 `21-30` 扩展块，sparse-GNSS 性能未必改善。 | fresh 对比：data2 baseline RMSE3D `1.723701→1.228326` 且 `H_att_max 2354→2.309982`；data2/data4 GNSS30 退化。后续消融显示 `no_gnss_vel` 基本不改善，而 `no_odo/no_nhc` 会大幅恶化。 | 继续重推 `ODO/NHC` 与 full-state reset，而不是优先处理 `GNSS_VEL`。 | open(very_strongly_supported) |
| HYP-8 | `true_iekf` 在 sparse-GNSS 下的主要 blocker 不是 `GNSS_VEL`，而是 road-constraint 通道（`ODO/NHC`）及其与 `21-30` 扩展块的 Jacobian/reset 一致性。 | 先前消融：`no_gnss_vel` 基本无帮助、`no_odo/no_nhc` 显著恶化；phase-2 后 data2 GNSS30 `201.158876→165.591338`，data4 GNSS30 `115.499686→22.405382`，且 `GNSS_POS H_att_max` 基本不变，说明改进主要来自 `ODO/NHC + reset` 而非 `GNSS_POS` 再变化。 | 继续定位 data2 GNSS30 的残余 gap，重点检查 `GNSS_VEL` 与 `21-30` 扩展块在 GNSS 关闭后的漂移。 | open(validated_but_not_exhausted) |
| HYP-9 | data2 GNSS30 在 phase-2 后仍显著落后于 hybrid，剩余 blocker 更可能是 GNSS 关闭后 `GNSS_VEL`/`21-30` 扩展块/激励条件的 dataset-specific 组合，而不是 `GNSS_POS` 或基础 `ODO/NHC` core Jacobian。 | phase-2 后：data2 GNSS30 RMSE3D `165.591338`，仍高于 hybrid `88.890702`；但 data4 GNSS30 已降至 `22.405382`，说明 phase-2 主修复本身有效。 | 对 data2 做 phase-2b：审查 `GNSS_VEL`、`21-30` 扩展块后GNSS窗口的漂移与互协方差演化。 | open |
| HYP-10 | `data4 GNSS30` 在 phase-2 下显著优于 `data2 GNSS30` 的主要原因不是“算法偏爱 data4”，而是 `data4` 的 post-GNSS 工况更短、更慢、且更强依赖并更能受益于被 phase-2 修正后的 `ODO/NHC` 路径。 | GNSS blackout 后 truth 统计：data2 `1691.8s/20.21km/11.95mps`，data4 `1421.0s/6.66km/4.69mps`；phase-2 精简消融：data2 no_odo/full `2.08x`，no_nhc/full `4.20x`，data4 no_odo/full `28.06x`，no_nhc/full `32.02x`；phase-2b 诊断：split 时速度 data2/data4 `11.13/0.001 mps`，yaw error growth `28.5/3.1 deg`。 | 若需进一步验证，检查 data2/data4 GNSS 关闭后 heading/bias 漂移率与 `21-30` 互协方差演化。 | open(very_strongly_supported) |
| HYP-11 | `data2 GNSS30` 的主残余 blocker 已从 `GNSS_POS/ODO-NHC core Jacobian` 转移为 post-GNSS `bg_z` 主导的 heading drift；`mounting` 漂移会放大位置误差，但不是 yaw growth 的首因；`GNSS_VEL` 缺失不是主导解释。 | `data4` 关掉 GNSS 速度后仍有 `RMSE3D≈19.37m`；phase-2c 中 `freeze_bg` 令 data2 RMSE3D `165.59→98.82`、yaw growth `-28.50→-9.28 deg`；`freeze_bg_mount` 进一步到 `56.03m`，而单独 `freeze_mount` 仅 `145.83m` 且 yaw growth 近乎不变。 | 若要继续修 data2，优先决定是否把 `post_gnss_ablation.disable_gyro_bias`（可选叠加 `disable_mounting`）作为 pragmatic 默认策略，或继续追求更 theory-pure 的 `bg` 过程/更新重构。 | open(validated_by_direct_intervention) |
| HYP-12 | 在当前 phase-2 代码下，剩余 `data2` gap 不太可能来自 `ODO/NHC/GNSS_VEL` measurement Jacobian；若程序链路仍有问题，更可能是 GNSS 期间 `att-bg_z` 约束建立不足，而不是 split 点 reset covariance 数值实现本身。 | Jacobian audit r1/r2 已排除 `ODO/NHC` 与 `GNSS_VEL` measurement Jacobian；新增 reset consistency audit 显示 data2/data4 在 split 点均满足 `Gamma * P_tilde * Gamma^T == P_after_reset`（`rel_fro=max_abs=0`）；同时 split 点 `corr(att_z,bg_z)` data2/data4=`-0.020641/-0.084320`，`|P(att_z,bg_z)|` 约差 `8x`。 | 继续追踪 GNSS 有效窗口内 `P[att,bg_z]`、对应 gain 与 `dx_bg_z` 的时序增长，定位 data2 为何未建立足够的 attitude-bias 约束。 | open(narrowed_to_gnss_coupling_build_up) |

### 已否决假设（压缩）

- `HYP-2`：rejected（新实现证据推翻）
- `HYP-4`：rejected（`p_ned_local` 单因子不是主因）
- `HYP-5`：rejected（仅补单项 `G(kVel,gyro)` 不能修复）

## 会话日志（Session Log）

### 阶段摘要（压缩，保留可追溯 ID）

- 阶段A `20260304-1435 ~ 20260304-1855`：
  - 完成协作文档、基础回归核对与历史问题修复（`ISSUE-002/003` 关闭）。
  - 相关 session: `20260304-1435-agent-doc-bootstrap`, `20260304-1505-agent-md-fix-bootstrap-and-context`, `20260304-1822-pre_experiment_audit`, `20260304-1855-fix-five-issues-and-md-check`。
- 阶段B `20260304-2016 ~ 20260304-2318`：
  - InEKF 从 FEJ 依赖版重写到 RI 实现，并测试 `G(kVel,gyro)` 单点修补。
  - 关键实验: `EXP-20260304-inekf-refactor-minrisk-debug`, `EXP-20260304-inekf-rewrite-no-fej`, `EXP-20260304-inekf-ri-realization-codeoverhaul3`, `EXP-20260304-inekf-ri-gvelgyro-fix`。
- 阶段C `20260304-2358 ~ 20260305-0018`：
  - 审查 + deep-research + 顺序 A/B，锁定 best 开关 `p_local=ON, g=+1, inject=ON`。
  - 关键实验: `EXP-20260304-inekf-ri-fgqd-audit-r2`, `EXP-20260305-inekf-deep-research-audit-r1`, `EXP-20260305-inekf-ab-seq-r1`。
- 阶段D `20260305-0032 ~ 20260305-1058`：
  - 完成 best4 主回归、`walkthrough.md` 首轮瘦身、post-GNSS 冻结矩阵、`GNSS_POS/GNSS_VEL` 机制补证，以及状态块 `21-30` 漂移分析。
  - 关键结果: baseline ESKF/InEKF RMSE3D `1.223242/1.723701`；GNSS30 `127.618202/88.890702`；`s1m1l0` post-GNSS 冻结分支 RMSE3D `61.048699`。
  - 相关 session: `20260305-0032-best4-regression-and-plot`, `20260305-0042-tidy-walkthrough-and-promote-inekf-baseline`, `20260305-1010-inekf-fghi-audit-and-doubleoff`, `20260305-1018-issue001-baseline-compare-refresh`, `20260305-1029-postgnss-freeze-matrix-r1`, `20260305-1033-baseline-gmode-sensitivity-r1`, `20260305-1039-postgnss-headratio-robustness-r1`, `20260305-1048-stage-summary-writeback`, `20260305-1058-gmode-gnssvel-and-state21-30-drift`。
- 阶段E `20260305-1611 ~ 20260305-2222`：
  - 对齐 data2/data4 主口径与自动化视图，建立 data4 的 best switch、主回归与冻结矩阵结论。
  - 关键结果: data4 best switch 为 `p0_g1_i0`；baseline ESKF/InEKF RMSE3D `0.929070/0.876829`；GNSS30 `51.371318/73.714930`。
  - 相关 session: `20260305-1611-automation-worktree-overview`, `20260305-2222-inekf-eskf-data2-data4-pipeline`。
- 阶段F `20260306-1003 ~ 20260306-1902`：
  - 完成 tex-code audit、`true_iekf` 分支落地、`GNSS_VEL/ODO/NHC` 消融、phase-2/2b/2c 重构和 `bg` 冻结验证，并导出全状态图辅助机理判断。
  - 关键结果: data2 baseline `1.723701→1.228326→1.228557`；data2 GNSS30 `201.158876→165.591338`；data4 GNSS30 `115.499686→22.405382`；data2 `freeze_bg` / `freeze_bg_mount` 分别降至 `98.82 / 56.03 m`。
  - 相关 session: `20260306-1003-inekf-tex-code-audit-r1`, `20260306-1012-inekf-architecture-choice`, `20260306-1128-true-iekf-refactor-and-compare-r1`, `20260306-1349-true-iekf-gnss30-ablation-r1`, `20260306-1600-github-sync-current-work`, `20260306-1654-true-iekf-phase2-and-tex-sync`, `20260306-1714-data4-vs-data2-gnss30-gap-analysis`, `20260306-1759-phase2b-data2-gnss30-diagnostics`, `20260306-1835-phase2b-heading-source-attribution`, `20260306-1848-phase2c-bg-freeze-validation`, `20260306-1902-gnss30-stateplots-export`。
- 阶段G `20260306-2317 ~ 20260307-0040`：
  - 先整理“转弯放大真实 heading drift、但不足以单独解释 `InEKF < ESKF`”的解释链，再转向程序级排错；修复 `GNSS_POS` residual covariance frame，并用 numerical audit 排除 `ODO/NHC` 与 `GNSS_VEL` Jacobian 作为当前主 bug。
  - 关键结果: `GNSS_POS` frame 修复后 data2/data4 GNSS30 RMSE3D `165.591338→164.245119` / `22.405382→23.144670`；`ODO/NHC` worst rel_fro `1.48e-4/4.12e-4`；data4 `GNSS_VEL` focus blocks `rel_fro=0`。
  - 相关 session: `20260306-2317-turning-vs-inekf-eskf-analysis`, `20260306-2336-program-audit-and-gnsspos-rframe-fix`, `20260307-0001-jacobian-audit-r1`, `20260307-0040-jacobian-audit-gnssvel-r2`。

### session_id: 20260307-1005-compress-walkthrough-and-github-sync

- timestamp: 2026-03-07 10:05 (local)
- objective: 压缩 `walkthrough.md`、保留关键实验链路，并将当前未提交代码整理后同步到 GitHub。
- scope:
  - 将冗长 `Session Log` 压缩为阶段摘要，保留关键实验、关键结果、开放假设与下一步动作。
  - 复核并提交当前代码改动，包括 `true_iekf` reset/测量模型修正、`gyro_bias` 消融支持，以及 `jacobian_audit` 构建入口。
- changed_files:
  - `CMakeLists.txt`
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `apps/jacobian_audit_main.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_true_iekf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - post-GNSS ablation config path 仍沿用现有 `fusion.post_gnss_ablation.*` 节点
- commands:
  - `git status --short --branch`
  - `git diff --stat`
  - `cmake --build build --config Release --target eskf_fusion jacobian_audit`
  - `git add CMakeLists.txt include/app/fusion.h src/app/config.cpp src/app/pipeline_fusion.cpp src/core/eskf_engine.cpp src/core/measurement_models_uwb.cpp apps/jacobian_audit_main.cpp walkthrough.md`
  - `git commit -m "Add jacobian audit and true IEKF reset updates"`
  - `git push origin main`
- artifacts:
  - `build/Release/eskf_fusion.exe`
  - `build/Release/jacobian_audit.exe`
  - `walkthrough.md`
- metrics:
  - 本会话不新增导航性能指标；目标是文档压缩、构建确认与代码同步。
- artifact_mtime:
  - `walkthrough.md`: 2026-03-07
  - `build/Release/eskf_fusion.exe`: fresh build in this session
  - `build/Release/jacobian_audit.exe`: fresh build in this session
- config_hash_or_mtime:
  - 本会话未改主配置文件；代码改动作用于现有 `true_iekf` 与 ablation 配置路径。
- dataset_time_window:
  - N/A（本会话不新跑解算）
- result_freshness_check:
  - pass（本会话已完成文档压缩与本地 fresh build；远端同步在同一会话内执行）
- observability_notes:
  - 新增 `disable_gyro_bias`，直接对应状态块 `12-14`，用于 `fusion.ablation.*` 与 `fusion.post_gnss_ablation.*` 的后续冻结/可观性实验。
  - `ApplyTrueInEkfReset` 补充 `kPos/kVel <- kAtt` 交叉项后，更贴近 true Lie-core reset 线性化；但本会话尚未给出新的性能结论。
  - `jacobian_audit` 已成为后续审查 `measurement/update/reset/covariance` 一致性的基础入口。
- decision:
  - 文档层面改为“实验索引保留全量、会话日志压缩为阶段摘要 + 当前会话”的维护策略，减少噪声并保留追溯性。
  - 当前代码可以先做构建确认并同步到远端；后续实验应围绕 `ApplyTrueInEkfReset` 与 covariance / cross-covariance 一致性继续推进。
- next_step:
  - 完成本次 build / commit / push 后，优先执行 reset/covariance consistency audit，并用 `disable_gyro_bias` 做 data2 post-GNSS 定位性验证。

### session_id: 20260307-1023-add-vframe-velocity-plots

- timestamp: 2026-03-07 10:23 (local)
- objective: 为 `plot_navresult.py` 增加车体 `v系` 速度图，并 fresh 生成 `data2/data4 GNSS30 true_iekf` 的可视化产物，方便研究 ODO/NHC 效果。
- scope:
  - 修改绘图脚本，按当前代码口径从 `NED -> body -> vehicle(v)` 复原 `v系` 速度。
  - 基于匹配配置文件复现 `mounting_base_rpy` 的 legacy 逻辑，并对两组 GNSS30 true_iekf 结果重新出图。
- changed_files:
  - `plot_navresult.py`
  - `output/result_data230_true_iekf/02b_velocity_vehicle_v.png`
  - `output/result_data430_true_iekf/02b_velocity_vehicle_v.png`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_true_iekf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
- commands:
  - `python -m py_compile plot_navresult.py`
  - `python plot_navresult.py SOL_data2_gnss30_true_iekf.txt config_data2_gnss30_true_iekf.yaml`
  - `python plot_navresult.py SOL_data4_gnss30_true_iekf.txt config_data4_gnss30_true_iekf.yaml`
  - `python` 统计 `v_y^v / v_z^v` 的 RMS 与 P95(abs)
- artifacts:
  - `plot_navresult.py`
  - `output/result_data230_true_iekf/02b_velocity_vehicle_v.png`
  - `output/result_data430_true_iekf/02b_velocity_vehicle_v.png`
- metrics:
  - data2 full/post-GNSS `v_y^v` RMS `0.0793/0.0848 m/s`, `v_z^v` RMS `0.0388/0.0389 m/s`
  - data2 full/post-GNSS `|v_y^v|` P95 `0.2217/0.2434 m/s`, `|v_z^v|` P95 `0.0774/0.0781 m/s`
  - data4 full/post-GNSS `v_y^v` RMS `0.0630/0.0649 m/s`, `v_z^v` RMS `0.0361/0.0377 m/s`
  - data4 full/post-GNSS `|v_y^v|` P95 `0.1617/0.1791 m/s`, `|v_z^v|` P95 `0.0597/0.0635 m/s`
- artifact_mtime:
  - `output/result_data230_true_iekf/02b_velocity_vehicle_v.png`: `2026-03-07 10:21:40`
  - `output/result_data430_true_iekf/02b_velocity_vehicle_v.png`: `2026-03-07 10:22:33`
- config_hash_or_mtime:
  - 本会话未改两份主配置；绘图时显式传入对应 `config_data2_gnss30_true_iekf.yaml` / `config_data4_gnss30_true_iekf.yaml`
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（`02b_velocity_vehicle_v.png` 与统计结果均在本会话 fresh 生成）
- observability_notes:
  - 新图直接对应 ODO/NHC 关注的速度块：`v_x^v` 对应 ODO 前向速度约束，`v_y^v / v_z^v` 对应 NHC 侧向/垂向近零约束。
  - 传感器窗口保持 GNSS30 主配置：`fusion.gnss_schedule.head_ratio=0.3`，GNSS 在前 30% 后关闭，ODO/NHC 全程持续工作。
  - 本会话不改变状态估计，只增强诊断可视化；可观性关联重点仍是姿态/安装角/偏置对 `v_y^v / v_z^v` 的间接影响，其中扩展状态块主要涉及 `odo_scale(21)`, `mounting(22-24)`。
- decision:
  - `plot_navresult.py` 现在可在新格式 SOL 文件上自动生成 `v系` 速度图，并能基于匹配配置复原 `C_b_v` 的 base mounting 逻辑。
  - 两组 GNSS30 true_iekf 图像已 fresh 落盘，可直接用于后续 ODO/NHC 机理审阅；当前仅新增诊断证据，不更新算法优劣结论。
- next_step:
  - 结合 `02b_velocity_vehicle_v.png` 与现有 `03_attitude.png/08_mount_angles.png`，优先人工审阅 GNSS split 前后 `v_y^v / v_z^v` 与 heading/mounting 漂移的联动时窗。

### session_id: 20260307-1101-reset-covariance-audit-r1

- timestamp: 2026-03-07 11:01 (local)
- objective: 完成 3 个 reset/covariance 相关实验：after-reset covariance floor 效果、GNSS split 点 `P[att,bg_z]` 对比、以及 update-reset-covariance 一致性审计。
- scope:
  - 在 true IEKF 路径中加入 after-reset covariance floor 开关，并对 `data2 GNSS30` 做三组对照运行。
  - 扩展 `RunFusion` / `jacobian_audit` 捕获 GNSS 有效窗口末尾互协方差与 true IEKF reset 前后 covariance 快照。
  - 用独立 `Gamma * P_tilde * Gamma^T` 重算 split 附近 GNSS 更新后的 reset covariance，并与实际 `P_` 做数值对比。
- changed_files:
  - `include/core/eskf.h`
  - `src/core/eskf_engine.cpp`
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `apps/jacobian_audit_main.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_true_iekf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/20260307-reset-floor-r1/cfg_ref.yaml`
  - `output/review/20260307-reset-floor-r1/cfg_predict_floor.yaml`
  - `output/review/20260307-reset-floor-r1/cfg_predict_and_reset_floor.yaml`
- commands:
  - `cmake --build build --config Release --target eskf_fusion jacobian_audit`
  - `build/Release/eskf_fusion.exe output/review/20260307-reset-floor-r1/cfg_ref.yaml`
  - `build/Release/eskf_fusion.exe output/review/20260307-reset-floor-r1/cfg_predict_floor.yaml`
  - `build/Release/eskf_fusion.exe output/review/20260307-reset-floor-r1/cfg_predict_and_reset_floor.yaml`
  - `build/Release/jacobian_audit.exe --outdir output/review/20260307-update-reset-consistency-r1`
  - `Get-Content output/review/20260307-reset-floor-r1/*.log | Select-String 'RMSE'`
- artifacts:
  - `output/review/20260307-reset-floor-r1/ref.log`
  - `output/review/20260307-reset-floor-r1/predict_floor.log`
  - `output/review/20260307-reset-floor-r1/predict_and_reset_floor.log`
  - `output/review/20260307-reset-floor-r1/SOL_data2_gnss30_true_iekf_ref.txt`
  - `output/review/20260307-reset-floor-r1/SOL_data2_gnss30_true_iekf_predict_floor.txt`
  - `output/review/20260307-reset-floor-r1/SOL_data2_gnss30_true_iekf_predict_and_reset_floor.txt`
  - `output/review/20260307-update-reset-consistency-r1/summary.md`
  - `output/review/20260307-update-reset-consistency-r1/split_covariance.csv`
  - `output/review/20260307-update-reset-consistency-r1/reset_consistency.csv`
  - `output/review/20260307-update-reset-consistency-r1/data2_reset_p_expected.txt`
  - `output/review/20260307-update-reset-consistency-r1/data4_reset_p_expected.txt`
- metrics:
  - experiment(1) data2 GNSS30 RMSE xyz / RMSE3D:
    - ref: `87.886 80.638 112.916` / `164.245119`
    - predict_floor: `81.700 98.323 101.952` / `163.513035`
    - predict_and_reset_floor: `81.386 98.487 101.551` / `163.205318`
    - after-reset floor 相对 ref 改善 `-1.039801 m` (`-0.633%`)，相对 predict-floor 额外改善 `-0.307717 m` (`-0.188%`)
  - experiment(2) GNSS split 点 `P[att,bg_z]`:
    - data2: `P_att_bgz=[-2e-9,-0,-1.3e-8]`, `corr_att_bgz=[-0.002308,-0.000052,-0.020641]`
    - data4: `P_att_bgz=[1e-9,6e-9,-1.04e-7]`, `corr_att_bgz=[0.000909,0.009055,-0.084320]`
    - `|P(att_z,bg_z)|` data4/data2 `≈8.0x`，`|corr(att_z,bg_z)|` data4/data2 `≈4.09x`
  - experiment(3) reset consistency:
    - data2: `expected_norm=actual_norm=0.009996280`, `diff_norm=0`, `rel_fro=0`, `max_abs=0`
    - data4: `expected_norm=actual_norm=0.005459867`, `diff_norm=0`, `rel_fro=0`, `max_abs=0`
- artifact_mtime:
  - `output/review/20260307-reset-floor-r1/ref.log`: `2026-03-07 10:55:45`
  - `output/review/20260307-reset-floor-r1/predict_floor.log`: `2026-03-07 10:56:28`
  - `output/review/20260307-reset-floor-r1/predict_and_reset_floor.log`: `2026-03-07 10:57:11`
  - `output/review/20260307-update-reset-consistency-r1/summary.md`: `2026-03-07 11:00:31`
  - `output/review/20260307-update-reset-consistency-r1/split_covariance.csv`: `2026-03-07 11:00:31`
  - `output/review/20260307-update-reset-consistency-r1/reset_consistency.csv`: `2026-03-07 11:00:31`
- config_hash_or_mtime:
  - 主配置仍为 `config_data2_gnss30_true_iekf.yaml` / `config_data4_gnss30_true_iekf.yaml`；本会话新增开关经 `output/review/20260307-reset-floor-r1/cfg_*.yaml` 明确固化。
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（reset-floor 三组日志与 reset-consistency 审计产物均在本会话 fresh 生成）
- observability_notes:
  - 关注状态块为 `att(6-8)` 与 `bg(12-14)`，其中本次直接审阅 `bg_z` 与 attitude 三轴的互协方差建立情况。
  - 传感器窗口采用 `fusion.gnss_schedule.head_ratio=0.3`：GNSS 仅在前 30% 有效，split 附近仍有 ODO/NHC 持续更新，适合检查 GNSS 是否在关闭前为 `att-bg` 建立足够约束。
  - after-reset covariance floor 仅带来小幅收益，说明它更像数值稳健性补丁，而非当前 `data2` 大误差的主因。
  - split 点 reset covariance 数值完全一致，说明 `ApplyTrueInEkfReset` 的 `Gamma` 传播本身未在该关键 GNSS 更新步引入额外数值误差；更可疑的是 GNSS 有效窗口内 `att-bg_z` 约束积累不足。
- decision:
  - 实验(1) 结论：`ApplyCovarianceFloor()` 在每次 `ApplyTrueInEkfReset` 后调用确有改善，但幅度很小，不足以解释或修复 `data2` 的主残余 gap。
  - 实验(2) 结论：data2 在 GNSS split 前的 `att_z-bg_z` 互协方差与相关系数显著弱于 data4，支持“GNSS 期间没有建立足够 attitude-bias 约束”的方向。
  - 实验(3) 结论：至少在已知 split 附近 GNSS 更新步，update-reset-covariance 数值链路是自洽的，后续排查重点应从“reset 算错”转向“约束为何没被建立/积累起来”。
- next_step:
  - 对 `data2/data4` 记录 GNSS 有效窗口内 `P[att,bg_z]`、相关 Kalman gain 与 `dx_bg_z` 的完整时序，定位 data2 约束增长为何偏弱。

### session_id: 20260308-0039-result-docs-r1

- timestamp: 2026-03-08 00:39 (local)
- objective: 清理 `算法文档/结果文档` 中旧结果文档，并基于当前主口径实验产物重建 3 份正式 LaTeX 结果报告。
- scope:
  - 删除 `算法文档/结果文档` 中旧 `.tex/.pdf/.aux/.log/.out/.toc/.compile.log` 等未调试稿及副产物。
  - 为 8 个实验组补齐/整理 11 张标准图（`01/02/02b/03/04/05/06/07/08/09/10`），统一归档到 `output/review/EXP-20260307-result-docs-r1/plots/`。
  - 新建并编译 `ESKF_data4_base_GNSS30.tex`、`InEKF_ESKF.tex`、`InEKF_data2_freeze.tex`。
- changed_files:
  - `算法文档/结果文档/*`（旧结果文档及编译副产物清理）
  - `算法文档/结果文档/ESKF_data4_base_GNSS30.tex`
  - `算法文档/结果文档/InEKF_ESKF.tex`
  - `算法文档/结果文档/InEKF_data2_freeze.tex`
  - `算法文档/结果文档/ESKF_data4_base_GNSS30.pdf`
  - `算法文档/结果文档/InEKF_ESKF.pdf`
  - `算法文档/结果文档/InEKF_data2_freeze.pdf`
  - `output/review/EXP-20260307-result-docs-r1/plots/*`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_baseline_eskf.yaml`
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_full.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_freeze_mount.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg_mount.yaml`
- commands:
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_baseline_eskf_doc.txt output/review/EXP-20260305-data4-main4-regression-r1/cfg_baseline_eskf.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_eskf_doc.txt output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data4_gnss30_true_iekf_doc.txt config_data4_gnss30_true_iekf.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_eskf_ref.txt output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_true_full_doc.txt output/review/20260306-phase2c-bg-freeze/cfg_full.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_true_freeze_mount_doc.txt output/review/20260306-phase2c-bg-freeze/cfg_freeze_mount.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_true_freeze_bg_doc.txt output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg.yaml`
  - `python plot_navresult.py output/review/EXP-20260307-result-docs-r1/sol_links/SOL_data2_gnss30_true_freeze_bg_mount_doc.txt output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg_mount.yaml`
  - `python` 基于 `scripts.analysis.run_research_pipeline` 计算 current `data4_true`、`data2_eskf_ref`、`data2_true_full` 的精确 RMSE/P95/tail/final 指标
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error ESKF_data4_base_GNSS30.tex`
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error InEKF_ESKF.tex`
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error InEKF_data2_freeze.tex`
- artifacts:
  - `算法文档/结果文档/ESKF_data4_base_GNSS30.tex`
  - `算法文档/结果文档/InEKF_ESKF.tex`
  - `算法文档/结果文档/InEKF_data2_freeze.tex`
  - `算法文档/结果文档/ESKF_data4_base_GNSS30.pdf`
  - `算法文档/结果文档/InEKF_ESKF.pdf`
  - `算法文档/结果文档/InEKF_data2_freeze.pdf`
  - `output/review/EXP-20260307-result-docs-r1/plots/data4_baseline_eskf/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data4_gnss30_eskf/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data4_gnss30_true_iekf/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_eskf_ref/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_full/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_freeze_mount/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_freeze_bg/*`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_freeze_bg_mount/*`
- metrics:
  - `ESKF_data4_base_GNSS30.tex`: data4 baseline/GNSS30 ESKF RMSE3D `0.929070/51.371318`，P95 `1.083456/111.887450`，tail70 `1.011770/61.398719`。
  - `InEKF_ESKF.tex`: data4 GNSS30 ESKF vs current true_iekf RMSE3D `51.371318/23.144284`，P95 `111.887450/55.562597`，tail70 `61.398719/27.658891`，末时刻 3D `29.004030/55.640252`。
  - `InEKF_data2_freeze.tex`: data2 true full/freeze_mount/freeze_bg/freeze_bg_mount RMSE3D `165.591644/145.828424/98.817682/56.032658`；yaw growth `-28.50/-28.66/-9.28/-9.17 deg`。
  - 归因对照：data2 GNSS30 ESKF 参照 `127.618348 m`；data4 true full `23.144284 m`；data4 true no_gnss_vel `19.369316 m`。
- artifact_mtime:
  - `output/review/EXP-20260307-result-docs-r1/plots/data4_baseline_eskf`: `2026-03-08 00:26:33`
  - `output/review/EXP-20260307-result-docs-r1/plots/data4_gnss30_eskf`: `2026-03-08 00:30:21`
  - `output/review/EXP-20260307-result-docs-r1/plots/data4_gnss30_true_iekf`: `2026-03-08 00:30:32`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_eskf_ref`: `2026-03-08 00:30:44`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_full`: `2026-03-08 00:30:56`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_freeze_mount`: `2026-03-08 00:31:08`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_freeze_bg`: `2026-03-08 00:31:21`
  - `output/review/EXP-20260307-result-docs-r1/plots/data2_gnss30_true_freeze_bg_mount`: `2026-03-08 00:31:33`
  - `算法文档/结果文档/ESKF_data4_base_GNSS30.pdf`: `2026-03-08 00:38:13`
  - `算法文档/结果文档/InEKF_ESKF.pdf`: `2026-03-08 00:38:30`
  - `算法文档/结果文档/InEKF_data2_freeze.pdf`: `2026-03-08 00:39:08`
- config_hash_or_mtime:
  - 本会话未修改任何主配置；所有配置均为复用既有实验配置并显式写入报告正文。
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（8 组图像在本会话 fresh 整理，3 份 `.tex/.pdf` 在本会话 fresh 生成并编译通过）
- observability_notes:
  - `InEKF_data2_freeze.tex` 直接映射了 post-GNSS ablation 的状态块：`disable_gyro_bias -> 12-14`，`disable_mounting -> 22-24`。其中冻结 `12-14` 显著改善，冻结 `22-24` 为次级改善，二者叠加进一步改善。
  - 传感器窗口保持 `fusion.gnss_schedule.head_ratio=0.3`：GNSS 仅在前 30% 时段有效，ODO/NHC 全程持续，因此图像与指标均在“GNSS 关闭后约束如何维持解算”的可观性口径下解读。
  - data4 true no_gnss_vel 仍可达到 `19.369316 m`，对“data2 弱于 data4 是否主要由缺少 GNSS 速度造成”给出负证据，说明主导差异更接近 `bg_z` 驱动的 heading drift 与 mounting 放大通道。
- decision:
  - 已完成旧结果目录清理，并按统一格式重建 3 份正式结果文档，满足“实验设置、量化指标、图像对比、实验结论”四段结构要求。
  - 新报告统一按“同类型图像横向对照”排版，而不是按实验组整组堆叠，且通过拆分 figure 保证了 5 组冻结对照场景中的图像可读性。
  - 文档层面的核心结论已稳定：data4 GNSS30 下 true_iekf 整体优于 ESKF；data2 GNSS30 的主残余问题是 post-GNSS `bg_z` 主导的 heading drift，mounting 漂移属于次级放大器，GNSS 速度缺失不是主导解释。
- next_step:
  - 人工审阅 3 份 PDF 的版式与图像可读性，若需要再针对个别图号做局部放大或分页微调。

### session_id: 20260308-0054-inekf-doc-typeset-r1

- timestamp: 2026-03-08 00:54 (local)
- objective: 参照新结果文档的字体、行距和页边距风格，优化 `可观性分析讨论与InEKF算法.tex` 的整体版式，使算法文档更统一、更易读。
- scope:
  - 仅调整导言区与全局排版参数，不改正文推导内容与章节结构。
  - 统一页边距、行距、标题层级间距、图注与列表间距，并保留原有 `tcolorbox`、定理和公式环境。
  - fresh 编译 `算法文档/可观性分析讨论与InEKF算法.pdf` 验证版式兼容性。
- changed_files:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `walkthrough.md`
- configs:
  - N/A（文档排版任务）
- commands:
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error 可观性分析讨论与InEKF算法.tex`
- artifacts:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
- metrics:
  - 文档编译通过，输出 `22` 页 PDF。
  - 版式参数更新：页边距 `left/right=2.3 cm`, `top/bottom=2.2 cm`；行距 `1.18`；表格 `arraystretch=1.15`。
- artifact_mtime:
  - `算法文档/可观性分析讨论与InEKF算法.tex`: `2026-03-08 00:53:32`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`: `2026-03-08 00:53:52`
- config_hash_or_mtime:
  - N/A（本会话不涉及实验配置）
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（`.tex` 已修改且 `.pdf` 在本会话 fresh 编译生成）
- observability_notes:
  - 本会话仅优化文档排版，不改变任何可观性结论、状态块分析或理论推导。
  - 文档内容仍围绕 `InEKF`、`FEJ-EKF` 与可观性结构误差展开，未引入新的实验口径。
- decision:
  - `可观性分析讨论与InEKF算法.tex` 已对齐到新结果文档的视觉风格：页边距更紧凑、行距更舒展、标题层级更清晰、图注和列表间距更统一。
  - 采用 `ctexart` 后编译稳定，说明该算法文档可以与结果文档保持统一 LaTeX 风格继续维护。
- next_step:
  - 人工检查 `算法文档/可观性分析讨论与InEKF算法.pdf` 的目录页、彩色框和长公式分页观感；如需进一步精修，再针对个别章节做局部标题/框体微调。

### session_id: 20260308-0059-inekf-doc-typeset-r2

- timestamp: 2026-03-08 00:59 (local)
- objective: 在首轮排版统一基础上做第二轮精修，重点解决“字号略大于结果文档”的问题，并进一步优化标题页、目录、图注和框体疏密。
- scope:
  - 将 `可观性分析讨论与InEKF算法.tex` 的字号由 `12pt` 调整为与结果文档一致的 `10pt`。
  - 进一步收紧正文行距与段间距，优化标题页垂直留白，并统一图注 skip 与 `tcolorbox` 前后间距。
  - fresh 编译 PDF，检查页数变化与 warning 情况。
- changed_files:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `walkthrough.md`
- configs:
  - N/A（文档排版任务）
- commands:
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error 可观性分析讨论与InEKF算法.tex`
- artifacts:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
- metrics:
  - 文档编译通过，输出 `17` 页 PDF。
  - 版式参数更新：字号 `10pt`；行距 `1.14`；段间距 `0.25em`；图注 skip 统一为 `6pt`。
- artifact_mtime:
  - `算法文档/可观性分析讨论与InEKF算法.tex`: `2026-03-08 00:58:45`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`: `2026-03-08 00:58:58`
- config_hash_or_mtime:
  - N/A（本会话不涉及实验配置）
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（`.tex` 已在本会话再次修改且 `.pdf` 已 fresh 编译生成）
- observability_notes:
  - 本会话继续只做排版精修，不改变任何可观性分析、状态块解释或 InEKF 理论结论。
  - 缩小字号后页数由 `22` 收敛到 `17`，更接近结果文档的紧凑风格，同时保留长公式与彩色框的可读性。
- decision:
  - 算法文档当前已与结果文档的视觉风格更接近：字号一致、页边距一致、整体更紧凑，用户此前感受到的“字号偏大”问题已直接处理。
  - 现阶段不继续做更强的结构改写，只保留必要的轻量 warning（主要来自章节标题中的数学符号 PDF string 处理）。
- next_step:
  - 如仍需更精细美化，可下一轮针对目录样式、表格宽度或特定长公式分页做局部定制。

### session_id: 20260308-0102-inekf-doc-typeset-r3

- timestamp: 2026-03-08 01:02 (local)
- objective: 继续对 `可观性分析讨论与InEKF算法.tex` 做局部美化，重点优化目录页、标题页、彩色框、表格/公式疏密以及带数学符号的目录书签。
- scope:
  - 引入 `tocloft` / `etoolbox` 做目录样式与表格字号微调。
  - 为含数学符号的 `section/subsection/subsubsection` 标题加入 `\texorpdfstring`，消除 PDF 书签中的数学警告并改善书签可读性。
  - 调整 `\contentsname`、标题页副标题、`tcolorbox` 前后间距、长公式分页和表格字号。
  - 用 `pdftoppm` 将前两页渲染为 PNG，做标题页与目录页的版式核查。
- changed_files:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `算法文档/tmp/pdfs/inekf_doc_review/page-01.png`
  - `算法文档/tmp/pdfs/inekf_doc_review/page-02.png`
  - `walkthrough.md`
- configs:
  - N/A（文档排版任务）
- commands:
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error 可观性分析讨论与InEKF算法.tex`
  - `pdftoppm -f 1 -l 2 -png 可观性分析讨论与InEKF算法.pdf tmp/pdfs/inekf_doc_review/page`
- artifacts:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `算法文档/tmp/pdfs/inekf_doc_review/page-01.png`
  - `算法文档/tmp/pdfs/inekf_doc_review/page-02.png`
- metrics:
  - 文档编译通过，输出 `16` 页 PDF。
  - 继续保留 `10pt` 正文字号，目录与标题页进一步收紧；前两页 PNG 渲染成功，可用于后续人工复核。
- artifact_mtime:
  - `算法文档/可观性分析讨论与InEKF算法.tex`: `2026-03-08 01:01:48` 
  - `算法文档/可观性分析讨论与InEKF算法.pdf`: `2026-03-08 01:02:36`
  - `算法文档/tmp/pdfs/inekf_doc_review/page-01.png`: `2026-03-08 01:02:36`
  - `算法文档/tmp/pdfs/inekf_doc_review/page-02.png`: `2026-03-08 01:02:36`
- config_hash_or_mtime:
  - N/A（本会话不涉及实验配置）
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（`.tex`、`.pdf` 以及前两页 PNG 均在本会话 fresh 生成）
- observability_notes:
  - 本会话仍仅涉及文档排版，不修改可观性分析结论、状态块解释或 InEKF 理论推导。
  - 通过 `\texorpdfstring` 处理 `SE_2(3)`、`F`、`A_c` 等标题内数学符号后，目录书签的可读性更好，避免了之前的 PDF string 警告。
- decision:
  - 算法文档已完成第三轮局部美化，当前风格已与结果文档基本统一，并且标题页、目录页、表格和盒子环境的观感更精致。
  - 后续若还需要继续优化，宜转向“个别页面的定制修饰”，而不是再做全局参数变更。
- next_step:
  - 若用户继续要求美化，优先针对个别表格宽度、特定长公式分页或个别彩色框的页内断点做点状修饰。

### session_id: 20260309-1049-inekf-doc-symbols-r1

- timestamp: 2026-03-09 10:55 (local)
- objective: 在 `可观性分析讨论与InEKF算法.tex` 正文前补足“符号说明”前置章节，并让其不占用正文编号。
- scope:
  - 将原有 `\section{符号说明}` 改为前置未编号章节，保留目录入口但不影响正文节号。
  - 补充 IMU 偏置/比例因子、ZUPT/NHC/ODO/UWB/GNSS 量测记号，以及 31 维状态块的 1-based 对应表。
  - fresh 编译 `.pdf` 并核对 `.toc`，确认“符号说明”进入目录且 `引言` 恢复为第 `1` 节。
- changed_files:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `算法文档/可观性分析讨论与InEKF算法.toc`
  - `walkthrough.md`
- configs:
  - N/A（文档结构任务）
- commands:
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error 可观性分析讨论与InEKF算法.tex`
- artifacts:
  - `算法文档/可观性分析讨论与InEKF算法.tex`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `算法文档/可观性分析讨论与InEKF算法.toc`
- metrics:
  - 文档编译通过，输出 `17` 页 PDF。
  - 目录包含 `符号说明`，且 `引言` 在目录中恢复为第 `1` 节。
- artifact_mtime:
  - `算法文档/可观性分析讨论与InEKF算法.tex`: `2026-03-09 10:54:35`
  - `算法文档/可观性分析讨论与InEKF算法.pdf`: `2026-03-09 10:54:52`
  - `算法文档/可观性分析讨论与InEKF算法.toc`: `2026-03-09 10:54:50`
- config_hash_or_mtime:
  - N/A（本会话不涉及实验配置）
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（`.tex`、`.pdf`、`.toc` 均在本会话 fresh 生成）
- observability_notes:
  - 新增的 `31维状态块对应` 明确了 Lie 核心、`b_a/b_g`、`s_g/s_a`、`odo_scale`、`mounting`、`lever_odo`、`lever_gnss` 的符号与维度，对应本文后续的状态块可观性讨论。
  - 本会话仅整理前置说明，不改变任何可观性结论、实验指标或 InEKF 理论推导。
- decision:
  - `符号说明` 现已成为真正的前置章节：目录可见，但不会把正文 `引言` 挤成第 `2` 节。
  - 文档前部现在同时覆盖数学记号、导航状态、31 维状态块与量测缩写，适合作为后续阅读入口。
- next_step:
  - 人工检查 `算法文档/可观性分析讨论与InEKF算法.pdf` 的目录页和新增符号说明页，确认表格换页、标题层级和页内留白观感是否还需微调。

### session_id: 20260309-1107-inekf-doc-compile-check

- timestamp: 2026-03-09 11:07 (local)
- objective: 检查 `可观性分析讨论与InEKF算法.tex` 是否存在实际编译问题，并确认当前文档在本机 TeX 环境下的可复现性。
- scope:
  - 直接在 `算法文档/` 目录运行 `latexmk -xelatex -interaction=nonstopmode -halt-on-error` 复现编译过程。
  - 检查首轮日志中的交叉引用 warning 是否在后续轮次自动消失。
  - 复核 VS Code 对 `.tex` 文件的即时诊断结果，确认是否存在语法级错误。
- changed_files:
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `算法文档/可观性分析讨论与InEKF算法.aux`
  - `算法文档/可观性分析讨论与InEKF算法.log`
  - `算法文档/可观性分析讨论与InEKF算法.toc`
  - `算法文档/可观性分析讨论与InEKF算法.xdv`
  - `walkthrough.md`
- configs:
  - N/A（文档编译检查任务）
- commands:
  - `latexmk -xelatex -interaction=nonstopmode -halt-on-error 可观性分析讨论与InEKF算法.tex`
  - VS Code Problems check on `算法文档/可观性分析讨论与InEKF算法.tex`
- artifacts:
  - `算法文档/可观性分析讨论与InEKF算法.pdf`
  - `算法文档/可观性分析讨论与InEKF算法.log`
  - `算法文档/可观性分析讨论与InEKF算法.toc`
- metrics:
  - `latexmk` 成功完成 `2` 轮 `xelatex` 与 `1` 轮 `xdvipdfmx`。
  - 最终输出 `17` 页 PDF，`340221 bytes`。
  - 首轮出现的 `undefined references` 属于目录与交叉引用尚未写回时的正常现象；第二轮后已消失。
  - VS Code 对该 `.tex` 文件返回 `No errors found`。
- artifact_mtime:
  - `算法文档/可观性分析讨论与InEKF算法.pdf`: fresh build in this session
  - `算法文档/可观性分析讨论与InEKF算法.log`: fresh build in this session
  - `算法文档/可观性分析讨论与InEKF算法.toc`: fresh build in this session
- config_hash_or_mtime:
  - N/A（本会话不涉及实验配置）
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（`.pdf/.log/.toc` 均由本会话 fresh 编译生成）
- observability_notes:
  - 本会话仅验证文档编译链路，不修改任何可观性分析、状态块解释或 InEKF 理论推导。
  - 首轮 `eq:*` 与 `sec:*` 的未定义引用 warning 来自 LaTeX 多轮编译机制，而非标签丢失；对应标签在源文件中均存在并于第二轮正确解析。
- decision:
  - 当前 `可观性分析讨论与InEKF算法.tex` 在本机 TeX Live 2025 + `latexmk/xelatex` 环境下可正常编译，不存在阻断性编译错误。
  - 若用户在编辑器里看到“未定义引用”，应优先区分是单轮编译造成的瞬时 warning，还是最终 `latexmk` 结果中的真实残留问题。
- next_step:
  - 如后续仍报告“编译失败”，优先收集用户使用的具体编译命令或 LaTeX Workshop 配置，判断是否是单轮 `xelatex`、错误引擎或工作目录不一致导致的环境型问题。

### session_id: 20260309-2328-interactive-report-notebook-r1

- timestamp: 2026-03-09 23:28 (local)
- objective: 基于 Plotly 与 Jupyter 为 3 组代表性实验构建交互式分析笔记，统一查看轨迹、n系速度误差、v系速度误差、v系航向误差、pitch/roll 误差与高程误差。
- scope:
  - 新增可复用分析模块 `scripts/analysis/interactive_nav_report.py`，负责读取既有 `SOL/truth/config`、对齐误差并生成 Plotly 图表与统计表。
  - 新增 notebook `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`，仅保留加载、构建报告、渲染结果三步，避免把分析逻辑散落在 `.ipynb` 中。
  - 全程复用既有求解产物，不新增 solver run；对 data2 无表头 `POS_converted.txt` 兼容处理后完成 smoke test。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_baseline_eskf.yaml`
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_full.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_freeze_mount.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_freeze_bg_mount.yaml`
- commands:
  - `python "C:/Users/不存在的骑士/.codex/skills/jupyter-notebook/scripts/new_notebook.py" --kind experiment --title "组合导航代表性实验交互分析" --out "D:/因子图算法科研训练/UWB/output/jupyter-notebook/representative_navigation_interactive_report.ipynb"`
  - `python -m py_compile scripts/analysis/interactive_nav_report.py`
  - `python scripts/analysis/interactive_nav_report.py --target-points 1200`
- artifacts:
  - `scripts/analysis/interactive_nav_report.py`
  - `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`
- metrics:
  - 第一组（data4 baseline ESKF vs GNSS30 ESKF）复核 RMSE3D `0.929070/51.371318`，GNSS cutoff `609.0 s`。
  - 第二组（data4 GNSS30 ESKF vs true_iekf）复核 RMSE3D `51.371318/23.144284`，GNSS cutoff `609.0 s`。
  - 第三组（data2 ESKF 参考 + full/freeze_mount/freeze_bg/freeze_bg_mount）复核 RMSE3D `127.618348/165.591644/145.828424/98.817682/56.032658`，GNSS cutoff `725.042 s`。
  - notebook 对每个误差通道统一输出 `mean / rmse / p95_abs / max_abs / final` 五类统计值，覆盖 `n系速度误差`、`v系速度误差`、`v系航向误差`、`pitch/roll` 与 `高程误差`。
- artifact_mtime:
  - `scripts/analysis/interactive_nav_report.py`: `2026-03-09 23:27:17`
  - `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`: `2026-03-09 23:24:41`
- config_hash_or_mtime:
  - 本会话未修改任何实验配置；全部复用既有报告口径配置，仅新增交互式读取与可视化层。
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（交互分析模块在本会话 fresh 生成，并通过 smoke test 复核出与正式报告一致的 RMSE3D；未新增 solver 输出，但所有 notebook 输入路径均显式记录且可复现）
- observability_notes:
  - 调度效应继续按代码真实顺序解读：IMU 预测、ZUPT、重力对准诊断、NHC、ODO、UWB、GNSS、诊断记录；其中本 notebook 重点观察 GNSS 头窗口结束后的误差增长。
  - 第一组只改 `fusion.gnss_schedule.*`：data4 从全程 GNSS 切到前 30% GNSS 后，n系/v系速度误差、v系航向误差与高程误差在后段整体劣化，用于建立 schedule 退化基线。
  - 第二组保持同一 GNSS30 窗口，对比 ESKF 与 true_iekf；在相同 ODO/NHC 活跃窗口下，true_iekf 的整体误差曲线与统计值更优，可直接用于 post-GNSS 可观性保持能力对照。
  - 第三组直接映射 post-GNSS ablation 状态块：`disable_gyro_bias -> 12-14`，`disable_mounting -> 22-24`。`freeze_bg` 对 v系航向误差与速度误差改善显著，`freeze_mount` 仅次级改善，`freeze_bg_mount` 最优，符合“`bg_z` 主导 heading drift、mounting 为放大器”的既有判断。
- decision:
  - 已完成一个“薄 notebook + 厚模块”的交互分析方案：notebook 负责展示，分析逻辑集中在 Python 模块，后续扩展新的实验组时只需补 `REPORT_SECTIONS`。
  - 新模块已兼容 data2 无表头真值文件，并通过 CLI smoke test 成功复核 3 组代表性实验的核心 RMSE 指标，说明该交互笔记可直接作为后续问题分析入口。
- next_step:
  - 在 `output/jupyter-notebook/representative_navigation_interactive_report.ipynb` 中实际执行并人工审图，优先检查 GNSS cutoff 后 `v系` 速度误差、v系航向误差与 `pitch/roll`、高程误差是否呈同步放大，再决定是否扩展到更多冻结矩阵或新的调度窗口。

### session_id: 20260310-0014-interactive-report-html-fix

- timestamp: 2026-03-10 00:14 (local)
- objective: 修复交互 notebook 的中文编码与运行入口问题，并额外导出一个可直接分享的单文件 HTML 交互报告。
- scope:
  - 将 `output/jupyter-notebook/representative_navigation_interactive_report.ipynb` 重写为 UTF-8 无 BOM 的原始 JSON，去掉损坏的中文与残留执行输出。
  - 新增 `scripts/analysis/export_interactive_nav_report_html.py`，将既有 Plotly 图和统计表打包为单个自包含 HTML 文件。
  - 复核 notebook 中文内容、HTML 导出成功路径，并记录“避免通过 `python -` 写中文 notebook”的维护提醒。
- changed_files:
  - `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `walkthrough.md`
- configs:
  - 同 `EXP-20260309-interactive-report-notebook-r1`：`output/review/EXP-20260305-data4-main4-regression-r1/cfg_{baseline_eskf,gnss30_eskf}.yaml`; `config_data4_gnss30_true_iekf.yaml`; `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`; `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_mount,freeze_bg,freeze_bg_mount}.yaml`
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
  - JSON / UTF-8 validation on `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`
- artifacts:
  - `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - 单文件 HTML 大小 `7999646 bytes`。
  - HTML 继续复用 3 组实验的既有核心指标：data4 baseline/GNSS30 ESKF RMSE3D `0.929070/51.371318`，data4 GNSS30 true_iekf `23.144284`，data2 ESKF/full/freeze_mount/freeze_bg/freeze_bg_mount `127.618348/165.591644/145.828424/98.817682/56.032658`。
  - notebook 中文首单元已能按 UTF-8 正确读回，且 JSON 解析通过。
- artifact_mtime:
  - `output/jupyter-notebook/representative_navigation_interactive_report.ipynb`: `2026-03-10 00:13:11`
  - `scripts/analysis/export_interactive_nav_report_html.py`: `2026-03-10 00:11:58`
  - `output/html/representative_navigation_interactive_report.html`: `2026-03-10 00:14:03`
- config_hash_or_mtime:
  - 本会话未修改实验配置，仍完全复用既有 3 组代表性实验输入。
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（HTML 在本会话 fresh 生成；notebook 已改为 UTF-8 无 BOM 并重新校验；无需新增 solver 运行即可复用现有结果）
- observability_notes:
  - 本会话不改变任何实验结论，只修复结果呈现与分享方式。
  - 单文件 HTML 仍按同一 3 组实验口径展示 post-GNSS 误差增长、`freeze_bg` 对航向相关误差的改善，以及 data4/data2 在调度窗口后的差异。
- decision:
  - 当前对外分享的首选产物应改为 `output/html/representative_navigation_interactive_report.html`；它是单文件、已嵌入 Plotly 与数据，不需要再附带原始结果目录。
  - notebook 保留为本地分析入口，但其内容写入今后必须避开 `python -` 这类可能走本地 codepage 的链路，统一改用 UTF-8 直接文件写入。
- next_step:
  - 先人工打开 `output/html/representative_navigation_interactive_report.html` 检查浏览器中的交互体验与排版，再决定是否继续对 HTML 做章节精简或摘要强化。

### session_id: 20260310-0039-interactive-report-html-restyle

- timestamp: 2026-03-10 00:39 (local)
- objective: 将单文件 HTML 交互报告重构为更接近 IEEE / Claude 的简约风格，并修复离线场景下公式标签显示为 LaTeX 原码的问题。
- scope:
  - 调整 `scripts/analysis/interactive_nav_report.py` 的 Plotly 主题：细线、低饱和蓝绿灰配色、浅色网格、衬线标题与无外链公式依赖的标签记法。
  - 重写 `scripts/analysis/export_interactive_nav_report_html.py` 的页面布局与 CSS，改为浅背景、细边框、目录导航、统一 plot/table card 的单文件 HTML 设计。
  - fresh 导出 `output/html/representative_navigation_interactive_report.html`，并用文本检索确认已无原始 `$...$` 标签残留。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - 同 `EXP-20260309-interactive-report-notebook-r1`：`output/review/EXP-20260305-data4-main4-regression-r1/cfg_{baseline_eskf,gnss30_eskf}.yaml`; `config_data4_gnss30_true_iekf.yaml`; `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`; `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_mount,freeze_bg,freeze_bg_mount}.yaml`
- commands:
  - `python -m py_compile scripts/analysis/interactive_nav_report.py`
  - `python -m py_compile scripts/analysis/export_interactive_nav_report_html.py`
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
  - `rg -n "北向速度误差 \$v_n\$|前向速度误差 \$v_x\^v\$|横向速度误差 \$v_y\^v\$|垂向速度误差 \$v_z\^v\$" output/html/representative_navigation_interactive_report.html`
  - `rg -n "<sub>n</sub>|<sub>x</sub><sup>v</sup>|<sub>y</sub><sup>v</sup>|<sub>z</sub><sup>v</sup>" output/html/representative_navigation_interactive_report.html`
- artifacts:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - 单文件 HTML 大小 `8035988 bytes`。
  - 原始 LaTeX 标签检索命中 `0`；`<sub>/<sup>` 离线符号标记已出现在图表 payload 与统计表中。
  - 3 组实验的底层结果口径保持不变：data4 baseline/GNSS30 ESKF RMSE3D `0.929070/51.371318`，data4 GNSS30 true_iekf `23.144284`，data2 ESKF/full/freeze_mount/freeze_bg/freeze_bg_mount `127.618348/165.591644/145.828424/98.817682/56.032658`。
- artifact_mtime:
  - `scripts/analysis/interactive_nav_report.py`: `2026-03-10 00:37:08`
  - `scripts/analysis/export_interactive_nav_report_html.py`: `2026-03-10 00:36:05`
  - `output/html/representative_navigation_interactive_report.html`: `2026-03-10 00:38:18`
- config_hash_or_mtime:
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_baseline_eskf.yaml`: `2026-03-05 17:09:26`
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`: `2026-03-05 17:10:45`
  - `config_data4_gnss30_true_iekf.yaml`: `2026-03-06 11:33:39`
  - `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`: `2026-03-05 00:32:21`
  - `output/review/20260306-phase2c-bg-freeze/cfg_full.yaml`: `2026-03-06 18:42:24`
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（HTML 与两份脚本均在本会话 fresh 生成/编译；仅重构呈现层，不改 solver 结果）
- observability_notes:
  - 本会话不改变任何可观性结论或状态块解释，只改善结果展示与离线分享体验。
  - 为保持单文件离线可用性，符号从 MathJax/LaTeX 记法改为 HTML 上下标；这不会改变 `freeze_bg`、`freeze_mount` 与 GNSS 调度相关的误差统计和图形趋势。
  - 编码维护提醒继续成立：涉及中文 notebook/markdown/html 写入时，必须使用 UTF-8 直接文件写入，避免 `python -` / stdin codepage 路径。
- decision:
  - `output/html/representative_navigation_interactive_report.html` 现在既是单文件自包含交付物，也是当前更适合对外分享的最终形态。
  - 页面风格已从偏装饰型 warm hero 收敛到浅底、细线、低饱和配色的技术报告风格；后续如需再调，只建议做小幅排版微调而非改回依赖外链资源的方案。
- next_step:
  - 在真实浏览器里人工审阅一次新版 HTML 的视觉细节、滚动节奏和交互流畅度；若无明显问题，后续继续把它作为默认分享产物。

### session_id: 20260310-0105-interactive-report-html-pdf-style

- timestamp: 2026-03-10 01:05 (local)
- objective: 继续收敛单文件 HTML 报告，使其更接近“可交互 PDF”的阅读体验：图像固定、纯白、无网格、细线、宋体/Times 字体，以及更清晰且不过度铺满页面的表格。
- scope:
  - 调整 `scripts/analysis/interactive_nav_report.py` 的 Plotly 图形主题，使坐标轴固定、背景纯白、网格移除、线条更细更亮，并统一到 `Times New Roman + 宋体` 字体栈。
  - 重写 `scripts/analysis/export_interactive_nav_report_html.py` 的页面与表格样式，去掉偏网页化的装饰，收敛到窄版心、白底、细边框、静态 modebar 与更居中的 plot/table 容器。
  - fresh 导出 `output/html/representative_navigation_interactive_report.html`，确保新的版式与图表交互限制进入最终单文件产物。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - 同 `EXP-20260309-interactive-report-notebook-r1`：`output/review/EXP-20260305-data4-main4-regression-r1/cfg_{baseline_eskf,gnss30_eskf}.yaml`; `config_data4_gnss30_true_iekf.yaml`; `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`; `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_mount,freeze_bg,freeze_bg_mount}.yaml`
- commands:
  - `python -m py_compile scripts/analysis/interactive_nav_report.py`
  - `python -m py_compile scripts/analysis/export_interactive_nav_report_html.py`
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
  - `rg -n "displayModeBar|scrollZoom|doubleClick|fixedrange|showgrid" output/html/representative_navigation_interactive_report.html`
  - `rg -n "虚线表示 GNSS 关闭分界线" output/html/representative_navigation_interactive_report.html`
- artifacts:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - 单文件 HTML 大小 `8026344 bytes`。
  - 图表样式改为白底、无网格、细线，交互限制改为保留 hover、关闭缩放/拖拽入口；HTML 注释已改为“虚线表示 GNSS 关闭分界线”。
  - 3 组实验的底层结果口径保持不变：data4 baseline/GNSS30 ESKF RMSE3D `0.929070/51.371318`，data4 GNSS30 true_iekf `23.144284`，data2 ESKF/full/freeze_mount/freeze_bg/freeze_bg_mount `127.618348/165.591644/145.828424/98.817682/56.032658`。
- artifact_mtime:
  - `scripts/analysis/interactive_nav_report.py`: `2026-03-10 01:01:59`
  - `scripts/analysis/export_interactive_nav_report_html.py`: `2026-03-10 01:03:05`
  - `output/html/representative_navigation_interactive_report.html`: `2026-03-10 01:04:07`
- config_hash_or_mtime:
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_baseline_eskf.yaml`: `2026-03-05 17:09:26`
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`: `2026-03-05 17:10:45`
  - `config_data4_gnss30_true_iekf.yaml`: `2026-03-06 11:33:39`
  - `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`: `2026-03-05 00:32:21`
  - `output/review/20260306-phase2c-bg-freeze/cfg_full.yaml`: `2026-03-06 18:42:24`
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（两份脚本与 HTML 均在本会话 fresh 编译/导出；仅修改呈现层，不改底层解算结果）
- observability_notes:
  - 本会话不改变任何实验结论，只继续收敛结果呈现形式。
  - GNSS 分界提示从“阴影区域”改为“虚线分界线”，更贴合当前纯白无网格图形样式；这不影响任何调度或冻结实验的误差统计解释。
  - 字体和样式调整仅作用于可视化层；`freeze_bg`、`freeze_mount`、GNSS 调度窗口等相关的机理判断保持不变。
- decision:
  - 单文件 HTML 的默认风格由“轻网页化展示”进一步收敛为“交互式 PDF”阅读风格，后续优先在这个方向上做小范围微调。
  - 当前版本已经更适合对外阅读与打印观感：页面更白、更窄、更静，图表交互被限制为信息查看而不是图形编辑。
- next_step:
  - 人工在浏览器里审阅这一版字体回退效果和表格密度；如果仍需收敛，下一轮优先微调标题层级、表格列宽和 section 间距，而不是再扩大页面装饰元素。

### session_id: 20260310-gnss-lever-vel-experiments-html-sidebar

- timestamp: 2026-03-10 (续前次会话)
- objective: ①验证 data4 全程 GNSS 下高程误差来源假设（GNSS 杆臂偏差）；②研究 GNSS 速度有无对解算的影响（true_iekf）；③将 HTML 交互报告扩展为左侧固定侧边栏分页布局，新增第4-6组实验。
- scope:
  - 计算 GNSS 天线相对 IMU 的体系杆臂（`scripts/analysis/compute_gnss_lever_arm.py`）。
  - 创建 6 个实验 config（杆臂×4 + 速度×2），修复 P0_diag 维度（31项）。
  - 编译并运行全部 6 组实验，记录 RMSE。
  - 修改 `interactive_nav_report.py`：新增 group4/5/6 颜色 + 3 个 SectionSpec。
  - 重构 `export_interactive_nav_report_html.py`：左侧 240px 固定侧边栏 + 右侧分页内容，JS 控制显示/隐藏。
  - 重新生成 HTML 报告。
- changed_files:
  - `scripts/analysis/compute_gnss_lever_arm.py` (新建)
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_free.yaml` (新建)
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_fixed.yaml` (新建)
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_free.yaml` (新建)
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_fixed.yaml` (新建)
  - `output/review/EXP-20260310-gnss-vel-effect-r1/cfg_with_vel.yaml` (新建)
  - `output/review/EXP-20260310-gnss-vel-effect-r1/cfg_no_vel.yaml` (新建)
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_free.yaml`
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_fixed.yaml`
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_free.yaml`
  - `output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_fixed.yaml`
  - `output/review/EXP-20260310-gnss-vel-effect-r1/cfg_with_vel.yaml`
  - `output/review/EXP-20260310-gnss-vel-effect-r1/cfg_no_vel.yaml`
- commands:
  - `cmake --build build --config Release --target eskf_fusion`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_free.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-gnss-lever-fix-r1/cfg_eskf_fixed.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_free.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-gnss-lever-fix-r1/cfg_true_iekf_fixed.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-gnss-vel-effect-r1/cfg_with_vel.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-gnss-vel-effect-r1/cfg_no_vel.yaml`
  - `python scripts/analysis/export_interactive_nav_report_html.py`
- artifacts:
  - `output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_eskf_free.txt`
  - `output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_eskf_fixed.txt`
  - `output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_true_iekf_free.txt`
  - `output/review/EXP-20260310-gnss-lever-fix-r1/SOL_data4_lever_true_iekf_fixed.txt`
  - `output/review/EXP-20260310-gnss-vel-effect-r1/SOL_data4_vel_with.txt`
  - `output/review/EXP-20260310-gnss-vel-effect-r1/SOL_data4_vel_no.txt`
  - `output/html/representative_navigation_interactive_report.html`
  - `scripts/analysis/compute_gnss_lever_arm.py`
- metrics:
  - 计算得杆臂（体系）: `[1.3028, 0.2932, -1.0797]` m，标准差 `[0.0046, 0.0059, 0.0119]` m（1839样本）
  - ESKF 自由杆臂 RMSE xyz: `0.567121 0.503075 0.537088` m
  - ESKF 固定杆臂 RMSE xyz: `0.038719 0.018372 0.017577` m（精度提升约15-30倍）
  - true_iekf 自由杆臂 RMSE xyz: `0.569762 0.504036 0.536038` m
  - true_iekf 固定杆臂 RMSE xyz: `0.015617 0.013005 0.016479` m（精度提升约35倍）
  - true_iekf 含速度 RMSE xyz: `0.569762 0.504036 0.536038` m
  - true_iekf 无速度 RMSE xyz: `0.580967 0.534876 0.535481` m（速度影响极小）
- artifact_mtime:
  - SOL 文件在本会话 fresh 生成（2026-03-10）
  - HTML 在本会话 fresh 生成
- config_hash_or_mtime:
  - 本会话新建 6 个 config 文件（2026-03-10）
- dataset_time_window:
  - data4: `275309.0` 到 `277300.0`（全程GNSS，无schedule截断）
- result_freshness_check:
  - pass（SOL 文件与 HTML 均在本会话 fresh 生成）
- observability_notes:
  - 实验一（GNSS杆臂）直接映射状态块 `gnss_lever_arm` (indices 28-30)：自由估计时误差 ~0.5m，固定真值后误差降至 ~0.02m，证实 **高程误差主因是 GNSS 杆臂未正确初始化**。
  - 实验二（GNSS速度）：在全程GNSS + ODO + NHC 组合下，含/不含 GNSS 速度的 RMSE 差异 < 0.04m，说明全程GNSS场景下速度约束边际贡献极小，ODO/NHC 已足够约束速度。
  - 杆臂固定实验使用 `fusion.ablation.disable_gnss_lever_arm: true`（全程生效，区别于 `post_gnss_ablation`）。
  - ESKF 和 true_iekf 在固定杆臂后均大幅改善，说明问题来自观测模型（杆臂补偿），而非滤波框架本身。
- decision:
  - **高程误差主因已确认**：GNSS天线相对IMU的杆臂（约 `[1.3, 0.3, -1.1]` m）若不正确给定，直接在每次GNSS更新中引入约1m量级的系统性偏差。
  - GNSS速度的边际贡献在全程GNSS场景下可以忽略；在稀疏GNSS（GNSS30）场景中的贡献仍需参考旧结论（HYP-8）。
  - HTML报告已扩展为6页侧边栏格式，第4-6页展示新实验。
- next_step:
  - 人工打开 `output/html/representative_navigation_interactive_report.html`，检查侧边栏导航与第4-6页的渲染效果。
  - 考虑在 `config_data4_baseline_eskf.yaml` 等主配置中将 `gnss_lever_arm0` 更新为计算值，以避免后续实验继续受杆臂偏差影响。

### session_id: 20260310-1545-mounting-median-report

- timestamp: 2026-03-10 15:45 (local)
- objective: 用全 GNSS true_iekf 安装角中位数作为 v 系真值基准，重算并导出交互报告。
- scope:
  - 新跑 data2 全程 GNSS true_iekf，输出 SOL 供中位数计算。
  - 在 `interactive_nav_report.py` 中加入基于 data2/data4 全 GNSS SOL 的安装角中位数覆盖逻辑（丢前 10%，roll=0）。
  - 重新导出 `output/html/representative_navigation_interactive_report.html`。
- changed_files:
  - `output/review/EXP-20260310-mounting-median-r1/cfg_data2_baseline_true_iekf_full.yaml`
  - `output/review/EXP-20260310-mounting-median-r1/SOL_data2_baseline_true_iekf_full.txt`
  - `scripts/analysis/interactive_nav_report.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260310-mounting-median-r1/cfg_data2_baseline_true_iekf_full.yaml`
  - `output/review/EXP-20260310-gnss-vel-effect-r1/cfg_with_vel.yaml`（data4 全 GNSS true_iekf）
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-mounting-median-r1/cfg_data2_baseline_true_iekf_full.yaml`
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/review/EXP-20260310-mounting-median-r1/SOL_data2_baseline_true_iekf_full.txt`
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - data2 安装角中位数（丢前10%，roll=0）：pitch `-0.242531` deg, yaw `-2.367232` deg（n=482576）
  - data4 安装角中位数（丢前10%，roll=0）：pitch `-0.373802` deg, yaw `1.358720` deg（n=398198）
- observability_notes:
  - 本会话仅更新 v 系真值基准计算，不涉及可观性结论更新。
- decision:
  - v 系真值与估计统一使用“全 GNSS true_iekf 安装角中位数”作为 base mounting，丢弃前10%样本，roll 固定 0。
- next_step:
  - 人工检查新版 HTML 中 v 系速度/航向误差的整体趋势是否符合预期。

### session_id: 20260310-1558-html-mounting-note

- timestamp: 2026-03-10 15:58 (local)
- objective: 在 HTML 报告中加入 v 系真值基准安装角说明，便于审阅与对照。
- scope:
  - 为 HTML 导出增加“v系真值基准安装角说明”卡片，显示每个 case 的安装角来源与 rpy 数值。
  - 重新导出 `output/html/representative_navigation_interactive_report.html`。
- changed_files:
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - N/A
- observability_notes:
  - 本会话仅补充 HTML 说明，不涉及可观性结论更新。
- decision:
  - 报告中明确标注 v 系真值基准安装角的来源与数值，减少误读。
- next_step:
  - 人工确认各实验页面的安装角说明与期望一致。

### session_id: 20260310-1643-inekf-mechanism-r1

- timestamp: 2026-03-10 16:43 (local)
- objective: 新增 InEKF 作用机理实验（第七/第八组），补齐 post-GNSS 漂移归因证据，并更新 HTML 报告。
- scope:
  - 运行 data4 GNSS30 true_iekf 的 freeze_mount 组，补齐三组对照。
  - 新建 `inekf_mechanism_report.py`，生成 post-GNSS 指标表与机理摘要。
  - 扩展交互报告新增第七/第八组并重新导出 HTML。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/inekf_mechanism_report.py`
  - `output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_freeze_mount.txt`
  - `output/review/EXP-20260310-inekf-mechanism-r1/metrics.csv`
  - `output/review/EXP-20260310-inekf-mechanism-r1/summary.md`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_{full,freeze_bg,freeze_mount}.yaml`
  - `output/review/20260306-phase2c-bg-freeze/cfg_{full,freeze_bg,freeze_mount}.yaml`（data2 复用）
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`
  - `output/review/20260305-inekf-best4-reg-r1/cfg_gnss30_eskf.yaml`
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-inekf-mechanism-r1/cfg_data4_gnss30_true_freeze_mount.yaml`
  - `python scripts/analysis/inekf_mechanism_report.py`
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/review/EXP-20260310-inekf-mechanism-r1/SOL_data4_gnss30_true_freeze_mount.txt`
  - `output/review/EXP-20260310-inekf-mechanism-r1/metrics.csv`
  - `output/review/EXP-20260310-inekf-mechanism-r1/summary.md`
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - data2 post-GNSS：full slope `-16.90` deg/ks → freeze_bg `-5.29` deg/ks；freeze_mount `-17.23` deg/ks；bg_z 漂移 `2.293e-04` rad/s。
  - data4 post-GNSS：full slope `-2.11` deg/ks → freeze_bg `-0.70` deg/ks；freeze_mount `-2.11` deg/ks；bg_z 漂移 `2.645e-05` rad/s。
  - data4 ESKF vs true_iekf：去趋势 RMS `4.562 → 0.342` deg；data2 ESKF vs true_iekf：`3.286 → 1.204` deg。
- observability_notes:
  - post-GNSS 冻结作用于状态块 `bg` (12-14) 与 `mounting` (22-24)，调度窗口为 `gnss_schedule.head_ratio=0.3`。
  - 冻结 `bg` 显著降低航向漂移斜率与残差 RMS，冻结 `mounting` 仅归零 mounting 漂移但对漂移斜率影响极小。
  - 说明漂移主要由 `bg_z` 累积导致，mounting 更像次级放大通道。
- decision:
  - InEKF 在 GNSS 关闭后呈现更明显的“漂移主导”形态，且 data2 的漂移强度显著大于 data4；
    bg_z 冻结是最有效的漂移抑制手段，mounting 冻结主要消除安装角通道但不足以改变漂移斜率。
- next_step:
  - 人工打开 HTML 报告确认第七/第八组曲线与指标表渲染正常，并把 summary.md 结论合并进最终实验叙述。

### session_id: 20260310-1654-interactive-report-drop-group3

- timestamp: 2026-03-10 16:54 (local)
- objective: 删除交互报告中与后续实验重复的第三组页面，避免内容冗余。
- scope:
  - 移除 `interactive_nav_report.py` 中第三组 SectionSpec 与配色定义。
  - 重新导出 `representative_navigation_interactive_report.html`。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - N/A
- observability_notes:
  - 本次调整仅为报告结构整理，不改变任何实验结论或可观性解释。
- decision:
  - 第三组页面已删除，报告内容从第七/第八组获取对应机理对照信息。
- next_step:
  - 人工确认新 HTML 目录中已无第三组页面，且编号顺序符合预期。

### session_id: 20260310-1710-interactive-report-add-analysis

- timestamp: 2026-03-10 17:10 (local)
- objective: 为每个实验页面补充“实验分析与总结”段落，解释主要现象与对比结论。
- scope:
  - 在 HTML 导出脚本中新增分析卡片，按实验类型生成简短分析要点。
  - 重新导出交互报告 HTML。
- changed_files:
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - N/A（分析内容引用页面已有 RMSE3D / tail70 指标）
- observability_notes:
  - 仅补充文字分析，不改变任何实验结论或可观性解释。
- decision:
  - 每个实验页末尾增加“实验分析与总结”卡片，便于读者快速理解现象与对比结果。
- next_step:
  - 人工确认每一页分析卡片是否与曲线和指标一致，若需更细的机理解释可再扩展。

### session_id: 20260310-1826-interactive-report-renumber

- timestamp: 2026-03-10 18:26 (local)
- objective: 调整实验编号与侧边栏分组，使基准对比为 2 组、问题定向为 5 组。
- scope:
  - 更新 Section 标题编号（第 3~7 组）并调整侧边栏分组范围。
  - 重新导出交互报告 HTML。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`（耗时较长，进程超时，但 HTML 已写入）
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - N/A
- observability_notes:
  - 仅调整报告编号与侧边栏，不改变实验结论。
- decision:
  - 第 1-2 组归为基准对比实验，第 3-7 组归为问题定向实验，编号与侧边栏一致。
- next_step:
  - 人工确认 HTML 目录编号与页面标题一致。

### session_id: 20260310-1854-data2-gnss-outage-cycle

- timestamp: 2026-03-10 18:54 (local)
- objective: 新增 data2 ESKF 多次 GNSS outage 基础对准实验，并加入交互报告基准组。
- scope:
  - 生成周期性 GNSS outage 文件（900s on + 300/120 周期）。
  - 新建配置并运行 ESKF 解算。
  - 交互报告新增第三组基础对准实验并重排编号。
- changed_files:
  - `scripts/analysis/filter_gnss_outage.py`
  - `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/GNSS_outage_cycle.txt`
  - `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/cfg_data2_eskf_gnss_outage_cycle.yaml`
  - `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/SOL_data2_eskf_gnss_outage_cycle.txt`
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`（派生）
  - `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/cfg_data2_eskf_gnss_outage_cycle.yaml`
- commands:
  - `python scripts/analysis/filter_gnss_outage.py`
  - `build/Release/eskf_fusion.exe --config output/review/EXP-20260310-data2-gnss-outage-cycle-r1/cfg_data2_eskf_gnss_outage_cycle.yaml`
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/GNSS_outage_cycle.txt`
  - `output/review/EXP-20260310-data2-gnss-outage-cycle-r1/SOL_data2_eskf_gnss_outage_cycle.txt`
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - GNSS rows kept: `2053/2413 (0.851)`.
  - ESKF RMSE xyz: `2.406998 2.579152 3.979878` m.
- observability_notes:
  - 本实验通过周期性 GNSS 失锁测试基础对准稳定性，不涉及额外可观性消融。
- decision:
  - 新增第三组基础对准实验，基准对比实验页数增至 3，问题定向实验页数为 5。
- next_step:
  - 人工打开 HTML，确认第三组曲线与分析卡片渲染正常。

### session_id: 20260310-1935-true-iekf-reset-audit

- timestamp: 2026-03-10 19:35 (local)
- objective: 核查 ApplyTrueInEkfReset 的 reset Jacobian 是否缺失 pos/vel ← att 交叉项，并对照文档公式。
- scope:
  - 检查 `BuildTrueInEkfResetGamma` 实现。
  - 对照 `可观性分析讨论与InEKF算法.tex` 的 $\bm\Gamma_k$ 公式。
- changed_files:
  - N/A
- configs:
  - N/A
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content src/core/eskf_engine.cpp`
  - `Select-String -Path 算法文档\\可观性分析讨论与InEKF算法.tex -Pattern "Gamma|\\bm\\Gamma" -Context 5,5`
- artifacts:
  - N/A
- metrics:
  - N/A
- observability_notes:
  - `BuildTrueInEkfResetGamma` 已包含 `-Skew(rho_p_body) * R_\Gamma` 与 `-Skew(rho_v_body) * R_\Gamma`，与文档公式一致；核心块外为单位阵。
  - 因此 P(pos/vel, att) 与欧氏扩展状态的互协方差在 reset 中会被重映射。
- decision:
  - 当前代码不存在“pos/vel ← att 交叉项缺失”的问题；若仍有 bg_z 约束偏弱迹象，需要从 true_iekf 启用与量测坐标一致性继续排查。
- next_step:
  - 若需进一步验证，添加 debug 输出（Gamma、dx、P[att,bg_z]）或定位未做 true_iekf 坐标变换的量测模型（如 ZUPT/UWB，若启用）。

### session_id: 20260310-2015-imu-precision-report

- timestamp: 2026-03-10 20:15 (local)
- objective: 汇总 data4 三种 IMU 精度对照实验的指标，并将第九/第十组写入交互报告。
- scope:
  - 重新导出交互报告 HTML，确认新增 IMU 对照页。
  - 提取第九/第十组 RMSE3D 与 tail70 指标用于实验记录。
- changed_files:
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260310-data4-imu-precision-r1/cfg_data4_gnss30_eskf_imu_{adis,ans,icm}.yaml`
  - `output/review/EXP-20260310-data4-imu-precision-r1/cfg_data4_gnss30_true_iekf_imu_{adis,ans,icm}.yaml`
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`（耗时较长，进程超时，但 HTML 已写入）
  - `python -`（import `interactive_nav_report.py`，仅计算第九/第十组 overview 表）
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - 第九组（ESKF）RMSE3D: ADIS `1277.644` m; ANS `51.374` m; ICM `1136.345` m。
  - 第九组（ESKF）tail70 RMSE3D: ADIS `1527.076` m; ANS `61.402` m; ICM `1358.190` m。
  - 第十组（true_iekf）RMSE3D: ADIS `180.320` m; ANS `23.144` m; ICM `1670.475` m。
  - 第十组（true_iekf）tail70 RMSE3D: ADIS `215.523` m; ANS `27.659` m; ICM `1996.597` m。
- observability_notes:
  - 本实验主要验证 IMU 精度差异对 GNSS30 工况误差的影响，未引入可观性消融或冻结策略。
- decision:
  - ANS-200R 在 ESKF 与 true_iekf 下均显著优于 ADIS/ICM；ICM-20602 在 GNSS30 下误差最大。
  - 交互报告新增第九/第十组 IMU 精度对照页，便于直观对比。
- next_step:
  - 人工确认 HTML 中第九/第十组曲线、指标表与分析卡片一致。

### session_id: 20260310-1956-remove-imu-pages

- timestamp: 2026-03-10 19:56 (local)
- objective: 按要求从 HTML 交互报告移除第九/第十组 IMU 精度对照页。
- scope:
  - 移除 `interactive_nav_report.py` 中第九/第十组 SectionSpec 与配色定义。
  - 移除 HTML 导出脚本中的 IMU 对照分析块，并更新侧边栏分组范围。
  - 重新导出交互报告 HTML。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - N/A
- observability_notes:
  - 本次调整仅影响报告展示，不改变实验数据与可观性结论。
- decision:
  - HTML 报告当前只保留第 1-8 组实验页，IMU 精度对照页暂时隐藏。
- next_step:
  - 人工打开 HTML，确认侧边栏编号为 1-8，且页面内容对应更新。

### session_id: 20260310-2010-report-rename-cleanup

- timestamp: 2026-03-10 20:10 (local)
- objective: 调整报告命名与方法名称显示，移除“实验分析与总结”卡片。
- scope:
  - 更新交互报告标题与副标题命名（实验 1/2/3/5/6）。
  - 将显示名称统一为 InEKF，避免出现 true_iekf。
  - 移除每组实验末尾“实验分析与总结”卡片。
  - 重新导出 HTML 报告。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `output/html/representative_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `python scripts/analysis/export_interactive_nav_report_html.py --target-points 1200`
- artifacts:
  - `output/html/representative_navigation_interactive_report.html`
- metrics:
  - N/A
- observability_notes:
  - 本次调整仅影响报告展示，不涉及可观性结论更新。
- decision:
  - HTML 报告已按要求统一命名，并移除分析卡片，页面中不再出现 true_iekf。
- next_step:
  - 人工打开 HTML，确认标题、方法名称与侧边栏一致。

### session_id: 20260310-2326-github-sync

- timestamp: 2026-03-10 23:26 (local)
- objective: 将当前待同步的代码、分析脚本与会话记录提交并推送到 GitHub。
- scope:
  - 核对 `main` 分支当前未提交变更与远端 `origin` 信息。
  - 追加本次 GitHub 同步会话记录。
  - 提交并推送当前待同步源码、分析脚本和 `walkthrough.md`。
- changed_files:
  - `apps/jacobian_audit_main.cpp`
  - `include/app/fusion.h`
  - `include/core/eskf.h`
  - `plot_navresult.py`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `scripts/analysis/compute_gnss_lever_arm.py`
  - `scripts/analysis/export_interactive_nav_report_html.py`
  - `scripts/analysis/filter_gnss_outage.py`
  - `scripts/analysis/inekf_mechanism_report.py`
  - `scripts/analysis/interactive_nav_report.py`
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `git status --short --branch`
  - `git remote -v`
  - `git diff --stat`
  - `git diff --numstat`
  - `git log --oneline -5`
  - `git add apps/jacobian_audit_main.cpp include/app/fusion.h include/core/eskf.h plot_navresult.py src/app/config.cpp src/app/pipeline_fusion.cpp src/core/eskf_engine.cpp scripts/analysis/compute_gnss_lever_arm.py scripts/analysis/export_interactive_nav_report_html.py scripts/analysis/filter_gnss_outage.py scripts/analysis/inekf_mechanism_report.py scripts/analysis/interactive_nav_report.py walkthrough.md`
  - `git commit -m "feat: add reset covariance audit and interactive report tooling"`
  - `git push origin main`
- artifacts:
  - GitHub remote: `origin https://github.com/zhang123-1999/combined_INS.git`
- metrics:
  - N/A
- observability_notes:
  - 本会话仅做版本同步，不新增实验结论；待同步内容包含 `GNSS split/reset` 协方差审计能力与交互式报告相关脚本。
- decision:
  - 将当前本地待同步更新整体推送到 `origin/main`，作为后续报告审阅与机理分析的远端基线。
- next_step:
  - 在远端确认提交可见后，继续执行 HTML 报告人工核对和 `bg_z`/杆臂相关后续分析。
## 下一步行动（Next Actions）

1. 在 GitHub 远端确认本次同步提交可见；随后人工打开 `output/html/representative_navigation_interactive_report.html`，确认基准组为 1-3 页、问题定向组为 4-8 页，且页面标题/方法名称与侧边栏一致。
2. 阅读 `output/review/EXP-20260310-inekf-mechanism-r1/summary.md`，将机理结论整理进最终实验报告章节。
3. 对 EXP-20260310-gnss-lever-fix-r1 的 `fixed` 组做更精细的杆臂收敛分析（若需要）：观察 GNSS 杆臂状态量 (index 27-29) 在 free 组的估计时序，确认估计值是否收敛到真值附近。
4. 基于杆臂实验结论，评估其他数据集（data2）是否同样存在未估计的 GNSS 杆臂偏差，决定是否在 data2 baseline 中也引入杆臂真值初始化。
5. 记录并对比 `data2/data4` 在 GNSS 有效窗口内的 `P[att,bg_z]`、`corr(att,bg_z)`、Kalman gain 与 `dx_bg_z` 时序，定位 data2 约束积累偏弱的具体更新段（延续旧 Next Actions）。
6. 基于新增 `disable_gyro_bias`，在 `fusion.post_gnss_ablation.*` 下做 data2 定向复验，确认状态块 `12-14` 冻结对 heading drift 和 RMSE3D 的边际收益（延续旧 Next Actions）。
7. 检查 HTML 中“v系真值基准安装角说明”卡片是否与 data2/data4 中位数一致。
8. 若继续追查 reset 相关问题，建议加日志核对 true_iekf 是否启用、`BuildTrueInEkfResetGamma` 是否被触发，以及 ZUPT/UWB 等模型是否做了 true_iekf 坐标变换。


