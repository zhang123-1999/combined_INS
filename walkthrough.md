# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-07`

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
| EXP-20260305-data2-gnsspos-mechanism-r2 | 2026-03-05 | data2 GNSS_POS 机制补全 | `config_data2_baseline_inekf_best.yaml`（派生 g0/g1 × p_on/off） | `output/review/EXP-20260305-data2-gnsspos-mechanism-r2/{gnsspos_mechanism_metrics.csv,h_att_timeline.csv,summary.md}` | p_on g0/g1 RMSE3D `1.692779/1.723701`; p_off g0/g1 `4.12e7/1.05e6`; corr `-0.091572` | 机制补证 | pass |
| EXP-20260305-data2-mountroll-observability-r1 | 2026-03-05 | mounting_roll 可观性补证 | `config_data2_gnss30_inekf_best.yaml`（冻结矩阵对照） | `output/review/EXP-20260305-data2-mountroll-observability-r1/{mountroll_observability_metrics.csv,std_mr_cmp.png,summary.md}` | `s1m1l0` post std_mr=0; RMSE3D `61.048699` vs ref `88.890428` | 状态块补证 | pass |
| EXP-20260305-data2-process-noise-regression-r1 | 2026-03-05 | RI 过程噪声映射回归 | `config_data2_baseline_inekf_best.yaml`, `config_data2_gnss30_inekf_best.yaml` | `output/review/EXP-20260305-data2-process-noise-regression-r1/process_noise_mode_regression.csv` | baseline g0 best `1.692779`; gnss30 g1 best `88.890428` | 回归证据 | pass |
| EXP-20260305-data2-postgnss-freeze-matrix-r2 | 2026-03-05 | post-GNSS 冻结矩阵重跑 | `config_data2_gnss30_inekf_best.yaml` | `output/review/EXP-20260305-data2-postgnss-freeze-matrix-r2/freeze_matrix_metrics.csv` | best `s1m1l0` RMSE3D `61.048699` (delta `-27.841730`) | 可观性证据补齐 | pass |
| EXP-20260305-data4-inekf-switchscan-r1 | 2026-03-05 | data4 InEKF 开关短扫 | `config_data4_baseline_inekf_best.yaml` | `output/review/EXP-20260305-data4-inekf-switchscan-r1/switch_scan_metrics.csv` | best `p0_g1_i0` RMSE3D `0.876829` | data4 最优开关 | pass |
| EXP-20260305-data4-main4-regression-r1 | 2026-03-05 | data4 baseline/GNSS30 × ESKF/InEKF | data4 4 组配置 | `output/review/EXP-20260305-data4-main4-regression-r1/{metrics_summary.csv,plots/}` | RMSE3D baseline ESKF/InEKF `0.929070/0.876829`; GNSS30 `51.371318/73.714930` | data4 主回归 | pass |
| EXP-20260305-data4-gnss30-freeze-matrix-r1 | 2026-03-05 | data4 GNSS30 冻结矩阵 | `config_data4_gnss30_inekf_best.yaml` | `output/review/EXP-20260305-data4-gnss30-freeze-matrix-r1/freeze_matrix_metrics.csv` | best `s0m1l1` RMSE3D `66.752679` (delta `-6.962252`) | data4 冻结证据 | pass |
| EXP-20260305-data4-gnssvel-sensitivity-r1 | 2026-03-05 | data4 GNSS 速度噪声敏感性 | data4 baseline/gnss30 InEKF | `output/review/EXP-20260305-data4-gnssvel-sensitivity-r1/sensitivity_metrics.csv` | baseline `0.876829` vs `5.178337`; gnss30 `73.714930` vs `26.479406` | 风险提示 | pass |
| EXP-20260305-data2-main4-docsync-r1 | 2026-03-05 | data2 4 组 doc 口径回归 | data2 4 组配置 | `output/review/EXP-20260305-data2-main4-docsync-r1/metrics_summary.csv` | RMSE3D baseline ESKF/InEKF `1.223243/1.723701`; GNSS30 `127.618348/88.890428` | 文档口径 | pass |

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
| HYP-12 | Under current phase-2 code, the remaining `data2` gap is unlikely to come from the `ODO/NHC` or `GNSS_VEL` measurement-model Jacobians; update/reset/covariance consistency is now the more suspicious remaining program path. | Jacobian audit r1: `ODO/NHC` max rel_fro only `1.48e-4/4.12e-4` for data2/data4; Jacobian audit r2: data4 `GNSS_VEL` focus blocks are numerically exact at output precision; data2 current config has no GNSS velocity updates. | Audit `ApplyTrueInEkfReset` and post-update covariance propagation next. | open(very_strongly_supported) |

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

## 下一步行动（Next Actions）

1. 优先审查 `ApplyTrueInEkfReset` 与更新后 covariance / cross-covariance 传播，验证其是否是 `data2` post-GNSS `bg_z -> yaw drift` 的主程序链路。
2. 基于新增 `disable_gyro_bias`，在 `fusion.post_gnss_ablation.*` 下做 data2 定向复验，确认状态块 `12-14` 冻结对 heading drift 和 RMSE3D 的边际收益。
3. 扩展 `jacobian_audit` 到 update-reset-covariance 局部一致性检查，不再重复审查已通过的 `ODO/NHC/GNSS_VEL` measurement Jacobian。
4. 若 reset/covariance 链路未发现决定性 bug，再回到 `GNSS_POS` 与 `21-30` 扩展块在 blackout 后的互协方差演化，解释 data2/data4 的 dataset-specific 差异。
5. 维持实验索引全量、会话日志摘要化；后续仅对新实验写入可复现证据和 fresh 指标，不再追加冗长过程性噪声。

