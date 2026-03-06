# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-06`

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
| ISSUE-007-inekf-tex-code-gap | 2026-03-06 | 代码-理论差距已部分收敛：新增 `true_iekf_mode` 后，Lie 核心 `F/GNSS_POS/Inject/reset` 已更接近 `可观性分析讨论与InEKF算法.tex`，但 ODO/NHC、GNSS_VEL 与 `21-30` 欧氏扩展块仍是 hybrid 实现，无法宣称“全31维完全无泄露”。 | `算法文档/可观性分析讨论与InEKF算法.tex`; `include/core/eskf.h`; `src/core/ins_mech.cpp`; `src/core/measurement_models_uwb.cpp`; `src/core/eskf_engine.cpp`; `output/review/20260306-true-iekf-refactor-r1/summary.md` | 影响 true-IEKF 的最终口径与 sparse-GNSS 性能。 | partially_resolved |

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
| HYP-8 | `true_iekf` 在 sparse-GNSS 下的主要 blocker 不是 `GNSS_VEL`，而是仍为 hybrid 的 road-constraint 通道（`ODO/NHC`）及其与 `21-30` 扩展块的耦合/reset 一致性。 | data2 GNSS30: full/no_gnss_vel `201.158876/201.158876`，no_odo `370.355380`，no_nhc `716.565952`；data4 GNSS30: full/no_gnss_vel `115.499686/121.404109`，no_odo `669.763471`，no_nhc `784.539644`。 | 重推 `ODO/NHC` Jacobian 与 full-state reset Jacobian，再复测 GNSS30。 | open(strongly_supported) |

### 已否决假设（压缩）

- `HYP-2`：rejected（新实现证据推翻）
- `HYP-4`：rejected（`p_ned_local` 单因子不是主因）
- `HYP-5`：rejected（仅补单项 `G(kVel,gyro)` 不能修复）

## 会话日志（Session Log）

### 历史阶段摘要（压缩，保留可追溯 ID）

- 阶段A `20260304-1435 ~ 20260304-1855`：
  - 完成协作文档与基础回归修复（`ISSUE-002/003` 关闭）。
  - 相关 session: `20260304-1435-agent-doc-bootstrap`, `20260304-1505-agent-md-fix-bootstrap-and-context`, `20260304-1822-pre_experiment_audit`, `20260304-1855-fix-five-issues-and-md-check`。
- 阶段B `20260304-2016 ~ 20260304-2318`：
  - InEKF 从“FEJ 依赖版”重写到 RI 实现，再到 `G(kVel,gyro)` 单点修补。
  - 关键实验: `EXP-20260304-inekf-refactor-minrisk-debug`, `EXP-20260304-inekf-rewrite-no-fej`, `EXP-20260304-inekf-ri-realization-codeoverhaul3`, `EXP-20260304-inekf-ri-gvelgyro-fix`。
- 阶段C `20260304-2358 ~ 20260305-0018`：
  - 审查 + deep-research + 顺序 A/B，锁定当前最优分支。
  - 关键实验: `EXP-20260304-inekf-ri-fgqd-audit-r2`, `EXP-20260305-inekf-deep-research-audit-r1`, `EXP-20260305-inekf-ab-seq-r1`。

### session_id: 20260305-0032-best4-regression-and-plot

- timestamp: 2026-03-05 00:33 (local)
- objective: 在最优开关分支上完成 4 组标准回归与绘图。
- scope:
  - 运行 baseline/GNSS30 × ESKF/InEKF 四组回归。
  - 生成单跑图和两组对比图。
- changed_files:
  - `output/review/20260305-inekf-best4-reg-r1/cfg_*.yaml`
  - `output/review/20260305-inekf-best4-reg-r1/*.log`
  - `output/review/20260305-inekf-best4-reg-r1/SOL_*.txt`
  - `output/review/20260305-inekf-best4-reg-r1/metrics_summary.csv`
  - `output/review/20260305-inekf-best4-reg-r1/run_summary.md`
  - `output/review/20260305-inekf-best4-reg-r1/plots/`
  - `output/review/20260305-inekf-best4-reg-r1/compare_*`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_eskf_nofreeze.yaml`
  - `config_data2_gnss30_inekf.yaml`
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/20260305-inekf-best4-reg-r1/cfg_*.yaml`
  - `python plot_navresult.py output/review/20260305-inekf-best4-reg-r1/SOL_*.txt`
  - `python scripts/analysis/compare_fej_vs_standard.py ...`
- artifacts:
  - `output/review/20260305-inekf-best4-reg-r1/metrics_summary.csv`
  - `output/review/20260305-inekf-best4-reg-r1/run_summary.md`
  - `output/review/20260305-inekf-best4-reg-r1/plots/`
  - `output/review/20260305-inekf-best4-reg-r1/compare_baseline_eskf_vs_inekf_best/`
  - `output/review/20260305-inekf-best4-reg-r1/compare_gnss30_eskf_vs_inekf_best/`
- metrics:
  - baseline ESKF/InEKF RMSE3D: `1.223242 / 1.723701`
  - GNSS30 ESKF/InEKF RMSE3D: `127.618202 / 88.890702`
  - `H_att_max`：baseline `2.321396 / 2354.037209`，GNSS30 `1.818384 / 2354.037209`
- artifact_mtime:
  - `metrics_summary.csv`: 2026-03-05 00:33:30
  - `run_summary.md`: 2026-03-05 00:35:41
- config_hash_or_mtime:
  - 基础配置未改；本轮使用 `output/review/20260305-inekf-best4-reg-r1/cfg_*.yaml`
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（回归与绘图均在本会话生成）
- observability_notes:
  - 状态块 `21-30` 未做新的冻结/消融；本轮聚焦 `kPos/kVel/kAtt` RI 分支一致性。
  - baseline: InEKF 相对 ESKF degraded；GNSS30: InEKF 相对 ESKF improved。
- decision:
  - 最优开关分支可复现，但 baseline 回归问题仍未闭环。
- next_step:
  - 开始 `G/F/H/Inject` 全链路符号一致性审查与交互项实验。

### session_id: 20260305-0042-tidy-walkthrough-and-promote-inekf-baseline

- timestamp: 2026-03-05 00:42 (local)
- objective: 精简 `walkthrough.md` 并将 best InEKF 方案固化为新的 InEKF 基线配置。
- scope:
  - 将过长会话日志压缩为阶段摘要，保留关键实验与未解决问题的可追溯性。
  - 新增并固化两个 InEKF 基线配置文件。
- changed_files:
  - `walkthrough.md`
  - `config_data2_baseline_inekf_best.yaml`
  - `config_data2_gnss30_inekf_best.yaml`
- configs:
  - `config_data2_baseline_inekf_best.yaml`
  - `config_data2_gnss30_inekf_best.yaml`
- commands:
  - `Get-Content/rg` 检查 `walkthrough.md` 结构与长度
  - 脚本化生成 best 配置并修正 `generator.output_path`
- artifacts:
  - `walkthrough.md`（压缩版）
  - `config_data2_baseline_inekf_best.yaml`
  - `config_data2_gnss30_inekf_best.yaml`
- metrics:
  - 文档整理任务，无新增解算指标。
- artifact_mtime:
  - `walkthrough.md`: 2026-03-05
  - `config_data2_baseline_inekf_best.yaml`: 2026-03-05
  - `config_data2_gnss30_inekf_best.yaml`: 2026-03-05
- config_hash_or_mtime:
  - 两个新配置均基于旧 InEKF 配置复制并显式写入 best 开关。
- dataset_time_window:
  - N/A（本会话未新跑解算）
- result_freshness_check:
  - pass（文档与配置更新完成，结构可读）
- observability_notes:
  - 本次仅文档与配置治理，无新增观测性实验。
- decision:
  - `config_data2_baseline_inekf_best.yaml` 与 `config_data2_gnss30_inekf_best.yaml` 作为新的 InEKF 基线入口。
- next_step:
  - 后续回归和对比默认使用 `*_inekf_best.yaml`；旧 `*_inekf_ctrl.yaml` 仅用于历史复现实验。

### session_id: 20260305-1010-inekf-fghi-audit-and-doubleoff

- timestamp: 2026-03-05 10:10 (local)
- objective: 按优先队列继续执行 `F/G/H/Inject` 一致性审查，并补做 `p_off + inject_off` 交互实验。
- scope:
  - 审查 `src/core/ins_mech.cpp`, `src/core/measurement_models_uwb.cpp`, `src/core/eskf_engine.cpp` 的符号链路。
  - 运行双关交互实验（baseline + GNSS30，各 1 组）。
- changed_files:
  - `output/review/20260305-inekf-fghi-sign-audit-r1/inekf_fghi_sign_audit.md`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/cfg_doubleoff_baseline.yaml`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/cfg_doubleoff_gnss30.yaml`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/doubleoff_baseline.log`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/doubleoff_gnss30.log`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/SOL_doubleoff_baseline.txt`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/SOL_doubleoff_gnss30.txt`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/doubleoff_metrics.csv`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/doubleoff_metrics.json`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/summary.md`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_inekf_best.yaml`（派生 `cfg_doubleoff_baseline.yaml`）
  - `config_data2_gnss30_inekf_best.yaml`（派生 `cfg_doubleoff_gnss30.yaml`）
- commands:
  - `rg -n "F.block<3, 3>(StateIdx::kVel, StateIdx::kAtt)|G.block<3, 3>(StateIdx::kVel, 3)|dv_ned -= Skew|dr_ned -= Skew|ri_gnss_pos_use_p_ned_local|ri_inject_pos_inverse" src/core/*.cpp include/core/eskf.h`
  - `build/Release/eskf_fusion.exe --config output/review/20260305-inekf-doubleoff-interaction-r1/cfg_doubleoff_baseline.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/20260305-inekf-doubleoff-interaction-r1/cfg_doubleoff_gnss30.yaml`
  - `Select-String`/`Import-Csv` 解析 RMSE 与 `H_att_max`
- artifacts:
  - `output/review/20260305-inekf-fghi-sign-audit-r1/inekf_fghi_sign_audit.md`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/doubleoff_metrics.csv`
  - `output/review/20260305-inekf-doubleoff-interaction-r1/summary.md`
- metrics:
  - double-off baseline RMSE xyz: `196364.475508 505849.364849 291200.065746`
  - double-off gnss30 RMSE xyz: `174296.339000 209616.276000 135334.838000`
  - double-off RMSE3D baseline/GNSS30: `615824.703508 / 304357.873776`
  - 对照（best 分支）RMSE3D baseline/GNSS30: `1.723701 / 88.890702`
  - double-off `H_att_max` baseline/GNSS30: `20.722139 / 20.384733`
- artifact_mtime:
  - `output/review/20260305-inekf-doubleoff-interaction-r1/doubleoff_metrics.csv`: 2026-03-05 10:07:42
  - `output/review/20260305-inekf-doubleoff-interaction-r1/summary.md`: 2026-03-05 10:07:42
  - `output/review/20260305-inekf-fghi-sign-audit-r1/inekf_fghi_sign_audit.md`: 2026-03-05 10:09:42
- config_hash_or_mtime:
  - `cfg_doubleoff_*.yaml` 由 `*_inekf_best.yaml` 派生，仅切换 `ri_gnss_pos_use_p_ned_local=false`, `ri_inject_pos_inverse=false`，保持 `ri_vel_gyro_noise_mode=1`。
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（实验与审查产物均在本会话生成）
- observability_notes:
  - 本轮直接影响状态块 `0-8`（`p/v/att`）的 RI 线性化与注入一致性。
  - 状态块 `21-30`（`odo_scale/mounting/lever_*`）未做冻结或消融变更，行为保持原配置。
  - 传感器窗口：baseline 为全程 GNSS；gnss30 为 `fusion.gnss_schedule.head_ratio=0.3` 的分段窗口。
  - 双关关闭（`p_off + inject_off`）在两种窗口下均显著 degraded（实测发散）。
- decision:
  - 完成 `F/G/H/Inject` 一致性审查与双关交互排错；支持继续沿用 `p_on + g=+1 + inject_on` 作为 InEKF 默认研究分支。
- next_step:
  - 先处理 `ISSUE-001`（重生 baseline 对比目录并统一口径），再推进 post-GNSS 冻结矩阵实验。

### session_id: 20260305-1018-issue001-baseline-compare-refresh

- timestamp: 2026-03-05 10:18 (local)
- objective: 执行优先队列第 1 项，重生 baseline 对比目录并降低 `ISSUE-001` 风险。
- scope:
  - 用 `config_data2_baseline_eskf.yaml` 与 `config_data2_baseline_inekf_best.yaml` 重新跑 baseline。
  - 重建 `output/compare_data2_baseline_eskf_vs_inekf_ctrl/` 的图表与指标文件。
- changed_files:
  - `output/review/20260305-issue001-baseline-compare-r1/cfg_baseline_eskf_r1.yaml`
  - `output/review/20260305-issue001-baseline-compare-r1/cfg_baseline_inekf_best_r1.yaml`
  - `output/review/20260305-issue001-baseline-compare-r1/baseline_eskf_r1.log`
  - `output/review/20260305-issue001-baseline-compare-r1/baseline_inekf_best_r1.log`
  - `output/review/20260305-issue001-baseline-compare-r1/SOL_baseline_eskf_r1.txt`
  - `output/review/20260305-issue001-baseline-compare-r1/SOL_baseline_inekf_best_r1.txt`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.json`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/metrics.csv`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/*.png`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_best.yaml`
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/20260305-issue001-baseline-compare-r1/cfg_baseline_eskf_r1.yaml`
  - `build/Release/eskf_fusion.exe --config output/review/20260305-issue001-baseline-compare-r1/cfg_baseline_inekf_best_r1.yaml`
  - `python scripts/analysis/compare_fej_vs_standard.py --std ...SOL_baseline_eskf_r1.txt --fej ...SOL_baseline_inekf_best_r1.txt --truth dataset/data2_converted/POS_converted.txt --outdir output/compare_data2_baseline_eskf_vs_inekf_ctrl`
- artifacts:
  - `output/review/20260305-issue001-baseline-compare-r1/*.log`
  - `output/review/20260305-issue001-baseline-compare-r1/SOL_baseline_*.txt`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/metrics.csv`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt`
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.json`
- metrics:
  - ESKF RMSE xyz: `0.763591 0.565924 0.770052`（log）
  - InEKF_best RMSE xyz: `0.947842 0.971213 1.062772`（log）
  - 对比目录 RMSE3D ESKF/InEKF_best: `1.223243 / 1.723701`（metrics.csv）
  - 旧冲突口径（历史目录）InEKF RMSE3D `54.721255` 已被新口径覆盖。
- artifact_mtime:
  - `output/review/20260305-issue001-baseline-compare-r1/baseline_eskf_r1.log`: 2026-03-05 10:17:09
  - `output/review/20260305-issue001-baseline-compare-r1/baseline_inekf_best_r1.log`: 2026-03-05 10:17:44
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/metrics.csv`: 2026-03-05 10:18:18
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt`: 2026-03-05 10:18:18
- config_hash_or_mtime:
  - 本轮配置由基线配置复制，仅改 `fusion.output_path` 指向本轮 `output/review/20260305-issue001-baseline-compare-r1/`。
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（解算、对比、图表均在本会话重生成）
- observability_notes:
  - 本轮未新增 ablation/freeze，状态块 `21-30` 行为未改。
  - 重点是状态块 `0-8` 在统一口径下的 baseline 对比可复现性。
  - 传感器窗口为 baseline 全程 GNSS（无 GNSS schedule 截断）。
- decision:
  - `ISSUE-001` 从 `open` 降级为 `downgraded`；后续 baseline 对比以重生产物为准。
- next_step:
  - 继续优先队列第 2 项：在 best 分支执行 post-GNSS 冻结矩阵（`odo_scale/mounting/lever`）。

### session_id: 20260305-1029-postgnss-freeze-matrix-r1

- timestamp: 2026-03-05 10:29 (local)
- objective: 执行优先队列第 2 项，完成 best 分支下 post-GNSS 冻结矩阵实验并补齐状态块 `21-30` 可观性证据。
- scope:
  - 以 `config_data2_gnss30_inekf_best.yaml` 为基线，构造 `odo_scale/mounting/lever_all` 的 2^3 冻结矩阵。
  - 统一启用 `constraints.enable_consistency_log=true`，记录 `accept_ratio/nis_*`。
- changed_files:
  - `output/review/20260305-postgnss-freeze-matrix-r1/cfg_s*.yaml`
  - `output/review/20260305-postgnss-freeze-matrix-r1/s*.log`
  - `output/review/20260305-postgnss-freeze-matrix-r1/SOL_s*.txt`
  - `output/review/20260305-postgnss-freeze-matrix-r1/freeze_matrix_metrics.csv`
  - `output/review/20260305-postgnss-freeze-matrix-r1/freeze_matrix_metrics.json`
  - `output/review/20260305-postgnss-freeze-matrix-r1/summary.md`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_inekf_best.yaml`（派生 8 组 `cfg_s{0|1}m{0|1}l{0|1}.yaml`）
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/20260305-postgnss-freeze-matrix-r1/cfg_s*.yaml`
  - 解析日志中的 `RMSE (融合) [m]`、`[Consistency] NHC/ODO ...`、`||H_att||_F`
- artifacts:
  - `output/review/20260305-postgnss-freeze-matrix-r1/freeze_matrix_metrics.csv`
  - `output/review/20260305-postgnss-freeze-matrix-r1/summary.md`
  - `output/review/20260305-postgnss-freeze-matrix-r1/s*.log`
- metrics:
  - 参考分支（无 post ablation）`s0m0l0` RMSE3D: `88.890702`
  - 最优分支 `s1m1l0`（freeze scale+mounting）RMSE3D: `61.048358`（delta `-27.842344`）
  - `freeze_mounting` 相关分支整体优于未冻结；`freeze_lever` 单独冻结最差（`95.202506`）
  - NHC/ODO `accept_ratio` 约 `1.000`，`nis_mean` 基本稳定（NHC `0.049`，ODO `0.183`）
- artifact_mtime:
  - `freeze_matrix_metrics.csv`: 2026-03-05 10:28:45
  - `summary.md`: 2026-03-05 10:28:45
  - `s1m1l0.log`: 2026-03-05 10:28:10
- config_hash_or_mtime:
  - 各分支仅改 `fusion.post_gnss_ablation.*`、`fusion.constraints.enable_consistency_log` 与 `fusion.output_path`。
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（8 组实验与汇总均在本会话生成）
- observability_notes:
  - 冻结块映射：`odo_scale(21)`、`mounting(22-24)`、`lever_all(25-30)`。
  - 传感器窗口：`gnss_schedule.head_ratio=0.3`，GNSS 在 split 后关闭，冻结在 post-GNSS 生效。
  - 行为：冻结 `mounting` 明显 improved；冻结 `lever_all` 在未冻结 `mounting` 时 degraded；冻结 `odo_scale` 单独影响较小。
- decision:
  - post-GNSS 阶段优先冻结 `mounting`（可叠加 `odo_scale`），暂不建议单独冻结 `lever_all`。
- next_step:
  - 执行优先队列第 3 项：定位 baseline `g=0` vs `g=+1` 的差异来源。

### session_id: 20260305-1033-baseline-gmode-sensitivity-r1

- timestamp: 2026-03-05 10:33 (local)
- objective: 执行优先队列项，定位 baseline `g=0` vs `g=+1` 差异并检查时段敏感性。
- scope:
  - 基于 `config_data2_baseline_inekf_best.yaml` 仅切换 `ri_vel_gyro_noise_mode={0,1}`。
  - 提取整体 RMSE、分时段 RMSE（head/tail）、`H_att` 分时段均值与 `accept_ratio/nis_*`。
- changed_files:
  - `output/review/20260305-baseline-gmode-sensitivity-r1/cfg_gmode_0.yaml`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/cfg_gmode_1.yaml`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/gmode_0.log`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/gmode_1.log`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/SOL_gmode_0.txt`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/SOL_gmode_1.txt`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/gmode_sensitivity_metrics.csv`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/gmode_sensitivity_metrics.json`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/summary.md`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_inekf_best.yaml`（派生 `cfg_gmode_0.yaml`, `cfg_gmode_1.yaml`）
- commands:
  - `build/Release/eskf_fusion.exe --config .../cfg_gmode_0.yaml`
  - `build/Release/eskf_fusion.exe --config .../cfg_gmode_1.yaml`
  - 解析日志中的 `RMSE (融合) [m]`、`[Consistency] NHC/ODO...` 与 `[GNSS_POS] ||H_att||_F`
- artifacts:
  - `output/review/20260305-baseline-gmode-sensitivity-r1/gmode_sensitivity_metrics.csv`
  - `output/review/20260305-baseline-gmode-sensitivity-r1/summary.md`
- metrics:
  - RMSE3D `g0/g1`: `1.692779 / 1.723701`（`g0` 略优 `-0.030922`）
  - head/tail RMSE3D：`g0=1.676679/1.708728`，`g1=1.789767/1.655000`
  - `H_att_mean_head/tail`：`g0=1242.988/1161.861`，`g1=1242.915/1161.867`
  - NHC/ODO `accept_ratio` 近似不变（约 `0.99998/0.99968`）
- artifact_mtime:
  - `gmode_sensitivity_metrics.csv`: 2026-03-05 10:33:14
  - `summary.md`: 2026-03-05 10:33:14
- config_hash_or_mtime:
  - 仅切换 `fusion.fej.ri_vel_gyro_noise_mode` 与 `constraints.enable_consistency_log`、`output_path`。
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（两组解算与统计均在本会话生成）
- observability_notes:
  - 本轮聚焦状态块 `0-8` 的时段敏感性；状态块 `21-30` 无新增冻结。
  - 传感器窗口为 baseline 全程 GNSS。
- decision:
  - baseline 的 `g` 模式差异较小但有时段互补，`ISSUE-005/006` 仍需结合 `GNSS vel` 进一步定位。
- next_step:
  - 继续验证 post-GNSS 冻结策略的窗口稳健性（`head_ratio` 多取值）。

### session_id: 20260305-1039-postgnss-headratio-robustness-r1

- timestamp: 2026-03-05 10:39 (local)
- objective: 执行优先队列项，验证 post-GNSS `freeze(scale+mounting)` 在不同 `head_ratio` 下的稳健性。
- scope:
  - `head_ratio={0.2,0.3,0.4,0.5}`。
  - 每个 ratio 对比 `ref(无冻结)` 与 `freeze_sm(冻结 scale+mounting)`。
- changed_files:
  - `output/review/20260305-postgnss-headratio-robustness-r1/cfg_r*_ref.yaml`
  - `output/review/20260305-postgnss-headratio-robustness-r1/cfg_r*_freeze_sm.yaml`
  - `output/review/20260305-postgnss-headratio-robustness-r1/r*_*.log`
  - `output/review/20260305-postgnss-headratio-robustness-r1/SOL_r*_*.txt`
  - `output/review/20260305-postgnss-headratio-robustness-r1/headratio_robustness_metrics.csv`
  - `output/review/20260305-postgnss-headratio-robustness-r1/headratio_robustness_metrics.json`
  - `output/review/20260305-postgnss-headratio-robustness-r1/summary.md`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_inekf_best.yaml`（派生 8 组 ratio×case 配置）
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/20260305-postgnss-headratio-robustness-r1/cfg_r*_*.yaml`
  - 解析日志中的 `RMSE (融合) [m]` 与 `Consistency` 统计
- artifacts:
  - `output/review/20260305-postgnss-headratio-robustness-r1/headratio_robustness_metrics.csv`
  - `output/review/20260305-postgnss-headratio-robustness-r1/summary.md`
- metrics:
  - ratio `0.2`: ref/freeze `114.476766 / 65.755126`（delta `-48.721640`）
  - ratio `0.3`: ref/freeze `88.890702 / 61.048358`（delta `-27.842344`）
  - ratio `0.4`: ref/freeze `87.671761 / 68.353867`（delta `-19.317894`）
  - ratio `0.5`: ref/freeze `64.185725 / 52.388133`（delta `-11.797592`）
  - NHC/ODO `accept_ratio` 维持 `1.000/1.000`
- artifact_mtime:
  - `headratio_robustness_metrics.csv`: 2026-03-05 10:38:52
  - `summary.md`: 2026-03-05 10:38:52
- config_hash_or_mtime:
  - 仅切换 `gnss_schedule.head_ratio`、`post_gnss_ablation` 开关与 `output_path`。
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（4×2 组实验与汇总均在本会话生成）
- observability_notes:
  - 冻结块作用于 `odo_scale(21)` 与 `mounting(22-24)`，在 GNSS 关闭后生效。
  - 各窗口均 improved，说明 `mounting` 冻结结论对 GNSS 调度比例具稳健性。
- decision:
  - `freeze(scale+mounting)` 可作为 post-GNSS 默认候选策略；后续可再做状态 `21-30` 漂移图佐证。
- next_step:
  - 回到 `HYP-1`：把 baseline 的时段差异定位到 `GNSS vel` 与过程噪声耦合项。

### session_id: 20260305-1048-stage-summary-writeback

- timestamp: 2026-03-05 10:48 (local)
- objective: 将“阶段性成果总结”写入 `walkthrough.md`，形成可直接引用的里程碑摘要。
- scope:
  - 汇总当前阶段关键实验与结论。
  - 在不改动实验结论的前提下，补充文档层总结段落。
- changed_files:
  - `walkthrough.md`
- configs:
  - N/A（文档任务）
- commands:
  - `Get-Content walkthrough.md`
  - 文档写回（summary section + session log）
- artifacts:
  - `walkthrough.md`（新增“阶段性成果总结”）
- metrics:
  - 文档任务，无新增解算指标。
- observability_notes:
  - 本次仅归纳已有证据，不新增状态块冻结/消融实验。
- decision:
  - 阶段性成果已在文档中固化，便于后续复盘与对外同步。
- next_step:
  - 继续执行当前优先队列（机制定位 + 状态块时序分析）。

## 阶段性成果总结（截至 2026-03-05）

1. InEKF 已收敛到可复现实验分支：`p_ned_local=ON, g=+1, inject=ON`。
2. `ISSUE-001` 已通过口径重生降级：baseline 对比统一为 ESKF/InEKF_best `RMSE3D=1.223243/1.723701`。
3. `F/G/H/Inject` 一致性证据链闭环：双关关闭（`p_off+inject_off`）出现灾难性发散，反证当前链路选择合理。
4. post-GNSS 冻结矩阵显示 `mounting` 是关键块：`s1m1l0` 相比参考 `s0m0l0` 改善 `-27.842344`。
5. 冻结策略对 `gnss_schedule.head_ratio` 具稳健性：`0.2/0.3/0.4/0.5` 全部 improved（delta `-48.72~-11.80`）。
6. baseline `g=0` vs `g=+1` 差异已缩小并定位为“时段敏感”问题：`g0/g1=1.692779/1.723701`，但 `HYP-1` 仍未闭合。
7. data2 `GNSS_POS` 机制补全完成：`p_local=on` 差异小，`p_local=off` 灾难性发散，corr `-0.091572`。
8. data2 mounting_roll 可观性补证：`s1m1l0` post std_mr=0，RMSE3D `61.048699`。
9. data4 InEKF 最优开关为 `p0_g1_i0`，baseline RMSE3D `0.876829` 优于 ESKF `0.929070`。
10. data4 GNSS30 退化明显：ESKF/InEKF RMSE3D `51.371318/73.714930`；冻结矩阵 best `s0m1l1` 改善 `-6.962252`。
11. 四份结果文档已全部重生并可编译通过（含 `组合导航.tex`）。

### session_id: 20260305-1058-gmode-gnssvel-and-state21-30-drift

- timestamp: 2026-03-05 10:58 (local)
- objective: 按优先队列继续执行，完成 `g=0/1` 机制排查并输出状态块 `21-30` 时序漂移证据。
- scope:
  - 跑 `g=0/1 × GNSS速度 on/off` 四组对照，验证 `GNSS_VEL` 是否为主因。
  - 基于 post-GNSS 冻结矩阵解算结果，生成状态块 `21-30` 漂移统计与图。
- changed_files:
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/GNSS_converted_7col.txt`
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/cfg_g*_vel*.yaml`
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/g*_vel*.log`
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/SOL_g*_vel*.txt`
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/gnssvel_mechanism_metrics.csv`
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/summary.md`
  - `output/review/20260305-state21_30-drift-r1/state21_30_postgnss_drift.csv`
  - `output/review/20260305-state21_30-drift-r1/summary.md`
  - `output/review/20260305-state21_30-drift-r1/01_odo_scale_postgnss.png`
  - `output/review/20260305-state21_30-drift-r1/02_mounting_postgnss.png`
  - `output/review/20260305-state21_30-drift-r1/03_lever_norm_postgnss.png`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_inekf_best.yaml`（派生 `cfg_g*_vel*.yaml`）
  - `output/review/20260305-postgnss-freeze-matrix-r1/cfg_s*.yaml`（作为状态块漂移分析输入）
- commands:
  - `build/Release/eskf_fusion.exe --config output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/cfg_g*_vel*.yaml`
  - 解析日志中的 `RMSE (融合) [m]`、`[Consistency] ...`、`[GNSS_POS] ||H_att||_F`
  - 读取 `SOL_s*.txt` 计算 post-GNSS 漂移统计并绘图
- artifacts:
  - `output/review/20260305-baseline-gmode-gnssvel-mechanism-r1/gnssvel_mechanism_metrics.csv`
  - `output/review/20260305-state21_30-drift-r1/state21_30_postgnss_drift.csv`
  - `output/review/20260305-state21_30-drift-r1/*.png`
- metrics:
  - 机制排查：`vel_on/off` 下 `g0/g1` 完全一致（`1.692779/1.723701`），`delta(g0-g1)=-0.030922` 不变。
  - 说明 data2 当前配置下 `GNSS_VEL` 非主因（文件仅 7 列，实际无速度更新）。
  - 状态块漂移：最优 `s1m1l0` 相对参考 `s0m0l0` RMSE3D 改善 `-27.842344`；且 `mounting_pitch/yaw` post-GNSS 漂移约 0。
- artifact_mtime:
  - `gnssvel_mechanism_metrics.csv`: 2026-03-05 10:55:57
  - `state21_30_postgnss_drift.csv`: 2026-03-05 10:57:55
  - `03_lever_norm_postgnss.png`: 2026-03-05 10:57:58
- config_hash_or_mtime:
  - 机制排查分支仅改 `ri_vel_gyro_noise_mode`、`gnss_path`、`output_path`、`enable_consistency_log`。
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check:
  - pass（四组机制对照 + 状态块漂移图均在本会话生成）
- observability_notes:
  - 状态块映射：`21=odo_scale`, `22-24=mounting`, `25-30=lever_*`（`mounting_roll` 未写入 SOL，当前仅可视化 pitch/yaw）。
  - post-GNSS 窗口下，冻结 `mounting` 的分支表现 improved，且 mounting 时序最稳定。
- decision:
  - `HYP-1` 机制已收缩：先排除 `GNSS_VEL`，后续重点转向 `GNSS_POS` 与过程噪声耦合。
- next_step:
  - 深挖 `GNSS_POS` 相关项在 `g=0/1` 下的时段作用机制，并定义 `ISSUE-005/006` 关闭门槛。

### session_id: 20260305-1611-automation-worktree-overview

- timestamp: 2026-03-05 16:11 (local)
- objective: 回答“自动化与工作树功能”的概念说明（面向 Codex app）。
- scope:
  - 读取官方说明，整理高层功能描述。
- changed_files:
  - `walkthrough.md`
- configs:
  - N/A
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Date -Format "yyyy-MM-dd HH:mm"`
  - `web.run (openai.com/help.openai.com)`
- artifacts:
  - `walkthrough.md`
- metrics:
  - N/A
- observability_notes:
  - 本次为产品说明，无新增实验或状态块分析。
- decision:
  - 输出 Codex app 的自动化与工作树功能概述，并提示可进一步讲解 Git worktree 细节。
- next_step:
  - 等待用户确认是否需要 Git worktree 的更深入讲解或示例。

### session_id: 20260305-2222-inekf-eskf-data2-data4-pipeline

- timestamp: 2026-03-05 22:22 (local)
- objective: 完成 data2 优先队列闭环、data4 主实验与四份结果文档重生。
- scope:
  - data2：GNSS_POS 机制补全、mounting_roll 可观性补证、过程噪声映射回归、冻结矩阵重跑。
  - data4：InEKF 开关短扫、main4 回归、GNSS30 冻结矩阵、GNSS 速度敏感性。
  - 文档：重生 4 份结果文档并编译通过。
- changed_files:
  - `scripts/analysis/run_research_pipeline.py`
  - `config_data4_baseline_eskf.yaml`
  - `config_data4_baseline_inekf_best.yaml`
  - `config_data4_gnss30_eskf.yaml`
  - `config_data4_gnss30_inekf_best.yaml`
  - `output/review/EXP-20260305-data2-gnsspos-mechanism-r2/*`
  - `output/review/EXP-20260305-data2-mountroll-observability-r1/*`
  - `output/review/EXP-20260305-data2-process-noise-regression-r1/*`
  - `output/review/EXP-20260305-data2-postgnss-freeze-matrix-r2/*`
  - `output/review/EXP-20260305-data4-inekf-switchscan-r1/*`
  - `output/review/EXP-20260305-data4-main4-regression-r1/*`
  - `output/review/EXP-20260305-data4-gnss30-freeze-matrix-r1/*`
  - `output/review/EXP-20260305-data4-gnssvel-sensitivity-r1/*`
  - `output/review/EXP-20260305-data2-main4-docsync-r1/*`
  - `output/review/EXP-20260305-research-pipeline-summary/*`
  - `算法文档/结果文档/组合导航.tex`
  - `算法文档/结果文档/data2_inekf_gnss30_对比冻结.tex`
  - `算法文档/结果文档/data4_eskf_全程_vs_30GNSS.tex`
  - `算法文档/结果文档/data4_inekf_gnss30_对比冻结.tex`
  - `算法文档/结果文档/tex_compile_status.csv`
  - `算法文档/结果文档/doc_assets_index.json`
- configs:
  - data2: `config_data2_baseline_eskf.yaml`, `config_data2_baseline_inekf_best.yaml`,
    `config_data2_gnss30_eskf_nofreeze.yaml`, `config_data2_gnss30_inekf_best.yaml`
  - data4: `config_data4_baseline_eskf.yaml`, `config_data4_baseline_inekf_best.yaml`,
    `config_data4_gnss30_eskf.yaml`, `config_data4_gnss30_inekf_best.yaml`
- commands:
  - `python scripts/analysis/run_research_pipeline.py --phase all --head-ratio 0.3`
  - `python scripts/analysis/run_research_pipeline.py --phase docs --head-ratio 0.3`
  - `xelatex` 编译 4 份结果文档
- artifacts:
  - `output/review/EXP-20260305-research-pipeline-summary/experiment_manifest.json`
  - `output/review/EXP-20260305-research-pipeline-summary/metrics_summary.csv`
  - 4 份 PDF（见 `算法文档/结果文档/*.pdf`）
- metrics:
  - data2 main4 RMSE3D: baseline ESKF/InEKF `1.223243/1.723701`, GNSS30 `127.618348/88.890428`
  - data2 冻结矩阵 best: `s1m1l0` RMSE3D `61.048699` (delta `-27.841730`)
  - data4 main4 RMSE3D: baseline ESKF/InEKF `0.929070/0.876829`, GNSS30 `51.371318/73.714930`
  - data4 冻结矩阵 best: `s0m1l1` RMSE3D `66.752679` (delta `-6.962252`)
- artifact_mtime:
  - `EXP-20260305-research-pipeline-summary/run_summary.json`: 2026-03-05 22:20:49
  - `tex_compile_status.csv`: 2026-03-05 22:20:49
- config_hash_or_mtime:
  - data4 InEKF best switches 固化为 `p_local=OFF, g=+1, inject=OFF`
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（所有实验与文档在本会话生成）
- observability_notes:
  - 状态块映射：`21=odo_scale`, `22=mounting_roll`, `23=mounting_pitch`, `24=mounting_yaw`,
    `25-27=lever_odo`, `28-30=lever_gnss`。
  - data2 post-GNSS 冻结：`freeze(scale+mounting)` improved；`mounting_roll` post std 为 0。
  - data4 GNSS30 冻结：best `s0m1l1` improved；`freeze_lever` 相对均衡但改善有限。
- decision:
  - data4 InEKF 最优开关采用 `p0_g1_i0`，基线与 GNSS30 以该开关为准。
  - HYP-3 已闭合；ISSUE-005/006 以证据闭合，无需代码改动。
  - 四份结果文档已重生并可编译通过。
- next_step:
  - 解释 data4 GNSS30 下 InEKF 相对 ESKF 的退化原因，并明确是否采用冻结策略作为最终报告口径。

### session_id: 20260306-1003-inekf-tex-code-audit-r1

- timestamp: 2026-03-06 10:03 (local)
- objective: 逐条检查程序中的 InEKF 实现是否符合 `算法文档/可观性分析讨论与InEKF算法.tex` 的推导。
- scope:
  - 对照 `A_c/B_\theta/G/H/Inject/Joseph/reset` 六条主链路。
  - 只做源码/文档一致性审查，不改动估计算法数值逻辑。
- changed_files:
  - `output/review/20260306-inekf-tex-code-audit-r1/inekf_tex_code_audit.md`
  - `walkthrough.md`
  - `算法文档/walkthrough.md`
- configs:
  - N/A（代码/文档审查）
- commands:
  - `rg -n` 定位 `walkthrough.md`、`算法文档/可观性分析讨论与InEKF算法.tex` 与 InEKF 相关源码
  - `Get-Content` 分段核对 `include/core/eskf.h`, `src/core/ins_mech.cpp`, `src/core/measurement_models_uwb.cpp`, `src/core/eskf_engine.cpp`
  - `New-Item -ItemType Directory -Force output/review/20260306-inekf-tex-code-audit-r1`
- artifacts:
  - `output/review/20260306-inekf-tex-code-audit-r1/inekf_tex_code_audit.md`
- metrics:
  - 检查项 `9`
  - exact match `1`（Joseph）
  - partial match `3`
  - mismatch/undocumented `5`
- artifact_mtime:
  - `output/review/20260306-inekf-tex-code-audit-r1/inekf_tex_code_audit.md`: 2026-03-06 10:03
- config_hash_or_mtime:
  - `算法文档/可观性分析讨论与InEKF算法.tex`: workspace 当前版本（直接对照）
- dataset_time_window:
  - N/A（本会话未运行解算）
- result_freshness_check:
  - pass（结论基于当前 workspace 源码与同会话生成的审查报告）
- observability_notes:
  - 状态块映射：Lie 核心审查聚焦 `kPos/kVel/kAtt`；外参与尺度块聚焦 `21=odo_scale`, `22-24=mounting`, `25-27=lever_odo`, `28-30=lever_gnss`。
  - GNSS 更新窗口：源码中 `GNSS_POS` 的 InEKF Jacobian 实际使用 `p_local + lever_ned` 姿态项与 `C_bn` 杆臂列；这意味着 `28-30` 并非文档当前写法下的 `I_3` 扩展列。
  - ODO/NHC 更新窗口：未冻结/未消融任何状态块；InEKF 分支将直接姿态列置零，但 `21-30` 相关外参列仍通过当前 `C_bn/v/omega` 参与线性化；本会话为静态审查，improved/degraded/neutral 记为 `neutral`。
  - `fusion.ablation.*`、`fusion.post_gnss_ablation.*`、`fusion.gnss_schedule.*`、`fusion.uwb_anchor_schedule.*` 本会话均未改动。
- decision:
  - 当前程序应定性为“RI-inspired hybrid ESKF”，而非 `tex` 当前正文那种直接在 `SE_2(3)` Lie 核心坐标上运行的逐式实现。
  - `ISSUE-007-inekf-tex-code-gap` 打开；`HYP-1/HYP-6` 的机理解释得到源码侧补强，但尚未替代时段量化证据。
- next_step:
  - 二选一收敛：要么把 `算法文档/可观性分析讨论与InEKF算法.tex` 改写为当前 NED/ECEF 混合实现的真实公式，要么把代码重构到文中 `SE_2(3)` / invariant residual / reset Jacobian 口径。

### session_id: 20260306-1012-inekf-architecture-choice

- timestamp: 2026-03-06 10:12 (local)
- objective: 基于“目标是实现尽可能避免可观性泄露的 IEKF”给出架构选择建议。
- scope:
  - 不改代码；只在前一轮源码-文档审查基础上做路线决策。
  - 明确“文档贴代码”与“代码贴文档”的适用场景。
- changed_files:
  - `walkthrough.md`
- configs:
  - N/A（架构建议）
- commands:
  - `Get-Content walkthrough.md -Tail 80`
  - `Get-Date -Format 'yyyyMMdd-HHmm'`
- artifacts:
  - N/A
- metrics:
  - N/A（决策性会话，无新增数值实验）
- artifact_mtime:
  - N/A
- config_hash_or_mtime:
  - 基于 `20260306-1003-inekf-tex-code-audit-r1` 审查结论作决策
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（建议直接基于同会话最新源码审查结果）
- observability_notes:
  - 若目标是“严格的 IEKF 可观性优势”，应把主目标限定为 Lie 核心 `kPos/kVel/kAtt` 及满足不变残差条件的量测通道；当前 `21-30` 外参/尺度块与 ODO/NHC 通道仍会通过当前 `C_bn/v/omega` 进入线性化，难以宣称“全 31 维无泄露”。
  - 因此推荐主线转向“代码贴理论”：新实现真正的 Lie-core IEKF，并把现有混合实现保留为 baseline；当前实现只适合作为 `neutral` 对照，不适合作为最终“无可观性泄露 IEKF”口径。
- decision:
  - 选择“代码贴文档/理论”作为主线更合适；同时短期应把文档显式注明当前实现只是 `RI-inspired hybrid ESKF`，避免研究结论混淆。
- next_step:
  - 设计一个最小可验证的 true-IEKF 版本：先收缩到 Lie 核心 + GNSS_POS invariant residual，再逐步接回 Euclidean 扩展状态与 ODO/NHC。

### session_id: 20260306-1128-true-iekf-refactor-and-compare-r1

- timestamp: 2026-03-06 11:28 (local)
- objective: 按“代码贴理论”主线落地一个更接近文档推导的 `true_iekf` 分支，并与现有 hybrid RI 实现做 fresh 对比。
- scope:
  - 代码：在不破坏现有 hybrid 基线的前提下，引入 `true_iekf_mode`。
  - 实验：data2/data4 × baseline/GNSS30 × hybrid/true 共 8 组 fresh 运行。
  - 报告：输出研究报告与指标汇总。
- changed_files:
  - `include/app/fusion.h`
  - `include/core/eskf.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/ins_mech.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `src/core/eskf_engine.cpp`
  - `config_data2_baseline_true_iekf.yaml`
  - `config_data2_gnss30_true_iekf.yaml`
  - `config_data4_baseline_true_iekf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/20260306-true-iekf-refactor-r1/metrics_summary.csv`
  - `output/review/20260306-true-iekf-refactor-r1/metrics_summary.json`
  - `output/review/20260306-true-iekf-refactor-r1/summary.md`
  - `.research/20260306-true-iekf-refactor-r1/polished_report.md`
  - `walkthrough.md`
- configs:
  - hybrid: `config_data2_baseline_inekf_best.yaml`, `config_data2_gnss30_inekf_best.yaml`, `config_data4_baseline_inekf_best.yaml`, `config_data4_gnss30_inekf_best.yaml`
  - true: `config_data2_baseline_true_iekf.yaml`, `config_data2_gnss30_true_iekf.yaml`, `config_data4_baseline_true_iekf.yaml`, `config_data4_gnss30_true_iekf.yaml`
- commands:
  - `cmake --build build --config Release --target eskf_fusion`
  - `build/Release/eskf_fusion.exe --config config_data2_baseline_inekf_best.yaml`
  - `build/Release/eskf_fusion.exe --config config_data2_baseline_true_iekf.yaml`
  - `build/Release/eskf_fusion.exe --config config_data2_gnss30_inekf_best.yaml`
  - `build/Release/eskf_fusion.exe --config config_data2_gnss30_true_iekf.yaml`
  - `build/Release/eskf_fusion.exe --config config_data4_baseline_inekf_best.yaml`
  - `build/Release/eskf_fusion.exe --config config_data4_baseline_true_iekf.yaml`
  - `build/Release/eskf_fusion.exe --config config_data4_gnss30_inekf_best.yaml`
  - `build/Release/eskf_fusion.exe --config config_data4_gnss30_true_iekf.yaml`
  - `python` 提取 fresh log 中的 `RMSE3D` / `||H_att||_F` 统计并生成 `metrics_summary.*`
- artifacts:
  - `output/review/20260306-true-iekf-refactor-r1/data2_baseline_hybrid.log`
  - `output/review/20260306-true-iekf-refactor-r1/data2_baseline_true.log`
  - `output/review/20260306-true-iekf-refactor-r1/data2_gnss30_hybrid.log`
  - `output/review/20260306-true-iekf-refactor-r1/data2_gnss30_true.log`
  - `output/review/20260306-true-iekf-refactor-r1/data4_baseline_hybrid.log`
  - `output/review/20260306-true-iekf-refactor-r1/data4_baseline_true.log`
  - `output/review/20260306-true-iekf-refactor-r1/data4_gnss30_hybrid.log`
  - `output/review/20260306-true-iekf-refactor-r1/data4_gnss30_true.log`
  - `output/review/20260306-true-iekf-refactor-r1/metrics_summary.csv`
  - `output/review/20260306-true-iekf-refactor-r1/summary.md`
  - `.research/20260306-true-iekf-refactor-r1/polished_report.md`
- metrics:
  - data2 baseline RMSE3D: hybrid/true `1.723701 / 1.228326`
  - data2 GNSS30 RMSE3D: hybrid/true `88.890702 / 201.158876`
  - data4 baseline RMSE3D: hybrid/true `0.876828 / 0.913143`
  - data4 GNSS30 RMSE3D: hybrid/true `73.715425 / 115.499686`
  - `GNSS_POS ||H_att||_F max`:
    - data2 baseline `2354.037209 / 2.309982`
    - data2 GNSS30 `2354.037209 / 1.801013`
    - data4 baseline `3.592688 / 3.760363`
    - data4 GNSS30 `3.076180 / 3.400609`
- artifact_mtime:
  - `output/review/20260306-true-iekf-refactor-r1/metrics_summary.csv`: 2026-03-06
  - `.research/20260306-true-iekf-refactor-r1/polished_report.md`: 2026-03-06
- config_hash_or_mtime:
  - true 配置由对应 `*_inekf_best.yaml` fresh 派生，并显式加入 `fusion.fej.true_iekf_mode: true`
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（8 组运行、日志与汇总均在本会话 fresh 生成）
- observability_notes:
  - 状态块 `kPos/kVel/kAtt`：`true_iekf` 将 Lie 核心改为更接近 body-frame invariant 语义；在 GNSS 全程窗口（data2 baseline）表现 improved，且 `GNSS_POS` 姿态列范数从 `2354` 压到 `2.31`。
  - 状态块 `21-30`：仍按 Euclidean/hybrid 方式与 ODO/NHC、GNSS_VEL 耦合；在 post-GNSS 稀疏窗口（data2/data4 GNSS30）整体表现 degraded，说明非不变通道与扩展块成为新的主误差源。
  - 传感器窗口：GNSS 连续可用时，true Lie-core 改造收益明显；GNSS30 稀疏窗口下，ODO/NHC/GNSS_VEL 主导阶段 true 分支尚未闭环。
  - 本会话未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*`、`fusion.gnss_schedule.*`、`fusion.uwb_anchor_schedule.*` 逻辑，只在既有 baseline/GNSS30 窗口下比较 hybrid vs true。
- decision:
  - `true_iekf` 主线代码已可运行，且明确修复了 data2 中 `GNSS_POS` 姿态列的结构性放大问题。
  - 但当前版本只能称为“Lie 核心更接近理论”的 phase-1 结果；在 sparse-GNSS 场景，ODE/NHC、GNSS_VEL 与 `21-30` 扩展块仍需继续代码贴理论改造。
- next_step:
  - 对 `true_iekf` 做 GNSS_VEL / ODO / NHC 消融矩阵，定位 GNSS30 退化主因，再继续推进 full-hybrid true IEKF 的第二阶段重构。

### session_id: 20260306-1349-true-iekf-gnss30-ablation-r1

- timestamp: 2026-03-06 13:49 (local)
- objective: 在 `true_iekf` 的 GNSS30 场景下，通过 `GNSS_VEL / ODO / NHC` 消融矩阵定位 sparse-GNSS 退化主因。
- scope:
  - 新增 `fusion.enable_gnss_velocity` 开关。
  - 仅在 `data2/data4` 的 `GNSS30 + true_iekf` 场景下做 targeted ablation。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `output/review/20260306-true-iekf-ablation-r1/cfg_*.yaml`
  - `output/review/20260306-true-iekf-ablation-r1/ablation_metrics.csv`
  - `output/review/20260306-true-iekf-ablation-r1/ablation_summary.json`
  - `output/review/20260306-true-iekf-ablation-r1/summary.md`
  - `.research/20260306-true-iekf-refactor-r1/polished_report.md`
  - `walkthrough.md`
- configs:
  - `output/review/20260306-true-iekf-ablation-r1/cfg_data2_gnss30_{full,no_gnss_vel,no_odo,no_nhc,no_odo_nhc}.yaml`
  - `output/review/20260306-true-iekf-ablation-r1/cfg_data4_gnss30_{full,no_gnss_vel,no_odo,no_nhc,no_odo_nhc}.yaml`
- commands:
  - `cmake --build build --config Release --target eskf_fusion`
  - `build/Release/eskf_fusion.exe --config output/review/20260306-true-iekf-ablation-r1/cfg_*.yaml`（10 组）
  - `python` 提取 fresh log 中的 `RMSE3D` / `||H_att||_F` 统计并生成 `ablation_metrics.*`
- artifacts:
  - `output/review/20260306-true-iekf-ablation-r1/ablation_metrics.csv`
  - `output/review/20260306-true-iekf-ablation-r1/ablation_summary.json`
  - `output/review/20260306-true-iekf-ablation-r1/summary.md`
  - `output/review/20260306-true-iekf-ablation-r1/cfg_*.log`
- metrics:
  - data2 GNSS30 full/no_gnss_vel/no_odo/no_nhc/no_odo_nhc RMSE3D:
    `201.158876 / 201.158876 / 370.355380 / 716.565952 / 4938.740955`
  - data4 GNSS30 full/no_gnss_vel/no_odo/no_nhc/no_odo_nhc RMSE3D:
    `115.499686 / 121.404109 / 669.763471 / 784.539644 / 4524.530626`
- artifact_mtime:
  - `output/review/20260306-true-iekf-ablation-r1/ablation_metrics.csv`: 2026-03-06
  - `output/review/20260306-true-iekf-ablation-r1/summary.md`: 2026-03-06
- config_hash_or_mtime:
  - ablation configs 均由对应 `*_gnss30_true_iekf.yaml` fresh 派生，仅修改 `enable_gnss_velocity` 与 `constraints.enable_odo/enable_nhc`
- dataset_time_window:
  - data2: `528076.009368` 到 `530488.900000`
  - data4: `275309.007957` 到 `277300.000000`
- result_freshness_check:
  - pass（10 组消融日志与汇总均在本会话 fresh 生成）
- observability_notes:
  - GNSS30 稀疏窗口下，关闭 `GNSS_VEL` 对 data2 无影响、对 data4 轻微 degraded，说明 `GNSS_VEL` 不是主要 blocker。
  - 关闭 `ODO` 或 `NHC` 均显著 degraded，关闭两者则灾难性 degraded；说明 road-constraint 通道是必要支撑，而非主要污染源本身。
  - 因此 `true_iekf` 当前 sparse-GNSS 问题已从 `GNSS_POS` 核心 Jacobian 迁移到 `ODO/NHC` 与 `21-30` 扩展块的 hybrid Jacobian/reset 耦合一致性。
- decision:
  - 优先级上，将 `GNSS_VEL` 从嫌疑主因降级；下一阶段应优先重推 `ODO/NHC` 与 full-state reset Jacobian。
- next_step:
  - 进入 phase-2：对 `ODO/NHC` 与 `21-30` 扩展块执行“代码贴理论”的第二阶段重构，并复测 GNSS30。

### session_id: 20260306-1600-github-sync-current-work

- timestamp: 2026-03-06 16:00 (local)
- objective: 将当前工作区中的代码与配置改动同步到 GitHub 远程仓库。
- scope:
  - 检查本地分支、远程地址、工作树与暂存状态。
  - 按当前工作树状态提交并推送到 `origin/main`。
- changed_files:
  - `walkthrough.md`
- configs:
  - N/A（仓库同步任务，无算法配置变更）
- commands:
  - `git status --short`
  - `git remote -v`
  - `git add -A`
  - `git commit -m "feat: add true-iekf mode and related configs"`
  - `git push origin main`
- artifacts:
  - Git commit（本地）
  - GitHub `origin/main`（远程）
- metrics:
  - N/A（本会话未新增解算实验）
- artifact_mtime:
  - `walkthrough.md`: 2026-03-06
- config_hash_or_mtime:
  - N/A
- dataset_time_window:
  - N/A
- result_freshness_check:
  - pass（同步动作在本会话内完成）
- observability_notes:
  - 本次为代码托管同步，无新增状态块冻结/消融实验。
  - `fusion.ablation.*`、`fusion.post_gnss_ablation.*`、`fusion.gnss_schedule.*`、`fusion.uwb_anchor_schedule.*` 未改动。
- decision:
  - 当前改动以单次提交形式同步至远程，后续 phase-2 继续在新提交基础上推进。
- next_step:
  - 基于已同步版本继续实现 `true_iekf` phase-2（`ODO/NHC` 与 `21-30` 扩展块 Jacobian/reset 重构）。

## 下一步（优先队列）

1. 基于本次已同步版本进入 `true_iekf` phase-2：显式重推 `ODO/NHC` 与 `21-30` 扩展块在 true 核心坐标下的 Jacobian/Reset。
2. phase-2 完成后复查 `GNSS_VEL` 是否仍需专门处理；当前证据显示它不是首要 blocker。
3. 修正文档口径：把当前 hybrid 分支与新的 `true_iekf` 分支分章写清，避免实验口径混写。
4. 完成 data4 GNSS30 退化机理说明，并决定是否将最佳冻结策略 `s0m1l1` 写入最终结论。
5. 明确 GNSS 速度噪声敏感性在报告中的风险说明范围（主口径保持 `GNSS_converted.txt`）。
