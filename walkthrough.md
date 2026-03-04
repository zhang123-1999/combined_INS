# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-04`

## 项目快照

### 长期目标

- 对比不同组合导航算法方案。
- 研究不同约束、调度与消融条件下状态量的可观性。

### 当前基线与代码事实

- 基线配置: `config_data2_baseline_eskf.yaml`
- 状态维度: `kStateDim = 31`
- 主要滤波模式:
  - 标准 ESKF（`fusion.fej.enable: false`）
  - InEKF 开关模式（`fusion.fej.enable: true`）
- 当前 data2 基线的核心组合:
  - INS + GNSS + ODO + NHC（UWB 在基线配置中关闭）

### 31维状态拆分备忘

- `p(3), v(3), att(3), ba(3), bg(3), sg(3), sa(3), odo_scale(1), mounting(3), lever_odo(3), lever_gnss(3)`

### 融合主流程顺序（对应代码实现）

1. IMU 预测
2. ZUPT
3. 重力对准诊断
4. NHC
5. ODO
6. UWB
7. GNSS（含时间调度关断与可选 post-GNSS 消融）
8. 状态诊断与结果记录

## 实验索引（Experiment Registry）

| exp_id | 日期 | 目的 | 配置 | 关键产物 | 关键指标 | 状态 | 新鲜度 |
|---|---|---|---|---|---|---|---|
| EXP-20260304-data2-baseline-eskf | 2026-03-04 | data2 标准 ESKF 基线 | `config_data2_baseline_eskf.yaml` | `SOL_data2_baseline_eskf.txt`, `output/logs_runcheck_baseline_eskf.log`, `output/result_data2_baseline_eskf/` | 日志 RMSE xyz: `0.763749 0.566040 0.770121` | 当前参考 | pass（日志与产物时间一致） |
| EXP-20260304-data2-baseline-inekf-ctrl | 2026-03-04 | data2 InEKF 开关基线 | `config_data2_baseline_inekf_ctrl.yaml` | `SOL_data2_baseline_inekf_ctrl.txt`, `output/logs_runcheck_baseline_inekf_ctrl.log`, `output/result_data2_baseline_inekf_ctrl/` | 日志 RMSE xyz: `0.763 0.566 0.769` | 当前参考 | pass（日志与产物时间一致） |
| EXP-20260304-data2-gnss30-postfix-gnsslever-eskf | 2026-03-04 | GNSS 前30%启用，后段进行 post-GNSS 状态冻结 | `config_data2_gnss30_postfix_gnsslever_eskf.yaml` | `SOL_data2_gnss30_postfix_gnsslever_eskf.txt`, `output/logs_runcheck_gnss30_postfix_gnsslever_eskf.log`, `output/result_data2_gnss30_postfix_gnsslever_eskf/` | 日志 RMSE xyz: `58.292 84.777 78.863` | 当前参考 | pass（日志与产物时间一致） |
| EXP-20260304-data2-gnss30-inekf | 2026-03-04 | InEKF 对比实验：GNSS 仅前30%启用，后段 INS/ODO/NHC | `config_data2_gnss30_inekf.yaml` | `SOL_data2_gnss30_inekf.txt`, `output/logs_runcheck_gnss30_inekf.log`, `output/result_data230_inekf/`, `output/compare_data2_gnss30_eskf_postfix_vs_inekf/` | 历史日志 RMSE xyz: `92.918 104.149 149.448`；对比 RMSE 3D: `204.488` | stale_or_conflict | conflict（被 `EXP-20260304-inekf-refactor-minrisk-debug` 同配置新日志覆盖） |
| EXP-20260304-data2-gnss30-eskf-nofreeze | 2026-03-04 | ESKF 对比实验：GNSS 仅前30%启用，后段不做 GNSS 杆臂冻结，状态照常更新 | `config_data2_gnss30_eskf_nofreeze.yaml` | `SOL_data2_gnss30_eskf_nofreeze.txt`, `output/logs_runcheck_gnss30_eskf_nofreeze.log`, `output/result_data230_eskf_nofreeze/`, `output/compare_data2_gnss30_eskf_postfix_vs_nofreeze/` | 历史日志 RMSE xyz: `58.292 84.777 78.863`；对比 RMSE 3D: `129.632` | stale_or_conflict | conflict（被 `EXP-20260304-inekf-refactor-minrisk-debug` 同配置新日志覆盖） |
| EXP-20260304-preexp-fix-regression | 2026-03-04 | 修复审查问题并执行门禁回归 | `config_data2_baseline_eskf.yaml`; `config_data2_baseline_inekf_ctrl.yaml`; `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`; `output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`; `output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml` | `output/review/20260304-1855-fix_implementation/` | baseline ESKF RMSE xyz: `0.763591 0.565924 0.770052`; baseline InEKF RMSE xyz: `0.763 0.565 0.769`; missing_truth_manual_init `EXIT_CODE=0`; invalid_gnss `EXIT_CODE=1`; maxdt_zero `EXIT_CODE=0` | 修复验证完成 | pass（构建与回归日志时间链一致） |
| EXP-20260304-inekf-refactor-minrisk-debug | 2026-03-04 | 按“代码大修”执行最小风险重构调试：FEJ 时序、H 线性化、过程模型签名一致性与回归验证 | `config_data2_baseline_eskf.yaml`; `config_data2_baseline_inekf_ctrl.yaml`; `config_data2_gnss30_inekf.yaml`; `config_data2_gnss30_eskf_nofreeze.yaml` | `output/review/20260304-2016-code_overhaul_debug/baseline_eskf_final.log`; `output/review/20260304-2016-code_overhaul_debug/baseline_inekf_after_gnss_lever_guard.log`; `output/review/20260304-2016-code_overhaul_debug/gnss30_inekf_final.log`; `output/review/20260304-2016-code_overhaul_debug/gnss30_eskf_nofreeze_final.log` | baseline ESKF RMSE xyz: `0.763591 0.565924 0.770052`; baseline InEKF RMSE xyz: `0.763260 0.565492 0.769397`; GNSS30 InEKF RMSE xyz: `49.895 72.609 102.467`; GNSS30 ESKF RMSE xyz: `57.533 82.961 78.063` | stale_or_conflict | conflict（该轮 InEKF 实现依赖 FEJ 冻结语义，已被 `EXP-20260304-inekf-rewrite-no-fej` 按 `代码大修2.md` 覆盖） |
| EXP-20260304-inekf-rewrite-no-fej | 2026-03-04 | 按 `代码大修2.md` 重写 InEKF：去除 FEJ 冻结依赖，仅保留 InEKF 开关；回归 4 组配置并按“绝对误差优先”评估 | `config_data2_baseline_eskf.yaml`; `config_data2_baseline_inekf_ctrl.yaml`; `config_data2_gnss30_inekf.yaml`; `config_data2_gnss30_eskf_nofreeze.yaml` | `output/review/20260304-2104-inekf-rewrite/baseline_eskf_final.log`; `output/review/20260304-2104-inekf-rewrite/baseline_inekf_ctrl_final.log`; `output/review/20260304-2104-inekf-rewrite/gnss30_inekf_final.log`; `output/review/20260304-2104-inekf-rewrite/gnss30_eskf_nofreeze_final.log` | baseline ESKF RMSE xyz: `0.763591 0.565924 0.770052`; baseline InEKF RMSE xyz: `0.763144 0.565423 0.769264`; GNSS30 InEKF RMSE xyz: `89.206 100.587 142.126`; GNSS30 ESKF RMSE xyz: `57.533 82.961 78.063` | stale_or_conflict | conflict（被 `EXP-20260304-inekf-ri-realization-codeoverhaul3` 覆盖） |
| EXP-20260304-inekf-ri-realization-codeoverhaul3 | 2026-03-04 | 按 `代码大修3.md` 实现 Right-Invariant InEKF（RI 注入逆变换 + 过程模型重写 + 量测模型 RI 雅可比）并回归 4 组配置 | `config_data2_baseline_eskf.yaml`; `config_data2_baseline_inekf_ctrl.yaml`; `config_data2_gnss30_inekf.yaml`; `config_data2_gnss30_eskf_nofreeze.yaml` | `output/review/20260304-2312-inekf-ri-implementation/baseline_eskf.log`; `output/review/20260304-2312-inekf-ri-implementation/baseline_inekf_ctrl.log`; `output/review/20260304-2312-inekf-ri-implementation/gnss30_inekf.log`; `output/review/20260304-2312-inekf-ri-implementation/gnss30_eskf_nofreeze.log` | baseline ESKF RMSE xyz: `0.763591 0.565924 0.770052`; baseline InEKF RMSE xyz: `0.953258 0.962653 1.014938`; GNSS30 InEKF RMSE xyz: `40.557 89.686 66.555` (RMSE3D `118.819`); GNSS30 ESKF RMSE xyz: `57.533 82.961 78.063` (RMSE3D `127.618`) | 当前参考（RI InEKF 实现） | pass（编译成功，4 组日志时间链一致；并记录 baseline InEKF 新回归风险） |
| EXP-20260304-inekf-ri-gvelgyro-fix | 2026-03-04 | 按 `ISSUE-005` 假设在 InEKF 过程噪声映射中补充 `G(kVel,gyro)` RI 耦合项并回归 4 组配置 | `config_data2_baseline_eskf.yaml`; `config_data2_baseline_inekf_ctrl.yaml`; `config_data2_gnss30_inekf.yaml`; `config_data2_gnss30_eskf_nofreeze.yaml` | `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_eskf.log`; `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_inekf_ctrl.log`; `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_inekf.log`; `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_eskf_nofreeze.log` | baseline ESKF RMSE xyz: `0.763591 0.565924 0.770052`; baseline InEKF RMSE xyz: `1.044499 1.277957 1.263830` (RMSE3D `2.078802`); GNSS30 InEKF RMSE xyz: `47.471 96.176 126.611` (RMSE3D `165.933`); GNSS30 ESKF RMSE xyz: `57.533 82.961 78.063` (RMSE3D `127.618`) | 当前代码快照（待调试） | pass（编译成功，4 组日志时间链一致；结果与“仅补 G_vel,gyro 可修复”假设冲突） |

## 已知不一致项（Known Inconsistencies）

| issue_id | 发现日期 | 描述 | 影响文件 | 影响 | 处理状态 |
|---|---|---|---|---|---|
| ISSUE-001-metric-conflict-baseline-inekf | 2026-03-04 | `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt` 中 InEKF 基线指标与同日 runcheck 日志不一致。 | `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt`, `output/compare_data2_baseline_eskf_vs_inekf_ctrl/metrics.csv`, `output/logs_runcheck_baseline_inekf_ctrl.log` | 可能导致算法对比结论被陈旧结果污染。 | open |
| ISSUE-005-baseline-inekf-ri-regression | 2026-03-04 | RI 实现后 baseline InEKF 与 ESKF 存在显著偏离，且补 `G(kVel,gyro)` 后进一步扩大（`1.044/1.278/1.264` vs `0.764/0.566/0.770`）。 | `src/core/eskf_engine.cpp`, `src/core/ins_mech.cpp`, `src/core/measurement_models_uwb.cpp`, `output/review/20260304-2312-inekf-ri-implementation/baseline_inekf_ctrl.log`, `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_inekf_ctrl.log` | 影响基线稳定性与可解释性，说明当前 RI 过程/量测噪声映射仍存在不一致。 | open |
| ISSUE-006-ri-process-noise-mapping-incomplete | 2026-03-04 | 当前仅补 `G(kVel,gyro)` 后，GNSS30 InEKF 从 RMSE3D `118.819` 恶化到 `165.933`，提示 RI 坐标过程噪声映射可能需成体系修正（例如与 `ξ_p` 或姿态噪声约定联动）。 | `src/core/ins_mech.cpp`, `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_inekf.log` | 可能导致 InEKF 协方差传播与量测线性化不自洽。 | open |
| ISSUE-004-inekf-fej-semantic-conflict | 2026-03-04 | `EXP-20260304-inekf-refactor-minrisk-debug` 的 InEKF 路径依赖 FEJ 冻结语义，与 `代码大修2.md` 的“无 FEJ InEKF”要求冲突。 | `include/core/eskf.h`, `src/core/ins_mech.cpp`, `src/core/measurement_models_uwb.cpp`, `src/app/pipeline_fusion.cpp`, `output/review/20260304-2016-code_overhaul_debug/*` | 旧实验中 InEKF 指标（尤其 GNSS30）不可直接作为“无 FEJ InEKF”结论。 | resolved（2026-03-04，见 `EXP-20260304-inekf-rewrite-no-fej`） |
| ISSUE-002-manual-init-missing-truth-crash | 2026-03-04 | 手动初始化且真值文件缺失时，程序在评估阶段崩溃（access violation）。 | `src/app/evaluation.cpp`, `src/utils/math_utils.cpp`, `output/review/20260304-1855-fix_implementation/missing_truth_manual_init.log` | 已改为受控退化：真值缺失时跳过 RMSE 并继续输出结果。 | resolved（2026-03-04，回归通过） |
| ISSUE-003-gnss-load-nonfatal | 2026-03-04 | GNSS 文件列数不足时，程序仅打印错误但仍返回成功并输出结果。 | `src/app/pipeline_fusion.cpp`, `output/review/20260304-1855-fix_implementation/invalid_gnss_clean.log`, `output/review/20260304-1855-fix_implementation/invalid_gnss_clean.exit.txt` | 已改为 fail-fast：GNSS 数据非法时直接非零退出。 | resolved（2026-03-04，回归通过） |

## 开放假设（Open Hypotheses）

| hyp_id | 假设 | 当前证据 | 下一步验证 | 状态 |
|---|---|---|---|---|
| HYP-1 | 在当前控制设置下，data2 的 InEKF 与 ESKF 结果应接近。 | `EXP-20260304-inekf-ri-gvelgyro-fix` baseline：InEKF `1.044499 1.277957 1.263830`，ESKF `0.763591 0.565924 0.770052`，差异进一步扩大。 | 继续定位 `ISSUE-005/006`：联动检查 RI 量测/过程噪声映射的一致性而非单点修补。 | open（conflict） |
| HYP-2 | 在当前 data2 设置中，仅冻结 `gnss_lever_arm` 的 post-GNSS 策略不足以保证后段稳定精度。 | 新 RI 版本 GNSS30：InEKF `40.557 89.686 66.555`（RMSE3D `118.819`）优于 ESKF `57.533 82.961 78.063`（RMSE3D `127.618`）。 | 将结论转为“实现相关”，继续补做冻结矩阵确认可观性收益是否稳健。 | rejected（被新实现证据推翻） |
| HYP-3 | 外参相关状态的可观性对 GNSS 调度和运动激励高度敏感。 | 代码中已有弱激励冻结、ablation 与 consistency 统计机制。 | 运行受控消融矩阵并跟踪状态轨迹与一致性指标。 | open |
| HYP-4 | RI GNSS 位置雅可比中的 `p_ned_local` 项可能在全程 GNSS 场景引入过强姿态耦合，从而拉低 baseline InEKF。 | `baseline_inekf_ctrl.log` 中 `||H_att||_F` 随时间增大到 `1e3` 量级，且 RMSE 较历史显著劣化。 | 设计 A/B：保留 RI 注入与 `F_vφ`，仅切换 `H_pos` 的 `p_ned_local` 项并比较 baseline/GNSS30 两场景。 | open |
| HYP-5 | 仅补 `G(kVel,gyro)` 缺失项即可修复 `ISSUE-005`。 | `EXP-20260304-inekf-ri-gvelgyro-fix` 显示 baseline 与 GNSS30 InEKF 均劣化（baseline RMSE3D `2.079`；GNSS30 RMSE3D `165.933`）。 | 转向系统性 A/B：噪声映射符号约定、`ξ_p` 相关项及量测噪声一致性联合验证。 | rejected |

## 会话日志（Session Log）

### 模板（每次任务复制一条）

```text
session_id: YYYYMMDD-HHMM-task_slug
timestamp: local datetime
objective:
scope:
changed_files:
configs:
commands:
artifacts:
metrics:
  rmse_xyz:
  rmse_3d:
  p95_3d:
  other:
artifact_mtime:
config_hash_or_mtime:
dataset_time_window:
result_freshness_check: pass|fail (reason)
observability_notes:
decision:
next_step:
```

### session_id: 20260304-1435-agent-doc-bootstrap

- timestamp: 2026-03-04 14:35 (local)
- objective: 增加持久化协作规范与跨会话连续性文档。
- scope:
  - 创建 `AGENT.md`
  - 创建初始版 `walkthrough.md`
  - 登记当前基线背景与已知冲突
- changed_files:
  - `AGENT.md`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_postfix_gnsslever_eskf.yaml`
- commands:
  - 仓库结构与关键文件扫描
  - `output/logs_runcheck_*.log` 基线日志检查
  - `output/compare_data2_baseline_eskf_vs_inekf_ctrl/*` 冲突检查
- artifacts:
  - `AGENT.md`
  - `walkthrough.md`
- metrics:
  - baseline ESKF 日志 RMSE xyz: `0.763749 0.566040 0.770121`
  - baseline InEKF 日志 RMSE xyz: `0.763 0.566 0.769`
  - GNSS30 postfix 日志 RMSE xyz: `58.292 84.777 78.863`
- artifact_mtime:
  - `output/logs_runcheck_baseline_eskf.log`: 2026-03-04
  - `output/logs_runcheck_baseline_inekf_ctrl.log`: 2026-03-04
  - `output/logs_runcheck_gnss30_postfix_gnsslever_eskf.log`: 2026-03-04
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04
  - `config_data2_baseline_inekf_ctrl.yaml`: 2026-03-04
  - `config_data2_gnss30_postfix_gnsslever_eskf.yaml`: 2026-03-04
- dataset_time_window:
  - baseline 运行窗口约为 `528076.009368` 到 `530488.900000`（来自日志）
- result_freshness_check: runcheck 日志为 pass；历史 compare 汇总为 fail（见 ISSUE-001）。
- observability_notes:
  - 已登记后续可观性分析关键开关:
    - `fusion.ablation.*`
    - `fusion.post_gnss_ablation.*`
    - `fusion.gnss_schedule.*`
    - pipeline 中 consistency 相关统计
- decision:
  - 在 compare 汇总重生前，以 runcheck 日志作为当前事实来源。
- next_step:
  - 使用同步的新 SOL 产物重算 ESKF vs InEKF 对比汇总。

### session_id: 20260304-1505-agent-md-fix-bootstrap-and-context

- timestamp: 2026-03-04 15:05 (local)
- objective: 修复 AGENT.md 的循环依赖风险与上下文耗尽应急缺口。
- scope:
  - 修正 Section 3：内联 `walkthrough.md` 初始化模板
  - 新增上下文即将耗尽时的强制收口协议
- changed_files:
  - `AGENT.md`
- configs:
  - 无
- commands:
  - 文档检查与补丁更新
- artifacts:
  - `AGENT.md`
- metrics:
  - 文档规范修订任务，无数值指标
- artifact_mtime:
  - `AGENT.md`: 2026-03-04
- config_hash_or_mtime:
  - N/A
- dataset_time_window:
  - N/A
- result_freshness_check: pass（文档规则变更已落地并可读）
- observability_notes:
  - 无（本次仅为协作规范修订）
- decision:
  - `walkthrough.md` 缺失时，agent 直接按 AGENT.md 内联模板创建，避免引用不存在 schema。
  - 上下文接近上限时，agent 必须立即执行 End Checklist 并在 `next_step` 写明中断点与恢复首步。
- next_step:
  - 在后续实际开发任务中执行一次“中断恢复演练”，验证应急协议可用性。

### session_id: 20260304-1510-add-gnss30-inekf-compare

- timestamp: 2026-03-04 15:10 (local)
- objective: 新增 InEKF 对比实验（GNSS 前30% + 后段 INS/ODO/NHC）并完成解算与绘图。
- scope:
  - 新建 `config_data2_gnss30_inekf.yaml`
  - 运行融合主程序并保存 runcheck 日志
  - 生成单实验图集与 ESKF vs InEKF 对比图
  - 修复 compare 脚本对无表头 LLA 真值的识别
- changed_files:
  - `config_data2_gnss30_inekf.yaml`
  - `scripts/analysis/compare_fej_vs_standard.py`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_inekf.yaml`
  - `config_data2_gnss30_postfix_gnsslever_eskf.yaml`（对比参考）
- commands:
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_inekf.yaml`
  - `python plot_navresult.py SOL_data2_gnss30_inekf.txt`
  - `python scripts/analysis/compare_fej_vs_standard.py --std SOL_data2_gnss30_postfix_gnsslever_eskf.txt --fej SOL_data2_gnss30_inekf.txt --truth dataset/data2_converted/POS_converted.txt --outdir output/compare_data2_gnss30_eskf_postfix_vs_inekf`
- artifacts:
  - `SOL_data2_gnss30_inekf.txt`
  - `output/logs_runcheck_gnss30_inekf.log`
  - `output/result_data230_inekf/`
  - `output/compare_data2_gnss30_eskf_postfix_vs_inekf/summary.txt`
  - `output/compare_data2_gnss30_eskf_postfix_vs_inekf/01_trajectory_compare.png`
  - `output/compare_data2_gnss30_eskf_postfix_vs_inekf/02_position_error_norm_compare.png`
- metrics:
  - InEKF 日志 RMSE xyz: `92.918 104.149 149.448`
  - InEKF 对比 RMSE 3D / P95 3D: `204.488 / 399.304` m
  - 参考 ESKF(postfix) RMSE xyz / RMSE 3D: `58.292 84.777 78.863` / `129.632` m
- artifact_mtime:
  - `config_data2_gnss30_inekf.yaml`: 2026-03-04 15:08:24
  - `SOL_data2_gnss30_inekf.txt`: 2026-03-04 15:09:08
  - `output/logs_runcheck_gnss30_inekf.log`: 2026-03-04 15:09:08
  - `output/result_data230_inekf/11_ecef_position.png`: 2026-03-04 15:09:30
  - `output/compare_data2_gnss30_eskf_postfix_vs_inekf/summary.txt`: 2026-03-04 15:10:08
- config_hash_or_mtime:
  - `config_data2_gnss30_inekf.yaml`: 2026-03-04 15:08:24
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`（来自 runcheck 日志）
- result_freshness_check: pass（日志、SOL、图像均为本次任务时间窗内新生成，时间链一致）
- observability_notes:
  - 状态块 `21-30`（`odo_scale`, `mounting`, `lever_odo`, `lever_gnss`）本次均未冻结，`fusion.ablation.*` 为默认全启用。
  - `fusion.post_gnss_ablation.*` 本次未启用；GNSS split 后未追加冻结。
  - 调度窗口：`fusion.gnss_schedule.enabled=true`, `head_ratio=0.30`，GNSS 在 `t=528799.876` 后关闭；后段仅 IMU 预测 + NHC + ODO（UWB 关闭）。
  - 行为判定：相对 `GNSS30 postfix ESKF`，InEKF 版本精度明显劣化（degraded）。
- decision:
  - 新增 `EXP-20260304-data2-gnss30-inekf` 作为 GNSS30 场景的 InEKF 对比基准。
  - 当前 `head_ratio=0.30` 下，InEKF 直接切 GNSS 后漂移较大，需继续做 post-GNSS 状态冻结矩阵。
- next_step:
  - 固定 `head_ratio=0.30`，在 InEKF 下测试 post-GNSS `odo_scale/mounting/lever` 多组合冻结并对比后段漂移。

### session_id: 20260304-1522-add-gnss30-eskf-nofreeze

- timestamp: 2026-03-04 15:22 (local)
- objective: 新增 ESKF 实验（GNSS 前30%启用，后段不固定 GNSS 杆臂，所有状态照常更新）。
- scope:
  - 新建 `config_data2_gnss30_eskf_nofreeze.yaml`
  - 运行融合主程序并生成 runcheck 日志
  - 生成单实验图集与 postfix-vs-nofreeze 对比摘要
- changed_files:
  - `config_data2_gnss30_eskf_nofreeze.yaml`
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_eskf_nofreeze.yaml`
  - `config_data2_gnss30_postfix_gnsslever_eskf.yaml`（对比参考）
- commands:
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_eskf_nofreeze.yaml`
  - `python plot_navresult.py SOL_data2_gnss30_eskf_nofreeze.txt`
  - `python scripts/analysis/compare_fej_vs_standard.py --std SOL_data2_gnss30_postfix_gnsslever_eskf.txt --fej SOL_data2_gnss30_eskf_nofreeze.txt --truth dataset/data2_converted/POS_converted.txt --outdir output/compare_data2_gnss30_eskf_postfix_vs_nofreeze`
- artifacts:
  - `SOL_data2_gnss30_eskf_nofreeze.txt`
  - `output/logs_runcheck_gnss30_eskf_nofreeze.log`
  - `output/result_data230_eskf_nofreeze/`
  - `output/compare_data2_gnss30_eskf_postfix_vs_nofreeze/summary.txt`
  - `output/compare_data2_gnss30_eskf_postfix_vs_nofreeze/02_position_error_norm_compare.png`
- metrics:
  - nofreeze 日志 RMSE xyz: `58.292 84.777 78.863`
  - nofreeze RMSE 3D / P95 3D: `129.632 / 229.661` m
  - 与 postfix-gnsslever 对比：`RMSE xyz / RMSE 3D / P95 3D` 全部一致
- artifact_mtime:
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
  - `SOL_data2_gnss30_eskf_nofreeze.txt`: 2026-03-04 15:21:31
  - `output/logs_runcheck_gnss30_eskf_nofreeze.log`: 2026-03-04 15:21:31
  - `output/result_data230_eskf_nofreeze/11_ecef_position.png`: 2026-03-04 15:21:52
  - `output/compare_data2_gnss30_eskf_postfix_vs_nofreeze/summary.txt`: 2026-03-04 15:22:06
- config_hash_or_mtime:
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`（来自 runcheck 日志）
- result_freshness_check: pass（日志、SOL、绘图、对比摘要均为本次任务时间窗内生成）
- observability_notes:
  - 状态块 `21-30`（`odo_scale`, `mounting`, `lever_odo`, `lever_gnss`）在 GNSS 关断后未冻结，持续按模型演化并参与约束更新。
  - `fusion.post_gnss_ablation.enabled=false`，未施加任何 post-GNSS 状态冻结。
  - 调度窗口：`fusion.gnss_schedule.enabled=true`, `head_ratio=0.30`，GNSS 在 `t=528799.876` 后关闭；后段传感器窗口为 IMU + NHC + ODO（UWB 关闭）。
  - 行为判定：相对 `postfix-gnsslever`，本实验表现为中性（neutral），误差指标无可见变化。
- decision:
  - 新增 `EXP-20260304-data2-gnss30-eskf-nofreeze` 作为 ESKF 下“无 post-GNSS 冻结”对照基准。
  - 在 ESKF+GNSS30 场景中，单独冻结 `gnss_lever_arm` 对结果影响不显著。
- next_step:
  - 以 nofreeze 为基准，继续测试 `odo_scale/mounting/lever` 多状态 post-GNSS 冻结组合。

### session_id: 20260304-1526-push-updates-to-github

- timestamp: 2026-03-04 15:26 (local)
- objective: 将本地更新提交并上传到 GitHub 远端仓库。
- scope:
  - 汇总当前工作区改动并创建提交
  - 推送到 `origin/main`
  - 记录提交号与远端同步结果
- changed_files:
  - `walkthrough.md`
- configs:
  - N/A（本次为版本管理操作）
- commands:
  - `git add -A`
  - `git commit -m "Add GNSS30 ESKF/InEKF experiments and update configs/docs"`
  - `git push origin main`
- artifacts:
  - 远端分支更新: `origin/main`
  - 提交: `e52160735c3c54881f65b1c7e95a776e3711de3c`
- metrics:
  - N/A（本次无新增算法数值指标）
- artifact_mtime:
  - commit time: 2026-03-04 15:25:59 +0800
- config_hash_or_mtime:
  - N/A
- dataset_time_window:
  - N/A
- result_freshness_check: pass（远端已接收：`b94bd0b..e521607 main -> main`）
- observability_notes:
  - 无（本次未新增可观性实验，仅做代码同步）
- decision:
  - 当前实验与脚本更新已作为一个提交同步到 GitHub，后续实验在该提交基础上继续。
- next_step:
  - 按既定优先队列继续执行 `ISSUE-001` 与 post-GNSS 冻结矩阵实验。

### session_id: 20260304-1822-pre_experiment_audit

- timestamp: 2026-03-04 18:22 (local)
- objective: 在下一轮解算实验前完成全仓系统审查并给出门禁结论。
- scope:
  - 全仓静态审查（C++ 主链路、配置解析、Python 分析脚本）
  - 构建与脚本语法检查
  - 关键异常输入复现（缺失真值、损坏 GNSS、`max_dt` 语义）
  - 生成 `findings/evidence/go_no_go` 审查产物
- changed_files:
  - `output/review/20260304-1822-pre_experiment_audit/findings.md`
  - `output/review/20260304-1822-pre_experiment_audit/findings.csv`
  - `output/review/20260304-1822-pre_experiment_audit/evidence.md`
  - `output/review/20260304-1822-pre_experiment_audit/go_no_go.md`
  - `output/review/20260304-1822-pre_experiment_audit/build.log`
  - `output/review/20260304-1822-pre_experiment_audit/py_compile.log`
  - `output/review/20260304-1822-pre_experiment_audit/run_missing_truth.log`
  - `output/review/20260304-1822-pre_experiment_audit/run_missing_truth_manual_init.exit.txt`
  - `output/review/20260304-1822-pre_experiment_audit/run_invalid_gnss.log`
  - `output/review/20260304-1822-pre_experiment_audit/run_invalid_gnss.exit.txt`
  - `output/review/20260304-1822-pre_experiment_audit/run_maxdt_zero.log`
  - `output/review/20260304-1822-pre_experiment_audit/run_maxdt_zero.exit.txt`
  - `output/review/20260304-1822-pre_experiment_audit/quat_delta_sign_case.txt`
  - `output/review/20260304-1822-pre_experiment_audit/restore_baseline.log`
  - `output/review/20260304-1822-pre_experiment_audit/restore_baseline.exit.txt`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`（复现模板来源）
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`
- commands:
  - `cmake --build build --config Release`
  - `python -m py_compile <all .py files>`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth.yaml`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`（恢复基线 SOL 产物）
- artifacts:
  - `output/review/20260304-1822-pre_experiment_audit/findings.md`
  - `output/review/20260304-1822-pre_experiment_audit/findings.csv`
  - `output/review/20260304-1822-pre_experiment_audit/evidence.md`
  - `output/review/20260304-1822-pre_experiment_audit/go_no_go.md`
  - `output/review/20260304-1822-pre_experiment_audit/restore_baseline.log`
- metrics:
  - findings 计数: `P0=2, P1=1, P2=2`
  - malformed GNSS 复现 RMSE xyz: `85.653040 248.759693 219.739878`
  - missing truth + manual init 退出码: `-1073741819`
  - baseline 恢复运行 RMSE xyz: `0.763749 0.566040 0.770121`
- artifact_mtime:
  - `output/review/20260304-1822-pre_experiment_audit/findings.md`: 2026-03-04 18:30:41
  - `output/review/20260304-1822-pre_experiment_audit/findings.csv`: 2026-03-04 18:31:15
  - `output/review/20260304-1822-pre_experiment_audit/evidence.md`: 2026-03-04 18:33:54
  - `output/review/20260304-1822-pre_experiment_audit/go_no_go.md`: 2026-03-04 18:32:27
  - `output/review/20260304-1822-pre_experiment_audit/run_invalid_gnss.log`: 2026-03-04 18:28:34
  - `output/review/20260304-1822-pre_experiment_audit/run_missing_truth_manual_init.exit.txt`: 2026-03-04 18:27:48
  - `output/review/20260304-1822-pre_experiment_audit/restore_baseline.log`: 2026-03-04 18:34:35
- config_hash_or_mtime:
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth.yaml`: 2026-03-04 18:24:43
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`: 2026-03-04 18:24:57
  - `output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`: 2026-03-04 18:26:13
  - `output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`: 2026-03-04 18:29:58
- dataset_time_window:
  - 复现配置沿用 data2 基线窗口：约 `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（审查日志、退出码与报告文件时间链一致）
- observability_notes:
  - 本次任务以工程健壮性门禁为主，未新增消融矩阵或状态可观性结论。
  - 涉及状态块 `21-30` 的行为仅做代码路径检查，未进行新的受控实验比较。
- decision:
  - 审查结论为 `NO-GO`：存在 `P0` 阻断项（`ISSUE-002`, `ISSUE-003`），不建议直接进入下一解算实验。
- next_step:
  - 先修复并回归 `ISSUE-002`（manual init 缺失真值崩溃）与 `ISSUE-003`（GNSS 载入失败非致命）。
  - 修复后复跑本次四个复现场景，再继续 `ISSUE-001` 与 post-GNSS 冻结矩阵实验。

### session_id: 20260304-1855-fix-five-issues-and-md-check

- timestamp: 2026-03-04 18:55 (local)
- objective: 修复审查中的 5 个问题并核查 `程序可能存在的问题.md` 对应代码，完成门禁回归。
- scope:
  - 修复 `F-001~F-005`（崩溃、GNSS 非致命错误、`max_dt` 语义、四元数双覆盖、GNSS 速度分支判据）
  - 核查并补齐 `程序可能存在的问题.md`：InEKF H 符号一致性运行时校验、`run_state_ablation.py` 无表头容错
  - 将剩余状态索引硬编码替换为 `StateIdx`（初始化与诊断日志）
  - 构建与复现场景回归、恢复 baseline 产物
- changed_files:
  - `include/core/eskf.h`
  - `src/app/config.cpp`
  - `src/app/diagnostics.cpp`
  - `src/app/evaluation.cpp`
  - `src/app/initialization.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `src/core/ins_mech.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `src/utils/math_utils.cpp`
  - `scripts/analysis/run_state_ablation.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`
  - `output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`
- commands:
  - `cmake --build build --config Release`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_inekf_ctrl.yaml`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`
  - `.\build\Release\eskf_fusion.exe --config output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`
  - `python` 导入 `scripts/analysis/run_state_ablation.py` 并验证 `load_sol` fallback（无表头样例）
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`（恢复 baseline SOL）
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_inekf_ctrl.yaml`（恢复 baseline SOL）
- artifacts:
  - `output/review/20260304-1855-fix_implementation/baseline_eskf.log`
  - `output/review/20260304-1855-fix_implementation/baseline_inekf_ctrl.log`
  - `output/review/20260304-1855-fix_implementation/missing_truth_manual_init.log`
  - `output/review/20260304-1855-fix_implementation/invalid_gnss_clean.log`
  - `output/review/20260304-1855-fix_implementation/invalid_gnss_clean.exit.txt`
  - `output/review/20260304-1855-fix_implementation/maxdt_zero.log`
  - `output/review/20260304-1855-fix_implementation/restore_baseline_eskf.log`
  - `output/review/20260304-1855-fix_implementation/restore_baseline_inekf_ctrl.log`
- metrics:
  - baseline ESKF RMSE xyz: `0.763591 0.565924 0.770052`（对参考差值 `< 0.001`）
  - baseline InEKF RMSE xyz: `0.763 0.565 0.769`（与参考差值在 `0.001` 量级内）
  - missing truth + manual init: `EXIT_CODE=0`，`RMSE=nan nan nan`（受控退化，不崩溃）
  - invalid GNSS: `EXIT_CODE=1`（fail-fast 生效）
  - `max_dt=0`: `EXIT_CODE=0`，运行成功
  - InEKF H 符号一致性运行时校验: `PASS`
- artifact_mtime:
  - `output/review/20260304-1855-fix_implementation/baseline_eskf.log`: 2026-03-04 18:58:16
  - `output/review/20260304-1855-fix_implementation/baseline_inekf_ctrl.log`: 2026-03-04 18:58:52
  - `output/review/20260304-1855-fix_implementation/missing_truth_manual_init.log`: 2026-03-04 18:59:24
  - `output/review/20260304-1855-fix_implementation/invalid_gnss_clean.log`: 2026-03-04 19:03:03
  - `output/review/20260304-1855-fix_implementation/maxdt_zero.log`: 2026-03-04 19:00:03
  - `output/review/20260304-1855-fix_implementation/restore_baseline_eskf.log`: 2026-03-04 19:01:47
  - `output/review/20260304-1855-fix_implementation/restore_baseline_inekf_ctrl.log`: 2026-03-04 19:02:22
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04 14:24:58
  - `config_data2_baseline_inekf_ctrl.yaml`: 2026-03-04 14:25:04
  - `output/review/20260304-1822-pre_experiment_audit/tmp_missing_truth_manual_init.yaml`: 2026-03-04 18:24:57
  - `output/review/20260304-1822-pre_experiment_audit/tmp_invalid_gnss.yaml`: 2026-03-04 18:26:13
  - `output/review/20260304-1822-pre_experiment_audit/tmp_maxdt_zero.yaml`: 2026-03-04 18:30:02
- dataset_time_window:
  - data2 基线窗口约为 `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（编译时间、回归日志时间、退出码与代码改动时间链一致）
- observability_notes:
  - 本次为工程健壮性修复，未新增可观性结论；状态块 `21-30` 的改动主要是索引统一与更新门控逻辑修正。
  - 调度窗口：回归基线为 GNSS 全程启用（未启用 `fusion.gnss_schedule.*`）；异常复现配置同 18:22 审查模板。
  - 行为判定：修复对 baseline 精度表现为中性（neutral），对异常输入处理显著改善（improved）。
- decision:
  - 解除 18:22 审查中的 `NO-GO` 阻断项（`ISSUE-002`, `ISSUE-003` 已验证关闭）。
  - `程序可能存在的问题.md` 涉及代码段已完成核查与修复落地，当前可进入下一轮实验。
- next_step:
  - 优先处理 `ISSUE-001`：重跑 baseline compare 产物并消除历史 summary 冲突，再继续可观性矩阵实验。

### session_id: 20260304-1912-write-phase1-combo-nav-tex

- timestamp: 2026-03-04 19:12 (local)
- objective: 编写第一阶段总结文档 `组合导航.tex`，对比“完整GNSS”与“前百分之30 GNSS”两组实验。
- scope:
  - 汇总 `config_data2_baseline_eskf.yaml` 与 `config_data2_gnss30_eskf_nofreeze.yaml` 参数
  - 提取两组 runcheck 指标
  - 按 01-10 图像编号将两组结果并排排版
  - 生成可直接编译的 LaTeX 文档
- changed_files:
  - `组合导航.tex`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_gnss30_eskf_nofreeze.yaml`
- commands:
  - 读取两组配置文件内容
  - 扫描 `output/result_data2_baseline_eskf/` 与 `output/result_data230_eskf_nofreeze/` 图像文件
  - 读取 `output/logs_runcheck_baseline_eskf.log` 与 `output/logs_runcheck_gnss30_eskf_nofreeze.log`
  - 写入 `组合导航.tex`
- artifacts:
  - `组合导航.tex`
- metrics:
  - 完整GNSS RMSE xyz: `0.763749 0.566040 0.770121`
  - 前百分之30 GNSS RMSE xyz: `58.292 84.777 78.863`
  - 图像对比范围: `01_trajectory_altitude` 到 `10_gnss_lever_arm`
- artifact_mtime:
  - `output/result_data2_baseline_eskf/01_trajectory_altitude.png`: 2026-03-04 14:11:01
  - `output/result_data230_eskf_nofreeze/01_trajectory_altitude.png`: 2026-03-04 15:21:47
  - `output/logs_runcheck_baseline_eskf.log`: 2026-03-04（同日生成）
  - `output/logs_runcheck_gnss30_eskf_nofreeze.log`: 2026-03-04（同日生成）
  - `组合导航.tex`: 2026-03-04
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04 14:24:58
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（配置、日志、图像均为同日可追溯产物，时间链一致）
- observability_notes:
  - 本次为文档汇总任务，未新增算法试验与可观性判定。
  - 展示的状态相关图覆盖状态块 `21-30`（`odo_scale/mounting/lever/gnss_lever`）的估计轨迹对比。
  - 调度窗口差异由 `fusion.gnss_schedule.enabled/head_ratio` 体现：完整GNSS（全程GNSS） vs 前30% GNSS（`head_ratio=0.30`）。
- decision:
  - 第一阶段总结文档已形成，满足“参数配置 + 指标对比 + 01-10 并排图”要求。
- next_step:
  - 编译 `组合导航.tex` 生成 PDF 并检查版式（图表跨页、字体、中文显示）。

### session_id: 20260304-tex-format-cleanup
- objective: 优化 `算法文档/结果文档/组合导航.tex` 格式，使其适合对外展示
- scope: 仅修改 tex 文档，不涉及代码或实验配置
- changed_files:
  - `算法文档/结果文档/组合导航.tex`
- configs: 无
- commands: 无（纯文档编辑）
- artifacts: `算法文档/结果文档/组合导航.tex`（已更新）
- metrics: 无（非算法实验）
- artifact_mtime: 2026-03-04
- config_hash_or_mtime: N/A
- dataset_time_window: N/A
- result_freshness_check: N/A
- observability_notes: 本次为文档格式任务，无可观性操作。
- decision:
  - 删除 `\lstinputlisting` 整段嵌入原始 YAML 配置（两段）及 listings/xcolor/subcaption/longtable 包引用；
  - 配置对比表改为只展示功能性差异（GNSS 调度开关、后段处理、滤波器类型），不展示工程路径/内部参数键名；
  - 结果指标表精简有效数字（4位→3位），移除内部日志路径引用；
  - 图像布局从 `0.48\textwidth` 双列并排改为每张独占一行 `0.82\textwidth`，大标题说明方案身份；
  - 添加目录 `\tableofcontents`，各小节用 `\subsection` 组织；
  - 标题/作者去除工程内部信息。
- next_step:
  - 使用 xelatex 或 lualatex 编译 `组合导航.tex` 并检查 PDF 输出。

### session_id: 20260304-2016-inekf-refactor-minrisk-debug

- timestamp: 2026-03-04 20:16 (local)
- objective: 按 `代码大修.md` 落地 RI-EKF/FEJ 相关改动，并按最小风险原则逐步回归调试。
- scope:
  - 扩展 `FejManager`（冻结比力/速度字段与初始化签名），并修复 `BuildProcessModel` 声明-实现不一致。
  - `EskfEngine::Predict` 统一传递 `fej_` 指针；保留/确认 `Correct` 的位置/速度/姿态修正防护。
  - `RunGnssUpdate` 将 FEJ 初始化前置到首次 GNSS `Correct` 之前，并增加 FEJ 初始化与 `H_att` 诊断日志。
  - GNSS 位置/速度模型补充 FEJ 感知线性化，且对 `gnss_lever_arm0≈0` 的场景增加冻结保护（避免不必要退化）。
  - 执行 `F_rφ` A/B 调试与 4 组配置回归（baseline ESKF/InEKF + GNSS30 ESKF/InEKF）。
- changed_files:
  - `include/core/eskf.h`
  - `src/core/ins_mech.cpp`
  - `src/core/eskf_engine.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
  - `config_data2_gnss30_eskf_nofreeze.yaml`
- commands:
  - `cmake --build build --config Release`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_inekf_ctrl.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_inekf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_eskf_nofreeze.yaml`
  - `F_rφ` A/B 调试日志：`baseline_eskf_frphi_zero.log`, `baseline_eskf_frphi_plus_serial.log`, `baseline_eskf_frphi_minus_serial.log`
- artifacts:
  - `output/review/20260304-2016-code_overhaul_debug/baseline_eskf_final.log`
  - `output/review/20260304-2016-code_overhaul_debug/baseline_inekf_after_gnss_lever_guard.log`
  - `output/review/20260304-2016-code_overhaul_debug/gnss30_inekf_final.log`
  - `output/review/20260304-2016-code_overhaul_debug/gnss30_eskf_nofreeze_final.log`
- metrics:
  - baseline ESKF RMSE xyz / RMSE3D: `0.763591 0.565924 0.770052` / `1.223`
  - baseline InEKF RMSE xyz / RMSE3D: `0.763260 0.565492 0.769397` / `1.222`
  - baseline InEKF - ESKF 差值 xyz: `-0.000331 -0.000432 -0.000655`
  - GNSS30 InEKF RMSE xyz / RMSE3D: `49.895 72.609 102.467` / `135.134`
  - GNSS30 ESKF RMSE xyz / RMSE3D: `57.533 82.961 78.063` / `127.618`
  - `F_rφ` 调试：`F_rφ=0` 时 baseline 恢复 `0.763591 0.565924 0.770052`；直接启用耦合项出现超过阈值的回归偏移（见 A/B 日志）。
- artifact_mtime:
  - `output/review/20260304-2016-code_overhaul_debug/baseline_eskf_final.log`: 2026-03-04 20:36:44
  - `output/review/20260304-2016-code_overhaul_debug/baseline_inekf_after_gnss_lever_guard.log`: 2026-03-04 20:36:00
  - `output/review/20260304-2016-code_overhaul_debug/gnss30_inekf_final.log`: 2026-03-04 20:37:26
  - `output/review/20260304-2016-code_overhaul_debug/gnss30_eskf_nofreeze_final.log`: 2026-03-04 20:38:09
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04 14:24:58
  - `config_data2_baseline_inekf_ctrl.yaml`: 2026-03-04 14:25:04
  - `config_data2_gnss30_inekf.yaml`: 2026-03-04 15:08:24
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
  - `include/core/eskf.h`: 2026-03-04 20:32:50
  - `src/core/ins_mech.cpp`: 2026-03-04 20:30:47
  - `src/core/eskf_engine.cpp`: 2026-03-04 20:21:34
  - `src/core/measurement_models_uwb.cpp`: 2026-03-04 20:35:05
  - `src/app/pipeline_fusion.cpp`: 2026-03-04 20:26:07
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（编译成功，4 组回归日志均为本次会话生成，时间链一致且含 FEJ 初始化前置证据）
- observability_notes:
  - 状态块 `21-30`：本次未启用显式 ablation；`gnss_lever` 相关线性化在 FEJ 路径加入“零初值保护”，避免无杠杆臂场景的虚假冻结退化。
  - 调度窗口：baseline 为 GNSS 全程启用；GNSS30 在 `t=528799.876` 后关闭 GNSS，仅保留 IMU+NHC+ODO。
  - 行为判定：baseline 精度对 ESKF 中性（neutral）；InEKF baseline 相对 ESKF 维持同量级（neutral）；GNSS30 下 InEKF 对历史结果显著改善（improved），但 3D 指标仍略劣于 ESKF（partial）。
- decision:
  - 在当前实现下，为满足“ESKF 回归误差 < 0.001m”的门禁，`F_rφ` 保持历史兼容（`0`）；相关耦合项作为后续受控开关实验处理。
  - FEJ 初始化时序修复为“首次 GNSS 更新前”，并保留运行期可追踪日志（初始化时刻、`det(C_bn_fej)`、`||H_att||_F`）。
  - GNSS H 冻结逻辑采用最小风险门控：仅在 `gnss_lever` 冻结点非零时启用冻结雅可比，防止零杠杆臂场景下不必要退化。
- next_step:
  - 用本次新 `SOL_*` 与日志重跑 `ISSUE-001` compare 产物并更新 `output/compare_data2_baseline_eskf_vs_inekf_ctrl/`。
  - 将 `F_rφ` 耦合项改为显式可控实验开关，做 A/B 对照并记录可观性与 RMSE 影响。
  - 在 `head_ratio=0.30` 下继续开展 `odo_scale/mounting/lever` post-GNSS 组合冻结矩阵，补齐 consistency 汇总。

### session_id: 20260304-2104-inekf-rewrite-no-fej

- timestamp: 2026-03-04 21:04 (local)
- objective: 按 `代码大修2.md` 将 InEKF 与 FEJ 冻结机制解耦，并基于“误差优先”执行最小风险回归调试。
- scope:
  - `FejManager` 退化为 InEKF 开关（保留兼容类型别名，不再持有冻结状态）。
  - 删除 pipeline 中 FEJ layer2 窗口与首次 GNSS FEJ 初始化生命周期。
  - 删除 GNSS 位置/速度模型中的 FEJ 冻结雅可比分支，统一使用当前 `C_bn`。
  - 过程模型保留 `InEKF F_vφ=-Skew(a_m_ned)`；对 `ESKF F_rφ` 进行 A/B，最终按误差回退为历史兼容 `0`。
  - 更新 InEKF 配置注释，标注 `fej.enable` 仅为 InEKF 开关、其余字段 deprecated/no-op。
- changed_files:
  - `include/core/eskf.h`
  - `src/core/ins_mech.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
  - `config_data2_gnss30_eskf_nofreeze.yaml`
- commands:
  - `cmake --build build --config Release`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_inekf_ctrl.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_inekf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_eskf_nofreeze.yaml`
- artifacts:
  - `output/review/20260304-2104-inekf-rewrite/baseline_eskf_final.log`
  - `output/review/20260304-2104-inekf-rewrite/baseline_inekf_ctrl_final.log`
  - `output/review/20260304-2104-inekf-rewrite/gnss30_inekf_final.log`
  - `output/review/20260304-2104-inekf-rewrite/gnss30_eskf_nofreeze_final.log`
- metrics:
  - baseline ESKF RMSE xyz / RMSE3D: `0.763591 0.565924 0.770052` / `1.223242`
  - baseline InEKF RMSE xyz / RMSE3D: `0.763144 0.565423 0.769264` / `1.222235`
  - GNSS30 InEKF RMSE xyz / RMSE3D: `89.206 100.587 142.126` / `195.640627`
  - GNSS30 ESKF RMSE xyz / RMSE3D: `57.533 82.961 78.063` / `127.618202`
  - compatibility delta（相对 `EXP-20260304-inekf-refactor-minrisk-debug`）:
    - baseline ESKF: `Δxyz = [0, 0, 0]`
    - baseline InEKF: `Δxyz = [-0.000116, -0.000069, -0.000133]`（轻微改善）
    - GNSS30 ESKF: `Δxyz = [0, 0, 0]`
    - GNSS30 InEKF: `Δxyz = [+39.311, +27.978, +39.659]`（显著劣化）
- artifact_mtime:
  - `output/review/20260304-2104-inekf-rewrite/baseline_eskf_final.log`: 2026-03-04 21:13:41
  - `output/review/20260304-2104-inekf-rewrite/baseline_inekf_ctrl_final.log`: 2026-03-04 21:13:40
  - `output/review/20260304-2104-inekf-rewrite/gnss30_inekf_final.log`: 2026-03-04 21:13:40
  - `output/review/20260304-2104-inekf-rewrite/gnss30_eskf_nofreeze_final.log`: 2026-03-04 21:13:41
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04 14:24:58
  - `config_data2_baseline_inekf_ctrl.yaml`: 2026-03-04 21:06:56
  - `config_data2_gnss30_inekf.yaml`: 2026-03-04 21:07:02
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
  - `include/core/eskf.h`: 2026-03-04 21:05:01
  - `src/core/ins_mech.cpp`: 2026-03-04 21:12:35
  - `src/core/measurement_models_uwb.cpp`: 2026-03-04 21:07:37
  - `src/app/pipeline_fusion.cpp`: 2026-03-04 21:08:11
  - `src/app/config.cpp`: 2026-03-04 21:06:51
  - `include/app/fusion.h`: 2026-03-04 21:06:45
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（构建成功，4 组最终日志均为本会话新生成且时间链一致）
- observability_notes:
  - 状态块 `21-30`：本次未启用显式 `fusion.ablation.*`；InEKF 改动主要影响 `kGnssLever` 与 `kLever` 相关雅可比线性化路径（由 FEJ 冻结切换为当前状态线性化）。
  - 调度窗口：`fusion.gnss_schedule.head_ratio=0.30` 时，GNSS 在 `t=528799.876` 后关闭，仅保留 IMU + NHC + ODO 更新。
  - 行为判定：baseline 阶段 InEKF 与 ESKF 均为中性（neutral）；GNSS30 阶段无 FEJ InEKF 明显劣化（degraded）。
- decision:
  - 接受“无 FEJ InEKF”结构性改造，并保留 `ESKF F_rφ=0` 的最小风险回退以避免绝对误差恶化。
  - 将 `EXP-20260304-inekf-refactor-minrisk-debug` 标记为语义冲突历史参考（FEJ 依赖路径）。
- next_step:
  - 在无 FEJ 前提下继续调试 InEKF（优先检查 GNSS30 段 ODO/NHC 姿态耦合与 GNSS 速度模型一致性）。
  - 重跑 `ISSUE-001` compare 汇总，更新 `output/compare_data2_baseline_eskf_vs_inekf_ctrl/` 与冲突状态。

### session_id: 20260304-2220-push-updated-code-to-github

- timestamp: 2026-03-04 22:20 (local)
- objective: 将当前本地更新后的代码与文档同步到 GitHub `origin/main`。
- scope:
  - 读取 `walkthrough.md` 并复用当前阶段上下文。
  - 检查工作区变更、当前分支与远端配置。
  - 提交当前修改并推送到 GitHub。
- changed_files:
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
  - `include/app/fusion.h`
  - `include/core/eskf.h`
  - `scripts/analysis/run_state_ablation.py`
  - `src/app/config.cpp`
  - `src/app/diagnostics.cpp`
  - `src/app/evaluation.cpp`
  - `src/app/initialization.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `src/core/ins_mech.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `src/utils/math_utils.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
- commands:
  - `git status --short`
  - `git branch --show-current`
  - `git remote -v`
  - `git add config_data2_baseline_inekf_ctrl.yaml config_data2_gnss30_inekf.yaml include/app/fusion.h include/core/eskf.h scripts/analysis/run_state_ablation.py src/app/config.cpp src/app/diagnostics.cpp src/app/evaluation.cpp src/app/initialization.cpp src/app/pipeline_fusion.cpp src/core/eskf_engine.cpp src/core/ins_mech.cpp src/core/measurement_models_uwb.cpp src/utils/math_utils.cpp walkthrough.md`
  - `git commit -m "chore: sync inekf rewrite updates and docs"`
  - `git push origin main`
- artifacts:
  - Git commit: `69919da`（`chore: sync inekf rewrite updates and docs`）
  - 远端更新: `origin/main` 从 `bc0f1b4` 前进到 `69919da`
- metrics:
  - N/A（本次任务为代码同步，不新增实验指标）
- observability_notes:
  - 本次未执行新实验；状态块 `21-30`、`fusion.ablation.*`、`fusion.post_gnss_ablation.*` 与调度窗口均未新增操作。
- decision:
  - 已完成本地到远端的版本同步；研究任务优先级保持不变，继续 `ISSUE-001` 与 GNSS30 可观性矩阵。
- next_step:
  - 推送完成后，按优先队列继续无 FEJ InEKF 退化定位与 compare 冲突清理。

### session_id: 20260304-2312-inekf-ri-realization-codeoverhaul3

- timestamp: 2026-03-04 23:12 (local)
- objective: 按 `代码大修3.md` 将当前 InEKF 从“ESKF 套壳”改为真实 RI 实现，并完成最小闭环回归验证。
- scope:
  - 在 `FejManager/InEkfConfig` 增加 RI 原点 `p_init_ecef`，并在 `EskfEngine::Initialize` 记录初始化参考点。
  - 在 `EskfEngine::InjectErrorState` 中加入 RI→加法误差逆变换（`δv, δp` 由 `ξ_v, ξ_p` 还原）。
  - 将 `InsMech::BuildProcessModel` 的 InEKF `F_vφ` 改为 `Skew(g_ned)`。
  - 更新 ODO/NHC/GNSS 位置/GNSS 速度四个量测模型的 InEKF 雅可比（含 ODO/NHC 姿态列解耦）。
  - 关闭基于“ESKF/InEKF 姿态列必为相反数”的旧一致性强校验，并调整 `SetFejManager` 与 `Initialize` 调用顺序。
  - 编译并回归 `baseline_eskf` / `baseline_inekf_ctrl` / `gnss30_inekf` / `gnss30_eskf_nofreeze`。
- changed_files:
  - `include/core/eskf.h`
  - `src/core/eskf_engine.cpp`
  - `src/core/ins_mech.cpp`
  - `src/core/measurement_models_uwb.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
  - `config_data2_gnss30_eskf_nofreeze.yaml`
- commands:
  - `cmake --build build --config Release`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_inekf_ctrl.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_inekf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_eskf_nofreeze.yaml`
- artifacts:
  - `output/review/20260304-2312-inekf-ri-implementation/baseline_eskf.log`
  - `output/review/20260304-2312-inekf-ri-implementation/baseline_inekf_ctrl.log`
  - `output/review/20260304-2312-inekf-ri-implementation/gnss30_inekf.log`
  - `output/review/20260304-2312-inekf-ri-implementation/gnss30_eskf_nofreeze.log`
- metrics:
  - baseline ESKF RMSE xyz / RMSE3D: `0.763591 0.565924 0.770052` / `1.223242`
  - baseline InEKF RMSE xyz / RMSE3D: `0.953258 0.962653 1.014938` / `1.692779`
  - GNSS30 InEKF RMSE xyz / RMSE3D: `40.557 89.686 66.555` / `118.819261`
  - GNSS30 ESKF RMSE xyz / RMSE3D: `57.533 82.961 78.063` / `127.618202`
  - 对比结论:
    - ESKF baseline 与历史 `0.763591 0.565924 0.770052` 完全一致（回归通过）。
    - GNSS30 InEKF 的 RMSE3D 从旧版 `195.641` 降至 `118.819`，并优于本轮 GNSS30 ESKF `127.618`。
    - baseline InEKF 明显劣化（新增 `ISSUE-005`）。
- artifact_mtime:
  - `output/review/20260304-2312-inekf-ri-implementation/baseline_eskf.log`: 2026-03-04 22:52:45
  - `output/review/20260304-2312-inekf-ri-implementation/baseline_inekf_ctrl.log`: 2026-03-04 22:54:37
  - `output/review/20260304-2312-inekf-ri-implementation/gnss30_inekf.log`: 2026-03-04 22:53:20
  - `output/review/20260304-2312-inekf-ri-implementation/gnss30_eskf_nofreeze.log`: 2026-03-04 22:53:55
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04 14:24:58
  - `config_data2_baseline_inekf_ctrl.yaml`: 2026-03-04 21:06:56
  - `config_data2_gnss30_inekf.yaml`: 2026-03-04 21:07:02
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
  - `include/core/eskf.h`: 2026-03-04 22:49:55
  - `src/core/eskf_engine.cpp`: 2026-03-04 22:50:07
  - `src/core/ins_mech.cpp`: 2026-03-04 22:50:17
  - `src/core/measurement_models_uwb.cpp`: 2026-03-04 22:50:40
  - `src/app/pipeline_fusion.cpp`: 2026-03-04 22:50:48
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（构建成功，4 组日志均为本会话生成且时间链一致；冲突项单列到 `ISSUE-005`）
- observability_notes:
  - 状态块 `21-30`：本次未启用 `fusion.ablation.*` 与 `fusion.post_gnss_ablation.*`；外参与尺度状态仍由 ODO/NHC/GNSS 量测驱动。
  - RI 量测改动映射：
    - ODO/NHC 在 InEKF 下将 `StateIdx::kAtt` 列解耦置零（速度类约束不再直接线性耦合姿态误差）。
    - GNSS 位置模型在 InEKF 下引入 `p_ned_local` 项，姿态列范数在基线中显著增大（见日志 `||H_att||_F`）。
    - GNSS 速度模型在 InEKF 下加入 `-Skew(v_ned)` 项。
  - 调度窗口：
    - baseline：GNSS 全程可用。
    - GNSS30：`fusion.gnss_schedule.enabled=true`，`head_ratio=0.30`，`split_t=528799.875306`。
  - 行为判定：
    - baseline：ESKF 中性（neutral），InEKF 劣化（degraded）。
    - GNSS30：InEKF 相比旧版显著改善（improved），且本轮优于 ESKF（improved）。
- decision:
  - 接受 `代码大修3` 的结构性改造，确认当前 InEKF 已不再是“单纯符号翻转”。
  - 将 `EXP-20260304-inekf-ri-realization-codeoverhaul3` 设为当前参考实现，同时将 baseline InEKF 回归问题登记为 `ISSUE-005` 并提升优先级。
  - 保留旧版“姿态列符号相反”一致性检查为停用状态，避免误报阻断运行。
- next_step:
  - 对 `ISSUE-005` 做 A/B 定位（优先切换 GNSS 位置 `p_ned_local` 项与注入逆变换项，确认 baseline 回归根因）。
  - 重跑 `ISSUE-001` compare 汇总，更新 `output/compare_data2_baseline_eskf_vs_inekf_ctrl/`。
  - 在 `head_ratio=0.30` 下继续执行 post-GNSS 组合冻结矩阵，验证 RI 实现下状态块 `21-30` 的可观性趋势。

### session_id: 20260304-2303-push-inekf-ri-github

- timestamp: 2026-03-04 23:03 (local)
- objective: 将 `代码大修3` 的 InEKF RI 实现改动上传到 GitHub `origin/main`。
- scope:
  - 读取 `walkthrough.md` 并复用当前阶段上下文。
  - 检查分支与远端状态，提交工作区改动并推送远端。
- changed_files:
  - `walkthrough.md`
- configs:
  - N/A（本次为代码同步任务）
- commands:
  - `git status --short`
  - `git branch --show-current`
  - `git remote -v`
  - `git add include/core/eskf.h src/core/eskf_engine.cpp src/core/ins_mech.cpp src/core/measurement_models_uwb.cpp src/app/pipeline_fusion.cpp walkthrough.md`
  - `git commit -m "feat: implement right-invariant inekf per overhaul3"`
  - `git push origin main`
- artifacts:
  - Git commit: `9592676`（`feat: implement right-invariant inekf per overhaul3`）
  - 远端更新: `origin/main` 从 `873b39f` 前进到 `9592676`
- metrics:
  - N/A（本次任务未新增实验运行）
- observability_notes:
  - 本次未新增量测/调度/消融实验；状态块 `21-30`、`fusion.ablation.*`、`fusion.post_gnss_ablation.*` 与调度窗口未发生新实验层变更。
- decision:
  - `代码大修3` 的实现已完成版本化并同步到 GitHub。
- next_step:
  - 按优先队列继续 `ISSUE-005` baseline InEKF 回归定位与 `ISSUE-001` compare 冲突清理。

### session_id: 20260304-2318-fix-inekf-gvel-gyro-noise-map

- timestamp: 2026-03-04 23:18 (local)
- objective: 按 `ISSUE-005` 假设修复 InEKF 过程噪声映射，补充 `G(kVel,gyro)` 的 RI 耦合项并验证效果。
- scope:
  - 在 `InsMech::BuildProcessModel` 的 InEKF 分支新增 `G.block<3,3>(kVel,3) = -Skew(v_ned) * C_bn`。
  - 编译并回归 `baseline_eskf` / `baseline_inekf_ctrl` / `gnss30_inekf` / `gnss30_eskf_nofreeze`。
- changed_files:
  - `src/core/ins_mech.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_baseline_inekf_ctrl.yaml`
  - `config_data2_gnss30_inekf.yaml`
  - `config_data2_gnss30_eskf_nofreeze.yaml`
- commands:
  - `cmake --build build --config Release`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_baseline_inekf_ctrl.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_inekf.yaml`
  - `.\build\Release\eskf_fusion.exe --config config_data2_gnss30_eskf_nofreeze.yaml`
- artifacts:
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_eskf.log`
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_inekf_ctrl.log`
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_inekf.log`
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_eskf_nofreeze.log`
- metrics:
  - baseline ESKF RMSE xyz / RMSE3D: `0.763591 0.565924 0.770052` / `1.223242`
  - baseline InEKF RMSE xyz / RMSE3D: `1.044499 1.277957 1.263830` / `2.078802`
  - GNSS30 InEKF RMSE xyz / RMSE3D: `47.471 96.176 126.611` / `165.932710`
  - GNSS30 ESKF RMSE xyz / RMSE3D: `57.533 82.961 78.063` / `127.618202`
  - 对比 `EXP-20260304-inekf-ri-realization-codeoverhaul3`:
    - baseline InEKF: `1.692779 -> 2.078802`（劣化）
    - GNSS30 InEKF: `118.819261 -> 165.932710`（劣化）
- artifact_mtime:
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_eskf.log`: 2026-03-04 23:27:10
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/baseline_inekf_ctrl.log`: 2026-03-04 23:27:45
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_inekf.log`: 2026-03-04 23:28:20
  - `output/review/20260304-2318-inekf-ri-gvelgyro-fix/gnss30_eskf_nofreeze.log`: 2026-03-04 23:28:55
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 2026-03-04 14:24:58
  - `config_data2_baseline_inekf_ctrl.yaml`: 2026-03-04 21:06:56
  - `config_data2_gnss30_inekf.yaml`: 2026-03-04 21:07:02
  - `config_data2_gnss30_eskf_nofreeze.yaml`: 2026-03-04 15:20:46
  - `src/core/ins_mech.cpp`: 2026-03-04 23:26:12
- dataset_time_window:
  - `528076.009368` 到 `530488.900000`
- result_freshness_check: pass（构建成功，4 组日志均为本会话生成且时间链一致）
- observability_notes:
  - 状态块 `21-30`：本次未启用 `fusion.ablation.*` 或 `fusion.post_gnss_ablation.*`；外参相关更新机制未改动。
  - 调度窗口：
    - baseline：GNSS 全程更新。
    - GNSS30：`fusion.gnss_schedule.enabled=true`，`head_ratio=0.30`，后段仅 IMU+NHC+ODO。
  - 行为判定：
    - baseline：InEKF 相对 ESKF 继续劣化（degraded）。
    - GNSS30：InEKF 从上一轮“优于 ESKF”退化为“劣于 ESKF”（degraded）。
- decision:
  - 已落地“补 `G(kVel,gyro)`”改动，但证据显示其单独引入不能修复 `ISSUE-005`，且导致性能进一步恶化。
  - 将“仅补该项即可修复”的假设标记为 rejected，后续按系统一致性继续 A/B。
- next_step:
  - 做符号与结构 A/B：`-Skew(v_ned)C_bn` / `+Skew(v_ned)C_bn` / 不启用，比较 baseline 与 GNSS30 双场景。
  - 检查 RI 下 `ξ_p` 相关过程噪声映射是否遗漏，并与量测噪声约定联合验证。

## 下一步（优先队列）

1. 处理 `ISSUE-005/006`：对 `G(kVel,gyro)` 做符号与开关 A/B（`-Skew(v)C_bn`、`+Skew(v)C_bn`、disabled），在 baseline+GNSS30 双场景找出一致改进方向。
2. 在上一步最优变体上，再做 `p_ned_local` 量测耦合项与 RI 注入逆变换的联合 A/B，定位主导退化源。
3. 重跑 `ISSUE-001` 对应 compare，使用最新 baseline 产物重生 `output/compare_data2_baseline_eskf_vs_inekf_ctrl/` 并关闭冲突。
4. 在 `head_ratio=0.30` 下执行 `odo_scale/mounting/lever` post-GNSS 组合冻结矩阵，记录状态块 `21-30` 的可观性变化。
5. 在 GNSS30 对比中补齐 consistency 统计（`accept_ratio`, `nis_mean`, `nis_max`）并纳入汇总。
6. 为关键健壮性回归补自动化脚本用例：`missing_truth_manual_init`、`invalid_gnss`、`max_dt=0`。
