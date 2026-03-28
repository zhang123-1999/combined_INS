# Plan: ODO NHC BgZ Chain Bug

**Generated**: 2026-03-25 14:06:50
**Estimated Complexity**: High

## Overview

目标不是继续证明 `INS/GNSS/ODO/NHC` 会不会坏，而是把 `ODO/NHC -> bg_z` 链路拆成可验证的 bug 候选：

1. 是 `ODO` 写坏，还是 `NHC` 写坏，还是两者串联才坏。
2. 是 analytic Jacobian 本身有符号/尺度错误，还是 state-update 路由错误。
3. 是 update admission 与真实 residual 参考点不一致，导致错误 update 被放行。
4. 是“外参被冻结后 residual 无处吸收”触发了 `bg_z` 代偿，还是即使外参可动也会写坏。

所有实验都基于当前 staged 主线：

- base config: `config_data2_baseline_eskf.yaml`
- reference control: `EXP-20260325-data2-staged-truth-ablation-r2::control_strict_gate_freeze_gnss_lever`
- stressed case: `EXP-20260325-data2-staged-truth-ablation-r2::fix_mounting_and_odo_lever_truth`
- fixed controls:
  - `strict weak gate`
  - `phase2/phase3 freeze GNSS lever`
  - `fusion.init.odo_scale=1.0`
  - `fusion.fej.enable=false`

## Prerequisites

- `build/Release/eskf_fusion.exe` 可运行
- 可复用现有脚本：
  - `scripts/analysis/run_data2_staged_truth_ablation_probe.py`
  - `scripts/analysis/run_nhc_state_convergence_research.py`
  - `scripts/analysis/odo_nhc_update_sweep.py`
- 新增脚本统一写入 `output/data2_*`，不要覆盖已有 fresh 产物
- 所有 case 固定使用同一 `gnss_schedule`、同一 `start/finaltime`、同一 truth/init 口径

## Sprint 1: Route Attribution
**Goal**: 先确认坏链到底由 `ODO` 还是 `NHC` 触发，以及坏在 Jacobian 还是坏在 state update。
**Demo/Validation**:
- 产出 `case_metrics.csv`、`bgz_step_metrics.csv`、`summary.md`
- 能回答 “哪一条 route 一旦切断，`bg_z` 和 `yaw` 立即恢复”

### Task 1.1: 搭建 route decomposition runner
- **Location**: `scripts/analysis/run_data2_bgz_route_decomposition.py`
- **Description**: 基于 `fix_mounting_and_odo_lever_truth` 生成以下 case。
- **Dependencies**: 无
- **Acceptance Criteria**:
  - 支持统一生成 config、运行 solver、汇总 metrics
  - 每个 case 都保留 exact config path
- **Validation**:
  - `python -m py_compile scripts/analysis/run_data2_bgz_route_decomposition.py`

### Task 1.2: 定义 `EXP-20260325-data2-bgz-route-decomposition-r1`
- **Location**: `output/data2_bgz_route_decomposition_r1_20260325/`
- **Description**: 运行以下最小矩阵。
- **Dependencies**: Task 1.1
- **Acceptance Criteria**:
  - 至少包含以下 cases
  - `control_strict_gate_freeze_gnss_lever`
  - `combined_truth_baseline`
  - `combined_truth_disable_odo_bgz_jacobian`
  - `combined_truth_disable_odo_bgz_state_update`
  - `combined_truth_disable_nhc_bgz_state_update`
  - `combined_truth_disable_both_bgz_state_update`
  - `combined_truth_disable_odo_jacobian_and_both_state_update`
- **Validation**:
  - 输出每 case 的 `phase2_rmse_3d_m`
  - 输出每 case 的 `phase3_rmse_3d_m`
  - 输出 `first_divergence_start_t`
  - 输出 `odo_accept_ratio` / `nhc_accept_ratio`

### Task 1.3: 增加每次成功 update 的 `bg_z` 写入日志
- **Location**: `src/app/pipeline_fusion.cpp`, `include/app/fusion.h`, `src/app/diagnostics.cpp`
- **Description**: 对 `ODO` 和 `NHC` 成功更新记录：
  - `t`
  - `sensor_tag`
  - `y`
  - `H_bgz`
  - `K_bgz`
  - `delta_bg_z`
  - `nis_gate`
  - `v_b.x/y/z`
  - `v_wheel_b.x/y/z`
  - `v_v.x/y/z`
  - `lever`
  - `mounting`
- **Dependencies**: Task 1.1
- **Acceptance Criteria**:
  - 日志可通过单个 config 开关启停
  - 不影响默认主线输出
- **Validation**:
  - `bgz_step_metrics.csv` 至少能定位 first bad write 的 sensor 和时间

## Sprint 2: Jacobian Correctness
**Goal**: 验证 `ODO/NHC` 对 `bg_z`、`sg_z`、`att` 的 analytic Jacobian 是否与有限差分一致。
**Demo/Validation**:
- 产出 finite-difference audit 报告
- 能回答 “错在公式、符号、尺度，还是公式本身没错但不该在该场景生效”

### Task 2.1: 新建 finite-difference 审计脚本
- **Location**: `scripts/analysis/audit_odo_nhc_bgz_jacobian_fd.py`
- **Description**: 对 `ComputeOdoModel()` / `ComputeNhcModel()` 做 offline audit，比较 analytic 与 finite difference：
  - `H_bgz`
  - `H_sgz`
  - `H_att_z`
  - `H_lever_y`
- **Dependencies**: Sprint 1 完成后复用 config/case
- **Acceptance Criteria**:
  - 支持读取指定 case config 与指定 timestamp
  - 支持 perturbation sweep：`1e-6`, `1e-7`, `1e-8`
- **Validation**:
  - 输出 `fd_vs_analytic.csv`
  - 输出 relative error 和 sign consistency

### Task 2.2: 定义 `EXP-20260325-data2-bgz-jacobian-fd-audit-r1`
- **Location**: `output/data2_bgz_jacobian_fd_audit_r1_20260325/`
- **Description**: 选 3 个关键时间窗做 audit。
- **Dependencies**: Task 2.1
- **Acceptance Criteria**:
  - `phase2 entry`: `528276~528320 s`
  - `first divergence`: `528314~528316 s`
  - `late stable control`: `528860~528870 s`
- **Validation**:
  - 若 combined-truth case 的 analytic `H_bgz` 与 FD 明显不一致，则 bug 在公式
  - 若一致，则 bug 更可能在 route activation / state competition

## Sprint 3: Admission And Reference-Frame Consistency
**Goal**: 检查 update admission 是否与 residual 使用的参考点一致。
**Demo/Validation**:
- 产出 admission audit 报告
- 能回答 `v_b` 预检是否放进了本不该进入的 update

### Task 3.1: 审计当前 admission 与 residual 口径
- **Location**: `src/app/pipeline_fusion.cpp`, `scripts/analysis/analyze_bgz_admission_alignment.py`
- **Description**: 对比三种 admission 口径：
  - current `v_b`
  - `v_wheel_b`
  - `v_v`
- **Dependencies**: Sprint 1 的 per-update log
- **Acceptance Criteria**:
  - 对每个 rejected/accepted update 记录三种口径下的 gate decision
  - 统计 “current accepted but v_wheel_b/v_v should reject” 的比例
- **Validation**:
  - 输出 `admission_mismatch_summary.csv`

### Task 3.2: 定义 `EXP-20260325-data2-nhc-admission-alignment-r1`
- **Location**: `output/data2_nhc_admission_alignment_r1_20260325/`
- **Description**: 运行 3 个 admission variants。
- **Dependencies**: Task 3.1
- **Acceptance Criteria**:
  - `baseline_current_v_b_gate`
  - `nhc_gate_use_v_wheel_b`
  - `nhc_gate_use_v_v`
- **Validation**:
  - 比较 `odo_accept_ratio` / `nhc_accept_ratio`
  - 比较 `first_divergence_start_t`
  - 比较 `phase2 bg_z_abs_max`

## Sprint 4: State-Competition Validation
**Goal**: 判断是不是“外参被冻结后 residual 只能灌进 bg_z”，还是即使外参开放也会照样写坏。
**Demo/Validation**:
- 产出 freeze/open 对比
- 能回答 route bug 是“错误状态竞争”还是“无论如何都错写”

### Task 4.1: 定义外参吸收通路对照组
- **Location**: `scripts/analysis/run_data2_bgz_competition_probe.py`
- **Description**: 基于 combined-truth 分别比较：
  - `mounting fixed + lever fixed`
  - `mounting open + lever fixed`
  - `mounting fixed + lever open`
  - `mounting open + lever open`
- **Dependencies**: Sprint 1
- **Acceptance Criteria**:
  - 保持其他 staged controls 不变
  - 重点观察 `delta_bg_z`、`delta_mount_yaw`、`delta_lever_y`
- **Validation**:
  - 输出 `state_competition_summary.md`

### Task 4.2: 定义 `EXP-20260325-data2-bgz-state-competition-r1`
- **Location**: `output/data2_bgz_state_competition_r1_20260325/`
- **Description**: 通过 state freeze/open 确认 residual 最终流向。
- **Dependencies**: Task 4.1
- **Acceptance Criteria**:
  - 若“外参开放后 bg_z 恶化显著减轻”，则主 bug 是状态竞争 / route prioritization
  - 若“外参开放后仍主要写坏 bg_z”，则主 bug 更接近 Jacobian 或 admission
- **Validation**:
  - 比较 `phase2_rmse_3d_m`, `phase3_rmse_3d_m`, `peak_bg_z`, `peak_total_mounting_yaw`, `peak_odo_lever_y`

## Sprint 5: Mitigation Validation
**Goal**: 在不引入 debug 开关的前提下验证候选正式修复。
**Demo/Validation**:
- 至少一个 non-debug candidate 让 `INS/GNSS/ODO/NHC` 不再显著差于 `INS/GNSS`

### Task 5.1: 固化正式候选修复
- **Location**: `src/app/pipeline_fusion.cpp`, `include/app/fusion.h`, `src/app/config.cpp`
- **Description**: 根据前面实验，只实现一个最强候选：
  - `ODO/NHC -> bg_z` observability gate
  - 或按 sensor 拆开的 `bg_z` update mask
  - 或 admission 改为 `v_wheel_b / v_v`
- **Dependencies**: Sprints 1-4
- **Acceptance Criteria**:
  - 不依赖 debug switch
  - 可配置启停
- **Validation**:
  - 编译通过

### Task 5.2: 定义 `EXP-20260325-data2-bgz-mitigation-validation-r1`
- **Location**: `output/data2_bgz_mitigation_validation_r1_20260325/`
- **Description**: 只做最终三组对比。
- **Dependencies**: Task 5.1
- **Acceptance Criteria**:
  - `ins_gnss_only`
  - `current_mainline_ins_gnss_odo_nhc`
  - `mitigated_ins_gnss_odo_nhc`
- **Validation**:
  - `mitigated` 的 `phase3_rmse_3d_m` 不应再远差于 `ins_gnss_only`
  - `bg_z_abs_max` 和 `yaw_abs_max` 应显著低于 current mainline

## Testing Strategy

- 所有实验固定使用相同时间窗与同一 solver build
- 所有对比必须同时报告：
  - `phase2_rmse_3d_m`
  - `phase3_rmse_3d_m`
  - `first_divergence_start_t`
  - `odo_accept_ratio`
  - `nhc_accept_ratio`
  - `peak_bg_z_abs_degh`
  - `peak_yaw_abs_deg`
  - `peak_total_mounting_yaw_deg`
  - `peak_odo_lever_y_m`
- 任何结论都必须回指：
  - exact config path
  - exact output path
  - artifact mtime

## Potential Risks & Gotchas

- 不要把 “debug route cut 恢复了系统” 直接等价成 “jacobian 公式错了”；它也可能只是说明状态竞争错了。
- 不要在同一轮里同时改 admission、改 Jacobian、改 observability gate；否则无法归因。
- `control` 与 `combined truth` 的 `lever_odo` 是否冻结不同，不要混淆 “几何更合理” 和 “坏 route 被关掉”。
- `freeze_extrinsics_when_weak_excitation=false` 是当前关键背景；若某轮实验改了它，必须单独标注。
- 需要继续保留 `INS/GNSS` 对照，否则无法判断修复后 wheel constraints 是转正、转中性，还是只是“没那么坏”。

## Rollback Plan

- 新增脚本仅写入 `scripts/analysis/` 与 `output/`，不覆盖现有产物
- 所有 instrumentation 用独立 config switch 包裹
- 若正式候选修复无收益，先回退 runtime logic，只保留分析脚本和日志格式
