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
| EXP-20260304-data2-gnss30-inekf | 2026-03-04 | InEKF 对比实验：GNSS 仅前30%启用，后段 INS/ODO/NHC | `config_data2_gnss30_inekf.yaml` | `SOL_data2_gnss30_inekf.txt`, `output/logs_runcheck_gnss30_inekf.log`, `output/result_data230_inekf/`, `output/compare_data2_gnss30_eskf_postfix_vs_inekf/` | 日志 RMSE xyz: `92.918 104.149 149.448`；对比 RMSE 3D: `204.488` | 新增对比 | pass（日志、SOL、图像时间一致） |
| EXP-20260304-data2-gnss30-eskf-nofreeze | 2026-03-04 | ESKF 对比实验：GNSS 仅前30%启用，后段不做 GNSS 杆臂冻结，状态照常更新 | `config_data2_gnss30_eskf_nofreeze.yaml` | `SOL_data2_gnss30_eskf_nofreeze.txt`, `output/logs_runcheck_gnss30_eskf_nofreeze.log`, `output/result_data230_eskf_nofreeze/`, `output/compare_data2_gnss30_eskf_postfix_vs_nofreeze/` | 日志 RMSE xyz: `58.292 84.777 78.863`；对比 RMSE 3D: `129.632` | 新增对比 | pass（日志、SOL、图像时间一致） |

## 已知不一致项（Known Inconsistencies）

| issue_id | 发现日期 | 描述 | 影响文件 | 影响 | 处理状态 |
|---|---|---|---|---|---|
| ISSUE-001-metric-conflict-baseline-inekf | 2026-03-04 | `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt` 中 InEKF 基线指标与同日 runcheck 日志不一致。 | `output/compare_data2_baseline_eskf_vs_inekf_ctrl/summary.txt`, `output/compare_data2_baseline_eskf_vs_inekf_ctrl/metrics.csv`, `output/logs_runcheck_baseline_inekf_ctrl.log` | 可能导致算法对比结论被陈旧结果污染。 | open |

## 开放假设（Open Hypotheses）

| hyp_id | 假设 | 当前证据 | 下一步验证 | 状态 |
|---|---|---|---|---|
| HYP-1 | 在当前控制设置下，data2 的 InEKF 与 ESKF 结果应接近。 | 同日 runcheck 日志显示 RMSE 接近，但历史 compare 汇总显示大幅偏差。 | 用同时间戳下的新 SOL 产物重跑 compare 脚本并重生汇总文件。 | open |
| HYP-2 | 在当前 data2 设置中，仅冻结 `gnss_lever_arm` 的 post-GNSS 策略不足以保证后段稳定精度。 | ESKF 下 `postfix-gnsslever` 与 `nofreeze` 指标完全一致（RMSE 3D 均为 `129.632`）；InEKF nofreeze 为 `204.488`。说明仅 `gnss_lever_arm` 冻结影响中性，且不足以单独改善后段。 | 在相同 split 与同一测量配置下，对比多状态 post-GNSS 冻结方案（`odo_scale/mounting/lever`）。 | open |
| HYP-3 | 外参相关状态的可观性对 GNSS 调度和运动激励高度敏感。 | 代码中已有弱激励冻结、ablation 与 consistency 统计机制。 | 运行受控消融矩阵并跟踪状态轨迹与一致性指标。 | open |

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

## 下一步（优先队列）

1. 使用已修复的 `scripts/analysis/compare_fej_vs_standard.py` 重跑 `ISSUE-001` 的 baseline compare 并重生摘要文件。
2. 以 `EXP-20260304-data2-gnss30-eskf-nofreeze` 为对照，启动 InEKF 的 post-GNSS 多状态冻结矩阵（`odo_scale/mounting/lever`）。
3. 在相同 split 下补齐 ESKF/InEKF 的 consistency 统计（accept ratio、NIS）并纳入对比摘要。
4. 在后续每个实验条目中强制补全可观性字段:
   - 状态块行为
   - consistency 指标
   - 调度窗口影响
5. 继续保持任务级摘要写法；达到阈值后执行历史压缩。
6. 后续每个完整实验批次结束后，执行一次远端同步以避免本地/远端漂移。
