# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-20`

## correction_notice

- 2026-03-20 已确认：`POS-320` 是参考真值，`T3` 才对应 data2 当前使用的低精度 ICM。
- 因此，本归档中凡是把 `POS-320`、`POS320` 或 `RTK/POS body-frame median` 当作 data2 `GNSS lever` 判优真值的条目，都只能作为历史留痕，不能再直接用于当前杆臂 proximity / axis-best / parameter ranking 结论。
- 当前允许直接引用的 fresh 杆臂重判结果仅为：
  - `EXP-20260320-data2-ins-gnss-lever-truth-imu-params-r2-t3-truth`
  - `EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r2-t3-truth`

## 项目快照

### 当前阶段目标
- 将当前正式研究目标收敛到 `data2 + ESKF + corrected RTK GNSS`，并以 `GNSS outage` 分段指标作为算法优化主目标。
- `data2` 官方 ESKF 基线固定为 `dataset/data2/rtk.txt`（7 列，仅位置与位置标准差，无速度），并显式关闭 `GNSS_VEL` 更新。
- 当前对“非 PVA 状态真值锚定”相关实验的正式解释统一切换到 `runtime_truth_anchor_pva` fresh rerun；旧的初始化式 `use_truth_pva=true` 结果只保留为历史留痕。若显式关闭 runtime anchor，则只作为“shared correction / 杆臂漂移对照”使用。
- 纯 `INS/GNSS` 子问题与 `ODO/NHC` 强耦合子问题继续拆开分析：前者聚焦 `bg/sg/gnss_lever` recoverability，后者聚焦 `road-constraint coupling + standard ESKF reset/covariance semantics`。
- 主 `walkthrough.md` 现在只保留当前工作台必需内容；更早完整 session 正文改存到 dated archive，保证 Start Checklist 可直接使用。

### 当前工作台速览
- last_completed_session: `20260320-0927-data2-gnss-lever-p0-q-sweep-closeout-r1`
- current_official_experiments: `EXP-20260316-data2-eskf-baseline-output-r2`、`EXP-20260316-data2-state-sanity-r1`、`EXP-20260317-data2-overall-update-attribution-r1`、`EXP-20260318-data2-gnss-lever-bias-attribution-r1`、`EXP-20260318-data2-ins-gnss-state-sanity-r1`、`EXP-20260318-data2-ins-gnss-lever-truth-init-free-run-r1`、`EXP-20260318-data2-ins-gnss-pva-component-anchor-r1`、`EXP-20260318-data2-ins-gnss-lever-truth-imu-params-r1`、`EXP-20260318-data2-ins-gnss-lever-noise-coupling-sweep-r1`、`EXP-20260318-data2-turn-window-shared-correction-r1`、`EXP-20260318-data2-turn-window-variant-ab-r1`、`EXP-20260318-data2-turn-window-position-gain-scale-r1`、`EXP-20260318-data2-turn-window-gnsspos-staged-r1`、`EXP-20260318-data2-turn-window-xy-term-source-r1`、`EXP-20260319-data2-turn-window-lgx-from-y-gain-scale-r1`、`EXP-20260319-data2-turn-window-y-same-axis-sign-diagnostic-r1`、`EXP-20260319-data2-turn-direction-proxy-r1`、`EXP-20260319-data2-turn-window-y-turn-conditioned-position-r1`、`EXP-20260319-data2-turn-window-lgy-from-y-gain-scale-r1`、`EXP-20260319-data2-turn-window-lgy-numerator-geometry-r1`、`EXP-20260319-data2-turn-window-lgy-lever-axis-focus-r1`、`EXP-20260319-data2-turn-window-lgy-history-r1`、`EXP-20260319-data2-turn-window-lgy-transition-carryover-r1`、`EXP-20260319-data2-turn-window-lgy-key-update-audit-r1`、`EXP-20260320-data2-turn-motion-stats-r1`、`EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1`
- open_hypotheses: `HYP-1`、`HYP-6`、`HYP-7`、`HYP-8`、`HYP-9`、`HYP-10`、`HYP-11`、`HYP-12`、`HYP-13`、`HYP-18`、`HYP-19`、`HYP-21`、`HYP-24`、`HYP-25`、`HYP-26`、`HYP-27`
- active_archive_docs: `docs/project_history/walkthrough_preservation_snapshot_20260313.md`、`docs/project_history/walkthrough_archive_registry_20260313.md`、`docs/project_history/walkthrough_archive_sessions_20260313.md`、`docs/project_history/walkthrough_archive_sessions_20260318.md`、`docs/project_history/artifact_relocation_index.md`
- pending_next_actions: 见文末 `## Next Actions`

### 当前基线与代码事实
- 状态维度: `kStateDim = 31`
- 主要滤波模式:
  - 标准 ESKF（`fusion.fej.enable: false`）
  - InEKF 开关模式（`fusion.fej.enable: true`）
- ESKF 官方基线配置: `config_data2_baseline_eskf.yaml`
- data2 当前正式 GNSS 源: `dataset/data2/rtk.txt`（7 列，仅 GNSS 位置与位置标准差）
- 运行期 `PVA` 真值锚定开关: `fusion.init.runtime_truth_anchor_pva`，默认 `false` 保持兼容；实验中显式开启时会在运行期持续回写 `p/v/q`
- `GNSS_POS` 的位置时间对齐现只依赖“数据集是否提供 GNSS 速度列”；`enable_gnss_velocity` 仅控制 `GNSS_VEL` 更新
- data2 官方 ESKF 附加约束:
  - `fusion.enable_gnss_velocity=false`
  - `fusion.ablation.disable_mounting_roll=true`
  - `fusion.ablation.disable_gnss_lever_z=true`
- 当前官方评价指标: `GNSS outage` 分段 `x/y/z/3D RMSE` 与分段末时刻 `x/y/z/3D` 误差；总体 RMSE 只保留为辅助诊断
- 当前官方标准结果目录: `output/data2_eskf_baseline/`

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

### 当前关键实验

| exp_id | 日期 | 目的 | 关键产物 | 关键指标 | 状态 | 新鲜度 |
|---|---|---|---|---|---|---|
| EXP-20260316-data2-eskf-baseline-output-r2 | 2026-03-16 | 固化 `data2 official outage` 标准输出目录与图集口径 | `output/data2_eskf_baseline/{metrics.csv,outage_segments.csv,summary.md,manifest.json,plot/*.png}` | outage mean/max RMSE3D=`9.248204/12.303623`; final3D=`24.330687/70.749305` | official_output_plot_refined | pass |
| EXP-20260316-data2-state-sanity-r1 | 2026-03-16 | 在 official outage 下做 `22` 个非 PVA 标量状态真值锚定控制组 + 单状态释放矩阵 | `output/data2_eskf_state_sanity/{case_metrics.csv,state_judgement.csv,summary.md,manifest.json}` | control=`12.063017/45.713949`; overall=`18 abnormal / 1 borderline / 3 normal` | state_sanity_matrix_complete | pass |
| EXP-20260317-overall-update-audit-r1 | 2026-03-17 | 用 deep-research 审计大量状态异常是否来自 shared update-chain / covariance 语义问题 | `.research/20260316-overall-update-audit-a1/{final_report.md,polished_report.md}` | suspects=`ODO/NHC high-rate coupling + standard ESKF reset semantics` | deep_research_audit_complete | pass |
| EXP-20260317-data2-overall-update-attribution-r1 | 2026-03-17 | 在 official outage 上区分 `standard_reset_gamma`、`ODO/NHC` 与状态异常的主导来源 | `output/data2_eskf_update_attribution/{summary.md,variant_overview.csv,tracked_state_summary.csv}` | `standard_reset_gamma` 改善 control；`nhc_20hz` 使 `release_bg_y` 灾难级发散 | update_attribution_complete | pass |
| EXP-20260317-data2-ins-gnss-state-sanity-r1 | 2026-03-17 | 去掉 `ODO/NHC + mounting/odo_lever/odo_scale` 后检查纯 `INS/GNSS` recoverability | `output/data2_eskf_ins_gnss_state_sanity/{case_metrics.csv,state_judgement.csv,summary.md}` | control=`0.049119/0.018997`; 多数 `ba/bg/sg/sa` 仍 abnormal | ins_gnss_state_sanity_complete | pass |
| EXP-20260317-data2-gnss-debug-audit-r1 | 2026-03-17 | 在纯 `INS/GNSS` 下审计 Jacobian、correction 符号、Markov 量级与分支选择 | `output/data2_gnss_debug_audit/jacobian_300s_vel13/{summary.md,column_errors.csv}` | `GNSS_POS att rel_max=0.007125`; `GNSS_VEL bg/sg/gnss_lever rel_max=0` | gnss_debug_audit_complete | pass |
| EXP-20260317-data2-gnss-pos-first-residual-r1 | 2026-03-17 | 分解 control 首个 `GNSS_POS` residual 的来源 | `output/data2_gnss_lever_bias_attribution/{first_update_residual_decomposition.csv,first_update_residual_summary.md}` | residual=`0.059517 m`; `dt_align` 项投影比=`0.786` | gnss_pos_first_residual_source_identified | pass |
| EXP-20260317-data2-gnss-lever-pva-lock-probe-r1 | 2026-03-17 | 用 `PVA` 近零协方差 probe 验证旧“真值锚定”实验存在残差分担 | `output/data2_gnss_lever_pva_lock_probe/*/{first_update_*.csv,state_series_*.csv}` | `release_gnss_lever_y` 首跳 `-0.140808→-0.220985 m`; `z` 首跳 `-0.721939→-1.062293 m` | pva_anchor_mismatch_primary_cause_validated | pass |
| EXP-20260318-data2-gnss-lever-bias-attribution-r1 | 2026-03-18 | 在真正 `runtime PVA anchor` 下 fresh 重跑纯 `INS/GNSS + vel13` 的 stage-B | `output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/{summary.md,plateau_metrics.csv,first_update_debug.csv}` | plateau=`0.149686/-0.220409/-1.139282 m`; `release_gnss_lever_y/z` 首更 `dp_ecef_norm=0/0` | runtime_anchor_stage_b_complete | pass |
| EXP-20260318-data2-ins-gnss-state-sanity-r1 | 2026-03-18 | 在真正 `runtime PVA anchor` 下重判纯 `INS/GNSS` 的 `gnss_lever_y/z` 与代表性 `bg/sg` | `output/data2_eskf_ins_gnss_state_sanity_vel13_r3_runtime_anchor/{summary.md,case_metrics.csv,state_judgement.csv}` | control≈`0`; `gnss_lever_y/z=overall normal`; `bg_y/bg_z/sg_y/sg_z=behavior abnormal, impact normal` | runtime_anchor_state_sanity_subset_complete | pass |
| EXP-20260318-data2-ins-gnss-lever-truth-init-free-run-r1 | 2026-03-18 | 在纯 `INS/GNSS` 下给 `GNSS lever` 真值初值但关闭 `runtime PVA anchor`，检查它是否仍被系统性拉离真值 | `output/data2_ins_gnss_lever_truth_init_probe/{summary.md,manifest.json,case_metrics.csv,lever_deviation_metrics.csv}` | `x/z=persistent_departure`, `y=near_truth`; 首更 `dx` 与 `lever_after-lever_before` 严格对账 | truth_init_free_run_probe_complete | pass |
| EXP-20260318-data2-ins-gnss-pva-component-anchor-r1 | 2026-03-18 | 在 `GNSS lever truth init` 基线上做 `A/B/C` 单分量 `P/V/A` GNSS后锚定，并加入 `full PVA anchor` 曲线对照 | `output/data2_ins_gnss_pva_component_anchor_probe/{summary.md,comparison_summary.csv,lever_case_metrics.csv,manifest.json,plots/*.png}` | `full PVA anchor` 为 overall 上界；单分量里 `position` 最优，`x` 轴最优=`attitude`，`z` 轴最优=`position` | pva_component_anchor_probe_complete_with_full_curve | pass |
| EXP-20260318-data2-ins-gnss-lever-truth-imu-params-r1 | 2026-03-18 | 在纯 `INS/GNSS` 且无 runtime `PVA` 锚定下，把 `ba/bg/sg/sa` 按 `data2/README.md` 的 IMU Markov 参数建模，并从零初值释放 `GNSS lever` 三轴；同时导出 bias/scale 图 | `output/data2_ins_gnss_lever_truth_imu_params_probe/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,imu_truth_model_reference.csv,plots/*.png}` | 已修复 nominal Markov propagation；使用 `POS320` 行后 `ba/bg/sg` 多数轴可随 `modeled truth mean` 衰减，`gnss_lever_z` 仍约 `22.6 cm`；`nav_rmse_3d=0.270254 m` | truth_imu_params_probe_readme_modeled_after_nominal_fix | pass |
| EXP-20260318-data2-ins-gnss-lever-noise-coupling-sweep-r1 | 2026-03-18 | 在 README-IMU 纯 `INS/GNSS` 基线上，对 `GNSS_POS` 有效量测噪声与 `GNSS lever` 过程噪声做 `ESKF/true_iekf` 双模型二维扫描，检查杆臂异常是否主要由参数耦合造成 | `output/data2_ins_gnss_lever_noise_coupling_sweep/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,first_update_metrics.csv,sweep_judgement.csv,plots/*.png}` | smoke 中 `eskf r=1 q=1` 与 `truth_imu_params_probe` 基线逐位复现；local best 仅改善 `16.6%/16.8%`，global best 虽改善 `41.4%/41.6%` 但 `early_plateau_z` 反而恶化约 `18.7%`；首更对 `R` 敏感、对 `Q` 几乎不敏感，且 `ESKF/true_iekf` 表现近乎一致 | noise_coupling_sweep_complete_not_tuning_primary | pass |
| EXP-20260318-data2-turn-window-shared-correction-r1 | 2026-03-18 | 在高转弯窗口内对 `baseline(r=1,q=1)`、`R×0.25`、`R×2` 三组 `ESKF q=1` case 做 `GNSS_POS residual / direct measurement-space correction` 分解，检查 turning 是否真正把 `GNSS lever` 从 `position+attitude` 解耦，并继续追 `GNSS_POS` 更新前 `p/att/gnss_lever` 的 gain-covariance 路由；随后补 matched straight windows 与逐历元 sign-consistency 对照，回答“转弯相对直行是否出现 `x/y` 解耦特征，以及失败到底发生在单步方向还是多历元累计” | `output/data2_turn_window_shared_correction_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,cause_judgement_summary.md,gain_cov_cancellation_summary.{csv,md},xy_focus_case_summary.csv,matched_straight_windows.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}` | top-5 turning windows 仍为 `t≈529536.54/530274.52/528723.77/528123.10/528610.02 s`；`z` 轴已定位到 `p_z-l_gz` cancellation。对用户更关心的 `x/y` 两轴，matched straight 对照显示：turning 相对 straight 的确出现“部分解耦特征”，即 `lever_share_x` 从 `0.0157~0.0361` 升到 `0.0568~0.0889`、`lever_share_y` 从 `0.0052~0.0219` 升到 `0.0630~0.1005`，同时 `pos_share_x` 下降约 `0.047~0.072`、`pos_share_y` 下降约 `0.084~0.103`；但这份 share shift 并没有转化成正确收敛。新的 sign-consistency 进一步表明：baseline turn windows 的 `x` 轴 `dx_sign_match=0.44`、`same_axis_sign=0.52`、`straight dx_sign_match=0.64`，说明 `x` 的同轴 lever 通道已被 turning 弱打开，但 final `dx_lgx` 仍被 cross-axis contamination / window-level reversal 拉坏；而 `y` 轴 baseline 为 `dx_sign_match=0.44`、`same_axis_sign=0.40`、`same_axis_dx_align=0.96`、`straight dx_sign_match=0.52`，说明 `y` 更接近“同轴 lever 通道本身就没有稳定指向真值”，而不是主要靠后续 epochs 才冲坏。因此 turning 不是完全没有解耦特征，而是 `x` 与 `y` 都只打开了不足够的弱通道，且失败主要已经发生在单步 `K*y` 路由层 | turn_window_shared_correction_probe_complete_with_axis_specific_xy_single_step_failure_identified | pass |
| EXP-20260318-data2-turn-window-variant-ab-r1 | 2026-03-18 | 在纯 `INS/GNSS + truth-modeled` 的 turn-window 工位上直接做 4 组 focused A/B：`baseline`、`standard_reset_gamma`、`true_iekf`、`position_anchor_diag`，用同一套 `turn_vs_straight_xy` 与 `turn_vs_straight_xy_sign` 指标判断到底是 reset/IEKF 语义还是 `position` 竞争在主导 `x/y` 杆臂失败 | `output/data2_turn_window_variant_ab_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,matched_straight_windows.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}` | `standard_reset_gamma` 对 `x/y` 几乎零改善；`true_iekf` 也没有把 `x/y` 拉回正确收敛。baseline vs `standard_reset_gamma` 的 turn `dx_sign_match_x/y` 都是 `0.44/0.44`，turn `total_window_improve_x/y` 都约 `-1.62/-2.54 mm`；`true_iekf` 也仍是 `0.44/0.44` 与 `-1.58/-2.51 mm`。只有 `position_anchor_diag` 明显翻正：turn `dx_sign_match_x/y=0.60/0.80`，turn `total_window_improve_x/y=+1.246/+6.215 mm`，且 `temporary_good_then_bad_rate=0/0`。这说明当前 `x/y` 的主矛盾更接近 `position` 竞争/alias，而不是标准 reset 语义或 `true_iekf` 开关本身 | turn_window_variant_ab_probe_complete_with_position_competition_dominant | pass |
| EXP-20260318-data2-turn-window-position-gain-scale-r1 | 2026-03-18 | 把上轮“`position` 竞争是主矛盾”的判断落到工程可行干预上：新增 `fusion.gnss_pos_position_gain_scale`，只在 `GNSS_POS` 更新时缩放 `position(0:2)` 的 Kalman gain 行，并在 pure `INS/GNSS` turn-window 工位上扫描 `1.0 / 0.5 / 0.25 / 0.0` 四档 | `output/data2_turn_window_position_gain_scale_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,matched_straight_windows.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}` | 常数缩放确实能改变 `x/y` 杆臂行为，但不是单调、也不是单一常数就能同时修好两轴。`pos_gain=0.5` 是当前最平衡的档位：turn `x/y total_window_improve=-0.547/-0.708 mm`，相比 baseline `-1.614/-2.534 mm` 明显改善；`pos_gain=0.25` 会把 `y` 翻正到 `+2.213 mm`，但把 `x` 拉坏到 `-2.244 mm`；`pos_gain=0.0` 则几乎把 reversal 消掉并让 `x` 微正 `+0.141 mm`，但 `y` 仍略负 `-0.124 mm`，同时 useful direct correction 也被过度杀伤。结论是：`position` 竞争确实是主因，但简单的全局常数 `position_gain_scale` 还不足以让 `x/y` 同时正常收敛，更像需要条件化/分轴化或分步更新策略 | turn_window_position_gain_scale_probe_complete_with_nonmonotonic_axis_tradeoff | pass |
| EXP-20260318-data2-turn-window-gnsspos-staged-r1 | 2026-03-18 | 继续验证“`position` 竞争是否只是更新次序问题”：新增 `fusion.gnss_pos_update_mode=stage_nonpos_then_pos`，把单次 `GNSS_POS` 拆成“先 non-position、后 position-only”两步，并与 `joint baseline`、`joint pos_gain=0.5`、`staged+0.5` 在同一 pure `INS/GNSS` turn-window 工位对比 | `output/data2_turn_window_gnss_pos_staged_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,matched_straight_windows.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}` | 分步更新并没有优于当前最好的常数 suppression。`staged` 本身 turn `x/y total_window_improve=-1.397/-2.697 mm`，只比 baseline 稍好 `x`、却把 `y` 拉得更差；`staged+0.5` 也仅为 `-0.571/-0.847 mm`，基本不如 `joint+0.5` 的 `-0.547/-0.708 mm`。sign-consistency 也支持同样判断：`staged` 的 turn `dx_sign_match_x/y=0.40/0.44`，不如 `joint+0.5` 的 `0.48/0.52`；`staged+0.5` 也只是回到 `0.44/0.52`，仍未超过 joint。结论是：`position` 竞争真实存在，但问题不只是“position 抢先拿走 innovation”；单纯改变 `GNSS_POS` 内部更新顺序并不能把 `x/y` 拉回正常收敛，主矛盾仍落在 `x` 的 cross-axis contamination 与 `y` 的 same-axis sign instability | turn_window_gnsspos_staged_probe_complete_with_update_order_not_primary | pass |
| EXP-20260318-data2-turn-window-xy-term-source-r1 | 2026-03-18 | 按当前优先级继续下钻 `x/y` 的单步失败来源：在 `turn-window position gain-scale probe` 产物上，把 `dx_lgx/dx_lgy` 显式分解成 `k_lg*_x*y_x + k_lg*_y*y_y + k_lg*_z*y_z` 三项，比较 `baseline / pos_gain=0.5 / 0.25 / 0.0` 的 term-level 变化，确认 `x` 的 cross-axis contaminant 与 `y` 的 same-axis 稳定性问题 | `output/data2_turn_window_xy_term_source_probe/{manifest.json,turn_vs_straight_xy_term_{per_update,window_summary,case_phase_summary,case_summary,summary}.{csv,md}}` | term-source 结果把 `x/y` 的机制差异具体化了。对 `x`，baseline turn `|term_x/y/z|=1.099/0.833/0.229 mm`，而 straight 仅 `0.040/0.047/0.053 mm`；在“same-axis 对、final 仍错”的 updates 里，dominant bad cross term 几乎总是 `y` 项，baseline turn `bad_cross_y_rate=0.60`、`bad_cross_z_rate=0`。`pos_gain=0.5` 时，turn `|term_y|` 从 `0.833→0.748 mm`，`same_good_final_bad_rate` 从 `0.20→0.12`，说明 moderate suppression 之所以能救 `x`，主要是压低了 `k_lgx_y * y_y` 这条污染路径，而不是增强了 same-axis `k_lgx_x * y_x`。对 `y`，baseline turn `|term_x/y/z|=0.657/1.590/0.183 mm`，dominant term 大多直接就是 same-axis `y` 项（rate=`0.80`），但 `same_axis_sign_match` 只有 `0.40`；`pos_gain=0.5` 也只把它拉到 `0.44`，而 `0.25` 才能到 `0.56` 并把 turn `total_window_improve_y` 翻正到 `+2.213 mm`。这说明 `y` 的主问题并不是 cross-axis 污染，而是 same-axis 通道本身就不稳定朝真值指向 | turn_window_xy_term_source_probe_complete_with_x_dominated_by_y_cross_term_and_y_dominated_by_same_axis_instability | pass |
| EXP-20260319-data2-turn-window-lgx-from-y-gain-scale-r1 | 2026-03-19 | 把上一轮 `x` 轴 term-source 归因从“现象级”推进到“干预级”验证：新增 `fusion.gnss_pos_lgx_from_y_gain_scale`，仅缩放 `GNSS_POS` 更新中 `K(l_gx, meas_y)` 这一条路径，并在 `pos_gain=0.5` 基线上扫描 `1.0 / 0.75 / 0.5 / 0.25 / 0.0`，检查是否能只修 `x` 而不误伤 `y` | `output/data2_turn_window_lgx_from_y_gain_scale_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,matched_straight_windows.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}`；`output/data2_turn_window_lgx_from_y_term_source_probe/{manifest.json,turn_vs_straight_xy_term_{per_update,window_summary,case_phase_summary,case_summary,summary}.{csv,md}}` | selective `lgx<-y` suppression 直接验证了 `x` 的 causal blocker。以 `joint+pos_gain=0.5` 为参考，turn `x total_window_improve` 从 `-0.547 mm` 随 suppression 强度连续翻到 `+0.009/+0.564/+1.360/+3.433 mm`，同时 `x dx_sign_match` 从 `0.48` 升到 `0.52/0.56/0.60/0.60`；term-source 里 `|term_y|` 从 `0.748 mm` 继续降到 `0.683/0.554/0.380/0.000 mm`，`same_good_final_bad_rate` 从 `0.12` 降到 `0.08/0.04/0.00/0.00`，`dominant_bad_cross_y_rate` 也从 `0.40` 降到 `0.40/0.20/0.00/0.00`。相对地，`y` 没有随之翻正，turn `y total_window_improve` 反而从 `-0.708 mm` 变为 `-0.727/-0.768/-0.835/-1.024 mm`，说明 `x` 的问题确实主要是 `k_lgx_y * y_y` 污染，而 `y` 的主问题仍是 same-axis `k_lgy_y * y_y` 自身方向不稳 | turn_window_lgx_from_y_gain_scale_probe_complete_with_x_causal_path_validated_and_y_unfixed | pass |
| EXP-20260319-data2-turn-window-y-same-axis-sign-diagnostic-r1 | 2026-03-19 | 接续 `x` 轴已验证的 selective result，专门拆 `y` 轴 same-axis 通道的失稳来源：联合读取 `position_gain_scale probe` 与 `lgx_from_y selective probe`，检查 `k_lgy_y * y_y` 的 sign match 究竟受 `innovation_y`、`k_lgy_y` 还是 turn direction 哪一层主导，并解释为什么 `pos_gain=0.25` 能把 `y` 翻正 | `output/data2_turn_window_y_same_axis_sign_diagnostic/{summary.md,manifest.json,y_same_axis_{per_update,case_phase_summary,turn_direction_summary,window_summary,case_comparison}.csv}` | `y` 轴问题已从“same-axis 自身不稳”收敛成“turn-direction-conditioned 的 gain-sign mismatch”。在 turn windows 中，`desired_dx_y` 基本恒为负，而 baseline / `pos_gain=0.5` 的 `innovation_y` 正号率分别仍为 `0.760/0.760`；真正拉低 `same_axis_sign_match_y` 的，是 `k_lgy_y` 没有按 `desired_dx_y * innovation_y` 的需要取到正确符号。更具体地，baseline / `pos_gain=0.5` 的正向转弯窗口 `same_axis_sign_match_y` 只有 `0.267/0.267`，但负向转弯可到 `0.600/0.700`；`innovation_y` 正号率在正向转弯里几乎不变（都约 `0.667`），而 `pos_gain=0.25` 把正向转弯的 `k_lgy_y<0` 比例从 `0.467` 提到 `0.533`，对应 `same_axis_sign_match_y` 从 `0.267` 拉到 `0.600`，并让 turn `total_window_improve_y` 翻正到 `+2.213 mm`。相对地，后续 `x-only selective suppression` 只把 `y` 的正向转弯 `same_axis_sign_match_y` 微调到 `0.333`，整体仍停在负改善区间，说明 `y` 需要独立的 targeted fix，而不是继续依赖 `x` 通道干预 | turn_window_y_same_axis_sign_diagnostic_complete_with_positive_turn_k_sign_mismatch_identified | pass |
| EXP-20260319-data2-turn-direction-proxy-r1 | 2026-03-19 | 在进入 solver 级 `y` 修复前，把离线 `POS yaw_rate` 与运行时可得的 turn-direction proxy 逐个对账，比较 `DIAG omega_z`、`SOL yaw-rate` 与速度曲率代理在 turn-window `GNSS_POS` 更新时刻的符号一致性 | `output/data2_turn_direction_proxy/{summary.md,manifest.json,proxy_per_update.csv,proxy_case_summary.csv,proxy_overall_summary.csv}` | `IMU omega_z` 已可作为稳定在线 proxy：在 `data2_turn_window_lgx_from_y_gain_scale_probe` 的全部 6 个 case、共 `150` 个 turn-window `GNSS_POS` updates 上，`imu_turn_sign` 与离线 `POS yaw_rate` 符号 direct match=`1.000`、inverted=`0.000`；相对地，`SOL yaw-rate` 与 `SOL curvature` 仅有 `0.727/0.640` 的 weighted best-match。并且 `IMU omega_z` 的正/负号与 `POS yaw_rate` 完全同向，不需要额外翻转映射。这说明后续针对 `y` 的 turn-conditioned 干预可以直接挂在运行时 `omega_z` 上，而不再依赖离线真值定义 | turn_direction_proxy_validated_with_runtime_imu_omega_z_as_official_online_proxy | pass |
| EXP-20260319-data2-turn-window-y-turn-conditioned-position-r1 | 2026-03-19 | 基于已验证的 `omega_z` 在线 proxy，在 `pos_gain=0.5` 基线上新增 `fusion.gnss_pos_turn_rate_threshold_deg_s / gnss_pos_positive_turn_position_gain_scale`，并将“强正向转弯时把 `position` competition 从 `0.5` 进一步压到 `0.25`”与 `x` 侧 `lgx<-y` suppression reference family 组合，检查是否能同时改善 `x/y` | `output/data2_turn_window_y_turn_conditioned_position_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}`；`output/data2_turn_window_y_turn_conditioned_position_diagnostic/{summary.md,manifest.json,y_same_axis_{per_update,case_phase_summary,turn_direction_summary,window_summary,case_comparison}.csv}`；`output/data2_turn_window_y_turn_conditioned_position_term_source/{manifest.json,turn_vs_straight_xy_term_{per_update,window_summary,case_phase_summary,case_summary,summary}.{csv,md}}` | 条件化 mitigation 已把 `x/y` 同时拉入正改善区，但其机理更接近“幅值抑制”而不是“符号修复”。最佳组合为 `pos_gain=0.5 + lgx_from_y_gain_scale=0.25 + positive-turn position gain=0.25 (|omega_z|>=8 deg/s)`：turn `x/y total_window_improve=+2.590/+1.015 mm`，turn `dx_sign_match_x/y=0.640/0.560`，turn `lever_share_x/y=0.0785/0.1111`，`pos_share_x/y=0.8392/0.8022`。更重要的是，这个收益确实集中在此前失效的正向转弯窗口：例如 `turn_window_1/4/5` 的 `lever_y_error_improve` 从 `-6.845/+0.280/+2.817 mm` 改到 `-2.878/+3.792/+4.496 mm`，而负向转弯窗口基本保持原有水平。然而 `y same-axis` 诊断又显示根因并未被真正修复：`joint_pos_gain_0p5_ref -> joint_pos_gain_0p5_pos_turn_0p25` 的正向转弯 `same_axis_sign_match_y` 仍是 `0.267 -> 0.267`，`k_negative_rate` 仍是 `0.467 -> 0.467`；即便叠加 `lgx_from_y=0.25`，正向转弯 `same_axis_sign_match_y` 也只停在 `0.333`。真正变化的是正向转弯 wrong-sign same-axis 通道的幅值：`mean_k_lgy_y` 从 `0.004779 -> 0.001966`（带 `lgx_from_y=0.25` 时 `0.005113 -> 0.002182`），`mean_same_axis_term_y` 从 `+0.523 -> +0.081 mm`（带 `lgx_from_y=0.25` 时 `+0.563 -> +0.068 mm`），对应 term-source 的 turn `|term_y|` 从 `1.303/1.339 mm` 降到 `1.043/1.050 mm`。因此该实验证明：`y` 的问题不是“缺少激励”或“缺少在线 turn proxy”，而是正向转弯下 `k_lgy_y * y_y` 的有效符号/几何仍然不对；当前条件化 position suppression 只能先把伤害压小 | turn_conditioned_position_probe_complete_with_xy_positive_and_y_root_cause_still_sign_geometry_mismatch | pass |
| EXP-20260319-data2-turn-window-lgy-from-y-gain-scale-r1 | 2026-03-19 | 在当前 best combined reference family `pos_gain=0.5 + lgx_from_y_gain_scale=0.25 + positive-turn position gain=0.25 (|omega_z|>=8 deg/s)` 上，新增 `fusion.gnss_pos_lgy_from_y_gain_scale` 与 `fusion.gnss_pos_positive_turn_lgy_from_y_gain_scale / negative_turn_*`，直接扫描 `K(l_gy, meas_y)` 的正向转弯 selective suppression，检查它能否真正修复 `y(29)` 的 same-axis sign mismatch，还是仍只是在压低 wrong-sign 幅值 | `output/data2_turn_window_lgy_from_y_gain_scale_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}`；`output/data2_turn_window_lgy_from_y_gain_scale_diagnostic/{summary.md,manifest.json,y_same_axis_{per_update,case_phase_summary,turn_direction_summary,window_summary,case_comparison}.csv}`；`output/data2_turn_window_lgy_from_y_gain_scale_term_source/{manifest.json,turn_vs_straight_xy_term_{per_update,window_summary,case_phase_summary,case_summary,summary}.{csv,md}}` | 直接控制 `lgy<-y` 路径只能带来边际改善，不能取代当前 best combined reference。相对 `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref` 的 turn `x/y total_window_improve=+2.590/+1.015 mm`，`pos_turn_lgy_from_y=0.75/0.5/0.25/0.0` 仅得到 `+2.536/+1.042`、`+2.476/+1.045`、`+2.411/+1.025`、`+2.338/+0.979 mm`。其中 `0.75` 是唯一把正向转弯 `same_axis_sign_match_y` 从 `0.333` 拉到 `0.400` 的档位，但幅度仍远低于此前 `pos_gain_0.25` 所达到的 `0.600`；更强 suppression 虽持续压低 turn `|term_y|`（`1.050→0.939→0.796→0.590→0.316 mm`）并把 turn `dx_sign_match_y` 推到 `0.720`，却不会稳定改善正向转弯的符号匹配，而且 `pos_turn_lgy_from_y=0.0` 会让正向转弯 same-axis sign 指标直接失去定义。逐窗上，收益主要仍集中在正向转弯 `turn_window_1/4/5`，例如 `turn_window_1` 从 `-2.895 mm` 改到 `-2.066/-1.353/-0.840/-0.582 mm`，但始终未被修好；与此同时负向转弯 `turn_window_3` 会从 `-4.403 mm` 进一步恶化到 `-4.882 mm`。因此当前证据支持：`y` 的主问题仍是正向转弯下 `k_lgy_y` 的生成机制/几何不对，直接压 `K(l_gy, meas_y)` 也主要是在减小 wrong-sign same-axis 幅值，而不是恢复正确符号 | turn_window_lgy_from_y_gain_scale_probe_complete_with_marginal_y_gain_and_positive_turn_sign_generation_still_unfixed | pass |
| EXP-20260319-data2-turn-window-lgy-numerator-geometry-r1 | 2026-03-19 | 在 fresh `lgy_from_y gain-scale probe` 基础上，把 `GNSS_POS` 调试输出扩展到 `S`、`H_pos`、`raw K(l_gy,:)` 与 `num_lgy=P(l_gy,*)H^T` 分块向量，直接比较 `turn_window_1/4/5` 与 `turn_window_2/3`，确认 `k_lgy_y` 的 raw sign 到底主要由同轴 numerator、`S^{-1}` 的 off-axis 混合，还是 `H_y` 几何差异主导 | `output/data2_turn_window_lgy_numerator_geometry_analysis/{summary.md,manifest.json,lgy_numerator_geometry_{per_update,window_summary,focus_comparison}.csv}`；fresh upstream source from `output/data2_turn_window_lgy_from_y_gain_scale_probe/*` | numerator/geometry 结果把 `y` 的根因再往前收了一层。首先，`raw_k_lgy_y` 基本由 same-axis numerator 决定，而不是 `S^{-1}` 的 `x/z` 混合：在 best reference 上，`turn_window_1/2/3/4/5` 的 `raw_k_y_from_num_y` 分别为 `+0.01049/-0.00005/-0.01584/-0.00362/-0.00057`，而对应 `raw_k_y_from_num_x/z` 仅约 `1e-4~1e-3`。其次，`att` 几乎可以排除：所有 focus windows 的 `num_lgy_from_att_y` 仅 `2e-5~3e-5`，同时 `h_y_pos` 始终固定为 `[0,1,0]`，说明 `position y` 几何本身没有随 turn sign 改写。真正决定 `num_lgy_y` 符号的，是 `num_y(pos)` 与 `num_y(gnss_lever)` 的近抵消及其残差符号：正向 `turn_window_1/5` 分别是 `+0.003082/-0.002571` 与 `+0.005550/-0.005260`，留下小的正残差 `+5.40e-4/+3.19e-4`；而同样是正向的 `turn_window_4` 却变成 `-0.007100/+0.006613`，留下负残差 `-5.19e-4`。这说明“正向转弯”内部至少还有两类不同的 covariance/geometry family，`turn_window_1/5` 与 `turn_window_4` 不能简单合并。再次，`turn_window_3` 在 `pos_turn_lgy_from_y=0` 时继续恶化，并不是因为该窗口里被直接 zero 掉了 `K(l_gy,meas_y)`；它是负向转弯，`post_k_y` 仍等于 `raw_k_y`（`-0.015535 -> -0.014971` 量级），因此更像 earlier positive-turn interventions 改写了进入该窗口前的 state/covariance history。最后，`pred_ant_ecef -> innovation` 的 strict cross-check 仍未闭环：focus windows 的 `innovation_recon_error_norm_max` 还在 `0.877~1.432 m`，当前几何重构链还不能直接升格为正式主证据 | turn_window_lgy_numerator_geometry_analysis_complete_with_raw_ky_dominated_by_pos_vs_lever_numerator_cancellation | pass |
| EXP-20260319-data2-turn-window-lgy-lever-axis-focus-r1 | 2026-03-19 | 在 `k_lgy_y` numerator 已确认由 `pos` vs `lever` 近抵消主导之后，继续把 `lever` 自块拆到 `lgx/lgy/lgz` 三轴，并只对 `best reference` 与 `positive-turn lgy_from_y=0` 两个 focused case fresh rerun，检查正向转弯 family split 到底来自 `gnss_lever` 自协方差符号翻转，还是来自 `H_y` 几何项 | `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/{summary.md,manifest.json,case_update_summary.csv}`；`output/data2_turn_window_lgy_lever_axis_focus_analysis/{summary.md,manifest.json,lgy_numerator_geometry_{per_update,window_summary,focus_comparison}.csv}` | `lever` 自块的 `y` numerator 已进一步收敛到单一主导轴：所有 focus windows 与两个 cases 中，dominant lever axis 始终是 `lgy`，`num_y(lever_lgx/lgy/lgz)` 的 `lgy` 项约为 `±0.002041~0.006678`，而 `lgx` 仅 `1.3e-05~1.7e-04`、`lgz≈1e-07`。更关键的是，`cov(lgy,lgy)` 在所有窗口都保持正值 `0.004433~0.010421`，因此 `lever` 自块符号不是由自协方差反号决定，而是几乎完全由 `H_y_lever_y` 的几何符号决定：`turn_window_1/5` 的 `h_y_lever_y=-0.4917/-0.6354`，于是 `num_y(lever_lgy)` 为负；`turn_window_2/3/4` 的 `h_y_lever_y=+0.4584/+0.7715/+0.6411`，于是 `num_y(lever_lgy)` 为正。结合 `h_y_pos=[0,1,0]` 固定不变，可见正向 family split 不是“`lgy` 自协方差方向突然翻了”，而是 `P(l_gy,p_y)` 与 `h_y_lever_y` 两条路径在不同窗口共同改号；其中 `turn_window_4` 与 `turn_window_1/5` 的差异已经可以明确落到 geometry sign family，而不是 axis-mixing。另一方面，focused rerun 也再次确认 `positive-turn lgy_from_y=0` 不是新 reference：turn `x/y total_window_improve` 从 reference `+2.590/+1.015 mm` 退到 `+2.338/+0.979 mm`，且 `pred_ant_ecef -> innovation` 重构误差仍在 `0.877~1.432 m`，几何链还不能升格为主证据 | turn_window_lgy_lever_axis_focus_complete_with_lgy_self_axis_dominant_and_geometry_sign_family_identified | pass |
| EXP-20260319-data2-turn-window-lgy-history-r1 | 2026-03-19 | 直接复用 focused rerun 的 fresh `gnss_updates_*.csv`，补 `20 s` pre-window history，追 `cov(lgy,p_y)`、`cov(lgy,lgy)`、`h_y_lever_y` 与 `lever_before_y` 的时序，回答正向 family split 是何时建立的，并单独检查 `turn_window_3` 为何会在 `positive-turn lgy_from_y=0` 下继续恶化 | `output/data2_turn_window_lgy_history_analysis/{summary.md,manifest.json,lgy_history_{per_update,window_summary,family_summary}.csv,turn_window_3_case_diff.csv}` | history 证据把 `y` 的 family split 进一步细化到“进入窗口前后”的时序层：`positive_family_1_5` 的 pre/in-window `cov(lgy,py)>0 rate=0.88/0.90`、`h_y_lever_y>0 rate=0.12/0.10`，而 `positive_family_4` 为 `0.10/0.00` 与 `0.95/1.00`，说明 `1/5` 与 `4` 的差异确实落在 `P(l_gy,p_y)` 与 `H_y(lever_y)` 的符号组合上，而不是 `cov(lgy,lgy)` 本身。更细地说，`turn_window_5` 从 entry `h_y_lever_y=-0.214` 到窗口末 `-0.969` 都保持负几何，`turn_window_4` 则从 `+0.231` 到 `+0.972` 都保持正几何；`turn_window_1` 不是一开始就属于 `1/5` family，而是在窗口内 `h_y_lever_y` 从 entry `+0.061` 快速翻到 `-0.978`。对 `turn_window_3` 的 reference vs `positive-turn lgy_from_y=0` 对照又表明：在最后一个 pre-window update 就已出现 `Δlever_before_y=-2.360 mm`、`Δcov(lgy,p_y)=-3.61e-4`、`Δcov(lgy,lgy)=-9.7e-5`，但 `Δh_y_lever_y` 仅 `+2.2e-05`；entry 时 `Δraw_k_lgy_y=-0.001692`，窗口指标则从 `-4.403 mm` 继续恶化到 `-4.882 mm`。这说明 `turn_window_3` 的退化主要是 earlier state/covariance carry-over，而不是该窗口局部几何突然翻坏 | turn_window_lgy_history_analysis_complete_with_family_split_timed_and_turn_window_3_carry_over_validated | pass |
| EXP-20260319-data2-turn-window-lgy-transition-carryover-r1 | 2026-03-19 | 在已有 `lgy history` 结论上继续做两件事：一是把 `turn_window_1/4/5` 压缩成 stable sign-family 路径，明确 `window_1` 的 transition 序列；二是把 `reference` vs `positive-turn lgy_from_y=0` 的差分扩展到全程 `GNSS_POS`，定位从 `turn_window_4` 开始到 `turn_window_3` 的 control/state carry-over 链 | `output/data2_turn_window_lgy_transition_carryover_analysis/{summary.md,manifest.json,positive_turn_family_segments.csv,positive_turn_pair_comparison.csv,full_case_diff_per_update.csv,carryover_checkpoint_summary.csv}` | transition/carry-over 证据把当前主线从“窗口级描述”推进到“链路级定位”。对 `window_1/4/5`，stable sign-family path 分别为：`turn_window_1: +-(16) -> ++(1) -> -+(4) -> +-(4)`、`turn_window_4: -+(6) -> transient(3) -> -+(10) -> transient(1) -> -+(5)`、`turn_window_5: +-(4) -> transient(5) -> +-(8) -> --(1) -> transient(1) -> +-(6)`。这说明 `window_4` 的 stable family 实际上全程维持 `(-cov_py,+h)`，`window_5` 则基本维持 `(+cov_py,-h)`，而 `window_1` 的关键不是单一 family，而是 pre-window 先后经过 `+- -> ++ -> -+`，再在第二个 in-window update 回到 `+-`。更关键的是，全程 case diff 已定位到首个分叉点：`first_control_divergence` 就发生在 `turn_window_4_entry @ 528121 s`，此时只有 `post K(lgy,y)` 被改写（`Δpost_k=-0.074382`），state/cov 仍相同；`first_state_divergence` 则紧接着出现在 `528122 s`，已经有 `Δlever_before_y=+3.777 mm`、`Δcov(lgy,p_y)=+0.001008`、`Δcov(lgy,lgy)=+1.022e-4`。随后这条差分链会穿过 `window_4 -> window_5 -> window_3` 持续传播：`turn_window_4_last` 为 `dlever=+1.922 mm`、`dcov_py=-0.002641`，`turn_window_5_entry/last` 分别为 `-0.949/+1.811 mm` 与 `-2.65e-4/+0.002179`，直到 `turn_window_3_last_pre` 仍保留 `dlever=-2.360 mm`、`dcov_py=-3.61e-4`。因此 `turn_window_3` 的恶化现在已经不仅知道“是 carry-over”，还可进一步定位为：它的最早控制分叉来自 `turn_window_4` 的 positive-turn suppression，而不是 `turn_window_3` 自身 | turn_window_lgy_transition_carryover_complete_with_window1_path_and_window4_origin_of_turn3_carryover_identified | pass |
| EXP-20260319-data2-turn-window-lgy-key-update-audit-r1 | 2026-03-19 | 在已有 `history + transition/carry-over` 基础上，把 `window_1` 的三次关键翻转点做成 update 级 driver audit，并对 `turn_window_4 -> window_5 -> window_3` 的差分链补 interval-level top-update ranking，明确哪些非窗口 `GNSS_POS` 更新在续命这条 carry-over 链 | `output/data2_turn_window_lgy_key_update_audit/{summary.md,manifest.json,window1_transition_context.csv,window1_transition_key_updates.csv,carryover_interval_summary.csv,carryover_top_updates.csv}` | key-update audit 把当前 unresolved 进一步压缩成可执行的下一步清单。`window_1` 的三次关键翻转不再只是路径描述，而已明确分成三类：`529531 s` 是 `geometry_only`（`Δh_y_lever_y=+0.181968`，而 `cov(lgy,p_y)` 仍保持正值 `+0.000190`）；`529532 s` 是 `covariance_only`（`Δcov(lgy,p_y)=-0.001908`，同时 `h_y_lever_y` 继续增到 `+0.325435`）；`529536 s` 则是 `joint_covariance_and_geometry`（`Δcov(lgy,p_y)=+0.001445` 且 `Δh_y_lever_y=-0.214149`），因此 `window_1` 的 `+- -> ++ -> -+ -> +-` 路径不是单一机制，而是“几何先翻、协方差再翻、入窗后两者共同翻回”。另一方面，carry-over survival ranking 又表明：`turn_window_3` 的分叉并不是靠窗口内几次更新单独维持。`state_divergence -> window_5_entry` 区间内最大的再放大已落在非窗口 `GNSS_POS` 更新：`528127 s` 给出 `step Δcov(lgy,p_y)=+0.004773` 与 `step Δpost_k=+0.097138`，`528128 s` 给出 `step Δlever_before_y=-2.565 mm`，`528158 s` 也再次提供 `step Δcov(lgy,p_y)=-0.002103`。`window_5_entry -> window_3_entry` 区间则由 `turn_window_5 @ 528609/528610 s` 与非窗口 `528614/528615 s` 共同续命：前者给出 `step Δlever_before_y=+3.028/+1.519 mm`，后者给出 `step Δcov(lgy,p_y)=-0.004151/+0.001705`、`step Δpost_k=-0.083641/+0.031571` 与 `step Δlever_before_y=-2.237 mm`。这说明 `turn_window_3` 的退化已经可以明确建模成“窗口内 + 非窗口 mixed GNSS_POS chain”的长链存活，而不是单一 turn-window 故障 | turn_window_lgy_key_update_audit_complete_with_window1_driver_classes_and_non_window_survival_updates_identified | pass |
| EXP-20260320-data2-turn-motion-stats-r1 | 2026-03-20 | 统计 `data2` 全程运动中的转弯时间占比，并按 `POS yaw_rate` 符号拆分左/右转事件，为后续“转弯丰富度”描述提供可复现口径 | `output/data2_turn_motion_stats/{summary.md,turn_segments.csv,threshold_sensitivity.csv,manifest.json}` | main rule (`speed>=3 m/s`, `|yaw_rate|>=8 deg/s`, `min_turn_duration>=1 s`): turn ratio=`9.6239%` (`211.901/2201.806 s`), left/right=`19/25`; sign sanity=`0.996978` | turn_motion_stats_complete | pass |
| EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1 | 2026-03-20 | 在 `README-IMU` 纯 `INS/GNSS` 基线上分离扫描 `GNSS lever` 初始 STD/P0 与过程噪声 `Q`，并把噪声路线与 archived baseline、turn-window gain-tuned reference 放到同一套全程杆臂/导航指标下对比 | `output/data2_ins_gnss_lever_p0_q_sweep/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,candidate_selection.csv,reference_metrics.csv,best_case_comparison.csv,plots/*.png}` | archived baseline `p0=0.2,q=1` 被逐位复现；stage1 最优为 `p0_std=1.0 m,q=1`，`final_lever_dev_norm=0.230047→0.111468 m` (`-51.5%`)，`z_final_abs_error=0.226201→0.105510 m`，`nav_rmse_3d=0.270254→0.088308 m`；在该 `P0` 上把 `Q` 降到 `0.1x` 仅再改进到 `0.108377 m`（额外 `2.77%`），说明当前噪声路线里 `P0` 主导、`Q` 次级；按 full-trajectory global lever norm，这组 noise-only best 甚至优于现有 turn-window gain-tuned reference 的 `0.275873 m` | gnss_lever_p0_q_sweep_complete_with_p0_dominant_noise_route_identified | pass |

### 历史关键实验摘要

| 主题 | 涉及 exp_id | 保留结论 | 为何仍重要 | archive_ref |
|---|---|---|---|---|
| official outage 源重置与标准化 | `EXP-20260315-data2-rtk-outage-eskf-r1`、`EXP-20260316-data2-rtk-outage-eskf-r2`、`EXP-20260316-data2-eskf-baseline-output-r1` | data2 正式口径已切到 corrected RTK + standardized output；更早旧 GNSS source 结果只保留为历史。 | 它定义了当前哪些 `data2` 指标还能被当作正式 evidence。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_02_official_outage_transition` |
| data2 GNSS30 的 NHC 扫频链 | `EXP-20260311-odo-nhc-rate-sweep-r2*`、`EXP-20260312-inekf-mechanism-attribution-r2`、`EXP-20260312-inekf-mechanism-rate-vs-weight-r2` | data2 的 `NHC` 最优频率具有算法依赖和 dataset boundary，不能简单迁移。 | 它仍支撑 `HYP-13` 与 `InEKF` 机制讨论。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_01_pre_official_outage_research_stack` |
| data4 `P0_diag` 根因与 canonical case 重建 | `EXP-20260312-data4-p0diag-check-r1`、`EXP-20260312-data4-gnss30-nhc-sweep-r2`、`EXP-20260313-dataset-report-cases-r4` | data4 ESKF 巨大冲突主要来自初始化协方差语义，不是 GNSS schedule 单因子。 | 它仍是 data4 口径解释的关键历史证据。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_01_pre_official_outage_research_stack` |
| 更早 InEKF 初版与 phase-2 主修复 | `EXP-20260304-*`、`EXP-20260305-*`、`EXP-20260306-*`、`EXP-20260307-*` | RI 版 InEKF、phase-2、reset/Jacobian 审计与 post-GNSS 因果链的更早正文已移出主文档。 | 当前 `HYP-7/HYP-8/HYP-11/HYP-12` 仍以这些阶段为技术背景。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_experiment_themes` |
## 已知不一致项（Known Inconsistencies）

| issue_id | 发现日期 | 描述 | 影响 | 处理状态 |
|---|---|---|---|---|
| ISSUE-20260315-gnss-source-mismatch-reset | 2026-03-15 | 用户确认 `data2` 正式 GNSS 源应为 `dataset/data2/rtk.txt`；旧 `GNSS_converted.txt` 结果已整体降级。 | 当前只有 corrected RTK 证据链可直接当正式结论引用。 | partially_resolved_data2_outage_only |
| ISSUE-007-inekf-tex-code-gap | 2026-03-06 | `ODO/NHC` Lie 核心 Jacobian 与 reset 已更贴近文档，但 `GNSS_VEL` 与 `21-30` 扩展块仍未完全 invariant/estimate-independent。 | 限制当前 `true_iekf` 的“理论完全贴合”表述强度。 | partially_resolved_phase2 |
| ISSUE-20260312-representative-report-data4-stale | 2026-03-12 | `representative_navigation_interactive_report.html` 仍含历史 data4 页面，尚未整体刷新到 `r4/r5` 证据链。 | 若继续把 representative HTML 当最终结论页，仍需补刷新或加历史标注。 | partially_resolved_config_fixed_report_pending_refresh |
| ISSUE-20260313-cpp-app-layer-overgrowth | 2026-03-13 | `src/app/` 仍承载过多研究期逻辑；仅 `dataset_loader.cpp` 已抽离，`config/diagnostics` 仍偏重。 | 提高后续维护与误改风险。 | partially_resolved_dataset_loader_extracted |
| ISSUE-20260317-eskf-standard-reset-gap | 2026-03-17 | 当前标准 ESKF 路径姿态注入后没有像 `true_iekf` 一样对协方差施加显式 reset Gamma。 | 可能影响 `bg/sg/sa/mounting/lever` 等姿态相关互协方差建立。 | open_static_audit_ab_needed |
| ISSUE-20260318-imu-markov-nominal-gap | 2026-03-18 | `ba/bg/sg/sa` 在协方差传播里使用了 `markov_corr_time` 的一阶 Markov `F=-I/T`，但 nominal state 在 `Predict()` 中并没有按 `-x/T` 自身衰减，只在 correction 时做加性注入。 | 先前会导致 README 构造的 `modeled truth mean` 与 `est` 明显语义错位。当前已在 `EskfEngine::Predict()` 中补上 nominal `exp(-dt/T)` 衰减；后续若再看偏差，优先解释为可观性/更新链而非 nominal 模型缺项。 | resolved_by_predict_nominal_markov_decay |

## 开放假设（Open Hypotheses）

| hyp_id | 假设 | 当前证据 | 下一步验证 | 状态 |
|---|---|---|---|---|
| HYP-1 | data2 InEKF 与 ESKF 差异主要来自 `GNSS_POS`-RI 耦合与过程噪声映射，而非 `GNSS_VEL`。 | `p_local=off` 会灾难性发散；但当前仍缺强机械因果链。 | 补 `GNSS_POS` 时段级因果解释。 | open(blocker: mechanistic_causality_missing) |
| HYP-6 | RI 误差约定下，`G(kVel,gyro)` 更合理实现是 `+Skew(v_ned)C_bn`。 | data2/data4 的最佳开关仍呈 dataset-specific 差异。 | 解释这一开关对不同数据集的边界。 | open(conditionally_supported) |
| HYP-7 | true IEKF 的主要收益先体现在 `GNSS_POS` Lie 核心结构修正上；若不继续重写 ODO/NHC、GNSS_VEL 和 `21-30` 扩展块，sparse-GNSS 性能未必改善。 | baseline 显著改善，但 GNSS30 不稳定；`no_odo/no_nhc` 会明显恶化。 | 继续审计 road-constraint 与 full-state reset。 | open(very_strongly_supported) |
| HYP-8 | `true_iekf` 在 sparse-GNSS 下的主要 blocker 不是 `GNSS_VEL`，而是 road-constraint 通道与 Lie-core process/reset 的组合失配。 | `veljac=eskf` 近乎 neutral；`process=eskf` 与 `reset=I` 单独都会显著退化。 | 继续检查 `21-30` 扩展块与 `att_z/bg_z` 协方差演化。 | open(refined_to_process_reset_nonadditivity) |
| HYP-9 | data2 GNSS30 的剩余 blocker 更像 GNSS 关闭后 `GNSS_VEL/21-30/激励条件` 的 dataset-specific 组合，而不是 `GNSS_POS` 或基础 ODO/NHC core Jacobian。 | phase-2 后 data2 GNSS30 仍远差于 hybrid，而 data4 GNSS30 已明显收敛。 | 继续做 data2 post-GNSS 漂移与互协方差诊断。 | open |
| HYP-10 | `data4 GNSS30` 优于 `data2 GNSS30` 的主因是工况更短、更慢且更依赖 `ODO/NHC`，不是“算法偏爱 data4”。 | blackout 时长/里程/速度与 phase-2 精简消融都支持该解释。 | 若有需要，继续做 heading/bias 漂移率对比。 | open(very_strongly_supported) |
| HYP-11 | `data2 GNSS30` 的主残余 blocker 已转成 post-GNSS `bg_z` 主导的 heading drift；`mounting` 放大位置误差但不是首因。 | `freeze_bg`/`freeze_bg_mount` 直接改善 yaw growth；`GNSS_VEL` 缺失不是主导解释。 | 决定是继续修 `bg` 过程/更新，还是采用 pragmatic freeze。 | open(validated_by_direct_intervention) |
| HYP-12 | 当前 phase-2 代码下，剩余 data2 gap 不太可能来自 `ODO/NHC/GNSS_VEL` measurement Jacobian；更像 GNSS 期间 `att-bg_z` 约束建立不足。 | Jacobian audit 与 reset consistency audit 已排除“整列反号”；fresh runtime 也确认 `GNSS_VEL` 在线接受。 | 追踪 GNSS 有效窗口内 `P[att,bg_z]`、gain 与 `dx_bg_z` 时序。 | open(narrowed_to_gnss_coupling_build_up_and_runtime_confirmed) |
| HYP-13 | 对 `data2 baseline ESKF` 而言，`NHC` 高于适度频率反而可能过强；保留 ODO 原始高频、把 NHC 降到约 `20Hz` 更有利。 | `ODO raw + NHC 20Hz` 优于 raw/raw，而 `ODO 20Hz + NHC raw` 最差。 | 检查该规律是否能迁移到更多场景。 | open(strongly_supported_on_data2_baseline) |
| HYP-18 | 当前 4 个 `GNSS30` canonical cases 的显式 `Q/R` 仍非统一局部最优，且最敏感 noise family 具明显 case-specific 差异。 | `noise-qr` sweep 中 4 个 case 的 best profile 明显不同，且 transferability 有边界。 | 继续审计 `r5` tradeoff case 的 consistency / physics。 | open(shared_profile_transferability_mixed) |
| HYP-19 | 在 `data2 official outage` 下，大多数非 PVA 扩展状态释放后不会稳定收敛；shared culprit 更像 `road-constraint coupling + standard ESKF reset/covariance semantics`。 | `22` 状态矩阵为 `18 abnormal / 1 borderline / 3 normal`；`standard_reset_gamma` 会改善控制组但不能消除状态异常。 | 深入做 `ODO/NHC -> bg/sg` correction-level 诊断。 | open(refined_to_reset_plus_road_constraint_coupling_no_simple_rate_fix) |
| HYP-21 | 即使移除 `ODO/NHC` 及其强相关状态块，`ba/bg/sg/sa` 也未必会从零初值稳定恢复；大量异常并不只是 road-constraint 本身造成。 | 纯 `INS/GNSS` 下控制组导航可很小，但 `ba/bg/sg/sa` behavior 仍多为 abnormal；`sigma_bg×10` 只改善部分 `bg_z`。 | 区分“更新耦合不足”和“过程模型过硬”的相对权重。 | open(strongly_supported_ins_gnss_recoverability_gap_persists_after_vel13_alignment_and_r2_reproduction) |
| HYP-24 | 在纯 `INS/GNSS` 下，仅修正 `GNSS lever` 初值或仅把 `ba/bg/sg/sa` 作为真值已知，都不足以替代 runtime `PVA` 锚定；剩余杆臂偏差仍主要受 `GNSS_POS` shared correction / free-PVA correction chain 影响。 | `truth-init free-run probe` 中 `gnss_lever_x/z` 分别出现 `0.037972/0.102767 m` 的最终偏差；README 参数驱动的 `truth_imu_params_probe` 中即使 `ba/bg/sg/sa` 使用 `POS320` 行的 `sigma_bg=0.5 deg/h`、`sigma_ba=25 mGal`、`sigma_sg/sa=300 ppm`、`tau=4 h` 建模，`gnss_lever_z` 仍有 `0.226201 m` 最终误差；进一步的 `noise_coupling_sweep` 显示，即使把 `GNSS_POS` 有效噪声与 `GNSS lever Q` 扫到 `R×{0.25..4}`、`Q×{0.1..10}`，两个模型在合理局部范围内也只改善约 `16-17%`，且 global best 的 `early_plateau_z` 反而变差。 | 直接转向 `GNSS_POS residual / shared correction` 的结构性归因，不再把简单 `R/Q` 调参当主解释；后续若再看 bias/scale，只做净贡献对照。 | open(very_strongly_supported_after_noise_coupling_sweep) |
| HYP-25 | 在纯 `INS/GNSS + GNSS lever truth init` 口径下，杆臂偏差的 dominant single-component PVA source 更偏向 `position`；`x/y` 两轴在 turning 相对 straight 时会出现“部分解耦特征”（`lever share` 上升、`position share` 下降），但这份解耦并没有形成朝真值稳定积累的有效更新，而且 `x` 与 `y` 的失败层级并不相同。进一步的 focused A/B、gain-scale、staged-update 与 term-source 实验表明：决定性因素确实更像 `position` 竞争/alias，但 `x` 的真正 blocker 已可具体落到 `k_lgx_y * y_y` 主导的 cross-axis contamination，而 `y` 的 blocker 更接近 same-axis `k_lgy_y * y_y` 自身的 sign instability。最新 selective intervention 已直接验证 `x` 的因果链，而最新 `y`-diagnostic、numerator geometry、lever-axis focused rerun、pre-window history、transition/carry-over 与 key-update audit 又把 `y` 的问题进一步收敛为带 turn-direction / geometry family 条件化、且能被追到具体 earlier updates 的 `k_lgy_y` 符号匹配失败。 | 单分量锚定实验中，overall 最优为 `position`：`final_dev_norm=0.082677 m`，优于 `attitude=0.084366 m` 和 `velocity=0.093515 m`；加入 `full PVA anchor` 后可降到 `0.006375 m`。`turn_vs_straight_xy_case_summary` 显示，turning 相对 straight 的 `x/y` `lever_share` 确实系统性抬升：baseline `x: 0.0568 vs 0.0242`、`y: 0.0815 vs 0.0093`，且 `pos_share_x/y` 同步下降；但 `lever_x_error_improve` 与 `lever_y_error_improve` 反而更差（baseline `x: -1.614 mm vs +0.086 mm`, `y: -2.534 mm vs -0.095 mm`）。sign-consistency 把这件事拆成两层：baseline turn windows `x` 轴 `dx_sign_match=0.44 < same_axis_sign=0.52 < straight dx_sign=0.64`，且 `temporary_good_then_bad_rate=0.4`，说明 `x` 的同轴 lever 通道被 weakly 打开了，但 final `dx_lgx` 仍常被 cross-axis contamination / reversal 拉坏；而 `y` 轴 `dx_sign_match=0.44`、`same_axis_sign=0.40`、`same_axis_dx_align=0.96`，说明 `y` 更像是同轴 lever 通道本身就没有稳定朝真值指向。focused A/B 又显示：`standard_reset_gamma` 与 `true_iekf` 对这些指标几乎无改观，而 diagnostic `position_anchor` 会把 turn `dx_sign_match_x/y` 拉到 `0.60/0.80`，并把 turn `total_window_improve_x/y` 翻正到 `+1.246/+6.215 mm`。随后 `gnss_pos_position_gain_scale` 扫描说明这是一个非单调 tradeoff：`0.5` 能把 `x/y` 同时明显拉近零，`0.25` 能把 `y` 翻正但会伤 `x`，`0.0` 几乎消除 `x` reversal 但会把 useful direct correction 一起压掉。`gnss_pos_update_mode=stage_nonpos_then_pos` 又进一步排除了“只是 position 先拿走 innovation”这一更弱解释：`staged` 的 turn `x/y total_window_improve=-1.397/-2.697 mm`，`staged+0.5=-0.571/-0.847 mm`，都没有优于 `joint+0.5`。term-source probe 则把轴间差异落到更具体层：baseline `x` turn `|term_x/y/z|=1.099/0.833/0.229 mm`，且在 `same-axis good but final bad` 的 updates 里，dominant bad cross term 几乎全是 `y` 项（rate=`0.60`）；`pos_gain=0.5` 把 `|term_y|` 压到 `0.748 mm`，并把这类 flips 从 `0.20` 降到 `0.12`。最新 `lgx_from_y` selective probe 更直接表明：在 `pos_gain=0.5` 基线上，仅抑制 `K(l_gx, meas_y)` 就能把 turn `x total_window_improve` 从 `-0.547 mm` 连续翻到 `+0.009/+0.564/+1.360/+3.433 mm`，同时 `same_axis_sign_match_x` 基本保持 `0.52`，而 `y` 仍停留在负改善区间。最新 `y same-axis` 诊断则进一步显示：turn windows 里 `desired_dx_y` 基本恒负，baseline / `pos_gain=0.5` 的 `innovation_y` 正号率却依然约 `0.760/0.760`；真正的分水岭是 `k_lgy_y` 是否按 `desired_dx_y * innovation_y` 的需要翻到负号，且这种失败高度集中在正向转弯窗口。baseline / `pos_gain=0.5` 的正向转弯 `same_axis_sign_match_y` 只有 `0.267/0.267`，但负向转弯为 `0.600/0.700`；`pos_gain=0.25` 在正向转弯里几乎不改变 `innovation_y` 正号率（仍约 `0.667`），却把 `k_lgy_y<0` 比例从 `0.467` 提到 `0.533`，从而把 `same_axis_sign_match_y` 拉到 `0.600` 并让 turn `total_window_improve_y` 翻正到 `+2.213 mm`。最新 `turn-direction proxy` 与 `turn-conditioned position probe` 进一步表明：运行时 `IMU omega_z` 已可无歧义替代离线 `POS yaw_rate`，而“强正向转弯时把 `pos_gain: 0.5 -> 0.25`”虽然能把 best combined case 的 turn `x/y total_window_improve` 拉到 `+2.590/+1.015 mm`，但正向转弯 `same_axis_sign_match_y` 仍停在 `0.267~0.333`；真正变化的是 wrong-sign same-axis 幅值，例如何正向转弯 `mean_k_lgy_y` 从 `0.004779/0.005113` 降到 `0.001966/0.002182`，`mean_same_axis_term_y` 从 `+0.523/+0.563 mm` 降到 `+0.081/+0.068 mm`。进一步的 numerator geometry 与 lever-axis focused rerun 又表明：`raw_k_lgy_y` 主要由 same-axis numerator 决定，不是 `S^{-1}` 的 `x/z` 混合；`lever` 自块内部也几乎完全由 `lgy` 自轴主导，`cov(lgy,lgy)` 在 focus windows 中始终为正 `0.004433~0.010421`，真正翻符号的是 `h_y_lever_y` 的几何项，`turn_window_1/5` 为负而 `turn_window_2/3/4` 为正。最新 `lgy history analysis` 又把这条结论推进到 entering-history 层：`positive_family_1_5` 的 pre/in-window `cov(lgy,py)>0 rate=0.88/0.90`、`h_y_lever_y>0 rate=0.12/0.10`，`positive_family_4` 则相反为 `0.10/0.00` 与 `0.95/1.00`。其中 `turn_window_5` 从 entry `h_y_lever_y=-0.214` 到窗口末 `-0.969` 都保持负几何，`turn_window_4` 则从 `+0.231` 到 `+0.972` 都保持正几何，而 `turn_window_1` 是窗口内翻号的 transition case：entry `h_y_lever_y=+0.061`，到窗口末已变成 `-0.978`。最新 transition/carry-over analysis 更把这条链闭到具体 GNSS_POS update：`turn_window_1` 的 stable path 为 `+-(16) -> ++(1) -> -+(4) -> +-(4)`，说明它先在 `529531 s` 发生 `h_y` 翻正，再在 `529532 s` 进入 `(-cov_py,+h)` family，最后于 `529536 s` 的第二个 in-window update 回到 `(+cov_py,-h)` family。最新 key-update audit 又把三次关键翻转细化为 `geometry_only @ 529531 s`、`covariance_only @ 529532 s` 与 `joint_covariance_and_geometry @ 529536 s`：前者由 `Δh_y_lever_y=+0.181968` 主导，后者由 `Δcov(lgy,p_y)=-0.001908` 主导，而入窗第二次翻回 `(+,-)` 则同时依赖 `Δcov(lgy,p_y)=+0.001445` 与 `Δh_y_lever_y=-0.214149`。另一方面，`turn_window_3` 的 carry-over 已定位到最早来自 `turn_window_4_entry @ 528121 s` 的 control divergence：该时刻只有 `Δpost_k=-0.074382`、state/cov 仍相同；紧接着 `528122 s` 就出现 `Δlever_before_y=+3.777 mm`、`Δcov(lgy,p_y)=+0.001008`、`Δcov(lgy,lgy)=+1.022e-4` 的 first state divergence。此后差分并不是靠单一 window 延续，而是靠窗口内外 mixed GNSS_POS chain 续命：`state_divergence -> window_5_entry` 区间里最大的再放大来自 non-window `528127/528128/528158 s`，而 `window_5_entry -> window_3_entry` 区间则由 `turn_window_5 @ 528609/528610 s` 与 non-window `528614/528615 s` 共同维持，到 `turn_window_3_last_pre` 仍保留 `Δlever_before_y=-2.360 mm`、`Δcov(lgy,p_y)=-3.61e-4`。这说明当前 unresolved 已不是“哪一个 turn window 坏了”，而是 `window_1` 的 sign-family transition 与 `window_4` 起始、并由后续非窗口 GNSS_POS 持续续命的 carry-over 如何在同一 full-history 轴上耦合。 | 下一步优先对 `window_1` 的 `529531/529532/529536 s` 回到 full `GNSS_POS` 历史里追更上游的 covariance 来源与 preceding update context；并沿 `528127/528128/528158/528609/528610/528614/528615 s` 这些已排名的 reinforcement updates 继续查明它们为什么能维持 `turn_window_3` 的分叉。 | open(refined_with_key_update_driver_classes_and_non_window_survival_updates_explicitly_ranked) |
| HYP-26 | 在当前 README-IMU 纯 `INS/GNSS` 基线上，`GNSS_POS` 有效量测噪声与 `GNSS lever` 过程噪声并不是杆臂异常的主因；`R` 主要控制首个 `GNSS_POS` 更新平台，`Q` 主要影响后续慢漂，而且 `ESKF/true_iekf` 的参数敏感性近乎一致。更关键的异常是：在高转弯激励窗口里，`GNSS_POS` 的 direct correction 仍主要流向 `position + attitude`，而其更底层的可计算机制是 `z` 轴前验已形成近乎完美的 `p_z ↔ l_gz` 负相关，使 `lever_z` 的 gain numerator 在更新前就被几乎抵消。 | `noise_coupling_sweep` 中 `eskf/true_iekf` 的 local best 都是 `r=1,q=4`，仅带来约 `16.6%/16.8%` 的 final-norm 改善；global best 都是 `r=2,q=10`，虽把 final norm 降到约 `0.134 m`，但 `early_plateau_z_abs_error` 却从 `0.341 m` 恶化到约 `0.405 m`。`first_update_z_r_rel_range≈0.281`，`first_update_z_q_rel_range≈8.9e-4`，说明首更几乎只随 `R` 变化。`turn_window_shared_correction_probe` 在 top-5 turning windows 中给出：baseline/`R×0.25`/`R×2` 的 mean `lever_share_z=0.1249/0.0915/0.1045`，但 `pos_share_z=0.7945/0.8380/0.8374`、`att_share_z=0.0806/0.0705/0.0581`，且 mean `lever_z_error_improve` 仅 `0.26~0.44 mm`；同一 probe 中 `dx_bg≈0`、`dx_sg` 仅 `2e-8` 量级。进一步用 `DIAG` 对账 turn windows 时，`std_lgz≈0.0768~0.1082 m`、`std_pz≈0.0773~0.1207 m`、`heading std≈1.56~1.76 deg`，说明当前并不是“lever covariance 已塌缩”或“完全没有 turning excitation”。最新 gain-cov 日志又给出：`prior_std_pz≈0.0895/0.1166/0.1484 m` 与 `prior_std_lgz≈0.0769/0.0922/0.1082 m` 同量级，但 `k_pos_z_norm≈0.972/0.840/0.728`、`k_lever_z_norm≈0.0364/0.0253/0.0169`；更关键的是 `corr_pz_lgz≈-0.855/-0.790/-0.730`，并且 `cov(p_z,l_gz)+var(l_gz)≈1.47e-5/1.43e-5/1.28e-5`、`lever_cancel_ratio_z≈0.00110/0.00118/0.00131`，这说明 `lever_z` gain 的直接分子项几乎被 `p_z-l_gz` 反相关完全抵消。 | 继续追这条 `p_z ↔ l_gz` 反相关是如何在早期 GNSS updates 中形成并维持的；优先比较 baseline、`standard_reset_gamma`、`true_iekf` 与轻度 `position` 约束干预，看 cancellation 是否被打破。 | open(very_strongly_supported_with_pz_lgz_gain_cancellation_identified) |
| HYP-27 | 在当前 `README-IMU` 纯 `INS/GNSS` 基线上，最有解释力的噪声路线不是再调 `R` 或大幅放松 `Q`，而是显式放宽 `GNSS lever` 的初始协方差/P0；当前 baseline 的 `std_gnss_lever=0.2 m` 过硬，增大到约 `1.0 m` 后即使保持基线 `Q` 也能显著改善 `x/y/z`，而 `Q` 只提供次级微调。 | `EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1` 已对 `P0_std={0.05,0.1,0.2,0.5,1.0}`、候选 `P0` 上的 `Q_scale={0.1,0.25,1,4,10}` 做 staged sweep，并先逐位复现 archived baseline：`eskf_p0std0p2_qlever1` 与 `truth_imu_archive_baseline` 的 `final_lever_dev_norm=0.230047 m`、`nav_rmse_3d=0.270254 m` 完全一致。stage1 里，单改 `P0_std: 0.2 -> 1.0 m` 就把 `final_lever_dev_norm` 从 `0.230047` 降到 `0.111468 m`（`-51.5%`），`lever_z_final_abs_error` 从 `0.226201` 降到 `0.105510 m`，`nav_rmse_3d` 从 `0.270254` 降到 `0.088308 m`；在同一 `P0=1.0 m` 上把 `Q` 再调到最优 `0.1x` 也只把 `final_lever_dev_norm` 从 `0.111468` 轻微降到 `0.108377 m`（额外 `2.77%`），说明 `Q` 是 secondary tuning knob。与 turn-window gain-tuned reference 比较时，这组 full-trajectory noise-only best 甚至在 global lever norm 上更优：`0.108377 m` vs `0.275873 m`，但该 reference 的优化目标仍是 turn-window `x/y` 路由，不应把这条比较误解成“gain route 被完全否定”。 | 把 `P0_std≈1.0 m, Q≈0.1~1.0x` 迁移到 full official `INS/GNSS/ODO/NHC` 流程做 transfer check；若收益主要集中在 `z(30)`，则进一步做 axis-specific `P0` 扫描，并在早期 `GNSS_POS` 更新上检查 `prior_std_lgz`、`corr(p_z,l_gz)` 与首更/前几更的 gain numerator 是否同步变化。 | open(strongly_supported_in_pure_ins_gnss_pending_full_pipeline_transfer) |

### 已解决 / 已否决假设摘要

| 主题 | 涉及 hyp_id | 结论 | 当前影响 | archive_ref |
|---|---|---|---|---|
| runtime PVA anchor 修复了旧 `GNSS lever` 错平台主叙事 | `HYP-23`、`HYP-22` | 旧“首更即错平台主要来自 GNSS_POS / ESKF 数学分支”的解释已被 runtime anchor fresh rerun 下调；主因是旧实验并未真正隔离 `PVA`。 | 后续 `GNSS lever` 讨论必须统一改引 `EXP-20260318-data2-gnss-lever-bias-attribution-r1`。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_04_pure_ins_gnss_pre_runtime_anchor` |
| `mounting_roll` 的伪正常 | `HYP-20` | `mounting_roll` 的 `normal` 更像半断开建模导致的 compatibility-only 行为。 | 不能再把它当 solver 健康证据。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_hypotheses_from_rows` |
| data4 full/outage ESKF 初始化口径统一 | `HYP-17` | `data4` full/outage ESKF 的大 gap 主要来自 `P0_diag` 口径不一致，而非 `GNSS schedule`。 | 后续 data4 对比可直接围绕算法差异展开。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_01_pre_official_outage_research_stack` |
## 会话日志（Session Log）

> 主文档当前保留最近一批仍在直接服务当前工作台的 session 正文；更早完整正文已转存到 `docs/project_history/walkthrough_archive_sessions_20260318.md`。

### session_id: 20260317-2000-data2-gnss-debug-audit-r1
- timestamp: 2026-03-17 20:00 (local)
- objective: 在纯 `INS/GNSS` 口径下排查 4 个问题：`GNSS_POS/GNSS_VEL` Jacobian 数值一致性、GNSS correction 级 `dx` 方向/量级、`bg_z` Markov 过程噪声量级，以及 `ESKF/hybrid/true_iekf` 分支选择与符号约定。
- scope:
  - 复用 `output/data2_gnss_debug_audit/` 工位，补做 fresh solver rebuild 与 fresh `_r2` runtime cases。
  - 核对 `jacobian_audit` 数值结果、`DiagnosticsEngine` 的 `first_update/gnss_update` 导出链，以及 `pipeline_fusion / measurement_models_uwb / eskf_engine` 中的分支与误差注入约定。
- changed_files:
  - `walkthrough.md`
- artifacts:
  - `output/data2_gnss_debug_audit/jacobian_300s_vel13/{summary.md,column_errors.csv}`
  - `output/data2_gnss_debug_audit/artifacts/data2_ins_gnss_*_r2/{gnss_update_*.csv,state_series_*.csv}`
- metrics:
  - `GNSS_POS att rel_max=0.007125`; `GNSS_VEL att rel_max=1.4e-05`; `GNSS_VEL bg/sg/gnss_lever rel_max=0`
  - fresh control `GNSS_POS/GNSS_VEL=299/299`
  - `release_bg_z_nominal_r2` 累计 `sum(dx_bg_z)=-39.391 deg/h`；`sigma10x` 对应 `-81.850 deg/h`
- observability_notes:
  - 约束/冻结块：仅保留 `GNSS_POS/GNSS_VEL`，显式关闭 `ODO/NHC`，并禁用 `mounting/odo_lever/odo_scale`
  - 行为结果：当前没有证据支持 `GNSS_POS att`、`GNSS_VEL att` 或 `GNSS_VEL bg/sg/gnss_lever` 存在整列反号；`bg_z` 的 correction 净方向是对的，更像量级不足而不是方向错误
- decision:
  - 先把排查重点从“H 符号是否反了”转到 `GNSS_POS` 首更残差来源与 `bg_z` 过程量级
- next_step:
  - 检查 `GNSS_POS` 首更平台残差来源，并继续做 `bg_z` 的 `sigma_bg / tau` 小范围联合扫描

### session_id: 20260317-2047-gnss-pos-first-residual-source-r1
- timestamp: 2026-03-17 20:47 (local)
- objective: 检查 `GNSS_POS` 控制组首更平台残差来源，把 `0.0595 m` residual 定量拆成 measurement、time alignment、传播误差与参考杆臂口径差四项。
- scope:
  - 扩展 `first_update` 导出链，补记录首更前 `state_before` 的 `p/v/q` 与修正后位置。
  - 重编译并重跑 `truth_anchor_all_non_pva` control case，再用独立分析脚本重建首更 GNSS 测量与预测天线位置。
- changed_files:
  - `src/app/diagnostics.cpp`
  - `scripts/analysis/analyze_gnss_pos_first_update_residual.py`
  - `walkthrough.md`
- artifacts:
  - `output/data2_gnss_lever_bias_attribution/artifacts/b/truth_anchor_all_non_pva/first_update_truth_anchor_all_non_pva.csv`
  - `output/data2_gnss_lever_bias_attribution/{first_update_residual_decomposition.csv,first_update_residual_context.json,first_update_residual_summary.md}`
- metrics:
  - control first `GNSS_POS` residual=`0.059517 m`; `dt_align=0.004371 s`
  - `measurement_minus_truth_at_gnss=0.031351 m`; `time_alignment_gnss_to_state=0.053148 m`; `filter_prediction_error_at_state_before_update=0.004714 m`
  - `README nominal vs RTK/POS implied lever=0.000214 m`
- observability_notes:
  - `7-column GNSS` 没有速度列，当前 `pipeline_fusion.cpp` 仅在 `has_vel` 时才做 `gnss_pos += vel_ecef * dt_align`；因此 `4.371 ms` 位移会直接进入首个 `GNSS_POS` residual
- decision:
  - 已确认 control residual 的主来源是 `GNSS_POS` 时间未对齐，而不是 README 杆臂口径差，也不是初始化后的大传播漂移
- next_step:
  - 把 `GNSS_POS` 位置时间对齐与 `enable_gnss_velocity` 解耦，或在 attribution 工位切到含速度的 13-column GNSS 再重跑

### session_id: 20260317-2100-gnss-pos-vel-decouple-r1
- timestamp: 2026-03-17 21:00 (local)
- objective: 把 `GNSS_POS` 的位置时间对齐从 `enable_gnss_velocity` 配置语义中显式解耦，确保 `GNSS_POS` 与 `GNSS_VEL` 成为独立系统。
- scope:
  - 复核 `pipeline_fusion.cpp`、`jacobian_audit_main.cpp` 与 `regression_checks_main.cpp` 中所有 `GNSS_POS/GNSS_VEL` 数据可用性判定。
  - 统一复用 `include/app/fusion.h` 中的 `HasGnssVelocityData()`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/pipeline_fusion.cpp`
  - `apps/jacobian_audit_main.cpp`
  - `apps/regression_checks_main.cpp`
  - `walkthrough.md`
- artifacts:
  - `build/Release/{eskf_fusion,jacobian_audit,regression_checks}.exe`
- metrics:
  - `regression_checks: PASS`
  - `GNSS_POS` 时间对齐现明确为：`dt_align > 1e-9 && HasGnssVelocityData(dataset)`
  - `GNSS_VEL` 更新现明确为：`HasGnssVelocityData(dataset) && options.enable_gnss_velocity`
- decision:
  - 代码层面的 GNSS 系统边界已明确：`enable_gnss_velocity` 只负责 `GNSS_VEL` 更新开关，不再承担 `GNSS_POS` 位置时间对齐的隐式语义
- next_step:
  - 用带速度列口径重跑 control + `release_gnss_lever_{y,z}`，检查首更错误平台与 `0.059517 m` control residual 是否同步收缩

### session_id: 20260317-2109-vel13-state-update-rerun-r1
- timestamp: 2026-03-17 21:09 (local)
- objective: 在解耦 `GNSS_POS` 时间对齐语义后，使用带速度列的 `13-col GNSS` 重新运行纯 `INS/GNSS` 实验，检查 `GNSS 杆臂` 与其它代表性状态的更新是否恢复正常。
- scope:
  - 将 `run_data2_ins_gnss_state_sanity.py` 与 `run_data2_gnss_lever_bias_attribution.py` 参数化为可切换 `--gnss-path`。
  - 兼容 `13-col GNSS` 的辅助分析，fresh 重跑 `GNSS lever stage-B` 与 `INS/GNSS-only state sanity`。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_state_sanity.py`
  - `scripts/analysis/run_data2_gnss_lever_bias_attribution.py`
  - `scripts/analysis/run_data2_state_sanity_matrix.py`
  - `scripts/analysis/analyze_gnss_pos_first_update_residual.py`
  - `walkthrough.md`
- artifacts:
  - `output/data2_gnss_lever_bias_attribution_vel13/{summary.md,manifest.json,plateau_metrics.csv,first_update_debug.csv}`
  - `output/data2_eskf_ins_gnss_state_sanity_vel13/{summary.md,manifest.json,case_metrics.csv,state_judgement.csv}`
- metrics:
  - control first-update residual `0.059517→0.028278 m`
  - `gnss_lever_x` plateau `0.085141→0.123712 m`; `gnss_lever_y/z` 10s plateau 基本不动
  - `INS/GNSS-only state sanity`: control `nav_rmse_3d_m=0.021594`, `nav_final_err_3d_m=0.019716`; overall 仍为 `13 abnormal / 1 borderline / 1 normal`
- decision:
  - 时间对齐缺失确实是 shared baseline residual 的重要来源；它足以改善 control 与 `gnss_lever_x`，但不足以解释 `gnss_lever_y/z` 的错误平台
- next_step:
  - 把 `gnss_lever_y/z` 的排查从“时间对齐”转到“横向/垂向参考口径 + 首更 `K*y` 构成 + 观测几何”

### session_id: 20260317-2355-pva-anchor-mismatch-audit-r1
- timestamp: 2026-03-17 23:55 (local)
- objective: 继续追查 `GNSS 杆臂首更即错平台` 的核心原因，验证它究竟来自测量模型/Jacobian 异常，还是实验口径里 `PVA` 没有真正被隔离。
- scope:
  - 用 `jacobian_audit` 对纯 `INS/GNSS` vel13 的控制组、`release_gnss_lever_y`、`release_gnss_lever_z` 做 fresh 数值 Jacobian 对比。
  - 构造两个最小 probe case：只把 `P0(pos/vel/att)` 压到近零，保留 `release_gnss_lever_y/z` 其它设置不变。
- changed_files:
  - `walkthrough.md`
- artifacts:
  - `output/data2_gnss_lever_jacobian_audit_vel13_r2/{summary.md,column_errors.csv,selected_samples.csv}`
  - `output/data2_gnss_lever_pva_lock_probe/eskf_release_gnss_lever_y_pva_locked/*`
  - `output/data2_gnss_lever_pva_lock_probe/eskf_release_gnss_lever_z_pva_locked/*`
- metrics:
  - `GNSS_POS Jacobian`: `gnss_lever_x/y/z` 列数值 Jacobian 误差仅约 `1e-5~4e-5`
  - 原始首更时 `release_gnss_lever_y/z` 同步伴随明显 `dp_ecef_norm=0.087233/0.374600 m`
  - `PVA-lock probe`: `release_gnss_lever_y` 首跳 `-0.220985 m`，`release_gnss_lever_z` 首跳 `-1.062293 m`
- decision:
  - 当前已经找到本轮审查里最核心的问题：实验所说的“`PVA` 真值锚定”在实现上并没有真正发生，`use_truth_pva=true` 只是初始化语义
- next_step:
  - 先在实验链中真正实现 `PVA truth anchor / freeze` 语义，再重跑 `GNSS lever attribution` 与 `INS/GNSS-only state sanity`

### session_id: 20260318-0040-runtime-pva-anchor-rerun-r1
- timestamp: 2026-03-18 00:40 (local)
- objective: 真正实现 `runtime PVA truth anchor` 语义，并用 fresh rerun 验证 `GNSS 杆臂首更即错平台` 是否因此消失，同时复核代表性 `bg/sg` 状态在纯 `INS/GNSS` 口径下的表现。
- scope:
  - 修改 `ESKF` 引擎与主循环，新增默认关闭的 `fusion.init.runtime_truth_anchor_pva`。
  - 将 `run_data2_gnss_lever_bias_attribution.py` 与 `run_data2_ins_gnss_state_sanity.py` 的实验配置生成逻辑切换到显式开启 `runtime_truth_anchor_pva=true`。
- changed_files:
  - `include/app/fusion.h`
  - `include/core/eskf.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `scripts/analysis/run_data2_ins_gnss_state_sanity.py`
  - `scripts/analysis/run_data2_state_sanity_matrix.py`
  - `walkthrough.md`
- artifacts:
  - `output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/{summary.md,manifest.json,plateau_metrics.csv,first_update_debug.csv}`
  - `output/data2_eskf_ins_gnss_state_sanity_vel13_r3_runtime_anchor/{summary.md,manifest.json,case_metrics.csv,state_judgement.csv}`
- metrics:
  - `eskf/true_iekf` plateau `x/y/z=0.149686/-0.220409/-1.139282 m`
  - `release_gnss_lever_y/z` 首更位置位移范数 `0.087233/0.374600→0/0 m`
  - `gnss_lever_y/z` 变为 `overall=normal`；`bg_y/bg_z/sg_y/sg_z` 仍 `behavior=abnormal`, `impact=normal`
- decision:
  - 当前已经找到并 fresh 验证了这轮最核心的问题：之前“非 PVA 状态真值锚定”实验的 `PVA` 实际并未在运行期锚定，导致 `GNSS_POS` 首更残差被 `PVA` 与杆臂共同分担，制造出 `gnss_lever_y/z` 的错误平台
- next_step:
  - 先把所有引用旧“PVA truth anchor”口径的 `GNSS lever` 结论下调为历史留痕，并在后续讨论中统一改引 `runtime_anchor` 结果

### session_id: 20260318-0059-walkthrough-compaction-r1
- timestamp: 2026-03-18 00:59 (local)
- objective: 压缩 `walkthrough.md`，把它重构成可直接支持 Start Checklist 的当前工作台文档，同时把旧 session 正文转存到新的 dated archive。
- scope:
  - 复核现有 `walkthrough.md` 的主结构、session 分布、开放假设与待办项。
  - 新建 `docs/project_history/walkthrough_archive_sessions_20260318.md`，收纳从主文档移出的完整 session 正文。
  - 重写主文档结构，只保留最近 `6` 个研究 session 的完整上下文，并把更早内容改成主题摘要 + archive_ref。
- changed_files:
  - `walkthrough.md`
  - `docs/project_history/walkthrough_archive_sessions_20260318.md`
- artifacts:
  - `walkthrough.md`
  - `docs/project_history/walkthrough_archive_sessions_20260318.md`
- metrics:
  - 主文档体量目标：从 `292518 bytes / 2508 lines` 压回到可直接支持 Start Checklist 的工作台规模
  - 新归档文档收纳旧 session：`33` 个完整 block，可按 `session_id` 检索
- decision:
  - `walkthrough.md` 已重新收敛为“当前工作台”文档；更早 session 的完整正文不再堆叠在主文档，而是转存到新的 dated archive
- next_step:
  - 后续若主文档再次超过约 `250 KB` 或持续积累 `20+` 新 session，继续沿“当前工作台 + dated archive”模式做增量归档

### session_id: 20260318-1037-data2-ins-gnss-lever-truth-init-free-run-r1
- timestamp: 2026-03-18 10:37 (local)
- objective: 在纯 `INS/GNSS` 口径下做“`GNSS lever` 真值初值但不做 runtime PVA anchor”的单-case 对照实验，检查杆臂是否仍会被系统性拉离真值。
- scope:
  - 新增独立实验驱动 `run_data2_ins_gnss_lever_truth_init_probe.py`，不修改 solver 数学。
  - 配置固定为 `use_truth_pva=true`、`runtime_truth_anchor_pva=false`、`enable_odo=false`、`enable_nhc=false`、`disable_mounting/odo_lever/odo_scale=true`、`enable_gnss_velocity=false`、`gnss_path=dataset/data2/rtk.txt`。
  - 对 `gnss_lever_x/y/z` 输出首更 `dx` 对账、`10 s` 平台值、最终偏差和三张对比图。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_lever_truth_init_probe.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_ins_gnss_lever_truth_init_probe/artifacts/cases/free_gnss_lever_truth_init/config_free_gnss_lever_truth_init.yaml`
- commands:
  - `python scripts/analysis/run_data2_ins_gnss_lever_truth_init_probe.py`
- artifacts:
  - `output/data2_ins_gnss_lever_truth_init_probe/{summary.md,manifest.json,case_metrics.csv,lever_deviation_metrics.csv,truth_reference.json}`
  - `output/data2_ins_gnss_lever_truth_init_probe/artifacts/cases/free_gnss_lever_truth_init/{SOL_free_gnss_lever_truth_init.txt,state_series_free_gnss_lever_truth_init.csv,first_update_free_gnss_lever_truth_init.csv,DIAG_free_gnss_lever_truth_init.txt,free_gnss_lever_truth_init.stdout.txt}`
  - `output/data2_ins_gnss_lever_truth_init_probe/plots/{gnss_lever_xyz_vs_truth.png,gnss_lever_xyz_first_10s.png,gnss_lever_xyz_deviation.png}`
- metrics:
  - navigation: `nav_rmse_3d_m=0.078974`, `nav_final_err_3d_m=0.106203`
  - `gnss_lever_x`: `initial=0.149981`, `early_plateau_10s=0.116039`, `final_dev=0.037972`, `max_dev=0.048278`, `label=persistent_departure`
  - `gnss_lever_y`: `initial=-0.219900`, `early_plateau_10s=-0.235581`, `final_dev=0.019291`, `max_dev=0.020568`, `label=near_truth`
  - `gnss_lever_z`: `initial=-1.149811`, `early_plateau_10s=-1.118217`, `final_dev=0.102767`, `max_dev=0.102853`, `label=persistent_departure`
  - first-update consistency: `x/y/z` 的 `lever_after - lever_before` 与 `dx_gnss_lever` 误差均约 `1e-16 m`
- artifact_mtime:
  - `manifest.json=2026-03-18T10:40:33`
  - `summary.md=2026-03-18T10:40:33`
  - `lever_deviation_metrics.csv=2026-03-18T10:40:30`
- config_hash_or_mtime:
  - `config_free_gnss_lever_truth_init.yaml=2026-03-18T10:37:46`
- dataset_time_window:
  - `528076.0 -> 530488.9 s`
- result_freshness_check:
  - fresh run completed with current `build/Release/eskf_fusion.exe` (`mtime=2026-03-18 00:29`)
  - all requested core artifacts and plots were regenerated under `output/data2_ins_gnss_lever_truth_init_probe/`
- observability_notes:
  - constrained/frozen blocks: `mounting(22:24)`、`odo_scale(21)`、`odo_lever(25:27)` 通过 ablation 禁用；`ODO/NHC` 更新链关闭
  - active windows: full GNSS，全程启用 `GNSS_POS`；`GNSS_VEL` 未启用，因为 `rtk.txt` 无速度列且 `enable_gnss_velocity=false`
  - behavior: `gnss_lever_x` 在首个 `GNSS_POS` 更新即出现主要偏差；`gnss_lever_z` 首更偏差较小但后续持续累计到约 `10 cm`
- decision:
  - 这次对照说明“只给 `GNSS lever` 真值初值”并不能替代 runtime PVA anchor 的隔离效果；在纯 `INS/GNSS` 下，`x/z` 仍会被 shared correction 拉离真值
  - `x` 的异常主要在首更形成，`z` 更像首更后累计漂移，因此下一步应并行检查 `GNSS_POS residual / PVA shared correction` 与后续累计更新链
- next_step:
  - 将 `runtime_anchor` 与 `truth-init free-run` 两条口径并排引用，继续解释为什么 `x/z` 在放开 runtime anchor 后重新偏离，而 `y` 仅表现为小幅近真值偏差

### session_id: 20260318-1110-data2-ins-gnss-pva-component-anchor-r1
- timestamp: 2026-03-18 11:10 (local)
- objective: 在 `GNSS lever truth init + no runtime anchor` 的纯 `INS/GNSS` 基线之上，分别只在 GNSS 更新后锚定 `attitude / position / velocity`，识别哪个 PVA 分量最能改善杆臂偏差。
- scope:
  - 扩展 `runtime_truth_anchor_pva` 为“分量选择 + GNSS-only trigger”语义，默认保持旧行为不变。
  - 新增 `run_data2_ins_gnss_pva_component_anchor_probe.py`，一次性运行 baseline + 实验 A/B/C 四个 case。
  - 对每个 case 统一导出 `state_series / first_update / DIAG / stdout`，并汇总 `final_dev_norm / final_dev_max / nav_rmse_3d`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/run_data2_ins_gnss_pva_component_anchor_probe.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_ins_gnss_pva_component_anchor_probe/artifacts/cases/*/config_*.yaml`
- commands:
  - `cmake --build build --config Release --target eskf_fusion regression_checks jacobian_audit`
  - `build/Release/regression_checks.exe`
  - `python scripts/analysis/run_data2_ins_gnss_pva_component_anchor_probe.py`
- artifacts:
  - `output/data2_ins_gnss_pva_component_anchor_probe/{summary.md,manifest.json,case_metrics.csv,lever_case_metrics.csv,comparison_summary.csv,truth_reference.json}`
  - `output/data2_ins_gnss_pva_component_anchor_probe/plots/{gnss_lever_x_compare.png,gnss_lever_y_compare.png,gnss_lever_z_compare.png,final_deviation_norm_bar.png}`
  - `output/data2_ins_gnss_pva_component_anchor_probe/artifacts/cases/{baseline_free_run,anchor_attitude_only,anchor_position_only,anchor_velocity_only}/*`
- metrics:
  - baseline: `final_dev_norm=0.111243 m`, `nav_rmse_3d=0.078974 m`
  - experiment A (`attitude`): `final_dev_norm=0.084366 m`, `final_dev_max=0.073120 m`, `nav_rmse_3d=0.065621 m`
  - experiment B (`position`): `final_dev_norm=0.082677 m`, `final_dev_max=0.069436 m`, `nav_rmse_3d=0.029843 m`
  - experiment C (`velocity`): `final_dev_norm=0.093515 m`, `final_dev_max=0.082566 m`, `nav_rmse_3d=0.060920 m`
  - axis winners:
    - `gnss_lever_x -> attitude` (`final_dev=0.016747 m`)
    - `gnss_lever_y -> velocity` (`final_dev=0.017157 m`, 但 baseline 已接近正常)
    - `gnss_lever_z -> position` (`final_dev=0.069436 m`)
- artifact_mtime:
  - `manifest.json=2026-03-18T11:16:30`
  - `summary.md=2026-03-18T11:16:30`
  - `comparison_summary.csv=2026-03-18T11:16:29`
- config_hash_or_mtime:
  - `anchor_attitude_only config=2026-03-18T11:12:53`
  - `anchor_position_only config=2026-03-18T11:13:56`
  - `anchor_velocity_only config=2026-03-18T11:15:12`
- dataset_time_window:
  - `528076.0 -> 530488.9 s`
- result_freshness_check:
  - solver rebuilt successfully after新增 component-anchor 开关
  - `regression_checks: PASS`
  - 四个 case 的 config / SOL / state_series / first_update / DIAG / stdout 都 fresh 落盘
- observability_notes:
  - constrained/frozen blocks: `mounting(22:24)`、`odo_scale(21)`、`odo_lever(25:27)` 通过 ablation 禁用；`ODO/NHC` 更新链关闭
  - active windows: full GNSS，仅 `GNSS_POS` 更新在线；`GNSS_VEL` 不参与
  - behavior:
    - 只拉回 `position` 时 overall 改善最大，说明 `z` 轴主导的杆臂偏差更像 position-driven shared correction
    - 只拉回 `attitude` 时 `x` 轴改善最大，说明 `x` 轴首更/早期偏差仍受 attitude-driven coupling 影响
    - 只拉回 `velocity` 的改善最弱，当前不支持把主要源头归到 velocity
- decision:
  - 目前不能把“杆臂偏差源头”粗暴归到整个 PVA；更准确的说法是：overall 由 `position` 主导，`x` 轴单独看又明显受 `attitude` 影响
  - 下一步 `GNSS_POS residual / shared correction` 分解应至少拆成 `position-driven z bias` 与 `attitude-driven x bias` 两条子链
- next_step:
  - 在 correction-level 诊断中分别追踪 `dx_pos / dx_att / lever residual` 如何传递到 `gnss_lever_x` 与 `gnss_lever_z`

### session_id: 20260318-1357-data2-ins-gnss-pva-component-anchor-fullcurve-r1
- timestamp: 2026-03-18 13:57 (local)
- objective: 在 A/B/C 单分量锚定对照图中加入 `full PVA anchor` 曲线，给出“所有 PVA 分量同时锚定”的上界对照。
- scope:
  - 扩展 `run_data2_ins_gnss_pva_component_anchor_probe.py`，新增 `anchor_pva_all` case。
  - 保持与 A/B/C 同一触发语义：仅在成功 `GNSS` 更新后同时锚定 `position + velocity + attitude`。
  - fresh 重跑整套 probe，更新图、CSV 与 summary。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_pva_component_anchor_probe.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts/analysis/run_data2_ins_gnss_pva_component_anchor_probe.py`
  - `python scripts/analysis/run_data2_ins_gnss_pva_component_anchor_probe.py`
- artifacts:
  - `output/data2_ins_gnss_pva_component_anchor_probe/{summary.md,comparison_summary.csv,lever_case_metrics.csv,manifest.json}`
  - `output/data2_ins_gnss_pva_component_anchor_probe/plots/{gnss_lever_x_compare.png,gnss_lever_y_compare.png,gnss_lever_z_compare.png,final_deviation_norm_bar.png}`
  - `output/data2_ins_gnss_pva_component_anchor_probe/artifacts/cases/anchor_pva_all/*`
- metrics:
  - `full PVA anchor`: `final_dev_norm=0.006375 m`, `final_dev_max=0.006131 m`, `nav_rmse_3d=0.001193 m`
  - vs baseline: `delta_final_dev_norm=-0.104869 m`
  - axis finals:
    - `x=0.006131 m`
    - `y=0.001746 m`
    - `z=0.000091 m`
- observability_notes:
  - `full PVA anchor` 代表多分量共同作用的最强上界，不应用来替代单分量归因
  - 与 A/B/C 使用相同的 `GNSS-only trigger`，因此曲线间具有直接可比性
- decision:
  - 图像中已加入 `full PVA anchor` 曲线；它清楚显示“PVA 全部分量同时锚定”几乎可以消除当前杆臂偏差
  - 但单分量归因结论不变：若只允许选一个 dominant source，仍更偏向 `position`，同时 `x` 轴保留明显的 `attitude` 敏感性
- next_step:
  - 后续汇报中将 `full PVA anchor` 明确作为 upper bound / sanity reference，而不是当作单分量归因结论本身

### session_id: 20260318-1418-full-pva-anchor-comparison-clarification-r1
- timestamp: 2026-03-18 14:18 (local)
- objective: 解释为什么这次 `data2_ins_gnss_pva_component_anchor_probe` 中的 `full PVA anchor` 曲线，与之前 `data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor` 的 runtime-anchor 结果不一致且误差更大。
- scope:
  - 对照两次实验的 `manifest.json`、`summary.md` 与核心 case config。
  - 逐项核对 `GNSS source`、`runtime anchor trigger`、非 `PVA` 状态是否真值锚定、以及初始化/过程噪声口径。
- changed_files:
  - `walkthrough.md`
- configs:
  - `output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/artifacts/b/truth_anchor_all_non_pva/config_truth_anchor_all_non_pva.yaml`
  - `output/data2_ins_gnss_pva_component_anchor_probe/artifacts/cases/anchor_pva_all/config_anchor_pva_all.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/{manifest.json,summary.md,plateau_metrics.csv}`
  - `Get-Content output/data2_ins_gnss_pva_component_anchor_probe/{manifest.json,summary.md,comparison_summary.csv,lever_case_metrics.csv}`
- artifacts:
  - `output/data2_gnss_lever_bias_attribution_vel13_r3_runtime_anchor/{manifest.json,summary.md,plateau_metrics.csv}`
  - `output/data2_ins_gnss_pva_component_anchor_probe/{manifest.json,summary.md,comparison_summary.csv,lever_case_metrics.csv}`
- metrics:
  - 旧 `runtime_anchor`：`gnss_path=output/data2_gnss_debug_audit/generated/data2_rtk_with_velocity_13col.txt`，`truth_anchor_all_non_pva` 的 `gnss_lever` 10s plateau=`0.149686/-0.220409/-1.139282 m`
  - 新 `full PVA anchor`：`gnss_path=dataset/data2/rtk.txt`，`final_dev_norm=0.006375 m`，分轴最终偏差 `x/y/z=0.006131/0.001746/0.000091 m`
  - 新旧关键口径差异：
    - `GNSS source`: 旧为 `13-col vel13`，新为 `7-col rtk.txt`
    - `runtime anchor trigger`: 旧为全运行期 `runtime_truth_anchor_pva=true`；新为 `runtime_truth_anchor_gnss_only=true`
    - 非 `PVA` 状态：旧为 `truth_anchor_all_non_pva`，`ba/bg/sg/sa` 使用参考真值 + 小噪声；新为这些状态零初值、baseline 大噪声、保持自由估计
- artifact_mtime:
  - `old manifest=2026-03-18T00:34:26`
  - `new manifest=2026-03-18T14:03:25`
- result_freshness_check:
  - 本次未生成新实验结果；仅复核两次已有 fresh artifact 的配置语义与指标
- observability_notes:
  - 旧实验属于“强隔离”口径：除目标杆臂外，其余非 `PVA` 状态都被真值锚定，因此杆臂几乎只承受 `GNSS_POS` 本身的残差
  - 新实验属于“更自由的归因 probe”：虽然 `PVA` 在 GNSS 后被拉回，但 `ba/bg/sg/sa/gnss_lever` 仍在 baseline 噪声下共同分担残差与后续漂移
  - 因此两者不能按同一 `full PVA anchor` 概念直接比较；新结果误差更大并不自动指向 solver regression
- decision:
  - 当前可以明确把“新 `full PVA anchor` 误差更大”解释为实验口径不同，而不是滤波器数学回退
  - 若要做 apples-to-apples 比较，应至少统一 `gnss_path=vel13`、统一 anchor 触发语义，并决定是否同时恢复 `truth_anchor_all_non_pva`
- next_step:
  - 增补一轮严格配平的对照：在 `component-anchor` 框架下分别切换 `vel13/rtk.txt`、`gnss_only/full_runtime_anchor`、`free_non_pva/truth_anchor_all_non_pva`，逐项量化这三类差异对杆臂误差的贡献

### session_id: 20260318-1518-data2-ins-gnss-lever-truth-imu-params-r1
- timestamp: 2026-03-18 15:18 (local)
- objective: 在纯 `INS/GNSS` 且不做 runtime `PVA` 锚定的前提下，将 `ba/bg/sg/sa` 作为真值已知建模，检查 `GNSS lever` 三轴从零初值释放后能否正常恢复。
- scope:
  - 新增独立实验驱动 `run_data2_ins_gnss_lever_truth_imu_params_probe.py`。
  - 固定口径为 `use_truth_pva=true`、`runtime_truth_anchor_pva=false`、`enable_odo=false`、`enable_nhc=false`、`disable_mounting/odo_lever/odo_scale=true`、`enable_gnss_velocity=false`、`gnss_path=dataset/data2/rtk.txt`。
  - `ba/bg/sg/sa` 采用 truth-anchor 语义：真值初值 + 小 `P0/Q`；`GNSS lever` 三轴从零初值开始，使用较大 `P0/Q` 在线估计。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/artifacts/cases/release_gnss_lever_truth_imu_params/config_release_gnss_lever_truth_imu_params.yaml`
- commands:
  - `python -m py_compile scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
  - `python scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
- artifacts:
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,truth_reference.json}`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/plots/{gnss_lever_xyz_vs_truth.png,gnss_lever_xyz_first_10s.png,gnss_lever_xyz_deviation.png}`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/artifacts/cases/release_gnss_lever_truth_imu_params/{SOL_release_gnss_lever_truth_imu_params.txt,state_series_release_gnss_lever_truth_imu_params.csv,first_update_release_gnss_lever_truth_imu_params.csv,DIAG_release_gnss_lever_truth_imu_params.txt,release_gnss_lever_truth_imu_params.stdout.txt}`
- metrics:
  - navigation: `nav_rmse_3d_m=0.269205`, `nav_final_err_3d_m=0.220326`
  - `gnss_lever_x`: `early_plateau_10s=0.087772 m`, `final=0.119233 m`, `final_abs_error=0.030748 m`, `recovery_ratio=0.205014`
  - `gnss_lever_y`: `early_plateau_10s=-0.178902 m`, `final=-0.192045 m`, `final_abs_error=0.027855 m`, `recovery_ratio=0.126672`
  - `gnss_lever_z`: `early_plateau_10s=-0.808758 m`, `final=-0.924834 m`, `final_abs_error=0.224977 m`, `recovery_ratio=0.195664`
  - first-update consistency: `x/y/z` 的 `lever_after - lever_before` 与 `dx_gnss_lever` 误差均为 `0`
- artifact_mtime:
  - `manifest.json=2026-03-18T15:17:27`
  - `summary.md=2026-03-18T15:17:27`
  - `lever_metrics.csv=2026-03-18T15:17:24`
- config_hash_or_mtime:
  - `config_release_gnss_lever_truth_imu_params.yaml=2026-03-18T15:15:49`
- dataset_time_window:
  - `528076.0 -> 530488.9 s`
- result_freshness_check:
  - 使用当前 `build/Release/eskf_fusion.exe` fresh 重跑完成
  - 所有核心产物与三张图均已重新生成到 `output/data2_ins_gnss_lever_truth_imu_params_probe/`
- observability_notes:
  - constrained/frozen blocks: `ba(9:11)`、`bg(12:14)`、`sg(15:17)`、`sa(18:20)` 以真值初值 + 小 `P0/Q` 建模；`mounting(22:24)`、`odo_scale(21)`、`odo_lever(25:27)` 通过 ablation 禁用
  - active windows: full GNSS，全程只有 `GNSS_POS` 在线；`GNSS_VEL` 未启用，因为 `rtk.txt` 无速度列
  - behavior: `GNSS lever` 三轴首更都朝真值方向跃迁，但 `z` 轴在无 runtime `PVA` 锚定下仍保留明显残差，说明仅修正零偏/比例因子不足以完全消除杆臂偏差
- decision:
  - 这轮实验支持一个更强的收敛结论：`ba/bg/sg/sa` 真值建模会明显帮助 `GNSS lever` 恢复，但并不能取代 runtime `PVA` 锚定；剩余主要问题仍在 free-PVA correction chain，尤其是 `z` 轴
  - 因而“杆臂异常主要是零偏/比例因子误差注入”的解释只能算部分成立，不能升级为主因
- next_step:
  - 做一轮严格配平对照：保持 `GNSS lever` 同样三轴零初值，只切换 `ba/bg/sg/sa truth-anchor` 与 `ba/bg/sg/sa free`，量化真值 IMU 参数建模到底改善了多少

### session_id: 20260318-1532-data2-ins-gnss-lever-truth-imu-readme-plots-r1
- timestamp: 2026-03-18 15:32 (local)
- objective: 修正上一轮 `truth_imu_params_probe` 对零偏/比例因子的处理口径，使其改用 `data2/README.md` 的 IMU 参数建模，并补导出 `ba/bg/sg/sa` 图像。
- scope:
  - 修改 `run_data2_ins_gnss_lever_truth_imu_params_probe.py`，从 `dataset/data2/README.md` 解析 IMU 参数。
  - 将 `ba/bg/sg/sa` 的过程模型参数切换为 README `POS320` 行的 `sigma_bg/sigma_ba/sigma_sg/sigma_sa/corr_time`。
  - 新增 `imu_truth_model_reference.csv`，并导出 `ba/bg/sg/sa` 四组 “估计曲线 vs Markov modeled truth mean ±3σ” 图。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
  - `walkthrough.md`
- configs:
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/artifacts/cases/release_gnss_lever_truth_imu_params/config_release_gnss_lever_truth_imu_params.yaml`
- commands:
  - `python -m py_compile scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
  - `python scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
- artifacts:
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,imu_truth_model_reference.csv}`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/plots/{gnss_lever_xyz_vs_truth.png,gnss_lever_xyz_first_10s.png,gnss_lever_xyz_deviation.png,ba_modeled_truth_compare.png,bg_modeled_truth_compare.png,sg_modeled_truth_compare.png,sa_modeled_truth_compare.png}`
- metrics:
  - README-driven IMU model: `sigma_bg=0.5 deg/h`, `sigma_ba=25 mGal`, `sigma_sg=300 ppm`, `sigma_sa=300 ppm`, `corr_time=4 h`
  - navigation: `nav_rmse_3d_m=0.269729`, `nav_final_err_3d_m=0.222314`
  - `gnss_lever_x/y/z` final abs error=`0.030533/0.027787/0.225026 m`
- artifact_mtime:
  - `manifest.json=2026-03-18T15:30:21`
  - `summary.md=2026-03-18T15:30:21`
  - `imu_truth_model_reference.csv=2026-03-18T15:29:26`
- result_freshness_check:
  - 当前 README 参数口径与新增图像已 fresh 重跑完成，所有图均写入 `output/data2_ins_gnss_lever_truth_imu_params_probe/plots/`
- observability_notes:
  - `ba/bg/sg/sa` 不再只用 docs 里的 instability 当近似常值锚定，而是改成“初值来自已有真值目录，过程参数来自 README 的 POS320 Markov 模型”
  - 结果上，README 参数并没有实质扭转 `GNSS lever_z` 的剩余大偏差，说明问题不主要来自 bias/scale 真值模型取值过粗
- decision:
  - 现在这轮 `truth_imu_params_probe` 的解释口径更自洽：`ba/bg/sg/sa` 图与参考模型已经补齐，可以直接用于后续和杆臂一起审阅
  - 在 README 参数驱动下结论仍不变：仅靠 bias/scale truth modeling 不能解释或消除 `GNSS lever_z` 异常
- next_step:
  - 保持 `GNSS lever` 三轴零初值不变，只切换 `ba/bg/sg/sa truth-modeled` 与 `ba/bg/sg/sa free`，做严格净贡献对照

### session_id: 20260318-1540-truth-imu-params-est-clarification-r1
- timestamp: 2026-03-18 15:40 (local)
- objective: 澄清 `truth_imu_params_probe` 图中的 `est` 语义，并核对当前 `ba/bg/sg/sa` 是否真的表现出与 README Markov 真值一致的变化特征。
- scope:
  - 直接统计 `state_series_release_gnss_lever_truth_imu_params.csv` 中 `ba/bg/sg/sa` 的起终值、range 和 std。
  - 对照 `imu_truth_model_reference.csv` 的 README-driven Markov 参考轨迹，判断当前曲线能否被解释为“正确恢复真值动态”。
- changed_files:
  - `walkthrough.md`
- commands:
  - `python - <<...>>` 读取 `state_series_release_gnss_lever_truth_imu_params.csv` 并统计 `ba/bg/sg/sa` range/std
  - `Get-Content output/data2_ins_gnss_lever_truth_imu_params_probe/{summary.md,imu_truth_model_reference.csv}`
- artifacts:
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/artifacts/cases/release_gnss_lever_truth_imu_params/state_series_release_gnss_lever_truth_imu_params.csv`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/imu_truth_model_reference.csv`
- metrics:
  - `bg_x/y/z` range=`0.008450/0.026342/0.009413 deg/h`
  - `ba_x/y/z` range=`0.079663/0.178386/1.356915 mGal`
  - `sg_x/y/z` range=`0.998353/2.678807/12.394169 ppm`
  - `sa_x/y/z` range=`0.713070/1.490876/190.790235 ppm`
- observability_notes:
  - 当前 `est` 的确是程序里的状态估计量，而不是外部构造的“真值轨迹”
  - 在当前 `truth_imu_params_probe` 语义下，`ba/bg/sg/sa` 被当作“真值已知、尽量不扰动”的 nuisance states，因此它们多数轴几乎不变是设计结果，不是“估计恢复了 README 真值动态”
- decision:
  - 用户指出的问题成立：当前图里 `est` 若被解读为“应该跟随真值动态”，那就是语义不对；这轮实验不能把 `ba/bg/sg/sa` 的曲线当成 truth-tracking 证据
  - 后续应把这轮实验只保留为“bias/scale 被压住后，GNSS lever 还会不会偏”的隔离证据
- next_step:
  - 若要真正检查 `ba/bg/sg/sa` 是否“按真值动态变化”，需要单独做 release 实验，而不是在 truth-modeled/fixed 语义下观察它们

### session_id: 20260318-1548-imu-markov-nominal-gap-audit-r1
- timestamp: 2026-03-18 15:48 (local)
- objective: 判断 `truth_imu_params_probe` 中 `ba/bg/sg/sa` 几乎不变，究竟主要是过程噪声太小，还是过程模型/nominal propagation 与 `modeled truth mean` 本身不一致。
- scope:
  - 核对 `ins_mech.cpp` 中 `ba/bg/sg/sa` 的 `F=-I/T` Markov 建模与 `eskf_engine.cpp` 中 nominal state 的实际传播逻辑。
  - 对比 README `tau=4h` 下的理论均值衰减与当前 `state_series` 终值。
- changed_files:
  - `walkthrough.md`
- commands:
  - `Select-String src/core/ins_mech.cpp -Pattern 'markov_corr_time|F.block<3, 3>(StateIdx::kBa'`
  - `Select-String src/core/eskf_engine.cpp -Pattern 'state_.ba|state_.bg|state_.sg|state_.sa|Predict'`
  - `python - <<...>>` 计算 `exp(-dt/tau)` 的理论末值
- metrics:
  - 代码证据：
    - `ins_mech.cpp:307-315` 为 `ba/bg/sg/sa` 设置了 `F=-I/T`
    - `eskf_engine.cpp:88-103` 的 `Predict()` 只更新 `state_=res.state` 与 `P=Phi P Phi^T + Qd`，未见对 nominal `ba/bg/sg/sa` 做 `-x/T` 衰减
    - `eskf_engine.cpp:327-331` 中 `state_.ba/bg/sg/sa += d*` 只发生在 correction 注入
  - README `POS320`, `tau=4h`，整段 `data2` 时间窗 `2412.886 s` 的理论衰减因子约 `0.845725`
  - 理论末值示例：
    - `bg_x: 100 -> 84.572 deg/h`
    - `bg_y: -300 -> -253.717 deg/h`
    - `sg_y: 6000 -> 5074.348 ppm`
  - 当前 `est` 末值却仍接近初值：
    - `bg_x≈100.008`
    - `bg_y≈-300.026`
    - `sg_y≈5997.321`
- observability_notes:
  - 这说明当前 `est` 不随 `modeled truth mean` 明显衰减，主要不是“噪声太小抖不动”，而是 nominal state 本身没有按同一 Markov 均值模型传播
  - 小过程噪声会进一步压低 correction 对这些状态的调整，但即使把噪声调大，也不会自动产生 `x0 e^{-t/T}` 形式的均值衰减
- decision:
  - 当前 `truth_imu_params_probe` 中 `modeled truth mean` 与程序 `est` 的比较，暴露的是一个更基础的建模语义差异：协方差侧用了一阶 Markov，nominal state 却没有同步做均值传播
  - 因此这里优先不是“过程噪声太小”结论，而是“当前 modeled truth mean 与程序 nominal state 语义不一致”；噪声大小是次级问题
- next_step:
  - 需要先决定 `ba/bg/sg/sa` 在程序中到底要采用哪种语义：
    1. 真正的零均值一阶 Markov：nominal 也按 `-x/T` 传播
    2. 实用型近常值标定参数：保留当前 nominal 常值，只把 `modeled truth mean` 改成“常值参考 + 包络”而不是指数衰减

### session_id: 20260318-1615-markov-nominal-propagation-fix-r1
- timestamp: 2026-03-18 16:15 (local)
- objective: 把 `ba/bg/sg/sa` 的 nominal propagation 改成与 README 一阶 Markov 模型一致，并原样重跑 `truth_imu_params_probe`。
- scope:
  - 在 `EskfEngine::Predict()` 中，为 `ba/bg/sg/sa` 新增 `exp(-dt/T)` 的 nominal 衰减。
  - 保持 `truth_imu_params_probe` 其余实验口径不变，fresh 重跑并检查 `est` 是否开始跟随 `modeled truth mean`。
- changed_files:
  - `src/core/eskf_engine.cpp`
  - `walkthrough.md`
- commands:
  - `cmake --build build --config Release --target eskf_fusion regression_checks`
  - `build/Release/regression_checks.exe`
  - `python scripts/analysis/run_data2_ins_gnss_lever_truth_imu_params_probe.py`
- artifacts:
  - `build/Release/{eskf_fusion.exe,regression_checks.exe}`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,imu_truth_model_reference.csv}`
  - `output/data2_ins_gnss_lever_truth_imu_params_probe/artifacts/cases/release_gnss_lever_truth_imu_params/state_series_release_gnss_lever_truth_imu_params.csv`
- metrics:
  - `regression_checks: PASS`
  - rerun navigation: `nav_rmse_3d_m=0.270254`, `nav_final_err_3d_m=0.223718`
  - rerun `gnss_lever_x/y/z` final abs error=`0.031524/0.027593/0.226201 m`
  - README mean tracking examples:
    - `bg_x` end=`84.579601 deg/h`, `ref_end=84.572485`, `mae_ref=0.002648`
    - `bg_y` end=`-253.739670 deg/h`, `ref_end=-253.717454`, `mae_ref=0.008098`
    - `ba_x` end=`-1268.517816 mGal`, `ref_end=-1268.587270`, `mae_ref=0.030992`
    - `sg_y` end=`5072.307576 ppm`, `ref_end=5074.349080`, `mae_ref=0.765465`
  - remaining mismatch example:
    - `sa_z` end=`95.087691 ppm`, `ref_end=253.717454`, `mae_ref=86.093964`
- result_freshness_check:
  - solver rebuilt after nominal Markov change
  - `truth_imu_params_probe` fresh rerun completed with updated core dynamics
- observability_notes:
  - 修复后，`ba/bg/sg` 多数轴不再“只和初值重合”，而是开始按 `modeled truth mean` 明显衰减
  - `sa_z`、`sg_z` 等轴仍与参考均值有偏差，说明剩余差异更多来自 correction/observability，而不是 nominal propagation 缺项
- decision:
  - 当前已经确认：你前面指出的核心问题成立，并且已被修复；之前 `est` 与 `modeled truth mean` 不符，主因确实是 nominal Markov propagation 缺失
  - 修复后，`GNSS lever_z` 仍保持大偏差，因此“杆臂异常”的主因不能归到这个 nominal 模型缺项上，后续仍应回到 GNSS/PVA shared correction 链
- next_step:
  - 在 nominal Markov 修复后的新代码上，继续做“`ba/bg/sg/sa truth-modeled` vs `free`”的严格净贡献对照

### session_id: 20260318-1737-gnss-lever-noise-coupling-sweep-r1
- timestamp: 2026-03-18 17:37 (local)
- objective: 在 README-IMU 纯 `INS/GNSS` 基线上，系统扫描 `GNSS_POS` 有效量测噪声与 `GNSS lever` 过程噪声，检查当前杆臂异常能否主要用参数耦合解释。
- scope:
  - 新增实验驱动 `scripts/analysis/run_data2_ins_gnss_lever_noise_coupling_sweep.py`。
  - 生成派生 `rtk` 文件，通过缩放每历元 `sigma_n/e/d` 实现有效 `GNSS_POS` 噪声扫描，而不是改 `sigma_gnss_pos` fallback。
  - 固定 `ba/bg/sg/sa` 为 README Markov truth-modeled 语义，固定 `GNSS lever` 零初值与 `P0_diag[28:31]=[0.04,0.04,0.04]`，只扫 `sigma_gnss_lever_arm`。
  - 同时运行 `ESKF` 与 `true_iekf` 两个模型，共 `2 * 5 * 5 = 50` 个核心 case。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_lever_noise_coupling_sweep.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/true_iekf_rstd2_qlever10/config_true_iekf_rstd2_qlever10.yaml`
- commands:
  - `python -m py_compile scripts/analysis/run_data2_ins_gnss_lever_noise_coupling_sweep.py`
  - `python scripts/analysis/run_data2_ins_gnss_lever_noise_coupling_sweep.py --output-dir output/data2_ins_gnss_lever_noise_coupling_sweep_smoke --models eskf,true_iekf --r-scales 0.5,1,2 --q-scales 0.25,1`
  - `python scripts/analysis/run_data2_ins_gnss_lever_noise_coupling_sweep.py`
- artifacts:
  - `output/data2_ins_gnss_lever_noise_coupling_sweep/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,first_update_metrics.csv,sweep_judgement.csv,gnss_variant_metrics.csv}`
  - `output/data2_ins_gnss_lever_noise_coupling_sweep/plots/{eskf_final_lever_dev_norm_heatmap.png,true_iekf_final_lever_dev_norm_heatmap.png,eskf_rq_slice_curves.png,true_iekf_rq_slice_curves.png,eskf_baseline_local_global_lever_compare.png,true_iekf_baseline_local_global_lever_compare.png}`
  - `output/data2_ins_gnss_lever_noise_coupling_sweep/gnss_variants/{rtk_sigma_scale_0p25.txt,rtk_sigma_scale_0p5.txt,rtk_sigma_scale_1.txt,rtk_sigma_scale_2.txt,rtk_sigma_scale_4.txt}`
- metrics:
  - smoke checks:
    - `sigma_n/e/d` median at `r=0.5` = `0.0065/0.0060/0.0165 m`
    - `sigma_n/e/d` median at `r=2.0` = `0.0260/0.0240/0.0660 m`
    - baseline reproduction delta (`eskf r=1 q=1` vs `truth_imu_params_probe`) for `nav_rmse_3d / final norm / x,y,z final abs error` all about `1e-9`
  - baseline:
    - `eskf_rstd1_qlever1`: `final_lever_dev_norm=0.230047 m`, `lever_z_final_abs_error=0.226201 m`, `early_plateau_z_abs_error=0.341050 m`, `nav_rmse_3d=0.270254 m`
    - `true_iekf_rstd1_qlever1`: `final_lever_dev_norm=0.229230 m`, `lever_z_final_abs_error=0.225193 m`, `early_plateau_z_abs_error=0.340758 m`, `nav_rmse_3d=0.269887 m`
  - local best:
    - `eskf_rstd1_qlever4`: `final_improve=16.595%`, `z_improve=16.144%`, `nav_rmse_delta=-10.896%`
    - `true_iekf_rstd1_qlever4`: `final_improve=16.818%`, `z_improve=16.313%`, `nav_rmse_delta=-10.973%`
  - global best:
    - `eskf_rstd2_qlever10`: `final_lever_dev_norm=0.134800 m`, `final_improve=41.404%`, `z_improve=41.714%`, `early_plateau_z_improve=-18.718%`
    - `true_iekf_rstd2_qlever10`: `final_lever_dev_norm=0.133858 m`, `final_improve=41.605%`, `z_improve=41.836%`, `early_plateau_z_improve=-18.778%`
  - first-update sensitivity:
    - `eskf`: `first_update_z_r_rel_range=0.281273`, `first_update_z_q_rel_range=0.000887`
    - `true_iekf`: `first_update_z_r_rel_range=0.281273`, `first_update_z_q_rel_range=0.000887`
- artifact_mtime:
  - `manifest.json=2026-03-18T17:35:34`
  - `summary.md=2026-03-18T17:35:34`
  - `sweep_judgement.csv=2026-03-18T17:35:22`
- result_freshness_check:
  - smoke run and 50-case full run both使用当前 `build/Release/eskf_fusion.exe` fresh 生成
  - `50/50` 个核心 case 均有 config、SOL、state_series、first_update、DIAG、stdout
- observability_notes:
  - constrained/frozen blocks: `mounting(22:24)`、`odo_scale(21)`、`odo_lever(25:27)` 继续禁用；`ba/bg/sg/sa(9:20)` 采用 README Markov truth-modeled 语义；`GNSS lever(28:30)` 零初值释放，`P0` 固定、`Q` 扫描
  - active windows: full GNSS，全程只有 `GNSS_POS` 在线；`GNSS_VEL` 未启用，因为 `rtk.txt` 无速度列
  - behavior: `R` 变化会明显改变 `z` 轴首更平台，`Q` 对首更几乎没有影响；但 `Q` 可以影响后续慢漂，所以极端 `q=10` 能把最终误差拉低，却并没有修正首更平台
  - model comparison: `ESKF` 与 `true_iekf` 的最优点、改善幅度和首更敏感性几乎一致，说明当前问题不是某一个分支独有的简单参数脆弱性
- decision:
  - 当前已经有比较强的证据支持：`GNSS_POS` 有效量测噪声和 `GNSS lever` 过程噪声不是这轮杆臂异常的主因；简单调参无法解释“首更平台就错”这个现象
  - 更准确的因果链是：`R` 决定首更平台跳到哪里，`Q` 主要决定后续能不能慢慢漂回来一些；因此应把下一步重心切换到 `GNSS_POS residual / shared correction` 的结构性归因
- next_step:
  - 在 baseline、`r=0.25,q=1` 与 `r=2,q=1` 三组 case 上，继续分解 `GNSS_POS` 首更的预测天线位置、残差和共享修正去向，确认为什么“方向正确但平台值错误”

### session_id: 20260318-1800-turn-window-sanity-r1
- timestamp: 2026-03-18 18:00 (local)
- objective: 按用户最新判断重新解释 `GNSS lever` 异常，验证在高转弯激励窗口里当前估计是否真的出现了解耦后的明显改善。
- scope:
  - 不再把“首更平台没到真值”直接当作主嫌疑，而是复用现有 `noise_coupling_sweep` 产物，检查 top turning windows 中 `GNSS lever` 误差的变化。
  - 从 `dataset/data2_converted/POS_converted.txt` 的航向变化中提取 sustained turning 窗口，并对比 `eskf baseline/local_best/global_best` 三组 case。
- changed_files:
  - `walkthrough.md`
- commands:
  - `python` 读取 `dataset/data2_converted/POS_converted.txt`，计算 `5 s` rolling `|yaw_rate|` 并筛选 top-5 turning windows
  - `python` 对比 `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/{eskf_rstd1_qlever1,eskf_rstd1_qlever4,eskf_rstd2_qlever10}/state_series_*.csv` 中 `gnss_lever_x/z` 在各 turning windows 的前后误差变化
- artifacts:
  - 复用 `output/data2_ins_gnss_lever_noise_coupling_sweep/{case_metrics.csv,first_update_metrics.csv}`
  - 复用 `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/state_series_eskf_rstd1_qlever1.csv`
  - 复用 `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever4/state_series_eskf_rstd1_qlever4.csv`
  - 复用 `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd2_qlever10/state_series_eskf_rstd2_qlever10.csv`
- metrics:
  - top-5 turning windows 约为：`t=529536.54/530274.52/528723.77/528123.10/528610.02 s`，对应 `5 s` rolling turn score 约 `15.15~18.54 deg/s`
  - `lever_z` 在这些窗口内的平均误差改善仅为：
    - baseline: `0.000939 m`
    - local_best (`r=1,q=4`): `0.001145 m`
    - global_best (`r=2,q=10`): `0.002145 m`
  - `lever_x` 在同样窗口中有时会改善、有时会变差，平均改善分别约：
    - baseline: `0.000836 m`
    - local_best: `0.003660 m`
    - global_best: `0.001187 m`
- observability_notes:
  - active windows: 仅分析 `POS` 中速度大于 `3 m/s` 的高转弯窗口，不引入 `ODO/NHC`
  - behavior: 即便在转弯激励较强的窗口里，`lever_z` 误差也几乎保持不变；这比“首更平台没到真值”更像当前真正的可观性异常
- decision:
  - 用户的修正判断成立：`GNSS lever` 与 `POS` 的首更耦合会导致平台值不在真值上，这本身不应被直接升级为 bug 证据
  - 当前更值得追的核心问题是：为什么高转弯窗口并没有给 `lever_z` 带来明显改善
- next_step:
  - 把后续 `GNSS lever` 归因实验聚焦到高转弯窗口内的 `GNSS_POS residual / prediction / dx` 分解，优先解释“为什么转弯激励没有真正解耦出 `lever_z`”

### session_id: 20260318-1813-turn-window-shared-correction-r1
- timestamp: 2026-03-18 18:13 (local)
- objective: 完成 `Next Actions #1`，在高转弯窗口内对 `baseline(r=1,q=1)`、`R×0.25`、`R×2` 三组 `ESKF q=1` case 做 `GNSS_POS residual / direct measurement-space correction` 分解，确认 turning 为什么没有把 `GNSS lever` 真正解耦出来。
- scope:
  - 扩展 `gnss_update_debug_output_path`，导出每次 `GNSS_POS/GNSS_VEL` 更新的 `y`、`state_before(p/q/lever)`、`dx_pos/att/ba/bg/sg/gnss_lever`。
  - 新增独立驱动 `scripts/analysis/run_data2_turn_window_shared_correction_probe.py`，复用 `noise_coupling_sweep` 的三组 source configs，仅 fresh rerun `baseline/r0.25/r2` 三个 case。
  - 在 top-5 sustained turning windows 内统计 `position / attitude / gnss_lever` 的 direct measurement-space share 与窗口前后 `lever_x/z` 误差改善。
- changed_files:
  - `src/app/diagnostics.cpp`
  - `scripts/analysis/run_data2_turn_window_shared_correction_probe.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd0p25_qlever1/config_eskf_rstd0p25_qlever1.yaml`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd2_qlever1/config_eskf_rstd2_qlever1.yaml`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
- artifacts:
  - `output/data2_turn_window_shared_correction_probe/{manifest.json,summary.md,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv}`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd1_qlever1/gnss_updates_eskf_rstd1_qlever1.csv`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd0p25_qlever1/gnss_updates_eskf_rstd0p25_qlever1.csv`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd2_qlever1/gnss_updates_eskf_rstd2_qlever1.csv`
- metrics:
  - top-5 turning windows 仍为：`t=529536.543/530274.515/528723.771/528123.104/528610.021 s`，对应 `5 s` rolling turn score 约 `18.538/16.610/16.300/15.275/15.148 deg/s`
  - case-level direct-share / window improvement:
    - `R×0.25`: `mean_pos_share_z=0.837971`、`mean_att_share_z=0.070548`、`mean_lever_share_z=0.091480`；`mean_pos_share_x=0.822571`、`mean_att_share_x=0.088509`、`mean_lever_share_x=0.088920`；`mean_lever_z_error_improve=0.000443 m`、`mean_lever_x_error_improve=-0.002315 m`
    - `baseline R×1`: `mean_pos_share_z=0.794494`、`mean_att_share_z=0.080560`、`mean_lever_share_z=0.124946`；`mean_pos_share_x=0.869280`、`mean_att_share_x=0.073885`、`mean_lever_share_x=0.056835`；`mean_lever_z_error_improve=0.000263 m`、`mean_lever_x_error_improve=-0.001614 m`
    - `R×2`: `mean_pos_share_z=0.837354`、`mean_att_share_z=0.058149`、`mean_lever_share_z=0.104497`；`mean_pos_share_x=0.865500`、`mean_att_share_x=0.074199`、`mean_lever_share_x=0.060301`；`mean_lever_z_error_improve=0.000329 m`、`mean_lever_x_error_improve=-0.001908 m`
  - strongest baseline window `turn_window_1`:
    - `mean_abs_y_z=0.007019 m`
    - `mean_pos_share_z=0.625212`
    - `mean_att_share_z=0.135220`
    - `mean_lever_share_z=0.239569`
    - `lever_z_error_improve=0.000065 m`
  - coupling side evidence in current truth-modeled setup:
    - `mean_dx_bg_norm=0.0` for all three cases
    - `mean_dx_sg_norm≈2.08e-08 ~ 2.94e-08`
    - `mean_dx_lever_norm≈0.00230 ~ 0.00241`
- artifact_mtime:
  - `manifest.json=2026-03-18T18:13:22`
  - `summary.md=2026-03-18T18:13:22`
  - `case_update_summary.csv=2026-03-18T18:13:22`
  - case rerun mtimes:
    - `eskf_rstd1_qlever1/gnss_updates=2026-03-18T18:12:12`
    - `eskf_rstd0p25_qlever1/gnss_updates=2026-03-18T18:12:40`
    - `eskf_rstd2_qlever1/gnss_updates=2026-03-18T18:13:09`
- result_freshness_check:
  - `eskf_fusion.exe` 在扩展 `gnss_update` 导出后 fresh rebuild
  - 三个 case 均用新的可执行文件 fresh rerun，且每个 case 都生成了新的 `SOL/state_series/first_update/gnss_updates/DIAG/stdout`
  - `manifest.json` 记录了 source config path、fresh artifact mtime 与 output directory
- observability_notes:
  - constrained/frozen blocks: `mounting(22:24)`、`odo_scale(21)`、`odo_lever(25:27)` 继续禁用；`ba/bg/sg/sa(9:20)` 维持 README Markov truth-modeled 语义；`GNSS lever(28:30)` 零初值释放
  - active windows: full GNSS，全程只有 `GNSS_POS` 在线；分析只取速度 `>3 m/s` 的 top-5 sustained turning windows，每窗约 `5` 个 `GNSS_POS` 更新
  - state-block behavior:
    - `GNSS lever(28:30)`: 在高转弯窗口内基本 neutral，`z` 轴每窗仅 `0.26~0.44 mm` 改善，`x` 轴反而轻微变差
    - `position(0:2)`: 是 `x/z` 两轴 direct correction 的主吸收块，`z` 轴 share 长期约 `0.79~0.84`
    - `attitude(6:8)`: 保持 secondary share；在 strongest baseline window 的 `z` 轴也只有约 `13.5%`
    - `bg(12:14)`: 当前 truth-modeled 口径下基本 neutral，`dx_bg≈0`
    - `sg(15:17)`: 只有极小 shared update（`~1e-8`），不足以解释 turning-window 中的主耦合
- decision:
  - `Next Actions #1` 已完成，并且结论比上一轮更明确：高转弯窗口没有建立起预期的 `GNSS lever` 可观性/解耦能力；当前 `GNSS_POS` turning updates 的主共享通道是 `position + attitude`，而不是 `gnss_lever`
  - `R` 的变化会改变 innovation 与 correction 的量级，但不会把 direct-share 结构从 `position-dominant` 改成 `lever-dominant`；因此 turning-window 异常不是简单 `R/Q` 调参问题
  - 在当前 truth-modeled `INS/GNSS` 设定里，`bg/sg` 不是 turn-window 主共享 sink；后续对“杆臂仍耦合”的解释应先收紧到 `position-driven z bias` 与 `position-dominant x path + attitude upper-bound`
- next_step:
  - 基于 `output/data2_turn_window_shared_correction_probe/turn_window_update_breakdown.csv`，继续对 `turn_window_1` 与 `turn_window_3` 做逐历元 `x/z` measurement-space 对账，解释为什么 `z` 仍被 `position` 吸收、以及为什么 `x` 在真实 GNSS_POS updates 中也仍是 `position` 主导而不是 `attitude/lever` 主导

### session_id: 20260318-1825-turn-window-cause-judgement-r1
- timestamp: 2026-03-18 18:25 (local)
- objective: 回答“转弯窗里杆臂不收敛到底更像协方差收缩，还是根本没有对应激励”，并把判断限定在当前 fresh turn-window probe 证据内。
- scope:
  - 不新增实验；只复用 `turn_window_shared_correction_probe` 的 `turn_window_update_breakdown.csv`、三组 case 的 `DIAG_*.txt` 与已有 case summary 做因果判断。
  - 核对 turn windows 内 `std_p/std_att/std_lg*` 是否已明显塌缩，并对比 `dx_lever`、`meas_pos_z`、`meas_att_z`、`meas_lever_z` 的量级关系。
- changed_files:
  - `walkthrough.md`
- commands:
  - `python` 读取 `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd{0p25,1,2}_qlever1/DIAG_*.txt`，按 `starttime` 对齐到 top-5 turn windows，汇总 `std_pz/std_lgz/std_phiD`
  - `python` 汇总 `turn_window_update_breakdown.csv` 中 `dx_lever_norm`、`meas_pos_z`、`meas_att_z`、`meas_lever_z` 与窗口前后 `lever_error_{x,z}`
- artifacts:
  - 复用 `output/data2_turn_window_shared_correction_probe/{turn_window_update_breakdown.csv,case_update_summary.csv,summary.md,manifest.json}`
  - 新增 `output/data2_turn_window_shared_correction_probe/{diag_turn_window_std_summary.csv,cause_judgement_summary.md}`
  - 复用 `output/data2_turn_window_shared_correction_probe/artifacts/cases/{eskf_rstd0p25_qlever1,eskf_rstd1_qlever1,eskf_rstd2_qlever1}/DIAG_*.txt`
- metrics:
  - turn-window covariance / uncertainty summary:
    - `R×0.25`: `mean_std_pz=0.077267 m`, `mean_std_lgz=0.076842 m`, `mean_heading_std=1.559872 deg`
    - `baseline R×1`: `mean_std_pz=0.096655 m`, `mean_std_lgz=0.092176 m`, `mean_heading_std=1.645889 deg`
    - `R×2`: `mean_std_pz=0.120738 m`, `mean_std_lgz=0.108220 m`, `mean_heading_std=1.756681 deg`
  - baseline turn-window update magnitude summary:
    - `mean_dx_lever_norm=0.002348 m`
    - `mean_abs(meas_pos_z)≈0.008371 m`
    - `mean_abs(meas_att_z)≈0.000461 m`
    - `mean_abs(meas_lever_z)≈0.000192 m`
    - `mean lever_error_norm before/after = 0.286585 / 0.286659 m`
  - strongest baseline window `turn_window_1` 的逐更新 `meas_lever_z` 在 `+0.000974,+0.001222,-0.001615,+0.001583,+0.001341 m` 间来回变化，最终 `lever_z_error` 只从 `0.241795 m` 变到 `0.241730 m`
- observability_notes:
  - active windows: 仍是 top-5 sustained turning windows；绝对时间对应 `529536.543/530274.515/528723.771/528123.104/528610.021 s`
  - `GNSS lever(28:30)` 并非完全 frozen：`std_lgz` 仍维持 `7.7~10.8 cm`，每次更新也有约 `2.3 mm` 的 `dx_lever_norm`
  - `attitude(6:8)` 也并非完全无激励：turn windows 内 `heading std≈1.56~1.76 deg`，`meas_att_z` 非零，但量级通常仍显著小于 `meas_pos_z`
  - 当前更像“激励没有被稳定积累成 lever 可观性”，而不是“杆臂协方差收缩到无法更新”或“转弯激励根本不存在”
- decision:
  - 当前最稳妥的判断是：这不是纯协方差塌缩问题，也不是纯无激励问题；物理上存在转弯激励，滤波器里 `GNSS lever` 也没有完全失去更新能力，但 residual 仍被持续优先分配给 `position + attitude`，导致 lever 修正方向在窗口内反复换号、无法朝真值稳定积累
  - 因此理论上的“转弯下杆臂应可观”并没有被直接推翻；更准确地说，是当前实现没有把这份理论可观性有效沉淀到 `GNSS lever` 状态中
- next_step:
  - 继续做 `pred_ant_ecef -> innovation` 与逐历元 `x/z` measurement-space 对账，区分“几何敏感度本来就弱”和“几何敏感度存在，但被位置直测项与当前协方差结构吞掉”这两层原因

### session_id: 20260318-1856-turn-window-gain-cov-routing-r1
- timestamp: 2026-03-18 18:56 (local)
- objective: 把 turn-window 里“residual 主要流向 `position + attitude`”继续压缩成更底层的可计算机制，确认这更像几何不足、协方差塌缩，还是 `GNSS_POS` 更新前的 gain-covariance 路由问题。
- scope:
  - 扩展 `gnss_update_debug_output_path`，对每次 `GNSS_POS/GNSS_VEL` 更新额外导出 `prior_std_{pos,att,gnss_lever}`、`P_prior` 的 `pos-att / pos-gnss_lever / att-gnss_lever` 子块，以及 `H/K` 在 `p/att/gnss_lever` 的关键 `x/z` 行列。
  - fresh rebuild `eskf_fusion` 并 fresh rerun `turn_window_shared_correction_probe`，随后对 `turn_window_1/3` 和全 case 的 `k_*`、`corr_*`、`cov+var` cancellation 量做定量汇总。
- changed_files:
  - `src/app/diagnostics.cpp`
  - `scripts/analysis/run_data2_turn_window_shared_correction_probe.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd0p25_qlever1/config_eskf_rstd0p25_qlever1.yaml`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/eskf_rstd2_qlever1/config_eskf_rstd2_qlever1.yaml`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
  - `python` 读取 `output/data2_turn_window_shared_correction_probe/turn_window_update_breakdown.csv`，计算 `var/cov` cancellation，并写出 `gain_cov_cancellation_summary.{csv,md}`
- artifacts:
  - `output/data2_turn_window_shared_correction_probe/{summary.md,cause_judgement_summary.md,case_update_summary.csv,turn_window_summary.csv,turn_window_update_breakdown.csv,diag_turn_window_std_summary.csv,gain_cov_cancellation_summary.csv,gain_cov_cancellation_summary.md,manifest.json}`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/{eskf_rstd0p25_qlever1,eskf_rstd1_qlever1,eskf_rstd2_qlever1}/gnss_updates_*.csv`
- metrics:
  - fresh case summary:
    - `R×0.25`: `prior_std_pz=0.089533 m`、`prior_std_lgz=0.076882 m`、`corr_pz_lgz=-0.854810`、`k_pos_z_norm=0.972151`、`k_att_z_norm=0.123014`、`k_lever_z_norm=0.036402`
    - `baseline R×1`: `prior_std_pz=0.116564 m`、`prior_std_lgz=0.092201 m`、`corr_pz_lgz=-0.789610`、`k_pos_z_norm=0.839718`、`k_att_z_norm=0.106000`、`k_lever_z_norm=0.025279`
    - `R×2`: `prior_std_pz=0.148355 m`、`prior_std_lgz=0.108236 m`、`corr_pz_lgz=-0.729633`、`k_pos_z_norm=0.727881`、`k_att_z_norm=0.085928`、`k_lever_z_norm=0.016896`
  - explicit cancellation summary:
    - `R×0.25`: `var_lz=0.005398561`、`cov_pz_lz=-0.005383878`、`cov_pz_lz+var_lz=1.4683e-05`、`lever_cancel_ratio_z=0.001100`
    - `baseline R×1`: `var_lz=0.008028084`、`cov_pz_lz=-0.008013786`、`cov_pz_lz+var_lz=1.4298e-05`、`lever_cancel_ratio_z=0.001182`
    - `R×2`: `var_lz=0.011356695`、`cov_pz_lz=-0.011343890`、`cov_pz_lz+var_lz=1.2804e-05`、`lever_cancel_ratio_z=0.001313`
  - baseline window-level highlights:
    - `turn_window_1`: `mean_prior_corr_pz_lgz=-0.768619`、`mean_k_pos_z_norm=0.830554`、`mean_k_lever_z_norm=0.014868`、`mean_cov_pz_lz+var_lz≈5.07e-06`
    - `turn_window_3`: `mean_prior_corr_pz_lgz=-0.820424`、`mean_k_pos_z_norm=0.855032`、`mean_k_lever_z_norm=0.034527`、`mean_cov_pz_lz+var_lz≈-1.13e-06`
- artifact_mtime:
  - `case_update_summary.csv=2026-03-18T18:53:31`
  - `summary.md=2026-03-18T18:53:31`
  - `cause_judgement_summary.md=2026-03-18T18:53:31`
  - `gain_cov_cancellation_summary.md=2026-03-18T18:56:44`
- result_freshness_check:
  - `eskf_fusion.exe` 在扩展 `gnss_update` 导出后 fresh rebuild；三组 case 均以新 executable fresh rerun，并生成新的 `gnss_updates/state_series/SOL/DIAG/stdout`
  - 新生成 `case_update_summary.csv`、`summary.md`、`cause_judgement_summary.md` 与 `gain_cov_cancellation_summary.{csv,md}` 都与本次 fresh rerun 时间一致
- observability_notes:
  - constrained/frozen blocks: `mounting(22:24)`、`odo_scale(21)`、`odo_lever(25:27)` 继续禁用；`ba/bg/sg/sa(9:20)` 维持 README Markov truth-modeled 语义；`GNSS lever(28:30)` 零初值释放
  - active windows: full GNSS、仅 `GNSS_POS` 在线；分析仍限于速度 `>3 m/s` 的 top-5 sustained turning windows，并对 `turn_window_1/3` 做逐历元 `x/z` gain-cov 对账
  - state-block behavior:
    - `GNSS lever z(30)`: 不是没有几何、不是方差塌缩，而是在更新前就与 `position z(2)` 形成近乎完美的负相关，导致 `lever_z` gain 直接分子几乎被抵消
    - `position z(2)`: 仍保留明显非零 residual mode（`var_pz + cov_pz_lz > 0`），因此实际 `GNSS_POS z` 更新继续优先流向 `position`
    - `attitude z(8)`: `k_att_z_norm` 处于 secondary 量级，能参与 share，但不是当前 `lever_z` 无法积累的主阻塞
    - `x` 轴没有出现 `cov_px_lgx≈-var_lgx` 的近完美抵消；它更像“直接位置通道更强 + attitude 只提供上界”的次级问题
- decision:
  - 目前已能把“turn excitation 没有转化成 `GNSS lever` 收敛”压缩成更本质的机制：`z` 轴前验形成了 `cov(p_z,l_gz)≈-var(l_gz)` 的强反相关，导致 `lever_z` gain numerator 在 `GNSS_POS` 更新前被几乎完全抵消，残差自然继续流向 `position`
  - 因此当前主问题已不是“有没有转弯激励”或“杆臂方差是不是塌了”，而是 `GNSS_POS` 更新链在多历元后把 `position` 与 `gnss_lever_z` 组织成了一个近乎自抵消的竞争结构；`runtime PVA anchor` 能工作，正是因为它强行切断了这个 `position` alias
- next_step:
  - 追溯 `cov(p_z,l_gz)≈-var(l_gz)` 是在早期哪些 `GNSS_POS` updates 中形成并锁住的；优先做 baseline、`standard_reset_gamma`、`true_iekf` 与轻度 `position` 约束/冻结的 A/B，对比 cancellation 是否被打破

### session_id: 20260318-1912-turn-window-xy-focus-r1
- timestamp: 2026-03-18 19:12 (local)
- objective: 按用户最新澄清，把 turn-window 归因从 `z` 轴转到更关心的 `x/y` 两轴，确认它们到底是“有激励但没积累”、还是也存在类似 `z` 轴的 cancellation。
- scope:
  - 不重跑 solver；直接复用 `turn_window_shared_correction_probe` fresh 产物，对 `x/y` 两轴补做 share、误差变化与 `pos-gnss_lever` 相关性汇总。
  - 从 `turn_window_update_breakdown.csv` 计算 `meas_pos/meas_att/meas_lever` 在 `y` 轴的 realized share，并从 case 级 `gnss_updates_*.csv` 读取 `prior_cov_pos_gnss_lever_mat` 以对账 `corr_py_lgy`。
- changed_files:
  - `walkthrough.md`
- commands:
  - `python` 读取 `output/data2_turn_window_shared_correction_probe/turn_window_update_breakdown.csv`，生成 `xy_focus_case_summary.csv`
  - `python` 读取 `output/data2_turn_window_shared_correction_probe/artifacts/cases/*/gnss_updates_*.csv`，汇总 `corr_py_lgy`
- artifacts:
  - `output/data2_turn_window_shared_correction_probe/xy_focus_case_summary.csv`
  - 复用 `output/data2_turn_window_shared_correction_probe/{turn_window_update_breakdown.csv,case_update_summary.csv}`
  - 复用 `output/data2_turn_window_shared_correction_probe/artifacts/cases/{eskf_rstd0p25_qlever1,eskf_rstd1_qlever1,eskf_rstd2_qlever1}/gnss_updates_*.csv`
- metrics:
  - case-level `x` summary:
    - `R×0.25 / baseline / R×2`: `mean_pos_share_x=0.893636/0.915155/0.934902`
    - `mean_att_share_x=0.065099/0.054403/0.042569`
    - `mean_lever_share_x=0.041264/0.030441/0.022529`
    - baseline `mean_abs_y_x=0.015010 m`, `mean_lever_error_x_before/after=0.047188/0.047139 m`
  - case-level `y` summary:
    - `R×0.25 / baseline / R×2`: `mean_pos_share_y=0.905835/0.927955/0.945523`
    - `mean_att_share_y=0.058510/0.045928/0.033101`
    - `mean_lever_share_y=0.035655/0.026117/0.021376`
    - baseline `mean_abs_y_y=0.017022 m`, `mean_lever_error_y_before/after=0.032592/0.032512 m`, `mean_lever_y_error_improve≈0.000080 m`
  - `pos-lever` correlation contrast:
    - `corr_py_lgy≈-0.112569/-0.104819/-0.094319`
    - 对照 `z` 轴 `corr_pz_lgz≈-0.854810/-0.789610/-0.729633`
    - 对照 `x` 轴 `corr_px_lgx≈0.017889/0.018087/0.017603`
  - baseline representative windows:
    - `turn_window_1` 中 `y` 轴 `pos_share_y≈0.892~0.983`、`lever_share_y≈0.007~0.075`，`lever_error_y` 从 `0.023344 m` 漂到 `0.030598 m`
    - `turn_window_3` 中 `y` 轴 `pos_share_y≈0.773~0.883`、`lever_share_y≈0.020~0.209`，但 `lever_error_y` 仍在 `0.033~0.0368 m` 间来回波动
- observability_notes:
  - `GNSS lever x(28)`: 没有 `z` 轴那种 near-perfect cancellation，但 realized share 与 gain 仍持续被 `position_x` 压制；turning window 里偶有短时改善，却没有稳定积累
  - `GNSS lever y(29)`: 量测 residual 并不小，且 `corr_py_lgy` 只有弱负相关，不支持“像 z 那样被近完美 cancellation”；它更像是更新几乎总被 `position_y` 吸走
  - 因此在用户真正关心的 `x/y` 两轴上，当前主问题可更准确地表述为 `position-dominant routing`，而不是 `z` 轴那种 `gain numerator cancellation`
- decision:
  - `x/y` 两轴都不是“根本没有 innovation”；尤其 `y` 轴的 `mean_abs_y_y` 甚至高于 `x`
  - 但 `x/y` 的 realized correction 都极度 `position` 主导，`lever` share 仅约 `2%~4%`，所以 turning 没有把 `x/y` 的杆臂信息有效沉淀下来；这和 `z` 轴的 failure mode 不同，不能再用同一套 cancellation 解释全部三轴
- next_step:
  - 后续若继续深挖 `GNSS lever`，优先把 `x/y` 作为一组单独问题处理：检查为什么 `position` 直测项会长期压过 `lever_x/lever_y`，以及 `attitude` 在什么条件下才会从 upper-bound 变成真正可积累通道

### session_id: 20260318-1922-turn-vs-straight-xy-r1
- timestamp: 2026-03-18 19:22 (local)
- objective: 回答用户新增问题：相对直行，转弯状态下 `x/y` 两轴的 innovation 分配有没有出现真实的解耦特征。
- scope:
  - 不重跑 solver；复用 `turn_window_shared_correction_probe` 的 fresh `GNSS_POS` update breakdown。
  - 为每个 top turning window 匹配一个速度相近、时长相同、且 `turn_score` 尽可能小的 straight window，再比较 `x/y` 两轴在 `turn vs straight` 下的 `pos/att/lever` share、innovation 量级与窗口前后杆臂误差改善。
- changed_files:
  - `scripts/analysis/analyze_turn_vs_straight_xy_decoupling.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py`
- artifacts:
  - `output/data2_turn_window_shared_correction_probe/{matched_straight_windows.csv,turn_vs_straight_xy_window_summary.csv,turn_vs_straight_xy_case_summary.csv,turn_vs_straight_xy_summary.md,turn_vs_straight_xy_manifest.json}`
  - 复用 `output/data2_turn_window_shared_correction_probe/{turn_windows.csv,turn_window_update_breakdown.csv}`
- metrics:
  - matched straight windows 的 `turn_score≈0.106/0.198/0.255/0.337/0.437 deg/s`，且 `speed_delta≈±0.004 m/s` 以内，满足“低转向 + 速度匹配”对照。
  - baseline `x` axis:
    - turn vs straight `lever_share_x = 0.056835 vs 0.024199`
    - turn vs straight `pos_share_x = 0.869280 vs 0.916062`
    - turn vs straight `lever_x_error_improve = -0.001614 vs +0.000086 m`
  - baseline `y` axis:
    - turn vs straight `lever_share_y = 0.081459 vs 0.009302`
    - turn vs straight `pos_share_y = 0.857279 vs 0.960555`
    - turn vs straight `lever_y_error_improve = -0.002534 vs -0.000095 m`
  - 三组 case 一致趋势：
    - turning 相对 straight，`lever_share_x` 提升约 `0.033~0.0528`
    - turning 相对 straight，`lever_share_y` 提升约 `0.0578~0.0785`
    - 但 `lever_x_error_improve` 与 `lever_y_error_improve` 在 turning 中都比 straight 更差
- observability_notes:
  - `GNSS lever x/y(28:29)` 在 turning 相对 straight 时，确实出现了“部分解耦特征”：一部分 innovation 从 `position` 转移到了 `lever`
  - 但这份解耦只是 share-level 的弱特征，不是有效收敛：在窗口尺度上，`lever_x/lever_y` 并没有因此向真值靠拢，反而通常更偏离
  - 这说明 turning 并非完全没打开 `lever_x/lever_y` 通道；真正的问题更像“lever 通道被打开了，但 update direction / multi-epoch accumulation 仍然不对”
- decision:
  - 对用户关心的 `x/y` 两轴，答案不是简单的“没有解耦”，而是更精确的：`turning` 相比 `straight` 的确表现出弱解耦特征，但这种特征没有变成正确收敛，反而在当前实现下常伴随更差的 `lever` 误差演化
  - 因此下一步不应再问“turning 有没有打开 `lever` 通道”，而应直接问“为什么打开后更新方向/积累仍然不把 `lever_x/y` 拉向真值”
- next_step:
  - 继续对 `x/y` 两轴补 `k_pos_y / k_att_y / k_lever_y` 与逐历元 `sign(innovation) -> sign(dx_lever)` 对账，区分是 gain 方向不对、还是单次方向对但被后续 epochs 反复冲掉

### session_id: 20260318-1949-turn-xy-sign-consistency-r1
- timestamp: 2026-03-18 19:49 (local)
- objective: 在 turn-vs-straight `x/y` 弱解耦已确认的基础上，进一步确认失败本质更接近“单步方向就错”还是“单步方向对但后续累计被冲掉”。
- scope:
  - 扩展 `GNSS_POS` 更新诊断，把 `y` 轴的 `H/K` 向量日志补齐到和现有 `x/z` 同一粒度。
  - fresh 重编译并重跑 `turn_window_shared_correction_probe`，再新增 `turn_vs_straight_xy_sign_consistency` 后处理，把 `turn` 与 matched `straight` 的 `dx_sign_match / same-axis sign / temporary-good-then-bad` 分开统计。
- changed_files:
  - `src/app/diagnostics.cpp`
  - `scripts/analysis/run_data2_turn_window_shared_correction_probe.py`
  - `scripts/analysis/analyze_turn_window_xy_sign_consistency.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
  - `python -m py_compile scripts\analysis\analyze_turn_window_xy_sign_consistency.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py`
- artifacts:
  - `output/data2_turn_window_shared_correction_probe/{summary.md,case_update_summary.csv,turn_window_update_breakdown.csv,turn_vs_straight_xy_summary.md,turn_vs_straight_xy_case_summary.csv}`
  - `output/data2_turn_window_shared_correction_probe/{turn_vs_straight_xy_sign_per_update.csv,turn_vs_straight_xy_sign_window_summary.csv,turn_vs_straight_xy_sign_case_phase_summary.csv,turn_vs_straight_xy_sign_case_summary.csv,turn_vs_straight_xy_sign_summary.md,turn_vs_straight_xy_sign_manifest.json}`
  - `output/data2_turn_window_shared_correction_probe/artifacts/cases/{eskf_rstd0p25_qlever1,eskf_rstd1_qlever1,eskf_rstd2_qlever1}/gnss_updates_*.csv`
- metrics:
  - baseline matched turn-vs-straight 仍保持上一轮结论：`x lever_share=0.056835 vs 0.024199`、`y lever_share=0.081459 vs 0.009302`，但 `x lever_error_improve=-1.614 mm vs +0.086 mm`、`y=-2.534 mm vs -0.095 mm`
  - baseline `x` axis sign-consistency:
    - turn `dx_sign_match=0.44` vs straight `0.64`
    - turn `same_axis_sign=0.52`, `same_axis_dx_align=0.68`
    - turn `best_prefix_improve=2.053 mm`, `temporary_good_then_bad_rate=0.40`
  - baseline `y` axis sign-consistency:
    - turn `dx_sign_match=0.44` vs straight `0.52`
    - turn `same_axis_sign=0.40`, `same_axis_dx_align=0.96`
    - turn `best_prefix_improve=1.688 mm`, `temporary_good_then_bad_rate=0.20`
  - 三组 case 一致趋势：
    - `x` 轴 `turn dx_sign_match=0.36~0.44 < straight 0.44~0.64`，且 `turn same_axis_sign=0.48~0.52 > turn dx_sign_match`，说明 turn 窗口里 `x` 的同轴 lever 通道被 weakly 打开，但 final `dx_lgx` 常被 cross-axis contamination 拉坏
    - `y` 轴 `turn dx_sign_match=0.40~0.48 < straight 0.52`，同时 `turn same_axis_sign=0.40~0.44`、`same_axis_dx_align=0.84~0.96`，说明 `y` 更像是同轴 lever 通道本身就没有稳定指向真值
- artifact_mtime:
  - `output/data2_turn_window_shared_correction_probe/turn_vs_straight_xy_sign_summary.md`: `2026-03-18 19:49` (local)
  - `output/data2_turn_window_shared_correction_probe/turn_vs_straight_xy_sign_case_summary.csv`: `2026-03-18 19:49` (local)
  - `output/data2_turn_window_shared_correction_probe/turn_vs_straight_xy_summary.md`: `2026-03-18 19:49` (local)
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml` reused as truth reference / experiment baseline; per-case运行配置由 `run_data2_turn_window_shared_correction_probe.py` fresh 生成到 `output/data2_turn_window_shared_correction_probe/artifacts/cases/*/config_*.yaml`
- dataset_time_window:
  - `data2` 全程 GNSS 有效窗口内抽取 top-5 turn windows，并为每个 turn window 匹配同速 straight window；核心对比窗口仍集中在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮 `eskf_fusion` fresh rebuild 后重新生成 `turn_window_shared_correction_probe` 全套产物，`turn_vs_straight_xy_*` 与 `turn_vs_straight_xy_sign_*` 均来自同一批 fresh `gnss_updates_*.csv`
- observability_notes:
  - 状态块：`GNSS lever x/y (28:29)` 在纯 `INS/GNSS` truth-modeled 口径下保持释放，转弯与 matched straight 窗口都处于 `GNSS_POS` 有效更新阶段；当前分析没有引入 `PVA runtime anchor`
  - 传感器窗口：仅比较 `GNSS_POS` 更新；turn windows 与 matched straight windows 都满足相同 `5 s` 窗宽、近似相同速度、GNSS schedule 未中断
  - 行为结果：
    - `x(28)`：turning 相对 straight 确实打开了更强的 lever 通道，但失败主因更接近 `same-axis lever channel weakly right + final K*y 被 cross-axis contamination / reversal 拉坏`
    - `y(29)`：turning 也提高了 lever share，但失败主因更接近 `same-axis lever channel itself still points wrong`，而不是先对后错
- decision:
  - 已可排除“turning 只是没有激励，所以 `x/y` 不可观”这一解释
  - 对当前实现更准确的本质判断是：`x/y` 的 turning 弱解耦确实存在，但失败主要已经发生在单步 `K*y` 路由层，而不是单纯因为多历元后才被冲掉；其中 `x` 和 `y` 的单步失败机理还不完全相同
- next_step:
  - 直接做 `baseline`、`standard_reset_gamma`、`true_iekf` 与轻度 `position` 约束/冻结的 A/B，并用当前 `turn_vs_straight_xy_sign_*` 指标分别检验：`x` 的 cross-axis contamination / reversal 能否下降，`y` 的 same-axis sign 能否转正

### session_id: 20260318-2025-turn-window-variant-ab-r1
- timestamp: 2026-03-18 20:25 (local)
- objective: 继续用户要求的进一步实验，在同一 pure `INS/GNSS + truth-modeled` turn-window 工位上直接比较 `baseline`、`standard_reset_gamma`、`true_iekf`、`position_anchor_diag`，确认 `x/y` 失败究竟更像 reset/IEKF 问题还是 `position` 竞争问题。
- scope:
  - 新增独立 runner `run_data2_turn_window_variant_ab_probe.py`，复用 `eskf_rstd1_qlever1` pure `INS/GNSS` 基线配置，只对四个变体做最小配置突变。
  - fresh 运行 variant probe，并复用已有 `analyze_turn_vs_straight_xy_decoupling.py` 与 `analyze_turn_window_xy_sign_consistency.py` 对新输出目录做后处理。
- changed_files:
  - `scripts/analysis/run_data2_turn_window_variant_ab_probe.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_variant_ab_probe.py`
  - `python scripts\analysis\run_data2_turn_window_variant_ab_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py --probe-dir output\data2_turn_window_variant_ab_probe`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py --probe-dir output\data2_turn_window_variant_ab_probe`
- artifacts:
  - `output/data2_turn_window_variant_ab_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv}`
  - `output/data2_turn_window_variant_ab_probe/{matched_straight_windows.csv,turn_vs_straight_xy_window_summary.csv,turn_vs_straight_xy_case_summary.csv,turn_vs_straight_xy_summary.md,turn_vs_straight_xy_manifest.json}`
  - `output/data2_turn_window_variant_ab_probe/{turn_vs_straight_xy_sign_per_update.csv,turn_vs_straight_xy_sign_window_summary.csv,turn_vs_straight_xy_sign_case_phase_summary.csv,turn_vs_straight_xy_sign_case_summary.csv,turn_vs_straight_xy_sign_summary.md,turn_vs_straight_xy_sign_manifest.json}`
  - `output/data2_turn_window_variant_ab_probe/artifacts/cases/{baseline,standard_reset_gamma,true_iekf,position_anchor_diag}/`
- metrics:
  - `baseline` 复现上一轮结论：turn `x/y lever_share=0.0568/0.0815`，turn `dx_sign_match_x/y=0.44/0.44`，turn `total_window_improve_x/y=-1.614/-2.534 mm`
  - `standard_reset_gamma` 基本无效：
    - turn `dx_sign_match_x/y=0.44/0.44`
    - turn `total_window_improve_x/y=-1.620/-2.545 mm`
    - `lever_share_x/y=0.0570/0.0817`
  - `true_iekf` 也未改变根结论：
    - turn `dx_sign_match_x/y=0.44/0.44`
    - turn `total_window_improve_x/y=-1.584/-2.508 mm`
    - `x` 的 `same_axis_sign` 反而降到 `0.28`，`y` 仅到 `0.44`
  - `position_anchor_diag` 明显翻正：
    - turn `x/y lever_share=0.0789/0.1188`
    - turn `dx_sign_match_x/y=0.60/0.80`
    - turn `total_window_improve_x/y=+1.246/+6.215 mm`
    - turn `temporary_good_then_bad_rate_x/y=0/0`
- artifact_mtime:
  - `output/data2_turn_window_variant_ab_probe/summary.md`: `2026-03-18 20:25` (local)
  - `output/data2_turn_window_variant_ab_probe/turn_vs_straight_xy_summary.md`: `2026-03-18 20:25` (local)
  - `output/data2_turn_window_variant_ab_probe/turn_vs_straight_xy_sign_summary.md`: `2026-03-18 20:25` (local)
- config_hash_or_mtime:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - per-case fresh configs written to `output/data2_turn_window_variant_ab_probe/artifacts/cases/*/config_*.yaml`
- dataset_time_window:
  - 仍使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；核心时间窗保持在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮所有 variant case 都是 fresh solver 运行，新目录下 `turn_window_update_breakdown.csv`、`turn_vs_straight_xy_*`、`turn_vs_straight_xy_sign_*` 共享同一批 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：`GNSS lever x/y (28:29)` 持续释放；`baseline`、`standard_reset_gamma`、`true_iekf` 都保持 free-PVA，只改 reset/IEKF 语义；`position_anchor_diag` 只在 GNSS 后对 `position(1:3)` 做 runtime truth anchor，`velocity/attitude` 仍释放
  - 传感器窗口：全部只比较 `GNSS_POS` 更新；turn vs straight 的 5s 窗宽、速度匹配与 GNSS schedule 保持一致
  - 行为结果：
    - `standard_reset_gamma` 和 `true_iekf` 都没有把 `x/y` 从负的 turn-window error-improve 拉正，说明当前 `x/y` 失败不主要卡在这两个开关
    - `position_anchor_diag` 会同时改善 `x` 的 `dx_sign_match` 与 `temporary_good_then_bad`，也会显著把 `y` 的 `dx_sign_match` 拉到 `0.8`，说明 `position` 竞争/alias 是当前更接近本质的 shared culprit
- decision:
  - 对当前 pure `INS/GNSS` turn-window 问题，`position` 竞争比 `standard_reset_gamma` 或 `true_iekf` 更像主导根因
  - 这意味着下一步不应继续把主要精力放在“切换 ESKF reset 语义 / true_iekf 开关”，而应优先寻找工程可行的、能削弱 `position` 直接竞争但不依赖真值 anchor 的替代干预
- next_step:
  - 设计一个不依赖真值的 `position-competition mitigation` 试验族，例如 `GNSS_POS` 分步/重加权、局部 position sink 抑制或轻量 pseudo-constraint，并继续用 `turn_vs_straight_xy_sign_*` 做唯一判据

### session_id: 20260318-2304-turn-pos-gain-scale-r1
- timestamp: 2026-03-18 23:04 (local)
- objective: 把上轮“`position` 竞争是主矛盾”的判断落到真正不依赖真值的工程可行干预上，测试只在 `GNSS_POS` 更新时缩放 `position(0:2)` 状态的 Kalman gain 行，能否把 `x/y` 杆臂 turn-window 行为推向正常收敛。
- scope:
  - 在 `ESKF/Diagnostics/Pipeline/Config` 链路新增 `fusion.gnss_pos_position_gain_scale`，仅影响 `GNSS_POS` 更新时 `position` 行的增益。
  - 新增 `run_data2_turn_window_position_gain_scale_probe.py`，在同一 pure `INS/GNSS + truth-modeled` 基线上扫描 `position_gain_scale = 1.0 / 0.5 / 0.25 / 0.0`，并复用现有 `turn_vs_straight_xy` 与 `turn_vs_straight_xy_sign` 后处理。
- changed_files:
  - `include/core/eskf.h`
  - `src/core/eskf_engine.cpp`
  - `include/app/diagnostics.h`
  - `src/app/diagnostics.cpp`
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/run_data2_turn_window_position_gain_scale_probe.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_position_gain_scale_probe.py`
  - `python -m py_compile scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py`
  - `python -m py_compile scripts\analysis\analyze_turn_window_xy_sign_consistency.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_position_gain_scale_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py --probe-dir output\data2_turn_window_position_gain_scale_probe`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py --probe-dir output\data2_turn_window_position_gain_scale_probe`
- artifacts:
  - `output/data2_turn_window_position_gain_scale_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv}`
  - `output/data2_turn_window_position_gain_scale_probe/{matched_straight_windows.csv,turn_vs_straight_xy_window_summary.csv,turn_vs_straight_xy_case_summary.csv,turn_vs_straight_xy_summary.md,turn_vs_straight_xy_manifest.json}`
  - `output/data2_turn_window_position_gain_scale_probe/{turn_vs_straight_xy_sign_per_update.csv,turn_vs_straight_xy_sign_window_summary.csv,turn_vs_straight_xy_sign_case_phase_summary.csv,turn_vs_straight_xy_sign_case_summary.csv,turn_vs_straight_xy_sign_summary.md,turn_vs_straight_xy_sign_manifest.json}`
- metrics:
  - baseline 复现：turn `x/y total_window_improve=-1.614/-2.534 mm`，turn `dx_sign_match_x/y=0.44/0.44`
  - `position_gain_scale=0.5`：
    - turn `x/y total_window_improve=-0.547/-0.708 mm`
    - turn `dx_sign_match_x/y=0.48/0.52`
    - 说明 moderate suppression 能同时明显缓解两轴，但还不足以把二者都翻正
  - `position_gain_scale=0.25`：
    - turn `x/y total_window_improve=-2.244/+2.213 mm`
    - turn `dx_sign_match_x/y=0.28/0.52`
    - 说明更强 suppression 能把 `y` 翻正，但会显著伤 `x`
  - `position_gain_scale=0.0`：
    - turn `x/y total_window_improve=+0.141/-0.124 mm`
    - turn `dx_sign_match_x/y=0.36/0.48`
    - 说明完全去掉 GNSS_POS 对 `position` 的直接校正会显著压低 reversal，但也把 useful direct correction 一并压掉
- artifact_mtime:
  - `output/data2_turn_window_position_gain_scale_probe/summary.md`: `2026-03-18 23:02:45` (local)
  - `output/data2_turn_window_position_gain_scale_probe/turn_vs_straight_xy_summary.md`: `2026-03-18 23:03:00` (local)
  - `output/data2_turn_window_position_gain_scale_probe/turn_vs_straight_xy_sign_summary.md`: `2026-03-18 23:03:01` (local)
- config_hash_or_mtime:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - per-case fresh configs written to `output/data2_turn_window_position_gain_scale_probe/artifacts/cases/*/config_*.yaml`
- dataset_time_window:
  - 仍使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；核心时间窗保持在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮先 fresh rebuild `eskf_fusion`，再 fresh 运行 `position_gain_scale` 四档变体；所有 `turn_vs_straight_xy_*` 与 `turn_vs_straight_xy_sign_*` 都来自新目录下 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：`GNSS lever x/y (28:29)` 持续释放；本轮唯一干预是 `GNSS_POS` 更新时 `position(0:2)` 的 gain scale，不涉及真值 anchor、reset gamma 开关或 `true_iekf`
  - 传感器窗口：仍只比较 `GNSS_POS` 更新；turn vs straight 的 5s 窗宽、速度匹配与 GNSS schedule 保持一致
  - 行为结果：
    - `x(28)` 对 suppression 强度更敏感于“过抑制会伤害本已很弱的正确通道”，因此 `0.25` 比 `0.5` 更差，`0.0` 只剩极小正改善
    - `y(29)` 对 suppression 更像“需要先压住 `position` 竞争才能把 lever 通道显出来”，所以 `0.25` 已能翻正，但 `0.0` 又会把 useful direct correction 一起杀掉
- decision:
  - `position` 竞争确实是纯 `INS/GNSS` turn-window 中 `x/y` 杆臂失败的主因之一，这一点已经被不依赖真值的 gain-scale 干预再次验证
  - 但常数型全局 `position_gain_scale` 还不足以同时修好 `x` 和 `y`；更合理的下一步应该是条件化/分轴化或分步更新策略，而不是继续扫单一常数
- next_step:
  - 继续做真正工程化的 `position-competition mitigation`，优先顺序：`GNSS_POS` 分步更新、按轴/按工况自适应的 `position_gain_scale`、以及局部 pseudo-constraint；仍以 `turn_vs_straight_xy_sign_*` 作为唯一验收标准

### session_id: 20260318-2311-turn-gnsspos-staged-r1
- timestamp: 2026-03-18 23:11 (local)
- objective: 继续执行 `Next Actions #2`，验证“`position` 竞争是否只是更新次序问题”：把单次 `GNSS_POS` 拆成“先 non-position、后 position-only”的 staged 更新，并与 `joint baseline`、`joint pos_gain=0.5`、`staged+0.5` 在同一 pure `INS/GNSS + truth-modeled` turn-window 工位上对比。
- scope:
  - 在 `Fusion/Config/Diagnostics` 链路新增 `fusion.gnss_pos_update_mode`，支持 `joint` 与 `stage_nonpos_then_pos`。
  - 让 `Diagnostics` 记录 `GNSS_POS_STAGE*` 标签，并在 postprocess 中把 staged 两步聚合回“每个 GNSS 历元一条有效 `GNSS_POS` 记录”，保持既有 `turn_vs_straight_xy_*` 与 `turn_vs_straight_xy_sign_*` 判据可复用。
  - 新增独立 runner `run_data2_turn_window_gnss_pos_staged_probe.py`，比较 `baseline_joint`、`joint_pos_gain_0p5`、`staged_nonpos_then_pos`、`staged_nonpos_then_pos_pos_gain_0p5` 四组。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/diagnostics.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/run_data2_turn_window_shared_correction_probe.py`
  - `scripts/analysis/analyze_turn_window_xy_sign_consistency.py`
  - `scripts/analysis/run_data2_turn_window_gnss_pos_staged_probe.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_gnss_pos_staged_probe.py`
  - `python -m py_compile scripts\analysis\run_data2_turn_window_shared_correction_probe.py`
  - `python -m py_compile scripts\analysis\analyze_turn_window_xy_sign_consistency.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_gnss_pos_staged_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py --probe-dir output\data2_turn_window_gnss_pos_staged_probe`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py --probe-dir output\data2_turn_window_gnss_pos_staged_probe`
- artifacts:
  - `output/data2_turn_window_gnss_pos_staged_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv}`
  - `output/data2_turn_window_gnss_pos_staged_probe/{matched_straight_windows.csv,turn_vs_straight_xy_window_summary.csv,turn_vs_straight_xy_case_summary.csv,turn_vs_straight_xy_summary.md,turn_vs_straight_xy_manifest.json}`
  - `output/data2_turn_window_gnss_pos_staged_probe/{turn_vs_straight_xy_sign_per_update.csv,turn_vs_straight_xy_sign_window_summary.csv,turn_vs_straight_xy_sign_case_phase_summary.csv,turn_vs_straight_xy_sign_case_summary.csv,turn_vs_straight_xy_sign_summary.md,turn_vs_straight_xy_sign_manifest.json}`
  - `output/data2_turn_window_gnss_pos_staged_probe/artifacts/cases/{baseline_joint,joint_pos_gain_0p5,staged_nonpos_then_pos,staged_nonpos_then_pos_pos_gain_0p5}/`
- metrics:
  - `baseline_joint` 复现前一轮：turn `x/y total_window_improve=-1.614/-2.534 mm`，turn `dx_sign_match_x/y=0.44/0.44`
  - `joint_pos_gain_0p5` 仍是当前最平衡 joint 方案：turn `x/y total_window_improve=-0.547/-0.708 mm`，turn `dx_sign_match_x/y=0.48/0.52`
  - `staged_nonpos_then_pos`：
    - turn `x/y total_window_improve=-1.397/-2.697 mm`
    - turn `dx_sign_match_x/y=0.40/0.44`
    - 只对 `x` 比 baseline 略有缓解，但 `y` 更差，且整体明显弱于 `joint_pos_gain_0p5`
  - `staged_nonpos_then_pos_pos_gain_0p5`：
    - turn `x/y total_window_improve=-0.571/-0.847 mm`
    - turn `dx_sign_match_x/y=0.44/0.52`
    - 与 `joint_pos_gain_0p5` 几乎同量级，但仍略差，未表现出“先 non-position 再 position”额外收益
- artifact_mtime:
  - `output/data2_turn_window_gnss_pos_staged_probe/summary.md`: `2026-03-18 23:18:40` (local)
  - `output/data2_turn_window_gnss_pos_staged_probe/turn_vs_straight_xy_summary.md`: `2026-03-18 23:19:13` (local)
  - `output/data2_turn_window_gnss_pos_staged_probe/turn_vs_straight_xy_sign_summary.md`: `2026-03-18 23:19:23` (local)
- config_hash_or_mtime:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - per-case fresh configs written to `output/data2_turn_window_gnss_pos_staged_probe/artifacts/cases/*/config_*.yaml`
- dataset_time_window:
  - 仍使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；核心时间窗保持在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮先 fresh rebuild `eskf_fusion`，再 fresh 运行 staged probe 四组变体；`turn_window_update_breakdown.csv` 与后续 `turn_vs_straight_xy_*` / `turn_vs_straight_xy_sign_*` 都来自新目录下 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：`GNSS lever x/y (28:29)` 持续释放；本轮新增干预是 `GNSS_POS` 内部更新模式 `joint` vs `stage_nonpos_then_pos`，其中 staged 先开放非 `position(0:2)` 的状态块，再做 `position-only` 补更新；不涉及真值 anchor
  - 传感器窗口：仍只比较 `GNSS_POS` 更新；turn vs straight 的 5s 窗宽、速度匹配与 GNSS schedule 保持一致
  - 行为结果：
    - staged 并没有把 `x/y` 从负的 turn-window error-improve 拉正，也没有在 `dx_sign_match` 上超过 `joint_pos_gain_0p5`
    - `x(28)`：staged 虽略减弱 baseline 的负改善，但 `dx_sign_match` 反而降到 `0.40`，说明“先 non-position”没有解决主导 `x` 的 cross-axis contamination / reversal
    - `y(29)`：staged 的 `same_axis_sign` 仍停在 `0.40`，且 total improve 更差，说明 `y` 的主要问题并不是 `position` 先抢走 innovation，而是 same-axis lever 通道本身就不稳定朝真值指向
- decision:
  - `position` 竞争是真问题，但并不能被简单归结为“position 在 joint update 里先拿走了该给 lever 的 innovation”
  - 在当前 pure `INS/GNSS` turn-window 口径下，`GNSS_POS` 分步顺序并不是主修复杠杆；`joint_pos_gain_0p5` 的改善主要来自 suppression 强度本身，而不是 update ordering
- next_step:
  - 把主线从“再设计不同 staged 次序”切到两条更细的 axis-specific 诊断：`x` 拆 `cross-axis contamination` 来源，`y` 拆 same-axis sign instability；若继续做工程干预，优先尝试按轴/按创新分量自适应，而不是继续改 GNSS_POS 内部顺序

### session_id: 20260318-2334-turn-xy-term-source-r1
- timestamp: 2026-03-18 23:34 (local)
- objective: 按当前优先级继续进行 axis-specific 实验，不再改 solver 顺序，而是直接分解 `dx_lgx/dx_lgy = k_lg*_x*y_x + k_lg*_y*y_y + k_lg*_z*y_z`，确认 `x` 的主要 cross-axis contaminant 是哪一项，以及 `y` 为何更像 same-axis 自身失稳。
- scope:
  - 新增独立后处理脚本 `analyze_turn_window_xy_term_source.py`，读取 `turn_window_position_gain_scale_probe` 的 raw `gnss_updates_*.csv`。
  - 在 turn / matched straight 双窗口口径下，对 `baseline`、`pos_gain_0p5`、`pos_gain_0p25`、`pos_gain_0p0` 输出 per-update / window / case-phase / case-summary 的 term-level 分解。
- changed_files:
  - `scripts/analysis/analyze_turn_window_xy_term_source.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_window_xy_term_source.py`
  - `python scripts\analysis\analyze_turn_window_xy_term_source.py`
- artifacts:
  - `output/data2_turn_window_xy_term_source_probe/{manifest.json,turn_vs_straight_xy_term_per_update.csv,turn_vs_straight_xy_term_window_summary.csv,turn_vs_straight_xy_term_case_phase_summary.csv,turn_vs_straight_xy_term_case_summary.csv,turn_vs_straight_xy_term_summary.md}`
- metrics:
  - `x(28)` baseline turn:
    - `total_window_improve=-1.614 mm`
    - `|term_x/y/z|=1.099/0.833/0.229 mm`
    - `same_axis_good_final_bad_rate=0.20`
    - `dominant_bad_cross_y/z_rate=0.60/0.00`
  - `x(28)` `pos_gain_0p5` turn:
    - `total_window_improve=-0.547 mm`
    - `|term_x/y/z|=1.006/0.748/0.252 mm`
    - `same_axis_good_final_bad_rate=0.12`
    - `dominant_bad_cross_y/z_rate=0.40/0.00`
    - 说明 moderate suppression 对 `x` 的帮助，主要来自压低 `k_lgx_y * y_y` 污染项，而不是显著增强 same-axis `k_lgx_x * y_x`
  - `y(29)` baseline turn:
    - `total_window_improve=-2.534 mm`
    - `|term_x/y/z|=0.657/1.590/0.183 mm`
    - `same_axis_sign_match=0.40`
    - dominant term `y` rate=`0.80`
  - `y(29)` `pos_gain_0p5` / `0.25` turn:
    - `pos_gain_0p5`: `same_axis_sign_match=0.44`, `total_window_improve=-0.708 mm`
    - `pos_gain_0p25`: `same_axis_sign_match=0.56`, `total_window_improve=+2.213 mm`
    - 说明 `y` 的瓶颈更像 same-axis `k_lgy_y * y_y` 自己的 sign/stability，而不是主要被 cross-axis 拖坏
- artifact_mtime:
  - `output/data2_turn_window_xy_term_source_probe/manifest.json`: `2026-03-18 23:33:40` (local)
  - `output/data2_turn_window_xy_term_source_probe/turn_vs_straight_xy_term_summary.md`: `2026-03-18 23:33:40` (local)
  - `output/data2_turn_window_xy_term_source_probe/turn_vs_straight_xy_term_case_summary.csv`: `2026-03-18 23:33:40` (local)
- config_hash_or_mtime:
  - source probe reused from `output/data2_turn_window_position_gain_scale_probe/`
  - reference config reused from `config_data2_baseline_eskf.yaml`
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；核心时间窗保持在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮 analysis 直接读取上一轮 fresh `position_gain_scale probe` 的 raw `gnss_updates_*.csv`，并重新生成独立的 term-source 汇总目录；`dx_reconstruction_error` 在各 case-phase 下均约 `1e-10 m`
- observability_notes:
  - 状态块：聚焦 `GNSS lever x/y (28:29)` 的单步 `K*y` 贡献分解，不新增任何状态冻结或真值 anchor
  - 传感器窗口：仍只分析 `GNSS_POS` 更新；turn vs straight 的 5s 窗宽、速度匹配与 GNSS schedule 保持一致
  - 行为结果：
    - `x(28)`：turning 下真正的 dominant bad cross term 已可明确落到 `y` 项，而不是 `z` 项；moderate suppression 的收益主要来自把 `k_lgx_y * y_y` 压小
    - `y(29)`：turning 下 dominant term 多数本来就是 same-axis `y` 项，但其自身 sign match 低，所以主问题不是 cross-axis，而是 same-axis channel 本身方向不稳
- decision:
  - `x` 与 `y` 现在可以明确区分成两类不同问题：`x` 是“same-axis 弱通道 + `y` 项污染翻转”，`y` 是“same-axis 主通道自己就不稳”
  - 因此下一步工程干预不应再是“统一再压一点 position”，而应转向更细粒度的 axis/component-aware suppression
- next_step:
  - 对 `x` 优先设计只抑制 `k_lgx_y * y_y` 一类污染项的实验；对 `y` 优先检查如何让 same-axis `k_lgy_y * y_y` 更稳定朝真值，而不是继续统一改 `GNSS_POS` 顺序或全局缩放

### session_id: 20260319-0039-turn-lgx-from-y-gain-scale-r1
- timestamp: 2026-03-19 00:39 (local)
- objective: 按 `Next Actions #2` 继续推进 component-aware mitigation：不再统一缩放全部 `position` 行，而是直接实现并测试只抑制 `GNSS_POS` 中 `K(l_gx, meas_y)` 路径的 focused 干预，验证 `x` 轴 turn-window 失败是否真的由 `k_lgx_y * y_y` 主导。
- scope:
  - 在 `ESKF/Diagnostics/Fusion` 链路新增 element-wise gain scaling 透传能力，并在 `fusion` 配置中加入 `gnss_pos_lgx_from_y_gain_scale`。
  - 保持 `GNSS_POS` 仍为 joint update，不引入新的 staged 次序变量；仅在 `K` 计算后对 `K(StateIdx::kGnssLever+0, meas_y)` 做附加缩放。
  - 新增独立 runner `run_data2_turn_window_lgx_from_y_gain_scale_probe.py`，以 `joint+pos_gain=0.5` 为 reference，扫描 `lgx_from_y_gain_scale=1.0/0.75/0.5/0.25/0.0`，并复用既有 `turn_vs_straight_xy_*`、`turn_vs_straight_xy_sign_*` 与 term-source 链路。
- changed_files:
  - `include/app/diagnostics.h`
  - `include/app/fusion.h`
  - `include/core/eskf.h`
  - `src/app/config.cpp`
  - `src/app/diagnostics.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `scripts/analysis/run_data2_turn_window_lgx_from_y_gain_scale_probe.py`
  - `walkthrough.md`
- configs:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - per-case fresh configs written to `output/data2_turn_window_lgx_from_y_gain_scale_probe/artifacts/cases/*/config_*.yaml`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_lgx_from_y_gain_scale_probe.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_lgx_from_y_gain_scale_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py --probe-dir output\data2_turn_window_lgx_from_y_gain_scale_probe`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py --probe-dir output\data2_turn_window_lgx_from_y_gain_scale_probe`
  - `python scripts\analysis\analyze_turn_window_xy_term_source.py --source-probe output\data2_turn_window_lgx_from_y_gain_scale_probe --output-dir output\data2_turn_window_lgx_from_y_term_source_probe`
- artifacts:
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,matched_straight_windows.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}`
  - `output/data2_turn_window_lgx_from_y_term_source_probe/{manifest.json,turn_vs_straight_xy_term_per_update.csv,turn_vs_straight_xy_term_window_summary.csv,turn_vs_straight_xy_term_case_phase_summary.csv,turn_vs_straight_xy_term_case_summary.csv,turn_vs_straight_xy_term_summary.md}`
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/artifacts/cases/{baseline_joint,joint_pos_gain_0p5_ref,joint_pos_gain_0p5_lgx_from_y_0p75,joint_pos_gain_0p5_lgx_from_y_0p5,joint_pos_gain_0p5_lgx_from_y_0p25,joint_pos_gain_0p5_lgx_from_y_0p0}/`
- metrics:
  - reference:
    - `baseline_joint`: turn `x/y total_window_improve=-1.614/-2.534 mm`, turn `dx_sign_match_x/y=0.44/0.44`
    - `joint_pos_gain_0p5_ref`: turn `x/y total_window_improve=-0.547/-0.708 mm`, turn `dx_sign_match_x/y=0.48/0.52`
  - selective `lgx_from_y` sweep on top of `pos_gain=0.5`:
    - `0.75`: turn `x/y total_window_improve=+0.009/-0.727 mm`, turn `dx_sign_match_x/y=0.52/0.52`
    - `0.50`: turn `x/y total_window_improve=+0.564/-0.768 mm`, turn `dx_sign_match_x/y=0.56/0.52`
    - `0.25`: turn `x/y total_window_improve=+1.360/-0.835 mm`, turn `dx_sign_match_x/y=0.60/0.52`
    - `0.00`: turn `x/y total_window_improve=+3.433/-1.024 mm`, turn `dx_sign_match_x/y=0.60/0.56`
  - term-source focus:
    - `x(28)`: turn `|term_y|` 随 `lgx_from_y_gain_scale=1.0/0.75/0.5/0.25/0.0` 从 `0.748/0.683/0.554/0.380/0.000 mm` 单调下降；`same_axis_good_final_bad_rate=0.12/0.08/0.04/0.00/0.00`
    - `x(28)`: `dominant_bad_cross_y_rate=0.40/0.40/0.20/0.00/0.00`，而 `same_axis_sign_match_x` 始终约 `0.52`
    - `y(29)`: turn `total_window_improve=-0.708/-0.727/-0.768/-0.835/-1.024 mm`，`same_axis_sign_match_y=0.44/0.44/0.48/0.48/0.48`
- artifact_mtime:
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/summary.md`: `2026-03-19 00:38:07` (local)
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/turn_vs_straight_xy_summary.md`: `2026-03-19 00:38:24` (local)
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/turn_vs_straight_xy_sign_summary.md`: `2026-03-19 00:38:34` (local)
  - `output/data2_turn_window_lgx_from_y_term_source_probe/turn_vs_straight_xy_term_summary.md`: `2026-03-19 00:38:33` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25/config_joint_pos_gain_0p5_lgx_from_y_0p25.yaml`: `2026-03-19 00:37:03` (local)
  - `output/data2_turn_window_lgx_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p0.yaml`: `2026-03-19 00:37:34` (local)
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；核心时间窗保持在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮先 fresh rebuild `eskf_fusion`，再 fresh 运行 `lgx_from_y` selective probe 六组变体；后续 `turn_vs_straight_xy_*`、`turn_vs_straight_xy_sign_*` 与 `turn_vs_straight_xy_term_*` 均来自新目录下 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：`GNSS lever x/y (28:29)` 持续释放；本轮唯一新增干预是 `GNSS_POS` 更新内的 element-wise gain scaling，只作用于 `K(l_gx, meas_y)`；`position(0:2)` 仍固定采用 `pos_gain=0.5` reference family，不涉及真值 anchor
  - 传感器窗口：仍只比较 `GNSS_POS` 更新；turn vs straight 的 5s 窗宽、速度匹配与 GNSS schedule 保持一致
  - 行为结果：
    - `x(28)`：只压 `lgx<-y_meas` 就能把 turn-window `x` 从 `-0.547 mm` 连续拉到正值，而 `same_axis_sign_match_x` 基本不变，说明 turning 下 `x` 的 weak same-axis channel 确实已经存在，真正 blocker 是 `y` 项污染
    - `y(29)`：没有随 `x` selective suppression 一起翻正，且强 suppression 下反而更差，说明 `y` 的主矛盾并不在 `x` cross-term，而仍是 same-axis `k_lgy_y * y_y` 自身方向不稳
- decision:
  - `x` 轴的主因已经从“term-source 推断”升级为“selective intervention 直接验证”：`k_lgx_y * y_y` 是当前 pure `INS/GNSS` turn-window 中阻碍 `GNSS lever x` 收敛的 causal blocker
  - 这也同时排除了“`x` 只是仍缺激励”这一解释；turning 已经给出足够的 weak decoupling，只是最终被 cross-axis routing 拉坏
  - `y` 轴没有被该干预救好，说明后续主线必须切到 `k_lgy_y * y_y` 的 same-axis sign diagnostic，而不是继续加大 `x` suppression
- next_step:
  - 对 `y(29)` 做 same-axis sign diagnostic，优先关联 `turn direction`、`innovation y` 符号、当前 `lever_y` 误差符号与 `k_lgy_y * y_y` 的 sign match；随后再考虑把已验证的 `x` selective suppression 与 `y` 的 targeted fix 组合成真正可同时修 `x/y` 的策略

### session_id: 20260319-0055-turn-y-same-axis-sign-diagnostic-r1
- timestamp: 2026-03-19 00:55 (local)
- objective: 按上一轮 `next_step` 继续把 `y(29)` 的 same-axis instability 拆到更具体层，确认它究竟来自 `innovation_y` 几何符号、`k_lgy_y` 符号分布，还是二者与 turn direction 的条件耦合。
- scope:
  - 新增独立后处理脚本 `analyze_turn_window_y_same_axis_sign_diagnostic.py`。
  - 联合读取 `output/data2_turn_window_position_gain_scale_probe/` 与 `output/data2_turn_window_lgx_from_y_gain_scale_probe/` 两套 raw `gnss_updates_*.csv`，只聚焦 `axis=y`。
  - 在 turn / straight 与 `turn_direction_sign` 条件下，显式构造 `needed_k_sign = sign(desired_dx_y * innovation_y)`，检查 `k_lgy_y` 是否匹配该符号，并输出 case / turn-direction / window 三层汇总。
- changed_files:
  - `scripts/analysis/analyze_turn_window_y_same_axis_sign_diagnostic.py`
  - `walkthrough.md`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_window_y_same_axis_sign_diagnostic.py`
  - `python scripts\analysis\analyze_turn_window_y_same_axis_sign_diagnostic.py`
- artifacts:
  - `output/data2_turn_window_y_same_axis_sign_diagnostic/{summary.md,manifest.json,y_same_axis_per_update.csv,y_same_axis_case_phase_summary.csv,y_same_axis_turn_direction_summary.csv,y_same_axis_window_summary.csv,y_same_axis_case_comparison.csv}`
- metrics:
  - overall turn:
    - `baseline`: `same_axis_sign_y=0.40`, `innovation_positive_rate=0.76`, `k_negative_rate=0.48`, `total_window_improve_y=-2.534 mm`
    - `pos_gain_0p5`: `same_axis_sign_y=0.44`, `innovation_positive_rate=0.76`, `k_negative_rate=0.52`, `total_window_improve_y=-0.708 mm`
    - `pos_gain_0p25`: `same_axis_sign_y=0.56`, `innovation_positive_rate=0.72`, `k_negative_rate=0.60`, `total_window_improve_y=+2.213 mm`
  - turn-direction split:
    - baseline / `pos_gain_0p5` positive-turn `same_axis_sign_y=0.267/0.267`，negative-turn `0.600/0.700`
    - baseline / `pos_gain_0p5` positive-turn `innovation_positive_rate` 都约 `0.667`，说明失败不主要来自 innovation 符号漂移
    - baseline / `pos_gain_0p5` positive-turn `k_negative_rate=0.467/0.467`；`pos_gain_0p25` 把它提到 `0.533`，同时 positive-turn `same_axis_sign_y` 提到 `0.600`
  - x-only selective control:
    - `joint_pos_gain_0p5_ref` 正向转弯 `same_axis_sign_y=0.267`
    - `joint_pos_gain_0p5_lgx_from_y_0p25/0p0` 也仅到 `0.333/0.333`，整体 `y total_window_improve` 仍为 `-0.835/-1.024 mm`
  - window examples from `position_gain_scale probe`:
    - positive-turn `turn_window_1/4/5` 在 baseline 下 `same_axis_sign_y=0.2/0.2/0.4`，到 `pos_gain_0p25` 变为 `0.6/0.6/0.6`
    - negative-turn `turn_window_2/3` 并非统一变好，其中 `turn_window_3` 从 `0.6` 反而降到 `0.2`，说明 `pos_gain_0.25` 不是全局最优规律，而更像在补偿一类特定 turn-direction 条件下的 gain-sign mismatch
- artifact_mtime:
  - `output/data2_turn_window_y_same_axis_sign_diagnostic/summary.md`: `2026-03-19 00:54:36` (local)
  - `output/data2_turn_window_y_same_axis_sign_diagnostic/y_same_axis_case_comparison.csv`: `2026-03-19 00:54:36` (local)
  - `output/data2_turn_window_y_same_axis_sign_diagnostic/y_same_axis_window_summary.csv`: `2026-03-19 00:54:36` (local)
- config_hash_or_mtime:
  - source probes reused from:
    - `output/data2_turn_window_position_gain_scale_probe/summary.md`: `2026-03-18 23:02:45` (local)
    - `output/data2_turn_window_lgx_from_y_gain_scale_probe/summary.md`: `2026-03-19 00:38:07` (local)
  - reference config reused from `config_data2_baseline_eskf.yaml`
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；turning 仍集中在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮不重跑 solver；而是显式复核并复用上一轮已 fresh 生成的 `position_gain_scale probe` 与 `lgx_from_y selective probe` raw `gnss_updates_*.csv`，重新生成独立 `y same-axis` 诊断目录
- observability_notes:
  - 状态块：只分析 `GNSS lever y (29)` 的 same-axis 项 `k_lgy_y * y_y`；不新增状态冻结或真值 anchor
  - 传感器窗口：仍只分析 `GNSS_POS` 更新，并沿用 turn / matched straight 的同一批时间窗
  - 行为结果：
    - `y(29)` 的失败更像 `k_lgy_y` 符号没有在正向转弯窗口里稳定匹配 `desired_dx_y * innovation_y` 的需要，而不是 innovation 自己随机翻转
    - `pos_gain_0p25` 能救 `y`，主要是把这类正向转弯窗口里的 `k_lgy_y` 符号分布往正确方向推，而不是明显改变 `innovation_y` 的符号分布
    - `x`-only selective suppression 对这些 `y` 条件统计几乎不动，因此 `y` 必须独立修，而不能指望 `x` 通道干预顺带解决
- decision:
  - `y` 轴的根因已经从“same-axis 通道本身不稳”进一步收敛成“turn-direction-conditioned 的 `k_lgy_y` 符号匹配失败”
  - 这比此前的结论更具体：`y` 不是简单需要更强全局 suppression，而是需要一个能在特定 turn direction 下稳定把 `k_lgy_y` 往所需符号推进的 targeted fix
- next_step:
  - 设计 `y` 侧 targeted fix 实验，优先方向是：在保留 `x` 参考族 `pos_gain=0.5 + lgx_from_y selective` 的前提下，尝试只对正向转弯窗口施加更强的 `position` competition mitigation，或直接为 `lgy<-y_meas` 暴露可控路径，检查能否把 `y` 的 positive-turn `same_axis_sign_y` 从 `0.267~0.333` 拉到接近 `0.6`

### session_id: 20260319-0143-turn-y-conditioned-proxy-probe-r1
- timestamp: 2026-03-19 01:43 (local)
- objective: 按 `Next Actions #1~4` 继续推进 `y(29)` 的 targeted fix：先验证在线 turn-direction proxy，再在 `omega_z` 条件下做 `position` competition mitigation，并检查它究竟是在修 `k_lgy_y` 的符号，还是只是在压低错误更新幅值。
- scope:
  - 新增 `analyze_turn_direction_proxy.py`，在 turn-window `GNSS_POS` 更新时刻对账 `POS yaw_rate`、`DIAG omega_z`、`SOL yaw-rate` 与速度曲率代理。
  - 在 solver 中新增 `fusion.gnss_pos_turn_rate_threshold_deg_s`、`fusion.gnss_pos_positive_turn_position_gain_scale`、`fusion.gnss_pos_negative_turn_position_gain_scale`，并在 `pipeline_fusion.cpp` 中基于 `IMU omega_z` 动态覆盖 `GNSS_POS` 的有效 `position gain`。
  - 新增 `run_data2_turn_window_y_turn_conditioned_position_probe.py`，把“正向强转弯时 `pos_gain: 0.5 -> 0.25`”与 `x` 侧 `lgx_from_y` reference family 组合，并补跑 `turn_vs_straight_xy`、`sign_consistency`、`y same-axis diagnostic` 与 `term-source`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/analyze_turn_direction_proxy.py`
  - `scripts/analysis/run_data2_turn_window_y_turn_conditioned_position_probe.py`
  - `walkthrough.md`
- configs:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - best combined config: `output/data2_turn_window_y_turn_conditioned_position_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25.yaml`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_direction_proxy.py scripts\analysis\run_data2_turn_window_y_turn_conditioned_position_probe.py`
  - `python scripts\analysis\analyze_turn_direction_proxy.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_y_turn_conditioned_position_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py --probe-dir output\data2_turn_window_y_turn_conditioned_position_probe`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py --probe-dir output\data2_turn_window_y_turn_conditioned_position_probe`
  - `python scripts\analysis\analyze_turn_window_y_same_axis_sign_diagnostic.py --source-probe output\data2_turn_window_y_turn_conditioned_position_probe --output-dir output\data2_turn_window_y_turn_conditioned_position_diagnostic`
  - `python scripts\analysis\analyze_turn_window_xy_term_source.py --source-probe output\data2_turn_window_y_turn_conditioned_position_probe --output-dir output\data2_turn_window_y_turn_conditioned_position_term_source`
- artifacts:
  - `output/data2_turn_direction_proxy/{summary.md,manifest.json,proxy_per_update.csv,proxy_case_summary.csv,proxy_overall_summary.csv}`
  - `output/data2_turn_window_y_turn_conditioned_position_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}`
  - `output/data2_turn_window_y_turn_conditioned_position_diagnostic/{summary.md,manifest.json,y_same_axis_{per_update,case_phase_summary,turn_direction_summary,window_summary,case_comparison}.csv}`
  - `output/data2_turn_window_y_turn_conditioned_position_term_source/{manifest.json,turn_vs_straight_xy_term_{per_update,window_summary,case_phase_summary,case_summary,summary}.{csv,md}}`
- metrics:
  - online turn proxy:
    - `imu_turn_sign` weighted best-match=`1.000`，且 direct=`1.000`、inverted=`0.000`；`POS yaw_rate` 与运行时 `omega_z` 同向，不需要符号翻转
    - `sol_yaw_turn_sign` / `sol_curvature_turn_sign` 仅为 `0.727 / 0.640`
  - turn-conditioned mitigation:
    - `joint_pos_gain_0p5_ref`: turn `x/y total_window_improve=-0.547/-0.708 mm`
    - `joint_pos_gain_0p5_pos_turn_0p25`: turn `x/y total_window_improve=+0.250/+0.958 mm`
    - `joint_pos_gain_0p5_lgx_from_y_0p25_ref`: turn `x/y total_window_improve=+1.360/-0.835 mm`
    - `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25`: turn `x/y total_window_improve=+2.590/+1.015 mm`; turn `dx_sign_match_x/y=0.640/0.560`; turn `lever_share_x/y=0.0785/0.1111`; turn `pos_share_x/y=0.8392/0.8022`
  - positive-turn window effect on `y`:
    - `joint_pos_gain_0p5_ref -> joint_pos_gain_0p5_pos_turn_0p25`: `turn_window_1/4/5` 的 `lever_y_error_improve` 从 `-6.845/+0.280/+2.817 mm` 变为 `-2.878/+3.792/+4.496 mm`
    - 带 `lgx_from_y=0.25` 时，对应从 `-7.320/-0.228/+2.449 mm` 变为 `-2.895/+3.650/+4.301 mm`
  - root-cause check for `y`:
    - 正向转弯 `same_axis_sign_match_y` 并未真正修复：`joint_pos_gain_0p5_ref -> joint_pos_gain_0p5_pos_turn_0p25` 仍为 `0.267 -> 0.267`；带 `lgx_from_y=0.25` 时 `0.333 -> 0.333`
    - 正向转弯 `k_negative_rate` 也基本不变：`0.467 -> 0.467`；带 `lgx_from_y=0.25` 时 `0.400 -> 0.400`
    - 真正变化的是 wrong-sign same-axis 幅值：正向转弯 `mean_k_lgy_y` 从 `0.004779 -> 0.001966`，带 `lgx_from_y=0.25` 时 `0.005113 -> 0.002182`；`mean_same_axis_term_y` 从 `+0.523 -> +0.081 mm`，带 `lgx_from_y=0.25` 时 `+0.563 -> +0.068 mm`
    - term-source 的 turn `|term_y|` 也从 `1.303/1.339 mm` 降到 `1.043/1.050 mm`
- artifact_mtime:
  - `output/data2_turn_direction_proxy/summary.md`: `2026-03-19 01:29:55` (local)
  - `output/data2_turn_window_y_turn_conditioned_position_probe/summary.md`: `2026-03-19 01:40:16` (local)
  - `output/data2_turn_window_y_turn_conditioned_position_probe/turn_vs_straight_xy_summary.md`: `2026-03-19 01:40:40` (local)
  - `output/data2_turn_window_y_turn_conditioned_position_probe/turn_vs_straight_xy_sign_summary.md`: `2026-03-19 01:40:43` (local)
  - `output/data2_turn_window_y_turn_conditioned_position_diagnostic/summary.md`: `2026-03-19 01:40:44` (local)
  - `output/data2_turn_window_y_turn_conditioned_position_term_source/turn_vs_straight_xy_term_summary.md`: `2026-03-19 01:40:41` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_y_turn_conditioned_position_probe/artifacts/cases/joint_pos_gain_0p5_pos_turn_0p25/config_joint_pos_gain_0p5_pos_turn_0p25.yaml`: fresh in this session
  - `output/data2_turn_window_y_turn_conditioned_position_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25.yaml`: fresh in this session
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；turning 仍集中在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮先 fresh rebuild `eskf_fusion`，再 fresh 运行 `y turn-conditioned position probe` 的 `9` 个 solver case；后续 `turn_vs_straight_xy_*`、`turn_vs_straight_xy_sign_*`、`y same-axis diagnostic` 与 `term-source` 均直接读取本轮 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：聚焦 `GNSS lever x/y (28:29)`；本轮新增的 solver 干预只作用于 `GNSS_POS` 中 `position(0:2)` 行的条件化 gain 覆盖，以及延续 `K(l_gx, meas_y)` selective suppression；未引入任何真值 anchor
  - 传感器窗口：仍只分析 `GNSS_POS` 更新；turn-direction proxy 明确固定为运行时 `IMU omega_z`，阈值使用 `|omega_z|>=8 deg/s`
  - 行为结果：
    - `x(28)`：与 `y` 条件化 mitigation 叠加后，`x` 仍保持正改善，说明此前已验证的 `lgx<-y` causal fix 可以稳定作为组合 reference family
    - `y(29)`：turn `total_window_improve` 已翻正，但 `same_axis_sign_match_y` 与 `k_negative_rate` 基本未动，说明当前条件化 `position` suppression 主要是在压低 wrong-sign same-axis 项的幅值，而不是把 `k_lgy_y` 的根本符号问题修好
    - 因而当前证据支持：`y` 的失败不属于“无激励”或“无在线 turn proxy”，而是正向转弯下 `k_lgy_y * y_y` 的有效符号/几何仍存在结构性 mismatch
- decision:
  - `turn direction` 的在线 proxy 已经闭环：后续所有 solver 级条件化实验默认使用 `IMU omega_z`
  - 当前 best combined mitigation 已出现：`pos_gain=0.5 + lgx_from_y_gain_scale=0.25 + positive-turn position gain=0.25 (|omega_z|>=8 deg/s)`，它能把 turn `x/y` 同时拉到正改善区
  - 但这仍不是根因修复；`y` 的根本问题仍是正向转弯下 `k_lgy_y` 的 same-axis sign/geometry mismatch，只是现在先被 conditional attenuation 压住了
- next_step:
  - 在保留当前 best combined reference family 的前提下，直接面向 `lgy<-y_meas` 路径做更细粒度的 solver 级干预或 Jacobian/gain 归因，优先回答“为什么正向转弯时 `k_lgy_y` 仍取不到所需符号，而 conditional position suppression 只能减小幅值、不能翻正符号”

### session_id: 20260319-1023-turn-lgy-from-y-gain-scale-probe-r1
- timestamp: 2026-03-19 10:23 (local)
- objective: 按上一轮 `next_step` 直接面向 `lgy<-y_meas` 路径继续实验：新增 `K(l_gy, meas_y)` 的 solver 级可控接口，并验证在当前 best combined reference family 上，正向转弯 selective attenuation 究竟能否真正修复 `y(29)` 的 sign mismatch。
- scope:
  - 在 solver 中新增 `fusion.gnss_pos_lgy_from_y_gain_scale` 与 `fusion.gnss_pos_positive_turn_lgy_from_y_gain_scale / negative_turn_lgy_from_y_gain_scale`，并沿用 `IMU omega_z` 与同一 turn-rate 阈值做 turn-conditioned 覆盖。
  - 新增 `run_data2_turn_window_lgy_from_y_gain_scale_probe.py`，以 `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref` 为基线，扫描正向转弯 `K(l_gy, meas_y)` 的 `0.75 / 0.5 / 0.25 / 0.0`。
  - 对新 probe fresh 产物补跑 `turn_vs_straight_xy`、`sign_consistency`、`y same-axis diagnostic` 与 `term-source`，重点复核 `turn_window_1/4/5` 与 `turn_window_2/3`。
- changed_files:
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `scripts/analysis/run_data2_turn_window_lgy_from_y_gain_scale_probe.py`
  - `walkthrough.md`
- configs:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - reference config: `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`
  - representative direct-path configs:
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p75/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p75.yaml`
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p5/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p5.yaml`
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_lgy_from_y_gain_scale_probe.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_lgy_from_y_gain_scale_probe.py`
  - `python scripts\analysis\analyze_turn_vs_straight_xy_decoupling.py --probe-dir output\data2_turn_window_lgy_from_y_gain_scale_probe`
  - `python scripts\analysis\analyze_turn_window_xy_sign_consistency.py --probe-dir output\data2_turn_window_lgy_from_y_gain_scale_probe`
  - `python scripts\analysis\analyze_turn_window_y_same_axis_sign_diagnostic.py --source-probe output\data2_turn_window_lgy_from_y_gain_scale_probe --output-dir output\data2_turn_window_lgy_from_y_gain_scale_diagnostic`
  - `python scripts\analysis\analyze_turn_window_xy_term_source.py --source-probe output\data2_turn_window_lgy_from_y_gain_scale_probe --output-dir output\data2_turn_window_lgy_from_y_gain_scale_term_source`
- artifacts:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/{summary.md,manifest.json,turn_windows.csv,turn_window_update_breakdown.csv,turn_window_summary.csv,case_update_summary.csv,diag_turn_window_std_summary.csv,turn_vs_straight_xy_{window_summary,case_summary,summary,manifest}.{csv,md,json},turn_vs_straight_xy_sign_{per_update,window_summary,case_phase_summary,case_summary,summary,manifest}.{csv,md,json}}`
  - `output/data2_turn_window_lgy_from_y_gain_scale_diagnostic/{summary.md,manifest.json,y_same_axis_{per_update,case_phase_summary,turn_direction_summary,window_summary,case_comparison}.csv}`
  - `output/data2_turn_window_lgy_from_y_gain_scale_term_source/{manifest.json,turn_vs_straight_xy_term_{per_update,window_summary,case_phase_summary,case_summary,summary}.{csv,md}}`
- metrics:
  - direct-path scan on top of current best combined reference:
    - `joint_pos_gain_0p5_lgx_from_y_0p25_ref`: turn `x/y total_window_improve=+1.360/-0.835 mm`
    - `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref`: turn `x/y total_window_improve=+2.590/+1.015 mm`
    - `+ pos_turn_lgy_from_y=0.75`: turn `x/y total_window_improve=+2.536/+1.042 mm`
    - `+ pos_turn_lgy_from_y=0.5`: turn `x/y total_window_improve=+2.476/+1.045 mm`
    - `+ pos_turn_lgy_from_y=0.25`: turn `x/y total_window_improve=+2.411/+1.025 mm`
    - `+ pos_turn_lgy_from_y=0.0`: turn `x/y total_window_improve=+2.338/+0.979 mm`
  - same-axis sign / route-level effect on `y`:
    - positive-turn `same_axis_sign_match_y`: reference `0.333`; only `0.75` 提到 `0.400`；`0.5/0.25` 仍为 `0.333`；`0.0` 因 same-axis path 被移除而变为 `NaN`
    - positive-turn `k_negative_rate`: reference `0.400`；`0.75` 提到 `0.467`；`0.5/0.25` 仍为 `0.400`；`0.0` 归零
    - term-source 的 turn `|term_y|` 从 reference `1.050 mm` 依次降到 `0.939/0.796/0.590/0.316 mm`
    - turn `dx_sign_match_y` 从 reference `0.560` 升到 `0.560/0.600/0.680/0.720`
  - window-level focus:
    - positive-turn `turn_window_1` 的 `lever_y_error_improve` 从 reference `-2.895 mm` 改到 `-2.066/-1.353/-0.840/-0.582 mm`
    - positive-turn `turn_window_4/5` 保持正改善，但会随更强 suppression 逐步回落：`window_4=+3.650→+3.454/+3.349/+3.462/+3.893 mm`，`window_5=+4.301→+3.849/+3.373/+2.853/+2.231 mm`
    - negative-turn `turn_window_2` 基本稳定在 `+4.425→+4.377/+4.321/+4.263/+4.235 mm`；`turn_window_3` 则从 `-4.403 mm` 继续恶化到 `-4.405/-4.465/-4.613/-4.882 mm`
- artifact_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/summary.md`: `2026-03-19 10:20:52` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/case_update_summary.csv`: `2026-03-19 10:20:52` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_diagnostic/summary.md`: `2026-03-19 10:21:32` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_diagnostic/y_same_axis_case_comparison.csv`: `2026-03-19 10:21:32` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_term_source/turn_vs_straight_xy_term_summary.md`: `2026-03-19 10:21:27` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p75/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p75.yaml`: `2026-03-19 10:15:27` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p5/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p5.yaml`: `2026-03-19 10:17:14` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`: `2026-03-19 10:19:34` (local)
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows 与对应 matched straight windows；turning 仍集中在 `t≈528123~530275 s`
- result_freshness_check:
  - 本轮先 fresh rebuild `eskf_fusion`，再 fresh 运行 `lgy_from_y gain-scale probe` 的 `6` 个 solver case；后续 `turn_vs_straight_xy_*`、`turn_vs_straight_xy_sign_*`、`y same-axis diagnostic` 与 `term-source` 均直接读取本轮 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：本轮主观察对象是 `GNSS lever y (29)` 的 same-axis 路径 `K(l_gy, meas_y)`；`GNSS lever x (28)` 继续固定在已验证有效的 `lgx_from_y_gain_scale=0.25` 参考族
  - 传感器窗口：仍只分析 `GNSS_POS` 更新；turn-conditioned control 统一使用运行时 `IMU omega_z`，阈值 `|omega_z|>=8 deg/s`
  - 行为结果：
    - `y(29)`：direct `lgy<-y` attenuation 的收益主要集中在正向转弯窗口，并且首先体现在 wrong-sign same-axis 幅值被继续压小，而不是 `same_axis_sign_match_y` 被稳定修好
    - `x(28)`：`y` 路径 attenuation 越强，`x` 的 turn `total_window_improve` 越从 `+2.590` 回落到 `+2.338 mm`，说明该干预不是免费收益
    - 负向转弯：`turn_window_2` 基本不动，而 `turn_window_3` 会随更强 suppression 稳定恶化，说明 `y` 的 unresolved 机理并不只存在于正向窗口的“错误 same-axis 幅值过大”
- decision:
  - `K(l_gy, meas_y)` 的 solver 级可控接口已经补齐，但 fresh evidence 表明：simple attenuation 只能提供边际 `y` 改善，不能替换当前 best combined reference，也不能构成根因修复
  - 当前官方 `x/y` 联合 reference 仍保持 `pos_gain=0.5 + lgx_from_y_gain_scale=0.25 + positive-turn position gain=0.25 (|omega_z|>=8 deg/s)`；新的 `positive-turn lgy_from_y` 扫描保留为归因证据，而不是新默认 case
  - 下一优先级不再是继续扫 `lgy_from_y` attenuation 常数，而是直接对 `k_lgy_y` 的 numerator / covariance / geometry 生成机制做正负转弯分解
- next_step:
  - 在 `turn_window_1/4/5` 与 `turn_window_2/3` 上补 `K(l_gy, meas_y)` 生成机制归因，优先输出或离线重建 `P(l_gy,*) H_y^T` 的 numerator 组成、`H_y` 几何项与 `pred_ant_ecef -> innovation_y` 链，解释为什么正向转弯下 `k_lgy_y` 仍难以取到所需符号，以及为什么 `turn_window_3` 会在 direct attenuation 下继续变坏

### session_id: 20260319-1112-turn-lgy-lever-axis-focus-r1
- timestamp: 2026-03-19 11:12 (local)
- objective: 按当前最高优先级继续追 `k_lgy_y` 的 raw 生成机制，在已知 `num_y(pos)` 与 `num_y(lever)` 近抵消的基础上，把 `lever` 自块再拆成 `lgx/lgy/lgz` 三轴，并用 focused rerun 验证正向转弯 family split 是否来自 `gnss_lever` 自协方差翻号。
- scope:
  - 在 `GNSS_POS` debug 导出中新增 `prior_cov_gnss_lever_mat` 与 `num_gnss_lever_y_from_lgx/lgy/lgz_vec`。
  - 给 `run_data2_turn_window_lgy_from_y_gain_scale_probe.py` 与 `analyze_turn_window_lgy_numerator_geometry.py` 增加 `--exp-id`，并让 probe 支持 `--case-id` focused rerun。
  - 只对 `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref` 与 `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0` 两个 case fresh 重跑，然后做 lever-axis focused analysis。
- changed_files:
  - `src/app/diagnostics.cpp`
  - `scripts/analysis/run_data2_turn_window_lgy_from_y_gain_scale_probe.py`
  - `scripts/analysis/analyze_turn_window_lgy_numerator_geometry.py`
  - `walkthrough.md`
- configs:
  - source config reused from `output/data2_ins_gnss_lever_noise_coupling_sweep/artifacts/cases/eskf_rstd1_qlever1/config_eskf_rstd1_qlever1.yaml`
  - focused configs:
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`
- commands:
  - `python -m py_compile scripts\analysis\run_data2_turn_window_lgy_from_y_gain_scale_probe.py scripts\analysis\analyze_turn_window_lgy_numerator_geometry.py`
  - `cmake --build build --config Release --target eskf_fusion`
  - `python scripts\analysis\run_data2_turn_window_lgy_from_y_gain_scale_probe.py --exp-id EXP-20260319-data2-turn-window-lgy-lever-axis-focus-r1 --output-dir output\data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1 --case-id joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref --case-id joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0`
  - `python scripts\analysis\analyze_turn_window_lgy_numerator_geometry.py --exp-id EXP-20260319-data2-turn-window-lgy-lever-axis-focus-r1 --source-probe output\data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1 --output-dir output\data2_turn_window_lgy_lever_axis_focus_analysis --case-id joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref --case-id joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0`
- artifacts:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/{summary.md,manifest.json,case_update_summary.csv,turn_window_summary.csv,turn_window_update_breakdown.csv}`
  - `output/data2_turn_window_lgy_lever_axis_focus_analysis/{summary.md,manifest.json,lgy_numerator_geometry_{per_update,window_summary,focus_comparison}.csv}`
- metrics:
  - focused probe reproduction:
    - reference turn `x/y total_window_improve=+2.590/+1.015 mm`
    - `positive-turn lgy_from_y=0` turn `x/y total_window_improve=+2.338/+0.979 mm`
  - lever-axis decomposition:
    - 所有 focus windows 与两个 cases 中，dominant `lever` axis 始终是 `lgy`；`num_y(lever_lgy)=±0.002041~0.006678`，`num_y(lever_lgx)=1.3e-05~1.7e-04`，`num_y(lever_lgz)≈1e-07`
    - `cov(lgy,lgy)` 在所有 focus windows 中始终为正 `0.004433~0.010421`
    - `h_y_lever_y` 的符号决定 `lever` 自块符号：`turn_window_1/5=-0.4917/-0.6354 -> num_y(lever_lgy)<0`；`turn_window_2/3/4=+0.4584/+0.7715/+0.6411 -> num_y(lever_lgy)>0`
    - 正向 family split 具体化为：
      - `turn_window_1/5`: `num_y(pos)=+0.003082/+0.005550`、`num_y(lever_lgy)=-0.002506/-0.005183`
      - `turn_window_4`: `num_y(pos)=-0.007100`、`num_y(lever_lgy)=+0.006598`
  - geometry-chain gap:
    - `pred_ant_ecef -> innovation` 的 `innovation_recon_error_norm_max` 仍为 `0.877~1.432 m`
- artifact_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/summary.md`: `2026-03-19 11:11:41` (local)
  - `output/data2_turn_window_lgy_lever_axis_focus_analysis/summary.md`: `2026-03-19 11:12:01` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`: `2026-03-19 11:09:19` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`: `2026-03-19 11:10:31` (local)
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows；重点对比 `turn_window_1/4/5` 与 `turn_window_2/3`
- result_freshness_check:
  - 本轮先 fresh rebuild `eskf_fusion`，再 fresh 运行 `2` 个 solver cases；lever-axis analysis 直接读取本轮 freshly generated `gnss_updates_*.csv`
- observability_notes:
  - 状态块：主观察对象是 `GNSS lever y (29)`；本轮把 `num_y(lever)` 再拆成 `lgx/lgy/lgz` 三轴，并记录 `prior_cov_gnss_lever_mat`
  - 传感器窗口：仍只分析 `GNSS_POS` top-5 turn windows；`x(28)` 继续固定在已验证有效的 `lgx_from_y_gain_scale=0.25` reference family
  - 行为结果：
    - `y(29)`：`lever` 自块并不存在“`lgx/lgz` 混进来把符号拉坏”的主导效应，真正的自块主项始终是 `lgy` 自轴
    - `y(29)`：`lever` 自块的正负主要由 `H_y(lever_y)` 决定，而不是 `cov(lgy,lgy)` 反号；因此正向转弯内部的 `turn_window_1/5` 与 `turn_window_4` 确实属于不同 geometry family
    - 当前 unresolved 部分收敛为：`P(l_gy,p_y)` 与 `H_y(lever_y)` 两条主路径是如何在进入窗口前共同改号的，以及为何 earlier positive-turn interventions 会把 `turn_window_3` 历史改坏
- decision:
  - `k_lgy_y` 的 lever 自块归因已经闭环到 axis level：后续不再把 `lever` 内部 axis-mixing 当主嫌疑
  - 下一优先级正式转为 `cov(lgy,p_y)` / `cov(lgy,lgy)` 的 history tracing 与 `turn_window_3` 特化对照；`lgy<-y` attenuation 常数扫描到此结束
  - `pred_ant_ecef -> innovation` 几何链仍未满足主证据标准，需要单独修 strict reconstruction 口径
- next_step:
  - 对 `turn_window_1/4/5` 与 `turn_window_2/3` 追进入窗口前的 `cov(lgy,p_y)`、`cov(lgy,lgy)` 与 `h_y_lever_y` history，优先解释为什么正向 family 会分裂成 `1/5` 与 `4`，并单独比较 reference vs `lgy_from_y=0` 在 `turn_window_3` 之前的 covariance/state history

### session_id: 20260319-1432-turn-lgy-history-r1
- timestamp: 2026-03-19 14:32 (local)
- objective: 按 `Next Actions` 的最高优先级继续实验：不再改 solver，而是直接复用 focused rerun 的 fresh `gnss_updates_*.csv`，补 `cov(lgy,p_y)` / `cov(lgy,lgy)` / `h_y_lever_y` / `lever_before_y` 的 pre-window history，解释正向 family split 的建立时刻，并单独检查 `turn_window_3` 为何会在 `positive-turn lgy_from_y=0` 下继续恶化。
- scope:
  - 新增 `scripts/analysis/analyze_turn_window_lgy_history.py`，直接读取 `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/` 的 focused rerun 产物，并从 probe 自带的 `turn_window_update_breakdown.csv` 回填 `yaw_rate_deg_s`。
  - 统一对 top-5 turn windows 保留 `20 s` pre-window history；输出 per-update、window summary、family summary 与 `turn_window_3` 双 case 差分表。
  - 不做新的 solver rerun；本轮只消费上一轮 `20260319-1112` 已 fresh 生成的 `gnss_updates_*.csv`。
- changed_files:
  - `scripts/analysis/analyze_turn_window_lgy_history.py`
  - `walkthrough.md`
- configs:
  - source focused configs reused:
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_window_lgy_history.py`
  - `python scripts\analysis\analyze_turn_window_lgy_history.py`
- artifacts:
  - `output/data2_turn_window_lgy_history_analysis/{summary.md,manifest.json,lgy_history_per_update.csv,lgy_history_window_summary.csv,lgy_history_family_summary.csv,turn_window_3_case_diff.csv}`
- metrics:
  - family-level sign pattern:
    - `positive_family_1_5`: pre/in-window `cov(lgy,py)>0 rate=0.88/0.90`，`h_y_lever_y>0 rate=0.12/0.10`
    - `positive_family_4`: pre/in-window `cov(lgy,py)>0 rate=0.10/0.00`，`h_y_lever_y>0 rate=0.95/1.00`
    - `negative_family_2_3`: pre/in-window `cov(lgy,py)>0 rate=0.40/0.10`，`h_y_lever_y>0 rate=0.65/0.90`
  - positive-turn family split timing:
    - `turn_window_5`: entry `cov(lgy,py)=+0.000990`、`h_y_lever_y=-0.213617`，窗口末 `cov(lgy,py)=+0.009204`、`h_y_lever_y=-0.968776`
    - `turn_window_4`: entry `cov(lgy,py)=-0.001084`、`h_y_lever_y=+0.230638`，窗口末 `cov(lgy,py)=-0.011719`、`h_y_lever_y=+0.971723`
    - `turn_window_1` 是 transition case：entry `cov(lgy,py)=-0.000180`、`h_y_lever_y=+0.061031`，窗口末已变为 `+0.006624/-0.977895`
  - `turn_window_3` reference vs `positive-turn lgy_from_y=0` carry-over:
    - last pre-window update 已出现 `Δlever_before_y=-2.360 mm`、`Δcov(lgy,py)=-0.000361`、`Δcov(lgy,lgy)=-0.000097`、`Δh_y_lever_y=+0.000022`
    - entry update `Δraw_k_lgy_y=-0.001692`、`Δdx_gnss_lever_y=+0.007 mm`
    - window metric 从 reference `lever_y_error_improve=-4.403 mm` 进一步恶化到 `-4.882 mm`
- artifact_mtime:
  - `output/data2_turn_window_lgy_history_analysis/summary.md`: `2026-03-19 14:31:54` (local)
  - `output/data2_turn_window_lgy_history_analysis/turn_window_3_case_diff.csv`: `2026-03-19 14:31:54` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`: `2026-03-19 11:09:19` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`: `2026-03-19 11:10:31` (local)
- dataset_time_window:
  - 继续使用 `data2` 全程 GNSS 有效窗口上的 top-5 turn windows；每个窗口额外保留 `20 s` pre-window GNSS_POS history
- result_freshness_check:
  - 本轮没有新的 solver rerun；分析直接消费上一轮 `20260319-1112` 已 fresh 生成、mtime 为 `2026-03-19 11:09~11:11` 的 `gnss_updates_*.csv`，并在 `14:31~14:32` 生成新的 history artifacts
- observability_notes:
  - 状态块：主观察对象仍是 `GNSS lever y (29)`，但本轮主证据从单窗 numerator/geometry 均值推进到 entering-history 级的 `P(l_gy,p_y)`、`P(l_gy,l_gy)` 与 `H_y(lever_y)` 时序
  - 传感器窗口：仍只分析 `GNSS_POS` top-5 turn windows；turn-direction 继续统一使用运行时 `IMU omega_z`
  - 行为结果：
    - `y(29)`：`cov(lgy,lgy)` 依旧不是 family split 的主 discriminant；真正区分 `positive_family_1_5` 与 `positive_family_4` 的是 `cov(lgy,p_y)` 与 `h_y_lever_y` 的符号组合
    - `y(29)`：`turn_window_1` 不是从入窗起就稳定落在 `1/5` family，而是在窗口内快速从正几何翻到负几何；`turn_window_5` 与 `turn_window_4` 则分别表现为稳定负/稳定正的 geometry family
    - `turn_window_3`：`positive-turn lgy_from_y=0` 的恶化在入窗前就已建立，而 `h_y_lever_y` 几乎不变，说明主因更像 earlier state/covariance carry-over，而不是该窗口内的局部几何翻号
- decision:
  - `positive-turn` 内部的 family split 已从“窗口均值结论”细化到时序层：`window_5` 是稳定的 `(+cov_py, -h_y)` family，`window_4` 是稳定的 `(-cov_py, +h_y)` family，`window_1` 则是在窗口内完成向 `1/5` family 的翻转
  - `turn_window_3` 在 `positive-turn lgy_from_y=0` 下继续恶化，已经可以排除“该窗口直接被 positive-turn scaling 改坏”的更弱解释；下一步应追更早的 divergence source
  - `pred_ant_ecef -> innovation` 仍未做 strict cross-check，本轮不把几何重构链升级为主证据
- next_step:
  - 对 `turn_window_1` 与 `turn_window_4/5` 做 paired history comparison，定位 `window_1` 的 sign flip 是从哪几个 updates 开始建立；并把 `turn_window_3` 的 carry-over audit 从当前 `20 s` 窗口继续向前追到首个分叉的 `GNSS_POS` update

### session_id: 20260319-1449-turn-lgy-transition-carryover-r1
- timestamp: 2026-03-19 14:49 (local)
- objective: 按最新 `Next Actions` 继续下钻：把 `turn_window_1/4/5` 的 history 压缩成 stable sign-family 路径，并把 `reference` vs `positive-turn lgy_from_y=0` 的差分扩展到全程 `GNSS_POS`，定位 `turn_window_3` carry-over 的最早起点。
- scope:
  - 新增 `scripts/analysis/analyze_turn_window_lgy_transition_carryover.py`，复用 `output/data2_turn_window_lgy_history_analysis/lgy_history_per_update.csv` 与 focused rerun 的 raw `gnss_updates_*.csv`。
  - 对 `turn_window_1/4/5` 输出 stable family segment 与 pair comparison。
  - 对两 case 的全程 `GNSS_POS` 构造 per-update diff，抽取 `first_control_divergence`、`first_state_divergence` 以及 `window_4 -> window_5 -> window_3` checkpoints。
- changed_files:
  - `scripts/analysis/analyze_turn_window_lgy_transition_carryover.py`
  - `walkthrough.md`
- configs:
  - source focused configs reused:
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_window_lgy_transition_carryover.py`
  - `python scripts\analysis\analyze_turn_window_lgy_transition_carryover.py`
- artifacts:
  - `output/data2_turn_window_lgy_transition_carryover_analysis/{summary.md,manifest.json,positive_turn_family_segments.csv,positive_turn_pair_comparison.csv,full_case_diff_per_update.csv,carryover_checkpoint_summary.csv}`
- metrics:
  - positive-turn stable family paths:
    - `turn_window_1: +-(16) -> ++(1) -> -+(4) -> +-(4)`
    - `turn_window_4: -+(6) -> transient(3) -> -+(10) -> transient(1) -> -+(5)`
    - `turn_window_5: +-(4) -> transient(5) -> +-(8) -> --(1) -> transient(1) -> +-(6)`
  - `window_1` transition checkpoints:
    - first stable `h_y` flip at `529531 s` (`pre_window`)
    - first stable `cov(lgy,p_y)` flip at `529532 s` (`pre_window`)
    - entry/second/last in-window family=`-+ / +- / +-`
  - carry-over chain:
    - `first_control_divergence` at `turn_window_4_entry @ 528121 s`: `Δpost_k=-0.074382`，但 `Δlever_before_y/Δcov=0`
    - `first_state_divergence` at `528122 s`: `Δlever_before_y=+3.777 mm`、`Δcov(lgy,p_y)=+0.001008`、`Δcov(lgy,lgy)=+1.022e-4`
    - `turn_window_4_last`: `dlever=+1.922 mm`、`dcov_py=-0.002641`
    - `turn_window_5_entry/last`: `dlever=-0.949/+1.811 mm`、`dcov_py=-2.65e-4/+0.002179`
    - `turn_window_3_last_pre/entry`: `dlever=-2.360/-2.375 mm`、`dcov_py=-3.61e-4/-3.53e-4`
- artifact_mtime:
  - `output/data2_turn_window_lgy_transition_carryover_analysis/summary.md`: `2026-03-19 14:48:57` (local)
  - `output/data2_turn_window_lgy_transition_carryover_analysis/carryover_checkpoint_summary.csv`: `2026-03-19 14:48:57` (local)
  - `output/data2_turn_window_lgy_transition_carryover_analysis/positive_turn_pair_comparison.csv`: `2026-03-19 14:48:57` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`: `2026-03-19 11:09:19` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`: `2026-03-19 11:10:31` (local)
- dataset_time_window:
  - positive-turn family path 继续只看 top-5 turn windows 中的 `turn_window_1/4/5`
  - carry-over diff 则扩展到 focused rerun 的全程 `GNSS_POS` updates，并在 `turn_window_4 -> turn_window_5 -> turn_window_3` 上抽 checkpoint
- result_freshness_check:
  - 本轮没有新的 solver rerun；分析直接消费上一轮 `20260319-1112` 已 fresh 生成的 `gnss_updates_*.csv` 与 `20260319-1432` 新生成的 history csv，并在 `14:48~14:49` 生成新的 transition/carry-over artifacts
- observability_notes:
  - 状态块：仍聚焦 `GNSS lever y (29)`；本轮把“可观测性 family”显式编码为 `sign(cov(lgy,p_y), h_y_lever_y)`
  - 传感器窗口：family path 只看 `GNSS_POS` turn windows；carry-over diff 看全程 `GNSS_POS`
  - 行为结果：
    - `y(29)`：`window_4` 的 stable family 本质上始终是 `(-cov_py,+h)`，`window_5` 则基本始终是 `(+cov_py,-h)`；`window_1` 的复杂性来自 pre-window / in-window 之间多次跨 family 切换，而不是单一 steady family
    - `y(29)`：`turn_window_3` 的 carry-over 现在可追到 `turn_window_4_entry` 的 control divergence 与 `528122 s` 的 first state divergence，说明该窗口的恶化确实是 earlier positive-turn intervention 的长链结果
- decision:
  - `window_1` 的关键研究对象已从“这窗为什么最终落到 `(+,-)`”进一步收紧为 `529531/529532/529536 s` 三个 sign-transition updates
  - `turn_window_3` 的 carry-over 起点已定位到 `turn_window_4`，后续不再把它笼统记为“某个 earlier history”
  - 在把这两条链接到同一 full-history 时间轴之前，不新增新的 solver gain-scale 扫描
- next_step:
  - 对 `window_1` 的 `529531/529532/529536 s` 三个关键 updates 继续追 numerator 分块、covariance 更新来源与 preceding GNSS_POS context；并沿 `528121 -> 528122 -> 528608 -> 528722 s` 这条链继续查哪些 non-window / window updates 让 `turn_window_3` 的分叉存活到入窗

### session_id: 20260319-1503-turn-lgy-key-update-audit-r1
- timestamp: 2026-03-19 15:03 (local)
- objective: 按最新最高优先级继续细化 `y(29)` 的 update 级机理：一方面把 `window_1` 的 `529531/529532/529536 s` 三次翻转从“路径描述”压到“每次是谁在翻”的 driver audit；另一方面给 `turn_window_4 -> window_5 -> window_3` 的 carry-over 链做 interval-level top-update ranking，找出具体哪些 non-window / window `GNSS_POS` updates 在续命这条分叉。
- scope:
  - 新增 `scripts/analysis/analyze_turn_window_lgy_key_update_audit.py`，直接复用已有的 `lgy_history_per_update.csv`、`full_case_diff_per_update.csv` 与 focused rerun 原始 `gnss_updates_*.csv`。
  - 对 `window_1` 输出本地 context 表与三次 target update summary，记录 family、driver class、`num_y(pos/att/lever)`、`cov(lgy,p_y)`、`h_y_lever_y`、`raw_k` 与 same-axis term。
  - 对 carry-over 链按 `528121 -> 528122 -> 528608 -> 528722 s` 自动切区间，并为 `dlever/dcov_py/dvar_lgy/dpost_k/draw_k` 输出 top-step reinforcement ranking。
- changed_files:
  - `scripts/analysis/analyze_turn_window_lgy_key_update_audit.py`
  - `walkthrough.md`
- configs:
  - source focused configs reused:
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`
    - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`
- commands:
  - `python -m py_compile scripts\analysis\analyze_turn_window_lgy_key_update_audit.py`
  - `python scripts\analysis\analyze_turn_window_lgy_key_update_audit.py`
- artifacts:
  - `output/data2_turn_window_lgy_key_update_audit/{summary.md,manifest.json,window1_transition_context.csv,window1_transition_key_updates.csv,carryover_interval_summary.csv,carryover_top_updates.csv}`
- metrics:
  - `window_1` 三次关键翻转已分型：
    - `529531 s`: `geometry_only`，`Δh_y_lever_y=+0.181968`，`cov(lgy,p_y)` 仍为 `+0.000190`，`raw_k_lgy_y=+0.047816`，但 `innovation_y=-7.19e-4`，same-axis term 仅 `-0.034 mm`
    - `529532 s`: `covariance_only`，`Δcov(lgy,p_y)=-0.001908`，`h_y_lever_y` 继续增到 `+0.325435`，`raw_k_lgy_y=-0.007441`，总 `dx_lgy=+0.579 mm`
    - `529536 s`: `joint_covariance_and_geometry`，`Δcov(lgy,p_y)=+0.001445` 与 `Δh_y_lever_y=-0.214149` 同时发生，`raw_k_lgy_y=+0.012265`，same-axis term=`+1.014 mm`
  - carry-over 生存链的 top reinforcement 不再只落在 turn windows：
    - `state_divergence -> window_5_entry` 区间：non-window `528127 s` 给出 `step Δcov(lgy,p_y)=+0.004773` 与 `step Δpost_k=+0.097138`，non-window `528128 s` 给出 `step Δlever_before_y=-2.565 mm`，non-window `528158 s` 再给 `step Δcov(lgy,p_y)=-0.002103`
    - `window_5_entry -> window_3_entry` 区间：`turn_window_5 @ 528609/528610 s` 提供 `step Δlever_before_y=+3.028/+1.519 mm`，non-window `528614 s` 提供 `step Δcov(lgy,p_y)=-0.004151` 与 `step Δpost_k=-0.083641`，non-window `528615 s` 再给 `step Δlever_before_y=-2.237 mm`
  - interval end-state：
    - 到 `window_5_entry @ 528608 s` 仍有 `dlever=-0.949 mm`、`dcov_py=-2.65e-4`、`dpost_k=+0.052328`
    - 到 `window_3_entry @ 528722 s` 仍有 `dlever=-2.375 mm`、`dcov_py=-3.53e-4`、`dpost_k=-0.001692`
- artifact_mtime:
  - `output/data2_turn_window_lgy_key_update_audit/summary.md`: `2026-03-19 15:02:24` (local)
  - `output/data2_turn_window_lgy_key_update_audit/window1_transition_key_updates.csv`: `2026-03-19 15:02:24` (local)
  - `output/data2_turn_window_lgy_key_update_audit/carryover_top_updates.csv`: `2026-03-19 15:02:24` (local)
- config_hash_or_mtime:
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_ref.yaml`: `2026-03-19 11:09:19` (local)
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe_focus_r1/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0/config_joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25_lgy_from_y_0p0.yaml`: `2026-03-19 11:10:31` (local)
- dataset_time_window:
  - `window_1` 本地 context 聚焦 `529529~529537 s`
  - carry-over survival audit 聚焦 `528121~528722 s` 全程 `GNSS_POS` chain，并按 `control_divergence -> state_divergence -> window_5_entry -> window_3_entry` 切区间
- result_freshness_check:
  - 本轮没有新的 solver rerun；分析直接消费 `20260319-1112` 已 fresh 生成的 focused rerun 原始 `gnss_updates_*.csv`、`20260319-1432` 的 history csv 与 `20260319-1449` 的 diff csv，并在 `15:02` 生成新的 key-update audit artifacts
- observability_notes:
  - 状态块：仍聚焦 `GNSS lever y (29)`；本轮把 `window_1` 的 family 转换进一步拆成 `geometry-only / covariance-only / joint` 三类 update driver
  - 传感器窗口：`window_1` 部分只看 `GNSS_POS` 的局部 update context；carry-over 部分则覆盖 `turn_window_4`、`turn_window_5`、`turn_window_3` 及其间的非窗口 `GNSS_POS` updates
  - 行为结果：
    - `y(29)`：`window_1` 的失败不是单一 steady family，而是几何先翻、协方差再翻、入窗后两者共同翻回
    - `y(29)`：`turn_window_3` 的 carry-over 生存链由窗口内外 mixed `GNSS_POS` updates 共同维持；`window_4` 只是起点，不是唯一续命点
- decision:
  - `window_1` 的下一步不再是泛泛“补更多 history”，而是对 `529531/529532/529536 s` 各自上游的 covariance 来源与 preceding GNSS_POS context 做 full-history 回放
  - `turn_window_3` 的下一步也不再只盯 `528121/528122/528608/528722 s` 四个 checkpoint，而是优先追已经排名出来的 `528127/528128/528158/528609/528610/528614/528615 s`
  - 在这些 ranked updates 的来源没闭环之前，不新增新的 solver gain-scale 扫描
- next_step:
  - 回到 full `GNSS_POS` 历史，分别追 `529531/529532/529536 s` 的前一到两次更新是如何把 `h_y_lever_y`、`cov(lgy,p_y)` 与 `num_y(pos/lever)` 推到当前翻号边界；并沿 `528127/528128/528158/528609/528610/528614/528615 s` 继续查明它们为何能维持 `turn_window_3` 的分叉

### session_id: 20260319-1515-continue-need-judgement-r1
- timestamp: 2026-03-19 15:15 (local)
- objective: 基于当前 `y(29)` 支线已有证据，判断是否还有继续追查的必要，并把“何时可以停、何时值得继续”的门槛写清楚，避免后续无条件深挖。
- scope:
  - 复核 `walkthrough.md` 当前 `Next Actions` 与最近几轮 `lgy` 支线证据。
  - 不新增 solver rerun，不新增分析脚本，只做阶段性 stop/continue judgement。
- changed_files:
  - `walkthrough.md`
- configs:
  - 无新增配置；沿用当前 official reference 与 focused rerun context
- commands:
  - `Get-Content walkthrough.md`（current snapshot + next actions）
- artifacts:
  - 无新增实验产物
- metrics:
  - 当前 `y(29)` 支线已具备的阶段性闭环：
    - official reference 已稳定在 `pos_gain=0.5 + lgx_from_y_gain_scale=0.25 + positive-turn position gain=0.25`
    - `cov(lgy,lgy)` 非主因、`lever` 内部 axis-mixing 非主因，这两条更弱解释已被排除
    - `window_1` 已细化到 `529531/529532/529536 s` 三类 driver：`geometry_only / covariance_only / joint_covariance_and_geometry`
    - `turn_window_3` 的恶化已从笼统 carry-over 收紧到 mixed `GNSS_POS` survival chain，重点 reinforcement updates 已排名到 `528127/528128/528158/528609/528610/528614/528615 s`
  - 当前仍未闭环的部分：
    - `window_1` 三次关键翻转的更上游 covariance 来源尚未回放到 full-history
    - `turn_window_3` 生存链中这些 ranked updates 为什么能持续维持分叉，仍未追到更底层实现/状态来源
    - `pred_ant_ecef -> innovation` strict reconstruction 仍未达主证据标准
- artifact_mtime:
  - 无新增 artifacts
- config_hash_or_mtime:
  - 无新增 config 引用
- dataset_time_window:
  - judgement 基于当前 `data2` pure `INS/GNSS`、turn-window `GNSS_POS` 支线的既有 evidence
- result_freshness_check:
  - 本轮没有新运行；判断只基于 `2026-03-19 11:09~15:02` 期间的 fresh focused rerun 与其派生分析 artifacts
- observability_notes:
  - 状态块：主判断对象仍是 `GNSS lever y (29)`
  - 行为结果：
    - 若目标只是“锁定当前 best reference + 给出可 defend 的归因总结”，当前证据已经足够阶段性收口
    - 若目标是“做 solver 级根因修复 / 论文级因果闭环”，当前证据仍差最后一层 full-history source attribution，不宜现在停
- decision:
  - 结论不是“必须继续”或“可以彻底停止”二选一，而是分目标处理：
    - 对工程决策和阶段汇报而言，这条支线已经可以暂停，先不继续深挖
    - 只有当目标明确是根因修复或形成更强发表级证据时，才值得继续追 `window_1` 与 `turn_window_3` 的 full-history source attribution
  - 因此默认建议是：把 `y(29)` 深挖从“必做”降为“条件触发的高优先级备选”
- next_step:
  - 若下一阶段目标偏交付/阶段总结，则保留当前结论并暂停 `y(29)` 深挖；若目标偏根因修复，则从 `529531/529532/529536 s` 与 `528127/528128/528158/528609/528610/528614/528615 s` 继续做 full-history source attribution

### session_id: 20260319-1540-gnss-pos-gain-semantics-clarify-r1
- timestamp: 2026-03-19 15:40 (local)
- objective: 回答“当前最优方案如何实现，以及 `position_gain_scale`/交叉通道 gain 是否属于噪声参数”的实现级问题，避免后续把 gain-scale 与 `Q/R` 调参混用。
- scope:
  - 复核 `walkthrough.md` 当前 official reference。
  - 对齐 `turn-conditioned position probe`、`lgy-from-y gain-scale probe` 的 case summary 与源码实现。
  - 不新增 solver rerun，不新增分析脚本，只做代码级语义澄清。
- changed_files:
  - `walkthrough.md`
- configs:
  - current best reference family:
    - `fusion.gnss_pos_position_gain_scale=0.5`
    - `fusion.gnss_pos_lgx_from_y_gain_scale=0.25`
    - `fusion.gnss_pos_turn_rate_threshold_deg_s=8.0`
    - `fusion.gnss_pos_positive_turn_position_gain_scale=0.25`
  - optional attribution-only switch:
    - `fusion.gnss_pos_positive_turn_lgy_from_y_gain_scale`（仅作 `y(29)` same-axis 归因扫描，不替换 official reference）
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content src\app\pipeline_fusion.cpp`
  - `Get-Content include\app\fusion.h`
  - `Get-Content src\core\eskf_engine.cpp`
  - `Get-Content src\app\config.cpp`
  - `Get-Content output\data2_turn_window_y_turn_conditioned_position_probe\summary.md`
  - `Get-Content output\data2_turn_window_lgy_from_y_gain_scale_probe\summary.md`
- artifacts:
  - `output/data2_turn_window_y_turn_conditioned_position_probe/summary.md`
  - `output/data2_turn_window_lgy_from_y_gain_scale_probe/summary.md`
- metrics:
  - 当前最平衡 engineering reference 仍是 `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25`：
    - turn-window `lever_x/y_error_improve = +0.002590 / +0.001015 m`
    - `lever_x/y share = 0.0785 / 0.1111`
    - `pos_x/y share = 0.8392 / 0.8022`
  - 后续 `positive-turn lgy_from_y` 扫描只带来轻微 `y` 轴边际变化，未替换上述 main reference
- artifact_mtime:
  - 复用既有 summary 产物，本轮无新增 artifact
- config_hash_or_mtime:
  - 复用既有 case config，本轮无新增 config
- dataset_time_window:
  - 说明基于 `data2` pure `INS/GNSS` turn-window `GNSS_POS` research branch 的既有 evidence
- result_freshness_check:
  - 本轮没有新运行；解释只基于 `2026-03-19` 已 fresh 生成的 turn-conditioned / lgy-from-y summaries 与当前源码
- observability_notes:
  - 状态块：`p(0:2)` 与 `gnss_lever_x/y(28/29)` 的 `GNSS_POS` update 路由
  - 行为结果：
    - `gnss_pos_position_gain_scale` 不是修改量测噪声或过程噪声，而是在 `GNSS_POS` 更新时直接缩放 `position(0:2)` 对应的 Kalman gain 行
    - `gnss_pos_lgx_from_y_gain_scale` 与 `gnss_pos_lgy_from_y_gain_scale` 也不是 `Q/R` 参数，而是直接缩放单个 gain 元素 `K(l_gx, meas_y)` 与 `K(l_gy, meas_y)`
    - turn-conditioned variants 通过运行时 `omega_z` 先决定 effective gain，再进入同一 `Correct()` 更新链；底层 `K = P H^T (H P H^T + R)^{-1}` 仍按原 `P/H/R` 计算
- decision:
  - 后续讨论中应把这些参数统一称为“GNSS_POS correction routing / Kalman gain scaling knobs”，不要再与 `fusion.noise.*` 的噪声建模参数混称
  - 若要讨论真正的噪声调参，应单独指向 `fusion.noise` 与历史 `noise coupling sweep`，而不是当前 gain-scale reference
- next_step:
  - 保持 official reference 不变；若后续继续追根因，仍按 `window_1 + carry-over` full-history source attribution 进行，而不是回到 `Q/R` 调参路线

### session_id: 20260319-1556-doc-best-scheme-section-r1
- timestamp: 2026-03-19 15:56 (local)
- objective: 在 `docs/gnss_lever_gnss_pos_coupling_investigation.tex` 中补写“最优方案设置及结果”一节，用标准 Kalman 更新公式说明当前最优方案属于增益矩阵后处理，而不是 `Q/R` 噪声调参。
- scope:
  - 复核当前 official reference 的配置含义与阶段性指标。
  - 在 LaTeX 报告中新增一节，明确写出标准增益、修正增益、条件化位置行缩放与单元素交叉通道缩放的数学形式。
  - 重新编译 PDF，确认目录、表格与新增章节进入成品。
- changed_files:
  - `docs/gnss_lever_gnss_pos_coupling_investigation.tex`
  - `docs/gnss_lever_gnss_pos_coupling_investigation.pdf`
  - `walkthrough.md`
- configs:
  - 当前主参考方案口径保持不变：
    - `fusion.gnss_pos_position_gain_scale=0.5`
    - `fusion.gnss_pos_lgx_from_y_gain_scale=0.25`
    - `fusion.gnss_pos_turn_rate_threshold_deg_s=8.0`
    - `fusion.gnss_pos_positive_turn_position_gain_scale=0.25`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content docs\gnss_lever_gnss_pos_coupling_investigation.tex`
  - `Get-Content include\app\fusion.h`
  - `Get-Content src\app\pipeline_fusion.cpp`
  - `Get-Content src\core\eskf_engine.cpp`
  - `Get-Content output\data2_turn_window_position_gain_scale_probe\summary.md`
  - `Get-Content output\data2_turn_window_lgx_from_y_gain_scale_probe\summary.md`
  - `Get-Content output\data2_turn_window_y_turn_conditioned_position_probe\summary.md`
  - `xelatex -interaction=nonstopmode -halt-on-error gnss_lever_gnss_pos_coupling_investigation.tex`
  - `pdftotext gnss_lever_gnss_pos_coupling_investigation.pdf -`
- artifacts:
  - `docs/gnss_lever_gnss_pos_coupling_investigation.tex`
  - `docs/gnss_lever_gnss_pos_coupling_investigation.pdf`
- metrics:
  - 新增文档节明确写入当前最优方案的逐步形成链：
    - baseline: `x/y=-1.614/-2.534 mm`
    - `pos_gain=0.5`: `x/y=-0.547/-0.708 mm`
    - `pos_gain=0.5 + lgx_from_y_gain_scale=0.25`: `x/y=+1.360/-0.835 mm`
    - `pos_gain=0.5 + lgx_from_y_gain_scale=0.25 + positive-turn position gain=0.25`: `x/y=+2.590/+1.015 mm`
  - 新增文档节明确写出：
    - 标准增益 `K = P^- H^T (H P^- H^T + R)^{-1}`
    - 实际修正增益 `\tilde{K} = (D K) \circ G`
    - `D` 为位置状态行缩放，`G` 为单元素通道缩放
- artifact_mtime:
  - `docs/gnss_lever_gnss_pos_coupling_investigation.tex`: `2026-03-19 15:56` (local)
  - `docs/gnss_lever_gnss_pos_coupling_investigation.pdf`: `2026-03-19 15:57` (local)
- config_hash_or_mtime:
  - 复用既有研究产物与现有源码，本轮无新增 solver config
- dataset_time_window:
  - 文档补写基于既有 `data2` pure `INS/GNSS` turn-window evidence，不新增数据窗口
- result_freshness_check:
  - 本轮没有新实验 rerun；文档中的数值均复用 `2026-03-19` 已 fresh 生成的 gain-scale 与 turn-conditioned probe summaries
  - PDF 已 fresh 重编译；`pdftotext` 已确认新增章节标题、公式小节标题与表格标题进入成品
- observability_notes:
  - 状态块：强调 `position(0:2)` 与 `gnss_lever_x(28)` 对应的 gain routing 干预
  - 传感器窗口：仅讨论 `GNSS_POS` 更新；不涉及 `GNSS_VEL`
  - 行为结果：
    - 文档已明确区分“噪声调参”与“增益矩阵后处理”
    - 文档已把最优方案表述为结构性缓解，而非根因闭环
- decision:
  - 后续若引用该报告，可直接把“最优方案”表述为：标准 Kalman 增益计算后，对位置状态行和特定交叉通道元素做条件化缩放
  - 不把该方案再误写成“调小观测噪声”或“调大过程噪声”
- next_step:
  - 若用户继续打磨论文文稿，可在该节基础上再补一张“标准增益矩阵与修正增益矩阵作用路径示意图”；若继续做机理研究，则仍回到 `window_1 + carry-over` source attribution 主线

### session_id: 20260320-0817-data2-turn-motion-stats-r1
- timestamp: 2026-03-20 08:17 (local)
- objective: 回答用户“`data2` 数据里转弯占运动时间比例，以及左转/右转各有多少”并固化可复现统计口径。
- scope:
  - 复核 `walkthrough.md` 当前转向代理约定与 `POS_converted` 轨迹来源。
  - 新增独立脚本，对 `dataset/data2_converted/POS_converted.txt` 统计 motion/turn masks、左右转分段与阈值敏感性。
  - 不改 solver；只新增分析脚本与 `output/` 产物。
- changed_files:
  - `scripts/analysis/analyze_data2_turn_motion_stats.py`
  - `output/data2_turn_motion_stats/summary.md`
  - `output/data2_turn_motion_stats/turn_segments.csv`
  - `output/data2_turn_motion_stats/threshold_sensitivity.csv`
  - `output/data2_turn_motion_stats/manifest.json`
  - `walkthrough.md`
- configs:
  - 无 solver config 变更；analysis parameters 固定为:
    - `pos_path=dataset/data2_converted/POS_converted.txt`
    - `speed_threshold_m_s=3.0`
    - `turn_rate_threshold_deg_s=8.0`
    - `min_turn_duration_s=1.0`
    - `smooth_window_s=0.0`
- commands:
  - `Get-Content walkthrough.md`
  - `python -m py_compile scripts\analysis\analyze_data2_turn_motion_stats.py`
  - `python scripts\analysis\analyze_data2_turn_motion_stats.py`
  - `Get-Content output\data2_turn_motion_stats\summary.md`
  - `Get-Content output\data2_turn_motion_stats\manifest.json`
- artifacts:
  - `output/data2_turn_motion_stats/summary.md`
  - `output/data2_turn_motion_stats/turn_segments.csv`
  - `output/data2_turn_motion_stats/threshold_sensitivity.csv`
  - `output/data2_turn_motion_stats/manifest.json`
- metrics:
  - 主口径（`speed>=3 m/s`, `|yaw_rate|>=8 deg/s`, `min_turn_duration>=1 s`）下:
    - `motion_duration=2201.806 s`
    - `turn_duration=211.901 s`
    - `turn_time_ratio_of_motion=9.6239%`
    - `left_turn_duration=87.005 s`, `right_turn_duration=124.895 s`
    - `left/right/total_turn_segments=19/25/44`
  - threshold sensitivity:
    - `|yaw_rate|>=5 deg/s`: `11.9584%`, `left/right=16/25`
    - `|yaw_rate|>=10 deg/s`: `7.5279%`, `left/right=19/25`
  - sign sanity:
    - `POS yaw_rate` vs planar `velocity-curvature` direct sign match=`0.996978`
- artifact_mtime:
  - `output/data2_turn_motion_stats/summary.md`: `2026-03-20 08:17` (local)
  - `output/data2_turn_motion_stats/turn_segments.csv`: `2026-03-20 08:17` (local)
  - `output/data2_turn_motion_stats/threshold_sensitivity.csv`: `2026-03-20 08:17` (local)
  - `output/data2_turn_motion_stats/manifest.json`: `2026-03-20 08:17` (local)
- config_hash_or_mtime:
  - 无 solver config；analysis parameter snapshot 已写入 `output/data2_turn_motion_stats/manifest.json`
- dataset_time_window:
  - `dataset/data2_converted/POS_converted.txt`, `t=[528076.009368, 530492.815840] s`, total log duration=`2416.806472 s`
- result_freshness_check:
  - 统计脚本与全部产物均于 `2026-03-20 08:17` fresh 生成；不是复用旧 summary
- observability_notes:
  - 本轮不涉及 `21-30` 可观性结论更新；仅固化 `data2` 的轨迹运动学/转弯丰富度口径
  - 方向判定使用 `POS yaw_rate`，并以 `velocity-curvature` sign match=`0.996978` 做几何一致性 sanity check；`yaw_rate>0` 记为右转，`yaw_rate<0` 记为左转
- decision:
  - 后续若需要引用 `data2` 的“转弯丰富度”或左/右转统计，默认使用该脚本和主口径，不再临时手算
  - 当前数据在主口径下呈现“右转略多且右转持续时间更长”的分布：`25` 次右转 vs `19` 次左转
- next_step:
  - 若用户后续要做跨数据集/跨阈值比较，直接复用 `scripts/analysis/analyze_data2_turn_motion_stats.py` 到 `data4` 或修改 `--turn-rate-threshold-deg-s`

### session_id: 20260320-0900-data2-gnss-lever-p0-q-sweep-r1
- timestamp: 2026-03-20 09:00 (local)
- objective: 落地“优先从噪声解释而不是直接改增益”的新路线，在纯 `INS/GNSS + README-IMU` 基线上 staged 扫描 `GNSS lever` 初始协方差/P0 与过程噪声 `Q`，并与 archived baseline、turn-window gain-tuned reference 做同指标对照。
- scope:
  - 新增独立实验脚本 `scripts/analysis/run_data2_ins_gnss_lever_p0_q_sweep.py`，复用现有 truth-modeled IMU / pure `INS/GNSS` 工位。
  - 先跑小规模 smoke（`P0_std={0.2,0.5}`、`Q={1,4}`、`top_k=1`）验证 stage1/stage2/reference 管线，再跑正式 staged sweep：`P0_std={0.05,0.1,0.2,0.5,1.0}`，固定 `Q=1` 做 stage1，并对 top-2 `P0` 候选展开 `Q_scale={0.1,0.25,1,4,10}`。
  - 不修改 solver 数学，只新增 Python 侧实验编排、汇总、绘图与 reference comparison。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_lever_p0_q_sweep.py`
  - `output/data2_ins_gnss_lever_p0_q_sweep/{summary.md,manifest.json,case_metrics.csv,lever_metrics.csv,candidate_selection.csv,reference_metrics.csv,best_case_comparison.csv,plots/*.png}`
  - `walkthrough.md`
- configs:
  - `base_config=config_data2_baseline_eskf.yaml`
  - `model_variant=eskf`
  - `p0_std_scales_m=[0.05,0.1,0.2,0.5,1.0]`
  - `stage1_q_scale=1.0`
  - `stage2_top_k=2`
  - `stage2_q_scales=[0.1,0.25,1.0,4.0,10.0]`
  - `gain_reference_case=output/data2_turn_window_y_turn_conditioned_position_probe/artifacts/cases/joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25`
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "std_gnss_lever_arm|sigma_gnss_lever_arm|P0_diag\\[28:31\\]" scripts src include -S`
  - `python -m py_compile scripts\analysis\run_data2_ins_gnss_lever_p0_q_sweep.py`
  - `python scripts\analysis\run_data2_ins_gnss_lever_p0_q_sweep.py --p0-std-scales 0.2,0.5 --q-scales 1.0,4.0 --stage2-top-k 1`
  - `python scripts\analysis\run_data2_ins_gnss_lever_p0_q_sweep.py`
  - `Get-Content output\data2_ins_gnss_lever_p0_q_sweep\summary.md`
  - `Get-Content output\data2_ins_gnss_lever_p0_q_sweep\manifest.json`
- artifacts:
  - `output/data2_ins_gnss_lever_p0_q_sweep/summary.md`
  - `output/data2_ins_gnss_lever_p0_q_sweep/manifest.json`
  - `output/data2_ins_gnss_lever_p0_q_sweep/case_metrics.csv`
  - `output/data2_ins_gnss_lever_p0_q_sweep/lever_metrics.csv`
  - `output/data2_ins_gnss_lever_p0_q_sweep/candidate_selection.csv`
  - `output/data2_ins_gnss_lever_p0_q_sweep/reference_metrics.csv`
  - `output/data2_ins_gnss_lever_p0_q_sweep/best_case_comparison.csv`
- metrics:
  - archived baseline reproduction:
    - `eskf_p0std0p2_qlever1` 与 `truth_imu_archive_baseline` 的 `final_lever_dev_norm=0.230047 m`、`nav_rmse_3d=0.270254 m` 逐位一致
  - stage1 (`Q=1`) 最优:
    - `eskf_p0std1_qlever1`: `final_lever_dev_norm=0.111468 m`，相对 baseline `-51.5%`
    - `lever_z_final_abs_error=0.226201→0.105510 m`
    - `nav_rmse_3d=0.270254→0.088308 m`
  - stage2 最优:
    - `eskf_p0std1_qlever0p1`: `final_lever_dev_norm=0.108377 m`
    - 相对 `p0_only_best` 的额外改善仅 `2.77%`
    - `lever_z_final_abs_error=0.101842 m`
    - `nav_rmse_3d=0.087395 m`
  - gain reference 对照:
    - `joint_pos_gain_0p5_lgx_from_y_0p25_pos_turn_0p25`: `final_lever_dev_norm=0.275873 m`, `nav_rmse_3d=0.321957 m`
    - 按 full-trajectory global lever norm，本轮 `noise_only_best` 更优，但该 case 仍只应被视作 turn-window heuristic reference
- artifact_mtime:
  - `output/data2_ins_gnss_lever_p0_q_sweep/case_metrics.csv`: `2026-03-20 09:17:37` (local)
  - `output/data2_ins_gnss_lever_p0_q_sweep/manifest.json`: `2026-03-20 09:17:43` (local)
  - `output/data2_ins_gnss_lever_p0_q_sweep/summary.md`: `2026-03-20 09:19:16` (local)
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: `2026-03-16 16:47:34` (local)
- dataset_time_window:
  - solver 使用 `config_data2_baseline_eskf.yaml` 的 `t=[528076.0, 530488.9] s`
  - `dataset/data2/rtk.txt` 全文件范围 `t=[527637.000000, 530489.000000] s`, `rows=2853`
- result_freshness_check:
  - `python -m py_compile` 通过
  - 正式 staged sweep 与全部 `output/data2_ins_gnss_lever_p0_q_sweep/*` 产物均于 `2026-03-20 09:17~09:19` fresh 生成
  - archived baseline 仅用于 reproduction check，`delta norm=0`；gain reference 仅作为外部 heuristic reference 读取，没有本轮重跑
- observability_notes:
  - 约束/冻结块：本轮只针对 `GNSS lever(28:30)` 的噪声语义做干预；显式关闭 `ODO/NHC`、`mounting`、`odo_lever`、`odo_scale`
  - 传感器窗口：主更新链为 `IMU predict -> GNSS_POS`；不涉及 `GNSS_VEL`、`ODO`、`NHC`、`UWB`
  - 行为结果：
    - 放宽 `GNSS lever P0` 后 `x/y/z` 全部改善，其中收益主要落在 `z(30)`，说明当前 baseline 的 prior stiffness 本身就是 recoverability blocker
    - 在大 `P0` 已放开的前提下，`Q` 只提供小幅增益，未改变“`P0` 主导、`Q` 次级”的基本结论
- decision:
  - 将“`GNSS lever` 噪声路线”的主叙事从“简单 `R/Q` 调参”更新为“先放宽 `GNSS lever P0`，再用 mild `Q` 微调”；当前 pure `INS/GNSS` 下最有解释力的噪声干预是 `P0_std≈1.0 m`
  - 保留 gain-tuned scheme 作为 heuristic benchmark，但不再把它当作唯一的最优 reference；至少在 full-trajectory global lever 指标上，noise-only route 已可优于该 reference
- next_step:
  - 把 `p0_std≈1.0 m, q≈0.1~1.0x` 迁移到 full official `INS/GNSS/ODO/NHC` 流程做 transfer check，并优先确认收益是否主要来自 `z(30)` prior relaxation
  - 若迁移后仍成立，继续做 axis-specific `P0` 扫描与 early `GNSS_POS` gain/cov numerator 对账；若迁移失败，再回到 shared-correction / carry-over 主线解释“为何 full pipeline 会重新锁死 lever”

### session_id: 20260320-0927-data2-gnss-lever-p0-q-sweep-closeout-r1
- timestamp: 2026-03-20 09:27 (local)
- objective: 核验 `GNSS lever P0/Q staged sweep` 的最终脚本与产物都已落地，并按仓库工作流补齐本次实现收尾记录。
- scope:
  - 复查 `walkthrough.md` 中的 `EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1`、`HYP-27` 与 `Next Actions` 是否已与实验结果一致。
  - 核对新增脚本与结果摘要文件存在且时间戳一致，不新增新的 solver rerun。
- changed_files:
  - `walkthrough.md`
- configs:
  - `base_config=config_data2_baseline_eskf.yaml`
  - `closeout_scope=verification_only`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content scripts/analysis/run_data2_ins_gnss_lever_p0_q_sweep.py`
  - `Get-Content output/data2_ins_gnss_lever_p0_q_sweep/summary.md`
  - `git diff -- scripts/analysis/run_data2_ins_gnss_lever_p0_q_sweep.py walkthrough.md output/data2_ins_gnss_lever_p0_q_sweep/summary.md`
  - `Get-Date -Format "yyyy-MM-dd HH:mm"`
- artifacts:
  - `scripts/analysis/run_data2_ins_gnss_lever_p0_q_sweep.py`
  - `output/data2_ins_gnss_lever_p0_q_sweep/summary.md`
  - `output/data2_ins_gnss_lever_p0_q_sweep/manifest.json`
- metrics:
  - `closeout_verification`: `EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1` 的主结论保持不变
  - `best_p0_only_final_lever_dev_norm=0.111468 m`
  - `best_noise_only_final_lever_dev_norm=0.108377 m`
  - `noise_interpretation=p0_dominant`
- artifact_mtime:
  - `scripts/analysis/run_data2_ins_gnss_lever_p0_q_sweep.py`: `2026-03-20 09:18:57` (local)
  - `output/data2_ins_gnss_lever_p0_q_sweep/summary.md`: `2026-03-20 09:19:16` (local)
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: 未在本次 closeout 中改动
- dataset_time_window:
  - 无新增 rerun；沿用 `EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1` 已记录的数据窗口
- result_freshness_check:
  - 本次仅做 closeout verification；未生成新 solver 结果，也未覆写 `output/data2_ins_gnss_lever_p0_q_sweep/*`
  - 已确认脚本与摘要文件存在，且与前一 session 记录的指标一致
- observability_notes:
  - 本次不新增 `21-30` 状态块可观性结论；只确认 `HYP-27` 仍是当前噪声主线的正式表述
  - 当前 open item 仍是把 `GNSS lever(28:30)` 的 `P0` 放宽策略迁移回 full official 管线，检查 `ODO/NHC` 重新耦合后结论是否保持
- decision:
  - “Implement the plan” 已完成到可复用实验脚本、完整产物和文档登记三层；本轮无需再追加补丁
  - 后续研究从 `Next Actions` 第 1 项继续，而不是回退到 gain-only 调整路线
- next_step:
  - 直接执行 `Next Actions` 第 1 项：把 `p0_std≈1.0 m, q≈0.1~1.0x` 迁移到 full official `INS/GNSS/ODO/NHC` 流程做 transfer check

## 已归档会话摘要

### 2026-03-18 新归档主题

| 主题 | 涉及 session_id | 核心动作 | 最终决定 / 产物 | 当前关联 | archive_ref |
|---|---|---|---|---|---|
| 研究栈与 reader/report 收敛 | `20260311-1022` ~ `20260313-1728` | 代码审查与 codefix、data4 `P0_diag` 根因、reader/report 导出与低风险重构。 | 形成 codefix 后基线、data4 canonical 口径与当前 reader-facing 报告体系。 | 作为 `HYP-7/8/10/17/18` 的历史背景。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_01_pre_official_outage_research_stack` |
| official outage 源重置与输出标准化 | `20260315-2114`、`20260316-1659`、`20260316-2052`、`20260316-2106` | 切换到 corrected RTK、重置 outage 指标、建立 `output/data2_eskf_baseline/` 标准目录与图集。 | 当前 `data2 official outage` 正式 evidence 入口被固定。 | 直接支撑 `EXP-20260316-data2-eskf-baseline-output-r2`。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_02_official_outage_transition` |
| official outage state sanity / update attribution | `20260316-2324` ~ `20260317-1353` | 做 `22` 状态矩阵、补状态语义与 ODO/NHC/INS 审计、引入 deep-research shared audit 与 attribution case。 | 把主问题收敛到 `road-constraint coupling + standard ESKF reset/covariance semantics`。 | 直接支撑 `HYP-19` 与 `EXP-20260317-data2-overall-update-attribution-r1`。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_03_state_sanity_and_update_attribution` |
| pure INS/GNSS recoverability 与 runtime anchor 前置归因 | `20260317-1515`、`20260317-1607`、`20260317-2000` | 建立纯 `INS/GNSS` state sanity、初版 `GNSS lever` attribution 与 Jacobian / correction / Markov 审计。 | 证明“杆臂首更即错平台”需要继续向 `PVA` 隔离语义排查，而不只是盯 `H` 符号。 | 当前 runtime anchor 结论的前史与对照链。 | `docs/project_history/walkthrough_archive_sessions_20260318.md :: theme_04_pure_ins_gnss_pre_runtime_anchor` |

### 更早归档主题

| 主题 | 涉及 session_id | 核心动作 | 当前关联 | archive_ref |
|---|---|---|---|---|
| InEKF 初版重写与 best switch 锁定 | `20260304-1435` ~ `20260305-1048` | 从 FEJ 依赖版转到 RI 实现，锁定 best switch。 | 仍是所有 `InEKF` 实验的技术起点。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_01_inekf_initial_rewrite_best_switch` |
| true_iekf 分支建立与 phase-2 主修复 | `20260306-1003` ~ `20260310-1935` | 完成 tex-code audit、Lie 核心 Jacobian / reset 重写与数值审计。 | 仍是 `HYP-7/HYP-8/HYP-12` 的背景。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_02_true_iekf_branch_phase2` |
| data2 post-GNSS 漂移与 bg/mounting 因果链 | `20260305-1029` ~ `20260310-1643` | 用冻结矩阵、状态图与 `freeze_bg`/`freeze_mount` 干预定位 data2 剩余 blocker。 | 仍是 `HYP-11` 的直接背景。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_03_data2_postgnss_bg_mounting_chain` |
| data2 GNSS30 的 NHC 频率扫描链 | `20260311-1051` ~ `20260312-1128` | 将 interval audit、smoke 与正式 sweep 收敛成算法分开的最优频率结论。 | 仍是 `HYP-13` 的直接背景。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_04_data2_gnss30_nhc_sweep_chain` |
| 报告交付演进链 | `20260305-1611` ~ `20260311-0104` | notebook、interactive html、dataset-partitioned html 多轮迭代。 | 解释仓库内多份导出器与 HTML 版本共存。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_05_report_delivery_evolution` |
| 被 supersede 的 data4 r1 / outage 误接链 | `20260305-2222` ~ `20260312-2150` | 回溯旧 data4 sweep、outage 误接与 representative 报告冲突。 | 提醒当前正式口径不要回退引用旧链路。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_06_superseded_data4_r1_and_outage_chain` |

## Next Actions

1. 最高优先级：把 `EXP-20260320-data2-ins-gnss-lever-p0-q-sweep-r1` 的最优噪声组合 `p0_std≈1.0 m, q≈0.1~1.0x` 迁移到 full official `INS/GNSS/ODO/NHC` 流程，检查它是否仍能改善 official outage 与全程杆臂指标。
2. 若 full official 流程仍受益，继续做 axis-specific `GNSS lever P0` 扫描，优先区分“主要是 `z(30)` prior 放松在起作用”还是 `x/y/z` 都需要同步放宽。
3. 在 best `P0` cases 上回到 early `GNSS_POS` 更新，直接对账 `prior_std_lg*`、`corr(p_*,l_g*)` 与首更/前几更的 gain numerator，解释为什么当前 baseline `p0_std=0.2 m` 会把杆臂锁死。
4. 当前噪声主线已改写：不再把简单 `GNSS_POS R / lever Q` 扫描当作首选解释；`GNSS lever P0` 现为官方优先噪声路线，`Q` 只作为 secondary fine-tuning。
5. 保留 `gain_tuned_reference` 作为 external heuristic benchmark，但在 full official 流程验证完成前，不再把 `gnss_pos_*_gain_scale` knobs 当作默认正式方案。
6. 若 `P0` 路线在 full official 流程失效，优先检查是否是 `ODO/NHC` 重新建立了 `position / attitude / lever` shared sink，随后再决定是否回到 `window_1 + carry-over` 的 solver 级根因深挖。
7. 若继续做 solver 级因果闭环，下一轮从已锁定的点位起步：`window_1 @ 529531/529532/529536 s` 的三类 transition driver，以及 non-window `528127/528128/528158/528614/528615 s` 的 reinforcement updates。
8. 若需要在报告或跨数据集对比里引用 `data2` 的“转弯丰富度”，默认复用 `EXP-20260320-data2-turn-motion-stats-r1` 的主口径：`speed>=3 m/s`、`|yaw_rate|>=8 deg/s`、`min_turn_duration>=1 s`；敏感性分析直接读 `output/data2_turn_motion_stats/threshold_sensitivity.csv`。
