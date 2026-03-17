# walkthrough.md

Schema: `v1`
文档语言: 中文（必要技术字段保留英文）
Last updated: `2026-03-17`

## 项目快照

### 当前阶段目标
- 将当前正式研究目标收敛到 `data2 + ESKF + corrected RTK GNSS`，并以 `GNSS outage` 指标作为算法优化主目标。
- 正式评价口径改为：每个无 GNSS 时段分别统计位置误差 `x/y/z/3D RMSE` 与该时段末时刻 `x/y/z/3D` 误差；整段总体 RMSE 仅保留为辅助诊断。
- `data2` 官方 ESKF 基线固定为 `dataset/data2/rtk.txt`（7 列，仅位置与位置标准差，无速度），并显式关闭 `GNSS_VEL` 更新。
- 弱可观状态 `mounting_roll`（状态 22）与 `gnss_lever_z`（状态 30）在当前官方 `data2 ESKF` 口径下全程不参与估计。
- 保持主 `walkthrough.md` 可直接完成 Start Checklist，并继续把旧总体 RMSE 口径与旧 GNSS source 结果保留为历史留痕而非当前证据。

### 当前工作台速览
- last_completed_session: `20260317-1515-data2-ins-gnss-state-sanity-r1`
- current_official_experiments: `EXP-20260316-data2-eskf-baseline-output-r2`（canonical result dir: `output/data2_eskf_baseline/`）；诊断矩阵 `EXP-20260316-data2-state-sanity-r1`（state sanity result dir: `output/data2_eskf_state_sanity/`）；`EXP-20260316-data2-eskf-baseline-output-r1` 仅保留为标准目录首版口径，已被新的误差图/柱状图规范 supersede；`EXP-20260316-data2-rtk-outage-eskf-r2` 仅保留为 metric-reset 前序会话记录，其 `output/review/*` 产物已按新规则删除；`2026-03-15` 前依赖旧 GNSS source 的结果暂不再视为当前正式口径
- open_hypotheses: `HYP-1`、`HYP-6`、`HYP-7`、`HYP-8`、`HYP-9`、`HYP-10`、`HYP-11`、`HYP-12`、`HYP-13`、`HYP-18`、`HYP-19`、`HYP-21`
- active_archive_docs: `docs/project_history/walkthrough_preservation_snapshot_20260313.md`、`docs/project_history/walkthrough_archive_registry_20260313.md`、`docs/project_history/walkthrough_archive_sessions_20260313.md`、`docs/project_history/artifact_relocation_index.md`
- pending_next_actions: 见文末 `## Next Actions`。

### 当前基线与代码事实
- 状态维度: `kStateDim = 31`
- 主要滤波模式:
  - 标准 ESKF（`fusion.fej.enable: false`）
  - InEKF 开关模式（`fusion.fej.enable: true`）
- ESKF 基线配置: `config_data2_baseline_eskf.yaml`
- data2 当前纠偏后的 GNSS 源: `dataset/data2/rtk.txt`（7 列，仅 GNSS 位置与位置标准差，不含速度）
- data2 官方 ESKF 基线附加约束:
  - `fusion.enable_gnss_velocity=false`
  - `fusion.ablation.disable_mounting_roll=true`
  - `fusion.ablation.disable_gnss_lever_z=true`
- data2 当前官方评价指标: `GNSS outage` 分段 `x/y/z/3D RMSE` 与分段末时刻 `x/y/z/3D` 误差；总体 RMSE 只保留为 auxiliary
- data2 当前官方标准结果目录: `output/data2_eskf_baseline/`
- InEKF 历史对照配置（保留复现实验）: `config_data2_baseline_inekf_ctrl.yaml`, `config_data2_gnss30_inekf.yaml`
- InEKF 当前推荐配置:
  - `config_data2_baseline_inekf_best.yaml`
  - `config_data2_gnss30_inekf_best.yaml`
  - 固定开关: `ri_gnss_pos_use_p_ned_local=true`, `ri_vel_gyro_noise_mode=1`, `ri_inject_pos_inverse=true`
- data4 主口径与配置:
  - GNSS 主口径: `dataset/data4_converted/GNSS_converted.txt`（13 列，含速度）
  - data4 4 组主配置: `config_data4_baseline_eskf.yaml`, `config_data4_baseline_inekf_best.yaml`, `config_data4_gnss30_eskf.yaml`, `config_data4_gnss30_inekf_best.yaml`
  - data4 当前正式初始化口径: `config_data4_gnss30_eskf.yaml` 的 `P0_diag` 已按 `std_*` 等效方差重写，并由 `EXP-20260312-data4-p0diag-check-r1` / `r2` fresh 证据链验证。
- 当前官方只保留 `data2 ESKF outage` 的 corrected RTK 分段评估链路；其余 `data2/data4` 组合尚未按同一 metric-reset + source-fix 规则重跑。

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

> 2026-03-15 superseding note: 凡是 `2026-03-15` 前依赖 `*_converted/GNSS_converted.txt` 的实验行，当前只保留为历史留痕；除非按 corrected GNSS source 重跑，否则不得再作为正式结论引用。

| exp_id | 日期 | 目的 | 配置 | 关键产物 | 关键指标 | 状态 | 新鲜度 |
|---|---|---|---|---|---|---|---|
| EXP-20260317-data2-ins-gnss-state-sanity-r1 | 2026-03-17 | 在 `data2 + ESKF + corrected RTK + full GNSS` 下移除 `ODO/NHC` 与 `mounting/odo_lever/odo_scale`，仅对 `ba/bg/sg/sa/gnss_lever` 做真值锚定控制组 + 单状态释放，区分“road-constraint 放大”与“INS/GNSS 链自身 recoverability” | `config_data2_baseline_eskf.yaml`; `scripts/analysis/run_data2_ins_gnss_state_sanity.py`; `scripts/analysis/run_data2_state_sanity_matrix.py` | `output/data2_eskf_ins_gnss_state_sanity/{summary.md,manifest.json,case_metrics.csv,state_judgement.csv,plots/*,artifacts/cases/*}` | control full-GNSS `nav_rmse/final3d=0.049119/0.018997`; `gnss_lever_x=normal`; `gnss_lever_y/z` behavior normal 但 absolute nav `0.061576/0.042114`、`0.268493/0.217890`; `ba/bg/sg/sa` 中仅 `sa_z=borderline`，其余 behavior 仍多为 abnormal | ins_gnss_state_sanity_complete | pass |
| EXP-20260317-data2-overall-update-attribution-r1 | 2026-03-17 | 在 `data2 official outage` 上对 `standard_reset_gamma`、`NHC/ODO off`、`NHC/ODO 20Hz` 与组合 case 做代表性 state-sanity 归因实验，区分 shared abnormality 的主导来源 | `config_data2_baseline_eskf.yaml`; `scripts/analysis/run_data2_overall_update_attribution.py`; `scripts/analysis/run_data2_state_sanity_matrix.py`; `src/core/eskf_engine.cpp`; `src/app/{config,pipeline_fusion}.cpp`; `include/{app/fusion,core/eskf}.h` | `output/data2_eskf_update_attribution/{summary.md,manifest.json,variant_overview.csv,tracked_state_summary.csv,variants/*}` | baseline control `12.063017/45.713949`; `standard_reset_gamma` control mean/max `10.729688/38.526999`; `nhc_off` control mean/max `48.057915/234.148274`; `odo_off` control mean/max `51.268643/178.081870`; `mounting_roll` remained `normal` with zero delta in all `7` variants; `nhc_20hz` and `standard_reset_gamma_nhc_20hz` made `release_bg_y` catastrophic (`548703.775/3865296.078`, `500788.097/3274396.121`) | update_attribution_complete | pass |
| EXP-20260317-overall-update-audit-r1 | 2026-03-17 | 使用 deep-research 多子审计复核 `data2 official outage` 下大量非 PVA 状态异常是否来自 shared update-chain / covariance 语义问题，而不只是单状态不可观 | `config_data2_baseline_eskf.yaml`; `.research/20260316-overall-update-audit-a1/{run_children.py,aggregate.py,prompts/*.md}`; 代码审计覆盖 `src/app/{pipeline_fusion,dataset_loader,evaluation}.cpp`, `src/core/{eskf_engine,measurement_models_uwb,ins_mech}.cpp`, `include/{app/fusion,core/eskf}.h` | `.research/20260316-overall-update-audit-a1/{child_outputs/*.md,final_report.md,polished_report.md}` | reference matrix `abnormal/borderline/normal=18/1/3`; active suspects=`ODO/NHC high-rate shared-state coupling + standard ESKF reset semantics`; latent-only=`GNSS velocity contract + weak-excitation freeze when enabled`; ruled-out primary=`current 7-col GNSS_POS path` | deep_research_audit_complete | pass |
| EXP-20260316-data2-state-sanity-r1 | 2026-03-16 | 在 official outage 条件下，对 `22` 个非 PVA 标量状态执行“真值锚定控制组 + 单状态释放”矩阵，检查状态是否向合理区间变化，并量化对 outage 指标的影响 | `config_data2_baseline_eskf.yaml`; `scripts/analysis/run_data2_state_sanity_matrix.py`; `apps/eskf_fusion_main.cpp`; `apps/regression_checks_main.cpp`; `src/app/{config,evaluation,pipeline_fusion}.cpp`; `src/core/ins_mech.cpp` | `output/data2_eskf_state_sanity/{case_metrics.csv,state_judgement.csv,truth_reference.json,summary.md,manifest.json,plots/*.png,artifacts/cases/*}` | control `mean_outage_rmse3d/max_final3d=12.063017/45.713949`; overall labels `abnormal/borderline/normal=18/1/3`; worst `bg_y` delta mean/max=`320.215229/827.053564`; normal=`mounting_roll,gnss_lever_z,sa_z`; borderline=`odo_scale` | state_sanity_matrix_complete | pass |
| EXP-20260316-data2-eskf-baseline-output-r2 | 2026-03-16 | 在标准结果目录基础上补齐速度误差、姿态误差与 outage 分段柱状图；移除 `3D` 误差图，仅保留三个方向误差图 | `config_data2_baseline_eskf.yaml`; `plot_navresult.py`; `scripts/analysis/run_data2_rtk_outage_eval.py` | `output/data2_eskf_baseline/{metrics.csv,outage_segments.csv,summary.md,manifest.json,plot/*.png,artifacts/*}` | official outage mean/max RMSE3D=`9.248204/12.303623`; official outage mean/max final3D=`24.330687/70.749305`; plot_count=`12`; no `error_position_3d.png`; outage bar charts=`2` | official_output_plot_refined | pass |
| EXP-20260316-data2-eskf-baseline-output-r1 | 2026-03-16 | 清理旧版 `output/` 并将 `data2` 官方 ESKF GNSS outage 结果标准化到 `output/<result_name>/`，统一 `metrics/outage_segments/summary/manifest/plot/artifacts` 结构 | `config_data2_baseline_eskf.yaml`; `plot_navresult.py`; `scripts/analysis/run_data2_rtk_outage_eval.py`; `scripts/analysis/cleanup_legacy_output.py` | `output/data2_eskf_baseline/{metrics.csv,outage_segments.csv,summary.md,manifest.json,plot/*.png,artifacts/*}`（目录已在 `r2` 中 fresh 刷新，当前图集已不再保留 `r1` 的 9 图版本） | official outage mean/max RMSE3D=`9.248204/12.303623`; official outage mean/max final3D=`24.330687/70.749305`; control overall RMSE3D(aux)=`1.616821`; plot_count=`9`; deleted legacy output entries=`62` | superseded_by_r2_plot_refresh | refreshed_in_place |
| EXP-20260316-data2-rtk-outage-eskf-r2 | 2026-03-16 | 将 `data2` 官方口径重定向到 corrected RTK + ESKF + outage segment metrics，并在官方基线中全程排除 `mounting_roll` / `gnss_lever_z` 估计 | `config_data2_baseline_eskf.yaml`; `scripts/analysis/run_data2_rtk_outage_eval.py`; `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/cfg_data2_{baseline_eskf_rtk_full_official,eskf_rtk_outage_cycle_official}.yaml` | `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/*`（已在 `EXP-20260316-data2-eskf-baseline-output-r1` 中按新白名单清理删除；当前 canonical successor 为 `output/data2_eskf_baseline/*`） | control overall RMSE3D(aux)=`1.616821`; outage mean/max RMSE3D=`9.248204/12.303623`; outage mean/max final3D=`24.330687/70.749305`; segments=`9` | superseded_by_output_normalization | deleted_after_standardization |
| EXP-20260315-data2-rtk-outage-eskf-r1 | 2026-03-15 | 按用户要求清空 `archive/` 后，仅恢复 `data2` 的 corrected GNSS outage ESKF 对照；GNSS 输入切换到 `dataset/data2/rtk.txt`，并把 outage 模板改为 `300 s` 初始可用 + `100 s off / 150 s on` 循环 | `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/cfg_data2_{baseline_eskf_rtk_full,eskf_rtk_outage_cycle}.yaml`; `scripts/analysis/filter_gnss_outage.py` | `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/{GNSS_outage_cycle_rtk_300on_100off_150on.txt,SOL_data2_baseline_eskf_rtk_full.txt,SOL_data2_eskf_rtk_outage_cycle.txt,data2_*.stdout.txt,metrics.csv,summary.md,gnss_outage_stats.json}` | full/outage RMSE3D=`1.528524/6.072556`; P95=`1.621292/14.091614`; final3D=`1.469204/1.538671`; outage GNSS keep ratio=`1513/2413=0.627020` | superseded_by_r2_metric_reset | pass |
| EXP-20260311-codefix-r1 | 2026-03-11 | 修复 code review 发现的 3 个 runtime/config 问题，并做 build/smoke/regression 与 fresh solver old/new 对比 | `output/review/EXP-20260311-codefix-r1/cfg_data2_baseline_eskf.yaml`; `output/review/EXP-20260311-codefix-r1/cfg_data2_gnss30_true_iekf.yaml` | `build/Release/{regression_checks,eskf_fusion,uwb_generator}.exe`; `output/review/EXP-20260311-codefix-r1/{SOL_data2_baseline_eskf_codefix.txt,SOL_data2_gnss30_true_iekf_codefix.txt,compare_summary.md}` | `regression_checks=PASS`; fail-fast smoke tests exit `1`; baseline RMSE3D `1.223243→1.492452`; data2 GNSS30 true_iekf RMSE3D `164.245168→165.542267` | codefix_verified | pass |
| EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf | 2026-03-11 | 在 `data2 GNSS30 ESKF` 口径下固定 `ODO=raw`，把 `NHC` 从 `50Hz` 一路下探到 `1Hz`，验证“继续降低 NHC 是否仍改善” | `config_data2_gnss30_eskf_nofreeze.yaml`; `scripts/analysis/odo_nhc_update_sweep.py` | `output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/{metrics.csv,summary.md,manifest.json,plots/*}` | raw/raw RMSE3D `195.786001`; `NHC=50/20/10/5/2/1Hz` 时 RMSE3D=`112.635659/117.465035/121.913750/185.751350/262.112664/353.196283` | gnss30_nhc_downsweep | pass |
| EXP-20260311-odo-nhc-rate-sweep-r2b-data2-gnss30-eskf-refine | 2026-03-11 | 围绕 `data2 GNSS30 ESKF` 的 `NHC` 最优点做局部细扫，定位 `ODO=raw` 时的更优 `NHC` 频率区间 | `config_data2_gnss30_eskf_nofreeze.yaml`; `scripts/analysis/odo_nhc_update_sweep.py` | `output/review/EXP-20260311-odo-nhc-rate-sweep-r2b-data2-gnss30-eskf-refine/{metrics.csv,summary.md,manifest.json,plots/*}` | `NHC=100/75/60/50/40/30/20Hz` 时 RMSE3D=`134.031334/125.905375/114.330334/112.635659/110.398047/101.122603/117.465035`; 当前最优在 `30Hz` 附近 | gnss30_nhc_refine | pass |
| EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf | 2026-03-11 | 在 `data2 GNSS30 true_iekf` 口径下固定 `ODO=raw`，把 `NHC` 从 `50Hz` 一路下探到 `1Hz`，检查 true_iekf 是否沿用与 ESKF 相同的最优频段 | `config_data2_gnss30_true_iekf.yaml`; `scripts/analysis/odo_nhc_update_sweep.py` | `output/review/EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf/{metrics.csv,summary.md,manifest.json,plots/*}` | raw/raw RMSE3D `165.542267`; `NHC=50/20/10/5/2/1Hz` 时 RMSE3D=`120.821842/88.149809/65.215814/44.939188/27.344130/25.790653` | gnss30_true_iekf_nhc_downsweep | pass |
| EXP-20260311-odo-nhc-rate-sweep-r3b-data2-gnss30-true-iekf-refine | 2026-03-11 | 在 `data2 GNSS30 true_iekf` 上继续把 `NHC` 扫到 `1Hz` 以下，定位低频最优点 | `config_data2_gnss30_true_iekf.yaml`; `scripts/analysis/odo_nhc_update_sweep.py` | `output/review/EXP-20260311-odo-nhc-rate-sweep-r3b-data2-gnss30-true-iekf-refine/{metrics.csv,summary.md,manifest.json,plots/*}` | `NHC=2/1.333/1/0.75/0.667/0.5/0.333/0.25Hz` 时 RMSE3D=`27.344130/22.210205/25.790653/21.656987/22.295462/22.666655/23.466039/23.834721`; 当前最优在 `0.75Hz` 附近 | gnss30_true_iekf_nhc_refine | pass |
| EXP-20260312-inekf-mechanism-attribution-r2 | 2026-03-12 | 在 `r1` 基础上补 `process=eskf + reset=I` 联合消融，检验 `process/reset` 是否近似可加，并形成 `.tex/HTML` 可直接引用的 fresh 机制证据 | `scripts/analysis/inekf_mechanism_attribution.py`（`r2` 默认输出）；4 组 GNSS30 基线配置复用 `r1` | `output/review/EXP-20260312-inekf-mechanism-attribution-r2/{metrics.csv,summary.md,mechanism_math_note.md,per_update_stats.csv,plots/*,cases/*}` | data2 base/proc/reset/combo=`21.657/558.636/350.215/376.956`，data4=`12.011/320.995/284.796/303.785`；`veljac=eskf` 仍与 base 几乎完全重合；联合 case additivity ratio data2/data4=`0.410/0.502` | mechanism_attribution_joint_case | pass |
| EXP-20260312-inekf-mechanism-rate-vs-weight-r2 | 2026-03-12 | 在 `r1` 基础上补 `data4` minimal ported rate-vs-weight，用于给 `data2` 的“降频≈减权”结论加上跨数据集边界条件 | `scripts/analysis/inekf_mechanism_attribution.py`（新增 `data4` ported 30Hz/0.75Hz 与 rate-equivalent `scaled R` case） | `output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/{metrics.csv,summary.md,per_update_stats.csv}` | data4 ESKF raw/30Hz/raw_weight=`281.328/302.543/241.276`；data4 true raw/0.75Hz/raw_weight=`12.011/33.249/35.054`；说明 data2 最优频率不可直接迁移到 data4，且 ESKF 的“降频≈减权”不具普适性 | rate_weight_boundary_check | pass |
| EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1 | 2026-03-12 | 使用真正的 `data2 true_iekf tuned` 最优配置（`ODO raw + NHC 0.75Hz`），复用同一份 `900s + 300/120s` GNSS 周期开断文件重跑实验，并替换错误的旧第十一组 | `output/review/EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1/cfg_{data2_baseline_eskf_fresh,data2_true_tuned_gnss_outage_cycle}.yaml` | `output/review/EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1/{GNSS_outage_cycle.txt,SOL_data2_baseline_eskf_fresh.txt,SOL_data2_true_tuned_gnss_outage_cycle.txt,metrics.csv,summary.md}` | full-GNSS ESKF vs outage true_tuned RMSE3D=`1.492452/1.186893`；tail70=`1.407619/1.020212`；final3D=`1.253928/0.556853` | outage_cycle_true_tuned_corrected | pass |
| EXP-20260312-data4-p0diag-check-r1 | 2026-03-12 | 在当前代码下对 `data4_gnss30_eskf` 做最小复现实验：仅移除 `P0_diag`，让初始化退回 `std_*` 语义，验证旧 `51.37m` 是否由 `P0_diag` 生效语义变化导致 | `output/review/EXP-20260312-data4-p0diag-check-r1/cfg_data4_gnss30_eskf_stdP0.yaml`; `config_data4_gnss30_eskf.yaml` | `output/review/EXP-20260312-data4-p0diag-check-r1/{cfg_data4_gnss30_eskf_stdP0.yaml,SOL_data4_gnss30_eskf_stdP0.txt,data4_gnss30_eskf_stdP0.stdout.txt}` | 当前代码 + `std_* P0` 复跑 RMSE xyz=`30.294/19.365/36.696`，RMSE3D=`51.374372`；与旧 `2026-03-05` 的 `30.292/19.365/36.694`、`51.371764` 几乎完全一致 | root_cause_reproduction | pass |
| EXP-20260312-data4-gnss30-nhc-sweep-r2 | 2026-03-12 | 将 `config_data4_gnss30_eskf.yaml` 的 `P0_diag` 改写为 `std_*` 等效方差后，对 `data4 GNSS30` 重新做 fresh `ODO raw + NHC` 扫频，校正 data4 ESKF 的最优 NHC 结论 | `config_data4_gnss30_eskf.yaml`; `config_data4_gnss30_true_iekf.yaml`; `scripts/analysis/generate_dataset_report_cases.py` | `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/{metrics.csv,summary.md,manifest.json,plots/*,data4_gnss30_*/*}` | ESKF: raw=`51.373795` 且即 best，fixed sweep 最优 `100Hz=61.005053`；`true_iekf`: raw=`12.011497` 且即 best，fixed sweep 最优 `50Hz=22.020263` | fresh_config_rewrite_validation | pass |
| EXP-20260312-dataset-report-cases-r2 | 2026-03-12 | 基于修正后的 `data4_gnss30_eskf` 初始化语义，重生新分组 HTML 所需 10 个 canonical cases，并统一导出状态量变化图与 fresh 指标 | `output/review/EXP-20260312-dataset-report-cases-r2/cfg_*.yaml`; 复用 `data2` 已锁定最优 NHC 与 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 的 data4 最优 NHC | `output/review/EXP-20260312-dataset-report-cases-r2/{metrics.csv,summary.md,manifest.json,GNSS_outage_cycle_data2.txt,GNSS_outage_cycle_data4.txt,SOL_*.txt,*.log}`; `output/result_data{2,4}_report_*/` | data2 full/gnss30_eskf/gnss30_true/outage_eskf/outage_true RMSE3D=`1.492452/101.122603/21.656987/6.139390/1.186893`；data4=`1.229320/51.373795/12.011497/1.279736/1.262719`；10 个案例状态图目录均存在且各含 `12` 张 PNG（含 `02b_velocity_vehicle_v.png`） | canonical_cases_before_data4_full_p0_alignment | partially_superseded_by_r4 |
| EXP-20260313-dataset-report-cases-r4 | 2026-03-13 | 以 `data4_gnss_outage_eskf_best` 的真实 `P0_diag` 口径为准，重新统一 `data4_full_gnss_eskf` 与 `data4_gnss_outage_eskf_best` 的初始化配置，并 fresh 重生 canonical cases | `scripts/analysis/generate_dataset_report_cases.py`; `output/review/EXP-20260313-dataset-report-cases-r4/cfg_*.yaml`; 复用 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 的 data4 最优 NHC | `output/review/EXP-20260313-dataset-report-cases-r4/{metrics.csv,summary.md,manifest.json,GNSS_outage_cycle_data2.txt,GNSS_outage_cycle_data4.txt,SOL_*.txt,*.log}`; `output/result_data{2,4}_report_*/` | data2 指标与 `r2` 保持一致；data4=`0.929072/51.373795/12.011497/1.279736/1.262719`；`cfg_data4_full_gnss_eskf` 与 `cfg_data4_gnss_outage_eskf_best` 的 `P0_diag/gnss_schedule` 一致，仅 `gnss_path/output_path` 不同 | canonical_cases_after_data4_full_p0_alignment | partially_superseded_by_r5 |
| EXP-20260313-dataset-partitioned-html-reader-r4 | 2026-03-13 | 在不重跑实验的前提下继续压缩新分组 HTML 的说明页表格：删除冗余列、合并 ESKF/InEKF 公共项，并移除说明页横向滚动 | `scripts/analysis/dataset_partitioned_nav_report.py`; `scripts/analysis/export_dataset_partitioned_nav_report_html.py`; 引用 `EXP-20260312-dataset-report-cases-r2` | `output/html/dataset_partitioned_navigation_interactive_report.html` | 说明页状态表/观测表均压缩为 `6` 列；旧表头 `是否参与当前主估计/初始协方差/是否启用/GNSS30 场景 ESKF` 命中均为 `0`；`nav_items/page_views=7/7`；`gallery_cards/gallery_refs=10/4`；data2 `GNSS30 InEKF RMSE3D=21.656987`、data4 `GNSS30 ESKF RMSE3D=51.373795` 保持不变 | reader_table_compaction | pass |
| EXP-20260313-dataset-partitioned-html-gnss-window-shading-r1 | 2026-03-13 | 为 `dataset_partitioned_navigation_interactive_report.html` 的周期开断页面时序误差图添加 GNSS 可用时段背景色，轨迹图和状态量变化图保持不变 | `scripts/analysis/interactive_nav_report.py`; `scripts/analysis/dataset_partitioned_nav_report.py`; `scripts/analysis/export_dataset_partitioned_nav_report_html.py`; 引用 `EXP-20260312-dataset-report-cases-r2` | `output/html/dataset_partitioned_navigation_interactive_report.html` | HTML 核验：`page-data2-3/page-data4-3` 的 shade_count=`40/30`（对应 `4/3` 个 GNSS 窗口 × `10` 个误差子图），其余页面 shade_count=`0`；数值指标口径不变 | outage_window_shading | pass |
| EXP-20260313-dataset-partitioned-html-r5 | 2026-03-13 | 在 `EXP-20260313-dataset-report-cases-r4` 基础上重导出读者版 HTML，使 `data4` full/outage 页面切换到统一后的 `P0_diag` 官方口径 | `scripts/analysis/dataset_partitioned_nav_report.py`; `scripts/analysis/export_dataset_partitioned_nav_report_html.py`; 引用 `EXP-20260313-dataset-report-cases-r4` | `output/html/dataset_partitioned_navigation_interactive_report.html` | 绑定 manifest=`EXP-20260313-dataset-report-cases-r4`；`page-data4-1` 总览为 full/GNSS30=`0.929072/51.373795`；`page-data4-3` 为 full/outage ESKF/outage InEKF=`0.929072/1.279736/1.262719` | reader_html_refreshed_to_r4 | partially_superseded_by_r6 |
| EXP-20260313-data4-baseline-gnssvel-off-audit-r1 | 2026-03-13 | 对 `data4 full GNSS baseline ESKF` 做单变量控制：关闭 `GNSS_VEL` 更新，审计末端 `v系航向误差` 显著漂移是否由 GNSS 速度更新触发 | `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/cfg_data4_full_gnss_eskf_no_gnss_vel.yaml`; 对照 `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml` | `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/SOL_data4_full_gnss_eskf_no_gnss_vel.txt`; 本文件会话记录中的 inline attribution metrics | `RMSE3D 1.229320→1.255640`；末端 `vehicle_heading_err_deg=-12.700→-14.535`、`body_yaw_err_deg=-11.730→-13.508`、`mounting_yaw_delta=0.964→1.021 deg`；baseline 最后 `120 s` truth 速度中位数 `5.83e-4 m/s` 且 `100%<0.1 m/s` | gnss_velocity_tail_audit | pass |
| EXP-20260313-noise-qr-gnss30-r1 | 2026-03-13 | 以 `EXP-20260313-dataset-report-cases-r4` 的 4 个 `GNSS30` canonical cases 为 reference，对显式 `Q/R` 做 reference 校验、单因子粗扫与局部细化，检查当前噪声设置是否存在 case-specific 优化空间 | `output/review/EXP-20260313-dataset-report-cases-r4/manifest.json`; `scripts/analysis/noise_qr_sweep.py`; `output/review/EXP-20260313-noise-qr-gnss30-r1/data*/best_config.yaml` | `output/review/EXP-20260313-noise-qr-gnss30-r1/{manifest.json,summary.md,metrics.csv,case_ranking.csv,gnss_vel_audit.json}`; `output/review/EXP-20260313-noise-qr-gnss30-r1/data*/{best_config.yaml,best_summary.md,best_vs_ref.json}` | reference 4/4 全部 `match=true`；best RMSE3D: data2 ESKF `101.123→65.182` (`R_nhc×4`), data2 true `21.657→8.120` (`Q_imu×0.25 + R_odo×2`), data4 ESKF `51.374→40.152` (`R_odo×4`, 但 final3D 变差), data4 true `12.011→4.393` (`Q_calib×0.25 + Q_bias×0.25`) | gnss30_noise_qr_sweep | pass |
| EXP-20260313-dataset-report-cases-r5 | 2026-03-13 | 将 `EXP-20260313-noise-qr-gnss30-r1` 的 4 个 `GNSS30` best noise profile 回灌到 dataset-partitioned canonical cases；同 dataset/method 下的 full GNSS 与 GNSSoutage 继承相同的 GNSS30 best profile | `scripts/analysis/generate_dataset_report_cases.py`; `output/review/EXP-20260313-dataset-report-cases-r5/cfg_*.yaml`; `output/review/EXP-20260313-noise-qr-gnss30-r1/manifest.json` | `output/review/EXP-20260313-dataset-report-cases-r5/{metrics.csv,summary.md,manifest.json,GNSS_outage_cycle_data2.txt,GNSS_outage_cycle_data4.txt,SOL_*.txt,*.log}`; `output/result_data{2,4}_report_*/` | data2=`1.080872/65.182069/8.119960/1.834075/0.984145`；data4=`0.890927/40.152394/4.392984/0.943338/1.421392`；其中 `data4_gnss_outage_true_iekf_best` 相比 `r4` 退化 `1.262719→1.421392 m` | canonical_cases_with_shared_gnss30_noise_profiles | pass_with_tradeoffs |
| EXP-20260313-dataset-partitioned-html-r6 | 2026-03-13 | 读取 `EXP-20260313-dataset-report-cases-r5` 并重导出读者版 HTML，使页面与共享 GNSS30 best profile 的 canonical cases 保持一致 | `scripts/analysis/dataset_partitioned_nav_report.py`; `scripts/analysis/export_dataset_partitioned_nav_report_html.py`; 引用 `EXP-20260313-dataset-report-cases-r5` | `output/html/dataset_partitioned_navigation_interactive_report.html` | 绑定 manifest=`EXP-20260313-dataset-report-cases-r5`；HTML 内可检索 `65.182/8.120/4.393/0.943/1.421/1.081/0.891`；GNSS 周期开断页背景色块逻辑保持不变 | reader_html_refreshed_to_r5 | partially_superseded_by_r7 |
| EXP-20260313-dataset-partitioned-html-r7 | 2026-03-13 | 将 dataset-partitioned 导出器改为默认 `asset_mode=inline`，把 `plotly` 与状态图 gallery 图片内联到单个 HTML，消除 `_report_assets` 外部依赖，便于邮件/IM 直接分发 | `scripts/analysis/export_dataset_partitioned_nav_report_html.py`; 引用 `EXP-20260313-dataset-report-cases-r5` | `output/html/{dataset_partitioned_navigation_interactive_report.html,实验结果终版.html}` | 导出验证：`has_plotly_script_src=false`、`has_report_assets_ref=false`、`has_base64_png=true`；两份单文件 HTML 大小均约 `44.75 MB`；页面数值口径保持 `r5` 不变 | reader_html_singlefile_inline_packaging | pass |
| EXP-20260313-cpp-refactor-baseline-r1 | 2026-03-13 | 启动 C++ 程序低风险修缮：抽离 `LoadDataset`、建立维护基线文档，并用 build/regression/smoke 锁定当前数值行为 | `config_data2_baseline_eskf.yaml`; `docs/cpp_refactor_baseline.md`; `src/app/dataset_loader.cpp` | `build/Release/{eskf_fusion,regression_checks}.exe`; `SOL_data2_baseline_eskf.txt`; `docs/cpp_refactor_baseline.md` | `regression_checks=PASS`；baseline smoke RMSE xyz=`0.827032/0.877094/0.879851`，RMSE3D=`1.492452`；`pipeline_fusion.cpp` 中的数据加载/裁剪逻辑已迁移到独立编译单元 | code_refactor_baseline_verified | pass |

### 历史关键实验摘要

| 主题 | 涉及 exp_id | 保留结论 | 为何仍重要 | archive_ref |
|---|---|---|---|---|
| InEKF 初版重写与 best switch 锁定 | `EXP-20260304-data2-baseline-eskf`、`EXP-20260304-data2-baseline-inekf-ctrl`、`EXP-20260304-inekf-ri-realization-codeoverhaul3`、`EXP-20260304-inekf-ri-gvelgyro-fix`、`EXP-20260305-inekf-ab-seq-r1`、`EXP-20260305-inekf-best4-regression-r1`、`EXP-20260305-inekf-fghi-sign-audit-r1`、`EXP-20260305-inekf-doubleoff-interaction-r1`、`EXP-20260305-issue001-baseline-compare-r1`、`EXP-20260305-baseline-gmode-sensitivity-r1`、`EXP-20260305-baseline-gmode-gnssvel-mechanism-r1`、`EXP-20260305-baseline-gmode-gnsspos-coupling-r1`、`EXP-20260305-data2-process-noise-regression-r1`、`EXP-20260304-data2-gnss30-postfix-gnsslever-eskf`、`EXP-20260304-data2-gnss30-inekf`、`EXP-20260304-data2-gnss30-eskf-nofreeze`、`EXP-20260304-preexp-fix-regression`、`EXP-20260304-inekf-refactor-minrisk-debug`、`EXP-20260304-inekf-rewrite-no-fej`、`EXP-20260304-inekf-ri-fgqd-audit-r2`、`EXP-20260305-inekf-deep-research-audit-r1` | 完成 RI 版 InEKF 初次落地，并通过 A/B 顺序实验把 best switch 锁定为 `p_local=ON, g=+1, inject=ON`；同时保留了 baseline 指标冲突和单因子排错链。 | 它解释了为什么后续 `InEKF` 研究以 best-switch 口径为起点，而不是回到早期 `inekf_ctrl` 或半成品 rewrite。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: theme_01_inekf_initial_rewrite_best_switch` |
| true_iekf 分支建立与 phase-2 主修复 | `EXP-20260306-inekf-tex-code-audit-r1`、`EXP-20260306-true-iekf-refactor-r1`、`EXP-20260306-true-iekf-ablation-r1`、`EXP-20260306-true-iekf-phase2-r1`、`EXP-20260306-true-iekf-phase2-ablation-lite`、`EXP-20260306-consistency-audit-r1`、`EXP-20260306-gnsspos-rframe-fix-r1`、`EXP-20260306-jacobian-audit-r1`、`EXP-20260307-jacobian-audit-r2`、`EXP-20260307-reset-floor-r1`、`EXP-20260307-update-reset-consistency-r1` | 建立 `true_iekf` 分支，完成 `ODO/NHC` Lie 核心 Jacobian、reset Gamma、一致性与 numerical Jacobian 审计，确认 phase-2 是 data4 GNSS30 大幅改善的主修复。 | 它定义了当前讨论“理论贴合实现”时所说的 `InEKF` 技术背景，也解释了为什么剩余问题不再优先归咎于 measurement Jacobian。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: theme_02_true_iekf_branch_phase2` |
| data2 post-GNSS 漂移与 bg/mounting 因果链 | `EXP-20260305-postgnss-freeze-matrix-r1`、`EXP-20260305-postgnss-headratio-robustness-r1`、`EXP-20260305-state21-30-drift-r1`、`EXP-20260305-data2-mountroll-observability-r1`、`EXP-20260305-data2-postgnss-freeze-matrix-r2`、`EXP-20260306-phase2b-attribution-r1`、`EXP-20260306-phase2c-bg-freeze-r1`、`EXP-20260306-gnss30-stateplots-r1`、`EXP-20260310-mounting-median-r1`、`EXP-20260310-inekf-mechanism-r1`、`EXP-20260305-data2-gnsspos-mechanism-r2`、`EXP-20260307-vframe-velocity-plot-r1` | 通过冻结矩阵、state21-30 漂移图、phase-2b/2c 与状态图，确认 data2 的主要残余 blocker 是 post-GNSS `bg_z` 主导的 heading drift，`mounting` 会放大位置误差但不是 yaw growth 首因。 | 这是当前解释 data2 下 `InEKF` 改善为何主要体现为 v 系航向误差，而不是全部状态均匀受益的核心因果链。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: theme_03_data2_postgnss_bg_mounting_chain` |
| data2 GNSS30 的 NHC 频率扫描链 | `EXP-20260311-project-code-review-r1`、`EXP-20260311-odo-nhc-rate-sweep-r1`、`EXP-20260311-odo-nhc-rate-sweep-smoke`、`EXP-20260312-inekf-mechanism-attribution-r1`、`EXP-20260312-inekf-mechanism-rate-vs-weight-r1` | 在正式 r2/r2b/r3/r3b 之前，完成 ODO/NHC 更新间隔审计、smoke 扫频和第一轮机理对比，证明 data2 GNSS30 的最优频率需要分算法单独搜索。 | 它解释了为什么当前 canonical report 里 ESKF `30 Hz` 与 InEKF `0.75 Hz` 的结论不是拍脑袋，而是由一条逐层细化的 sweep 链得到。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: theme_04_data2_gnss30_nhc_sweep_chain` |
| 报告交付演进链（interactive html / dataset-partitioned html） | `EXP-20260308-result-docs-r1`、`EXP-20260308-inekf-doc-typeset-r1`、`EXP-20260308-inekf-doc-typeset-r2`、`EXP-20260308-inekf-doc-typeset-r3`、`EXP-20260309-inekf-doc-symbols-r1`、`EXP-20260309-inekf-doc-compile-check-r1`、`EXP-20260309-interactive-report-notebook-r1`、`EXP-20260310-interactive-report-html-r1`、`EXP-20260310-interactive-report-html-r2`、`EXP-20260310-interactive-report-html-r3`、`EXP-20260310-interactive-report-html-r4`、`EXP-20260311-interactive-report-html-r5`、`EXP-20260311-interactive-attitude-error-fix-r1`、`EXP-20260311-html-attitude-cn-audit-r1`、`EXP-20260311-html-accuracy-audit-r1`、`EXP-20260312-interactive-report-html-r6`、`EXP-20260312-interactive-report-html-r7`、`EXP-20260312-dataset-partitioned-html-r1`、`EXP-20260312-dataset-partitioned-html-r2`、`EXP-20260313-dataset-partitioned-html-reader-r3` | 报告系统经历了 notebook、interactive html、reader-facing dataset-partitioned html 多轮重构；旧版本保留设计与命名演进证据，但当前读者版口径以后续 `reader-r4` 为准。 | 它解释了为什么仓库里会同时存在多个报告导出器、HTML 版本与文档排版实验。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: theme_05_report_delivery_evolution` |
| 被 supersede 的 data4 r1 与 outage 误接链 | `EXP-20260305-data4-inekf-switchscan-r1`、`EXP-20260305-data4-main4-regression-r1`、`EXP-20260305-data4-gnss30-freeze-matrix-r1`、`EXP-20260305-data4-gnssvel-sensitivity-r1`、`EXP-20260305-data2-main4-docsync-r1`、`EXP-20260310-gnss-lever-fix-r1`、`EXP-20260310-gnss-vel-effect-r1`、`EXP-20260310-data4-imu-precision-r1`、`EXP-20260310-data2-gnss-outage-cycle-r1`、`EXP-20260312-data2-gnss-outage-cycle-inekf-best-r1`、`EXP-20260312-data4-gnss30-nhc-sweep-r1`、`EXP-20260312-dataset-report-cases-r1` | 早期 data4 r1 sweep / canonical cases 与一轮 InEKF outage 对接存在 superseded/误接问题；在 `P0_diag` 根因定位前后，这些目录仍保留为历史转折证据，但已不再作为正式结论来源。 | 它解释了为什么部分 data4 ESKF 和 outage 结果会与当前 `r2` canonical cases 明显不一致。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: theme_06_superseded_data4_r1_and_outage_chain` |

## 已知不一致项（Known Inconsistencies）

| issue_id | 发现日期 | 描述 | 影响文件 | 影响 | 处理状态 |
|---|---|---|---|---|---|
| ISSUE-20260316-output-layout-legacy-sprawl | 2026-03-16 | `output/` 曾长期混杂 `result_*`、`compare_*`、`review/*`、`logs/*` 与散落 `.log/.txt/.md`，导致当前正式结果与历史调试产物共存、主结果路径不唯一。本会话已建立新的 canonical 目录 `output/data2_eskf_baseline/`，并删除旧版 `output/` 产物共 `62` 项；删除清单写入 `output/data2_eskf_baseline/artifacts/legacy_cleanup_report.json`。 | `output/*`; `plot_navresult.py`; `scripts/analysis/run_data2_rtk_outage_eval.py`; `scripts/analysis/cleanup_legacy_output.py`; `walkthrough.md` | 后续正式 evidence 只能引用新的标准结果目录；旧 `output/review/*` / `output/result_*` 路径已不再存在。 | resolved_in_output_standardization |
| ISSUE-20260315-gnss-source-mismatch-reset | 2026-03-15 | 用户确认前序实验使用了错误 GNSS source。对 `data2` 而言，真实 GNSS 输入应为 `dataset/data2/rtk.txt`（7 列，仅位置与位置标准差），而非 `dataset/data2_converted/GNSS_converted.txt`。本会话已按指令物理清空 `archive/`（删除前 `631` 个条目、约 `17.88 GB`），并仅 fresh 重跑 `EXP-20260315-data2-rtk-outage-eskf-r1`；其余旧 GNSS-source-dependent 结论暂全部降级为 historical/stale。 | `archive/*`; `config_data2_*`; `scripts/analysis/filter_gnss_outage.py`; `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/*`; `walkthrough.md` | 当前 reader/report/summary 中凡引用旧 GNSS source 的数值都不应继续当作正式 evidence；目前只恢复了 `data2 ESKF outage` 的 corrected 证据。 | partially_resolved_data2_outage_only |
| ISSUE-007-inekf-tex-code-gap | 2026-03-06 | 代码-理论差距已进一步收敛：phase-2 后，`ODO/NHC` 的 Lie 核心 Jacobian 与 `reset Jacobian` 已贴近 `可观性分析讨论与InEKF算法.tex`；但 `GNSS_VEL` 与 `21-30` 欧氏扩展块仍未完全 invariant/estimate-independent，故仍不能宣称“全31维完全无泄露”。 | `算法文档/可观性分析讨论与InEKF算法.tex`; `include/core/eskf.h`; `src/core/ins_mech.cpp`; `src/core/measurement_models_uwb.cpp`; `src/core/eskf_engine.cpp`; `output/review/20260306-true-iekf-phase2-r1/summary.md` | 影响 true-IEKF 的最终口径与 data2 sparse-GNSS 残余误差。 | partially_resolved_phase2 |
| ISSUE-20260312-representative-report-data4-stale | 2026-03-12 | `representative_navigation_interactive_report.html` 里的 `data4_gnss30_eskf` 仍是历史产物页面；不过经 `EXP-20260312-data4-p0diag-check-r1` 定位根因后，本轮已将 `config_data4_gnss30_eskf.yaml` 的 `P0_diag` 重写为 `std_*` 等效方差，并在 `EXP-20260312-data4-gnss30-nhc-sweep-r2` / `EXP-20260312-dataset-report-cases-r2` 中 fresh 复跑得到 `raw=51.373795`（即当前 best）。因此“51.37 vs 145.23/281.33”的数值冲突已由配置修正消除，但 representative 报告本身仍未刷新到 `r2` 证据链。 | `output/html/representative_navigation_interactive_report.html`; `output/review/EXP-20260312-data4-p0diag-check-r1/*`; `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/*`; `output/review/EXP-20260312-dataset-report-cases-r2/*`; `config_data4_gnss30_eskf.yaml`; `src/app/config.cpp`; `src/app/initialization.cpp` | 若继续把 representative 报告当作最终结论页面，仍需补做刷新或加注“historical”；但新 dataset-partitioned 报告中的 data4 ESKF GNSS30 数值已与 fresh `r2` 结论一致。 | partially_resolved_config_fixed_report_pending_refresh |
| ISSUE-20260313-data4-outage-eskf-control-mismatch | 2026-03-13 | `EXP-20260312-dataset-report-cases-r2` 中 `data4_full_gnss_eskf` 与 `data4_gnss_outage_eskf_best` 的 `P0_diag` 口径不一致，导致 full/outage ESKF 对比混入初始化差异。`EXP-20260313-dataset-report-cases-r4` 已按用户要求把 `data4_full_gnss_eskf` 对齐到 `data4_gnss_outage_eskf_best` 的真实配置口径：两者都从 `config_data4_gnss30_eskf.yaml` 继承，并统一禁用 `gnss_schedule`。 | `scripts/analysis/generate_dataset_report_cases.py`; `output/review/EXP-20260313-dataset-report-cases-r4/{cfg_data4_full_gnss_eskf.yaml,cfg_data4_gnss_outage_eskf_best.yaml,manifest.json}` | 当前官方 `data4 full GNSS ESKF` vs `data4 outage ESKF` 已可按“同一 `P0_diag` + 不同 GNSS 文件”解释；`r2` 保留为历史口径。 | resolved_in_r4_official_alignment |
| ISSUE-20260313-cpp-app-layer-overgrowth | 2026-03-13 | `src/app/` 仍存在研究期演化留下的职责堆积：`pipeline_fusion.cpp`、`config.cpp`、`diagnostics.cpp` 承载的数据加载、兼容解析、运行调度与研究日志过多，导致维护风险和误改风险偏高。`EXP-20260313-cpp-refactor-baseline-r1` 已先把 `LoadDataset` 与时间裁剪逻辑抽离到 `src/app/dataset_loader.cpp`，但 `config` 与 `diagnostics` 仍待继续拆分。 | `src/app/pipeline_fusion.cpp`; `src/app/dataset_loader.cpp`; `src/app/config.cpp`; `src/app/diagnostics.cpp`; `docs/cpp_refactor_baseline.md` | 影响后续 C++ 程序修缮的速度与安全性，但当前 solver 数值行为已通过 fresh build/regression/smoke 保持稳定。 | partially_resolved_dataset_loader_extracted |
| ISSUE-20260313-gnsspos-invariance-overclaim | 2026-03-13 | `true_iekf` 的 `GNSS_POS` 量测矩阵已与文档中的 `b` 系右不变残差形式基本对齐，但当前实现把 `NED` 协方差旋转到 body residual 坐标：`R = C_bn^T R_ned C_bn`。因此 `H` 对核心块可保持 estimate-independent，并不等于整次 `GNSS_POS` 更新的 `S/K` 也完全与当前姿态估计无关；文档中“strictly estimate-independent update”的表述偏强。 | `算法文档/可观性分析讨论与InEKF算法.tex`; `src/core/measurement_models_uwb.cpp` | 影响对 `GNSS_POS` 可观性收益来源的理论口径；若后续继续把该通道描述成“整次更新完全无泄露”，会高估当前实现与严格 InEKF 的一致性。 | open_doc_sync_needed |
| ISSUE-20260313-inekf-core-order-notation-drift | 2026-03-13 | `true_iekf` 文档在 `GNSS_POS` 推导处仍以抽象顺序 `(\phi,\rho_v,\rho_p)` 书写核心列，但 reset 节与代码实现实际映射到状态索引 `1-9` 的顺序是 `(\rho_p^b,\rho_v^b,\phi^b)`。当前代码内部是一致的，但文档未在所有公式旁显式提醒“这里只是推导记号，不是代码列序”，容易让后续 Jacobian 审计或重构把列写错位置。 | `算法文档/可观性分析讨论与InEKF算法.tex`; `src/core/measurement_models_uwb.cpp`; `src/core/eskf_engine.cpp` | 影响后续 `true_iekf` 文档同步、代码审计和新功能扩展的正确性；属于高误导风险的 notation drift，而非当前 solver 数值 bug。 | open_doc_sync_needed |
| ISSUE-20260317-eskf-standard-reset-gap | 2026-03-17 | 当前标准 ESKF 路径（`fusion.fej.enable=false`）在 `EskfEngine::Correct` 中会把姿态误差注入四元数，但没有像 `true_iekf` 那样再对协方差施加 reset Gamma；唯一的 reset Jacobian 只存在于 `ApplyTrueInEkfReset()`。这意味着姿态相关互协方差可能继续停留在注入前的误差坐标中。 | `config_data2_baseline_eskf.yaml`; `include/core/eskf.h`; `src/core/eskf_engine.cpp`; `src/core/ins_mech.cpp` | 可同时影响所有依赖姿态互协方差建立约束的状态块，优先怀疑 `bg/sg/sa`，其次是 `mounting/lever`；与当前 `data2 official outage` 中 `bg_*`、`sg_z` 的异常形态相容。 | open_static_audit_ab_needed |

### 已关闭问题摘要

| 主题 | 涉及 issue_id | 已关闭结论 | archive_ref |
|---|---|---|---|
| baseline 指标冲突与 RI 解释闭环 | `ISSUE-001-metric-conflict-baseline-inekf`、`ISSUE-005-baseline-inekf-ri-regression`、`ISSUE-006-ri-process-noise-mapping-incomplete` | baseline 指标冲突已用 best 口径重生，RI 回归与过程噪声映射链已由 fresh 产物闭合。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_issues_from_rows` |
| 量测模型与配置解析修复 | `ISSUE-008-true-gnsspos-r-cov-frame-bug`、`ISSUE-20260311-config-load-fallback`、`ISSUE-20260311-constraint-update-success-propagation`、`ISSUE-20260311-p0diag-dead-config` | GNSS_POS 协方差坐标系、配置 fail-fast、约束 accepted 传播与 `P0_diag` 语义都已修复并有 fresh 证据。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_issues_from_rows` |
| UTF-8 / notebook 输入链路限制 | `ISSUE-20260310-utf8-notebook-stdin` | 已确认 stdin/codepage 会损伤中文内容，后续统一使用 UTF-8 直接写文件。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_issues_from_rows` |
| 启动期基础缺陷（压缩留痕） | `ISSUE-002-manual-init-missing-truth-crash`、`ISSUE-003-gnss-load-nonfatal`、`ISSUE-004-inekf-fej-semantic-conflict` | 这些启动期问题已在更早阶段关闭，但仍保留摘要，避免历史因果链断裂。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_issue_bullets_from_original_walkthrough` |

## 开放假设（Open Hypotheses）

| hyp_id | 假设 | 当前证据 | 下一步验证 | 状态 |
|---|---|---|---|---|
| HYP-1 | data2 InEKF 与 ESKF 差异来自 `GNSS_POS`-RI 耦合与过程噪声映射，而非 `GNSS_VEL`。 | `p_local=on` 时 g0/g1 RMSE3D `1.692779/1.723701`；`p_local=off` 出现灾难性发散 (`4.12e7/1.05e6`)；corr(delta_err3d, delta_hatt) `-0.091572`。 | 需要进一步给出 `GNSS_POS` Jacobian 的时段因果解释（blocker：相关性较弱）。 | open(blocker: mechanistic_causality_missing) |
| HYP-6 | RI 误差约定下，`G(kVel,gyro)` 更合理实现是 `+Skew(v_ned)C_bn`。 | data2 回归：baseline g0 最优但 gnss30 g1 最优；data4 开关短扫最佳 `p0_g1_i0`。 | 解释数据集差异的 RI 开关依赖关系。 | open(conditionally_supported) |
| HYP-7 | true IEKF 的主要收益先体现在 `GNSS_POS` Lie 核心结构修正上；若不继续重写 ODO/NHC、GNSS_VEL 和 `21-30` 扩展块，sparse-GNSS 性能未必改善。 | fresh 对比：data2 baseline RMSE3D `1.723701→1.228326` 且 `H_att_max 2354→2.309982`；data2/data4 GNSS30 退化。后续消融显示 `no_gnss_vel` 基本不改善，而 `no_odo/no_nhc` 会大幅恶化。 | 继续重推 `ODO/NHC` 与 full-state reset，而不是优先处理 `GNSS_VEL`。 | open(very_strongly_supported) |
| HYP-8 | `true_iekf` 在 sparse-GNSS 下的主要 blocker 不是 `GNSS_VEL`，而是 road-constraint 通道及其与 Lie-core propagation/reset 一致性的组合；直接 `ODO/NHC` core Jacobian 形式本身未必是主因。 | `r1/r2` 一致显示：`veljac=eskf` 对 data2/data4 都近乎完全 neutral；`process=eskf` 与 `reset=I` 单独都会大幅退化；联合 case 既不线性叠加、也不比最坏单项更坏（data2/data4 additivity ratio=`0.410/0.502`），说明主失配来自 process/reset 通道且两者强耦合。 | 若继续追 data2 残余 gap，转向检查 `21-30` 扩展块与 `att_z/bg_z` 的 post-GNSS 互协方差演化。 | open(refined_to_process_reset_nonadditivity) |
| HYP-9 | data2 GNSS30 在 phase-2 后仍显著落后于 hybrid，剩余 blocker 更可能是 GNSS 关闭后 `GNSS_VEL`/`21-30` 扩展块/激励条件的 dataset-specific 组合，而不是 `GNSS_POS` 或基础 `ODO/NHC` core Jacobian。 | phase-2 后：data2 GNSS30 RMSE3D `165.591338`，仍高于 hybrid `88.890702`；但 data4 GNSS30 已降至 `22.405382`，说明 phase-2 主修复本身有效。 | 对 data2 做 phase-2b：审查 `GNSS_VEL`、`21-30` 扩展块后GNSS窗口的漂移与互协方差演化。 | open |
| HYP-10 | `data4 GNSS30` 在 phase-2 下显著优于 `data2 GNSS30` 的主要原因不是“算法偏爱 data4”，而是 `data4` 的 post-GNSS 工况更短、更慢、且更强依赖并更能受益于被 phase-2 修正后的 `ODO/NHC` 路径。 | GNSS blackout 后 truth 统计：data2 `1691.8s/20.21km/11.95mps`，data4 `1421.0s/6.66km/4.69mps`；phase-2 精简消融：data2 no_odo/full `2.08x`，no_nhc/full `4.20x`，data4 no_odo/full `28.06x`，no_nhc/full `32.02x`；phase-2b 诊断：split 时速度 data2/data4 `11.13/0.001 mps`，yaw error growth `28.5/3.1 deg`。 | 若需进一步验证，检查 data2/data4 GNSS 关闭后 heading/bias 漂移率与 `21-30` 互协方差演化。 | open(very_strongly_supported) |
| HYP-11 | `data2 GNSS30` 的主残余 blocker 已从 `GNSS_POS/ODO-NHC core Jacobian` 转移为 post-GNSS `bg_z` 主导的 heading drift；`mounting` 漂移会放大位置误差，但不是 yaw growth 的首因；`GNSS_VEL` 缺失不是主导解释。 | `data4` 关掉 GNSS 速度后仍有 `RMSE3D≈19.37m`；phase-2c 中 `freeze_bg` 令 data2 RMSE3D `165.59→98.82`、yaw growth `-28.50→-9.28 deg`；`freeze_bg_mount` 进一步到 `56.03m`，而单独 `freeze_mount` 仅 `145.83m` 且 yaw growth 近乎不变。 | 若要继续修 data2，优先决定是否把 `post_gnss_ablation.disable_gyro_bias`（可选叠加 `disable_mounting`）作为 pragmatic 默认策略，或继续追求更 theory-pure 的 `bg` 过程/更新重构。 | open(validated_by_direct_intervention) |
| HYP-12 | 在当前 phase-2 代码下，剩余 `data2` gap 不太可能来自 `ODO/NHC/GNSS_VEL` measurement Jacobian；若程序链路仍有问题，更可能是 GNSS 期间 `att-bg_z` 约束建立不足，而不是 split 点 reset covariance 数值实现本身。 | Jacobian audit r1/r2 已排除 `ODO/NHC` 与 `GNSS_VEL` measurement Jacobian；新增 reset consistency audit 显示 data2/data4 在 split 点均满足 `Gamma * P_tilde * Gamma^T == P_after_reset`（`rel_fro=max_abs=0`）；同时 split 点 `corr(att_z,bg_z)` data2/data4=`-0.020641/-0.084320`，`|P(att_z,bg_z)|` 约差 `8x`。 | 继续追踪 GNSS 有效窗口内 `P[att,bg_z]`、对应 gain 与 `dx_bg_z` 的时序增长，定位 data2 为何未建立足够的 attitude-bias 约束。 | open(narrowed_to_gnss_coupling_build_up) |
| HYP-13 | 对 `data2 baseline ESKF` 而言，codefix 后的 baseline 漂移里更敏感的不是“约束越高频越好”，而是 `NHC` 原始高频更可能带来过强/过密约束；保留 `ODO` 原始高频、把 `NHC` 适度降到 `20Hz` 左右更有利。 | `EXP-20260311-odo-nhc-rate-sweep-r1`: matched raw→10Hz RMSE3D `1.492452→1.284296`；matrix best `ODO raw + NHC 20Hz`=`1.225207`，优于 raw/raw；而 `ODO 20Hz + NHC raw`=`1.548824` 为最差，说明降 ODO 不利、降 NHC 更可能有益。 | 扩展到 `data2 GNSS30 true_iekf` 与 `data4`，确认该不对称性是 baseline/data2 特有，还是更一般的时序调度规律。 | open(strongly_supported_on_data2_baseline) |
| HYP-18 | 当前 4 个 `GNSS30` canonical cases 的显式 `Q/R` 标量仍未处于局部最优，且最敏感的 noise family 具有明显 case-specific 差异，不能直接外推成一套统一官方配置。 | `EXP-20260313-noise-qr-gnss30-r1` 中 reference 4/4 全部复现成功；best 分别为 `data2 ESKF: R_nhc×4`、`data2 true: Q_imu×0.25 + R_odo×2`、`data4 ESKF: R_odo×4`、`data4 true: Q_calib×0.25 + Q_bias×0.25`。进一步在 `EXP-20260313-dataset-report-cases-r5` 中按 dataset/method 共享这些 profile 后，`9/10` 个 canonical case 指标优于 `r4`，但 `data4_gnss_outage_true_iekf_best` 反而 `1.262719→1.421392 m`，说明 best-profile 的 transferability 也具有 case-specific 边界。 | 对 `r5` 中的 tradeoff case 做 consistency/physics 审计，重点检查 `data4_gnss30_eskf_best` 的 `final_err_3d` 恶化与 `data4_gnss_outage_true_iekf_best` 的整体退化，判断是否要把“共享 GNSS30 best profile”保留为长期正式口径或仅作为 reader-facing 对照。 | open(shared_profile_transferability_mixed) |
| HYP-19 | 在 `data2 + ESKF + corrected RTK + 300s on / 100s off / 150s on` 的 official outage 条件下，大多数非 PVA 扩展状态单独释放后不会稳定收敛；真正可继续保留在线估计的只会是少数状态，而 `gyro bias` block 是会直接恶化导航的主风险。进一步地，shared culprit 并不是“简单把 ODO/NHC 降频就能修好”，而更像 `road-constraint coupling + standard ESKF reset/covariance semantics` 的组合问题。 | `EXP-20260316-data2-state-sanity-r1` 的全矩阵结果为 `18 abnormal / 1 borderline / 3 normal`；`bg_x/bg_y/bg_z` 是仅有的强导航退化主族（`delta_mean_rmse3d=1.072/320.215/4.643`），`sg_z` 也会显著恶化导航。`EXP-20260317-data2-overall-update-attribution-r1` 进一步表明：`standard_reset_gamma` 可把控制组 outage mean/max 从 `12.063/45.714` 改善到 `10.730/38.527`，说明标准 ESKF reset/covariance 语义确有影响；但代表性 released states 仍全部异常。`nhc_off/odo_off` 可把 `release_bg_y` 的绝对 mean 从 `3874.8` 压到 `1662.1/3023.7`，说明 road-constraint 链是错误 `bg` 传播的主要通道；同时控制组会恶化到 `48.058/234.148` 与 `51.269/178.082`，说明它们又是导航必需约束。相反，简单 `20Hz` 降频并不稳健：`nhc_20hz` 把 `release_bg_y` 推到 `548703.8/3865296.1` 的灾难级别，`odo_20hz` 也把 `release_bg_z` 扩大到 `224.47/1180.73`。 | 优先细化 `standard_reset_gamma` 的理论/实现正确性，并对 `ODO/NHC -> bg/sg` 的共享耦合路径做更细的 correction-level 诊断，而不是继续把“简单降频”当作主修复方向。随后再审计 `bg/sg` 的 Markov 语义、等效驱动噪声与后续 `sg/sa/lever/mounting_pitch/yaw` 的符号和 Jacobian 一致性。 | open(refined_to_reset_plus_road_constraint_coupling_no_simple_rate_fix) |
| HYP-21 | 即使移除 `ODO/NHC` 及其强相关状态块（`mounting/odo_lever/odo_scale`），`ba/bg/sg/sa` 也未必会从零初值稳定恢复到参考区间；若如此，则先前“大量状态异常”不只是 road-constraint 本身造成，而是“INS/GNSS 链 recoverability 不足 + road-constraint 进一步把错误状态放大到导航”的组合。 | `EXP-20260317-data2-ins-gnss-state-sanity-r1` 在 `full GNSS + enable_odo=false + enable_nhc=false + disable_mounting/odo_lever/odo_scale=true` 下，控制组导航仅 `0.049/0.019 m`；但 `ba/bg/sg/sa` 中除 `sa_z=borderline` 外仍以 `behavior=abnormal` 为主，说明去掉 road-constraint 后状态 recoverability 问题仍在。与此同时，这些释放 case 的导航多数仍维持厘米级，说明它们更像“弱 recoverability / 弱可观”而不一定直接伤导航。`gnss_lever_x` 为 `normal`，`gnss_lever_y/z` 最终值已接近参考值（recovery ratio `0.136/0.196`），但 `gnss_lever_z` 仍把 absolute nav 提高到 `0.268/0.218 m`，说明 GNSS 杆臂竖直向仍是当前纯 INS/GNSS 下的弱项。 | 下一步转向纯 INS/GNSS 链路本身：记录 `GNSS_POS` 更新在 `bg/sg/gnss_lever` 上的 `K/dx` 时序，并对 `ins_mech` 中 `ba/bg/sg/sa` 的 Markov / scale-factor 过程语义做针对性审计。 | open(strongly_supported_ins_gnss_recoverability_gap) |
| HYP-20 | 当前矩阵里少数“看起来正常”的状态，不一定比其它状态更健康；其中至少 `mounting_roll` 的 `normal` 更可能是半断开建模导致的伪正常，而不是可靠可观。 | `EXP-20260317-overall-update-audit-r1` 显示：state-sanity 使用 `state_series.csv` 评估行为，不存在单纯 `SOL` 读错解释；当前 baseline 中 `GNSS` 不直接约束 `mounting`；`mounting_roll` 虽是正式状态并会被注入，但 `ODO/NHC` 的动态 `C_b_v` 和 Jacobian 只使用 `mounting_pitch/yaw`，不使用 `mounting_roll`。`EXP-20260317-data2-overall-update-attribution-r1` 进一步验证：在 `baseline / standard_reset_gamma / nhc_off / odo_off / nhc_20hz / odo_20hz / standard_reset_gamma_nhc_20hz` 七个变体里，`release_mounting_roll` 始终 `overall=normal` 且 `delta_mean/max=0`。 | 除非后续显式补齐其在 `C_b_v` 与 ODO/NHC Jacobian 中的建模，否则应将 `mounting_roll` 视为“默认 hard-disable 或 compatibility-only state”，不再把它的 `normal` 当作 solver 健康证据。 | validated_by_code_and_experiment |
| HYP-17 | 当 `data4_full_gnss_eskf` 的 `P0_diag` 与 `data4_gnss_outage_eskf_best` 的真实配置口径统一后，`data4` full/outage ESKF 的 `v系航向误差` 会回到同一量级；此前 `r2` 中 full baseline 明显更差，主因是初始化口径不一致，而不是 `GNSS schedule` 或 `GNSS_VEL` 单独造成。 | `EXP-20260313-dataset-report-cases-r4` 中，full/outage ESKF 的末端 `vehicle_heading_err_deg=-2.360/-2.265 deg`，`p95 |vehicle_heading|=2.118/2.173 deg`；同时 `cfg_data4_full_gnss_eskf` 与 `cfg_data4_gnss_outage_eskf_best` 的 `P0_diag`、`gnss_schedule` 一致，仅 `gnss_path/output_path` 不同。 | 后续如需追 residual gap，直接比较 `outage_eskf` 与 `outage_true_iekf` 的算法差异即可，不再把 `r2` 的 full/outage 大 gap 当作官方现象。 | validated_r4_official_alignment |

### 已解决/已否决假设摘要

| 主题 | 涉及 hyp_id | 结论 | 当前影响 | archive_ref |
|---|---|---|---|---|
| 外参冻结与可观性补证 | `HYP-3` | 外参相关状态对调度与激励敏感已被冻结矩阵、漂移图和 mounting_roll 证据链确认。 | 当前只需跨数据集复验，不再是开放 blocker。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_hypotheses_from_rows` |
| NHC 最优频率具有 dataset boundary | `HYP-14`、`HYP-15` | data2 上的 `30 Hz` / `0.75 Hz` 最优频率不能直接迁移到 data4，必须 per-dataset local sweep。 | 已成为当前 canonical report 的配置规则。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_hypotheses_from_rows` |
| 当前 codepath 的 InEKF 机制归因 | `HYP-16` | 当前 codepath 下的主要收益来自 process/reset consistency，而非 measurement Jacobian 写法本身。 | 后续机理分析聚焦 `att_z-bg_z` 与 `21-30` 协方差演化。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_hypotheses_from_rows` |
| 已否决的单因子解释 | `HYP-2`、`HYP-4`、`HYP-5` | `p_ned_local` 单因子、单补 `G(kVel,gyro)` 等解释已被后续 fresh 证据否决。 | 提醒后续不要回退到已排除的简单叙事。 | `docs/project_history/walkthrough_archive_registry_20260313.md :: archived_hypothesis_bullets_from_original_walkthrough` |

## 会话日志（Session Log）

### 阶段摘要

- 阶段A `InEKF 初版重写与 best switch 锁定`：从 FEJ 依赖版转到 RI 实现，并把 best switch 锁定为后续统一口径。
- 阶段B `true_iekf 分支与 phase-2 主修复`：完成 tex-code audit、Lie 核心 Jacobian / reset 重写，以及 measurement Jacobian 数值审计。
- 阶段C `data2 post-GNSS 因果链`：冻结矩阵、state21-30 漂移图与 bg/mounting 干预把 data2 剩余 blocker 收敛到 heading drift。
- 阶段D `data2 GNSS30 NHC 频率扫描链`：把 ODO/NHC interval audit、smoke 和正式 sweep 收敛成算法分开的最优频率结论。
- 阶段E `data4 P0_diag 修正与 canonical r2`：定位 old/new data4 ESKF 巨大差异来自初始化协方差语义，并用 r2 sweep / canonical cases 重建正式结果。
- 阶段F `读者版报告与仓库整理`：reader-facing dataset report 已收敛，当前继续把主 walkthrough 和历史产物改成“工作台 + 历史摘要”。
- 原始阶段摘要全文已移至 `docs/project_history/walkthrough_archive_sessions_20260313.md :: phase_summary_original`。

### 当前关键会话

### session_id: 20260311-1022-project-code-review

- timestamp: 2026-03-11 10:22 (local)
- objective: 对整个项目工程做一次代码审查，优先识别会影响数值行为、实验复现和可观性结论可信度的问题。
- scope:
  - 审阅构建入口、核心融合链路、配置解析、初始化、诊断与代表性分析脚本。
  - 重点核对 `config`/`pipeline_fusion`/`diagnostics`/`initialization` 等高风险路径。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_gnss30_true_iekf.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `git status --short --branch`
  - `rg --files -g "*.cpp" -g "*.h" -g "*.hpp" -g "*.py" -g "CMakeLists.txt" -g "*.md" src include apps scripts`
  - `cmake --build build --config Release --target eskf_fusion jacobian_audit`
  - `rg -n "P0_diag|std_pos|std_vel|std_att" -g "config*.yaml" -g "test_config.yaml"`
  - `Get-Content src/app/{config.cpp,initialization.cpp,pipeline_fusion.cpp,diagnostics.cpp,evaluation.cpp}`
  - `Get-Content src/core/{eskf_engine.cpp,ins_mech.cpp,measurement_models_uwb.cpp}`
- artifacts:
  - `build/Release/eskf_fusion.exe`
  - `build/Release/jacobian_audit.exe`
- metrics:
  - Release build: `eskf_fusion` / `jacobian_audit` 编译通过。
  - Code review 主 findings: `3`。
- artifact_mtime:
  - `build/Release/eskf_fusion.exe`: fresh in session
  - `build/Release/jacobian_audit.exe`: fresh in session
- config_hash_or_mtime:
  - 本会话未改动 solver 配置；仅审阅现有 `config_data2_baseline_eskf.yaml` 与 `config_data2_gnss30_true_iekf.yaml`。
- dataset_time_window:
  - N/A（未新增 solver 运行，仅静态审查与构建验证）。
- result_freshness_check:
  - pass（构建在本会话 fresh 完成；findings 直接绑定当前源码行号与现有配置内容）。
- observability_notes:
  - 本次审查未新增状态块 `21-30` 的性能结论，但确认 `P0_diag` 死配置会影响对初始协方差相关可观性实验的解释。
  - 约束更新成功状态未向上传递的问题，会污染 `ODO/NHC` 接受率与最小更新间隔相关诊断，间接影响可观性归因日志可信度。
- decision:
  - 当前工程最值得优先修复的是：配置加载失败必须 hard fail、约束更新成功状态必须显式向上传递、`P0_diag` 需要与 `std_*` 二选一并清理文档/配置。
  - 代表性分析脚本未发现比上述更高优先级的交付风险；当前主要问题集中在核心 runtime/config 路径。
- next_step:
  - 先修 `DiagnosticsEngine::Correct` / `CorrectConstraintWithRobustness` 的成功返回链路，并补最小回归测试；随后修 `LoadFusionOptions/LoadGeneratorOptions` 的 fail-fast，再统一清理 `P0_diag` 语义与相关配置/文档。

### session_id: 20260311-1045-code-review-fixes-r1

- timestamp: 2026-03-11 10:45 (local)
- objective: 修复 `20260311-1022-project-code-review` 发现的 3 个问题，并做 build/smoke/regression 验证及 old/new 解算结果对比。
- scope:
  - 修改 `config.cpp` / `uwb_generator_main.cpp`，让 fusion/generator 配置缺失或无效时 hard fail，并统一 `--config` 入口。
  - 修改 `diagnostics.cpp` / `pipeline_fusion.cpp` / `diagnostics.h`，把约束更新成功状态显式向上传递，只在 real update success 时统计 accepted 和推进最小更新间隔时间戳。
  - 修改 `initialization.cpp` / `fusion.h` / `config.cpp`，让显式 `P0_diag` 优先于 `std_*`；补充 `regression_checks` 与 `prompt4.md` 的 31 维语义。
- changed_files:
  - `CMakeLists.txt`
  - `apps/regression_checks_main.cpp`
  - `apps/uwb_generator_main.cpp`
  - `include/app/diagnostics.h`
  - `include/app/fusion.h`
  - `src/app/config.cpp`
  - `src/app/diagnostics.cpp`
  - `src/app/initialization.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `docs/prompts/prompt4.md`
  - `output/review/EXP-20260311-codefix-r1/compare_summary.md`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260311-codefix-r1/cfg_data2_baseline_eskf.yaml`
  - `output/review/EXP-20260311-codefix-r1/cfg_data2_gnss30_true_iekf.yaml`
- commands:
  - `cmake --build build --config Release --target regression_checks`
  - `cmake --build build --config Release --target eskf_fusion uwb_generator jacobian_audit`
  - `build\Release\regression_checks.exe`
  - `build\Release\eskf_fusion.exe --config __missing_config__.yaml`
  - `build\Release\uwb_generator.exe --config __missing_generator_config__.yaml`
  - `build\Release\eskf_fusion.exe --config output/review/EXP-20260311-codefix-r1/cfg_data2_baseline_eskf.yaml`
  - `build\Release\eskf_fusion.exe --config output/review/EXP-20260311-codefix-r1/cfg_data2_gnss30_true_iekf.yaml`
  - `python -`（old/new 轨迹差分与对真值 RMSE 对比）
- artifacts:
  - `output/review/EXP-20260311-codefix-r1/SOL_data2_baseline_eskf_codefix.txt`
  - `output/review/EXP-20260311-codefix-r1/SOL_data2_gnss30_true_iekf_codefix.txt`
  - `output/review/EXP-20260311-codefix-r1/compare_summary.md`
  - `build/Release/regression_checks.exe`
- metrics:
  - `regression_checks: PASS`
  - `eskf_fusion --config __missing_config__.yaml`: exit `1`，输出 `运行失败: error: 无法读取配置文件 __missing_config__.yaml (bad file: __missing_config__.yaml)`
  - `uwb_generator --config __missing_generator_config__.yaml`: exit `1`，输出 `运行失败: error: 无法读取配置文件 __missing_generator_config__.yaml (bad file: __missing_generator_config__.yaml)`
  - data2 baseline ESKF 对真值 RMSE xyz / 3D：`0.763591 0.565924 0.770052 / 1.223243` → `0.827032 0.877094 0.879851 / 1.492452`
  - data2 GNSS30 true_iekf 对真值 RMSE xyz / 3D：`87.886377 80.638180 112.915649 / 164.245168` → `88.403667 79.441248 115.230733 / 165.542267`
  - old/new 轨迹差分 RMSE3D：baseline ESKF `0.706109 m`，GNSS30 true_iekf `6.496947 m`
- artifact_mtime:
  - `output/review/EXP-20260311-codefix-r1/compare_summary.md`: `2026-03-11 10:46:16`
  - `output/review/EXP-20260311-codefix-r1/SOL_data2_baseline_eskf_codefix.txt`: `2026-03-11 10:37:49`
  - `output/review/EXP-20260311-codefix-r1/SOL_data2_gnss30_true_iekf_codefix.txt`: `2026-03-11 10:38:07`
- config_hash_or_mtime:
  - `output/review/EXP-20260311-codefix-r1/cfg_data2_baseline_eskf.yaml`: `2026-03-11 10:36:02`
  - `output/review/EXP-20260311-codefix-r1/cfg_data2_gnss30_true_iekf.yaml`: `2026-03-11 10:36:02`
- dataset_time_window:
  - data2: `528076.0 -> 530488.9`
- result_freshness_check:
  - pass（本会话 fresh 生成 `compare_summary.md`，并重新执行 `regression_checks` 与 missing-config smoke tests；solver fresh 输出来自同一会话的 `EXP-20260311-codefix-r1` 产物）
- observability_notes:
  - 本次修复未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*`、`fusion.gnss_schedule.*` 的配置语义，也未新增 `21-30` 状态块的性能结论。
  - 但 `P0_diag` 优先级修复恢复了初始协方差相关实验的可解释性；约束成功状态修复则提升了 `ODO/NHC` accepted 统计与最小更新间隔诊断的可信度。
- decision:
  - code review 的 3 个主问题均已修复，并通过 build/smoke/regression 行为验证。
  - 修复并非“数值完全中性”：baseline ESKF RMSE3D 增加 `0.269210 m`，data2 GNSS30 true_iekf RMSE3D 增加 `1.297099 m`；其中 GNSS30 相对影响较小，但 baseline 漂移已足够值得继续审计。
- next_step:
  - 先对 `EXP-20260311-codefix-r1` 的 baseline ESKF 打开 `enable_consistency_log` 做前后对比，确认 RMSE 漂移是否来自被修正的 `ODO/NHC` 成功统计与 min-update-interval 行为。

### session_id: 20260312-1353-inekf-mechanism-r2-complete

- timestamp: 2026-03-12 13:53 (local)
- objective: 继续并完成 `r2` 机制实验，拿到联合消融与 `data4` minimal ported rate-vs-weight 的 fresh 结果。
- scope:
  - 在不改求解器代码、不重编译的前提下，直接重跑 `python scripts\analysis\inekf_mechanism_attribution.py`。
  - 完整执行 20 个 case：14 个 attribution case + 12 个 rate case 中的去重总集。
  - 用 fresh `metrics.csv / summary.md / mechanism_math_note.md` 更新实验索引和相关假设状态。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_gnss30_eskf_nofreeze.yaml`
  - `config_data2_gnss30_true_iekf.yaml`
  - `config_data4_gnss30_eskf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/cases/data{2,4}/cfg_*.yaml`
- commands:
  - `Get-Content walkthrough.md | Select-Object -Last 140`
  - `Get-Item build\Release\eskf_fusion.exe | Select-Object LastWriteTime,Length`
  - `python scripts\analysis\inekf_mechanism_attribution.py`
  - `Get-Content output\review\EXP-20260312-inekf-mechanism-attribution-r2\summary.md`
  - `Get-Content output\review\EXP-20260312-inekf-mechanism-rate-vs-weight-r2\summary.md`
- artifacts:
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/metrics.csv`
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/summary.md`
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/mechanism_math_note.md`
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/per_update_stats.csv`
  - `output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/metrics.csv`
  - `output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/summary.md`
  - `output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/per_update_stats.csv`
- metrics:
  - attribution joint case:
    data2 base/proc/reset/combo RMSE3D=`21.656987/558.636121/350.215138/376.955771` m，heading slope=`-1.480531/67.828521/43.661503/8.831406` deg/ks；
    data4 base/proc/reset/combo RMSE3D=`12.011497/320.995023/284.796149/303.785137` m，heading slope=`-1.207022/16.220964/-23.721211/14.134621` deg/ks。
  - joint-case non-additivity:
    additivity ratio data2/data4=`0.410/0.502`；联合 case 都没有比最坏单项更坏，说明 `process` 与 `reset` 的退化是强耦合而非线性叠加。
  - rate-vs-weight boundary:
    data4 ESKF raw/30Hz/raw_weight=`281.328288/302.543170/241.276241` m；
    data4 true raw/0.75Hz/raw_weight=`12.011497/33.248591/35.053552` m。
- artifact_mtime:
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/metrics.csv`: `2026-03-12 13:53:08`
  - `output/review/EXP-20260312-inekf-mechanism-attribution-r2/summary.md`: `2026-03-12 13:53:08`
  - `output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/metrics.csv`: `2026-03-12 13:53:09`
  - `output/review/EXP-20260312-inekf-mechanism-rate-vs-weight-r2/summary.md`: `2026-03-12 13:53:09`
- config_hash_or_mtime:
  - 本会话未改 base config；派生 `cfg_*.yaml` 均由脚本在 `r2` 目录下 fresh 生成。
  - `build\Release\eskf_fusion.exe` 复用 `2026-03-12 10:08:53` 的既有 fresh build，运行期间未重编译。
- dataset_time_window:
  - data2 GNSS30 base IMU window: `527639.203184 -> 530492.825839`
  - data4 GNSS30 base IMU window: `274900.204914 -> 277339.018817`
  - 两组均使用 `fusion.gnss_schedule.head_ratio = 0.30`
- result_freshness_check:
  - pass（`r2` 的 attribution 与 rate 汇总文件均在本会话完成后重写，20 个目标 case 均已 fresh 跑完）
- observability_notes:
  - 本会话未新增 `fusion.ablation.*` / `fusion.post_gnss_ablation.*`，仍只操作 `fusion.fej.debug_*` 与 `constraints.sigma_nhc_* / nhc_min_update_interval`。
  - `veljac=eskf` 继续对 data2/data4 结果保持 neutral，进一步排除了 `ODO/NHC` Lie-core Jacobian 形式是主因。
  - `process=eskf` 与 `reset=I` 的联合 case 显示退化强耦合但非线性叠加，说明 true_iekf 的收益并不是“两项独立小改动简单相加”。
  - `data4` minimal ported test 给出边界：data2 的最优 NHC 调度不能直接迁移到 data4；其中 true_iekf 的“减弱信息注入”方向仍保持一致，而 ESKF 则表现出明显的数据集依赖。
- decision:
  - `r2` 进一步坐实了当前 codepath 的主结论：true_iekf 的收益主导项是 Lie-core process consistency + reset Gamma consistency，而不是 `ODO/NHC` Jacobian 写法本身。
  - 对 `data2`，`NHC` 的 schedule/weight 等价解释仍成立；但扩展到 `data4` 后，这个结论必须带上边界条件，不能再写成“跨数据集普适”。
- next_step:
  - 把 `r2` 的联合消融与 data4 boundary 结论同步进 `算法文档/可观性分析讨论与InEKF算法.tex` 与 HTML 报告。
  - 若后续还要继续追 data2 残余 gap，再转向 `21-30` 扩展块与 `att_z/bg_z` 互协方差的 post-GNSS 演化分析。

### session_id: 20260312-1626-dataset-partitioned-report-r1

- timestamp: 2026-03-12 16:26 (local)
- objective: 完成按 `data2/data4` 重组的新 HTML 报告链路，补做 `data4 GNSS30` 本地 NHC sweep、固化 10 个 canonical cases，并核对 `data4_gnss30_eskf` 的 best NHC 是否确为 `5Hz`。
- scope:
  - 在 `interactive_nav_report.py` 中抽出通用 section 构建接口，新增 dataset-partitioned 报告构建器与专用 HTML 导出器。
  - 新增 orchestration 脚本，复用/执行 `data4 GNSS30` fresh sweep，生成 `data2/data4` 各 5 个 canonical cases、两份 `GNSS_outage_cycle` 文件、统一 `metrics.csv/summary.md/manifest.json`，并对全部最终案例导出状态图。
  - 重新导出 `output/html/dataset_partitioned_navigation_interactive_report.html`，验证页面结构、去重状态图与新鲜度；并额外核对 `data4_gnss30_eskf` 的 `raw` vs `5Hz` 指标，确认 best-case 选择无误。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/filter_gnss_outage.py`
  - `scripts/analysis/generate_dataset_report_cases.py`
  - `scripts/analysis/dataset_partitioned_nav_report.py`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_gnss30_eskf.yaml`
  - `config_data2_gnss30_true_iekf.yaml`
  - `config_data4_baseline_eskf.yaml`
  - `config_data4_gnss30_eskf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r1/cfg_data{2,4}_*.yaml`
- commands:
  - `python scripts/analysis/generate_dataset_report_cases.py`
  - `python -m py_compile scripts/analysis/interactive_nav_report.py scripts/analysis/filter_gnss_outage.py scripts/analysis/generate_dataset_report_cases.py scripts/analysis/dataset_partitioned_nav_report.py scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 1200 --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - `Select-String output\html\dataset_partitioned_navigation_interactive_report.html -Pattern '实验说明|data2|data4|状态量变化图|点击跳转到首次展示位置'`
  - `Import-Csv output\review\EXP-20260312-data4-gnss30-nhc-sweep-r1\metrics.csv | Where-Object { $_.scenario -eq 'data4_gnss30_eskf' -and ($_.case -eq 'matched_odo_raw_nhc_raw' -or $_.case -eq 'fixed_odo_nhc_sweep_odo_raw_nhc_5hz') }`
- artifacts:
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r1/{metrics.csv,summary.md,manifest.json}`
  - `output/review/EXP-20260312-dataset-report-cases-r1/{metrics.csv,summary.md,manifest.json,GNSS_outage_cycle_data2.txt,GNSS_outage_cycle_data4.txt,SOL_*.txt,*.log}`
  - `output/result_data2_report_*/*.png`
  - `output/result_data4_report_*/*.png`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
- metrics:
  - `data4_gnss30_eskf` fresh sweep: `raw=281.328288` m，`5Hz=145.233540` m，因此本轮 best-case 不是 worse-than-raw，而是相对 raw 改善约 `-136.094748` m。
  - `data4_gnss30_true_iekf` fresh sweep: `raw=12.011497` m，且优于 fixed sweep 最优 `50Hz=22.020263` m，因此 data4 true 路径的最优 NHC 与 ESKF 不同。
  - canonical cases RMSE3D：
    data2=`1.492452/101.122603/21.656987/6.139390/1.186893`，
    data4=`1.229320/145.233540/12.011497/5.944614/1.262719`
    （顺序均为 `full / gnss30_eskf / gnss30_true / outage_eskf / outage_true`）。
- artifact_mtime:
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r1/metrics.csv`: `2026-03-12 16:01:23`
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r1/summary.md`: `2026-03-12 16:01:23`
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r1/manifest.json`: `2026-03-12 16:01:24`
  - `output/review/EXP-20260312-dataset-report-cases-r1/metrics.csv`: `2026-03-12 16:15:06`
  - `output/review/EXP-20260312-dataset-report-cases-r1/summary.md`: `2026-03-12 16:15:06`
  - `output/review/EXP-20260312-dataset-report-cases-r1/manifest.json`: `2026-03-12 16:15:06`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-12 16:26:29`
- config_hash_or_mtime:
  - 本会话未改 solver 核心，仅新增/修改分析与报告脚本，并派生报告专用 `cfg_*.yaml`。
  - `data2` 复用已锁定最优 NHC：ESKF `30Hz`、`true_iekf 0.75Hz`；`data4` fresh sweep 重新确定：ESKF `5Hz`、`true_iekf raw`。
- dataset_time_window:
  - data2 outage template window: `528076.000 -> 530488.900`
  - data4 outage template window: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（新 HTML、canonical cases 汇总与 `data4` fresh sweep 均在 2026-03-12 生成；新 HTML 未检出 `2026-03-05` 旧 data4 主回归引用）
- observability_notes:
  - 本会话未对 `21-30` 外参/尺度状态块施加新的 `fusion.ablation.*` 或 `fusion.post_gnss_ablation.*` 冻结；这些状态在 10 个 canonical cases 中均保持自由估计。
  - 传感器窗口按 3 类固定调度对比：`full GNSS`、`GNSS30`（head `30%`）和 `GNSS outage cycle`（`900s` 收敛后 `300s on / 120s off`）。
  - 行为上，`data2/data4` 的 `true_iekf` 在 GNSS30 与 outage canonical cases 中都优于对应 ESKF；但 `data4` 的 NHC 最优点与 data2 明显不同，ESKF 需要降到 `5Hz`，而 `true_iekf` 在 data4 上反而保持 `raw` 最优。
- decision:
  - 新报告链路已完成并独立输出到 `output/html/dataset_partitioned_navigation_interactive_report.html`，可以作为后续文档与汇报的正式新口径。
  - `data4_gnss30_eskf` 的 best NHC 选择没有选反：同一 fresh sweep 中 `5Hz` 明显优于 `raw`；如果阅读时产生“还不如默认频率”的印象，多半是把 `full GNSS` 页面数值或 `true_iekf raw` 最优结果混到了 `ESKF GNSS30` 对照里。
  - 需要在后续文档中更明确地区分“算法内 sweep 最优”与“跨实验页比较”，否则 data4 这组 mixed optimum 容易被误读。
- next_step:
  - 把 `EXP-20260312-data4-gnss30-nhc-sweep-r1` 的 raw-vs-best 摘要表或 sweep 小图补进新 HTML/`.tex`，专门解释为什么 `data4 ESKF best=5Hz` 而 `data4 true_iekf best=raw`。
  - 继续把 canonical cases 与 `EXP-20260312-inekf-mechanism-attribution-r2` 汇合，围绕 `v系航向误差`、`P(att_z,bg_z)` 与 `21-30` 互协方差做 data2/data4 差异解释。

### session_id: 20260312-2216-data4-p0diag-root-cause-check

- timestamp: 2026-03-12 22:16 (local)
- objective: 验证 `data4_gnss30_eskf` 从旧 `51.37m` 到当前 `145.23/281.33m` 的主因是否来自 `P0_diag` 语义变化，而不是 NHC/ODO 更新统计或 IMU 配置差异。
- scope:
  - 审阅 `src/app/config.cpp` 与 `src/app/initialization.cpp` 中 `P0_diag` 的解析与初始化逻辑。
  - 基于当前 `config_data4_gnss30_eskf.yaml` 派生临时配置，仅移除 `P0_diag`，保留其余参数与数据路径不变。
  - 用当前代码 fresh 复跑 `data4_gnss30_eskf`，并与 2026-03-05 历史 `gnss30_eskf.log` 及 2026-03-12 current-code `matched_odo_raw_nhc_raw.log` 做对照。
- changed_files:
  - `output/review/EXP-20260312-data4-p0diag-check-r1/cfg_data4_gnss30_eskf_stdP0.yaml`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/SOL_data4_gnss30_eskf_stdP0.txt`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/data4_gnss30_eskf_stdP0.stdout.txt`
  - `walkthrough.md`
- configs:
  - `config_data4_gnss30_eskf.yaml`
  - `output/review/EXP-20260305-data4-main4-regression-r1/cfg_gnss30_eskf.yaml`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/cfg_data4_gnss30_eskf_stdP0.yaml`
- commands:
  - `Select-String src/app/config.cpp,src/app/initialization.cpp -Pattern 'has_custom_P0_diag|P0_diag'`
  - `build\\Release\\eskf_fusion.exe --config output\\review\\EXP-20260312-data4-p0diag-check-r1\\cfg_data4_gnss30_eskf_stdP0.yaml`
  - `Select-String output\\review\\EXP-20260312-data4-p0diag-check-r1\\data4_gnss30_eskf_stdP0.stdout.txt -Pattern 'P0 source|Consistency\\] NHC seen|Consistency\\] ODO seen|RMSE \\(融合\\)|结果已保存到'`
  - `Select-String output\\review\\EXP-20260305-data4-main4-regression-r1\\gnss30_eskf.log -Pattern 'Consistency\\] NHC seen|Consistency\\] ODO seen|RMSE \\(融合\\)'`
  - `Select-String output\\review\\EXP-20260312-data4-gnss30-nhc-sweep-r1\\data4_gnss30_eskf\\matched_odo_raw_nhc_raw.log -Pattern 'Consistency\\] NHC seen|Consistency\\] ODO seen|RMSE \\(融合\\)'`
- artifacts:
  - `output/review/EXP-20260312-data4-p0diag-check-r1/cfg_data4_gnss30_eskf_stdP0.yaml`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/SOL_data4_gnss30_eskf_stdP0.txt`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/data4_gnss30_eskf_stdP0.stdout.txt`
  - `output/review/EXP-20260305-data4-main4-regression-r1/gnss30_eskf.log`
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r1/data4_gnss30_eskf/matched_odo_raw_nhc_raw.log`
- metrics:
  - old historical run (`2026-03-05`): RMSE xyz=`30.292/19.365/36.694`，RMSE3D=`51.371764`；NHC accepted=`398134`，ODO accepted=`396812`。
  - current code + custom `P0_diag`: RMSE xyz=`145.584/94.658/221.339`，RMSE3D=`281.328617`；NHC accepted=`397928`，ODO accepted=`396809`。
  - current code + removed `P0_diag` (`std_* P0`): RMSE xyz=`30.294/19.365/36.696`，RMSE3D=`51.374372`；NHC accepted=`398134`，ODO accepted=`396812`。
  - 结论：只要恢复旧的 `std_*` 初始化语义，当前代码即可几乎逐位复现旧 `51.37m`；约束接受统计也同步回到旧值。
- artifact_mtime:
  - `output/review/EXP-20260312-data4-p0diag-check-r1/cfg_data4_gnss30_eskf_stdP0.yaml`: `2026-03-12 22:15:08`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/SOL_data4_gnss30_eskf_stdP0.txt`: `2026-03-12 22:15:57`
  - `output/review/EXP-20260312-data4-p0diag-check-r1/data4_gnss30_eskf_stdP0.stdout.txt`: `2026-03-12 22:15:57`
- config_hash_or_mtime:
  - `src/app/config.cpp` 当前在 `ParseInitNode` 中对显式 `P0_diag` 设置 `has_custom_P0_diag=true`。
  - `src/app/initialization.cpp` 当前优先使用 `has_custom_P0_diag`，仅在未显式提供时才从 `std_*` 构造 `P0`。
  - `config_data4_gnss30_eskf.yaml` 中 `P0_diag` 与 `std_*` 差异显著：
    `att` 方差 `0.0001` vs `std_att` 导出的 `3.05e-06`，
    `mount_pitch` `0.09` vs `7.62e-05`，
    `mount_yaw` `0.01` vs `2.74e-05`。
- dataset_time_window:
  - data4 GNSS30 window: `275309.0 -> 277300.0`
  - GNSS 调度: `head_ratio = 0.3`
- result_freshness_check:
  - pass（`EXP-20260312-data4-p0diag-check-r1` 全部产物均在本会话 fresh 生成）
- observability_notes:
  - 本会话未改动观测模型或调度，仅改变初始协方差构造语义。
  - `P0_diag` 对姿态与安装角状态块赋予远大于 `std_*` 的初始方差，尤其 `mount_pitch/mount_yaw`，这会改变 GNSS 头 30% 窗口内外参/姿态相关状态的早期收敛路径。
  - 在 `data4_gnss30_eskf` 中，旧结果与 `std_* P0` 对齐，说明这里的性能差异主要是初始化协方差语义，而不是可观性实验本身的 schedule 或约束频率。
- decision:
  - 根因已定位：旧 `51.37m` 与新 `145.23/281.33m` 的主差异来自 `P0_diag` 修复，而不是 `NHC` best 选择错误。
  - 旧结果本质上对应“`P0_diag` 未生效、实际使用 `std_*` 初始化”的历史语义；新结果对应“`P0_diag` 真正生效”的当前语义。
  - 对 `data4_gnss30_eskf` 的最终结论，需要二选一统一口径：要么接受当前 `P0_diag` 语义并更新旧报告，要么修改配置文件中的 `P0_diag` 使其与期望的 `std_*` 水平一致。
- next_step:
  - 决定 `config_data4_gnss30_eskf.yaml` 是否保留当前 `P0_diag`；若不保留，应按 `std_*` 或重新设计的 `P0_diag` 重跑 data4 canonical cases 与 sweep。
  - 无论是否重跑，都应在 `representative_navigation_interactive_report.html` 和 `.tex` 中明确旧 `51.37m` 对应的是“旧 `P0_diag` 语义未生效”的历史结果。

### session_id: 20260312-2230-data4-p0diag-rerun-and-html-refresh

- timestamp: 2026-03-12 23:00 (local)
- objective: 将 `config_data4_gnss30_eskf.yaml` 的 `P0_diag` 改写为 `std_*` 等效方差，fresh 重跑受影响的 data4 sweep / canonical cases，并刷新新分组 HTML 与实验说明页中的关键参数配置。
- scope:
  - 重写 `config_data4_gnss30_eskf.yaml` 中 31 维 `P0_diag`，使其与 `std_pos/std_vel/std_att/std_bg/std_mounting_*` 等效。
  - 修改 `scripts/analysis/dataset_partitioned_nav_report.py`，使其读取最新 `EXP-*-dataset-report-cases-r*` manifest，并在实验说明页新增 data2/data4 关键参数配置摘要。
  - 修改 `scripts/analysis/export_dataset_partitioned_nav_report_html.py` 的说明页代码样式，便于展示配置路径。
  - 用 fresh `r2` 目录重跑 `data4 GNSS30` sweep、10 个 canonical cases 与 HTML 导出。
- changed_files:
  - `config_data4_gnss30_eskf.yaml`
  - `scripts/analysis/dataset_partitioned_nav_report.py`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `walkthrough.md`
- configs:
  - `config_data4_gnss30_eskf.yaml`
  - `config_data4_gnss30_true_iekf.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data{2,4}_*.yaml`
- commands:
  - `python -m py_compile scripts/analysis/dataset_partitioned_nav_report.py scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python scripts/analysis/generate_dataset_report_cases.py --sweep-outdir output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2 --cases-outdir output/review/EXP-20260312-dataset-report-cases-r2`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 1200 --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - `python - <<'PY' ...` / PowerShell 检查：确认 HTML 引用 `r2`、`gallery-card=10`、`gallery-ref=4`，并核对 data4 best 行为与状态图目录完整性。
- artifacts:
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/{metrics.csv,summary.md,manifest.json,plots/*,data4_gnss30_*/*}`
  - `output/review/EXP-20260312-dataset-report-cases-r2/{metrics.csv,summary.md,manifest.json,GNSS_outage_cycle_data2.txt,GNSS_outage_cycle_data4.txt,SOL_*.txt,*.log}`
  - `output/result_data2_report_*/*.png`
  - `output/result_data4_report_*/*.png`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
- metrics:
  - `data4_gnss30_eskf` fresh sweep：raw=`51.373795` m，且即 best；fixed sweep 最优 `100Hz=61.005053` m。
  - `data4_gnss30_true_iekf` fresh sweep：raw=`12.011497` m，且即 best；fixed sweep 最优 `50Hz=22.020263` m。
  - canonical cases RMSE3D：
    data2=`1.492452/101.122603/21.656987/6.139390/1.186893`，
    data4=`1.229320/51.373795/12.011497/1.279736/1.262719`
    （顺序均为 `full / gnss30_eskf / gnss30_true / outage_eskf / outage_true`）。
  - `data4_gnss30_eskf_best` 接受统计回到旧语义量级：ODO accepted=`396812`，NHC accepted=`398134`；与 `EXP-20260312-data4-p0diag-check-r1` 的 `std_*` 复现实验一致。
  - 10 个 canonical cases 的状态图目录均存在且各含 `12` 张 PNG（当前 `plot_navresult.py` 额外输出 `02b_velocity_vehicle_v.png`）。
- artifact_mtime:
  - `config_data4_gnss30_eskf.yaml`: `2026-03-12 22:30:02`
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/metrics.csv`: `2026-03-12 22:48:22`
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/summary.md`: `2026-03-12 22:48:22`
  - `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/manifest.json`: `2026-03-12 22:48:22`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`: `2026-03-12 22:56:26`
  - `output/review/EXP-20260312-dataset-report-cases-r2/summary.md`: `2026-03-12 22:56:26`
  - `output/review/EXP-20260312-dataset-report-cases-r2/manifest.json`: `2026-03-12 22:56:26`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-12 22:58:03`
- config_hash_or_mtime:
  - `config_data4_gnss30_eskf.yaml` 已将 `P0_diag` 改为 `std_*` 等效方差：`att=3.046e-06`、`bg=5.429e-08`、`mount_roll/pitch=7.615e-05`、`mount_yaw=2.742e-05`。
  - `scripts/analysis/dataset_partitioned_nav_report.py` 现在按 `r*` 自动选择最新 canonical cases manifest，并在说明页读各案例 `cfg_*.yaml` 生成 `P0 source / P0 摘要 / ODO / NHC` 配置说明。
- dataset_time_window:
  - data2 window: `528076.000 -> 530488.900`
  - data4 window: `275309.000 -> 277300.000`
  - outage template: `900s` GNSS on convergence + `300s on / 120s off`
- result_freshness_check:
  - pass（`r2` sweep / canonical cases / HTML 均在本会话 fresh 生成；HTML 已确认包含 `EXP-20260312-dataset-report-cases-r2`，`gallery-card=10`、`gallery-ref=4`，状态图目录完整）
- observability_notes:
  - 本会话未新增 `fusion.ablation.*` 或 `fusion.post_gnss_ablation.*`，也未改变 GNSS30 / outage 调度模板；唯一算法相关变更是 `data4_gnss30_eskf` 的初始化协方差语义。
  - 被直接影响的状态块主要是 `att(6:8)`、`bg(12:14)` 与 `mounting(22:24)` 的初始方差；修正后 `data4_gnss30_eskf` 的 ODO/NHC 接受统计与旧 `std_*` 语义一致，说明先前 `145/281m` 的异常主要来自初始化而不是约束频率最优点本身。
  - 在修正后的 data4 口径下，`GNSS30` 与 `outage` 的 ESKF / `true_iekf` 都使用 `ODO raw + NHC raw` 作为本地最优频率设置。
- decision:
  - `data4_gnss30_eskf` 的官方配置现统一为“显式 `P0_diag`，但数值与 `std_*` 等效”；基于该配置的 fresh `r2` 结果重新回到 `RMSE3D=51.373795` m。
  - `EXP-20260312-data4-gnss30-nhc-sweep-r2`、`EXP-20260312-dataset-report-cases-r2` 与 `output/html/dataset_partitioned_navigation_interactive_report.html` 现在是 data4 ESKF GNSS30 的正式新口径，应替代 `r1` 中受旧 `P0_diag` 影响的 `145.23/281.33` 结论。
  - 新 HTML 的实验说明页已补入两组数据的关键参数配置，包含初始协方差摘要、`P0` 来源说明以及各案例实际 ODO/NHC 更新频率。
- next_step:
  - 刷新 `representative_navigation_interactive_report.html` 与 `.tex` 文档，使其明确引用 `EXP-20260312-data4-gnss30-nhc-sweep-r2` / `EXP-20260312-dataset-report-cases-r2` 的 fresh data4 ESKF GNSS30 结果。
  - 在机理分析里继续围绕 `v系航向误差`、`att_z-bg_z` 互协方差与 `21-30` 状态块，解释为什么修正初始化后 data4 ESKF 的 NHC 最优重新回到 raw。

### session_id: 20260313-0034-dataset-report-reader-copy

- timestamp: 2026-03-13 00:39 (local)
- objective: 在不重跑实验的前提下，把 `dataset_partitioned_navigation_interactive_report.html` 的“实验说明”改成正式读者版，并统一整份报告的可见口径。
- scope:
  - 重写 `scripts/analysis/dataset_partitioned_nav_report.py` 的展示元数据层，按 `数据集差异 / 实验设计 / data2 参数表 / data4 参数表` 重构说明页。
  - 修改 `scripts/analysis/export_dataset_partitioned_nav_report_html.py`，统一正式标题为 `INS/GNSS/ODO/NHC组合导航结果`，移除可见 `true_iekf`、`cfg_*`、`output/review`、`source=` 与 case id，并把状态图导出为展示别名目录。
  - 重导出 `output/html/dataset_partitioned_navigation_interactive_report.html`，确认指标来源仍是 `EXP-20260312-dataset-report-cases-r2`，只变化展示层。
- changed_files:
  - `scripts/analysis/dataset_partitioned_nav_report.py`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `output/html/_report_assets/gallery_{1..10}/`
  - `output/html/_report_assets/plotly-6.5.2.min.js`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260312-dataset-report-cases-r2/manifest.json`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data2_gnss30_eskf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data2_gnss30_true_iekf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss30_eskf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss30_true_iekf_best.yaml`
- commands:
  - `Get-Content -Raw walkthrough.md`
  - `Get-Content -Raw C:\Users\不存在的骑士\.codex\skills\using-superpowers\SKILL.md`
  - `Get-Content -Raw C:\Users\不存在的骑士\.codex\skills\frontend-responsive-ui\SKILL.md`
  - `python -m py_compile scripts\analysis\dataset_partitioned_nav_report.py scripts\analysis\export_dataset_partitioned_nav_report_html.py`
  - `python scripts\analysis\export_dataset_partitioned_nav_report_html.py --target-points 1200 --out output\html\dataset_partitioned_navigation_interactive_report.html`
  - `Select-String -Path output\html\dataset_partitioned_navigation_interactive_report.html -Pattern 'INS/GNSS/ODO/NHC组合导航结果|数据集差异|实验设计|data2 关键参数配置|data4 关键参数配置|安装角 roll|GNSS 速度'`
  - PowerShell raw-text checks: `Contains('true_iekf'/'cfg_'/'output/review'/'source=')`
  - PowerShell regex counts: `nav_items/page_views/gallery_cards/gallery_refs`
- artifacts:
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `output/html/_report_assets/gallery_1/` ... `output/html/_report_assets/gallery_10/`
  - `output/html/_report_assets/plotly-6.5.2.min.js`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`
- metrics:
  - HTML 结构核验：`nav_items/page_views=7/7`，侧边栏分组为 `实验说明 / data2 / data4`，其中 `data2/data4` 各 `3` 个实验页。
  - 状态图去重核验：`gallery_cards=10`，`gallery_refs=4`，与既有 canonical cases 口径一致。
  - 可见文本审计：`true_iekf/cfg_/output/review/source=/Dataset Partitioned Navigation Report` 在最终 HTML 源中命中均为 `0`。
  - 结果指标未改：仍沿用 `EXP-20260312-dataset-report-cases-r2/metrics.csv`，例如 data2 `GNSS30 InEKF RMSE3D=21.656987`、data4 `GNSS30 ESKF RMSE3D=51.373795`。
- artifact_mtime:
  - `scripts/analysis/dataset_partitioned_nav_report.py`: `2026-03-13 00:31:00`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`: `2026-03-13 00:37:58`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-13 00:39:06`
  - `output/html/_report_assets/plotly-6.5.2.min.js`: `2026-03-13 00:39:05`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`: `2026-03-12 22:56:26`
- config_hash_or_mtime:
  - 本会话未修改 solver、配置 YAML 或 canonical case 产物；仅重写报告构造与导出脚本。
  - 说明页中的状态/噪声与观测更新表均直接读取 `EXP-20260312-dataset-report-cases-r2` 对应配置生成。
- dataset_time_window:
  - data2: `528076.000 -> 530488.900`
  - data4: `275309.000 -> 277300.000`
  - outage 模板说明保持 `900 s` 收敛后 `300 s on / 120 s off`
- result_freshness_check:
  - pass（本会话 fresh 导出了新 HTML；其指标来源继续指向 2026-03-12 的 `r2` canonical cases，未混入旧 `r1`/2026-03-05 历史口径）
- observability_notes:
  - 本会话未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或任何观测调度，只重写展示文本。
  - 说明页明确补记了 `kMountRoll=22`：状态保留，但当前主 ODO/NHC Jacobian 默认不主动估计/更新；`kMountPitch/kMountYaw=23/24` 为当前主安装角估计自由度。
  - 说明页同时区分了 `kGnssLever=28:30` 在两组数据中的观测来源差异：data2 无 GNSS 速度链路，data4 有 GNSS 位置+速度双链路。
- decision:
  - 新分组 HTML 已从“内部实验说明”改成面向读者的正式版说明页，正式标题统一为 `INS/GNSS/ODO/NHC组合导航结果`，并且整份报告的可见方法名统一为 `InEKF`。
  - 本轮不改变任何实验数值，只改变展示层；当前正式结果来源仍是 `EXP-20260312-dataset-report-cases-r2`。
  - 为避免 HTML 源中残留内部 case 词，状态图改为输出到展示别名目录 `output/html/_report_assets/gallery_*/` 后再引用。
- next_step:
  - 将同一套读者口径同步到 `representative_navigation_interactive_report.html` 与 `.tex` 文档，避免旧报告继续出现 `true_iekf` 或内部配置术语。
  - 在机理分析文档里继续围绕 `v系航向误差`、`att_z-bg_z` 互协方差与 `21-30` 状态块演化，解释 data2/data4 下 ESKF 与 InEKF 的差异来源。

### session_id: 20260313-0103-dataset-report-reader-table-compaction

- timestamp: 2026-03-13 01:12 (local)
- objective: 在不重跑实验的前提下，继续压缩 `dataset_partitioned_navigation_interactive_report.html` 的说明页表格，减少冗余列并消除说明页横向滚动。
- scope:
  - 修改 `scripts/analysis/dataset_partitioned_nav_report.py`，把说明页状态表与观测表改成“公共值 + 差异列”的读者版结构。
  - 修改 `scripts/analysis/export_dataset_partitioned_nav_report_html.py`，为 4 类说明页表格添加定制 class，并让 intro table 贴合内容区宽度、自动换行、不再依赖横向滚动。
  - 重导出 `output/html/dataset_partitioned_navigation_interactive_report.html`，确认结果页指标与状态图去重逻辑保持不变。
- changed_files:
  - `scripts/analysis/dataset_partitioned_nav_report.py`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260312-dataset-report-cases-r2/manifest.json`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data2_gnss30_eskf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data2_gnss30_true_iekf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss30_eskf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss30_true_iekf_best.yaml`
- commands:
  - `Get-Content -Path walkthrough.md -TotalCount 220`
  - `git status --short --branch`
  - `python -m py_compile scripts/analysis/dataset_partitioned_nav_report.py scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - PowerShell raw HTML checks：统计 `nav_items/page_views/gallery_cards/gallery_refs`、新旧表头命中数，以及 `true_iekf/cfg_/output/review/source=` 是否残留。
- artifacts:
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`
- metrics:
  - 说明页表头压缩结果：`<th>初始标准差</th>/<th>过程噪声</th>/<th>算法差异</th>` 各命中 `2` 次；`<th>公共更新节奏</th>/<th>场景差异</th>` 各命中 `2` 次。
  - 旧表头清理结果：`是否参与当前主估计/初始协方差/是否启用/GNSS30 场景 ESKF/min-width: 62rem` 在最终 HTML 中命中均为 `0`。
  - 结构核验：`nav_items/page_views=7/7`，`gallery_cards/gallery_refs=10/4`。
  - 结果口径未变：HTML 仍展示 data2 `GNSS30 InEKF RMSE3D=21.656987`、data4 `GNSS30 ESKF RMSE3D=51.373795`，与 `EXP-20260312-dataset-report-cases-r2/metrics.csv` 一致。
- artifact_mtime:
  - `scripts/analysis/dataset_partitioned_nav_report.py`: `2026-03-13 01:08:36`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`: `2026-03-13 01:09:15`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-13 01:10:43`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`: `2026-03-12 22:56:26`
- config_hash_or_mtime:
  - 本会话未修改任何 solver 代码、YAML 配置或 canonical cases；仅调整说明页表结构和 CSS。
  - `EXP-20260312-dataset-report-cases-r2` 继续作为唯一正式结果源。
- dataset_time_window:
  - data2: `528076.000 -> 530488.900`
  - data4: `275309.000 -> 277300.000`
  - outage 模板说明保持 `900 s` 收敛后 `300 s on / 120 s off`
- result_freshness_check:
  - pass（本会话 fresh 导出了新 HTML；结果指标继续引用 2026-03-12 的 `r2` canonical cases，没有重跑实验，也未混入旧 `r1` 或 2026-03-05 历史口径）
- observability_notes:
  - 本会话未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或观测调度，只收缩说明页展示层。
  - `mounting_roll (22)` 的“状态保留但默认不主动估计/更新”说明保留在状态表正文中；`GNSS 速度` 在 data2/data4 的可用性差异保留在观测表正文中。
- decision:
  - 说明页已从“算法列平铺”改为“公共值 + 差异列”的读者版，状态表与观测表都收敛为 `6` 列。
  - intro table 已去掉强制最小宽度与横向滚动依赖，改为贴合内容区宽度的固定布局换行。
  - 结果页指标、轨迹图、误差图与状态图去重逻辑未改变。
- next_step:
  - 将同一套简化后的说明页表格设计同步到 `representative_navigation_interactive_report.html` 与 `.tex` 文档。
  - 继续围绕 `v系航向误差`、`att_z-bg_z` 互协方差与 `21-30` 状态块演化做机理分析。

### session_id: 20260313-0141-walkthrough-importance-archive-r1

- timestamp: 2026-03-13 01:41 (local)
- objective: 按内容重要性重组仓库与 `walkthrough.md`，把完整历史原文迁入受版本管理的归档文档，并把 superseded/临时大产物迁出当前工作位。
- scope:
  - 从原 `walkthrough.md` 提取 `EXP/ISSUE/HYP/session` 保全快照，并按 A/B/C 层改写主档。
  - 新增 `docs/project_history/` 下的归档文档与 `docs/repo_layout.md`。
  - 迁移根目录历史 `SOL_*.txt`、`output/review/_tmp*` 和已 supersede 的 `r1` 目录到 `archive/`，同时写入 relocation index。
- changed_files:
  - `walkthrough.md`
  - `docs/project_history/walkthrough_preservation_snapshot_20260313.md`
  - `docs/project_history/walkthrough_archive_registry_20260313.md`
  - `docs/project_history/walkthrough_archive_sessions_20260313.md`
  - `docs/project_history/artifact_relocation_index.md`
  - `docs/repo_layout.md`
  - `scripts/analysis/archive_walkthrough_by_importance.py`
  - `.gitignore`
- configs:
  - 本会话未修改 solver 配置、未重跑实验；当前正式结果源仍为 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 与 `EXP-20260312-dataset-report-cases-r2`。
- commands:
  - `Get-Content walkthrough.md` / `git status --short --branch` / `Get-ChildItem output/review`
  - `python scripts/analysis/archive_walkthrough_by_importance.py`
  - 校验脚本：比对 `EXP/ISSUE/HYP/session` 集合、`walkthrough.md` 体量、`output/review/_tmp*` 与根目录 `SOL_*.txt` 清理情况
- artifacts:
  - `docs/project_history/walkthrough_preservation_snapshot_20260313.md`
  - `docs/project_history/walkthrough_archive_registry_20260313.md`
  - `docs/project_history/walkthrough_archive_sessions_20260313.md`
  - `docs/project_history/artifact_relocation_index.md`
  - `docs/repo_layout.md`
  - `archive/root_sol_legacy/20260313/`
  - `archive/output_review/tmp/20260313/`
  - `archive/output_review/superseded/20260313/`
- metrics:
  - `walkthrough.md` bytes: `258979 -> 83425`。
  - 保全校验目标：`EXP=100`、`ISSUE=16`、`HYP=16`、`session=79`，要求新主档 + archive 文档集合完全覆盖原集合。
  - 迁移数量：root `SOL_*.txt`=`26(pre)`，`output/review/_tmp*`=`5(pre)`，superseded review dirs=`3(pre)`。
- artifact_mtime:
  - `docs/project_history/walkthrough_preservation_snapshot_20260313.md`: `2026-03-13 02:02:20`
  - `docs/project_history/walkthrough_archive_registry_20260313.md`: `2026-03-13 02:02:20`
  - `docs/project_history/walkthrough_archive_sessions_20260313.md`: `2026-03-13 02:02:20`
  - `docs/project_history/artifact_relocation_index.md`: `2026-03-13 02:02:20`
  - `docs/repo_layout.md`: `2026-03-13 02:02:20`
- config_hash_or_mtime:
  - 本会话未改任何实验配置或 solver 代码；仅整理文档和产物位置。
- dataset_time_window:
  - 不适用（本会话不重跑实验，沿用既有 canonical cases）。
- result_freshness_check:
  - pass（本会话只重构工作区与文档层；当前正式实验结果来源未变，且未混入新旧数值口径）。
- observability_notes:
  - 本会话未改变 `fusion.ablation.*`、`fusion.post_gnss_ablation.*`、GNSS 调度或任何 measurement model。
  - 通过把历史会话/实验按主题摘要保留在主档，后续仍可直接追溯 `bg_z`、`mounting`、`NHC` 频率与 `P0_diag` 等状态块/调度相关结论。
- decision:
  - 主 `walkthrough.md` 现在定位为“当前工作台 + 历史摘要”，完整历史原文统一移交到 `docs/project_history/`。
  - 当前正式证据链不迁移；仅把临时、重复和已 supersede 的大产物移入 `archive/` 并建立路径映射。
- next_step:
  - 在保持本轮 archive 规则的前提下，继续执行既有研究主线：刷新 `representative_navigation_interactive_report.html` / `.tex`，并围绕 `v系航向误差`、`att_z-bg_z` 与 `21-30` 状态块做机理分析。

### session_id: 20260313-0832-dataset-report-gnss-window-shading

- timestamp: 2026-03-13 08:32 (local)
- objective: 为 `dataset_partitioned_navigation_interactive_report.html` 的 GNSS 周期开断页面时序误差图添加 GNSS 可用时段背景色，并保持轨迹图与状态量变化图不变。
- scope:
  - 修改 `scripts/analysis/interactive_nav_report.py`，为 case/section 增加 GNSS 可用窗口解析与 Plotly `vrect` 背景层能力。
  - 修改 `scripts/analysis/dataset_partitioned_nav_report.py`，仅对 `outage_cycle` 页面启用背景色。
  - 修改 `scripts/analysis/export_dataset_partitioned_nav_report_html.py`，把对应页面说明改成“背景色表示 GNSS 可用时段”。
  - 重导出 `output/html/dataset_partitioned_navigation_interactive_report.html` 并做 HTML 静态核验。
- changed_files:
  - `scripts/analysis/interactive_nav_report.py`
  - `scripts/analysis/dataset_partitioned_nav_report.py`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260312-dataset-report-cases-r2/manifest.json`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`
  - `output/review/EXP-20260312-dataset-report-cases-r2/GNSS_outage_cycle_data2.txt`
  - `output/review/EXP-20260312-dataset-report-cases-r2/GNSS_outage_cycle_data4.txt`
- commands:
  - `python -m py_compile scripts/analysis/interactive_nav_report.py scripts/analysis/dataset_partitioned_nav_report.py scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 2500 --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - `python - <<'PY'`（核验 `GNSS_outage_cycle_data{2,4}.txt` 的窗口数与 HTML 中各页面 `shade_count`）
- artifacts:
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `output/review/EXP-20260312-dataset-report-cases-r2/GNSS_outage_cycle_data2.txt`
  - `output/review/EXP-20260312-dataset-report-cases-r2/GNSS_outage_cycle_data4.txt`
- metrics:
  - GNSS 窗口恢复：`data2/data4` 周期开断文件分别解析出 `4/3` 段 GNSS 可用窗口。
  - HTML 着色核验：`page-data2-3/page-data4-3` 的 `shade_count=40/30`，其余 `page-intro/page-data2-1/page-data2-2/page-data4-1/page-data4-2` 均为 `0`。
  - 说明文案核验：最终 HTML 中 `背景色表示 GNSS 可用时段。` 命中 `2` 次，对应 `data2/data4` 的 outage 页面。
  - 指标口径保持不变：继续引用 `EXP-20260312-dataset-report-cases-r2/metrics.csv`，未新增任何 solver 数值改动。
- artifact_mtime:
  - `scripts/analysis/interactive_nav_report.py`: `2026-03-13 08:29:04`
  - `scripts/analysis/dataset_partitioned_nav_report.py`: `2026-03-13 08:17:41`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`: `2026-03-13 08:17:51`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-13 08:31:50`
  - `output/review/EXP-20260312-dataset-report-cases-r2/metrics.csv`: `2026-03-12 22:56:26`
- config_hash_or_mtime:
  - 本会话未修改任何 solver 代码、YAML 配置或 canonical cases；仅调整报告展示逻辑。
  - 结果口径仍以 `EXP-20260312-dataset-report-cases-r2` 为唯一正式来源。
- dataset_time_window:
  - data2: `528076.000 -> 530488.900`
  - data4: `275309.000 -> 277300.000`
  - 周期开断模板按现有文件恢复为 `900 s` 收敛后 `300 s on / 120 s off`。
- result_freshness_check:
  - pass（本会话 fresh 导出了新 HTML；背景色窗口直接从 `EXP-20260312-dataset-report-cases-r2` 的 outage GNSS 文件恢复，且最终页面着色分布已静态核验）
- observability_notes:
  - 本会话未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或主融合链路，只在展示层标出 GNSS 可用时段。
  - `outage_cycle` 页面现在能直接把 `GNSS` 更新窗口与 `v系速度误差`、`v系航向误差`、`高程误差` 的漂移时段对齐；`GNSS30` 与全程 GNSS 页面保持原状，不新增背景层。
  - `21-30` 状态块相关的状态量变化图未改动，避免把展示层色块误解为状态冻结/可观性结论。
- decision:
  - 背景色仅应用于 `dataset-partitioned` 报告中的 `outage_cycle` 页面，并按断续 GNSS 文件本身的可用窗口绘制，不按同页 full-GNSS 对照组的并集或交集绘制。
  - 轨迹图、状态量变化图和非 outage 页保持不变，避免读者把色块误读为所有工况共有的 GNSS 调度。
  - 窗口恢复逻辑采用“仅长间隙分段”的阈值，避免 `data4` 文件中的零星缺测把单个 GNSS on 窗口错误切成大量小块。
- next_step:
  - 如需保持多份报告体验一致，可把同样的 GNSS 可用窗口背景色同步到 `representative_navigation_interactive_report.html` 的 outage 页面。
  - 继续围绕 `v系航向误差`、`att_z-bg_z` 互协方差与 `21-30` 状态块演化做机理分析，并保持 `EXP-20260312-dataset-report-cases-r2` 为唯一数值口径。

### session_id: 20260313-0847-data4-outage-setting-audit

- timestamp: 2026-03-13 08:47 (local)
- objective: 检查 `data4` 周期开断实验设置是否存在配置继承或调度模板异常，解释当前结果为何与直觉不符。
- scope:
  - 审阅 `EXP-20260312-dataset-report-cases-r2` 的 manifest、生成脚本、`data4` outage/full 对应 YAML 与 outage GNSS 文件。
  - 对比 `config_data4_baseline_eskf.yaml`、`config_data4_gnss30_eskf.yaml` 与生成后的 `cfg_data4_{full_gnss,outage_eskf_best}.yaml`。
  - 统计 `data2/data4` outage GNSS 文件的 on/off 窗口、off-ratio 与 off-window 期间速度量级。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data4_baseline_eskf.yaml`
  - `config_data4_gnss30_eskf.yaml`
  - `config_data4_baseline_inekf_best.yaml`
  - `config_data4_gnss30_inekf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_{full_gnss_eskf,gnss_outage_eskf_best,gnss_outage_true_iekf_best}.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 120`
  - `python - <<'PY'`（读取 `manifest.json`，抽取 `data4` full/outage cases 的 `config_path/gnss_path/nhc_label/metrics`）
  - `Get-Content output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_*.yaml -TotalCount 220`
  - `Get-Content scripts/analysis/{generate_dataset_report_cases.py,filter_gnss_outage.py}`
  - `git diff --no-index -- config_data4_baseline_eskf.yaml config_data4_gnss30_eskf.yaml`
  - `git diff --no-index -- output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss_outage_eskf_best.yaml`
  - `python - <<'PY'`（重建 `data2/data4` outage on/off 窗口并统计 off-window 平均速度与近似路程）
- artifacts:
  - `output/review/EXP-20260312-dataset-report-cases-r2/manifest.json`
  - `output/review/EXP-20260312-dataset-report-cases-r2/GNSS_outage_cycle_data4.txt`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss_outage_eskf_best.yaml`
- metrics:
  - `data4` outage 文件设置本身正常：实验窗口 `1991.0 s`，GNSS on 窗口为 `[0,1200] / [1320,1620] / [1740,1991] s`，off 总时长 `240.0 s`，off-ratio `0.1205`。
  - `data4` off-window 期间平均速度约 `4.739 m/s`，近似 dead-reckoning 路程约 `1137.4 m`；对应 `data2` 为 `12.426 m/s` 与 `4473.4 m`，说明 data4 outage 工况本身显著更轻。
  - `data4` outage ESKF 的设置存在实质性配置混入：生成脚本把 `case_id=data4_gnss_outage_eskf_best` 的 `base_config_rel` 指向 `config_data4_gnss30_eskf.yaml`，而 full control 用的是 `config_data4_baseline_eskf.yaml`。
  - 生成后的 full/outage ESKF cfg 除 `gnss_path` 外还存在 `P0_diag` 差异；首个差异出现在状态索引 `6-8(att)`，并且 `bg(12-14)`、`mounting(22-24)`、`gnss_lever_z(30)` 也不一致。
- observability_notes:
  - 本次检查未改动任何 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或测量模型，但确认 `data4 outage ESKF` 与 full control 之间混入了初始化协方差差异，因而不应把当前页面直接当作“仅 GNSS 调度变化”来解释状态块 `att/bg/mounting/gnss_lever` 的差异。
  - `data4` outage 文件只有两段完整 `120 s` 无 GNSS 窗口，且 vehicle speed 明显低于 data2；因此即便修正为纯 A/B，对 `v系航向误差` 与位置误差的冲击也很可能仍显著小于 data2。
- decision:
  - 当前 `data4` 周期开断结果“看起来过于正常”并不主要来自 outage 文件模板错误；模板本身符合 `900 s + 300/120 s` 设定，且 data4 工况确实更容易。
  - 但 `data4 full GNSS ESKF` vs `data4 outage ESKF` 这组对比目前不是纯 A/B，因为 outage ESKF 继承了 `config_data4_gnss30_eskf.yaml` 的 `P0_diag` 而不是 full baseline ESKF 配置。
  - `data4` outage `true_iekf` case 的自身配置链路基本干净；不过整个页面仍以 full ESKF 为公共对照，因此也不能读成纯算法控制实验。
- next_step:
  - 重新生成 `data4_gnss_outage_eskf_best`，把 `base_config_rel` 改为 `config_data4_baseline_eskf.yaml`，仅保留 `gnss_path=GNSS_outage_cycle_data4.txt` 作为 outage 变量；必要时同步补一个 `data4 full GNSS true_iekf` 控制组。
  - 在报告文案中明确当前 `outage_cycle` 页面是“公共 full-GNSS ESKF 对照 + 两个断续场景结果”，避免误读为严格单变量 A/B。

### session_id: 20260313-0904-data4-baseline-gnssvel-tail-audit

- timestamp: 2026-03-13 09:04 (local)
- objective: 审计 `data4 full GNSS baseline ESKF` 末端 `v系航向误差` 显著漂移的来源，验证是否由 `GNSS_VEL` 更新触发。
- scope:
  - 以 `EXP-20260312-dataset-report-cases-r2` 的 canonical `data4_full_gnss_eskf` 为对照，检查 fresh `enable_gnss_velocity=false` 控制组。
  - 用 `scripts/analysis/interactive_nav_report.py` 的同口径计算拆分 `vehicle_heading_err_deg`、机体系 yaw 误差与 `mounting_yaw` 贡献。
  - 审阅 `GNSS_VEL` 代码路径，并统计末段 truth 速度与 GNSS 速度量测量级。
- changed_files:
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml`
  - `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/cfg_data4_full_gnss_eskf_no_gnss_vel.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "vehicle_heading_err_deg|build_case_result|relative_euler_error_deg" scripts/analysis/interactive_nav_report.py`
  - `Get-Content scripts/analysis/interactive_nav_report.py` / `Get-Content src/app/pipeline_fusion.cpp` / `Get-Content src/core/measurement_models_uwb.cpp`
  - `python - <<'PY'`（同口径计算 baseline vs `no_gnss_vel` 的 `RMSE3D`、`vehicle_heading_err_deg`、body yaw error、`mounting_yaw_delta`）
  - `python - <<'PY'`（统计 baseline 最后 `600/300/120/60 s` 的 truth 速度、yaw error 与 `bg_z` 演化）
  - `python - <<'PY'`（读取 `dataset/data4_converted/GNSS_converted.txt`，比较末段 `speed_h` 与 `sigma_vh` 量级）
- artifacts:
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/SOL_data4_report_full_eskf.txt`
  - `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/cfg_data4_full_gnss_eskf_no_gnss_vel.yaml`
  - `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/SOL_data4_full_gnss_eskf_no_gnss_vel.txt`
- metrics:
  - baseline / `no_gnss_vel` `RMSE3D=1.2293197724 / 1.2556400743` m。
  - 末端 `vehicle_heading_err_deg=-12.700457 / -14.535365`，`body_yaw_err_deg=-11.729758 / -13.508358`，`mounting_yaw_delta=0.964180 / 1.020567 deg`。
  - baseline 最后 `120 s`：truth 水平速度中位数 `5.8309519e-4 m/s`、`p95=0.0013601471 m/s`、`100%<0.1 m/s`；同时 body yaw error `-6.184390 -> -11.729758 deg`，`bg_z 0.001176936 -> 0.001209749 rad/s`。
  - baseline 最后 `300 s` 的 GNSS 水平速度中位数 `0.026846 m/s`，而 `sigma_vh_rss` 中位数 `0.100838 m/s`；末 `5` 个 GNSS 样本速度幅值仅 `0.021~0.052 m/s`。
- artifact_mtime:
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml`: `2026-03-12 22:52:32`
  - `output/review/EXP-20260312-dataset-report-cases-r2/SOL_data4_report_full_eskf.txt`: `2026-03-12 22:53:03`
  - `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/cfg_data4_full_gnss_eskf_no_gnss_vel.yaml`: `2026-03-13 08:47:36`
  - `output/review/EXP-20260313-data4-baseline-gnssvel-off-audit-r1/SOL_data4_full_gnss_eskf_no_gnss_vel.txt`: `2026-03-13 08:50:14`
- config_hash_or_mtime:
  - `cfg_data4_full_gnss_eskf_no_gnss_vel.yaml` 相对 canonical baseline 仅新增 `fusion.enable_gnss_velocity: false` 与新的 `output_path`；其余主融合配置保持一致。
- dataset_time_window:
  - data4: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（`no_gnss_vel` 控制组产物于 `2026-03-13` fresh 生成；baseline 对照仍为 `2026-03-12` 正式 canonical case）
- observability_notes:
  - `GNSS_VEL` measurement model 的 `H` 直接作用于 `v(3)`、`att(6:8)`、`bg(12:14)`、`sg(15:17)`、`gnss_lever(28:30)`，不直接更新 `mounting(22:24)`。
  - 当前 `data4 full GNSS baseline` 的末端 `v系航向误差` 主要由 `att_z` / `bg_z` 路径贡献：车体系末端 `-12.70 deg` 中约 `-11.73 deg` 来自机体系 yaw，`mounting_yaw` 只占次级约 `+0.96 deg`。
  - 最后 `120 s` 车辆几乎静止，GNSS 速度幅值已低于其噪声量级，意味着 GNSS 位置与小速度量测对 heading 的即时约束都显著减弱，tail drift 更符合“弱激励下姿态/陀螺偏置可观性不足”的机理。
- decision:
  - `data4` 全 GNSS baseline 末端 `v系航向误差` 的显著漂移没有证据表明是 `GNSS_VEL` 更新“拉坏”的；关闭 `GNSS_VEL` 后误差没有消失，反而略有变差。
  - 当前更合理的解释是：末段进入低速/近停车弱激励区后，`att_z-bg_z` 约束不足，机体系 yaw 漂移继续积累；`mounting_yaw` 只提供次级放大量，不是首因。
- next_step:
  - 若需进一步做严格因果闭环，补一组 `data4 full GNSS baseline` 的 `freeze_bg_z` / `freeze_mounting_yaw` 对照，确认 tail drift 对 `bg_z` 的灵敏度。
  - 在 `representative` / `.tex` 文案中补一句说明：`data4` 全 GNSS baseline 末端 `v系航向误差` 漂移主要对应低速弱激励 tail，而非 `GNSS_VEL` 更新副作用。

### session_id: 20260313-0928-data4-baseline-vs-outage-heading-gap-audit

- timestamp: 2026-03-13 09:28 (local)
- objective: 解释为什么 `data4_eskf_baseline` 的 `v系航向误差` 明显差于 `data4_GNSSoutage_eskf` 与 `data4_GNSSoutage_InEKF`，并判断这是否意味着三者“应共享同一个末端问题”。
- scope:
  - 在同一 `data4` truth 口径下对比 `full_gnss_eskf`、`gnss_outage_eskf_best`、`gnss_outage_true_iekf_best` 的 `vehicle_heading/body_yaw/mounting_yaw` 时序。
  - 检查最后一段 GNSS 恢复窗口前后的 `bg_z` 水平。
  - 复核 `outage_eskf` 相对 baseline 的配置差异，确认当前页面能否读成纯 A/B。
- changed_files:
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_full_gnss_eskf.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss_outage_eskf_best.yaml`
  - `output/review/EXP-20260312-dataset-report-cases-r2/cfg_data4_gnss_outage_true_iekf_best.yaml`
- commands:
  - `git diff --no-index -- cfg_data4_full_gnss_eskf.yaml cfg_data4_gnss_outage_eskf_best.yaml`
  - `git diff --no-index -- cfg_data4_full_gnss_eskf.yaml cfg_data4_gnss_outage_true_iekf_best.yaml`
  - `python - <<'PY'`（计算三个 case 的末端 `vehicle_heading/body_yaw/mounting_yaw`、`p95`、最后 `300/120/60 s` 的起终点）
  - `python - <<'PY'`（在 `t=1200/1320/1620/1740/1870/1931 s` 采样三条曲线的 `body_yaw` 与 `bg_z`）
- artifacts:
  - `output/review/EXP-20260312-dataset-report-cases-r2/{cfg_data4_full_gnss_eskf.yaml,cfg_data4_gnss_outage_eskf_best.yaml,cfg_data4_gnss_outage_true_iekf_best.yaml}`
  - `output/review/EXP-20260312-dataset-report-cases-r2/{SOL_data4_report_full_eskf.txt,SOL_data4_report_outage_cycle_eskf_best.txt,SOL_data4_report_outage_cycle_true_iekf_best.txt}`
- metrics:
  - 末端 `vehicle/body/mounting`：
    baseline=`-12.700/-11.730/+0.964 deg`；
    outage_eskf=`-2.265/-1.055/+1.203 deg`；
    outage_true=`-1.085/+0.224/+1.302 deg`。
  - 最后 `120 s` 的 `vehicle_heading_err_deg`：
    baseline=`-7.152→-12.700 deg`；
    outage_eskf=`-1.900→-2.265 deg`；
    outage_true=`-1.253→-1.085 deg`。
  - 最后一次 GNSS 恢复窗口起点 `t≈1740 s` 的 `body_yaw/bg_z`：
    baseline=`-0.258 deg / 1.143489e-3 rad/s`；
    outage_eskf=`-0.465 deg / 9.0049e-5 rad/s`；
    outage_true=`-0.033 deg / -4.7742e-5 rad/s`。
  - `outage_eskf` 与 baseline 不只差 `gnss_path`：还混入 `P0_diag` 差异，尤其 `att(6:8)` `1e-4→3.046e-6`、`bg(12:14)` `2.5e-7→5.4289e-8`、`mounting(22:24)` `0.0001/0.09/0.01→7.615e-5/7.615e-5/2.742e-5`。
- artifact_mtime:
  - `cfg_data4_full_gnss_eskf.yaml`: `2026-03-12 22:52:32`
  - `cfg_data4_gnss_outage_eskf_best.yaml`: `2026-03-12 22:56:10`
  - `cfg_data4_gnss_outage_true_iekf_best.yaml`: `2026-03-12 22:56:10`
  - `SOL_data4_report_full_eskf.txt`: `2026-03-12 22:53:03`
  - `SOL_data4_report_outage_cycle_eskf_best.txt`: `2026-03-12 22:56:23`
  - `SOL_data4_report_outage_cycle_true_iekf_best.txt`: `2026-03-12 22:56:26`
- config_hash_or_mtime:
  - 当前 `outage_eskf` 不是纯 `baseline + GNSS_outage_cycle_data4.txt`；它继承的是 `config_data4_gnss30_eskf.yaml` 口径。
- dataset_time_window:
  - data4: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（全部读取 `EXP-20260312-dataset-report-cases-r2` 正式 canonical cases，并对其做同口径后处理）
- observability_notes:
  - 三条曲线共享同一个末端低速弱激励时段，但它们进入该时段时的 `bg_z` 与 body yaw 已经明显不同，因此不能期待“末段表现必然相同”。
  - 当前差异主要不是 `mounting(22:24)` 的末段继续漂移，而是 `att_z/bg_z` 历史状态不同：baseline 在最后 GNSS 恢复段内迅速掉到 `body_yaw≈-6 deg`，另外两条保持在 `±1 deg` 内。
  - 对 `outage_eskf` 而言，当前更像“配置口径混入后形成的另一条状态历史”，而不是严格意义上的 GNSS 周期开断对 baseline 的单变量影响。
- decision:
  - `data4_eskf_baseline` 明显差于 `data4_GNSSoutage_eskf` / `data4_GNSSoutage_InEKF`，并不意味着“GNSS 开断反而改善了航向”；现阶段最重要的解释是三者并未以相同状态/协方差进入末端弱激励段。
  - 尤其 `outage_eskf` 当前不是纯对照组，因此不能把它优于 baseline 解释成 schedule 本身有益；`outage_true` 优于 baseline 则仍包含算法差异。
- next_step:
  - 重新生成 pure-AB 的 `data4_gnss_outage_eskf_best`，用 `config_data4_baseline_eskf.yaml` 仅替换 `gnss_path` 为 `GNSS_outage_cycle_data4.txt`。
  - 完成后再重新比较 `full_gnss_eskf` vs `outage_eskf_pure_ab` 的 `v系航向误差`，判断 schedule 自身是否会改变末端 `bg_z/body_yaw` 进入值。

### session_id: 20260313-1132-data4-full-outage-p0-align-r1

- timestamp: 2026-03-13 11:32 (local)
- objective: 按用户指定口径，将 `data4_full_gnss_eskf` 的初始化配置与 `data4_gnss_outage_eskf_best` 的真实 `P0_diag` 口径统一，并 fresh 重算 canonical cases 与读者版 HTML。
- scope:
  - 修改 `scripts/analysis/generate_dataset_report_cases.py`，让 `data4_full_gnss_eskf` 与 `data4_gnss_outage_eskf_best` 都从 `config_data4_gnss30_eskf.yaml` 继承。
  - fresh 重生新的 canonical cases 目录 `EXP-20260313-dataset-report-cases-r4`，复用 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 的 best NHC。
  - 重导出 `output/html/dataset_partitioned_navigation_interactive_report.html`，确认页面已切换到 `r4` manifest。
- changed_files:
  - `scripts/analysis/generate_dataset_report_cases.py`
  - `walkthrough.md`
  - `output/review/EXP-20260313-dataset-report-cases-r4/*`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
- configs:
  - `config_data4_gnss30_eskf.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_full_gnss_eskf.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_gnss_outage_eskf_best.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_gnss_outage_true_iekf_best.yaml`
- commands:
  - `python -m py_compile scripts/analysis/generate_dataset_report_cases.py scripts/analysis/dataset_partitioned_nav_report.py scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python scripts/analysis/generate_dataset_report_cases.py --sweep-outdir output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2 --cases-outdir output/review/EXP-20260313-dataset-report-cases-r4`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 2500 --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - `git diff --no-index -- output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_full_gnss_eskf.yaml output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_gnss_outage_eskf_best.yaml`
  - `python - <<'PY'`（核对 `build_dataset_partitioned_report()` 绑定的 manifest 路径与 `data4` 页面概览指标）
  - `python - <<'PY'`（计算 `r4` 下 data4 full/outage/outage_true 的末端 `vehicle_heading/body_yaw/mounting_yaw`）
- artifacts:
  - `output/review/EXP-20260313-dataset-report-cases-r4/manifest.json`
  - `output/review/EXP-20260313-dataset-report-cases-r4/metrics.csv`
  - `output/review/EXP-20260313-dataset-report-cases-r4/summary.md`
  - `output/review/EXP-20260313-dataset-report-cases-r4/{cfg_data4_full_gnss_eskf.yaml,cfg_data4_gnss_outage_eskf_best.yaml}`
  - `output/review/EXP-20260313-dataset-report-cases-r4/{SOL_data4_report_full_eskf.txt,SOL_data4_report_outage_cycle_eskf_best.txt,SOL_data4_report_outage_cycle_true_iekf_best.txt}`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
- metrics:
  - `EXP-20260313-dataset-report-cases-r4` 的 data4 `RMSE3D`：full / GNSS30 ESKF / GNSS30 InEKF / outage ESKF / outage InEKF = `0.929072 / 51.373795 / 12.011497 / 1.279736 / 1.262719` m。
  - `r4` 下 data4 末端 `vehicle_heading_err_deg`：full / outage ESKF / outage InEKF = `-2.359895 / -2.264628 / -1.084793 deg`；对应 `p95 |vehicle_heading| = 2.117607 / 2.173009 / 1.672434 deg`。
  - `r4` 下 full/outage ESKF 的配置差异已收敛到 `gnss_path/output_path`；两者 `P0_diag` 与 `gnss_schedule(enabled=false, head_ratio=0.3)` 完全一致。
- artifact_mtime:
  - `output/review/EXP-20260313-dataset-report-cases-r4/manifest.json`: `2026-03-13 11:27:01`
  - `output/review/EXP-20260313-dataset-report-cases-r4/metrics.csv`: `2026-03-13 11:27:01`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_full_gnss_eskf.yaml`: `2026-03-13 11:23:04`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_gnss_outage_eskf_best.yaml`: `2026-03-13 11:25:23`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-13 11:28:37`
- config_hash_or_mtime:
  - 本会话未改 `config_data4_gnss30_eskf.yaml` 数值本身；统一口径是通过 canonical case 生成脚本实现的：`data4_full_gnss_eskf` 改为与 `data4_gnss_outage_eskf_best` 共用 `config_data4_gnss30_eskf.yaml` 的初始化语义。
- dataset_time_window:
  - data4: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（`r4` canonical cases 与 HTML 均于 `2026-03-13` fresh 生成；读者版 HTML 当前绑定的 manifest 为 `EXP-20260313-dataset-report-cases-r4`）
- observability_notes:
  - 本会话没有改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或 GNSS 周期开断模板；只统一了 `data4` full/outage ESKF 的初始化口径。
  - 对 `att(6:8)`、`bg(12:14)`、`mounting(22:24)` 这些与航向最相关的状态块，full/outage ESKF 现在共享相同初始协方差，因此 `data4` 周期开断页上的 full/outage 对比可以按“相同 `P0_diag` + 不同 GNSS 文件”解释。
  - 统一后，data4 full/outage ESKF 的 `v系航向误差` 已回到同一量级，说明此前 `r2` 中 full baseline 的大 tail gap 主要是配置口径问题，而不是单独的 schedule 机理结论。
- decision:
  - 官方 canonical cases 现切换到 `EXP-20260313-dataset-report-cases-r4`；`EXP-20260312-dataset-report-cases-r2` 保留为历史口径，不再用于解释 data4 full/outage ESKF 对比。
  - 读者版 HTML 已刷新到 `r4` 口径；今后若引用 `data4` 的 full/outage ESKF，对应数值应以 `full RMSE3D=0.929072 m`、`outage RMSE3D=1.279736 m` 与末端 `vehicle_heading≈-2.36/-2.26 deg` 为准。
- next_step:
  - 刷新 `representative_navigation_interactive_report.html` 与 `.tex`，把仍引用 `r2` data4 full baseline 的页面切到 `r4`。
  - 若需继续解释 data4 周期开断差异，下一步应直接分析 `outage_eskf` vs `outage_true_iekf` 的算法差异，而不是继续沿用已 supersede 的 `r2` full/outage 大 gap。

### session_id: 20260313-1334-gnss30-noise-qr-sweep-r1

- timestamp: 2026-03-13 13:34 (local)
- objective: 对 `GNSS30` 四个 canonical cases 做显式 `Q/R` 合理性实验，确认当前官方噪声设置是否仍存在明显优化空间。
- scope:
  - 新增 `scripts/analysis/noise_qr_sweep.py`，以 `EXP-20260313-dataset-report-cases-r4/manifest.json` 中的 4 个 `GNSS30` case 为唯一 reference。
  - 只改动显式 `fusion.noise.*` 与 `constraints.sigma_*` 标量，先做 reference 校验，再做 6 个 family 的粗扫和 top-2 family 的局部细化。
  - 额外记录 `GNSS_VEL` 数据文件统计，但不把文件内逐历元速度标准差纳入 sweep 轴。
- changed_files:
  - `scripts/analysis/noise_qr_sweep.py`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260313-dataset-report-cases-r4/manifest.json`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data2_gnss30_eskf_best.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data2_gnss30_true_iekf_best.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_gnss30_eskf_best.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_data4_gnss30_true_iekf_best.yaml`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data*/cfg_*.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 220`
  - `Get-Content scripts/analysis/generate_dataset_report_cases.py`
  - `Get-Content output/review/EXP-20260313-dataset-report-cases-r4/{manifest.json,summary.md}`
  - `Get-Content config_data{2,4}_gnss30_{eskf,true_iekf}.yaml`
  - `python -m py_compile scripts/analysis/noise_qr_sweep.py`
  - `python scripts/analysis/noise_qr_sweep.py --help`
  - `python scripts/analysis/noise_qr_sweep.py`
  - `Get-Content output/review/EXP-20260313-noise-qr-gnss30-r1/{summary.md,manifest.json,gnss_vel_audit.json}`
- artifacts:
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/manifest.json`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/summary.md`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/metrics.csv`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/case_ranking.csv`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/gnss_vel_audit.json`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data*/{best_config.yaml,best_summary.md,best_vs_ref.json}`
- metrics:
  - reference 4/4 全部精确复现 `r4` 官方数值：`data2 ESKF/true = 101.122603/21.656987 m`，`data4 ESKF/true = 51.373795/12.011497 m`。
  - `data2_gnss30_eskf_best`：best=`R_nhc×4`，`RMSE3D 101.122603→65.182069 m`，`P95 186.866628→161.817751 m`，`final3D 278.279012→204.246374 m`。
  - `data2_gnss30_true_iekf_best`：best=`Q_imu×0.25 + R_odo×2`，`RMSE3D 21.656987→8.119960 m`，`P95 52.295784→17.014575 m`，`final3D 58.009538→22.677560 m`。
  - `data4_gnss30_eskf_best`：best=`R_odo×4`，`RMSE3D 51.373795→40.152394 m`，`P95 111.892467→105.134877 m`，但 `final3D 29.001817→62.581789 m` 明显变差。
  - `data4_gnss30_true_iekf_best`：best=`Q_calib×0.25 + Q_bias×0.25`，`RMSE3D 12.011497→4.392984 m`，`P95 30.969501→9.051201 m`，`final3D 31.069894→4.142923 m`。
  - `GNSS_VEL` audit：`data2` GNSS 文件只有 `7` 列，无速度和速度标准差列，因此 `GNSS_VEL` update 不执行；`data4` GNSS 文件含 `13` 列，`sigma_vn/ve/vd` 均值约 `0.078/0.060/0.154 m/s`，不存在 fallback 或 floor clamp 触发。
- artifact_mtime:
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/summary.md`: `2026-03-13 13:33:17`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/manifest.json`: `2026-03-13 13:33:17`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/metrics.csv`: `2026-03-13 13:33:17`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/case_ranking.csv`: `2026-03-13 13:33:17`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/gnss_vel_audit.json`: `2026-03-13 13:34:17`
- config_hash_or_mtime:
  - 根配置与 `r4` canonical cfg 本体未被修改；全部候选配置均在 `EXP-20260313-noise-qr-gnss30-r1/data*/cfg_*.yaml` 下派生生成。
  - `build/Release/eskf_fusion.exe` 复用 `2026-03-12 10:08:53` 的既有 fresh build；本会话未重编译 solver。
- dataset_time_window:
  - data2 GNSS30: `528076.000 -> 530488.900`
  - data4 GNSS30: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（reference、粗扫、局部细化和 top-level 汇总均在本会话 fresh 生成；4 个 reference case 全部 `match=true`）
- observability_notes:
  - 本会话未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或 `gnss_schedule.*`；只审计并缩放显式 `Q/R` 标量。
  - 结果显示当前 `GNSS30` 表现对不同状态块的噪声家族敏感性明显不同：`data2 ESKF` 更敏感于 `R_nhc`，`data2 true_iekf` 更敏感于 `Q_imu + R_odo`，`data4 true_iekf` 更敏感于 `Q_calib/Q_bias`（覆盖 `sg/sa/odo_scale/mounting/lever/gnss_lever` 与 `ba/bg`）。
  - `data4 ESKF` 的 `R_odo×4` 虽降低整体 `RMSE3D`，但末端 `final3D` 变差，说明“按 RMSE 最优”与“按 tail/终点稳定性最优”在该 case 下不完全一致，暂不宜直接升格为统一正式口径。
- decision:
  - 当前官方 `GNSS30` 噪声设置在 4 个 canonical cases 上都存在明确优化空间，但最优方向具有强烈的 case-specific 差异，不能直接写成一套统一的官方 `Q/R`。
  - 这轮实验足以否定“当前四个 GNSS30 case 的显式 Q/R 已经大体最优”的说法；但在做完 consistency/physics 审计前，还不应把这些 best cfg 直接回灌为 canonical 正式结果。
- next_step:
  - 对 4 个 best cfg 做 consistency/physics 审计，重点检查 `NIS/accept ratio/tail stability` 是否支持把它们升级为正式口径。
  - 若决定升格某些 best cfg，再回头重生对应 canonical cases、HTML 报告与 `.tex` 文案；若不升格，则把这轮结果保留为“敏感性与优化空间”证据链。

### session_id: 20260313-1510-dataset-report-r5-doc-sync

- timestamp: 2026-03-13 15:10 (local)
- objective: 在不重跑整轮 solver 实验的前提下，核对并固化 `r5` canonical cases / reader HTML 口径，补写最优配置差异与 fresh 证据到 `walkthrough.md`。
- scope:
  - 基于已有 `EXP-20260313-dataset-report-cases-r5` 与 `EXP-20260313-noise-qr-gnss30-r1` 产物，确认 full GNSS 与 GNSSoutage 已按 dataset/method 继承相同的 `GNSS30` best profile。
  - 只做 `r4 -> r5` 指标对比、配置差异提取与 HTML 绑定核验，不重新触发整轮 canonical cases 解算。
  - 更新 `Experiment Registry`、`Open Hypotheses` 与 `Next Actions`，把当前 dataset-partitioned 正式口径切换到 `r5/r6`。
- changed_files:
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/manifest.json`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data2_gnss30_eskf_best/best_config.yaml`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data2_gnss30_true_iekf_best/best_config.yaml`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data4_gnss30_eskf_best/best_config.yaml`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data4_gnss30_true_iekf_best/best_config.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r4/cfg_{data2_gnss30_eskf_best,data2_gnss30_true_iekf_best,data4_gnss30_eskf_best,data4_gnss30_true_iekf_best}.yaml`
  - `output/review/EXP-20260313-dataset-report-cases-r5/cfg_*.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 260`
  - `Get-Content output/review/EXP-20260313-dataset-report-cases-r5/{summary.md,manifest.json}`
  - `Get-Content output/review/EXP-20260313-dataset-report-cases-r4/summary.md`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 2500 --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - `python - <<'PY'`（核对 `_latest_cases_manifest()` 与 `build_dataset_partitioned_report()` 当前都绑定到 `EXP-20260313-dataset-report-cases-r5/manifest.json`）
  - `python - <<'PY'`（对 `r4/r5` 的 10 个 canonical cases 计算 `RMSE3D` 变化）
  - `python - <<'PY'`（提取 `r4` canonical cfg 与 4 份 `best_config.yaml` 的 noise / constraint 差异）
  - `python - <<'PY'`（核验 HTML 内可检索 `65.182/8.120/4.393/0.943/1.421/1.081/0.891` 等 `r5` 指标）
  - `python scripts/analysis/generate_dataset_report_cases.py --cases-outdir output/review/EXP-20260313-dataset-report-cases-r5 --noise-sweep-manifest output/review/EXP-20260313-noise-qr-gnss30-r1/manifest.json`（用户中途取消，未用于本次 fresh 证据）
- artifacts:
  - `output/review/EXP-20260313-dataset-report-cases-r5/manifest.json`
  - `output/review/EXP-20260313-dataset-report-cases-r5/metrics.csv`
  - `output/review/EXP-20260313-dataset-report-cases-r5/summary.md`
  - `output/review/EXP-20260313-noise-qr-gnss30-r1/data*/best_config.yaml`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
- metrics:
  - `r5` 的 data2 `RMSE3D`：full / GNSS30 ESKF / GNSS30 InEKF / outage ESKF / outage InEKF = `1.080872 / 65.182069 / 8.119960 / 1.834075 / 0.984145` m。
  - `r5` 的 data4 `RMSE3D`：full / GNSS30 ESKF / GNSS30 InEKF / outage ESKF / outage InEKF = `0.890927 / 40.152394 / 4.392984 / 0.943338 / 1.421392` m。
  - 相比 `r4`，`9/10` 个 canonical cases 的 `RMSE3D` 改善；唯一退化的是 `data4_gnss_outage_true_iekf_best: 1.262719→1.421392 m`（`+12.57%`）。
  - 4 份最优 profile 的主要参数变化：`data2 ESKF: sigma_nhc_y/z 0.1→0.4`；`data2 true: sigma_acc 0.05→0.0125, sigma_gyro 0.005→0.00125, sigma_odo 0.1→0.2`；`data4 ESKF: sigma_odo 0.1→0.4`；`data4 true: sigma_ba/bg/sg/sa/odo_scale/mounting{,_rpy}/lever_arm/gnss_lever_arm` 全部缩小到原来的 `0.25x`。
- artifact_mtime:
  - `output/review/EXP-20260313-dataset-report-cases-r5/manifest.json`: `2026-03-13 14:30:50`
  - `output/review/EXP-20260313-dataset-report-cases-r5/summary.md`: `2026-03-13 14:30:50`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-13 14:49:10`
- config_hash_or_mtime:
  - 本会话未改动根 YAML；`r5` 只是在 canonical case 生成阶段把 `output/review/EXP-20260313-noise-qr-gnss30-r1/data*/best_config.yaml` 中的 `fusion.noise` 与约束噪声/更新间隔映射到 report cases。
  - `P0_diag`、`fej.*`、`gnss_schedule.*`、`post_gnss_ablation.*` 与 GNSS 数据文件内容没有在本会话内被重写。
- dataset_time_window:
  - data2: `528076.000 -> 530488.900`
  - data4: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（`r5` manifest/summary 的 mtime 为 `2026-03-13 14:30:50`；HTML 于 `2026-03-13 14:49:10` fresh 导出；脚本入口与 HTML 文本命中都证明当前 reader 报告绑定的是 `EXP-20260313-dataset-report-cases-r5`）
- observability_notes:
  - 本会话未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*` 或 `fusion.gnss_schedule.*`；变化集中在显式 `Q/R` 标量如何从 `GNSS30` best profile 传递到 full/outage cases。
  - 对状态块的影响具有明显 case-specific 特征：`data2 ESKF` 主要改 `NHC` 侧向/竖向量测噪声；`data2 true_iekf` 主要改 `IMU` 过程噪声与 `ODO` 量测噪声；`data4 ESKF` 只改 `ODO` 量测噪声；`data4 true_iekf` 则同时收紧 `ba/bg` 与 `21-30` 扩展块相关的 `sg/sa/odo_scale/mounting/lever_arm/gnss_lever_arm` 过程噪声。
  - `data4_gnss_outage_true_iekf_best` 在共享 `data4_gnss30_true_iekf_best` profile 后反而变差，说明“同 dataset/method 共享 GNSS30 最优 profile”可以作为 reader-facing 统一口径，但未必是 physics-optimal 的长期配置策略。
- decision:
  - `dataset_partitioned_navigation_interactive_report.html` 的当前 reader-facing 官方口径切换到 `EXP-20260313-dataset-report-cases-r5` / `EXP-20260313-dataset-partitioned-html-r6`。
  - 参数变化的主口径统一为“让 `P0_diag` 与真实配置保持一致，同时把 `GNSS30` 已验证的 best `Q/R` profile 同步到同 dataset/method 的 full/outage cases”；不再把这件事描述成“单纯不混入某个额外 `P0_diag`”。
- next_step:
  - 继续审计 `r5` 中的两个 tradeoff：`data4_gnss30_eskf_best` 的 `final_err_3d` 恶化，以及 `data4_gnss_outage_true_iekf_best` 的整体退化。
  - 若后续要把 `r5` 推广到其他 reader-facing 文档，优先同步 `representative_navigation_interactive_report.html` 与 `.tex`，并注明 `data4` true outage 的例外性。

### session_id: 20260313-1528-dataset-report-html-singlefile-r1

- timestamp: 2026-03-13 15:28 (local)
- objective: 将 dataset-partitioned HTML 导出器改成“单文件可分发版”，避免邮件客户端因外部图片/脚本引用而拦截内容。
- scope:
  - 修改 `scripts/analysis/export_dataset_partitioned_nav_report_html.py`，把 `plotly` 脚本与状态图 gallery 图片支持内联为 `data:` URI。
  - 保留 `external` 资源模式作为后备，但默认改为 `inline`，让直接执行导出脚本时产出单文件 HTML。
  - 基于已有 `EXP-20260313-dataset-report-cases-r5` 数据重新导出 HTML，不重新计算 solver 结果。
- changed_files:
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `output/html/实验结果终版.html`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260313-dataset-report-cases-r5/manifest.json`
  - `scripts/analysis/export_dataset_partitioned_nav_report_html.py`
- commands:
  - `Get-Content scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python -m py_compile scripts/analysis/export_dataset_partitioned_nav_report_html.py`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 2500 --out output/html/dataset_partitioned_navigation_interactive_report.html`
  - `python scripts/analysis/export_dataset_partitioned_nav_report_html.py --target-points 2500 --out output/html/实验结果终版.html`
  - `python - <<'PY'`（核验默认导出 HTML 中不再包含 `<script src=` 与 `_report_assets/`，且存在 `data:image/png;base64,`）
  - `Select-String -LiteralPath 'output\\html\\实验结果终版.html' -Pattern '_report_assets/|<script src=|data:image/png;base64,'`
  - `Get-Item output\\html\\dataset_partitioned_navigation_interactive_report.html, 'output\\html\\实验结果终版.html'`
- artifacts:
  - `output/html/dataset_partitioned_navigation_interactive_report.html`
  - `output/html/实验结果终版.html`
- metrics:
  - 单文件导出验证：`dataset_partitioned_navigation_interactive_report.html` 中 `has_plotly_script_src=false`、`has_report_assets_ref=false`、`has_base64_png=true`、`has_r5_metric=true`。
  - `dataset_partitioned_navigation_interactive_report.html` 大小 `44,745,625 B`，mtime=`2026-03-13 15:20:43`。
  - `实验结果终版.html` 大小 `44,745,625 B`，mtime=`2026-03-13 15:24:24`。
  - 导出只改变打包方式，不改变 `r5` 指标口径。
- artifact_mtime:
  - `output/html/dataset_partitioned_navigation_interactive_report.html`: `2026-03-13 15:20:43`
  - `output/html/实验结果终版.html`: `2026-03-13 15:24:24`
- config_hash_or_mtime:
  - 本会话未改动 canonical case 配置，也未改动 `manifest.json` 中的数值结果；只修改 HTML 导出脚本的资源打包模式。
- dataset_time_window:
  - data2: `528076.000 -> 530488.900`
  - data4: `275309.000 -> 277300.000`
- result_freshness_check:
  - pass（两份 HTML 均在本会话 fresh 导出，且内容检查表明不再依赖 `_report_assets` 外部资源目录）
- observability_notes:
  - 本会话不涉及 `fusion.ablation.*`、`post_gnss_ablation.*`、`gnss_schedule.*`、`21-30` 状态块或任何求解器数值逻辑改动。
  - 影响仅限 reader-facing 报告的分发方式，避免外部图片/脚本被邮件客户端按隐私策略阻断。
- decision:
  - dataset-partitioned HTML 的默认发布格式切换为“单文件内联版”。
  - 对外发送时，优先使用 [dataset_partitioned_navigation_interactive_report.html](D:/因子图算法科研训练/UWB/output/html/dataset_partitioned_navigation_interactive_report.html) 或 [实验结果终版.html](D:/因子图算法科研训练/UWB/output/html/实验结果终版.html) 的当前版本，不再要求收件方同时持有 `_report_assets`。
- next_step:
  - 若后续遇到邮箱大小限制，可再补一个“单文件 HTML + PDF”双导出流程；单文件 HTML 负责交互，PDF 负责轻量分发。

### session_id: 20260313-1608-cpp-refactor-baseline-r1

- timestamp: 2026-03-13 16:08 (local)
- objective: 实施 C++ 修缮计划的第一阶段，在不改变当前 solver 数值口径的前提下拆分 app-layer 大文件、补回归护栏并建立维护基线文档。
- scope:
  - 将 `LoadDataset`、时间裁剪与 GNSS 列数探测逻辑从 `src/app/pipeline_fusion.cpp` 抽离到新的 `src/app/dataset_loader.cpp`，把主循环文件收敛到运行调度职责。
  - 扩展 `apps/regression_checks_main.cpp`，补 `BuildAnchors` 契约测试，并新增 `docs/cpp_refactor_baseline.md` 记录当前稳定边界、兼容锁与回归命令。
  - 重新编译 `eskf_fusion/regression_checks`，执行 `regression_checks` 与 `config_data2_baseline_eskf.yaml` smoke run 验证结构整理未改变 baseline 结果。
- changed_files:
  - `CMakeLists.txt`
  - `apps/regression_checks_main.cpp`
  - `src/app/dataset_loader.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `docs/cpp_refactor_baseline.md`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 220`
  - `Get-Content apps/regression_checks_main.cpp`
  - `Get-Content src/app/{config.cpp,initialization.cpp,pipeline_fusion.cpp}`
  - `rg -n "Dataset LoadDataset|FusionResult RunFusion|vector<ImuData> BuildImuSequence|MatrixXd CropMatrixRows|DetectNumericColumnCount" src/app/pipeline_fusion.cpp`
  - `cmake --build build --config Release --target eskf_fusion regression_checks`
  - `build\Release\regression_checks.exe`
  - `build\Release\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`
- artifacts:
  - `build/Release/eskf_fusion.exe`
  - `build/Release/regression_checks.exe`
  - `SOL_data2_baseline_eskf.txt`
  - `docs/cpp_refactor_baseline.md`
  - `src/app/dataset_loader.cpp`
- metrics:
  - `regression_checks: PASS`
  - 新增 `BuildAnchors` 契约检查：`fixed-empty` 允许纯惯导、显式 `fixed` 基站数量保真、`auto + empty truth` fail-fast。
  - baseline smoke 对 `config_data2_baseline_eskf.yaml` 的 RMSE xyz / 3D 为 `0.827032 / 0.877094 / 0.879851 / 1.492452`，与当前正式 baseline 口径一致。
  - `pipeline_fusion.cpp` 已不再承载 `LoadDataset` 与时间裁剪逻辑；该职责迁移到 `src/app/dataset_loader.cpp`。
- artifact_mtime:
  - `build/Release/eskf_fusion.exe`: `2026-03-13 16:20:54`
  - `build/Release/regression_checks.exe`: `2026-03-13 16:20:56`
  - `SOL_data2_baseline_eskf.txt`: `2026-03-13 16:21:54`
  - `docs/cpp_refactor_baseline.md`: `2026-03-13 16:19:51`
- config_hash_or_mtime:
  - 本会话未改动根 `config_*.yaml`；`config_data2_baseline_eskf.yaml` 仅作为 smoke 验证入口使用。
  - 未改动 `fusion.ablation.*`、`fusion.post_gnss_ablation.*`、`fusion.gnss_schedule.*`、`fusion.noise.*` 或任何 solver 数值配置默认值。
- dataset_time_window:
  - data2 baseline smoke: `528076.009368 -> 530488.900000`
- result_freshness_check:
  - pass（build、`regression_checks` 与 baseline smoke 均在本会话 fresh 执行；`SOL_data2_baseline_eskf.txt` 由当前 build fresh 生成）
- observability_notes:
  - 本会话不涉及 `21-30` 状态块、`fusion.ablation.*`、`post_gnss_ablation.*`、`gnss_schedule.*` 或任何量测/过程模型公式改动。
  - 影响仅限工程结构：把数据加载和时间窗裁剪从主调度文件抽离，降低后续整理 `ZUPT/NHC/ODO/UWB/GNSS` 调度逻辑时误碰数值路径的风险。
- decision:
  - C++ 修缮第一阶段通过：可在保持 baseline 数值口径不变的前提下继续拆分 `config.cpp` 和 `diagnostics.cpp`。
  - 后续所有结构性整理都应继续以 `regression_checks + config_data2_baseline_eskf.yaml smoke` 作为最小门槛。
- next_step:
  - 把 `config.cpp` 中的 YAML 兼容解析与运行时构造逻辑继续拆开，优先隔离 deprecated/legacy 配置语义。
  - 随后拆分 `diagnostics.cpp`，区分运行安全日志与研究归因日志，继续压缩 app-layer 热点文件。

### session_id: 20260313-1728-inekf-doc-code-review-r2

- timestamp: 2026-03-13 17:28 (local)
- objective: 以“算法文档与当前程序一致性”为主线，对核心 C++ 融合实现做静态代码审查，优先排查算法层面的潜在 bug，并区分真实数值问题与文档口径过强的问题。
- scope:
  - 复核 `true_iekf` 的 process model、`GNSS_POS/GNSS_VEL`、`ODO/NHC` 与 reset 实现，确认当前基线 ESKF/NED 重构后的代码路径不再沿用旧的 ECEF-state-error 结论。
  - 将 `src/core/{ins_mech.cpp,measurement_models_uwb.cpp,eskf_engine.cpp}` 与 `算法文档/{kf-gins-docs.tex,可观性分析讨论与InEKF算法.tex}` 对照，重点审查 `GNSS_POS` 右不变残差、核心状态列序与 reset 口径。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `config_data2_gnss30_true_iekf.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 260`
  - `rg -n "BuildProcessModel|true_iekf|ComputeNhcModel|ComputeOdoModel|ComputeGnssPositionModel|ComputeGnssVelocityModel|InjectErrorState|BuildTrueInEkfResetGamma" src\core\ins_mech.cpp src\core\measurement_models_uwb.cpp src\core\eskf_engine.cpp`
  - `rg -n "GNSS.*position|GNSS.*velocity|ODO|NHC|Gamma|rho_p|rho_v|phi|Lie" 算法文档\kf-gins-docs.tex 算法文档\可观性分析讨论与InEKF算法.tex InEKF.md`
  - `Get-Content src\core\measurement_models_uwb.cpp`（按行号抽查 `ComputeNhcModel`、`ComputeOdoModel`、`ComputeGnssPositionModel`、`ComputeGnssVelocityModel`）
  - `Get-Content src\core\eskf_engine.cpp`（按行号抽查 `InjectErrorState`、`BuildTrueInEkfResetGamma`）
  - `Get-Content src\core\ins_mech.cpp`（按行号抽查 `BuildProcessModel` 的 true_iekf override）
  - `Get-Date -Format "yyyyMMdd-HHmm"`
- artifacts:
  - `walkthrough.md`
- metrics:
  - 静态审查高置信度 findings: `2` 个 doc-code mismatch（`GNSS_POS` estimate-independence overclaim、Lie-core 列序 notation drift）。
  - 未发现新的高置信度 baseline ESKF / true_iekf 核心数值 bug；当前高风险面主要仍集中在 `true_iekf` 文档口径，而不是基础 NED refactor 代码。
  - `GNSS_VEL` 与 `21-30` 欧氏扩展块仍属于既有 `ISSUE-007` 范畴：当前实现不是“全31维完全 invariant”，但这次未发现比已知问题更高优先级的新数值错误。
- artifact_mtime:
  - `walkthrough.md`: `2026-03-13`（本会话追加 session log 与 inconsistency rows）
- config_hash_or_mtime:
  - 本会话未改动任何 solver 配置；`config_data2_baseline_eskf.yaml` 与 `config_data2_gnss30_true_iekf.yaml` 仅用于确认当前代码口径与审查上下文。
- dataset_time_window:
  - N/A（本会话未新增实验运行，仅做静态审查）。
- result_freshness_check:
  - pass（findings 直接绑定当前源码行号与当前算法文档内容，无历史产物复用歧义）。
- observability_notes:
  - 导航核心 `1-9`：`true_iekf` 的 `GNSS_POS` 核心 Jacobian 与 reset 核心块在代码中已相互匹配，但 `GNSS_POS` 协方差仍通过当前 `C_bn` 旋转到 body residual 坐标，因此“整次更新完全 estimate-independent”并不成立。
  - 扩展状态 `28-30`（`gnss_lever`）：当前 `GNSS_POS` 代码与文档都使用 `b` 系参数化，扩展列取 `I_3`；未发现新的杆臂列错误。
  - 扩展状态 `21-30` 与 `GNSS_VEL`：本会话未新增时窗/ablation 证据；维持 `ISSUE-007` 与 `HYP-7/HYP-8/HYP-12` 的现有判断，即当前 remaining gap 更偏向 process/reset 与 GNSS 窗口内耦合建立不足，而非基础 measurement Jacobian 再次写错。
- decision:
  - 当前程序最需要优先修的不是 baseline 数值公式，而是 `true_iekf` 文档口径：应把 `GNSS_POS` 的结论收敛为“`H` 对核心块 estimate-independent，但若 `R_ned` 非各向同性，则更新增益仍经 `C_bn` 进入当前估计依赖”。
  - 同步统一文档中的 Lie-core 列序记号；凡是映射到代码状态索引 `1-9` 的公式，统一明确为 `(\rho_p^b,\rho_v^b,\phi^b)`，避免后续按 `(\phi,\rho_v,\rho_p)` 误抄列位置。
- next_step:
  - 先修改 `算法文档/可观性分析讨论与InEKF算法.tex` 的 `GNSS_POS` 节与算法框架总述，收紧“strict estimate-independent update”的说法，并在公式附近显式标注“抽象推导顺序 vs 代码状态列序”。
  - 若继续追 `data2 GNSS30` 剩余 gap，再回到 `GNSS` 有效窗口内的 `P(att_z,bg_z)`、对应 gain 与 `dx_bg_z` 时序，而不是重开已审过的 `ODO/NHC/GNSS_VEL` Jacobian 方向。

### session_id: 20260315-2114-data2-rtk-outage-eskf-r1

- timestamp: 2026-03-15 21:14 (local)
- objective: 按用户要求清空 `archive/`，并只恢复 `data2` 的 corrected GNSS outage ESKF 对照实验。
- scope:
  - 物理清空 `archive/` 下全部历史结果目录，停止继续引用其中内容。
  - 修改 `scripts/analysis/filter_gnss_outage.py`，让 GNSS outage 生成器支持“初始连续可用 + 周期从 off 开始”的调度，并将默认输入切到 `dataset/data2/rtk.txt`。
  - 基于 `config_data2_baseline_eskf.yaml` 派生 fresh `rtk` 全程控制组与 `rtk` 周期开断组配置，运行 `eskf_fusion.exe` 并生成同口径 `metrics.csv` / `summary.md`。
- changed_files:
  - `scripts/analysis/filter_gnss_outage.py`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/`
  - `walkthrough.md`
- configs:
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/cfg_data2_baseline_eskf_rtk_full.yaml`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/cfg_data2_eskf_rtk_outage_cycle.yaml`
- commands:
  - `git status --short`
  - `python -m py_compile scripts\analysis\filter_gnss_outage.py`
  - `Remove-Item archive\* -Recurse -Force`
  - `python scripts\analysis\filter_gnss_outage.py --output output\review\EXP-20260315-data2-rtk-outage-eskf-r1\GNSS_outage_cycle_rtk_300on_100off_150on.txt`
  - `build\Release\eskf_fusion.exe --config output\review\EXP-20260315-data2-rtk-outage-eskf-r1\cfg_data2_baseline_eskf_rtk_full.yaml`
  - `build\Release\eskf_fusion.exe --config output\review\EXP-20260315-data2-rtk-outage-eskf-r1\cfg_data2_eskf_rtk_outage_cycle.yaml`
  - `python - <<'PY'`（复用 `scripts/analysis/interactive_nav_report.py` 的统计逻辑生成 `metrics.csv` / `summary.md`）
  - `python - <<'PY'`（生成 `gnss_outage_stats.json` 并核对 outage 时间窗）
- artifacts:
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/GNSS_outage_cycle_rtk_300on_100off_150on.txt`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/SOL_data2_baseline_eskf_rtk_full.txt`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/SOL_data2_eskf_rtk_outage_cycle.txt`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/data2_baseline_eskf_rtk_full.stdout.txt`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/data2_eskf_rtk_outage_cycle.stdout.txt`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/metrics.csv`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/summary.md`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/gnss_outage_stats.json`
- metrics:
  - `archive/` 删除前共有 `631` 个条目、累计约 `17.88 GB`；本会话后 `archive/` 已清空。
  - corrected `data2` GNSS 输入改为 `dataset/data2/rtk.txt`（`7` 列，仅位置与位置标准差，无速度）；solver stdout 显示 `GNSS data: 2853 records (converted LLA->ECEF)` / `1513 records (converted LLA->ECEF)`，未出现 `with velocity`。
  - outage GNSS 文件在分析时间窗 `[528076.0, 530488.9]` 内保留 `1513/2413` 条记录，keep ratio=`0.627020`；首个 outage 窗口 `[528376.000, 528476.000)`，首个恢复窗口 `[528476.000, 528626.000)`。
  - full GNSS ESKF (`rtk`): RMSE3D=`1.528524 m`，P95=`1.621292 m`，tail70=`1.531877 m`，final3D=`1.469204 m`。
  - outage ESKF (`rtk`, `300 + 100 off / 150 on`): RMSE3D=`6.072556 m`，P95=`14.091614 m`，tail70=`6.498556 m`，final3D=`1.538671 m`。
  - delta (outage - full): RMSE3D=`+4.544032 m`，final3D=`+0.069467 m`。
- artifact_mtime:
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/GNSS_outage_cycle_rtk_300on_100off_150on.txt`: `2026-03-15 21:09:50`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/cfg_data2_baseline_eskf_rtk_full.yaml`: `2026-03-15 21:10:31`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/cfg_data2_eskf_rtk_outage_cycle.yaml`: `2026-03-15 21:10:31`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/SOL_data2_baseline_eskf_rtk_full.txt`: `2026-03-15 21:11:26`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/SOL_data2_eskf_rtk_outage_cycle.txt`: `2026-03-15 21:11:26`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/metrics.csv`: `2026-03-15 21:12:50`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/summary.md`: `2026-03-15 21:13:55`
  - `output/review/EXP-20260315-data2-rtk-outage-eskf-r1/gnss_outage_stats.json`: `2026-03-15 21:13:42`
- config_hash_or_mtime:
  - 根配置 `config_data2_baseline_eskf.yaml` 本会话未修改。
  - `cfg_data2_baseline_eskf_rtk_full.yaml` 与 `cfg_data2_eskf_rtk_outage_cycle.yaml` 均从 `config_data2_baseline_eskf.yaml` fresh 派生，仅修改 `fusion.gnss_path` 与 `fusion.output_path`。
- dataset_time_window:
  - data2 corrected outage recovery: `528076.0 -> 530488.9`
- result_freshness_check:
  - pass（GNSS outage 文件、两组配置、两组 `SOL`、`metrics.csv` 与 `summary.md` 均在本会话 fresh 生成）
- observability_notes:
  - 本会话未触发 `fusion.ablation.*` 或 `fusion.post_gnss_ablation.*`；状态块 `21-30`（尺度、安装角、杆臂）未被显式冻结或消融，变化仅来自 GNSS 调度与 corrected GNSS source 切换。
  - 传感器窗口改为：导航核心 `1-9` 在 `0-300 s` 与其后每个 `150 s` GNSS-on 窗口内受 `GNSS_POS` 约束，在每个 `100 s` GNSS-off 窗口内仅依赖 IMU + ODO + NHC 传播/更新；`GNSS_VEL` 整个 data2 corrected 口径均不存在。
  - 行为结论：相对 `rtk` 全程 GNSS ESKF，新的 corrected outage ESKF 在 `RMSE3D/P95/tail70` 上明显退化（`1.528524/1.621292/1.531877 -> 6.072556/14.091614/6.498556`），说明该 schedule 对核心导航状态的长期约束显著减弱；末端 `final3D` 仅小幅变差（`1.469204 -> 1.538671 m`）。
- decision:
  - 按用户要求完成 `archive/` 物理清空，并建立了当前唯一 fresh 的 corrected GNSS 证据：`EXP-20260315-data2-rtk-outage-eskf-r1`。
  - 在后续完成 corrected rerun 之前，不再把 `2026-03-15` 前依赖旧 GNSS source 的 data2/data4 结果当作当前正式结论引用。
  - `data2` 的 corrected GNSS outage 对照当前只保留 ESKF；true_iekf 与 reader-facing 报告重导出均延后到 source-fix rerun 范围明确之后。
- next_step:
  - 先决定 corrected GNSS source 下是否继续重跑 `data2` 的 full/GNSS30/true_iekf families，再判断是否需要刷新 reader/report 产物。
  - 随后审计 `data4` 是否存在同类 GNSS source 问题；在 source 未核实前，不继续扩写基于旧口径的 data4 结论。

### session_id: 20260316-1659-data2-outage-official-r2

- timestamp: 2026-03-16 16:59 (local)
- objective: 将 `data2` 官方研究口径重定向到 `ESKF + corrected RTK + GNSS outage` 分段指标，并把 `mounting_roll` / `gnss_lever_z` 从官方状态估计中排除。
- scope:
  - 扩展 `fusion.ablation` / `fusion.post_gnss_ablation`，支持细粒度冻结 `disable_mounting_roll` 与 `disable_gnss_lever_z`。
  - 更新 `config_data2_baseline_eskf.yaml`：切换到 `dataset/data2/rtk.txt`、关闭 `GNSS_VEL` 更新，并全程启用上述两项细粒度冻结。
  - 新增 `scripts/analysis/run_data2_rtk_outage_eval.py`，生成 `data2` 官方 full/outage fresh 结果、`outage_segments.csv`、新的 `metrics.csv` / `summary.md` / `manifest.json`。
  - 运行 build / regression / fresh `data2` official outage 实验，并把结果写回 `walkthrough.md`。
- changed_files:
  - `apps/regression_checks_main.cpp`
  - `config_data2_baseline_eskf.yaml`
  - `include/app/fusion.h`
  - `scripts/analysis/filter_gnss_outage.py`
  - `scripts/analysis/run_data2_rtk_outage_eval.py`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/cfg_data2_baseline_eskf_rtk_full_official.yaml`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/cfg_data2_eskf_rtk_outage_cycle_official.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 220`
  - `git status --short --branch`
  - `python -m py_compile scripts\analysis\filter_gnss_outage.py scripts\analysis\run_data2_rtk_outage_eval.py`
  - `cmake --build build --config Release --target regression_checks eskf_fusion`
  - `build\Release\regression_checks.exe`
  - `python scripts\analysis\run_data2_rtk_outage_eval.py`
  - `python - <<'PY'`（校验 `metrics.csv` 与 `outage_segments.csv` 的 3D mean/max 聚合一致性）
- artifacts:
  - `build/Release/regression_checks.exe`
  - `build/Release/eskf_fusion.exe`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/GNSS_outage_cycle_rtk_300on_100off_150on.txt`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/SOL_data2_baseline_eskf_rtk_full_official.txt`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/SOL_data2_eskf_rtk_outage_cycle_official.txt`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/data2_baseline_eskf_rtk_full_control.stdout.txt`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/data2_eskf_rtk_outage_cycle_official.stdout.txt`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/metrics.csv`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/outage_segments.csv`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/summary.md`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/gnss_outage_stats.json`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/manifest.json`
- metrics:
  - `regression_checks: PASS`。
  - 官方 control（full GNSS，aux only）overall RMSE xyz / 3D=`0.840519/1.023716/0.927171 / 1.616821 m`，overall final3D=`1.679755 m`。
  - 官方 outage（`300 s on + 100 s off / 150 s on`）共有 `9` 个无 GNSS 时段；mean outage RMSE xyz / 3D=`4.321464/3.179879/7.044246 / 9.248204 m`，max outage RMSE xyz / 3D=`7.789375/5.797262/10.523110 / 12.303623 m`。
  - 官方 outage mean 末时刻误差 xyz / 3D=`8.478422/9.153287/20.101192 / 24.330687 m`；max 末时刻误差 xyz / 3D=`15.958317/32.881419/60.577294 / 70.749305 m`。
  - 第一段 outage 窗口仍为 `[528376.000, 528476.000)`；outage GNSS keep ratio=`1513/2413=0.627020`。
  - `metrics.csv` 与 `outage_segments.csv` 的 `mean/max outage rmse/final 3d` 聚合一致性校验通过，diff 均为 `0`（浮点舍入误差内）。
- artifact_mtime:
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/metrics.csv`: `2026-03-16 16:58:23`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/outage_segments.csv`: `2026-03-16 16:58:23`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/summary.md`: `2026-03-16 16:58:23`
  - `output/review/EXP-20260316-data2-rtk-outage-eskf-r2/manifest.json`: `2026-03-16 16:58:23`
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`：本会话改为 `dataset/data2/rtk.txt` + `enable_gnss_velocity=false` + `ablation.disable_mounting_roll/disable_gnss_lever_z=true`。
  - `cfg_data2_baseline_eskf_rtk_full_official.yaml` 与 `cfg_data2_eskf_rtk_outage_cycle_official.yaml` 均由新的官方 baseline fresh 派生，仅修改 `fusion.output_path` 与 outage case 的 `fusion.gnss_path`。
- dataset_time_window:
  - data2 official outage evaluation: `528076.0 -> 530488.9`
- result_freshness_check:
  - pass（build、regression、GNSS outage 文件、两组官方配置、两组 `SOL`、`metrics.csv`、`outage_segments.csv`、`summary.md`、`manifest.json` 均在本会话 fresh 生成）
- observability_notes:
  - 状态块 `22`（`mounting_roll`）: 在 official full/outage 两个 config 中全程冻结；激活窗口为全导航时段，行为归类为 `neutral(protocol change)`，本会话未做与“自由估计 roll”同口径 A/B，只将其从官方估计目标中移出。
  - 状态块 `30`（`gnss_lever_z`）: 在 official full/outage 两个 config 中全程冻结；激活窗口为全导航时段，行为归类为 `neutral(protocol change)`，本会话未做与“自由估计 lever_z”同口径 A/B，只将其从官方估计目标中移出。
  - schedule effects (`fusion.gnss_schedule.*` 未启用，GNSS 可用性由 outage 文件决定): `0-300 s` GNSS on，随后循环 `100 s off + 150 s on`；对导航核心 `1-9` 与其余仍启用的扩展块而言，GNSS-off 段相对 full control 呈 `degraded`，具体体现在 outage mean/max RMSE3D=`9.248204/12.303623 m` 与 mean/max final3D=`24.330687/70.749305 m`。
- decision:
  - `EXP-20260316-data2-rtk-outage-eskf-r2` 取代 `r1`，成为当前唯一正式 `data2 ESKF` 研究口径：corrected RTK source、无 `GNSS_VEL`、并以 outage segment metrics 而非总体 RMSE 为正式评价。
  - `config_data2_baseline_eskf.yaml` 已升级为新的官方 baseline；后续 data2 ESKF 优化必须继承该 config，而不是回到旧的 `GNSS_converted` / overall RMSE 口径。
  - `EXP-20260315-data2-rtk-outage-eskf-r1` 仅保留为 corrected source recovery 过渡证据，不再作为当前正式评价基线。
- next_step:
  - 基于 `EXP-20260316-data2-rtk-outage-eskf-r2`，开始只针对 `data2 ESKF` 做算法优化，所有排序统一使用 outage segment mean/max 指标。
  - 做一次同 schedule 的 direct ablation A/B，定量比较“自由估计 vs 冻结 `mounting_roll` / `gnss_lever_z`”对新 outage 指标的影响，补上本会话未做的 protocol-change 量化。
  - 在 data2 ESKF 新口径稳定前，暂不恢复 data4 或 true_iekf 的正式结论扩写；若后续要扩展，需先沿用同一套分段 outage 指标。

### session_id: 20260316-2052-output-normalization-r1

- timestamp: 2026-03-16 20:52 (local)
- objective: 清理旧版 `output/` 并将当前唯一正式 `data2 ESKF GNSS outage` 结果标准化到 `output/data2_eskf_baseline/`。
- scope:
  - 为 `plot_navresult.py` 增加显式标准模式与显式输出目录，官方链路只生成误差曲线和非 PVA 状态曲线。
  - 重写 `scripts/analysis/run_data2_rtk_outage_eval.py`，将 `metrics.csv/outage_segments.csv/summary.md/manifest.json/plot/artifacts` 统一输出到标准结果目录，并把 control 仅保留为辅助证据。
  - 新增 `scripts/analysis/cleanup_legacy_output.py`，先列出旧版 `output/` 删除清单，再按白名单删除历史 `result_* / compare_* / review/* / logs/*` 等产物。
- changed_files:
  - `plot_navresult.py`
  - `scripts/analysis/run_data2_rtk_outage_eval.py`
  - `scripts/analysis/cleanup_legacy_output.py`
  - `output/data2_eskf_baseline/`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_eskf_baseline/artifacts/cfg_data2_baseline_eskf_full_control.yaml`
  - `output/data2_eskf_baseline/artifacts/cfg_data2_eskf_outage_official.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 220`
  - `python -m py_compile plot_navresult.py scripts\analysis\run_data2_rtk_outage_eval.py scripts\analysis\cleanup_legacy_output.py`
  - `python scripts\analysis\run_data2_rtk_outage_eval.py`
  - `python scripts\analysis\cleanup_legacy_output.py`
  - `python scripts\analysis\cleanup_legacy_output.py --execute`
  - `python -`（校验 `metrics.csv` 单行、`plot/` 9 张 PNG、以及 `metrics.csv` 与 `outage_segments.csv` 的 mean/max 聚合一致性）
- artifacts:
  - `output/data2_eskf_baseline/metrics.csv`
  - `output/data2_eskf_baseline/outage_segments.csv`
  - `output/data2_eskf_baseline/summary.md`
  - `output/data2_eskf_baseline/manifest.json`
  - `output/data2_eskf_baseline/plot/{error_position_xyz,error_position_3d,state_gyro_bias,state_accel_bias,state_gyro_scale,state_accel_scale,state_mounting_odo_scale,state_odo_lever_arm,state_gnss_lever_arm}.png`
  - `output/data2_eskf_baseline/artifacts/{cfg_data2_baseline_eskf_full_control.yaml,cfg_data2_eskf_outage_official.yaml,SOL_data2_baseline_eskf_full_control.txt,SOL_data2_eskf_outage_official.txt,data2_baseline_eskf_full_control.stdout.txt,data2_eskf_outage_official.stdout.txt,GNSS_outage_cycle_rtk_300on_100off_150on.txt,gnss_outage_stats.json,plot_generation.stdout.txt,legacy_cleanup_report.json}`
- metrics:
  - `metrics.csv` 行数=`1`；`plot/` PNG 数量=`9`；目录结构满足 `metrics.csv/outage_segments.csv/summary.md/manifest.json/plot/artifacts` 固定规范。
  - official outage mean/max RMSE3D=`9.248204/12.303623 m`；official outage mean/max final3D=`24.330687/70.749305 m`；segments=`9`。
  - control overall RMSE3D(aux)=`1.616821 m`；official overall RMSE3D(aux)=`6.086996 m`。
  - `metrics.csv` 与 `outage_segments.csv` 聚合校验通过：`mean/max outage rmse/final 3d` diff 全部为 `0`（浮点舍入误差内）。
  - `cleanup_legacy_output.py` dry-run / execute 均输出同一份删除候选；最终删除旧版 `output/` 顶层条目共 `62` 项，清理后 `output/` 下仅剩 `data2_eskf_baseline/`。
- artifact_mtime:
  - `output/data2_eskf_baseline/metrics.csv`: `2026-03-16 20:50:05`
  - `output/data2_eskf_baseline/outage_segments.csv`: `2026-03-16 20:50:05`
  - `output/data2_eskf_baseline/summary.md`: `2026-03-16 20:50:11`
  - `output/data2_eskf_baseline/manifest.json`: `2026-03-16 20:52:12`
  - `output/data2_eskf_baseline/artifacts/legacy_cleanup_report.json`: `2026-03-16 20:51:26`
- config_hash_or_mtime:
  - 本会话未修改 `config_data2_baseline_eskf.yaml` 的数值内容，仅复用当前官方 baseline 重新派生 control / outage 配置快照。
- dataset_time_window:
  - data2 standardized official outage evaluation: `528076.0 -> 530488.9`
- result_freshness_check:
  - pass（标准结果目录、9 张标准图、删除清单以及清理后的 `output/` 目录结构均在本会话 fresh 生成/验证）
- observability_notes:
  - 状态块 `22`（`mounting_roll`）与 `30`（`gnss_lever_z`）的 official freeze 口径保持不变；本会话只重构输出链路，不改算法或 schedule。
  - GNSS 可用窗口保持 `300 s on + 100 s off / 150 s on` 循环，第一段 outage 仍为 `[528376.0, 528476.0)`；新标准误差图使用该分段结果做阴影标注。
  - 行为归类为 `neutral(protocol/output refactor)`：数值评价口径与上一 official outage baseline 一致，变化只在 canonical artifact layout 与正式引用路径。
- decision:
  - 当前唯一正式结果路径固定为 `output/data2_eskf_baseline/`；后续 `data2 ESKF` 官方排序、汇报和引用均应从该目录读取。
  - 旧 `output/review/*`、`output/result_*`、`output/compare_*`、`output/logs/*` 等旧版输出已按白名单清理删除，不再作为当前 evidence path。
  - `plot_navresult.py` 的 legacy 行为继续保留给历史脚本兼容；但官方链路必须使用新增 standard mode，把图写入目标结果目录的 `plot/`。
- next_step:
  - 在 `output/data2_eskf_baseline/` 这一 canonical 目录上继续做 `data2 ESKF` 算法优化，统一按 outage 分段指标排序。
  - 若新增正式实验结果，沿用 `output/<dataset>_<method>_<case>/` 命名，并保持同一套 `metrics/outage_segments/summary/manifest/plot/artifacts` 结构。
  - 补做 `mounting_roll` / `gnss_lever_z` direct A/B ablation 时，直接复用新的标准结果目录规范，避免重新生成旧式 `result_*` 目录。

### session_id: 20260316-2106-output-plot-refine-r1

- timestamp: 2026-03-16 21:06 (local)
- objective: 按用户要求细化 `output/data2_eskf_baseline/plot/`，补齐速度误差、姿态误差与 outage 分段柱状图，并移除 `3D` 误差图。
- scope:
  - 修改 `plot_navresult.py` 的 standard mode，仅输出位置/速度/姿态三分量误差图，不再输出 `error_position_3d.png`。
  - 基于 `outage_segments.csv` 额外生成两张柱状图：各段 `rmse_x/y/z` 与各段 `final_err_x/y/z`。
  - 更新 `scripts/analysis/run_data2_rtk_outage_eval.py` 的标准图校验清单，并 fresh 重跑当前官方结果目录。
- changed_files:
  - `plot_navresult.py`
  - `scripts/analysis/run_data2_rtk_outage_eval.py`
  - `output/data2_eskf_baseline/`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_eskf_baseline/artifacts/cfg_data2_baseline_eskf_full_control.yaml`
  - `output/data2_eskf_baseline/artifacts/cfg_data2_eskf_outage_official.yaml`
- commands:
  - `Get-Content walkthrough.md -TotalCount 80`
  - `python -m py_compile plot_navresult.py scripts\analysis\run_data2_rtk_outage_eval.py`
  - `python scripts\analysis\run_data2_rtk_outage_eval.py`
  - `python -`（校验 `plot/` 文件清单、确认不存在 `error_position_3d.png`，以及核对 `metrics.csv` 与 `outage_segments.csv` 聚合一致性）
- artifacts:
  - `output/data2_eskf_baseline/plot/{error_position_xyz,error_velocity_xyz,error_attitude_xyz,outage_segment_rmse_xyz_bar,outage_segment_final_err_xyz_bar,state_gyro_bias,state_accel_bias,state_gyro_scale,state_accel_scale,state_mounting_odo_scale,state_odo_lever_arm,state_gnss_lever_arm}.png`
  - `output/data2_eskf_baseline/artifacts/plot_generation.stdout.txt`
  - `output/data2_eskf_baseline/manifest.json`
  - `output/data2_eskf_baseline/metrics.csv`
  - `output/data2_eskf_baseline/outage_segments.csv`
- metrics:
  - `plot/` PNG 数量=`12`；新增误差图=`error_velocity_xyz.png`,`error_attitude_xyz.png`；新增柱状图=`outage_segment_rmse_xyz_bar.png`,`outage_segment_final_err_xyz_bar.png`；`error_position_3d.png` 不再存在。
  - official outage mean/max RMSE3D=`9.248204/12.303623 m`；official outage mean/max final3D=`24.330687/70.749305 m`；数值评价结果与前一版输出规范保持一致。
  - `metrics.csv` 与 `outage_segments.csv` 聚合校验继续通过：`mean/max outage rmse/final 3d` diff 全部为 `0`（浮点舍入误差内）。
- artifact_mtime:
  - `output/data2_eskf_baseline/metrics.csv`: `2026-03-16 21:05:38`
  - `output/data2_eskf_baseline/outage_segments.csv`: `2026-03-16 21:05:38`
  - `output/data2_eskf_baseline/manifest.json`: `2026-03-16 21:05:45`
  - `output/data2_eskf_baseline/artifacts/plot_generation.stdout.txt`: `2026-03-16 21:05:45`
- config_hash_or_mtime:
  - 本会话未修改 `config_data2_baseline_eskf.yaml` 数值配置，只刷新标准输出图集与 `manifest.exp_id`。
- dataset_time_window:
  - data2 standardized official outage evaluation: `528076.0 -> 530488.9`
- result_freshness_check:
  - pass（12 张标准图、重跑后的 `metrics.csv/outage_segments.csv/manifest.json` 以及标准绘图日志均在本会话 fresh 生成）
- observability_notes:
  - 仅调整结果展示层，不改 `mounting_roll` / `gnss_lever_z` 冻结口径，也不改 outage schedule；GNSS-off 段阴影仍与 `[528376.0, 528476.0)` 等分段边界一致。
  - 新增速度误差和姿态误差三分量图后，post-GNSS 漂移可直接从 `plot/` 目录做 reader-facing 浏览，但这不构成新的算法结论。
- decision:
  - `output/data2_eskf_baseline/plot/` 的正式规范更新为 `12` 张图：`3` 张三分量误差图、`2` 张 outage 段柱状图、`7` 张非 PVA 状态图。
  - 正式结果图中不再展示 `3D` 误差曲线；浏览时优先看三个方向误差和每段柱状图。
- next_step:
  - 后续 `data2 ESKF` 优化实验统一沿用这一版 `12` 图标准，避免回退到 `3D` 误差 headline。
  - 若后续需要更强 reader-facing 汇报，可在不改指标口径的前提下继续给柱状图补排序或数值标注。

### session_id: 20260316-2324-data2-state-sanity-r1

- timestamp: 2026-03-16 23:24 (local)
- objective: 按“关闭状态=真值锚定 + 小噪声、实验对象=零初值 + 大噪声”的新语义，对 `data2 official outage ESKF` 做非 PVA `22` 维单状态释放矩阵，判断状态量是否正常变化。
- scope:
  - 扩展 solver/config/result plumbing：新增 `fusion.state_series_output_path`、逐轴向量过程噪声优先回退逻辑，并把 `mounting_roll` 加入实验状态 CSV 导出。
  - 新增 `scripts/analysis/run_data2_state_sanity_matrix.py`，统一生成 outage GNSS 文件、控制组与 `22` 个单状态实验配置、运行 solver、汇总导航指标、判定状态行为并输出图表。
  - 先完成 `truth_anchor_all_non_pva + release_bg_z + release_gnss_lever_y` smoke，再跑完整 `23` case 核心矩阵。
- changed_files:
  - `apps/eskf_fusion_main.cpp`
  - `apps/regression_checks_main.cpp`
  - `scripts/analysis/run_data2_state_sanity_matrix.py`
  - `src/app/config.cpp`
  - `src/app/evaluation.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/ins_mech.cpp`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_eskf_state_sanity/artifacts/cases/truth_anchor_all_non_pva/config_truth_anchor_all_non_pva.yaml`
  - `output/data2_eskf_state_sanity/artifacts/cases/release_bg_y/config_release_bg_y.yaml`
  - `output/data2_eskf_state_sanity/artifacts/cases/release_mounting_roll/config_release_mounting_roll.yaml`
  - `output/data2_eskf_state_sanity/artifacts/cases/release_gnss_lever_z/config_release_gnss_lever_z.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content scripts\analysis\run_data2_state_sanity_matrix.py`
  - `Get-Content config_data2_baseline_eskf.yaml`
  - `Get-Content scripts\analysis\run_data2_rtk_outage_eval.py`
  - `Get-Content scripts\analysis\filter_gnss_outage.py`
  - `cmake --build build --config Release --target eskf_fusion regression_checks`
  - `build\Release\regression_checks.exe`
  - `python -m py_compile scripts\analysis\run_data2_state_sanity_matrix.py`
  - `python scripts\analysis\run_data2_state_sanity_matrix.py --cases truth_anchor_all_non_pva release_bg_z release_gnss_lever_y`
  - `python scripts\analysis\run_data2_state_sanity_matrix.py`
- artifacts:
  - `output/data2_eskf_state_sanity/case_metrics.csv`
  - `output/data2_eskf_state_sanity/state_judgement.csv`
  - `output/data2_eskf_state_sanity/truth_reference.json`
  - `output/data2_eskf_state_sanity/summary.md`
  - `output/data2_eskf_state_sanity/manifest.json`
  - `output/data2_eskf_state_sanity/plots/{state_judgement_heatmap,mounting_roll_std_compare,*.png}`
  - `output/data2_eskf_state_sanity/artifacts/GNSS_outage_cycle_rtk_300on_100off_150on.txt`
  - `output/data2_eskf_state_sanity/artifacts/cases/*/{config_*.yaml,SOL_*.txt,state_series_*.csv,DIAG_*.txt,*stdout.txt,outage_segments_*.csv}`
- metrics:
  - `regression_checks: PASS`；新增 `state_series_output_path` / vector-noise parser / `SaveStateSeries` smoke / `bg_z` 逐轴噪声优先测试均通过。
  - smoke subset（`truth_anchor_all_non_pva`, `release_bg_z`, `release_gnss_lever_y`）先行通过，且 `bg_z`、`gnss_lever_y` 都被判为 `abnormal`，与既有可疑状态预期一致。
  - full matrix 完整性：`case_count=23`，`judgement_count=22`，所有 case 均具备 `config + stdout + SOL + state_series + DIAG + outage_segments`；`plots/*.png=24`。
  - 控制组 `truth_anchor_all_non_pva`：`mean_outage_rmse_3d_m=12.063017`，`max_outage_final_err_3d_m=45.713949`，`overall_rmse_3d_m_aux=7.781084`。
  - 全矩阵判定：`overall abnormal/borderline/normal = 18/1/3`；`impact abnormal/borderline/normal = 4/1/17`。
  - 最强异常状态为 `bg_y/bg_z/bg_x/sg_z`，其 `delta_mean_rmse3d = 320.215229 / 4.643153 / 1.072376 / 1.055609`，`delta_max_final3d = 827.053564 / 4.647732 / 1.042796 / 1.300455`。
  - 当前 `normal` 状态仅 `mounting_roll`、`gnss_lever_z`、`sa_z`；`odo_scale` 为唯一 `borderline`；既有异常候选 `sg_y` / `sa_x` / `gnss_lever_y` 全部仍为 `abnormal`。
- artifact_mtime:
  - `output/data2_eskf_state_sanity/case_metrics.csv`: `2026-03-16 23:24:20`
  - `output/data2_eskf_state_sanity/state_judgement.csv`: `2026-03-16 23:24:20`
  - `output/data2_eskf_state_sanity/summary.md`: `2026-03-16 23:24:20`
  - `output/data2_eskf_state_sanity/manifest.json`: `2026-03-16 23:24:20`
  - `output/data2_eskf_state_sanity/truth_reference.json`: `2026-03-16 23:05:51`
- config_hash_or_mtime:
  - 本会话复用 `config_data2_baseline_eskf.yaml` 作为显式基线，不改其 official baseline 数值；所有实验 config 均由脚本显式写出 `P0_diag`、`noise.*`、`state_series_output_path`、`enable_gnss_velocity=false`、`ablation/post_gnss_ablation` 全关。
  - “关闭状态”语义已按用户最新要求改为真值锚定：非目标非 PVA 状态采用参考值初始化并赋予很小初始/过程噪声，不再使用 mask freeze。
- dataset_time_window:
  - data2 state sanity matrix: `528076.0 -> 530488.9`
- result_freshness_check:
  - pass（build/regression、smoke/full matrix、`23` 份 case configs、`23` 份 `SOL`/state CSV/DIAG、以及根目录 `case_metrics/state_judgement/summary/manifest` 均在本会话 fresh 生成）
- observability_notes:
  - 状态块 `10-12`（`ba_x/ba_y/ba_z`）: `0-300 s` GNSS-on + 后续 `100 s off / 150 s on` 循环下，单独释放后 `behavior=abnormal` 但 `impact=normal`，说明当前 outage 口径下更像是“状态自身不稳定/不可稳健估计”而非直接导航主风险。
  - 状态块 `13-15`（`bg_x/bg_y/bg_z`）: 单独释放后全部 `behavior=abnormal` 且 `impact=abnormal`；其中 `bg_y` 为灾难性退化，`bg_z` 与 `bg_x` 也显著恶化，说明 gyro bias block 是当前 official outage 条件下最优先的风险源。
  - 状态块 `16-18`（`sg_x/sg_y/sg_z`）与 `19-21`（`sa_x/sa_y/sa_z`）: `sg_x/sg_y/sg_z/sa_x/sa_y` 为 `behavior=abnormal`，但多数 `impact=normal`；仅 `sg_z` 升级到 `impact=abnormal`。这更像可观性不足或状态定义/单位/噪声映射存在问题，而不是普遍的导航即刻失稳。
  - 状态块 `22-24`（`mounting_roll/pitch/yaw`）: `mounting_roll` 在新语义下保持 `normal`，且 `std_mr` 前后段中位数约 `0.008863 -> 0.009893 deg`；`mounting_pitch/yaw` 则 `behavior=abnormal` 但 `impact=normal`。
  - 状态块 `25-27`（`odo_lever_*`）与 `28-30`（`gnss_lever_*`）: `odo_lever_x/y/z` 与 `gnss_lever_x/y` 均 `behavior=abnormal`；`gnss_lever_z` 为 `normal`。因此与上一轮官方 freeze 口径相比，`gnss_lever_z` 不再像当前最弱状态，反而 `x/y` 更值得审计。
  - 本轮 `PVA` 全程使用 `truth`，非目标非 PVA 全部真值锚定；因此本实验结论解释的是“单状态单独释放的 sanity/observability”，不是对 official multi-state 全开配置的直接替代。
- decision:
  - `EXP-20260316-data2-state-sanity-r1` 完成了当前 `data2 official outage` 口径下的第一版全状态 sanity matrix，并建立了新的诊断基线目录 `output/data2_eskf_state_sanity/`。
  - 在“真值锚定对照 + 单状态释放”的定义下，当前可直接判为 `normal` 的只有 `mounting_roll`、`gnss_lever_z`、`sa_z`，`odo_scale` 仅 `borderline`；其余 `18` 个非 PVA 标量状态都不满足“正常变化”判据。
  - 后续优先级不再是泛泛地“继续放开更多状态”，而是先针对 `bg` block 做逻辑/单位/调参审计，再对 `sg/sa/lever/mounting_pitch/yaw` 做状态定义与 Jacobian 检查；“关闭状态=真值锚定而非 mask freeze”自此成为后续此类实验的固定语义。
- next_step:
  - 先对 `release_bg_y/bg_z/bg_x` 做定向代码审计：检查单位换算、Markov 过程噪声、GNSS/NHC/ODO 对 bias 的约束建立，以及 `DIAG` 中相关方差/Gain 的时序。
  - 再对 `sg/sa/odo_lever/gnss_lever/mounting_pitch/yaw` 做“状态定义-符号-单位-Jacobian”四联检查，判断它们是单纯弱可观，还是代码/调参存在系统性偏差。
  - 在 official baseline 上单独做 `mounting_roll` / `gnss_lever_z` 重新启用的 direct A/B，评估是否需要缩减当前 official freeze 列表。

### session_id: 20260317-0015-state_semantics_reset

- timestamp: 2026-03-17 00:15 (local)
- objective: 针对 `state semantics / masks / noise / reset / export plumbing` 做 focused audit，检查是否存在能解释“多数状态异常、少数弱可观状态却看起来正常”的共享机制。
- scope:
  - 静态审阅 `src/app/config.cpp`、`src/app/pipeline_fusion.cpp`、`src/app/evaluation.cpp`、`apps/eskf_fusion_main.cpp`、`apps/regression_checks_main.cpp`、`include/core/eskf.h`。
  - 补读直接相关实现 `src/app/initialization.cpp`、`src/core/eskf_engine.cpp`、`src/core/ins_mech.cpp`、`include/app/fusion.h`，仅用于核对语义和调用链，不改源码。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`（仅作为当前 baseline 语义上下文，未改动）
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "kStateDim|P0_diag|std_|vector|noise|ablation|disable_|mask|reset|inject|state_series|export" src/app/config.cpp src/app/pipeline_fusion.cpp src/app/evaluation.cpp apps/eskf_fusion_main.cpp apps/regression_checks_main.cpp include/core/eskf.h`
  - `rg -n "ResolveVectorNoise|ApplyStateMaskToCov|ApplyStateMaskToDx|ApplyUpdateMaskToKalmanGain|BuildTrueInEkfResetGamma|SetStateMask" src/core/eskf_engine.cpp src/core/ins_mech.cpp src/app/initialization.cpp include/app/fusion.h src/app/diagnostics.cpp`
  - `Get-Content`（带行号）上述审计文件的局部代码段
- artifacts:
  - 无新增运行产物；结论直接绑定当前源码行号
- metrics:
  - focused audit findings: `4` 个高/中优先级共享风险点
  - explicit non-finding: 未发现 scoped solver 内部 `31` 维状态索引在 `StateIdx / P0 / process model / mask` 间存在直接错位
- artifact_mtime:
  - N/A（未生成新结果文件）
- config_hash_or_mtime:
  - N/A（未改配置；结论基于当前工作树源码静态审计）
- dataset_time_window:
  - N/A（静态审计，无新增数据运行）
- result_freshness_check:
  - pass（findings 直接绑定 `2026-03-17` 当前源码行号；未引用历史 summary 代替代码证据）
- observability_notes:
  - 状态块 `22-24` 与 `28-30`: `fusion.ablation.*` / `fusion.post_gnss_ablation.*` 在代码里仍是 `mask freeze` 语义，不是“真值锚定 + 小噪声”；被 disable 的状态协方差/修正被清零，但名义值仍保留并继续参与量测预测。这会让弱可观块在图上显得“稳定/正常”，同时把不匹配转嫁给其余启用状态；`behavior` 对被冻块表面 `neutral`，对其它状态可能 `degraded`。
  - 状态块 `21-30` 的导出解释: `SOL_*`/`output_matrix` 不是 state-order，且 `state_series.csv` 又使用 `mGal / deg/h / ppm / deg` 并额外导出 `total_mounting=*base+delta*`；若分析脚本混用 `output_path` 与 `state_series_output_path`，会把单位、顺序和“总安装角/状态增量”混在一起，导致对可观性矩阵的解释失真。
  - `fusion.gnss_schedule.*` / `fusion.post_gnss_ablation.*`: GNSS split 后代码只调用 `engine.SetStateMask(...)`，没有把“新语义 disable=小噪声锚定”编码进 runtime；因此 post-GNSS 行为仍是硬冻结风格，而不是软锚定风格。
  - reset/injection 链路: `dx` 在注入前会经过 state mask 清零，true-InEKF reset 后也再次 `ApplyStateMaskToCov()`；因此当前没有证据表明 reset 会把已 mask 的状态重新放活。问题更像语义/导出解释，而不是 reset 打破冻结。
- decision:
  - 当前最可能解释“多数状态异常、少数弱可观状态却正常”的共享因素，不是 solver 内部 state ordering 错位，而是 `ablation=hard freeze` 语义仍存在、`P0_diag` 与 `std_*` 语义/单位差异极大、以及两类导出矩阵（`SOL` vs `state_series`）的顺序/单位并不等同于内部状态。
  - vector noise override 本身优先级没有发现反转 bug；但它是“整向量 override”，不是“单轴部分回退”，因此任何只想改一轴的小噪声锚定都必须显式给满 `3` 轴，否则会回退到标量或直接被校验拒绝。
- next_step:
  - 先确认 `EXP-20260316-data2-state-sanity-r1` 当前矩阵和图究竟读取 `SOL_*` 还是 `state_series_*.csv`；若混用，先统一映射表与单位转换，再讨论 observability 结论。
  - 再逐案核对是否仍有实验配置或脚本路径在使用 `fusion.ablation.*` / `post_gnss_ablation.*` 作为“关闭状态”；若有，需要与“真值锚定 + 小噪声”新语义彻底切开。
  - 随后对 `P0_diag` 与 `std_*` 做显式 equivalence audit，尤其检查角度类状态（`att / mounting`）是否有人按 STD/deg 误填到 `P0_diag`。

### session_id: 20260317-0138-odo_nhc_update_audit

- timestamp: 2026-03-17 01:38 (local)
- objective: 对 `ODO/NHC update chain` 做 focused audit，检查量测构造、门控、鲁棒加权、弱激励冻结与共享标定状态误驱动风险。
- scope:
  - 静态审阅 `src/core/measurement_models_uwb.cpp`、`src/app/pipeline_fusion.cpp`、`src/app/diagnostics.cpp`、`include/app/diagnostics.h`。
  - 补读直接相关定义 `include/core/eskf.h`、`src/core/eskf_engine.cpp`、`include/app/fusion.h`、`apps/regression_checks_main.cpp`、`apps/jacobian_audit_main.cpp`，仅用于核对 freeze/mask/roll 语义与审计覆盖范围，不改源码。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`（仅作为 official baseline 上下文；未改动）
- commands:
  - `Get-Content walkthrough.md`
  - `rg -n "ComputeNhcModel|ComputeOdoModel|NHC|ODO|robust|NIS|weak|excitation|mount|lever|gyro|scale" src/core/measurement_models_uwb.cpp src/app/pipeline_fusion.cpp src/app/diagnostics.cpp include/app/diagnostics.h`
  - `Get-Content`（带行号）`src/core/measurement_models_uwb.cpp`、`src/app/pipeline_fusion.cpp`、`src/app/diagnostics.cpp`、`include/app/diagnostics.h`
  - `rg -n "kMountRoll|mounting_roll|disable_mounting_roll|last_correction_debug|SetStateMask|ApplyUpdateMaskToKalmanGain|nhc_max_abs_v|freeze_extrinsics_when_weak_excitation" src/core src/app include apps`
- artifacts:
  - 无新增运行产物；结论直接绑定当前源码行号
- metrics:
  - focused audit findings: `4` 个（其中 `1` 个高优先级实现缺陷、`2` 个中优先级系统风险、`1` 个中优先级诊断失真）
  - explicit non-finding: 未在 scoped files 中看到新的 `ODO/NHC pitch/yaw` 雅可比符号反转证据；当前更突出的风险在 update-chain 语义而非该处公式表面符号
- artifact_mtime:
  - N/A（静态审计，无新产物）
- config_hash_or_mtime:
  - N/A（未改配置；结论基于当前工作树源码）
- dataset_time_window:
  - N/A（静态审计）
- result_freshness_check:
  - pass（findings 直接绑定 `2026-03-17` 当前源码；未用历史 summary 代替代码证据）
- observability_notes:
  - 状态块 `21-27`（`odo_scale`、`mounting_*`、`odo_lever_*`）: `freeze_extrinsics_when_weak_excitation` 的实现只清零 `H` 列，不施加 `update_mask`；因此在弱激励窗口里这些块仍可经由协方差互相关被 `ODO/NHC` 间接修正，代码行为是“去直接敏感度”而不是“真正冻结”，对弱可观块 `behavior=degraded risk`。
  - 状态块 `22`（`mounting_roll`）: `ODO/NHC` 预测和 Jacobian 都未把 `state.mounting_roll` 写入 `C_b_v` 或量测导数；若某配置未显式 `disable_mounting_roll`，该状态只能靠过程噪声与互协方差被间接推动，而非被真实量测约束，`behavior=neutral_or_spurious`。
  - 状态块 `12-17`（`bg/sg`）: `ComputeNhcModel/ComputeOdoModel` 始终保留 `H_bg/H_sg` 耦合；结合上面的弱激励“伪冻结”，重复的 road constraints 仍可能把 lever/mounting/scale 失配转嫁进 gyro bias / gyro scale，`behavior=degraded risk`。
  - 传感器窗口顺序仍是 `Predict -> ZUPT -> Gravity diag -> NHC -> ODO -> UWB -> GNSS -> diagnostics`；因此在 GNSS 稀疏或关闭窗口内，`ODO/NHC` 的高频更新链对共享状态的支配风险尤其值得优先修复。
- decision:
  - 当前最需要修的不是再猜测某个单独 Jacobian 列符号，而是把 `weak excitation freeze` 从“清 H 列”升级为“真正禁止对应状态更新”，否则 ODO/NHC 会继续在弱可观窗口里悄悄推动 `21-27`。
  - `mounting_roll` 目前属于“接口里有状态、process/guard/support 仍在，但 ODO/NHC measurement path 不使用”的半连接状态；official baseline 因显式 disable 而暂时避开，但代码本身对任何重新启用该状态的实验都不安全。
  - 现有 consistency/mechanism 统计不足以回答“鲁棒权重是否真的削弱了 ODO/NHC 对共享标定块的主导”，因为 `noise_scale_mean` 统计实现失真，且日志只覆盖 `att_z/bg_z/mount_yaw`，没有覆盖 `odo_scale/mount_pitch/lever/sg`。
- next_step:
  - 先把 `CorrectConstraintWithRobustness` 的弱激励分支改成真实 `update_mask`（至少覆盖 `odo_scale + mounting + lever`），并补一个最小回归：验证弱激励窗口内这些状态的 `dx/K row` 为零。
  - 随后决定 `mounting_roll` 的正式语义：要么彻底从 ODO/NHC 估计链移除并强制 mask，要么把它纳入 `C_b_v` 和 Jacobian，并同步更新数值审计工具。
  - 再补 NHC 的 pre-gate 审计：把 `nhc_max_abs_v` 检查从 `body v_b` 对齐到实际受约束的 `vehicle/wheel v_v`，并把 mechanism log 扩到 `odo_scale/mount_pitch/lever/bg/sg`。

### session_id: 20260317-0145-ins_propagation_audit

- timestamp: 2026-03-17 01:45 (local)
- objective: 对 `src/core/ins_mech.cpp`、`src/core/eskf_engine.cpp`、`include/core/eskf.h`、`src/app/initialization.cpp` 做 focused audit，检查 INS 名义传播、过程模型离散化、状态索引与注入语义是否能触发多非 PVA 状态的共享失效。
- scope:
  - 静态审阅 `StateIdx / P0 / F / G / Qd / InjectErrorState / Correct / true_iekf reset` 链路。
  - 补读 `config_data2_baseline_eskf.yaml` 与 `src/utils/math_utils.cpp`，仅用于核对当前 baseline 开关和四元数/坐标系约定。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
- commands:
  - `rg -n "kStateDim|StateIdx|Inject|Predict|BuildProcessModel|sigma_|markov_corr_time|P0" include/core/eskf.h src/core/eskf_engine.cpp src/core/ins_mech.cpp src/app/initialization.cpp`
  - `Get-Content`（带行号）`include/core/eskf.h`、`src/core/ins_mech.cpp`、`src/core/eskf_engine.cpp`、`src/app/initialization.cpp`
  - `rg -n "sigma_bg|sigma_sg|markov_corr_time|fej" config_data2_baseline_eskf.yaml src/app/config.cpp`
  - `Get-Content`（带行号）`src/utils/math_utils.cpp`
- artifacts:
  - 无新增运行产物；结论直接绑定当前源码与 baseline 配置行号
- metrics:
  - focused audit findings: `2` 个主要共享风险点
  - explicit non-findings: `31` 维状态索引在 `StateIdx / initialization / process model / inject` 间未见直接错位；ECEF 名义传播与 NED 误差态转换链未见直接 frame-order bug
- artifact_mtime:
  - N/A（静态审计，无新产物）
- config_hash_or_mtime:
  - N/A（未改配置）
- dataset_time_window:
  - N/A（静态审计）
- result_freshness_check:
  - pass（仅使用当前工作树源码与 `2026-03-17` baseline 配置内容）
- observability_notes:
  - 当前 official baseline `fusion.fej.enable=false`，因此运行时走标准 ESKF 更新分支；该分支在姿态注入后缺少协方差 reset Gamma，最可能污染所有依赖姿态互协方差建立可观性的块，优先是 `12-20`（`bg/sg/sa`），其次是 `22-30`（`mounting/lever`）；`behavior=degraded risk`。
  - `NoiseParams.markov_corr_time` 会统一作用于 `9-20` 中的 `ba/bg/sg/sa`，把 `sigma_*` 从驱动噪声密度切换为稳态标准差；在当前 baseline (`T=3600s`) 下，这会把 `bg/sg` 的等效驱动噪声显著压低，使错误更新更像长期偏置而非可快速扩散的随机游走；`behavior=degraded risk`。
- decision:
  - 当前更值得优先怀疑的共享机制不是 state index 错位，而是标准 ESKF 的 reset/injection 协方差语义缺口，以及 `bg/sg` 过程噪声的 Markov 解释与共享时间常数。
  - 在补做 runtime A/B 之前，不应把 `bg` / `sg_z` 的异常首先归咎于单个量测 Jacobian 或单个外参状态定义。
- next_step:
  - 先做 `data2 official outage` 下的 standard-ESKF reset Gamma A/B，直接比较 `bg/sg/sa` 与 `mounting/lever` 的 state sanity 变化。
  - 再把 `sigma_bg/sigma_sg` 与 `markov_corr_time` 做等效驱动噪声审计，必要时拆分 `bg` 与 `sg/sa` 的相关时间常数或显式切回随机游走语义。

### session_id: 20260317-0205-deep-research-overall-update-audit-r1

- timestamp: 2026-03-17 02:05 (local)
- objective: 使用 deep-research 多子审计工作流，检查 `data2 official outage` 下大量非 PVA 状态异常是否指向共享的 `INS/GNSS/ODO/NHC` 更新问题，而不是仅由单状态不可观造成。
- scope:
  - 复用 `.research/20260316-overall-update-audit-a1/` 中已完成的四个子审计：`state_semantics_reset`、`ins_propagation`、`gnss_update`、`odo_nhc_update`。
  - 再次对当前主进程代码做关键复核，重点确认 `state_sanity` 脚本读数链路、official baseline 开关、`ODO/NHC` 调用顺序、`GNSS` 直接约束范围、以及标准 ESKF 的 correction/reset 语义。
- changed_files:
  - `.research/20260316-overall-update-audit-a1/polished_report.md`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_eskf_state_sanity/artifacts/cases/release_bg_z/config_release_bg_z.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content -Raw C:\\Users\\不存在的骑士\\.codex\\skills\\{using-superpowers,deep-research}\\SKILL.md`
  - `python .research/20260316-overall-update-audit-a1/run_children.py`
  - `python .research/20260316-overall-update-audit-a1/aggregate.py`
  - `Get-Content -Raw .research/20260316-overall-update-audit-a1/final_report.md`
  - `Get-Content`（带行号）`src/app/{pipeline_fusion,dataset_loader,diagnostics,evaluation,initialization}.cpp`
  - `Get-Content`（带行号）`src/core/{eskf_engine,measurement_models_uwb}.cpp`
  - `Get-Content`（带行号）`include/{app/fusion,core/eskf}.h`
  - `Select-String` 审计 `scripts/analysis/run_data2_state_sanity_matrix.py` 与生成的 `release_bg_z` 实验配置
- artifacts:
  - `.research/20260316-overall-update-audit-a1/child_outputs/state_semantics_reset.md`
  - `.research/20260316-overall-update-audit-a1/child_outputs/ins_propagation.md`
  - `.research/20260316-overall-update-audit-a1/child_outputs/gnss_update.md`
  - `.research/20260316-overall-update-audit-a1/child_outputs/odo_nhc_update.md`
  - `.research/20260316-overall-update-audit-a1/final_report.md`
  - `.research/20260316-overall-update-audit-a1/polished_report.md`
- metrics:
  - reference state matrix labels: `abnormal/borderline/normal=18/1/3`
  - current active shared suspects: `2` 个主族，分别是 `ODO/NHC high-rate shared-state coupling` 与 `standard ESKF reset/covariance semantics`
  - latent-only risks kept separate from current baseline: `2` 个，分别是 `13-column GNSS velocity contract` 与 `weak-excitation freeze semantics when enabled`
  - explicit non-finding: 当前 `state_sanity` 行为判据读取 `state_series.csv`，不是 `SOL`
- artifact_mtime:
  - `.research/20260316-overall-update-audit-a1/polished_report.md`: fresh on `2026-03-17`
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: used as current official baseline reference; no edit in this session
- dataset_time_window:
  - official outage baseline context: `data2 + corrected RTK + 300s on / 100s off / 150s on`
- result_freshness_check:
  - pass（结论直接绑定当前工作树源码、当前 official baseline 配置与 `EXP-20260316-data2-state-sanity-r1` 结果目录）
- observability_notes:
  - `state_sanity` 当前对行为判据读取的是 `state_series.csv`，只在 `mounting_roll` 上额外读取 `DIAG` 的 `std_mr`；因此这轮矩阵不能简单归因于 `SOL` 顺序/单位误读。
  - 当前 official baseline `fusion.enable_gnss_velocity=false` 且 GNSS 源为 `dataset/data2/rtk.txt`（7 列位置）；GNSS 直接约束主要落在 `pos/att/gnss_lever`，并不是当前大量 `mounting/bg/sg/lever` 异常的首要共享来源。
  - 当前 official baseline 中 `freeze_extrinsics_when_weak_excitation=false`，因此“弱激励冻结只清 H 列而不是真冻结”是 repo 级 latent risk，但不是这轮 official matrix 的 active 解释。
  - `ODO/NHC` 在当前 baseline 下 raw-rate 生效、顺序上先于 GNSS，且 Jacobian 直接覆盖 `bg/sg/odo_scale/mounting_pitch/mounting_yaw/lever`；这是当前最值得优先验证的 shared active mechanism。
  - `mounting_roll` 虽保留在状态向量并支持注入，但当前 `ODO/NHC` 动态 `C_b_v` 与 Jacobian 未建模该量；其 `normal` 更像“半断开状态”而不是可靠的健康证据。
  - 当前 standard ESKF 分支在姿态注入后没有显式 reset Gamma，而 baseline 又使用 `fej.enable=false`；这使 `bg/sg/sa` 及 `mounting/lever` 的姿态相关互协方差成为第二个强共享嫌疑点。
- decision:
  - “多数状态异常、少数状态正常”的当前图样，不应再被优先解释为纯粹的单状态不可观或导出误读；它更像 shared update-chain / covariance semantics 问题与部分弱可观状态伪正常共同叠加的结果。
  - 当前官方 baseline 下最值得优先排查的是 `ODO/NHC` 高频共享约束链与标准 ESKF 的 reset/covariance 语义；GNSS 的 13 列速度契约问题是真 bug，但不是当前 7 列 corrected-RTK official matrix 的主因。
- next_step:
  - 先对 `data2 official outage` 做 `standard ESKF reset Gamma` A/B。
  - 再做 `NHC off / ODO off / NHC rate / ODO rate` attribution A/B，区分 road-constraint 与 reset 语义的主次关系。
  - 随后明确 `mounting_roll` 的正式策略：当前 ESKF road-constraint 链路里要么全程硬冻结，要么补齐 `C_b_v` 与 ODO/NHC Jacobian 建模。
  - 在跨数据集复用前，修正 `13-column GNSS velocity` 的坐标契约，并把“位置对齐是否使用 GNSS 速度”与 `enable_gnss_velocity` 拆开。

### session_id: 20260317-1350-data2-overall-update-attribution-r1

- timestamp: 2026-03-17 13:50 (local)
- objective: 在 `data2 official outage` 上实际运行 `standard_reset_gamma / ODO off / NHC off / ODO 20Hz / NHC 20Hz / reset+NHC20Hz` 归因实验，判断 shared abnormality 的主导来源，并验证“mounting_roll 正常是否只是伪正常”。
- scope:
  - 补一个 debug-only 的 `standard ESKF reset Gamma` 开关，仅用于归因实验，不改变默认 baseline 路径。
  - 新增实验驱动脚本，统一调用 `run_data2_state_sanity_matrix.py`，对控制组与 `release_bg_y/bg_z/sg_z/mounting_pitch/mounting_roll` 五个代表性状态做机制变体对比。
  - 运行 `7` 个变体：`baseline_subset`、`standard_reset_gamma`、`nhc_off`、`odo_off`、`nhc_20hz`、`odo_20hz`、`standard_reset_gamma_nhc_20hz`。
- changed_files:
  - `include/app/fusion.h`
  - `include/core/eskf.h`
  - `src/app/config.cpp`
  - `src/app/pipeline_fusion.cpp`
  - `src/core/eskf_engine.cpp`
  - `apps/regression_checks_main.cpp`
  - `scripts/analysis/run_data2_overall_update_attribution.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_eskf_update_attribution/variants/*/config_*.yaml`
- commands:
  - `cmake --build build --config Release --target eskf_fusion regression_checks`
  - `build/Release/regression_checks.exe`
  - `python scripts/analysis/run_data2_overall_update_attribution.py`
  - `Import-Csv output/data2_eskf_update_attribution/{variant_overview.csv,tracked_state_summary.csv}`
- artifacts:
  - `output/data2_eskf_update_attribution/summary.md`
  - `output/data2_eskf_update_attribution/manifest.json`
  - `output/data2_eskf_update_attribution/variant_overview.csv`
  - `output/data2_eskf_update_attribution/tracked_state_summary.csv`
  - `output/data2_eskf_update_attribution/variants/*`
- metrics:
  - baseline control `mean_outage_rmse_3d / max_outage_final_err_3d = 12.063017 / 45.713949`
  - `standard_reset_gamma` control `10.729688 / 38.526999`，相对 baseline `-11.05% / -15.72%`
  - `nhc_off` control `48.057915 / 234.148274`，相对 baseline `+298.39% / +412.20%`
  - `odo_off` control `51.268643 / 178.081870`，相对 baseline `+325.01% / +289.56%`
  - `nhc_20hz` control `13.277913 / 38.823360`，相对 baseline `+10.07% / -15.07%`
  - `odo_20hz` control `11.875594 / 36.528710`，相对 baseline `-1.55% / -20.09%`
  - `release_bg_y` absolute outage mean/max:
    baseline `3874.825 / 37853.598`
    `nhc_off` `1662.078 / 22387.017`
    `odo_off` `3023.700 / 33411.797`
    `nhc_20hz` `548703.775 / 3865296.078`
  - `release_bg_z` absolute outage mean/max:
    baseline `68.073 / 258.180`
    `odo_off` `99.155 / 363.227`
    `odo_20hz` `224.472 / 1180.727`
  - `release_mounting_roll` 在全部 `7` 个变体中始终 `overall=normal` 且 `delta_mean/max=0`
- artifact_mtime:
  - `output/data2_eskf_update_attribution/summary.md`: fresh on `2026-03-17`
  - `output/data2_eskf_update_attribution/variant_overview.csv`: fresh on `2026-03-17`
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: unchanged in this session; all variant configs derived from it
- dataset_time_window:
  - official outage baseline context: `data2 + corrected RTK + 300s on / 100s off / 150s on`
- result_freshness_check:
  - pass（本轮结论全部绑定 fresh build、fresh regression 与 `output/data2_eskf_update_attribution/*` 新产物）
- observability_notes:
  - `standard_reset_gamma` 明显改善控制组导航，但没有把代表性 released abnormal states 变成正常，说明标准 ESKF 的 reset/covariance 语义确实是 shared factor，但不是单独足以解释所有状态异常的唯一原因。
  - `nhc_off/odo_off` 会把 `release_bg_y`、`release_bg_z`、`release_sg_z` 的绝对 blow-up 压低，说明 road-constraint 链就是错误 `bg/sg` 扩散到导航的主要通道；但控制组导航同时严重恶化，说明这条链又不能简单关掉。
  - `nhc_20hz/odo_20hz` 没有形成稳健修复：尤其 `nhc_20hz` 与 `reset+nhc20hz` 会把 `release_bg_y` 推到灾难级，说明“简单降频”不是当前 official outage 的主修复方向。
  - `release_mounting_roll` 在所有变体下都保持零变化，进一步验证它是半断开/compatibility-only 风格状态，而不是可拿来证明 solver 健康的正常可观状态。
- decision:
  - 当前最稳的实验结论是：shared abnormality 的主因不是“某个状态自己不可观”或“把 NHC/ODO 简单降频就能解决”，而是 `road-constraint coupling` 与 `standard ESKF reset/covariance semantics` 共同造成的。
  - 其中 `ODO/NHC` 更像“错误状态影响导航的主传播通道”，而 `standard reset gap` 更像“使控制组整体导航也偏差偏大的 shared baseline defect”；两者都重要，但角色不同。
- next_step:
  - 先把 `standard_reset_gamma` 做更严格的理论/实现核对，再决定是否扩大到 full matrix A/B，而不是直接把当前 debug 版本当最终修复。
  - 针对 `ODO/NHC -> bg/sg` 的 correction-level 耦合补更细诊断，优先记录 `K/dx` 在 `bg/sg/mounting/lever` 上的时序与量测来源。
  - 把 `mounting_roll` 在当前 ESKF road-constraint 链路中正式降级为 hard-disable / compatibility-only，除非后续补齐建模。

### session_id: 20260317-1353-data2-experiment-conclusion-r1

- timestamp: 2026-03-17 13:53 (local)
- objective: 基于 fresh `state_sanity` 与 `overall_update_attribution` 产物，汇总 `data2 official outage` 下非 PVA 状态实验结论，并把“多数异常是否意味着整体更新链路存在问题”的判断收敛成当前工作结论。
- scope:
  - 复核 `walkthrough.md` 当前开放假设与最新实验行是否一致。
  - 复核 `output/data2_eskf_state_sanity/*`、`output/data2_eskf_update_attribution/*` 与 `.research/20260316-overall-update-audit-a1/polished_report.md`。
  - 补做代码级定位，确认 `standard_reset_gamma`、`mounting_roll`、`ODO/NHC` 动态 `C_b_v` 与 Jacobian 覆盖范围的实现位置。
- changed_files:
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `Get-Content output/data2_eskf_state_sanity/summary.md`
  - `Get-Content output/data2_eskf_update_attribution/{summary.md,variant_overview.csv}`
  - `Get-Content .research/20260316-overall-update-audit-a1/polished_report.md`
  - `Select-String -Path src/core/eskf_engine.cpp,include/core/eskf.h,include/app/fusion.h,src/app/config.cpp,src/app/pipeline_fusion.cpp,apps/regression_checks_main.cpp -Pattern 'ApplyStandardEskfReset|debug_enable_standard_reset_gamma|UpdateCovarianceJoseph|InjectErrorState'`
  - `Select-String -Path src/core/measurement_models_uwb.cpp,src/app/pipeline_fusion.cpp,src/core/eskf_engine.cpp,include/core/eskf.h -Pattern 'mounting_roll|mounting_pitch|mounting_yaw|C_b_v|NHC|ODO|std_mr'`
- artifacts:
  - `output/data2_eskf_state_sanity/summary.md`
  - `output/data2_eskf_update_attribution/{summary.md,variant_overview.csv,tracked_state_summary.csv,manifest.json}`
  - `.research/20260316-overall-update-audit-a1/polished_report.md`
- metrics:
  - state-sanity matrix: `18 abnormal / 1 borderline / 3 normal`
  - control baseline outage mean/max: `12.063017 / 45.713949`
  - `standard_reset_gamma` control mean/max: `10.729688 / 38.526999`
  - `nhc_off` control mean/max: `48.057915 / 234.148274`
  - `odo_off` control mean/max: `51.268643 / 178.081870`
  - `release_bg_y` 在 `nhc_20hz` 下恶化到 `548703.775 / 3865296.078`
  - `release_mounting_roll` 在全部 `7` 个变体中始终 `overall=normal` 且 `delta_mean/max=0`
- artifact_mtime:
  - `output/data2_eskf_state_sanity/summary.md`: fresh from `2026-03-16`
  - `output/data2_eskf_update_attribution/summary.md`: fresh from `2026-03-17`
  - `.research/20260316-overall-update-audit-a1/polished_report.md`: fresh from `2026-03-17`
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: unchanged in this session; conclusions bound to current official baseline config
- dataset_time_window:
  - official outage baseline context: `data2 + corrected RTK + 300s on / 100s off / 150s on`
- result_freshness_check:
  - pass（本会话未新增 solver 运行，但全部结论均回绑到 `2026-03-16 ~ 2026-03-17` 的 fresh 产物与当前源码定位）
- observability_notes:
  - 当前最合理的解释已从“多数状态单独不可观”收敛为“shared update-chain defect + 少数伪正常状态”的混合图样。
  - `ODO/NHC` 依旧是当前 active shared culprit 的首位嫌疑：在主流程中先于 GNSS 执行，并直接覆盖 `bg/sg/odo_scale/mounting_pitch/mounting_yaw/lever`。
  - `mounting_roll` 的 `normal` 已不再被视为健康证据：状态本身可注入，但当前动态 `C_b_v` 与 ODO/NHC Jacobian 只建模 `pitch/yaw`，未真正使用 `roll`。
  - 标准 ESKF 分支存在真实的 reset/covariance 语义影响：debug `standard_reset_gamma` 能显著改善控制组导航，但不能单独消除 released abnormal states。
- decision:
  - 当前 official outage 口径下，可以正式给出的实验结论是：多数非 PVA 状态的异常不能主要归因于“它们刚好都不可观”；更合理的主解释是 `ODO/NHC road-constraint coupling` 与 `standard ESKF reset/covariance semantics` 的组合问题。
  - 少数看起来正常的状态里，`mounting_roll` 已被实验和代码同时证明为 pseudo-normal / compatibility-only，不应继续当作在线估计健康性的证据。
- next_step:
  - 保持现有优先级不变，下一轮直接进入 `standard_reset_gamma` 理论核对与 `ODO/NHC -> bg/sg/mounting/lever` correction-level 日志诊断。

### session_id: 20260317-1515-data2-ins-gnss-state-sanity-r1

- timestamp: 2026-03-17 15:15 (local)
- objective: 按“先去掉 ODO/NHC 及其强相关状态块，再检查零偏/比例因子/GNSS 杆臂”的新思路，在 `data2` 上执行纯 `INS/GNSS` 的真值锚定控制组 + 单状态释放实验。
- scope:
  - 新增 `scripts/analysis/run_data2_ins_gnss_state_sanity.py`，复用现有 truth-anchor / state-series 评估框架，但切换为 `full GNSS` 与全程导航指标。
  - 配置上强制 `enable_odo=false`、`enable_nhc=false`、`disable_mounting=true`、`disable_odo_lever_arm=true`、`disable_odo_scale=true`。
  - 仅运行 `ba/bg/sg/sa/gnss_lever` 共 `15` 个释放状态，加控制组共 `16` 个 case。
- changed_files:
  - `scripts/analysis/run_data2_ins_gnss_state_sanity.py`
  - `walkthrough.md`
- configs:
  - `config_data2_baseline_eskf.yaml`
  - `output/data2_eskf_ins_gnss_state_sanity/artifacts/cases/*/config_*.yaml`
- commands:
  - `Get-Content walkthrough.md`
  - `python scripts/analysis/run_data2_ins_gnss_state_sanity.py --cases truth_anchor_all_non_pva release_bg_z release_gnss_lever_y`
  - `python scripts/analysis/run_data2_ins_gnss_state_sanity.py`
  - `Import-Csv output/data2_eskf_ins_gnss_state_sanity/{case_metrics.csv,state_judgement.csv}`
- artifacts:
  - `output/data2_eskf_ins_gnss_state_sanity/summary.md`
  - `output/data2_eskf_ins_gnss_state_sanity/manifest.json`
  - `output/data2_eskf_ins_gnss_state_sanity/case_metrics.csv`
  - `output/data2_eskf_ins_gnss_state_sanity/state_judgement.csv`
  - `output/data2_eskf_ins_gnss_state_sanity/plots/*`
  - `output/data2_eskf_ins_gnss_state_sanity/artifacts/cases/*`
- metrics:
  - control full-GNSS `nav_rmse_3d / nav_final_err_3d = 0.049119 / 0.018997`
  - `gnss_lever_x`: `overall=normal`, final `0.119331 m` vs ref `0.149981 m`
  - `gnss_lever_y`: behavior `normal`, final `-0.189893 m` vs ref `-0.219900 m`, absolute nav `0.061576 / 0.042114`
  - `gnss_lever_z`: behavior `normal`, final `-0.924430 m` vs ref `-1.149811 m`, absolute nav `0.268493 / 0.217890`
  - `ba/bg/sg/sa`: 除 `sa_z=borderline` 外，其余释放 case 行为标签均为 `abnormal`
  - `bg_y`: final `736.793 deg/h`，但 full-GNSS nav 仍仅 `0.041679 / 0.040237`
- artifact_mtime:
  - `output/data2_eskf_ins_gnss_state_sanity/summary.md`: fresh on `2026-03-17`
  - `output/data2_eskf_ins_gnss_state_sanity/state_judgement.csv`: fresh on `2026-03-17`
- config_hash_or_mtime:
  - `config_data2_baseline_eskf.yaml`: unchanged in this session; all INS/GNSS-only configs derived from it
- dataset_time_window:
  - full GNSS baseline context: `data2 + corrected RTK + no outage schedule`
- result_freshness_check:
  - pass（脚本、smoke 和 full matrix 都在本会话 fresh 运行，结论绑定 `output/data2_eskf_ins_gnss_state_sanity/*`）
- observability_notes:
  - 去掉 `ODO/NHC` 之后，`ba/bg/sg/sa` 的“从零回到参考值/合理包络”问题依旧存在，说明前一轮大面积 behavior abnormal 不能只归咎于 road-constraint。
  - 但在 full-GNSS 下，多数这些状态释放后的导航仍维持厘米级，说明 road-constraint 更像“把坏状态放大成导航劣化的传播通道”，而不是这些状态 behavior abnormal 的唯一来源。
  - `gnss_lever_x` 在当前纯 INS/GNSS 下可正常恢复；`gnss_lever_y` 最终值也接近参考，但 relative impact 判据会因 control 已到厘米级而显得偏严，解读时应同时看 absolute nav。
  - `gnss_lever_z` 是当前纯 INS/GNSS 下最明确的弱项：最终值虽朝参考值收敛，但仍使 absolute nav 明显恶化到约 `27 cm / 22 cm`。
- decision:
  - 新实验支持把问题拆成两层：`ODO/NHC` 主要负责把错误状态强耦合并放大到导航；但 `INS/GNSS` 链自身对 `ba/bg/sg/sa` 的 recoverability 也不够好，不能简单认为“关掉 road-constraint 后就会正常收敛”。
  - 因此下一阶段不应只盯着 `ODO/NHC`，还要直接审计 `INS propagation + GNSS_POS update` 对 `bg/sg/gnss_lever` 的耦合与过程噪声语义。
- next_step:
  - 在纯 INS/GNSS 口径下增加 `GNSS_POS` correction-level `K/dx` 日志，重点看 `bg/sg/gnss_lever`。
  - 对 `ins_mech` 里的 `ba/bg/sg/sa` Markov 与 scale-factor 过程模型做定向语义审计。
  - 后续若继续做 full-GNSS state sanity，导航 impact 判据应补一个 absolute floor，避免厘米级 control 下的纯相对比值过度放大。

### 已归档会话摘要

| 主题 | 涉及 session_id | 核心动作 | 最终决定/产物 | 当前关联 | archive_ref |
|---|---|---|---|---|---|
| InEKF 初版重写与 best switch 锁定 | `20260304-1435-agent-doc-bootstrap`、`20260304-1505-agent-md-fix-bootstrap-and-context`、`20260304-1822-pre_experiment_audit`、`20260304-1855-fix-five-issues-and-md-check`、`20260305-0032-best4-regression-and-plot`、`20260305-0042-tidy-walkthrough-and-promote-inekf-baseline`、`20260305-1010-inekf-fghi-audit-and-doubleoff`、`20260305-1018-issue001-baseline-compare-refresh`、`20260305-1033-baseline-gmode-sensitivity-r1`、`20260305-1048-stage-summary-writeback` | 完成 InEKF 初版实现、best switch 锁定、baseline 冲突重生与单因子机制排错。 | 确定后续实验统一从 best-switch `InEKF` 出发，历史 `inekf_ctrl` 仅保留为复现实验。 | 为当前 `InEKF` 与 `true_iekf`/phase-2 证据链提供起点。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_01_inekf_initial_rewrite_best_switch` |
| true_iekf 分支建立与 phase-2 主修复 | `20260306-1003-inekf-tex-code-audit-r1`、`20260306-1012-inekf-architecture-choice`、`20260306-1128-true-iekf-refactor-and-compare-r1`、`20260306-1349-true-iekf-gnss30-ablation-r1`、`20260306-1654-true-iekf-phase2-and-tex-sync`、`20260306-2317-turning-vs-inekf-eskf-analysis`、`20260306-2336-program-audit-and-gnsspos-rframe-fix`、`20260307-0001-jacobian-audit-r1`、`20260307-0040-jacobian-audit-gnssvel-r2`、`20260307-1101-reset-covariance-audit-r1`、`20260310-1935-true-iekf-reset-audit` | 围绕 tex-code gap、phase-2、reset consistency 与 GNSS_POS/GNSS_VEL Jacobian 做逐层排错。 | 把当前 `InEKF` 机理问题收敛到 process/reset 与 GNSS 期间耦合建立，而非单纯 measurement Jacobian 错误。 | 直接支撑当前关于 `v系航向误差`、`att_z-bg_z` 的机理分析入口。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_02_true_iekf_branch_phase2` |
| data2 post-GNSS 漂移与 bg/mounting 因果链 | `20260305-1029-postgnss-freeze-matrix-r1`、`20260305-1039-postgnss-headratio-robustness-r1`、`20260305-1058-gmode-gnssvel-and-state21-30-drift`、`20260306-1759-phase2b-data2-gnss30-diagnostics`、`20260306-1835-phase2b-heading-source-attribution`、`20260306-1848-phase2c-bg-freeze-validation`、`20260306-1902-gnss30-stateplots-export`、`20260307-1023-add-vframe-velocity-plots`、`20260310-1545-mounting-median-report`、`20260310-1558-html-mounting-note`、`20260310-1643-inekf-mechanism-r1` | 围绕 post-GNSS 漂移做冻结、状态图、航向误差与 mounting 中位值分析。 | 把 data2 剩余问题明确定位到 `bg_z` 主导、`mounting` 放大、`GNSS_VEL` 非主因的链路。 | 与当前 open hypotheses `HYP-11/HYP-12` 直接相连。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_03_data2_postgnss_bg_mounting_chain` |
| data2 GNSS30 的 NHC 频率扫描链 | `20260311-1051-odo-nhc-update-interval-audit`、`20260311-1413-odo-nhc-rate-sweep`、`20260311-1442-data2-gnss30-eskf-nhc-sweep`、`20260311-1511-data2-gnss30-true-iekf-nhc-sweep`、`20260311-1532-html-report-exp9-exp10`、`20260312-1010-inekf-mechanism-attribution-r1`、`20260312-1128-inekf-mechanism-r2-partial` | 把 interval audit、smoke 与 r1 机理实验串成正式 GNSS30 扫频链。 | 为当前保留的 `r2/r2b/r3/r3b` 与 `rate-vs-weight r2` 正式证据链铺平口径。 | 当前保留的 `EXP-20260311-odo-nhc-rate-sweep-r2/r2b/r3/r3b` 与 `EXP-20260312-inekf-mechanism-attribution-r2`、`EXP-20260312-inekf-mechanism-rate-vs-weight-r2` 是这条链的正式收敛结果。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_04_data2_gnss30_nhc_sweep_chain` |
| 报告交付演进链（interactive html / dataset-partitioned html） | `20260305-1611-automation-worktree-overview`、`20260306-1600-github-sync-current-work`、`20260307-1005-compress-walkthrough-and-github-sync`、`20260308-0039-result-docs-r1`、`20260308-0054-inekf-doc-typeset-r1`、`20260308-0059-inekf-doc-typeset-r2`、`20260308-0102-inekf-doc-typeset-r3`、`20260309-1049-inekf-doc-symbols-r1`、`20260309-1107-inekf-doc-compile-check`、`20260309-2328-interactive-report-notebook-r1`、`20260310-0014-interactive-report-html-fix`、`20260310-0039-interactive-report-html-restyle`、`20260310-0105-interactive-report-html-pdf-style`、`20260310-1654-interactive-report-drop-group3`、`20260310-1710-interactive-report-add-analysis`、`20260310-1826-interactive-report-renumber`、`20260310-1956-remove-imu-pages`、`20260310-2010-report-rename-cleanup`、`20260310-2015-imu-precision-report`、`20260310-2326-github-sync`、`20260311-0019-report-attitude-error-fix`、`20260311-0044-html-attitude-cn-audit`、`20260311-0104-html-accuracy-audit` | 迭代 HTML、PDF/tex、reader-facing copy 与 walkthrough 压缩形态。 | 形成当前两类正式展示口径：`representative` 历史链与 `dataset-partitioned` 读者链。 | 当前下一步仍需把 reader-facing 口径同步回 `representative` 与 `.tex`。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_05_report_delivery_evolution` |
| 被 supersede 的 data4 r1 与 outage 误接链 | `20260305-2222-inekf-eskf-data2-data4-pipeline`、`20260306-1714-data4-vs-data2-gnss30-gap-analysis`、`20260310-gnss-lever-vel-experiments-html-sidebar`、`20260310-1854-data2-gnss-outage-cycle`、`20260312-1410-data2-gnss-outage-cycle-inekf-best-and-html`、`20260312-1414-outage-cycle-inekf-best-handoff-check`、`20260312-1429-outage-cycle-inekf-best-reference-correction`、`20260312-1443-outage-cycle-true-tuned-replacement`、`20260312-2150-representative-data4-conflict-check` | 围绕 data4 r1、outage 误接与 representative 报告冲突做回溯、替换和 supersede 标记。 | 正式结果统一切换到 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 与 `EXP-20260312-dataset-report-cases-r2`。 | 当前 cleanup 仍需把 superseded 目录移出工作位，只保留索引和历史摘要。 | `docs/project_history/walkthrough_archive_sessions_20260313.md :: theme_06_superseded_data4_r1_and_outage_chain` |

## Next Actions

1. 在纯 `INS/GNSS` 口径下增加 `GNSS_POS` correction-level `K/dx` 日志，重点记录 `bg/sg/gnss_lever` 的更新时序，判断 recoverability gap 更像来自 GNSS 更新耦合不足还是过程模型过硬。
2. 对 `standard_reset_gamma` 做更严格的理论/实现核对，并在确认形式正确后决定是否扩大到 full `23` case state-sanity matrix；当前 debug 版本只足够证明“reset/covariance 语义确实影响控制组导航”。
3. 针对 `ODO/NHC -> bg/sg/mounting/lever` 的共享耦合补 correction-level 诊断，优先记录 `K/dx` 在这些块上的时序与量测来源，判断到底是 Jacobian、cross-covariance 还是 gating/weighting 在放大错误更新。
4. 在当前 ESKF road-constraint 链路中明确 `mounting_roll` 的正式策略：默认 hard-disable / compatibility-only；除非后续补齐其在 `C_b_v` 与 ODO/NHC Jacobian 中的建模，否则不再把它当作“正常可观状态”。
5. 对 `bg_x/bg_y/bg_z` 与 `sg_z` 做过程噪声语义审计：量化 `sigma_* + markov_corr_time` 的等效驱动噪声，检查当前 `3600s` 共享时间常数是否把 `ba/bg/sg/sa` 统一建模得过于僵硬。
6. 修正 `13-column GNSS velocity` 的坐标契约，并把“GNSS 速度是否参与位置时间对齐”与 `enable_gnss_velocity` 拆开，避免后续跨数据集归因继续被混淆。
7. 保持后续 observability 审计统一读取 `state_series.csv`，并为 `SOL/state_series/total_mounting_*` 建立显式列映射与单位说明，避免在辅助分析脚本里重新引入解释偏差。
8. 若继续沿用 full-GNSS state-sanity 作为辅证，为导航 impact 判据补 absolute floor，避免厘米级 control 下纯相对比值把 `gnss_lever_y` 这类 case 过度标成 `abnormal`。
9. 在完成前五项后，再回到 `sg/sa/odo_lever/gnss_lever/mounting_pitch/yaw` 这些“behavior abnormal 但多数 impact normal”的状态做更细的符号、单位与 Jacobian 一致性检查，区分弱可观与实现细节错误。
