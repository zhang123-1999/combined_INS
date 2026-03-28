# repo_layout.md

Updated: `2026-03-26`

| 区域 | 作用 | 典型路径 | 管理规则 |
|---|---|---|---|
| 当前工作区 | 当前要直接编辑、运行和复现的代码与文档。 | `src/`, `include/`, `apps/`, `scripts/`, `config_*.yaml`, `walkthrough.md` | 保持轻量；主线只保留 canonical baseline 与最小 supporting evidence。 |
| 官方 baseline 结果 | 当前唯一官方 `data2` baseline evidence root。 | `output/data2_baseline_current/` | 后续正式引用、对照和 fresh 重跑统一从这里读取。 |
| supporting evidence | 仍保留在 `output/` 下的最小解释链目录。 | `output/data2_staged_g5_no_imu_scale_r2_20260325/`, `output/data2_staged_g5_odo_scale_phase2_seed_sweep_r1_20260326/`, `output/data2_staged_g5_odo_lever_phase2_seed_sweep_r2_20260326/`, `output/data2_staged_g5_odo_lever_process_q_sweep_r1_20260326/` | 只保留支撑 baseline 形成的必要目录；新增 sweep 若 supersede 旧目录，应立即归档旧目录。 |
| 历史文本归档 | 承接从主 `walkthrough.md` 移出的完整原文、压缩前快照与迁移索引。 | `docs/project_history/` | 必须受版本管理；任何从主档移出的研究结论都要在这里保留全文。 |
| 历史产物归档 | 承接已 supersede 的 `data2_*` 大产物与旧官方 baseline 目录。 | `archive/output_legacy/20260326-baseline-finalization/`, `archive/root_sol_legacy/20260313/`, `archive/output_review/tmp/20260313/` | `2026-03-26` 起，非最小证据链的 `output/data2_*` 目录统一迁入该归档根；保留原目录名以维持 traceability。 |
| 本地临时区 | 允许短期调试，但不应长期堆积。 | `output/review/_tmp*`（现已迁走）、手工 smoke 目录 | 形成结论前可短留；一旦 superseded 或任务结束，应迁入 `archive/` 或删除。 |
| 研究 seed 配置 | 保留旧 baseline 语义，供 supporting / historical sweep 脚本复用。 | `config_data2_research_seed_eskf.yaml` | 只能作为 sweep seed；不得再作为官方 baseline 对外引用。 |
