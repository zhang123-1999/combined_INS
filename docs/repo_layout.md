# repo_layout.md

Updated: `2026-03-13`

| 区域 | 作用 | 典型路径 | 管理规则 |
|---|---|---|---|
| 当前工作区 | 当前要直接编辑、运行和复现的代码与文档。 | `src/`, `include/`, `apps/`, `scripts/`, `config_*.yaml`, `walkthrough.md` | 保持轻量；只保留当前工作链路与近期关键摘要。 |
| 历史文本归档 | 承接从主 `walkthrough.md` 移出的完整原文、压缩前快照与迁移索引。 | `docs/project_history/` | 必须受版本管理；任何从主档移出的研究结论都要在这里保留全文。 |
| 历史产物归档 | 承接根目录遗留 `SOL_*.txt`、临时目录和已 supersede 的大产物。 | `archive/root_sol_legacy/20260313/`, `archive/output_review/tmp/20260313/`, `archive/output_review/superseded/20260313/` | 只移动临时/重复/superseded 产物；当前正式证据链不迁移。 |
| 本地临时区 | 允许短期调试，但不应长期堆积。 | `output/review/_tmp*`（现已迁走）、手工 smoke 目录 | 形成结论前可短留；一旦 superseded 或任务结束，应迁入 `archive/` 或删除。 |
| 当前正式结果源 | 当前结论与 HTML 正式引用的 canonical artifacts。 | `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/`, `output/review/EXP-20260312-dataset-report-cases-r2/`, `output/html/dataset_partitioned_navigation_interactive_report.html` | 不迁移、不重命名；如被 supersede，需先更新 `walkthrough.md` 的正式口径。 |
| historical-key 结果目录 | 仍有解释价值但不是当前正式结果源的历史关键目录。 | `output/review/20260306-true-iekf-phase2-r1/`, `output/review/20260306-phase2c-bg-freeze/`, `output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/` 等 | 可以留在原位，但需在 `walkthrough.md`/报告中明确其角色是 historical-key 而非 canonical final。 |
