from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re
import shutil


ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
PROJECT_HISTORY = DOCS / "project_history"
ARCHIVE = ROOT / "archive"
OUT_REVIEW = ROOT / "output" / "review"

TODAY = "2026-03-13"
SESSION_ID = "20260313-0141-walkthrough-importance-archive-r1"
SESSION_TIMESTAMP = "2026-03-13 01:41 (local)"

SNAPSHOT_NAME = "walkthrough_preservation_snapshot_20260313.md"
REGISTRY_NAME = "walkthrough_archive_registry_20260313.md"
SESSIONS_NAME = "walkthrough_archive_sessions_20260313.md"
RELOCATION_NAME = "artifact_relocation_index.md"
REPO_LAYOUT_NAME = "repo_layout.md"

KEEP_EXP_IDS = [
    "EXP-20260311-codefix-r1",
    "EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf",
    "EXP-20260311-odo-nhc-rate-sweep-r2b-data2-gnss30-eskf-refine",
    "EXP-20260311-odo-nhc-rate-sweep-r3-data2-gnss30-true-iekf",
    "EXP-20260311-odo-nhc-rate-sweep-r3b-data2-gnss30-true-iekf-refine",
    "EXP-20260312-inekf-mechanism-attribution-r2",
    "EXP-20260312-inekf-mechanism-rate-vs-weight-r2",
    "EXP-20260312-data2-gnss-outage-cycle-true-tuned-r1",
    "EXP-20260312-data4-p0diag-check-r1",
    "EXP-20260312-data4-gnss30-nhc-sweep-r2",
    "EXP-20260312-dataset-report-cases-r2",
    "EXP-20260313-dataset-partitioned-html-reader-r4",
]

KEEP_SESSION_IDS = [
    "20260311-1022-project-code-review",
    "20260311-1045-code-review-fixes-r1",
    "20260312-1353-inekf-mechanism-r2-complete",
    "20260312-1626-dataset-partitioned-report-r1",
    "20260312-2216-data4-p0diag-root-cause-check",
    "20260312-2230-data4-p0diag-rerun-and-html-refresh",
    "20260313-0034-dataset-report-reader-copy",
    "20260313-0103-dataset-report-reader-table-compaction",
]

CURRENT_ISSUE_IDS = [
    "ISSUE-007-inekf-tex-code-gap",
    "ISSUE-20260312-representative-report-data4-stale",
]

CURRENT_OPEN_HYP_IDS = [
    "HYP-1",
    "HYP-6",
    "HYP-7",
    "HYP-8",
    "HYP-9",
    "HYP-10",
    "HYP-11",
    "HYP-12",
    "HYP-13",
]


@dataclass(frozen=True)
class ThemeMeta:
    title: str
    retained_conclusion: str
    why_important: str
    session_ids: list[str]
    session_action: str
    session_outcome: str
    session_relation: str


@dataclass
class MoveRecord:
    old_path: str
    new_path: str
    category: str
    reason: str
    summary_ref: str
    status: str


EXP_THEME_MAP = {
    "theme_01_inekf_initial_rewrite_best_switch": [
        "EXP-20260304-data2-baseline-eskf",
        "EXP-20260304-data2-baseline-inekf-ctrl",
        "EXP-20260304-inekf-ri-realization-codeoverhaul3",
        "EXP-20260304-inekf-ri-gvelgyro-fix",
        "EXP-20260305-inekf-ab-seq-r1",
        "EXP-20260305-inekf-best4-regression-r1",
        "EXP-20260305-inekf-fghi-sign-audit-r1",
        "EXP-20260305-inekf-doubleoff-interaction-r1",
        "EXP-20260305-issue001-baseline-compare-r1",
        "EXP-20260305-baseline-gmode-sensitivity-r1",
        "EXP-20260305-baseline-gmode-gnssvel-mechanism-r1",
        "EXP-20260305-baseline-gmode-gnsspos-coupling-r1",
        "EXP-20260305-data2-process-noise-regression-r1",
        "EXP-20260304-data2-gnss30-postfix-gnsslever-eskf",
        "EXP-20260304-data2-gnss30-inekf",
        "EXP-20260304-data2-gnss30-eskf-nofreeze",
        "EXP-20260304-preexp-fix-regression",
        "EXP-20260304-inekf-refactor-minrisk-debug",
        "EXP-20260304-inekf-rewrite-no-fej",
        "EXP-20260304-inekf-ri-fgqd-audit-r2",
        "EXP-20260305-inekf-deep-research-audit-r1",
    ],
    "theme_02_true_iekf_branch_phase2": [
        "EXP-20260306-inekf-tex-code-audit-r1",
        "EXP-20260306-true-iekf-refactor-r1",
        "EXP-20260306-true-iekf-ablation-r1",
        "EXP-20260306-true-iekf-phase2-r1",
        "EXP-20260306-true-iekf-phase2-ablation-lite",
        "EXP-20260306-consistency-audit-r1",
        "EXP-20260306-gnsspos-rframe-fix-r1",
        "EXP-20260306-jacobian-audit-r1",
        "EXP-20260307-jacobian-audit-r2",
        "EXP-20260307-reset-floor-r1",
        "EXP-20260307-update-reset-consistency-r1",
    ],
    "theme_03_data2_postgnss_bg_mounting_chain": [
        "EXP-20260305-postgnss-freeze-matrix-r1",
        "EXP-20260305-postgnss-headratio-robustness-r1",
        "EXP-20260305-state21-30-drift-r1",
        "EXP-20260305-data2-mountroll-observability-r1",
        "EXP-20260305-data2-postgnss-freeze-matrix-r2",
        "EXP-20260306-phase2b-attribution-r1",
        "EXP-20260306-phase2c-bg-freeze-r1",
        "EXP-20260306-gnss30-stateplots-r1",
        "EXP-20260310-mounting-median-r1",
        "EXP-20260310-inekf-mechanism-r1",
        "EXP-20260305-data2-gnsspos-mechanism-r2",
        "EXP-20260307-vframe-velocity-plot-r1",
    ],
    "theme_04_data2_gnss30_nhc_sweep_chain": [
        "EXP-20260311-project-code-review-r1",
        "EXP-20260311-odo-nhc-rate-sweep-r1",
        "EXP-20260311-odo-nhc-rate-sweep-smoke",
        "EXP-20260312-inekf-mechanism-attribution-r1",
        "EXP-20260312-inekf-mechanism-rate-vs-weight-r1",
    ],
    "theme_05_report_delivery_evolution": [
        "EXP-20260308-result-docs-r1",
        "EXP-20260308-inekf-doc-typeset-r1",
        "EXP-20260308-inekf-doc-typeset-r2",
        "EXP-20260308-inekf-doc-typeset-r3",
        "EXP-20260309-inekf-doc-symbols-r1",
        "EXP-20260309-inekf-doc-compile-check-r1",
        "EXP-20260309-interactive-report-notebook-r1",
        "EXP-20260310-interactive-report-html-r1",
        "EXP-20260310-interactive-report-html-r2",
        "EXP-20260310-interactive-report-html-r3",
        "EXP-20260310-interactive-report-html-r4",
        "EXP-20260311-interactive-report-html-r5",
        "EXP-20260311-interactive-attitude-error-fix-r1",
        "EXP-20260311-html-attitude-cn-audit-r1",
        "EXP-20260311-html-accuracy-audit-r1",
        "EXP-20260312-interactive-report-html-r6",
        "EXP-20260312-interactive-report-html-r7",
        "EXP-20260312-dataset-partitioned-html-r1",
        "EXP-20260312-dataset-partitioned-html-r2",
        "EXP-20260313-dataset-partitioned-html-reader-r3",
    ],
    "theme_06_superseded_data4_r1_and_outage_chain": [
        "EXP-20260305-data4-inekf-switchscan-r1",
        "EXP-20260305-data4-main4-regression-r1",
        "EXP-20260305-data4-gnss30-freeze-matrix-r1",
        "EXP-20260305-data4-gnssvel-sensitivity-r1",
        "EXP-20260305-data2-main4-docsync-r1",
        "EXP-20260310-gnss-lever-fix-r1",
        "EXP-20260310-gnss-vel-effect-r1",
        "EXP-20260310-data4-imu-precision-r1",
        "EXP-20260310-data2-gnss-outage-cycle-r1",
        "EXP-20260312-data2-gnss-outage-cycle-inekf-best-r1",
        "EXP-20260312-data4-gnss30-nhc-sweep-r1",
        "EXP-20260312-dataset-report-cases-r1",
    ],
}

THEME_META = {
    "theme_01_inekf_initial_rewrite_best_switch": ThemeMeta(
        title="InEKF 初版重写与 best switch 锁定",
        retained_conclusion="完成 RI 版 InEKF 初次落地，并通过 A/B 顺序实验把 best switch 锁定为 `p_local=ON, g=+1, inject=ON`；同时保留了 baseline 指标冲突和单因子排错链。",
        why_important="它解释了为什么后续 `InEKF` 研究以 best-switch 口径为起点，而不是回到早期 `inekf_ctrl` 或半成品 rewrite。",
        session_ids=[
            "20260304-1435-agent-doc-bootstrap",
            "20260304-1505-agent-md-fix-bootstrap-and-context",
            "20260304-1822-pre_experiment_audit",
            "20260304-1855-fix-five-issues-and-md-check",
            "20260305-0032-best4-regression-and-plot",
            "20260305-0042-tidy-walkthrough-and-promote-inekf-baseline",
            "20260305-1010-inekf-fghi-audit-and-doubleoff",
            "20260305-1018-issue001-baseline-compare-refresh",
            "20260305-1033-baseline-gmode-sensitivity-r1",
            "20260305-1048-stage-summary-writeback",
        ],
        session_action="完成 InEKF 初版实现、best switch 锁定、baseline 冲突重生与单因子机制排错。",
        session_outcome="确定后续实验统一从 best-switch `InEKF` 出发，历史 `inekf_ctrl` 仅保留为复现实验。",
        session_relation="为当前 `InEKF` 与 `true_iekf`/phase-2 证据链提供起点。",
    ),
    "theme_02_true_iekf_branch_phase2": ThemeMeta(
        title="true_iekf 分支建立与 phase-2 主修复",
        retained_conclusion="建立 `true_iekf` 分支，完成 `ODO/NHC` Lie 核心 Jacobian、reset Gamma、一致性与 numerical Jacobian 审计，确认 phase-2 是 data4 GNSS30 大幅改善的主修复。",
        why_important="它定义了当前讨论“理论贴合实现”时所说的 `InEKF` 技术背景，也解释了为什么剩余问题不再优先归咎于 measurement Jacobian。",
        session_ids=[
            "20260306-1003-inekf-tex-code-audit-r1",
            "20260306-1012-inekf-architecture-choice",
            "20260306-1128-true-iekf-refactor-and-compare-r1",
            "20260306-1349-true-iekf-gnss30-ablation-r1",
            "20260306-1654-true-iekf-phase2-and-tex-sync",
            "20260306-2317-turning-vs-inekf-eskf-analysis",
            "20260306-2336-program-audit-and-gnsspos-rframe-fix",
            "20260307-0001-jacobian-audit-r1",
            "20260307-0040-jacobian-audit-gnssvel-r2",
            "20260307-1101-reset-covariance-audit-r1",
            "20260310-1935-true-iekf-reset-audit",
        ],
        session_action="围绕 tex-code gap、phase-2、reset consistency 与 GNSS_POS/GNSS_VEL Jacobian 做逐层排错。",
        session_outcome="把当前 `InEKF` 机理问题收敛到 process/reset 与 GNSS 期间耦合建立，而非单纯 measurement Jacobian 错误。",
        session_relation="直接支撑当前关于 `v系航向误差`、`att_z-bg_z` 的机理分析入口。",
    ),
    "theme_03_data2_postgnss_bg_mounting_chain": ThemeMeta(
        title="data2 post-GNSS 漂移与 bg/mounting 因果链",
        retained_conclusion="通过冻结矩阵、state21-30 漂移图、phase-2b/2c 与状态图，确认 data2 的主要残余 blocker 是 post-GNSS `bg_z` 主导的 heading drift，`mounting` 会放大位置误差但不是 yaw growth 首因。",
        why_important="这是当前解释 data2 下 `InEKF` 改善为何主要体现为 v 系航向误差，而不是全部状态均匀受益的核心因果链。",
        session_ids=[
            "20260305-1029-postgnss-freeze-matrix-r1",
            "20260305-1039-postgnss-headratio-robustness-r1",
            "20260305-1058-gmode-gnssvel-and-state21-30-drift",
            "20260306-1759-phase2b-data2-gnss30-diagnostics",
            "20260306-1835-phase2b-heading-source-attribution",
            "20260306-1848-phase2c-bg-freeze-validation",
            "20260306-1902-gnss30-stateplots-export",
            "20260307-1023-add-vframe-velocity-plots",
            "20260310-1545-mounting-median-report",
            "20260310-1558-html-mounting-note",
            "20260310-1643-inekf-mechanism-r1",
        ],
        session_action="围绕 post-GNSS 漂移做冻结、状态图、航向误差与 mounting 中位值分析。",
        session_outcome="把 data2 剩余问题明确定位到 `bg_z` 主导、`mounting` 放大、`GNSS_VEL` 非主因的链路。",
        session_relation="与当前 open hypotheses `HYP-11/HYP-12` 直接相连。",
    ),
    "theme_04_data2_gnss30_nhc_sweep_chain": ThemeMeta(
        title="data2 GNSS30 的 NHC 频率扫描链",
        retained_conclusion="在正式 r2/r2b/r3/r3b 之前，完成 ODO/NHC 更新间隔审计、smoke 扫频和第一轮机理对比，证明 data2 GNSS30 的最优频率需要分算法单独搜索。",
        why_important="它解释了为什么当前 canonical report 里 ESKF `30 Hz` 与 InEKF `0.75 Hz` 的结论不是拍脑袋，而是由一条逐层细化的 sweep 链得到。",
        session_ids=[
            "20260311-1051-odo-nhc-update-interval-audit",
            "20260311-1413-odo-nhc-rate-sweep",
            "20260311-1442-data2-gnss30-eskf-nhc-sweep",
            "20260311-1511-data2-gnss30-true-iekf-nhc-sweep",
            "20260311-1532-html-report-exp9-exp10",
            "20260312-1010-inekf-mechanism-attribution-r1",
            "20260312-1128-inekf-mechanism-r2-partial",
        ],
        session_action="把 interval audit、smoke 与 r1 机理实验串成正式 GNSS30 扫频链。",
        session_outcome="为当前保留的 `r2/r2b/r3/r3b` 与 `rate-vs-weight r2` 正式证据链铺平口径。",
        session_relation="当前保留的 `EXP-20260311-odo-nhc-rate-sweep-r2/r2b/r3/r3b` 与 `EXP-20260312-inekf-mechanism-attribution-r2`、`EXP-20260312-inekf-mechanism-rate-vs-weight-r2` 是这条链的正式收敛结果。",
    ),
    "theme_05_report_delivery_evolution": ThemeMeta(
        title="报告交付演进链（interactive html / dataset-partitioned html）",
        retained_conclusion="报告系统经历了 notebook、interactive html、reader-facing dataset-partitioned html 多轮重构；旧版本保留设计与命名演进证据，但当前读者版口径以后续 `reader-r4` 为准。",
        why_important="它解释了为什么仓库里会同时存在多个报告导出器、HTML 版本与文档排版实验。",
        session_ids=[
            "20260305-1611-automation-worktree-overview",
            "20260306-1600-github-sync-current-work",
            "20260307-1005-compress-walkthrough-and-github-sync",
            "20260308-0039-result-docs-r1",
            "20260308-0054-inekf-doc-typeset-r1",
            "20260308-0059-inekf-doc-typeset-r2",
            "20260308-0102-inekf-doc-typeset-r3",
            "20260309-1049-inekf-doc-symbols-r1",
            "20260309-1107-inekf-doc-compile-check",
            "20260309-2328-interactive-report-notebook-r1",
            "20260310-0014-interactive-report-html-fix",
            "20260310-0039-interactive-report-html-restyle",
            "20260310-0105-interactive-report-html-pdf-style",
            "20260310-1654-interactive-report-drop-group3",
            "20260310-1710-interactive-report-add-analysis",
            "20260310-1826-interactive-report-renumber",
            "20260310-1956-remove-imu-pages",
            "20260310-2010-report-rename-cleanup",
            "20260310-2015-imu-precision-report",
            "20260310-2326-github-sync",
            "20260311-0019-report-attitude-error-fix",
            "20260311-0044-html-attitude-cn-audit",
            "20260311-0104-html-accuracy-audit",
        ],
        session_action="迭代 HTML、PDF/tex、reader-facing copy 与 walkthrough 压缩形态。",
        session_outcome="形成当前两类正式展示口径：`representative` 历史链与 `dataset-partitioned` 读者链。",
        session_relation="当前下一步仍需把 reader-facing 口径同步回 `representative` 与 `.tex`。",
    ),
    "theme_06_superseded_data4_r1_and_outage_chain": ThemeMeta(
        title="被 supersede 的 data4 r1 与 outage 误接链",
        retained_conclusion="早期 data4 r1 sweep / canonical cases 与一轮 InEKF outage 对接存在 superseded/误接问题；在 `P0_diag` 根因定位前后，这些目录仍保留为历史转折证据，但已不再作为正式结论来源。",
        why_important="它解释了为什么部分 data4 ESKF 和 outage 结果会与当前 `r2` canonical cases 明显不一致。",
        session_ids=[
            "20260305-2222-inekf-eskf-data2-data4-pipeline",
            "20260306-1714-data4-vs-data2-gnss30-gap-analysis",
            "20260310-gnss-lever-vel-experiments-html-sidebar",
            "20260310-1854-data2-gnss-outage-cycle",
            "20260312-1410-data2-gnss-outage-cycle-inekf-best-and-html",
            "20260312-1414-outage-cycle-inekf-best-handoff-check",
            "20260312-1429-outage-cycle-inekf-best-reference-correction",
            "20260312-1443-outage-cycle-true-tuned-replacement",
            "20260312-2150-representative-data4-conflict-check",
        ],
        session_action="围绕 data4 r1、outage 误接与 representative 报告冲突做回溯、替换和 supersede 标记。",
        session_outcome="正式结果统一切换到 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 与 `EXP-20260312-dataset-report-cases-r2`。",
        session_relation="当前 cleanup 仍需把 superseded 目录移出工作位，只保留索引和历史摘要。",
    ),
}

CLOSED_ISSUE_SUMMARY = [
    {
        "theme": "baseline 指标冲突与 RI 解释闭环",
        "ids": [
            "ISSUE-001-metric-conflict-baseline-inekf",
            "ISSUE-005-baseline-inekf-ri-regression",
            "ISSUE-006-ri-process-noise-mapping-incomplete",
        ],
        "conclusion": "baseline 指标冲突已用 best 口径重生，RI 回归与过程噪声映射链已由 fresh 产物闭合。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_issues_from_rows",
    },
    {
        "theme": "量测模型与配置解析修复",
        "ids": [
            "ISSUE-008-true-gnsspos-r-cov-frame-bug",
            "ISSUE-20260311-config-load-fallback",
            "ISSUE-20260311-constraint-update-success-propagation",
            "ISSUE-20260311-p0diag-dead-config",
        ],
        "conclusion": "GNSS_POS 协方差坐标系、配置 fail-fast、约束 accepted 传播与 `P0_diag` 语义都已修复并有 fresh 证据。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_issues_from_rows",
    },
    {
        "theme": "UTF-8 / notebook 输入链路限制",
        "ids": ["ISSUE-20260310-utf8-notebook-stdin"],
        "conclusion": "已确认 stdin/codepage 会损伤中文内容，后续统一使用 UTF-8 直接写文件。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_issues_from_rows",
    },
    {
        "theme": "启动期基础缺陷（压缩留痕）",
        "ids": [
            "ISSUE-002-manual-init-missing-truth-crash",
            "ISSUE-003-gnss-load-nonfatal",
            "ISSUE-004-inekf-fej-semantic-conflict",
        ],
        "conclusion": "这些启动期问题已在更早阶段关闭，但仍保留摘要，避免历史因果链断裂。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_issue_bullets_from_original_walkthrough",
    },
]

RESOLVED_HYP_SUMMARY = [
    {
        "theme": "外参冻结与可观性补证",
        "ids": ["HYP-3"],
        "conclusion": "外参相关状态对调度与激励敏感已被冻结矩阵、漂移图和 mounting_roll 证据链确认。",
        "impact": "当前只需跨数据集复验，不再是开放 blocker。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_hypotheses_from_rows",
    },
    {
        "theme": "NHC 最优频率具有 dataset boundary",
        "ids": ["HYP-14", "HYP-15"],
        "conclusion": "data2 上的 `30 Hz` / `0.75 Hz` 最优频率不能直接迁移到 data4，必须 per-dataset local sweep。",
        "impact": "已成为当前 canonical report 的配置规则。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_hypotheses_from_rows",
    },
    {
        "theme": "当前 codepath 的 InEKF 机制归因",
        "ids": ["HYP-16"],
        "conclusion": "当前 codepath 下的主要收益来自 process/reset consistency，而非 measurement Jacobian 写法本身。",
        "impact": "后续机理分析聚焦 `att_z-bg_z` 与 `21-30` 协方差演化。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_hypotheses_from_rows",
    },
    {
        "theme": "已否决的单因子解释",
        "ids": ["HYP-2", "HYP-4", "HYP-5"],
        "conclusion": "`p_ned_local` 单因子、单补 `G(kVel,gyro)` 等解释已被后续 fresh 证据否决。",
        "impact": "提醒后续不要回退到已排除的简单叙事。",
        "archive_ref": f"docs/project_history/{REGISTRY_NAME} :: archived_hypothesis_bullets_from_original_walkthrough",
    },
]

CURRENT_PHASE_SUMMARY = [
    ("阶段A", "InEKF 初版重写与 best switch 锁定", "从 FEJ 依赖版转到 RI 实现，并把 best switch 锁定为后续统一口径。"),
    ("阶段B", "true_iekf 分支与 phase-2 主修复", "完成 tex-code audit、Lie 核心 Jacobian / reset 重写，以及 measurement Jacobian 数值审计。"),
    ("阶段C", "data2 post-GNSS 因果链", "冻结矩阵、state21-30 漂移图与 bg/mounting 干预把 data2 剩余 blocker 收敛到 heading drift。"),
    ("阶段D", "data2 GNSS30 NHC 频率扫描链", "把 ODO/NHC interval audit、smoke 和正式 sweep 收敛成算法分开的最优频率结论。"),
    ("阶段E", "data4 P0_diag 修正与 canonical r2", "定位 old/new data4 ESKF 巨大差异来自初始化协方差语义，并用 r2 sweep / canonical cases 重建正式结果。"),
    ("阶段F", "读者版报告与仓库整理", "reader-facing dataset report 已收敛，当前继续把主 walkthrough 和历史产物改成“工作台 + 历史摘要”。"),
]

EXP_THEME_ORDER = list(EXP_THEME_MAP.keys())
SESSION_THEME_ORDER = list(THEME_META.keys())

EXP_HEADER = "| exp_id | 日期 | 目的 | 配置 | 关键产物 | 关键指标 | 状态 | 新鲜度 |\n|---|---|---|---|---|---|---|---|"
ISSUE_HEADER = "| issue_id | 发现日期 | 描述 | 影响文件 | 影响 | 处理状态 |\n|---|---|---|---|---|---|"
HYP_HEADER = "| hyp_id | 假设 | 当前证据 | 下一步验证 | 状态 |\n|---|---|---|---|---|"


def md_list(items: list[str]) -> str:
    return "、".join(f"`{item}`" for item in items)


def md_bullets(items: list[str]) -> str:
    if not items:
        return "- none"
    return "\n".join(f"- `{item}`" for item in items)


def markdown_table(header: str, rows: list[str], fallback: str) -> str:
    if rows:
        return header + "\n" + "\n".join(rows)
    return header + "\n" + fallback


def read_walkthrough() -> str:
    return (ROOT / "walkthrough.md").read_text(encoding="utf-8")


def section(text: str, start_title: str, end_title: str) -> str:
    start = text.index(start_title)
    end = text.index(end_title)
    return text[start:end]


def parse_table_rows(section_text: str, prefix: str) -> dict[str, dict[str, list[str] | str]]:
    rows: dict[str, dict[str, list[str] | str]] = {}
    for line in section_text.splitlines():
        if line.startswith(f"| {prefix}"):
            cols = [part.strip() for part in line.strip().strip("|").split("|")]
            rows[cols[0]] = {"line": line, "cols": cols}
    return rows


def parse_session_blocks(session_text: str) -> dict[str, str]:
    matches = list(re.finditer(r"^### session_id: (.+)$", session_text, re.M))
    blocks: dict[str, str] = {}
    for index, match in enumerate(matches):
        session_id = match.group(1).strip()
        start = match.start()
        end = matches[index + 1].start() if index + 1 < len(matches) else len(session_text)
        blocks[session_id] = session_text[start:end].strip()
    return blocks


def parse_phase_summary(session_text: str) -> str:
    match = re.search(r"### 阶段摘要（压缩，保留可追溯 ID）\n\n(.*?)(?=\n### session_id:)", session_text, re.S)
    if not match:
        raise RuntimeError("Cannot find original phase summary block.")
    return "### 阶段摘要（压缩，保留可追溯 ID）\n\n" + match.group(1).strip() + "\n"


def parse_phase_session_ids(phase_summary_text: str) -> list[str]:
    session_ids: list[str] = []
    for match in re.finditer(r"相关 session: (.+?)。", phase_summary_text):
        ids = [part.strip().strip("`") for part in match.group(1).split(",")]
        for session_id in ids:
            if session_id and session_id not in session_ids:
                session_ids.append(session_id)
    return session_ids


def parse_compressed_ids(section_text: str, prefix: str) -> list[tuple[str, str]]:
    pattern = rf"^- `({prefix}[^`]+)`：(.+)$"
    return [(match.group(1), match.group(2)) for match in re.finditer(pattern, section_text, re.M)]


def parse_compressed_exp_ids(section_text: str) -> list[str]:
    return [match.group(1) for match in re.finditer(r"^- `(EXP-[^`]+)`", section_text, re.M)]


def all_ids(text: str) -> tuple[list[str], list[str], list[str]]:
    exp_ids = sorted(dict.fromkeys(re.findall(r"EXP-[A-Za-z0-9][A-Za-z0-9\-]*", text)))
    issue_ids = sorted(dict.fromkeys(re.findall(r"ISSUE-[A-Za-z0-9][A-Za-z0-9\-]*", text)))
    hyp_ids = sorted(dict.fromkeys(re.findall(r"HYP-[0-9]+", text)))
    return exp_ids, issue_ids, hyp_ids


def ensure_theme_coverage(archived_exp_ids: list[str], compressed_exp_ids: list[str], all_session_ids: list[str]) -> None:
    covered_exp_ids: list[str] = []
    for theme in EXP_THEME_ORDER:
        covered_exp_ids.extend(EXP_THEME_MAP[theme])
    missing_exp = sorted(set(archived_exp_ids + compressed_exp_ids) - set(covered_exp_ids))
    if missing_exp:
        raise RuntimeError(f"Archived experiment IDs missing from theme mapping: {missing_exp}")

    covered_sessions: list[str] = []
    for theme in SESSION_THEME_ORDER:
        covered_sessions.extend(THEME_META[theme].session_ids)
    missing_sessions = sorted(set(all_session_ids) - set(KEEP_SESSION_IDS) - set(covered_sessions))
    if missing_sessions:
        raise RuntimeError(f"Archived session IDs missing from summary theme mapping: {missing_sessions}")


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.rstrip() + "\n", encoding="utf-8")


def file_mtime_str(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")


def write_snapshot(
    original_text: str,
    exp_rows: dict[str, dict[str, list[str] | str]],
    issue_rows: dict[str, dict[str, list[str] | str]],
    hyp_rows: dict[str, dict[str, list[str] | str]],
    session_blocks: dict[str, str],
    phase_session_ids: list[str],
) -> None:
    exp_ids, issue_ids, hyp_ids = all_ids(original_text)
    session_ids = list(dict.fromkeys(phase_session_ids + list(session_blocks.keys())))
    lines = [
        f"# {SNAPSHOT_NAME}",
        "",
        f"Source: `walkthrough.md` before importance-based compression on `{TODAY}`.",
        "",
        "## Source Snapshot",
        f"- source_path: `{ROOT / 'walkthrough.md'}`",
        f"- bytes_before: `{(ROOT / 'walkthrough.md').stat().st_size}`",
        f"- lines_before: `{original_text.count(chr(10)) + 1}`",
        f"- experiment_rows: `{len(exp_rows)}`",
        f"- issue_rows: `{len(issue_rows)}`",
        f"- hypothesis_rows: `{len(hyp_rows)}`",
        f"- session_blocks: `{len(session_blocks)}`",
        f"- phase_summary_session_mentions: `{len(phase_session_ids)}`",
        "",
        "## Keep Policy Snapshot",
        f"- current_key_experiments: {md_list(KEEP_EXP_IDS)}",
        f"- current_key_sessions: {md_list(KEEP_SESSION_IDS)}",
        f"- current_issue_rows_kept_in_main: {md_list(CURRENT_ISSUE_IDS)}",
        f"- current_open_hypotheses_kept_in_main: {md_list(CURRENT_OPEN_HYP_IDS)}",
        "",
        f"## EXP IDs ({len(exp_ids)})",
        md_bullets(exp_ids),
        "",
        f"## ISSUE IDs ({len(issue_ids)})",
        md_bullets(issue_ids),
        "",
        f"## HYP IDs ({len(hyp_ids)})",
        md_bullets(hyp_ids),
        "",
        f"## Session IDs ({len(session_ids)})",
        "### From phase summary carry-forward",
        md_bullets(phase_session_ids),
        "",
        "### From full session blocks",
        md_bullets(list(session_blocks.keys())),
    ]
    write_text(PROJECT_HISTORY / SNAPSHOT_NAME, "\n".join(lines))


def write_registry_archive(
    exp_rows: dict[str, dict[str, list[str] | str]],
    issue_rows: dict[str, dict[str, list[str] | str]],
    hyp_rows: dict[str, dict[str, list[str] | str]],
    archived_issue_ids: list[str],
    archived_hyp_ids: list[str],
    compressed_exp_ids: list[str],
    resolved_issue_bullets: list[tuple[str, str]],
    rejected_hyp_bullets: list[tuple[str, str]],
) -> None:
    lines = [
        f"# {REGISTRY_NAME}",
        "",
        f"Source: archived registry content moved out of `walkthrough.md` on `{TODAY}`.",
        "",
        "## archived_experiments_by_theme",
    ]
    for theme in EXP_THEME_ORDER:
        meta = THEME_META[theme]
        lines.extend(
            [
                "",
                f"### {theme} | {meta.title}",
                "",
                "#### complete_experiment_rows",
                "",
                EXP_HEADER,
            ]
        )
        rows = [exp_rows[exp_id]["line"] for exp_id in EXP_THEME_MAP[theme] if exp_id in exp_rows]
        lines.extend(rows if rows else ["| — | — | — | — | — | — | — | — |"])
        extra = [exp_id for exp_id in EXP_THEME_MAP[theme] if exp_id not in exp_rows]
        if extra:
            lines.extend(["", "#### archived_ids_from_original_compressed_list", ""])
            lines.extend(f"- `{exp_id}`" for exp_id in extra)

    lines.extend(["", "## archived_issues_from_rows", "", ISSUE_HEADER])
    issue_lines = [issue_rows[issue_id]["line"] for issue_id in archived_issue_ids]
    lines.extend(issue_lines if issue_lines else ["| — | — | — | — | — | — |"])
    lines.extend(["", "## archived_issue_bullets_from_original_walkthrough", ""])
    lines.extend(
        [f"- `{issue_id}`：{desc}" for issue_id, desc in resolved_issue_bullets]
        if resolved_issue_bullets
        else ["- none"]
    )
    lines.extend(["", "## archived_hypotheses_from_rows", "", HYP_HEADER])
    hyp_lines = [hyp_rows[hyp_id]["line"] for hyp_id in archived_hyp_ids]
    lines.extend(hyp_lines if hyp_lines else ["| — | — | — | — | — |"])
    lines.extend(["", "## archived_hypothesis_bullets_from_original_walkthrough", ""])
    lines.extend(
        [f"- `{hyp_id}`：{desc}" for hyp_id, desc in rejected_hyp_bullets]
        if rejected_hyp_bullets
        else ["- none"]
    )
    if compressed_exp_ids:
        lines.extend(["", "## archived_experiment_ids_from_original_compressed_list", ""])
        lines.extend(f"- `{exp_id}`" for exp_id in compressed_exp_ids)
    write_text(PROJECT_HISTORY / REGISTRY_NAME, "\n".join(lines))


def write_session_archive(phase_summary_block: str, session_blocks: dict[str, str], archived_session_ids: list[str]) -> None:
    lines = [
        f"# {SESSIONS_NAME}",
        "",
        f"Source: archived session content moved out of `walkthrough.md` on `{TODAY}`.",
        "",
        "## phase_summary_original",
        "",
        phase_summary_block.strip(),
        "",
        "## archived_session_blocks_by_theme",
    ]
    for theme in SESSION_THEME_ORDER:
        meta = THEME_META[theme]
        lines.extend(
            [
                "",
                f"### {theme} | {meta.title}",
                "",
                "#### session_ids_covered",
                "",
                md_bullets(meta.session_ids),
                "",
                "#### complete_raw_blocks",
                "",
            ]
        )
        raw_blocks = [session_blocks[session_id] for session_id in meta.session_ids if session_id in archived_session_ids]
        if raw_blocks:
            for block in raw_blocks:
                lines.extend([block, ""])
        else:
            lines.append("- none (phase-summary-only carry-forward).")
    write_text(PROJECT_HISTORY / SESSIONS_NAME, "\n".join(lines))


def move_path(src: Path, dest_dir: Path, category: str, reason: str, summary_ref: str, records: list[MoveRecord]) -> None:
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / src.name
    if not src.exists():
        records.append(
            MoveRecord(
                old_path=str(src.relative_to(ROOT)),
                new_path=str(dest.relative_to(ROOT)),
                category=category,
                reason=reason,
                summary_ref=summary_ref,
                status="missing_before_move",
            )
        )
        return
    if dest.exists():
        records.append(
            MoveRecord(
                old_path=str(src.relative_to(ROOT)),
                new_path=str(dest.relative_to(ROOT)),
                category=category,
                reason=reason,
                summary_ref=summary_ref,
                status="already_exists",
            )
        )
        return
    shutil.move(str(src), str(dest))
    records.append(
        MoveRecord(
            old_path=str(src.relative_to(ROOT)),
            new_path=str(dest.relative_to(ROOT)),
            category=category,
            reason=reason,
            summary_ref=summary_ref,
            status="moved",
        )
    )


def relocate_artifacts() -> list[MoveRecord]:
    records: list[MoveRecord] = []
    sol_target = ARCHIVE / "root_sol_legacy" / "20260313"
    tmp_target = ARCHIVE / "output_review" / "tmp" / "20260313"
    superseded_target = ARCHIVE / "output_review" / "superseded" / "20260313"

    for src in sorted(ROOT.glob("SOL_*.txt")):
        move_path(
            src,
            sol_target,
            "root_sol_legacy",
            "根目录历史 `SOL_*.txt` 体量大、非当前正式结果源，迁入 archive 但保留索引。",
            f"docs/project_history/{RELOCATION_NAME}",
            records,
        )
    for src in sorted(OUT_REVIEW.glob("_tmp*")):
        move_path(
            src,
            tmp_target,
            "output_review_tmp",
            "临时产物/调试目录不再占用当前工作区，迁入 archive。",
            f"docs/project_history/{RELOCATION_NAME}",
            records,
        )
    for name in [
        "EXP-20260312-data2-gnss-outage-cycle-inekf-best-r1",
        "EXP-20260312-data4-gnss30-nhc-sweep-r1",
        "EXP-20260312-dataset-report-cases-r1",
    ]:
        move_path(
            OUT_REVIEW / name,
            superseded_target,
            "superseded_review_dir",
            "已明确被 r2 正式口径 supersede，迁出当前工作位但保留结论摘要与路径映射。",
            f"docs/project_history/{RELOCATION_NAME}",
            records,
        )

    lines = [
        f"# {RELOCATION_NAME}",
        "",
        f"Relocation date: `{TODAY}`",
        "",
        "| old_path | new_path | category | reason | summary_ref | status |",
        "|---|---|---|---|---|---|",
    ]
    for rec in records:
        lines.append(
            f"| `{rec.old_path}` | `{rec.new_path}` | `{rec.category}` | {rec.reason} | `{rec.summary_ref}` | `{rec.status}` |"
        )
    write_text(PROJECT_HISTORY / RELOCATION_NAME, "\n".join(lines))
    return records


def write_repo_layout() -> None:
    lines = [
        f"# {REPO_LAYOUT_NAME}",
        "",
        f"Updated: `{TODAY}`",
        "",
        "| 区域 | 作用 | 典型路径 | 管理规则 |",
        "|---|---|---|---|",
        "| 当前工作区 | 当前要直接编辑、运行和复现的代码与文档。 | `src/`, `include/`, `apps/`, `scripts/`, `config_*.yaml`, `walkthrough.md` | 保持轻量；只保留当前工作链路与近期关键摘要。 |",
        "| 历史文本归档 | 承接从主 `walkthrough.md` 移出的完整原文、压缩前快照与迁移索引。 | `docs/project_history/` | 必须受版本管理；任何从主档移出的研究结论都要在这里保留全文。 |",
        "| 历史产物归档 | 承接根目录遗留 `SOL_*.txt`、临时目录和已 supersede 的大产物。 | `archive/root_sol_legacy/20260313/`, `archive/output_review/tmp/20260313/`, `archive/output_review/superseded/20260313/` | 只移动临时/重复/superseded 产物；当前正式证据链不迁移。 |",
        "| 本地临时区 | 允许短期调试，但不应长期堆积。 | `output/review/_tmp*`（现已迁走）、手工 smoke 目录 | 形成结论前可短留；一旦 superseded 或任务结束，应迁入 `archive/` 或删除。 |",
        "| 当前正式结果源 | 当前结论与 HTML 正式引用的 canonical artifacts。 | `output/review/EXP-20260312-data4-gnss30-nhc-sweep-r2/`, `output/review/EXP-20260312-dataset-report-cases-r2/`, `output/html/dataset_partitioned_navigation_interactive_report.html` | 不迁移、不重命名；如被 supersede，需先更新 `walkthrough.md` 的正式口径。 |",
        "| historical-key 结果目录 | 仍有解释价值但不是当前正式结果源的历史关键目录。 | `output/review/20260306-true-iekf-phase2-r1/`, `output/review/20260306-phase2c-bg-freeze/`, `output/review/EXP-20260311-odo-nhc-rate-sweep-r2-data2-gnss30-eskf/` 等 | 可以留在原位，但需在 `walkthrough.md`/报告中明确其角色是 historical-key 而非 canonical final。 |",
    ]
    write_text(DOCS / REPO_LAYOUT_NAME, "\n".join(lines))


def build_new_session_block(
    original_bytes: int,
    pre_root_sol_count: int,
    pre_tmp_count: int,
    all_exp_ids: list[str],
    all_issue_ids: list[str],
    all_hyp_ids: list[str],
    all_session_ids: list[str],
) -> str:
    return f"""### session_id: {SESSION_ID}

- timestamp: {SESSION_TIMESTAMP}
- objective: 按内容重要性重组仓库与 `walkthrough.md`，把完整历史原文迁入受版本管理的归档文档，并把 superseded/临时大产物迁出当前工作位。
- scope:
  - 从原 `walkthrough.md` 提取 `EXP/ISSUE/HYP/session` 保全快照，并按 A/B/C 层改写主档。
  - 新增 `docs/project_history/` 下的归档文档与 `docs/repo_layout.md`。
  - 迁移根目录历史 `SOL_*.txt`、`output/review/_tmp*` 和已 supersede 的 `r1` 目录到 `archive/`，同时写入 relocation index。
- changed_files:
  - `walkthrough.md`
  - `docs/project_history/{SNAPSHOT_NAME}`
  - `docs/project_history/{REGISTRY_NAME}`
  - `docs/project_history/{SESSIONS_NAME}`
  - `docs/project_history/{RELOCATION_NAME}`
  - `docs/{REPO_LAYOUT_NAME}`
  - `.gitignore`
- configs:
  - 本会话未修改 solver 配置、未重跑实验；当前正式结果源仍为 `EXP-20260312-data4-gnss30-nhc-sweep-r2` 与 `EXP-20260312-dataset-report-cases-r2`。
- commands:
  - `Get-Content walkthrough.md` / `git status --short --branch` / `Get-ChildItem output/review`
  - `python scripts/analysis/archive_walkthrough_by_importance.py`
  - 校验脚本：比对 `EXP/ISSUE/HYP/session` 集合、`walkthrough.md` 体量、`output/review/_tmp*` 与根目录 `SOL_*.txt` 清理情况
- artifacts:
  - `docs/project_history/{SNAPSHOT_NAME}`
  - `docs/project_history/{REGISTRY_NAME}`
  - `docs/project_history/{SESSIONS_NAME}`
  - `docs/project_history/{RELOCATION_NAME}`
  - `docs/{REPO_LAYOUT_NAME}`
  - `archive/root_sol_legacy/20260313/`
  - `archive/output_review/tmp/20260313/`
  - `archive/output_review/superseded/20260313/`
- metrics:
  - `walkthrough.md` bytes: `{original_bytes} -> PENDING_NEW_WALKTHROUGH_BYTES`。
  - 保全校验目标：`EXP={len(all_exp_ids)}`、`ISSUE={len(all_issue_ids)}`、`HYP={len(all_hyp_ids)}`、`session={len(all_session_ids)}`，要求新主档 + archive 文档集合完全覆盖原集合。
  - 迁移数量：root `SOL_*.txt`=`{pre_root_sol_count}(pre)`，`output/review/_tmp*`=`{pre_tmp_count}(pre)`，superseded review dirs=`3(pre)`。
- artifact_mtime:
  - `docs/project_history/{SNAPSHOT_NAME}`: `PENDING_SNAPSHOT_MTIME`
  - `docs/project_history/{REGISTRY_NAME}`: `PENDING_REGISTRY_MTIME`
  - `docs/project_history/{SESSIONS_NAME}`: `PENDING_SESSIONS_MTIME`
  - `docs/project_history/{RELOCATION_NAME}`: `PENDING_RELOCATION_MTIME`
  - `docs/{REPO_LAYOUT_NAME}`: `PENDING_LAYOUT_MTIME`
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
  - 在保持本轮 archive 规则的前提下，继续执行既有研究主线：刷新 `representative_navigation_interactive_report.html` / `.tex`，并围绕 `v系航向误差`、`att_z-bg_z` 与 `21-30` 状态块做机理分析。"""


def build_walkthrough(
    exp_rows: dict[str, dict[str, list[str] | str]],
    issue_rows: dict[str, dict[str, list[str] | str]],
    hyp_rows: dict[str, dict[str, list[str] | str]],
    session_blocks: dict[str, str],
    all_exp_ids: list[str],
    all_issue_ids: list[str],
    all_hyp_ids: list[str],
    all_session_ids: list[str],
    original_bytes: int,
    pre_root_sol_count: int,
    pre_tmp_count: int,
) -> str:
    lines: list[str] = [
        "# walkthrough.md",
        "",
        "Schema: `v1`",
        "文档语言: 中文（必要技术字段保留英文）",
        f"Last updated: `{TODAY}`",
        "",
        "## 项目快照",
        "",
        "### 当前阶段目标",
        "- 统一当前正式结果来源：`EXP-20260312-data4-gnss30-nhc-sweep-r2` 与 `EXP-20260312-dataset-report-cases-r2`。",
        "- 在 `data4 P0_diag` 修正后的正式口径下，继续解释 `InEKF` 与 ESKF 的 `v系航向误差`、`att_z-bg_z` 耦合和 `21-30` 状态块演化。",
        "- 保持主 `walkthrough.md` 可直接完成 Start Checklist，同时把完整历史原文移交到 `docs/project_history/`。",
        "",
        "### 当前工作台速览",
        "- last_completed_session: `20260313-0103-dataset-report-reader-table-compaction`",
        f"- current_official_experiments: {md_list(['EXP-20260312-data4-gnss30-nhc-sweep-r2', 'EXP-20260312-dataset-report-cases-r2', 'EXP-20260313-dataset-partitioned-html-reader-r4'])}",
        f"- open_hypotheses: {md_list(CURRENT_OPEN_HYP_IDS)}",
        f"- active_archive_docs: {md_list([f'docs/project_history/{SNAPSHOT_NAME}', f'docs/project_history/{REGISTRY_NAME}', f'docs/project_history/{SESSIONS_NAME}', f'docs/project_history/{RELOCATION_NAME}'])}",
        "- pending_next_actions: 见文末 `## Next Actions`。",
        "",
        "### 当前基线与代码事实",
        "- 状态维度: `kStateDim = 31`",
        "- 主要滤波模式:",
        "  - 标准 ESKF（`fusion.fej.enable: false`）",
        "  - InEKF 开关模式（`fusion.fej.enable: true`）",
        "- ESKF 基线配置: `config_data2_baseline_eskf.yaml`",
        "- InEKF 历史对照配置（保留复现实验）: `config_data2_baseline_inekf_ctrl.yaml`, `config_data2_gnss30_inekf.yaml`",
        "- InEKF 当前推荐配置:",
        "  - `config_data2_baseline_inekf_best.yaml`",
        "  - `config_data2_gnss30_inekf_best.yaml`",
        "  - 固定开关: `ri_gnss_pos_use_p_ned_local=true`, `ri_vel_gyro_noise_mode=1`, `ri_inject_pos_inverse=true`",
        "- data4 主口径与配置:",
        "  - GNSS 主口径: `dataset/data4_converted/GNSS_converted.txt`（13 列，含速度）",
        "  - data4 4 组主配置: `config_data4_baseline_eskf.yaml`, `config_data4_baseline_inekf_best.yaml`, `config_data4_gnss30_eskf.yaml`, `config_data4_gnss30_inekf_best.yaml`",
        "  - data4 当前正式初始化口径: `config_data4_gnss30_eskf.yaml` 的 `P0_diag` 已按 `std_*` 等效方差重写，并由 `EXP-20260312-data4-p0diag-check-r1` / `r2` fresh 证据链验证。",
        "- 当前 data2/data4 基线组合均为 INS + GNSS + ODO + NHC（UWB 关闭）。",
        "",
        "### 31维状态拆分备忘",
        "- `p(3), v(3), att(3), ba(3), bg(3), sg(3), sa(3), odo_scale(1), mounting(3), lever_odo(3), lever_gnss(3)`",
        "",
        "### 融合主流程顺序（代码一致）",
        "1. IMU 预测",
        "2. ZUPT",
        "3. 重力对准诊断",
        "4. NHC",
        "5. ODO",
        "6. UWB",
        "7. GNSS（含 schedule 与 post-GNSS ablation）",
        "8. 诊断与结果记录",
        "",
        "## 实验索引（Experiment Registry）",
        "",
        "### 当前关键实验",
        "",
        markdown_table(EXP_HEADER, [exp_rows[exp_id]['line'] for exp_id in KEEP_EXP_IDS], "| — | — | — | — | — | — | — | — |"),
        "",
        "### 历史关键实验摘要",
        "",
        "| 主题 | 涉及 exp_id | 保留结论 | 为何仍重要 | archive_ref |",
        "|---|---|---|---|---|",
    ]
    for theme in EXP_THEME_ORDER:
        meta = THEME_META[theme]
        lines.append(
            f"| {meta.title} | {md_list(EXP_THEME_MAP[theme])} | {meta.retained_conclusion} | {meta.why_important} | `docs/project_history/{REGISTRY_NAME} :: {theme}` |"
        )
    lines.extend(
        [
            "",
            "## 已知不一致项（Known Inconsistencies）",
            "",
            markdown_table(ISSUE_HEADER, [issue_rows[issue_id]['line'] for issue_id in CURRENT_ISSUE_IDS], "| — | — | — | — | — | — |"),
            "",
            "### 已关闭问题摘要",
            "",
            "| 主题 | 涉及 issue_id | 已关闭结论 | archive_ref |",
            "|---|---|---|---|",
        ]
    )
    for item in CLOSED_ISSUE_SUMMARY:
        lines.append(f"| {item['theme']} | {md_list(item['ids'])} | {item['conclusion']} | `{item['archive_ref']}` |")
    lines.extend(
        [
            "",
            "## 开放假设（Open Hypotheses）",
            "",
            markdown_table(HYP_HEADER, [hyp_rows[hyp_id]['line'] for hyp_id in CURRENT_OPEN_HYP_IDS], "| — | — | — | — | — |"),
            "",
            "### 已解决/已否决假设摘要",
            "",
            "| 主题 | 涉及 hyp_id | 结论 | 当前影响 | archive_ref |",
            "|---|---|---|---|---|",
        ]
    )
    for item in RESOLVED_HYP_SUMMARY:
        lines.append(
            f"| {item['theme']} | {md_list(item['ids'])} | {item['conclusion']} | {item['impact']} | `{item['archive_ref']}` |"
        )
    lines.extend(["", "## 会话日志（Session Log）", "", "### 阶段摘要", ""])
    for phase_id, phase_title, phase_desc in CURRENT_PHASE_SUMMARY:
        lines.append(f"- {phase_id} `{phase_title}`：{phase_desc}")
    lines.extend(
        [
            f"- 原始阶段摘要全文已移至 `docs/project_history/{SESSIONS_NAME} :: phase_summary_original`。",
            "",
            "### 当前关键会话",
            "",
        ]
    )
    for session_id in KEEP_SESSION_IDS:
        lines.extend([session_blocks[session_id], ""])
    lines.extend(
        [
            build_new_session_block(
                original_bytes=original_bytes,
                pre_root_sol_count=pre_root_sol_count,
                pre_tmp_count=pre_tmp_count,
                all_exp_ids=all_exp_ids,
                all_issue_ids=all_issue_ids,
                all_hyp_ids=all_hyp_ids,
                all_session_ids=all_session_ids,
            ),
            "",
            "### 已归档会话摘要",
            "",
            "| 主题 | 涉及 session_id | 核心动作 | 最终决定/产物 | 当前关联 | archive_ref |",
            "|---|---|---|---|---|---|",
        ]
    )
    for theme in SESSION_THEME_ORDER:
        meta = THEME_META[theme]
        lines.append(
            f"| {meta.title} | {md_list(meta.session_ids)} | {meta.session_action} | {meta.session_outcome} | {meta.session_relation} | `docs/project_history/{SESSIONS_NAME} :: {theme}` |"
        )
    lines.extend(
        [
            "",
            "## Next Actions",
            "",
            "1. Refresh `representative_navigation_interactive_report.html` 与 `.tex` 文档，使其采用 `EXP-20260312-data4-gnss30-nhc-sweep-r2` / `EXP-20260312-dataset-report-cases-r2` 的 fresh 结果，并同步 reader-facing 文案口径。",
            "2. 基于修正后的 data4 ESKF 基线，继续做机理分析：重点对照 `data4_gnss30_eskf` vs `data4_gnss30_true_iekf` 的 `v系航向误差`、`att_z-bg_z` 耦合与 `21-30` 互协方差演化。",
            "3. 若后续仍需比较跨时间的历史结果，统一在文档中区分 `r1`（修正前 `P0_diag` 语义）与 `r2`（修正后正式口径），避免再次把两套 data4 ESKF 数值混用。",
            "4. 继续沿用“压缩展示、不压缩证据”的归档规则：新增大体积 superseded 产物时，先补主档摘要与 archive_ref，再迁入 `archive/`。",
        ]
    )
    return "\n".join(lines).rstrip() + "\n"


def fill_walkthrough_placeholders(text: str) -> str:
    replacements = {
        "PENDING_NEW_WALKTHROUGH_BYTES": str((ROOT / "walkthrough.md").stat().st_size),
        "PENDING_SNAPSHOT_MTIME": file_mtime_str(PROJECT_HISTORY / SNAPSHOT_NAME),
        "PENDING_REGISTRY_MTIME": file_mtime_str(PROJECT_HISTORY / REGISTRY_NAME),
        "PENDING_SESSIONS_MTIME": file_mtime_str(PROJECT_HISTORY / SESSIONS_NAME),
        "PENDING_RELOCATION_MTIME": file_mtime_str(PROJECT_HISTORY / RELOCATION_NAME),
        "PENDING_LAYOUT_MTIME": file_mtime_str(DOCS / REPO_LAYOUT_NAME),
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def main() -> None:
    original_text = read_walkthrough()
    if "### 阶段摘要（压缩，保留可追溯 ID）" not in original_text:
        raise SystemExit("walkthrough.md is already in compacted format; this one-off archival script should not be rerun on the rewritten file.")
    original_bytes = (ROOT / "walkthrough.md").stat().st_size
    pre_root_sol_count = len(list(ROOT.glob("SOL_*.txt")))
    pre_tmp_count = len(list(OUT_REVIEW.glob("_tmp*")))

    exp_section = section(original_text, "## 实验索引（Experiment Registry）", "## 已知不一致项（Known Inconsistencies）")
    issue_section = section(original_text, "## 已知不一致项（Known Inconsistencies）", "## 开放假设（Open Hypotheses）")
    hyp_section = section(original_text, "## 开放假设（Open Hypotheses）", "## 会话日志（Session Log）")
    session_section = section(original_text, "## 会话日志（Session Log）", "## Next Actions")

    exp_rows = parse_table_rows(exp_section, "EXP-")
    issue_rows = parse_table_rows(issue_section, "ISSUE-")
    hyp_rows = parse_table_rows(hyp_section, "HYP-")
    session_blocks = parse_session_blocks(session_section)
    phase_summary_block = parse_phase_summary(session_section)
    phase_session_ids = parse_phase_session_ids(phase_summary_block)

    all_exp_ids, all_issue_ids, all_hyp_ids = all_ids(original_text)
    all_session_ids = list(dict.fromkeys(phase_session_ids + list(session_blocks.keys())))
    archived_exp_ids = [exp_id for exp_id in exp_rows if exp_id not in KEEP_EXP_IDS]
    archived_issue_ids = [issue_id for issue_id in issue_rows if issue_id not in CURRENT_ISSUE_IDS]
    archived_hyp_ids = [hyp_id for hyp_id in hyp_rows if hyp_id not in CURRENT_OPEN_HYP_IDS]
    archived_session_ids = [session_id for session_id in session_blocks if session_id not in KEEP_SESSION_IDS]
    compressed_exp_ids = parse_compressed_exp_ids(exp_section)
    resolved_issue_bullets = parse_compressed_ids(issue_section, "ISSUE-")
    rejected_hyp_bullets = parse_compressed_ids(hyp_section, "HYP-")

    ensure_theme_coverage(archived_exp_ids, compressed_exp_ids, all_session_ids)
    write_snapshot(original_text, exp_rows, issue_rows, hyp_rows, session_blocks, phase_session_ids)
    write_registry_archive(exp_rows, issue_rows, hyp_rows, archived_issue_ids, archived_hyp_ids, compressed_exp_ids, resolved_issue_bullets, rejected_hyp_bullets)
    write_session_archive(phase_summary_block, session_blocks, archived_session_ids)
    relocate_artifacts()
    write_repo_layout()

    write_text(
        ROOT / "walkthrough.md",
        build_walkthrough(
            exp_rows=exp_rows,
            issue_rows=issue_rows,
            hyp_rows=hyp_rows,
            session_blocks=session_blocks,
            all_exp_ids=all_exp_ids,
            all_issue_ids=all_issue_ids,
            all_hyp_ids=all_hyp_ids,
            all_session_ids=all_session_ids,
            original_bytes=original_bytes,
            pre_root_sol_count=pre_root_sol_count,
            pre_tmp_count=pre_tmp_count,
        ),
    )
    write_text(ROOT / "walkthrough.md", fill_walkthrough_placeholders(read_walkthrough()))


if __name__ == "__main__":
    main()
